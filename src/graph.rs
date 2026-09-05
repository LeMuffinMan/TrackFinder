//! Local trail graph and isochrone.
//!
//! Built on demand from the OSM network already loaded — **never country-wide**.
//! Nodes are the OSM nodes shared between ways (the junctions); one edge carries
//! a whole segment between two junctions, which divides the graph size by about
//! ten compared with one node per shape point.

use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap};

use crate::dem::GRAPH_DEM_ZOOM;
use crate::dem::DemStore;
use crate::geo::{haversine_m, LatLon};
use crate::track::{SpeedModel, WalkSettings};
use crate::trails::{Snap, TrailNetwork};

/// Cost in milliseconds: an integer, so it orders cleanly with no floating
/// point comparison surprises inside the binary heap.
pub type CostMs = u64;

pub struct Edge {
    pub a: u32,
    pub b: u32,
    pub way_id: i64,
    /// Indices into `way.points`, with `from < to`.
    pub from: u32,
    pub to: u32,
    pub len_m: f64,
    /// Climb in the `from → to` direction. `None` until the DEM has arrived.
    pub climb: Option<(f32, f32)>,
}

impl Edge {
    /// Travel time in the given direction, in milliseconds.
    pub fn cost_ms(&self, forward: bool, w: &WalkSettings) -> CostMs {
        let (up, down) = match self.climb {
            Some((up, down)) if forward => (up as f64, down as f64),
            Some((up, down)) => (down as f64, up as f64),
            None => (0.0, 0.0),
        };
        let factor = w.speed_factor().max(0.1);
        let base_h = match w.model {
            SpeedModel::Naismith => {
                (self.len_m / 1000.0) / w.flat_kmh.max(0.1) + up / w.ascent_mh.max(1.0)
            }
            // An edge only knows its climb and its drop, not where they sit
            // along it. Split it into an up part and a down part in proportion,
            // both at the same slope magnitude — the coarsest reading of Tobler
            // that still slows a steep edge down in both directions.
            SpeedModel::Tobler => {
                let total = up + down;
                if total <= 0.0 || self.len_m <= 0.0 {
                    (self.len_m / 1000.0) / w.slope_kmh(0.0)
                } else {
                    let slope = total / self.len_m;
                    let up_len = self.len_m * up / total;
                    let down_len = self.len_m - up_len;
                    (up_len / 1000.0) / w.slope_kmh(slope)
                        + (down_len / 1000.0) / w.slope_kmh(-slope)
                }
            }
        };
        (base_h / factor * 3_600_000.0) as CostMs
    }
}

#[derive(Default)]
pub struct Graph {
    /// Position of each node, indexed by internal id.
    pub nodes: Vec<LatLon>,
    /// OSM id → internal index.
    osm_to_node: HashMap<i64, u32>,
    pub edges: Vec<Edge>,
    /// node → incident edges.
    adj: Vec<Vec<u32>>,
    /// OSM way → edges derived from that way.
    by_way: HashMap<i64, Vec<u32>>,
}

/// Where a snapped point falls inside the graph.
#[derive(Clone, Copy, Debug)]
pub struct EdgePos {
    pub edge: u32,
    /// Distance in metres to the edge's `a` node, along the segment.
    pub to_a_m: f64,
    pub to_b_m: f64,
}

impl Graph {
    pub fn is_empty(&self) -> bool {
        self.edges.is_empty()
    }

    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Builds the graph: nodes are junctions (an OSM node carried by several
    /// ways) plus the way endpoints.
    pub fn build(net: &TrailNetwork) -> Self {
        let mut occurrences: HashMap<i64, u32> = HashMap::new();
        for way in net.ways() {
            for id in &way.nodes {
                *occurrences.entry(*id).or_insert(0) += 1;
            }
        }

        let mut g = Graph::default();
        for way in net.ways() {
            // `out geom` aligns `nodes` and `geometry`; without that alignment
            // we cannot tell which point carries which id.
            if way.nodes.len() != way.points.len() {
                continue;
            }
            let last = way.points.len() - 1;
            let mut cut = 0usize;
            let mut len_m = 0.0;
            for i in 1..=last {
                len_m += haversine_m(way.points[i - 1], way.points[i]);
                let is_junction = occurrences.get(&way.nodes[i]).copied().unwrap_or(0) > 1;
                if is_junction || i == last {
                    if len_m > 0.0 {
                        let a = g.node(way.nodes[cut], way.points[cut]);
                        let b = g.node(way.nodes[i], way.points[i]);
                        if a != b {
                            g.push_edge(Edge {
                                a,
                                b,
                                way_id: way.id,
                                from: cut as u32,
                                to: i as u32,
                                len_m,
                                climb: None,
                            });
                        }
                    }
                    cut = i;
                    len_m = 0.0;
                }
            }
        }
        g
    }

    fn node(&mut self, osm_id: i64, pos: LatLon) -> u32 {
        *self.osm_to_node.entry(osm_id).or_insert_with(|| {
            self.nodes.push(pos);
            self.adj.push(Vec::new());
            (self.nodes.len() - 1) as u32
        })
    }

    fn push_edge(&mut self, edge: Edge) {
        let idx = self.edges.len() as u32;
        self.adj[edge.a as usize].push(idx);
        self.adj[edge.b as usize].push(idx);
        self.by_way.entry(edge.way_id).or_default().push(idx);
        self.edges.push(edge);
    }

    /// Fills in the missing climbs from the DEM. Returns true once everything is
    /// filled — tiles arrive asynchronously, so this is called every frame until
    /// then.
    pub fn update_elevations(
        &mut self,
        net: &TrailNetwork,
        dem: &mut DemStore,
        ctx: &egui::Context,
    ) -> bool {
        let mut complete = true;
        for edge in &mut self.edges {
            if edge.climb.is_some() {
                continue;
            }
            let Some(way) = net.way_by_id(edge.way_id) else {
                continue;
            };
            let mut up = 0.0f32;
            let mut down = 0.0f32;
            let mut prev: Option<f32> = None;
            let mut missing = false;
            // The graph DEM is ~38 m/pixel: sampling finer than the shape points
            // would not say anything more.
            for i in edge.from..=edge.to {
                let Some(alt) = dem.elevation_at(way.points[i as usize], GRAPH_DEM_ZOOM, ctx)
                else {
                    missing = true;
                    break;
                };
                if let Some(p) = prev {
                    let d = alt - p;
                    if d > 0.0 {
                        up += d;
                    } else {
                        down -= d;
                    }
                }
                prev = Some(alt);
            }
            if missing {
                complete = false;
            } else {
                edge.climb = Some((up, down));
            }
        }
        complete
    }

    /// Finds the edge carrying a snapped point.
    pub fn locate(&self, net: &TrailNetwork, snap: &Snap) -> Option<EdgePos> {
        let way = net.way_by_id(snap.way_id)?;
        let candidates = self.by_way.get(&snap.way_id)?;
        let seg = snap.seg as u32;
        let &edge_idx = candidates
            .iter()
            .find(|i| {
                let e = &self.edges[**i as usize];
                seg >= e.from && seg < e.to
            })
            .or(candidates.first())?;
        let e = &self.edges[edge_idx as usize];

        // Distance along the segment, from `from` to the snapped point.
        let mut to_a_m = 0.0;
        for i in e.from..seg.min(e.to) {
            to_a_m += haversine_m(way.points[i as usize], way.points[i as usize + 1]);
        }
        if seg < e.to {
            let a = way.points[seg as usize];
            let b = way.points[seg as usize + 1];
            to_a_m += haversine_m(a, b) * snap.t;
        }
        let to_a_m = to_a_m.clamp(0.0, e.len_m);
        Some(EdgePos {
            edge: edge_idx,
            to_a_m,
            to_b_m: e.len_m - to_a_m,
        })
    }

    /// Two weighted sources: both ends of the edge we stand on, each carrying the
    /// cost of the partial stretch needed to reach it.
    pub fn sources_from(&self, pos: &EdgePos, w: &WalkSettings) -> Vec<(u32, CostMs)> {
        let e = &self.edges[pos.edge as usize];
        let full = e.cost_ms(true, w).max(1) as f64;
        let back = e.cost_ms(false, w).max(1) as f64;
        let ratio_a = if e.len_m > 0.0 { pos.to_a_m / e.len_m } else { 0.0 };
        let ratio_b = if e.len_m > 0.0 { pos.to_b_m / e.len_m } else { 0.0 };
        vec![
            (e.a, (back * ratio_a) as CostMs),
            (e.b, (full * ratio_b) as CostMs),
        ]
    }

    /// Dijkstra bounded by the budget. This is both the isochrone and the
    /// shortest-path tree the leg is built from.
    pub fn explore(
        &self,
        sources: &[(u32, CostMs)],
        budget_ms: CostMs,
        w: &WalkSettings,
    ) -> Reach {
        let mut dist: Vec<CostMs> = vec![CostMs::MAX; self.nodes.len()];
        let mut prev: Vec<Option<(u32, u32)>> = vec![None; self.nodes.len()];
        let mut heap = BinaryHeap::new();

        for (node, cost) in sources {
            if (*node as usize) < dist.len() && *cost < dist[*node as usize] {
                dist[*node as usize] = *cost;
                heap.push(Reverse((*cost, *node)));
            }
        }

        while let Some(Reverse((d, node))) = heap.pop() {
            if d > dist[node as usize] || d > budget_ms {
                continue;
            }
            for &edge_idx in &self.adj[node as usize] {
                let e = &self.edges[edge_idx as usize];
                let forward = e.a == node;
                let other = if forward { e.b } else { e.a };
                let next = d.saturating_add(e.cost_ms(forward, w));
                if next <= budget_ms && next < dist[other as usize] {
                    dist[other as usize] = next;
                    prev[other as usize] = Some((node, edge_idx));
                    heap.push(Reverse((next, other)));
                }
            }
        }
        Reach { dist, prev }
    }
}

/// Result of an exploration: access cost of each node, plus the path tree.
pub struct Reach {
    pub dist: Vec<CostMs>,
    prev: Vec<Option<(u32, u32)>>,
}

impl Reach {
    pub fn cost(&self, node: u32) -> Option<CostMs> {
        self.dist
            .get(node as usize)
            .copied()
            .filter(|c| *c != CostMs::MAX)
    }

    pub fn reached_count(&self) -> usize {
        self.dist.iter().filter(|c| **c != CostMs::MAX).count()
    }

    /// Edges with both ends reached: those are the ones we colour. We draw no
    /// area polygon — you only ever walk on trails, and a filled blob would lie
    /// about which ground can be crossed.
    pub fn reachable_edges(&self, graph: &Graph) -> Vec<(u32, CostMs)> {
        graph
            .edges
            .iter()
            .enumerate()
            .filter_map(|(i, e)| {
                let a = self.cost(e.a)?;
                let b = self.cost(e.b)?;
                Some((i as u32, a.max(b)))
            })
            .collect()
    }

    /// Walks the tree back from a node to a source.
    fn edges_to(&self, node: u32) -> Option<Vec<(u32, u32)>> {
        let mut out = Vec::new();
        let mut cur = node;
        while let Some((parent, edge)) = self.prev[cur as usize] {
            out.push((parent, edge));
            cur = parent;
            if out.len() > 100_000 {
                return None; // guard rail: never seen, but a cycle would be fatal
            }
        }
        out.reverse();
        Some(out)
    }
}

/// A leg computed between two snapped points.
pub struct Route {
    pub points: Vec<LatLon>,
    pub cost_ms: CostMs,
}

/// Builds the geometry of a route between two snapped points, following the
/// shortest-path tree.
///
/// Both ends fall in the middle of an edge: the stretch between the clicked
/// point and the graph node has to be stitched back on, otherwise the track
/// "jumps" to the nearest junction.
pub fn route(
    graph: &Graph,
    net: &TrailNetwork,
    reach: &Reach,
    from: (&Snap, &EdgePos),
    to: (&Snap, &EdgePos),
    w: &WalkSettings,
) -> Option<Route> {
    let (from_snap, from_pos) = from;
    let (to_snap, to_pos) = to;
    let end_edge = &graph.edges[to_pos.edge as usize];

    // Of the two ends of the arrival edge, keep the one that minimises
    // "cost to reach it + remaining stretch to walk".
    let partial_ms = |m: f64| (m / 1000.0 / w.flat_kmh.max(0.1) * 3_600_000.0) as CostMs;
    let (total_ms, end_node, end_toward_a) = [
        reach.cost(end_edge.a).map(|c| (c + partial_ms(to_pos.to_a_m), end_edge.a, true)),
        reach.cost(end_edge.b).map(|c| (c + partial_ms(to_pos.to_b_m), end_edge.b, false)),
    ]
    .into_iter()
    .flatten()
    .min_by_key(|(c, _, _)| *c)?;

    let chain = reach.edges_to(end_node)?;
    let start_node = chain.first().map(|(p, _)| *p).unwrap_or(end_node);

    // Start: from the clicked point to the node the tree starts from.
    let start_edge = &graph.edges[from_pos.edge as usize];
    let start_toward_a = start_node == start_edge.a;
    let mut points = partial_geometry(net, start_edge, from_snap, start_toward_a)?;

    for (parent, edge_idx) in &chain {
        let edge = &graph.edges[*edge_idx as usize];
        let forward = edge.a == *parent;
        points.extend(edge_geometry(net, edge, forward)?.skip(1));
    }

    // Arrival: from the node to the clicked point (reversed geometry).
    let mut tail = partial_geometry(net, end_edge, to_snap, end_toward_a)?;
    tail.reverse();
    points.extend(tail.into_iter().skip(1));

    points.dedup_by(|a, b| (a.lat - b.lat).abs() < 1e-9 && (a.lon - b.lon).abs() < 1e-9);
    Some(Route {
        points,
        cost_ms: total_ms,
    })
}

/// Stretch of way between a snapped point and one end of its edge. The first
/// point is always the snapped one.
fn partial_geometry(
    net: &TrailNetwork,
    edge: &Edge,
    snap: &Snap,
    toward_a: bool,
) -> Option<Vec<LatLon>> {
    let way = net.way_by_id(edge.way_id)?;
    let seg = (snap.seg as u32).clamp(edge.from, edge.to.saturating_sub(1));
    let mut out = vec![snap.pos];
    if toward_a {
        for i in (edge.from..=seg).rev() {
            out.push(way.points[i as usize]);
        }
    } else {
        for i in (seg + 1)..=edge.to {
            out.push(way.points[i as usize]);
        }
    }
    Some(out)
}

/// Geometry of an edge, in the requested direction.
pub fn edge_geometry<'a>(
    net: &'a TrailNetwork,
    edge: &Edge,
    forward: bool,
) -> Option<Box<dyn Iterator<Item = LatLon> + 'a>> {
    let way = net.way_by_id(edge.way_id)?;
    let slice = &way.points[edge.from as usize..=edge.to as usize];
    Some(if forward {
        Box::new(slice.iter().copied())
    } else {
        Box::new(slice.iter().rev().copied())
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::trails::{Way, WayKind, SNAP_RADIUS_M};

    fn way(id: i64, nodes: &[i64], pts: &[(f64, f64)]) -> Way {
        let points: Vec<LatLon> = pts.iter().map(|(a, b)| LatLon::new(*a, *b)).collect();
        let bounds = points.iter().fold(
            (f64::MAX, f64::MAX, f64::MIN, f64::MIN),
            |b, p| (b.0.min(p.lat), b.1.min(p.lon), b.2.max(p.lat), b.3.max(p.lon)),
        );
        Way {
            id,
            kind: WayKind::Path,
            name: None,
            nodes: nodes.to_vec(),
            points,
            sac_scale: None,
            bounds,
        }
    }

    /// A cross: one east-west way and one north-south way sharing node 3.
    ///
    /// ```text
    ///            (5) 45.012
    ///             |
    ///  (1)--(2)--(3)--(4)      45.010
    ///             |
    ///            (6) 45.008
    /// ```
    fn cross() -> TrailNetwork {
        let mut net = TrailNetwork::default();
        net.insert(way(
            10,
            &[1, 2, 3, 4],
            &[
                (45.010, 6.000),
                (45.010, 6.002),
                (45.010, 6.004),
                (45.010, 6.006),
            ],
        ));
        net.insert(way(
            20,
            &[5, 3, 6],
            &[(45.012, 6.004), (45.010, 6.004), (45.008, 6.004)],
        ));
        net
    }

    #[test]
    fn graph_nodes_are_the_junctions() {
        let g = Graph::build(&cross());
        // Kept nodes: 1, 3, 4 (endpoints + junction) on way 10, plus 5 and 6.
        // Node 2, a mere shape point, does not become a graph node.
        assert_eq!(g.node_count(), 5, "a shape point must not become a node");
        assert_eq!(g.edges.len(), 4);
        // The 1→3 stretch carries both segments.
        let e = g.edges.iter().find(|e| e.way_id == 10 && e.from == 0).unwrap();
        assert_eq!(e.to, 2);
        // two ~157 m segments
        assert!((e.len_m - 314.0).abs() < 5.0, "len = {}", e.len_m);
    }

    #[test]
    fn a_malformed_way_is_ignored() {
        // `nodes` and `geometry` out of step: we cannot tell which point carries
        // which id, so we invent nothing.
        let mut net = TrailNetwork::default();
        net.insert(way(1, &[1, 2], &[(45.0, 6.0), (45.0, 6.001), (45.0, 6.002)]));
        assert!(Graph::build(&net).is_empty());
    }

    #[test]
    fn cost_depends_on_direction() {
        let w = WalkSettings {
            load_kg: 0.0,
            ..Default::default()
        };
        let edge = Edge {
            a: 0,
            b: 1,
            way_id: 1,
            from: 0,
            to: 1,
            len_m: 5000.0,
            climb: Some((600.0, 0.0)),
        };
        // 5 km flat = 1 h; +600 m = one more hour uphill, nothing downhill.
        let up = edge.cost_ms(true, &w) as f64 / 3_600_000.0;
        let down = edge.cost_ms(false, &w) as f64 / 3_600_000.0;
        assert!((up - 2.0).abs() < 1e-6, "uphill {up}");
        assert!((down - 1.0).abs() < 1e-6, "downhill {down}");
    }

    #[test]
    fn the_isochrone_is_bounded_by_the_budget() {
        let net = cross();
        let mut g = Graph::build(&net);
        for e in &mut g.edges {
            e.climb = Some((0.0, 0.0));
        }
        let w = WalkSettings {
            flat_kmh: 5.0,
            load_kg: 0.0,
            ..Default::default()
        };
        // From the westernmost node (OSM node 1).
        let start = g.osm_to_node[&1];

        // 5 minutes at 5 km/h ≈ 416 m: we reach the junction (314 m), not the
        // next one (314 + 157 m).
        let tight = g.explore(&[(start, 0)], 300_000, &w);
        assert_eq!(tight.reached_count(), 2);

        // 10 minutes: the whole network.
        let wide = g.explore(&[(start, 0)], 600_000, &w);
        assert_eq!(wide.reached_count(), g.node_count());
        assert_eq!(wide.reachable_edges(&g).len(), 4);
    }

    #[test]
    fn climb_slows_the_isochrone_down() {
        let net = cross();
        let mut flat = Graph::build(&net);
        for e in &mut flat.edges {
            e.climb = Some((0.0, 0.0));
        }
        let mut steep = Graph::build(&net);
        for e in &mut steep.edges {
            e.climb = Some((200.0, 0.0));
        }
        let w = WalkSettings::default();
        let start = flat.osm_to_node[&1];
        let budget = 900_000; // 15 min
        assert!(
            flat.explore(&[(start, 0)], budget, &w).reached_count()
                > steep.explore(&[(start, 0)], budget, &w).reached_count(),
            "for the same budget, uphill gets you less far"
        );
    }

    #[test]
    fn the_route_follows_the_trails() {
        let net = cross();
        let mut g = Graph::build(&net);
        for e in &mut g.edges {
            e.climb = Some((0.0, 0.0));
        }
        let w = WalkSettings::default();

        // West end to north end: the route must go through the junction rather
        // than cut across.
        let from = net.snap(LatLon::new(45.0100, 6.0005), SNAP_RADIUS_M).unwrap();
        let to = net.snap(LatLon::new(45.0118, 6.0040), SNAP_RADIUS_M).unwrap();
        let from_pos = g.locate(&net, &from).unwrap();
        let to_pos = g.locate(&net, &to).unwrap();

        let reach = g.explore(&g.sources_from(&from_pos, &w), CostMs::MAX / 4, &w);
        let r = route(&g, &net, &reach, (&from, &from_pos), (&to, &to_pos), &w).unwrap();

        assert!(r.points.len() >= 3, "{:?}", r.points);
        assert!(
            r.points
                .iter()
                .any(|p| (p.lat - 45.010).abs() < 1e-9 && (p.lon - 6.004).abs() < 1e-9),
            "the route must pass through the junction: {:?}",
            r.points
        );
        assert!(r.cost_ms > 0);
    }

    #[test]
    fn locate_puts_the_point_on_the_right_stretch() {
        let net = cross();
        let g = Graph::build(&net);
        // Just west of the junction, on the second segment of way 10.
        let snap = net.snap(LatLon::new(45.010, 6.0035), SNAP_RADIUS_M).unwrap();
        let pos = g.locate(&net, &snap).unwrap();
        let e = &g.edges[pos.edge as usize];
        assert_eq!(e.way_id, 10);
        assert!((pos.to_a_m + pos.to_b_m - e.len_m).abs() < 1e-6);
        // ~117 m from node 1, ~39 m before the junction.
        assert!(pos.to_b_m < pos.to_a_m, "we are nearer the junction");
    }
}
