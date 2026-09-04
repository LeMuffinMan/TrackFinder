//! The trail network: data model, spatial index, snapping, segment following.
//!
//! Local and on demand — never country-wide. Where the ways come from is
//! `archive`'s business; this module only knows how to hold them and answer
//! questions about them.

use std::collections::HashMap;

use crate::geo::LatLon;

/// Side of a spatial index cell, in degrees (~220 m).
const INDEX_CELL_DEG: f64 = 0.002;

/// Largest accepted distance between a click and the nearest trail.
pub const SNAP_RADIUS_M: f64 = 60.0;

// ---------------------------------------------------------------------------
// Data
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WayKind {
    Path,
    Track,
    Footway,
    Steps,
    Cycleway,
    Road,
}

impl WayKind {
    /// True for the ways one actually walks a mountain route on, as opposed to
    /// the roads and streets that merely happen to be walkable.
    pub fn is_hiking(self) -> bool {
        matches!(self, WayKind::Path | WayKind::Footway | WayKind::Steps)
    }

    /// True for anything off-tarmac: hiking ways plus forestry tracks and
    /// cycleways, which carry a route just as well.
    pub fn is_offroad(self) -> bool {
        self.is_hiking() || matches!(self, WayKind::Track | WayKind::Cycleway)
    }

    /// Wire code → class. Unknown codes fall back to `Road`: a future archive
    /// version may add classes, and an old application must still draw them
    /// rather than drop the trail.
    pub fn from_code(code: u8) -> Self {
        match code {
            trailfmt::kind::PATH => WayKind::Path,
            trailfmt::kind::TRACK => WayKind::Track,
            trailfmt::kind::FOOTWAY => WayKind::Footway,
            trailfmt::kind::STEPS => WayKind::Steps,
            trailfmt::kind::CYCLEWAY => WayKind::Cycleway,
            _ => WayKind::Road,
        }
    }

    pub fn color(self) -> egui::Color32 {
        match self {
            WayKind::Path => egui::Color32::from_rgb(180, 60, 200),
            WayKind::Track => egui::Color32::from_rgb(150, 110, 60),
            WayKind::Footway | WayKind::Steps => egui::Color32::from_rgb(80, 140, 220),
            WayKind::Cycleway => egui::Color32::from_rgb(60, 170, 160),
            WayKind::Road => egui::Color32::from_gray(120),
        }
    }
}

pub struct Way {
    pub id: i64,
    pub kind: WayKind,
    #[allow(dead_code)] // route name display
    pub name: Option<String>,
    /// OSM node ids: two ways sharing a node are connected. This is the topology
    /// the graph is built from.
    #[allow(dead_code)]
    pub nodes: Vec<i64>,
    pub points: Vec<LatLon>,
    #[allow(dead_code)] // alpine grading, for a future hazard overlay
    pub sac_scale: Option<String>,
    /// (south, west, north, east) — precomputed for render culling.
    pub bounds: (f64, f64, f64, f64),
}

impl Way {
    /// Builds a way from its archive form.
    ///
    /// `synthetic` hands out identities for the points that are **not** shared
    /// between ways. They must be unique across the whole network and must never
    /// collide with a real OSM id, or `Graph::build` — which finds junctions by
    /// counting how often an id appears — would invent junctions out of nothing.
    /// Real ids are positive, so the counter walks downwards from zero.
    pub fn from_archive(aw: trailfmt::ArchiveWay, synthetic: &mut i64) -> Option<Self> {
        if !aw.is_valid() {
            return None;
        }
        let points: Vec<LatLon> = aw
            .points
            .iter()
            .map(|(lat, lon)| LatLon::new(*lat, *lon))
            .collect();
        let nodes: Vec<i64> = aw
            .shared
            .iter()
            .map(|s| {
                s.unwrap_or_else(|| {
                    *synthetic -= 1;
                    *synthetic
                })
            })
            .collect();
        let bounds = points.iter().fold(
            (f64::MAX, f64::MAX, f64::MIN, f64::MIN),
            |b, p| (b.0.min(p.lat), b.1.min(p.lon), b.2.max(p.lat), b.3.max(p.lon)),
        );
        Some(Way {
            id: aw.id,
            kind: WayKind::from_code(aw.kind),
            name: aw.name,
            nodes,
            points,
            sac_scale: trailfmt::sac_label(aw.sac).map(|s| s.to_owned()),
            bounds,
        })
    }

    pub fn intersects(&self, view: (f64, f64, f64, f64)) -> bool {
        self.bounds.0 <= view.2 && self.bounds.2 >= view.0 && self.bounds.1 <= view.3 && self.bounds.3 >= view.1
    }
}

/// A position snapped onto a trail.
#[derive(Clone, Copy, Debug)]
pub struct Snap {
    pub way_id: i64,
    pub seg: usize,
    /// Position along the segment, in [0, 1].
    pub t: f64,
    pub pos: LatLon,
    pub dist_m: f64,
}

// ---------------------------------------------------------------------------
// Network
// ---------------------------------------------------------------------------

#[derive(Default)]
pub struct TrailNetwork {
    ways: Vec<Way>,
    by_id: HashMap<i64, usize>,
    /// cell → (way index, segment index)
    index: HashMap<(i32, i32), Vec<(u32, u32)>>,
}

fn cell_of(ll: LatLon) -> (i32, i32) {
    (
        (ll.lat / INDEX_CELL_DEG).floor() as i32,
        (ll.lon / INDEX_CELL_DEG).floor() as i32,
    )
}

/// Metres per degree at this latitude. Used for point-to-segment distance in a
/// local planar approximation — valid at the scale of one index cell.
fn deg_to_m(lat: f64) -> (f64, f64) {
    (111_320.0 * lat.to_radians().cos(), 111_132.0)
}

impl TrailNetwork {
    pub fn ways(&self) -> &[Way] {
        &self.ways
    }

    pub fn way_by_id(&self, id: i64) -> Option<&Way> {
        self.by_id.get(&id).map(|i| &self.ways[*i])
    }

    pub fn len(&self) -> usize {
        self.ways.len()
    }

    pub fn insert(&mut self, way: Way) {
        // A way is filed under the tile holding its first point, but it can
        // reach into neighbours, so two adjacent tiles report the same border
        // ways. Dropping the repeat here is what makes that free.
        if self.by_id.contains_key(&way.id) || way.points.len() < 2 {
            return;
        }
        let idx = self.ways.len() as u32;
        for (seg, pair) in way.points.windows(2).enumerate() {
            let (a, b) = (cell_of(pair[0]), cell_of(pair[1]));
            for cy in a.0.min(b.0)..=a.0.max(b.0) {
                for cx in a.1.min(b.1)..=a.1.max(b.1) {
                    self.index
                        .entry((cy, cx))
                        .or_default()
                        .push((idx, seg as u32));
                }
            }
        }
        self.by_id.insert(way.id, self.ways.len());
        self.ways.push(way);
    }

    /// Nearest point of the network, within a given radius.
    pub fn snap(&self, ll: LatLon, max_dist_m: f64) -> Option<Snap> {
        let (mx, my) = deg_to_m(ll.lat);
        let cell_m = INDEX_CELL_DEG * my.min(mx);
        let reach = (max_dist_m / cell_m).ceil() as i32 + 1;
        let (cy, cx) = cell_of(ll);

        let mut best: Option<Snap> = None;
        for dy in -reach..=reach {
            for dx in -reach..=reach {
                let Some(bucket) = self.index.get(&(cy + dy, cx + dx)) else {
                    continue;
                };
                for (wi, si) in bucket {
                    let way = &self.ways[*wi as usize];
                    let a = way.points[*si as usize];
                    let b = way.points[*si as usize + 1];
                    let (t, dist) = point_segment(ll, a, b, mx, my);
                    if dist <= max_dist_m && best.is_none_or(|s| dist < s.dist_m) {
                        best = Some(Snap {
                            way_id: way.id,
                            seg: *si as usize,
                            t,
                            pos: crate::geo::lerp_latlon(a, b, t),
                            dist_m: dist,
                        });
                    }
                }
            }
        }
        best
    }

    /// Geometry between two positions snapped **onto the same OSM way**. This is
    /// segment following: two clicks on one trail follow its real shape instead
    /// of the straight chord.
    pub fn follow(&self, from: &Snap, to: &Snap) -> Option<Vec<LatLon>> {
        if from.way_id != to.way_id {
            return None;
        }
        let way = self.way_by_id(from.way_id)?;
        let forward = (from.seg, from.t) <= (to.seg, to.t);
        let (a, b) = if forward { (from, to) } else { (to, from) };

        let mut out = vec![a.pos];
        for i in (a.seg + 1)..=b.seg {
            out.push(way.points[i]);
        }
        out.push(b.pos);
        if !forward {
            out.reverse();
        }
        Some(out)
    }
}

/// Projection of a point onto a segment, in a local planar approximation.
/// Returns (t in [0,1], distance in metres).
fn point_segment(p: LatLon, a: LatLon, b: LatLon, mx: f64, my: f64) -> (f64, f64) {
    let ax = (a.lon - p.lon) * mx;
    let ay = (a.lat - p.lat) * my;
    let bx = (b.lon - p.lon) * mx;
    let by = (b.lat - p.lat) * my;
    let (dx, dy) = (bx - ax, by - ay);
    let len2 = dx * dx + dy * dy;
    let t = if len2 <= f64::EPSILON {
        0.0
    } else {
        ((-ax * dx - ay * dy) / len2).clamp(0.0, 1.0)
    };
    let (px, py) = (ax + dx * t, ay + dy * t);
    (t, (px * px + py * py).sqrt())
}

/// Length of a polyline, in metres.
pub fn path_length_m(points: &[LatLon]) -> f64 {
    points
        .windows(2)
        .map(|w| crate::geo::haversine_m(w[0], w[1]))
        .sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Builds a way through the archive conversion, so these tests exercise the
    /// same path the application uses rather than a hand-made shortcut.
    ///
    /// ⚠️ `synthetic` is passed in, never restarted per way. An earlier version
    /// of this helper reset it on every call, and the first point of one way
    /// silently collided with the last point of another: `Graph::build` counts
    /// identities to find junctions, so it welded two unrelated trail ends
    /// together. That is the phantom junction, and it is exactly why
    /// `TrailArchive` keeps one counter for the whole network.
    fn way(
        synthetic: &mut i64,
        id: i64,
        kind: u8,
        pts: &[(f64, f64)],
        shared: &[Option<i64>],
    ) -> Way {
        Way::from_archive(
            trailfmt::ArchiveWay {
                id,
                kind,
                sac: 0,
                name: None,
                points: pts.to_vec(),
                shared: shared.to_vec(),
            },
            synthetic,
        )
        .expect("a valid way")
    }

    /// Two ways meeting at node 12: an L, then a spur running north from its end.
    ///
    /// ```text
    ///        (13) 45.9030
    ///         |
    /// (11)---(12)   45.9010
    ///  |
    /// (10)          45.9000
    /// ```
    fn network() -> TrailNetwork {
        let mut net = TrailNetwork::default();
        let mut synthetic = 0i64;
        net.insert(way(
            &mut synthetic,
            1,
            trailfmt::kind::PATH,
            &[(45.9000, 6.8700), (45.9010, 6.8700), (45.9010, 6.8720)],
            &[None, None, Some(12)],
        ));
        net.insert(way(
            &mut synthetic,
            2,
            trailfmt::kind::TRACK,
            &[(45.9010, 6.8720), (45.9030, 6.8720)],
            &[Some(12), None],
        ));
        net
    }

    /// A way arriving twice must not be stored twice. Neighbouring tiles both
    /// carry the ways that straddle their border.
    #[test]
    fn insertion_is_idempotent() {
        let mut net = network();
        assert_eq!(net.len(), 2);
        let mut synthetic = -50i64;
        net.insert(way(
            &mut synthetic,
            1,
            trailfmt::kind::PATH,
            &[(45.9000, 6.8700), (45.9010, 6.8700)],
            &[None, None],
        ));
        assert_eq!(net.len(), 2, "way 1 came back and must be ignored");
    }

    /// Degenerate geometry never enters the network.
    #[test]
    fn a_one_point_way_is_refused() {
        let net = TrailNetwork::default();
        let lone = trailfmt::ArchiveWay {
            id: 9,
            kind: trailfmt::kind::PATH,
            sac: 0,
            name: None,
            points: vec![(45.9, 6.87)],
            shared: vec![None],
        };
        let mut synthetic = 0;
        assert!(Way::from_archive(lone, &mut synthetic).is_none());
        assert_eq!(net.len(), 0);
    }

    /// The archive's class and difficulty codes must survive into the model, and
    /// unshared points must each get their own identity.
    #[test]
    fn archive_conversion_keeps_identity_and_metadata() {
        let mut synthetic = 0;
        let w = Way::from_archive(
            trailfmt::ArchiveWay {
                id: 7,
                kind: trailfmt::kind::STEPS,
                sac: 4,
                name: Some("Échelles".to_owned()),
                points: vec![(45.9, 6.87), (45.91, 6.88), (45.92, 6.89)],
                shared: vec![Some(100), None, Some(200)],
            },
            &mut synthetic,
        )
        .unwrap();

        assert_eq!(w.kind, WayKind::Steps);
        assert_eq!(w.sac_scale.as_deref(), Some("alpine_hiking"));
        assert_eq!(w.name.as_deref(), Some("Échelles"));
        assert_eq!(w.nodes[0], 100);
        assert_eq!(w.nodes[2], 200);
        assert!(
            w.nodes[1] < 0,
            "an unshared point must not borrow a real OSM id: {:?}",
            w.nodes
        );
        // Bounds are derived, and used to cull at render time.
        assert!((w.bounds.0 - 45.9).abs() < 1e-9 && (w.bounds.3 - 6.89).abs() < 1e-9);
    }

    /// An unknown class must still draw rather than vanish: a future archive
    /// version may add classes an older application has never heard of.
    #[test]
    fn an_unknown_class_falls_back_rather_than_disappearing() {
        assert_eq!(WayKind::from_code(200), WayKind::Road);
        assert_eq!(WayKind::from_code(trailfmt::kind::PATH), WayKind::Path);
    }

    #[test]
    fn snapping_finds_the_nearest_segment() {
        let net = network();
        // ~15 m east of the first segment (vertical, lon 6.8700).
        let click = LatLon::new(45.9005, 6.87019);
        let snap = net.snap(click, SNAP_RADIUS_M).expect("must snap");
        assert_eq!(snap.way_id, 1);
        assert_eq!(snap.seg, 0);
        assert!(snap.dist_m < 20.0, "dist = {}", snap.dist_m);
        assert!((snap.pos.lon - 6.8700).abs() < 1e-5);
        assert!((snap.t - 0.5).abs() < 0.05, "t = {}", snap.t);
    }

    #[test]
    fn snapping_refuses_beyond_the_radius() {
        let net = network();
        // ~800 m west of everything.
        assert!(net.snap(LatLon::new(45.9005, 6.860), SNAP_RADIUS_M).is_none());
    }

    #[test]
    fn segment_following_works_both_ways() {
        let net = network();
        let a = net.snap(LatLon::new(45.9002, 6.8700), SNAP_RADIUS_M).unwrap();
        let b = net.snap(LatLon::new(45.9010, 6.8715), SNAP_RADIUS_M).unwrap();
        let forward = net.follow(&a, &b).expect("same way");
        // Must go through the elbow (45.9010, 6.8700), not cut the corner.
        assert!(forward.len() >= 3, "{forward:?}");
        assert!(forward
            .iter()
            .any(|p| (p.lat - 45.9010).abs() < 1e-5 && (p.lon - 6.8700).abs() < 1e-5));
        let backward = net.follow(&b, &a).unwrap();
        assert_eq!(backward.len(), forward.len());
        assert!((backward[0].lat - forward.last().unwrap().lat).abs() < 1e-9);
    }

    #[test]
    fn no_following_between_different_ways() {
        let net = network();
        let a = net.snap(LatLon::new(45.9002, 6.8700), SNAP_RADIUS_M).unwrap();
        let b = net.snap(LatLon::new(45.9025, 6.8720), SNAP_RADIUS_M).unwrap();
        assert_eq!(b.way_id, 2);
        // Joining the two ways is the graph's job.
        assert!(net.follow(&a, &b).is_none());
    }

    /// The shared node must become a junction, and **only** it.
    ///
    /// ⚠️ This is the phantom-junction guard. It fails loudly if unshared points
    /// ever end up sharing an identity — the failure mode that produces an
    /// isochrone teleporting between unrelated trails, with no error anywhere.
    #[test]
    fn the_shared_node_is_the_only_junction() {
        let net = network();
        let ids: Vec<i64> = net.ways().iter().flat_map(|w| w.nodes.clone()).collect();
        let distinct: std::collections::HashSet<i64> = ids.iter().copied().collect();
        assert_eq!(ids.len(), 5, "three points plus two");
        assert_eq!(distinct.len(), 4, "exactly one identity is shared: {ids:?}");

        let graph = crate::graph::Graph::build(&net);
        assert_eq!(graph.edges.len(), 2);
        assert_eq!(graph.node_count(), 3, "two ends plus one junction");
    }

    #[test]
    fn polyline_length_is_measured_on_the_ground() {
        // ~1.11 km north-south.
        let d = path_length_m(&[LatLon::new(45.0, 6.0), LatLon::new(45.01, 6.0)]);
        assert!((d - 1112.0).abs() < 5.0, "{d}");
        assert_eq!(path_length_m(&[LatLon::new(45.0, 6.0)]), 0.0);
    }
}
