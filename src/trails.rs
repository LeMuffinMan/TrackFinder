//! OSM trails: Overpass request per zone, local cache, spatial index, snapping.
//!
//! The network is **local and on demand** — never country-wide. The world is cut
//! into `ZONE_DEG` zones, we only ask for the ones we need, and we never ask
//! twice.

use std::collections::{HashMap, HashSet, VecDeque};
use std::rc::Rc;
use std::sync::{Arc, Mutex};

use crate::geo::LatLon;

/// Two zone sizes, in degrees.
///
/// - level 0 (~2.2 km): the click. Trade-off measured over Chamonix — 0.03° ×
///   0.04° weighs 1.15 MB of JSON, 199 KB on the wire after gzip. Any bigger and
///   the response becomes painful in town; any smaller and we multiply requests.
/// - level 1 (~11 km): the isochrone, which needs a whole basin at once.
///   Covering 25 km with level 0 zones would take ~500 requests.
///
/// Both levels feed the **same** `TrailNetwork`: insertion is idempotent by way
/// id, so the overlap costs nothing.
pub const ZONE_LEVELS: [f64; 2] = [0.02, 0.10];

/// Size of the click zone — the finer one.
pub const ZONE_DEG: f64 = ZONE_LEVELS[0];

/// Side of a spatial index cell, in degrees (~220 m).
const INDEX_CELL_DEG: f64 = 0.002;

/// Largest accepted distance between a click and the nearest trail.
pub const SNAP_RADIUS_M: f64 = 60.0;

const HIGHWAY_FILTER: &str =
    "^(path|track|footway|bridleway|steps|cycleway|pedestrian|living_street|unclassified|residential|tertiary)$";

// ---------------------------------------------------------------------------
// Zones
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ZoneKey {
    pub level: u8,
    pub lat: i32,
    pub lon: i32,
}

impl ZoneKey {
    /// The fine zone, the one a click needs.
    pub fn of(ll: LatLon) -> Self {
        Self::of_level(ll, 0)
    }

    pub fn of_level(ll: LatLon, level: u8) -> Self {
        let size = ZONE_LEVELS[level as usize];
        Self {
            level,
            lat: (ll.lat / size).floor() as i32,
            lon: (ll.lon / size).floor() as i32,
        }
    }

    pub fn size(self) -> f64 {
        ZONE_LEVELS[self.level as usize]
    }

    /// (south, west, north, east)
    pub fn bbox(self) -> (f64, f64, f64, f64) {
        let size = self.size();
        let s = self.lat as f64 * size;
        let w = self.lon as f64 * size;
        (s, w, s + size, w + size)
    }
}

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
    fn from_tag(tag: &str) -> Self {
        match tag {
            "path" | "bridleway" => WayKind::Path,
            "track" => WayKind::Track,
            "footway" | "pedestrian" => WayKind::Footway,
            "steps" => WayKind::Steps,
            "cycleway" => WayKind::Cycleway,
            _ => WayKind::Road,
        }
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
        // Overpass returns whole ways, not clipped to the bbox: two neighbouring
        // zones therefore report the same border ways.
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

// ---------------------------------------------------------------------------
// Overpass source
// ---------------------------------------------------------------------------

pub type TrailCallback = Box<dyn FnOnce(Result<String, String>) + Send + 'static>;

pub trait TrailSource {
    /// `attempt` drives endpoint switching: Overpass is a public, quota-limited
    /// service that regularly answers 504 "too busy" — or stops answering at all.
    fn request(&self, zone: ZoneKey, attempt: usize, done: TrailCallback);
    fn endpoint_count(&self) -> usize;

    /// The endpoint currently preferred, for the debug panel. When loading drags,
    /// this is the first thing worth looking at.
    fn preferred_endpoint(&self) -> Option<String> {
        None
    }
}

pub struct OverpassSource {
    /// **Planet-wide** instances that send `Access-Control-Allow-Origin: *`
    /// (verified 2026-09-03 — the header only appears when the request carries
    /// an `Origin`, so it is invisible to bare `curl`).
    ///
    /// ⚠️ `overpass.osm.ch` does have CORS but only serves a **Swiss** extract:
    /// it answers `200` with `elements: []` everywhere else in France. A regional
    /// mirror is invisible in the status code — it shows up in the data. Do not
    /// add it here.
    endpoints: Vec<String>,
    /// Index of the endpoint that last answered with data.
    ///
    /// ⚠️ **This is what makes bulk loading bearable.** Without it every zone
    /// restarts the failover at instance 0, so a dead primary is rediscovered
    /// once per zone: measured 2026-09-04, `overpass-api.de` refusing connections
    /// and `overpass.kumi.systems` hanging cost ~13 s of dead time *per zone*
    /// before the third instance answered in 1.65 s. Twelve zones spent two and a
    /// half minutes doing nothing at all.
    ///
    /// `Arc<AtomicUsize>` rather than `Cell`: the callback that records success
    /// has to be `Send`.
    preferred: Arc<std::sync::atomic::AtomicUsize>,
}

impl Default for OverpassSource {
    fn default() -> Self {
        Self {
            endpoints: vec![
                "https://overpass-api.de/api/interpreter".to_owned(),
                "https://overpass.kumi.systems/api/interpreter".to_owned(),
                "https://maps.mail.ru/osm/tools/overpass/api/interpreter".to_owned(),
            ],
            preferred: Arc::new(std::sync::atomic::AtomicUsize::new(0)),
        }
    }
}

impl OverpassSource {
    /// Endpoint to use for this attempt, counted from the last one that worked.
    ///
    /// Attempt 0 is the preferred instance, then the others in order, wrapping —
    /// so a full round still covers every endpoint exactly once.
    fn endpoint_index(&self, attempt: usize) -> usize {
        let base = self.preferred.load(std::sync::atomic::Ordering::Relaxed);
        (base + attempt) % self.endpoints.len()
    }

    /// Records the instance that answered, so the next zone starts there.
    ///
    /// Free-standing over the shared counter rather than a `&self` method: the
    /// callback that calls it has to be `Send` and cannot hold the source.
    fn note_success(preferred: &std::sync::atomic::AtomicUsize, index: usize) {
        preferred.store(index, std::sync::atomic::Ordering::Relaxed);
    }

    pub fn query(zone: ZoneKey) -> String {
        let (s, w, n, e) = zone.bbox();
        format!(
            "[out:json][timeout:60];\nway[\"highway\"~\"{HIGHWAY_FILTER}\"]({s:.4},{w:.4},{n:.4},{e:.4});\nout geom;"
        )
    }
}

impl TrailSource for OverpassSource {
    fn request(&self, zone: ZoneKey, attempt: usize, done: TrailCallback) {
        let index = self.endpoint_index(attempt);
        let url = self.endpoints[index].clone();
        let mut request = ehttp::Request::post(url, OverpassSource::query(zone).into_bytes());
        request
            .headers
            .insert("Content-Type", "text/plain;charset=UTF-8");
        let preferred = Arc::clone(&self.preferred);
        ehttp::fetch(request, move |result| {
            let body = match result {
                Err(e) => Err(e),
                Ok(resp) if !resp.ok => Err(format!("HTTP {} {}", resp.status, resp.status_text)),
                Ok(resp) => resp
                    .text()
                    .map(|t| t.to_owned())
                    .ok_or_else(|| "non-textual response".to_owned()),
            };
            // Only a body that actually looks like the JSON we asked for counts:
            // Overpass serves its overload page as HTML with a 200, and sticking
            // to an instance that only ever returns that would be worse than not
            // remembering anything.
            if body.as_deref().is_ok_and(|b| b.trim_start().starts_with('{')) {
                Self::note_success(&preferred, index);
            }
            done(body);
        });
    }

    fn endpoint_count(&self) -> usize {
        self.endpoints.len()
    }

    fn preferred_endpoint(&self) -> Option<String> {
        self.endpoints.get(self.endpoint_index(0)).cloned()
    }
}

// ---------------------------------------------------------------------------
// Response parsing
// ---------------------------------------------------------------------------

pub fn parse_overpass(body: &str) -> Result<Vec<Way>, String> {
    let root: serde_json::Value =
        serde_json::from_str(body).map_err(|e| format!("Overpass JSON: {e}"))?;
    // Overpass reports some errors as HTML with a 200 status code; we then land
    // on the parse error above rather than on silence.
    let elements = root
        .get("elements")
        .and_then(|e| e.as_array())
        .ok_or_else(|| "no `elements` field".to_owned())?;

    let mut ways = Vec::with_capacity(elements.len());
    for el in elements {
        if el.get("type").and_then(|t| t.as_str()) != Some("way") {
            continue;
        }
        let Some(id) = el.get("id").and_then(|v| v.as_i64()) else {
            continue;
        };
        let Some(geometry) = el.get("geometry").and_then(|g| g.as_array()) else {
            continue;
        };
        let points: Vec<LatLon> = geometry
            .iter()
            .filter_map(|p| {
                Some(LatLon::new(
                    p.get("lat")?.as_f64()?,
                    p.get("lon")?.as_f64()?,
                ))
            })
            .collect();
        if points.len() < 2 {
            continue;
        }
        let nodes = el
            .get("nodes")
            .and_then(|n| n.as_array())
            .map(|a| a.iter().filter_map(|v| v.as_i64()).collect())
            .unwrap_or_default();
        let tags = el.get("tags");
        let tag = |k: &str| {
            tags.and_then(|t| t.get(k))
                .and_then(|v| v.as_str())
                .map(|s| s.to_owned())
        };
        let bounds = points.iter().fold(
            (f64::MAX, f64::MAX, f64::MIN, f64::MIN),
            |b, p| (b.0.min(p.lat), b.1.min(p.lon), b.2.max(p.lat), b.3.max(p.lon)),
        );
        ways.push(Way {
            id,
            bounds,
            kind: WayKind::from_tag(tag("highway").as_deref().unwrap_or("")),
            name: tag("name"),
            nodes,
            points,
            sac_scale: tag("sac_scale"),
        });
    }
    Ok(ways)
}

/// Distance from a point to the nearest edge of a zone — zero when the point is
/// inside it. Used to order zone requests: the one under the eye first.
fn zone_distance_m(key: ZoneKey, from: LatLon) -> f64 {
    let (s, w, n, e) = key.bbox();
    crate::geo::haversine_m(
        from,
        LatLon::new(from.lat.clamp(s, n), from.lon.clamp(w, e)),
    )
}

/// Length of a polyline, in metres.
pub fn path_length_m(points: &[LatLon]) -> f64 {
    points
        .windows(2)
        .map(|w| crate::geo::haversine_m(w[0], w[1]))
        .sum()
}

// ---------------------------------------------------------------------------
// Per-zone cache
// ---------------------------------------------------------------------------

/// (zone, attempt number, response body)
type ZoneReply = (ZoneKey, usize, Result<String, String>);

/// Concurrent requests to Overpass. `https://overpass-api.de/api/status`
/// announces "Rate limit: 2": beyond that the extra ones queue server side and a
/// small urgent zone ends up stuck behind a large basin.
const MAX_IN_FLIGHT: usize = 2;

/// Delay past which a request is considered lost and we switch endpoint.
/// Neither `fetch` (browser) nor `ehttp` exposes a timeout: without this guard
/// an instance that never answers holds a slot for the whole session.
///
/// Seen for real on 2026-09-03: `overpass-api.de` answers `200` over IPv4 and
/// nothing at all over IPv6 (TLS connection reset) from the dev machine. The
/// browser tries IPv6 first, so without this delay the app would sit on a dead
/// instance.
const REQUEST_TIMEOUT_S: f64 = 12.0;

pub struct TrailStore {
    pub net: TrailNetwork,
    source: Rc<dyn TrailSource>,
    inbox: Arc<Mutex<Vec<ZoneReply>>>,
    loaded: HashSet<ZoneKey>,
    /// Zones asked for but not yet sent.
    queue: VecDeque<(ZoneKey, usize)>,
    in_flight: usize,
    /// In-flight zone → (attempt number, send instant).
    pending: HashMap<ZoneKey, (usize, web_time::Instant)>,
    failed: HashMap<ZoneKey, String>,
    pub last_error: Option<String>,
}

impl TrailStore {
    pub fn new(source: Rc<dyn TrailSource>) -> Self {
        Self {
            net: TrailNetwork::default(),
            source,
            inbox: Arc::new(Mutex::new(Vec::new())),
            loaded: HashSet::new(),
            queue: VecDeque::new(),
            in_flight: 0,
            pending: HashMap::new(),
            failed: HashMap::new(),
            last_error: None,
        }
    }

    pub fn overpass() -> Self {
        Self::new(Rc::new(OverpassSource::default()))
    }

    /// True when this point is covered by a zone already in memory, **at any
    /// level**: a large isochrone zone covers the fine zones inside it.
    pub fn zone_ready(&self, ll: LatLon) -> bool {
        (0..ZONE_LEVELS.len() as u8).any(|l| self.loaded.contains(&ZoneKey::of_level(ll, l)))
    }

    pub fn zone_failed(&self, ll: LatLon) -> Option<&str> {
        self.failed.get(&ZoneKey::of(ll)).map(|s| s.as_str())
    }

    /// Requests the zone containing this point if it is missing. Idempotent —
    /// that is the whole cache: a loaded zone is never requested again.
    pub fn ensure(&mut self, ll: LatLon, ctx: &egui::Context) {
        if self.zone_ready(ll) {
            return;
        }
        self.ensure_key(ZoneKey::of(ll), ctx);
    }

    /// True when this zone is loaded, in flight, queued or known to have failed:
    /// nothing more will be sent for it right now.
    fn zone_settled(&self, key: &ZoneKey) -> bool {
        self.loaded.contains(key)
            || self.pending.contains_key(key)
            || self.queue.iter().any(|(k, _)| k == key)
            || self.failed.contains_key(key)
    }

    fn ensure_key(&mut self, key: ZoneKey, ctx: &egui::Context) {
        if self.zone_settled(&key) {
            return;
        }
        self.spawn(key, 0, ctx);
    }

    /// Fine zones covering a lat/lon window, in the order they should be asked
    /// for: nearest to `center` first, so a cap keeps the useful ones.
    pub fn zones_covering(
        &self,
        south: f64,
        west: f64,
        north: f64,
        east: f64,
        center: LatLon,
    ) -> Vec<ZoneKey> {
        let size = ZONE_DEG;
        let mut keys = Vec::new();
        let mut lat = (south / size).floor() * size;
        while lat <= north {
            let mut lon = (west / size).floor() * size;
            while lon <= east {
                keys.push(ZoneKey::of(LatLon::new(
                    lat + size / 2.0,
                    lon + size / 2.0,
                )));
                lon += size;
            }
            lat += size;
        }
        // The zone actually holding the centre comes first, explicitly: a centre
        // sitting exactly on a zone boundary is equidistant from both, and the
        // one the user is looking at is the one worth requesting first.
        let home = ZoneKey::of(center);
        keys.sort_by_key(|k| (*k != home, zone_distance_m(*k, center) as i64));
        keys
    }

    /// Requests these zones, capped. Returns how many were actually sent.
    pub fn ensure_zones(
        &mut self,
        keys: &[ZoneKey],
        max_zones: usize,
        ctx: &egui::Context,
    ) -> usize {
        let mut asked = 0;
        for key in keys.iter().take(max_zones) {
            if !self.zone_settled(key) {
                asked += 1;
            }
            self.ensure_key(*key, ctx);
        }
        asked
    }

    /// The large zones covering a disc, nearest to the centre first.
    ///
    /// Coarse level: covering 25 km with click-sized zones would be ~500
    /// requests. A pure query — nothing is sent.
    pub fn area_zones(&self, center: LatLon, radius_m: f64) -> Vec<ZoneKey> {
        let level = (ZONE_LEVELS.len() - 1) as u8;
        let size = ZONE_LEVELS[level as usize];
        let dlat = radius_m / 111_132.0;
        let dlon = radius_m / (111_320.0 * center.lat.to_radians().cos()).max(1.0);

        let mut keys = Vec::new();
        let mut lat = ((center.lat - dlat) / size).floor() * size;
        while lat <= center.lat + dlat {
            let mut lon = ((center.lon - dlon) / size).floor() * size;
            while lon <= center.lon + dlon {
                keys.push(ZoneKey::of_level(
                    LatLon::new(lat + size / 2.0, lon + size / 2.0),
                    level,
                ));
                lon += size;
            }
            lat += size;
        }
        // Nearest first, and the zone actually holding the centre ahead of the
        // rest: whatever a cap cuts should be the far edge, never the middle.
        let home = ZoneKey::of_level(center, level);
        keys.sort_by_key(|k| (*k != home, zone_distance_m(*k, center) as i64));
        keys
    }

    /// Requests at most `max_new` of these zones that are not already loaded,
    /// in flight, queued or failed. Returns how many were actually sent.
    ///
    /// The cap is on **new** requests, not on the list: that is what lets a
    /// caller grow an area a couple of zones at a time instead of dumping fifty
    /// on a service that grants two slots.
    pub fn ensure_some(
        &mut self,
        keys: &[ZoneKey],
        max_new: usize,
        ctx: &egui::Context,
    ) -> usize {
        let mut asked = 0;
        for key in keys {
            if asked >= max_new {
                break;
            }
            if self.zone_settled(key) {
                continue;
            }
            self.ensure_key(*key, ctx);
            asked += 1;
        }
        asked
    }

    /// How many of these zones are already in memory.
    pub fn zones_loaded(&self, keys: &[ZoneKey]) -> usize {
        keys.iter().filter(|k| self.loaded.contains(k)).count()
    }

    /// Loads the large zones covering a disc — what the isochrone needs.
    /// Returns the number of zones requested, capped: Overpass is a public
    /// quota-limited service, and an 8 h isochrone already spans ~25 km.
    ///
    /// Retries the failed ones on the way: this is the explicit button, and the
    /// user asking again means "try again".
    pub fn ensure_area(
        &mut self,
        center: LatLon,
        radius_m: f64,
        max_zones: usize,
        ctx: &egui::Context,
    ) -> usize {
        let keys: Vec<ZoneKey> = self
            .area_zones(center, radius_m)
            .into_iter()
            .take(max_zones)
            .collect();
        for key in &keys {
            self.failed.remove(key);
        }
        self.ensure_some(&keys, max_zones, ctx)
    }

    /// Retries the failed zones (the "retry" button).
    pub fn retry_failed(&mut self, ctx: &egui::Context) {
        let zones: Vec<ZoneKey> = self.failed.keys().copied().collect();
        self.failed.clear();
        self.last_error = None;
        for key in zones {
            self.spawn(key, 0, ctx);
        }
    }

    fn spawn(&mut self, key: ZoneKey, attempt: usize, ctx: &egui::Context) {
        // Small zones (the user's click) jump ahead of large isochrone basins:
        // it is the interaction that waits, not the other way round.
        if key.level == 0 {
            self.queue.push_front((key, attempt));
        } else {
            self.queue.push_back((key, attempt));
        }
        self.dispatch(ctx);
    }

    fn dispatch(&mut self, ctx: &egui::Context) {
        while self.in_flight < MAX_IN_FLIGHT {
            let Some((key, attempt)) = self.queue.pop_front() else {
                return;
            };
            self.send(key, attempt, ctx);
        }
    }

    fn send(&mut self, key: ZoneKey, attempt: usize, ctx: &egui::Context) {
        self.in_flight += 1;
        self.pending.insert(key, (attempt, web_time::Instant::now()));
        let inbox = Arc::clone(&self.inbox);
        let ctx = ctx.clone();
        self.source.request(
            key,
            attempt,
            Box::new(move |body| {
                inbox.lock().unwrap().push((key, attempt, body));
                ctx.request_repaint();
            }),
        );
    }

    /// True when new zones entered the network.
    pub fn pump(&mut self, ctx: &egui::Context) -> bool {
        let arrived = {
            let mut inbox = self.inbox.lock().unwrap();
            std::mem::take(&mut *inbox)
        };
        let mut changed = false;
        for (key, attempt, body) in arrived {
            // Reply from an attempt already abandoned on timeout: its slot was
            // given back, do not give it back twice.
            if self.pending.get(&key).map(|(a, _)| *a) != Some(attempt) {
                continue;
            }
            self.pending.remove(&key);
            self.in_flight = self.in_flight.saturating_sub(1);
            match body.and_then(|b| parse_overpass(&b)) {
                Ok(ways) => {
                    for way in ways {
                        self.net.insert(way);
                    }
                    self.loaded.insert(key);
                    changed = true;
                }
                Err(e) => {
                    // 504 "too busy" is the common case: switch endpoint before
                    // giving up.
                    if attempt + 1 < self.source.endpoint_count() {
                        log::warn!("Overpass {e} — trying another instance");
                        self.spawn(key, attempt + 1, ctx);
                    } else {
                        log::warn!("Overpass gave up: {e}");
                        self.last_error = Some(e.clone());
                        self.failed.insert(key, e);
                    }
                }
            }
        }
        self.expire(ctx);
        self.dispatch(ctx);
        changed
    }

    /// Abandons requests that grew too old and switches endpoint.
    fn expire(&mut self, ctx: &egui::Context) {
        let stale: Vec<(ZoneKey, usize)> = self
            .pending
            .iter()
            .filter(|(_, (_, sent))| sent.elapsed().as_secs_f64() > REQUEST_TIMEOUT_S)
            .map(|(k, (a, _))| (*k, *a))
            .collect();
        for (key, attempt) in stale {
            self.pending.remove(&key);
            self.in_flight = self.in_flight.saturating_sub(1);
            let msg = format!("no answer within {REQUEST_TIMEOUT_S:.0} s");
            if attempt + 1 < self.source.endpoint_count() {
                log::warn!("Overpass {msg} — trying another instance");
                self.spawn(key, attempt + 1, ctx);
            } else {
                self.last_error = Some(msg.clone());
                self.failed.insert(key, msg);
            }
        }
    }

    /// The Overpass instance requests currently go to first.
    pub fn preferred_endpoint(&self) -> Option<String> {
        self.source.preferred_endpoint()
    }

    /// (loaded zones, in progress, failed, ways)
    pub fn stats(&self) -> (usize, usize, usize, usize) {
        (
            self.loaded.len(),
            self.pending.len() + self.queue.len(),
            self.failed.len(),
            self.net.len(),
        )
    }
}

#[cfg(test)]
pub mod tests {
    use super::*;

    /// Two L-shaped ways that touch, around (45.900, 6.870).
    const SAMPLE: &str = r#"{"version":0.6,"elements":[
      {"type":"way","id":1,"nodes":[10,11,12],"tags":{"highway":"path","name":"Test Trail","sac_scale":"hiking"},
       "geometry":[{"lat":45.9000,"lon":6.8700},{"lat":45.9010,"lon":6.8700},{"lat":45.9010,"lon":6.8720}]},
      {"type":"way","id":2,"nodes":[12,13],"tags":{"highway":"track"},
       "geometry":[{"lat":45.9010,"lon":6.8720},{"lat":45.9030,"lon":6.8720}]},
      {"type":"node","id":10,"lat":45.9,"lon":6.87},
      {"type":"way","id":3,"tags":{"highway":"path"},"geometry":[{"lat":45.9,"lon":6.87}]}
    ]}"#;

    fn network() -> TrailNetwork {
        let mut net = TrailNetwork::default();
        for way in parse_overpass(SAMPLE).unwrap() {
            net.insert(way);
        }
        net
    }

    #[test]
    fn parsing_skips_nodes_and_degenerate_geometry() {
        let ways = parse_overpass(SAMPLE).unwrap();
        assert_eq!(ways.len(), 2, "the node and the one-point way are dropped");
        assert_eq!(ways[0].kind, WayKind::Path);
        assert_eq!(ways[0].name.as_deref(), Some("Test Trail"));
        assert_eq!(ways[0].nodes, vec![10, 11, 12]);
        assert_eq!(ways[1].kind, WayKind::Track);
    }

    #[test]
    fn an_html_error_is_reported() {
        assert!(parse_overpass("<html>too busy</html>").is_err());
    }

    #[test]
    fn insertion_is_idempotent() {
        // Overpass returns whole ways: two neighbouring zones report the same
        // border ways, and they must not be duplicated.
        let mut net = network();
        for way in parse_overpass(SAMPLE).unwrap() {
            net.insert(way);
        }
        assert_eq!(net.len(), 2);
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
        assert!((snap.pos.lon - 6.8700).abs() < 1e-6);
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
        let forward = net.follow(&a, &b).expect("same OSM way");
        // Must go through the elbow (45.9010, 6.8700), not cut the corner.
        assert!(forward.len() >= 3, "{forward:?}");
        assert!(forward
            .iter()
            .any(|p| (p.lat - 45.9010).abs() < 1e-9 && (p.lon - 6.8700).abs() < 1e-9));
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

    #[test]
    fn a_zone_contains_its_own_point() {
        let ll = LatLon::new(45.9234, 6.8712);
        let (s, w, n, e) = ZoneKey::of(ll).bbox();
        assert!(s <= ll.lat && ll.lat < n && w <= ll.lon && ll.lon < e);
        assert!((n - s - ZONE_DEG).abs() < 1e-12);
        // Two points in the same zone share the key: that is what stops the zone
        // from being requested twice.
        assert_eq!(ZoneKey::of(ll), ZoneKey::of(LatLon::new(45.9236, 6.8714)));
    }

    #[test]
    fn the_overpass_query_is_well_formed() {
        let q = OverpassSource::query(ZoneKey::of(LatLon::new(45.92, 6.87)));
        assert!(q.contains("[out:json]"), "{q}");
        assert!(q.contains("out geom;"), "{q}");
        assert!(q.contains("(45.9200,6.8600,45.9400,6.8800)"), "{q}");
    }

    /// The zones covering a window must contain its corners, and come out
    /// nearest-first so that a cap keeps what is under the eye.
    #[test]
    fn covering_zones_are_ordered_from_the_centre() {
        let store = TrailStore::new(Rc::new(OverpassSource::default()));
        let center = LatLon::new(45.92, 6.87);
        let keys = store.zones_covering(45.90, 6.85, 45.94, 6.89, center);
        assert!(keys.len() >= 4, "{} zones", keys.len());
        assert_eq!(keys[0], ZoneKey::of(center), "the centre comes first");
        assert!(keys.contains(&ZoneKey::of(LatLon::new(45.901, 6.851))));
        assert!(keys.contains(&ZoneKey::of(LatLon::new(45.939, 6.889))));
        // Nothing has been requested: this is a pure query.
        assert_eq!(store.stats(), (0, 0, 0, 0));
    }

    /// A dead instance must be discovered once, not once per zone.
    ///
    /// This is the whole failover cost: measured 2026-09-04, walking two
    /// unreachable instances cost ~13 s before the third answered in 1.65 s.
    /// Paying that per zone turned a ten-second basin into two and a half
    /// minutes of waiting.
    #[test]
    fn the_working_instance_is_remembered() {
        let src = OverpassSource::default();
        let n = src.endpoint_count();
        assert!(n >= 3, "the test needs a few instances");

        // Cold: attempts walk the list from the top.
        assert_eq!(src.endpoint_index(0), 0);
        assert_eq!(src.endpoint_index(1), 1);
        assert_eq!(src.endpoint_index(2), 2);

        // The third one answered. Every later zone starts there.
        OverpassSource::note_success(&src.preferred, 2);
        assert_eq!(src.endpoint_index(0), 2, "the next zone must start there");
        assert_eq!(
            src.preferred_endpoint().unwrap(),
            src.endpoints[2],
            "and the panel must say so"
        );

        // A full round still covers every instance exactly once, wrapping.
        let visited: std::collections::HashSet<usize> =
            (0..n).map(|a| src.endpoint_index(a)).collect();
        assert_eq!(visited.len(), n, "a round must not skip or repeat: {visited:?}");
    }

    /// Fetches a zone synchronously, switching instance as needed.
    /// Overpass regularly answers 504 "too busy".
    #[cfg(not(target_arch = "wasm32"))]
    pub fn fetch_zone_blocking(zone: ZoneKey) -> Result<Vec<Way>, String> {
        let src = OverpassSource::default();
        let mut last_err = String::new();
        src.endpoints
            .iter()
            .find_map(|url| {
                let mut request =
                    ehttp::Request::post(url.clone(), OverpassSource::query(zone).into_bytes());
                request
                    .headers
                    .insert("Content-Type", "text/plain;charset=UTF-8");
                match ehttp::fetch_blocking(&request) {
                    Ok(resp) if resp.ok => parse_overpass(resp.text().unwrap_or_default()).ok(),
                    Ok(resp) => {
                        last_err = format!("{url} → HTTP {}", resp.status);
                        None
                    }
                    Err(e) => {
                        last_err = format!("{url} → {e}");
                        None
                    }
                }
            })
            .ok_or_else(|| format!("no Overpass instance answered: {last_err}"))
    }

    /// End to end against Overpass: `cargo test -- --ignored`.
    #[test]
    #[ignore = "network"]
    #[cfg(not(target_arch = "wasm32"))]
    fn real_zone_over_chamonix() {
        let zone = ZoneKey::of(LatLon::new(45.92, 6.87));
        let ways = fetch_zone_blocking(zone).unwrap();
        assert!(ways.len() > 50, "{} ways", ways.len());

        let mut net = TrailNetwork::default();
        for way in ways {
            net.insert(way);
        }
        // Chamonix station sits a few metres from a pedestrian way.
        let snap = net.snap(LatLon::new(45.9237, 6.8703), SNAP_RADIUS_M);
        assert!(snap.is_some(), "no trail near the centre of Chamonix");
    }
}

