//! Reading trail tiles from static files.
//!
//! Replaces Overpass. The data sits next to the `.wasm`, on the same origin,
//! behind a CDN — see `etat-code.md` for the measurements that killed the
//! query-service approach, and `trailfmt` for why the tiles are separate files
//! rather than ranges into one archive.
//!
//! Two layers: the **manifest** naming regions, then one **tile** per request.
//! A region's index says which tiles exist, so empty ground costs nothing.

// ⚠️ Temporary: nothing in the application reaches this module yet. Overpass is
// still what feeds the network. **Delete this attribute in the same change that
// wires `TrailArchive` into `app.rs`**, so the compiler starts guarding this
// module against real dead code again.
#![allow(dead_code)]

use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex};

use trailfmt::{Index, TileId};

use crate::geo::LatLon;

/// Where the data lives, relative to the page under WASM.
///
/// Natively there is no page to be relative to, so development expects
/// `trunk serve` on its default port. The tiles are never committed: CI
/// generates them.
#[cfg(target_arch = "wasm32")]
pub const DATA_BASE: &str = "trails/";
#[cfg(not(target_arch = "wasm32"))]
pub const DATA_BASE: &str = "http://localhost:8080/trails/";

pub const MANIFEST_FILE: &str = "regions.json";

/// One region of the world, with its own directory of tiles.
#[derive(Clone, Debug, PartialEq)]
pub struct Region {
    pub name: String,
    pub dir: String,
    pub south: f64,
    pub west: f64,
    pub north: f64,
    pub east: f64,
}

impl Region {
    pub fn contains(&self, ll: LatLon) -> bool {
        ll.lat >= self.south && ll.lat <= self.north && ll.lon >= self.west && ll.lon <= self.east
    }
}

/// The regions this deployment carries.
///
/// ⚠️ Deliberately a list from day one, not a single hard-coded path. Adding a
/// massif then costs one `trailprep` run and one line here.
#[derive(Clone, Debug, Default)]
pub struct Manifest {
    pub regions: Vec<Region>,
}

impl Manifest {
    pub fn region_for(&self, ll: LatLon) -> Option<&Region> {
        self.regions.iter().find(|r| r.contains(ll))
    }
}

pub fn parse_manifest(body: &str) -> Result<Manifest, String> {
    let root: serde_json::Value =
        serde_json::from_str(body).map_err(|e| format!("manifest JSON: {e}"))?;
    let list = root
        .get("regions")
        .and_then(|r| r.as_array())
        .ok_or_else(|| "manifest has no `regions` array".to_owned())?;

    let mut regions = Vec::new();
    for r in list {
        let number = |k: &str| r.get(k).and_then(|v| v.as_f64());
        let text = |k: &str| r.get(k).and_then(|v| v.as_str()).map(|s| s.to_owned());
        let (Some(name), Some(dir)) = (text("name"), text("dir")) else {
            return Err("a region is missing `name` or `dir`".to_owned());
        };
        let (Some(south), Some(west), Some(north), Some(east)) = (
            number("south"),
            number("west"),
            number("north"),
            number("east"),
        ) else {
            return Err(format!("region `{name}` is missing part of its bbox"));
        };
        if south >= north || west >= east {
            return Err(format!("region `{name}` has an inside-out bbox"));
        }
        regions.push(Region {
            name,
            dir,
            south,
            west,
            north,
            east,
        });
    }
    if regions.is_empty() {
        return Err("manifest lists no region".to_owned());
    }
    Ok(Manifest { regions })
}

/// Tiles covering a disc.
pub fn tiles_covering(center: LatLon, radius_m: f64, zoom: u8) -> Vec<TileId> {
    let dlat = radius_m / 111_132.0;
    let dlon = radius_m / (111_320.0 * center.lat.to_radians().cos()).max(1.0);
    // Tile rows count southward, so the north edge gives the smaller y.
    let nw = trailfmt::tile_of(
        (center.lat + dlat).min(85.0),
        (center.lon - dlon).max(-180.0),
        zoom,
    );
    let se = trailfmt::tile_of(
        (center.lat - dlat).max(-85.0),
        (center.lon + dlon).min(180.0),
        zoom,
    );

    let mut out = Vec::new();
    for x in nw.x..=se.x.max(nw.x) {
        for y in nw.y..=se.y.max(nw.y) {
            out.push(TileId { x, y });
        }
    }
    out
}

// ---------------------------------------------------------------------------
// Fetching
// ---------------------------------------------------------------------------

enum Message {
    Manifest(Result<String, String>),
    Index(String, Result<Vec<u8>, String>),
    Tile(String, TileId, Result<Vec<u8>, String>),
}

#[derive(Clone, Copy, PartialEq)]
enum Fetch {
    Idle,
    InFlight,
    Done,
    Failed,
}

/// Trail data read from static tiles.
///
/// Replaces `TrailStore`'s Overpass machinery: no queue, no rate limit, no
/// endpoint failover, no timeouts, and no request pacing. These are static files
/// behind a CDN; holding them back would only make the map slower.
pub struct TrailArchive {
    base: String,
    manifest: Manifest,
    manifest_state: Fetch,
    indexes: HashMap<String, Index>,
    index_state: HashMap<String, Fetch>,
    /// Tiles already resolved — decoded, empty, or failed for good.
    settled: HashSet<(String, TileId)>,
    in_flight: HashSet<(String, TileId)>,
    inbox: Arc<Mutex<Vec<Message>>>,
    /// Identities for points not shared between ways — see `Way::from_archive`.
    synthetic: i64,
    pub last_error: Option<String>,
}

impl Default for TrailArchive {
    fn default() -> Self {
        Self::new(DATA_BASE)
    }
}

impl TrailArchive {
    pub fn new(base: &str) -> Self {
        Self {
            base: base.to_owned(),
            manifest: Manifest::default(),
            manifest_state: Fetch::Idle,
            indexes: HashMap::new(),
            index_state: HashMap::new(),
            settled: HashSet::new(),
            in_flight: HashSet::new(),
            inbox: Arc::new(Mutex::new(Vec::new())),
            synthetic: 0,
            last_error: None,
        }
    }

    /// (tiles settled, tiles in flight, regions known)
    pub fn stats(&self) -> (usize, usize, usize) {
        (
            self.settled.len(),
            self.in_flight.len(),
            self.manifest.regions.len(),
        )
    }

    pub fn manifest(&self) -> &Manifest {
        &self.manifest
    }

    pub fn ready(&self) -> bool {
        self.manifest_state == Fetch::Done
    }

    /// True when a region covers this point — the manifest has to be here first.
    pub fn covers(&self, ll: LatLon) -> bool {
        self.manifest.region_for(ll).is_some()
    }

    /// True when everything within `radius_m` of `center` has been resolved.
    /// Lets a caller wait for a click's ground before snapping it.
    pub fn area_ready(&self, center: LatLon, radius_m: f64) -> bool {
        let Some(region) = self.manifest.region_for(center) else {
            return false;
        };
        if self.index_state.get(&region.dir) != Some(&Fetch::Done) {
            return false;
        }
        tiles_covering(center, radius_m, trailfmt::TILE_ZOOM)
            .into_iter()
            .all(|t| self.settled.contains(&(region.dir.clone(), t)))
    }

    /// Makes sure the trails within `radius_m` of `center` are on their way.
    ///
    /// Idempotent and cheap to call every frame: anything settled or in flight is
    /// skipped.
    pub fn ensure_area(&mut self, center: LatLon, radius_m: f64, ctx: &egui::Context) {
        if self.manifest_state == Fetch::Idle {
            self.fetch_manifest(ctx);
        }
        let Some(region) = self.manifest.region_for(center) else {
            return;
        };
        let dir = region.dir.clone();

        match self.index_state.get(&dir).copied() {
            None | Some(Fetch::Idle) => {
                self.fetch_index(&dir, ctx);
                return;
            }
            Some(Fetch::InFlight) | Some(Fetch::Failed) => return,
            Some(Fetch::Done) => {}
        }

        let Some(index) = self.indexes.get(&dir) else {
            return;
        };
        let wanted: Vec<TileId> = tiles_covering(center, radius_m, trailfmt::TILE_ZOOM)
            .into_iter()
            .filter(|t| {
                let key = (dir.clone(), *t);
                !self.settled.contains(&key) && !self.in_flight.contains(&key)
            })
            .collect();

        let mut to_fetch = Vec::new();
        for tile in wanted {
            if index.has(tile) {
                to_fetch.push(tile);
            } else {
                // Empty ground: the archive holds no tile there. Settling it
                // stops us asking again, and costs no request at all.
                self.settled.insert((dir.clone(), tile));
            }
        }
        for tile in to_fetch {
            self.in_flight.insert((dir.clone(), tile));
            self.fetch_tile(&dir, tile, ctx);
        }
    }

    /// Consumes what arrived and feeds it into the network.
    /// Returns true when the network gained ways.
    pub fn pump(&mut self, net: &mut crate::trails::TrailNetwork) -> bool {
        let arrived = {
            let mut inbox = self.inbox.lock().unwrap();
            std::mem::take(&mut *inbox)
        };
        let mut changed = false;
        for message in arrived {
            match message {
                Message::Manifest(Ok(body)) => match parse_manifest(&body) {
                    Ok(m) => {
                        self.manifest = m;
                        self.manifest_state = Fetch::Done;
                    }
                    Err(e) => {
                        self.manifest_state = Fetch::Failed;
                        self.last_error = Some(e);
                    }
                },
                Message::Manifest(Err(e)) => {
                    self.manifest_state = Fetch::Failed;
                    self.last_error = Some(format!("manifest: {e}"));
                }
                Message::Index(dir, Ok(bytes)) => match trailfmt::read_index(&bytes) {
                    Ok(index) => {
                        self.index_state.insert(dir.clone(), Fetch::Done);
                        self.indexes.insert(dir, index);
                    }
                    Err(e) => {
                        self.index_state.insert(dir, Fetch::Failed);
                        self.last_error = Some(format!("index: {e}"));
                    }
                },
                Message::Index(dir, Err(e)) => {
                    self.index_state.insert(dir, Fetch::Failed);
                    self.last_error = Some(format!("index: {e}"));
                }
                Message::Tile(dir, tile, result) => {
                    self.in_flight.remove(&(dir.clone(), tile));
                    match result {
                        Ok(bytes) => match trailfmt::decode_tile(&bytes) {
                            Ok(ways) => {
                                for aw in ways {
                                    if let Some(way) =
                                        crate::trails::Way::from_archive(aw, &mut self.synthetic)
                                    {
                                        net.insert(way);
                                        changed = true;
                                    }
                                }
                                self.settled.insert((dir, tile));
                            }
                            Err(e) => {
                                // Settled anyway: a tile that does not decode
                                // will not decode on a retry either, and asking
                                // forever would hammer the CDN.
                                self.settled.insert((dir, tile));
                                self.last_error = Some(format!("tile: {e}"));
                            }
                        },
                        Err(e) => {
                            // Left unsettled on purpose: a transport failure is
                            // worth retrying, unlike a decode failure.
                            self.last_error = Some(format!("tile: {e}"));
                        }
                    }
                }
            }
        }
        changed
    }

    fn fetch_manifest(&mut self, ctx: &egui::Context) {
        self.manifest_state = Fetch::InFlight;
        let url = format!("{}{}", self.base, MANIFEST_FILE);
        let inbox = Arc::clone(&self.inbox);
        let ctx = ctx.clone();
        ehttp::fetch(ehttp::Request::get(url), move |result| {
            let body = match result {
                Err(e) => Err(e),
                Ok(r) if !r.ok => Err(format!("HTTP {}", r.status)),
                Ok(r) => r
                    .text()
                    .map(|t| t.to_owned())
                    .ok_or_else(|| "non-textual manifest".to_owned()),
            };
            inbox.lock().unwrap().push(Message::Manifest(body));
            ctx.request_repaint();
        });
    }

    fn fetch_index(&mut self, dir: &str, ctx: &egui::Context) {
        self.index_state.insert(dir.to_owned(), Fetch::InFlight);
        let url = format!("{}{}/{}", self.base, dir, trailfmt::INDEX_FILE);
        let inbox = Arc::clone(&self.inbox);
        let ctx = ctx.clone();
        let dir = dir.to_owned();
        ehttp::fetch(ehttp::Request::get(url), move |result| {
            let bytes = match result {
                Err(e) => Err(e),
                Ok(r) if !r.ok => Err(format!("HTTP {}", r.status)),
                Ok(r) => Ok(r.bytes),
            };
            inbox.lock().unwrap().push(Message::Index(dir, bytes));
            ctx.request_repaint();
        });
    }

    fn fetch_tile(&mut self, dir: &str, tile: TileId, ctx: &egui::Context) {
        let url = format!(
            "{}{}/{}",
            self.base,
            dir,
            trailfmt::tile_path(trailfmt::TILE_ZOOM, tile)
        );
        let inbox = Arc::clone(&self.inbox);
        let ctx = ctx.clone();
        let dir = dir.to_owned();
        ehttp::fetch(ehttp::Request::get(url), move |result| {
            let bytes = match result {
                Err(e) => Err(e),
                Ok(r) if !r.ok => Err(format!("HTTP {}", r.status)),
                Ok(r) => Ok(r.bytes),
            };
            inbox.lock().unwrap().push(Message::Tile(dir, tile, bytes));
            ctx.request_repaint();
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CHAMONIX: LatLon = LatLon::new(45.92, 6.87);

    const MANIFEST: &str = r#"{"regions":[
        {"name":"Alpes du Nord","dir":"alps",
         "south":43.95,"west":5.35,"north":46.45,"east":7.75}
    ]}"#;

    #[test]
    fn the_manifest_parses_and_locates() {
        let m = parse_manifest(MANIFEST).unwrap();
        assert_eq!(m.regions.len(), 1);
        assert_eq!(m.region_for(CHAMONIX).unwrap().dir, "alps");
        // Outside every region: the deployment simply does not cover it.
        assert!(m.region_for(LatLon::new(48.85, 2.35)).is_none());
    }

    /// The exact shape CI produces, newline and all — the workflow joins
    /// per-region fragments with `paste`, and this locks that output down.
    ///
    /// ⚠️ Two regions on purpose: with a single one the first, broken version of
    /// that shell pipeline still emitted valid JSON. The bug would only have
    /// surfaced the day a second massif was added.
    #[test]
    fn the_manifest_ci_produces_parses() {
        let assembled = concat!(
            r#"{"regions":["#,
            r#"{"name":"Alpes françaises","dir":"alps","south":43.95,"west":5.35,"north":46.45,"east":7.75},"#,
            r#"{"name":"Pyrénées","dir":"pyrenees","south":42.4,"west":-1.8,"north":43.3,"east":3.2}"#,
            "\n]}"
        );
        let m = parse_manifest(assembled).unwrap();
        assert_eq!(m.regions.len(), 2);
        assert_eq!(m.region_for(CHAMONIX).unwrap().dir, "alps");
        assert_eq!(
            m.region_for(LatLon::new(42.8, 0.15)).unwrap().dir,
            "pyrenees"
        );
    }

    /// A malformed manifest must fail loudly. It is generated by CI and never
    /// read by a human; a silently empty region list would look exactly like a
    /// country with no trails.
    #[test]
    fn a_broken_manifest_is_refused() {
        assert!(parse_manifest("not json").is_err());
        assert!(parse_manifest(r#"{"regions":[]}"#).is_err());
        assert!(parse_manifest(r#"{"nope":1}"#).is_err());
        assert!(parse_manifest(r#"{"regions":[{"name":"x"}]}"#).is_err());
        let inside_out = r#"{"regions":[{"name":"x","dir":"d","south":46.0,
                             "west":5.0,"north":44.0,"east":7.0}]}"#;
        assert!(
            parse_manifest(inside_out).is_err(),
            "an inside-out bbox would match nothing and explain nothing"
        );
    }

    #[test]
    fn a_disc_is_covered_by_its_tiles() {
        let tiles = tiles_covering(CHAMONIX, 30_000.0, trailfmt::TILE_ZOOM);
        assert!(
            (16..=49).contains(&tiles.len()),
            "{} tiles for a 30 km radius",
            tiles.len()
        );
        for (dlat, dlon) in [(0.0, 0.0), (0.26, 0.0), (-0.26, 0.0), (0.0, 0.38), (0.0, -0.38)] {
            let p = LatLon::new(CHAMONIX.lat + dlat, CHAMONIX.lon + dlon);
            let t = trailfmt::tile_of(p.lat, p.lon, trailfmt::TILE_ZOOM);
            assert!(tiles.contains(&t), "{p:?} → {t:?} missing");
        }
    }

    #[test]
    fn a_smaller_radius_asks_for_fewer_tiles() {
        let z = trailfmt::TILE_ZOOM;
        let near = tiles_covering(CHAMONIX, 5_000.0, z).len();
        let far = tiles_covering(CHAMONIX, 30_000.0, z).len();
        assert!(near < far, "{near} vs {far}");
        assert!(near >= 1);
    }

    /// Two ways that share one OSM node, in one tile.
    fn tile_with_a_junction() -> (TileId, Vec<u8>) {
        let shared_node = 1_842_665_301i64;
        let west = trailfmt::ArchiveWay {
            id: 10,
            kind: trailfmt::kind::PATH,
            sac: 2,
            name: Some("Sentier ouest".to_owned()),
            points: vec![(45.900, 6.860), (45.900, 6.870)],
            shared: vec![None, Some(shared_node)],
        };
        let north = trailfmt::ArchiveWay {
            id: 20,
            kind: trailfmt::kind::TRACK,
            sac: 0,
            name: None,
            points: vec![(45.900, 6.870), (45.910, 6.870)],
            shared: vec![Some(shared_node), None],
        };
        let tile = trailfmt::tile_of(45.90, 6.86, trailfmt::TILE_ZOOM);
        (tile, trailfmt::encode_tile(&[west, north]))
    }

    /// End to end, offline: tile bytes → network → graph. The junction must land
    /// exactly where the two ways share a node, and nowhere else.
    ///
    /// ⚠️ This is the test that guards the phantom junction. Points that are not
    /// shared get synthetic identities, and if those ever collided the graph
    /// would weld unrelated trails together with no error at all.
    #[test]
    fn a_tile_becomes_a_network_with_exactly_one_junction() {
        let (tile, blob) = tile_with_a_junction();
        let mut store = TrailArchive::new("unused/");
        store
            .inbox
            .lock()
            .unwrap()
            .push(Message::Tile("alps".to_owned(), tile, Ok(blob)));

        let mut net = crate::trails::TrailNetwork::default();
        assert!(store.pump(&mut net), "the network must gain ways");
        assert_eq!(net.len(), 2);
        assert!(store.settled.contains(&("alps".to_owned(), tile)));

        let ids: Vec<i64> = net.ways().iter().flat_map(|w| w.nodes.clone()).collect();
        let distinct: HashSet<i64> = ids.iter().copied().collect();
        assert_eq!(ids.len(), 4);
        assert_eq!(distinct.len(), 3, "exactly one identity is shared: {ids:?}");
        assert_eq!(
            ids.iter().filter(|i| **i > 0).count(),
            2,
            "only the shared node keeps a real OSM id: {ids:?}"
        );

        let graph = crate::graph::Graph::build(&net);
        assert_eq!(graph.node_count(), 3);
        assert_eq!(graph.edges.len(), 2);

        let west = net.way_by_id(10).unwrap();
        assert_eq!(west.kind, crate::trails::WayKind::Path);
        assert_eq!(west.name.as_deref(), Some("Sentier ouest"));
        assert_eq!(west.sac_scale.as_deref(), Some("mountain_hiking"));
        assert!((west.points[0].lat - 45.900).abs() < 1e-5);
    }

    /// A transport failure is worth retrying; a decode failure is not. Confusing
    /// the two either leaves a permanent hole in the network or hammers the CDN
    /// forever.
    #[test]
    fn only_transport_failures_are_retried() {
        let (tile, _) = tile_with_a_junction();
        let mut net = crate::trails::TrailNetwork::default();

        let mut store = TrailArchive::new("unused/");
        store.in_flight.insert(("alps".to_owned(), tile));
        store.inbox.lock().unwrap().push(Message::Tile(
            "alps".to_owned(),
            tile,
            Err("network down".to_owned()),
        ));
        assert!(!store.pump(&mut net));
        assert!(!store.settled.contains(&("alps".to_owned(), tile)));
        assert!(!store.in_flight.contains(&("alps".to_owned(), tile)));

        let mut store = TrailArchive::new("unused/");
        store.inbox.lock().unwrap().push(Message::Tile(
            "alps".to_owned(),
            tile,
            Ok(b"<!DOCTYPE html>".to_vec()),
        ));
        assert!(!store.pump(&mut net));
        assert!(
            store.settled.contains(&("alps".to_owned(), tile)),
            "a tile that cannot decode will not decode on a retry either"
        );
        assert!(store.last_error.is_some());
    }

    /// Ground the archive does not cover costs no request at all — the Alps are
    /// largely rock and ice, and the index is what makes that free.
    #[test]
    fn empty_ground_is_settled_without_a_request() {
        let ctx = egui::Context::default();
        let mut store = TrailArchive::new("unused/");
        store.manifest = parse_manifest(MANIFEST).unwrap();
        store.manifest_state = Fetch::Done;
        store.index_state.insert("alps".to_owned(), Fetch::Done);
        // An index that holds nothing at all.
        store.indexes.insert(
            "alps".to_owned(),
            Index {
                zoom: trailfmt::TILE_ZOOM,
                tiles: Vec::new(),
            },
        );

        store.ensure_area(CHAMONIX, 30_000.0, &ctx);
        let (settled, in_flight, _) = store.stats();
        assert!(settled > 0, "the empty tiles must be settled");
        assert_eq!(in_flight, 0, "and none of them requested");

        // Asking again changes nothing.
        store.ensure_area(CHAMONIX, 30_000.0, &ctx);
        assert_eq!(store.stats(), (settled, 0, 1));
        assert!(store.area_ready(CHAMONIX, 30_000.0));
    }

    /// The whole chain against the **deployed** tiles: manifest → index → tile →
    /// decode → network → snap → graph.
    ///
    /// `cargo test --release -- --ignored deployed --nocapture`
    ///
    /// ⚠️ This is the only test that exercises the real CDN, the data CI
    /// produced and the reader together. It caught the range-versus-gzip
    /// incompatibility that curl and Python had both missed.
    #[test]
    #[ignore = "network"]
    #[cfg(not(target_arch = "wasm32"))]
    fn the_deployed_tiles_are_readable() {
        const BASE: &str = "https://lemuffinman.github.io/TrackFinder/trails/";

        fn get(url: &str) -> Vec<u8> {
            let r = ehttp::fetch_blocking(&ehttp::Request::get(url))
                .unwrap_or_else(|e| panic!("{url}: {e}"));
            assert!(r.ok, "{url}: HTTP {}", r.status);
            r.bytes
        }

        let manifest = parse_manifest(
            std::str::from_utf8(&get(&format!("{BASE}{MANIFEST_FILE}"))).expect("utf-8"),
        )
        .expect("manifest");
        let region = manifest
            .region_for(CHAMONIX)
            .expect("the Alps must be published");

        let index = trailfmt::read_index(&get(&format!(
            "{BASE}{}/{}",
            region.dir,
            trailfmt::INDEX_FILE
        )))
        .expect("index");
        assert_eq!(index.zoom, trailfmt::TILE_ZOOM);
        assert!(index.tiles.len() > 100, "{} tiles", index.tiles.len());

        let tile = trailfmt::tile_of(CHAMONIX.lat, CHAMONIX.lon, trailfmt::TILE_ZOOM);
        assert!(index.has(tile), "Chamonix must be in the index");
        let blob = get(&format!(
            "{BASE}{}/{}",
            region.dir,
            trailfmt::tile_path(trailfmt::TILE_ZOOM, tile)
        ));

        let mut store = TrailArchive::new(BASE);
        store
            .inbox
            .lock()
            .unwrap()
            .push(Message::Tile(region.dir.clone(), tile, Ok(blob.clone())));
        let mut net = crate::trails::TrailNetwork::default();
        assert!(store.pump(&mut net));
        assert!(net.len() > 200, "{} ways in the Chamonix tile", net.len());
        assert!(store.last_error.is_none(), "{:?}", store.last_error);

        let snap = net
            .snap(LatLon::new(45.9237, 6.8703), crate::trails::SNAP_RADIUS_M)
            .expect("no trail near the centre of Chamonix");
        assert!(snap.dist_m <= crate::trails::SNAP_RADIUS_M);

        let graph = crate::graph::Graph::build(&net);
        assert!(!graph.is_empty());
        eprintln!(
            "Chamonix tile: {} KB, {} ways, {} graph nodes, {} edges",
            blob.len() / 1024,
            net.len(),
            graph.node_count(),
            graph.edges.len()
        );
    }
}
