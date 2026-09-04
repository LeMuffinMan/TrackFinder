//! Reading trail archives over HTTP range requests.
//!
//! Replaces Overpass. The data is a static file next to the `.wasm`, on the same
//! origin, behind a CDN — see `etat-code.md` for the measurements that killed the
//! query-service approach.
//!
//! Three layers, each fetched once:
//!
//! 1. the **manifest**, listing regions and their bounding boxes;
//! 2. a region's **index**, a couple of kilobytes naming every tile's byte range;
//! 3. the **tiles** themselves, as coalesced range requests.

// ⚠️ Temporary: nothing in the application reaches this module yet. Overpass is
// still what feeds the network, and it stays until CI publishes a real archive —
// removing it first would deploy a map with no trails at all. **Delete this
// attribute in the same change that wires `TrailArchive` into `app.rs`**, so the
// compiler starts guarding this module against real dead code again.
#![allow(dead_code)]

use std::collections::HashMap;

use trailfmt::{Index, TileId, TileSpan};

use crate::geo::LatLon;

/// Where the data lives, relative to the page under WASM.
///
/// Natively there is no page to be relative to, so development expects
/// `trunk serve` on its default port. The archives are never committed: CI
/// generates them, and a local run needs `trailprep` to have written them into
/// `dist/trails/` first.
#[cfg(target_arch = "wasm32")]
pub const DATA_BASE: &str = "trails/";
#[cfg(not(target_arch = "wasm32"))]
pub const DATA_BASE: &str = "http://localhost:8080/trails/";

pub const MANIFEST_FILE: &str = "regions.json";

/// Byte gap small enough that swallowing it beats a second round trip.
///
/// A request costs a round trip — tens of milliseconds — while 64 KB off a CDN
/// costs a handful. Merging across small holes turns a 5 × 5 block of tiles into
/// a handful of requests instead of twenty-five.
pub const MAX_COALESCE_GAP: u64 = 64 * 1024;

/// One region of the world, with its own archive file.
#[derive(Clone, Debug, PartialEq)]
pub struct Region {
    pub name: String,
    pub file: String,
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

/// The list of regions the deployment carries.
///
/// ⚠️ Deliberately a list from day one, not a single hard-coded archive URL.
/// Adding a massif then costs one `trailprep` run and one line here; hard-coding
/// one file would mean reworking this module instead.
#[derive(Clone, Debug, Default)]
pub struct Manifest {
    pub regions: Vec<Region>,
}

impl Manifest {
    /// The region covering this point, if the deployment carries one.
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
        let field = |k: &str| r.get(k).and_then(|v| v.as_f64());
        let text = |k: &str| r.get(k).and_then(|v| v.as_str()).map(|s| s.to_owned());
        let (Some(name), Some(file)) = (text("name"), text("file")) else {
            return Err("a region is missing `name` or `file`".to_owned());
        };
        let (Some(south), Some(west), Some(north), Some(east)) = (
            field("south"),
            field("west"),
            field("north"),
            field("east"),
        ) else {
            return Err(format!("region `{name}` is missing part of its bbox"));
        };
        if south >= north || west >= east {
            return Err(format!("region `{name}` has an inside-out bbox"));
        }
        regions.push(Region {
            name,
            file,
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

/// Tiles covering a disc, in file order.
///
/// Sorted by `(x, y)` — the same order the archive index uses — so that
/// neighbouring tiles come out adjacent and [`coalesce`] can merge them.
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

/// A contiguous stretch of the archive, covering one or more tiles.
#[derive(Clone, Debug, PartialEq)]
pub struct Run {
    pub start: u64,
    pub len: u64,
    pub tiles: Vec<TileSpan>,
}

impl Run {
    /// The `Range` header value for this stretch.
    pub fn range_header(&self) -> String {
        format!("bytes={}-{}", self.start, self.start + self.len - 1)
    }

    /// Slices one tile's bytes out of the fetched stretch.
    pub fn slice<'a>(&self, body: &'a [u8], span: &TileSpan) -> Option<&'a [u8]> {
        let from = span.offset.checked_sub(self.start)? as usize;
        body.get(from..from + span.len as usize)
    }
}

/// Merges spans into as few range requests as possible.
///
/// The archive index is sorted by `(x, y)`, so a block of neighbouring tiles is
/// already mostly contiguous on disk; the gaps are the tiles of that block we do
/// not want. Swallowing a small gap costs a few kilobytes, refusing to costs a
/// whole round trip.
pub fn coalesce(spans: &[TileSpan], max_gap: u64) -> Vec<Run> {
    let mut sorted: Vec<TileSpan> = spans.to_vec();
    sorted.sort_by_key(|s| s.offset);

    let mut runs: Vec<Run> = Vec::new();
    for span in sorted {
        let end = span.offset + span.len as u64;
        match runs.last_mut() {
            Some(run) if span.offset <= run.start + run.len + max_gap => {
                run.len = end.saturating_sub(run.start).max(run.len);
                run.tiles.push(span);
            }
            _ => runs.push(Run {
                start: span.offset,
                len: span.len as u64,
                tiles: vec![span],
            }),
        }
    }
    runs
}

/// Per-region index, once fetched.
#[derive(Default)]
pub struct Indexes {
    by_file: HashMap<String, Index>,
}

impl Indexes {
    pub fn get(&self, file: &str) -> Option<&Index> {
        self.by_file.get(file)
    }

    pub fn insert(&mut self, file: String, index: Index) {
        self.by_file.insert(file, index);
    }

    /// Spans for the tiles of `tiles` this index actually holds.
    ///
    /// Missing tiles are simply absent: the archive does not store empty tiles,
    /// and rock and ice make up a lot of the Alps.
    pub fn spans_for(&self, file: &str, tiles: &[TileId]) -> Vec<TileSpan> {
        let Some(index) = self.by_file.get(file) else {
            return Vec::new();
        };
        tiles.iter().filter_map(|t| index.span(*t)).collect()
    }
}

// ---------------------------------------------------------------------------
// Fetching
// ---------------------------------------------------------------------------

/// How much of the archive to grab when reading an index blind.
///
/// The index length depends on the tile count, which is *in* the header — so a
/// strictly correct reader needs two round trips. One generous first request
/// avoids that: 64 KB holds 3 276 tiles, where the whole northern Alps needs
/// 147. The second request stays implemented for the day a region outgrows it.
const INDEX_PROBE_BYTES: u64 = 64 * 1024;

enum Message {
    Manifest(Result<String, String>),
    Index(String, Result<Vec<u8>, String>),
    Run(String, Run, Result<(u16, Vec<u8>), String>),
}

#[derive(PartialEq)]
enum Fetch {
    Idle,
    InFlight,
    Done,
    Failed,
}

/// Trail data read from static archives.
///
/// Replaces `TrailStore`'s Overpass machinery: no queue, no rate limit, no
/// endpoint failover, no timeouts. A static file either answers or it does not.
pub struct TrailArchive {
    base: String,
    manifest: Manifest,
    manifest_state: Fetch,
    indexes: Indexes,
    index_state: HashMap<String, Fetch>,
    /// Tiles already inserted into the network.
    loaded: std::collections::HashSet<(String, TileId)>,
    in_flight: std::collections::HashSet<(String, TileId)>,
    inbox: std::sync::Arc<std::sync::Mutex<Vec<Message>>>,
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
            indexes: Indexes::default(),
            index_state: HashMap::new(),
            loaded: std::collections::HashSet::new(),
            in_flight: std::collections::HashSet::new(),
            inbox: std::sync::Arc::new(std::sync::Mutex::new(Vec::new())),
            synthetic: 0,
            last_error: None,
        }
    }

    /// (loaded tiles, tiles in flight, regions known)
    pub fn stats(&self) -> (usize, usize, usize) {
        (
            self.loaded.len(),
            self.in_flight.len(),
            self.manifest.regions.len(),
        )
    }

    pub fn manifest(&self) -> &Manifest {
        &self.manifest
    }

    /// True once the manifest is here and a region covers this point.
    pub fn covers(&self, ll: LatLon) -> bool {
        self.manifest.region_for(ll).is_some()
    }

    pub fn ready(&self) -> bool {
        self.manifest_state == Fetch::Done
    }

    /// Makes sure the trails within `radius_m` of `center` are on their way.
    ///
    /// Idempotent and cheap to call every frame: everything already loaded or in
    /// flight is skipped. Unlike the Overpass version there is no queue and no
    /// pacing — these are static files behind a CDN, and holding them back would
    /// only make the map slower.
    pub fn ensure_area(&mut self, center: LatLon, radius_m: f64, ctx: &egui::Context) {
        if self.manifest_state == Fetch::Idle {
            self.fetch_manifest(ctx);
        }
        let Some(region) = self.manifest.region_for(center) else {
            return;
        };
        let file = region.file.clone();

        match self.index_state.get(&file) {
            None | Some(Fetch::Idle) => {
                self.fetch_index(&file, 0, INDEX_PROBE_BYTES, ctx);
                return;
            }
            Some(Fetch::InFlight) | Some(Fetch::Failed) => return,
            Some(Fetch::Done) => {}
        }

        let tiles: Vec<TileId> = tiles_covering(center, radius_m, trailfmt::TILE_ZOOM)
            .into_iter()
            .filter(|t| {
                let key = (file.clone(), *t);
                !self.loaded.contains(&key) && !self.in_flight.contains(&key)
            })
            .collect();
        if tiles.is_empty() {
            return;
        }
        let spans = self.indexes.spans_for(&file, &tiles);
        // Tiles the archive does not hold are empty ground: mark them done so we
        // stop asking. Rock and ice make up a lot of the Alps.
        for t in &tiles {
            self.loaded.insert((file.clone(), *t));
        }
        for run in coalesce(&spans, MAX_COALESCE_GAP) {
            for span in &run.tiles {
                self.in_flight.insert((file.clone(), span.tile));
            }
            self.fetch_run(&file, run, ctx);
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
                Message::Index(file, Ok(bytes)) => match trailfmt::read_index(&bytes) {
                    Ok(index) => {
                        self.index_state.insert(file.clone(), Fetch::Done);
                        self.indexes.insert(file, index);
                    }
                    Err(e) => {
                        self.index_state.insert(file, Fetch::Failed);
                        self.last_error = Some(format!("index: {e}"));
                    }
                },
                Message::Index(file, Err(e)) => {
                    self.index_state.insert(file, Fetch::Failed);
                    self.last_error = Some(format!("index: {e}"));
                }
                Message::Run(file, run, result) => {
                    for span in &run.tiles {
                        self.in_flight.remove(&(file.clone(), span.tile));
                    }
                    match result {
                        Ok((status, body)) => {
                            // A server that ignores `Range` answers 200 with the
                            // whole file. Slicing that with run-relative offsets
                            // would read the wrong bytes and decode plausible
                            // nonsense, so the base depends on the status.
                            let base = if status == 206 { run.start } else { 0 };
                            for span in &run.tiles {
                                let from = match span.offset.checked_sub(base) {
                                    Some(v) => v as usize,
                                    None => continue,
                                };
                                let Some(blob) = body.get(from..from + span.len as usize) else {
                                    self.last_error =
                                        Some("range response shorter than asked".to_owned());
                                    continue;
                                };
                                match trailfmt::decode_tile(blob) {
                                    Ok(ways) => {
                                        for aw in ways {
                                            if let Some(way) = crate::trails::Way::from_archive(
                                                aw,
                                                &mut self.synthetic,
                                            ) {
                                                net.insert(way);
                                                changed = true;
                                            }
                                        }
                                    }
                                    Err(e) => {
                                        self.last_error = Some(format!("tile: {e}"));
                                    }
                                }
                            }
                        }
                        Err(e) => {
                            // Let it be asked for again rather than silently
                            // leaving a hole in the network.
                            for span in &run.tiles {
                                self.loaded.remove(&(file.clone(), span.tile));
                            }
                            self.last_error = Some(format!("tiles: {e}"));
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
        let inbox = std::sync::Arc::clone(&self.inbox);
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

    fn fetch_index(&mut self, file: &str, from: u64, to: u64, ctx: &egui::Context) {
        self.index_state.insert(file.to_owned(), Fetch::InFlight);
        let url = format!("{}{}", self.base, file);
        let mut request = ehttp::Request::get(url);
        request
            .headers
            .insert("Range", format!("bytes={from}-{}", to - 1));
        let inbox = std::sync::Arc::clone(&self.inbox);
        let ctx = ctx.clone();
        let file = file.to_owned();
        ehttp::fetch(request, move |result| {
            let bytes = match result {
                Err(e) => Err(e),
                Ok(r) if !r.ok => Err(format!("HTTP {}", r.status)),
                Ok(r) => Ok(r.bytes),
            };
            inbox.lock().unwrap().push(Message::Index(file, bytes));
            ctx.request_repaint();
        });
    }

    fn fetch_run(&mut self, file: &str, run: Run, ctx: &egui::Context) {
        let url = format!("{}{}", self.base, file);
        let mut request = ehttp::Request::get(url);
        request.headers.insert("Range", run.range_header());
        let inbox = std::sync::Arc::clone(&self.inbox);
        let ctx = ctx.clone();
        let file = file.to_owned();
        ehttp::fetch(request, move |result| {
            let payload = match result {
                Err(e) => Err(e),
                Ok(r) if !r.ok => Err(format!("HTTP {}", r.status)),
                Ok(r) => Ok((r.status, r.bytes)),
            };
            inbox.lock().unwrap().push(Message::Run(file, run, payload));
            ctx.request_repaint();
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CHAMONIX: LatLon = LatLon::new(45.92, 6.87);

    const MANIFEST: &str = r#"{"regions":[
        {"name":"Alpes du Nord","file":"alps-north.tft",
         "south":43.95,"west":5.35,"north":46.45,"east":7.75}
    ]}"#;

    fn span(x: u32, y: u32, offset: u64, len: u32) -> TileSpan {
        TileSpan {
            tile: TileId { x, y },
            offset,
            len,
        }
    }

    #[test]
    fn the_manifest_parses_and_locates() {
        let m = parse_manifest(MANIFEST).unwrap();
        assert_eq!(m.regions.len(), 1);
        assert_eq!(m.region_for(CHAMONIX).unwrap().file, "alps-north.tft");
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
            r#"{"name":"Alpes françaises","file":"alps.tft","south":43.95,"west":5.35,"north":46.45,"east":7.75},"#,
            r#"{"name":"Pyrénées","file":"pyrenees.tft","south":42.4,"west":-1.8,"north":43.3,"east":3.2}"#,
            "\n]}"
        );
        let m = parse_manifest(assembled).unwrap();
        assert_eq!(m.regions.len(), 2);
        assert_eq!(m.region_for(CHAMONIX).unwrap().file, "alps.tft");
        assert_eq!(
            m.region_for(LatLon::new(42.8, 0.15)).unwrap().file,
            "pyrenees.tft"
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
        let inside_out = r#"{"regions":[{"name":"x","file":"f","south":46.0,
                             "west":5.0,"north":44.0,"east":7.0}]}"#;
        assert!(
            parse_manifest(inside_out).is_err(),
            "an inside-out bbox would match nothing and explain nothing"
        );
    }

    /// The disc must be covered, corners included, and the count must stay in the
    /// handful-of-tiles range the design assumes.
    #[test]
    fn a_disc_is_covered_by_its_tiles() {
        let tiles = tiles_covering(CHAMONIX, 30_000.0, trailfmt::TILE_ZOOM);
        assert!(
            (16..=49).contains(&tiles.len()),
            "{} tiles for a 30 km radius",
            tiles.len()
        );
        // The centre and the four cardinal edges of the disc are all inside.
        for (dlat, dlon) in [(0.0, 0.0), (0.26, 0.0), (-0.26, 0.0), (0.0, 0.38), (0.0, -0.38)] {
            let p = LatLon::new(CHAMONIX.lat + dlat, CHAMONIX.lon + dlon);
            let t = trailfmt::tile_of(p.lat, p.lon, trailfmt::TILE_ZOOM);
            assert!(tiles.contains(&t), "{p:?} → {t:?} missing");
        }
        // Sorted by (x, y): that is what makes coalescing work.
        let mut sorted = tiles.clone();
        sorted.sort_by_key(|t| (t.x, t.y));
        assert_eq!(tiles, sorted);
    }

    #[test]
    fn a_smaller_radius_asks_for_fewer_tiles() {
        let z = trailfmt::TILE_ZOOM;
        let near = tiles_covering(CHAMONIX, 5_000.0, z).len();
        let far = tiles_covering(CHAMONIX, 30_000.0, z).len();
        assert!(near < far, "{near} vs {far}");
        assert!(near >= 1);
    }

    /// Adjacent tiles become one request; a real hole stays a separate one.
    #[test]
    fn coalescing_turns_a_block_into_a_few_requests() {
        let spans = vec![
            span(1, 1, 1000, 500),
            span(1, 2, 1500, 500), // touching the previous one
            span(1, 3, 2100, 400), // 100-byte gap: worth swallowing
            span(9, 9, 5_000_000, 200), // far away: its own request
        ];
        let runs = coalesce(&spans, MAX_COALESCE_GAP);
        assert_eq!(runs.len(), 2, "{runs:#?}");
        assert_eq!(runs[0].start, 1000);
        assert_eq!(runs[0].len, 1500, "covers 1000..2500");
        assert_eq!(runs[0].tiles.len(), 3);
        assert_eq!(runs[1].tiles.len(), 1);
        assert_eq!(runs[0].range_header(), "bytes=1000-2499");
    }

    /// With no merging allowed, every tile is its own request — the behaviour
    /// coalescing exists to avoid.
    #[test]
    fn without_a_gap_budget_every_tile_is_a_request() {
        let spans = vec![span(1, 1, 0, 10), span(1, 2, 100, 10), span(1, 3, 200, 10)];
        assert_eq!(coalesce(&spans, 0).len(), 3);
        assert_eq!(coalesce(&spans, 200).len(), 1);
    }

    /// Out-of-order spans must not produce a run that runs backwards.
    #[test]
    fn coalescing_sorts_before_merging() {
        let spans = vec![span(2, 0, 900, 100), span(1, 0, 0, 100), span(3, 0, 500, 100)];
        let runs = coalesce(&spans, MAX_COALESCE_GAP);
        assert_eq!(runs.len(), 1);
        assert_eq!(runs[0].start, 0);
        assert_eq!(runs[0].len, 1000);
    }

    /// Each tile must be recoverable from the bytes of the run that carried it.
    #[test]
    fn a_tile_is_sliced_back_out_of_its_run() {
        let spans = vec![span(1, 1, 10, 4), span(1, 2, 14, 3)];
        let runs = coalesce(&spans, MAX_COALESCE_GAP);
        assert_eq!(runs.len(), 1);
        let body: Vec<u8> = (10u8..=16).collect(); // bytes 10..17 of the archive
        assert_eq!(runs[0].slice(&body, &spans[0]).unwrap(), &[10, 11, 12, 13]);
        assert_eq!(runs[0].slice(&body, &spans[1]).unwrap(), &[14, 15, 16]);
        // A span outside this run must not silently return the wrong bytes.
        assert_eq!(runs[0].slice(&body, &span(9, 9, 0, 4)), None);
        assert_eq!(runs[0].slice(&body, &span(9, 9, 20, 4)), None);
    }

    /// Two ways that share one OSM node, in one tile.
    fn archive_with_a_junction() -> (Vec<u8>, trailfmt::TileSpan) {
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
        let archive = trailfmt::write_archive(trailfmt::TILE_ZOOM, &[(tile, vec![west, north])]);
        let span = trailfmt::read_index(&archive).unwrap().span(tile).unwrap();
        (archive, span)
    }

    /// End to end, offline: archive bytes → network → graph. The junction must
    /// land exactly where the two ways share a node, and nowhere else.
    ///
    /// ⚠️ This is the test that guards the phantom junction. Points that are not
    /// shared get synthetic identities, and if those ever collided the graph
    /// would weld unrelated trails together with no error at all.
    #[test]
    fn a_run_becomes_a_network_with_exactly_one_junction() {
        let (archive, span) = archive_with_a_junction();
        let run = coalesce(&[span], MAX_COALESCE_GAP).into_iter().next().unwrap();
        let body = archive[run.start as usize..][..run.len as usize].to_vec();

        let mut store = TrailArchive::new("unused/");
        store
            .inbox
            .lock()
            .unwrap()
            .push(Message::Run("f".to_owned(), run, Ok((206, body))));

        let mut net = crate::trails::TrailNetwork::default();
        assert!(store.pump(&mut net), "the network must gain ways");
        assert_eq!(net.len(), 2);

        // Four points, of which two are the same shared node.
        let ids: Vec<i64> = net.ways().iter().flat_map(|w| w.nodes.clone()).collect();
        let distinct: std::collections::HashSet<i64> = ids.iter().copied().collect();
        assert_eq!(ids.len(), 4);
        assert_eq!(distinct.len(), 3, "exactly one identity is shared: {ids:?}");
        assert!(
            ids.iter().filter(|i| **i > 0).count() == 2,
            "only the shared node keeps a real OSM id: {ids:?}"
        );

        // The graph agrees: three nodes, two edges, meeting at one junction.
        let graph = crate::graph::Graph::build(&net);
        assert_eq!(graph.node_count(), 3);
        assert_eq!(graph.edges.len(), 2);

        // Metadata survived the round trip.
        let west = net.way_by_id(10).unwrap();
        assert_eq!(west.kind, crate::trails::WayKind::Path);
        assert_eq!(west.name.as_deref(), Some("Sentier ouest"));
        assert_eq!(west.sac_scale.as_deref(), Some("mountain_hiking"));
        assert!((west.points[0].lat - 45.900).abs() < 1e-5);
    }

    /// A server that ignores `Range` answers 200 with the whole file. Slicing
    /// that with run-relative offsets would decode plausible nonsense, so the
    /// reader must key off the status code.
    #[test]
    fn a_server_ignoring_range_still_works() {
        let (archive, span) = archive_with_a_junction();
        let run = coalesce(&[span], MAX_COALESCE_GAP).into_iter().next().unwrap();
        assert!(run.start > 0, "the tile must not sit at offset zero");

        let mut store = TrailArchive::new("unused/");
        store.inbox.lock().unwrap().push(Message::Run(
            "f".to_owned(),
            run,
            Ok((200, archive.clone())),
        ));
        let mut net = crate::trails::TrailNetwork::default();
        assert!(store.pump(&mut net));
        assert_eq!(net.len(), 2, "the whole-file answer must decode the same");
        assert!(store.last_error.is_none(), "{:?}", store.last_error);
    }

    /// A failed run must be forgotten, not remembered as loaded — otherwise the
    /// network keeps a hole nothing will ever fill.
    #[test]
    fn a_failed_run_can_be_asked_for_again() {
        let (_, span) = archive_with_a_junction();
        let run = coalesce(&[span], MAX_COALESCE_GAP).into_iter().next().unwrap();
        let mut store = TrailArchive::new("unused/");
        let key = ("f".to_owned(), span.tile);
        store.loaded.insert(key.clone());
        store.in_flight.insert(key.clone());
        store.inbox.lock().unwrap().push(Message::Run(
            "f".to_owned(),
            run,
            Err("network down".to_owned()),
        ));

        let mut net = crate::trails::TrailNetwork::default();
        assert!(!store.pump(&mut net));
        assert!(!store.loaded.contains(&key), "a failed tile must be retried");
        assert!(!store.in_flight.contains(&key));
        assert!(store.last_error.is_some());
    }

    #[test]
    fn missing_tiles_are_skipped_not_invented() {
        let mut indexes = Indexes::default();
        indexes.insert(
            "f".to_owned(),
            Index {
                zoom: trailfmt::TILE_ZOOM,
                spans: vec![span(1, 1, 0, 10)],
            },
        );
        let wanted = vec![TileId { x: 1, y: 1 }, TileId { x: 7, y: 7 }];
        let got = indexes.spans_for("f", &wanted);
        assert_eq!(got.len(), 1, "an absent tile is absent, not empty bytes");
        assert!(indexes.spans_for("unknown", &wanted).is_empty());
    }
}
