//! Tile source (abstraction) + asynchronous cache.
//!
//! A source answers "give me the bytes for this z/x/y". HTTP today; local files
//! or an offline cache later, without touching anything else.

use std::collections::{HashMap, HashSet};
use std::hash::Hash;
use std::rc::Rc;
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// Tile description
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum RasterLayer {
    PlanIgn,
    Contours,
    Slopes,
    Ortho,
}

impl RasterLayer {
    pub fn wmts_name(self) -> &'static str {
        match self {
            RasterLayer::PlanIgn => "GEOGRAPHICALGRIDSYSTEMS.PLANIGNV2",
            RasterLayer::Contours => "ELEVATION.CONTOUR.LINE",
            RasterLayer::Slopes => "GEOGRAPHICALGRIDSYSTEMS.SLOPES.MOUNTAIN",
            RasterLayer::Ortho => "ORTHOIMAGERY.ORTHOPHOTOS",
        }
    }

    pub fn format(self) -> &'static str {
        match self {
            RasterLayer::Ortho => "image/jpeg",
            _ => "image/png",
        }
    }

    /// Zoom ceilings read from the GetCapabilities document: past them the layer
    /// no longer exists and we have to fall back to a magnified parent tile.
    pub fn zoom_range(self) -> (u8, u8) {
        match self {
            RasterLayer::PlanIgn => (0, 19),
            RasterLayer::Contours => (6, 18),
            RasterLayer::Slopes => (0, 17),
            RasterLayer::Ortho => (0, 19),
        }
    }

    /// A base map is an opaque ground layer: exactly one of them is drawn, and
    /// the transparent overlays go on top. Two opaque grounds stacked would only
    /// ever show the top one while paying for both.
    pub fn is_base(self) -> bool {
        matches!(self, RasterLayer::PlanIgn | RasterLayer::Ortho)
    }

    pub fn label(self) -> &'static str {
        match self {
            RasterLayer::PlanIgn => "IGN topo map",
            RasterLayer::Contours => "Contour lines",
            RasterLayer::Slopes => "Slope shading",
            RasterLayer::Ortho => "Aerial imagery",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Dataset {
    Raster(RasterLayer),
    /// High-resolution DEM, WGS84G grid, 32-bit BIL behind zlib.
    Elevation,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct TileDesc {
    pub dataset: Dataset,
    pub z: u8,
    pub x: u32,
    pub y: u32,
}

// ---------------------------------------------------------------------------
// Abstraction
// ---------------------------------------------------------------------------

pub type TileBytes = Result<Vec<u8>, String>;
pub type TileCallback = Box<dyn FnOnce(TileBytes) + Send + 'static>;

pub trait TileSource {
    fn request(&self, desc: TileDesc, done: TileCallback);
}

/// HTTP source: IGN Géoplateforme, WMTS, no API key (CORS `*` verified).
pub struct HttpTileSource {
    base: String,
}

impl Default for HttpTileSource {
    fn default() -> Self {
        Self {
            base: "https://data.geopf.fr/wmts".to_owned(),
        }
    }
}

impl HttpTileSource {
    pub fn url(&self, desc: TileDesc) -> String {
        let (layer, matrix_set, format) = match desc.dataset {
            Dataset::Raster(l) => (l.wmts_name(), "PM", l.format()),
            Dataset::Elevation => (
                "ELEVATION.ELEVATIONGRIDCOVERAGE.HIGHRES",
                "WGS84G",
                "image/x-bil;bits=32",
            ),
        };
        format!(
            "{base}?SERVICE=WMTS&VERSION=1.0.0&REQUEST=GetTile&LAYER={layer}&STYLE=normal\
&TILEMATRIXSET={matrix_set}&TILEMATRIX={z}&TILEROW={y}&TILECOL={x}&FORMAT={format}",
            base = self.base,
            z = desc.z,
            y = desc.y,
            x = desc.x,
        )
    }
}

impl TileSource for HttpTileSource {
    fn request(&self, desc: TileDesc, done: TileCallback) {
        let request = ehttp::Request::get(self.url(desc));
        ehttp::fetch(request, move |result| {
            let bytes = match result {
                Err(e) => Err(e),
                Ok(resp) if !resp.ok => Err(format!("HTTP {} {}", resp.status, resp.status_text)),
                Ok(resp) => {
                    // WMTS reports its errors as XML with a 200 status code:
                    // decoding that as a PNG gives an incomprehensible error.
                    let ct = resp.content_type().unwrap_or_default().to_owned();
                    if ct.contains("xml") || ct.contains("text/html") {
                        Err(format!(
                            "non-tile response ({ct}): {}",
                            String::from_utf8_lossy(&resp.bytes[..resp.bytes.len().min(200)])
                        ))
                    } else {
                        Ok(resp.bytes)
                    }
                }
            };
            done(bytes);
        });
    }
}

// ---------------------------------------------------------------------------
// Asynchronous cache
// ---------------------------------------------------------------------------

/// Generic cache: asks the source for bytes, decodes them on arrival, keeps the
/// decoded value, evicts the least recently used ones.
pub struct TileCache<K: Eq + Hash + Clone + Send + 'static, V> {
    source: Rc<dyn TileSource>,
    inbox: Arc<Mutex<Vec<(K, TileBytes)>>>,
    ready: HashMap<K, V>,
    /// Requested and not yet usable — either still on the wire, or arrived and
    /// waiting for a decode slot.
    pending: HashSet<K>,
    failed: HashMap<K, String>,
    last_used: HashMap<K, u64>,
    clock: u64,
    capacity: usize,
    /// Requests actually on the wire right now. Distinct from `pending`: a reply
    /// that has landed but is not decoded yet no longer occupies the network.
    in_flight: usize,
    max_in_flight: usize,
    /// Keys already asked for during this frame, so the same tile is not queued
    /// four times by four callers.
    asked_this_frame: HashSet<K>,
}

impl<K: Eq + Hash + Clone + Send + 'static, V> TileCache<K, V> {
    /// `max_in_flight` bounds concurrent requests.
    ///
    /// ⚠️ This is what keeps panning responsive. Without a cap, dragging across
    /// the map fires a request per tile per frame; they cannot be cancelled, so
    /// tiles for ground you have already left keep occupying the connection
    /// while the ones under your eyes wait behind them. With a cap, at most that
    /// many stale requests can ever be in the way — everything else is simply
    /// asked again next frame, by which time the view has decided what it wants.
    pub fn new(source: Rc<dyn TileSource>, capacity: usize, max_in_flight: usize) -> Self {
        Self {
            source,
            inbox: Arc::new(Mutex::new(Vec::new())),
            ready: HashMap::new(),
            pending: HashSet::new(),
            failed: HashMap::new(),
            last_used: HashMap::new(),
            clock: 0,
            capacity,
            in_flight: 0,
            max_in_flight: max_in_flight.max(1),
            asked_this_frame: HashSet::new(),
        }
    }

    pub fn tick(&mut self) {
        self.clock += 1;
        // Wants do not carry over: whatever is still visible will ask again this
        // frame, and whatever scrolled off never will.
        self.asked_this_frame.clear();
    }

    /// Consumes replies that arrived since the last frame, at most `budget` of
    /// them.
    ///
    /// ⚠️ The budget is what keeps zooming smooth. Decoding a PNG tile and
    /// uploading it as a texture costs a few milliseconds; a zoom change
    /// invalidates the whole visible grid at once, so without a cap forty tiles
    /// would be decoded inside a single frame and the interface would freeze for
    /// a fifth of a second. Deferred replies keep their arrival order and are
    /// picked up next frame, which the `request_repaint` below guarantees will
    /// happen.
    pub fn pump(
        &mut self,
        budget: usize,
        ctx: &egui::Context,
        mut decode: impl FnMut(&K, &[u8]) -> Result<V, String>,
    ) {
        let mut arrived: Vec<(K, TileBytes)> = {
            let mut inbox = self.inbox.lock().unwrap();
            std::mem::take(&mut *inbox)
        };
        // Off the wire, decoded or not: the connection slot is free either way.
        // They stay in `pending` until decoded, so nothing re-requests them.
        self.in_flight = self.in_flight.saturating_sub(arrived.len());
        let deferred = if arrived.len() > budget {
            arrived.split_off(budget)
        } else {
            Vec::new()
        };

        for (key, bytes) in arrived {
            self.pending.remove(&key);
            match bytes.and_then(|b| decode(&key, &b)) {
                Ok(value) => {
                    self.ready.insert(key.clone(), value);
                    self.last_used.insert(key, self.clock);
                }
                Err(e) => {
                    log::warn!("tile failed: {e}");
                    self.failed.insert(key, e);
                }
            }
        }

        if !deferred.is_empty() {
            // Put them back in front of anything that landed meanwhile: a tile
            // that has been waiting one frame should not wait behind a newcomer.
            let mut inbox = self.inbox.lock().unwrap();
            let mut newer = std::mem::replace(&mut *inbox, deferred);
            inbox.append(&mut newer);
            ctx.request_repaint();
        }
    }

    /// True when the tile is available; starts the fetch when it is unknown and
    /// a connection slot is free.
    ///
    /// A tile refused a slot is simply not requested this frame. Since the
    /// visible grid is walked again every frame, it will ask again as soon as a
    /// slot frees — and if the view moved on, it never will, which is exactly
    /// the cancellation `fetch` does not give us.
    pub fn ensure(&mut self, key: &K, desc: TileDesc, ctx: &egui::Context) -> bool {
        if self.ready.contains_key(key) {
            self.last_used.insert(key.clone(), self.clock);
            return true;
        }
        if self.pending.contains(key)
            || self.failed.contains_key(key)
            || !self.asked_this_frame.insert(key.clone())
        {
            return false;
        }
        if self.in_flight < self.max_in_flight {
            self.spawn(key.clone(), desc, ctx);
        }
        false
    }

    /// Looks up without starting a fetch (used by the parent-tile fallback).
    pub fn peek(&mut self, key: &K) -> Option<&V> {
        if self.ready.contains_key(key) {
            self.last_used.insert(key.clone(), self.clock);
        }
        self.ready.get(key)
    }

    /// True when the tile is decoded and ready, without touching the LRU clock.
    pub fn is_ready(&self, key: &K) -> bool {
        self.ready.contains_key(key)
    }

    fn spawn(&mut self, key: K, desc: TileDesc, ctx: &egui::Context) {
        self.pending.insert(key.clone());
        self.in_flight += 1;
        let inbox = Arc::clone(&self.inbox);
        // The Context is cloned BEFORE the closure: without request_repaint the
        // tile would only show up on the next mouse move.
        let ctx = ctx.clone();
        self.source.request(
            desc,
            Box::new(move |bytes| {
                inbox.lock().unwrap().push((key, bytes));
                ctx.request_repaint();
            }),
        );
    }

    /// Evicts the least recently used decoded tiles.
    pub fn evict(&mut self) {
        if self.ready.len() <= self.capacity {
            return;
        }
        let mut ages: Vec<(u64, K)> = self
            .ready
            .keys()
            .map(|k| (*self.last_used.get(k).unwrap_or(&0), k.clone()))
            .collect();
        ages.sort_by_key(|(age, _)| *age);
        let drop_count = self.ready.len() - self.capacity;
        for (_, key) in ages.into_iter().take(drop_count) {
            self.ready.remove(&key);
            self.last_used.remove(&key);
        }
    }

    /// (ready, pending, failed)
    pub fn stats(&self) -> (usize, usize, usize) {
        (self.ready.len(), self.pending.len(), self.failed.len())
    }

    /// Requests currently on the wire.
    pub fn in_flight(&self) -> usize {
        self.in_flight
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The generated URL must be exactly the one verified on 2026-09-02 (200 OK).
    #[test]
    fn planign_base_map_url() {
        let s = HttpTileSource::default();
        let url = s.url(TileDesc {
            dataset: Dataset::Raster(RasterLayer::PlanIgn),
            z: 14,
            x: 8504,
            y: 5833,
        });
        assert_eq!(
            url,
            "https://data.geopf.fr/wmts?SERVICE=WMTS&VERSION=1.0.0&REQUEST=GetTile\
&LAYER=GEOGRAPHICALGRIDSYSTEMS.PLANIGNV2&STYLE=normal&TILEMATRIXSET=PM\
&TILEMATRIX=14&TILEROW=5833&TILECOL=8504&FORMAT=image/png"
        );
    }

    #[test]
    fn dem_url() {
        let s = HttpTileSource::default();
        let url = s.url(TileDesc {
            dataset: Dataset::Elevation,
            z: 14,
            x: 17010,
            y: 4012,
        });
        assert_eq!(
            url,
            "https://data.geopf.fr/wmts?SERVICE=WMTS&VERSION=1.0.0&REQUEST=GetTile\
&LAYER=ELEVATION.ELEVATIONGRIDCOVERAGE.HIGHRES&STYLE=normal&TILEMATRIXSET=WGS84G\
&TILEMATRIX=14&TILEROW=4012&TILECOL=17010&FORMAT=image/x-bil;bits=32"
        );
    }

    fn raster(x: u32) -> TileDesc {
        TileDesc {
            dataset: Dataset::Raster(RasterLayer::PlanIgn),
            z: 14,
            x,
            y: 0,
        }
    }

    /// A source that records requests and never answers — a hung endpoint.
    #[derive(Default)]
    struct SilentSource {
        calls: std::cell::RefCell<usize>,
    }

    impl TileSource for SilentSource {
        fn request(&self, _desc: TileDesc, _done: TileCallback) {
            *self.calls.borrow_mut() += 1;
            // `done` is dropped: the reply never comes.
        }
    }

    /// A source that answers on the spot.
    struct InstantSource;

    impl TileSource for InstantSource {
        fn request(&self, _desc: TileDesc, done: TileCallback) {
            done(Ok(vec![1, 2, 3]));
        }
    }

    /// Wanting fifty tiles must not put fifty requests on the wire. This is what
    /// keeps a fast pan from burying the tiles under your eyes beneath requests
    /// for ground you already left.
    #[test]
    fn concurrent_requests_are_capped() {
        let ctx = egui::Context::default();
        let src = Rc::new(SilentSource::default());
        let mut cache: TileCache<u32, ()> =
            TileCache::new(Rc::clone(&src) as Rc<dyn TileSource>, 100, 4);

        cache.tick();
        for i in 0..50u32 {
            assert!(!cache.ensure(&i, raster(i), &ctx));
        }
        assert_eq!(cache.in_flight(), 4);
        assert_eq!(*src.calls.borrow(), 4, "the cap must hold");

        // Next frame, same wants: the slots are still busy on hung requests, and
        // nothing already asked for is asked again.
        cache.tick();
        for i in 0..50u32 {
            cache.ensure(&i, raster(i), &ctx);
        }
        assert_eq!(*src.calls.borrow(), 4, "a hung request must not be re-sent");
    }

    /// Four callers wanting the same tile in one frame produce one request —
    /// four fine tiles falling back on one parent is the everyday case.
    #[test]
    fn one_tile_asked_four_times_is_fetched_once() {
        let ctx = egui::Context::default();
        let src = Rc::new(SilentSource::default());
        let mut cache: TileCache<u32, ()> =
            TileCache::new(Rc::clone(&src) as Rc<dyn TileSource>, 100, 8);
        cache.tick();
        for _ in 0..4 {
            cache.ensure(&7u32, raster(7), &ctx);
        }
        assert_eq!(*src.calls.borrow(), 1);
    }

    /// As replies land the slots free up and the queue drains — the cap must
    /// throttle, never deadlock.
    #[test]
    fn freed_slots_let_the_rest_through() {
        let ctx = egui::Context::default();
        let mut cache: TileCache<u32, usize> = TileCache::new(Rc::new(InstantSource), 100, 4);

        cache.tick();
        for i in 0..50u32 {
            cache.ensure(&i, raster(i), &ctx);
        }
        assert_eq!(cache.in_flight(), 4);

        // Generous decode budget: everything that landed is consumed.
        cache.pump(100, &ctx, |_, b| Ok(b.len()));
        assert_eq!(cache.in_flight(), 0);
        assert_eq!(cache.stats().0, 4, "four tiles decoded");

        cache.tick();
        for i in 0..50u32 {
            cache.ensure(&i, raster(i), &ctx);
        }
        assert_eq!(cache.in_flight(), 4, "the next four went out");
        cache.pump(100, &ctx, |_, b| Ok(b.len()));
        assert_eq!(cache.stats().0, 8);
    }

    /// The decode budget defers work without losing it or re-requesting it.
    #[test]
    fn the_decode_budget_defers_rather_than_drops() {
        let ctx = egui::Context::default();
        let mut cache: TileCache<u32, usize> = TileCache::new(Rc::new(InstantSource), 100, 16);
        cache.tick();
        for i in 0..6u32 {
            cache.ensure(&i, raster(i), &ctx);
        }
        cache.pump(2, &ctx, |_, b| Ok(b.len()));
        assert_eq!(cache.stats().0, 2, "only the budget was decoded");
        assert_eq!(cache.stats().1, 4, "the rest stay pending, not re-requested");
        cache.pump(10, &ctx, |_, b| Ok(b.len()));
        assert_eq!(cache.stats().0, 6, "nothing was lost");
        assert_eq!(cache.stats().1, 0);
    }

    /// Exactly the opaque ground layers are base maps — the rest composite on
    /// top and must stay stackable.
    #[test]
    fn only_ground_layers_are_base_maps() {
        assert!(RasterLayer::PlanIgn.is_base());
        assert!(RasterLayer::Ortho.is_base());
        assert!(!RasterLayer::Contours.is_base());
        assert!(!RasterLayer::Slopes.is_base());
    }
}
