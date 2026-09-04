//! Map view: tile grid, pan/zoom, parent-tile fallback, transparent layer
//! compositing.

use std::collections::HashMap;
use std::rc::Rc;

use egui::{Color32, Pos2, Rect, Sense, TextureHandle, TextureOptions, Vec2};
use web_time::Instant;

use crate::geo::{pm, LatLon, TILE_PX};
use crate::tiles::{Dataset, RasterLayer, TileCache, TileDesc, TileSource};

/// How many parent levels are explored when a tile is missing.
const MAX_PARENT_LEVELS: u8 = 5;

/// Guard rail: past this, drop to a coarser tile level rather than flooding the
/// network.
const MAX_TILES_PER_LAYER: usize = 400;

/// Concurrent tile requests. Bounded so a fast pan cannot bury the tiles you are
/// looking at under requests for ground you already left — see `TileCache::new`.
/// High enough to keep an HTTP/2 connection busy, low enough to bound the stale
/// backlog.
pub const MAX_TILE_REQUESTS: usize = 16;

/// Raster tiles decoded and uploaded as textures per frame. See
/// `TileCache::pump` — this is the cap that keeps a zoom change from freezing
/// the interface while forty PNGs are decoded at once.
const RASTER_DECODE_BUDGET: usize = 4;

/// How long the wheel has to be still before we commit to a new tile level.
///
/// Below this the zoom is treated as still in motion and the already-loaded
/// level is scaled instead, which costs nothing. A wheel gesture crosses three
/// or four integer levels; without this delay every one of them would trigger a
/// full grid of fetches and decodes that the next level immediately obsoletes.
const ZOOM_SETTLE_MS: u128 = 130;

/// How long the previous tile level keeps being painted underneath after a
/// level switch, so the map never flashes empty while the new tiles land.
const LEVEL_CROSSFADE_MS: u128 = 500;

/// How long the view has to be still before background data (trails) is fetched
/// for it. Longer than the tile delay: a drag would otherwise ask for a fresh
/// disc of trail tiles on every frame, and most of them would be thrown away
/// before they arrived.
const VIEW_SETTLE_MS: u128 = 500;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct RasterKey {
    pub layer: RasterLayer,
    pub z: u8,
    pub x: u32,
    pub y: u32,
}

// ---------------------------------------------------------------------------
// View state
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
pub struct MapView {
    /// Centre in normalised Web Mercator world coordinates (0..1).
    center: (f64, f64),
    /// Continuous zoom (the integer tile zoom is derived from it).
    pub zoom: f32,
    /// Last zoom change — gates committing to a new tile level.
    /// `None` means "never moved", hence settled.
    zoom_changed_at: Option<Instant>,
    /// Last movement of any kind — gates background data loading.
    moved_at: Option<Instant>,
}

impl MapView {
    pub fn centered_on(ll: LatLon, zoom: f32) -> Self {
        // `None` rather than an instant in the past: under WASM the clock starts
        // at zero on page load, so subtracting from `now` can underflow.
        // A freshly built view must count as settled — the first frame has to
        // load its tiles without waiting.
        Self {
            center: pm::latlon_to_world(ll),
            zoom,
            zoom_changed_at: None,
            moved_at: None,
        }
    }

    pub fn center_latlon(&self) -> LatLon {
        pm::world_to_latlon(self.center.0, self.center.1)
    }

    /// Screen pixels per world unit.
    pub fn scale(&self) -> f64 {
        TILE_PX as f64 * 2f64.powf(self.zoom as f64)
    }

    /// Integer tile zoom.
    pub fn tile_zoom(&self) -> u8 {
        self.zoom.round().clamp(0.0, 19.0) as u8
    }

    /// True once the wheel has been still long enough to commit to a new tile
    /// level.
    pub fn zoom_settled(&self) -> bool {
        self.zoom_changed_at
            .is_none_or(|t| t.elapsed().as_millis() >= ZOOM_SETTLE_MS)
    }

    /// True once the view has been still long enough to fetch background data.
    pub fn view_settled(&self) -> bool {
        self.moved_at
            .is_none_or(|t| t.elapsed().as_millis() >= VIEW_SETTLE_MS)
    }

    /// Marks the view as moved from outside (a slider, a recentre button).
    pub fn touch(&mut self) {
        self.moved_at = Some(Instant::now());
    }

    pub fn world_to_screen(&self, world: (f64, f64), rect: Rect) -> Pos2 {
        let s = self.scale();
        let c = rect.center();
        Pos2::new(
            c.x + ((world.0 - self.center.0) * s) as f32,
            c.y + ((world.1 - self.center.1) * s) as f32,
        )
    }

    pub fn screen_to_world(&self, pos: Pos2, rect: Rect) -> (f64, f64) {
        let s = self.scale();
        let c = rect.center();
        (
            self.center.0 + (pos.x - c.x) as f64 / s,
            self.center.1 + (pos.y - c.y) as f64 / s,
        )
    }

    pub fn latlon_to_screen(&self, ll: LatLon, rect: Rect) -> Pos2 {
        self.world_to_screen(pm::latlon_to_world(ll), rect)
    }

    pub fn screen_to_latlon(&self, pos: Pos2, rect: Rect) -> LatLon {
        let (x, y) = self.screen_to_world(pos, rect);
        pm::world_to_latlon(x, y)
    }

    pub fn pan_pixels(&mut self, delta: Vec2) {
        let s = self.scale();
        self.center.0 -= delta.x as f64 / s;
        self.center.1 -= delta.y as f64 / s;
        self.center.1 = self.center.1.clamp(0.0, 1.0);
        self.center.0 = self.center.0.rem_euclid(1.0);
        self.moved_at = Some(Instant::now());
    }

    /// Zooms while keeping the point under the cursor fixed.
    pub fn zoom_at(&mut self, delta: f32, anchor: Pos2, rect: Rect) {
        let before = self.screen_to_world(anchor, rect);
        let previous = self.zoom;
        self.zoom = (self.zoom + delta).clamp(3.0, 19.0);
        let after = self.screen_to_world(anchor, rect);
        self.center.0 += before.0 - after.0;
        self.center.1 += before.1 - after.1;
        if self.zoom != previous {
            let now = Some(Instant::now());
            self.zoom_changed_at = now;
            self.moved_at = now;
        }
    }

    /// Centres and zooms so that the whole lat/lon box fits inside `rect`.
    ///
    /// The zoom that shows a given area depends on the window, so this cannot be
    /// a constant: it is computed once the map rectangle is known.
    pub fn fit_bounds(&mut self, sw: LatLon, ne: LatLon, rect: Rect) {
        // North-west and south-east corners, in world coordinates.
        let nw = pm::latlon_to_world(LatLon::new(ne.lat, sw.lon));
        let se = pm::latlon_to_world(LatLon::new(sw.lat, ne.lon));
        let w = (se.0 - nw.0).abs().max(1e-12);
        let h = (se.1 - nw.1).abs().max(1e-12);
        self.center = ((nw.0 + se.0) / 2.0, (nw.1 + se.1) / 2.0);

        // scale = screen pixels per world unit, and scale = TILE_PX * 2^zoom.
        // A little margin so the box does not touch the edges.
        let scale = (rect.width() as f64 / w).min(rect.height() as f64 / h) * 0.92;
        self.zoom = ((scale / TILE_PX as f64).log2() as f32).clamp(3.0, 19.0);
    }

    /// Metres per screen pixel at the centre latitude (the scale bar).
    pub fn meters_per_pixel(&self) -> f64 {
        let lat = self.center_latlon().lat.to_radians();
        40_075_016.686 * lat.cos() / self.scale()
    }
}

// ---------------------------------------------------------------------------
// Base map rendering
// ---------------------------------------------------------------------------

/// Tile level currently drawn for one layer, and the one it replaced.
#[derive(Clone, Copy)]
struct LayerLevel {
    z: u8,
    previous: u8,
    switched_at: Instant,
}

/// One pass of the tile grid. Bundled rather than passed as eight positional
/// arguments, half of which are the same type.
struct GridPass<'a> {
    painter: &'a egui::Painter,
    rect: Rect,
    view: &'a MapView,
    layer: RasterLayer,
    z: u8,
    tint: Color32,
    /// `Some` to fetch what is missing, `None` to draw only what is decoded.
    ctx: Option<&'a egui::Context>,
}

pub struct TileRenderer {
    cache: TileCache<RasterKey, TextureHandle>,
    /// Committed tile level per layer. Only updated once the zoom has settled,
    /// which is what stops a wheel gesture from requesting every level it
    /// crosses.
    levels: HashMap<RasterLayer, LayerLevel>,
    /// Last-frame counters, for the debug panel.
    pub painted: usize,
    pub fallbacks: usize,
}

impl TileRenderer {
    pub fn new(source: Rc<dyn TileSource>) -> Self {
        Self {
            cache: TileCache::new(source, 512, MAX_TILE_REQUESTS),
            levels: HashMap::new(),
            painted: 0,
            fallbacks: 0,
        }
    }

    pub fn begin_frame(&mut self, ctx: &egui::Context) {
        self.cache.tick();
        self.painted = 0;
        self.fallbacks = 0;
        let tex_ctx = ctx.clone();
        self.cache.pump(RASTER_DECODE_BUDGET, ctx, move |key, bytes| {
            let img = decode_image(bytes)?;
            let name = format!("{:?}/{}/{}/{}", key.layer, key.z, key.x, key.y);
            Ok(tex_ctx.load_texture(name, img, TextureOptions::LINEAR))
        });
    }

    pub fn end_frame(&mut self) {
        self.cache.evict();
    }

    pub fn stats(&self) -> (usize, usize, usize) {
        self.cache.stats()
    }

    /// Tile requests currently on the wire — the number to look at when loading
    /// feels slow: pinned at the cap means the network is the bottleneck.
    pub fn in_flight(&self) -> usize {
        self.cache.in_flight()
    }

    /// Paints one layer over `rect`. `opacity` in [0,1]: this is the transparent
    /// compositing — every layer shares the PM grid, so they stack pixel for
    /// pixel with no reprojection.
    pub fn paint_layer(
        &mut self,
        painter: &egui::Painter,
        rect: Rect,
        view: &MapView,
        layer: RasterLayer,
        opacity: f32,
        ctx: &egui::Context,
    ) {
        if opacity <= 0.0 {
            return;
        }
        let (min_z, max_z) = layer.zoom_range();
        let target_z = view.tile_zoom().clamp(min_z, max_z);
        let level = self.level_for(layer, target_z, view);

        let tint = Color32::from_white_alpha((opacity.clamp(0.0, 1.0) * 255.0) as u8);

        // While the new level is still loading, the previous one stays visible
        // underneath. Without it the map flashes empty on every zoom-out, which
        // reads as slowness even when the tiles arrive quickly.
        if level.previous != level.z
            && level.switched_at.elapsed().as_millis() < LEVEL_CROSSFADE_MS
        {
            self.paint_grid(GridPass {
                painter,
                rect,
                view,
                layer,
                z: level.previous,
                tint,
                ctx: None,
            });
        }
        self.paint_grid(GridPass {
            painter,
            rect,
            view,
            layer,
            z: level.z,
            tint,
            ctx: Some(ctx),
        });
    }

    /// Decides which tile level to draw, and commits to a new one only once the
    /// zoom has settled.
    fn level_for(&mut self, layer: RasterLayer, target_z: u8, view: &MapView) -> LayerLevel {
        let (min_z, max_z) = layer.zoom_range();
        let current = self.levels.get(&layer).copied();

        if view.zoom_settled() || current.is_none() {
            let previous = current.map(|l| l.z).unwrap_or(target_z);
            let level = if previous == target_z {
                // Unchanged: keep the original switch instant so the crossfade
                // does not restart on every frame.
                current.unwrap_or(LayerLevel {
                    z: target_z,
                    previous: target_z,
                    switched_at: Instant::now(),
                })
            } else {
                LayerLevel {
                    z: target_z,
                    previous,
                    switched_at: Instant::now(),
                }
            };
            self.levels.insert(layer, level);
            return level;
        }

        // Mid-gesture: keep the loaded level and let egui scale it. Bounded so a
        // long zoom-out never asks for thousands of fine tiles at once.
        let held = current.expect("handled above");
        let low = target_z.saturating_sub(3).max(min_z);
        let high = (target_z + 1).min(max_z);
        LayerLevel {
            z: held.z.clamp(low, high),
            previous: held.previous,
            switched_at: held.switched_at,
        }
    }

    /// Paints the visible tile grid of one layer at one level.
    ///
    /// A `ctx` means missing tiles are fetched; `None` means draw only what is
    /// already decoded — that is the underlay, which must never generate traffic
    /// for a level we are about to leave.
    fn paint_grid(&mut self, pass: GridPass) {
        let GridPass {
            painter,
            rect,
            view,
            layer,
            z,
            tint,
            ctx,
        } = pass;
        let (min_z, _) = layer.zoom_range();
        let s = view.scale();
        let half = Vec2::new(rect.width(), rect.height()) / 2.0;
        let w_min = view.screen_to_world(rect.center() - half, rect);
        let w_max = view.screen_to_world(rect.center() + half, rect);

        // Drop to a coarser level rather than paint nothing: a very wide window
        // used to leave the map grey, which is the worst possible answer.
        let mut z = z;
        let (x0, x1, y0, y1) = loop {
            let n = (1u64 << z) as f64;
            let x0 = (w_min.0 * n).floor() as i64;
            let x1 = (w_max.0 * n).floor() as i64;
            let y0 = ((w_min.1 * n).floor() as i64).max(0);
            let y1 = ((w_max.1 * n).floor() as i64).min(n as i64 - 1);
            let count = ((x1 - x0 + 1).max(0) * (y1 - y0 + 1).max(0)) as usize;
            if count == 0 {
                return;
            }
            if count <= MAX_TILES_PER_LAYER {
                break (x0, x1, y0, y1);
            }
            if z <= min_z {
                return;
            }
            z -= 1;
        };

        let n = (1u64 << z) as f64;
        let tile_side = (s / n) as f32;
        for ty in y0..=y1 {
            for tx in x0..=x1 {
                // East-west wrap: the column folds back onto the world.
                let key = RasterKey {
                    layer,
                    z,
                    x: tx.rem_euclid(n as i64) as u32,
                    y: ty as u32,
                };
                let origin = view.world_to_screen((tx as f64 / n, ty as f64 / n), rect);
                let dest = Rect::from_min_size(origin, Vec2::splat(tile_side));

                let available = match ctx {
                    Some(ctx) => {
                        let desc = TileDesc {
                            dataset: Dataset::Raster(layer),
                            z,
                            x: key.x,
                            y: key.y,
                        };
                        self.cache.ensure(&key, desc, ctx)
                    }
                    None => self.cache.is_ready(&key),
                };
                if available {
                    if let Some(tex) = self.cache.peek(&key) {
                        painter.image(tex.id(), dest, FULL_UV, tint);
                        self.painted += 1;
                        continue;
                    }
                }
                let Some(ctx) = ctx else {
                    continue;
                };
                // Fallback: a magnified ancestor tile, cropped through its UVs.
                if let Some((tex_id, uv)) = self.parent_fallback(layer, z, key.x, key.y) {
                    painter.image(tex_id, dest, uv, tint);
                    self.fallbacks += 1;
                } else if z > min_z {
                    // No ancestor to stand in, so fetch the immediate parent.
                    //
                    // ⚠️ This is what makes zooming out bearable. The fallback
                    // only ever looks *upwards*, and zooming out is precisely the
                    // case where nothing coarser was ever loaded — the map had no
                    // source of coverage at all and simply stayed grey. One
                    // coarse tile covers four fine ones and arrives sooner, so
                    // the view now fills in coarse first, then sharpens.
                    let parent = RasterKey {
                        layer,
                        z: z - 1,
                        x: key.x >> 1,
                        y: key.y >> 1,
                    };
                    let desc = TileDesc {
                        dataset: Dataset::Raster(layer),
                        z: parent.z,
                        x: parent.x,
                        y: parent.y,
                    };
                    self.cache.ensure(&parent, desc, ctx);
                }
            }
        }
    }

    /// Looks for an already-loaded ancestor and returns the matching UV window.
    fn parent_fallback(
        &mut self,
        layer: RasterLayer,
        z: u8,
        x: u32,
        y: u32,
    ) -> Option<(egui::TextureId, Rect)> {
        for k in 1..=MAX_PARENT_LEVELS.min(z) {
            let key = RasterKey {
                layer,
                z: z - k,
                x: x >> k,
                y: y >> k,
            };
            let f = (1u32 << k) as f32;
            let sx = (x & ((1 << k) - 1)) as f32;
            let sy = (y & ((1 << k) - 1)) as f32;
            let uv = Rect::from_min_max(
                Pos2::new(sx / f, sy / f),
                Pos2::new((sx + 1.0) / f, (sy + 1.0) / f),
            );
            if let Some(tex) = self.cache.peek(&key) {
                return Some((tex.id(), uv));
            }
        }
        None
    }
}

const FULL_UV: Rect = Rect {
    min: Pos2 { x: 0.0, y: 0.0 },
    max: Pos2 { x: 1.0, y: 1.0 },
};

fn decode_image(bytes: &[u8]) -> Result<egui::ColorImage, String> {
    let img = image::load_from_memory(bytes).map_err(|e| format!("image decode: {e}"))?;
    let rgba = img.to_rgba8();
    let size = [rgba.width() as usize, rgba.height() as usize];
    Ok(egui::ColorImage::from_rgba_unmultiplied(
        size,
        rgba.as_raw(),
    ))
}

/// Interactive map area: drag to pan, wheel to zoom.
pub fn interact(ui: &mut egui::Ui, rect: Rect, view: &mut MapView) -> egui::Response {
    let response = ui.interact(rect, ui.id().with("map"), Sense::click_and_drag());
    if response.dragged() {
        view.pan_pixels(response.drag_delta());
    }
    if response.hovered() {
        let scroll = ui.input(|i| i.smooth_scroll_delta.y);
        if scroll.abs() > 0.0 {
            if let Some(pointer) = ui.input(|i| i.pointer.hover_pos()) {
                view.zoom_at(scroll * 0.005, pointer, rect);
            }
        }
    }
    // While the zoom is settling nothing else would ask for a repaint, and the
    // map would stay on the coarse level until the next mouse move.
    if !view.zoom_settled() {
        ui.ctx()
            .request_repaint_after(std::time::Duration::from_millis(ZOOM_SETTLE_MS as u64));
    }
    response
}

#[cfg(test)]
mod tests {
    use super::*;

    const CHAMONIX: LatLon = LatLon::new(45.92, 6.87);

    /// A freshly built view must count as settled, otherwise the very first
    /// frame would refuse to load any tile.
    #[test]
    fn a_new_view_is_settled() {
        let view = MapView::centered_on(CHAMONIX, 14.0);
        assert!(view.zoom_settled());
        assert!(view.view_settled());
    }

    /// Zooming marks the view as moving, so the renderer holds its tile level
    /// instead of chasing every intermediate zoom.
    #[test]
    fn zooming_unsettles_the_view() {
        let rect = Rect::from_min_size(Pos2::ZERO, Vec2::splat(800.0));
        let mut view = MapView::centered_on(CHAMONIX, 14.0);
        view.zoom_at(0.4, rect.center(), rect);
        assert!(!view.zoom_settled());
        assert!(!view.view_settled());
    }

    /// A zoom clamped at the limit changes nothing and must not restart the
    /// settle delay — otherwise scrolling against the stop would freeze loading.
    #[test]
    fn a_zoom_that_changes_nothing_stays_settled() {
        let rect = Rect::from_min_size(Pos2::ZERO, Vec2::splat(800.0));
        let mut view = MapView::centered_on(CHAMONIX, 19.0);
        view.zoom_at(1.0, rect.center(), rect);
        assert_eq!(view.zoom, 19.0);
        assert!(view.zoom_settled());
    }

    /// Panning does not disturb the tile level, only background loading.
    #[test]
    fn panning_keeps_the_tile_level() {
        let mut view = MapView::centered_on(CHAMONIX, 14.0);
        view.pan_pixels(Vec2::new(50.0, 0.0));
        assert!(view.zoom_settled(), "a pan must not reset the tile level");
        assert!(!view.view_settled());
    }

    /// A fitted box must actually land inside the rectangle, corners included,
    /// and stay centred on it.
    #[test]
    fn fit_bounds_contains_the_box() {
        let rect = Rect::from_min_size(Pos2::new(0.0, 0.0), Vec2::new(1280.0, 900.0));
        let sw = LatLon::new(43.95, 5.35);
        let ne = LatLon::new(46.45, 7.75);
        let mut view = MapView::centered_on(LatLon::new(0.0, 0.0), 14.0);
        view.fit_bounds(sw, ne, rect);

        for corner in [
            LatLon::new(sw.lat, sw.lon),
            LatLon::new(ne.lat, ne.lon),
            LatLon::new(sw.lat, ne.lon),
            LatLon::new(ne.lat, sw.lon),
        ] {
            let p = view.latlon_to_screen(corner, rect);
            assert!(rect.contains(p), "{corner:?} lands outside at {p:?}");
        }
        let center = view.center_latlon();
        assert!((center.lon - (sw.lon + ne.lon) / 2.0).abs() < 0.01);
        assert!(center.lat > sw.lat && center.lat < ne.lat);
        // A wider window fits the same box at a closer zoom, never further out.
        let mut wide = view;
        wide.fit_bounds(sw, ne, Rect::from_min_size(Pos2::ZERO, Vec2::new(2560.0, 1800.0)));
        assert!(wide.zoom > view.zoom, "{} vs {}", wide.zoom, view.zoom);
    }

    /// The point under the cursor must stay put across a zoom — that is the
    /// whole contract of `zoom_at`.
    #[test]
    fn zoom_keeps_the_anchor_fixed() {
        let rect = Rect::from_min_size(Pos2::ZERO, Vec2::splat(800.0));
        let mut view = MapView::centered_on(CHAMONIX, 14.0);
        let anchor = Pos2::new(200.0, 600.0);
        let before = view.screen_to_latlon(anchor, rect);
        view.zoom_at(0.7, anchor, rect);
        let after = view.screen_to_latlon(anchor, rect);
        assert!(crate::geo::haversine_m(before, after) < 1.0);
    }
}
