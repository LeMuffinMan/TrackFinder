//! Vue carte : grille de tuiles, pan/zoom, repli sur tuile parente, composition
//! des couches en transparence.

use std::rc::Rc;

use egui::{Color32, Pos2, Rect, Sense, TextureHandle, TextureOptions, Vec2};

use crate::geo::{pm, LatLon, TILE_PX};
use crate::tiles::{Dataset, RasterLayer, TileCache, TileDesc, TileSource};

/// Nombre de niveaux parents explorés quand une tuile manque.
const MAX_PARENT_LEVELS: u8 = 5;
/// Garde-fou : au-delà, on ne peint rien plutôt que d'inonder le réseau.
const MAX_TILES_PER_LAYER: usize = 400;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct RasterKey {
    pub layer: RasterLayer,
    pub z: u8,
    pub x: u32,
    pub y: u32,
}

// ---------------------------------------------------------------------------
// État de la vue
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
pub struct MapView {
    /// Centre en coordonnées monde Web Mercator normalisées (0..1).
    center: (f64, f64),
    /// Zoom continu (le zoom entier des tuiles en est dérivé).
    pub zoom: f32,
}

impl MapView {
    pub fn centered_on(ll: LatLon, zoom: f32) -> Self {
        Self {
            center: pm::latlon_to_world(ll),
            zoom,
        }
    }

    pub fn center_latlon(&self) -> LatLon {
        pm::world_to_latlon(self.center.0, self.center.1)
    }

    /// Pixels écran par unité monde.
    pub fn scale(&self) -> f64 {
        TILE_PX as f64 * 2f64.powf(self.zoom as f64)
    }

    /// Zoom entier des tuiles.
    pub fn tile_zoom(&self) -> u8 {
        self.zoom.round().clamp(0.0, 19.0) as u8
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
    }

    /// Zoome en gardant fixe le point sous le curseur.
    pub fn zoom_at(&mut self, delta: f32, anchor: Pos2, rect: Rect) {
        let before = self.screen_to_world(anchor, rect);
        self.zoom = (self.zoom + delta).clamp(3.0, 19.0);
        let after = self.screen_to_world(anchor, rect);
        self.center.0 += before.0 - after.0;
        self.center.1 += before.1 - after.1;
    }

    /// Mètres par pixel écran à la latitude du centre (échelle).
    pub fn meters_per_pixel(&self) -> f64 {
        let lat = self.center_latlon().lat.to_radians();
        40_075_016.686 * lat.cos() / self.scale()
    }
}

// ---------------------------------------------------------------------------
// Rendu des fonds
// ---------------------------------------------------------------------------

pub struct TileRenderer {
    cache: TileCache<RasterKey, TextureHandle>,
    /// Comptes de la dernière frame, pour l'affichage de debug.
    pub painted: usize,
    pub fallbacks: usize,
}

impl TileRenderer {
    pub fn new(source: Rc<dyn TileSource>) -> Self {
        Self {
            cache: TileCache::new(source, 512),
            painted: 0,
            fallbacks: 0,
        }
    }

    pub fn begin_frame(&mut self, ctx: &egui::Context) {
        self.cache.tick();
        self.painted = 0;
        self.fallbacks = 0;
        let ctx = ctx.clone();
        self.cache.pump(move |key, bytes| {
            let img = decode_image(bytes)?;
            let name = format!("{:?}/{}/{}/{}", key.layer, key.z, key.x, key.y);
            Ok(ctx.load_texture(name, img, TextureOptions::LINEAR))
        });
    }

    pub fn end_frame(&mut self) {
        self.cache.evict();
    }

    pub fn stats(&self) -> (usize, usize, usize) {
        self.cache.stats()
    }

    /// Peint une couche sur `rect`. `opacity` dans [0,1] : c'est la composition
    /// en transparence — les couches partagent la grille PM, donc superposition
    /// pixel à pixel, sans reprojection.
    pub fn paint_layer(
        &mut self,
        painter: &egui::Painter,
        rect: Rect,
        view: &MapView,
        layer: RasterLayer,
        opacity: f32,
        ctx: &egui::Context,
    ) {
        let (min_z, max_z) = layer.zoom_range();
        let z = view.tile_zoom().clamp(min_z, max_z);
        let n = (1u64 << z) as f64;
        let s = view.scale();

        // Fenêtre visible en coordonnées monde.
        let half = Vec2::new(rect.width(), rect.height()) / 2.0;
        let w_min = view.screen_to_world(rect.center() - half, rect);
        let w_max = view.screen_to_world(rect.center() + half, rect);

        let x0 = (w_min.0 * n).floor() as i64;
        let x1 = (w_max.0 * n).floor() as i64;
        let y0 = ((w_min.1 * n).floor() as i64).max(0);
        let y1 = ((w_max.1 * n).floor() as i64).min(n as i64 - 1);

        let count = ((x1 - x0 + 1).max(0) * (y1 - y0 + 1).max(0)) as usize;
        if count == 0 || count > MAX_TILES_PER_LAYER {
            return;
        }

        let tint = Color32::from_white_alpha((opacity.clamp(0.0, 1.0) * 255.0) as u8);
        let tile_side = (s / n) as f32;

        for ty in y0..=y1 {
            for tx in x0..=x1 {
                // Enroulement est-ouest : la colonne se replie sur le monde.
                let wrapped_x = tx.rem_euclid(n as i64) as u32;
                let key = RasterKey {
                    layer,
                    z,
                    x: wrapped_x,
                    y: ty as u32,
                };
                let origin = view.world_to_screen((tx as f64 / n, ty as f64 / n), rect);
                let dest = Rect::from_min_size(origin, Vec2::splat(tile_side));

                let desc = TileDesc {
                    dataset: Dataset::Raster(layer),
                    z,
                    x: key.x,
                    y: key.y,
                };
                if self.cache.ensure(&key, desc, ctx) {
                    if let Some(tex) = self.cache.peek(&key) {
                        painter.image(tex.id(), dest, FULL_UV, tint);
                        self.painted += 1;
                        continue;
                    }
                }
                // Repli : tuile parente agrandie, découpée par ses UV.
                if let Some((tex_id, uv)) = self.parent_fallback(layer, z, key.x, key.y) {
                    painter.image(tex_id, dest, uv, tint);
                    self.fallbacks += 1;
                }
            }
        }
    }

    /// Cherche un ancêtre déjà chargé et renvoie la portion d'UV correspondante.
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
    let img = image::load_from_memory(bytes).map_err(|e| format!("décodage image : {e}"))?;
    let rgba = img.to_rgba8();
    let size = [rgba.width() as usize, rgba.height() as usize];
    Ok(egui::ColorImage::from_rgba_unmultiplied(
        size,
        rgba.as_raw(),
    ))
}

/// Zone interactive de la carte : pan à la souris, zoom à la molette.
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
    response
}
