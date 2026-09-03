use std::rc::Rc;

use egui::{Color32, Pos2, Rect, Stroke, Vec2};

use crate::dem::DemStore;
use crate::geo::LatLon;
use crate::map::{self, MapView, TileRenderer};
use crate::tiles::{HttpTileSource, RasterLayer, TileSource};
use crate::graph::{self, CostMs, EdgePos, Graph, Reach};
use crate::track::{format_duration, Track, WalkSettings, Waypoint};
use crate::trails::{Snap, TrailStore, SNAP_RADIUS_M};

const CHAMONIX: LatLon = LatLon::new(45.92, 6.87);

struct LayerSetting {
    layer: RasterLayer,
    enabled: bool,
    opacity: f32,
}

pub struct TrackFinderApp {
    view: MapView,
    renderer: TileRenderer,
    dem: DemStore,
    track: Track,
    walk: WalkSettings,
    layers: Vec<LayerSetting>,
    trails: TrailStore,
    graph: Graph,
    /// Le réseau a changé : le graphe doit être reconstruit.
    graph_dirty: bool,
    /// Les dénivelés des arêtes sont tous renseignés.
    graph_elevated: bool,
    iso: Option<Isochrone>,
    iso_dirty: bool,
    /// Budget de l'étape, en heures. C'est un réglage du moment, pas du profil :
    /// une seule source de vérité.
    budget_h: f32,
    show_iso: bool,
    /// Clic en attente de la zone Overpass qui permettra de l'accrocher.
    pending_click: Option<LatLon>,
    snap_to_trail: bool,
    show_trails: bool,
    hover: Option<(LatLon, Option<f32>)>,
    status: Option<String>,
    /// Dernière zone carte peinte — sert au bouton « charger les sentiers de la vue ».
    last_map_rect: Rect,
    show_debug: bool,
}

impl Default for TrackFinderApp {
    fn default() -> Self {
        Self::new()
    }
}

impl TrackFinderApp {
    pub fn new() -> Self {
        // Une seule source partagée : demain un cache hors-ligne s'insère ici,
        // sans que le rendu ni le MNT ne changent d'une ligne.
        let source: Rc<dyn TileSource> = Rc::new(HttpTileSource::default());
        Self {
            view: MapView::centered_on(CHAMONIX, 14.0),
            renderer: TileRenderer::new(Rc::clone(&source)),
            dem: DemStore::new(source),
            track: Track::default(),
            walk: WalkSettings::default(),
            layers: vec![
                LayerSetting {
                    layer: RasterLayer::Ortho,
                    enabled: false,
                    opacity: 1.0,
                },
                LayerSetting {
                    layer: RasterLayer::PlanIgn,
                    enabled: true,
                    opacity: 1.0,
                },
                LayerSetting {
                    layer: RasterLayer::Slopes,
                    enabled: false,
                    opacity: 0.45,
                },
                LayerSetting {
                    layer: RasterLayer::Contours,
                    enabled: true,
                    opacity: 0.8,
                },
            ],
            trails: TrailStore::overpass(),
            graph: Graph::default(),
            graph_dirty: false,
            graph_elevated: false,
            iso: None,
            iso_dirty: false,
            budget_h: 3.0,
            show_iso: true,
            pending_click: None,
            snap_to_trail: true,
            show_trails: true,
            hover: None,
            status: None,
            last_map_rect: Rect::NOTHING,
            show_debug: false,
        }
    }
}

impl eframe::App for TrackFinderApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.ui_impl(ui);
    }
}

impl TrackFinderApp {
    /// Le corps de la frame, sans `eframe::Frame` : pilotable hors interface
    /// (mesures de performance, tests).
    fn ui_impl(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();
        self.renderer.begin_frame(&ctx);
        self.dem.begin_frame();
        if self.trails.pump(&ctx) {
            // De nouveaux sentiers peuvent compléter des liaisons déjà posées.
            self.track.invalidate();
            self.graph_dirty = true;
        }
        self.update_graph(&ctx);
        self.resolve_pending_click(&ctx);

        egui::Panel::right("panneau")
            .default_size(320.0)
            .show(ui, |ui| self.side_panel(ui));

        self.map_area(ui, &ctx);

        self.renderer.end_frame();
        self.dem.end_frame();
    }
}

impl TrackFinderApp {
    // -----------------------------------------------------------------------
    // Carte
    // -----------------------------------------------------------------------
    fn map_area(&mut self, ui: &mut egui::Ui, ctx: &egui::Context) {
        let rect = ui.available_rect_before_wrap();
        if rect.width() < 1.0 || rect.height() < 1.0 {
            return;
        }
        self.last_map_rect = rect;
        let response = map::interact(ui, rect, &mut self.view);

        if response.clicked() {
            if let Some(pos) = response.interact_pointer_pos() {
                let ll = self.view.screen_to_latlon(pos, rect);
                if self.snap_to_trail {
                    self.trails.ensure(ll, ctx);
                    self.pending_click = Some(ll);
                    self.status = None;
                } else {
                    self.track.push(Waypoint::free(ll));
                }
            }
        }
        if response.secondary_clicked() {
            self.track.pop();
            self.iso_dirty = true;
        }

        self.hover = response.hover_pos().map(|pos| {
            let ll = self.view.screen_to_latlon(pos, rect);
            (ll, self.dem.elevation(ll, ctx))
        });

        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 0.0, Color32::from_gray(30));

        for setting in &self.layers {
            if setting.enabled {
                self.renderer.paint_layer(
                    &painter,
                    rect,
                    &self.view,
                    setting.layer,
                    setting.opacity,
                    ctx,
                );
            }
        }

        if self.show_trails {
            self.paint_trails(&painter, rect);
        }
        self.paint_isochrone(&painter, rect);
        self.track
            .refresh(&self.trails.net, &mut self.dem, &self.walk, ctx);
        self.paint_track(&painter, rect);
        self.paint_scale_bar(&painter, rect);
    }

    /// Réseau OSM chargé, en fin de liste des couches : c'est un repère pour
    /// savoir où l'on peut cliquer.
    fn paint_trails(&self, painter: &egui::Painter, rect: Rect) {
        let sw = self.view.screen_to_latlon(rect.left_bottom(), rect);
        let ne = self.view.screen_to_latlon(rect.right_top(), rect);
        let view = (sw.lat, sw.lon, ne.lat, ne.lon);
        for way in self.trails.net.ways() {
            if !way.intersects(view) {
                continue;
            }
            let pts: Vec<Pos2> = way
                .points
                .iter()
                .map(|ll| self.view.latlon_to_screen(*ll, rect))
                .collect();
            painter.add(egui::Shape::line(
                pts,
                Stroke::new(1.5, way.kind.color().gamma_multiply(0.75)),
            ));
        }
    }

    fn paint_track(&self, painter: &egui::Painter, rect: Rect) {
        let path: Vec<Pos2> = self
            .track
            .path()
            .iter()
            .map(|ll| self.view.latlon_to_screen(*ll, rect))
            .collect();
        if path.len() >= 2 {
            painter.add(egui::Shape::line(
                path,
                Stroke::new(4.0, Color32::from_rgb(220, 40, 40)),
            ));
        }
        let pts: Vec<Pos2> = self
            .track
            .waypoints
            .iter()
            .map(|wp| self.view.latlon_to_screen(wp.pos, rect))
            .collect();
        for (i, p) in pts.iter().enumerate() {
            let color = if i == 0 {
                Color32::from_rgb(40, 180, 80)
            } else {
                Color32::WHITE
            };
            painter.circle(*p, 5.0, color, Stroke::new(2.0, Color32::BLACK));
        }
    }

    fn paint_scale_bar(&self, painter: &egui::Painter, rect: Rect) {
        let mpp = self.view.meters_per_pixel();
        // Longueur « ronde » la plus proche de 100 px.
        let target_m = mpp * 100.0;
        let pow = 10f64.powf(target_m.log10().floor());
        let nice = [1.0, 2.0, 5.0, 10.0]
            .into_iter()
            .map(|f| f * pow)
            .min_by(|a, b| {
                (a - target_m)
                    .abs()
                    .partial_cmp(&(b - target_m).abs())
                    .unwrap()
            })
            .unwrap_or(pow);
        let px = (nice / mpp) as f32;
        let y = rect.bottom() - 24.0;
        let x0 = rect.left() + 16.0;
        let stroke = Stroke::new(3.0, Color32::BLACK);
        painter.line_segment([Pos2::new(x0, y), Pos2::new(x0 + px, y)], stroke);
        painter.line_segment([Pos2::new(x0, y - 5.0), Pos2::new(x0, y + 5.0)], stroke);
        painter.line_segment(
            [
                Pos2::new(x0 + px, y - 5.0),
                Pos2::new(x0 + px, y + 5.0),
            ],
            stroke,
        );
        let label = if nice >= 1000.0 {
            format!("{:.0} km", nice / 1000.0)
        } else {
            format!("{nice:.0} m")
        };
        painter.text(
            Pos2::new(x0, y - 8.0),
            egui::Align2::LEFT_BOTTOM,
            label,
            egui::FontId::proportional(13.0),
            Color32::BLACK,
        );
    }

    /// Un clic ne peut être accroché qu'une fois la zone Overpass arrivée : on le
    /// garde en attente au lieu de le perdre ou de bloquer l'interface.
    fn resolve_pending_click(&mut self, ctx: &egui::Context) {
        let Some(ll) = self.pending_click else {
            return;
        };
        if let Some(err) = self.trails.zone_failed(ll) {
            self.status = Some(format!("Sentiers indisponibles : {err}"));
            self.pending_click = None;
            return;
        }
        if !self.trails.zone_ready(ll) {
            return;
        }
        self.pending_click = None;
        let Some(snap) = self.trails.net.snap(ll, SNAP_RADIUS_M) else {
            self.status = Some(format!(
                "Aucun sentier à moins de {SNAP_RADIUS_M:.0} m — point refusé"
            ));
            return;
        };
        // Premier point : il n'y a rien à relier. Ensuite, on privilégie
        // l'itinéraire par le graphe, et on retombe sur le suivi de tronçon.
        if self.track.waypoints.is_empty() {
            self.status = Some(format!("Départ accroché à {:.0} m du clic", snap.dist_m));
            self.track.push(Waypoint::snapped(snap));
        } else if let Some(via) = self.route_to(&snap) {
            let km = crate::trails::path_length_m(&via) / 1000.0;
            self.status = Some(format!("Étape tracée sur sentier : {km:.2} km"));
            self.track.push(Waypoint::routed(snap, via));
        } else {
            self.status = Some(
                "Hors de l'isochrone (ou graphe incomplet) — liaison directe".to_owned(),
            );
            self.track.push(Waypoint::snapped(snap));
        }
        self.iso_dirty = true;
        ctx.request_repaint();
    }

    // -----------------------------------------------------------------------
    // Panneau
    // -----------------------------------------------------------------------
    fn side_panel(&mut self, ui: &mut egui::Ui) {
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.heading("TrackFinder");
            ui.label("Clic gauche : ajouter un point · clic droit : retirer le dernier");
            ui.separator();

            ui.strong("Couches");
            let z = self.view.tile_zoom();
            for setting in &mut self.layers {
                ui.horizontal(|ui| {
                    ui.checkbox(&mut setting.enabled, setting.layer.label());
                    let (_, max_z) = setting.layer.zoom_range();
                    if z > max_z {
                        ui.weak(format!("(≤ z{max_z})"));
                    }
                });
                if setting.enabled {
                    ui.add(
                        egui::Slider::new(&mut setting.opacity, 0.0..=1.0)
                            .text("opacité")
                            .show_value(false),
                    );
                }
            }

            ui.separator();
            ui.strong("Sentiers (OSM)");
            ui.checkbox(&mut self.show_trails, "Afficher le réseau");
            ui.checkbox(&mut self.snap_to_trail, "Coller les points au sentier");
            let (zones, pending, failed, ways) = self.trails.stats();
            ui.monospace(format!("{zones} zones · {ways} chemins"));
            if pending > 0 {
                ui.horizontal(|ui| {
                    ui.spinner();
                    ui.label(format!("{pending} zone(s) en cours"));
                });
            }
            if failed > 0 {
                ui.colored_label(
                    Color32::from_rgb(200, 90, 90),
                    format!("{failed} zone(s) en échec"),
                );
                if ui.button("Réessayer").clicked() {
                    let ctx = ui.ctx().clone();
                    self.trails.retry_failed(&ctx);
                }
            }
            if ui.button("Charger les sentiers de la vue").clicked() {
                let ctx = ui.ctx().clone();
                self.load_visible_zones(&ctx);
            }
            if let Some(status) = &self.status {
                ui.weak(status);
            }

            ui.separator();
            ui.strong("Isochrone");
            if ui
                .checkbox(&mut self.show_iso, "Depuis le dernier point")
                .changed()
            {
                self.iso_dirty = true;
            }
            if ui
                .add(
                    egui::Slider::new(&mut self.budget_h, 0.5..=10.0)
                        .text("budget")
                        .suffix(" h"),
                )
                .changed()
            {
                self.iso_dirty = true;
            }
            if self.graph.is_empty() {
                ui.weak("Graphe vide — pose un point ou charge des sentiers");
            } else {
                ui.monospace(format!(
                    "graphe    {} nœuds · {} arêtes",
                    self.graph.node_count(),
                    self.graph.edges.len()
                ));
                if !self.graph_elevated {
                    ui.weak("dénivelés en cours de lecture (MNT)");
                }
            }
            match &self.iso {
                Some(iso) => {
                    ui.monospace(format!(
                        "atteint   {} nœuds · {} tronçons",
                        iso.reach.reached_count(),
                        iso.edges.len()
                    ));
                }
                None if !self.track.waypoints.is_empty() && self.show_iso => {
                    ui.weak("dernier point hors graphe");
                }
                None => {}
            }
            if ui
                .button("Charger le bassin de l'isochrone")
                .on_hover_text("Grandes zones Overpass autour du dernier point")
                .clicked()
            {
                let ctx = ui.ctx().clone();
                self.load_isochrone_area(&ctx);
            }

            ui.separator();
            ui.strong("Vue");
            ui.add(egui::Slider::new(&mut self.view.zoom, 3.0..=19.0).text("zoom"));
            let c = self.view.center_latlon();
            ui.monospace(format!("centre  {:.5}, {:.5}", c.lat, c.lon));
            match self.hover {
                Some((ll, Some(alt))) => {
                    ui.monospace(format!("curseur {:.5}, {:.5}", ll.lat, ll.lon));
                    ui.monospace(format!("altitude {alt:.0} m"));
                }
                Some((ll, None)) => {
                    ui.monospace(format!("curseur {:.5}, {:.5}", ll.lat, ll.lon));
                    ui.monospace("altitude …");
                }
                None => {
                    ui.monospace("curseur —");
                    ui.monospace("altitude —");
                }
            }

            ui.separator();
            ui.strong("Marche");
            let mut changed = false;
            changed |= ui
                .add(egui::Slider::new(&mut self.walk.flat_kmh, 2.0..=7.0).text("plat km/h"))
                .changed();
            changed |= ui
                .add(
                    egui::Slider::new(&mut self.walk.ascent_mh, 200.0..=1000.0)
                        .text("ascension m/h"),
                )
                .changed();
            changed |= ui
                .add(egui::Slider::new(&mut self.walk.body_weight_kg, 40.0..=120.0).text("poids kg"))
                .changed();
            changed |= ui
                .add(egui::Slider::new(&mut self.walk.pack_weight_kg, 0.0..=30.0).text("sac kg"))
                .changed();
            if changed {
                self.track.recompute_time(&self.walk);
                // Les coûts d'arête dépendent des mêmes réglages que le temps.
                self.iso_dirty = true;
            }
            ui.label(format!(
                "facteur vitesse {:.2} · seuil de charge {:.1} kg",
                self.walk.speed_factor(),
                self.walk.load_limit_kg()
            ));
            if self.walk.overloaded() {
                ui.colored_label(
                    Color32::from_rgb(200, 120, 0),
                    "⚠ sac au-delà de 20 % du poids corporel",
                );
            }

            ui.separator();
            ui.strong("Étape");
            let stats = *self.track.stats();
            ui.monospace(format!("distance  {:.2} km", stats.distance_m / 1000.0));
            ui.monospace(format!("D+        {:.0} m", stats.ascent_m));
            ui.monospace(format!("D−        {:.0} m", stats.descent_m));
            ui.monospace(format!("durée     {}", format_duration(stats.time_h)));
            let legs = self.track.waypoints.len().saturating_sub(1);
            if legs > 0 {
                let followed = self.track.followed_legs(&self.trails.net);
                ui.monospace(format!("liaisons  {followed}/{legs} sur sentier"));
            }
            if !self.track.waypoints.is_empty() && !stats.elevation_complete {
                ui.weak("MNT en cours de chargement — chiffres partiels");
            }
            ui.horizontal(|ui| {
                if ui.button("Effacer le tracé").clicked() {
                    self.track.clear();
                    self.iso_dirty = true;
                }
                if ui.button("Recentrer").clicked() {
                    if let Some(first) = self.track.waypoints.first() {
                        self.view = MapView::centered_on(first.pos, self.view.zoom);
                    }
                }
            });

            ui.separator();
            ui.strong("Profil altimétrique");
            self.paint_profile(ui);

            ui.separator();
            ui.checkbox(&mut self.show_debug, "Debug");
            if self.show_debug {
                let (r, p, f) = self.renderer.stats();
                ui.monospace(format!("tuiles  prêtes {r} · en cours {p} · échecs {f}"));
                ui.monospace(format!(
                    "peintes {} · replis parent {}",
                    self.renderer.painted, self.renderer.fallbacks
                ));
                let (dr, dp, df) = self.dem.stats();
                ui.monospace(format!("MNT     prêtes {dr} · en cours {dp} · échecs {df}"));
                ui.monospace(format!("échelle {:.1} m/px", self.view.meters_per_pixel()));
                ui.monospace(format!("points de trace {}", self.track.path().len()));
                if let Some(e) = &self.trails.last_error {
                    ui.colored_label(Color32::from_rgb(200, 90, 90), format!("Overpass : {e}"));
                }
            }
        });
    }

    fn paint_profile(&mut self, ui: &mut egui::Ui) {
        let height = 130.0;
        let (rect, _) = ui.allocate_exact_size(
            Vec2::new(ui.available_width(), height),
            egui::Sense::hover(),
        );
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 2.0, Color32::from_gray(24));

        let profile = self.track.profile();
        let stats = self.track.stats();
        let (Some(min), Some(max)) = (stats.min_elev_m, stats.max_elev_m) else {
            painter.text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                "pas encore de profil",
                egui::FontId::proportional(12.0),
                Color32::from_gray(140),
            );
            return;
        };
        let span = (max - min).max(1.0);
        let total = stats.distance_m.max(1.0);
        let pad = 6.0;
        let inner = rect.shrink(pad);

        let pts: Vec<Pos2> = profile
            .iter()
            .filter_map(|s| {
                let e = s.elev_m?;
                Some(Pos2::new(
                    inner.left() + inner.width() * (s.dist_m / total) as f32,
                    inner.bottom() - inner.height() * ((e - min) / span),
                ))
            })
            .collect();

        if pts.len() >= 2 {
            let mut poly = pts.clone();
            poly.push(Pos2::new(inner.right(), inner.bottom()));
            poly.push(Pos2::new(inner.left(), inner.bottom()));
            painter.add(egui::Shape::convex_polygon(
                poly,
                Color32::from_rgba_unmultiplied(80, 140, 200, 60),
                Stroke::NONE,
            ));
            painter.add(egui::Shape::line(
                pts,
                Stroke::new(2.0, Color32::from_rgb(120, 190, 255)),
            ));
        }
        painter.text(
            rect.left_top() + Vec2::new(4.0, 2.0),
            egui::Align2::LEFT_TOP,
            format!("{max:.0} m"),
            egui::FontId::monospace(11.0),
            Color32::from_gray(180),
        );
        painter.text(
            rect.left_bottom() + Vec2::new(4.0, -2.0),
            egui::Align2::LEFT_BOTTOM,
            format!("{min:.0} m"),
            egui::FontId::monospace(11.0),
            Color32::from_gray(180),
        );
        painter.text(
            rect.right_bottom() + Vec2::new(-4.0, -2.0),
            egui::Align2::RIGHT_BOTTOM,
            format!("{:.1} km", total / 1000.0),
            egui::FontId::monospace(11.0),
            Color32::from_gray(180),
        );
    }
}

impl TrackFinderApp {
    /// Charge les zones de sentiers couvrant la vue courante. Plafonné : une vue
    /// dézoomée couvre des centaines de zones, et Overpass est un service public
    /// à quotas.
    fn load_visible_zones(&mut self, ctx: &egui::Context) {
        const MAX_ZONES: usize = 12;
        let rect = self.last_map_rect;
        let sw = self.view.screen_to_latlon(rect.left_bottom(), rect);
        let ne = self.view.screen_to_latlon(rect.right_top(), rect);
        let step = crate::trails::ZONE_DEG;
        let mut count = 0;
        let mut lat = (sw.lat / step).floor() * step;
        while lat < ne.lat && count < MAX_ZONES {
            let mut lon = (sw.lon / step).floor() * step;
            while lon < ne.lon && count < MAX_ZONES {
                self.trails
                    .ensure(LatLon::new(lat + step / 2.0, lon + step / 2.0), ctx);
                lon += step;
                count += 1;
            }
            lat += step;
        }
        if count >= MAX_ZONES {
            self.status = Some(format!(
                "Vue trop large : {MAX_ZONES} zones chargées seulement, zoome davantage"
            ));
        }
    }
}

/// Le chemin complet d'un clic : demande de zone → attente → accrochage.
/// `cargo test -- --ignored`.
#[cfg(test)]
#[cfg(not(target_arch = "wasm32"))]
mod tests {
    use super::*;

    /// Overpass est un service public : il tombe, il limite, il expire. Un test
    /// réseau qui échoue là-dessus n'apprend rien — on le signale et on sort.
    /// La patience couvre la chaîne complète de bascule d'instance (3 × 20 s).
    const PATIENCE: u32 = 700;

    fn unavailable(app: &TrackFinderApp) -> bool {
        let (_, _, failed, _) = app.trails.stats();
        failed > 0
    }

    /// Fait tourner la boucle jusqu'à ce que le clic en attente soit résolu.
    /// Renvoie faux si Overpass n'a rien donné — au test de se déclarer ignoré.
    #[must_use]
    fn pump_until(app: &mut TrackFinderApp, ctx: &egui::Context) -> bool {
        for _ in 0..PATIENCE {
            app.trails.pump(ctx);
            app.resolve_pending_click(ctx);
            if app.pending_click.is_none() {
                return !unavailable(app);
            }
            std::thread::sleep(std::time::Duration::from_millis(100));
        }
        false
    }

    /// Pose un point comme le ferait un clic sur la carte.
    #[must_use]
    fn click(app: &mut TrackFinderApp, ll: LatLon, ctx: &egui::Context) -> bool {
        app.trails.ensure(ll, ctx);
        app.pending_click = Some(ll);
        pump_until(app, ctx)
    }

    macro_rules! skip_if_offline {
        ($ok:expr) => {
            if !$ok {
                eprintln!("Overpass indisponible — test ignoré");
                return;
            }
        };
    }

    // -----------------------------------------------------------------------
    // Mesure de performance, sans interface : même chemin que celui d'eframe
    // (`Context::run_ui`), tessellation comprise.
    // -----------------------------------------------------------------------

    /// Réseau synthétique : `count` chemins de 12 points autour de Chamonix,
    /// densité comparable à ce que rend Overpass sur une vallée habitée.
    fn fake_network(count: usize) -> Vec<crate::trails::Way> {
        (0..count)
            .map(|i| {
                let lat0 = 45.90 + (i % 60) as f64 * 0.0008;
                let lon0 = 6.84 + (i / 60) as f64 * 0.0012;
                let points: Vec<LatLon> = (0..12)
                    .map(|k| {
                        LatLon::new(
                            lat0 + k as f64 * 0.00006,
                            lon0 + k as f64 * 0.00008 + (k % 3) as f64 * 0.00002,
                        )
                    })
                    .collect();
                let bounds = points.iter().fold(
                    (f64::MAX, f64::MAX, f64::MIN, f64::MIN),
                    |b, p| (b.0.min(p.lat), b.1.min(p.lon), b.2.max(p.lat), b.3.max(p.lon)),
                );
                crate::trails::Way {
                    id: i as i64,
                    kind: crate::trails::WayKind::Path,
                    name: None,
                    nodes: Vec::new(),
                    points,
                    sac_scale: None,
                    bounds,
                }
            })
            .collect()
    }

    /// Joue `frames` frames en simulant un glissement de la carte, et renvoie
    /// (médiane, pire cas) du temps de frame, tessellation comprise.
    fn bench_pan(app: &mut TrackFinderApp, frames: usize) -> (f64, f64) {
        let ctx = egui::Context::default();
        let screen = Rect::from_min_size(Pos2::ZERO, Vec2::new(1600.0, 900.0));
        let mut times = Vec::with_capacity(frames);
        let mut pos = Pos2::new(500.0, 450.0);

        for f in 0..frames {
            pos.x += 3.0;
            pos.y += 1.0;
            let input = egui::RawInput {
                screen_rect: Some(screen),
                time: Some(f as f64 / 60.0),
                events: vec![
                    egui::Event::PointerMoved(pos),
                    egui::Event::PointerButton {
                        pos,
                        button: egui::PointerButton::Primary,
                        pressed: true,
                        modifiers: Default::default(),
                    },
                ],
                ..Default::default()
            };
            let start = std::time::Instant::now();
            let mut output = ctx.run_ui(input, |ui| app.ui_impl(ui));
            let _ = ctx.tessellate(output.shapes, output.pixels_per_point);
            // En debug, epaint refuse qu'on jette un delta de texture non appliqué.
            output.textures_delta.clear();
            times.push(start.elapsed().as_secs_f64() * 1000.0);
        }
        times.sort_by(|a, b| a.partial_cmp(b).unwrap());
        (times[times.len() / 2], *times.last().unwrap())
    }

    #[test]
    #[ignore = "perf"]
    fn cout_dune_frame() {
        let mut app = TrackFinderApp::new();
        app.snap_to_trail = false;
        let (med, worst) = bench_pan(&mut app, 60);
        println!("carte nue          : médiane {med:.2} ms · pire {worst:.2} ms");

        for way in fake_network(1500) {
            app.trails.net.insert(way);
        }
        let (med, worst) = bench_pan(&mut app, 60);
        println!("+ 1500 sentiers    : médiane {med:.2} ms · pire {worst:.2} ms");

        // Un tracé de ~15 km, comme une étape réelle.
        for i in 0..40 {
            app.track.push(Waypoint::free(LatLon::new(
                45.90 + i as f64 * 0.004,
                6.86 + i as f64 * 0.002,
            )));
        }
        let (med, worst) = bench_pan(&mut app, 60);
        println!("+ tracé 40 points  : médiane {med:.2} ms · pire {worst:.2} ms");
    }

    #[test]
    #[ignore = "réseau"]
    fn clic_sur_sentier_est_accroche() {
        let ctx = egui::Context::default();
        let mut app = TrackFinderApp::new();

        // Place du Triangle de l'Amitié, Chamonix : zone piétonne dense.
        let point = LatLon::new(45.9237, 6.8694);
        skip_if_offline!(click(&mut app, point, &ctx));

        let status = app.status.clone();
        assert_eq!(app.track.waypoints.len(), 1, "statut : {status:?}");
        let wp = app.track.waypoints[0].clone();
        let click = point;
        let snap = wp.snap.expect("doit être accroché");
        assert!(snap.dist_m <= SNAP_RADIUS_M);
        assert!(crate::geo::haversine_m(wp.pos, click) <= SNAP_RADIUS_M);
    }

    /// Fait tourner la boucle jusqu'à ce que le graphe soit construit et ses
    /// dénivelés lus dans le MNT (les deux arrivent de façon asynchrone).
    #[must_use]
    fn settle(app: &mut TrackFinderApp, ctx: &egui::Context) -> bool {
        for _ in 0..PATIENCE {
            app.trails.pump(ctx);
            app.dem.begin_frame();
            app.update_graph(ctx);
            if !app.graph.is_empty()
                && app.graph_elevated
                && app.trails.stats().1 == 0
                && app.iso.is_some()
            {
                return true;
            }
            std::thread::sleep(std::time::Duration::from_millis(100));
        }
        false
    }

    #[test]
    #[ignore = "réseau"]
    fn isochrone_puis_etape_sur_le_terrain() {
        let ctx = egui::Context::default();
        let mut app = TrackFinderApp::new();
        app.budget_h = 1.0;

        // Départ du sentier de la Flégère, au-dessus des Praz de Chamonix.
        let depart = LatLon::new(45.9328, 6.8846);
        skip_if_offline!(click(&mut app, depart, &ctx));
        assert_eq!(app.track.waypoints.len(), 1, "{:?}", app.status);

        // Puis le bassin autour : deux grandes zones suffisent pour 1 h de budget.
        app.trails.ensure_area(depart, 2500.0, 2, &ctx);
        app.graph_dirty = true;
        skip_if_offline!(settle(&mut app, &ctx));

        let iso = app.iso.as_ref().unwrap();
        assert!(
            iso.reach.reached_count() > 20,
            "isochrone trop petite : {} nœuds",
            iso.reach.reached_count()
        );
        assert!(!iso.edges.is_empty());

        // Une cible à mi-budget : ni sur place, ni en limite.
        let budget = iso.budget_ms;
        let target = (0..app.graph.node_count() as u32)
            .filter_map(|n| iso.reach.cost(n).map(|c| (c, n)))
            .filter(|(c, _)| *c > budget / 3 && *c < budget * 3 / 4)
            .max_by_key(|(c, _)| *c)
            .map(|(_, n)| app.graph.nodes[n as usize])
            .expect("un nœud à mi-budget");

        skip_if_offline!(click(&mut app, target, &ctx));
        assert_eq!(app.track.waypoints.len(), 2, "{:?}", app.status);

        let leg = app.track.waypoints[1].clone();
        let via = leg.via.expect("l'étape doit être routée par le graphe");
        assert!(via.len() > 4, "géométrie trop courte : {}", via.len());

        // Le trajet par les sentiers est forcément plus long que la ligne droite.
        let direct = crate::geo::haversine_m(app.track.waypoints[0].pos, leg.pos);
        let walked = crate::trails::path_length_m(&via);
        assert!(walked >= direct * 0.99, "walked {walked} < direct {direct}");
        assert!(walked > 100.0);

        app.track
            .refresh(&app.trails.net, &mut app.dem, &app.walk, &ctx);
        assert!(app.track.stats().distance_m > 100.0);
    }

    #[test]
    #[ignore = "réseau"]
    fn clic_hors_sentier_est_refuse() {
        let ctx = egui::Context::default();
        let mut app = TrackFinderApp::new();

        // Plein milieu du glacier des Bossons : aucun chemin cartographié.
        skip_if_offline!(click(&mut app, LatLon::new(45.8770, 6.8480), &ctx));

        let status = app.status.clone();
        assert!(app.track.waypoints.is_empty(), "statut : {status:?}");
        assert!(
            status.unwrap().contains("refusé"),
            "l'utilisateur doit savoir pourquoi rien ne s'est passé"
        );
    }
}

/// Isochrone calculée depuis le dernier point du tracé.
struct Isochrone {
    reach: Reach,
    /// Arêtes atteintes, avec leur coût d'accès.
    edges: Vec<(u32, CostMs)>,
    source: Snap,
    source_pos: EdgePos,
    budget_ms: CostMs,
}

impl TrackFinderApp {
    /// Reconstruit le graphe quand le réseau a bougé, complète les dénivelés au
    /// fil de l'arrivée des tuiles MNT, puis recalcule l'isochrone si besoin.
    fn update_graph(&mut self, ctx: &egui::Context) {
        if self.graph_dirty {
            self.graph = Graph::build(&self.trails.net);
            self.graph_dirty = false;
            self.graph_elevated = false;
            self.iso_dirty = true;
        }
        if !self.graph.is_empty() && !self.graph_elevated {
            self.graph_elevated = self.graph.update_elevations(&self.trails.net, &mut self.dem, ctx);
            if self.graph_elevated {
                // Les coûts changent une fois le dénivelé connu : l'isochrone
                // affichée jusque-là était optimiste.
                self.iso_dirty = true;
            }
        }
        if self.iso_dirty {
            self.iso_dirty = false;
            self.iso = self.compute_isochrone();
        }
    }

    fn compute_isochrone(&self) -> Option<Isochrone> {
        if !self.show_iso {
            return None;
        }
        let source = self.track.waypoints.last()?.snap?;
        let source_pos = self.graph.locate(&self.trails.net, &source)?;
        let budget_ms = (self.budget_h as f64 * 3_600_000.0) as CostMs;
        let reach = self.graph.explore(
            &self.graph.sources_from(&source_pos, &self.walk),
            budget_ms,
            &self.walk,
        );
        let edges = reach.reachable_edges(&self.graph);
        Some(Isochrone {
            reach,
            edges,
            source,
            source_pos,
            budget_ms,
        })
    }

    /// Itinéraire depuis le dernier point du tracé jusqu'au point accroché,
    /// s'il est dans l'isochrone.
    fn route_to(&self, target: &Snap) -> Option<Vec<LatLon>> {
        let iso = self.iso.as_ref()?;
        let target_pos = self.graph.locate(&self.trails.net, target)?;
        let r = graph::route(
            &self.graph,
            &self.trails.net,
            &iso.reach,
            (&iso.source, &iso.source_pos),
            (target, &target_pos),
            &self.walk,
        )?;
        (r.cost_ms <= iso.budget_ms).then_some(r.points)
    }

    /// Colore les tronçons atteignables — pas de polygone de zone : on ne marche
    /// que sur les sentiers, une tache pleine mentirait sur le terrain traversable.
    fn paint_isochrone(&self, painter: &egui::Painter, rect: Rect) {
        let Some(iso) = &self.iso else {
            return;
        };
        let sw = self.view.screen_to_latlon(rect.left_bottom(), rect);
        let ne = self.view.screen_to_latlon(rect.right_top(), rect);
        let view = (sw.lat, sw.lon, ne.lat, ne.lon);
        let budget = iso.budget_ms.max(1) as f32;

        for (edge_idx, cost) in &iso.edges {
            let edge = &self.graph.edges[*edge_idx as usize];
            let Some(way) = self.trails.net.way_by_id(edge.way_id) else {
                continue;
            };
            if !way.intersects(view) {
                continue;
            }
            // Vert au départ, orange en limite de budget.
            let t = (*cost as f32 / budget).clamp(0.0, 1.0);
            let color = Color32::from_rgb(
                (60.0 + 195.0 * t) as u8,
                (200.0 - 40.0 * t) as u8,
                (90.0 - 60.0 * t) as u8,
            );
            let pts: Vec<Pos2> = way.points[edge.from as usize..=edge.to as usize]
                .iter()
                .map(|ll| self.view.latlon_to_screen(*ll, rect))
                .collect();
            painter.add(egui::Shape::line(pts, Stroke::new(3.0, color)));
        }
    }
}

impl TrackFinderApp {
    /// Charge les sentiers du bassin que l'isochrone peut atteindre. Le rayon
    /// est majoré : à vol d'oiseau on ne fait jamais mieux que la vitesse sur
    /// plat, donc `budget × vitesse` borne l'isochrone par construction.
    fn load_isochrone_area(&mut self, ctx: &egui::Context) {
        const MAX_ZONES: usize = 9;
        let Some(center) = self.track.waypoints.last().map(|w| w.pos).or_else(|| {
            (self.last_map_rect != Rect::NOTHING).then(|| self.view.center_latlon())
        }) else {
            return;
        };
        let radius_m = self.budget_h as f64 * self.walk.flat_kmh * 1000.0;
        let asked = self.trails.ensure_area(center, radius_m, MAX_ZONES, ctx);
        self.status = Some(if asked == 0 {
            "Bassin déjà chargé".to_owned()
        } else {
            format!("{asked} zone(s) demandée(s) sur ~{:.0} km de rayon", radius_m / 1000.0)
        });
    }
}
