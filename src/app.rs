use std::rc::Rc;

use egui::{Color32, Pos2, Rect, Stroke, Vec2};

use crate::dem::DemStore;
use crate::geo::LatLon;
use crate::map::{self, MapView, TileRenderer};
use crate::tiles::{HttpTileSource, RasterLayer, TileSource};
use crate::track::{format_duration, Track, WalkSettings};

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
    hover: Option<(LatLon, Option<f32>)>,
    show_debug: bool,
}

impl TrackFinderApp {
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
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
            hover: None,
            show_debug: false,
        }
    }
}

impl eframe::App for TrackFinderApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        self.renderer.begin_frame(&ctx);
        self.dem.begin_frame();

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
        let response = map::interact(ui, rect, &mut self.view);

        if response.clicked() {
            if let Some(pos) = response.interact_pointer_pos() {
                self.track.push(self.view.screen_to_latlon(pos, rect));
            }
        }
        if response.secondary_clicked() {
            self.track.pop();
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

        self.track.refresh(&mut self.dem, &self.walk, ctx);
        self.paint_track(&painter, rect);
        self.paint_scale_bar(&painter, rect);
    }

    fn paint_track(&self, painter: &egui::Painter, rect: Rect) {
        let pts: Vec<Pos2> = self
            .track
            .points
            .iter()
            .map(|ll| self.view.latlon_to_screen(*ll, rect))
            .collect();
        if pts.len() >= 2 {
            painter.add(egui::Shape::line(
                pts.clone(),
                Stroke::new(4.0, Color32::from_rgb(220, 40, 40)),
            ));
        }
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
            if !self.track.points.is_empty() && !stats.elevation_complete {
                ui.weak("MNT en cours de chargement — chiffres partiels");
            }
            ui.horizontal(|ui| {
                if ui.button("Effacer le tracé").clicked() {
                    self.track.clear();
                }
                if ui.button("Recentrer").clicked() {
                    if let Some(first) = self.track.points.first() {
                        self.view = MapView::centered_on(*first, self.view.zoom);
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
