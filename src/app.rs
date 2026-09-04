use std::rc::Rc;

use egui::{Color32, Pos2, Rect, Stroke, Vec2};

use crate::dem::DemStore;
use crate::geo::LatLon;
use crate::graph::{self, CostMs, EdgePos, Graph, Reach};
use crate::map::{self, MapView, TileRenderer, MAX_TILE_REQUESTS};
use crate::terrain::{self, TerrainAnalysis};
use crate::tiles::{HttpTileSource, RasterLayer, TileSource};
use crate::track::{format_duration, Track, WalkSettings, Waypoint};
use crate::trails::{Snap, TrailStore, ZoneKey, SNAP_RADIUS_M};

/// The French Alps, from Lake Geneva down to the Mercantour.
///
/// The application opens on the whole range rather than on one valley: you pick
/// the massif first, then zoom in. Deliberately below `AUTO_TRAILS_MIN_ZOOM`, so
/// simply opening the page asks Overpass for nothing.
const ALPS_SW: LatLon = LatLon::new(43.95, 5.35);
const ALPS_NE: LatLon = LatLon::new(46.45, 7.75);

/// The opaque ground layers, exactly one of which is drawn. Stacking two of them
/// only ever shows the top one while paying to fetch and decode both.
const BASE_MAPS: [RasterLayer; 2] = [RasterLayer::PlanIgn, RasterLayer::Ortho];

/// Below this zoom the OSM network is not drawn: at 1:200000 thousands of
/// polylines collapse into an unreadable smear that costs milliseconds a frame.
const TRAILS_MIN_ZOOM: u8 = 12;

/// Below this zoom trails are not auto-loaded either. A wide view spans hundreds
/// of Overpass zones, and Overpass is a public, quota-limited service.
const AUTO_TRAILS_MIN_ZOOM: u8 = 13;

/// Ceiling on the zones one view may request at once, automatically or not.
const MAX_VISIBLE_ZONES: usize = 12;

/// New basin zones requested per settled frame. Overpass grants two slots; more
/// than a trickle only builds a queue.
const BASIN_STEP: usize = 2;

/// Hard ceiling on the zones one basin may ever request. A 10 h budget would
/// otherwise sweep sixty coarse zones off a public service.
const MAX_BASIN_ZONES: usize = 16;

/// Bootstrap radius before any isochrone exists, in metres — enough to give the
/// graph something to expand into.
const BASIN_BOOTSTRAP_M: f64 = 5_000.0;

/// Margin added beyond the reach the isochrone actually achieved, so the next
/// ring of trails can extend it. One coarse zone.
const BASIN_MARGIN_M: f64 = 11_000.0;

/// Squared screen distance below which a shape point is dropped when drawing the
/// network. Two points inside the same pixel draw the same line twice.
const MIN_SEGMENT_PX2: f32 = 4.0;

/// A transparent layer composited over the base map.
struct OverlaySetting {
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
    /// The single ground layer. Always one of `BASE_MAPS`.
    base: RasterLayer,
    overlays: Vec<OverlaySetting>,
    trails: TrailStore,
    graph: Graph,
    /// The network changed: the graph has to be rebuilt.
    graph_dirty: bool,
    /// Every edge climb has been read from the DEM.
    graph_elevated: bool,
    iso: Option<Isochrone>,
    iso_dirty: bool,
    /// Leg budget, in hours. This is a here-and-now setting, not a profile one:
    /// a single source of truth.
    budget_h: f32,
    show_iso: bool,
    /// A click waiting for the Overpass zone that will let it snap.
    pending_click: Option<LatLon>,
    snap_to_trail: bool,
    show_trails: bool,
    /// Load the visible trails on their own once the view stops moving.
    auto_load_trails: bool,
    /// Last (centre zone, zoom) already scanned for auto-loading — panning
    /// inside one zone must not rescan on every settled frame. Only used while
    /// no point is placed; after that the basin takes over.
    last_auto_scan: Option<(ZoneKey, u8)>,
    /// Progressive loading of the basin around the last point.
    basin: Option<Basin>,
    hover: Option<(LatLon, Option<f32>)>,
    status: Option<String>,
    /// The bivouac candidate: always the last point of the track.
    bivouac: Option<Bivouac>,
    /// Last painted map area — used by the "load visible trails" button.
    last_map_rect: Rect,
    /// Fit the view to the Alps on the first frame. The zoom that shows a given
    /// area depends on the window, so it cannot be decided in `new`.
    fit_alps: bool,
    show_debug: bool,
}

impl Default for TrackFinderApp {
    fn default() -> Self {
        Self::new()
    }
}

impl TrackFinderApp {
    pub fn new() -> Self {
        // One shared source: an offline cache slots in here tomorrow without the
        // renderer or the DEM changing a line.
        Self::with_source(Rc::new(HttpTileSource::default()))
    }

    /// Same application over any tile source. The benchmarks use it to measure
    /// the render path without the network in the way.
    pub fn with_source(source: Rc<dyn TileSource>) -> Self {
        Self {
            view: MapView::centered_on(
                LatLon::new(
                    (ALPS_SW.lat + ALPS_NE.lat) / 2.0,
                    (ALPS_SW.lon + ALPS_NE.lon) / 2.0,
                ),
                8.0,
            ),
            renderer: TileRenderer::new(Rc::clone(&source)),
            dem: DemStore::new(source),
            track: Track::default(),
            walk: WalkSettings::default(),
            base: RasterLayer::PlanIgn,
            overlays: vec![
                OverlaySetting {
                    layer: RasterLayer::Slopes,
                    enabled: false,
                    opacity: 0.45,
                },
                OverlaySetting {
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
            auto_load_trails: true,
            last_auto_scan: None,
            basin: None,
            hover: None,
            status: None,
            bivouac: None,
            last_map_rect: Rect::NOTHING,
            fit_alps: true,
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
    /// The body of a frame, without `eframe::Frame`: drivable headlessly
    /// (performance measurements, tests).
    fn ui_impl(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();
        self.renderer.begin_frame(&ctx);
        self.dem.begin_frame(&ctx);
        if self.trails.pump(&ctx) {
            // New trails may complete legs that are already placed.
            self.track.invalidate();
            self.graph_dirty = true;
        }
        self.update_graph(&ctx);
        self.resolve_pending_click(&ctx);
        self.update_bivouac(&ctx);

        egui::Panel::right("panel")
            .default_size(320.0)
            .show(ui, |ui| self.side_panel(ui));

        self.map_area(ui, &ctx);
        // After the map, so the view rectangle of this very frame is known.
        self.update_auto_trails(&ctx);

        self.renderer.end_frame();
        self.dem.end_frame();
    }
}

impl TrackFinderApp {
    // -----------------------------------------------------------------------
    // Map
    // -----------------------------------------------------------------------
    fn map_area(&mut self, ui: &mut egui::Ui, ctx: &egui::Context) {
        let rect = ui.available_rect_before_wrap();
        if rect.width() < 1.0 || rect.height() < 1.0 {
            return;
        }
        self.last_map_rect = rect;
        if self.fit_alps {
            self.fit_alps = false;
            self.view.fit_bounds(ALPS_SW, ALPS_NE, rect);
        }
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

        // Ground first, transparent overlays on top.
        debug_assert!(
            self.base.is_base(),
            "the base map must be an opaque ground layer"
        );
        self.renderer
            .paint_layer(&painter, rect, &self.view, self.base, 1.0, ctx);
        for setting in &self.overlays {
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
        self.paint_bivouac(&painter, rect);
        self.paint_scale_bar(&painter, rect);
    }

    /// The loaded OSM network, drawn under everything else: it is the cue for
    /// where one may usefully click.
    ///
    /// Shape points closer than a couple of pixels are dropped. An OSM way holds
    /// a point every few metres; zoomed out that is dozens of vertices inside one
    /// pixel, all of them tessellated for nothing.
    fn paint_trails(&self, painter: &egui::Painter, rect: Rect) {
        if self.view.tile_zoom() < TRAILS_MIN_ZOOM {
            return;
        }
        let sw = self.view.screen_to_latlon(rect.left_bottom(), rect);
        let ne = self.view.screen_to_latlon(rect.right_top(), rect);
        let view = (sw.lat, sw.lon, ne.lat, ne.lon);
        let mut pts: Vec<Pos2> = Vec::with_capacity(64);
        for way in self.trails.net.ways() {
            if !way.intersects(view) {
                continue;
            }
            pts.clear();
            let last = way.points.len() - 1;
            for (i, ll) in way.points.iter().enumerate() {
                let p = self.view.latlon_to_screen(*ll, rect);
                // The final vertex is always kept, otherwise ways would come up
                // visibly short of their junctions.
                let keep = i == last
                    || pts
                        .last()
                        .is_none_or(|prev| (p - *prev).length_sq() >= MIN_SEGMENT_PX2);
                if keep {
                    pts.push(p);
                }
            }
            if pts.len() >= 2 {
                painter.add(egui::Shape::line(
                    pts.clone(),
                    Stroke::new(1.5, way.kind.color().gamma_multiply(0.75)),
                ));
            }
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
        // The "round" length nearest to 100 px.
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

    fn resolve_pending_click(&mut self, ctx: &egui::Context) {
        let Some(ll) = self.pending_click else {
            return;
        };
        if let Some(err) = self.trails.zone_failed(ll) {
            self.status = Some(format!("Trails unavailable: {err}"));
            self.pending_click = None;
            return;
        }
        if !self.trails.zone_ready(ll) {
            return;
        }
        self.pending_click = None;
        let Some(snap) = self.trails.net.snap(ll, SNAP_RADIUS_M) else {
            self.status = Some(format!(
                "No trail within {SNAP_RADIUS_M:.0} m — point refused"
            ));
            return;
        };
        // First point: there is nothing to join yet. After that, prefer a route
        // through the graph and fall back to segment following.
        if self.track.waypoints.is_empty() {
            self.status = Some(format!("Start snapped {:.0} m from the click", snap.dist_m));
            self.track.push(Waypoint::snapped(snap));
        } else if let Some(via) = self.route_to(&snap) {
            let km = crate::trails::path_length_m(&via) / 1000.0;
            self.status = Some(format!("Leg drawn along trails: {km:.2} km"));
            self.track.push(Waypoint::routed(snap, via));
        } else {
            self.status =
                Some("Outside the isochrone (or incomplete graph) — direct link".to_owned());
            self.track.push(Waypoint::snapped(snap));
        }
        self.iso_dirty = true;
        ctx.request_repaint();
    }

    // -----------------------------------------------------------------------
    // Panel
    // -----------------------------------------------------------------------
    fn side_panel(&mut self, ui: &mut egui::Ui) {
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.heading("TrackFinder");
            ui.label("Left click: add a point · right click: remove the last one");
            ui.separator();

            let z = self.view.tile_zoom();
            ui.strong("Base map");
            for layer in BASE_MAPS {
                ui.horizontal(|ui| {
                    ui.radio_value(&mut self.base, layer, layer.label());
                    let (_, max_z) = layer.zoom_range();
                    if z > max_z {
                        ui.weak(format!("(<= z{max_z})"));
                    }
                });
            }

            ui.add_space(4.0);
            ui.strong("Overlays");
            for setting in &mut self.overlays {
                ui.horizontal(|ui| {
                    ui.checkbox(&mut setting.enabled, setting.layer.label());
                    let (_, max_z) = setting.layer.zoom_range();
                    if z > max_z {
                        ui.weak(format!("(<= z{max_z})"));
                    }
                });
                if setting.enabled {
                    ui.add(
                        egui::Slider::new(&mut setting.opacity, 0.0..=1.0)
                            .text("opacity")
                            .show_value(false),
                    );
                }
            }

            ui.separator();
            ui.strong("Trails (OpenStreetMap)");
            ui.checkbox(&mut self.show_trails, "Show the network");
            ui.checkbox(&mut self.snap_to_trail, "Snap points to trails");
            ui.checkbox(&mut self.auto_load_trails, "Load them automatically");
            let (zones, pending, failed, ways) = self.trails.stats();
            ui.monospace(format!("{zones} zones · {ways} ways"));
            if let Some((text, _)) = self.basin_status() {
                ui.monospace(text);
            }
            if let Some(hint) = self.auto_trails_hint() {
                ui.weak(hint);
            }
            if pending > 0 {
                ui.horizontal(|ui| {
                    ui.spinner();
                    ui.label(format!("{pending} zone(s) loading"));
                });
            }
            if failed > 0 {
                ui.colored_label(
                    Color32::from_rgb(200, 90, 90),
                    format!("{failed} zone(s) failed"),
                );
                if ui.button("Retry").clicked() {
                    let ctx = ui.ctx().clone();
                    self.trails.retry_failed(&ctx);
                }
            }
            if ui.button("Load the trails in view").clicked() {
                let ctx = ui.ctx().clone();
                self.load_visible_zones(&ctx, true);
            }
            if let Some(status) = &self.status {
                ui.weak(status);
            }

            ui.separator();
            ui.strong("Isochrone");
            if ui
                .checkbox(&mut self.show_iso, "From the last point")
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
                ui.weak("Empty graph — place a point or load some trails");
            } else {
                ui.monospace(format!(
                    "graph     {} nodes · {} edges",
                    self.graph.node_count(),
                    self.graph.edges.len()
                ));
                if !self.graph_elevated {
                    ui.weak("reading climbs from the DEM");
                }
            }
            match &self.iso {
                Some(iso) => {
                    ui.monospace(format!(
                        "reached   {} nodes · {} stretches",
                        iso.reach.reached_count(),
                        iso.edges.len()
                    ));
                    ui.monospace(format!("furthest  {:.1} km away", iso.reach_m / 1000.0));
                    // An isochrone computed on a partial basin does not look
                    // partial: it stops at the edge of the data and reads as
                    // "not reachable". Say so rather than let it mislead.
                    if self.basin_status().is_some_and(|(_, partial)| partial) {
                        ui.colored_label(
                            Color32::from_rgb(200, 150, 60),
                            "! trails still loading — reach shown is a lower bound",
                        );
                    }
                }
                None if !self.track.waypoints.is_empty() && self.show_iso => {
                    ui.weak("last point is off the graph");
                }
                None => {}
            }
            if ui
                .button("Load the isochrone basin")
                .on_hover_text("Large Overpass zones around the last point")
                .clicked()
            {
                let ctx = ui.ctx().clone();
                self.load_isochrone_area(&ctx);
            }

            ui.separator();
            ui.strong("View");
            if ui
                .add(egui::Slider::new(&mut self.view.zoom, 3.0..=19.0).text("zoom"))
                .changed()
            {
                self.view.touch();
            }
            let c = self.view.center_latlon();
            ui.monospace(format!("centre  {:.5}, {:.5}", c.lat, c.lon));
            match self.hover {
                Some((ll, Some(alt))) => {
                    ui.monospace(format!("cursor  {:.5}, {:.5}", ll.lat, ll.lon));
                    ui.monospace(format!("elev    {alt:.0} m"));
                }
                Some((ll, None)) => {
                    ui.monospace(format!("cursor  {:.5}, {:.5}", ll.lat, ll.lon));
                    ui.monospace("elev    ...");
                }
                None => {
                    ui.monospace("cursor  —");
                    ui.monospace("elev    —");
                }
            }

            ui.separator();
            ui.strong("Walking");
            let mut changed = false;
            changed |= ui
                .add(egui::Slider::new(&mut self.walk.flat_kmh, 2.0..=7.0).text("flat km/h"))
                .changed();
            changed |= ui
                .add(
                    egui::Slider::new(&mut self.walk.ascent_mh, 200.0..=1000.0)
                        .text("ascent m/h"),
                )
                .changed();
            changed |= ui
                .add(
                    egui::Slider::new(&mut self.walk.body_weight_kg, 40.0..=120.0)
                        .text("body kg"),
                )
                .changed();
            changed |= ui
                .add(egui::Slider::new(&mut self.walk.pack_weight_kg, 0.0..=30.0).text("pack kg"))
                .changed();
            if changed {
                self.track.recompute_time(&self.walk);
                // Edge costs depend on the same settings as the time.
                self.iso_dirty = true;
            }
            ui.label(format!(
                "speed factor {:.2} · load limit {:.1} kg",
                self.walk.speed_factor(),
                self.walk.load_limit_kg()
            ));
            if self.walk.overloaded() {
                ui.colored_label(
                    Color32::from_rgb(200, 120, 0),
                    "! pack over 20% of body weight",
                );
            }

            ui.separator();
            ui.strong("Leg");
            let stats = *self.track.stats();
            ui.monospace(format!("distance  {:.2} km", stats.distance_m / 1000.0));
            ui.monospace(format!("ascent    {:.0} m", stats.ascent_m));
            ui.monospace(format!("descent   {:.0} m", stats.descent_m));
            ui.monospace(format!("duration  {}", format_duration(stats.time_h)));
            let legs = self.track.waypoints.len().saturating_sub(1);
            if legs > 0 {
                let followed = self.track.followed_legs(&self.trails.net);
                ui.monospace(format!("on trail  {followed}/{legs} legs"));
            }
            if !self.track.waypoints.is_empty() && !stats.elevation_complete {
                ui.weak("DEM still loading — figures are partial");
            }
            ui.horizontal(|ui| {
                if ui.button("Clear track").clicked() {
                    self.track.clear();
                    self.iso_dirty = true;
                }
                if ui.button("Recentre").clicked() {
                    if let Some(first) = self.track.waypoints.first() {
                        self.view = MapView::centered_on(first.pos, self.view.zoom);
                    }
                }
            });

            ui.separator();
            ui.strong("Bivouac");
            self.bivouac_panel(ui);

            ui.separator();
            ui.strong("Elevation profile");
            self.paint_profile(ui);

            ui.separator();
            ui.checkbox(&mut self.show_debug, "Debug");
            if self.show_debug {
                let (r, p, f) = self.renderer.stats();
                ui.monospace(format!(
                    "tiles   ready {r} · loading {p} · failed {f} · on wire {}/{}",
                    self.renderer.in_flight(),
                    MAX_TILE_REQUESTS
                ));
                ui.monospace(format!(
                    "painted {} · parent fallbacks {}",
                    self.renderer.painted, self.renderer.fallbacks
                ));
                let (dr, dp, df) = self.dem.stats();
                ui.monospace(format!(
                    "DEM     ready {dr} · loading {dp} · failed {df} · on wire {}",
                    self.dem.in_flight()
                ));
                ui.monospace(format!("scale   {:.1} m/px", self.view.meters_per_pixel()));
                ui.monospace(format!("track points {}", self.track.path().len()));
                if let Some(url) = self.trails.preferred_endpoint() {
                    // First thing to look at when trail loading drags: a dead
                    // primary is invisible otherwise.
                    let host = url
                        .trim_start_matches("https://")
                        .split('/')
                        .next()
                        .unwrap_or(&url)
                        .to_owned();
                    ui.monospace(format!("trails  via {host}"));
                }
                if let Some(e) = &self.trails.last_error {
                    ui.colored_label(Color32::from_rgb(200, 90, 90), format!("Overpass: {e}"));
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
                "no profile yet",
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

// ---------------------------------------------------------------------------
// Trail loading
// ---------------------------------------------------------------------------

impl TrackFinderApp {
    /// Why automatic loading is currently doing nothing, if it is not.
    fn auto_trails_hint(&self) -> Option<&'static str> {
        if !self.auto_load_trails {
            return None;
        }
        // Once a point is placed the basin drives loading and the view zoom no
        // longer gates anything.
        if !self.track.waypoints.is_empty() {
            return None;
        }
        (self.view.tile_zoom() < AUTO_TRAILS_MIN_ZOOM)
            .then_some("zoom in to z13 to load trails automatically")
    }

    /// Loads trails on its own once the view has stopped moving.
    ///
    /// **The anchor is the last point of the track, not the viewport.** What the
    /// application needs is the network the isochrone can reach from where you
    /// stand; the visible rectangle is a poor proxy for that — you can pan away
    /// to read the map and load zones you will never route through, while the
    /// basin you actually depend on stays half loaded. Worse, a half-loaded basin
    /// makes the isochrone *wrong without saying so*: it stops at the edge of the
    /// data and reads as "not reachable in 3 h".
    ///
    /// Before the first point there is no anchor, so the viewport is all we have
    /// — and there the network is only a cue for where one may click.
    ///
    /// The view has to be **settled** either way: Overpass allows two requests at
    /// a time, and a drag would otherwise queue a zone per frame.
    fn update_auto_trails(&mut self, ctx: &egui::Context) {
        if !self.auto_load_trails
            || self.last_map_rect == Rect::NOTHING
            || !self.view.view_settled()
        {
            return;
        }
        match self.track.waypoints.last().map(|wp| wp.pos) {
            Some(anchor) => self.grow_basin(anchor, ctx),
            None => self.scan_visible_zones(ctx),
        }
    }

    /// Viewport scan, used only while no point is placed.
    ///
    /// Keyed on the centre zone plus the zoom, so panning inside one zone does
    /// not re-walk the grid sixty times a second for an answer that cannot have
    /// changed.
    fn scan_visible_zones(&mut self, ctx: &egui::Context) {
        let z = self.view.tile_zoom();
        if z < AUTO_TRAILS_MIN_ZOOM {
            return;
        }
        let key = (ZoneKey::of(self.view.center_latlon()), z);
        if self.last_auto_scan == Some(key) {
            return;
        }
        self.last_auto_scan = Some(key);
        self.load_visible_zones(ctx, false);
    }

    /// Grows the trail basin around `anchor`, a couple of zones at a time, until
    /// the isochrone stops growing.
    ///
    /// The loop is: load a ring → the graph reaches further → the radius derived
    /// from that reach grows → load the next ring. It converges on its own, and
    /// on ground where the isochrone is bounded by climb rather than by data it
    /// converges quickly — which is most of the Alps.
    ///
    /// Nothing is asked while zones are still in flight: the answer to "did that
    /// help?" only exists once they have landed.
    fn grow_basin(&mut self, anchor: LatLon, ctx: &egui::Context) {
        if self.basin.as_ref().is_none_or(|b| b.anchor != anchor) {
            self.basin = Some(Basin::new(anchor));
        }
        let Some(basin) = self.basin.as_ref() else {
            return;
        };
        // Done deciding: the counters were made accurate before `complete` was
        // set, so there is nothing left to recompute every frame.
        if basin.complete || basin.asked >= MAX_BASIN_ZONES {
            return;
        }
        // Zones already on the wire: wait for them before deciding anything.
        if self.trails.stats().1 > 0 {
            return;
        }

        // Measure first, decide second. Refreshing the readout before any early
        // return is what keeps the panel from freezing on stale figures at the
        // exact moment the basin settles.
        let reach_m = self.iso.as_ref().map(|i| i.reach_m).unwrap_or(0.0);
        let theoretical = self.budget_h as f64 * self.walk.flat_kmh * 1000.0;
        let radius = (reach_m + BASIN_MARGIN_M)
            .max(BASIN_BOOTSTRAP_M)
            .min(theoretical.max(BASIN_BOOTSTRAP_M));
        let keys = self.trails.area_zones(anchor, radius);
        let loaded = self.trails.zones_loaded(&keys);

        let Some(basin) = self.basin.as_mut() else {
            return;
        };
        basin.radius_m = radius;
        basin.zones = keys.len();
        basin.loaded = loaded;

        // Converged: the last batch landed and the isochrone did not get any
        // further. More data would not change the answer.
        if basin.asked > 0 && reach_m <= basin.reach_at_last_ask + 1.0 {
            basin.complete = true;
            return;
        }
        basin.reach_at_last_ask = reach_m;
        let budget = BASIN_STEP.min(MAX_BASIN_ZONES - basin.asked);
        let asked = self.trails.ensure_some(&keys, budget, ctx);

        let Some(basin) = self.basin.as_mut() else {
            return;
        };
        if asked == 0 {
            // Everything within the radius is in memory and the isochrone has
            // nothing left to eat.
            basin.complete = true;
        } else {
            basin.asked += asked;
        }
    }

    /// What to tell the user about basin coverage, if anything.
    ///
    /// An isochrone computed on a partial basin is not merely incomplete, it is
    /// misleading — so this is said plainly rather than left to a spinner.
    fn basin_status(&self) -> Option<(String, bool)> {
        let basin = self.basin.as_ref()?;
        // "Partial" means the basin is still working — **not** that every
        // candidate zone is loaded. It stops as soon as more data stops moving
        // the isochrone, which normally happens well before the outer ring is
        // fetched: those zones were candidates, never requirements.
        let text = format!(
            "basin {} zones · {:.0} km around the last point{}",
            basin.loaded,
            basin.radius_m / 1000.0,
            if basin.complete { "" } else { " · growing" }
        );
        Some((text, !basin.complete))
    }

    /// Requests the trail zones covering the current view, capped: a zoomed-out
    /// view spans hundreds of zones and Overpass is a public quota-limited
    /// service.
    ///
    /// `announce` writes to the status line — the button says what it did, the
    /// automatic path stays silent.
    fn load_visible_zones(&mut self, ctx: &egui::Context, announce: bool) {
        let rect = self.last_map_rect;
        if rect == Rect::NOTHING {
            return;
        }
        let sw = self.view.screen_to_latlon(rect.left_bottom(), rect);
        let ne = self.view.screen_to_latlon(rect.right_top(), rect);
        let center = self.view.center_latlon();
        let keys = self
            .trails
            .zones_covering(sw.lat, sw.lon, ne.lat, ne.lon, center);
        let total = keys.len();
        let asked = self.trails.ensure_zones(&keys, MAX_VISIBLE_ZONES, ctx);
        if announce {
            self.status = Some(if total > MAX_VISIBLE_ZONES {
                format!(
                    "View too wide: {MAX_VISIBLE_ZONES} of {total} zones requested, zoom in for the rest"
                )
            } else if asked == 0 {
                "Visible trails already loaded".to_owned()
            } else {
                format!("{asked} zone(s) requested")
            });
        }
    }

    /// Loads the trails of the basin the isochrone can reach. The radius is an
    /// upper bound: as the crow flies you never beat the flat speed, so
    /// `budget × speed` bounds the isochrone by construction.
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
            "Basin already loaded".to_owned()
        } else {
            format!(
                "{asked} zone(s) requested over a ~{:.0} km radius",
                radius_m / 1000.0
            )
        });
    }
}

/// Progressive loading of the trail basin around the last point of the track.
struct Basin {
    anchor: LatLon,
    /// Zones actually requested for this anchor, against `MAX_BASIN_ZONES`.
    asked: usize,
    /// Zones the current radius covers, and how many are in memory.
    zones: usize,
    loaded: usize,
    radius_m: f64,
    /// Isochrone reach when zones were last requested — the growth test.
    reach_at_last_ask: f64,
    /// The isochrone stopped growing, or everything in radius is loaded.
    complete: bool,
}

impl Basin {
    fn new(anchor: LatLon) -> Self {
        Self {
            anchor,
            asked: 0,
            zones: 0,
            loaded: 0,
            radius_m: 0.0,
            reach_at_last_ask: 0.0,
            complete: false,
        }
    }
}

// ---------------------------------------------------------------------------
// Graph and isochrone
// ---------------------------------------------------------------------------

/// Isochrone computed from the last point of the track.
struct Isochrone {
    reach: Reach,
    /// Reached edges, with their access cost.
    edges: Vec<(u32, CostMs)>,
    source: Snap,
    source_pos: EdgePos,
    budget_ms: CostMs,
    /// Straight-line distance from the source to the furthest reached node.
    ///
    /// Far smaller than `budget × flat speed` in the mountains, where climb and
    /// winding trails eat most of the budget — which is exactly why the basin is
    /// grown from this rather than from the theoretical maximum.
    reach_m: f64,
}

impl TrackFinderApp {
    /// Rebuilds the graph when the network moved, fills in climbs as DEM tiles
    /// arrive, then recomputes the isochrone if needed.
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
                // Costs change once the climb is known: the isochrone shown until
                // now was optimistic.
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
        let reach_m = (0..self.graph.node_count() as u32)
            .filter(|n| reach.cost(*n).is_some())
            .map(|n| crate::geo::haversine_m(source.pos, self.graph.nodes[n as usize]))
            .fold(0.0f64, f64::max);
        Some(Isochrone {
            reach,
            edges,
            source,
            source_pos,
            budget_ms,
            reach_m,
        })
    }

    /// Route from the last point of the track to a snapped point, when that
    /// point is inside the isochrone.
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

    /// Colours the reachable stretches — no area polygon: you only walk on
    /// trails, and a filled blob would lie about which ground can be crossed.
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
            // Green near the start, orange at the edge of the budget.
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

// ---------------------------------------------------------------------------
// Bivouac candidate analysis
// ---------------------------------------------------------------------------

/// The bivouac candidate and its terrain analysis.
///
/// The analysis is stored together with the point it belongs to: without that,
/// extending the track would keep showing the previous point's figures without
/// saying so.
struct Bivouac {
    at: LatLon,
    /// `None` until the DEM has delivered everything. Frozen once filled — same
    /// logic as the elevation profile.
    analysis: Option<TerrainAnalysis>,
}

/// Red -> amber -> green ramp for a likelihood in `0..=1`.
fn chance_color(chance: f32) -> Color32 {
    const RED: [f32; 3] = [200.0, 62.0, 52.0];
    const AMBER: [f32; 3] = [214.0, 158.0, 46.0];
    const GREEN: [f32; 3] = [72.0, 184.0, 96.0];
    let c = chance.clamp(0.0, 1.0);
    let (from, to, t) = if c < 0.5 {
        (RED, AMBER, c * 2.0)
    } else {
        (AMBER, GREEN, (c - 0.5) * 2.0)
    };
    Color32::from_rgb(
        (from[0] + (to[0] - from[0]) * t) as u8,
        (from[1] + (to[1] - from[1]) * t) as u8,
        (from[2] + (to[2] - from[2]) * t) as u8,
    )
}

impl TrackFinderApp {
    /// The last point of the track is a bivouac candidate by default (no
    /// question asked on click, that would break the flow). Recomputes while the
    /// DEM is incomplete; the tile cache's `request_repaint` brings the next
    /// frame.
    fn update_bivouac(&mut self, ctx: &egui::Context) {
        let Some(at) = self.track.waypoints.last().map(|wp| wp.pos) else {
            self.bivouac = None;
            return;
        };
        if self.bivouac.as_ref().is_none_or(|b| b.at != at) {
            self.bivouac = Some(Bivouac {
                at,
                analysis: None,
            });
        }
        let Some(spot) = self.bivouac.as_mut() else {
            return;
        };
        if spot.analysis.is_some() {
            return;
        }
        // `dem` pulled out before the closure: otherwise `self` is borrowed twice.
        let dem = &mut self.dem;
        spot.analysis = terrain::analyze(at, |ll, z| dem.elevation_at(ll, z, ctx));
    }

    fn bivouac_panel(&mut self, ui: &mut egui::Ui) {
        let Some(spot) = &self.bivouac else {
            ui.weak("Place a point: it becomes the bivouac candidate.");
            return;
        };
        let Some(a) = spot.analysis else {
            ui.horizontal(|ui| {
                ui.spinner();
                ui.label("reading the DEM...");
            });
            return;
        };

        let chance = terrain::flat_chance(a.slope_deg, a.roughness_m);
        ui.label("Chance of flat ground");
        chance_ramp(ui, chance);
        ui.weak(
            "A 5 m DEM cannot see a two-metre terrace. This points at where to \
             look, it is not a verdict.",
        );

        ui.add_space(6.0);
        ui.monospace(format!("elevation  {:.0} m", a.elevation_m));
        ui.monospace(format!("slope      {:.1} deg", a.slope_deg));
        ui.monospace(format!("roughness  {:.1} m", a.roughness_m));

        match a.aspect_deg {
            Some(aspect) => {
                ui.monospace(format!(
                    "aspect     {} ({aspect:.0} deg)",
                    terrain::cardinal(aspect)
                ));
                ui.weak(if terrain::sunny(aspect) {
                    "in the sun for much of the day"
                } else {
                    "cold aspect, dries slowly and holds snow late"
                });
            }
            None => {
                ui.monospace("aspect     — (flat ground)");
            }
        }

        let pos = terrain::position(a.tpi_m);
        ui.monospace(format!("position   {:+.0} m ({})", a.tpi_m, pos.label()));
        ui.weak(pos.hint());
    }

    /// Ring around the bivouac candidate, coloured by the same red-to-green ramp
    /// as the panel. It doubles the last waypoint's marker rather than replacing
    /// it: one glance should give both where the leg ends and what the spot is
    /// worth.
    fn paint_bivouac(&self, painter: &egui::Painter, rect: Rect) {
        let Some(spot) = &self.bivouac else {
            return;
        };
        let Some(a) = spot.analysis else {
            return;
        };
        let p = self.view.latlon_to_screen(spot.at, rect);
        let color = chance_color(terrain::flat_chance(a.slope_deg, a.roughness_m));
        painter.circle_stroke(p, 11.0, Stroke::new(2.5, color));
    }
}

/// Continuous red-to-green scale with a marker at `chance`.
///
/// Deliberately not a percentage and not a label: the underlying number carries
/// far less precision than either would imply. A position on a gradient says
/// "promising" or "forget it" without pretending to a figure.
fn chance_ramp(ui: &mut egui::Ui, chance: f32) {
    const SLICES: usize = 48;
    let (rect, _) = ui.allocate_exact_size(
        Vec2::new(ui.available_width(), 16.0),
        egui::Sense::hover(),
    );
    let painter = ui.painter_at(rect);
    let w = rect.width() / SLICES as f32;
    for i in 0..SLICES {
        let t = i as f32 / (SLICES - 1) as f32;
        let slice = Rect::from_min_size(
            Pos2::new(rect.left() + i as f32 * w, rect.top()),
            Vec2::new(w + 1.0, rect.height()),
        );
        painter.rect_filled(slice, 0.0, chance_color(t).gamma_multiply(0.55));
    }
    let x = rect.left() + rect.width() * chance.clamp(0.0, 1.0);
    painter.line_segment(
        [Pos2::new(x, rect.top() - 2.0), Pos2::new(x, rect.bottom() + 2.0)],
        Stroke::new(2.5, Color32::WHITE),
    );
    painter.circle_filled(Pos2::new(x, rect.center().y), 5.0, chance_color(chance));
}

#[cfg(test)]
#[cfg(not(target_arch = "wasm32"))]
mod tests {
    use super::*;

    const CHAMONIX: LatLon = LatLon::new(45.92, 6.87);

    /// Overpass is a public service: it goes down, it throttles, it times out. A
    /// network test failing on that teaches nothing — report and bail out.
    /// The patience covers the whole endpoint-switching chain (3 × 20 s).
    const PATIENCE: u32 = 700;

    fn unavailable(app: &TrackFinderApp) -> bool {
        let (_, _, failed, _) = app.trails.stats();
        failed > 0
    }

    /// Spins the loop until the pending click resolves. Returns false when
    /// Overpass gave nothing — up to the test to declare itself skipped.
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

    /// Places a point the way a click on the map would.
    #[must_use]
    fn click(app: &mut TrackFinderApp, ll: LatLon, ctx: &egui::Context) -> bool {
        app.trails.ensure(ll, ctx);
        app.pending_click = Some(ll);
        pump_until(app, ctx)
    }

    macro_rules! skip_if_offline {
        ($ok:expr) => {
            if !$ok {
                eprintln!("Overpass unavailable — test skipped");
                return;
            }
        };
    }

    // -----------------------------------------------------------------------
    // Performance measurement, headless: the same path eframe takes
    // (`Context::run_ui`), tessellation included.
    // -----------------------------------------------------------------------

    /// Synthetic network: `count` twelve-point ways around Chamonix, at a
    /// density comparable to what Overpass returns over an inhabited valley.
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

    /// Trail source that records what is asked for and never answers — the
    /// requests stay on the wire forever, like a hung Overpass instance.
    #[derive(Default)]
    struct RecordingTrails {
        zones: std::cell::RefCell<Vec<ZoneKey>>,
    }

    impl crate::trails::TrailSource for RecordingTrails {
        fn request(
            &self,
            zone: ZoneKey,
            _attempt: usize,
            _done: crate::trails::TrailCallback,
        ) {
            self.zones.borrow_mut().push(zone);
        }
        fn endpoint_count(&self) -> usize {
            1
        }
    }

    /// Trail source that answers on the spot with one way crossing the zone.
    #[derive(Default)]
    struct AnsweringTrails {
        zones: std::cell::RefCell<Vec<ZoneKey>>,
    }

    impl crate::trails::TrailSource for AnsweringTrails {
        fn request(&self, zone: ZoneKey, _attempt: usize, done: crate::trails::TrailCallback) {
            self.zones.borrow_mut().push(zone);
            let (s, w, n, e) = zone.bbox();
            // Ids unique per zone, otherwise insertion would dedupe them away.
            let id = (zone.lat as i64) * 100_000 + zone.lon as i64;
            done(Ok(format!(
                r#"{{"elements":[{{"type":"way","id":{id},"nodes":[{a},{b}],
                   "tags":{{"highway":"path"}},
                   "geometry":[{{"lat":{s},"lon":{w}}},{{"lat":{n},"lon":{e}}}]}}]}}"#,
                a = id * 10,
                b = id * 10 + 1,
            )));
        }
        fn endpoint_count(&self) -> usize {
            1
        }
    }

    /// Swaps in a trail source that never touches the network.
    fn silent_trails(app: &mut TrackFinderApp) -> Rc<RecordingTrails> {
        let src = Rc::new(RecordingTrails::default());
        app.trails = TrailStore::new(Rc::clone(&src) as Rc<dyn crate::trails::TrailSource>);
        src
    }

    /// Distance from a zone's centre to a point, in metres.
    fn zone_center_distance_m(zone: ZoneKey, to: LatLon) -> f64 {
        let (s, w, n, e) = zone.bbox();
        crate::geo::haversine_m(LatLon::new((s + n) / 2.0, (w + e) / 2.0), to)
    }

    /// Tile source that answers instantly, offline.
    ///
    /// The benchmark used to hit `data.geopf.fr`, which made every number depend
    /// on the network and on how many tiles happened to land in a given frame —
    /// unusable to compare two runs. Here every tile is there at once, so what is
    /// measured is decode, upload and tessellation: exactly the work a zoom
    /// triggers.
    struct StubTiles {
        png: Vec<u8>,
        bil: Vec<u8>,
    }

    impl Default for StubTiles {
        fn default() -> Self {
            // Not a flat colour: a uniform image compresses to nothing and would
            // make PNG decoding look free, which is the very cost under study.
            let mut img = image::RgbaImage::new(256, 256);
            for (x, y, px) in img.enumerate_pixels_mut() {
                let v = ((x * 7 + y * 13) % 256) as u8;
                *px = image::Rgba([v, v.wrapping_mul(3), 255 - v, 255]);
            }
            let mut png = std::io::Cursor::new(Vec::new());
            image::DynamicImage::ImageRgba8(img)
                .write_to(&mut png, image::ImageFormat::Png)
                .expect("PNG encoding");
            Self {
                png: png.into_inner(),
                // All zeros: a valid elevation of 0 m, so the profile completes
                // and stops recomputing, as it would with the real service.
                bil: vec![0u8; crate::dem::DEM_BYTES],
            }
        }
    }

    impl TileSource for StubTiles {
        fn request(&self, desc: crate::tiles::TileDesc, done: crate::tiles::TileCallback) {
            let bytes = match desc.dataset {
                crate::tiles::Dataset::Elevation => self.bil.clone(),
                crate::tiles::Dataset::Raster(_) => self.png.clone(),
            };
            done(Ok(bytes));
        }
    }

    fn offline_app() -> TrackFinderApp {
        let mut app = TrackFinderApp::with_source(Rc::new(StubTiles::default()));
        app.snap_to_trail = false;
        app.auto_load_trails = false;
        // Measured where the work actually happens: at the opening zoom over the
        // whole range no trail is drawn, and the numbers would mean nothing.
        app.fit_alps = false;
        app.view = MapView::centered_on(CHAMONIX, 14.0);
        app
    }

    /// Runs `frames` frames of a wheel zoom, alternating direction, and returns
    /// (median, worst) frame time.
    ///
    /// This is the gesture the tile-level machinery exists for: without it every
    /// integer level crossed rebuilds the whole visible grid.
    fn bench_zoom(app: &mut TrackFinderApp, frames: usize) -> (f64, f64, f64) {
        let ctx = egui::Context::default();
        let screen = Rect::from_min_size(Pos2::ZERO, Vec2::new(1600.0, 900.0));
        let pos = Pos2::new(700.0, 450.0);
        let mut times = Vec::with_capacity(frames);

        for f in 0..frames {
            // Roughly one integer zoom level every ten frames, in and back out.
            let dir = if (f / 30) % 2 == 0 { 1.0 } else { -1.0 };
            let input = egui::RawInput {
                screen_rect: Some(screen),
                time: Some(f as f64 / 60.0),
                events: vec![
                    egui::Event::PointerMoved(pos),
                    egui::Event::MouseWheel {
                        unit: egui::MouseWheelUnit::Point,
                        delta: Vec2::new(0.0, 20.0 * dir),
                        phase: egui::TouchPhase::Move,
                        modifiers: Default::default(),
                    },
                ],
                ..Default::default()
            };
            let start = std::time::Instant::now();
            let mut output = ctx.run_ui(input, |ui| app.ui_impl(ui));
            let _ = ctx.tessellate(output.shapes, output.pixels_per_point);
            output.textures_delta.clear();
            times.push(start.elapsed().as_secs_f64() * 1000.0);
        }
        summarise(times)
    }

    /// (median, p95, worst) in milliseconds.
    ///
    /// The single worst frame over a short run is noisy — one scheduler hiccup
    /// moves it. p95 is what says whether hitches are systematic.
    fn summarise(mut times: Vec<f64>) -> (f64, f64, f64) {
        times.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let p95 = times[(times.len() * 95 / 100).min(times.len() - 1)];
        (times[times.len() / 2], p95, *times.last().unwrap())
    }

    /// Runs `frames` frames simulating a map drag, and returns (median, worst)
    /// frame time, tessellation included.
    fn bench_pan(app: &mut TrackFinderApp, frames: usize) -> (f64, f64, f64) {
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
            // In debug, epaint refuses to have an unapplied texture delta dropped.
            output.textures_delta.clear();
            times.push(start.elapsed().as_secs_f64() * 1000.0);
        }
        summarise(times)
    }

    #[test]
    #[ignore = "perf"]
    fn frame_cost() {
        let mut app = offline_app();
        let (med, p95, worst) = bench_pan(&mut app, 200);
        println!("pan  · bare map         : median {med:.2} · p95 {p95:.2} · worst {worst:.2} ms");

        for way in fake_network(1500) {
            app.trails.net.insert(way);
        }
        let (med, p95, worst) = bench_pan(&mut app, 200);
        println!("pan  · + 1500 trails    : median {med:.2} · p95 {p95:.2} · worst {worst:.2} ms");

        // A ~15 km track, like a real leg.
        for i in 0..40 {
            app.track.push(Waypoint::free(LatLon::new(
                45.90 + i as f64 * 0.004,
                6.86 + i as f64 * 0.002,
            )));
        }
        let (med, p95, worst) = bench_pan(&mut app, 200);
        println!("pan  · + 40-point track : median {med:.2} · p95 {p95:.2} · worst {worst:.2} ms");

        // The gesture the whole tile-level machinery exists for.
        let mut app = offline_app();
        let (med, p95, worst) = bench_zoom(&mut app, 200);
        println!("zoom · bare map         : median {med:.2} · p95 {p95:.2} · worst {worst:.2} ms");

        for way in fake_network(1500) {
            app.trails.net.insert(way);
        }
        let (med, p95, worst) = bench_zoom(&mut app, 200);
        println!("zoom · + 1500 trails    : median {med:.2} · p95 {p95:.2} · worst {worst:.2} ms");
    }

    #[test]
    #[ignore = "network"]
    fn a_click_on_a_trail_snaps() {
        let ctx = egui::Context::default();
        let mut app = TrackFinderApp::new();

        // Place du Triangle de l'Amitie, Chamonix: dense pedestrian area.
        let point = LatLon::new(45.9237, 6.8694);
        skip_if_offline!(click(&mut app, point, &ctx));

        let status = app.status.clone();
        assert_eq!(app.track.waypoints.len(), 1, "status: {status:?}");
        let wp = app.track.waypoints[0].clone();
        let click = point;
        let snap = wp.snap.expect("must be snapped");
        assert!(snap.dist_m <= SNAP_RADIUS_M);
        assert!(crate::geo::haversine_m(wp.pos, click) <= SNAP_RADIUS_M);
    }

    /// Spins the loop until the graph is built and its climbs read from the DEM
    /// (both arrive asynchronously).
    #[must_use]
    fn settle(app: &mut TrackFinderApp, ctx: &egui::Context) -> bool {
        for _ in 0..PATIENCE {
            app.trails.pump(ctx);
            app.dem.begin_frame(ctx);
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
    #[ignore = "network"]
    fn isochrone_then_a_leg_on_real_ground() {
        let ctx = egui::Context::default();
        let mut app = TrackFinderApp::new();
        app.budget_h = 1.0;

        // Start of the Flegere trail, above Les Praz de Chamonix.
        let start = LatLon::new(45.9328, 6.8846);
        skip_if_offline!(click(&mut app, start, &ctx));
        assert_eq!(app.track.waypoints.len(), 1, "{:?}", app.status);

        // Then the basin around it: two large zones cover a 1 h budget.
        app.trails.ensure_area(start, 2500.0, 2, &ctx);
        app.graph_dirty = true;
        skip_if_offline!(settle(&mut app, &ctx));

        let iso = app.iso.as_ref().unwrap();
        assert!(
            iso.reach.reached_count() > 20,
            "isochrone too small: {} nodes",
            iso.reach.reached_count()
        );
        assert!(!iso.edges.is_empty());

        // A target at mid-budget: neither on the spot nor at the edge.
        let budget = iso.budget_ms;
        let target = (0..app.graph.node_count() as u32)
            .filter_map(|n| iso.reach.cost(n).map(|c| (c, n)))
            .filter(|(c, _)| *c > budget / 3 && *c < budget * 3 / 4)
            .max_by_key(|(c, _)| *c)
            .map(|(_, n)| app.graph.nodes[n as usize])
            .expect("a node at mid-budget");

        skip_if_offline!(click(&mut app, target, &ctx));
        assert_eq!(app.track.waypoints.len(), 2, "{:?}", app.status);

        let leg = app.track.waypoints[1].clone();
        let via = leg.via.expect("the leg must be routed through the graph");
        assert!(via.len() > 4, "geometry too short: {}", via.len());

        // Going by the trails is necessarily longer than the straight line.
        let direct = crate::geo::haversine_m(app.track.waypoints[0].pos, leg.pos);
        let walked = crate::trails::path_length_m(&via);
        assert!(walked >= direct * 0.99, "walked {walked} < direct {direct}");
        assert!(walked > 100.0);

        app.track
            .refresh(&app.trails.net, &mut app.dem, &app.walk, &ctx);
        assert!(app.track.stats().distance_m > 100.0);
    }

    #[test]
    #[ignore = "network"]
    fn a_click_off_trail_is_refused() {
        let ctx = egui::Context::default();
        let mut app = TrackFinderApp::new();

        // Middle of the Bossons glacier: no mapped path at all.
        skip_if_offline!(click(&mut app, LatLon::new(45.8770, 6.8480), &ctx));

        let status = app.status.clone();
        assert!(app.track.waypoints.is_empty(), "status: {status:?}");
        assert!(
            status.unwrap().contains("refused"),
            "the user has to know why nothing happened"
        );
    }

    // -----------------------------------------------------------------------
    // Offline behaviour
    // -----------------------------------------------------------------------

    /// The colour ramp has to actually travel from red to green, and land on
    /// amber in between — the whole point is that the reading is continuous.
    #[test]
    fn the_chance_ramp_runs_from_red_to_green() {
        let low = chance_color(0.0);
        let mid = chance_color(0.5);
        let high = chance_color(1.0);
        assert!(low.r() > low.g(), "0 must read red: {low:?}");
        assert!(high.g() > high.r(), "1 must read green: {high:?}");
        assert!(mid.r() > 150 && mid.g() > 100, "0.5 must read amber: {mid:?}");
        // Monotonic on the green channel: no colour inversion mid-scale.
        let greens: Vec<u8> = (0..=10).map(|i| chance_color(i as f32 / 10.0).g()).collect();
        for w in greens.windows(2) {
            assert!(w[1] >= w[0], "green channel not monotonic: {greens:?}");
        }
        // Out-of-range values are clamped rather than wrapping to a wrong colour.
        assert_eq!(chance_color(-1.0), low);
        assert_eq!(chance_color(2.0), high);
    }

    /// Automatic loading must stay quiet while the view is moving and below the
    /// zoom threshold: those two guards are what keeps Overpass from being
    /// hammered one zone per frame.
    #[test]
    fn auto_loading_waits_for_a_settled_close_view() {
        let ctx = egui::Context::default();
        let mut app = TrackFinderApp::new();
        silent_trails(&mut app);
        app.last_map_rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(800.0, 600.0));

        // Too far out: nothing is requested, and the panel says why.
        app.view = MapView::centered_on(CHAMONIX, 9.0);
        app.update_auto_trails(&ctx);
        assert_eq!(app.trails.stats().1, 0, "no request should go out at z9");
        assert!(app.auto_trails_hint().is_some());

        // Close in but still moving: still nothing.
        let rect = app.last_map_rect;
        app.view = MapView::centered_on(CHAMONIX, 15.0);
        app.view.zoom_at(0.3, rect.center(), rect);
        app.update_auto_trails(&ctx);
        assert_eq!(app.trails.stats().1, 0, "a moving view must not fetch");

        // Switched off entirely: nothing either, and no hint to show.
        app.view = MapView::centered_on(CHAMONIX, 15.0);
        app.auto_load_trails = false;
        app.update_auto_trails(&ctx);
        assert_eq!(app.trails.stats().1, 0);
        assert!(app.auto_trails_hint().is_none());
    }

    /// Once settled and close enough, one scan happens and the next frames add
    /// nothing: the scan is keyed on the centre zone plus the zoom.
    #[test]
    fn auto_loading_scans_once_per_zone() {
        let ctx = egui::Context::default();
        let mut app = TrackFinderApp::new();
        silent_trails(&mut app);
        app.last_map_rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(800.0, 600.0));
        app.view = MapView::centered_on(CHAMONIX, 15.0);

        app.update_auto_trails(&ctx);
        let after_first = app.trails.stats().1;
        assert!(after_first > 0, "a settled close view must load its trails");
        assert!(
            after_first <= MAX_VISIBLE_ZONES,
            "{after_first} zones exceeds the cap"
        );

        for _ in 0..10 {
            app.update_auto_trails(&ctx);
        }
        assert_eq!(
            app.trails.stats().1,
            after_first,
            "staying still must not queue anything more"
        );
    }

    /// The app opens on the whole range, and quietly: the opening zoom sits
    /// below the auto-loading threshold, so merely loading the page must not
    /// send anything to Overpass.
    #[test]
    fn it_opens_on_the_whole_alps_without_fetching_trails() {
        let ctx = egui::Context::default();
        let mut app = TrackFinderApp::with_source(Rc::new(StubTiles::default()));
        silent_trails(&mut app);
        let rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(1280.0, 900.0));
        app.last_map_rect = rect;
        assert!(app.fit_alps, "the first frame owes us a fit");
        app.view.fit_bounds(ALPS_SW, ALPS_NE, rect);
        app.fit_alps = false;

        // Chamonix, Grenoble and Nice's hinterland are all on screen.
        for place in [
            LatLon::new(45.92, 6.87),
            LatLon::new(45.19, 5.72),
            LatLon::new(44.09, 7.20),
        ] {
            let p = app.view.latlon_to_screen(place, rect);
            assert!(rect.contains(p), "{place:?} is off screen at {p:?}");
        }
        assert!(
            app.view.tile_zoom() < AUTO_TRAILS_MIN_ZOOM,
            "opening zoom z{} would trigger Overpass",
            app.view.tile_zoom()
        );
        app.update_auto_trails(&ctx);
        assert_eq!(app.trails.stats().1, 0, "opening must not hit Overpass");
    }

    /// Once a point is placed, loading follows **the point**, not the viewport.
    /// Panning away to read the map must not send Overpass off to the far side of
    /// the range while the basin you depend on stays half loaded.
    #[test]
    fn auto_loading_follows_the_last_point_not_the_view() {
        let ctx = egui::Context::default();
        let mut app = TrackFinderApp::with_source(Rc::new(StubTiles::default()));
        let src = silent_trails(&mut app);
        app.fit_alps = false;
        app.last_map_rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(1280.0, 900.0));

        app.track.push(Waypoint::free(CHAMONIX));
        // The map is looking 200 km away, in the southern Alps.
        let elsewhere = LatLon::new(44.10, 6.20);
        app.view = MapView::centered_on(elsewhere, 15.0);

        app.update_auto_trails(&ctx);

        let zones = src.zones.borrow();
        assert!(!zones.is_empty(), "the basin must start loading");
        for zone in zones.iter() {
            assert!(
                zone_center_distance_m(*zone, CHAMONIX) < 25_000.0,
                "{zone:?} is not around the anchor"
            );
            assert!(
                zone_center_distance_m(*zone, elsewhere) > 100_000.0,
                "{zone:?} follows the view instead of the point"
            );
        }
        // And the zoom gate no longer applies: the anchor decides, not the view.
        assert!(app.auto_trails_hint().is_none());
    }

    /// A trickle, not a flood: Overpass grants two slots, so at most `BASIN_STEP`
    /// new zones go out per settled frame, and nothing more while they are still
    /// in flight — the answer to "did that help?" does not exist yet.
    #[test]
    fn the_basin_grows_a_few_zones_at_a_time() {
        let ctx = egui::Context::default();
        let mut app = TrackFinderApp::with_source(Rc::new(StubTiles::default()));
        let src = silent_trails(&mut app);
        app.fit_alps = false;
        app.last_map_rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(1280.0, 900.0));
        app.view = MapView::centered_on(CHAMONIX, 15.0);
        app.track.push(Waypoint::free(CHAMONIX));

        app.update_auto_trails(&ctx);
        let after_first = app.trails.stats().1;
        assert!(after_first > 0 && after_first <= BASIN_STEP, "{after_first}");

        for _ in 0..20 {
            app.update_auto_trails(&ctx);
        }
        assert_eq!(
            app.trails.stats().1,
            after_first,
            "nothing more may go out while zones are in flight"
        );
        assert!(src.zones.borrow().len() <= BASIN_STEP);
    }

    /// The basin stops when more data would change nothing: zones landed, the
    /// isochrone did not get any further, so there is nothing to grow toward.
    #[test]
    fn the_basin_stops_when_the_isochrone_stops_growing() {
        let ctx = egui::Context::default();
        let mut app = TrackFinderApp::with_source(Rc::new(StubTiles::default()));
        let src = Rc::new(AnsweringTrails::default());
        app.trails = TrailStore::new(Rc::clone(&src) as Rc<dyn crate::trails::TrailSource>);
        app.fit_alps = false;
        app.last_map_rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(1280.0, 900.0));
        app.view = MapView::centered_on(CHAMONIX, 15.0);
        app.track.push(Waypoint::free(CHAMONIX));

        for _ in 0..30 {
            app.update_auto_trails(&ctx);
            app.trails.pump(&ctx);
        }
        let basin = app.basin.as_ref().expect("a basin");
        assert!(basin.complete, "the basin must converge, not grind on");
        assert!(basin.asked <= MAX_BASIN_ZONES, "{}", basin.asked);
        assert!(
            src.zones.borrow().len() <= MAX_BASIN_ZONES,
            "{} zones requested",
            src.zones.borrow().len()
        );
        // And it says so, rather than leaving a spinner turning.
        let (text, partial) = app.basin_status().expect("a readout");
        assert!(text.contains("basin"), "{text}");
        assert!(!partial, "a converged basin is not partial: {text}");
        assert!(!text.contains("growing"), "{text}");
        assert!(basin.loaded > 0, "zones landed but none counted: {text}");
    }

    /// Moving the last point moves the basin with it — the previous anchor's
    /// budget must not carry over.
    #[test]
    fn a_new_point_restarts_the_basin() {
        let ctx = egui::Context::default();
        let mut app = TrackFinderApp::with_source(Rc::new(StubTiles::default()));
        silent_trails(&mut app);
        app.fit_alps = false;
        app.last_map_rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(1280.0, 900.0));
        app.view = MapView::centered_on(CHAMONIX, 15.0);

        app.track.push(Waypoint::free(CHAMONIX));
        app.update_auto_trails(&ctx);
        assert_eq!(app.basin.as_ref().unwrap().anchor, CHAMONIX);

        let next = LatLon::new(45.98, 6.95);
        app.track.push(Waypoint::free(next));
        app.update_auto_trails(&ctx);
        let basin = app.basin.as_ref().unwrap();
        assert_eq!(basin.anchor, next);
        assert!(!basin.complete);
    }

    /// The ceiling holds: a huge budget must not sweep the whole range off a
    /// public service.
    #[test]
    fn the_basin_never_exceeds_its_ceiling() {
        let ctx = egui::Context::default();
        let mut app = TrackFinderApp::with_source(Rc::new(StubTiles::default()));
        let src = silent_trails(&mut app);
        app.fit_alps = false;
        app.budget_h = 10.0;
        app.last_map_rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(1280.0, 900.0));
        app.view = MapView::centered_on(CHAMONIX, 15.0);
        app.track.push(Waypoint::free(CHAMONIX));

        app.update_auto_trails(&ctx);
        app.basin.as_mut().unwrap().asked = MAX_BASIN_ZONES;
        let before = src.zones.borrow().len();
        for _ in 0..50 {
            app.update_auto_trails(&ctx);
        }
        assert_eq!(src.zones.borrow().len(), before, "the ceiling must hold");
    }

    /// The base map is a single choice, and an overlay is never a ground layer.
    #[test]
    fn the_base_map_is_exclusive() {
        let mut app = TrackFinderApp::new();
        assert!(app.base.is_base());
        app.base = RasterLayer::Ortho;
        assert!(app.base.is_base());
        assert!(
            app.overlays.iter().all(|o| !o.layer.is_base()),
            "an overlay must never be a ground layer"
        );
    }
}
