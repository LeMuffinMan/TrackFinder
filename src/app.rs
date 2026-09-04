use std::rc::Rc;

use egui::{Color32, Pos2, Rect, Stroke, Vec2};

use crate::dem::DemStore;
use crate::geo::LatLon;
use crate::graph::{self, CostMs, EdgePos, Graph, Reach};
use crate::map::{self, MapView, TileRenderer, MAX_TILE_REQUESTS};
use crate::terrain::{self, TerrainAnalysis};
use crate::tiles::{HttpTileSource, RasterLayer, TileSource};
use crate::track::{format_duration, Track, WalkSettings, Waypoint};
use crate::archive::TrailArchive;
use crate::trails::{Snap, TrailNetwork, SNAP_RADIUS_M};

/// The French Alps, from Lake Geneva down to the Mercantour.
///
/// The application opens on the whole range rather than on one valley: you pick
/// the massif first, then zoom in. Deliberately below `TRAILS_MIN_ZOOM`, so
/// opening the page draws no network and fetches no trail tile.
const ALPS_SW: LatLon = LatLon::new(43.95, 5.35);
const ALPS_NE: LatLon = LatLon::new(46.45, 7.75);

/// The opaque ground layers, exactly one of which is drawn. Stacking two of them
/// only ever shows the top one while paying to fetch and decode both.
const BASE_MAPS: [RasterLayer; 2] = [RasterLayer::PlanIgn, RasterLayer::Ortho];

/// Below this zoom the OSM network is not drawn: at 1:200000 thousands of
/// polylines collapse into an unreadable smear that costs milliseconds a frame.
const TRAILS_MIN_ZOOM: u8 = 12;

/// Trails loaded around the last point of the track, in metres.
///
/// Generous on purpose: the point of placing a point is to plan a whole leg from
/// it, seeing where the trails go without zooming around to make them load. At
/// ~250 points/km² this is a few megabytes of static tiles, fetched in parallel.
const LEG_RADIUS_M: f64 = 30_000.0;

/// Trails loaded around the view while no point is placed yet.
///
/// Smaller: at this stage the network is only a cue for where one may usefully
/// click, not the ground a leg will be planned on.
const VIEW_RADIUS_M: f64 = 8_000.0;

/// Trails a click needs before it can snap.
///
/// The tile holding the click is almost always enough — they are ~13 km across
/// and the snap radius is 60 m — but a click near a tile edge needs its
/// neighbour too.
const CLICK_RADIUS_M: f64 = 500.0;

/// How much of the network to draw, independently of the map zoom.
///
/// ⚠️ **A display control, deliberately — it changes nothing that is loaded, and
/// nothing that is computed.** Two alternatives were considered and rejected:
///
/// - *simplifying the geometry in the archive*: Douglas-Peucker cuts the
///   switchbacks, distances shorten by a few percent and Naismith turns
///   optimistic. Weighted walking time is the core of this application; trading
///   its accuracy for bytes is a bad deal.
/// - *fetching fewer bytes at low detail*: that means separate files per level,
///   so changing the slider refetches, and routing would have to keep the full
///   geometry anyway — the saving evaporates.
///
/// What is left is what the eye actually needs: fewer classes, and a coarser
/// screen-space decimation. Both act at the current zoom, both are instant.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Detail {
    /// Hiking ways only, heavily decimated: the shape of the network.
    Sparse,
    /// Everything off-tarmac.
    Normal,
    /// Roads and streets included, near-full geometry.
    Full,
}

impl Detail {
    fn label(self) -> &'static str {
        match self {
            Self::Sparse => "Paths only",
            Self::Normal => "Off-road",
            Self::Full => "Everything",
        }
    }

    fn hint(self) -> &'static str {
        match self {
            Self::Sparse => "hiking ways, simplified — the shape of the network",
            Self::Normal => "adds tracks and cycleways",
            Self::Full => "adds roads and streets, near-full geometry",
        }
    }

    fn draws(self, kind: crate::trails::WayKind) -> bool {
        match self {
            Self::Sparse => kind.is_hiking(),
            Self::Normal => kind.is_offroad(),
            Self::Full => true,
        }
    }

    /// Squared screen distance below which a shape point is dropped.
    ///
    /// An OSM way holds a point every few metres; zoomed out that is dozens of
    /// vertices inside one pixel, all of them tessellated for nothing. Raising
    /// the threshold trades fidelity for a cleaner, cheaper map — at whatever
    /// zoom the map happens to be.
    fn min_segment_px2(self) -> f32 {
        match self {
            Self::Sparse => 36.0,
            Self::Normal => 9.0,
            Self::Full => 2.25,
        }
    }
}

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
    /// Where the trails come from: static tiles on the same origin.
    archive: TrailArchive,
    /// The trails themselves. Owned here now that no store owns them.
    net: TrailNetwork,
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
    /// How much of the network to draw — independent of the map zoom.
    detail: Detail,
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
            archive: TrailArchive::default(),
            net: TrailNetwork::default(),
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
            detail: Detail::Normal,
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
        if self.archive.pump(&mut self.net) {
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
        self.load_trails(&ctx);

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
                    self.archive.ensure_area(ll, CLICK_RADIUS_M, ctx);
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
            .refresh(&self.net, &mut self.dem, &self.walk, ctx);
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
        let min_px2 = self.detail.min_segment_px2();
        let mut pts: Vec<Pos2> = Vec::with_capacity(64);
        for way in self.net.ways() {
            if !way.intersects(view) || !self.detail.draws(way.kind) {
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
                        .is_none_or(|prev| (p - *prev).length_sq() >= min_px2);
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
        // The manifest has to be here before we can say anything at all.
        if self.archive.ready() && !self.archive.covers(ll) {
            self.status = Some("Outside the mapped area — the Alps only".to_owned());
            self.pending_click = None;
            return;
        }
        if !self.archive.area_ready(ll, CLICK_RADIUS_M) {
            return;
        }
        self.pending_click = None;
        let Some(snap) = self.net.snap(ll, SNAP_RADIUS_M) else {
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
            ui.strong("Trails");
            ui.checkbox(&mut self.show_trails, "Show the network");
            ui.checkbox(&mut self.snap_to_trail, "Snap points to trails");
            if self.show_trails {
                ui.horizontal(|ui| {
                    ui.label("Detail");
                    for level in [Detail::Sparse, Detail::Normal, Detail::Full] {
                        ui.selectable_value(&mut self.detail, level, level.label())
                            .on_hover_text(level.hint());
                    }
                });
                ui.weak(self.detail.hint());
            }
            let (settled, loading, regions) = self.archive.stats();
            if regions == 0 {
                ui.horizontal(|ui| {
                    ui.spinner();
                    ui.label("loading the trail index…");
                });
            } else {
                ui.monospace(format!("{settled} tiles · {} ways", self.net.len()));
                // The deployment only carries some massifs, so say plainly where
                // it has data. Outside them nothing can be placed at all, and a
                // silent empty map would look like a bug.
                let here = self
                    .track
                    .waypoints
                    .last()
                    .map(|wp| wp.pos)
                    .unwrap_or_else(|| self.view.center_latlon());
                match self.archive.manifest().region_for(here) {
                    Some(region) => ui.weak(region.name.clone()),
                    None => ui.colored_label(
                        Color32::from_rgb(200, 150, 60),
                        "no trail data here — this build covers the Alps only",
                    ),
                };
            }
            if loading > 0 {
                ui.horizontal(|ui| {
                    ui.spinner();
                    ui.label(format!("{loading} tile(s) loading"));
                });
            }
            if let Some(e) = &self.archive.last_error {
                ui.colored_label(Color32::from_rgb(200, 90, 90), format!("Trails: {e}"));
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
                    // An isochrone computed on partly loaded ground does not
                    // look partial: it stops at the edge of the data and reads
                    // as "not reachable". Say so rather than let it mislead.
                    if self.archive.stats().1 > 0 {
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
                let followed = self.track.followed_legs(&self.net);
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
                let (settled, loading, regions) = self.archive.stats();
                ui.monospace(format!(
                    "trails  {settled} tiles settled · {loading} loading · {regions} region(s)"
                ));
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
    /// Keeps the trails around the working point loaded.
    ///
    /// **The anchor is the last point of the track, not the viewport.** What the
    /// application needs is the ground a leg will be planned on; the visible
    /// rectangle is a poor proxy — you can pan away to read the map while the
    /// area you actually depend on stays half loaded.
    ///
    /// Before the first point there is no anchor, so the view is all we have,
    /// and a smaller radius: at that stage the network is only a cue for where
    /// one may usefully click.
    ///
    /// No queue, no pacing, no zoom gate on the anchored path. These are static
    /// tiles behind a CDN and `ensure_area` already skips everything settled or
    /// in flight; holding them back would only make the map slower.
    fn load_trails(&mut self, ctx: &egui::Context) {
        match self.track.waypoints.last().map(|wp| wp.pos) {
            Some(anchor) => self.archive.ensure_area(anchor, LEG_RADIUS_M, ctx),
            None => {
                // Only once the view stops moving: a drag would otherwise ask
                // for a fresh disc of tiles on every frame.
                if self.last_map_rect != Rect::NOTHING
                    && self.view.view_settled()
                    && self.view.tile_zoom() >= TRAILS_MIN_ZOOM
                {
                    let center = self.view.center_latlon();
                    self.archive.ensure_area(center, VIEW_RADIUS_M, ctx);
                }
            }
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
            self.graph = Graph::build(&self.net);
            self.graph_dirty = false;
            self.graph_elevated = false;
            self.iso_dirty = true;
        }
        if !self.graph.is_empty() && !self.graph_elevated {
            self.graph_elevated = self.graph.update_elevations(&self.net, &mut self.dem, ctx);
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
        let source_pos = self.graph.locate(&self.net, &source)?;
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
        let target_pos = self.graph.locate(&self.net, target)?;
        let r = graph::route(
            &self.graph,
            &self.net,
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
            let Some(way) = self.net.way_by_id(edge.way_id) else {
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
                    // A realistic mix: the detail control filters by class, and a
                    // network of nothing but paths would hide that entirely.
                    kind: match i % 3 {
                        0 => crate::trails::WayKind::Path,
                        1 => crate::trails::WayKind::Track,
                        _ => crate::trails::WayKind::Road,
                    },
                    name: None,
                    nodes: Vec::new(),
                    points,
                    sac_scale: None,
                    bounds,
                }
            })
            .collect()
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
        // No manifest is ever fetched here, so no tile request goes out: the
        // benchmark measures rendering, not loading.
        app.archive = TrailArchive::new("offline/");
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
            app.net.insert(way);
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

        // What the detail control is worth, on the same network and the same
        // gesture. If the numbers do not move, the slider is decoration.
        for level in [Detail::Sparse, Detail::Normal, Detail::Full] {
            let mut app = offline_app();
            for way in fake_network(1500) {
                app.net.insert(way);
            }
            app.detail = level;
            let (med, p95, worst) = bench_pan(&mut app, 200);
            println!(
                "pan  · detail {:<10}: median {med:.2} · p95 {p95:.2} · worst {worst:.2} ms",
                level.label()
            );
        }

        // The gesture the whole tile-level machinery exists for.
        let mut app = offline_app();
        let (med, p95, worst) = bench_zoom(&mut app, 200);
        println!("zoom · bare map         : median {med:.2} · p95 {p95:.2} · worst {worst:.2} ms");

        for way in fake_network(1500) {
            app.net.insert(way);
        }
        let (med, p95, worst) = bench_zoom(&mut app, 200);
        println!("zoom · + 1500 trails    : median {med:.2} · p95 {p95:.2} · worst {worst:.2} ms");
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

    /// The app opens on the whole range, and quietly: the opening zoom sits
    /// below the auto-loading threshold, so merely loading the page must not
    /// send anything to Overpass.
    #[test]
    fn it_opens_on_the_whole_alps_without_fetching_trails() {
        let ctx = egui::Context::default();
        let mut app = TrackFinderApp::with_source(Rc::new(StubTiles::default()));
        app.archive = TrailArchive::new("offline/");
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
            app.view.tile_zoom() < TRAILS_MIN_ZOOM,
            "opening zoom z{} would already draw and load trails",
            app.view.tile_zoom()
        );
        app.load_trails(&ctx);
        assert_eq!(
            app.archive.stats().1,
            0,
            "opening the whole range must not fetch a single tile"
        );
    }

    /// Detail levels must nest: anything a sparser level draws, a denser one
    /// draws too. A slider that hid a trail as you asked for *more* detail would
    /// be worse than no slider.
    #[test]
    fn detail_levels_nest() {
        use crate::trails::WayKind::*;
        let kinds = [Path, Track, Footway, Steps, Cycleway, Road];
        let (sparse, normal, full) = (Detail::Sparse, Detail::Normal, Detail::Full);
        for k in kinds {
            if sparse.draws(k) {
                assert!(normal.draws(k), "{k:?} vanishes between sparse and normal");
            }
            if normal.draws(k) {
                assert!(full.draws(k), "{k:?} vanishes between normal and full");
            }
        }
        assert!(full.draws(Road) && !normal.draws(Road), "roads separate the top two");
        assert!(normal.draws(Track) && !sparse.draws(Track), "tracks separate the bottom two");
        assert!(sparse.draws(Path), "a hiking path is drawn at every level");
    }

    /// More detail means a finer decimation threshold, never a coarser one.
    #[test]
    fn more_detail_keeps_more_geometry() {
        assert!(Detail::Sparse.min_segment_px2() > Detail::Normal.min_segment_px2());
        assert!(Detail::Normal.min_segment_px2() > Detail::Full.min_segment_px2());
        // Even at full detail, points inside the same pixel are still dropped:
        // drawing the same line twice helps nobody.
        assert!(Detail::Full.min_segment_px2() > 1.0);
    }

    /// The detail control is a display concern only. It must never change what
    /// is loaded — the walking time is computed from the full geometry, and a
    /// slider that quietly shortened the route would be a correctness bug.
    #[test]
    fn detail_changes_nothing_that_is_loaded() {
        let ctx = egui::Context::default();
        let mut app = TrackFinderApp::with_source(Rc::new(StubTiles::default()));
        app.archive = TrailArchive::new("offline/");
        app.fit_alps = false;
        app.last_map_rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(1280.0, 900.0));
        app.view = MapView::centered_on(CHAMONIX, 15.0);

        app.detail = Detail::Sparse;
        app.load_trails(&ctx);
        let sparse = app.archive.stats();
        app.detail = Detail::Full;
        app.load_trails(&ctx);
        assert_eq!(app.archive.stats(), sparse, "detail must not drive loading");
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
