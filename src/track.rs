//! Track: polyline, sampling, elevation profile, walking time.

use crate::dem::DemStore;
use crate::geo::{haversine_m, lerp_latlon, LatLon};
use crate::trails::{Snap, TrailNetwork};

/// Profile sampling step. 50 m is finer than the DEM (~5 m/px at zoom 14)
/// without blowing up the number of tiles requested.
const STEP_M: f64 = 50.0;
const MAX_SAMPLES: usize = 4000;
/// Hysteresis on climb: without it DEM noise inflates the total ascent.
const ASCENT_THRESHOLD_M: f32 = 3.0;

#[derive(Clone, Copy, Debug)]
pub struct Sample {
    pub dist_m: f64,
    pub pos: LatLon,
    pub elev_m: Option<f32>,
}

/// Which walking-time model turns geometry into hours.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SpeedModel {
    /// Flat distance + total ascent, two independent rates. Ignores descent
    /// entirely, so it is optimistic on a steep way down.
    Naismith,
    /// Speed as a function of the local slope, evaluated segment by segment.
    /// Slows the descent down too, and is the only one of the two that reacts
    /// to a profile that climbs and drops repeatedly.
    Tobler,
}

impl SpeedModel {
    pub const ALL: [SpeedModel; 2] = [SpeedModel::Naismith, SpeedModel::Tobler];

    pub fn label(self) -> &'static str {
        match self {
            SpeedModel::Naismith => "Naismith",
            SpeedModel::Tobler => "Tobler",
        }
    }
}

/// Tobler's hiking function: `6 · exp(−3.5 · |slope + 0.05|)` km/h, where `slope`
/// is the tangent (dh/dx), signed — negative downhill. The peak sits at a 5 %
/// descent, not on the flat, which is what makes it worth having.
pub fn tobler_kmh(slope: f64) -> f64 {
    6.0 * (-3.5 * (slope + 0.05).abs()).exp()
}

/// Tobler's speed on the flat. Used to rescale the curve onto the user's own
/// flat speed, so switching model does not silently change the flat pace.
const TOBLER_FLAT_KMH: f64 = 5.036_742_124_615_245; // tobler_kmh(0.0)

#[derive(Clone, Copy, Debug)]
pub struct WalkSettings {
    pub model: SpeedModel,
    /// Speed on the flat, km/h (Naismith baseline: 5).
    pub flat_kmh: f64,
    /// Ascent rate, m/h (Naismith baseline: 600).
    pub ascent_mh: f64,
    pub body_weight_kg: f64,
    /// Load actually on the back right now: base pack + food + water. Written by
    /// the supply simulation, never by the profile form — the pack gets lighter
    /// as the days pass and heavier at a resupply, and the speed follows.
    pub load_kg: f64,
}

impl Default for WalkSettings {
    fn default() -> Self {
        Self {
            model: SpeedModel::Naismith,
            flat_kmh: 5.0,
            ascent_mh: 600.0,
            body_weight_kg: 70.0,
            load_kg: 12.0,
        }
    }
}

impl WalkSettings {
    /// speed_factor = 1 − 0.01 × max(0, load − 10)
    pub fn speed_factor_for(&self, load_kg: f64) -> f64 {
        1.0 - 0.01 * (load_kg - 10.0).max(0.0)
    }

    pub fn speed_factor(&self) -> f64 {
        self.speed_factor_for(self.load_kg)
    }

    /// load_limit = body_weight × 0.20
    pub fn load_limit_kg(&self) -> f64 {
        self.body_weight_kg * 0.20
    }

    pub fn overloaded(&self) -> bool {
        self.load_kg > self.load_limit_kg()
    }

    /// Speed on a segment of the given signed slope, km/h, load excluded.
    /// Naismith has no notion of local slope, so it answers with the flat speed
    /// and lets the caller add the ascent term separately.
    pub fn slope_kmh(&self, slope: f64) -> f64 {
        match self.model {
            SpeedModel::Naismith => self.flat_kmh.max(0.1),
            SpeedModel::Tobler => {
                tobler_kmh(slope) * self.flat_kmh.max(0.1) / TOBLER_FLAT_KMH
            }
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct TrackStats {
    pub distance_m: f64,
    pub ascent_m: f64,
    pub descent_m: f64,
    pub min_elev_m: Option<f32>,
    pub max_elev_m: Option<f32>,
    /// Walking time at an unloaded pace. The load only ever divides it, so the
    /// supply simulation can re-price a leg without walking the profile again.
    pub base_time_h: f64,
    pub time_h: f64,
    /// False while DEM tiles are missing: the figures are partial.
    pub elevation_complete: bool,
}

/// A point placed by the user. `snap` is filled in when the point was snapped
/// onto an OSM trail — that is what makes segment following possible.
#[derive(Clone, Debug)]
pub struct Waypoint {
    pub pos: LatLon,
    pub snap: Option<Snap>,
    /// Geometry from the previous waypoint, when it comes from the graph. Takes
    /// precedence over segment following, which can only join two points of one
    /// and the same OSM way.
    pub via: Option<Vec<LatLon>>,
}

impl Waypoint {
    pub fn free(pos: LatLon) -> Self {
        Self {
            pos,
            snap: None,
            via: None,
        }
    }

    pub fn snapped(snap: Snap) -> Self {
        Self {
            pos: snap.pos,
            snap: Some(snap),
            via: None,
        }
    }

    pub fn routed(snap: Snap, via: Vec<LatLon>) -> Self {
        Self {
            pos: snap.pos,
            snap: Some(snap),
            via: Some(via),
        }
    }
}

#[derive(Default)]
pub struct Track {
    pub waypoints: Vec<Waypoint>,
    /// Geometry actually walked: follows the trails between two waypoints snapped
    /// to the same OSM way, a straight line otherwise.
    path: Vec<LatLon>,
    /// Index in `path` of each waypoint. What makes per-leg figures possible:
    /// a leg's geometry is several path vertices long once it follows a trail.
    wp_vertex: Vec<usize>,
    profile: Vec<Sample>,
    /// Sample range `[start, end]` of each leg, one per pair of waypoints.
    leg_bounds: Vec<(usize, usize)>,
    stats: TrackStats,
    legs: Vec<TrackStats>,
    dirty: bool,
}

impl Track {
    pub fn push(&mut self, wp: Waypoint) {
        self.waypoints.push(wp);
        self.dirty = true;
    }

    pub fn pop(&mut self) {
        self.waypoints.pop();
        self.dirty = true;
    }

    pub fn clear(&mut self) {
        self.waypoints.clear();
        self.dirty = true;
    }

    /// Call when the trail network changed: a stretch that was missing until now
    /// may become followable.
    pub fn invalidate(&mut self) {
        self.dirty = true;
    }

    pub fn path(&self) -> &[LatLon] {
        &self.path
    }

    pub fn profile(&self) -> &[Sample] {
        &self.profile
    }

    pub fn stats(&self) -> &TrackStats {
        &self.stats
    }

    /// One entry per leg — that is, per day of walking, since every waypoint
    /// after the start is a bivouac.
    pub fn legs(&self) -> &[TrackStats] {
        &self.legs
    }

    /// Samples of one leg, for drawing it apart from the rest of the profile.
    pub fn leg_profile(&self, leg: usize) -> &[Sample] {
        match self.leg_bounds.get(leg) {
            Some(&(a, b)) => &self.profile[a..=b],
            None => &[],
        }
    }

    /// Number of legs that actually follow a trail.
    pub fn followed_legs(&self, net: &TrailNetwork) -> usize {
        self.waypoints
            .windows(2)
            .filter(|w| leg_geometry(net, &w[0], &w[1]).is_some())
            .count()
    }

    fn rebuild_path(&mut self, net: &TrailNetwork) {
        self.path.clear();
        self.wp_vertex.clear();
        let Some(first) = self.waypoints.first() else {
            return;
        };
        self.path.push(first.pos);
        self.wp_vertex.push(0);
        for pair in self.waypoints.windows(2) {
            match leg_geometry(net, &pair[0], &pair[1]) {
                // The first point of the geometry is already in `path`.
                Some(geom) => self.path.extend(geom.into_iter().skip(1)),
                None => self.path.push(pair[1].pos),
            }
            self.wp_vertex.push(self.path.len() - 1);
        }
    }

    /// Recomputes when needed. While the DEM is incomplete this runs every frame:
    /// tiles trickle in and gradually complete the profile.
    pub fn refresh(
        &mut self,
        net: &TrailNetwork,
        dem: &mut DemStore,
        settings: &WalkSettings,
        ctx: &egui::Context,
    ) {
        if !self.dirty && self.stats.elevation_complete {
            return;
        }
        if self.dirty {
            self.rebuild_path(net);
        }
        self.dirty = false;
        let (profile, vertex_sample) = sample_polyline(&self.path);
        self.profile = profile;
        self.leg_bounds = self
            .wp_vertex
            .windows(2)
            .filter_map(|w| Some((*vertex_sample.get(w[0])?, *vertex_sample.get(w[1])?)))
            .filter(|(a, b)| a < b)
            .collect();
        let mut complete = true;
        for s in &mut self.profile {
            s.elev_m = dem.elevation(s.pos, ctx);
            complete &= s.elev_m.is_some();
        }
        // `recompute_time` carries the completeness flag over, so set it first.
        self.stats.elevation_complete = complete;
        self.recompute_time(settings);
    }

    /// Recomputes the time only (a settings change, without touching the DEM).
    pub fn recompute_time(&mut self, settings: &WalkSettings) {
        let complete = self.stats.elevation_complete;
        self.stats = compute_stats(&self.profile, settings, complete);
        self.legs = (0..self.leg_bounds.len())
            .map(|i| compute_stats(self.leg_profile(i), settings, complete))
            .collect();
    }
}

/// Geometry of a leg between two waypoints, when it follows a known trail.
fn leg_geometry(net: &TrailNetwork, a: &Waypoint, b: &Waypoint) -> Option<Vec<LatLon>> {
    if let Some(via) = &b.via {
        return Some(via.clone());
    }
    net.follow(a.snap.as_ref()?, b.snap.as_ref()?)
}

/// Splits the polyline into points about `STEP_M` apart, vertices included.
/// Also returns, for each input vertex, the index of the sample sitting exactly
/// on it — that is what cuts the profile back into legs.
fn sample_polyline(points: &[LatLon]) -> (Vec<Sample>, Vec<usize>) {
    let mut out = Vec::new();
    let mut vertex_sample = Vec::with_capacity(points.len());
    if points.is_empty() {
        return (out, vertex_sample);
    }
    let total: f64 = points.windows(2).map(|w| haversine_m(w[0], w[1])).sum();
    let step = (total / MAX_SAMPLES as f64).max(STEP_M);

    let mut dist = 0.0;
    out.push(Sample {
        dist_m: 0.0,
        pos: points[0],
        elev_m: None,
    });
    vertex_sample.push(0);
    for w in points.windows(2) {
        let seg = haversine_m(w[0], w[1]);
        if seg > 0.0 {
            let n = (seg / step).ceil().max(1.0) as usize;
            for i in 1..=n {
                let t = i as f64 / n as f64;
                out.push(Sample {
                    dist_m: dist + seg * t,
                    pos: lerp_latlon(w[0], w[1], t),
                    elev_m: None,
                });
            }
            dist += seg;
        }
        // A zero-length segment adds no sample: the vertex lands on the last one.
        vertex_sample.push(out.len() - 1);
    }
    (out, vertex_sample)
}

/// Stats over one slice of the profile — the whole track or a single leg.
/// `dist_m` is absolute along the track, so the distance is a difference.
fn compute_stats(profile: &[Sample], settings: &WalkSettings, complete: bool) -> TrackStats {
    let span = match (profile.first(), profile.last()) {
        (Some(a), Some(b)) => b.dist_m - a.dist_m,
        _ => 0.0,
    };
    let mut stats = TrackStats {
        distance_m: span,
        elevation_complete: complete && !profile.is_empty(),
        ..Default::default()
    };

    // Climb with hysteresis: a change of direction is only recorded past the
    // threshold, which discards DEM noise.
    let mut anchor: Option<f32> = None;
    for s in profile.iter().filter_map(|s| s.elev_m) {
        stats.min_elev_m = Some(stats.min_elev_m.map_or(s, |m: f32| m.min(s)));
        stats.max_elev_m = Some(stats.max_elev_m.map_or(s, |m: f32| m.max(s)));
        match anchor {
            None => anchor = Some(s),
            Some(a) => {
                let d = s - a;
                if d > ASCENT_THRESHOLD_M {
                    stats.ascent_m += d as f64;
                    anchor = Some(s);
                } else if d < -ASCENT_THRESHOLD_M {
                    stats.descent_m += (-d) as f64;
                    anchor = Some(s);
                }
            }
        }
    }

    stats.base_time_h = match settings.model {
        // Naismith: flat distance + total ascent, two independent rates.
        SpeedModel::Naismith => {
            let flat_h = (stats.distance_m / 1000.0) / settings.flat_kmh.max(0.1);
            flat_h + stats.ascent_m / settings.ascent_mh.max(1.0)
        }
        // Tobler: segment by segment on the raw samples. No hysteresis here on
        // purpose — smoothing the profile first would flatten the very slopes
        // the curve reads. DEM noise does bias this upward slightly, since the
        // exponential is convex, but at a 50 m step the wobble is a percent or
        // two of slope.
        SpeedModel::Tobler => profile
            .windows(2)
            .map(|w| {
                let dx = w[1].dist_m - w[0].dist_m;
                if dx <= 0.0 {
                    return 0.0;
                }
                let slope = match (w[0].elev_m, w[1].elev_m) {
                    (Some(a), Some(b)) => (b - a) as f64 / dx,
                    _ => 0.0,
                };
                (dx / 1000.0) / settings.slope_kmh(slope)
            })
            .sum(),
    };
    stats.time_h = stats.base_time_h / settings.speed_factor().max(0.1);
    stats
}

pub fn format_duration(hours: f64) -> String {
    if !hours.is_finite() || hours <= 0.0 {
        return "—".to_owned();
    }
    let total_min = (hours * 60.0).round() as i64;
    format!("{} h {:02}", total_min / 60, total_min % 60)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn samples(elevs: &[f32], total_m: f64) -> Vec<Sample> {
        let n = elevs.len().max(2) - 1;
        elevs
            .iter()
            .enumerate()
            .map(|(i, e)| Sample {
                dist_m: total_m * i as f64 / n as f64,
                pos: LatLon::new(45.0, 6.0),
                elev_m: Some(*e),
            })
            .collect()
    }

    #[test]
    fn load_penalises_speed() {
        let s = WalkSettings {
            load_kg: 20.0,
            ..Default::default()
        };
        assert!((s.speed_factor() - 0.90).abs() < 1e-9);
        // Below 10 kg, no penalty.
        let light = WalkSettings {
            load_kg: 8.0,
            ..Default::default()
        };
        assert!((light.speed_factor() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn load_limit_is_twenty_percent() {
        let s = WalkSettings {
            body_weight_kg: 70.0,
            load_kg: 15.0,
            ..Default::default()
        };
        assert!((s.load_limit_kg() - 14.0).abs() < 1e-9);
        assert!(s.overloaded());
    }

    #[test]
    fn naismith_on_the_flat() {
        let s = WalkSettings {
            load_kg: 0.0,
            ..Default::default()
        };
        let stats = compute_stats(&samples(&[1000.0, 1000.0], 10_000.0), &s, true);
        assert!((stats.time_h - 2.0).abs() < 1e-6, "{}", stats.time_h);
        assert_eq!(stats.ascent_m, 0.0);
    }

    #[test]
    fn naismith_with_ascent() {
        let s = WalkSettings {
            load_kg: 0.0,
            ..Default::default()
        };
        // 5 km flat (1 h) + 600 m of climb (1 h)
        let stats = compute_stats(&samples(&[1000.0, 1600.0], 5_000.0), &s, true);
        assert!((stats.ascent_m - 600.0).abs() < 1e-6);
        assert!((stats.time_h - 2.0).abs() < 1e-6, "{}", stats.time_h);
    }

    #[test]
    fn hysteresis_filters_dem_noise() {
        let s = WalkSettings::default();
        // ±1 m wobble: noise, not climb.
        let noisy: Vec<f32> = (0..100)
            .map(|i| 1000.0 + if i % 2 == 0 { 1.0 } else { -1.0 })
            .collect();
        let stats = compute_stats(&samples(&noisy, 1000.0), &s, true);
        assert_eq!(stats.ascent_m, 0.0);
        assert_eq!(stats.descent_m, 0.0);

        // A genuine 100 m climb is still counted.
        let real = compute_stats(&samples(&[1000.0, 1050.0, 1100.0], 1000.0), &s, true);
        assert!((real.ascent_m - 100.0).abs() < 1e-6);
    }

    #[test]
    fn sampling_respects_the_step() {
        // ~1.11 km north-south
        let pts = vec![LatLon::new(45.0, 6.0), LatLon::new(45.01, 6.0)];
        let (out, vertices) = sample_polyline(&pts);
        assert!(out.len() > 20, "{} samples", out.len());
        for w in out.windows(2) {
            assert!(w[1].dist_m - w[0].dist_m <= STEP_M + 1.0);
        }
        assert!((out.last().unwrap().dist_m - haversine_m(pts[0], pts[1])).abs() < 1.0);
        // Every vertex must land exactly on a sample, or the legs cannot be cut.
        assert_eq!(vertices, vec![0, out.len() - 1]);
    }

    #[test]
    fn sampling_is_capped() {
        // 100 km: the step widens instead of exploding the number of points.
        let pts = vec![LatLon::new(45.0, 6.0), LatLon::new(45.9, 6.0)];
        let (out, _) = sample_polyline(&pts);
        assert!(out.len() <= MAX_SAMPLES + 2, "{} samples", out.len());
    }

    #[test]
    fn tobler_peaks_on_a_gentle_descent() {
        // The curve's maximum sits at −5 %, not on the flat: that is what makes
        // it different from a symmetric slope penalty.
        assert!(tobler_kmh(-0.05) > tobler_kmh(0.0));
        assert!(tobler_kmh(0.0) > tobler_kmh(-0.30));
        assert!(tobler_kmh(0.30) < tobler_kmh(0.0));
        // Steep in either direction is slow.
        assert!(tobler_kmh(0.50) < 2.0);
        assert!(tobler_kmh(-0.50) < 2.0);
    }

    #[test]
    fn the_tobler_flat_constant_matches_the_curve() {
        // Hand-typed, so pinned: a drift here would silently rescale every
        // Tobler time.
        assert!((TOBLER_FLAT_KMH - tobler_kmh(0.0)).abs() < 1e-12);
    }

    #[test]
    fn tobler_keeps_the_users_flat_speed() {
        let s = WalkSettings {
            model: SpeedModel::Tobler,
            flat_kmh: 4.0,
            load_kg: 0.0,
            ..Default::default()
        };
        assert!((s.slope_kmh(0.0) - 4.0).abs() < 1e-9);
        let stats = compute_stats(&samples(&[1000.0, 1000.0], 8_000.0), &s, true);
        assert!((stats.time_h - 2.0).abs() < 1e-6, "{}", stats.time_h);
    }

    #[test]
    fn tobler_charges_for_the_descent_where_naismith_does_not() {
        let profile = samples(&[2000.0, 1000.0], 5_000.0);
        let flat = |model| WalkSettings {
            model,
            load_kg: 0.0,
            ..Default::default()
        };
        let n = compute_stats(&profile, &flat(SpeedModel::Naismith), true);
        let t = compute_stats(&profile, &flat(SpeedModel::Tobler), true);
        // 1000 m down over 5 km: Naismith sees pure flat distance.
        assert!((n.time_h - 1.0).abs() < 1e-6, "{}", n.time_h);
        assert!(t.time_h > n.time_h, "tobler {} naismith {}", t.time_h, n.time_h);
    }

    #[test]
    fn the_load_only_divides_the_base_time() {
        let profile = samples(&[1000.0, 1600.0], 5_000.0);
        let light = WalkSettings {
            load_kg: 0.0,
            ..Default::default()
        };
        let heavy = WalkSettings {
            load_kg: 20.0,
            ..Default::default()
        };
        let a = compute_stats(&profile, &light, true);
        let b = compute_stats(&profile, &heavy, true);
        assert!((a.base_time_h - b.base_time_h).abs() < 1e-12);
        assert!((b.time_h - a.base_time_h / 0.9).abs() < 1e-9);
    }

    #[test]
    fn a_slice_measures_its_own_span_not_the_track() {
        // A leg starts partway along the track: its distance is a difference.
        let profile = samples(&[1000.0, 1000.0, 1000.0], 10_000.0);
        let s = WalkSettings {
            load_kg: 0.0,
            ..Default::default()
        };
        let stats = compute_stats(&profile[1..], &s, true);
        assert!((stats.distance_m - 5_000.0).abs() < 1e-6);
        assert!((stats.time_h - 1.0).abs() < 1e-6);
    }

    #[test]
    fn legs_are_cut_at_the_waypoints() {
        let mut track = Track::default();
        for lat in [45.0, 45.02, 45.05] {
            track.push(Waypoint::free(LatLon::new(lat, 6.0)));
        }
        let net = TrailNetwork::default();
        track.rebuild_path(&net);
        let (profile, vertex_sample) = sample_polyline(&track.path);
        track.profile = profile;
        track.leg_bounds = track
            .wp_vertex
            .windows(2)
            .map(|w| (vertex_sample[w[0]], vertex_sample[w[1]]))
            .collect();
        track.recompute_time(&WalkSettings::default());

        assert_eq!(track.legs().len(), 2);
        // The legs tile the track, without overlap and without a gap.
        let total: f64 = track.legs().iter().map(|l| l.distance_m).sum();
        assert!((total - track.stats().distance_m).abs() < 1.0, "{total}");
        // The second leg is longer on the ground, so it is longer in time.
        assert!(track.legs()[1].distance_m > track.legs()[0].distance_m);
    }

    #[test]
    fn duration_formatting() {
        assert_eq!(format_duration(2.5), "2 h 30");
        assert_eq!(format_duration(0.0), "—");
    }
}

/// Headless integration against the deployed data: trail tile → snap → segment
/// following → DEM → Naismith. `cargo test -- --ignored`.
#[cfg(test)]
#[cfg(not(target_arch = "wasm32"))]
mod integration {
    use super::*;
    use crate::dem::DemStore;
    use crate::tiles::HttpTileSource;
    use crate::trails::{TrailNetwork, SNAP_RADIUS_M};
    use std::rc::Rc;

    #[test]
    #[ignore = "network"]
    fn a_real_leg_at_chamonix() {
        const BASE: &str = "https://lemuffinman.github.io/TrackFinder/trails/alps/";
        let ctx = egui::Context::default();

        // One published tile, straight over the Chamonix valley.
        let tile = trailfmt::tile_of(45.92, 6.87, trailfmt::TILE_ZOOM);
        let url = format!("{BASE}{}", trailfmt::tile_path(trailfmt::TILE_ZOOM, tile));
        let resp = ehttp::fetch_blocking(&ehttp::Request::get(&url))
            .unwrap_or_else(|e| panic!("{url}: {e}"));
        assert!(resp.ok, "{url}: HTTP {}", resp.status);

        let mut net = TrailNetwork::default();
        let mut synthetic = 0i64;
        for aw in trailfmt::decode_tile(&resp.bytes).expect("tile") {
            if let Some(way) = crate::trails::Way::from_archive(aw, &mut synthetic) {
                net.insert(way);
            }
        }
        assert!(net.len() > 200, "{} ways", net.len());

        // Two points on one long way: the track must follow its geometry, not
        // the chord.
        let way = net
            .ways()
            .iter()
            .max_by_key(|w| w.points.len())
            .expect("non-empty network");
        let (a_pos, b_pos) = (way.points[1], way.points[way.points.len() - 2]);
        let way_id = way.id;

        let a = net.snap(a_pos, SNAP_RADIUS_M).unwrap();
        let b = net.snap(b_pos, SNAP_RADIUS_M).unwrap();
        assert_eq!((a.way_id, b.way_id), (way_id, way_id));

        let mut track = Track::default();
        track.push(Waypoint::snapped(a));
        track.push(Waypoint::snapped(b));

        let settings = WalkSettings::default();
        let mut dem = DemStore::new(Rc::new(HttpTileSource::default()));

        // DEM tiles arrive asynchronously: spin like the render loop would until
        // the profile is complete.
        for _ in 0..200 {
            dem.begin_frame(&ctx);
            track.refresh(&net, &mut dem, &settings, &ctx);
            if track.stats().elevation_complete {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(50));
        }

        let stats = *track.stats();
        assert!(stats.elevation_complete, "DEM still incomplete after 10 s");
        assert!(track.path().len() > 2, "the track must follow the geometry");
        assert!(stats.distance_m > 0.0);
        assert!(
            (500.0..4500.0).contains(&stats.min_elev_m.unwrap()),
            "elevation outside the Alps: {:?}",
            stats.min_elev_m
        );
        assert!(stats.time_h > 0.0);
        assert_eq!(track.followed_legs(&net), 1);
    }
}
