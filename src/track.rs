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

#[derive(Clone, Copy, Debug)]
pub struct WalkSettings {
    /// Speed on the flat, km/h (Naismith baseline: 5).
    pub flat_kmh: f64,
    /// Ascent rate, m/h (Naismith baseline: 600).
    pub ascent_mh: f64,
    pub body_weight_kg: f64,
    pub pack_weight_kg: f64,
}

impl Default for WalkSettings {
    fn default() -> Self {
        Self {
            flat_kmh: 5.0,
            ascent_mh: 600.0,
            body_weight_kg: 70.0,
            pack_weight_kg: 12.0,
        }
    }
}

impl WalkSettings {
    /// speed_factor = 1 − 0.01 × max(0, pack_weight − 10)
    pub fn speed_factor(&self) -> f64 {
        1.0 - 0.01 * (self.pack_weight_kg - 10.0).max(0.0)
    }

    /// load_limit = body_weight × 0.20
    pub fn load_limit_kg(&self) -> f64 {
        self.body_weight_kg * 0.20
    }

    pub fn overloaded(&self) -> bool {
        self.pack_weight_kg > self.load_limit_kg()
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct TrackStats {
    pub distance_m: f64,
    pub ascent_m: f64,
    pub descent_m: f64,
    pub min_elev_m: Option<f32>,
    pub max_elev_m: Option<f32>,
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
    profile: Vec<Sample>,
    stats: TrackStats,
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

    /// Number of legs that actually follow a trail.
    pub fn followed_legs(&self, net: &TrailNetwork) -> usize {
        self.waypoints
            .windows(2)
            .filter(|w| leg_geometry(net, &w[0], &w[1]).is_some())
            .count()
    }

    fn rebuild_path(&mut self, net: &TrailNetwork) {
        self.path.clear();
        let Some(first) = self.waypoints.first() else {
            return;
        };
        self.path.push(first.pos);
        for pair in self.waypoints.windows(2) {
            match leg_geometry(net, &pair[0], &pair[1]) {
                // The first point of the geometry is already in `path`.
                Some(geom) => self.path.extend(geom.into_iter().skip(1)),
                None => self.path.push(pair[1].pos),
            }
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
        self.profile = sample_polyline(&self.path);
        let mut complete = true;
        for s in &mut self.profile {
            s.elev_m = dem.elevation(s.pos, ctx);
            complete &= s.elev_m.is_some();
        }
        self.stats = compute_stats(&self.profile, settings, complete);
    }

    /// Recomputes the time only (a settings change, without touching the DEM).
    pub fn recompute_time(&mut self, settings: &WalkSettings) {
        self.stats = compute_stats(&self.profile, settings, self.stats.elevation_complete);
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
fn sample_polyline(points: &[LatLon]) -> Vec<Sample> {
    let mut out = Vec::new();
    if points.is_empty() {
        return out;
    }
    let total: f64 = points.windows(2).map(|w| haversine_m(w[0], w[1])).sum();
    let step = (total / MAX_SAMPLES as f64).max(STEP_M);

    let mut dist = 0.0;
    out.push(Sample {
        dist_m: 0.0,
        pos: points[0],
        elev_m: None,
    });
    for w in points.windows(2) {
        let seg = haversine_m(w[0], w[1]);
        if seg <= 0.0 {
            continue;
        }
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
    out
}

fn compute_stats(profile: &[Sample], settings: &WalkSettings, complete: bool) -> TrackStats {
    let mut stats = TrackStats {
        distance_m: profile.last().map(|s| s.dist_m).unwrap_or(0.0),
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

    // Naismith: flat + ascent, corrected by the load factor.
    let factor = settings.speed_factor().max(0.1);
    let flat_h = (stats.distance_m / 1000.0) / settings.flat_kmh.max(0.1);
    let up_h = stats.ascent_m / settings.ascent_mh.max(1.0);
    stats.time_h = (flat_h + up_h) / factor;
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
            pack_weight_kg: 20.0,
            ..Default::default()
        };
        assert!((s.speed_factor() - 0.90).abs() < 1e-9);
        // Below 10 kg, no penalty.
        let light = WalkSettings {
            pack_weight_kg: 8.0,
            ..Default::default()
        };
        assert!((light.speed_factor() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn load_limit_is_twenty_percent() {
        let s = WalkSettings {
            body_weight_kg: 70.0,
            pack_weight_kg: 15.0,
            ..Default::default()
        };
        assert!((s.load_limit_kg() - 14.0).abs() < 1e-9);
        assert!(s.overloaded());
    }

    #[test]
    fn naismith_on_the_flat() {
        let s = WalkSettings {
            pack_weight_kg: 0.0,
            ..Default::default()
        };
        let stats = compute_stats(&samples(&[1000.0, 1000.0], 10_000.0), &s, true);
        assert!((stats.time_h - 2.0).abs() < 1e-6, "{}", stats.time_h);
        assert_eq!(stats.ascent_m, 0.0);
    }

    #[test]
    fn naismith_with_ascent() {
        let s = WalkSettings {
            pack_weight_kg: 0.0,
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
        let out = sample_polyline(&pts);
        assert!(out.len() > 20, "{} samples", out.len());
        for w in out.windows(2) {
            assert!(w[1].dist_m - w[0].dist_m <= STEP_M + 1.0);
        }
        assert!((out.last().unwrap().dist_m - haversine_m(pts[0], pts[1])).abs() < 1.0);
    }

    #[test]
    fn sampling_is_capped() {
        // 100 km: the step widens instead of exploding the number of points.
        let pts = vec![LatLon::new(45.0, 6.0), LatLon::new(45.9, 6.0)];
        let out = sample_polyline(&pts);
        assert!(out.len() <= MAX_SAMPLES + 2, "{} samples", out.len());
    }

    #[test]
    fn duration_formatting() {
        assert_eq!(format_duration(2.5), "2 h 30");
        assert_eq!(format_duration(0.0), "—");
    }
}

/// Headless integration: Overpass → snap → segment following → DEM → Naismith.
/// `cargo test -- --ignored`.
#[cfg(test)]
#[cfg(not(target_arch = "wasm32"))]
mod integration {
    use super::*;
    use crate::dem::DemStore;
    use crate::tiles::HttpTileSource;
    use crate::trails::{tests::fetch_zone_blocking, TrailNetwork, ZoneKey, SNAP_RADIUS_M};
    use std::rc::Rc;

    #[test]
    #[ignore = "network"]
    fn a_real_leg_at_chamonix() {
        let ctx = egui::Context::default();
        let zone = ZoneKey::of(LatLon::new(45.92, 6.87));
        let mut net = TrailNetwork::default();
        for way in fetch_zone_blocking(zone).unwrap() {
            net.insert(way);
        }

        // Two points on one long OSM way: the track must follow its geometry,
        // not the chord.
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
