//! Terrain analysis for bivouac spots: slope, aspect, roughness, TPI.
//!
//! ⚠️ **The DEM grid is in degrees, not metres.** At 45°N a degree of longitude
//! is worth about 78 km against 111 km for a degree of latitude: computing a
//! slope from neighbouring pixels would be wrong by a factor of 1.4 east-west,
//! with no visible error whatsoever.
//!
//! The chosen defence is not to correct afterwards (fragile: one line that
//! forgets the distinction is enough), but to **never read neighbouring
//! pixels**. We build a 3×3 window on a **constant metric step** around the
//! point and let `DemStore`'s bilinear interpolation do the rest. The grid
//! becomes square by construction.
//!
//! `wgs84g::pixel_size_m` is therefore no longer a correction but a guard rail:
//! it checks that the chosen step spans several pixels — see the
//! `step_spans_several_pixels` test.
//!
//! This module knows nothing of `egui` or `DemStore`: DEM access arrives as a
//! `FnMut(LatLon, zoom) -> Option<f32>` closure. That is what makes the
//! measurements testable on analytic surfaces, with no network and no rendering
//! context.

use crate::dem::{DEM_ZOOM, GRAPH_DEM_ZOOM};
use crate::geo::LatLon;

/// Step of the 3×3 slope window, in metres.
///
/// At z14 the DEM pixel is ~3.3 m east-west and ~4.8 m north-south at 45.9°N,
/// so a 15 m step spans three to four pixels. Below that we would mostly
/// interpolate noise (RGE ALTI gives ±10° of slope on flat ground); above it we
/// would smooth away the very flat spots we are looking for.
pub const SLOPE_STEP_M: f64 = 15.0;

/// Radius of the TPI ring, in metres.
///
/// TPI answers "ridge or valley floor", a question at the scale of a few
/// hundred metres — not at the scale of a tent pitch.
pub const TPI_RADIUS_M: f64 = 300.0;

/// Number of points on the TPI ring.
///
/// A ring, not a disc: at z11 a 300 m disc would be ~200 pixels for exactly the
/// same signal (the gap between the centre and its surroundings).
pub const TPI_RING_POINTS: usize = 16;

/// Below this slope, aspect is meaningless: `atan2` on a near-zero gradient
/// returns an azimuth dictated by DEM noise.
pub const FLAT_SLOPE_DEG: f32 = 2.0;

/// DEM zoom for slope and aspect: ~5 m/pixel, the fine resolution.
pub const SLOPE_ZOOM: u8 = DEM_ZOOM;

/// DEM zoom for TPI: ~38 m/pixel. A 300 m ring is ~8 pixels there, well
/// resolved, and those tiles are already cached — they are the isochrone's.
pub const TPI_ZOOM: u8 = GRAPH_DEM_ZOOM;

const M_PER_DEG_LAT: f64 = 111_132.0;
const M_PER_DEG_LON_EQ: f64 = 111_320.0;

/// Metric offset → `LatLon`. `east_m` points east, `north_m` points north.
///
/// This is the only place in the module that converts metres to degrees;
/// everything else reasons in metres.
pub fn offset_m(at: LatLon, east_m: f64, north_m: f64) -> LatLon {
    // Near the poles cos(lat) tends to zero; the clamp avoids an infinity.
    // Irrelevant for France, but a silent NaN would propagate all the way to
    // the display.
    let cos_lat = at.lat.to_radians().cos().abs().max(1e-6);
    LatLon::new(
        at.lat + north_m / M_PER_DEG_LAT,
        at.lon + east_m / (M_PER_DEG_LON_EQ * cos_lat),
    )
}

/// Local surface fit over the 3×3 window.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurfaceFit {
    pub slope_deg: f32,
    /// Azimuth of the steepest **descent**, in degrees clockwise from north.
    /// `None` when the ground is near flat.
    pub aspect_deg: Option<f32>,
    /// RMS deviation of the window from its own best-fit plane, in metres.
    ///
    /// Exactly zero on any plane, whatever its tilt. It is what separates a
    /// genuine terrace from a boulder field that happens to average out flat —
    /// and the slope reading alone cannot tell those apart.
    pub roughness_m: f32,
}

/// Raw terrain measurements at a point. No interpretation: the readable verdict
/// sits on top (`flat_chance`, `Position`), so the thresholds can be
/// recalibrated without touching the maths — and so the wind work can cross raw
/// TPI with a forecast later on.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TerrainAnalysis {
    pub elevation_m: f32,
    pub slope_deg: f32,
    pub aspect_deg: Option<f32>,
    pub roughness_m: f32,
    /// Topographic position: elevation of the point minus that of its
    /// surroundings at `TPI_RADIUS_M`. Positive means ridge, negative hollow.
    pub tpi_m: f32,
}

/// Slope, aspect and roughness by Horn's method over a 3×3 window on a constant
/// metric step.
///
/// Horn rather than a two-point central difference: the 1-2-1 weights filter the
/// DEM noise, which otherwise dominates the signal completely at this scale.
/// Horn is exact on a plane, which is what lets the same gradient serve as the
/// reference plane for the roughness residual.
///
/// `None` as soon as a single one of the nine samples is missing — tile not here
/// yet, or edge of coverage. No patching: a slope computed over a holed window
/// is wrong without saying so.
pub fn fit_surface(
    at: LatLon,
    step_m: f64,
    mut sample: impl FnMut(LatLon) -> Option<f32>,
) -> Option<SurfaceFit> {
    // Window indexed [row][column], row 0 to the north, column 0 to the west.
    let mut w = [[0.0f64; 3]; 3];
    for (r, row) in w.iter_mut().enumerate() {
        let north = (1 - r as i32) as f64 * step_m;
        for (c, cell) in row.iter_mut().enumerate() {
            let east = (c as i32 - 1) as f64 * step_m;
            *cell = sample(offset_m(at, east, north))? as f64;
        }
    }

    // Horn: dz/dx counted eastwards, dz/dy northwards.
    let dzdx = ((w[0][2] + 2.0 * w[1][2] + w[2][2]) - (w[0][0] + 2.0 * w[1][0] + w[2][0]))
        / (8.0 * step_m);
    let dzdy = ((w[0][0] + 2.0 * w[0][1] + w[0][2]) - (w[2][0] + 2.0 * w[2][1] + w[2][2]))
        / (8.0 * step_m);

    let slope_deg = dzdx.hypot(dzdy).atan().to_degrees() as f32;

    // The gradient points uphill; aspect is the direction of descent. Azimuth is
    // the angle from north going clockwise, hence atan2(east, north) — not the
    // other way round.
    let aspect_deg = (slope_deg >= FLAT_SLOPE_DEG).then(|| {
        let az = (-dzdx).atan2(-dzdy).to_degrees();
        az.rem_euclid(360.0) as f32
    });

    // Residual against the plane through the window mean carrying that same
    // gradient. Zero on a plane, so it measures only what is not the slope.
    let mean = w.iter().flatten().sum::<f64>() / 9.0;
    let mut sq = 0.0;
    for (r, row) in w.iter().enumerate() {
        let north = (1 - r as i32) as f64 * step_m;
        for (c, value) in row.iter().enumerate() {
            let east = (c as i32 - 1) as f64 * step_m;
            let predicted = mean + dzdx * east + dzdy * north;
            sq += (value - predicted).powi(2);
        }
    }
    let roughness_m = (sq / 9.0).sqrt() as f32;

    Some(SurfaceFit {
        slope_deg,
        aspect_deg,
        roughness_m,
    })
}

/// Topographic position index: elevation of the point minus the mean of a ring
/// of radius `radius_m`.
///
/// ⚠️ The ring is laid out in **metres** through [`offset_m`]. Placed in degrees
/// it would become an ellipse squashed east-west, and TPI would end up measuring
/// the orientation of the valley instead of topographic position.
pub fn tpi(
    at: LatLon,
    radius_m: f64,
    points: usize,
    mut sample: impl FnMut(LatLon) -> Option<f32>,
) -> Option<f32> {
    debug_assert!(points > 0);
    let center = sample(at)? as f64;
    let mut sum = 0.0;
    for k in 0..points {
        let angle = std::f64::consts::TAU * k as f64 / points as f64;
        let (east, north) = (radius_m * angle.sin(), radius_m * angle.cos());
        sum += sample(offset_m(at, east, north))? as f64;
    }
    Some((center - sum / points as f64) as f32)
}

/// Full analysis of a bivouac candidate.
///
/// `sample` receives `(point, zoom)`: slope is read at the fine zoom, TPI at the
/// graph zoom. On the application side that is
/// `|ll, z| dem.elevation_at(ll, z, &ctx)`.
///
/// `None` until the DEM is complete — the caller retries next frame, like the
/// elevation profile does. The tile cache's `request_repaint` makes that happen.
pub fn analyze(
    at: LatLon,
    mut sample: impl FnMut(LatLon, u8) -> Option<f32>,
) -> Option<TerrainAnalysis> {
    let elevation_m = sample(at, SLOPE_ZOOM)?;
    let fit = fit_surface(at, SLOPE_STEP_M, |ll| sample(ll, SLOPE_ZOOM))?;
    let tpi_m = tpi(at, TPI_RADIUS_M, TPI_RING_POINTS, |ll| {
        sample(ll, TPI_ZOOM)
    })?;
    Some(TerrainAnalysis {
        elevation_m,
        slope_deg: fit.slope_deg,
        aspect_deg: fit.aspect_deg,
        roughness_m: fit.roughness_m,
        tpi_m,
    })
}

// ---------------------------------------------------------------------------
// Interpretation — empirical, kept apart from the measurements
// ---------------------------------------------------------------------------

/// Hermite interpolation between two edges, the usual `smoothstep`.
fn smoothstep(edge0: f32, edge1: f32, x: f32) -> f32 {
    let t = ((x - edge0) / (edge1 - edge0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Likelihood of finding ground flat enough to sleep on, in `0..=1`.
///
/// Deliberately **not** a verdict. A 5 m DEM cannot see a two-metre terrace, and
/// smoothing means a reading of 4° over a 15 m window covers everything from a
/// genuine shelf to a slope broken by boulders. Announcing "flat, pitch here"
/// from that would be inventing precision the data does not have — so this
/// returns a continuous confidence that the interface renders as a colour, from
/// red (forget it) to green (worth walking over to look).
///
/// Two independent reasons to lose confidence, multiplied:
/// - the slope itself, fading out between 3° and 18°;
/// - the roughness, which is what tells a smooth shelf from broken ground that
///   merely averages flat. It is the term the slope alone cannot provide.
pub fn flat_chance(slope_deg: f32, roughness_m: f32) -> f32 {
    let from_slope = 1.0 - smoothstep(3.0, 18.0, slope_deg);
    let from_roughness = 1.0 - smoothstep(1.0, 6.0, roughness_m);
    (from_slope * from_roughness).clamp(0.0, 1.0)
}

/// Topographic position, a proxy for wind exposure (a real forecast would refine
/// this; there is no physical simulation here).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Position {
    Ridge,
    Shoulder,
    Even,
    Dip,
    Hollow,
}

impl Position {
    pub fn label(self) -> &'static str {
        match self {
            Self::Ridge => "ridge",
            Self::Shoulder => "shoulder",
            Self::Even => "even slope",
            Self::Dip => "slight dip",
            Self::Hollow => "valley floor",
        }
    }

    /// What the position implies for a night out — the app informs, it does not
    /// decide.
    pub fn hint(self) -> &'static str {
        match self {
            Self::Ridge => "very exposed to wind",
            Self::Shoulder => "exposed to wind",
            Self::Even => "neither sheltered nor exposed",
            Self::Dip => "sheltered, cool air at night",
            Self::Hollow => "sheltered but cold air and damp",
        }
    }
}

/// Thresholds in metres, for a ring of [`TPI_RADIUS_M`].
pub fn position(tpi_m: f32) -> Position {
    match tpi_m {
        t if t > 15.0 => Position::Ridge,
        t if t > 5.0 => Position::Shoulder,
        t if t < -15.0 => Position::Hollow,
        t if t < -5.0 => Position::Dip,
        _ => Position::Even,
    }
}

/// Compass sector of an azimuth, in eight 45° sectors.
pub fn cardinal(aspect_deg: f32) -> &'static str {
    const SECTORS: [&str; 8] = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"];
    // +22.5 so each sector is centred on its cardinal rather than bounded by it.
    let i = ((aspect_deg / 45.0 + 0.5).floor() as i32).rem_euclid(8) as usize;
    SECTORS[i]
}

/// True for a slope that catches the sun and dries out fast — east through south
/// to west.
pub fn sunny(aspect_deg: f32) -> bool {
    (90.0..=270.0).contains(&aspect_deg)
}

#[cfg(test)]
mod tests {
    use super::*;

    const CHAMONIX: LatLon = LatLon::new(45.92, 6.87);

    /// Analytic inclined plane: elevation = base + gradient, in real metres.
    /// `up_east` / `up_north` = rise in metres per metre travelled.
    fn plane(at: LatLon, base: f32, up_east: f64, up_north: f64) -> impl Fn(LatLon) -> Option<f32> {
        move |ll: LatLon| {
            // Degrees back to metres, the mirror of `offset_m`.
            let cos_lat = at.lat.to_radians().cos();
            let east = (ll.lon - at.lon) * M_PER_DEG_LON_EQ * cos_lat;
            let north = (ll.lat - at.lat) * M_PER_DEG_LAT;
            Some(base + (east * up_east + north * up_north) as f32)
        }
    }

    /// The sampling step must span several DEM pixels, otherwise we interpolate
    /// noise between two identical values. This is the only use of
    /// `pixel_size_m`: checking the choice, not correcting the maths.
    #[test]
    fn step_spans_several_pixels() {
        let (mx, my) = crate::geo::wgs84g::pixel_size_m(CHAMONIX.lat, SLOPE_ZOOM);
        assert!(SLOPE_STEP_M > 3.0 * mx, "mx = {mx}");
        assert!(SLOPE_STEP_M > 2.5 * my, "my = {my}");
        // And TPI must stay resolved at its own, coarser zoom.
        let (_, my11) = crate::geo::wgs84g::pixel_size_m(CHAMONIX.lat, TPI_ZOOM);
        assert!(TPI_RADIUS_M > 5.0 * my11, "my11 = {my11}");
    }

    /// A metric offset must cover the same ground distance on both axes — which
    /// is exactly what the degree grid does not do.
    #[test]
    fn metric_offset_is_isotropic() {
        use crate::geo::haversine_m;
        let e = haversine_m(CHAMONIX, offset_m(CHAMONIX, 1000.0, 0.0));
        let n = haversine_m(CHAMONIX, offset_m(CHAMONIX, 0.0, 1000.0));
        assert!((e - 1000.0).abs() < 5.0, "east = {e}");
        assert!((n - 1000.0).abs() < 5.0, "north = {n}");
        // The same offset expressed in degrees is markedly anisotropic.
        let d_lon = offset_m(CHAMONIX, 1000.0, 0.0).lon - CHAMONIX.lon;
        let d_lat = offset_m(CHAMONIX, 0.0, 1000.0).lat - CHAMONIX.lat;
        assert!(d_lon > d_lat * 1.3, "d_lon = {d_lon}, d_lat = {d_lat}");
    }

    #[test]
    fn flat_ground_has_no_aspect() {
        let fit =
            fit_surface(CHAMONIX, SLOPE_STEP_M, plane(CHAMONIX, 2000.0, 0.0, 0.0)).unwrap();
        assert!(fit.slope_deg < 1e-3, "slope = {}", fit.slope_deg);
        assert_eq!(fit.aspect_deg, None);
    }

    /// A 30% slope facing south: the ground rises to the north, so it "looks"
    /// south → azimuth 180.
    #[test]
    fn south_facing_slope() {
        let fit =
            fit_surface(CHAMONIX, SLOPE_STEP_M, plane(CHAMONIX, 2000.0, 0.0, 0.5)).unwrap();
        assert!((fit.slope_deg - 26.565).abs() < 0.05, "slope = {}", fit.slope_deg);
        let a = fit.aspect_deg.unwrap();
        assert!((a - 180.0).abs() < 0.5, "aspect = {a}");
        assert_eq!(cardinal(a), "S");
        assert!(sunny(a));
    }

    /// All four cardinals, one each: this is where the arguments of `atan2` get
    /// swapped without anyone noticing.
    #[test]
    fn the_four_cardinals() {
        // (rise east, rise north) → expected descent azimuth
        let cases = [
            (0.0, 0.5, 180.0, "S"),  // rises north → faces south
            (0.0, -0.5, 0.0, "N"),   // rises south → faces north
            (0.5, 0.0, 270.0, "W"),  // rises east  → faces west
            (-0.5, 0.0, 90.0, "E"),  // rises west  → faces east
        ];
        for (ue, un, expected, card) in cases {
            let fit =
                fit_surface(CHAMONIX, SLOPE_STEP_M, plane(CHAMONIX, 2000.0, ue, un)).unwrap();
            let a = fit.aspect_deg.unwrap();
            assert!(
                (a - expected).abs() < 0.5,
                "rise ({ue}, {un}) → {a}, expected {expected}"
            );
            assert_eq!(cardinal(a), card);
        }
    }

    /// Slope must not depend on the plane's orientation: had the degrees to
    /// metres conversion been skipped, an east-west slope would come out ~1.4×
    /// steeper than a north-south one at this latitude.
    #[test]
    fn slope_is_independent_of_orientation() {
        let mut slopes = Vec::new();
        for k in 0..8 {
            let angle = std::f64::consts::TAU * k as f64 / 8.0;
            let fit = fit_surface(
                CHAMONIX,
                SLOPE_STEP_M,
                plane(CHAMONIX, 2000.0, 0.5 * angle.sin(), 0.5 * angle.cos()),
            )
            .unwrap();
            slopes.push(fit.slope_deg);
        }
        let min = slopes.iter().copied().fold(f32::MAX, f32::min);
        let max = slopes.iter().copied().fold(f32::MIN, f32::max);
        assert!((max - min) < 0.05, "slopes = {slopes:?}");
    }

    /// A missing sample (edge of coverage) invalidates the whole window.
    #[test]
    fn a_hole_in_the_window_yields_no_slope() {
        let mut n = 0;
        let r = fit_surface(CHAMONIX, SLOPE_STEP_M, |_| {
            n += 1;
            (n != 5).then_some(2000.0)
        });
        assert_eq!(r, None);
    }

    /// Roughness measures what is *not* the slope: it must be zero on any
    /// plane, however steep. Otherwise a steep-but-smooth face would be
    /// penalised twice.
    #[test]
    fn a_plane_has_no_roughness() {
        for (ue, un) in [(0.0, 0.0), (0.0, 0.5), (0.4, -0.3), (1.2, 0.0)] {
            let fit =
                fit_surface(CHAMONIX, SLOPE_STEP_M, plane(CHAMONIX, 2000.0, ue, un)).unwrap();
            assert!(
                fit.roughness_m < 0.01,
                "rise ({ue}, {un}) → roughness {}",
                fit.roughness_m
            );
        }
    }

    /// Broken ground averaging out flat: the slope reading says "flat", the
    /// roughness says "do not trust it". That gap is the whole point.
    #[test]
    fn broken_ground_raises_roughness() {
        // ±4 m checkerboard on an otherwise level surface.
        let mut n = 0;
        let fit = fit_surface(CHAMONIX, SLOPE_STEP_M, |_| {
            n += 1;
            Some(2000.0 + if n % 2 == 0 { 4.0 } else { -4.0 })
        })
        .unwrap();
        assert!(fit.slope_deg < 12.0, "slope = {}", fit.slope_deg);
        assert!(fit.roughness_m > 3.0, "roughness = {}", fit.roughness_m);
        // A gentle slope alone would look promising; the roughness pulls it down.
        assert!(
            flat_chance(fit.slope_deg, fit.roughness_m)
                < flat_chance(fit.slope_deg, 0.0) * 0.5
        );
    }

    /// The likelihood must be continuous and monotonic on both inputs — no
    /// threshold anywhere, which is the point of showing a gradient.
    #[test]
    fn flat_chance_falls_with_slope_and_roughness() {
        assert!((flat_chance(0.0, 0.0) - 1.0).abs() < 1e-6);
        assert_eq!(flat_chance(40.0, 0.0), 0.0);
        assert_eq!(flat_chance(0.0, 20.0), 0.0);
        let ramp: Vec<f32> = (0..=20).map(|i| flat_chance(i as f32, 0.5)).collect();
        for w in ramp.windows(2) {
            assert!(w[1] <= w[0] + 1e-6, "not monotonic: {ramp:?}");
        }
        assert!((0.0..=1.0).contains(&flat_chance(7.0, 2.0)));
        // Between the two extremes it must actually vary, not sit at 0 or 1.
        let middle = flat_chance(10.0, 2.0);
        assert!(middle > 0.05 && middle < 0.95, "middle = {middle}");
    }

    /// Summit of a cone: the centre stands above its ring, TPI clearly positive.
    #[test]
    fn tpi_is_positive_on_a_summit() {
        use crate::geo::haversine_m;
        let cone = |ll: LatLon| Some(3000.0 - haversine_m(CHAMONIX, ll) as f32 * 0.3);
        let t = tpi(CHAMONIX, TPI_RADIUS_M, TPI_RING_POINTS, cone).unwrap();
        // 300 m radius × 0.3 slope = 90 m above the ring.
        assert!((t - 90.0).abs() < 1.0, "tpi = {t}");
        assert_eq!(position(t), Position::Ridge);
    }

    /// A bowl: same magnitude, opposite sign.
    #[test]
    fn tpi_is_negative_in_a_bowl() {
        use crate::geo::haversine_m;
        let bowl = |ll: LatLon| Some(2000.0 + haversine_m(CHAMONIX, ll) as f32 * 0.3);
        let t = tpi(CHAMONIX, TPI_RADIUS_M, TPI_RING_POINTS, bowl).unwrap();
        assert!((t + 90.0).abs() < 1.0, "tpi = {t}");
        assert_eq!(position(t), Position::Hollow);
    }

    /// An even slope is neither a ridge nor a valley: an inclined plane must
    /// give a TPI of zero. This is the test that catches a ring laid out in
    /// degrees, which would be an ellipse and would no longer average out.
    #[test]
    fn tpi_is_zero_on_an_even_slope() {
        let t = tpi(
            CHAMONIX,
            TPI_RADIUS_M,
            TPI_RING_POINTS,
            plane(CHAMONIX, 2000.0, 0.2, 0.3),
        )
        .unwrap();
        assert!(t.abs() < 0.5, "tpi = {t}");
        assert_eq!(position(t), Position::Even);
    }

    #[test]
    fn analysis_uses_both_dem_zooms() {
        use std::cell::RefCell;
        let zooms = RefCell::new(Vec::new());
        let a = analyze(CHAMONIX, |ll, z| {
            zooms.borrow_mut().push(z);
            plane(CHAMONIX, 2000.0, 0.0, 0.5)(ll)
        })
        .unwrap();
        assert!((a.elevation_m - 2000.0).abs() < 0.5);
        assert!((a.slope_deg - 26.565).abs() < 0.05);
        assert!((a.aspect_deg.unwrap() - 180.0).abs() < 0.5);
        assert!(a.roughness_m < 0.01);
        assert!(a.tpi_m.abs() < 0.5);
        let z = zooms.borrow();
        assert!(z.contains(&SLOPE_ZOOM) && z.contains(&TPI_ZOOM));
        assert_ne!(SLOPE_ZOOM, TPI_ZOOM);
    }

    #[test]
    fn position_thresholds() {
        assert_eq!(position(0.0), Position::Even);
        assert_eq!(position(-8.0), Position::Dip);
        assert_eq!(position(20.0), Position::Ridge);
        assert_eq!(position(-20.0), Position::Hollow);
    }

    /// Sectors are centred on the cardinals: 350° is still north.
    #[test]
    fn sectors_are_centred_on_the_cardinals() {
        assert_eq!(cardinal(0.0), "N");
        assert_eq!(cardinal(350.0), "N");
        assert_eq!(cardinal(22.0), "N");
        assert_eq!(cardinal(23.0), "NE");
        assert_eq!(cardinal(359.9), "N");
        assert!(!sunny(0.0) && !sunny(350.0) && sunny(180.0));
    }

    /// End to end against the real DEM, natively: `cargo test -- --ignored`.
    /// The Aiguille du Midi is a summit; the analysis has to see that.
    #[test]
    #[ignore = "network"]
    #[cfg(not(target_arch = "wasm32"))]
    fn real_terrain_aiguille_du_midi() {
        use crate::dem::decode_bil;
        use crate::geo::{wgs84g, TILE_PX};
        use crate::tiles::{Dataset, HttpTileSource, TileDesc};
        use std::collections::HashMap;

        const AIGUILLE: LatLon = LatLon::new(45.8785, 6.8873);

        // Tiny local cache: DemStore is asynchronous and tied to egui, so we
        // redo the strict minimum here (blocking fetch + bilinear read).
        let src = HttpTileSource::default();
        let mut tiles: HashMap<(u8, u32, u32), Vec<f32>> = HashMap::new();
        let mut sample = |ll: LatLon, z: u8| -> Option<f32> {
            let span = wgs84g::span_deg(z);
            let side = TILE_PX as f64;
            let gx = (ll.lon + 180.0) / span * side - 0.5;
            let gy = (90.0 - ll.lat) / span * side - 0.5;
            let (i0, j0) = (gx.floor(), gy.floor());
            let (tx, ty) = ((gx - i0) as f32, (gy - j0) as f32);
            let mut v = [0.0f32; 4];
            for (k, (di, dj)) in [(0, 0), (1, 0), (0, 1), (1, 1)].into_iter().enumerate() {
                let (gi, gj) = (i0 as i64 + di, j0 as i64 + dj);
                let key = (
                    z,
                    gi.div_euclid(TILE_PX as i64) as u32,
                    gj.div_euclid(TILE_PX as i64) as u32,
                );
                let values = tiles.entry(key).or_insert_with(|| {
                    let desc = TileDesc {
                        dataset: Dataset::Elevation,
                        z: key.0,
                        x: key.1,
                        y: key.2,
                    };
                    let resp =
                        ehttp::fetch_blocking(&ehttp::Request::get(src.url(desc))).unwrap();
                    assert!(resp.ok, "HTTP {}", resp.status);
                    decode_bil(&resp.bytes).unwrap().values
                });
                let px = gi.rem_euclid(TILE_PX as i64) as usize;
                let py = gj.rem_euclid(TILE_PX as i64) as usize;
                v[k] = values[py * TILE_PX as usize + px];
            }
            let top = v[0] + (v[1] - v[0]) * tx;
            let bottom = v[2] + (v[3] - v[2]) * tx;
            Some(top + (bottom - top) * ty)
        };

        let a = analyze(AIGUILLE, &mut sample).unwrap();
        eprintln!(
            "Aiguille du Midi: {a:?}  {:?}  flat chance {:.2}",
            position(a.tpi_m),
            flat_chance(a.slope_deg, a.roughness_m)
        );
        // Summit at 3842 m, wide tolerance: the DEM smooths rocky spires.
        assert!(
            (3500.0..4000.0).contains(&a.elevation_m),
            "elevation = {}",
            a.elevation_m
        );
        assert!(a.slope_deg > 20.0, "slope = {}", a.slope_deg);
        assert!(a.tpi_m > 100.0, "tpi = {}", a.tpi_m);
        assert_eq!(position(a.tpi_m), Position::Ridge);
        // Nobody pitches a tent on the Aiguille du Midi.
        assert_eq!(flat_chance(a.slope_deg, a.roughness_m), 0.0);
    }
}
