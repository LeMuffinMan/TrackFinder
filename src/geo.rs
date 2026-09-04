//! Geographic conversions.
//!
//! ⚠️ The two tile pyramids are deliberately implemented separately
//! (`pm` = Web Mercator for base maps, `wgs84g` = geographic WGS84 for the DEM).
//! Do NOT merge them into one parameterised function: the payoff is elevations
//! off by several kilometres with no visible error.

use std::f64::consts::PI;

pub const TILE_PX: u32 = 256;
pub const EARTH_RADIUS_M: f64 = 6_371_008.8;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LatLon {
    pub lat: f64,
    pub lon: f64,
}

impl LatLon {
    pub const fn new(lat: f64, lon: f64) -> Self {
        Self { lat, lon }
    }
}

/// Great-circle distance in metres (haversine).
pub fn haversine_m(a: LatLon, b: LatLon) -> f64 {
    let phi1 = a.lat.to_radians();
    let phi2 = b.lat.to_radians();
    let dphi = (b.lat - a.lat).to_radians();
    let dlam = (b.lon - a.lon).to_radians();
    let h = (dphi / 2.0).sin().powi(2) + phi1.cos() * phi2.cos() * (dlam / 2.0).sin().powi(2);
    2.0 * EARTH_RADIUS_M * h.sqrt().asin()
}

/// Linear interpolation between two points (accurate enough over one leg).
pub fn lerp_latlon(a: LatLon, b: LatLon, t: f64) -> LatLon {
    LatLon::new(a.lat + (b.lat - a.lat) * t, a.lon + (b.lon - a.lon) * t)
}

// ---------------------------------------------------------------------------
// Web Mercator — TILEMATRIXSET=PM — base map layers
// ---------------------------------------------------------------------------
pub mod pm {
    use super::{LatLon, PI};

    pub const MAX_LAT: f64 = 85.051_128_779_806_59;

    /// Normalised world coordinates: (0,0) is the north-west corner, (1,1) the
    /// south-east one.
    pub fn latlon_to_world(ll: LatLon) -> (f64, f64) {
        let lat = ll.lat.clamp(-MAX_LAT, MAX_LAT);
        let x = (ll.lon + 180.0) / 360.0;
        let phi = lat.to_radians();
        let y = (1.0 - (phi.tan() + 1.0 / phi.cos()).ln() / PI) / 2.0;
        (x, y)
    }

    pub fn world_to_latlon(x: f64, y: f64) -> LatLon {
        let lon = x * 360.0 - 180.0;
        let n = PI * (1.0 - 2.0 * y);
        let lat = n.sinh().atan().to_degrees();
        LatLon::new(lat, lon)
    }

    /// Tile indices (col, row) at an integer zoom.
    #[allow(dead_code)] // rendering works in world coordinates; reference + test
    pub fn tile_of(ll: LatLon, z: u8) -> (u32, u32) {
        let n = (1u64 << z) as f64;
        let (x, y) = latlon_to_world(ll);
        let col = (x * n).floor().clamp(0.0, n - 1.0) as u32;
        let row = (y * n).floor().clamp(0.0, n - 1.0) as u32;
        (col, row)
    }
}

// ---------------------------------------------------------------------------
// Geographic WGS84 — TILEMATRIXSET=WGS84G — DEM
// 256 px tiles, 180° span at zoom 0 (two tiles wide, one tall).
// ---------------------------------------------------------------------------
pub mod wgs84g {
    use super::LatLon;

    /// Span of one tile, in degrees, at zoom `z`.
    pub fn span_deg(z: u8) -> f64 {
        180.0 / (1u64 << z) as f64
    }

    /// Tile indices (col, row).
    #[allow(dead_code)] // `pixel_of` covers current needs; reference + test
    pub fn tile_of(ll: LatLon, z: u8) -> (u32, u32) {
        let span = span_deg(z);
        let col = ((ll.lon + 180.0) / span).floor().max(0.0) as u32;
        let row = ((90.0 - ll.lat) / span).floor().max(0.0) as u32;
        (col, row)
    }

    /// Floating-point pixel position inside the tile, in [0, 256[.
    /// `(col, row, px, py)` — px points east, py points south.
    #[allow(dead_code)] // used by DemStore through global pyramid coordinates
    pub fn pixel_of(ll: LatLon, z: u8) -> (u32, u32, f64, f64) {
        let span = span_deg(z);
        let fx = (ll.lon + 180.0) / span;
        let fy = (90.0 - ll.lat) / span;
        let col = fx.floor().max(0.0) as u32;
        let row = fy.floor().max(0.0) as u32;
        let px = (fx - fx.floor()) * super::TILE_PX as f64;
        let py = (fy - fy.floor()) * super::TILE_PX as f64;
        (col, row, px, py)
    }

    /// Size of one DEM pixel, in metres, at this latitude.
    ///
    /// ⚠️ The grid is in degrees: at 45°N one degree of longitude is about
    /// 78 km against 111 km for one degree of latitude.
    ///
    /// This is no longer a correction but a guard rail: `terrain` samples on a
    /// constant metric step and uses this size only to check that the step
    /// spans several pixels (see `terrain::step_spans_several_pixels`).
    #[allow(dead_code)] // only called from the `terrain` tests
    pub fn pixel_size_m(lat: f64, z: u8) -> (f64, f64) {
        let deg = span_deg(z) / super::TILE_PX as f64;
        let m_per_deg_lat = 111_132.0;
        let m_per_deg_lon = 111_320.0 * lat.to_radians().cos();
        (deg * m_per_deg_lon, deg * m_per_deg_lat)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CHAMONIX: LatLon = LatLon::new(45.92, 6.87);
    /// Tiles actually exercised on 2026-09-02 (both services answered 200).
    const PM_TILE: (u32, u32) = (8504, 5833);
    const WGS84G_TILE: (u32, u32) = (17010, 4012);

    /// The centre of the reference PM tile must map back to that same tile, and
    /// land within 3 km of Chamonix.
    #[test]
    fn pm_reference_tile() {
        let n = (1u64 << 14) as f64;
        let center = pm::world_to_latlon(
            (PM_TILE.0 as f64 + 0.5) / n,
            (PM_TILE.1 as f64 + 0.5) / n,
        );
        assert_eq!(pm::tile_of(center, 14), PM_TILE);
        assert!(haversine_m(center, CHAMONIX) < 3000.0, "centre = {center:?}");
    }

    /// Same for the DEM grid — this is the formula validated over the Chamonix
    /// valley (elevations 1043–1827 m on that tile).
    #[test]
    fn wgs84g_reference_tile() {
        let span = wgs84g::span_deg(14);
        let center = LatLon::new(
            90.0 - (WGS84G_TILE.1 as f64 + 0.5) * span,
            (WGS84G_TILE.0 as f64 + 0.5) * span - 180.0,
        );
        assert_eq!(wgs84g::tile_of(center, 14), WGS84G_TILE);
        assert!(haversine_m(center, CHAMONIX) < 3000.0, "centre = {center:?}");
    }

    /// `pixel_of` and `tile_of` must agree, and the pixel must stay inside the
    /// tile.
    #[test]
    fn wgs84g_pixel_inside_tile() {
        let (col, row, px, py) = wgs84g::pixel_of(CHAMONIX, 14);
        assert_eq!((col, row), wgs84g::tile_of(CHAMONIX, 14));
        assert!((0.0..256.0).contains(&px) && (0.0..256.0).contains(&py));
    }

    #[test]
    fn pm_roundtrip() {
        let (x, y) = pm::latlon_to_world(CHAMONIX);
        let back = pm::world_to_latlon(x, y);
        assert!((back.lat - CHAMONIX.lat).abs() < 1e-9);
        assert!((back.lon - CHAMONIX.lon).abs() < 1e-9);
    }

    /// At 45°N a degree of longitude is much shorter than a degree of latitude:
    /// that is the whole trap behind the terrain calculations.
    #[test]
    fn dem_pixel_is_anisotropic() {
        let (mx, my) = wgs84g::pixel_size_m(45.92, 14);
        assert!(mx < my * 0.75, "mx = {mx}, my = {my}");
        assert!((my - 4.77).abs() < 0.2, "my = {my}");
    }

    #[test]
    fn haversine_known_distance() {
        // 1° of latitude ≈ 111.2 km
        let d = haversine_m(LatLon::new(45.0, 6.0), LatLon::new(46.0, 6.0));
        assert!((d - 111_195.0).abs() < 500.0, "d = {d}");
    }
}
