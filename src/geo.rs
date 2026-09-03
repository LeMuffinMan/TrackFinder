//! Conversions géographiques.
//!
//! ⚠️ Les deux pyramides de tuiles sont volontairement implémentées séparément
//! (`pm` = Web Mercator pour les fonds, `wgs84g` = WGS84 géographique pour le MNT).
//! Ne PAS les factoriser en une fonction paramétrée : altitudes décalées de
//! plusieurs kilomètres sans erreur visible à la clé.

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

/// Distance orthodromique en mètres (haversine).
pub fn haversine_m(a: LatLon, b: LatLon) -> f64 {
    let phi1 = a.lat.to_radians();
    let phi2 = b.lat.to_radians();
    let dphi = (b.lat - a.lat).to_radians();
    let dlam = (b.lon - a.lon).to_radians();
    let h = (dphi / 2.0).sin().powi(2) + phi1.cos() * phi2.cos() * (dlam / 2.0).sin().powi(2);
    2.0 * EARTH_RADIUS_M * h.sqrt().asin()
}

/// Interpolation linéaire entre deux points (suffisante aux distances d'une étape).
pub fn lerp_latlon(a: LatLon, b: LatLon, t: f64) -> LatLon {
    LatLon::new(a.lat + (b.lat - a.lat) * t, a.lon + (b.lon - a.lon) * t)
}

// ---------------------------------------------------------------------------
// Web Mercator — TILEMATRIXSET=PM — fonds de carte
// ---------------------------------------------------------------------------
pub mod pm {
    use super::{LatLon, PI};

    pub const MAX_LAT: f64 = 85.051_128_779_806_59;

    /// Coordonnées monde normalisées : (0,0) = coin nord-ouest, (1,1) = sud-est.
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

    /// Indices de tuile (col, row) pour un zoom entier.
    #[allow(dead_code)] // le rendu travaille en coordonnées monde ; référence + test
    pub fn tile_of(ll: LatLon, z: u8) -> (u32, u32) {
        let n = (1u64 << z) as f64;
        let (x, y) = latlon_to_world(ll);
        let col = (x * n).floor().clamp(0.0, n - 1.0) as u32;
        let row = (y * n).floor().clamp(0.0, n - 1.0) as u32;
        (col, row)
    }
}

// ---------------------------------------------------------------------------
// WGS84 géographique — TILEMATRIXSET=WGS84G — MNT
// Tuiles 256 px, étendue de 180° au zoom 0 (2 tuiles en largeur, 1 en hauteur).
// ---------------------------------------------------------------------------
pub mod wgs84g {
    use super::LatLon;

    /// Étendue d'une tuile, en degrés, au zoom `z`.
    pub fn span_deg(z: u8) -> f64 {
        180.0 / (1u64 << z) as f64
    }

    /// Indices de tuile (col, row).
    #[allow(dead_code)] // `pixel_of` couvre les besoins actuels ; référence + test
    pub fn tile_of(ll: LatLon, z: u8) -> (u32, u32) {
        let span = span_deg(z);
        let col = ((ll.lon + 180.0) / span).floor().max(0.0) as u32;
        let row = ((90.0 - ll.lat) / span).floor().max(0.0) as u32;
        (col, row)
    }

    /// Position en pixels flottants à l'intérieur de la tuile, dans [0, 256[.
    /// `(col, row, px, py)` — px vers l'est, py vers le sud.
    #[allow(dead_code)] // utilisé par DemStore via les coordonnées globales
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

    /// Taille d'un pixel MNT en mètres à cette latitude.
    /// ⚠️ La grille est en degrés : à 45°N, 1° de longitude ≈ 78 km contre 111 km
    /// pour la latitude. Indispensable avant tout calcul de pente / TPI (M3).
    #[allow(dead_code)] // M3 : pente, exposition, TPI
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
    /// Tuiles réellement testées le 02/09/2026 (réponse 200 des deux services).
    const PM_TILE: (u32, u32) = (8504, 5833);
    const WGS84G_TILE: (u32, u32) = (17010, 4012);

    /// Le centre de la tuile PM de référence doit retomber sur cette même tuile,
    /// et se situer sur Chamonix à moins de 3 km.
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

    /// Idem pour la grille du MNT — c'est la formule validée sur la vallée de
    /// Chamonix (altitudes 1043–1827 m sur cette tuile).
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

    /// `pixel_of` et `tile_of` doivent être cohérents, et le pixel rester dans
    /// la tuile.
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

    /// À 45°N un degré de longitude vaut nettement moins qu'un degré de latitude :
    /// c'est tout le piège des calculs de pente du M3.
    #[test]
    fn dem_pixel_is_anisotropic() {
        let (mx, my) = wgs84g::pixel_size_m(45.92, 14);
        assert!(mx < my * 0.75, "mx = {mx}, my = {my}");
        assert!((my - 4.77).abs() < 0.2, "my = {my}");
    }

    #[test]
    fn haversine_known_distance() {
        // 1° de latitude ≈ 111.2 km
        let d = haversine_m(LatLon::new(45.0, 6.0), LatLon::new(46.0, 6.0));
        assert!((d - 111_195.0).abs() < 500.0, "d = {d}");
    }
}
