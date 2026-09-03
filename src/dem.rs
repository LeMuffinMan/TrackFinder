//! MNT : fetch → (inflate si besoin) → f32 little-endian → altitude en un point.
//!
//! 262 144 octets utiles = 256 × 256 × 4, little-endian. Aucun décodeur d'image.
//!
//! ⚠️ La « compression zlib » du BIL est en réalité un `Content-Encoding: deflate`
//! HTTP (vérifié le 03/09/2026 : `curl` seul renvoie 215 799 octets zlib,
//! `curl --compressed` renvoie 262 144 octets bruts). Le `fetch` du navigateur
//! décode ce header **de façon transparente et non désactivable** : en WASM les
//! octets arrivent déjà inflatés. On accepte donc les deux formes.

use std::rc::Rc;

use crate::geo::{wgs84g, LatLon, TILE_PX};
use crate::tiles::{Dataset, TileCache, TileDesc, TileSource};

/// Zoom MNT retenu : ~5 m/pixel sous nos latitudes, résolution validée sur Chamonix.
pub const DEM_ZOOM: u8 = 14;

/// Les tuiles en bord de couverture portent une valeur sentinelle très négative.
const NODATA_THRESHOLD: f32 = -1000.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct DemKey {
    pub z: u8,
    pub col: u32,
    pub row: u32,
}

pub struct DemTile {
    /// 256 × 256 altitudes en mètres, ligne par ligne du nord au sud.
    pub values: Vec<f32>,
}

impl DemTile {
    pub fn sample(&self, px: u32, py: u32) -> Option<f32> {
        let v = *self.values.get((py * TILE_PX + px) as usize)?;
        (v > NODATA_THRESHOLD && v.is_finite()).then_some(v)
    }
}

pub const DEM_BYTES: usize = (TILE_PX * TILE_PX) as usize * 4;

pub fn decode_bil(bytes: &[u8]) -> Result<DemTile, String> {
    let raw = if bytes.len() == DEM_BYTES {
        // Déjà décodé par la couche HTTP (cas du navigateur).
        std::borrow::Cow::Borrowed(bytes)
    } else {
        std::borrow::Cow::Owned(
            miniz_oxide::inflate::decompress_to_vec_zlib(bytes)
                .map_err(|e| format!("inflate zlib : {e:?}"))?,
        )
    };
    let expected = DEM_BYTES;
    if raw.len() != expected {
        return Err(format!(
            "taille MNT inattendue : {} octets au lieu de {expected}",
            raw.len()
        ));
    }
    let values = raw
        .chunks_exact(4)
        .map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]]))
        .collect();
    Ok(DemTile { values })
}

pub struct DemStore {
    cache: TileCache<DemKey, DemTile>,
}

impl DemStore {
    pub fn new(source: Rc<dyn TileSource>) -> Self {
        Self {
            cache: TileCache::new(source, 128),
        }
    }

    pub fn begin_frame(&mut self) {
        self.cache.tick();
        self.cache.pump(|_, bytes| decode_bil(bytes));
    }

    pub fn end_frame(&mut self) {
        self.cache.evict();
    }

    pub fn stats(&self) -> (usize, usize, usize) {
        self.cache.stats()
    }

    /// Altitude d'un pixel MNT global (indices sur toute la pyramide, zoom `DEM_ZOOM`).
    /// Déclenche le chargement de la tuile manquante.
    fn pixel(&mut self, gi: i64, gj: i64, ctx: &egui::Context) -> Option<f32> {
        if gj < 0 {
            return None;
        }
        let side = TILE_PX as i64;
        let key = DemKey {
            z: DEM_ZOOM,
            col: gi.div_euclid(side) as u32,
            row: gj.div_euclid(side) as u32,
        };
        let desc = TileDesc {
            dataset: Dataset::Elevation,
            z: key.z,
            x: key.col,
            y: key.row,
        };
        if !self.cache.ensure(&key, desc, ctx) {
            return None;
        }
        let px = gi.rem_euclid(side) as u32;
        let py = gj.rem_euclid(side) as u32;
        self.cache.peek(&key)?.sample(px, py)
    }

    /// Altitude interpolée bilinéairement, y compris à cheval sur quatre tuiles.
    /// `None` = tuiles pas encore là (ou hors couverture) — l'appelant réessaiera
    /// à la frame suivante, le `request_repaint` du cache la provoquera.
    pub fn elevation(&mut self, ll: LatLon, ctx: &egui::Context) -> Option<f32> {
        let span = wgs84g::span_deg(DEM_ZOOM);
        let side = TILE_PX as f64;
        // Coordonnées en pixels sur toute la pyramide, recalées sur les centres.
        let gx = (ll.lon + 180.0) / span * side - 0.5;
        let gy = (90.0 - ll.lat) / span * side - 0.5;
        let i0 = gx.floor();
        let j0 = gy.floor();
        let tx = (gx - i0) as f32;
        let ty = (gy - j0) as f32;
        let (i0, j0) = (i0 as i64, j0 as i64);

        let v00 = self.pixel(i0, j0, ctx)?;
        let v10 = self.pixel(i0 + 1, j0, ctx)?;
        let v01 = self.pixel(i0, j0 + 1, ctx)?;
        let v11 = self.pixel(i0 + 1, j0 + 1, ctx)?;

        let top = v00 + (v10 - v00) * tx;
        let bottom = v01 + (v11 - v01) * tx;
        Some(top + (bottom - top) * ty)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn zlib(values: &[f32]) -> Vec<u8> {
        let mut raw = Vec::with_capacity(values.len() * 4);
        for v in values {
            raw.extend_from_slice(&v.to_le_bytes());
        }
        miniz_oxide::deflate::compress_to_vec_zlib(&raw, 6)
    }

    #[test]
    fn decode_accepts_already_inflated_bytes() {
        // Cas navigateur : Content-Encoding: deflate déjà décodé par fetch.
        let mut values = vec![0.0f32; (TILE_PX * TILE_PX) as usize];
        values[1] = 2400.0;
        let mut raw = Vec::new();
        for v in &values {
            raw.extend_from_slice(&v.to_le_bytes());
        }
        assert_eq!(raw.len(), DEM_BYTES);
        let tile = decode_bil(&raw).unwrap();
        assert_eq!(tile.sample(1, 0), Some(2400.0));
    }

    #[test]
    fn decode_roundtrip() {
        let mut values = vec![0.0f32; (TILE_PX * TILE_PX) as usize];
        values[0] = 1035.0;
        values[(TILE_PX * TILE_PX - 1) as usize] = 1827.5;
        let tile = decode_bil(&zlib(&values)).unwrap();
        assert_eq!(tile.values.len(), 65_536);
        assert_eq!(tile.sample(0, 0), Some(1035.0));
        assert_eq!(tile.sample(255, 255), Some(1827.5));
    }

    #[test]
    fn nodata_is_rejected() {
        let mut values = vec![0.0f32; (TILE_PX * TILE_PX) as usize];
        values[42] = -99999.0;
        let tile = decode_bil(&zlib(&values)).unwrap();
        assert_eq!(tile.sample(42, 0), None);
    }

    #[test]
    fn wrong_size_is_reported() {
        let err = decode_bil(&zlib(&[1.0, 2.0])).err().unwrap();
        assert!(err.contains("taille MNT inattendue"), "{err}");
    }

    /// Bout en bout contre le vrai service (natif) : `cargo test -- --ignored`.
    #[test]
    #[ignore = "réseau"]
    #[cfg(not(target_arch = "wasm32"))]
    fn tuile_reelle_chamonix() {
        use crate::tiles::{Dataset, HttpTileSource, TileDesc};
        let desc = TileDesc {
            dataset: Dataset::Elevation,
            z: DEM_ZOOM,
            x: 17010,
            y: 4012,
        };
        let url = HttpTileSource::default().url(desc);
        let resp = ehttp::fetch_blocking(&ehttp::Request::get(url)).unwrap();
        assert!(resp.ok, "HTTP {}", resp.status);
        let tile = decode_bil(&resp.bytes).unwrap();
        let alts: Vec<f32> = tile.values.iter().copied().filter(|v| *v > -1000.0).collect();
        let min = alts.iter().copied().fold(f32::MAX, f32::min);
        let max = alts.iter().copied().fold(f32::MIN, f32::max);
        // Vallée de Chamonix : 1043-1827 m relevés le 02/09/2026.
        assert!((min - 1042.6).abs() < 5.0, "min = {min}");
        assert!((max - 1827.2).abs() < 5.0, "max = {max}");
    }

    #[test]
    fn raw_bytes_are_not_mistaken_for_data() {
        // Lire la charge BIL sans inflate donnerait du bruit : on doit échouer net.
        assert!(decode_bil(&[0u8; 1024]).is_err());
    }
}
