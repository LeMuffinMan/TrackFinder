//! DEM: fetch → (inflate if needed) → little-endian f32 → elevation at a point.
//!
//! 262,144 useful bytes = 256 × 256 × 4, little-endian. No image decoder involved.
//!
//! ⚠️ The BIL payload's "zlib compression" is really an HTTP
//! `Content-Encoding: deflate` (checked 2026-09-03: plain `curl` returns 215,799
//! zlib bytes, `curl --compressed` returns 262,144 raw ones). The browser's
//! `fetch` decodes that header **transparently and unconditionally**, so under
//! WASM the bytes arrive already inflated. Both forms are accepted.

use std::rc::Rc;

use crate::geo::{wgs84g, LatLon, TILE_PX};
use crate::tiles::{Dataset, TileCache, TileDesc, TileSource};

/// DEM zoom for the elevation profile: ~5 m/pixel, resolution validated over
/// Chamonix.
pub const DEM_ZOOM: u8 = 14;

/// DEM zoom used to weight the graph: ~38 m/pixel, but one tile covers ~10 km
/// instead of 1.2 km. A 25 km isochrone radius therefore needs about thirty
/// tiles instead of more than a thousand — that is what makes the isochrone
/// playable without a pre-processing pipeline.
pub const GRAPH_DEM_ZOOM: u8 = 11;

/// Concurrent DEM requests. Its own budget, separate from the raster tiles: a
/// long elevation profile asks for hundreds of tiles at once and must not starve
/// the map of connections.
const MAX_DEM_REQUESTS: usize = 8;

/// Tiles on the edge of coverage carry a very negative sentinel value.
const NODATA_THRESHOLD: f32 = -1000.0;

/// Decoded DEM tiles handled per frame.
///
/// Decoding is a 256 KB copy (plus an inflate when running natively): cheap on
/// its own, but a zoom change can land dozens of tiles in the same frame. The
/// budget spreads that over several frames instead of one visible hitch.
const DEM_DECODE_BUDGET: usize = 12;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct DemKey {
    pub z: u8,
    pub col: u32,
    pub row: u32,
}

pub struct DemTile {
    /// 256 × 256 elevations in metres, row by row from north to south.
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
        // Already decoded by the HTTP layer (the browser case).
        std::borrow::Cow::Borrowed(bytes)
    } else {
        std::borrow::Cow::Owned(
            miniz_oxide::inflate::decompress_to_vec_zlib(bytes)
                .map_err(|e| format!("zlib inflate: {e:?}"))?,
        )
    };
    let expected = DEM_BYTES;
    if raw.len() != expected {
        return Err(format!(
            "unexpected DEM size: {} bytes instead of {expected}",
            raw.len()
        ));
    }
    let (quads, _) = raw.as_chunks::<4>();
    let values = quads.iter().copied().map(f32::from_le_bytes).collect();
    Ok(DemTile { values })
}

pub struct DemStore {
    cache: TileCache<DemKey, DemTile>,
}

impl DemStore {
    pub fn new(source: Rc<dyn TileSource>) -> Self {
        Self {
            cache: TileCache::new(source, 128, MAX_DEM_REQUESTS),
        }
    }

    pub fn begin_frame(&mut self, ctx: &egui::Context) {
        self.cache.tick();
        self.cache
            .pump(DEM_DECODE_BUDGET, ctx, |_, bytes| decode_bil(bytes));
    }

    pub fn end_frame(&mut self) {
        self.cache.evict();
    }

    pub fn stats(&self) -> (usize, usize, usize) {
        self.cache.stats()
    }

    /// DEM requests currently on the wire.
    pub fn in_flight(&self) -> usize {
        self.cache.in_flight()
    }

    /// Elevation of one global DEM pixel (pyramid-wide indices, at the given
    /// zoom). Triggers the fetch of a missing tile.
    fn pixel(&mut self, gi: i64, gj: i64, zoom: u8, ctx: &egui::Context) -> Option<f32> {
        if gj < 0 {
            return None;
        }
        let side = TILE_PX as i64;
        let key = DemKey {
            z: zoom,
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

    /// Bilinearly interpolated elevation at the profile zoom.
    pub fn elevation(&mut self, ll: LatLon, ctx: &egui::Context) -> Option<f32> {
        self.elevation_at(ll, DEM_ZOOM, ctx)
    }

    /// Bilinearly interpolated elevation, including across four tiles.
    /// `None` means the tiles are not here yet (or the point is outside
    /// coverage) — the caller retries next frame, driven by the cache's
    /// `request_repaint`.
    pub fn elevation_at(&mut self, ll: LatLon, zoom: u8, ctx: &egui::Context) -> Option<f32> {
        let span = wgs84g::span_deg(zoom);
        let side = TILE_PX as f64;
        // Pyramid-wide pixel coordinates, shifted onto pixel centres.
        let gx = (ll.lon + 180.0) / span * side - 0.5;
        let gy = (90.0 - ll.lat) / span * side - 0.5;
        let i0 = gx.floor();
        let j0 = gy.floor();
        let tx = (gx - i0) as f32;
        let ty = (gy - j0) as f32;
        let (i0, j0) = (i0 as i64, j0 as i64);

        let v00 = self.pixel(i0, j0, zoom, ctx)?;
        let v10 = self.pixel(i0 + 1, j0, zoom, ctx)?;
        let v01 = self.pixel(i0, j0 + 1, zoom, ctx)?;
        let v11 = self.pixel(i0 + 1, j0 + 1, zoom, ctx)?;

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
        // Browser case: Content-Encoding: deflate already handled by fetch.
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
        assert!(err.contains("unexpected DEM size"), "{err}");
    }

    /// End to end against the real service (native): `cargo test -- --ignored`.
    #[test]
    #[ignore = "network"]
    #[cfg(not(target_arch = "wasm32"))]
    fn real_tile_over_chamonix() {
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
        // Chamonix valley: 1043-1827 m recorded on 2026-09-02.
        assert!((min - 1042.6).abs() < 5.0, "min = {min}");
        assert!((max - 1827.2).abs() < 5.0, "max = {max}");
    }

    #[test]
    fn raw_bytes_are_not_mistaken_for_data() {
        // Reading the BIL payload without inflating would yield noise: fail loudly.
        assert!(decode_bil(&[0u8; 1024]).is_err());
    }
}
