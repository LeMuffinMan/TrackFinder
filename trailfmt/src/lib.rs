//! Binary trail archive — the wire format shared by the offline preprocessor and
//! the application.
//!
//! ## Why this exists
//!
//! Measured on 2026-09-04, one 11 × 7.8 km area around Chamonix cost **3.44 MB
//! of Overpass JSON, 740 KB gzipped**, for 2 284 ways and 43 752 points. That is
//! **~80 bytes per point** carrying eight bytes of information, fetched from a
//! rate-limited query engine with no CDN, whose primary instance was refusing
//! connections that day. This format exists to replace that with static bytes on
//! a CDN.
//!
//! ## Layout
//!
//! ```text
//! header  MAGIC(8) | u8 zoom | u8 version | u16 pad | u32 tile_count
//! index   tile_count × { u32 x, u32 y, u64 offset, u32 len }   (sorted by x,y)
//! blobs   one per tile, at its own offset
//! ```
//!
//! The index is small enough to fetch whole (a few kilobytes for the whole
//! Alps), after which every tile is one HTTP range request against a single
//! static file. GitHub Pages answers those with `206` and `accept-ranges: bytes`
//! — verified on the live site before this format was designed.
//!
//! ## Encoding choices
//!
//! - **Coordinates are quantised to [`QUANTUM`] degrees and delta-encoded** as
//!   zigzag varints along each way. Consecutive trail points sit tens of metres
//!   apart, so a delta costs one to two bytes per axis instead of the ~40 bytes
//!   JSON spends writing `{"lat":45.9012345,"lon":6.8712345}`.
//! - **Node identity is a bitmask, not a list of OSM ids.** The graph only needs
//!   to know which points are *shared between ways* — those become junctions.
//!   Storing raw i64 OSM ids would cost five bytes on every point; one bit plus a
//!   small id on the ~10% that are shared costs a fraction of that.
//! - **Way kind and `sac_scale` share one byte**, four bits each. `sac_scale` is
//!   not used yet but its vocabulary is closed and it is free here; leaving it
//!   out would mean a format change later for nothing gained now.

#![forbid(unsafe_code)]

pub const MAGIC: &[u8; 8] = b"TFTRAIL1";
pub const VERSION: u8 = 1;

/// Coordinate quantum, in degrees. 1e-6° is ~11 cm of latitude.
///
/// Chosen against the finest zoom the map reaches: at z19 a screen pixel is
/// ~0.3 m, so 11 cm stays sub-pixel and quantisation is never visible. A coarser
/// 1e-5 (~1.1 m) would save roughly a byte per point but show as jitter on
/// zoomed-in geometry.
pub const QUANTUM: f64 = 1e-6;

// ---------------------------------------------------------------------------
// Shared vocabulary
// ---------------------------------------------------------------------------

/// Way classes. The numbering is part of the wire format: the preprocessor
/// writes it and the application reads it, so **never renumber** — append.
pub mod kind {
    pub const ROAD: u8 = 0;
    pub const PATH: u8 = 1;
    pub const TRACK: u8 = 2;
    pub const FOOTWAY: u8 = 3;
    pub const STEPS: u8 = 4;
    pub const CYCLEWAY: u8 = 5;
}

/// OSM `highway` value → way class, or `None` for values we do not carry.
///
/// This single function decides what ends up in the archive at all. Keeping it
/// here rather than in the preprocessor is deliberate: the application must
/// classify exactly the same way, or the colours and the graph would disagree
/// with the data.
pub fn kind_from_highway(tag: &str) -> Option<u8> {
    Some(match tag {
        "path" | "bridleway" => kind::PATH,
        "track" => kind::TRACK,
        "footway" | "pedestrian" => kind::FOOTWAY,
        "steps" => kind::STEPS,
        "cycleway" => kind::CYCLEWAY,
        "living_street" | "unclassified" | "residential" | "tertiary" => kind::ROAD,
        _ => return None,
    })
}

/// OSM `sac_scale` → 1..=6 in order of difficulty, 0 when absent or unknown.
pub fn sac_from_tag(tag: &str) -> u8 {
    match tag {
        "hiking" => 1,
        "mountain_hiking" => 2,
        "demanding_mountain_hiking" => 3,
        "alpine_hiking" => 4,
        "demanding_alpine_hiking" => 5,
        "difficult_alpine_hiking" => 6,
        _ => 0,
    }
}

/// Zoom of the data grid. At 45°N a z11 tile is ~13.7 km across, so a 30 km
/// working radius is a 5 × 5 block — few enough requests, small enough that the
/// edges are not mostly waste.
pub const TILE_ZOOM: u8 = 11;

/// 1..=6 back to the OSM `sac_scale` value, for display.
pub fn sac_label(sac: u8) -> Option<&'static str> {
    Some(match sac {
        1 => "hiking",
        2 => "mountain_hiking",
        3 => "demanding_mountain_hiking",
        4 => "alpine_hiking",
        5 => "demanding_alpine_hiking",
        6 => "difficult_alpine_hiking",
        _ => return None,
    })
}

/// A tile of the data grid, on the Web Mercator pyramid.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct TileId {
    pub x: u32,
    pub y: u32,
}

/// Web Mercator tile holding this point.
///
/// ⚠️ This is the **PM** grid, the one the base map uses — never the WGS84
/// geographic grid of the DEM. The two are kept apart everywhere in this project
/// precisely because mixing them produces plausible-looking results that are
/// kilometres off.
///
/// It lives here rather than in the application because the tiling scheme *is*
/// part of the format: the preprocessor and the reader must agree on it exactly.
pub fn tile_of(lat: f64, lon: f64, zoom: u8) -> TileId {
    use std::f64::consts::PI;
    let n = (1u64 << zoom) as f64;
    let lat = lat.clamp(-85.051_128_779_806_59, 85.051_128_779_806_59);
    let phi = lat.to_radians();
    let x = (lon + 180.0) / 360.0 * n;
    let y = (1.0 - (phi.tan() + 1.0 / phi.cos()).ln() / PI) / 2.0 * n;
    TileId {
        x: x.floor().clamp(0.0, n - 1.0) as u32,
        y: y.floor().clamp(0.0, n - 1.0) as u32,
    }
}

/// One way as it travels on the wire.
#[derive(Clone, Debug, PartialEq)]
pub struct ArchiveWay {
    pub id: i64,
    /// Way class, 0..=15 — see `WayKind` on the application side.
    pub kind: u8,
    /// `sac_scale`, 0 = absent, 1..=6 = the OSM scale in order.
    pub sac: u8,
    pub name: Option<String>,
    /// `(lat, lon)` pairs.
    pub points: Vec<(f64, f64)>,
    /// One entry per point, same length as `points`: `Some(id)` when the node is
    /// shared with another way (a potential junction), `None` for a plain shape
    /// point.
    ///
    /// ⚠️ **This is the real OSM node id, and it has to be.** A per-archive
    /// counter would be smaller, but two archives would then reuse the same
    /// small numbers for unrelated nodes: loading a neighbouring region
    /// alongside this one would fuse junctions hundreds of kilometres apart, and
    /// nothing would report an error — the isochrone would simply teleport.
    /// Real ids are globally unique, stable across regenerations, and make
    /// junctions on a region boundary stitch together for free.
    pub shared: Vec<Option<i64>>,
}

impl ArchiveWay {
    /// A way is only meaningful if both arrays line up and it has a length.
    ///
    /// ⚠️ The same trap as Overpass's `nodes` / `geometry` alignment: without
    /// this check a mismatch would silently attach the wrong identity to a point
    /// and invent junctions that do not exist.
    pub fn is_valid(&self) -> bool {
        self.points.len() >= 2 && self.points.len() == self.shared.len()
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum FormatError {
    BadMagic,
    BadVersion(u8),
    Truncated,
    /// A way whose point and identity arrays disagree — refused rather than
    /// guessed at.
    Misaligned,
}

impl std::fmt::Display for FormatError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::BadMagic => write!(f, "not a TrackFinder trail archive"),
            Self::BadVersion(v) => write!(f, "unsupported archive version {v}"),
            Self::Truncated => write!(f, "archive data ends mid-record"),
            Self::Misaligned => write!(f, "way point and identity arrays disagree"),
        }
    }
}

// ---------------------------------------------------------------------------
// Varints
// ---------------------------------------------------------------------------

fn put_uvarint(out: &mut Vec<u8>, mut v: u64) {
    while v >= 0x80 {
        out.push((v as u8) | 0x80);
        v >>= 7;
    }
    out.push(v as u8);
}

fn zigzag(v: i64) -> u64 {
    ((v << 1) ^ (v >> 63)) as u64
}

fn unzigzag(v: u64) -> i64 {
    ((v >> 1) as i64) ^ -((v & 1) as i64)
}

fn put_ivarint(out: &mut Vec<u8>, v: i64) {
    put_uvarint(out, zigzag(v));
}

/// Cursor over a byte slice. Every read is checked: a truncated archive must
/// fail loudly, never produce plausible-looking coordinates.
struct Reader<'a> {
    bytes: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, pos: 0 }
    }

    fn u8(&mut self) -> Result<u8, FormatError> {
        let b = *self.bytes.get(self.pos).ok_or(FormatError::Truncated)?;
        self.pos += 1;
        Ok(b)
    }

    fn take(&mut self, n: usize) -> Result<&'a [u8], FormatError> {
        let end = self.pos.checked_add(n).ok_or(FormatError::Truncated)?;
        let slice = self.bytes.get(self.pos..end).ok_or(FormatError::Truncated)?;
        self.pos = end;
        Ok(slice)
    }

    fn uvarint(&mut self) -> Result<u64, FormatError> {
        let mut result = 0u64;
        let mut shift = 0;
        loop {
            let b = self.u8()?;
            result |= ((b & 0x7f) as u64) << shift;
            if b & 0x80 == 0 {
                return Ok(result);
            }
            shift += 7;
            if shift > 63 {
                return Err(FormatError::Truncated);
            }
        }
    }

    fn ivarint(&mut self) -> Result<i64, FormatError> {
        Ok(unzigzag(self.uvarint()?))
    }

    fn u32le(&mut self) -> Result<u32, FormatError> {
        Ok(u32::from_le_bytes(self.take(4)?.try_into().expect("4 bytes")))
    }

    fn u64le(&mut self) -> Result<u64, FormatError> {
        Ok(u64::from_le_bytes(self.take(8)?.try_into().expect("8 bytes")))
    }
}

fn quantise(deg: f64) -> i64 {
    (deg / QUANTUM).round() as i64
}

fn dequantise(q: i64) -> f64 {
    q as f64 * QUANTUM
}

// ---------------------------------------------------------------------------
// Tile blobs
// ---------------------------------------------------------------------------

/// Encodes the ways of one tile. Invalid ways are dropped rather than encoded:
/// the format must never carry a record the decoder would have to guess at.
pub fn encode_tile(ways: &[ArchiveWay]) -> Vec<u8> {
    let usable: Vec<&ArchiveWay> = ways.iter().filter(|w| w.is_valid()).collect();
    let mut out = Vec::new();
    put_uvarint(&mut out, usable.len() as u64);

    let mut prev_id = 0i64;
    for way in usable {
        put_ivarint(&mut out, way.id - prev_id);
        prev_id = way.id;
        out.push((way.kind & 0x0f) | (way.sac << 4));

        let name = way.name.as_deref().unwrap_or("");
        put_uvarint(&mut out, name.len() as u64);
        out.extend_from_slice(name.as_bytes());

        put_uvarint(&mut out, way.points.len() as u64);

        // Identity: one bit per point, then the ids of the set bits only.
        let mut mask = vec![0u8; way.points.len().div_ceil(8)];
        for (i, s) in way.shared.iter().enumerate() {
            if s.is_some() {
                mask[i / 8] |= 1 << (i % 8);
            }
        }
        out.extend_from_slice(&mask);
        let mut prev_shared = 0i64;
        for id in way.shared.iter().flatten() {
            put_ivarint(&mut out, *id - prev_shared);
            prev_shared = *id;
        }

        // Geometry: first point absolute, the rest as deltas.
        let mut prev = (0i64, 0i64);
        for (i, (lat, lon)) in way.points.iter().enumerate() {
            let q = (quantise(*lat), quantise(*lon));
            if i == 0 {
                put_ivarint(&mut out, q.0);
                put_ivarint(&mut out, q.1);
            } else {
                put_ivarint(&mut out, q.0 - prev.0);
                put_ivarint(&mut out, q.1 - prev.1);
            }
            prev = q;
        }
    }
    out
}

/// Decodes one tile blob.
pub fn decode_tile(bytes: &[u8]) -> Result<Vec<ArchiveWay>, FormatError> {
    let mut r = Reader::new(bytes);
    let count = r.uvarint()? as usize;
    // Not `with_capacity(count)`: a corrupt length would otherwise reserve
    // gigabytes before the first read fails.
    let mut ways = Vec::new();
    let mut prev_id = 0i64;

    for _ in 0..count {
        let id = prev_id + r.ivarint()?;
        prev_id = id;
        let packed = r.u8()?;
        let (kind, sac) = (packed & 0x0f, packed >> 4);

        let name_len = r.uvarint()? as usize;
        let name = if name_len == 0 {
            None
        } else {
            Some(String::from_utf8_lossy(r.take(name_len)?).into_owned())
        };

        let n = r.uvarint()? as usize;
        if n < 2 {
            return Err(FormatError::Misaligned);
        }
        let mask = r.take(n.div_ceil(8))?.to_vec();

        let mut shared = Vec::with_capacity(n);
        let mut prev_shared = 0i64;
        for i in 0..n {
            if mask[i / 8] & (1 << (i % 8)) != 0 {
                prev_shared += r.ivarint()?;
                shared.push(Some(prev_shared));
            } else {
                shared.push(None);
            }
        }

        let mut points = Vec::with_capacity(n);
        let mut prev = (0i64, 0i64);
        for i in 0..n {
            let q = if i == 0 {
                (r.ivarint()?, r.ivarint()?)
            } else {
                (prev.0 + r.ivarint()?, prev.1 + r.ivarint()?)
            };
            prev = q;
            points.push((dequantise(q.0), dequantise(q.1)));
        }

        ways.push(ArchiveWay {
            id,
            kind,
            sac,
            name,
            points,
            shared,
        });
    }
    Ok(ways)
}

// ---------------------------------------------------------------------------
// Archive
// ---------------------------------------------------------------------------

pub const HEADER_LEN: usize = 8 + 1 + 1 + 2 + 4;
pub const INDEX_ENTRY_LEN: usize = 4 + 4 + 8 + 4;

/// Where one tile's bytes live inside the archive.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TileSpan {
    pub tile: TileId,
    pub offset: u64,
    pub len: u32,
}

/// Everything needed to turn a tile request into a byte range.
#[derive(Clone, Debug, Default)]
pub struct Index {
    pub zoom: u8,
    pub spans: Vec<TileSpan>,
}

impl Index {
    pub fn span(&self, tile: TileId) -> Option<TileSpan> {
        self.spans
            .binary_search_by_key(&(tile.x, tile.y), |s| (s.tile.x, s.tile.y))
            .ok()
            .map(|i| self.spans[i])
    }

    /// Bytes to fetch to hold the whole index, known before reading anything.
    pub fn header_and_index_len(tile_count: usize) -> usize {
        HEADER_LEN + tile_count * INDEX_ENTRY_LEN
    }
}

/// Builds a complete archive. `tiles` need not be sorted.
pub fn write_archive(zoom: u8, tiles: &[(TileId, Vec<ArchiveWay>)]) -> Vec<u8> {
    let mut blobs: Vec<(TileId, Vec<u8>)> = tiles
        .iter()
        .map(|(t, ways)| (*t, encode_tile(ways)))
        .filter(|(_, b)| !b.is_empty() && b != &[0u8])
        .collect();
    // Sorted so the index can be searched by bisection, and so neighbouring
    // tiles land next to each other — a block of tiles is then one contiguous
    // range rather than a scattering of them.
    blobs.sort_by_key(|(t, _)| (t.x, t.y));

    let mut out = Vec::new();
    out.extend_from_slice(MAGIC);
    out.push(zoom);
    out.push(VERSION);
    out.extend_from_slice(&[0u8; 2]);
    out.extend_from_slice(&(blobs.len() as u32).to_le_bytes());

    let mut offset = Index::header_and_index_len(blobs.len()) as u64;
    for (tile, blob) in &blobs {
        out.extend_from_slice(&tile.x.to_le_bytes());
        out.extend_from_slice(&tile.y.to_le_bytes());
        out.extend_from_slice(&offset.to_le_bytes());
        out.extend_from_slice(&(blob.len() as u32).to_le_bytes());
        offset += blob.len() as u64;
    }
    for (_, blob) in &blobs {
        out.extend_from_slice(blob);
    }
    out
}

/// Reads the header and index. `bytes` may be just the first kilobytes of the
/// archive — that is the point of keeping the index at the front.
pub fn read_index(bytes: &[u8]) -> Result<Index, FormatError> {
    let mut r = Reader::new(bytes);
    if r.take(8)? != MAGIC {
        return Err(FormatError::BadMagic);
    }
    let zoom = r.u8()?;
    let version = r.u8()?;
    if version != VERSION {
        return Err(FormatError::BadVersion(version));
    }
    r.take(2)?;
    let count = r.u32le()? as usize;

    let mut spans = Vec::new();
    for _ in 0..count {
        let x = r.u32le()?;
        let y = r.u32le()?;
        let offset = r.u64le()?;
        let len = r.u32le()?;
        spans.push(TileSpan {
            tile: TileId { x, y },
            offset,
            len,
        });
    }
    Ok(Index { zoom, spans })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn way(id: i64, pts: &[(f64, f64)], shared: &[Option<i64>]) -> ArchiveWay {
        ArchiveWay {
            id,
            kind: 1,
            sac: 2,
            name: Some("Sentier des Aiguilles".to_owned()),
            points: pts.to_vec(),
            shared: shared.to_vec(),
        }
    }

    fn sample() -> ArchiveWay {
        way(
            123_456_789,
            &[
                (45.900_000, 6.870_000),
                (45.900_450, 6.870_310),
                (45.901_020, 6.870_880),
            ],
            // Real OSM node ids: ten-digit, and not consecutive.
            &[Some(1_842_665_301), None, Some(9_874_112_006)],
        )
    }

    #[test]
    fn zigzag_roundtrips_over_the_whole_range() {
        for v in [0i64, 1, -1, 63, -64, i32::MAX as i64, i64::MIN, i64::MAX] {
            assert_eq!(unzigzag(zigzag(v)), v, "{v}");
        }
    }

    #[test]
    fn varints_roundtrip() {
        let mut buf = Vec::new();
        let values = [0u64, 1, 127, 128, 300, u32::MAX as u64, u64::MAX];
        for v in values {
            put_uvarint(&mut buf, v);
        }
        let mut r = Reader::new(&buf);
        for v in values {
            assert_eq!(r.uvarint().unwrap(), v);
        }
    }

    /// Coordinates survive the round trip to within the quantum — that is the
    /// whole precision contract of the format.
    #[test]
    fn a_tile_roundtrips_within_the_quantum() {
        let blob = encode_tile(&[sample()]);
        let back = decode_tile(&blob).unwrap();
        assert_eq!(back.len(), 1);
        let (a, b) = (&sample(), &back[0]);
        assert_eq!(a.id, b.id);
        assert_eq!(a.kind, b.kind);
        assert_eq!(a.sac, b.sac);
        assert_eq!(a.name, b.name);
        assert_eq!(a.shared, b.shared);
        for (p, q) in a.points.iter().zip(&b.points) {
            assert!((p.0 - q.0).abs() <= QUANTUM, "lat {} vs {}", p.0, q.0);
            assert!((p.1 - q.1).abs() <= QUANTUM, "lon {} vs {}", p.1, q.1);
        }
    }

    #[test]
    fn a_way_without_a_name_costs_one_byte() {
        let mut w = sample();
        w.name = None;
        let named = encode_tile(&[sample()]).len();
        let bare = encode_tile(&[w.clone()]).len();
        assert_eq!(named - bare, "Sentier des Aiguilles".len());
        assert_eq!(decode_tile(&encode_tile(&[w])).unwrap()[0].name, None);
    }

    /// Misaligned ways are dropped at encode time, never written out. Encoding
    /// one would push the alignment trap onto the decoder, which has no way to
    /// tell which point carries which identity.
    #[test]
    fn misaligned_ways_never_reach_the_wire() {
        let bad = way(1, &[(45.0, 6.0), (45.1, 6.1)], &[Some(1)]);
        assert!(!bad.is_valid());
        assert_eq!(decode_tile(&encode_tile(&[bad])).unwrap().len(), 0);

        let too_short = way(2, &[(45.0, 6.0)], &[None]);
        assert!(!too_short.is_valid());
        assert_eq!(decode_tile(&encode_tile(&[too_short])).unwrap().len(), 0);
    }

    /// A truncated archive must fail, not hand back plausible coordinates.
    #[test]
    fn truncation_is_an_error_not_garbage() {
        let blob = encode_tile(&[sample()]);
        for cut in 1..blob.len() {
            match decode_tile(&blob[..cut]) {
                Err(FormatError::Truncated | FormatError::Misaligned) => {}
                Err(e) => panic!("cut at {cut}: unexpected {e}"),
                Ok(ways) => assert!(
                    ways.is_empty(),
                    "cut at {cut} produced a way out of thin air: {ways:?}"
                ),
            }
        }
    }

    #[test]
    fn the_archive_index_bisects() {
        let tiles: Vec<(TileId, Vec<ArchiveWay>)> = [(5u32, 9u32), (1, 2), (5, 1)]
            .into_iter()
            .map(|(x, y)| (TileId { x, y }, vec![sample()]))
            .collect();
        let archive = write_archive(11, &tiles);
        let index = read_index(&archive).unwrap();

        assert_eq!(index.zoom, 11);
        assert_eq!(index.spans.len(), 3);
        // Sorted, so neighbouring tiles are adjacent in the file.
        let keys: Vec<(u32, u32)> = index.spans.iter().map(|s| (s.tile.x, s.tile.y)).collect();
        assert_eq!(keys, vec![(1, 2), (5, 1), (5, 9)]);

        for (tile, _) in &tiles {
            let span = index.span(*tile).expect("every tile is findable");
            let blob = &archive[span.offset as usize..][..span.len as usize];
            assert_eq!(decode_tile(blob).unwrap()[0].id, sample().id);
        }
        assert_eq!(index.span(TileId { x: 42, y: 42 }), None);
    }

    /// The index can be read from a prefix of the file: that is what makes one
    /// range request per tile possible.
    #[test]
    fn the_index_is_readable_from_a_prefix() {
        let tiles = vec![(TileId { x: 1, y: 1 }, vec![sample()])];
        let archive = write_archive(11, &tiles);
        let prefix_len = Index::header_and_index_len(1);
        let index = read_index(&archive[..prefix_len]).unwrap();
        assert_eq!(index.spans.len(), 1);
        assert!(prefix_len < archive.len(), "the prefix is not the whole file");
    }

    #[test]
    fn a_foreign_file_is_refused() {
        assert_eq!(
            read_index(b"not an archive at all").unwrap_err(),
            FormatError::BadMagic
        );
        assert_eq!(read_index(b"TFTRAIL1").unwrap_err(), FormatError::Truncated);
        let mut archive = write_archive(11, &[(TileId { x: 0, y: 0 }, vec![sample()])]);
        archive[9] = 99;
        assert_eq!(
            read_index(&archive).unwrap_err(),
            FormatError::BadVersion(99)
        );
    }

    /// Empty tiles are not written at all: the Alps are mostly rock and ice, and
    /// an index entry per empty tile would be pure overhead.
    #[test]
    fn empty_tiles_are_dropped() {
        let archive = write_archive(
            11,
            &[
                (TileId { x: 0, y: 0 }, vec![]),
                (TileId { x: 1, y: 0 }, vec![sample()]),
            ],
        );
        let index = read_index(&archive).unwrap();
        assert_eq!(index.spans.len(), 1);
        assert_eq!(index.spans[0].tile, TileId { x: 1, y: 0 });
    }

    /// Two archives built independently must never claim the same identity for
    /// different nodes — that is what real OSM ids buy, and it is the whole
    /// reason for not using a compact per-archive counter.
    #[test]
    fn node_identity_is_global_not_per_archive() {
        let alps = way(
            1,
            &[(45.9, 6.87), (45.91, 6.88)],
            &[Some(1_842_665_301), Some(1_842_665_302)],
        );
        let pyrenees = way(
            2,
            &[(42.8, 0.15), (42.81, 0.16)],
            &[Some(4_411_009_877), Some(4_411_009_878)],
        );
        let a = decode_tile(&encode_tile(&[alps])).unwrap();
        let p = decode_tile(&encode_tile(&[pyrenees])).unwrap();
        let ids_a: Vec<i64> = a[0].shared.iter().flatten().copied().collect();
        let ids_p: Vec<i64> = p[0].shared.iter().flatten().copied().collect();
        assert!(
            ids_a.iter().all(|x| !ids_p.contains(x)),
            "identities collide across archives: {ids_a:?} vs {ids_p:?}"
        );
        // And ids beyond u32 survive: OSM passed four billion nodes long ago.
        assert!(ids_p.iter().any(|id| *id > u32::MAX as i64));
    }

    /// The class vocabulary is on the wire, so it must not drift.
    #[test]
    fn the_class_vocabulary_is_stable() {
        assert_eq!(kind_from_highway("path"), Some(kind::PATH));
        assert_eq!(kind_from_highway("bridleway"), Some(kind::PATH));
        assert_eq!(kind_from_highway("residential"), Some(kind::ROAD));
        assert_eq!(kind_from_highway("motorway"), None, "not walkable, not carried");
        assert_eq!(kind_from_highway(""), None);
        // Four bits each: the packed byte must survive the round trip.
        assert!(kind::CYCLEWAY <= 0x0f && sac_from_tag("difficult_alpine_hiking") <= 0x0f);
        assert_eq!(sac_from_tag("mountain_hiking"), 2);
        assert_eq!(sac_from_tag("nonsense"), 0);
        // The label must round-trip, or the hazard overlay would show nonsense.
        for tag in [
            "hiking",
            "mountain_hiking",
            "demanding_mountain_hiking",
            "alpine_hiking",
            "demanding_alpine_hiking",
            "difficult_alpine_hiking",
        ] {
            assert_eq!(sac_label(sac_from_tag(tag)), Some(tag));
        }
        assert_eq!(sac_label(0), None);
    }

    /// Every class and every sac value must survive the packed byte.
    #[test]
    fn kind_and_sac_share_a_byte_without_collision() {
        for k in 0..=15u8 {
            for sac in 0..=15u8 {
                let mut w = sample();
                w.kind = k;
                w.sac = sac;
                let back = &decode_tile(&encode_tile(&[w])).unwrap()[0];
                assert_eq!((back.kind, back.sac), (k, sac));
            }
        }
    }

    /// Anchored on the tile that was actually fetched from the WMTS service on
    /// 2026-09-02 (z14, col 8504, row 5833, over the Chamonix valley): its own
    /// centre must map back to it.
    ///
    /// ⚠️ Chamonix itself (45.92) is **not** in that tile — row 5833 spans
    /// 45.9224 to 45.9370, so the village sits just south of its lower edge, in
    /// row 5834. Asserting otherwise is the obvious mistake here.
    #[test]
    fn the_tile_grid_matches_the_verified_reference() {
        let n = (1u64 << 14) as f64;
        // Centre of the reference tile, back through the inverse projection.
        let t = (5833.0 + 0.5) / n;
        let lat = (std::f64::consts::PI * (1.0 - 2.0 * t)).sinh().atan().to_degrees();
        let lon = (8504.0 + 0.5) / n * 360.0 - 180.0;
        assert_eq!(tile_of(lat, lon, 14), TileId { x: 8504, y: 5833 });

        assert_eq!(tile_of(45.92, 6.87, 14), TileId { x: 8504, y: 5834 });
        // Dropping three zoom levels shifts the indices right by three.
        assert_eq!(tile_of(45.92, 6.87, 11), TileId { x: 8504 >> 3, y: 5834 >> 3 });
        // Edges of the world stay in range rather than wrapping.
        assert_eq!(tile_of(0.0, -180.0, 11), TileId { x: 0, y: 1024 });
        assert_eq!(tile_of(89.0, 180.0, 11), TileId { x: 2047, y: 0 });
    }

    /// The point of the whole exercise: bytes per point, against the measured
    /// Overpass baseline of ~80.
    #[test]
    fn bytes_per_point_beat_overpass_by_an_order_of_magnitude() {
        // A realistic trail: 200 points, ~25 m apart, one shared node at each end.
        let mut points = Vec::new();
        let mut shared = Vec::new();
        for i in 0..200 {
            points.push((45.9 + i as f64 * 0.000_225, 6.87 + i as f64 * 0.000_160));
            shared.push(if i == 0 || i == 199 {
                Some(1_842_665_301 + i as i64)
            } else {
                None
            });
        }
        let w = ArchiveWay {
            id: 123_456_789,
            kind: 1,
            sac: 0,
            name: None,
            points,
            shared,
        };
        let bytes = encode_tile(&[w]).len();
        let per_point = bytes as f64 / 200.0;
        eprintln!("{per_point:.2} bytes/point ({bytes} bytes for 200 points)");
        assert!(
            per_point < 8.0,
            "{per_point:.1} bytes/point — Overpass JSON costs ~80"
        );
    }
}
