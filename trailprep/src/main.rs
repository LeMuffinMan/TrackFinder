//! Offline preprocessor: OSM extracts → a directory of binary trail tiles.
//!
//! Run from CI, never at runtime:
//!
//! ```text
//! trailprep --name "Alpes" --out dist/trails/alps --bbox 43.95,5.35,46.45,7.75 \
//!     rhone-alpes-latest.osm.pbf provence-alpes-cote-d-azur-latest.osm.pbf
//! ```
//!
//! `--out` is a **directory**: one file per tile plus an index. See `trailfmt`
//! for why a single archive addressed by byte ranges had to be abandoned.
//!
//! ## Why three passes
//!
//! A way carries node *references*, not coordinates, so nothing can be built
//! until the nodes have been read — and the nodes are useless until we know
//! which ones a walkable way refers to. Hence: collect the ways, then the
//! coordinates they need, then assemble.
//!
//! The occurrence count from the first pass is what makes the graph possible: a
//! node referenced by two or more ways is a **junction**. Working it out here
//! means the archive carries one bit per point, and an id only on the ~10% that
//! are junctions.

use std::collections::HashMap;
use std::path::PathBuf;

use osmpbf::{Element, ElementReader};
use trailfmt::{ArchiveWay, TileId};

/// Quantum used for the intermediate coordinate store, in degrees.
///
/// 1e-7 keeps a full degree inside an `i32` (900 000 000 < 2 147 483 647) and is
/// finer than the archive's own quantum, so nothing is lost twice.
const STORE_QUANTUM: f64 = 1e-7;

struct Candidate {
    id: i64,
    kind: u8,
    sac: u8,
    name: Option<String>,
    refs: Vec<i64>,
}

struct Bbox {
    south: f64,
    west: f64,
    north: f64,
    east: f64,
}

impl Bbox {
    fn contains(&self, lat: f64, lon: f64) -> bool {
        lat >= self.south && lat <= self.north && lon >= self.west && lon <= self.east
    }
}

/// Minimal JSON string escaping — the manifest carries a region name and
/// nothing else exotic, but a stray quote would produce a file the application
/// refuses to parse, on a path nobody watches.
fn json_escape(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out
}

fn main() {
    let mut out = PathBuf::from("trails.tft");
    let mut name = String::from("Region");
    // The French Alps, matching the application's opening view.
    let mut bbox = Bbox {
        south: 43.95,
        west: 5.35,
        north: 46.45,
        east: 7.75,
    };
    let mut inputs: Vec<PathBuf> = Vec::new();

    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--out" => out = PathBuf::from(args.next().expect("--out needs a path")),
            "--name" => name = args.next().expect("--name needs a label"),
            "--bbox" => {
                let raw = args.next().expect("--bbox needs S,W,N,E");
                let v: Vec<f64> = raw
                    .split(',')
                    .map(|p| p.trim().parse().expect("--bbox wants four numbers"))
                    .collect();
                assert_eq!(v.len(), 4, "--bbox wants S,W,N,E");
                bbox = Bbox {
                    south: v[0],
                    west: v[1],
                    north: v[2],
                    east: v[3],
                };
            }
            other => inputs.push(PathBuf::from(other)),
        }
    }
    assert!(!inputs.is_empty(), "give me at least one .osm.pbf extract");

    // --- pass 1: walkable ways, and how often each node is referenced --------
    let mut candidates: Vec<Candidate> = Vec::new();
    // Saturating count: two references already mean "junction", more changes
    // nothing, and a u8 keeps this map a third of the size of a u32 one.
    let mut refs_count: HashMap<i64, u8> = HashMap::new();
    let mut seen_ways: HashMap<i64, ()> = HashMap::new();

    for path in &inputs {
        eprintln!("pass 1: ways from {}", path.display());
        let reader = ElementReader::from_path(path).expect("cannot open extract");
        reader
            .for_each(|element| {
                let Element::Way(way) = element else {
                    return;
                };
                // Extracts overlap at their borders; a way must be kept once.
                if seen_ways.contains_key(&way.id()) {
                    return;
                }
                let mut kind = None;
                let mut sac = 0;
                let mut name = None;
                for (k, v) in way.tags() {
                    match k {
                        "highway" => kind = trailfmt::kind_from_highway(v),
                        "sac_scale" => sac = trailfmt::sac_from_tag(v),
                        "name" => name = Some(v.to_owned()),
                        _ => {}
                    }
                }
                let Some(kind) = kind else {
                    return;
                };
                let refs: Vec<i64> = way.refs().collect();
                if refs.len() < 2 {
                    return;
                }
                for r in &refs {
                    let c = refs_count.entry(*r).or_insert(0);
                    *c = c.saturating_add(1);
                }
                seen_ways.insert(way.id(), ());
                candidates.push(Candidate {
                    id: way.id(),
                    kind,
                    sac,
                    name,
                    refs,
                });
            })
            .expect("failed reading ways");
    }
    eprintln!(
        "  {} walkable ways, {} distinct nodes referenced",
        candidates.len(),
        refs_count.len()
    );

    // --- pass 2: coordinates of exactly those nodes -------------------------
    let mut coords: HashMap<i64, (i32, i32)> = HashMap::with_capacity(refs_count.len());
    for path in &inputs {
        eprintln!("pass 2: nodes from {}", path.display());
        let reader = ElementReader::from_path(path).expect("cannot open extract");
        reader
            .for_each(|element| {
                let (id, lat, lon) = match element {
                    Element::Node(n) => (n.id(), n.lat(), n.lon()),
                    Element::DenseNode(n) => (n.id(), n.lat(), n.lon()),
                    _ => return,
                };
                if refs_count.contains_key(&id) {
                    coords.insert(
                        id,
                        (
                            (lat / STORE_QUANTUM).round() as i32,
                            (lon / STORE_QUANTUM).round() as i32,
                        ),
                    );
                }
            })
            .expect("failed reading nodes");
    }
    eprintln!("  {} node positions resolved", coords.len());

    // --- pass 3: assemble, filter, tile -------------------------------------
    let mut tiles: HashMap<TileId, Vec<ArchiveWay>> = HashMap::new();
    let (mut kept, mut dropped_outside, mut dropped_incomplete) = (0usize, 0usize, 0usize);
    let mut total_points = 0usize;

    for c in &candidates {
        let mut points = Vec::with_capacity(c.refs.len());
        let mut shared = Vec::with_capacity(c.refs.len());
        let mut complete = true;
        for r in &c.refs {
            // A node missing from the extract means the way runs off its edge.
            // Encoding it would bend the geometry through a hole.
            let Some((qlat, qlon)) = coords.get(r) else {
                complete = false;
                break;
            };
            points.push((
                *qlat as f64 * STORE_QUANTUM,
                *qlon as f64 * STORE_QUANTUM,
            ));
            // The real OSM id, deliberately — see `ArchiveWay::shared`. A compact
            // per-run counter would be smaller and would silently fuse junctions
            // between two archives.
            let is_junction = refs_count.get(r).copied().unwrap_or(0) >= 2;
            shared.push(is_junction.then_some(*r));
        }
        if !complete || points.len() < 2 {
            dropped_incomplete += 1;
            continue;
        }
        // Keep a way when any part of it is inside: cutting at the border would
        // sever the network exactly where routing needs it whole.
        if !points.iter().any(|(lat, lon)| bbox.contains(*lat, *lon)) {
            dropped_outside += 1;
            continue;
        }
        let (lat, lon) = points[0];
        let tile = trailfmt::tile_of(lat, lon, trailfmt::TILE_ZOOM);
        kept += 1;
        total_points += points.len();
        tiles.entry(tile).or_default().push(ArchiveWay {
            id: c.id,
            kind: c.kind,
            sac: c.sac,
            name: c.name.clone(),
            points,
            shared,
        });
    }

    // One file per tile, plus an index of which tiles exist. Empty tiles are
    // never written: the Alps are largely rock and ice, and a file per empty
    // tile would be pure overhead on the CDN and in the index.
    let zoom = trailfmt::TILE_ZOOM;
    std::fs::create_dir_all(&out).expect("cannot create the output directory");
    let mut published: Vec<TileId> = Vec::new();
    let mut total_bytes = 0usize;
    for (tile, ways) in &tiles {
        if ways.is_empty() {
            continue;
        }
        let blob = trailfmt::encode_tile(ways);
        let path = out.join(trailfmt::tile_path(zoom, *tile));
        std::fs::create_dir_all(path.parent().expect("tile path has a parent"))
            .expect("cannot create the tile directory");
        std::fs::write(&path, &blob).expect("cannot write a tile");
        total_bytes += blob.len();
        published.push(*tile);
    }
    published.sort_by_key(|t| (t.x, t.y));
    let index = trailfmt::write_index(zoom, &published);
    std::fs::write(out.join(trailfmt::INDEX_FILE), &index).expect("cannot write the index");
    total_bytes += index.len();

    // A one-line manifest fragment beside the archive. The bounding box lives in
    // exactly one place — the command line — instead of being repeated in a
    // hand-written manifest that would drift from what was actually built.
    let dir_name = out
        .file_name()
        .and_then(|f| f.to_str())
        .expect("--out needs a directory name");
    // One line, newline-terminated: the manifest is assembled by joining these
    // fragments, and a missing newline silently glues two regions together.
    let fragment = format!(
        "{}\n",
        format_args!(
            r#"{{"name":"{}","dir":"{}","south":{},"west":{},"north":{},"east":{}}}"#,
            json_escape(&name),
            json_escape(dir_name),
            bbox.south,
            bbox.west,
            bbox.north,
            bbox.east
        )
    );
    let mut fragment_path = out.clone().into_os_string();
    fragment_path.push(".json");
    std::fs::write(&fragment_path, &fragment).expect("cannot write the manifest fragment");

    verify(&out, zoom, &published, kept, total_points);

    eprintln!(
        "kept {kept} ways ({total_points} points) in {} tiles · dropped {dropped_outside} outside, {dropped_incomplete} incomplete",
        published.len()
    );
    eprintln!(
        "{} → {} tiles, {:.1} MB, {:.2} bytes/point",
        out.display(),
        published.len(),
        total_bytes as f64 / 1_048_576.0,
        total_bytes as f64 / total_points.max(1) as f64
    );
}

/// Reads every published tile straight back and checks it holds what we meant
/// to write.
///
/// CI publishes these files without a human ever opening them; a silent encoding
/// bug would reach the application as missing trails, which looks exactly like a
/// region with no paths. Decoding costs a second — refusing to ship bad data is
/// worth far more than that.
fn verify(
    dir: &std::path::Path,
    zoom: u8,
    published: &[TileId],
    expected_ways: usize,
    expected_points: usize,
) {
    let index_bytes = std::fs::read(dir.join(trailfmt::INDEX_FILE)).expect("index unreadable");
    let index = trailfmt::read_index(&index_bytes).expect("the index we just wrote is unreadable");
    assert_eq!(index.zoom, zoom);
    assert_eq!(index.tiles.len(), published.len(), "index and tiles disagree");

    let (mut ways, mut points) = (0usize, 0usize);
    for tile in &index.tiles {
        let path = dir.join(trailfmt::tile_path(zoom, *tile));
        let blob = std::fs::read(&path)
            .unwrap_or_else(|e| panic!("the index names {} but {e}", path.display()));
        for way in trailfmt::decode_tile(&blob).expect("a tile failed to decode") {
            assert!(way.is_valid(), "way {} decoded misaligned", way.id);
            ways += 1;
            points += way.points.len();
        }
    }
    assert_eq!(ways, expected_ways, "ways lost between encode and decode");
    assert_eq!(points, expected_points, "points lost between encode and decode");
    eprintln!(
        "verified: {ways} ways / {points} points read back from {} tiles",
        index.tiles.len()
    );
}
