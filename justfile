deployed_base := "https://lemuffinman.github.io/TrackFinder/trails/"
bbox := "43.95,5.35,46.45,7.75"
region_name := "Alpes occidentales"
extracts_dir := "data/extracts"
trails_dir := "data/trails"

default:
    @just --list --unsorted

# ⚠️ `--release` is not optional: the dev profile stutters on pan and zoom, and
# you end up debugging a performance problem that does not exist.
[doc("Serve the app on the local trail tiles")]
serve:
    trunk serve --release

[doc("Serve the app on the deployed archive instead of the local tiles")]
serve-deployed:
    TRACKFINDER_DATA_BASE={{deployed_base}} trunk serve --release

[doc("Run the native build")]
run:
    cargo run --release

# `--public-url` is required, or the page looks for the `.wasm` at the domain root.
[doc("Production build, as the CI does it")]
build repo="TrackFinder":
    trunk build --release --public-url "/{{repo}}/"

[doc("Tests, clippy and the wasm check — what the CI gates on")]
check: test lint wasm

[doc("Unit tests, app and trailfmt")]
test:
    cargo test
    cargo test -p trailfmt

[doc("Clippy, warnings denied")]
lint:
    cargo clippy --all-targets -- -D warnings

[doc("Type-check the build that actually ships")]
wasm:
    cargo check --target wasm32-unknown-unknown

[doc("Tests that hit the real IGN service and the deployed tiles")]
test-net:
    cargo test --release -- --ignored --nocapture

# ⚠️ Meaningless on a loaded machine: a series run next to a release build came
# out 3-4x too high and inverted the ordering of the detail levels.
[doc("Frame cost of the real render path (needs an idle machine)")]
bench:
    @uptime
    cargo test --release -- --ignored frame_cost --nocapture

[doc("Download one Geofabrik extract, e.g. europe/italy/nord-ovest")]
extract region:
    mkdir -p {{extracts_dir}}
    curl -sSL --retry 3 --retry-delay 5 \
        -o "{{extracts_dir}}/$(echo '{{region}}' | tr / -).osm.pbf" \
        "https://download.geofabrik.de/{{region}}-latest.osm.pbf"

# ⚠️ Peak memory follows the walkable ways of every extract, not the box: the
# first two passes read each file whole. Rhône-Alpes alone peaked at 1.08 GB.
[doc("Generate the trail tiles from the extracts in data/extracts")]
trails:
    #!/usr/bin/env bash
    set -euo pipefail
    shopt -s nullglob
    files=({{extracts_dir}}/*.osm.pbf)
    if [ ${#files[@]} -eq 0 ]; then
        echo "no extract in {{extracts_dir}} — run 'just extracts-alps' first" >&2
        exit 1
    fi
    /usr/bin/time -v cargo run --release -p trailprep -- \
        --name "{{region_name}}" --out {{trails_dir}}/alps \
        --bbox {{bbox}} "${files[@]}"
    { printf '{"regions":['; cat {{trails_dir}}/*/region.json | paste -sd,; printf ']}'; } \
        > {{trails_dir}}/regions.json
    echo "$(find {{trails_dir}} -name '*.tft' | wc -l) tiles"

[doc("Download the two extracts covering Chamonix and Mont Thabor (~2 GB peak RSS)")]
extracts-alps:
    just extract europe/france/rhone-alpes
    just extract europe/italy/nord-ovest

[doc("Download the four extracts the CI uses (~4 GB peak RSS)")]
extracts-all: extracts-alps
    just extract europe/france/provence-alpes-cote-d-azur
    just extract europe/switzerland

[doc("Remove the downloaded extracts (~2 GB, only needed while generating)")]
clean-extracts:
    rm -rf {{extracts_dir}}

# The first thing to look at when trails seem to be missing.
[doc("What the local archive holds, and when it was built")]
data-status:
    @echo "tiles:   $(find {{trails_dir}} -name '*.tft' 2>/dev/null | wc -l)"
    @echo "size:    $(du -sh {{trails_dir}} 2>/dev/null | cut -f1)"
    @echo "built:   $(stat -c %y {{trails_dir}}/alps/index.tfi 2>/dev/null || echo 'never')"
    @cat {{trails_dir}}/regions.json 2>/dev/null || echo "manifest: missing"
