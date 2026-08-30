#!/usr/bin/env bash
# Copy built Emscripten bundles from the project build trees into public/demo/.
# The bundles are deliberately not tracked in git: they are build output, and
# every rebuild would add a permanent multi-megabyte blob to history.
#
# Build one first, e.g.:
#   source ~/emsdk/emsdk_env.sh
#   cd ../projects/<project> && emcmake cmake -S . -B build-web && cmake --build build-web -j8
set -euo pipefail
cd "$(dirname "$0")"

copied=0
for proj in ../projects/*/; do
    name=$(basename "$proj")
    src="$proj/build-web"
    [ -f "$src/visualizer.wasm" ] || continue
    mkdir -p "public/demo/$name"
    cp "$src"/visualizer.{html,js,wasm,data} "public/demo/$name/" 2>/dev/null || \
        cp "$src"/visualizer.{html,js,wasm} "public/demo/$name/"
    echo "  $name -> public/demo/$name ($(du -sh "public/demo/$name" | cut -f1))"
    copied=$((copied + 1))
done

[ "$copied" -gt 0 ] || { echo "No web build found. Build one first — see the header of this script." >&2; exit 1; }
