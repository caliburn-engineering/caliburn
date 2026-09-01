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

# Which project build tree publishes to which demo path. The two names differ:
# the merged Ball-Balancer application is built in the linear-analyzer project,
# because that repository is the survivor of the merge and the rename is
# sequenced after the deploy.
#
# Demo paths absent from this map are never written to. demo/linear-analyzer is
# one of them on purpose — it holds the frozen analyzer-only bundle, which the
# merge removed the build path for and which no build tree can regenerate.
declare -A DEMOS=(
    [ball-balancer]=linear-analyzer
)

for demo in "${!DEMOS[@]}"; do
    proj="../projects/${DEMOS[$demo]}"
    src="$proj/build-web"
    if [ ! -f "$src/visualizer.wasm" ]; then
        echo "No web build in $src — see the header of this script." >&2
        exit 1
    fi
    mkdir -p "public/demo/$demo"
    cp "$src"/visualizer.{html,js,wasm,data} "public/demo/$demo/" 2>/dev/null || \
        cp "$src"/visualizer.{html,js,wasm} "public/demo/$demo/"
    echo "  ${DEMOS[$demo]}/build-web -> public/demo/$demo ($(du -sh "public/demo/$demo" | cut -f1))"
done
