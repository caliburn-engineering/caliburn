#!/usr/bin/env bash
# Copy built Emscripten bundles from the project build trees into public/demo/.
# Bundles are build output and are gitignored: every rebuild would otherwise add
# a permanent multi-megabyte blob to history.  demo/linear-analyzer is the one
# exception, and this script is why it can be — see its README.
#
# Build one first, e.g.:
#   source ~/emsdk/emsdk_env.sh
#   cd ../projects/<project> && emcmake cmake -S . -B build-web && cmake --build build-web -j8
set -euo pipefail
cd "$(dirname "$0")"

# The published demos, as "<demo path>:<project directory it is built in>".
#
# The two agree again since the rename in #20, but the pair is still written out
# rather than inferred. Inferring it is what this script used to do, and while
# the merged application was still built in the linear-analyzer project that
# published Ball-Balancer *as* the analyzer and destroyed the analyzer bundle
# doing it. The names can drift apart again; the guard should not depend on
# their agreeing.
#
# A demo path absent from this list is never written to. demo/linear-analyzer is
# absent deliberately: it holds the frozen analyzer-only bundle, which the merge
# removed the build path for and no build tree can regenerate.
DEMOS="ball-balancer:ball-balancer"

for entry in $DEMOS; do
    demo=${entry%%:*}
    src="../projects/${entry##*:}/build-web"

    if [ ! -f "$src/visualizer.wasm" ]; then
        echo "No web build in $src — see the header of this script." >&2
        exit 1
    fi

    mkdir -p "public/demo/$demo"
    cp "$src"/visualizer.html "$src"/visualizer.js "$src"/visualizer.wasm "public/demo/$demo/"
    # Only builds that preload files emit a .data alongside the wasm.
    if [ -f "$src/visualizer.data" ]; then
        cp "$src/visualizer.data" "public/demo/$demo/"
    fi

    echo "  $src -> public/demo/$demo ($(du -sh "public/demo/$demo" | cut -f1))"
done
