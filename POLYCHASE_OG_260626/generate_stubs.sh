#!/bin/sh

SCRIPT=$(readlink -f "$0")
SCRIPT_DIR=$(dirname "$SCRIPT")

source "$SCRIPT_DIR/.venv/bin/activate"
export PYTHONPATH="$SCRIPT_DIR/build/cpp"

cd "$SCRIPT_DIR/build"

stubgen -m polychase_core &&
    mkdir -p "$SCRIPT_DIR/blender_addon/lib" &&
    mv ./out/polychase_core.pyi "$SCRIPT_DIR/blender_addon/lib/polychase_core.pyi" &&
    rm ./out -rf
