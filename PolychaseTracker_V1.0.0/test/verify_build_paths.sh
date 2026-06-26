#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Peter Mercell
#
# verify_build_paths.sh — confirm the two "VERIFY ON BUILD" SDK-dependent spots
# resolve and compile against the REAL Nuke 17 Linux SDK. This isolates them into
# tiny translation units so you get a definitive yes/no in seconds without a full
# plugin rebuild. (They can't be checked off-box: they need ${NUKE_ROOT}/include.)
#
# Usage:
#   NUKE_ROOT=/opt/Nuke17.0v1 ./verify_build_paths.sh
#   NUKE_ROOT=/opt/Nuke17.0v1 POLYCHASE_NEW_3D=ON ./verify_build_paths.sh   # also check usg
#
# Exit 0 = everything that was checked compiled/resolved.
set -u

NUKE_ROOT="${NUKE_ROOT:-/opt/Nuke17.0v1}"
INC="${NUKE_ROOT}/include"
CXX="${CXX:-g++}"
STD="${STD:-c++17}"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
rc=0

say()  { printf '%s\n' "$*"; }
ok()   { printf '  \033[32m[ ok ]\033[0m %s\n' "$*"; }
bad()  { printf '  \033[31m[FAIL]\033[0m %s\n' "$*"; rc=1; }

say "== verify_build_paths.sh =="
say "NUKE_ROOT = ${NUKE_ROOT}"
if [ ! -d "$INC" ]; then
    bad "include dir not found: $INC  (set NUKE_ROOT correctly)"
    exit 1
fi
ok "include dir present: $INC"
say ""

# ---------------------------------------------------------------------------
# (1) DDImage Row / get scanline read  (tracker_mask2d.cpp::sample_mask2d_frame)
#     Compile the exact idiom: Row row(0,w); ... row[Chan_Alpha] -> const float*.
# ---------------------------------------------------------------------------
say "(1) DDImage Row/get scanline read (tracker_mask2d.cpp)"
cat > "$TMP/row_probe.cpp" <<'CPP'
#include "DDImage/Row.h"
#include "DDImage/Channel.h"
#include "DDImage/Iop.h"
using namespace DD::Image;
// Exercise the exact calls sample_mask2d_frame() makes.
void probe(Iop* m, int y, int w) {
    ChannelSet chans(Mask_Alpha);
    Row row(0, w);
    m->get(y, 0, w, chans, row);
    const float* a = row[Chan_Alpha];   // <-- the line the VERIFY note flags
    (void)a;
}
CPP
if "$CXX" -std="$STD" -fsyntax-only -I"$INC" "$TMP/row_probe.cpp" 2>"$TMP/row.err"; then
    ok "Row(0,w) + Iop::get(...) + row[Chan_Alpha] compiles"
else
    bad "Row/get probe did NOT compile — read the errors below and adjust the loop:"
    sed 's/^/      /' "$TMP/row.err"
fi
say ""

# ---------------------------------------------------------------------------
# (2) usg / USD-abstraction headers (polychase_util.h, PCN_NEW_3D path)
#     Only relevant for a -DPOLYCHASE_NEW_3D=ON build. We (a) check each header
#     resolves, then (b) syntax-check them together.
# ---------------------------------------------------------------------------
if [ "${POLYCHASE_NEW_3D:-OFF}" = "ON" ]; then
    say "(2) usg headers (polychase_util.h PCN_NEW_3D path)"
    HDRS=(
        "DDImage/GeomOp.h"
        "DDImage/GeometryProviderI.h"
        "usg/geom/Stage.h"
        "usg/geom/Prim.h"
        "usg/geom/MeshPrim.h"
        "usg/geom/PointBasedPrim.h"
        "usg/geom/XformCache.h"
        "usg/base/ArrayTypes.h"
    )
    for h in "${HDRS[@]}"; do
        if [ -f "$INC/$h" ]; then ok "exists: $h"
        else bad "missing: $INC/$h  (path differs on this install — fix the #include)"; fi
    done
    cat > "$TMP/usg_probe.cpp" <<'CPP'
#include "DDImage/GeomOp.h"
#include "DDImage/GeometryProviderI.h"
#include "usg/geom/Stage.h"
#include "usg/geom/Prim.h"
#include "usg/geom/MeshPrim.h"
#include "usg/geom/PointBasedPrim.h"
#include "usg/geom/XformCache.h"
#include "usg/base/ArrayTypes.h"
int main() { return 0; }
CPP
    if "$CXX" -std="$STD" -fsyntax-only -I"$INC" "$TMP/usg_probe.cpp" 2>"$TMP/usg.err"; then
        ok "all usg headers parse together"
    else
        bad "usg headers did NOT parse — errors:"
        sed 's/^/      /' "$TMP/usg.err"
    fi
else
    say "(2) usg headers — SKIPPED (set POLYCHASE_NEW_3D=ON to check; classic-3D build doesn't use them)"
fi
say ""

if [ "$rc" -eq 0 ]; then say "ALL CHECKED PATHS OK"; else say "SOME CHECKS FAILED (see above)"; fi
exit "$rc"
