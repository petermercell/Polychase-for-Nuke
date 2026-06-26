#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Peter Mercell
#
# Developed with assistance from Claude (Anthropic).

# =============================================================================
# build.sh — configure + build PolychaseTracker.so.
# Expects gcc-toolset-13 to be available (Rocky 9). Sources it if not active.
#
# Optional toggles (env vars, forwarded to CMake; both default to the CMake
# default so a plain ./build.sh stays silent + with the Qt progress dialog on):
#   POLYCHASE_DEBUG_LOG=ON    verbose PCN_LOG diagnostics to stdout (default OFF)
#   POLYCHASE_PROGRESS=OFF     Qt-free build; TrackIt progress dialog is a no-op
#   POLYCHASE_NEW_3D=OFF       classic-only build (no usg link); default ON reads
#                              new-3D geometry (GeoCube) on the geo input
#   USG_LIB_NAME=<name>        override Nuke's USD-abstraction lib (no 'lib'/'.so')
#                              if libFnUsdAbstraction.so isn't the right name
#   NUKE_VERSION=17.0v1        Nuke install to build against
#   POLYCHASE_NDK_BUILD_DIR    override the build directory (default ./build)
#
#   e.g.  POLYCHASE_DEBUG_LOG=ON ./build.sh     # build with logging enabled
# =============================================================================
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${POLYCHASE_NDK_BUILD_DIR:-${HERE}/build}"

# ---- Toolchain ----
# Only source gcc-toolset-13 if we're not already inside it. The .enable script
# sets PCP_GCC_TOOLSET_VERSION, which we use as the sentinel.
if [[ "${PCP_GCC_TOOLSET_VERSION:-}" != "13" ]]; then
    if [[ -f /opt/rh/gcc-toolset-13/enable ]]; then
        echo "[build] activating gcc-toolset-13"
        source /opt/rh/gcc-toolset-13/enable
    else
        echo "[build] WARNING: gcc-toolset-13 not found; using system compiler" >&2
    fi
fi

# ---- Optional toggles (only passed through when the env var is set) ----
CMAKE_EXTRA=()
if [[ -n "${POLYCHASE_DEBUG_LOG:-}" ]]; then
    CMAKE_EXTRA+=("-DPOLYCHASE_DEBUG_LOG=${POLYCHASE_DEBUG_LOG}")
    echo "[build] debug logging: ${POLYCHASE_DEBUG_LOG}"
fi
if [[ -n "${POLYCHASE_PROGRESS:-}" ]]; then
    CMAKE_EXTRA+=("-DPOLYCHASE_PROGRESS=${POLYCHASE_PROGRESS}")
    echo "[build] Qt progress: ${POLYCHASE_PROGRESS}"
fi
if [[ -n "${POLYCHASE_NEW_3D:-}" ]]; then
    CMAKE_EXTRA+=("-DPOLYCHASE_NEW_3D=${POLYCHASE_NEW_3D}")
    echo "[build] new-3D (usg): ${POLYCHASE_NEW_3D}"
fi
if [[ -n "${USG_LIB_NAME:-}" ]]; then
    CMAKE_EXTRA+=("-DUSG_LIB_NAME=${USG_LIB_NAME}")
    echo "[build] usg lib name: ${USG_LIB_NAME}"
fi

# ---- Configure ----
echo "[build] configuring -> ${BUILD_DIR}"
cmake -B "${BUILD_DIR}" -S "${HERE}" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DNUKE_VERSION="${NUKE_VERSION:-17.0v1}" \
    "${CMAKE_EXTRA[@]}"

# ---- Build ----
echo "[build] building"
cmake --build "${BUILD_DIR}" -j"$(nproc)"

# ---- Verify ----
SO="${BUILD_DIR}/PolychaseTracker.so"
if [[ -f "${SO}" ]]; then
    echo ""
    echo "[build] OK: ${SO}"
    echo ""
    echo "Activate in Nuke by adding to your ~/.nuke/init.py:"
    echo ""
    echo "    import nuke"
    echo "    nuke.pluginAddPath('${HERE}/plugin')"
    echo ""
else
    echo "[build] FAIL: ${SO} not produced"
    exit 1
fi
