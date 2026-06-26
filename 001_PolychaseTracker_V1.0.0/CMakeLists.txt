# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Peter Mercell
#
# Developed with assistance from Claude (Anthropic).

# =============================================================================
# CMakeLists.txt — Polychase-for-Nuke NDK plugin
#
# Builds PolychaseTracker.so by linking libpolychase.a + its vcpkg static deps
# against Nuke 17's DDImage. The optical-flow database is built OUTSIDE Nuke (the
# mvflow_to_db converter), so this plugin only reads a pre-built DB and solves —
# it never calls GenerateOpticalFlowDatabase and doesn't touch OpenCV/Qt directly.
#
# Conventions follow Peter's MegaFlowSolve (_GLIBCXX_USE_CXX11_ABI=1, fPIC,
# fvisibility=hidden) and WhatsThePoint (no-RPATH + patchelf, SYSTEM includes).
# =============================================================================
cmake_minimum_required(VERSION 3.18)
project(polychase_nuke_ndk LANGUAGES CXX)

# C++17. We tried C++20, but Nuke 17's DDImage headers (RefCountedObject.h,
# GeoInfo.h) use the old pre-C++20 in-class constructor syntax
# `ClassName<T>() : ...` which gcc-13 rejects in C++20 mode (the constructor
# name in C++20 must be just `ClassName()`). Those are Foundry's headers, so we
# compile against them in C++17 mode.
#
# Note: the optical-flow database is now built OUTSIDE Nuke (the mvflow_to_db
# converter), so this plugin no longer calls GenerateOpticalFlowDatabase or
# touches OpenCV / Qt directly — it only reads the pre-built DB and solves.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# -----------------------------------------------------------------------------
# RPATH disabled (plugin runs inside Nuke; libs are in-process already)
# -----------------------------------------------------------------------------
set(CMAKE_SKIP_BUILD_RPATH      TRUE)
set(CMAKE_SKIP_INSTALL_RPATH    TRUE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH         "")
set(CMAKE_BUILD_RPATH           "")
set(CMAKE_SHARED_LINKER_FLAGS
    "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--disable-new-dtags -Wl,--no-undefined")

# -----------------------------------------------------------------------------
# Nuke 17 NDK
# -----------------------------------------------------------------------------
if(NOT DEFINED NUKE_VERSION)
    set(NUKE_VERSION "17.0v1")
endif()
set(NUKE_ROOT "/opt/Nuke${NUKE_VERSION}" CACHE PATH "Nuke install root")

if(NOT EXISTS "${NUKE_ROOT}")
    message(FATAL_ERROR "Nuke directory not found: ${NUKE_ROOT}")
endif()
if(NOT EXISTS "${NUKE_ROOT}/libDDImage.so")
    message(FATAL_ERROR "libDDImage.so not found in: ${NUKE_ROOT}")
endif()

# -----------------------------------------------------------------------------
# Polychase library + vcpkg deps (Phase 0.5 addition)
# -----------------------------------------------------------------------------
set(POLYCHASE_ROOT "$ENV{HOME}/Documents/POLYCHASE/polychase"
    CACHE PATH "Polychase source tree root")
set(POLYCHASE_BUILD "${POLYCHASE_ROOT}/build")
set(POLYCHASE_LIB   "${POLYCHASE_BUILD}/cpp/libpolychase.a")
set(POLYCHASE_VCPKG "${POLYCHASE_BUILD}/vcpkg_installed/x64-linux")

if(NOT EXISTS "${POLYCHASE_LIB}")
    message(FATAL_ERROR
        "libpolychase.a not found at ${POLYCHASE_LIB}.\n"
        "Build polychase first, or pass -DPOLYCHASE_ROOT=/path/to/polychase.")
endif()
if(NOT EXISTS "${POLYCHASE_VCPKG}/lib")
    message(FATAL_ERROR
        "vcpkg lib dir not found at ${POLYCHASE_VCPKG}/lib.")
endif()

# TBB: prefer Nuke's libtbb.so.12 (avoids scheduler conflicts when the plugin
# loads into the same process). Fall back to vcpkg's if Nuke's is missing.
set(_NUKE_TBB "${NUKE_ROOT}/libtbb.so.12")
set(_VCPKG_TBB "${POLYCHASE_VCPKG}/lib/libtbb.so.12")
if(EXISTS "${_NUKE_TBB}")
    set(TBB_LIB "${_NUKE_TBB}")
    set(TBB_SOURCE "Nuke")
elseif(EXISTS "${_VCPKG_TBB}")
    set(TBB_LIB "${_VCPKG_TBB}")
    set(TBB_SOURCE "vcpkg (Nuke's not found)")
else()
    message(FATAL_ERROR "Neither Nuke nor vcpkg TBB found.")
endif()

# Embree: use vcpkg's CMake config instead of hand-enumerating .a files. This
# is what polychase's own CMakeLists does (find_package(embree 4 REQUIRED)),
# and it's the only way to get the ISA-archive arrangement right — each ISA
# kernel archive has its own .o files referencing same-namespace symbols
# defined inside the same archive, which requires --whole-archive applied
# per-archive in the right order. The exported `embree` target encodes all
# of that as transitive interface link libraries.
#
# embree-config.cmake calls find_dependency(TBB), and TBBConfig.cmake links
# its tbb target against Threads::Threads — so we must find_package(Threads)
# first to make that imported target exist.
find_package(Threads REQUIRED)
set(embree_DIR "${POLYCHASE_VCPKG}/share/embree")
find_package(embree 4 REQUIRED CONFIG)

# OpenGL: Nuke's libDDImage.so does NOT transitively re-export libGL symbols,
# so any plugin that issues raw GL calls (glBegin/glVertex2f/etc. for the
# viewer overlay) must link OpenGL explicitly. find_package gives us the
# `OpenGL::GL` imported target which resolves to /usr/lib64/libGL.so on
# Linux. Plugin runs inside Nuke whose own viewer is already a GL client,
# so there's no version-mismatch risk.
find_package(OpenGL REQUIRED)

# -----------------------------------------------------------------------------
# PoseLib (vcpkg, in the SAME tree as the other deps). Header + static lib used
# by tracker_focal_pnpf.cpp's from-scratch per-frame focal solve (poselib::p4pf).
# Larsson's BSD-3 PoseLib; pulls only Eigen (already present). The exported config
# lives under ${POLYCHASE_VCPKG}/share/poselib and provides the PoseLib::PoseLib
# imported target. Point find_package at the vcpkg tree explicitly so it resolves
# even without the vcpkg toolchain in CMAKE_PREFIX_PATH.
# -----------------------------------------------------------------------------
set(PoseLib_DIR "${POLYCHASE_VCPKG}/share/poselib")
find_package(PoseLib CONFIG REQUIRED)

# -----------------------------------------------------------------------------
# Optional Qt progress dialog (TrackIt). ON pulls in track_progress.cpp's Qt
# code path + Qt6 HEADERS (matched to Nuke's bundled 6.5.3) and links Nuke's OWN
# Qt6 libs BY NAME, so Qt is HOST-PROVIDED at load time exactly like
# libDDImage.so — never bundled, never shipped. Mirrors Motion2DB's
# MOTION2DB_PROGRESS option.
#
# track_progress.cpp is ALWAYS compiled (it carries a no-op fallback under
# #else), so turning this OFF just drops the Qt headers/libs/define and the
# dialog calls in tracker_track.cpp become silent no-ops — no #ifdef at the
# call sites. The dialog is also a no-op in headless/terminal Nuke regardless.
#
# Reasons to set OFF: (a) build machines without the Qt 6.5.3 SDK headers, or
# (b) reusing this Op in a non-Qt host.
#   cmake ... -DPOLYCHASE_PROGRESS=OFF
# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------
# New (USD/usg) 3D-system support. ON by default for Nuke 17: lets the geo input
# read new-system geometry (GeoCube / any GeomOp) via usg::Stage, in addition to
# the classic GeoOp/GeometryList path. Turn OFF for a classic-only build
# (rollback/bisect):   cmake ... -DPOLYCHASE_NEW_3D=OFF
#
# GeometryProviderI / GeomOp / Op::geomOp() live in libDDImage (already linked).
# The usg DATA API (usg::Stage / MeshPrim / PointBasedPrim / XformCache) lives in
# a separate shipped lib that must be linked explicitly. On macOS the research
# found it in libFnUsdAbstraction.dylib; on Linux it should be the matching .so
# in NUKE_ROOT. The exact name can vary by build — if the check below FATALs, run
#   ls ${NUKE_ROOT}/libFnUsd*.so ${NUKE_ROOT}/libFn*Usd*.so
# and pass -DUSG_LIB_NAME=<name without 'lib' prefix and '.so' suffix>.
# usg headers are under ${NUKE_ROOT}/include/usg (already on the include path).
# -----------------------------------------------------------------------------
option(POLYCHASE_NEW_3D "Read Nuke 17 new-3D (USD/usg) geometry on the geo input" ON)
set(USG_LIB_NAME "FnUsdAbstraction" CACHE STRING "Nuke USD-abstraction lib (no lib/.so)")
if(POLYCHASE_NEW_3D)
    set(_USG_SO "${NUKE_ROOT}/lib${USG_LIB_NAME}.so")
    if(NOT EXISTS "${_USG_SO}")
        message(FATAL_ERROR
            "POLYCHASE_NEW_3D=ON but ${_USG_SO} not found.\n"
            "Find Nuke's USD-abstraction lib:  ls ${NUKE_ROOT}/libFn*Usd*.so\n"
            "then pass -DUSG_LIB_NAME=<name without 'lib' and '.so'>, or build "
            "classic-only with -DPOLYCHASE_NEW_3D=OFF.")
    endif()
    message(STATUS "  New 3D (usg)   : ON  (linking lib${USG_LIB_NAME}.so)")
else()
    message(STATUS "  New 3D (usg)   : OFF (classic GeoOp only)")
endif()

option(POLYCHASE_PROGRESS "Show a Qt progress dialog during TrackIt" ON)
if(POLYCHASE_PROGRESS)
    if(NOT DEFINED Qt6_ROOT)
        set(Qt6_ROOT "/home/pm/Qt6NEW/6.5.3/gcc_64")
    endif()
    set(Qt6_DIR "${Qt6_ROOT}/lib/cmake/Qt6")
    set(CMAKE_PREFIX_PATH "${Qt6_ROOT}" ${CMAKE_PREFIX_PATH})
    find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets)
    message(STATUS "  Qt6 progress   : ON  (headers ${Qt6_ROOT}, libs = Nuke's)")
else()
    message(STATUS "  Qt6 progress   : OFF (Qt-free build; dialog is a no-op)")
endif()

# -----------------------------------------------------------------------------
# Plugin target
# -----------------------------------------------------------------------------
add_library(PolychaseTracker SHARED
    src/PolychaseTracker.cpp   # op core: ctor, knobs, knob_changed, track/export, registration
    src/wireframe_knob.cpp     # PolychaseWireframeKnob::draw_handle (2D viewer overlay)
    src/tracker_pins.cpp       # pin lifecycle, solve, persistence
    src/tracker_pose.cpp       # keyed pose: effective_model_matrix, key/clear, has_pose_keys
    src/tracker_gizmo.cpp      # Phase G0: pose gizmo overlay (draw-only)
    src/tracker_track.cpp      # Phase 1D: Track (TrackSequence) + Apply/Export + Stop
    src/tracker_refine.cpp     # Refine: global bundle adjustment between anchors (RefineTrajectory)
    src/tracker_intrinsics.cpp # Background fixed-pose focal / principal solve (non-destructive to the track)
    src/tracker_focal_pnpf.cpp # From-scratch per-frame focal via PoseLib P4Pf (varying-focal / zoom shots)
    src/tracker_mask.cpp       # 3D mask: per-triangle bitset excluded from the solve (storage + feed)
    src/tracker_mask2d.cpp     # 2D occlusion mask (mask plate): per-frame image-space exclusion + overlay
    src/tracker_usertracks.cpp # User (helper) tracks: ingest Tracker-node 2D tracks, anchor + feed the solve
    src/track_progress.cpp     # Qt TrackIt progress dialog (Qt isolated; no-op when OFF/headless)
)
# Headers (not compiled directly, listed for IDEs / clarity):
#   src/polychase_util.h      — shared types + leaf helpers (namespace pcn)
#   src/polychase_tracker.h   — PolychaseTracker / PolychaseWireframeKnob declarations
#   src/polychase_pose_math.h — DDImage-free TRS compose/decompose (unit-tested)
#   src/pcn_convention.h      — shared GL<->CV convention bridge (Track/Refine/usertracks/intrinsics)
#   src/pcn_scoped_guard.h    — ScopedFlags RAII guard for suppress_*_callback_ flags
#   src/pcn_blob_mirror.h     — BlobMirror: cache/suppress/reload for hidden blobs
#   src/track_progress.h      — Qt-free progress-dialog facade (all Qt in the .cpp)

# SYSTEM suppresses warnings emitted from inside third-party headers.
# Note: the OpenCV include dir is kept because some libpolychase public headers
# may transitively pull <opencv2/...>. Our own code no longer includes OpenCV.
target_include_directories(PolychaseTracker SYSTEM PRIVATE
    "${NUKE_ROOT}/include"
    "${POLYCHASE_ROOT}/cpp"
    "${POLYCHASE_VCPKG}/include"
    "${POLYCHASE_VCPKG}/include/opencv4"
)

# ABI flag MUST match how libpolychase.a was built (same as MegaFlowSolve).
target_compile_definitions(PolychaseTracker PRIVATE
    _GLIBCXX_USE_CXX11_ABI=1
)

# New-3D (usg) read path: PCN_NEW_3D guards all usg includes/code in
# polychase_util.h so a -DPOLYCHASE_NEW_3D=OFF build is classic-only.
if(POLYCHASE_NEW_3D)
    target_compile_definitions(PolychaseTracker PRIVATE PCN_NEW_3D=1)
endif()

# -----------------------------------------------------------------------------
# Diagnostic terminal logging. OFF by default: the plugin runs silent. Turn ON
# to restore the verbose per-frame / per-event prints (routed through PCN_LOG)
# to the terminal Nuke was launched from:
#   cmake ... -DPOLYCHASE_DEBUG_LOG=ON
# (equivalently, flip `#define PCN_DEBUG 0` to 1 in polychase_util.h).
# -----------------------------------------------------------------------------
option(POLYCHASE_DEBUG_LOG "Print diagnostic logging to stdout" OFF)
if(POLYCHASE_DEBUG_LOG)
    target_compile_definitions(PolychaseTracker PRIVATE PCN_DEBUG=1)
    message(STATUS "  Debug logging  : ON  (PCN_LOG -> stdout)")
else()
    message(STATUS "  Debug logging  : OFF (silent; -DPOLYCHASE_DEBUG_LOG=ON to enable)")
endif()

# Qt headers + the code-path switch (only when the progress dialog is enabled).
# The Qt LIBS are linked below, by name, from Nuke. POLYCHASE_HAVE_QT selects
# the real implementation in track_progress.cpp; without it the no-op body is
# compiled. The Qt headers are included only into track_progress.cpp (which pulls
# NO GL/glew), so the Qt<->glew header clash that affects the overlay TUs can't
# happen — same isolation rule as Motion2DB / analyze_progress.
if(POLYCHASE_PROGRESS)
    target_compile_definitions(PolychaseTracker PRIVATE POLYCHASE_HAVE_QT)
    target_include_directories(PolychaseTracker SYSTEM PRIVATE
        $<TARGET_PROPERTY:Qt6::Core,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:Qt6::Gui,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:Qt6::Widgets,INTERFACE_INCLUDE_DIRECTORIES>
    )
endif()

target_compile_options(PolychaseTracker PRIVATE
    -Wall -Wextra
    -Wno-unused-parameter
    -Wno-deprecated-declarations
    -O3
    -fvisibility=hidden
)

# -----------------------------------------------------------------------------
# Link line.
#
# Order matters for static archives: a consumer must precede its providers, or
# the linker forgets the unresolved symbols. Hence the cascade below.
#
# Embree ISA kernels (sse42/avx/avx2/avx512) need --whole-archive because their
# CPU dispatch tables are registered via static initializers — without it the
# kernels exist in the .a but never get pulled in, and rtcIntersect1 falls
# back to a no-op.
# -----------------------------------------------------------------------------
target_link_directories(PolychaseTracker PRIVATE "${NUKE_ROOT}")

target_link_libraries(PolychaseTracker PRIVATE
    # ---- Nuke ----
    DDImage

    # ---- Nuke usg (new-3D geometry read). Shared lib like DDImage; linked only
    #      when POLYCHASE_NEW_3D is ON. Provides usg::Stage / MeshPrim /
    #      PointBasedPrim / XformCache used by extract_mesh_usg(). Order after
    #      DDImage; among shared libs order is not critical. ----
    $<$<BOOL:${POLYCHASE_NEW_3D}>:${USG_LIB_NAME}>

    # ---- Nuke NDK lib (libNdk.so). Defines the ndk::Selection symbols
    #      (GlobalGeoSelection / GeoSelection) that the new-system mask read in
    #      tracker_mask.cpp uses. libDDImage.so only *references* them (undefined
    #      U entries), so DDImage alone leaves them unresolved at link with
    #      --no-undefined. Linked only when POLYCHASE_NEW_3D is ON. ----
    $<$<BOOL:${POLYCHASE_NEW_3D}>:Ndk>

    # ---- Polychase core ----
    "${POLYCHASE_LIB}"

    # ---- PoseLib (find_package above) ----
    # Larsson's P4Pf minimal solver used by tracker_focal_pnpf.cpp. Static archive
    # (PoseLib::PoseLib); our own object files reference it, so they precede it in
    # the link. Header-light, Eigen-only — no extra transitive deps to order.
    PoseLib::PoseLib

    # ---- Embree (from find_package above) ----
    # Pulls in libembree4.a + the per-ISA archives + --whole-archive flags as
    # encoded by vcpkg's exported CMake config. Replaces ~15 lines of fragile
    # hand-rolled link orchestration.
    embree

    # ---- OpenCV ----
    # Our plugin no longer calls OpenCV directly (the in-node Analyze pass and
    # its plate decoder were removed). These archives remain ONLY to satisfy any
    # internal references inside libpolychase (opticalflow.cc / gftt.cc) IF the
    # linker pulls those .o files. Tracking now reads a pre-built DB, so
    # TrackSequence most likely does NOT pull them — if your link succeeds with
    # this block (and libpng/libz below) removed, you can delete it. Kept by
    # default because an unreferenced static archive is simply skipped (no cost).
    "${POLYCHASE_VCPKG}/lib/libopencv_videoio4.a"
    "${POLYCHASE_VCPKG}/lib/libopencv_imgcodecs4.a"
    "${POLYCHASE_VCPKG}/lib/libopencv_video4.a"
    "${POLYCHASE_VCPKG}/lib/libopencv_imgproc4.a"
    "${POLYCHASE_VCPKG}/lib/libopencv_features2d4.a"
    "${POLYCHASE_VCPKG}/lib/libopencv_flann4.a"
    "${POLYCHASE_VCPKG}/lib/libopencv_photo4.a"
    "${POLYCHASE_VCPKG}/lib/libopencv_ml4.a"
    "${POLYCHASE_VCPKG}/lib/libopencv_core4.a"

    # ---- Image formats (OpenCV imgcodecs deps) ----
    "${POLYCHASE_VCPKG}/lib/libpng16.a"
    "${POLYCHASE_VCPKG}/lib/libz.a"

    # ---- SQLite (database.cc) ----
    "${POLYCHASE_VCPKG}/lib/libsqlite3.a"

    # ---- Logging ----
    "${POLYCHASE_VCPKG}/lib/libspdlog.a"
    "${POLYCHASE_VCPKG}/lib/libfmt.a"

    # ---- CLI flags (examples may reference; harmless if unused) ----
    "${POLYCHASE_VCPKG}/lib/libgflags.a"

    # ---- TBB (dynamic; preferred from Nuke, see resolution above) ----
    "${TBB_LIB}"

    # ---- hwloc (TBB topology helper) ----
    # NOTE: removed. We link Nuke's own libtbb.so.12 (see TBB resolution above),
    # which carries its own topology handling, so vcpkg's static libhwloc.a is not
    # needed. It was also dropped from vcpkg_installed when PoseLib was added (it
    # was only a transitive dep of the vcpkg TBB we no longer link), so hard-linking
    # it here broke the build. If a future vcpkg-TBB build reintroduces the need,
    # restore:  "${POLYCHASE_VCPKG}/lib/libhwloc.a"

    # ---- System ----
    pthread dl rt m

    # ---- OpenGL (viewer overlay) ----
    # GL function symbols (glBegin, glVertex2f, glColor4f, etc.) used by
    # Phase 1C's draw_handle override. libDDImage.so is itself a GL client
    # but doesn't re-export the symbols to plugins, so we must link libGL
    # ourselves. Imported target via find_package(OpenGL) above.
    OpenGL::GL

    # ---- Qt6 (TrackIt progress dialog; Nuke's OWN libs, by name, from
    #      NUKE_ROOT above). Linked only when POLYCHASE_PROGRESS is ON; the
    #      generator expressions expand to nothing when OFF, keeping the Qt-free
    #      build clean. Same approach as Motion2DB. ----
    $<$<BOOL:${POLYCHASE_PROGRESS}>:Qt6Core>
    $<$<BOOL:${POLYCHASE_PROGRESS}>:Qt6Gui>
    $<$<BOOL:${POLYCHASE_PROGRESS}>:Qt6Widgets>
)

set_target_properties(PolychaseTracker PROPERTIES
    PREFIX ""
    SUFFIX ".so"
)

# -----------------------------------------------------------------------------
# Post-build: strip residual RPATH
# -----------------------------------------------------------------------------
find_program(PATCHELF patchelf)
if(PATCHELF)
    add_custom_command(TARGET PolychaseTracker POST_BUILD
        COMMAND ${PATCHELF} --remove-rpath $<TARGET_FILE:PolychaseTracker>
        COMMENT "Stripping RPATH for portability"
    )
endif()

# -----------------------------------------------------------------------------
# Unit tests (DDImage-free). The pose-math bridge (compose_trs / decompose_trs)
# and the BlobMirror decision logic were extracted so they can be tested without
# Nuke — they need only Eigen + the headers in src/. Build + run with:
#     cmake --build build --target pose_math_test blob_mirror_test
#     ctest --test-dir build --output-on-failure
# Toggle off with -DPOLYCHASE_BUILD_TESTS=OFF. Never fatal: if Eigen can't be
# located the tests are skipped, the plugin still builds.
# -----------------------------------------------------------------------------
option(POLYCHASE_BUILD_TESTS "Build the DDImage-free unit tests" ON)
if(POLYCHASE_BUILD_TESTS)
    # Locate Eigen: prefer a CONFIG package, else fall back to the vcpkg tree the
    # plugin already uses (Eigen/Core under include or include/eigen3).
    set(_pcn_eigen_inc "")
    find_package(Eigen3 QUIET CONFIG)
    if(TARGET Eigen3::Eigen)
        set(_pcn_eigen_ok TRUE)
    else()
        foreach(_cand "${POLYCHASE_VCPKG}/include" "${POLYCHASE_VCPKG}/include/eigen3")
            if(EXISTS "${_cand}/Eigen/Core")
                set(_pcn_eigen_inc "${_cand}")
                break()
            endif()
        endforeach()
        if(_pcn_eigen_inc)
            set(_pcn_eigen_ok TRUE)
        endif()
    endif()

    if(_pcn_eigen_ok)
        enable_testing()
        foreach(_t pose_math_test blob_mirror_test)
            if(_t STREQUAL "pose_math_test")
                add_executable(${_t} test/test_pose_math.cpp)
            else()
                add_executable(${_t} test/test_blob_mirror.cpp)
            endif()
            target_include_directories(${_t} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
            if(TARGET Eigen3::Eigen)
                target_link_libraries(${_t} PRIVATE Eigen3::Eigen)
            else()
                target_include_directories(${_t} SYSTEM PRIVATE "${_pcn_eigen_inc}")
            endif()
            target_compile_features(${_t} PRIVATE cxx_std_17)
            add_test(NAME ${_t} COMMAND ${_t})
        endforeach()
        message(STATUS "  Unit tests     : ON  (ctest: pose_math_test, blob_mirror_test)")
    else()
        message(STATUS "  Unit tests     : SKIPPED (Eigen not found; -DPOLYCHASE_BUILD_TESTS=OFF to silence)")
    endif()
endif()

# -----------------------------------------------------------------------------
# Summary
# -----------------------------------------------------------------------------
message(STATUS "")
message(STATUS "=== polychase_nuke_ndk (Phase 0.5) ===")
message(STATUS "  output         : PolychaseTracker.so")
message(STATUS "  Nuke           : ${NUKE_ROOT}")
message(STATUS "  Polychase root : ${POLYCHASE_ROOT}")
message(STATUS "  libpolychase.a : ${POLYCHASE_LIB}")
message(STATUS "  vcpkg libs     : ${POLYCHASE_VCPKG}/lib")
message(STATUS "  TBB            : ${TBB_LIB}  [${TBB_SOURCE}]")
message(STATUS "  New 3D (usg)   : ${POLYCHASE_NEW_3D}  (lib${USG_LIB_NAME}.so when ON)")
message(STATUS "  Build type     : ${CMAKE_BUILD_TYPE}")
message(STATUS "  patchelf       : ${PATCHELF}")
message(STATUS "======================================")
