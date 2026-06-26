# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Peter Mercell
#
# Developed with assistance from Claude (Anthropic).

# =============================================================================
# CMakeLists.txt — Motion2DB (standalone Nuke NDK plugin)
#
# Builds Motion2DB.so: a no-input, button-driven NoIop that reads a pre-rendered
# motion-vector EXR sequence from disk (TinyEXR) and writes the polychase
# optical-flow .db that PolychaseTracker reads and VisualizeFlowDB inspects.
# Path A of MOTION2DB_PLAN.md — no Nuke frame-stepping.
#
# Conventions copied verbatim from the PolychaseTracker CMakeLists (C++17,
# _GLIBCXX_USE_CXX11_ABI=1, fPIC, fvisibility=hidden, no-RPATH + patchelf).
#
# Layout assumed:
#   <this dir>/src/motion2db.cpp
#   <this dir>/src/motion2db.h
#   <this dir>/src/tinyexr.h        <-- the one you were told you could delete; keep it here
#
# NOTE: Motion2DB needs NONE of embree / OpenCV / Qt / OpenGL. It only pulls the
# Database path out of libpolychase (-> sqlite3, fmt, spdlog), plus zlib for
# TinyEXR. The OpenCV/embree cascade is left commented out and is only needed if
# your linker reports unresolved symbols dragged in from libpolychase.a (see the
# block at the bottom of the link list).
# =============================================================================
cmake_minimum_required(VERSION 3.18)
project(motion2db_nuke_ndk LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)            # same reason as PolychaseTracker: Nuke 17
set(CMAKE_CXX_STANDARD_REQUIRED ON)   # DDImage headers reject C++20 ctor syntax
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# RPATH disabled (plugin runs inside Nuke; libs are in-process already).
set(CMAKE_SKIP_BUILD_RPATH      TRUE)
set(CMAKE_SKIP_INSTALL_RPATH    TRUE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH         "")
set(CMAKE_BUILD_RPATH           "")
set(CMAKE_SHARED_LINKER_FLAGS
    "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--disable-new-dtags -Wl,--no-undefined")
# Static libstdc++/libgcc are applied via target_link_options() on the target
# below (more reliable than CMAKE_SHARED_LINKER_FLAGS, which can miss on rebuilds
# and is sometimes placed too late on the link line to take effect).

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
# Polychase library + vcpkg deps (Database + sqlite3/fmt/spdlog + zlib header)
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
    message(FATAL_ERROR "vcpkg lib dir not found at ${POLYCHASE_VCPKG}/lib.")
endif()

# -----------------------------------------------------------------------------
# Optional progress dialog (Qt). ON pulls in analyze_progress.cpp + Qt6 HEADERS
# (matched to Nuke's bundled 6.5.3) and links Nuke's OWN Qt6 libs BY NAME, so Qt
# is HOST-PROVIDED at load time exactly like libDDImage.so — never bundled, never
# shipped. The portability story is therefore identical with it ON or OFF: the
# plugin still bundles every third-party dep statically (incl. libstdc++/libgcc)
# and the only runtime deps are Nuke's own libs (DDImage [+ Qt6 when ON]) + glibc.
# So the static, portable build CAN keep the progress bar — leave this ON.
#
# Reasons to set OFF: (a) build machines without the Qt 6.5.3 SDK headers
# installed, or (b) reusing this Op in a non-Qt host. The dialog is a no-op in
# headless/terminal Nuke regardless.
#   cmake ... -DMOTION2DB_PROGRESS=OFF
# -----------------------------------------------------------------------------
option(MOTION2DB_PROGRESS "Show a Qt progress dialog during Convert" ON)
if(MOTION2DB_PROGRESS)
    if(NOT DEFINED Qt6_ROOT)
        set(Qt6_ROOT "/home/pm/Qt6NEW/6.5.3/gcc_64")
    endif()
    set(Qt6_DIR "${Qt6_ROOT}/lib/cmake/Qt6")
    set(CMAKE_PREFIX_PATH "${Qt6_ROOT}" ${CMAKE_PREFIX_PATH})
    find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets)
    message(STATUS "  Qt6 progress   : ON  (headers ${Qt6_ROOT}, libs = Nuke's)")
else()
    message(STATUS "  Qt6 progress   : OFF (Qt-free static build)")
endif()

# -----------------------------------------------------------------------------
# Plugin target
# -----------------------------------------------------------------------------
add_library(Motion2DB SHARED
    src/motion2db.cpp          # Motion2DB Op: knobs, convert, TinyEXR read, DB write, registration
    $<$<BOOL:${MOTION2DB_PROGRESS}>:src/analyze_progress.cpp>  # Qt progress dialog (isolated)
)
# Header (not compiled directly): src/motion2db.h  (+ src/analyze_progress.h when MOTION2DB_PROGRESS)

# SYSTEM include of the vcpkg dir gives us database.h's deps AND <zlib.h>
# (TinyEXR is configured with TINYEXR_USE_MINIZ 0 / TINYEXR_USE_STB_ZLIB 0, so it
# #includes <zlib.h> and links libz.a below). The opencv4 subdir is only here in
# case a libpolychase public header transitively pulls <opencv2/...>; harmless.
target_include_directories(Motion2DB SYSTEM PRIVATE
    "${NUKE_ROOT}/include"
    "${POLYCHASE_ROOT}/cpp"          # database.h
    "${POLYCHASE_VCPKG}/include"     # Eigen, sqlite3.h, zlib.h, fmt, spdlog
    "${POLYCHASE_VCPKG}/include/opencv4"
)
# tinyexr.h is in src/ next to motion2db.cpp (quoted include resolves there).
target_include_directories(Motion2DB PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")

# Qt6 headers (only when the progress dialog is enabled). Link is Nuke's Qt6
# libs, by name, below. MOTION2DB_HAVE_QT switches the code path in motion2db.cpp.
if(MOTION2DB_PROGRESS)
    target_compile_definitions(Motion2DB PRIVATE MOTION2DB_HAVE_QT)
    target_include_directories(Motion2DB SYSTEM PRIVATE
        $<TARGET_PROPERTY:Qt6::Core,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:Qt6::Gui,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:Qt6::Widgets,INTERFACE_INCLUDE_DIRECTORIES>
    )
endif()

# ABI flag MUST match how libpolychase.a was built.
target_compile_definitions(Motion2DB PRIVATE _GLIBCXX_USE_CXX11_ABI=1)

target_compile_options(Motion2DB PRIVATE
    -Wall -Wextra
    -Wno-unused-parameter
    -Wno-deprecated-declarations
    -O3
    -fvisibility=hidden
)

target_link_directories(Motion2DB PRIVATE "${NUKE_ROOT}")

# Static-archive order: consumer before provider.
target_link_libraries(Motion2DB PRIVATE
    # ---- Nuke ----
    DDImage

    # ---- Polychase core (only database.o should be pulled by our Database use) ----
    "${POLYCHASE_LIB}"

    # ---- Database's direct deps ----
    "${POLYCHASE_VCPKG}/lib/libsqlite3.a"     # database.cc
    "${POLYCHASE_VCPKG}/lib/libspdlog.a"      # database.cc logging
    "${POLYCHASE_VCPKG}/lib/libfmt.a"         # spdlog/fmt

    # ---- zlib (TinyEXR: TINYEXR_USE_MINIZ 0 -> system zlib) ----
    "${POLYCHASE_VCPKG}/lib/libz.a"

    # ---- System ----
    pthread dl rt m

    # ---- Qt6 (progress dialog; Nuke's OWN libs, by name, from NUKE_ROOT above).
    #      Linked only when MOTION2DB_PROGRESS is ON; the generator expressions
    #      expand to nothing when OFF, keeping the Qt-free build clean. ----
    $<$<BOOL:${MOTION2DB_PROGRESS}>:Qt6Core>
    $<$<BOOL:${MOTION2DB_PROGRESS}>:Qt6Gui>
    $<$<BOOL:${MOTION2DB_PROGRESS}>:Qt6Widgets>

    # ---- ONLY-IF-NEEDED cascade --------------------------------------------
    # If the linker reports unresolved symbols pulled transitively from
    # libpolychase.a (embree*, cv::*, tbb*, hwloc*), uncomment the matching
    # lines. Pure Database use should NOT need these. embree in particular is a
    # --whole-archive target, so leave it OUT unless an error names it.
    #
    # find_package(Threads REQUIRED)
    # set(embree_DIR "${POLYCHASE_VCPKG}/share/embree")
    # find_package(embree 4 REQUIRED CONFIG)
    # embree
    # "${POLYCHASE_VCPKG}/lib/libopencv_videoio4.a"
    # "${POLYCHASE_VCPKG}/lib/libopencv_imgcodecs4.a"
    # "${POLYCHASE_VCPKG}/lib/libopencv_video4.a"
    # "${POLYCHASE_VCPKG}/lib/libopencv_imgproc4.a"
    # "${POLYCHASE_VCPKG}/lib/libopencv_features2d4.a"
    # "${POLYCHASE_VCPKG}/lib/libopencv_flann4.a"
    # "${POLYCHASE_VCPKG}/lib/libopencv_photo4.a"
    # "${POLYCHASE_VCPKG}/lib/libopencv_ml4.a"
    # "${POLYCHASE_VCPKG}/lib/libopencv_core4.a"
    # "${POLYCHASE_VCPKG}/lib/libpng16.a"
    # "${TBB_LIB}"   # see PolychaseTracker CMake for the Nuke-vs-vcpkg pick
    # "${POLYCHASE_VCPKG}/lib/libhwloc.a"
    # ------------------------------------------------------------------------
)

# Bake libstdc++/libgcc INTO Motion2DB.so so it doesn't need a matching (newer)
# GLIBCXX/GCC_* on the target. Key when building with gcc-toolset-13 (newer than
# the deployment machines' system GCC). --exclude-libs,ALL keeps the symbols
# pulled from the static archives from being re-exported by the plugin.
# All third-party deps above are already static .a, so after this the only
# runtime deps are Nuke's libDDImage.so and base glibc (libc/libm/libdl/...).
target_link_options(Motion2DB PRIVATE
    -static-libstdc++
    -static-libgcc
    "LINKER:--exclude-libs,ALL")

set_target_properties(Motion2DB PROPERTIES PREFIX "" SUFFIX ".so")

# Strip residual RPATH for portability.
find_program(PATCHELF patchelf)
if(PATCHELF)
    add_custom_command(TARGET Motion2DB POST_BUILD
        COMMAND ${PATCHELF} --remove-rpath $<TARGET_FILE:Motion2DB>
        COMMENT "Stripping RPATH for portability"
    )
endif()

message(STATUS "")
message(STATUS "=== motion2db_nuke_ndk ===")
message(STATUS "  output         : Motion2DB.so")
message(STATUS "  Nuke           : ${NUKE_ROOT}")
message(STATUS "  libpolychase.a : ${POLYCHASE_LIB}")
message(STATUS "  vcpkg libs     : ${POLYCHASE_VCPKG}/lib")
message(STATUS "  Build type     : ${CMAKE_BUILD_TYPE}")
message(STATUS "==========================")
