# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Peter Mercell
#
# Developed with assistance from Claude (Anthropic).

# =============================================================================
# CMakeLists.txt — VisualizeFlowDB: standalone Nuke 17.0 NDK plugin
#
# Builds VisualizeFlowDB.so, an overlay node that draws a polychase optical-flow
# database on top of the plate (input 0). Separate from PolychaseTracker.so.
#
# Trimmed link line: the node only READS the database and draws a GL overlay —
# it pulls frames from input 0 (an Iop; Nuke's Read decodes EXR), so it needs
# NO OpenCV / TinyEXR / Embree / Qt. Just DDImage + libpolychase (Database ->
# SQLite + fmt/spdlog) + OpenGL.
#
# Expected layout:
#   <this dir>/
#     CMakeLists.txt
#     src/visualize_flow_node.cpp
#     src/visualize_flow_node.h
#
# Configure / build, e.g.:
#   cmake -S . -B build \
#     -DNUKE_VERSION=17.0v1 \
#     -DPOLYCHASE_ROOT=$HOME/Documents/POLYCHASE/polychase
#   cmake --build build -j
# Then put build/VisualizeFlowDB.so on your Nuke plugin path (e.g. ~/.nuke).
#
# IMPORTANT: build this OR add src/visualize_flow_node.cpp to PolychaseTracker's
# CMakeLists — not both. Two .so files registering the same node name would
# clash on load. This standalone build is the "as its own plugin" option.
# =============================================================================
cmake_minimum_required(VERSION 3.18)
project(visualize_flow_db_nuke LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# -----------------------------------------------------------------------------
# RPATH disabled (plugin runs inside Nuke; its .so deps are in-process already,
# and SQLite/fmt/spdlog are linked statically so there are no extra runtime .so
# deps beyond DDImage and system GL).
# -----------------------------------------------------------------------------
set(CMAKE_SKIP_BUILD_RPATH          TRUE)
set(CMAKE_SKIP_INSTALL_RPATH        TRUE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH  FALSE)
set(CMAKE_INSTALL_RPATH             "")
set(CMAKE_BUILD_RPATH               "")
set(CMAKE_SHARED_LINKER_FLAGS
    "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--disable-new-dtags -Wl,--no-undefined")

# -----------------------------------------------------------------------------
# Nuke 17 NDK
# -----------------------------------------------------------------------------
if(NOT DEFINED NUKE_VERSION)
    set(NUKE_VERSION "17.0v1")
endif()
set(NUKE_ROOT "/opt/Nuke${NUKE_VERSION}" CACHE PATH "Nuke install root")

if(NOT EXISTS "${NUKE_ROOT}/libDDImage.so")
    message(FATAL_ERROR "libDDImage.so not found in: ${NUKE_ROOT}\n"
                        "Pass -DNUKE_ROOT=/path/to/Nuke17.0v1 (or -DNUKE_VERSION=...).")
endif()

# -----------------------------------------------------------------------------
# Polychase library + vcpkg deps (database.o needs SQLite + fmt + spdlog)
# -----------------------------------------------------------------------------
set(POLYCHASE_ROOT "$ENV{HOME}/Documents/POLYCHASE/polychase"
    CACHE PATH "Polychase source tree root")
set(POLYCHASE_BUILD "${POLYCHASE_ROOT}/build")
set(POLYCHASE_LIB   "${POLYCHASE_BUILD}/cpp/libpolychase.a")
set(POLYCHASE_VCPKG "${POLYCHASE_BUILD}/vcpkg_installed/x64-linux")

if(NOT EXISTS "${POLYCHASE_LIB}")
    message(FATAL_ERROR "libpolychase.a not found at ${POLYCHASE_LIB}.")
endif()
if(NOT EXISTS "${POLYCHASE_VCPKG}/lib")
    message(FATAL_ERROR "vcpkg lib dir not found at ${POLYCHASE_VCPKG}/lib.")
endif()

# OpenGL: Nuke's libDDImage.so does not re-export GL symbols, so the viewer
# overlay (glBegin/glVertex2f/...) must link GL explicitly.
find_package(OpenGL REQUIRED)

# -----------------------------------------------------------------------------
# Plugin target
# -----------------------------------------------------------------------------
add_library(VisualizeFlowDB SHARED
    src/visualize_flow_node.cpp     # Op + overlay knob + registration
)
# Header (listed for IDEs): src/visualize_flow_node.h

target_include_directories(VisualizeFlowDB SYSTEM PRIVATE
    "${NUKE_ROOT}/include"
    "${POLYCHASE_ROOT}/cpp"          # database.h
    "${POLYCHASE_VCPKG}/include"     # Eigen, sqlite3.h, spdlog, fmt
)

# ABI flag MUST match how libpolychase.a was built.
target_compile_definitions(VisualizeFlowDB PRIVATE _GLIBCXX_USE_CXX11_ABI=1)

target_compile_options(VisualizeFlowDB PRIVATE
    -Wall -Wextra -Wno-unused-parameter -Wno-deprecated-declarations
    -O3 -fvisibility=hidden
)

target_link_directories(VisualizeFlowDB PRIVATE "${NUKE_ROOT}")

# Static-archive order: consumer before provider.
#   visualize_flow_node.cpp -> DDImage + polychase(Database) + GL
#   Database                -> sqlite3 + fmt + spdlog
target_link_libraries(VisualizeFlowDB PRIVATE
    DDImage
    "${POLYCHASE_LIB}"
    "${POLYCHASE_VCPKG}/lib/libsqlite3.a"
    "${POLYCHASE_VCPKG}/lib/libspdlog.a"
    "${POLYCHASE_VCPKG}/lib/libfmt.a"
    OpenGL::GL
    pthread dl rt m
)

set_target_properties(VisualizeFlowDB PROPERTIES PREFIX "" SUFFIX ".so")

# Strip residual RPATH for portability (matches the main plugin).
find_program(PATCHELF patchelf)
if(PATCHELF)
    add_custom_command(TARGET VisualizeFlowDB POST_BUILD
        COMMAND ${PATCHELF} --remove-rpath $<TARGET_FILE:VisualizeFlowDB>
        COMMENT "Stripping RPATH for portability")
endif()

# -----------------------------------------------------------------------------
# Summary
# -----------------------------------------------------------------------------
message(STATUS "")
message(STATUS "=== VisualizeFlowDB (Nuke ${NUKE_VERSION} plugin) ===")
message(STATUS "  output         : VisualizeFlowDB.so")
message(STATUS "  Nuke           : ${NUKE_ROOT}")
message(STATUS "  libpolychase.a : ${POLYCHASE_LIB}")
message(STATUS "  vcpkg libs     : ${POLYCHASE_VCPKG}/lib")
message(STATUS "  Build type     : ${CMAKE_BUILD_TYPE}")
message(STATUS "  patchelf       : ${PATCHELF}")
message(STATUS "====================================================")
