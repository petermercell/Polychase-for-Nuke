# Building PolychaseTracker

`PolychaseTracker.so` is a Nuke 17 NDK plugin. It links a prebuilt
`libpolychase.a` (plus its vcpkg static deps) against Nuke's DDImage. The
optical-flow database is built **outside** Nuke by the `mvflow_to_db` converter,
so this plugin only reads a prebuilt DB and solves.

This document lists everything that is specific to the original build machine,
so you know what to change for your own setup.

---

## Prerequisites

- **Nuke 17** (default target `17.0v1`), with `libDDImage.so` present.
- A **C++17 toolchain**. The reference build uses `gcc-toolset-13` on Rocky/RHEL 9.
- **CMake ≥ 3.18** and **Ninja**.
- A built **Polychase** tree (`libpolychase.a` + `vcpkg_installed/`), including
  Embree 4, PoseLib, OpenCV, SQLite, spdlog/fmt, TBB.
- **Qt 6.5.3** SDK headers — only if you build with the progress dialog (`POLYCHASE_PROGRESS=ON`, the default). Qt **libraries** are taken from Nuke at load time; you only need the matching headers to compile against.
- `patchelf` (optional; used post-build to strip RPATH).

---

## What you must change

The reference `CMakeLists.txt` and `build.sh` contain paths that point at the
original author's machine. Three are hardcoded to a specific layout; the rest
are overridable from the command line.

### 1. Qt SDK path — **edit required** (or disable Qt)

`CMakeLists.txt` hardcodes the Qt 6.5.3 SDK location:

```cmake
set(Qt6_ROOT "/home/pm/Qt6NEW/6.5.3/gcc_64")
```

With the progress dialog on (the default) you must do **one** of:

- pass your own path: `-DQt6_ROOT=/path/to/Qt/6.5.3/gcc_64`, **or**
- edit that line in `CMakeLists.txt`, **or**
- build Qt-free: `POLYCHASE_PROGRESS=OFF ./build.sh` (the dialog becomes a
  no-op; everything else builds identically).

> Note: `build.sh` does not currently forward a `Qt6_ROOT` variable, so if you
> keep the dialog on you must either edit the CMake line or invoke `cmake`
> directly with `-DQt6_ROOT=...`.

### 2. Compiler toolchain — **edit if not on Rocky/RHEL 9**

`build.sh` sources `gcc-toolset-13`:

```bash
source /opt/rh/gcc-toolset-13/enable
```

On other distros, remove or replace this with however you put a C++17 compiler
on `PATH`. The ABI flag `_GLIBCXX_USE_CXX11_ABI=1` **must match** how your
`libpolychase.a` was built — keep them consistent.

### 3. Polychase source tree — **override recommended**

`CMakeLists.txt` defaults to:

```cmake
set(POLYCHASE_ROOT "$ENV{HOME}/Documents/POLYCHASE/polychase")
```

The `$HOME` prefix is portable, but the `Documents/POLYCHASE/polychase` subpath
is author-specific. Point it at your own tree:

```bash
cmake ... -DPOLYCHASE_ROOT=/path/to/polychase
```

Everything downstream derives from this automatically: `libpolychase.a`, the
vcpkg tree, Embree's CMake config, PoseLib, OpenCV/SQLite/spdlog archives, and
the include dirs.

The vcpkg triplet is fixed to **`x64-linux`**. If you build on a different
architecture/OS, change it in `CMakeLists.txt` (the `POLYCHASE_VCPKG` line).

### 4. Nuke install — **override if not under `/opt`**

```cmake
set(NUKE_VERSION "17.0v1")            # forwarded by build.sh
set(NUKE_ROOT "/opt/Nuke${NUKE_VERSION}")
```

- Different version: `NUKE_VERSION=17.0v4 ./build.sh`
- Installed elsewhere: `cmake ... -DNUKE_ROOT=/your/Nuke/path`

### 5. USD/usg abstraction lib — **usually automatic**

For the Nuke 17 new-3D read path (`POLYCHASE_NEW_3D=ON`, default), the build
links Nuke's USD-abstraction lib, defaulting to `libFnUsdAbstraction.so`. If
your Nuke build names it differently, the configure step fails with a message
telling you to:

```bash
ls $NUKE_ROOT/libFn*Usd*.so
cmake ... -DUSG_LIB_NAME=<name without 'lib' prefix and '.so' suffix>
```

Or build classic-only with `POLYCHASE_NEW_3D=OFF`.

---

## Quick reference

| What | Where | Default | How to change |
|------|-------|---------|---------------|
| Qt SDK headers | `CMakeLists.txt` | `/home/pm/Qt6NEW/6.5.3/gcc_64` | `-DQt6_ROOT=...` / edit / `POLYCHASE_PROGRESS=OFF` |
| Compiler | `build.sh` | `gcc-toolset-13` (Rocky 9) | edit the `source` line |
| Polychase tree | `CMakeLists.txt` | `$HOME/Documents/POLYCHASE/polychase` | `-DPOLYCHASE_ROOT=...` |
| vcpkg triplet | `CMakeLists.txt` | `x64-linux` | edit `POLYCHASE_VCPKG` |
| Nuke version | `build.sh` / CMake | `17.0v1` | `NUKE_VERSION=...` |
| Nuke root | `CMakeLists.txt` | `/opt/Nuke<ver>` | `-DNUKE_ROOT=...` |
| usg lib name | `CMakeLists.txt` | `FnUsdAbstraction` | `-DUSG_LIB_NAME=...` |

---

## Build

After adjusting the paths above:

```bash
./build.sh
```

Useful toggles (environment variables, forwarded to CMake):

| Variable | Default | Effect |
|----------|---------|--------|
| `POLYCHASE_PROGRESS` | `ON` | Qt progress dialog; set `OFF` for a Qt-free build |
| `POLYCHASE_NEW_3D` | `ON` | Read Nuke 17 new-3D (USD/usg) geometry; `OFF` for classic-only |
| `POLYCHASE_DEBUG_LOG` | `OFF` | Verbose `PCN_LOG` diagnostics to stdout |
| `NUKE_VERSION` | `17.0v1` | Nuke install to build against |
| `USG_LIB_NAME` | `FnUsdAbstraction` | Override Nuke's USD-abstraction lib name |
| `POLYCHASE_NDK_BUILD_DIR` | `./build` | Build directory |

Example — Qt-free build with logging:

```bash
POLYCHASE_PROGRESS=OFF POLYCHASE_DEBUG_LOG=ON ./build.sh
```

A successful build produces `build/PolychaseTracker.so`.

---

## Activate in Nuke

Add to your `~/.nuke/init.py`:

```python
import nuke
nuke.pluginAddPath('/path/to/PolychaseTracker/plugin')
```

(`build.sh` prints the exact line for your checkout on success.)

---

## Unit tests (optional)

DDImage-free tests build by default when Eigen is found:

```bash
cmake --build build --target pose_math_test blob_mirror_test
ctest --test-dir build --output-on-failure
```

Disable with `-DPOLYCHASE_BUILD_TESTS=OFF`. They are never fatal — if Eigen
isn't located the tests are skipped and the plugin still builds.
