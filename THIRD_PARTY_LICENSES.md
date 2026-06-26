# Third-Party Licenses

Polychase-for-Nuke is licensed under the GPL-3.0 (see `LICENSE`). It builds on,
links against, or bundles the third-party components listed below.

This file describes the components **actually linked into each distributed Nuke
plugin** (`PolychaseTracker.so`, `Motion2DB.so`, `VisualizeFlowDB.so`), as
verified against the built binaries (symbol inspection), not merely against the
build scripts. Components that appear on a link line but are dead-stripped from
the final binary (an unreferenced static archive is skipped by the linker) are
listed separately under "Present in the build tree but not linked," so an
auditor who finds them in `vcpkg.json` understands why they carry no notice.

---

## Upstream / original work

This project is a derivative work (a Nuke port) of the original **Polychase**
Blender add-on.

- Project: Polychase — https://github.com/theartful/polychase
- Author: Ahmed Essam (theartful)
- License: GPL-3.0

Because Polychase is licensed under GPL-3.0, this derivative is also distributed
under GPL-3.0 (see `LICENSE`). The full GPL-3.0 text in this repository is the
same one shipped by upstream Polychase.

---

## What is linked into each plugin

All third-party libraries below are compiled into the plugins as **static
archives** (`.a`), except where noted as host-provided. After static linking and
`--exclude-libs,ALL`, the only runtime dependencies of each `.so` are Nuke's own
libraries and base glibc.

| Component | Version | License (SPDX) | PolychaseTracker | Motion2DB | VisualizeFlowDB |
|-----------|---------|----------------|:---:|:---:|:---:|
| PoseLib   | 2.0.4   | BSD-3-Clause   | ✓ | | |
| Embree    | 4.4.0   | Apache-2.0     | ✓ | | |
| SQLite    | 3.51.1  | blessing (public domain) | ✓ | ✓ | ✓ |
| spdlog    | 1.16.0  | MIT            | ✓ | ✓ | ✓ |
| fmt       | 12.1.0  | MIT            | ✓ | ✓ | ✓ |
| Eigen     | 3.4.1   | MPL-2.0        | ✓ | ✓ | ✓ |
| zlib      | 1.3.1   | Zlib           | ✓ | ✓ | |
| TinyEXR   | —       | BSD-3-Clause   | | ✓ | |

Notes:

- **Eigen** is header-only; it is compiled in (inlined) and leaves no named
  symbols, but it is genuinely part of every plugin via `database.h` and the
  solver headers.
- **PoseLib** is likewise Eigen-templated and leaves few named symbols, but is
  linked into PolychaseTracker via `find_package(PoseLib CONFIG REQUIRED)` +
  `PoseLib::PoseLib`. Its P4Pf minimal solver is used by the focal-length solve.
- **zlib** is pulled into PolychaseTracker transitively (confirmed: `inflate`
  symbols present in the binary) and into Motion2DB by TinyEXR
  (`TINYEXR_USE_MINIZ 0` → system zlib).
- **TinyEXR** is vendored (`src/tinyexr.h`), not a vcpkg port; it carries an
  embedded OpenEXR/ILM notice that must travel with it (both texts below).
- **SQLite** is public domain ("blessing") and needs no attribution, but is
  listed for completeness. **Apache-2.0** components (Embree) additionally
  require preserving any NOTICE file shipped with them.

The exact, version-accurate copyright text for each vcpkg-managed port is at
`build/vcpkg_installed/x64-linux/share/<port>/copyright`. The per-plugin notice
bundles shipped with each binary (`<Plugin>_copyright.txt`) are assembled from
exactly those files plus the vendored TinyEXR text.

---

## Host-provided (linked by name, never bundled — no attribution owed)

These are Nuke's own libraries (or the system's), resolved at load time exactly
like `libDDImage.so`. They are not redistributed, so no notice is owed:

- **DDImage** (`libDDImage.so`) — Foundry Nuke NDK core
- **TBB** (`libtbb.so.12`) — PolychaseTracker prefers **Nuke's** TBB; the vcpkg
  static TBB is only a fallback for build hosts where Nuke's is absent (a
  non-default configuration). In the shipped build, TBB is host-provided.
- **Qt6** (Core/Gui/Widgets) — linked by name against Nuke's bundled Qt 6.5.3 for
  the optional progress dialog; never bundled or shipped.
- **usg / Ndk** (`libFnUsdAbstraction.so`, `libNdk.so`) — Nuke 17 new-3D geometry
  read, linked only when `POLYCHASE_NEW_3D=ON`.
- **OpenGL** (system `libGL`) — viewer overlay drawing.

---

## Present in the build tree but NOT linked (no attribution required)

These are built by vcpkg (so they have a `share/<port>/copyright` entry) but do
**not** end up in any shipped binary. Their licenses do not apply to the
distribution.

| Component | Version | License (SPDX) | Why it is not linked |
|-----------|---------|----------------|----------------------|
| OpenCV (opencv4) | 4.12.0 | Apache-2.0 | On PolychaseTracker's link line as a fallback for internal `libpolychase` references, but **dead-stripped**: tracking reads a pre-built DB, so no `cv::` symbols survive into the binary (verified — zero `cv::` symbols). |
| libpng | 1.6.53 | libpng-2.0 | Only reachable via OpenCV `imgcodecs`; since OpenCV is stripped, libpng is too (verified — zero `png_` symbols). |
| gflags | 2.3.0 | BSD-3-Clause | Used only by the `cpp/examples` CLI tools, which are not part of any plugin. |

> If a future change makes the tracker call OpenCV directly, re-verify with
> `nm -C PolychaseTracker.so | grep -c 'cv::'` and restore the opencv4/libpng
> notices if the count is non-zero.

---

## Full license texts

### PoseLib — BSD-3-Clause (PolychaseTracker)

```
BSD 3-Clause License

Copyright (c) 2020, Viktor Larsson
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

### Embree, SQLite, spdlog, fmt, Eigen, zlib

The exact texts for these vcpkg-managed ports are reproduced in each plugin's
`<Plugin>_copyright.txt` bundle, assembled verbatim from
`build/vcpkg_installed/x64-linux/share/<port>/copyright`. Embree is Apache-2.0;
spdlog and fmt are MIT; Eigen is MPL-2.0 (with a few BSD/Apache-2.0 internal
files, included in its copyright text); zlib is the Zlib license; SQLite is
public domain.

### TinyEXR — BSD-3-Clause (Motion2DB)

TinyEXR is header-only and compiled directly into `Motion2DB.so`. Its
BSD-3-Clause notice, and the OpenEXR notice it carries, must accompany any
binary distribution of that plugin. The standalone `LICENSE-TinyEXR.txt` shipped
beside the plugin binaries contains both, and they are appended to
`Motion2DB_copyright.txt`.

```
Copyright (c) 2014 - 2021, Syoyo Fujita and many contributors.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Syoyo Fujita nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

TinyEXR also contains OpenEXR code, licensed under BSD-3-Clause:

```
Copyright (c) 2002, Industrial Light & Magic, a division of Lucas
Digital Ltd. LLC
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
*   Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
*   Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
*   Neither the name of Industrial Light & Magic nor the names of its
    contributors may be used to endorse or promote products derived from this
    software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

---

## Note for binary releases

Ship, next to the plugin binaries:

- `LICENSE` (GPL-3.0) — the project license and corresponding-source offer.
- The per-plugin notice bundles — `PolychaseTracker_copyright.txt`,
  `Motion2DB_copyright.txt`, `VisualizeFlowDB_copyright.txt` — each assembled
  verbatim from the relevant `share/<port>/copyright` files (plus the vendored
  TinyEXR text for Motion2DB).
- `LICENSE-TinyEXR.txt` (standalone, for Motion2DB).

Per-plugin port sets (for reassembling the bundles):

- **PolychaseTracker**: poselib, embree, sqlite3, spdlog, fmt, eigen3, zlib
- **Motion2DB**: sqlite3, spdlog, fmt, eigen3, zlib + TinyEXR (vendored)
- **VisualizeFlowDB**: sqlite3, spdlog, fmt, eigen3

The permissive licenses above (Apache-2.0, MIT, BSD-3-Clause, MPL-2.0, Zlib)
require their **full license text and copyright notices** to accompany the
binary — naming them is not sufficient. The bundles satisfy this.
