# Third-Party Licenses

Polychase-for-Nuke is licensed under the GPL-3.0 (see `COPYING` / `LICENSE`).
It builds on, links against, or bundles the third-party components listed
below. This file reflects the components **actually linked into the compiled
module** (`polychase_core.cpython-311-x86_64-linux-gnu.so`) as of the current
build, plus declared-but-not-yet-linked and vendored components.

---

## Upstream / original work

This project is a derivative work (a Nuke port) of the original **Polychase**
Blender add-on.

- Project: Polychase — https://github.com/theartful/polychase
- Author: Ahmed Essam (theartful)
- License: GPL-3.0

Because Polychase is licensed under GPL-3.0, this derivative is also distributed
under GPL-3.0 (see `COPYING` / `LICENSE`). The full GPL-3.0 text in this
repository is the same one shipped by upstream Polychase.

---

## Linked into the distributed binary (attribution required)

These libraries are compiled/linked into the shipped module. Their notices must
accompany any binary distribution. After a build, each port's exact license
text is written to
`build/vcpkg_installed/x64-linux/share/<port>/copyright` — concatenating those
is the reliable way to assemble a complete notice bundle.

| Component | Version | License (SPDX) | Role |
|-----------|---------|----------------|------|
| OpenCV (opencv4) | 4.12.0 | Apache-2.0 | image I/O, optical flow |
| Embree | 4.4.0 | Apache-2.0 | ray casting / mesh intersection |
| oneTBB (tbb) | 2021.13.0 | Apache-2.0 | tasking backend (dynamic: libtbb.so) |
| Eigen (eigen3) | 3.4.1 | MPL-2.0 | linear algebra (header-only, compiled in) |
| spdlog | 1.16.0 | MIT | logging |
| fmt | 12.1.0 | MIT | formatting |
| libpng | 1.6.53 | libpng-2.0 | PNG support (via OpenCV) |
| zlib | 1.3.1 | Zlib | compression (via libpng/OpenCV) |
| SQLite (sqlite3) | 3.51.1 | blessing (public domain) | optical-flow database |

> SQLite is public domain ("blessing") and needs no attribution, but is listed
> for completeness. Apache-2.0 components (OpenCV, Embree, TBB) additionally
> require preserving any NOTICE file shipped with them.

---

## Declared but NOT currently linked (attribution not yet required)

These are present in `vcpkg.json` and built by vcpkg, but are **not** linked
into the current module artifact. Their licenses do not apply to the binary
until they are actually linked in.

| Component | Version | License (SPDX) | Status |
|-----------|---------|----------------|--------|
| PoseLib | 2.0.4 | BSD-3-Clause | installed; not yet referenced in code or linked |
| gflags | 2.3.0 | BSD-3-Clause | used only by the `cpp/examples` CLI tools, not the module |

When PoseLib is wired in (`find_package(PoseLib CONFIG REQUIRED)` +
`target_link_libraries(polychase_core PRIVATE PoseLib::PoseLib)`) and actually
used, its BSD-3-Clause notice below becomes mandatory for binary distribution.

### PoseLib — BSD-3-Clause (for when it is linked)

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

---

## Vendored in source (carry their own LICENSE in-tree)

Committed directly under `cpp/external/` with their own license files, and
linked into the module:

- **pybind11** — BSD-3-Clause — `cpp/external/pybind11/LICENSE`
- **cvnp** — MIT (Copyright Pascal Thomet) — `cpp/external/cvnp/LICENSE`

---

## Note for binary releases

When distributing the compiled module (e.g. as a GitHub Release asset), the
permissive licenses above (Apache-2.0, MIT, BSD-3-Clause, MPL-2.0, libpng,
Zlib) require their **full license text and copyright notices** to accompany
the binary — naming them is not sufficient. Assemble the exact texts from the
build tree (`build/vcpkg_installed/<triplet>/share/<port>/copyright`) plus the
vendored `LICENSE` files, and ship that bundle next to the binary along with
`COPYING` (GPL-3.0).
