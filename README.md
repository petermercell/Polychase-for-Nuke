# Polychase for Nuke

A port of the open-source **Polychase** 3D motion tracker to **Foundry Nuke**
(NDK / Nuke 17). Polychase tracks camera or object motion in footage by fitting
a 3D mesh to the shot, driven by optical-flow analysis, PnP, and user-placed
pins. This repository brings that workflow into Nuke as a set of native plugins.

This is a derivative work of the original Polychase Blender add-on by Ahmed
Essam (theartful), and like the original it is distributed under the **GPL-3.0**.

## What's in here

| Folder | Component | What it is |
|---|---|---|
| `000_POLYCHASE_OG_260626` | **Polychase (core)** | The upstream Polychase source — the tracking/solver core that the Nuke plugins build on. |
| `001_PolychaseTracker_V1.0.0` | **PolychaseTracker** | The main Nuke node: fits a 3D mesh to the plate and solves camera/object motion, then exports an animated Camera or TransformGeo. |
| `002_MOTION2DB_V1.0.0` | **Motion2DB** | Builds the Polychase optical-flow database (`.db`) from a pre-rendered motion-vector EXR sequence. |
| `003_VisualizeFlowDB_V1.0.0` | **VisualizeFlowDB** | Overlays a flow database on the plate in the 2D viewer for inspection — no tracking, pure visualization. |

## How the pieces fit together

The three Nuke nodes form one pipeline:

1. **Motion2DB** converts a motion-vector EXR sequence into an optical-flow
   database (`.db`) — built outside Nuke's render loop, straight from disk.
2. **PolychaseTracker** reads that `.db`, fits your mesh to the footage with
   pins, solves the motion, and exports an animated Camera or TransformGeo.
3. **VisualizeFlowDB** lets you inspect the same `.db` over the plate to sanity-
   check the flow before (or after) solving.

## Getting started

Each Nuke node lives in its own numbered folder (`001`–`003`). Build
instructions live with each component (see each folder's `BUILDING.md` /
`stepbystep.txt`). The Nuke plugins target **Nuke 17** and link a prebuilt
Polychase core; the core itself is built from `000_POLYCHASE_OG_260626`.

## License & credits

Licensed under **GPL-3.0** (see `LICENSE` / `COPYING`). This project is a
derivative of the original Polychase by Ahmed Essam (theartful),
https://github.com/theartful/polychase. Third-party components linked or bundled
into the build are listed in `THIRD_PARTY_LICENSES.md` and `NOTICE-binary.txt`.

---

© 2026 Peter Mercell · Developed with assistance from Claude (Anthropic).
