# PolychaseTracker

A Nuke 17 (NDK) mesh-based motion tracker. Given footage, a camera and a 3D
mesh that matches an object in the shot, it solves the object's (or camera's)
motion from a prebuilt Polychase optical-flow database and exports an animated
Camera or TransformGeo.

The optical-flow database is built **outside** Nuke (by Motion2DB); this node
only reads a prebuilt `.db` and solves.

![PolychaseTracker wireframe overlay in the viewer](PolychaseTracker.png)

## Inputs

| Pin | Accepts | Purpose |
|---|---|---|
| `img` | Read / plate | Footage frames |
| `cam` | Camera | Intrinsics (focal, aperture) |
| `geo` | Geo / mesh | The 3D mesh to track |
| `mask` | (optional) | Region mask |

All inputs are optional and type-gated — Nuke greys out wires that don't match.

## Setup

1. Connect `img` + `cam` + `geo`.
2. Set the **Database** path to the `.db` built by Motion2DB.

Then follow the workflow that matches your shot — a **static (prime) lens** or a
**zoom lens**.

## Workflow — static lens

1. On the **first frame**, set the wireframe position with the gizmo.
2. Fine-tune the position with **Pins**.
3. **Set Pose Key**.
4. **Track Forward**.
5. If the track drifts, adjust the offending frames — anchors are created
   automatically where you correct it.
6. **Set the refine range**.
7. **Refine**.
8. **Export**.

## Workflow — zoom lens

Solving a changing focal length only works **with user tracks**, so this path
adds a Tracker node before solving. It's more involved than the static-lens
workflow.

1. On the **first frame**, set the wireframe position with the gizmo.
2. Fine-tune the position with **Pins**.
3. **Set Pose Key**.
4. In the **User Tracks** tab, add a Tracker node, set the **reference frame**
   (the first frame), and add **~±50 px overscan**.
5. **Load Tracks**.
6. **Connect**.
7. In the **Intrinsics** tab, **Solve Focal** and tick **Export Solved Focal**.
8. After solving, click **Copy Solved Focal → Camera**.
9. Track forward — or, better for a zoom, set two keyframes (anchors) on the
   **mid** and **last** frames.
10. **Set the refine range**.
11. **Refine**.
12. **Export**.

> 📹 A video walkthrough of the zoom-lens workflow is coming — it's the more
> complex of the two.

When the solve looks right, pick a **Solve Mode** (Camera or Model); **Export**
spawns an animated Camera2 or TransformGeo carrying the tracked motion.

## Handy controls

- **Pins** — place, remove, and key pins to anchor the mesh to the plate;
  refine ranges with anchors.
- **Solve Focal (P4Pf)** / **Smooth Solved Focal** / **Copy Solved Focal → Camera**
  for focal-length solving.
- **Load Tracks** / **Connect** to drive the solve from a Nuke Tracker4 node's 2D tracks.
- Inspect the flow database itself with **VisualizeFlowDB**.

### Viewer hotkeys

| Key | Action |
|---|---|
| `shift+P` | Toggle Pin Edit |
| `ctrl+Z` | Undo Move (the tracker's own move history) |
| `ctrl+shift+Z` | Redo Move |

The move history is independent of Nuke's undo stack — gizmo moves, pin edits
and wireframe-colour changes all undo/redo through it while a PolychaseTracker
is the active node.

## Install

After building (see [`BUILDING.md`](../../001_PolychaseTracker_V1.0.0/BUILDING.md)),
add the plugin path to your `~/.nuke/init.py`:

```python
import nuke
nuke.pluginAddPath('/path/to/PolychaseTracker/plugin')
```

Then create it from the **Polychase → PolychaseTracker (NDK)** menu.

## Build (summary)

```bash
./build.sh
```

A successful build produces `build/PolychaseTracker.so`. The full reference —
prerequisites (Nuke 17, gcc-toolset-13, CMake/Ninja, a built Polychase tree),
the machine-specific paths to change, and all build toggles — is in
[`BUILDING.md`](../../001_PolychaseTracker_V1.0.0/BUILDING.md).

---

GPL-3.0-or-later · © 2026 Peter Mercell
