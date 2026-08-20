# PE live viewer

Interactive Polyscope viewer for small (≤ ~50 body) lubrication scenarios: a 3D view of
the running simulation next to ImPlot panes (probe gap/velocity history, analytic Kroupa
wall-normal resistance), with all runtime lubrication knobs exposed as live ImGui widgets
through a small declarative parameter registry (`ParamRegistry.h`).

## Build

```bash
cmake -S . -B build-viewer -DPE_BUILD_LIVE_VIEWER=ON
cmake --build build-viewer --target pe_live_viewer -j
./build-viewer/tools/live_viewer/pe_live_viewer
```

Polyscope `v2.3.0` and ImPlot `v0.16` are downloaded via `FetchContent` at configure time;
nothing is added to the core PE build. Requires `pe_CONSTRAINT_SOLVER` to be a
lubrication-stage-capable solver, i.e. `pe::response::HardContactAndFluid` (configure in
`pe/config/Collisions.h`). Lubrication itself is a runtime switch: the viewer calls
`pe::lubrication::setEnabled(true)` at startup and exposes it as the "enabled" toggle.

On Debian/Ubuntu (including WSL2 with WSLg) the GLFW build inside Polyscope needs the X11
development packages:

```bash
sudo apt install xorg-dev libgl1-mesa-dev
```

## Structure

- `live_viewer.cpp` — scenario construction (sphere column over a plane), the PE→Polyscope
  mirror, the per-frame callback (stepping, controls, plots), and the registry entries for
  the lubrication knobs (`pe::lubrication::*`, `minGap`, world viscosity/damping/gravity).
- `ParamRegistry.h` — `{name, range, getter, setter}` parameter descriptions; the GUI
  iterates the registry, so a new tweakable is one `push_back`.

Parameters come in two flavors: the **live** groups apply immediately mid-run; the
**Scenario** window stages values (sphere count, radius, dt, …) that take effect on Reset.

## Extending

- Boxes/capsules: register a unit-size `polyscope::registerSurfaceMesh` once and update it
  per frame with `bodyTransform()` (see the template in `live_viewer.cpp`).
- Per-contact lubrication forces in the plots need a small accessor/callback surface on
  `pe/core/lubrication/LubricationStage.h`; the current plots use kinematics of the probe sphere plus the
  analytic model curve, which needs no engine changes.

## See also

- `tools/lubrication_explorer/` — a zero-build web sister tool that plots every lubrication
  force/torque component vs gap for the same knob set (a JS port of the analytic model, no
  simulation). Good for reasoning about the force laws; this viewer is for watching the
  dynamics. Open its `index.html` directly — no build required.
