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

Polyscope `v2.3.0` and ImPlot `v0.16` are downloaded via `FetchContent` at configure time
(`tools/cmake/ViewerDeps.cmake`, shared with `tools/contact_viewer`); nothing is added to the
core PE build. Requires `pe_CONSTRAINT_SOLVER` to be a
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

## Sister tool: contact viewer (`tools/contact_viewer/`)

A second Polyscope tool, built from the same build tree, for contact generation and resting
contact (stacking). Full reference: `tools/contact_viewer/README.md`.

```bash
cmake -S . -B build-viewer -DPE_BUILD_CONTACT_VIEWER=ON
cmake --build build-viewer --target pe_contact_viewer -j
./build-viewer/tools/contact_viewer/pe_contact_viewer            # Stack Lab (simulation, default)
./build-viewer/tools/contact_viewer/pe_contact_viewer --pair     # Pair Lab (static two-body view)
```

A **Mode** window switches between the two at any time.

**Stack Lab** — a small simulation on a visible ground plane at z = 0.

- Run/Pause, Step, Reset (keys space, `n`, `r`); steps per Step click and steps per frame.
- `dt`: logarithmic slider 1e-5 … 5e-2 (Ctrl+click to type), dt / 2 and dt x 2 for bisecting
  the largest stable step, one-click values 1e-4 … 1e-2. Applies on the next step, also while
  running; the window shows the simulated time per frame.
- Live solver settings: gravity, error reduction, max iterations, relaxation, friction model.
- Scenarios (applied on Reset): box tower, pyramid, brick wall, mixed-shape stack, boxes on a
  ramp, shapes dropped onto the ground; count, size, gap, random offset/rotation (seeded),
  friction, restitution, density.
- Contact points/normals redrawn after each frame; optional coloring of bodies by speed.
- Plots: kinetic energy, solver max penetration, contact count, drift of the top body.
- Ctrl + left-drag pulls a body with a spring (same mechanism as this viewer).

**Pair Lab** — first things to try:

- Pick a preset, tick **gizmo** on body B and drag it; contacts and sweep plots update live.
- In the **Contacts** window, click a row (or a red marker in the 3D view) for the support-gap
  check of that contact.
- In the **Sweep** window, drag the yellow line to scrub body B's `pos z` through the range.

**Things to know** (observed with the repo's default `HardContactEulerLagrange` solver):

- **Gravity is applied by the contact viewer**, not the world: this solver ignores
  `World::setGravity()` on purpose (body forces belong to the outer CFD driver). The contact
  viewer adds `m g` to each body before every step and keeps the world gravity at 0, so a solver
  that does honour it does not apply it twice.
- **Friction is additive per pair**: pe sums the two materials' coefficients, so the contact
  viewer gives each body mu / 2 and the mu in the GUI is the real contact friction.
- **Dropped boxes bounce at restitution 0**: landing at ~4.9 m/s with dt = 2e-3 penetrates ~1 cm
  in one step; they bounce back at ~0.9 m/s and settle after ~1.8 s. Likely the position
  correction (error reduction 0.7) — lower it or halve dt and watch the kinetic-energy plot.
- **Boxes slide down the 20° ramp at mu = 0.4**, although tan 20° ≈ 0.36 < mu. The default
  friction model is the approximate one; switching it is the first thing to try (not
  investigated further).
- **Cylinders get no ground contact**: `MaxContacts::collideCylinderPlane()` is an empty stub.
  The drop scenario warns when a cylinder is selected.

**Polyscope's own demo app** (from the Polyscope repo) is not built by the fetch. Its source is
in `build-viewer/_deps/polyscope-src/examples/demo-app/`; add `add_subdirectory(examples/demo-app)`
there or build a separate checkout with its own CMake. It shows the Polyscope UI on a few meshes
and point clouds and is not connected to PE.

## See also

- `tools/lubrication_explorer/` — a zero-build web sister tool that plots every lubrication
  force/torque component vs gap for the same knob set (a JS port of the analytic model, no
  simulation). Good for reasoning about the force laws; this viewer is for watching the
  dynamics. Open its `index.html` directly — no build required.
