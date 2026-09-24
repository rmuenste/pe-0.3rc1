# PE contact viewer

Interactive Polyscope harness for the narrow-phase contact generation
(`pe::detection::fine::MaxContacts`). It calls the fine collision detector directly on the
bodies and records what it emits, so it works with every constraint solver configuration and
needs no time stepping to show a contact manifold.

Two modes in one binary, switched in the **Mode** window:

- **Stack Lab** (default) — run / pause / step a small simulation on a visible ground plane.
- **Pair Lab** — pose two bodies and inspect their contacts statically.

## Build

```bash
cmake -S . -B build-viewer -DPE_BUILD_CONTACT_VIEWER=ON
cmake --build build-viewer --target pe_contact_viewer -j
./build-viewer/tools/contact_viewer/pe_contact_viewer
```

Polyscope and ImPlot are downloaded via `FetchContent` at configure time
(`tools/cmake/ViewerDeps.cmake`, shared with `tools/live_viewer`); nothing is added to the core
PE build. For the X11 development packages GLFW needs, see `tools/live_viewer/README.md`.

Command line:

- `pe_contact_viewer [--pair | --stack]` — start mode (default Stack Lab).
- `pe_contact_viewer --smoke` — headless self check of both modes on Polyscope's mock OpenGL
  backend: every Pair Lab preset and all 36 shape pairs, the gizmo pose round trip, and every
  Stack Lab scenario stepped for 1 s (3 s for the drop). Prints contacts, a contacts-per-pair
  matrix (a `0` for an overlapping pair means the routine generates nothing, e.g. the
  `collideCylinderPlane()` stub) and per-scenario stability numbers.
- `pe_contact_viewer [--pair | --stack] --screenshot out.png [--preset k] [--steps n]` — renders
  preset/scenario `k` (after `n` steps in Stack Lab) to an image and exits.

## Stack Lab

- **Simulation window** — Run/Pause (space), Step (`n`; "steps per Step click" sets how many),
  Reset (`r`), steps per frame, and the time step `dt`: a logarithmic slider (Ctrl+click to type),
  dt / 2 and dt x 2 for bisecting a stability limit, and presets 1e-4 ... 1e-2. dt applies on the
  next step, also while running. Live world/solver knobs: gravity, error reduction parameter,
  max iterations, relaxation parameter, friction (relaxation) model.
- **Gravity** is applied by the viewer as a force `m g` per step: the repo's default
  `HardContactEulerLagrange` solver ignores `World::setGravity()` (body forces belong to the outer
  CFD driver). The world gravity is kept at 0 so other solvers do not apply it twice.
- **Ground plane** — a fixed pe plane at z = 0, drawn with Polyscope's tiled ground.
- **Scenarios** (staged, applied on Reset) — box tower, box pyramid, brick wall (offset rows),
  mixed-shape stack (box / cylinder / capsule / ellipsoid / sphere), boxes on a fixed ramp (shows
  tan(angle) vs mu), shapes dropped onto the ground. Box size, initial gap, lateral and yaw
  jitter with a seed, pair friction mu, restitution, density. pe adds the two materials' friction
  coefficients, so each body gets mu / 2.
- **Overlay** — after each frame's steps `MaxContacts::collide()` is re-run over all
  AABB-overlapping pairs (the collision system clears its contacts at the end of a step), so the
  markers show the contacts of the current configuration. Bodies can be colored by speed.
- **Diagnostics** — kinetic energy, the solver's maximum penetration, solver vs overlay contact
  count, and the drift of the initially highest body.
- **Ctrl + left-drag** a body to pull it with a damped spring.

## Pair Lab

Two bodies (sphere, box, capsule, cylinder, ellipsoid, plane), posed by drag fields or the
Polyscope transform gizmo. Every edit re-runs `MaxContacts::collide( A, B )`:

- **Overlay** — contact points (red = penetrating, yellow = within `contactThreshold`; or colored
  by vertex-face / edge-edge type) and the contact normals at world length. The normal points
  from `g2` to `g1`.
- **Contacts window** — one row per contact; click a row or a marker in the 3D view to select it.
  The header compares against `collide( B, A )` (count, position, distance, normal) and turns red
  when the two dispatch orders disagree. Note that for mixed pairs the dispatch reorders the
  arguments itself, so the comparison is most telling for same-type pairs.
- **Support witnesses** — `g1->support( -n )` and `g2->support( n )` with the segment between
  them, and for the selected contact the support gap along the normal next to the reported
  distance: for a convex pair with a consistent normal and depth the two agree. Against a flat
  face the support point is not unique (pe returns a corner), which is exactly the ambiguity
  `MaxContacts::compatibleContactPoint()` has to resolve.
- **Sweep window** — contact count and minimum distance over one pose degree of freedom of one
  body. Jumps in the count are manifold flicker, jumps in the distance are discontinuities of
  the routine. Drag the yellow line to scrub the pose through the sweep.
- **Copy as test case** — the pair as a C++ snippet (clipboard and stdout) with the currently
  generated contacts as comments, to turn a visual finding into a `tests/interface` regression
  test. The snippet assumes the `ContactLog` container of `pe_ellipsoid_contact_test.cpp`.
- **Presets** — box face-face (stacking), offset + yaw, crossed edges, corner-face, capsule on a
  box face, sphere on a box edge, box/cylinder on a plane, off-centre ellipsoid-box face, tilted
  ellipsoid pair.

Orientations use pe's Euler convention, `Quat( xangle, yangle, zangle )` (applied in the order
x, y, z); capsule and cylinder axes run along the body-frame x axis, the plane normal is the
body-frame z axis.

## Structure

- `contact_viewer.cpp` — Polyscope/ImPlot setup, command line, mode switch.
- `stack_lab.cpp` / `StackLab.h` — the Stack Lab mode.
- `pair_lab.cpp` / `PairLab.h` — the Pair Lab mode.
- `ContactOverlay.h` — `ContactLog`, the minimal recording container `MaxContacts::collide()`
  accepts, and its Polyscope overlay (used by both modes).
- `ShapeMeshes.h` — body-frame meshes of the primitives, `makeBodyMesh()`, `bodyTransform()`.

## Extending

- New shape: a `ShapeKind` entry, a case in `createBody()` / `createMesh()` / `createCall()`,
  the size widgets in `drawBodyControls()`, and a mesh generator in `ShapeMeshes.h`.
- New Pair Lab preset: one line in `presets()`.
- New Stack Lab scenario: a `ScenarioKind` entry and a `build*()` function calling `addBody()`.
