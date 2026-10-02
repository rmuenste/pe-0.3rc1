# PE contact viewer

Interactive Polyscope harness for the narrow-phase contact generation
(`pe::detection::fine::MaxContacts`). It calls the fine collision detector directly on the
bodies and records what it emits, so it works with every constraint solver configuration and
needs no time stepping to show a contact manifold.

Two modes in one binary, switched in the **Mode** window:

- **Stack Lab** (default) — run / pause / step a small simulation on a visible ground plane.
- **Pair Lab** — pose two bodies, inspect their contacts, and simulate the pair from that pose.

Both modes share the simulation controls (Run/Pause/Step/Reset, `dt`, solver settings,
Ctrl + left-drag), described under Stack Lab.

Known problems, gaps in the contact generation and solver behaviour that affects what you see:
`contact-issues.md`.

## Build

```bash
cmake -S . -B build-viewer -DPE_BUILD_CONTACT_VIEWER=ON
cmake --build build-viewer --target pe_contact_viewer -j
./build-viewer/tools/contact_viewer/pe_contact_viewer
```

Polyscope and ImPlot are downloaded via `FetchContent` at configure time
(`tools/cmake/ViewerDeps.cmake`, shared with `tools/live_viewer`); nothing is added to the core
PE build. For the X11 development packages GLFW needs, see `tools/live_viewer/README.md`.

The mesh shape of Pair Lab needs CGAL for its DistanceMap (`doc/technical-notes/build-with-cmake.md`,
"CGAL and DistanceMap Builds"). A CGAL build of the viewer, reusing a CGAL that an earlier
`PE_USE_CGAL=ON` build already installed under its `extern/cgal/install`:

```bash
cmake -S . -B build-viewer-cgal -G Ninja -DCMAKE_BUILD_TYPE=Release -DPE_USE_CGAL=ON \
      -DCGAL_DIR=$PWD/build_ninja_cgal/extern/cgal/install/lib/cmake/CGAL \
      -DPE_BUILD_CONTACT_VIEWER=ON -DPE_BUILD_EXAMPLES=OFF
cmake --build build-viewer-cgal --target pe_contact_viewer -j
```

Without `CGAL_DIR` CMake clones and builds CGAL into the new tree. Note that the CGAL option is
`PE_USE_CGAL`; a tree configured with the former option name `CGAL=ON` builds without CGAL. The
viewer works without CGAL too: meshes then have no DistanceMap and go through GJK/EPA as if they
were convex (the mesh controls say so).

Command line:

- `pe_contact_viewer [--pair | --stack]` — start mode (default Stack Lab).
- `pe_contact_viewer --smoke` — headless self check of both modes on Polyscope's mock OpenGL
  backend: every Pair Lab preset and all 36 shape pairs, the gizmo pose round trip, and every
  Stack Lab scenario stepped for 1 s (3 s for the drop), plus a cylinder drop that must end
  resting on the ground, and a 2 s Pair Lab simulation (cylinder standing on a box over the
  ground) that must end at rest and be restored exactly by Reset. With CGAL also the torus
  presets: no contact for the sphere in the hole, contacts with the upward normal for the sphere
  on the tube, and the SDF grid through a frame. Prints contacts, a contacts-per-pair
  matrix (a `0` for an overlapping pair means the routine generates nothing; plane-plane is
  0 by design) and per-scenario stability numbers.
- `pe_contact_viewer [--pair | --stack] --screenshot out.png [--preset k] [--steps n]` — renders
  preset/scenario `k` after `n` steps to an image and exits.

## Stack Lab

- **Simulation window** — Run/Pause (space), Step (`n`; "steps per Step click" sets how many),
  Reset (`r`), steps per frame, and the time step `dt`: a logarithmic slider (Ctrl+click to type),
  dt / 2 and dt x 2 for bisecting a stability limit, and presets 1e-4 ... 1e-2. dt applies on the
  next step, also while running. Live world/solver knobs: gravity, error reduction parameter,
  max iterations, relaxation parameter, friction (relaxation) model.
- **Gravity** — the "gravity z" slider; the viewer applies it as a force `m g` per step.
- **Ground plane** — a fixed pe plane at z = 0, drawn with Polyscope's tiled ground.
- **Scenarios** (staged, applied on Reset) — box tower, triangular wall (broad bottom row, one
  box fewer per row, one box on top), square wall (N x N boxes, straight columns), brick wall
  (offset rows),
  mixed-shape stack (box / cylinder / capsule / ellipsoid / sphere), boxes on a fixed ramp (shows
  tan(angle) vs mu), shapes dropped onto the ground, upright cylinder stack (box / cylinder /
  cylinder / box ..., exercises the box-cylinder and cylinder-cylinder face manifolds). Box
  size, initial gap, lateral and yaw jitter with a seed, friction mu (the contact friction of a
  pair), restitution, density. The triangular and square walls have a side gap between
  neighbours (default 0, touching); their rows are laid out from each box's width after its yaw
  jitter, so boxes never start overlapping, and the lateral jitter acts across the wall only.
- **Overlay** — after each frame's steps `MaxContacts::collide()` is re-run over all
  AABB-overlapping pairs (the collision system clears its contacts at the end of a step), so the
  markers show the contacts of the current configuration. Bodies can be colored by speed.
- **Diagnostics** — kinetic energy, the solver's maximum penetration, solver vs overlay contact
  count, and the drift of the initially highest body.
- **Ctrl + left-drag** a body to pull it with a damped spring.

## Pair Lab

Two bodies (sphere, box, capsule, cylinder, ellipsoid, plane, mesh), posed by drag fields or the
Polyscope transform gizmo, over an optional ground plane. Every edit re-runs
`MaxContacts::collide( A, B )`:

- **Mesh** — a torus (major radius R, minor radius r, segment counts; hole axis = body z), the
  simplest closed non-convex shape, with a DistanceMap (resolution and padding as in
  `enableDistanceMapAcceleration()`; the controls apply on Enter since a rebuild takes a moment,
  the build time is shown). With the DistanceMap, contacts of the mesh with any primitive, the
  ground plane or another mesh come from the signed distance field
  (`MaxContacts::collideTMeshWithDistanceMap()` for primitives): surface samples of the primitive
  looked up in the field, clustered into a manifold. The "distance map (SDF grid)" overlay option
  draws the field as a Polyscope volume grid in the body frame with its zero isosurface; a slice
  plane (Polyscope's View menu) shows the signed distance inside. "copy as test case" refers to
  the torus generator of `tests/interface/pe_primitive_mesh_distancemap_test.cpp`.

- **Ground plane** — on by default and kept just under the posed pair ("keep under the pair");
  untick that to set the height by hand, or switch the ground off. Its contacts with A and B are
  drawn too and counted in the Contacts window.
- **Simulation window** — the pose is the initial state: Run/Pause/Step start the simulation from
  it, Reset (`r`) returns to it. "Start from a touching state" (on by default) moves penetrating
  bodies apart before the first step; the presets penetrate by 0.01 for the static view, and the
  solver would otherwise launch them (see `contact-issues.md`). Per body: **fixed** (immovable, e.g. a base or a ramp) and an
  initial linear / angular velocity. While t > 0 the pose, shape and ground editors and the Sweep
  window are locked (they act on the posed state); the overlay, the Contacts window and
  "copy as test case" follow the current state, so a contact seen at some step can be exported
  directly. History plots: A-B minimum distance, A-B contact count and kinetic energy over time.
- **Ctrl + left-drag** a body to pull it with a damped spring (while running or stepping).

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
  face the support point is not unique, so the witness may sit on a corner of that face.
- **Sweep window** — contact count and minimum distance over one pose degree of freedom of one
  body. Jumps in the count show manifold flicker, jumps in the distance discontinuities. Drag the
  yellow line to scrub the pose through the sweep.
- **Copy as test case** — the pair as a C++ snippet (clipboard and stdout) with the currently
  generated contacts as comments, to turn a visual finding into a `tests/interface` regression
  test. The snippet assumes the `ContactLog` container of `pe_ellipsoid_contact_test.cpp`.
- **Presets** — box face-face (stacking), offset + yaw, crossed edges, corner-face, capsule on a
  box face, sphere on a box edge, box/cylinder on a plane, off-centre ellipsoid-box face, tilted
  ellipsoid pair, and the multi-point cylinder cases: cylinder standing / lying on a box, box on
  a cylinder cap, coaxial cylinders, parallel lying cylinders, cylinder standing on a lying one,
  capsule on a cylinder cap; and with the torus mesh: sphere / box / capsule / cylinder /
  ellipsoid on the tube, and a sphere in the hole (no contact: the convex hull would contain it).

Orientations use pe's Euler convention, `Quat( xangle, yangle, zangle )` (applied in the order
x, y, z); capsule and cylinder axes run along the body-frame x axis, the plane normal is the
body-frame z axis.

## Structure

- `contact_viewer.cpp` — Polyscope/ImPlot setup, command line, mode switch.
- `sim_controls.cpp` / `SimControls.h` — shared simulation controls: step/run state, `dt`, solver
  settings, gravity as a force, the mouse spring, keyboard shortcuts.
- `stack_lab.cpp` / `StackLab.h` — the Stack Lab mode.
- `pair_lab.cpp` / `PairLab.h` — the Pair Lab mode.
- `ContactOverlay.h` — `ContactLog`, the minimal recording container `MaxContacts::collide()`
  accepts, and its Polyscope overlay (used by both modes).
- `ShapeMeshes.h` — body-frame meshes of the primitives, `makeBodyMesh()`, `bodyTransform()`.
- `contact-issues.md` — open problems and observations (not usage).

## Extending

- New shape: a `ShapeKind` entry, a case in `createBody()` / `createMesh()` / `createCall()`,
  the size widgets in `drawBodyControls()`, and a mesh generator in `ShapeMeshes.h`.
- New Pair Lab preset: one line in `presets()`.
- New Stack Lab scenario: a `ScenarioKind` entry and a `build*()` function calling `addBody()`.
