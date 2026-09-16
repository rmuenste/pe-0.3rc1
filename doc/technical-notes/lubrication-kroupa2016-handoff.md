# Kroupa-2016 Lubrication — Agent Handoff

Status handoff for continuing the production lubrication work (July 2026,
branch `feature/kroupa2016-lubrication`, commits `8667c2a` + `6f20b21`).
Audience: an agent (or developer) starting fresh with no session context.

## 1. Where the context lives — read in this order

1. [lubrication-production-design.md](lubrication-production-design.md) — the
   **authoritative design + validation plan**, with implementation status and
   deviations in its Status header, and "Running the Level 0–2 tests" in §4.
2. [lubrication-contacts.md](lubrication-contacts.md) — current-state reference:
   force model summary, the full runtime parameter table (12 JSON keys),
   cutoff/AABB semantics, legacy-mode recipe, validation inventory.
3. The paper (repo root):
   `utilizing-the-discrete-element-method-for-the-modeling-of-viscosity-in-concentrated-suspensions.pdf`
   (Kroupa, Vonka, Soos, Kosek, Langmuir 2016, 32, 8451−8460). Pair resistances
   eqs 12–15, wall eqs 16–19, slip correction eq 20, viscosity evaluation eqs 25–33.

Code map:

| What | Where |
|---|---|
| Model physics (header-only, pure) | `pe/core/lubrication/LubricationModel.h` |
| Runtime parameter store + cutoff/AABB helpers | `pe/core/lubrication/Params.{h,cpp}` (`src/core/...`) |
| Solver integration (lubrication loop, between the two `synchronizeVelocities()`) | `pe/core/collisionsystem/HardContactLubricated.h`, search `lubCfg` |
| Contact emission + blend weights + relative cutoff | `pe/core/detection/fine/MaxContacts.h` (`PE_LUBRICATION_CONTACTS` branches) |
| AABB inflation (switchable) | `SphereBase::calcBoundingBox`, `src/core/rigidbody/PlaneBase.cpp`, via `lubrication::aabbPadding()` |
| JSON parameters | `pe/config/SimulationConfig.h` + `src/config/SimulationConfig.cpp` (`lubrication*_` keys) |
| Config→engine wiring (single point) | `applyOptionalLubricationParams()` in `pe/interface/setup_optional_collision_params.h` |
| Tests + plot script | `tests/interface/pe_lubrication_*.cpp`, `tests/interface/plot_two_sphere_approach.py` |
| Level-2 skeleton | `examples/shear_cell_kroupa/` (+ `evaluate_viscosity.py`) |

Essential gotchas (each bit us once):

- **Solver selection is compile-time** (`pe/config/Collisions.h`,
  `pe_CONSTRAINT_SOLVER`; currently `HardContactLubricated`). The Level-1 World
  tests self-SKIP (exit 77) under any other solver; a per-build override is
  `-DCMAKE_CXX_FLAGS="-Dpe_CONSTRAINT_SOLVER=pe::response::HardContactLubricated"`.
- **Paper Fig. 7 is particle–WALL despite its "two spheres" caption** (wall
  resistance set, leading `ε⁻¹`, ≈4× the pair set) — see the geometry-caveat
  callout in the design note §4 Level 1.1 before comparing anything to Fig. 7.
- **The hard-contact relaxation models are inelastic** (no material
  restitution) — the true Gondret e(St) curve is unreachable until a
  restitution-capable contact path exists (`pe_lubrication_bounce_stokes_test.cpp`
  header explains what is asserted instead).
- **ε_c calibrations live in different geometries**: Fig. 7's slip lengths
  (0.0025–0.02) are the AFM/wall context; the production default
  `lubricationEpsCritical_ = 0.1` follows the bulk-rheology fit (paper Fig. 3).
- Legacy behavior is preserved bit-for-bit via
  `{"lubricationModel_": "legacy", "lubricationCutoffFactor_": 0.0}` and
  regression-gated — do not break `pe-lubrication-legacy-regression`.

## 2. Validation already done (all green)

Build + run: see "Running the Level 0–2 tests" in the design note. In short:
`ctest --test-dir <build> -R pe-lubrication --output-on-failure`.

- **Level 0** `pe-lubrication-model`: every resistance coefficient vs
  hand-evaluated values (pair + wall), slip-factor limits/monotonicity,
  saturation continuity, dissipativity on 2000 random states (all switch
  combinations), semi-implicit impulse bounds, cutoff/padding helpers.
  Note: the paper's printed twisting-torque sign (eq 15/19) is anti-dissipative
  and was corrected; the tests enforce power ≤ 0.
- **Level 1** (through the full solver pipeline):
  - `pe-lubrication-legacy-regression` — bit-for-bit trajectory gate vs the
    pre-change solver (references embedded in the test).
  - `pe-lubrication-two-sphere-approach` — Fig. 7 pointwise: measured
    resistance matches the model to <1%, slip departure, saturation plateaus
    monotone in ε_c, switchable suction. CSV mode (`PE_LUB_CSV=1`) emits pair
    AND `wall …` curves; wall plateaus 405/805/1605/3202 for
    ε_c = 0.02/0.01/0.005/0.0025 match the paper's figure to the linewidth
    (`plot_two_sphere_approach.py approach.csv 5000 wall`).
  - `pe-lubrication-bounce-stokes` — immersed wall impact: dissipativity
    (v_out ≤ v_in), monotone viscosity attenuation of the near-wall impact
    speed, lubrication on/off effect, dt robustness over 3 decades
    (semi-implicit scheme), wall tangential force / forward-rolling torque /
    twist-decay signs and switches.
- **Level 2 (smoke only)**: `shear_cell_kroupa` at φ = 0.2 gave
  η_L ≈ 2×10⁻⁴ Pa·s ≈ 0.2·η_f via the lubrication-stress virial — plausible
  order vs the paper; NOT yet a converged result.
- MPI+HCL and non-EL builds **compile** (portability fixes in
  `pe/interface/el_hydro_compat.h` + optional shims); no MPI run-level test yet.

## 3. Open points (priority order)

1. **Level-2 η(φ) campaign** (compute, not code): sweep φ = 0.05…0.5 with
   `shear_cell_kroupa`, long steady-state window, average η_L per φ,
   `evaluate_viscosity.py` vs Krieger–Dougherty/Maron–Pierce (paper Fig. 3).
   Acceptance criteria are written in the design note §4 Level 2 (η tracks the
   correlations with ε_c = 0.1; shear-rate and R_p invariance; tangential
   terms ON raises η at high φ — one-switch rerun with
   `lubricationTangential_=false` reproduces the Fig. 6 trend). Improvements
   likely needed: longer runs, seeding at high φ (current random seeding stalls
   above ~0.45), possibly periodic boundaries instead of side walls.
2. **MPI parity test**: 2-rank variant of a Level-1 scene with the domain
   boundary through the contact zone; serial vs MPI trajectories must match to
   solver tolerance. Needs an MPI+HCL test build; model on
   `tests/interface/pe_frozen_field_trace_mpi.cpp`. The two
   `synchronizeVelocities()` calls around the lubrication loop are the
   load-bearing pieces to regress.
3. **Level 3, FeatFloWer-coupled**: (a) terminal-velocity null test
   (`setup_el_terminal_velocity`) — lubrication must not change a single
   particle; (b) DKT (`setup_dkt`) with/without lubrication vs published
   benchmarks; (c) CFD-resolved bounce to calibrate the mesh clamp
   (`lubricationMeshClampFactor_`; `lubrication::setMeshDx()` is WIRED:
   the CFD layer pushes its reduced global h_min through the extern-C
   entry `set_lubrication_mesh_dx` (`pe/interface/c_interface_queries.h`)
   from FeatFloWer's ComputeCFL, setup-function-agnostic and identical on
   every rank; arming prints a one-time `PE_LUB_MESHDX` line, and a
   configured clamp with no pushed dx warns loudly at the first stage
   sweep instead of silently degrading to the bare relative cutoff).
   Record recommended cutoff settings per mesh-resolution regime in the docs
   (design note §2.4 has the two coupling regimes).
4. **Enhancements**: Jeffrey–Onishi β-dependent log coefficients for strongly
   polydisperse packings (swap inside `computeWrench`, interface unchanged);
   a restitution-capable hard-contact path to unlock the true Gondret e(St)
   gate (the bounce test already records e(St) CSV for that day).

Smaller loose ends: `applyOptionalLubricationParams()` is wired in all serial
setups and `setup_kroupa.h` only — other MPI setups need the same one-liner;
`examples/basic_lubrication` still calls the deprecated no-op stubs
(`setLubrication/setSlipLength/setMinEps`) and could be modernized.
