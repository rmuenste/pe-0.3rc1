# Production Lubrication Component — Design

Status: **implemented — phases 1–4** (July 2026). Deviations from this design as written:
`PairWrench` splits `Fn`/`Ft` (semi-implicit scheme integrates the normal mode
separately); the paper's printed twisting-torque sign is corrected to the dissipative
form; the sliding-torque exponential guard uses the heuristic `K_s·a_ref²` rotational
stiffness; the solver-dependent World tests self-SKIP (exit 77) unless built with
`-Dpe_CONSTRAINT_SOLVER=pe::response::HardContactLubricated`; the Level-2 shear cell
measures `η_L` via the bulk lubrication-stress virial instead of the wall force balance
(PE plane walls carry no measurable reaction force); the Gondret restitution curve is
deferred because the hard-contact relaxation models are inelastic (see
`pe_lubrication_bounce_stokes_test.cpp`). MPI-parity test variant and Level 3 (CFD)
remain open. Original design text below. Target: replace the leading-order,
normal-only lubrication force in `HardContactLubricated` with the full pairwise
resistance model of Kroupa, Vonka, Soos & Kosek, *Utilizing the Discrete Element
Method for the Modeling of Viscosity in Concentrated Suspensions*, Langmuir 2016,
32, 8451−8460 (DOI 10.1021/acs.langmuir.6b02335), with runtime-switchable
features and a validation suite suitable for CFD-coupled production use.

Related notes: [lubrication-contacts.md](lubrication-contacts.md),
[lubrication-redesign.md](lubrication-redesign.md),
[adding-json-parameters.md](adding-json-parameters.md).

---

## 1. Goals and non-goals

Goals:

1. Full pairwise lubrication resistance set: normal squeeze force, tangential
   sliding force, sliding torque, twisting torque — for particle–particle and
   particle–wall pairs (paper eqs 12–19, from Dance & Maxey 2003).
2. Physically motivated regularization of the `h → 0` divergence via a slip
   length / critical distance `ε_c` with the Vinogradova correction `f*`
   (paper eq 20), replacing the ad-hoc absolute gap floor `minEpsLub_`.
3. Symmetric dissipation: force is linear in relative velocity on approach
   **and** separation (the current implementation drops the separation branch).
4. Every model feature individually switchable at runtime, including AABB
   inflation, with all parameters flowing through `SimulationConfig` (JSON).
5. Unconditionally stable time integration at CFD-scale timesteps
   (semi-implicit normal update), retiring the impulse cap as the primary
   stability mechanism.
6. A layered validation suite (unit → two-particle → bulk rheology →
   CFD-coupled) so the component can be trusted quantitatively in
   FeatFloWer-coupled particulate-flow runs.

Non-goals (for this iteration):

- Many-body lubrication (Stokesian-dynamics-style resistance matrices).
- Lubrication for non-spherical primitives (capsules, boxes, meshes). The
  architecture keeps the door open via `R_eff`, but only sphere–sphere and
  sphere–plane are in scope.
- Cavitation, non-Newtonian films, temperature-dependent viscosity.

What is deliberately **kept** from the current implementation: the
lubrication-contact flag/weight plumbing (`ContactTrait`), the blend ramps in
`MaxContacts`, the once-per-timestep explicit application before the hard
contact sweep, and the two MPI velocity synchronizations around the
lubrication loop. These are production-quality already.

---

## 2. Physical model

### 2.1 Notation and geometry

For a pair of bodies `i`, `j` with contact normal `n` (pointing from `i` to
`j`, PE convention), surface gap `h` (from fine detection, `c->getDistance()`):

- Reduced radius `a = R_i R_j / (R_i + R_j)`; sphere–plane: `a = R_sphere`.
- Relative separation `ε = h / a_ref`, where `a_ref = 2a` for sphere–sphere
  (recovers the paper's `ε = h/R_p` for equal spheres, where `a = R_p/2`) and
  `a_ref = R_sphere` for sphere–plane (matches the paper's wall equations).
- Relative velocity at the contact:
  `g = v_i − v_j + ω_i × r_i − ω_j × r_j` (with `r_i`, `r_j` the lever arms),
  normal part `v_rn = n · g`, translational sliding velocity
  `v_s = v_r − (v_r · n) n` with `v_r = v_i − v_j`, rotational sliding velocity
  `v_cs = ω_i × r_i − ω_j × r_j` projected tangentially, and relative spin
  `(ω_i − ω_j) · n` for the twisting term. Planes use their actual velocity
  and rotation rate (they may move, e.g. in a shear cell), not hard zeros.

### 2.2 Force and torque set

Particle–particle (paper eqs 12–15, equally sized spheres, `ε = h/R_p`):

```
F_nl = −6π η R_p (v_r·n) n [ ¼ ε⁻¹ − 9/40 ln ε − 3/112 ε ln ε ]
F_sl = −6π η R_p [ v_s (−1/6 ln ε) + v_cs (−1/6 ln ε − 1/12 ε ln ε) ]
M_sl = −8π η R_p² [ (n × v_s)(−1/6 ln ε − 1/12 ε ln ε)
                  + (n × v_cs)(−1/5 ln ε − 47/250 ε ln ε) ]
M_tl = −8π η R_p² [ (Ω_i − Ω_j)·n ] n ( 1/8 ε ln ε )
```

Particle–wall (paper eqs 16–19, `ε = h/R_p`):

```
F_nl^w = −6π η R_p (v_r·n) n [ ε⁻¹ − 1/5 ln ε − 1/21 ε ln ε ]
F_sl^w = −6π η R_p [ v_s (−8/15 ln ε − 64/375 ε ln ε)
                   + v_cs (−2/15 ln ε − 86/375 ε ln ε) ]
M_sl^w = −8π η R_p² [ (n × v_s)(−2/15 ln ε − 86/375 ε ln ε)
                    + (n × v_cs)(−2/5 ln ε − 66/125 ε ln ε) ]
M_tl^w = −8π η R_p² [ (Ω_i − Ω_j)·n ] n ( 1/2 ε ln ε )
```

Note the sign conventions of the paper are kept as printed; during
implementation each term must be checked against PE's normal-direction
convention (see [collision-detection-conventions.md](collision-detection-conventions.md))
with a dedicated unit test per term (dissipativity check: `F·g ≤ 0`,
`M·(ω_i−ω_j) ≤ 0` for the twisting term).

Forces act on body `i` with the opposite force on `j`; torques from `F` use
the lever arms (`r × F`) exactly as in the current code, and the pure
lubrication torques `M_sl`, `M_tl` are applied with opposite sign on `j`.

Unequal radii: the log-term coefficients above are exact only for `β = R_j/R_i
= 1` (sphere–sphere) and `β → ∞` (wall). For unequal spheres we evaluate the
sphere–sphere formulas with `ε = h/(2a)`; the leading `ε⁻¹` pole is then exact
(it equals the classical `6π η a² v_rn / h`) and the log terms are an `O(1)`
approximation. This is acceptable for moderately polydisperse suspensions; a
follow-up can swap in the full Jeffrey & Onishi `β`-dependent coefficients
behind the same interface without touching the solver.

### 2.3 Divergence treatment: slip correction and saturation

Following the paper exactly:

1. For `h ≥ h_c`: multiply **all** terms by the Vinogradova slip correction

   ```
   f*(h) = (h / 3h_c) [ (1 + h/6h_c) ln(1 + 6h_c/h) − 1 ]
   ```

   with slip length `h_c = ε_c · a_ref`. (`f* → 1` for `h ≫ h_c`; unit test.)
2. For `h < h_c`: evaluate all formulas (including `f*`) at the frozen
   separation `h = h_c`, i.e. forces **saturate** instead of diverging.

`ε_c` is the single physical regularization parameter (paper calibration:
`ε_c ≈ 0.1` reproduces Krieger/de Kruif hard-sphere viscosity data; the
implied slip lengths 10–40 nm match AFM measurements). The slip correction is
independently switchable so users can run pure saturation (`f* ≡ 1`), which is
what some reference codes do.

The legacy `minEpsLub_` gap floor remains only as a final numerical guard
(`h ← max(h, minEpsLub_)`) and should be irrelevant in practice since
`h_c ≫ minEpsLub_`.

### 2.4 Outer cutoff and blending

The existing blend machinery is reused unchanged: `MaxContacts` emits
lubrication contacts with a weight that ramps up at the hard-contact threshold
and down at the outer cutoff, and the solver scales the full wrench by that
weight.

Change: the outer cutoff becomes **relative to pair geometry**,
`h_cut = ε_cut · a_ref` (new parameter `lubricationCutoffFactor_`, default
`0.5`), instead of the absolute `lubricationThreshold = 1e-2`. Rationale:

- The lubrication expansion is valid for `ε ≪ 1`; an absolute cutoff is wrong
  for both very small and very large particles.
- **CFD double-counting control**: in coupled runs the lubrication force must
  only represent hydrodynamics the fluid solver cannot resolve.

Both constraints combine into a single effective cutoff:

```
h_cut = min( ε_cut · a_ref ,  c · Δx_CFD )        c ≈ 1–2
```

where `ε_cut` (`lubricationCutoffFactor_`) is the model-validity bound and the
`c · Δx` term is a mesh clamp (`lubricationMeshClampFactor_`, with `Δx`
supplied by the CFD side through the interface setup — PE does not know the
mesh). This one formula covers both intended coupling regimes:

- **Unresolved Euler–Lagrange** (point particles, `d_p ≲ Δx`): the fluid
  resolves no pairwise near-field at all — drag correlations couple each
  particle to the averaged flow and capture neighbor hindrance only
  statistically, never the squeeze film of a specific pair. `c · Δx` is huge,
  the `min` picks `ε_cut · a_ref`, and `ε_cut` is chosen from model validity
  alone (default `0.5`, up to ~1). Wall and tangential terms all active.
- **Marginally resolved** (fictitious-domain-style, ~8–16 cells per diameter):
  the grid resolves pair hydrodynamics down to gaps of ~1–2 cells; below that
  the mesh clamp binds (e.g. `d_p/Δx = 10`, `c = 2` → `h_cut = 0.4 R`). The
  coarser the resolution, the earlier lubrication takes over. Caveat: toward
  ~5 cells per diameter, `c · Δx` approaches the validity bound and an
  intermediate gap range is described well by neither the grid nor the
  asymptotics; the Level 3 mesh-resolution sweep (§4) calibrates `c` and
  detects where this starts to matter.

Unset mesh clamp (`lubricationMeshClampFactor_ = 0` or no `Δx` provided) means
no clamp — the correct behavior for unresolved EL and PE-only runs.

An absolute-cutoff fallback (`lubricationCutoffFactor_ = 0` → use legacy
absolute `lubricationThreshold`) preserves current behavior.

### 2.5 Time integration

All lubrication terms are linear in the relative velocities: the pair wrench
is `−R(h) · u` with `u = (v_rel, ω_rel)` and a positive semi-definite
resistance `R(h)`. Freezing `h` over one step, the normal mode obeys

```
m_eff dv_n/dt = −K_n(h) v_n,   K_n(h) = 6π η a_ref · f* · [¼ ε⁻¹ − …] (pair case)
```

**Semi-implicit (default) scheme**: integrate this mode exactly over `dt`:

```
v_n ← v_n · exp(−K_n dt / m_eff)
J_n  = m_eff (1 − exp(−K_n dt / m_eff)) |v_rn|      // applied impulse
```

This is unconditionally stable, never overshoots (`J_n < m_eff |v_rn|` — the
current impulse cap `α · m_eff · |v_rn|` is recovered as the `K_n dt → ∞`
limit with `α = 1`), and remains accurate when `K_n dt / m_eff = O(1)`, which
is exactly the CFD-coupled regime. The impulse is applied through the existing
`dv_`/`dw_` correction arrays, scaled by the blend weight, so nothing changes
downstream (including the mandatory second MPI velocity sync).

Tangential force and torques are `O(ln ε)` — orders of magnitude softer than
the `ε⁻¹` pole — and are applied explicitly. As a guard, the same exponential
clamp is applied per tangential mode using the corresponding scalar resistance
and the effective mass/inertia of the mode (cheap, and removes any residual
stiffness concern). The `alphaImpulseCap_` parameter is kept only for the
legacy explicit scheme.

Solid contact (`h ≤ 0`): unchanged — the hard-contact impulse solver takes
over; lubrication weight has ramped to zero by then. Unlike the paper we do
not need Hertz softness; the paper itself shows results are insensitive to
the contact stiffness model (their Fig. on Young's-modulus independence),
which supports combining their lubrication closure with PE's hard contacts.

---

## 3. Software design

### 3.1 New module: `pe/core/lubrication/LubricationModel.h`

A header-only, solver-agnostic model so the physics is unit-testable without a
`World`:

```cpp
namespace pe { namespace lubrication {

// Immutable snapshot of all model switches/parameters (filled once per step
// from Params; no singleton access inside the hot loop).
struct ModelConfig {
   bool enabled;              // master switch
   bool tangential;           // F_sl + M_sl terms
   bool twisting;             // M_tl term
   bool slipCorrection;       // Vinogradova f*
   bool resistSeparation;     // apply for v_rn > 0 as well
   bool wallTerms;            // use eqs 16-19 for sphere-plane (else pair eqs)
   int  scheme;               // SemiImplicit (default) | ExplicitCapped (legacy)
   real epsCritical;          // eps_c = h_c / a_ref          (default 0.1)
   real cutoffFactor;         // eps_cut = h_cut / a_ref      (default 0.5; 0 = legacy absolute)
   real minGap;               // final numerical floor        (legacy minEpsLub_)
   real alphaImpulseCap;      // legacy explicit scheme only
   real viscosity;            // fluid dynamic viscosity
};

struct PairKinematics { /* n, h, r1, r2, v1, v2, w1, w2, a_ref, wall flag */ };

struct PairWrench {
   Vec3 F;      // force on body i (−F on j)
   Vec3 M1;     // total torque on body i (lever-arm + M_sl + M_tl share)
   Vec3 M2;     // total torque on body j
};

// Pure function: full Kroupa-2016 wrench, before blend-weight scaling and
// before time integration. Deterministic, no globals.
PairWrench computeWrench( const PairKinematics&, const ModelConfig& );

// Scalar resistances used by the semi-implicit update.
real normalResistance   ( real h, real a_ref, bool wall, const ModelConfig& );
real slidingResistance  ( real h, real a_ref, bool wall, const ModelConfig& );
real twistingResistance ( real h, real a_ref, bool wall, const ModelConfig& );

} } // namespaces
```

`HardContactLubricated.h`'s lubrication block (currently ~170 lines inline,
`pe/core/collisionsystem/HardContactLubricated.h:2196-2365`) shrinks to:
gather kinematics → `computeWrench`/resistances → exponential update →
accumulate `dv_`/`dw_`. The geometry dispatch (sphere–sphere, sphere–plane,
plane–sphere) and `a_ref` computation stay where they are.

### 3.2 Runtime parameters: `SimulationConfig` (single source of truth)

All model parameters and switches live in `SimulationConfig`
(`pe/config/SimulationConfig.h`) and are parsed by
`SimulationConfig::loadFromFile` from the JSON input file, following the
existing pattern (see [adding-json-parameters.md](adding-json-parameters.md)).
New members + JSON keys (naming follows the existing `minEpsLub_` style):

| JSON key / member                | Type   | Default        | Meaning |
|----------------------------------|--------|----------------|---------|
| `lubricationEnabled_`            | bool   | `true`         | Master switch for lubrication forces |
| `lubricationModel_`              | string | `"kroupa2016"` | `"kroupa2016"` or `"legacy"` (current normal-only force, for A/B comparison) |
| `lubricationTangential_`         | bool   | `true`         | Sliding force `F_sl` + sliding torque `M_sl` |
| `lubricationTwisting_`           | bool   | `true`         | Twisting torque `M_tl` |
| `lubricationSlipCorrection_`     | bool   | `true`         | Vinogradova `f*` for `h ≥ h_c` |
| `lubricationOnSeparation_`       | bool   | `true`         | Also resist separating motion |
| `lubricationWallTerms_`          | bool   | `true`         | Use wall eqs 16–19 for sphere–plane |
| `lubricationEpsCritical_`        | real   | `0.1`          | `ε_c = h_c / a_ref` (saturation + slip length) |
| `lubricationCutoffFactor_`       | real   | `0.5`          | Outer cutoff `h_cut = ε_cut · a_ref`; `0` → legacy absolute threshold |
| `lubricationMeshClampFactor_`    | real   | `0` (off)      | Mesh clamp `c`: caps `h_cut` at `c · Δx_CFD` in resolved coupled runs; `Δx` set by the interface layer |
| `lubricationIntegration_`        | string | `"semi-implicit"` | `"semi-implicit"` or `"explicit-capped"` |
| `lubricationAabbInflation_`      | bool   | `true`         | Grow AABBs by the lubrication cutoff (coarse detection) |
| (existing) `minEpsLub_`          | real   | `1e-8`         | Final numerical gap floor |
| (existing) `alphaImpulseCap_`    | real   | `1.0`          | Cap factor, `explicit-capped` scheme only |
| (existing) `lubricationHysteresisDelta_`, `contactHysteresisDelta_` | real | unchanged | Blend half-widths |
| (existing) `fluidViscosity_`     | real   | unchanged      | `η` in all formulas |

### 3.3 Parameter flow and layering (why `lubrication::Params` stays)

`pe/core` cannot depend on `SimulationConfig` internals from hot paths (and
body classes like `SphereBase` should not pull `boost::filesystem` via the
config header). The existing two-level pattern is kept and extended:

```
JSON file ──loadFromFile──▶ SimulationConfig (pe/config, parsing + storage)
                                   │  applyLubricationConfig()   (one call,
                                   ▼   in interface setup / example startup)
                     pe::lubrication::Params (pe/core, plain getters — hot-path safe)
                                   +
                     CollisionSystem<HardContactLubricated> setters (per-solver state)
```

- `pe/core/lubrication/Params.h` grows getter/setter pairs mirroring every
  model switch above (plain `real`/`bool` statics in `Params.cpp`, exactly
  like the current three parameters).
- A new free function `pe::lubrication::applyFromSimulationConfig()` (declared
  in `pe/core/lubrication/Params.h`, defined in `src/core/lubrication/Params.cpp`
  — or in the interface layer if we want `pe/config` → `pe/core` dependency
  direction kept one-way) copies `SimulationConfig` values into `Params` and
  into the collision system. Every setup function (`setup_kroupa.h`,
  `sim_setup_serial.h`, examples) calls it once after `loadFromFile`.
- The solver snapshots `Params` into a `ModelConfig` **once per timestep**, so
  the per-contact loop reads a local struct, not singletons.

### 3.4 Switchable AABB inflation

Current state: `SphereBase::calcBoundingBox()` and `PlaneBase::calcBoundingBox()`
unconditionally pad by `contactThreshold + lubricationThreshold` (compile-time
constant from `pe/core/Thresholds.h`).

Design:

- Bounding-box code queries a single runtime helper
  `pe::lubrication::aabbPadding(real bodyRadius)`:
  - returns `0` when `lubricationAabbInflation_` is off or lubrication is
    disabled entirely;
  - returns `cutoffFactor × bodyRadius` in relative-cutoff mode (per-body
    padding; since `a_ref ≤ 2·min(R_i,R_j)` and both AABBs are padded, pair
    coverage is conservative);
  - returns the legacy absolute `lubricationThreshold` when
    `lubricationCutoffFactor_ = 0`. For planes (no radius) the padding is
    `cutoffFactor × R_max` with `R_max` tracked in `Params` as the largest
    sphere radius registered so far (updated in `Sphere` construction), or the
    absolute threshold in legacy mode.
- Bounding boxes are recomputed every position update, so toggling before a
  run (the supported use) needs no extra invalidation logic.
- Why switchable: the inflation enlarges coarse-detection candidate sets and
  hence contact-list size and MPI traffic. Pure granular runs (no fluid) and
  debugging sessions want it off; the switch also lets us measure its real
  cost in benchmarks instead of guessing.
- Consistency guard: if lubrication is enabled but inflation is off, emit a
  one-time warning that pairs may enter the lubrication band undetected
  between coarse-detection updates (this is a legitimate expert mode when
  `ε_cut` is small, but it must be loud).

The `PE_LUBRICATION_CONTACTS` compile-time gate in the detection code stays
(it selects the `HardContactLubricated` code paths); everything else is
runtime.

### 3.5 Diagnostics (small but necessary for production)

Behind `verbose_`/a new `lubricationDiagnostics_` flag: per-step counters
(number of lubrication contacts, number of saturated pairs `h < h_c`, max
normal impulse, total lubrication dissipation `Σ F·g dt`). The dissipation
scalar is the quantity the rheology validation integrates, so it doubles as a
model-health metric in CFD runs (it must be ≤ 0 by construction; a positive
value is a sign-convention bug caught for free).

---

## 4. Validation plan

Layered; each level gates the next. Levels 0–2 become CTest targets next to
the existing `pe-interface-serial-*` cases; level 3 runs in the FeatFloWer
coupling.

### Running the Level 0–2 tests

The World-based tests (Level 1) require the lubricated constraint solver. Either
set `pe_CONSTRAINT_SOLVER` to `pe::response::HardContactLubricated` in
`pe/config/Collisions.h` (the current repository default), or override it per
build without touching the header:

```bash
cmake -S . -B build-interface-tests-hcl -DCMAKE_BUILD_TYPE=Release -DPE_LIBRARY_TYPE=STATIC \
      -DBUILD_TESTING=ON -DPE_USE_JSON=ON -DPE_USE_EIGEN=ON \
      -DCMAKE_CXX_FLAGS="-Dpe_CONSTRAINT_SOLVER=pe::response::HardContactLubricated"
cmake --build build-interface-tests-hcl -j
```

Under any other solver the Level 1 tests self-report as SKIPPED (exit code 77);
the Level 0 test runs in every build.

```bash
# Level 0 (pure math) + Level 1 gates:
ctest --test-dir build-interface-tests-hcl -R pe-lubrication --output-on-failure
#   pe-lubrication-model                (Level 0)
#   pe-lubrication-legacy-regression   (bit-for-bit legacy gate)
#   pe-lubrication-two-sphere-approach (Level 1.1/1.3: Fig. 7, suction switch)
#   pe-lubrication-bounce-stokes       (Level 1.2/1.4/1.5: impact, dt sweep, tangential)

# Plot data (PE_LUB_CSV=1 makes the Level 1 binaries emit CSV on stdout):
PE_LUB_CSV=1 ./build-interface-tests-hcl/tests/interface/pe_lubrication_two_sphere_approach_test > approach.csv
./tests/interface/plot_two_sphere_approach.py approach.csv             # pair curves, zoomed
./tests/interface/plot_two_sphere_approach.py approach.csv 5000 wall   # the paper's exact Fig. 7 frame
PE_LUB_CSV=1 ./build-interface-tests-hcl/tests/interface/pe_lubrication_bounce_stokes_test > bounce.csv
```

Level 2 (shear-cell skeleton; needs an examples build, named target only —
never build the full examples tree):

```bash
cmake -S . -B build-examples -DCMAKE_BUILD_TYPE=Release -DPE_LIBRARY_TYPE=STATIC \
      -DPE_USE_JSON=ON -DPE_USE_EIGEN=ON -DPE_BUILD_EXAMPLES=ON
cmake --build build-examples -j --target shear_cell_kroupa
./build-examples/examples/shear_cell_kroupa/shear_cell_kroupa 0.3 2000 100   # phi, steps, gammaDot
# one shear_cell_eta.csv per phi, then:
./examples/shear_cell_kroupa/evaluate_viscosity.py run_phi*.csv   # eta(phi) vs Krieger-Dougherty
```

The plotting scripts need matplotlib (e.g. `python3 -m venv venv && venv/bin/pip
install matplotlib`; the system Python is PEP-668-managed).

### Level 0 — unit tests (pure math, no World)

- Each resistance term against hand-evaluated reference values at several `ε`
  (both pair and wall variants), including the `ε ln ε` corrections.
- `f*` limits: `f* → 1` for `h/h_c → ∞`; monotone; saturation continuity at
  `h = h_c` (force continuous, by construction of the freeze).
- Dissipativity for random kinematic states: `F·g ≤ 0`, twisting-torque power
  ≤ 0, for all switch combinations.
- Semi-implicit update: `J_n < m_eff |v_rn|` always; matches explicit force
  `× dt` in the `K_n dt/m_eff → 0` limit; velocity sign never flips.
- Legacy-model regression: `lubricationModel_="legacy"` reproduces current
  `HardContactLubricated` forces bit-for-bit (protects existing users).

### Level 1 — two-particle canonical tests

1. **Paper Fig. 7 reproduction (calibration anchor).** Push two spheres
   together with constant force; plot `F/(v R_p η)` vs `R_p/h` for
   `ε_c ∈ {0.0025, 0.005, 0.01, 0.02}` and no-slip. Must show the linear
   no-slip line, the `f*` departure, and the plateau at saturation. This
   directly validates the divergence treatment against the paper and,
   qualitatively, the Bonaccurso AFM data.

   > **Geometry caveat (found during implementation).** The paper's Fig. 7
   > caption says "approach of two spheres", but the plotted curves are the
   > **particle–wall** resistance set (eq 16, leading `ε⁻¹`) — the paper treats
   > "the wall as particle j with velocity and rotation rate equal to zero" for
   > the comparison with Bonaccurso's sphere-on-flat AFM data. The equal-sphere
   > pair set (eq 12, leading `¼ε⁻¹`) sits a factor ≈ 4 lower (3.7–4.0 including
   > log terms); no reading of "approach velocity" (relative → 1×, single-particle
   > → 2×) reconciles the pair coefficients with the figure. Verified: with the
   > wall set our plateaus land at 405/805/1605/3202 for
   > `ε_c = 0.02/0.01/0.005/0.0025` vs ≈ 400/800/1600/3200 in the figure, and the
   > no-slip slope is `6π` (exits their 0–5000 frame at `R_p/h ≈ 265`).
   > `pe_lubrication_two_sphere_approach_test` therefore emits **both**
   > configurations in CSV mode (pair curves and `wall …` curves); use
   > `plot_two_sphere_approach.py approach.csv 5000 wall` to reproduce the
   > figure literally. Also note the two calibrations live in different
   > geometries: Fig. 7's slip lengths (`ε_c` ~ 0.0025–0.02) are the AFM/wall
   > context, while the production default `lubricationEpsCritical_ = 0.1`
   > follows the bulk-rheology fit (Fig. 3, genuine sphere–sphere pairs).
2. **Immersed normal bounce (Gondret/Joseph benchmark).** Sphere falling onto
   a plane in quiescent fluid (PE-only: gravity + Stokes drag + lubrication;
   later repeated CFD-coupled): effective restitution vs Stokes number must
   collapse onto the experimental curve — `e ≈ 0` below `St ≈ 10`, rising
   toward the dry value above. This is *the* standard test for immersed
   collision modeling and the single most important gate for CFD use.
3. **Symmetric dissipation.** Oscillating gap (prescribed motion): net work
   over a cycle strictly negative with `lubricationOnSeparation_=true`;
   roughly halved dissipation when off (documents what the switch does).
4. **Timestep robustness.** Repeat test 2 with `dt` spanning 3 decades into
   the stiff regime (`K_n dt/m_eff up to ~10²`): semi-implicit scheme must stay
   stable and converge monotonically; document where `explicit-capped`
   degrades.
5. **Tangential sanity.** Sphere translating parallel to a wall at fixed gap:
   force/torque ratios against eqs 17–18; angular velocity induced on a free
   sphere sheared past another matches sign/order of the analytic coupling.

### Level 2 — bulk rheology (the paper's headline result)

Shear cell as in the paper: moving upper wall (`setup_kroupa.h` already
targets this class of experiment), periodic lateral boundaries, `N ≈ O(10³)`
monodisperse spheres, high shear rate.

- Measure suspension viscosity via the wall force balance (paper eqs 25–33):
  `η = η_H + η_L`, with `η_L` summed from wall-lubrication sliding forces —
  the diagnostics dissipation/force accumulators from §3.5 provide `F_L`.
- Acceptance: `η(φ)` for `φ ∈ [0.05, 0.5]` with `ε_c = 0.1` tracks
  Krieger–Dougherty / Maron–Pierce and the paper's Fig. 3 within the scatter
  of the experimental data sets; `η` independent of shear rate and of `R_p`
  (hard-sphere invariances, paper's key checks); tangential terms ON raises
  `η` at high `φ` (Fig. 6 trend).
- This is the calibration point for the default `ε_c`; the run also doubles
  as a long-horizon stability soak (no drift in kinetic energy, no
  contact-count blowup).

### Level 3 — CFD-coupled (FeatFloWer)

Only after Levels 0–2 pass:

1. **Terminal velocity regression** (`setup_el_terminal_velocity` exists):
   lubrication must not alter single-particle terminal velocity (no pair →
   no force) — a null test against double-counting in the coupling plumbing.
2. **Drafting–kissing–tumbling** (`setup_dkt` exists): two sedimenting
   spheres; compare kissing time and post-tumbling separation against
   published DKT benchmarks with/without lubrication; the near-contact phase
   is where the model acts.
3. **Immersed bounce, CFD-resolved** (repeat Level 1.2 with the real fluid):
   restitution-vs-Stokes curve again; this exposes double-counting, because
   the CFD already resolves part of the approach hydrodynamics. Tune/verify
   the outer cutoff guidance (`h_cut` vs CFD cell size, §2.4) here and record
   the recommended `lubricationCutoffFactor_` per mesh-resolution regime in
   the docs.
4. **Sheared suspension at moderate φ** through the coupling, cross-checked
   against the Level 2 PE-only result — agreement bounds the coupling error.

### MPI parity (orthogonal axis)

Every Level 1–2 test also runs on 2–4 ranks with a domain boundary through
the contact zone; results must match serial to solver tolerance. The existing
second velocity sync is load-bearing here; the tests make its necessity
regress-able.

---

## 5. Implementation phases

1. **Model core**: `LubricationModel.h` + Level 0 unit tests (pure math; no
   solver changes). Includes the legacy model as a code path.
2. **Config plumbing**: `SimulationConfig` members + JSON parsing +
   `lubrication::Params` extension + `applyFromSimulationConfig()`; AABB
   inflation switch + `aabbPadding()` helper; one-time consistency warning.
3. **Solver integration**: replace the inline force block in
   `HardContactLubricated::resolveContacts()` with the model call and the
   semi-implicit update; keep both MPI syncs; add diagnostics counters.
4. **Validation**: Level 1 tests as CTest targets (extend
   `examples/basic_lubrication` and add a two-sphere approach driver); Level 2
   shear-cell example + evaluation script; MPI parity variants.
5. **Docs**: update [lubrication-contacts.md](lubrication-contacts.md) with
   the new formulas/parameters, retire the superseded parts of
   [lubrication-redesign.md](lubrication-redesign.md), record calibrated
   defaults and CFD cutoff guidance from Level 3.

Backward compatibility: `lubricationModel_="legacy"` +
`lubricationCutoffFactor_=0` reproduces today's behavior exactly; defaults
move to the new model, which is the intended production configuration.

---

## 6. Risks / open points

- **Sign conventions**: the paper's normal/velocity conventions differ from
  PE's contact conventions; every term gets a dissipativity unit test rather
  than trusting transcription.
- **Unequal radii**: log-term coefficients are `β`-approximate (§2.2). If
  Level 2 with polydisperse packings shows sensitivity, upgrade to Jeffrey &
  Onishi coefficients behind the same `computeWrench` interface.
- **Twisting/sliding torques on hard contacts**: near the blend band both the
  lubrication torques and the hard-contact friction act; the blend weights
  already partition normal forces smoothly, and the same weight scales the
  torques — Level 1.5 verifies no double-counted tangential resistance.
- **Wall padding with `R_max`** (§3.4) is conservative for strongly
  polydisperse systems; if candidate-set growth is measurable, refine to
  per-pair padding in the coarse detector rather than per-body.
