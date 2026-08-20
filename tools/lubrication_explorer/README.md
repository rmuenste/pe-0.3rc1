# Lubrication Force Explorer

Interactive, zero-build web tool for tuning the Kroupa-2016 lubrication model. It plots each
force/torque component vs gap and recomputes live as you drag the same lubrication knobs the
live viewer exposes. It is a **model** explorer (the analytic pair/wall laws), not a
simulation — no PE build, no engine, no server.

## Run

Just open `index.html` in a browser (double-click, or `file://…/index.html`). Plain classic
scripts, no ES modules, no CDN — the plotting library (uPlot) is vendored under `vendor/`,
so nothing needs installing or serving.

Sister tool: `tools/live_viewer/` (the Polyscope viewer that runs an actual simulation).

## Layout

A **4×2 grid**, one row per component (normal force, sliding force, sliding torque, twisting
torque); left column = the **continuous** model force, right column = the **per-step
effective** force (impulse/dt) as the solver actually applies it, with the semi-implicit /
explicit-capped scheme and the blend ramp. The per-step panel also draws the continuous force
as a faint dashed reference so the scheme's clamp and the blend roll-off are visible. Orchid
dashed rules mark the saturation onset `h_c = ε_c·a_ref` and the outer cutoff `h_cut`.

- **X-axis**: **log gap `h`** by default, so the near-contact→cutoff transition fills the plot
  instead of being crushed against a saturation plateau. Top-bar toggle switches to linear
  `a_ref/h` (Kroupa Fig. 7 framing, `= 1/ε` before saturation).
- **Y-axis**: dimensional [N] / [N·m] from the velocity sliders; a toggle switches to log
  (plots `|value|`). A separate toggle switches the **continuous** panels to Fig. 7
  dimensionless `F/(v·a·μ)` (torques `M/(v·a²·μ)`); the per-step panels are always dimensional.
- Panels appear/disappear with their switches (`tangential terms`, `twisting torque`);
  `force model = legacy` collapses to the normal panel only.

## Interactions

The scope array behaves like a multi-channel instrument sharing one gap axis:

- **Drag** a box to zoom; **wheel** to zoom the gap axis about the cursor (**Shift+wheel** zooms
  Y). Zooming one panel zooms **all** panels together (shared gap axis).
- **Double-click** any panel — or **Reset view** — to restore the full range on every panel.
- **Hover** any panel to drop a **linked crosshair on all 8**, with each trace's value read out
  in its legend at the same gap.
- Zoom is **preserved** while you keep dragging parameter sliders, so you can inspect a region
  and keep tuning; it re-fits automatically once you reset.

## Parameters

Mirrors the live viewer's lubrication set (fluid viscosity, the model switches, cutoffs /
regularization) plus the physical inputs a force plot needs: geometry (`pair ↔ wall`, radii),
the kinematic scalars (`vₙ` approach, `v_s` sliding, `v_cs` rotational sliding, `ω_twist`),
and the per-step controls (`dt`, effective mass `m_eff`, effective inertia `I_eff`, `α`
impulse cap). Ranges and log-scaling match `setupParamRegistry()` in
`tools/live_viewer/live_viewer.cpp`.

The explorer starts with **visualization defaults** rather than production defaults. For its
default 1 cm wall case, the contact blend half-width is 5% of `h_c` and the cutoff blend
half-width is 10% of `h_cut`; this makes both transitions readable without letting the outer
blend dominate the curve. **Visualization defaults** restores the complete illustrative
scenario. **Set engine defaults** loads the engine-owned lubrication values from
the lubrication stage / `SimulationConfig` (including `contact = 1e-9 m` and
`cutoff = 1e-3 m`) while preserving plot-only geometry, kinematics, mass/inertia and view.

The plot extends through `h_cut + lubricationHysteresis`, so the per-step curve displays the
complete outer fade to zero. The readout reports the dimensionless blend-width ratios.

**Intentionally not exposed** (no effect on a force-vs-gap curve): world damping/gravity,
solver ERP, mouse-spring, and AABB inflation (coarse-detection only). The `scheme`, `dt` and
mass knobs affect the **per-step** column only.

## Fidelity / sync contract

`lubforces.js` is a faithful, function-for-function port of the C++ source of truth, each
coefficient tagged with its source line:

- `pe/core/lubrication/LubricationModel.h` — coefficients, slip `f*`, saturation, wrench.
- `pe/core/lubrication/LubricationStage.h` — per-step impulse (semi-implicit /
  explicit-capped, `expClamp`), blend application.
- `pe/core/detection/fine/MaxContacts.h` — blend-weight ramp.
- `src/core/lubrication/Params.cpp` — outer cutoff.

**If you change the model in C++, change `lubforces.js` too**, then re-check: the tool runs a
built-in **self-check** (shown in the readout bar, hover for details) that reproduces the
Level-0 evidence asserted by `tests/interface/pe_lubrication_model_test.cpp` — the wall normal
plateau `F/(v·a·μ) = 6π·C_n(ε_c)·f* = 405 / 805 / 1605 / 3202` for
`ε_c = 0.02 / 0.01 / 0.005 / 0.0025`, and the pair-vs-wall leading-term ratio ≈ ¼.

To run the port headlessly (CI-style), the model layer has no DOM dependency:

```bash
cd tools/lubrication_explorer
node -e 'global.window={};new Function(require("fs").readFileSync("lubforces.js","utf8"))();
         const s=window.L.selfCheck();console.log(s.pass?"PASS":"FAIL");s.lines.forEach(l=>console.log(l));'
```

## Files

- `index.html` — UI shell (control desk + 4×2 scope array), all CSS inline.
- `lubforces.js` — the model port (`window.L`); pure functions, no DOM.
- `charts.js` — uPlot wiring (`window.Charts`): builds the 8 charts, log/linear scales, the
  linked cursor + x-zoom sync, the `h_c`/`h_cut` marker rules, and live `setData` updates.
- `app.js` — control registry, state, frame building, axis-mode toggles.
- `vendor/uPlot.iife.min.js`, `vendor/uPlot.min.css` — the plotting library (MIT), committed.

### Refreshing the vendored uPlot

```bash
cd tools/lubrication_explorer/vendor
curl -fsSLO https://cdn.jsdelivr.net/npm/uplot@1.6.32/dist/uPlot.iife.min.js
curl -fsSLO https://cdn.jsdelivr.net/npm/uplot@1.6.32/dist/uPlot.min.css
```

## Extending

- New parameter: add one entry to `REG` in `app.js` (getter/setter is just the `S` field).
- A structural change (visible panels / axis mode) triggers a chart rebuild in `charts.js`;
  everything else is a fast `setData` that preserves zoom. Keep that split when adding features.
- Export current knobs as a `SimulationConfig` JSON (the `lubrication*_` keys) is a natural
  future addition.
