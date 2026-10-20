# Impact lab: the two restitution schemes side by side

`pe_impact_lab` runs the same small impact scene under both restitution schemes of
`HardContactSemiImplicitTimesteppingSolvers` and plays the two results back together:
the selected scheme solid, the other one transparent, with the energies plotted over time.

- **Newton target in the solve** (`setRestitutionPostPass( false )`, the original scheme): an
  impacting contact's normal target inside the relaxation is `-e gdot_n-`.
- **Capped post-pass** (`setRestitutionPostPass( true )`, the default): the relaxation stays
  inelastic and `setRestitutionIterations()` sweeps (default 4) add the bounce afterwards, with
  the separating part of each contact's impulse capped at `e` times its compression impulse
  and clamped at zero.

Single contacts give the same answer under both. The lab's scenes all have several contacts
of one body impacting in the same step, which is where the schemes part ways. The post-pass
run also reports the solver's `getRestitutionPassStats()`: how many contact visits were armed,
how often the Poisson cap cut an increment, and how often the zero clamp of the restitution
impulse engaged (with the impulse it moved).

Build with `-DPE_BUILD_IMPACT_LAB=ON` in a tree configured with
`-DPE_CONSTRAINT_SOLVER=pe::response::HardContactSemiImplicitTimesteppingSolvers`.

## Scenarios

| Scenario | Setup | What to look at |
|---|---|---|
| Flat landing | 0.4 × 0.2 × 0.1 m box lands exactly flat on four corners at 1 m/s, no spin, e = 0.8 | must come up at `e |v|` without rotating; the post-pass is a fixed number of sweeps, with two the corners swept first take more of the bounce |
| Spinning box | same box with angular velocity about y at touchdown (8 rad/s), e = 1 | leading edge impacts, trailing edge separates but its constraint still takes part; the in-solve scheme creates energy |
| Wedge | 5 cm ball touching both faces of a V (half-angle 30°) with an oblique 2 m/s velocity (50° off the axis), e = 0.9, no gravity | one face hit hard, the other only through the rebound; the in-solve scheme returns the ball faster than it came in |
| Chain | N touching balls, the first arrives at 1 m/s, e = 1, no gravity | both schemes share the impulse along the chain; neither gives the single outgoing ball |
| Pile | 12 boxes dropped in a loose column, e = 0.8, mu = 0.3, 3 s | mechanical energy over time; the one scene where the zero clamp engages |

All instantaneous scenes start at touchdown (1 µm into the contact) so that every contact
exists in the first step. The wedge ball does not fly in: approaching from a distance it
reaches the two faces one step apart, and the impacts become sequential single-contact ones on
which the schemes agree.

## Measured with the defaults (`--batch`)

Kinetic energy after / before the first step with contact impulses; `|omega'|` of body 0 after
it; the maximum of the mechanical energy over the run relative to its start.

| Scenario | in-solve: KE ratio | post-pass: KE ratio | in-solve: max E / E0 | post-pass: armed / capped / clamped |
|---|---|---|---|---|
| Flat landing (e² = 0.64) | 0.6400, no spin | 0.6397, 0.0024 rad/s | 1.000 | 7280 / 12 / 0 |
| Spinning box (e² = 1) | **1.2003** | 0.9518 | **1.134** | 24 / 6 / 0 |
| Wedge (e² = 0.81) | **1.0474** | 0.7152 | **1.047** | 4 / 3 / 0 |
| Chain (3 balls, e² = 1) | 1.0000, velocities (−1/3, 2/3, 2/3) | 0.8333 | 1.000 | 8 / 0 / 0 |
| Pile | 0.9715 (first box on the ground) | 0.9716 | 1.000 | 4229 / 1805 / 7 (0.067 N s) |

Two things this table shows beyond the artifact's charts. The zero clamp of the restitution
impulse never engages in the clean two-contact impacts; its only hits are in the pile, seven
visits in three seconds, so it is a safeguard rather than a behaviour change. And the pile's
mechanical energy did not exceed its start under the in-solve scheme in this run, so the
energy creation needs the specific geometry of the first two scenes; a pile does not produce it
by itself.

## Controls

- Scenario combo and its parameters (spin, wedge half-angle and incidence, number of balls or
  boxes), restitution, friction, dt, duration, post-pass sweeps (Box3D uses 2, pe 4), solver
  iterations, split impulse.
- `Run both schemes` (Enter) simulates both and records the trajectories; `Play`/`Pause`
  (space), `Restart` (R), time slider and playback speed scrub the recording.
- Radio buttons pick the scheme drawn solid (orange: in-solve Newton, green: capped post-pass);
  the other one is drawn transparent.
- The Results window has the summary table and the energy plot (kinetic energy for the
  gravity-free scenes, mechanical energy otherwise) with a marker at the playback time.

## Headless

```
pe_impact_lab --batch                       # all scenarios with their defaults, one line each
pe_impact_lab --smoke                       # same with checks (energy creation in-solve, none with the pass, flat landing symmetric)
pe_impact_lab --scenario 1 --sweeps 2 --screenshot box.png --at 0.2
```
