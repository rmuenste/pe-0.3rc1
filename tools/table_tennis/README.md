# PE table tennis demo

Interactive Polyscope demo: an ITTF-size table tennis table, the net as a box block, a 40 mm /
2.7 g ball, and a GUI that launches a serve from an adjustable position with an adjustable
velocity and spin. Built to explore the **ghost serve**: a heavy-backspin serve that bounces once
on the opponent's side and, because of the backspin, comes back over the net to the server's side.

## Build

The demo needs the semi-implicit hard-contact solver (the restitution switch lives there):

```bash
cmake -S . -B build-viewer -DPE_BUILD_TABLE_TENNIS_DEMO=ON \
      -DPE_CONSTRAINT_SOLVER=pe::response::HardContactSemiImplicitTimesteppingSolvers
cmake --build build-viewer --target pe_table_tennis -j
./build-viewer/tools/table_tennis/pe_table_tennis
```

Polyscope and ImPlot are fetched at configure time (`tools/cmake/ViewerDeps.cmake`, shared with
the other viewers); nothing is added to the core build. The body-frame meshes come from
`tools/contact_viewer/ShapeMeshes.h`.

## Scene

SI units, z up, the table's length along x, the server at x < 0.

| Part | Model | Size |
|---|---|---|
| Table top | fixed box, top surface at z = 0.76 m | 2.74 x 1.525 x 0.03 m |
| Net | fixed box centered on x = 0 | 0.01 x 1.83 x 0.1525 m (net top at 0.9125 m) |
| Legs | four fixed boxes, cosmetic | 0.05 x 0.05 x 0.73 m |
| Floor | plane at z = 0 | |
| Ball | hollow sphere (`Sphere::setShellThickness`), 2.7 g | radius 20 mm, 0.4 mm wall |

The ball is a pe sphere with a wall thickness: the collision geometry is the outer sphere, the
mass is the shell volume times the material density (chosen for 2.7 g, about 1340 kg/m^3, close
to celluloid / ABS) and the inertia is the shell value, I / (m r^2) = 0.654 for this wall
instead of the solid 0.4. Mass and wall thickness are GUI inputs (0 mm = solid ball), applied on
the next serve; the shell inertia converts spin into velocity more readily at the bounce (the
slip changes by 2.5 x the velocity change instead of 3.5 x), so the hollow ball needs less spin
for the same return.

## GUI

- **Serve**: mode radio (Launch: ball velocity + spin; Toss & serve: bat stroke; Toss, serve &
  receive: the stroke plus a held bat on the other side, see below; in Launch mode a checkbox adds
  the held bat too);
  preset combo (ghost serve, flat, fast topspin, sidespin that curves back; all launch
  legally from behind the end line and land on the opponent's side, checked by `--smoke`), ball
  position, launch velocity
  [m/s], spin [revolutions per second, omega vector], Serve / Run / Pause / Step / Reset. Keys:
  `s` serve, space run/pause, `r` reset. For a ball travelling +x, omega_y < 0 is backspin,
  omega_y > 0 topspin, omega_z sidespin.
- **Physics (live)**: aerodynamics (quadratic drag with C_d, Magnus lift with
  C_L = scale / (2 + v / (r omega)), air density, optional spin decay), ball mass and wall
  thickness (applied on the next serve), ball/table restitution and
  friction (pairwise `Material::set*`), solver (dt, steps per frame, iterations, erp, split
  impulse (on by default: the position correction adds no momentum; off is the Baumgarte
  term, an artificial rebound), restitution switch, and `capped post-pass (Box3D scheme)` (on
  by default, as in the solver): the solve stays inelastic and a separate pass adds the bounce
  with the restitution impulse capped at e times the compression impulse, so several contacts
  impacting at once cannot create energy; single contacts such as the ball on the table are
  unchanged. Off is the Newton target inside the solve. Headless: `--post-pass` /
  `--no-post-pass`).
- **Status**: time, ball state, the ghost-serve verdict (prefixed with the launch-point legality)
  and the event list (bounces by side, net crossings in either direction, net hits, floor).
- **Side view**: x-z plot of the trajectory with the table and the net.

The 3D view shows the trajectory as a curve, the bounce points in red and the serve position in
green.

## Physics

- **Flight**: drag F = -1/2 rho C_d A |v| v and the Magnus force
  F = 1/2 rho A C_L |v|^2 (omega x v) / (|omega| |v|), applied by the demo as external forces per
  step (pe has no aerodynamics). With C_d = 0.4 and the C_L form above a 40 mm ball at 5 m/s with
  50 rev/s of spin sees a lift of about 40 % of its weight.
- **Bounce**: `HardContactSemiImplicitTimesteppingSolvers` with `setRestitution( true )` (Newton
  impact law, see the solver header) and Coulomb friction. A backspin ball whose contact point
  slides throughout the impact leaves with v_x' = v_x - mu (1 + e) |v_z|, so the horizontal
  velocity reverses when mu (1 + e) |v_z| > v_x: a slow, steep arrival with enough spin to keep
  the contact sliding (relative surface speed > 2.5 x that velocity change for the thin shell,
  3.5 x for a solid sphere).
- Defaults: e = 0.9 (ITTF drop test, 0.305 m -> about 0.25 m), mu = 0.3, dt = 2e-4 s.

## Ghost serve

The **Ghost serve** preset (also the startup values) launches from a legal and realistic point:
38 cm behind the end line (x < -1.37 m) and 29 cm above the playing surface, about where a
served ball is struck after its toss. The Serve window and the verdict flag an illegal launch
point (over the table or below the surface).

| | x | y | z |
|---|---|---|---|
| position [m] | -1.75 | 0 | 1.05 |
| velocity [m/s] | 2.9 | 0 | 2.25 |
| spin [rev/s] | 0 | -50 | 0 |

50 rev/s is within what strong players produce on a backspin serve (measurements top out around
55 rev/s). The launch is a lob, 38 degrees upward: from a realistic strike height the ball has
to gain its height in flight so that it can arrive at the second bounce slowly and steeply.
Event trace (`--serve 2.9 0 2.25 0 -50 0 --pos -1.75 0 1.05`): own-side bounce at x = -0.21 m,
over the net 20 cm above the tape, a short bounce 17 cm past the net on the opponent's side, back
over the net 11 cm above the tape, then it bounces on the server's side and dribbles to the end
line. The result is identical for dt = 1e-4 ... 4e-4 s. At this launch the return works from
40 rev/s up (checked to 70); below that the ball bounces a second time on the opponent's side.

Why it works: the ball arrives at the second bounce slowly (about 1 m/s forward, the drag and the
own-side bounce took most of the launch speed) but steeply (about 3 m/s down), with most of its
backspin left. The sliding impact removes mu (1 + e) |v_z| = 0.57 x 3 = 1.7 m/s of forward speed,
more than it has, so it leaves backwards and 2.7 m/s up, enough to clear the net 17 cm away. The
remaining backspin now acts as topspin relative to the new direction and the Magnus force pulls
it down onto the server's side.

Window found by batch sweeps of legal launch points with the hollow ball and spin capped at
55 rev/s (about 14 000 serves): from a realistic strike height of 0.85 ... 1.15 m the ghost
serve needs a lobbed launch, 2.5 ... 2.9 m/s forward and 2 ... 3 m/s upward, from 1.45 ... 1.75 m
behind the net; from an unrealistic 1.3 ... 1.5 m strike height a flatter launch works too. The
preset keeps its margin against the knobs that are uncertain in the flight model: identical for
dt = 1e-4 ... 4e-4 s, still a ghost serve with the lift scaled by 0.7 ... 1.3, with e = 0.95 and
from 40 to 70 rev/s. It needs the Magnus lift (without it the lobbed ball arrives too fast and
the return clips the net top), it needs the hollow ball (a solid sphere with the same spin does
not reverse: its slip budget per unit velocity change is 3.5 instead of 2.5), and it is
sensitive to the contact pair: e = 0.85 or mu = 0.27 / 0.33 do not come back. That matches the
sport: the ghost serve is a precision serve that lives on the ball's backspin lift, its shell
inertia and a well-defined table friction.

## Toss & serve (bat)

The second mode of the Serve window replaces the launch velocity by a toss and a bat stroke.

- **Bat**: a pe union of a blade cylinder (155 mm, 6 mm), one 2 mm rubber disc per side and a
  handle. It is a dynamic body of about a tonne (blade material density 1e7 kg/m^3) whose pose
  and velocity are prescribed before every step, so it behaves as a kinematic bat: a fixed body
  cannot carry a velocity in this build, and the ball's reaction on a tonne is negligible. The
  contact solver sees the bat's velocity, so the ball takes the normal impulse (restitution,
  `rubber restitution` slider, default 0.8) and the Coulomb tangential impulse (`rubber
  friction`, default 0.8) from a moving surface, which is what turns a brushing stroke into spin.
  Missing by construction: the sponge's tangential elasticity, i.e. real rubbers give more spin
  than Coulomb sliding allows.
- **Toss**: the ball is released at rest from the palm position with an upward speed (default
  2 m/s, 20 cm rise; the rule asks for 16 cm) and falls back; no spin.
- **Stroke**: a straight segment at constant bat speed through the contact point, from
  `preTime` before contact to `postTime` after it, then the bat stops. The contact time is the
  moment the falling ball passes the chosen contact height; the bat centre is placed so that the
  rubber face is tangent to the ball there. Stroke direction: azimuth about z and elevation;
  blade orientation: tilt of the normal above the horizontal (open face > 0) and yaw; optional
  wrist rotation rate about z during the stroke (sidespin).
- **Legality**: toss at least 16 cm, ball struck on the way down above the release height, and
  the contact point behind the end line and above the playing surface.
- **Readouts**: a "struck by the bat" event, the ball velocity and spin right after the contact,
  then the usual events and verdicts.

Preset **Flat serve (toss + bat)** (batch sweep over bat speed, stroke elevation, blade tilt and
contact height, 200 strokes):

| | value |
|---|---|
| toss release | (-1.75, 0, 0.90) m, 2.0 m/s up |
| contact height | 1.02 m (8 cm below the apex, 0.33 s after release) |
| bat | 2.0 m/s, stroke 15 deg up, blade 30 deg open |
| ball after contact | (3.9, 0, 1.2) m/s, 3 rev/s topspin |
| flight | own-side bounce at x = -0.42 m, 6.5 cm over the tape, lands 0.42 m past the net |

Preset **Backspin serve (brush, toss + bat)** (sweeps over contact height, bat speed, blade tilt
and stroke elevation, 600 strokes, after two bat-geometry fixes: the contact is re-planned every
step from the ball's actual fall, and the union is positioned by its blade centre, not its centre
of mass):

| | value |
|---|---|
| toss release | (-1.75, 0, 0.85) m, 2.5 m/s up (32 cm) |
| contact height | 0.95 m, the ball falling at 2.1 m/s |
| bat | 7.0 m/s, horizontal stroke, blade 85 deg open (5 deg from tangential) |
| ball after contact | (3.1, 0, 2.4) m/s, 33 rev/s backspin |
| flight | own-side bounce at x = -0.12 m, 9 cm over the tape, lands 0.34 m past the net, then bounces back toward the net |

What sets the spin: with a tangential bat speed v_t the contact reaches sticking, and a thin
shell then takes v_t / 2.5 of tangential velocity, i.e. omega = v_t / (2.5 x 2/3 x r) = 33 rev/s
at 7 m/s. The rubber friction (0.6 ... 1.0) does not change it; bat speed does, 30 rev/s at
6.5 m/s and 35 at 7.5, but the faster ball then needs a different stroke angle to clear the net.
The real sponge adds spin beyond this Coulomb limit (tangential restitution), which this model
does not have. The serve is precise: 5 deg of stroke elevation or 0.5 m/s of bat speed either
way puts it into the net or short; dt = 1e-4 ... 4e-4 s and the contact height 0.93 ... 0.97 m
do not matter. A solid ball with the same stroke gets 39 rev/s but a different exit velocity
and does not clear the net.

Preset **Pendulum serve (right-handed swing, toss + bat)**. The stroke model has a pendulum
option: the bat hangs from a pivot (wrist / forearm) with the handle pointing at it and swings on
a circle (`swing rate`, `swing radius`, arm angle at contact, swing axis yaw / pitch) while the
arm translates (bat speed / azimuth / elevation are then the arm's motion). Found with two sweeps
over swing rate, contact phase, arm speed, blade tilt and yaw, swing-plane orientation and toss
position (3456 strokes):

| | value |
|---|---|
| toss | (-1.75, -0.5, 0.85) m, i.e. from the server's right, 2.5 m/s up, contact at 0.95 m |
| swing | 16 rad/s on a 0.30 m arm about +x (right to left, 4.8 m/s at the bat), contact at the lowest point, handle up |
| arm | 3.5 m/s forward |
| blade | 60 deg open, yawed 20 deg to the right |
| ball after contact | (3.1, 1.3, 1.9) m/s, spin (21, -10, -14) rev/s |
| flight | own-side bounce at x = -0.39 m, 8.5 cm over the tape, lands 0.30 m past the net near the centre line, then kicks to the right on every bounce |

Where the three spin components come from, and why the mix is what it is in a rigid Coulomb
contact: a lateral brush on the bottom of the ball gives roll about the direction of travel (the
component that kicks the ball sideways on the table), a lateral brush on the back of the ball
gives sidespin, and the forward arm motion under an open face gives backspin. All three are fed
by the same tangential slip, which the ball takes at 1/2.5 of its speed when the contact sticks,
and the ball also takes that fraction as sideways velocity: 4.8 m/s of lateral brush is 1.3 m/s
of sideways ball speed, which is why the serve starts from the right side and the face is yawed
right. A more upright face trades roll for sidespin but loses the backspin and the net clearance;
no stroke in the sweeps delivered more than about 10 rev/s of backspin together with sidespin.
Sensitivity: identical for dt = 1e-4 ... 4e-4 s and swing rates 14 ... 18 rad/s, tilt 55 ... 65;
0.5 m/s of arm speed, 5 deg of yaw or 10 deg of contact phase either way puts it into the net or
too short.

Preset **Reverse pendulum serve (right-handed swing, toss + bat)**: the same forehand swing
with the wrist snapping the other way, left to right (swing rate -16 rad/s), served from the left
side (toss y = +0.5) with the blade yawed 20 deg to the left. The bat in the sim is symmetric, so
this is the exact mirror of the pendulum: every event reproduces with y -> -y, spin
(-21, -10, 14) rev/s, 8.5 cm over the tape, lands 0.30 m past the net just left of the centre
line and kicks to the server's left on every bounce. In reality the two serves share the same
preparation and differ only in the direction of the wrist snap, which is what makes the pair hard
to read.

The earlier solution, kept as **Forward brush, side-backspin (straight stroke)**, is a straight
7.5 m/s stroke 30 deg to the server's right with a 60 deg open, 60 deg yawed blade: spin
(-21, -24, 18) rev/s with more backspin, but it brushes from below to the side the way a
left-hander's pendulum would, and the handle hangs down.

Headless: `--toss [--toss-pos x y z] [--toss-speed u] [--contact-z z] [--bat speed az el tilt yaw]
[--wrist rate] [--rubber e mu]` prints the events, a `BAT` line with the hit and the ball state
after it, and the `RESULT` line. `--smoke` checks that every bat preset is legal, hits the ball
and lands on the opponent's side over the net.

## Toss, serve & receive (held bat)

The third mode adds a receiver who does not swing: a second bat, the same union as the server's
but a fixed body, held still where the serve arrives. The point is to see what each serve's spin
does to the ball coming off a passive blade, and what the receiver has to do with the blade angle
to cope with it.

- **Placement**: automatic by default. Before every serve the serve is run once with the bat
  parked (a dry run, a few tenths of a second of wall time), and every sample between the first
  bounce on the receiver's side and the next bounce is a candidate intercept. The face is put
  against the ball's horizontal direction at that sample, tilted and yawed by the GUI angles,
  with the handle to the receiver's right; the bat is raised if the blade would dip into the
  table. A candidate is valid if the bat so placed is not hit from behind or at the rim by the
  ball earlier in the serve and does not straddle the net plane. The highest valid candidate is
  the apex; `meet the ball at` puts the bat at a fraction of the time from the bounce to it (1 =
  apex, 0.5 = halfway up). With automatic placement off, the ball centre at the block is typed
  in and the ball is assumed to arrive along +x.
- **Angles**: `blade angle` is the face normal above the horizontal (0 vertical, > 0 open, face
  up; < 0 closed), `blade yaw` turns it about z. Both apply at once, also mid-flight, without a
  new dry run, so the same serve can be replayed against different blade angles.
- **Readouts**: a "struck by the receiving bat" event, the ball's velocity and spin right after
  it, how high it rose, and a return verdict: landed on the server's side (with the net
  clearance), into the net, over the net but off the table, bounced back on the receiver's own
  side, or struck before the bounce (see below). In batch mode a `RECEIVE` line carries the same.

What the presets do against a vertical blade (angle 0), all met at the apex of the rise:

| serve | where it is met | ball off the blade | why |
|---|---|---|---|
| Flat (bat or launch) | 0.78 m past the net, 17 cm up | (-1.5, 0, +0.8) m/s | the table bounce left it rolling with topspin, which climbs a vertical face |
| Topspin fast | 1.59 m, behind the end line | (-3.4, 0, +1.8) m/s | heavy topspin pops up, the return floats long |
| Sidespin | 0.71 m, 16 cm up | (-0.8, -1.4, +0.3) m/s | sidespin kicks the ball sideways off the face |
| Pendulum / reverse pendulum | 0.49 m, 18 cm up | (-0.4, +/-1.1, +0.5) m/s | sideways kick, mirrored between the two |
| Banana lobs | 82 cm up, deep | | the lob is met at its apex |

A passive block never adds energy, so with the rubber restitution 0.8 these slow serves come off
at 1 ... 3 m/s and drop on the receiver's own side; the fast topspin serve carries back almost to
the net. Closing the blade (angle -20) against the topspin serve sends the ball down onto the
table instead of up; opening it (+20) lifts it further. The sign of the vertical component and
the sideways kick are the spin reading a receiver has to make.

**Serves that cannot be met by a held bat.** The backspin bat serve (33 rev/s) and the ghost
launch preset rebound almost vertically from the receiver's side (horizontal speed 0.1 ... 0.7
m/s after the bounce), so the ball comes back up through the column it came down in, and the
return over the server's side of the ghost serve runs 10 cm under the serve's own incoming
path. Every candidate position puts the blade into the serve before it bounces; the placement
then takes the apex anyway, says so in its note, and the verdict reads "the bat stood in the
serve's path: struck before the ball bounced". That is the honest statement for these serves: a
receiver has to move the bat in after the ball has passed, which is what makes them effective.

Headless: `--receive` adds the held bat to either mode (`--mode-receive` selects the mode for a
screenshot), `--receiver tilt yaw riseFraction` sets the angles and the intercept, and
`--receiver-pos x y z` places it by hand. `--smoke` runs every bat preset and every launch preset
against the held bat and checks that the bat is placed and touched.

## Headless modes

```bash
# one serve, events + RESULT line (ghost=0/1, legal=0/1), no window (spin in rev/s)
./build-viewer/tools/table_tennis/pe_table_tennis --serve 2.9 0 2.25  0 -50 0 --pos -1.75 0 1.05
# options: --cor e --mu m --shell t_m (0 = solid) --mass kg --dt s --time T --lift k --cd c
#          --no-drag --no-magnus --quiet
# the same serve against a held bat, blade 20 deg closed (prints a RECEIVE line)
./build-viewer/tools/table_tennis/pe_table_tennis --serve 6 0 -1  0 40 0 --pos -1.75 0 1.05 --receiver -20 0 1
# self check on Polyscope's mock backend (runs the ghost preset through the GUI loop)
./build-viewer/tools/table_tennis/pe_table_tennis --smoke
```
