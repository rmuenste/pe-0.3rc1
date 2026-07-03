#!/usr/bin/env python3
"""Plot the two-sphere approach data in the style of Kroupa et al. 2016, Fig. 7.

Fig. 7 of the paper shows the force divided by velocity, normalized by viscosity and
particle radius, F/(v R_p eta), as a function of the dimensionless inverse separation
distance R_p/h — a dashed no-slip line plus slip-corrected curves that plateau once the
separation is constrained at the slip length h_c = eps_c * R_p.

Generate the data through the full solver pipeline with:

    PE_LUB_CSV=1 ./build-interface-tests/tests/interface/pe_lubrication_two_sphere_approach_test > approach.csv
    ./tests/interface/plot_two_sphere_approach.py approach.csv

Writes fig7_two_sphere_approach.png next to the CSV.
"""

import csv
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load(path):
    """Return {curve name: ([R/h], [F/(v R eta)])} preserving file order."""
    curves = {}
    with open(path, newline="") as f:
        for row in csv.reader(f):
            if len(row) != 3 or row[0] == "curve" or "passed" in row[0]:
                continue
            x, y = curves.setdefault(row[0], ([], []))
            x.append(float(row[1]))
            y.append(float(row[2]))
    return curves


def main(path, ymax=None, wall=False):
    explicit_frame = ymax is not None
    curves = load(path)
    # CSV may contain both configurations: equal-sphere pair curves and "wall ..."
    # sphere-plane curves (the paper's Fig. 7 uses the wall resistance set, eq 16).
    curves = {name[5:] if wall else name: xy for name, xy in curves.items()
              if name.startswith("wall ") == wall}
    if not curves:
        sys.exit(f"no curve data found in {path}")

    fig, ax = plt.subplots(figsize=(6.0, 4.6))
    label_ys = []  # data-space y of placed labels, for collision avoidance

    # Slip-corrected / saturating curves: solid lines with symbols, like the paper
    styles = [("o", "#0072B2"), ("s", "#E69F00"), ("^", "#009E73"), ("v", "#CC79A7")]
    i = 0
    for name, (x, y) in curves.items():
        pairs = sorted(zip(x, y))
        xs = [p[0] for p in pairs]
        ys = [p[1] for p in pairs]
        if name == "no-slip":
            # dashed reference line, no symbols (paper's "no-slip" line)
            ax.plot(xs, ys, "--", color="0.35", lw=1.4, zorder=2, label="no-slip")
            noslip = (xs, ys)
        else:
            marker, color = styles[i % len(styles)]
            i += 1
            label = name.replace("eps_c=", r"$\epsilon_c$ = ")
            ax.plot(xs, ys, "-", marker=marker, ms=4, lw=1.3, color=color, zorder=3,
                    label=label)
            label_ys.append((xs[-1], ys[-1], label, color))

    ax.set_xlabel(r"inverse separation distance  $R_p\,/\,h$")
    ax.set_ylabel(r"force / velocity  $F\,/\,(v\,R_p\,\eta)$")
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)
    # Clip the y-range to the slip curves (the no-slip line diverges). Pass an explicit
    # ymax as second argument to use the paper's frame instead (Fig. 7 uses 5000, sized
    # to show the full no-slip line rather than to zoom the plateaus).
    if ymax is None:
        ymax = 1.25 * max(max(y) for name, (x, y) in curves.items() if name != "no-slip")
    ax.set_ylim(0, ymax)
    ax.grid(True, color="0.9", lw=0.6)
    ax.set_axisbelow(True)
    if explicit_frame:
        # Compressed paper frame: plateaus are too close for direct labels; use a
        # legend in the empty wedge below the no-slip line (as the paper does).
        ax.legend(frameon=False, loc="center right", fontsize=9)
    else:
        # Zoomed frame: direct labels at the plateau right ends, no legend box.
        for x, y, label, color in label_ys:
            ax.annotate(label, (x, y), xytext=(-4, 7), textcoords="offset points",
                        ha="right", fontsize=9, color=color, fontweight="bold")
        # label the no-slip line where it exits the clipped y-range
        ymax_plot = ax.get_ylim()[1]
        xs, ys = noslip
        xexit = next((x for x, y in zip(xs, ys) if y > ymax_plot), xs[-1])
        ax.annotate("no-slip", (xexit, ymax_plot), xytext=(8, -14),
                    textcoords="offset points", fontsize=9, color="0.35")
    ax.set_title("Two-sphere approach through the PE solver pipeline\n"
                 "(cf. Kroupa et al. 2016, Fig. 7)", fontsize=10)

    out = os.path.join(os.path.dirname(os.path.abspath(path)),
                       "fig7_wall.png" if wall else "fig7_two_sphere_approach.png")
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print(f"Wrote {out}")


if __name__ == "__main__":
    args = sys.argv[1:]
    wall = "wall" in args
    args = [a for a in args if a != "wall"]
    if len(args) not in (1, 2):
        print(__doc__)
        print("Options: [ymax]  wall   (wall = plot the sphere-plane curves, as in Fig. 7)")
        sys.exit(1)
    main(args[0], float(args[1]) if len(args) == 2 else None, wall)
