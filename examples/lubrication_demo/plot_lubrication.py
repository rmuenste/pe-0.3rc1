#!/usr/bin/env python3
"""
Quick plotter for lubrication_demo logs.

Usage:
    python plot_lubrication.py --input lines.txt --output f_est.png
`lines.txt` is created via: grep 'F_est' pe.log > lines.txt
"""
from __future__ import annotations

import argparse
import re
from pathlib import Path
from typing import List, Tuple

import matplotlib.pyplot as plt


PATTERN = re.compile(
    r"step\s+(?P<step>\d+).*?(?:gap=(?P<gap>[+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?).*?)?"
    r"F_est=(?P<force>[+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)"
)


def parse_lines(path: Path) -> Tuple[List[int], List[float], List[float]]:
    steps: List[int] = []
    forces: List[float] = []
    gaps: List[float] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            match = PATTERN.search(line)
            if not match:
                continue
            steps.append(int(match.group("step")))
            forces.append(float(match.group("force")))
            if match.group("gap") is not None:
                gaps.append(float(match.group("gap")))
    return steps, forces, gaps


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Plot F_est vs step from grep-filtered pe.log output."
    )
    parser.add_argument(
        "-i",
        "--input",
        default="lines.txt",
        help="Path to text file containing lines with F_est (default: lines.txt).",
    )
    parser.add_argument(
        "-o",
        "--output",
        help="If provided, save step-vs-force plot to this file instead of showing it.",
    )
    parser.add_argument(
        "--output-gap",
        help="Optional output path for force-vs-gap plot. "
             "If omitted but --output is given, a sibling '<output>_gap.png' is produced.",
    )
    args = parser.parse_args()

    log_path = Path(args.input)
    if not log_path.exists():
        print(f"Input file not found: {log_path}")
        return 1

    steps, forces, gaps = parse_lines(log_path)
    if not steps:
        print(f"No parsable lines found in {log_path}")
        return 1

    # Primary plot: force vs step
    plt.figure(figsize=(8, 4))
    plt.plot(steps, forces, marker="o", markersize=2, linewidth=1)
    plt.xlabel("Step")
    plt.ylabel("F_est")
    plt.title("Lubrication Demo: F_est vs Step")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()

    if args.output:
        plt.savefig(args.output, dpi=200)
        print(f"Saved plot to {args.output}")
    else:
        plt.show()

    # Secondary plot: force vs gap (only if gaps were parsed)
    if gaps:
        plt.figure(figsize=(8, 4))
        plt.plot(gaps, forces[: len(gaps)], marker="o", markersize=2, linewidth=1)
        plt.xlabel("Gap")
        plt.ylabel("F_est")
        plt.title("Lubrication Demo: F_est vs Gap")
        plt.grid(True, alpha=0.3)
        plt.tight_layout()

        gap_out = args.output_gap
        if not gap_out and args.output:
            stem = Path(args.output)
            gap_out = str(stem.with_name(f"{stem.stem}_gap{stem.suffix or '.png'}"))

        if gap_out:
            plt.savefig(gap_out, dpi=200)
            print(f"Saved force-vs-gap plot to {gap_out}")
        else:
            plt.show()
    else:
        print("No gap values found; skipping force-vs-gap plot.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
