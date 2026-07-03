#!/usr/bin/env python3
"""Evaluate shear_cell_kroupa output against hard-sphere viscosity correlations.

Collects one or more shear_cell_eta.csv files (one per volume fraction phi), averages
eta_L over the second half of each run (steady state), and plots eta(phi) against
Krieger-Dougherty and Maron-Pierce. The measured curve is the LUBRICATION contribution
only (paper's eta_L, eq 32); the hydrodynamic baseline eta_H is not resolved by PE-only
runs, so compare shapes/trends rather than absolute values below phi ~ 0.2.

Usage: evaluate_viscosity.py run_phi0.1.csv run_phi0.2.csv ...
"""

import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ETA_F = 1e-3   # fluid viscosity used in shear_cell_kroupa.cpp
PHI_M = 0.64   # maximum packing fraction (random close packing)


def krieger_dougherty(phi):
    return ETA_F * (1.0 - phi / PHI_M) ** (-2.5 * PHI_M)


def maron_pierce(phi):
    return ETA_F * (1.0 - phi / PHI_M) ** (-2.0)


def main(paths):
    phis, etas = [], []
    for path in paths:
        data = np.genfromtxt(path, delimiter=",", names=True)
        half = len(data) // 2
        phis.append(float(np.mean(data["phi"][half:])))
        etas.append(float(np.mean(data["eta_L"][half:])))
        print(f"{path}: phi = {phis[-1]:.3f}  eta_L = {etas[-1]:.4g} Pa s")

    phi_ref = np.linspace(0.01, 0.55, 200)
    fig, ax = plt.subplots(figsize=(6, 4.5))
    ax.plot(phi_ref, krieger_dougherty(phi_ref), "-", label="Krieger-Dougherty")
    ax.plot(phi_ref, maron_pierce(phi_ref), "--", label="Maron-Pierce")
    if phis:
        ax.plot(phis, np.array(etas) + ETA_F, "o", label="PE: eta_f + eta_L")
    ax.set_xlabel("particle volume fraction phi")
    ax.set_ylabel("suspension viscosity eta [Pa s]")
    ax.set_yscale("log")
    ax.legend()
    ax.set_title("Sheared suspension viscosity vs. hard-sphere correlations")
    fig.tight_layout()
    fig.savefig("eta_vs_phi.png", dpi=150)
    print("Wrote eta_vs_phi.png")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    main(sys.argv[1:])
