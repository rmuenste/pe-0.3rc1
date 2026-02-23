import argparse
import numpy as np
import matplotlib.pyplot as plt


def ramp_up(x, start, end):
    if end <= start:
        return 1.0 if x >= start else 0.0
    if x <= start:
        return 0.0
    if x >= end:
        return 1.0
    return (x - start) / (end - start)


def ramp_down(x, start, end):
    if end <= start:
        return 1.0 if x <= start else 0.0
    if x <= start:
        return 1.0
    if x >= end:
        return 0.0
    return (end - x) / (end - start)


def clamp01(val):
    return max(0.0, min(1.0, val))


def hard_weight(dist, threshold, blend_half_width):
    if blend_half_width <= 0.0:
        return 1.0 if dist < threshold else 0.0
    lower = threshold - blend_half_width
    upper = threshold + blend_half_width
    return clamp01(ramp_down(dist, lower, upper))


def lub_weight(dist, threshold, lub_threshold, contact_blend, lub_blend):
    entry_start = threshold - contact_blend
    entry_end = threshold + contact_blend
    exit_center = threshold + lub_threshold
    exit_start = exit_center - lub_blend
    exit_end = exit_center + lub_blend

    weight_up = ramp_up(dist, entry_start, entry_end) if contact_blend > 0 else (1.0 if dist > threshold else 0.0)
    weight_down = ramp_down(dist, exit_start, exit_end) if lub_blend > 0 else (1.0 if dist < exit_center else 0.0)
    return clamp01(weight_up * weight_down)


def parse_args():
    p = argparse.ArgumentParser(description="Plot lubrication/hard-contact blends and impulse cap ramp.")
    p.add_argument("--contact-threshold", type=float, default=1e-8, help="contactThreshold [m]")
    p.add_argument("--lubrication-threshold", type=float, default=5.25e-3, help="lubricationThreshold [m]")
    p.add_argument("--contact-blend", type=float, default=1e-9, help="contactHysteresisDelta (half-width) [m]")
    p.add_argument("--lubrication-blend", type=float, default=1e-9, help="lubricationHysteresisDelta (half-width) [m]")
    p.add_argument("--alpha-impulse-cap", type=float, default=1.0, help="alphaImpulseCap_ (max factor)")
    p.add_argument("--mu", type=float, default=212.0, help="dynamic viscosity μ [Pa·s]")
    p.add_argument("--dt", type=float, default=1e-3, help="time step [s]")
    p.add_argument("--vrn", type=float, default=-0.0462489, help="pre-lubrication normal velocity vrn [m/s]")
    p.add_argument("--r-eff", type=float, default=0.0075, help="effective radius R_eff [m]")
    p.add_argument("--mass1", type=float, default=0.0019792, help="mass of body1 [kg]")
    p.add_argument("--mass2", type=float, default=np.inf, help="mass of body2 [kg]; use inf for plane")
    p.add_argument("--min-eps-lub", type=float, default=1e-8, help="lubrication gap regularization [m]")
    p.add_argument("--gap-min", type=float, default=None, help="min gap for plot [m]")
    p.add_argument("--gap-max", type=float, default=None, help="max gap for plot [m]")
    p.add_argument("--points", type=int, default=800, help="number of samples across gap")
    return p.parse_args()


def main():
    args = parse_args()

    contact_threshold = args.contact_threshold
    lubrication_threshold = args.lubrication_threshold
    contact_blend = args.contact_blend
    lubrication_blend = args.lubrication_blend
    alpha_impulse_cap = args.alpha_impulse_cap
    mu = args.mu
    dt = args.dt
    vrn = args.vrn
    R_eff = args.r_eff
    min_eps_lub = args.min_eps_lub

    invm1 = 0.0 if np.isinf(args.mass1) else 1.0 / args.mass1
    invm2 = 0.0 if np.isinf(args.mass2) else 1.0 / args.mass2

    gap_min = args.gap_min if args.gap_min is not None else contact_threshold * 0.1
    gap_max = args.gap_max if args.gap_max is not None else (contact_threshold + lubrication_threshold) * 1.05
    gaps = np.linspace(gap_min, gap_max, args.points)

    hard = np.array([hard_weight(g, contact_threshold, contact_blend) for g in gaps])
    lub = np.array([lub_weight(g, contact_threshold, lubrication_threshold, contact_blend, lubrication_blend) for g in gaps])

    blend = lub  # same value used in solver for lubrication contacts
    cap_factor = alpha_impulse_cap * blend

    invm_sum = invm1 + invm2
    m_eff = 1.0 / invm_sum if invm_sum > 0 else np.inf

    gap_clamped = np.maximum(gaps, min_eps_lub)
    F_uncapped = 6.0 * np.pi * mu * (R_eff ** 2) * (-vrn) / gap_clamped
    J = F_uncapped * dt
    Jcap = cap_factor * m_eff * (-vrn)
    cap_ratio = np.where(J > 0, np.minimum(1.0, Jcap / J), 1.0)
    # Solver applies: Fmag_weighted = Fmag_capped * blend  (line 2310 in HCL solver)
    # The blend factor appears twice: once inside Jcap, once after capping.
    F_applied = F_uncapped * cap_ratio * blend

    fig, axes = plt.subplots(2, 1, figsize=(8, 8), sharex=True)

    ax = axes[0]
    ax.plot(gaps, hard, label="hard weight")
    ax.plot(gaps, lub, label="lubrication weight")
    ax.set_ylabel("blend weight")
    ax.legend()
    ax.grid(True, which="both", ls="--", alpha=0.4)
    ax.set_title("Blend weights vs gap")

    ax = axes[1]
    ax.plot(gaps, cap_factor, label="cap factor (alpha * blend)")
    ax.plot(gaps, cap_ratio, label="impulse cap ratio (Jcap/J)")
    ax.plot(gaps, F_applied, label="applied force [N] (capped * blend)")
    ax.set_xlabel("gap distance [m]")
    ax.set_ylabel("cap / force")
    ax.legend()
    ax.grid(True, which="both", ls="--", alpha=0.4)
    ax.set_title("Impulse cap ramp and resulting force")

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
