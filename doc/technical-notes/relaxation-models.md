# Relaxation Models and Material Parameters Analysis

## Analysis of Hard Contact Relaxation Models

Based on the analysis of `pe/core/collisionsystem/HardContactAndFluid.h`:

### 1. Inelasticity of Models
**Yes, the models are completely inelastic.**
The relaxation models available in the `HardContactAndFluid` solver (and similar hard contact solvers) are designed to be completely inelastic.
- **Mechanism:** The solver explicitly computes impulses to drive the relative velocity at the contact point to **zero** (`v_rel = 0`).
- **Code Evidence:** In the relaxation loop (e.g., `relaxApproximateInelasticCoulombContactsByDecoupling`), the impulse is calculated as:
  ```cpp
  p_cf[0] = -( diag_n_inv_[i] * gdot_nto );
  ```
  This targets a final velocity of 0. A solver supporting restitution would target `-e * v_rel_initial`.

### 2. Origin of "Bounciness"
**Bounciness is purely a side effect of Baumgarte Stabilization.**
Since the solver goal is $v_{rel} = 0$, any observed post-collision separation velocity is due to the **Baumgarte Stabilization** term attempting to resolve existing penetration.
- **Mechanism:** The solver adds a bias term to the relative velocity equation:
  ```cpp
  gdot_nto[0] += dist_[i] * dtinv;
  ```
- **Effect:** When bodies overlap (`dist < 0`), this term acts as a "fake" approach velocity. The solver generates a stronger repulsive impulse to cancel this "velocity," which physically results in the bodies being pushed apart, manifesting visually as a bounce.

### 3. Unbounded Nature of Baumgarte Stabilization
**Yes, the stabilization is effectively unbounded in the relaxation loop.**
- **Bias Term:** `dist_[i] / dt`
- **Issue:** There is no clamping of this velocity *inside* the relaxation loop. If penetration (`dist`) is large or the timestep (`dt`) is very small, this term becomes huge, generating massive impulses that can cause bodies to gain excessive energy ("explode").
- **Mitigation:** The `resolveContacts` function applies an Error Reduction Parameter (`erp_`) to `dist` before the loop (`dist *= erp_`), but this only scales the problem (e.g., by 0.7) rather than capping the maximum correction velocity.

### 4. Coefficient of Restitution in `createMaterial`
The `pe::Material` class is a **generic container** designed to support multiple different physics solvers, not just the hard contact ones.

- **Unused Parameters:** For `HardContactAndFluid` and `HardContactSemiImplicitTimesteppingSolvers`, the **Coefficient of Restitution (`cor`)**, **Stiffness**, and **Damping** parameters in `createMaterial` are effectively **dead code**. The solvers simply ignore them.
- **Purpose:** These parameters are intended for:
    - **DEM Solvers:** Discrete Element Method solvers which use penalty-based forces (springs and dampers).
    - **Future/Alternative Solvers:** Solvers that explicitly implement Newton's law of restitution ($v_{new} = -e \cdot v_{old}$).

#### Material Parameter Usage for Hard Contact Solvers
Example: `pe::createMaterial("default", 1.0, 0.3, 0.5, 0.05, 0.2, 80, 100, 10, 11)`

| Value | Parameter | Used by Hard Contact Solver? |
| :--- | :--- | :--- |
| `1.0` | `density` | **Yes** (Mass calculation) |
| `0.3` | `cor` (Restitution) | **NO** (Ignored) |
| `0.5` | `csf` (Static Friction) | **Yes** |
| `0.05` | `cdf` (Dynamic Friction) | **Yes** |
| `0.2` | `poisson` | **NO** (Ignored) |
| `80` | `young` | **NO** (Ignored) |
| `100` | `stiffness` | **NO** (Ignored) |
| `10` | `dampingN` | **NO** (Ignored) |
| `11` | `dampingT` | **NO** (Ignored) |

---

## Solver Classification

The following solvers were scanned to determine if they use the inelastic relaxation models defined in the `RelaxationModel` enum (e.g., `InelasticFrictionlessContact`, `ApproximateInelasticCoulombContactByDecoupling`).

### Solvers using Inelastic Relaxation Models
These solvers share the same hard contact logic and suffer from the "bounciness via Baumgarte" behavior described above.

1.  **`pe/core/collisionsystem/HardContactAndFluid.h`**
    - Contains `RelaxationModel` enum.
    - Implements specific relaxation functions like `relaxInelasticFrictionlessContacts`.
2.  **`pe/core/collisionsystem/HardContactSemiImplicitTimesteppingSolvers.h`**
    - Contains `RelaxationModel` enum.
    - Implements the same relaxation functions as `HardContactAndFluid`.

### Solvers NOT using these Inelastic Relaxation Models
These solvers use different physics formulations (e.g., penalty methods or impulse-based FFD) and likely utilize the other material parameters (stiffness, damping, restitution).

1.  **`pe/core/collisionsystem/DEMSolver.h`** (Discrete Element Method)
    - Does **not** use the `RelaxationModel` enum.
    - Uses a penalty-based approach (`solver_.resolveContact(*c)`), which typically utilizes stiffness and damping parameters.
2.  **`pe/core/collisionsystem/FFDSolver.h`** (Fast Frictional Dynamics)
    - Does **not** use the `RelaxationModel` enum.
    - Uses a specific FFD resolution steps (`solver_.resolveContacts(*body)`).
