// Unit test for the Euler-Lagrange semi-implicit drag velocity update.
//
// The arithmetic lives in pe/core/collisionsystem/ELSemiImplicitDrag.h and is
// called from CollisionSystem<...HardContactEulerLagrange>::
// initializeVelocityCorrections. Exercising it through a real PE step would
// require standing up MPI, a domain, and the full collision pipeline; instead we
// test the pure helper directly, which is exactly the function the integrator
// uses, so the discrete scheme is covered without that machinery.
//
// Covered (from the Phase 2 plan, "PE interface coverage"):
//   * the discrete semi-implicit relation is solved exactly (defining residual)
//   * closed-form analytic match over a range of dt
//   * stiff stability (tau_p << dt): no overshoot, monotone contraction
//   * pure-forcing limit (B = 0)
//   * convergence to the exact exponential solution as dt -> 0

#include <pe/core/collisionsystem/ELSemiImplicitDrag.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace pe;

namespace {

int failures = 0;

void check(bool ok, const char* what) {
  if (!ok) {
    std::cerr << "FAILED: " << what << "\n";
    ++failures;
  }
}

bool close(real a, real b, real tol) { return std::fabs(a - b) <= tol; }

bool vclose(const Vec3& a, const Vec3& b, real tol) {
  return close(a[0], b[0], tol) && close(a[1], b[1], tol) && close(a[2], b[2], tol);
}

}  // namespace

int main() {
  const real tol = real(1e-12);

  // ---- 1. Defining implicit relation: m (unew-uold)/dt = B(u_f - unew) + F ----
  {
    const Vec3 uold(0.3, -0.1, 0.05);
    const Vec3 uf(1.0, 0.2, -0.4);
    const Vec3 fother(0.7, -0.3, 0.9);
    const real mass = 2.5;
    const real B = 1.3;
    for (real dt = real(1e-4); dt <= real(2.0); dt *= real(10.0)) {
      const Vec3 unew = elSemiImplicitVelocity(uold, mass, B, uf, fother, dt);
      const Vec3 lhs = (mass / dt) * (unew - uold);
      const Vec3 rhs = B * (uf - unew) + fother;
      check(vclose(lhs, rhs, real(1e-10)), "implicit relation residual ~ 0");

      // closed-form of that same linear solve, computed independently
      const Vec3 expected = (uold + (dt / mass) * (B * uf + fother)) /
                            (real(1) + dt * B / mass);
      check(vclose(unew, expected, tol), "analytic closed form");
    }
  }

  // ---- 2. Pure forcing limit, B = 0  ->  unew = uold + dt/m F ----
  {
    const Vec3 uold(0.2, 0.0, -0.5);
    const Vec3 fother(1.0, -2.0, 3.0);
    const real mass = 0.8;
    const real dt = 0.05;
    const Vec3 unew = elSemiImplicitVelocity(uold, mass, real(0), Vec3(9, 9, 9),
                                             fother, dt);
    check(vclose(unew, uold + (dt / mass) * fother, tol), "B=0 pure forcing");
  }

  // ---- 3. Stiff stability: no overshoot, monotone contraction toward u_f ----
  {
    const Vec3 uold(0.0, 0.0, 0.0);
    const Vec3 uf(1.0, -1.0, 0.5);
    const Vec3 fzero(0, 0, 0);
    const real mass = 1.0;
    const real dt = 0.1;
    for (real B = real(1.0); B <= real(1e8); B *= real(100.0)) {
      const Vec3 unew = elSemiImplicitVelocity(uold, mass, B, uf, fzero, dt);
      // each component stays in the closed interval [uold, uf] (no overshoot)
      for (int k = 0; k < 3; ++k) {
        const real lo = std::min(uold[k], uf[k]);
        const real hi = std::max(uold[k], uf[k]);
        check(unew[k] >= lo - tol && unew[k] <= hi + tol, "stiff: no overshoot");
      }
      // strict contraction toward equilibrium u_f
      check((unew - uf).sqrLength() < (uold - uf).sqrLength() + tol,
            "stiff: monotone contraction");
    }
    // B -> very large collapses to u_f within a small tolerance
    const Vec3 unewStiff =
        elSemiImplicitVelocity(uold, mass, real(1e10), uf, fzero, dt);
    check(vclose(unewStiff, uf, real(1e-6)), "stiff: B->inf gives u_f");
  }

  // ---- 4. Convergence to exact exponential over many small steps (F = 0) ----
  // ODE: m du/dt = B (u_f - u)  =>  u(T) = u_f + (u0 - u_f) exp(-(B/m) T)
  {
    const Vec3 u0(0.0, 0.0, 0.0);
    const Vec3 uf(1.0, 0.0, 0.0);
    const Vec3 fzero(0, 0, 0);
    const real mass = 1.0;
    const real B = 2.0;
    const real T = 1.0;
    const Vec3 exact = uf + std::exp(-(B / mass) * T) * (u0 - uf);
    real prevErr = 1e30;
    for (int n = 10; n <= 100000; n *= 10) {
      const real dt = T / real(n);
      Vec3 u = u0;
      for (int i = 0; i < n; ++i) {
        u = elSemiImplicitVelocity(u, mass, B, uf, fzero, dt);
      }
      const real err = (u - exact).length();
      check(err < prevErr, "exponential: error decreases as dt shrinks");
      prevErr = err;
    }
    check(prevErr < real(1e-4), "exponential: converged at finest dt");
  }

  if (failures == 0) {
    std::cout << "pe-el-semi-implicit-drag: all checks passed\n";
    return EXIT_SUCCESS;
  }
  std::cerr << "pe-el-semi-implicit-drag: " << failures << " check(s) failed\n";
  return EXIT_FAILURE;
}
