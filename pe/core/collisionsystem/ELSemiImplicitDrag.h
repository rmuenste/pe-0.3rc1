//=================================================================================================
/*!
 *  \file pe/core/collisionsystem/ELSemiImplicitDrag.h
 *  \brief Semi-implicit Euler-Lagrange drag velocity update.
 *
 *  This isolates the one-line arithmetic of the Euler-Lagrange semi-implicit
 *  particle velocity update so it can be unit-tested without standing up a full
 *  PE world (MPI, domain, body storage, collision pipeline). The collision
 *  system specialization for response::HardContactEulerLagrange calls this from
 *  initializeVelocityCorrections.
 */
//=================================================================================================

#ifndef _PE_CORE_COLLISIONSYSTEM_ELSEMIIMPLICITDRAG_H_
#define _PE_CORE_COLLISIONSYSTEM_ELSEMIIMPLICITDRAG_H_

//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/math/Vector3.h>


namespace pe {

//*************************************************************************************************
/*!\brief One-step semi-implicit update of a particle velocity under linearized drag.
 *
 * \param uOld       Particle velocity at the start of the step.
 * \param mass       Particle mass (> 0).
 * \param dragB      Linearized drag coefficient B = |F_drag| / |u_rel| (>= 0), units mass/time.
 * \param carrierVel Carrier (fluid) velocity u_f sampled at the particle.
 * \param otherForce Sum of the non-drag forces (pressure + lift + gravity/buoyancy).
 * \param dt         Time step (> 0).
 * \return The updated particle velocity u_{n+1}.
 *
 * Solves the momentum balance with the drag term treated implicitly in the new
 * velocity (the rest explicit):
 * \f[
 *    m\,\frac{u_{n+1}-u_n}{dt} = B\,(u_f - u_{n+1}) + F_\text{other}
 * \f]
 * which rearranges to
 * \f[
 *    u_{n+1} = \frac{u_n + \frac{dt}{m}\,(B\,u_f + F_\text{other})}
 *                   {1 + \frac{dt\,B}{m}} .
 * \f]
 * The denominator is always > 1 for B >= 0, dt > 0, so the update is
 * unconditionally stable and cannot overshoot the equilibrium velocity even for
 * stiff drag (tau_p = m/B << dt).
 */
inline Vec3 elSemiImplicitVelocity( const Vec3& uOld, real mass, real dragB,
                                    const Vec3& carrierVel, const Vec3& otherForce,
                                    real dt )
{
   const Vec3 numerator = uOld + ( dt / mass ) * ( dragB * carrierVel + otherForce );
   return numerator / ( real(1) + dt * dragB / mass );
}
//*************************************************************************************************

} // namespace pe

#endif
