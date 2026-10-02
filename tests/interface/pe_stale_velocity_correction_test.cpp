//=================================================================================================
/*!
 *  \file tests/interface/pe_stale_velocity_correction_test.cpp
 *  \brief A fixed body whose per-step storage index is recycled from a departed mobile body
 *         must not inherit that body's velocity correction (dv_/dw_).
 *
 *  The hard-contact collision systems keep the per-body velocity corrections dv_/dw_ in
 *  std::vector members that are resize()d at the start of every step and never cleared, and
 *  initializeVelocityCorrections() assigns them only for awake, NON-fixed bodies. Body indices
 *  (body->index_) are re-derived from the storage order every step, and BodyStorage::remove()
 *  is swap-with-last, so whenever the storage reorders between two steps (a body destroyed, a
 *  particle migrated, a shadow copy added) a fixed body can land on an index whose dv_/dw_
 *  entry still holds what the previous mobile occupant left there. The contact relaxation reads
 *  v_ + dv_ for BOTH partners of a contact, so every contact against that wall is then sized as
 *  if the wall moved at the ghost velocity: the mobile partner receives an impulse no body
 *  absorbs (a momentum-bookkeeping violation) while the wall itself is integrated with v_ only
 *  (fixed GLOBAL branch) and never moves -- or, for a non-global fixed body in a serial world,
 *  drifts by dv_*dt as well.
 *
 *  Scenario (the exact recycling the swap-with-last removal produces):
 *    1. mobile spheres A and B (plus a free-flight control C) are created and stepped once with
 *       a force on each, so their dv_ entries are nonzero; A's force has a tangential (x) and a
 *       normal (z) component so a stale copy of it is visible in both contact directions;
 *    2. a FIXED plane W is created AFTER them, and A is destroyed: W is swapped into A's slot
 *       (storage position 0), so on the next step W->index_ == A's old index and W's dv_ entry
 *       is A's stale correction;
 *    3. B is placed exactly resting on W (gap 0, zero velocity) under its "gravity" force and
 *       stepped once. Asserted:
 *         a. W->index_ is A's old index (the precondition that makes the test meaningful);
 *         b. B's post-step velocity is that of a body resting on a wall AT REST: zero normal
 *            velocity and no tangential kick -- on the unfixed code B ends the step moving at
 *            A's stale dv (the static-friction contact drives the velocity RELATIVE to the ghost
 *            wall to zero);
 *         c. momentum bookkeeping: sum over mobile bodies of m*dv equals the external force
 *            impulse plus the wall's normal reaction (which for a resting body exactly cancels
 *            the normal external impulse) -- with no tangential component, which could only
 *            come from a moving wall;
 *         d. W has not moved (displacement, normal, velocity unchanged).
 *
 *  Body forces are applied explicitly (world gravity = 0) so the same source drives all three
 *  solvers identically: HardContactAndFluid and HardContactSemiImplicitTimesteppingSolvers add
 *  world gravity to v_ themselves, HardContactEulerLagrange leaves body forces to the driver.
 *  The ghost mechanism is the same either way -- the solver only ever sees v_ + dv_.
 *
 *  Serial world, no MPI. Registered once per solver via the library variants
 *  (pe_static_lubstage_fluid / _plain / shipped pe_static); see tests/interface/CMakeLists.txt.
 */
//=================================================================================================

#include <pe/core.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace pe;

namespace {

int failures = 0;

void expect( bool ok, const char* what )
{
   if( !ok ) {
      std::printf( "FAIL: %s\n", what );
      ++failures;
   }
}

// Resting-contact residual: the serial relaxation is plain Gauss-Seidel, so a single contact is
// solved exactly in one sweep up to roundoff. The ghost signal on the unfixed code is 2e-3 / 5e-3
// in velocity (see below), five to six orders of magnitude above this.
const real tol = real(1e-9);

bool near( real a, real b, real t = tol )
{
   return std::abs( a - b ) <= t;
}

void print3( const char* label, const Vec3& v )
{
   std::printf( "  %-34s = (% .12e, % .12e, % .12e)\n", label, (double)v[0], (double)v[1], (double)v[2] );
}

}  // namespace

int main()
{
   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, 0.0 );   // body forces are applied explicitly, see file comment
   world->setLiquidDensity( 0.0 );
   world->setDamping( 1.0 );

   // Silence the HardContactAndFluid per-step representative-rank stdout block (cfdRank_ == 1 is
   // the default and matches MPISettings::rank() == 0 in a serial run). Diagnostic only.
   SimulationConfig::getInstance().setCfdRank( 0 );

   // (name, density, cor, csf, cdf, poisson, young, stiffness, dampingN, dampingT)
   // Static friction 0.5 keeps the resting contact static both for a wall at rest and under the
   // ghost tangential velocity, so the comparison below is static-vs-static (no sliding branch).
   MaterialID mat = createMaterial( "stale_dv", 1000.0, 0.0, 0.5, 0.5, 0.3, 1e6, 1e3, 1e2, 1e2 );

   const real R  = real(0.01);
   const real dt = real(1e-3);
   const real g  = real(10.0);                      // "gravity" magnitude for B and C
   const Vec3 aGhost( real(2.0), real(0.0), real(-5.0) );   // A's acceleration -> stale dv = aGhost*dt

   unsigned int id = 0;

   // Phase 1: three mobile spheres in free flight, far apart, nothing else in the world.
   SphereID A = createSphere( id++, 0.0, 0.0, 10.0 * R, R, mat );
   SphereID B = createSphere( id++, 1.0, 0.0, 10.0 * R, R, mat );
   SphereID C = createSphere( id++, 2.0, 0.0, 10.0 * R, R, mat );

   const real mA = A->getMass();
   const real mB = B->getMass();
   const real mC = C->getMass();
   expect( mA > real(0) && mB == mA && mC == mA, "phase 1: equal positive sphere masses" );

   A->addForce( mA * aGhost );
   B->addForce( Vec3( 0.0, 0.0, -mB * g ) );
   C->addForce( Vec3( 0.0, 0.0, -mC * g ) );
   world->simulationStep( dt );

   const Vec3 vA1 = A->getLinearVel();
   std::printf( "phase 1 (free flight):\n" );
   print3( "A velocity after step", vA1 );
   print3( "B velocity after step", B->getLinearVel() );
   expect( A->index_ == 0 && B->index_ == 1 && C->index_ == 2, "phase 1: storage-order indices A=0, B=1, C=2" );
   expect( near( vA1[0], aGhost[0] * dt ) && near( vA1[1], real(0) ) && near( vA1[2], aGhost[2] * dt ),
           "phase 1: A's velocity correction is aGhost*dt (nonzero in x and z)" );
   const size_t indexA = A->index_;

   // Phase 2: a FIXED plane W created after the mobile bodies, then A destroyed. BodyStorage::remove
   // is swap-with-last, so W moves into A's slot.
   PlaneID W = createPlane( id++, 0.0, 0.0, 1.0, 0.0, mat );   // z = 0, normal +z
   expect( W->isFixed(), "phase 2: plane is fixed" );
   const Vec3 nW0 = W->getNormal();
   const real dW0 = W->getDisplacement();

   destroy( A );
   expect( world->size() == 3, "phase 2: three bodies remain (W, B, C)" );
   expect( *( world->begin() ) == BodyID( W ), "phase 2: W occupies storage position 0 (A's former slot)" );

   // Phase 3: B resting exactly on W (gap 0) with zero velocity under its gravity force; C stays in
   // free flight as the momentum control.
   B->setPosition( 1.0, 0.0, R );
   B->setLinearVel( 0.0, 0.0, 0.0 );
   B->setAngularVel( 0.0, 0.0, 0.0 );
   const Vec3 vB0 = B->getLinearVel();
   const Vec3 vC0 = C->getLinearVel();
   const Vec3 FB( 0.0, 0.0, -mB * g );
   const Vec3 FC( 0.0, 0.0, -mC * g );
   B->addForce( FB );
   C->addForce( FC );
   world->simulationStep( dt );

   const Vec3 vB = B->getLinearVel();
   const Vec3 wB = B->getAngularVel();
   const Vec3 vC = C->getLinearVel();
   std::printf( "phase 3 (B resting on W, W's index recycled from A):\n" );
   std::printf( "  W->index_ = %zu (A's old index %zu)\n", W->index_, indexA );
   print3( "B velocity after step", vB );
   print3( "B angular velocity after step", wB );
   print3( "C velocity after step", vC );
   print3( "W velocity after step", W->getLinearVel() );
   std::printf( "  W displacement after step        = % .12e (was % .12e)\n", (double)W->getDisplacement(), (double)dW0 );

   // (a) precondition: the recycling actually happened.
   expect( W->index_ == indexA, "phase 3: W's index is A's old index (dv_ slot recycled)" );

   // (b) B saw a wall AT REST: zero normal velocity, no tangential kick, no spin.
   expect( near( vB[2], real(0) ), "phase 3: B's normal velocity is zero (resting on a wall at rest)" );
   expect( near( vB[0], real(0) ) && near( vB[1], real(0) ), "phase 3: B received no tangential kick" );
   expect( near( wB[0], real(0) ) && near( wB[1], real(0) ) && near( wB[2], real(0) ), "phase 3: B received no spin" );

   // (c) momentum bookkeeping over the mobile bodies: sum m*dv = external impulse + wall reaction.
   //     The wall reaction of a wall at rest acting on a resting body is purely normal and exactly
   //     cancels the normal external impulse; anything tangential would have to come from a moving wall.
   const Vec3 dpB = mB * ( vB - vB0 );
   const Vec3 dpC = mC * ( vC - vC0 );
   const Vec3 Jext = ( FB + FC ) * dt;
   const Vec3 Jwall = ( dpB + dpC ) - Jext;      // what the wall must have supplied
   print3( "sum m*dv (B+C)", dpB + dpC );
   print3( "external impulse", Jext );
   print3( "implied wall impulse", Jwall );
   const real ptol = tol * mB;
   expect( near( Jwall[0], real(0), ptol ) && near( Jwall[1], real(0), ptol ),
           "phase 3: wall impulse has no tangential component" );
   expect( near( Jwall[2], mB * g * dt, ptol ),
           "phase 3: wall normal reaction exactly cancels B's normal external impulse" );
   expect( near( dpC[0], FC[0] * dt, ptol ) && near( dpC[1], FC[1] * dt, ptol ) && near( dpC[2], FC[2] * dt, ptol ),
           "phase 3: free-flight control C received exactly its external impulse" );

   // (d) the fixed wall did not move.
   expect( near( W->getDisplacement(), dW0 ) && W->getNormal() == nW0, "phase 3: W did not move" );
   expect( W->getLinearVel() == Vec3() && W->getAngularVel() == Vec3(), "phase 3: W has zero velocity" );

   if( failures == 0 )
      std::printf( "pe_stale_velocity_correction_test: all checks passed\n" );
   else
      std::printf( "pe_stale_velocity_correction_test: %d check(s) FAILED\n", failures );
   return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
