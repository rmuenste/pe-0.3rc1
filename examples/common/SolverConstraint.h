//=================================================================================================
/*!
 *  \file examples/common/SolverConstraint.h
 *  \brief Runtime refusal for examples that are only valid under specific constraint solvers
 *
 *  Many examples are written and tuned for a particular pe_CONSTRAINT_SOLVER and produce
 *  meaningless results under any other. They used to express that with the compile-time
 *  pe_CONSTRAINT_MUST_BE_EITHER_TYPE constraint, which made a wrongly-configured build fail
 *  to compile. That is a sound instinct -- a user must not be able to run an example under a
 *  pipeline it was never intended for -- but it also means the entire examples tree fails to
 *  build under the shipped default solver, so nothing ever compile-checks it (there is no CI)
 *  and unrelated rot goes unnoticed for years.
 *
 *  This header keeps the guarantee and drops the collateral damage: the example COMPILES under
 *  any solver, and refuses to RUN under an unintended one. It exits before touching the world,
 *  so an unintended configuration still cannot produce a single simulated step -- it produces a
 *  message pointing at the constraint in the example's own source.
 *
 *  Usage (see examples/mpiperiodic for a worked case):
 *
 *     typedef Configuration<...,response::HardContactAndFluid>::Config TargetConfig3;
 *
 *     if( !pe::examples::requireIntendedSolver<TargetConfig2,TargetConfig3>(
 *            "mpiperiodic", intendedSolvers, rationale ) ) {
 *        return EXIT_FAILURE;
 *     }
 */
//=================================================================================================

#pragma once

#include <cstdlib>
#include <iostream>
#include <type_traits>

#include <pe/core/Configuration.h>
#include <pe/core/Types.h>

#if HAVE_MPI
#  include <mpi.h>
#endif

namespace pe {
namespace examples {

//*************************************************************************************************
/*!\brief Whether the compile-time selected pe::Config is one of the listed target configurations.
 *
 * This is the constexpr equivalent of pe_CONSTRAINT_MUST_BE_EITHER_TYPE: same predicate, but it
 * yields a value instead of failing the build, so the caller decides what to do with it.
 */
template< typename... TargetConfigs >
constexpr bool configIsOneOf()
{
   return ( std::is_same< pe::Config, TargetConfigs >::value || ... );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Prints the refusal banner for an unintended pe_CONSTRAINT_SOLVER.
 *
 * \param example         Name of the example, used to point the reader at the right source file.
 * \param intendedSolvers Newline-separated list of the solvers this example accepts.
 * \param rationale       Why the example is restricted to them.
 */
inline void reportUnintendedSolver( const char* example,
                                    const char* intendedSolvers,
                                    const char* rationale )
{
   std::cerr
      << "\n================================================================================\n"
      << " UNINTENDED CONFIGURATION DETECTED -- refusing to run\n"
      << "================================================================================\n"
      << " example : " << example << "\n\n"
      << " This example is only valid when pe_CONSTRAINT_SOLVER is one of:\n"
      << intendedSolvers << "\n"
      << " Why: " << rationale << "\n\n"
      << " The active solver is not one of them, so the results would be meaningless.\n"
      << " Nothing has been simulated. Read the solver constraint near the top of the\n"
      << " example's source file, then reconfigure, e.g.:\n\n"
      << "   cmake -S . -B build-examples -DPE_BUILD_EXAMPLES=ON \\\n"
      << "     -DCMAKE_CXX_FLAGS=\"-Dpe_CONSTRAINT_SOLVER=pe::response::HardContactAndFluid\"\n"
      << "================================================================================\n"
      << std::endl;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Refuses to run when the active solver is not one of the listed target configurations.
 *
 * Returns \a true when the example may proceed, \a false when the caller must bail out with a
 * non-zero exit code. The predicate is a compile-time constant, so under MPI every rank reaches
 * the same verdict and the banner is printed by rank 0 alone. Safe to call before or after
 * MPI_Init: without MPI it degrades to a plain print.
 *
 * Call it as early as possible -- the point of the guard is that an unintended configuration
 * never reaches the simulation.
 */
template< typename... TargetConfigs >
inline bool requireIntendedSolver( const char* example,
                                   const char* intendedSolvers,
                                   const char* rationale )
{
   if constexpr ( configIsOneOf< TargetConfigs... >() ) {
      return true;
   }
   else {
#if HAVE_MPI
      int initialized( 0 );
      MPI_Initialized( &initialized );
      if( initialized ) {
         int rank( 0 );
         MPI_Comm_rank( MPI_COMM_WORLD, &rank );
         if( rank != 0 ) return false;
      }
#endif
      reportUnintendedSolver( example, intendedSolvers, rationale );
      return false;
   }
}
//*************************************************************************************************

} // namespace examples
} // namespace pe
