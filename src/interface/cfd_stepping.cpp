//=================================================================================================
/*!\file src/interface/cfd_stepping.cpp
 *  \brief Time stepping of a coupled PE simulation and the globals the parallel setups
 *         publish into. Parallel mode (HAVE_MPI): stepSimulation(), stepELFrozenTrace() and
 *         the interface globals (world, mpisystem, ...). Serial mode: thin wrappers over
 *         pe/interface/support/serial_stepping.h. The extern "C" step_* entry points of
 *         both modes are declared in pe/interface/cfd_entry_points.h.
 */
//=================================================================================================

#include <map>
#include <iomanip>
#include <sstream>
#include <cstdint>
#include <pe/vtk.h>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <pe/util/Checkpointer.h>
#include <pe/interface/support/el_optional_api.h>
#include <pe/core/MPISystem.h>
#include <pe/core/MPISystemID.h>
#include <pe/core/domaindecomp/DomainDecomposition.h>
#include <pe/core/domaindecomp/HalfSpace.h>
#include <pe/core/domaindecomp/Intersection.h>
#include <pe/core/domaindecomp/Merging.h>
#include <pe/core/domaindecomp/Process.h>
#include <pe/core/domaindecomp/RectilinearGrid.h>
#include <pe/engine.h>
#include <pe/interface/support/el_hydro_compat.h>
#include <pe/support.h>

#if HAVE_JSON
#include <nlohmann/json.hpp>
#endif

//using namespace fc2::povray;
using boost::filesystem::path;

using namespace pe;
#if HAVE_MPI
#include <pe/interface/cfd_entry_points.h>
#include <pe/interface/setup/parallel_common.h>

#ifdef HAVE_CGAL
//=================================================================================================
//
//    INTERFACE CGAL DELAUNAY
//
//=================================================================================================
#include <pe/interface/support/compute_delaunay.h>
void outputDelaunay(int timeStep);
//=================================================================================================
//
//=================================================================================================
extern "C" void output_delaunay_(int *istep) {
  int step = *istep;
  outputDelaunay(step);
}
//=================================================================================================
#endif // endif HAVE_CGAL


//=================================================================================================
//
//    Global Definitions
//
//=================================================================================================

// SimulationConfig implementation has been moved to src/config/SimulationConfig.cpp
namespace pe {

} // namespace pe

// Global variables
bool g_vtk = pe::SimulationConfig::getInstance().getVtk();
PlaneID g_ground(0);

// Configuration of the simulation world
WorldID world;
int called = 0;

// Configuration of the MPI system
MPISystemID mpisystem;

namespace {
inline void ensureCheckpointerActivatedFromRuntimeConfig() {
  auto& config = pe::SimulationConfig::getInstance();
  if (config.getUseCheckpointer() && !pe::isCheckpointerActive()) {
    pe::activateCheckpointer(
        config.getCheckpointPath(),
        config.getPointerspacing(),
        0,
        config.getTimesteps());
  }
}
}  // namespace

real degreesToRadians(real deg) {
  return deg * M_PI / 180.0;
}

//=================================================================================================
//
//    INTERFACE SIMULATION CONTROL FUNCTIONS
//
//=================================================================================================

//=================================================================================================
// C Wrapper for stepSimulation
//=================================================================================================
extern "C" void step_simulation_() {
  stepSimulation();
}
//=================================================================================================

//=================================================================================================
// C wrapper: re-file every body into the cell its CURRENT AABB belongs to.
//
// stepSimulation() files bodies in findContacts() and then integrates positions, so afterwards
// the cell assignment is one step behind the geometry. A point query visits 27 cells and a body
// is filed by its AABB MINIMUM, so a body whose minimum sits just below a cell boundary can have
// its far surface two cells from where it is filed - an interior point there comes back as fluid.
// Callers that classify points after motion (FeatFloWer's FBM alpha field) must call this first.
// It performs no contact detection and no physics; it is the same incremental update
// findContacts() runs, which touches a cell only when a body's hash actually changes, and it
// leaves pending bodies in bodiesToAdd_ where getBodiesNearPoint() already looks for them.
//=================================================================================================
extern "C" void refresh_spatial_index_() {
  pe::theCollisionSystem()->getCoarseDetector().refreshSpatialIndex();
}
//=================================================================================================


//=================================================================================================
extern "C" void step_el_frozen_trace_() {
  stepELFrozenTrace();
}
//=================================================================================================


//=================================================================================================
void singleOutput_v1(BodyID body, int timestep) {
      auto& config = SimulationConfig::getInstance();
      real stepsize = config.getStepsize();
      std::cout << "==Single Particle Data========================================================" << std::endl;
      std::cout << "Position: " << body->getSystemID() << " " << timestep * stepsize << " " <<
                                   body->getPosition()[0] << " " <<
                                   body->getPosition()[1] << " " <<
                                   body->getPosition()[2] << std::endl;
      std::cout << "Velocity: " << body->getSystemID() << " " << timestep * stepsize << " " << 
                                   body->getLinearVel()[0] << " " <<
                                   body->getLinearVel()[1] << " " <<
                                   body->getLinearVel()[2] << std::endl;
      std::cout << "Omega:    " << body->getSystemID() << " " << timestep * stepsize << " " << 
                                   body->getAngularVel()[0] << " " <<
                                   body->getAngularVel()[1] << " " <<
                                   body->getAngularVel()[2] << std::endl;
}
//=================================================================================================



//=================================================================================================
void singleOutput_v2(BodyID body, int timestep) {
      auto& config = SimulationConfig::getInstance();
      real stepsize = config.getStepsize();
      std::cout << "==Single Particle Data========================================================" << std::endl;
      std::cout << "Position: " << body->getSystemID() << " " << body->getPosition()[2]  << " " << timestep * stepsize << std::endl;
      std::cout << "Velocity: " << body->getSystemID() << " " << body->getLinearVel()[2]  << " " << timestep * stepsize << std::endl;
      std::cout << "Angular: " << body->getSystemID() << " "<< body->getAngularVel()  << " " << timestep * stepsize << std::endl;
}
//=================================================================================================


//=================================================================================================
void groupOutput_v1(real totalV, real dt, real h, real totalA) {
      std::cout << "==Particle Group Data=========================================================" << std::endl;
      std::cout << "Maximum Vp : " << totalV << std::endl;
      std::cout << "Maximum CFL: " << (totalV * dt) / h << std::endl;
      std::cout << "Maximum Ap : " << totalA << std::endl;
}
//=================================================================================================


//=================================================================================================
//=================================================================================================
// This functions steps the physics simulation one time step
//=================================================================================================
void stepSimulation() {
  auto& config = SimulationConfig::getInstance();

  // Activate checkpointer lazily from runtime-loaded configuration.
  ensureCheckpointerActivatedFromRuntimeConfig();
  
  static int timestep = 0;
  unsigned long particlesTotalBefore = 0;
  int numBodies =  theCollisionSystem()->getBodyStorage().size();
  unsigned long bodiesUpdate = static_cast<unsigned long>(numBodies);
  unsigned long particlesTotal ( 0 );
  MPI_Comm cartcomm = theMPISystem()->getComm();

  /*
   * The first argument to MPI_Reduce is the communicated value
   * The 2nd argument to MPI_Reduce is cummulative value
   */
  MPI_Reduce( &bodiesUpdate, &particlesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm );
  particlesTotalBefore = particlesTotal;

  real h = 0.125;

  //=================================================================================================
  int subSteps = 1;
  real subStepSize = config.getStepsize() / static_cast<real>(subSteps);
  TimeStep::stepsize(subStepSize);
  for (int istep(0); istep < subSteps; ++istep) {
    world->simulationStep( subStepSize );
  }
  TimeStep::stepsize(config.getStepsize());
  //=================================================================================================

  //world->simulationStepDebug( config.getStepsize() );
#define OUTPUT_LEVEL3
#ifdef OUTPUT_LEVEL3
  unsigned int i(0);
  real maxV(0.0);
  real maxA(0.0);
  real totalV(0.0);
  real totalA(0.0);
  Vec3 vv(0,0,0);
  Vec3 maxPos(0,0,0);
  BodyID maxParticle;
  for (; i < theCollisionSystem()->getBodyStorage().size(); i++) {
    World::SizeType widx = static_cast<World::SizeType>(i);
    BodyID body = world->getBody(static_cast<unsigned int>(widx));
    if(body->getType() == sphereType || 
       body->getType() == capsuleType || 
       body->getType() == ellipsoidType || 
       body->getType() == cylinderType || 
       body->getType() == triangleMeshType) {

      Vec3 vel = body->getLinearVel();
      Vec3 ang = body->getAngularVel();
      real v = vel.length();
      real a = ang.length();
      if( maxV <= v) {
        maxV = v;
        vv = body->getLinearVel();
        maxPos = body->getPosition();
        maxParticle = body;
      } 
      if( maxA <= a) 
        maxA = a;
      
#define SINGLE_PARTICLE_OUTPUT
#ifdef SINGLE_PARTICLE_OUTPUT
      singleOutput_v2(body, timestep);
#endif
    }
  }

  MPI_Reduce( &maxV, &totalV, 1, MPI_DOUBLE, MPI_MAX, 0, cartcomm );
  MPI_Reduce( &maxA, &totalA, 1, MPI_DOUBLE, MPI_MAX, 0, cartcomm );
  pe_EXCLUSIVE_SECTION(0) {
     groupOutput_v1(totalV, subStepSize, h, totalA);
  }
#endif 

  //=================================================================================================
  pe_EXCLUSIVE_SECTION(0) {
      std::cout << "==DEM Time Data===============================================================" << std::endl;
      std::cout << "DEM timestep: " << timestep << "|| sim time: " << timestep * config.getStepsize() << " || substepping:  " << subSteps << std::endl;
  }
  
  
  timestep++;

  // MPI Finalization
  MPI_Barrier(cartcomm);

}


namespace pe {

//*************************************************************************************************
/*!\brief Wrapper function for backwards compatibility.
 *
 * \param fileName Path to the JSON configuration file.
 * \return void
 *
 * This function is a backwards-compatible wrapper that calls SimulationConfig::loadFromFile().
 * New code should call SimulationConfig::loadFromFile() directly.
 */
void loadSimulationConfig(const std::string &fileName) {
    SimulationConfig::loadFromFile(fileName);
}

} // namespace pe


// The parallel case setups (pe/interface/setup/<case>.h) are included by the entry-point
// unit src/interface/cfd_entry_points.cpp; this unit owns the globals they publish into.

namespace {

bool elFrozenTraceIsFiniteVec3(const Vec3 &value) {
  return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}

int countELFrozenTraceLocalParticles() {
  int particleCount = 0;
  WorldID world = theWorld();
  for (auto it = theCollisionSystem()->getBodyStorage().begin();
       it != theCollisionSystem()->getBodyStorage().end(); ++it) {
    BodyID body = *it;
    if (body->getType() == sphereType) {
      ++particleCount;
    }
  }
  return particleCount;
}

void applyELFrozenTraceFluidForces(const real fullStepSize, const real alpha) {
  for (auto it = theCollisionSystem()->getBodyStorage().begin();
       it != theCollisionSystem()->getBodyStorage().end(); ++it) {
    BodyID body = *it;
    if (body->isFixed()) {
      continue;
    }
    // Bodies carrying an Euler-Lagrange hydro state (q2p1_el_pipeflow) are
    // advanced by the unconditionally stable semi-implicit drag update inside
    // initializeVelocityCorrections; applying the legacy under-relaxed fluid
    // force here as well would double-count their forcing. The relaxed kick is
    // therefore restricted to non-hydro bodies (q2p1_el_frozen_trace), which
    // still rely on it to damp the explicit-coupling time-Nyquist mode.
    if (elHasHydroState(body)) {
      continue;
    }
    body->applyFluidForces(fullStepSize, alpha);
  }
}

// Clear the Euler-Lagrange hydro state on every local body. Called once at the
// end of a PE step so that the semi-implicit drag state lives for exactly one
// PE step (all of its substeps) and never persists into a step in which the
// CFD driver did not re-arm the body. Without this, a body that stops being
// updated (ELApplyParticleForces=No after being on, ownership migration, a
// particle leaving the active region) would keep being integrated with stale
// carrier velocity / drag, producing silent, physically wrong motion.
void clearELHydroStates() {
  for (auto it = theCollisionSystem()->getBodyStorage().begin();
       it != theCollisionSystem()->getBodyStorage().end(); ++it) {
    BodyID body = *it;
    if (!trySetELHydroState(body, real(0), Vec3(0, 0, 0), Vec3(0, 0, 0), false)) {
      throw std::logic_error(
        "stepELFrozenTrace requires "
        "pe_CONSTRAINT_SOLVER == pe::response::HardContactEulerLagrange");
    }
  }
}

int cullELFrozenTraceLocalParticles(const Vec3 &domainMin, const Vec3 &domainMax) {
  int removed = 0;
  WorldID world = theWorld();
  const real epsX = std::max(real(1e-12), std::abs(domainMax[0]) * real(1e-12));
  const real epsY = std::max(real(1e-12), std::abs(domainMax[1]) * real(1e-12));
  const real epsZ = std::max(real(1e-12), std::abs(domainMax[2]) * real(1e-12));

  for (auto it = world->begin(); it != world->end();) {
    BodyID body = *it;
    if (body->getType() != sphereType) {
      ++it;
      continue;
    }

    const Vec3 pos = body->getPosition();
    const Vec3 vel = body->getLinearVel();
    const Vec3 ang = body->getAngularVel();

    const bool invalidState = !elFrozenTraceIsFiniteVec3(pos) ||
                              !elFrozenTraceIsFiniteVec3(vel) ||
                              !elFrozenTraceIsFiniteVec3(ang);
    const bool outsideClosedFaces =
        (pos[0] < domainMin[0] - epsX) ||
        (pos[1] < domainMin[1] - epsY) || (pos[1] > domainMax[1] + epsY) ||
        (pos[2] < domainMin[2] - epsZ) || (pos[2] > domainMax[2] + epsZ);
    const bool outsideOpenOutflow = (pos[0] > domainMax[0] + epsX);

    if (invalidState || outsideClosedFaces || outsideOpenOutflow) {
      it = world->destroy(it);
      ++removed;
    } else {
      ++it;
    }
  }

  return removed;
}

}  // namespace

void stepELFrozenTrace() {
  auto &config = SimulationConfig::getInstance();
  WorldID world = theWorld();
  MPI_Comm cartcomm = theMPISystem()->getComm();

  static int timestep = 0;

  const int substeps = std::max(1, config.getSubsteps());
  const real fullStepSize = config.getStepsize();
  const real substepSize = fullStepSize / static_cast<real>(substeps);

  const int localBefore = countELFrozenTraceLocalParticles();
  int globalBefore = 0;
  MPI_Reduce(&localBefore, &globalBefore, 1, MPI_INT, MPI_SUM, 0, cartcomm);

  // Under-relaxed fluid-force kick for frozen-trace (non-hydro) bodies only.
  // Hydro bodies (el_pipeflow) are skipped here and advanced semi-implicitly.
  const real alpha = real(0.35);
  applyELFrozenTraceFluidForces(fullStepSize, alpha);

  // Lubrication impulse-virial accumulates over the substeps of this macro
  // step; reset here so the CFD-side query reads exactly one step's worth.
  elResetLubricationVirial(*theCollisionSystem());

  TimeStep::stepsize(substepSize);
  ElRestitutionTally restitutionTally;
  bool restitutionOn = false;
  for (int istep = 0; istep < substeps; ++istep) {
    world->simulationStep(substepSize);
    restitutionOn = elAccumulateRestitutionStats(*theCollisionSystem(), restitutionTally);
  }
  TimeStep::stepsize(fullStepSize);

  // Restitution statistics of this CFD step (only while the switch is on, so an inelastic run
  // prints nothing new). Counts summed over the sub-steps and the processes (candidates: approach
  // faster than the threshold; armed: candidates with compression impulse; delivered: armed rows
  // ending with a booked restitution impulse; separating: rows armed at the classification that
  // end separating; demoted: armed rows whose compression impulse was withdrawn (support rows
  // afterwards); in_solve: in-solve Newton targets installed; impulse_gross: separating increments
  // booked as restitution, impulse_final: booked at the end of the passes, impulse_fallback:
  // support impulse of demoted rows, all unrelaxed); sub-step stop reasons, sweeps and
  // stop-test violations at the last test (the same on every process; sweeps_mean_active is the
  // mean over the sub-steps whose pass ran, skipped sub-steps excluded); velocity measures maxima
  // over sub-steps and processes (support_scale_max: largest local impact scale of a support row).
  // pe_step is this counter, cfd_step = pe_step + 1 the step number the CFD audits print.
  if (restitutionOn) {
    typedef ElRestitutionTally T;
    MPI_Allreduce(MPI_IN_PLACE, restitutionTally.sum, T::numSums, MPI_DOUBLE, MPI_SUM, cartcomm);
    MPI_Allreduce(MPI_IN_PLACE, restitutionTally.same, T::numSame, MPI_DOUBLE, MPI_MAX, cartcomm);
    MPI_Allreduce(MPI_IN_PLACE, restitutionTally.max, T::numMax, MPI_DOUBLE, MPI_MAX, cartcomm);
    pe_EXCLUSIVE_SECTION(0) {
      const double passes = restitutionTally.same[T::converged] + restitutionTally.same[T::capped];
      std::ostringstream line;   // own stream: the precision of std::cout stays untouched
      line << std::setprecision(6) << "EL_RESTITUTION_STATS pe_step= " << timestep << " cfd_step= " << timestep + 1
           << " scheme= " << (restitutionTally.postPass ? "post-pass" : "in-solve")
           << " candidates= " << restitutionTally.sum[T::candidates]
           << " armed= " << restitutionTally.sum[T::armed]
           << " delivered= " << restitutionTally.sum[T::delivered]
           << " separating= " << restitutionTally.sum[T::separating]
           << " demoted= " << restitutionTally.sum[T::demoted]
           << " in_solve= " << restitutionTally.sum[T::inSolve]
           << " support= " << restitutionTally.sum[T::support]
           << " armed_visits= " << restitutionTally.sum[T::armedVisits]
           << " impulse_gross= " << restitutionTally.sum[T::impulseGross]
           << " impulse_final= " << restitutionTally.sum[T::impulseFinal]
           << " impulse_fallback= " << restitutionTally.sum[T::impulseFallback]
           << " substeps_converged= " << restitutionTally.same[T::converged]
           << " substeps_capped= " << restitutionTally.same[T::capped]
           << " substeps_skipped= " << restitutionTally.same[T::skipped]
           << " sweeps_mean_active= " << (passes > 0 ? restitutionTally.same[T::sweeps] / passes : 0.0)
           << " sweeps_max= " << restitutionTally.max[T::sweepsMax]
           << " residual= " << restitutionTally.max[T::residual]
           << " residual_rel= " << restitutionTally.max[T::residualRel]
           << " tangential_residual= " << restitutionTally.max[T::tangential]
           << " support_closing_before= " << restitutionTally.max[T::closingBefore]
           << " support_closing_after= " << restitutionTally.max[T::closingAfter]
           << " impact_error= " << restitutionTally.max[T::impactError]
           << " newton_shortfall= " << restitutionTally.max[T::shortfall]
           << " approach_max= " << restitutionTally.max[T::approach]
           << " resting_approach_max= " << restitutionTally.max[T::restingApproach]
           << " violations_last_test= " << restitutionTally.same[T::violations]
           << " support_scale_max= " << restitutionTally.max[T::supportScale];
      std::cout << line.str() << std::endl;
    }
  }

  // The semi-implicit hydro state is valid for exactly this PE step (all
  // substeps above consumed the same frozen carrier state). Clear it now so it
  // cannot leak into a later step that did not re-arm the body.
  clearELHydroStates();

  const int localRemoved = cullELFrozenTraceLocalParticles(config.getCfdDomainMin(),
                                                          config.getCfdDomainMax());
  world->synchronize();

  const int localAfter = countELFrozenTraceLocalParticles();
  int globalAfter = 0;
  int globalRemoved = 0;
  MPI_Reduce(&localAfter, &globalAfter, 1, MPI_INT, MPI_SUM, 0, cartcomm);
  MPI_Reduce(&localRemoved, &globalRemoved, 1, MPI_INT, MPI_SUM, 0, cartcomm);

  pe_EXCLUSIVE_SECTION(0) {
    std::cout << "Frozen-field PE parallel step: timestep=" << timestep
              << " | owned particles before/after = " << globalBefore
              << " / " << globalAfter
              << " | outflow removed = " << globalRemoved
              << " | substeps = " << substeps
              << std::endl;
  }

  ++timestep;
  MPI_Barrier(cartcomm);
}
//
//=================================================================================================
//

//=================================================================================================
//
//    SERIAL PE MODE IMPLEMENTATIONS
//
//=================================================================================================
#else  // !HAVE_MPI: serial PE mode (PE_SERIAL_MODE)

#include <pe/interface/support/serial_stepping.h>

//=================================================================================================
// C Wrapper for stepSimulationSerial - used when PE_SERIAL_MODE is defined
//=================================================================================================
extern "C" void step_simulation_() {
  pe::stepSimulationSerial();
}
//=================================================================================================

//=================================================================================================
// C wrapper: re-file every body into the cell its CURRENT AABB belongs to.
//
// stepSimulation() files bodies in findContacts() and then integrates positions, so afterwards
// the cell assignment is one step behind the geometry. A point query visits 27 cells and a body
// is filed by its AABB MINIMUM, so a body whose minimum sits just below a cell boundary can have
// its far surface two cells from where it is filed - an interior point there comes back as fluid.
// Callers that classify points after motion (FeatFloWer's FBM alpha field) must call this first.
// It performs no contact detection and no physics; it is the same incremental update
// findContacts() runs, which touches a cell only when a body's hash actually changes, and it
// leaves pending bodies in bodiesToAdd_ where getBodiesNearPoint() already looks for them.
//=================================================================================================
extern "C" void refresh_spatial_index_() {
  pe::theCollisionSystem()->getCoarseDetector().refreshSpatialIndex();
}
//=================================================================================================


//=================================================================================================
extern "C" void step_el_frozen_trace_() {
  pe::stepELFrozenTraceSerial();
}
//=================================================================================================

#endif  // HAVE_MPI
