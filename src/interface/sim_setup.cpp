#include <map>
#include <iomanip>
#include <cstdint>
#include <pe/vtk.h>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <pe/util/Checkpointer.h>
#include <pe/core/MPISystem.h>
#include <pe/core/MPISystemID.h>
#include <pe/core/domaindecomp/DomainDecomposition.h>
#include <pe/core/domaindecomp/HalfSpace.h>
#include <pe/core/domaindecomp/Intersection.h>
#include <pe/core/domaindecomp/Merging.h>
#include <pe/core/domaindecomp/Process.h>
#include <pe/core/domaindecomp/RectilinearGrid.h>
#include <pe/engine.h>
#include <pe/support.h>

#if HAVE_JSON
#include <nlohmann/json.hpp>
#endif

//using namespace fc2::povray;
using boost::filesystem::path;

using namespace pe;
#if HAVE_MPI
#include <pe/interface/sim_setup.h>

#ifdef HAVE_CGAL
//=================================================================================================
//
//    INTERFACE CGAL DELAUNAY
//
//=================================================================================================
#include <pe/interface/compute_delaunay.h>
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

// The Checkpointer
Checkpointer checkpointer = Checkpointer(
    pe::SimulationConfig::getInstance().getCheckpointPath(),
    pe::SimulationConfig::getInstance().getPointerspacing(),
    0,
    pe::SimulationConfig::getInstance().getTimesteps());

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
  
  
  //=================================================================================================
  // Trigger a new checkpointer write if activated
  if (config.getUseCheckpointer()) {
    checkpointer.trigger();
    checkpointer.flush();
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


#include <pe/interface/setup_part_bench.h>
#include <pe/interface/setup_nxnxn.h>
#include <pe/interface/setup_cyl.h>
#include <pe/interface/setup_dkt.h>
#include <pe/interface/setup_bench.h>
#include <pe/interface/setup_fsi_bench.h>
#include <pe/interface/setup_kroupa.h>
#include <pe/interface/setup_creep.h>
#include <pe/interface/setup_archimedes.h>
#include <pe/interface/setup_archimedes_z.h>
#include <pe/interface/setup_archimedes_xy.h>
#include <pe/interface/setup_archimedes_empty.h>
#include <pe/interface/setup_span.h>
#include <pe/interface/setup_drill.h>
//
//=================================================================================================
//

#endif

//=================================================================================================
//
//    SERIAL PE MODE IMPLEMENTATIONS
//
//=================================================================================================
#ifdef PE_SERIAL_MODE

#include <pe/interface/sim_setup_serial.h>

//=================================================================================================
// C Wrapper for stepSimulationSerial - used when PE_SERIAL_MODE is defined
//=================================================================================================
extern "C" void step_simulation_() {
  pe::stepSimulationSerial();
}
//=================================================================================================

#endif  // PE_SERIAL_MODE
