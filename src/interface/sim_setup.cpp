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
#endif


//=================================================================================================
//
//    Global Definitions
//
//=================================================================================================

// Implementation of SimulationConfig
namespace pe {

SimulationConfig::SimulationConfig()
    : timesteps_(15000)
    , stepsize_(0.001)
    , processesX_(3)
    , processesY_(3)
    , processesZ_(3)
    , seed_(12345)
    , verbose_(false)
    , vtk_(true)
    , visspacing_(50)
    , pointerspacing_(100)
    , useCheckpointer_(true)
    , checkpoint_path_("checkpoints/")
    , volumeFraction_(0.3)
    , benchRadius_(0.0015)
    , resume_(false)
{
}

SimulationConfig& SimulationConfig::getInstance() {
    static SimulationConfig instance;
    return instance;
}

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

  real h = 0.00104167;

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
      
//#define SINGLE_PARTICLE_OUTPUT
#ifdef SINGLE_PARTICLE_OUTPUT
      singleOutput_v1(body, timestep);
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
//  if (config.getUseCheckpointer()) {
//    checkpointer.trigger();
//    checkpointer.flush();
//  }

  timestep++;

  // MPI Finalization
  MPI_Barrier(cartcomm);

}


namespace pe {

void loadSimulationConfig(const std::string &fileName) {

#if HAVE_JSON
    // Open the configuration file
    std::ifstream file(fileName);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open config file: " + fileName);
    }

    // Parse the JSON file using nlohmann::json
    nlohmann::json j;
    file >> j;

    // Retrieve the singleton instance of SimulationConfig
    SimulationConfig &config = SimulationConfig::getInstance();

    // Set time parameters
    if (j.contains("timesteps_"))
        config.setTimesteps(j["timesteps_"].get<size_t>());
    if (j.contains("stepsize_"))
        config.setStepsize(j["stepsize_"].get<real>());

    // Set process parameters
    if (j.contains("processesX_"))
        config.setProcessesX(j["processesX_"].get<int>());
    if (j.contains("processesY_"))
        config.setProcessesY(j["processesY_"].get<int>());
    if (j.contains("processesZ_"))
        config.setProcessesZ(j["processesZ_"].get<int>());

    // Set random number generator parameter
    if (j.contains("seed_"))
        config.setSeed(j["seed_"].get<size_t>());

    // Set verbose mode
    if (j.contains("verbose_"))
        config.setVerbose(j["verbose_"].get<bool>());

    // Set VTK visualization flag
    if (j.contains("vtk_"))
        config.setVtk(j["vtk_"].get<bool>());

    // Set visualization parameters
    if (j.contains("visspacing_"))
        config.setVisspacing(j["visspacing_"].get<unsigned int>());
    if (j.contains("pointerspacing_"))
        config.setPointerspacing(j["pointerspacing_"].get<unsigned int>());

    // Set checkpointer usage
    if (j.contains("useCheckpointer_"))
        config.setUseCheckpointer(j["useCheckpointer_"].get<bool>());

    // Set the checkpoint path (assuming the JSON key is a string)
    if (j.contains("checkpoint_path_"))
        config.setCheckpointPath(boost::filesystem::path(j["checkpoint_path_"].get<std::string>()));

    // Set simulation parameters
    if (j.contains("volumeFraction_"))
        config.setVolumeFraction(j["volumeFraction_"].get<real>());
    
    if (j.contains("benchRadius_"))
        config.setBenchRadius(j["benchRadius_"].get<real>());


#endif
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
#include <pe/interface/setup_span.h>
//
//=================================================================================================
//

#endif
