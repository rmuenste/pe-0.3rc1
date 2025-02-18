#include <map>
#include <iomanip>
#include <cstdint>
#include <pe/engine.h>
#include <pe/support.h>
#include <pe/vtk.h>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <pe/util/Checkpointer.h>

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
/////////////////////////////////////////////////////
// Simulation parameters

// Particle parameters
const bool   spheres ( true   );  // Switch between spheres and granular particles
const real   radius  ( 0.2    );  // The radius of spheres of the granular media
const real   spacing (real(0.5)*radius + 0.0025 );  // Initial spacing in-between two spheres
const real   velocity( 0.0025 );  // Initial maximum velocity of the spheres

// Time parameters
const size_t initsteps     (  20000 );  // Initialization steps with closed outlet door
const size_t focussteps    (    100 );  // Number of initial close-up time steps
const size_t animationsteps(    200 );  // Number of time steps for the camera animation
const size_t timesteps     ( 15000 );  // Number of time steps for the flowing granular media
const real   stepsize      ( 0.001 );  // Size of a single time step

// Process parameters
const int    processesX( 3 );    // Number of processes in x-direction
const int    processesY( 3 );    // Number of processes in y-direction
const int    processesZ( 3 );    // Number of processes in y-direction
const real   adaption  ( 1.5 );  // Dynamic adaption factor for the sizes of the subdomains

// Random number generator parameters
const size_t seed( 12345 );

// Verbose mode
const bool verbose( false );  // Switches the output of the simulation on and off

// Fixed simulation parameters
const size_t N    ( 30 );        // Number of boxes forming the hourglass
const real   H    ( 9.0 );       // Height of the hourglass
const real   L    ( 1.5 );       // Size of the center opening
const real   W    ( 0.5 );       // Thickness of the hourglass walls
const real   alpha( M_PI/3.9 );  // Slope of the hourglass walls (with respect to the x-axis)

const real   beta ( M_PI*real(0.5)-alpha );                   // Slope of the hourglass walls (with respect to the z-axis)
const real   g_gamma( real(2)*M_PI / N );                       // Sector angle for each box forming the hourglass
const real   sina ( std::sin( alpha ) );                      // Sinus of the angle alpha
const real   tana ( std::tan( alpha ) );                      // Tangens of the angle alpha
const real   Z    ( H / sina );                               // Z-size of a single hourglass box
const real   hZ   ( Z * real(0.5) );                          // Half Z-size of a single hourglass box
const real   hL   ( L * real(0.5) );                          // Half the opening size
const real   hW   ( W * real(0.5) );                          // Half the thickness of an hourglass wall
//const real   R    ( H / tana + hL );                          // Radius of the hourglass
const real   R    ( 1 );                          // Radius of the hourglass
const real   Y    ( real(2)*R*std::tan( g_gamma*real(0.5) ) );  // Y-Size of a single hourglass box
const real   space(real(2.)*radius+spacing );                 // Space initially required by a single particle

bool g_povray  ( false );
bool g_vtk( true );
// 
const unsigned int visspacing( 50  );  // Spacing between two visualizations (POV-Ray & Irrlicht)
const unsigned int pointerspacing( 100  );  // Spacing between two visualizations (POV-Ray & Irrlicht)
//const unsigned int visspacing( 100  );  // Spacing between two visualizations (POV-Ray & Irrlicht)

// Switch for checkpointer useage
const bool useCheckpointer( true );  // Switches the checkpointer output of the simulation on and off
 
const int    px(processesX);    // Number of processes in x-direction
const int    py(processesY);    // Number of processes in y-direction
const int    pz(processesZ);    // Number of processes in z-direction
PlaneID g_ground(0);

// Configuration of the simulation world
WorldID world;
int called = 0;

// Configuration of the MPI system
MPISystemID mpisystem;
//*************************************************************************************************
// The Checkpointer
path                 checkpoint_path( "checkpoints/" );            // The path where to store the checkpointing data
Checkpointer checkpointer = Checkpointer(checkpoint_path, pointerspacing, 0, timesteps);

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

  real h = 0.005;
  real epsilon              = 2e-4;
  real radius2              = 0.01 - epsilon;

  real L = 0.1;
  real domainVol = L * L * L;
  real partVol = 4./3. * M_PI * std::pow(radius2, 3);
  real phi = (particlesTotal * partVol)/domainVol * 100.0;

  //=================================================================================================
  int subSteps = 1;
  real subStepSize = stepsize / static_cast<real>(subSteps);
  TimeStep::stepsize(subStepSize);
  for (int istep(0); istep < subSteps; ++istep) {
    world->simulationStep( subStepSize );
  }
  TimeStep::stepsize(stepsize);
  //=================================================================================================

  //world->simulationStepDebug( stepsize );
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
      std::cout << "DEM timestep: " << timestep << "|| sim time: " << timestep * stepsize << " || substepping:  " << subSteps << std::endl;
  }
  
  
  //=================================================================================================
  // Trigger a new checkpointer write if activated
  if (useCheckpointer) {
    checkpointer.trigger();
    checkpointer.flush();
  }

  timestep++;

  // MPI Finalization
  MPI_Barrier(cartcomm);

}

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
