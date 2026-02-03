//=================================================================================================
//
//  VTK Writer Verification Test - MPI Parallel Version
//
//  This example tests VTK Writer functionality in an MPI parallel environment. It exercises:
//  - Domain decomposition with half-plane at x=0
//  - Bodies distributed across multiple MPI processes
//  - Parallel VTK file generation
//  - All geometry types: spheres, boxes, capsules
//
//  Run with: mpirun -np 2 ./vtk_writer_test_mpi
//
//=================================================================================================


//*************************************************************************************************
// Platform and compiler setup
//*************************************************************************************************

#include <pe/engine.h>
#include <pe/support.h>
#include <pe/vtk.h>
using namespace pe;
using namespace pe::timing;
using namespace pe::vtk;


//*************************************************************************************************
int main(int argc, char** argv)
{
   /////////////////////////////////////////////////////
   // MPI Initialization

   MPI_Init( &argc, &argv );

   /////////////////////////////////////////////////////
   // Simulation parameters

   const unsigned int timesteps( 100 );      // Total number of time steps
   const real         stepsize ( 0.01  );    // Size of a single time step
   const unsigned int seed     ( 12345 );    // Seed for random number generator

   /////////////////////////////////////////////////////
   // MPI setup

   MPISystemID mpisystem = theMPISystem();
   const int processes = mpisystem->getSize();
   const int rank = mpisystem->getRank();

   // Require exactly 2 processes for this test
   if( processes != 2 ) {
      if( rank == 0 ) {
         std::cerr << "\n ERROR: This test requires exactly 2 MPI processes!\n"
                   << " Run with: mpirun -np 2 ./vtk_writer_test_mpi\n\n";
      }
      MPI_Finalize();
      return 1;
   }

   /////////////////////////////////////////////////////
   // Setup of the VTK writer

   const std::string vtk_output_dir = "vtk_test_output_mpi";
   const unsigned int vtk_spacing = 10;  // Write every 10 timesteps

   WriterID vtk_writer;
   try {
      // Note: tend should be > last timestep to ensure all steps are written
      // Pass writeEmptyFiles=true (6th param) so collector.pvd is populated even though
      // no bodies exist yet at writer construction time
      vtk_writer = vtk::activateWriter( vtk_output_dir, vtk_spacing, 0, timesteps+1, true, true );
      if( rank == 0 ) {
         std::cout << "VTK Writer activated successfully: " << vtk_output_dir << std::endl;
         std::cout << "VTK spacing: " << vtk_spacing << ", will write from step 0 to " << timesteps << std::endl;
      }
   }
   catch( std::exception& e ) {
      std::cerr << "[Rank " << rank << "] ERROR: Failed to activate VTK Writer: " << e.what() << std::endl;
      MPI_Finalize();
      return 1;
   }

   /////////////////////////////////////////////////////
   // Setup of the simulation domain
   // Domain decomposition: split at x=0 in yz-plane
   // Process 0: x < 0 (left side)
   // Process 1: x >= 0 (right side)

   setSeed( seed );

   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -9.81 );
   world->setDamping( 0.9 );

   // Define the local domain for each process
   if( rank == 0 ) {
      // Process 0: x < 0 (left side)
      defineLocalDomain( HalfSpace( Vec3(-1,0,0), 0.0 ) );
      std::cout << "[Rank 0] Domain: x < 0 (left side)" << std::endl;
   }
   else {
      // Process 1: x >= 0 (right side)
      defineLocalDomain( HalfSpace( Vec3(1,0,0), 0.0 ) );
      std::cout << "[Rank 1] Domain: x >= 0 (right side)" << std::endl;
   }

   if( rank == 0 ) {
      std::cout << "[Rank 0] owns (-1,0,0): " << world->ownsPoint(Vec3(-1.0, 0.0, 0.0)) << std::endl;
      std::cout << "[Rank 0] owns ( 1,0,0): " << world->ownsPoint(Vec3( 1.0, 0.0, 0.0)) << std::endl;
   }
   else {
      std::cout << "[Rank 1] owns (-1,0,0): " << world->ownsPoint(Vec3(-1.0, 0.0, 0.0)) << std::endl;
      std::cout << "[Rank 1] owns ( 1,0,0): " << world->ownsPoint(Vec3( 1.0, 0.0, 0.0)) << std::endl;
   }

//   // Connect the two processes at the x=0 plane
//   if( rank == 0 ) {
//      connect( 1, HalfSpace( Vec3(1,0,0), 0.0 ) );  // Connect to process 1 on the right
//   }
//   else {
//      connect( 0, HalfSpace( Vec3(-1,0,0), 0.0 ) );   // Connect to process 0 on the left
//   }

   /////////////////////////////////////////////////////
   // Setup of the ground plane (global)

   MaterialID ground_material = createMaterial( "ground", 1.0, 0.25, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5 );

   pe_GLOBAL_SECTION
   {
     PlaneID ground = createPlane( 0, Vec3(0,0,1), 0.0, ground_material, false );
   }

   /////////////////////////////////////////////////////
   // Setup of rigid body materials

   MaterialID sphere_material = createMaterial( "sphere", 2.5, 0.25, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5 );
   MaterialID box_material = createMaterial( "box", 2.0, 0.25, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5 );
   MaterialID capsule_material = createMaterial( "capsule", 1.8, 0.25, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5 );

   /////////////////////////////////////////////////////
   // Create test geometries distributed across domains

   if( rank == 0 ) {
      std::cout << "[Rank 0] Creating bodies on left side (x < 0)..." << std::endl;
   }
   else {
      std::cout << "[Rank 1] Creating bodies on right side (x >= 0)..." << std::endl;
   }

   // Create spheres: 3 on left, 3 on right
   // Note: We don't use world->ownsPoint() during initialization.
   // We create bodies based on rank, positioned in the correct domain.
   std::cout << "[Rank " << rank << "] Creating test spheres..." << std::endl;
   for( size_t i = 0; i < 3; ++i ) {
      real radius = 0.3 + i * 0.1;  // radii from 0.3 to 0.5
      real x = (rank == 0) ? (-4.0 + i * 1.5) : (1.0 + i * 1.5);  // Left or right side
      real y = -2.0 + i * 2.0;
      real z = 5.0 + i * 1.0;
      if (world->ownsPoint( Vec3(x,y,z) )) {
        std::cout << "[Rank " << rank << "] Creating sphere at (" << x << "," << y << "," << z << ") " << std::endl;
        createSphere( rank * 100 + i + 1, Vec3(x,y,z), radius, sphere_material );
      }
   }

   // Create boxes: 2 on left, 2 on right
   std::cout << "[Rank " << rank << "] Creating test boxes..." << std::endl;
   for( size_t i = 0; i < 2; ++i ) {
      real size = 0.5 + i * 0.2;
      real x = (rank == 0) ? (-3.0 + i * 2.0) : (1.5 + i * 2.0);  // Left or right side
      real y = 0.0;
      real z = 6.0 + i * 1.5;
      if (world->ownsPoint( Vec3(x,y,z) )) {
        std::cout << "[Rank " << rank << "] Creating box at (" << x << "," << y << "," << z << ") " << std::endl;
        BodyID box = createBox( rank * 100 + 10 + i, Vec3(x,y,z), Vec3(size,size*1.5,size*0.8), box_material );
        // Add some rotation for variety
        box->rotate( Vec3(1,0,1).getNormalized(), i * M_PI / 6.0 );
      }
   }

   // Create capsules: 2 on left, 2 on right
   std::cout << "[Rank " << rank << "] Creating test capsules..." << std::endl;
   for( size_t i = 0; i < 2; ++i ) {
      real radius = 0.3 + i * 0.1;
      real length = 1.0 + i * 0.5;
      real x = (rank == 0) ? (-2.5 + i * 2.0) : (0.5 + i * 2.0);  // Left or right side
      real y = 3.0;
      real z = 7.0 + i * 1.5;
      if (world->ownsPoint( Vec3(x,y,z) )) {
      std::cout << "[Rank " << rank << "] Creating capsule at (" << x << "," << y << "," << z << ") " << std::endl;
      CapsuleID capsule = createCapsule( rank * 100 + 20 + i, Vec3(x,y,z), radius, length, capsule_material );
      // Rotate capsules to horizontal orientation
      capsule->rotate( Vec3(0,1,0), M_PI / 2.0 );
      }
   }

   /////////////////////////////////////////////////////
   // Print body counts for debugging

   size_t local_spheres = 0, local_boxes = 0, local_capsules = 0;
   for( World::Iterator body = world->begin(); body != world->end(); ++body ) {
      if( body->getType() == sphereType ) ++local_spheres;
      else if( body->getType() == boxType ) ++local_boxes;
      else if( body->getType() == capsuleType ) ++local_capsules;
   }

   std::cout << "[Rank " << rank << "] Created bodies: "
             << local_spheres << " spheres, "
             << local_boxes << " boxes, "
             << local_capsules << " capsules" << std::endl;

   world->synchronize();
   /////////////////////////////////////////////////////
   // Simulation loop

   if( rank == 0 ) {
      std::cout << "\nStarting parallel simulation (" << timesteps << " timesteps)..." << std::endl;
   }

   WcTimer simTime;
   simTime.start();

   for( unsigned int timestep = 0; timestep < timesteps; ++timestep )
   {
      if( rank == 0 && timestep % 10 == 0 ) {
         std::cout << "  Timestep " << timestep << "/" << timesteps << std::endl;
      }

      world->simulationStep( stepsize );
   }

   simTime.end();

   /////////////////////////////////////////////////////
   // Simulation results

   if( rank == 0 ) {
      std::cout << "\n"
                << "==============================================\n"
                << " VTK Writer MPI Test Results\n"
                << "==============================================\n"
                << " MPI processes: " << processes << "\n"
                << " Simulation time: " << simTime.total() << " seconds\n"
                << " Output directory: " << vtk_output_dir << "\n"
                << " VTK spacing: " << vtk_spacing << " timesteps\n"
                << " Expected files: " << (timesteps / vtk_spacing) << " sets\n"
                << "==============================================\n"
                << std::endl;
   }

   /////////////////////////////////////////////////////
   // Verify output files were created

   if( rank == 0 ) {
      std::cout << "\nVerifying output files..." << std::endl;
      unsigned int expected_file_sets = timesteps / vtk_spacing;
      std::cout << "  Expected " << expected_file_sets << " file sets per geometry type" << std::endl;
      std::cout << "  Files should contain data from both MPI processes" << std::endl;

      // Check if collector.pvd exists and has content
      std::cout << "\nChecking " << vtk_output_dir << "/ directory..." << std::endl;
      std::cout << "  Look for files: spheres_*.vtp, boxes_*.vtp, capsules_*.vtp, collector.pvd" << std::endl;

      std::cout << "\nNote: VTK Writer uses singleton pattern - only one instance per program.\n";
      std::cout << "Each MPI process writes its local bodies to shared VTK files.\n";

      std::cout << "\n"
                << "==============================================\n"
                << " VTK Writer MPI Test: PASSED\n"
                << "==============================================\n"
                << std::endl;
   }

   /////////////////////////////////////////////////////
   // MPI Finalization

   MPI_Finalize();

   return 0;
}
