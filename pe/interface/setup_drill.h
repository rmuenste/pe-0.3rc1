
#include <pe/interface/decompose.h>
#include <pe/config/SimulationConfig.h>
#include <random>
#include <algorithm>
#include <vector>
#include <iostream>
#include <sstream>
#include <pe/core/Types.h>
#include <pe/core/detection/fine/DistanceMap.h>
#include <pe/vtk/UtilityWriters.h>

using namespace pe::povray;
//=================================================================================================

// Helper function for coordinate transformation debug test
void testChipPointContainment(const TriangleMeshID& chip, const Vec3& testPoint, const std::string& description) {
    std::cout << "\n=== " << description << " ===" << std::endl;
    
    // Print chip position
    std::cout << "Chip position: (" << chip->getPosition()[0] << ", " << chip->getPosition()[1] << ", " << chip->getPosition()[2] << ")" << std::endl;
    std::cout << "Test point:    (" << testPoint[0] << ", " << testPoint[1] << ", " << testPoint[2] << ")" << std::endl;
    
    // Test with DistanceMap if available
    if (chip->hasDistanceMap()) {
        bool dmResult = chip->containsPoint(testPoint);
        std::cout << "DistanceMap result: " << (dmResult ? "INSIDE" : "OUTSIDE") << std::endl;
        
        // Get additional DistanceMap info for debugging
        const DistanceMap* dm = chip->getDistanceMap();
        if (dm) {
            // Transform point to local coordinates (same as mesh does internally)
            Vec3 localPoint = chip->pointFromWFtoBF(testPoint);
            std::cout << "Local point:   (" << localPoint[0] << ", " << localPoint[1] << ", " << localPoint[2] << ")" << std::endl;
            
            // Query distance directly
            pe::real distance = dm->interpolateDistance(localPoint[0], localPoint[1], localPoint[2]);
            std::cout << "Signed distance: " << distance << std::endl;
            std::cout << "Distance sign indicates: " << (distance < 0 ? "INSIDE" : "OUTSIDE") << std::endl;
        }
    } else {
        std::cout << "No DistanceMap available!" << std::endl;
    }
    
    // Print chip AABB for reference
    const auto& bbox = chip->getAABB();
    std::cout << "Chip AABB: [" << bbox[0] << "," << bbox[3] << "] x ["
              << bbox[1] << "," << bbox[4] << "] x ["
              << bbox[2] << "," << bbox[5] << "]" << std::endl;
    
    // Check if point is in AABB
    bool inAABB = (testPoint[0] >= bbox[0] && testPoint[0] <= bbox[3]) &&
                  (testPoint[1] >= bbox[1] && testPoint[1] <= bbox[4]) &&
                  (testPoint[2] >= bbox[2] && testPoint[2] <= bbox[5]);
    std::cout << "Point in AABB: " << (inAABB ? "YES" : "NO") << std::endl;
}

//=================================================================================================
// Setup for the Drill Application
//=================================================================================================
void setupDrill(MPI_Comm ex0) {

  auto& config = SimulationConfig::getInstance();
  world = theWorld();

  loadSimulationConfig("example.json");
  world->setGravity( config.getGravity() );

  real simRho( config.getFluidDensity() );
  world->setLiquidDensity( simRho );

  // Particle Bench Config 
  world->setLiquidSolid(true);
  world->setDamping( 1.0 );

  // Lubrication switch
  bool useLubrication(false);

  // Configuration of the MPI system
  mpisystem = theMPISystem();
  mpisystem->setComm(ex0);

  unsigned int id( 0 );              // User-specific ID counter

  int my_rank;
  MPI_Comm_rank(ex0, &my_rank);

  // Checking the total number of MPI processes
  if( config.getProcessesX()*config.getProcessesY()*config.getProcessesZ() != mpisystem->getSize() ) {
     std::cerr << "\n Invalid number of MPI processes: " << mpisystem->getSize() << "!=" << config.getProcessesX()*config.getProcessesY()*config.getProcessesZ() << "\n\n" << std::endl;
     std::exit(EXIT_FAILURE);
  }

  /////////////////////////////////////////////////////
  // Setup of the MPI processes: 3D Rectilinear Domain Decomposition

  // Computing the Cartesian coordinates of the neighboring processes
  int dims   [] = { config.getProcessesX(), config.getProcessesY(), config.getProcessesZ() };
  int periods[] = { false, false, false };
  int reorder   = false;

  int rank;           // Rank of the neighboring process
  int center[3];      // Definition of the coordinates array 'center' (the cartesian topology)
  MPI_Comm cartcomm;  // The new MPI communicator with Cartesian topology

  /*
   * Here the actual cartesian communicator is created from MPI_COMM_WORLD and the parameters
   * of the cartesian grid setup
   * \param MPI_COMM_WORLD The default communicator
   * \param ndims Number of dimensions of the cartesian grid
   * \param dims Array of size ndims, dims[i] = number of processes in dimension i 
   * \param wrap_around Array of size ndims with wrap_around[i] = wrapping on/off for dimension i 
   */
  MPI_Cart_create(ex0, 3, dims, periods, reorder, &cartcomm);
  if( cartcomm == MPI_COMM_NULL ) {
     std::cout << "Error creating 3D communicator" << std::endl;
     MPI_Finalize();
     return;
  }

  mpisystem->setComm(cartcomm);

  pe_EXCLUSIVE_SECTION(0) {
    std::cout << "> 3D communicator created" << std::endl;
    std::cout << (Vec3(dims[0], dims[1], dims[2])) << std::endl;
  }

  // Here the cartesian coordinates of the different processes are created
  /*  
   * \param comm2D The cartesian communicator created by MPI_Cart_create
   * \param my_rank The rank with regard to MPI_COMM_WORLD
   * \param ndims Dimensions of the cartesian grid
   * \param coord An array of a size equivalent to the dimension of the cartesian grid
   *  coord[0] x coord[0] would correspond to the cartesian coordinates of the first process of a 2D cartesian grid
   */
  MPI_Cart_coords(cartcomm, mpisystem->getRank(), 3, center);

  int my_cart_rank;
  MPI_Cart_rank(cartcomm, center, &my_cart_rank);

  pe_EXCLUSIVE_SECTION(0) {
    std::cout << "3D coordinates were created" << std::endl;
    std::cout << (Vec3(center[0], center[1], center[2])) << std::endl;
  }

//===================================================================================================

  int px = config.getProcessesX();
  int py = config.getProcessesY();
  int pz = config.getProcessesZ();

  real bx = 0.0;
  real by = 0.0;
  real bz = 0.0;

  const real dx( 6.0 / px );
  const real dy( 6.0 / py );
  const real dz( 15. / pz );

  decomposeDomain(center, bx, by, bz, dx, dy, dz, px, py, pz);

//===================================================================================================

//#ifndef NDEBUG
   // Checking the process setup
   theMPISystem()->checkProcesses();
//#endif

  MaterialID gr = createMaterial("ground", config.getParticleDensity(), 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);

  // Setup of the VTK visualization
  if( g_vtk ) {
     vtk::WriterID vtk = vtk::activateWriter( "./paraview", config.getVisspacing(), 0, config.getTimesteps(), false);
  }

  //======================================================================================== 
  // Here is how to create some random positions on a grid up to a certain
  // volume fraction.
  //======================================================================================== 
  bool resume               = false;
  real epsilon              = 2e-4;
  real targetVolumeFraction = 0.35;
  real radius2              = 0.01 - epsilon;

  int idx = 0;
  real h = 0.00125;

  std::string fileName = std::string("chip1.obj");
  real chipDensity = 1000.00;  // Keep hardcoded - no config function available

  //=========================================================================================
  // Creation and positioning of the global tool
  //=========================================================================================
  MaterialID toolMat = createMaterial("tool", chipDensity, 0.01, 0.05, 0.05, 0.2, 80, 100, 10, 11);

//  pe_GLOBAL_SECTION
//  {
//    Vec3 toolPos = Vec3(0.0, 0.0, 0.0);
//    TriangleMeshID tool = createTriangleMesh(++id, toolPos, "tool.obj", toolMat, false, true);
//    tool->setFixed(true);
//    std::cout << "Global fixed tool created at position: (" << toolPos[0] << ", " << toolPos[1] << ", " << toolPos[2] << ")" << std::endl;
//
//    // Enable DistanceMap acceleration for the tool
//    tool->enableDistanceMapAcceleration(0.05, 64, 3);  // spacing, resolution, tolerance
//    if (!tool->hasDistanceMap()) {
//      std::cerr << "WARNING: DistanceMap acceleration failed to initialize for tool" << std::endl;
//    } else {
//      std::cout << "DistanceMap acceleration enabled successfully for tool!" << std::endl;
//      const DistanceMap* dm = tool->getDistanceMap();
//      if (dm) {
//        std::cout << "Tool DistanceMap grid: " << dm->getNx() << " x " << dm->getNy() << " x " << dm->getNz() << std::endl;
//        std::cout << "Tool DistanceMap origin: (" << dm->getOrigin()[0] << ", " << dm->getOrigin()[1] << ", " << dm->getOrigin()[2] << ")" << std::endl;
//        std::cout << "Tool DistanceMap spacing: " << dm->getSpacing() << std::endl;
//        
//        // Export tool DistanceMap to VTI file for visualization
//        std::cout << "\n=== EXPORTING TOOL DISTANCEMAP TO VTI ===" << std::endl;
//        std::string toolVtiFile = "tool.vti";
//        std::cout << "Exporting tool DistanceMap to " << toolVtiFile << "..." << std::endl;
//        
//        try {
//          pe::vtk::DistanceMapWriter::writeVTI(toolVtiFile, *dm);
//          std::cout << "Tool DistanceMap export completed successfully!" << std::endl;
//          std::cout << "Note: DistanceMap is in LOCAL tool coordinates" << std::endl;
//        } catch (const std::exception& e) {
//          std::cerr << "ERROR: Failed to export tool DistanceMap: " << e.what() << std::endl;
//        }
//      }
//    }
//  }

  //=========================================================================================
  // Creation and positioning of the chip
  //=========================================================================================
  // Create a custom material for the chip
  // Creates the material "myMaterial" with the following material properties:
  //  - material density               : 2.54
  //  - coefficient of restitution     : 0.8
  //  - coefficient of static friction : 0.1
  //  - coefficient of dynamic friction: 0.05
  //  - Poisson's ratio                : 0.2
  //  - Young's modulus                : 80
  //  - Contact stiffness              : 100
  //  - dampingN                       : 10
  //  - dampingT                       : 11
  MaterialID chipMat = createMaterial("chip"    , chipDensity , 0.01, 0.05, 0.05, 0.2, 80, 100, 10, 11);

  Vec3 chipPos = Vec3(3.0, 3.0, 7.5);  // Keep hardcoded - no config function available
  TriangleMeshID chip;

  if(!resume) {
    if(world->ownsPoint(chipPos)) {
      chip = createTriangleMesh(++id, chipPos, fileName, chipMat, false, true);
      std::cout << "Chip is owned by domain: " << mpisystem->getRank() << " initially." << std::endl;
      std::cout << "Chip x:[" << chip->getAABB()[3] << "," << chip->getAABB()[0] << "]" << std::endl;
      
      // Enable DistanceMap acceleration for the chip
      chip->enableDistanceMapAcceleration(64, 3);  // spacing, resolution, tolerance
      if (!chip->hasDistanceMap()) {
        std::cerr << "WARNING: DistanceMap acceleration failed to initialize for chip" << std::endl;
      } else {
        std::cout << "DistanceMap acceleration enabled successfully for chip!" << std::endl;
        const DistanceMap* dm = chip->getDistanceMap();
        if (dm) {
          std::cout << "DistanceMap grid: " << dm->getNx() << " x " << dm->getNy() << " x " << dm->getNz() << std::endl;
          std::cout << "DistanceMap origin: (" << dm->getOrigin()[0] << ", " << dm->getOrigin()[1] << ", " << dm->getOrigin()[2] << ")" << std::endl;
          std::cout << "DistanceMap spacing: " << dm->getSpacing() << std::endl;
        }
        
        // Coordinate transformation debug test (similar to debug_coordinate_transform.cpp)
        Vec3 testPoint(-0.086035, -0.869566, 0.477526);  // Fixed test point from debug example
        
        std::cout << "\n=== COORDINATE TRANSFORMATION DEBUG TEST ===" << std::endl;
        std::cout << "Running 4-phase coordinate validation test..." << std::endl;
        
        // Test 1: Original point with chip at current position
        testChipPointContainment(chip, testPoint, "Test 1: Point vs Chip at Original Position");
        
        // Test 2: Translate point +10 in Z - should be OUTSIDE
        Vec3 translatedPoint = testPoint + Vec3(0, 0, 10);
        testChipPointContainment(chip, translatedPoint, "Test 2: Point+10Z vs Chip at Original Position (Expected: OUTSIDE)");
        
        // Test 3: Translate chip +10 in Z to align with translated point
        Vec3 originalChipPos = chip->getPosition();
        chip->setPosition(originalChipPos + Vec3(0, 0, 10));
        chip->calcBoundingBox();  // Force update of cached data
        
        testChipPointContainment(chip, translatedPoint, "Test 3: Point+10Z vs Chip+10Z (Expected: INSIDE if coord transforms work)");
        
        // Test 4: Original point vs translated chip - should be OUTSIDE
        testChipPointContainment(chip, testPoint, "Test 4: Original Point vs Chip+10Z (Expected: OUTSIDE)");
        
        // Reset chip to original position for simulation
        chip->setPosition(originalChipPos);
        chip->calcBoundingBox();
        
        // Export DistanceMap to VTI file for visualization
//#ifdef PE_USE_CGAL
//        if (chip->hasDistanceMap()) {
//          const DistanceMap* dm = chip->getDistanceMap();
//          if (dm) {
//            std::cout << "\n=== EXPORTING DISTANCEMAP TO VTI ===" << std::endl;
//            std::string dmVtiFile = "chip_distance_map.vti";
//            std::cout << "Exporting DistanceMap to " << dmVtiFile << "..." << std::endl;
//            
//            write_vti(dmVtiFile,
//                     dm->getSdfData(),
//                     dm->getAlphaData(),
//                     dm->getNormalData(),
//                     dm->getContactPointData(),
//                     std::vector<int>(dm->getSdfData().size(), 0), // face_index placeholder
//                     dm->getNx(), dm->getNy(), dm->getNz(),
//                     dm->getSpacing(), dm->getSpacing(), dm->getSpacing(),
//                     dm->getOrigin()[0], dm->getOrigin()[1], dm->getOrigin()[2]);
//            
//            std::cout << "DistanceMap export completed successfully!" << std::endl;
//            std::cout << "Note: DistanceMap is in LOCAL chip coordinates" << std::endl;
//          }
//        }
//#endif
        
        std::cout << "\n=== COORDINATE DEBUG TEST COMPLETE ===" << std::endl;
        std::cout << "Chip reset to original position for simulation." << std::endl;
      }
    }
  }
  else {
    checkpointer.read( "../start.1" );
  }

  //=========================================================================================
  // Creation and positioning of the drill (global rigid body)
  //=========================================================================================
  pe_GLOBAL_SECTION
  {
    std::string drillFileName = std::string("drill.obj");
    Vec3 drillPos(3.0, 3.0, 10.0);  // Position drill above workpiece
    MaterialID drillMat = createMaterial("drill", 7800.0, 0.01, 0.05, 0.05, 0.2, 80, 100, 10, 11);  // Steel density

    TriangleMeshID drill = createTriangleMesh(++id, drillPos, drillFileName, drillMat, false, true);
    std::cout << "Global drill created at position: (" << drillPos[0] << ", " << drillPos[1] << ", " << drillPos[2] << ")" << std::endl;

    // Enable DistanceMap acceleration for the drill
    drill->enableDistanceMapAcceleration(64, 3);  // spacing, resolution, tolerance
    if (!drill->hasDistanceMap()) {
      std::cerr << "WARNING: DistanceMap acceleration failed to initialize for drill" << std::endl;
    } else {
      std::cout << "DistanceMap acceleration enabled successfully for drill!" << std::endl;
      const DistanceMap* dm = drill->getDistanceMap();
      if (dm) {
        std::cout << "Drill DistanceMap grid: " << dm->getNx() << " x " << dm->getNy() << " x " << dm->getNz() << std::endl;
        std::cout << "Drill DistanceMap origin: (" << dm->getOrigin()[0] << ", " << dm->getOrigin()[1] << ", " << dm->getOrigin()[2] << ")" << std::endl;
        std::cout << "Drill DistanceMap spacing: " << dm->getSpacing() << std::endl;
      }
    }
  }

//  //=========================================================================================
//  // No bounding planes for now
//  //=========================================================================================
//  pe_GLOBAL_SECTION
//  {
//     // Setup of the ground plane
//     PlaneID plane = createPlane( id++, 0.0, 0.0, 1.0, -0.0, gr );
//     // +y
//     createPlane( id++, 0.0, 1.0, 0.0,  0.0, gr );
//     // -y
//     createPlane( id++, 0.0,-1.0, 0.0,  -0.02, gr );
//  }
//  //=========================================================================================

  // Synchronization of the MPI processes
  world->synchronize();

  //=========================================================================================  
  // Calculating the total number of particles and primitives
  unsigned long particlesTotal ( 0 );
  unsigned long primitivesTotal( 0 );
  unsigned long bla = idx;

  int numBodies (0);
  int numTotal (0);
  unsigned int j(0);
  for (; j < theCollisionSystem()->getBodyStorage().size(); j++) {
    World::SizeType widx = static_cast<World::SizeType>(j);
    BodyID body = world->getBody(static_cast<unsigned int>(widx));
    if(body->getType() == triangleMeshType) {
      numBodies++;
      numTotal++;
    } else {
      numTotal++;
    }
  }

  unsigned long bodiesUpdate = static_cast<unsigned long>(numBodies);
  unsigned long bodiesTotal = static_cast<unsigned long>(numTotal);
  MPI_Reduce( &bodiesUpdate, &particlesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm );
  MPI_Reduce( &bodiesTotal, &primitivesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm );

  real partVol = 4./3. * M_PI * std::pow(radius2, 3);

  std::string resOut = (resume) ? "resuming " : "not resuming ";
  std::string useLub = (useLubrication) ? "enabled" : "disabled";

  pe_EXCLUSIVE_SECTION( 0 ) {
    std::cout << "\n--" << "DRILL APPLICATION SETUP"
      << "--------------------------------------------------------------\n"
      << " Total number of MPI processes           = " << px * py * pz << "\n"
      << " Simulation stepsize dt                  = " << TimeStep::size() << "\n"
      << " Total number of triangle meshes         = " << 2 << " (workpiece + drill)\n"
      << " Total number of objects                 = " << primitivesTotal << "\n"
      << " Fluid Density                           = " << simRho << " [g/cm**3]"  << "\n"
      << " Workpiece Density                       = " << chipDensity << " [g/cm**3]"  << "\n"
      << " Drill Density                           = " << 7800.0 << " [g/cm**3] (steel)"  << "\n"
      << " Gravity constant                        = " << world->getGravity() << "\n"
      << " Contact threshold                       = " << contactThreshold << "\n"
      << " Domain volume                           = " << 0.1 * 0.1 * 0.1 << " [cm**3]" << "\n"
      << " Workpiece volume                        = " << 0.000103797 << " [cm**3]" << "\n"
      << " Resume                                  = " << resOut  << "\n"
      << " Total objects                           = " << primitivesTotal << "\n" << std::endl;
     std::cout << "--------------------------------------------------------------------------------\n" << std::endl;
  }

  MPI_Barrier(cartcomm);
   

}

