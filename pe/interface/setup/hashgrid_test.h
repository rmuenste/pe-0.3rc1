//=================================================================================================
/*!\file pe/interface/setup/hashgrid_test.h
 *  \brief HashGrid / FBM classification gate test setup (serial mode only).
 *
 *  Entry point: commf2c_hashgrid_test_() (serial mode only).
 *
 *  Parallel PE mode (HAVE_MPI builds only): the setupXxx(MPI_Comm) variant, written against
 *  setup/parallel_common.h. Serial PE mode (PE_SERIAL_MODE, every CFD rank runs its own PE
 *  instance): the setupXxxSerial(int cfd_rank) variant, written against setup/serial_common.h
 *  and compiled in every build so the smoke test can use it. See
 *  doc/technical-notes/pe-interface-serial-mode.md.
 */
//=================================================================================================
#ifndef _PE_INTERFACE_SETUP_HASHGRID_TEST_H_
#define _PE_INTERFACE_SETUP_HASHGRID_TEST_H_

// No parallel setup and no parallel entry point.

#include <pe/interface/setup/serial_common.h>

namespace pe {

/**
 * @brief HashGrid test setup for serial mode
 *
 * Creates 1000 spheres in a 10×10×10 grid for HashGrid acceleration testing.
 * Minimal setup with no JSON dependency - all parameters hardcoded.
 *
 * @param cfd_rank The MPI rank from the CFD domain (used for unique log filenames)
 */
inline void setupHashGridTest(int cfd_rank) {
  // Logger setup
  pe::logging::Logger::setCustomRank(cfd_rank);

  // Load configuration from JSON file
  SimulationConfig::loadFromFile("pe_user_config.json");

  auto &config = SimulationConfig::getInstance();
  config.setCfdRank(cfd_rank);
  const bool isRepresentative = (config.getCfdRank() == 1);

  // Get World and CollisionSystem
  WorldID world = theWorld();
  CollisionSystemID cs = theCollisionSystem();

  // Configure world from config file
  world->setGravity(config.getGravity());
  world->setLiquidSolid(true);
  world->setLiquidDensity(config.getFluidDensity());
  world->setViscosity(config.getFluidViscosity());
  world->setDamping(1.0);
  world->setAutoForceReset(true);
  
  //==============================================================================================
  // Visualization Configuration
  //==============================================================================================
  if (isRepresentative) {
      // Adjust VTK output spacing for substepping
      // With substepping, world->simulationStep() is called substeps times per main timestep
      // To maintain the same VTK output frequency, multiply spacing by substeps
      unsigned int effectiveVisspacing = config.getVisspacing() * config.getSubsteps();
      vtk::WriterID vtk = vtk::activateWriter( "./paraview", effectiveVisspacing, 0,
                                               config.getTimesteps() * config.getSubsteps(),
                                               true);
  }

  // Create Iron material (PE standard)
//  MaterialID iron = createMaterial("iron",
//    7.874,    // density kg/dm³
//    0.5,      // coefficient of restitution
//    0.1,      // static friction
//    0.1,      // dynamic friction
//    0.24,     // Poisson ratio
//    200.0,    // Young's modulus (GPa)
//    200.0,    // stiffness
//    0.0,      // normal damping
//    0.0);     // tangential damping

  // Grid parameters
  const real spacing = 0.1;              // center-to-center
  const int nx = 10, ny = 10, nz = 10;   // 1000 objects total
  const real startPos = 0.05;            // offset to center grid

  // Get packing method from config (default to Grid if not set)
  auto packingMethod = config.getPackingMethod();

  int idx = 0;
  int sphereCount = 0, boxCount = 0, capsuleCount = 0, meshCount = 0;
  int ellipsoidCount = 0, cylinderCount = 0, spacerCount = 0, probeCount = 0;

  if (packingMethod == SimulationConfig::PackingMethod::Grid) {
    // ==================================================================================
    // GRID PACKING: Regular sphere grid (original behavior)
    // ==================================================================================
    const real radius = 0.01;

    for (int iz = 0; iz < nz; ++iz) {
      for (int iy = 0; iy < ny; ++iy) {
        for (int ix = 0; ix < nx; ++ix) {
          Vec3 position(
            startPos + ix * spacing,
            startPos + iy * spacing,
            startPos + iz * spacing
          );

          // Create sphere (all local in serial mode)
          SphereID sphere = createSphere(idx, position, radius, iron, true);
          sphere->setLinearVel(0.0, 0.0, 0.0);
          sphere->setAngularVel(0.0, 0.0, 0.0);
          ++idx;
          ++sphereCount;
        }
      }
    }
  }
  else if (packingMethod == SimulationConfig::PackingMethod::MixedGrid) {
    // ==================================================================================
    // MIXED GRID PACKING: Mixed shapes with bounding sphere radius 0.02
    // ==================================================================================

    // Every shape is sized by its x HALF-EXTENT ex = 0.02 (not a bounding radius) through the
    // same helper the Layer-2 gate uses, so the two populations cannot drift apart. The largest
    // reach of any shape is the box's half-diagonal ex*sqrt(3) = 0.0346 < 0.05, half the lattice
    // spacing, so neighbours never touch. The five shapes close the gap to the Layer-1 stress
    // test; the stale-index defect is shape-independent (a broad-phase defect), but the
    // CONTAINMENT functions of ellipsoid and cylinder were never exercised against real DOFs.
    // No shape is fixed: both point queries skip fixed cylinders and fixed triangle meshes.
    const real probeHalfExtent = 0.02;
    int shapeType = 0;
    const int numShapeTypes = fbmgate::NUM_SHAPES;
    int* shapeCounters[fbmgate::NUM_SHAPES] = { &sphereCount, &boxCount, &capsuleCount,
                                                &ellipsoidCount, &cylinderCount };

    for (int iz = 0; iz < nz; ++iz) {
      for (int iy = 0; iy < ny; ++iy) {
        for (int ix = 0; ix < nx; ++ix) {
          Vec3 position(
            startPos + ix * spacing,
            startPos + iy * spacing,
            startPos + iz * spacing
          );

          fbmgate::createProbeShape(idx, shapeType, position, probeHalfExtent, iron);
          ++*shapeCounters[shapeType];

          ++idx;
          shapeType = (shapeType + 1) % numShapeTypes;
        }
      }
    }
  }
  else if (packingMethod == SimulationConfig::PackingMethod::TriangleMeshGrid) {
    // ==================================================================================
    // TRIANGLE MESH GRID PACKING: Grid of cone meshes from cone.obj
    // ==================================================================================

    // Load cone.obj mesh - it should fit within bounding sphere of radius 0.02
    std::string coneFileName = std::string("cone.obj");

    for (int iz = 0; iz < nz; ++iz) {
      for (int iy = 0; iy < ny; ++iy) {
        for (int ix = 0; ix < nx; ++ix) {
          Vec3 position(
            startPos + ix * spacing,
            startPos + iy * spacing,
            startPos + iz * spacing
          );

          // Create triangle mesh cone at this position
          // global = true (not fixed, can move with fluid forces)
          TriangleMeshID cone = createTriangleMesh(idx, position, coneFileName, iron, false, true);
          cone->setLinearVel(0.0, 0.0, 0.0);
          cone->setAngularVel(0.0, 0.0, 0.0);
          ++idx;
          ++meshCount;
        }
      }
    }
  }

  else if (packingMethod == SimulationConfig::PackingMethod::GateProbe) {
    // ==================================================================================
    // GATE PROBE PACKING: the Layer-2 FBM classification gate construction.
    // Spacers first (they set the HashGrid span), then probes parked outside the mesh until
    // the Fortran driver places them stage by stage. Why it has to be built this way is in
    // pe/interface/fbm_gate_probe.h.
    // ==================================================================================
    fbmgate::createGateProbeBodies(iron, spacerCount, probeCount);
    idx = spacerCount + probeCount;
  }

  // Set timestep from config
  TimeStep::stepsize(config.getStepsize());

  // Diagnostic output
  if (cfd_rank == 1) {
    std::cout << "\n--HASHGRID TEST SETUP\n";
    std::cout << "Configuration file: pe_user_config.json\n";
    std::cout << "Domain: [0, 1]^3\n";
    std::cout << "Grid: " << nx << "x" << ny << "x" << nz << " = " << idx << " objects\n";
    std::cout << "Spacing: " << spacing << "\n";
    std::cout << "Material: Iron (density=" << 7.874 << ")\n";
    std::cout << "Timestep: " << config.getStepsize() << " (" << config.getTimesteps() << " steps)\n";
    std::cout << "Fluid density: " << config.getFluidDensity() << "\n";
    std::cout << "Fluid viscosity: " << config.getFluidViscosity() << "\n";
    std::cout << "Gravity: " << config.getGravity() << "\n";

    if (packingMethod == SimulationConfig::PackingMethod::Grid) {
      std::cout << "Packing: Grid (spheres only)\n";
      std::cout << "  Spheres: " << sphereCount << " (radius=0.01)\n";
    }
    else if (packingMethod == SimulationConfig::PackingMethod::MixedGrid) {
      std::cout << "Packing: MixedGrid (x half-extent=0.02)\n";
      std::cout << "  Spheres: " << sphereCount << " (radius=0.02)\n";
      std::cout << "  Boxes: " << boxCount << " (side=0.04)\n";
      std::cout << "  Capsules: " << capsuleCount << " (r=0.01, cylindrical part 0.02, along x)\n";
      std::cout << "  Ellipsoids: " << ellipsoidCount << " (a=0.02, b=c=0.01)\n";
      std::cout << "  Cylinders: " << cylinderCount << " (r=0.01, l=0.04, along x, not fixed)\n";
    }
    else if (packingMethod == SimulationConfig::PackingMethod::TriangleMeshGrid) {
      std::cout << "Packing: TriangleMeshGrid (bounding radius=0.02)\n";
      std::cout << "  Triangle Meshes (cones): " << meshCount << " (from cone.obj)\n";
    }
    else if (packingMethod == SimulationConfig::PackingMethod::GateProbe) {
      const fbmgate::Construction& gc = fbmgate::construction();
      std::cout << "Packing: GateProbe (Layer-2 FBM classification gate)\n";
      std::cout << "  Spacers: " << spacerCount << " spheres, r=" << gc.rSpacer
                << ", parked at y=z=2 - they set the span\n";
      std::cout << "  Span: " << gc.span << " = (2r + 2ct)*sqrt(hierarchyFactor); planes at k*span: ";
      for (size_t k = 0; k < gc.planes.size(); ++k) std::cout << gc.planes[k] << " ";
      std::cout << "\n  Probes: " << probeCount << ", x half-extent ex=" << gc.ex
                << " (AABB = span - 2*delta, delta=" << gc.delta << "), parked at y=z=3\n";
    }
    std::cout << std::endl;
  }
}

}  // namespace pe

#endif  // _PE_INTERFACE_SETUP_HASHGRID_TEST_H_
