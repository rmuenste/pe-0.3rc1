//=================================================================================================
/*!\file pe/interface/setup/span_complex.h
 *  \brief Span complex case: tool, chip and workpiece meshes with DistanceMaps (serial mode only).
 *
 *  Entry point: commf2c_span_complex_() (serial mode only).
 *
 *  Parallel PE mode (HAVE_MPI builds only): the setupXxx(MPI_Comm) variant, written against
 *  setup/parallel_common.h. Serial PE mode (PE_SERIAL_MODE, every CFD rank runs its own PE
 *  instance): the setupXxxSerial(int cfd_rank) variant, written against setup/serial_common.h
 *  and compiled in every build so the smoke test can use it. See
 *  doc/technical-notes/pe-interface-serial-mode.md.
 */
//=================================================================================================
#ifndef _PE_INTERFACE_SETUP_SPAN_COMPLEX_H_
#define _PE_INTERFACE_SETUP_SPAN_COMPLEX_H_

// No parallel setup and no parallel entry point.

#include <pe/interface/setup/serial_common.h>

namespace pe {

inline size_t estimateDistanceMapMemoryBytes(const DistanceMap& dm) {
  return dm.getSdfData().size() * sizeof(pe::real) +
         dm.getAlphaData().size() * sizeof(int) +
         dm.getNormalData().size() * sizeof(pe::Vec3) +
         dm.getContactPointData().size() * sizeof(pe::Vec3);
}

/**
 * @brief Span complex setup for serial mode
 *
 * Creates the tool, chip, and workpiece triangle meshes used by the complex
 * span case and enables DistanceMap acceleration for each mesh.
 *
 * @param cfd_rank The MPI rank from the CFD domain (used for unique log filenames)
 */
inline void setupSpanComplexSerial(int cfd_rank) {
  pe::logging::Logger::setCustomRank(cfd_rank);
  SimulationConfig::loadFromFile("example.json");

  auto& config = SimulationConfig::getInstance();
  config.setCfdRank(cfd_rank);
  const bool isRepresentative = (config.getCfdRank() == 1);

  WorldID world = theWorld();
  CollisionSystemID cs = theCollisionSystem();
  applyOptionalLubricationParams(*cs, config);

  world->setGravity(config.getGravity());
  world->setLiquidSolid(true);
  world->setLiquidDensity(config.getFluidDensity());
  world->setViscosity(config.getFluidViscosity());
  world->setDamping(1.0);
  world->setAutoForceReset(true);
  TimeStep::stepsize(config.getStepsize());

  struct SpanComplexMeshSpec {
    const char* label;
    const char* file;
    Vec3 position;
  };

  const SpanComplexMeshSpec meshSpecs[] = {
      {"tool", "cm_tool_cm.obj", Vec3(0.0, 0.0, 0.0)},
      {"chip", "cm_chip_negy_cm.obj", Vec3(0.0, 0.0, 0.0)},
      {"workpiece", "cm_workpiece_cm.obj", Vec3(0.0, 0.0, 0.0)}};

  MaterialID meshMaterial = createMaterial("span_complex_mesh",
                                           config.getParticleDensity(),
                                           0.01, 0.05, 0.05, 0.2,
                                           80, 100, 10, 11);

  const int dmResolution = config.getDomainBoundaryDistanceMapResolution();
  const int dmTolerance = config.getDomainBoundaryDistanceMapTolerance();

  int idx = 0;
  int meshesCreated = 0;
  int distanceMapsCreated = 0;
  size_t totalDistanceMapBytes = 0;

  if (isRepresentative) {
    std::cout << "\n--" << "SPAN COMPLEX SETUP"
              << "--------------------------------------------------------------\n"
              << " Simulation stepsize dt                  = " << TimeStep::size() << "\n"
              << " Fluid viscosity                         = " << config.getFluidViscosity() << "\n"
              << " Fluid density                           = " << config.getFluidDensity() << "\n"
              << " Gravity                                 = " << world->getGravity() << "\n"
              << " Distance map resolution                 = " << dmResolution << "\n"
              << " Distance map tolerance                  = " << dmTolerance << "\n";
  }

  for (size_t i = 0; i < sizeof(meshSpecs) / sizeof(meshSpecs[0]); ++i) {
    const SpanComplexMeshSpec& spec = meshSpecs[i];
    TriangleMeshID mesh = createTriangleMesh(++idx, spec.position, spec.file, meshMaterial, false, true);
    mesh->setFixed(true);
    ++meshesCreated;

#ifdef PE_USE_CGAL
    mesh->enableDistanceMapAcceleration(dmResolution, dmTolerance);
    const bool distanceMapEnabled = mesh->hasDistanceMap();
    const DistanceMap* dm = distanceMapEnabled ? mesh->getDistanceMap() : nullptr;
    if (distanceMapEnabled && dm) {
      const size_t bytes = estimateDistanceMapMemoryBytes(*dm);
      totalDistanceMapBytes += bytes;
      ++distanceMapsCreated;

      if (isRepresentative) {
        const double mib = static_cast<double>(bytes) / (1024.0 * 1024.0);
        std::cout << " " << spec.label << " mesh file                      = " << spec.file << "\n"
                  << " " << spec.label << " DistanceMap grid               = "
                  << dm->getNx() << " x " << dm->getNy() << " x " << dm->getNz() << "\n"
                  << " " << spec.label << " DistanceMap spacing            = " << dm->getSpacing() << "\n"
                  << " " << spec.label << " DistanceMap memory estimate    = "
                  << bytes << " bytes (" << mib << " MiB)\n";
      }
    } else if (isRepresentative) {
      std::cerr << "WARNING: DistanceMap acceleration failed to initialize for "
                << spec.label << " mesh " << spec.file << std::endl;
    }
#else
    if (isRepresentative) {
      std::cout << " " << spec.label << " mesh file                      = " << spec.file << "\n"
                << " " << spec.label << " DistanceMap                    = unavailable; rebuild with PE_USE_CGAL\n";
    }
#endif
  }

  if (isRepresentative) {
    const double totalMib = static_cast<double>(totalDistanceMapBytes) / (1024.0 * 1024.0);
    std::cout << " Triangle meshes created                  = " << meshesCreated << "\n"
              << " Distance maps created                    = " << distanceMapsCreated << "\n"
              << " Total DistanceMap memory estimate        = "
              << totalDistanceMapBytes << " bytes (" << totalMib << " MiB)\n"
              << "--------------------------------------------------------------------------------\n"
              << std::endl;
  }
}

}  // namespace pe

#endif  // _PE_INTERFACE_SETUP_SPAN_COMPLEX_H_
