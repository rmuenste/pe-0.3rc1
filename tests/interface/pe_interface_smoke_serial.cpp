#include <pe/config/SimulationConfig.h>
#include <pe/core.h>
#include <pe/core/BodyBinaryWriter.h>
#include <pe/interface/sim_setup_serial.h>

#include <boost/filesystem.hpp>

#include <cstdlib>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <sys/wait.h>
#include <unistd.h>

namespace {

namespace fs = boost::filesystem;

struct Args {
  std::string caseName;
  std::string phase;
  fs::path fixtureDir;
  fs::path workRoot;
  bool prepare;
};

void fail(const std::string& message) {
  throw std::runtime_error(message);
}

void require(bool condition, const std::string& message) {
  if (!condition) {
    fail(message);
  }
}

Args parseArgs(int argc, char** argv) {
  Args args;
  args.phase = "normal";
  args.prepare = true;
  for (int i = 1; i < argc; ++i) {
    const std::string key(argv[i]);
    if (key == "--case" && i + 1 < argc) {
      args.caseName = argv[++i];
    } else if (key == "--phase" && i + 1 < argc) {
      args.phase = argv[++i];
    } else if (key == "--fixtures" && i + 1 < argc) {
      args.fixtureDir = fs::path(argv[++i]);
    } else if (key == "--work-root" && i + 1 < argc) {
      args.workRoot = fs::path(argv[++i]);
    } else if (key == "--no-prepare") {
      args.prepare = false;
    } else {
      fail("Unknown or incomplete argument: " + key);
    }
  }

  require(!args.caseName.empty(), "Missing --case");
  require(!args.fixtureDir.empty(), "Missing --fixtures");
  require(!args.workRoot.empty(), "Missing --work-root");
  return args;
}

void runSeedProcess(const fs::path& program, const Args& args) {
  std::vector<std::string> childArgs;
  childArgs.push_back(program.string());
  childArgs.push_back("--case");
  childArgs.push_back(args.caseName);
  childArgs.push_back("--phase");
  childArgs.push_back("seed");
  childArgs.push_back("--fixtures");
  childArgs.push_back(args.fixtureDir.string());
  childArgs.push_back("--work-root");
  childArgs.push_back(args.workRoot.string());
  childArgs.push_back("--no-prepare");

  std::vector<char*> argv;
  for (auto& arg : childArgs) {
    argv.push_back(&arg[0]);
  }
  argv.push_back(nullptr);

  const pid_t pid = ::fork();
  if (pid < 0) {
    fail("Could not fork seed process");
  }
  if (pid == 0) {
    ::execv(program.string().c_str(), argv.data());
    std::cerr << "Could not exec seed process: " << program << std::endl;
    _exit(127);
  }

  int status = 0;
  if (::waitpid(pid, &status, 0) < 0) {
    fail("Could not wait for seed process");
  }
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    fail("Seed process failed for " + args.caseName);
  }
}

void copyFixtures(const fs::path& source, const fs::path& destination) {
  require(fs::exists(source), "Fixture directory does not exist: " + source.string());
  fs::remove_all(destination);
  fs::create_directories(destination);

  for (fs::recursive_directory_iterator it(source), end; it != end; ++it) {
    const fs::path relative = fs::relative(it->path(), source);
    const fs::path target = destination / relative;
    if (fs::is_directory(it->path())) {
      fs::create_directories(target);
    } else if (fs::is_regular_file(it->path())) {
      fs::create_directories(target.parent_path());
      fs::copy_file(it->path(), target, fs::copy_option::overwrite_if_exists);
    }
  }
}

void selectConfig(const fs::path& runDir, const std::string& configName) {
  const fs::path source = runDir / configName;
  const fs::path target = runDir / "example.json";
  require(fs::exists(source), "Missing fixture config: " + source.string());
  fs::copy_file(source, target, fs::copy_option::overwrite_if_exists);
}

void enterRunDir(const fs::path& runDir) {
  if (::chdir(runDir.string().c_str()) != 0) {
    fail("Could not chdir to runtime directory: " + runDir.string());
  }
}

size_t countBodies(pe::GeomType type) {
  size_t count = 0;
  const auto& storage = pe::theCollisionSystem()->getBodyStorage();
  for (auto it = storage.begin(); it != storage.end(); ++it) {
    if ((*it)->getType() == type) {
      ++count;
    }
  }
  return count;
}

void requireCounts(size_t spheres, size_t triangleMeshes, size_t planes) {
  require(countBodies(pe::sphereType) == spheres,
          "Unexpected sphere count: expected " + std::to_string(spheres) +
              ", got " + std::to_string(countBodies(pe::sphereType)));
  require(countBodies(pe::triangleMeshType) == triangleMeshes,
          "Unexpected triangle mesh count: expected " + std::to_string(triangleMeshes) +
              ", got " + std::to_string(countBodies(pe::triangleMeshType)));
  require(countBodies(pe::planeType) == planes,
          "Unexpected plane count: expected " + std::to_string(planes) +
              ", got " + std::to_string(countBodies(pe::planeType)));
}

void requireAtLeastPlanes(size_t minimum) {
  const size_t planes = countBodies(pe::planeType);
  require(planes >= minimum,
          "Unexpected plane count: expected at least " + std::to_string(minimum) +
              ", got " + std::to_string(planes));
}

void requireAllSpheresYLocked() {
  size_t spheres = 0;
  const auto& storage = pe::theCollisionSystem()->getBodyStorage();
  for (auto it = storage.begin(); it != storage.end(); ++it) {
    if ((*it)->getType() != pe::sphereType) {
      continue;
    }
    ++spheres;
    const pe::Vec3& mask = (*it)->getLinearDofMask();
    require(mask[0] == pe::real(1) && mask[1] == pe::real(0) && mask[2] == pe::real(1),
            "Sphere y-DOF lock was not applied");
  }
  require(spheres > 0, "No spheres found while checking y-DOF lock");
}

void writeCheckpoint(const fs::path& checkpointPath, const std::string& name) {
  fs::create_directories(checkpointPath);
  pe::BodyBinaryWriter writer;
  writer.writeFile((checkpointPath / (name + ".peb")).string().c_str());
}

void requireCommonConfig(bool resume, pe::SimulationConfig::PackingMethod packingMethod) {
  const auto& config = pe::SimulationConfig::getInstance();
  require(config.getVtk() == false, "Config vtk_ should be false");
  require(config.getUseCheckpointer() == false, "Config useCheckpointer_ should be false");
  require(config.getResume() == resume, "Config resume_ has unexpected value");
  require(config.getPackingMethod() == packingMethod, "Config packingMethod_ has unexpected value");
}

void runAtcExternal() {
  pe::setupATCSerial(1);
  requireCommonConfig(false, pe::SimulationConfig::External);
  require(pe::SimulationConfig::getInstance().getDomainBoundaryDistanceMapEnabled() == false,
          "ATC distance map should be disabled for lightweight fixture");
  requireCounts(3, 1, 0);
}

void seedAtcRoundtrip() {
  pe::setupATCSerial(1);
  requireCommonConfig(false, pe::SimulationConfig::External);
  require(countBodies(pe::sphereType) == 3, "ATC seed did not create fixture particles");
  require(countBodies(pe::triangleMeshType) == 1, "ATC seed did not create boundary mesh");
  writeCheckpoint("checkpoints", "resume_seed");
}

void runFluidizationSrrFresh() {
  pe::setupFluidizationSRRSerial(1);
  requireCommonConfig(false, pe::SimulationConfig::Grid);
  require(countBodies(pe::sphereType) == 1204,
          "Fluidization SRR fresh setup did not create 1204 particles");
  requireAtLeastPlanes(3);
  requireAllSpheresYLocked();
}

void seedFluidizationSrrRoundtrip() {
  pe::setupFluidizationSRRSerial(1);
  requireCommonConfig(false, pe::SimulationConfig::Grid);
  require(countBodies(pe::sphereType) == 1204,
          "Fluidization SRR seed did not create 1204 particles");
  requireAtLeastPlanes(3);
  requireAllSpheresYLocked();
  writeCheckpoint("checkpoints", "resume_seed");
}

} // namespace

int main(int argc, char** argv) {
  try {
    const Args args = parseArgs(argc, argv);
    const fs::path program = fs::system_complete(argv[0]);
    const fs::path runDir = args.workRoot / args.caseName;
    if (args.prepare) {
      copyFixtures(args.fixtureDir, runDir);
    }

    if (args.caseName == "atc-external") {
      selectConfig(runDir, "atc_external.json");
    } else if (args.caseName == "atc-resume-roundtrip") {
      selectConfig(runDir, "atc_external.json");
    } else if (args.caseName == "fluidization-srr-fresh") {
      selectConfig(runDir, "fluidization_srr_fresh.json");
    } else if (args.caseName == "fluidization-srr-resume-roundtrip") {
      selectConfig(runDir, "fluidization_srr_fresh.json");
    } else {
      fail("Unknown test case: " + args.caseName);
    }

    if (args.phase == "normal" && args.caseName == "atc-resume-roundtrip") {
      runSeedProcess(program, args);
      selectConfig(runDir, "atc_resume.json");
      enterRunDir(runDir);
      pe::setupATCSerial(1);
      requireCommonConfig(true, pe::SimulationConfig::Grid);
      require(countBodies(pe::sphereType) == 3,
              "ATC resume changed particle count; grid packing may have run during resume");
      require(countBodies(pe::triangleMeshType) == 1,
              "ATC resume should discard checkpoint meshes and recreate exactly one boundary mesh");
    } else if (args.phase == "normal" &&
               args.caseName == "fluidization-srr-resume-roundtrip") {
      runSeedProcess(program, args);
      selectConfig(runDir, "fluidization_srr_resume.json");
      enterRunDir(runDir);
      pe::setupFluidizationSRRSerial(1);
      requireCommonConfig(true, pe::SimulationConfig::Grid);
      require(countBodies(pe::sphereType) == 1204,
              "Fluidization SRR resume changed particle count");
      requireAtLeastPlanes(3);
      requireAllSpheresYLocked();
    } else {
      enterRunDir(runDir);
      if (args.phase == "seed" && args.caseName == "atc-resume-roundtrip") {
        seedAtcRoundtrip();
      } else if (args.phase == "seed" && args.caseName == "fluidization-srr-resume-roundtrip") {
        seedFluidizationSrrRoundtrip();
      } else if (args.phase != "normal") {
        fail("Unknown test phase: " + args.phase);
      } else if (args.caseName == "atc-external") {
        runAtcExternal();
      } else if (args.caseName == "fluidization-srr-fresh") {
        runFluidizationSrrFresh();
      }
    }

    std::cout << args.caseName << ": PASS" << std::endl;
    return EXIT_SUCCESS;
  } catch (const std::exception& e) {
    std::cerr << "pe_interface_smoke_serial: FAIL: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
}
