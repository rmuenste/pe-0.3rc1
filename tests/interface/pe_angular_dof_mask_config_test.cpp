//=================================================================================================
/*!
 *  \file tests/interface/pe_angular_dof_mask_config_test.cpp
 *  \brief Contract test for the optional angularDofMask_ deck key and its setup-time guard.
 *
 *  Json contract (src/config/SimulationConfig.cpp):
 *    - absent            -> (1,1,1), hasAngularDofMask() == false
 *    - [0,1,0]           -> (0,1,0), hasAngularDofMask() == true
 *    - [0,1], [0,0.5,1], [2,1,1], non-array -> std::invalid_argument naming the key
 *  Setup guard (pe/interface/sim_setup_serial_features.h, called from setupDNSDragSerial):
 *    - key present and particleMotion_ = "fixed" -> std::invalid_argument
 *      "angularDofMask_ requires particleMotion_ = rotationOnly or free"
 *    - key present and rotationOnly (or free), or key absent under any motion -> accepted
 *    (translationOnly and free are pinned by pe_particle_motion_test.cpp)
 *
 *  SimulationConfig is a process singleton and loadFromFile() only overwrites keys that are
 *  present, so the cases run in an order that never needs a reset: the "absent" case goes
 *  first while the instance is still pristine. Decks are written under ./_angmask_config_test/.
 *  No world/MPI setup - mirrors the harness class of pe_lubrication_refusal_test.
 */
//=================================================================================================

#include <pe/config/SimulationConfig.h>
#include <pe/interface/sim_setup_serial_features.h>

#include <boost/filesystem.hpp>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace pe;

namespace {

int failures = 0;

void expect(bool ok, const char* what) {
  if (!ok) {
    std::cerr << "FAILED: " << what << "\n";
    ++failures;
  }
}

bool same(const Vec3& a, const Vec3& b) {
  return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
}

const boost::filesystem::path deckDir("_angmask_config_test");

std::string writeDeck(const std::string& name, const std::string& body) {
  const boost::filesystem::path p = deckDir / (name + ".json");
  std::ofstream out(p.string());
  out << body;
  return p.string();
}

// Returns the invalid_argument message if loadFromFile threw one, "" if it did not throw.
std::string loadThrows(const std::string& file) {
  try {
    SimulationConfig::loadFromFile(file);
  } catch (const std::invalid_argument& e) {
    return e.what();
  }
  return "";
}

// Returns the invalid_argument message if the guard threw one, "" if it did not throw.
std::string guardThrows(const SimulationConfig& config) {
  try {
    checkAngularDofMaskRequiresRotationOnly(config);
  } catch (const std::invalid_argument& e) {
    return e.what();
  }
  return "";
}

bool mentionsKey(const std::string& msg) {
  return msg.find("angularDofMask_") != std::string::npos;
}

}  // namespace

int main() {
  boost::filesystem::create_directories(deckDir);
  SimulationConfig& config = SimulationConfig::getInstance();

  // --- 1. absent key: default (1,1,1), not flagged as set; guard accepts under "fixed" -------
  expect(same(config.getAngularDofMask(), Vec3(1, 1, 1)), "pristine default is (1,1,1)");
  expect(!config.hasAngularDofMask(), "pristine default: hasAngularDofMask() == false");

  expect(loadThrows(writeDeck("absent", "{ \"particleMotion_\": \"fixed\" }")).empty(),
         "deck without angularDofMask_ loads");
  expect(same(config.getAngularDofMask(), Vec3(1, 1, 1)), "absent key -> (1,1,1)");
  expect(!config.hasAngularDofMask(), "absent key -> hasAngularDofMask() == false");
  expect(config.getParticleMotion() == "fixed", "particleMotion_ = fixed was read");
  expect(guardThrows(config).empty(), "guard accepts: key absent, motion fixed");

  // --- 2. malformed keys throw std::invalid_argument naming the key, state untouched ---------
  {
    const std::string m = loadThrows(writeDeck("two", "{ \"angularDofMask_\": [0, 1] }"));
    expect(!m.empty() && mentionsKey(m), "[0,1] throws invalid_argument naming angularDofMask_");
  }
  {
    const std::string m = loadThrows(writeDeck("half", "{ \"angularDofMask_\": [0, 0.5, 1] }"));
    expect(!m.empty() && mentionsKey(m), "[0,0.5,1] throws invalid_argument naming angularDofMask_");
  }
  {
    const std::string m = loadThrows(writeDeck("two_one_one", "{ \"angularDofMask_\": [2, 1, 1] }"));
    expect(!m.empty() && mentionsKey(m), "[2,1,1] throws invalid_argument naming angularDofMask_");
  }
  {
    const std::string m = loadThrows(writeDeck("scalar", "{ \"angularDofMask_\": 1 }"));
    expect(!m.empty() && mentionsKey(m), "non-array throws invalid_argument naming angularDofMask_");
  }
  {
    const std::string m = loadThrows(writeDeck("strings", "{ \"angularDofMask_\": [\"0\", 1, 0] }"));
    expect(!m.empty() && mentionsKey(m), "non-numeric entry throws invalid_argument naming angularDofMask_");
  }
  expect(same(config.getAngularDofMask(), Vec3(1, 1, 1)), "rejected decks leave the mask at (1,1,1)");
  expect(!config.hasAngularDofMask(), "rejected decks leave hasAngularDofMask() == false");

  // --- 3. well-formed key under rotationOnly: parsed, flagged, guard accepts -----------------
  expect(loadThrows(writeDeck("ok", "{ \"particleMotion_\": \"rotationOnly\", "
                                     "\"angularDofMask_\": [0, 1, 0] }")).empty(),
         "[0,1,0] under rotationOnly loads");
  expect(same(config.getAngularDofMask(), Vec3(0, 1, 0)), "[0,1,0] -> (0,1,0)");
  expect(config.hasAngularDofMask(), "[0,1,0] -> hasAngularDofMask() == true");
  expect(guardThrows(config).empty(), "guard accepts: key present, motion rotationOnly");

  // Integer and floating spellings of 0/1 are both exact.
  expect(loadThrows(writeDeck("ok_float", "{ \"angularDofMask_\": [1.0, 0.0, 1.0] }")).empty(),
         "[1.0,0.0,1.0] loads");
  expect(same(config.getAngularDofMask(), Vec3(1, 0, 1)), "[1.0,0.0,1.0] -> (1,0,1)");

  // --- 4. guard: key present but particleMotion_ = fixed is refused --------------------------
  expect(loadThrows(writeDeck("fixed_with_mask", "{ \"particleMotion_\": \"fixed\", "
                                                  "\"angularDofMask_\": [0, 1, 0] }")).empty(),
         "parsing itself does not refuse angularDofMask_ under fixed (that is the setup guard's job)");
  {
    const std::string m = guardThrows(config);
    expect(m == "angularDofMask_ requires particleMotion_ = rotationOnly or free",
           "guard refuses: key present, motion fixed (exact message)");
  }
  // Same state via the setters (the path setupDNSDragSerial sees after loadFromFile).
  config.setParticleMotion("rotationOnly");
  expect(guardThrows(config).empty(), "guard accepts after switching motion to rotationOnly");
  config.setParticleMotion("fixed");
  expect(!guardThrows(config).empty(), "guard refuses again after switching motion back to fixed");

  if (failures == 0) {
    std::cout << "pe-angular-dof-mask-config: all checks passed\n";
    return EXIT_SUCCESS;
  }
  std::cerr << "pe-angular-dof-mask-config: " << failures << " check(s) failed\n";
  return EXIT_FAILURE;
}
