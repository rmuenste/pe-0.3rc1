//=================================================================================================
/*!
 *  \file tests/interface/pe_cylinder_wall_config_test.cpp
 *  \brief Contract test for the optional cylinderWallEnabled_ / cylinderWallRadius_ deck keys
 *         (EL terminal-velocity setup: fixed inner cylinder as the particle-side container wall).
 *
 *  Json contract (src/config/SimulationConfig.cpp):
 *    - absent            -> getCylinderWallEnabled() == false, radius negative (existing decks unchanged)
 *    - present           -> the values
 *    - a later deck without the keys leaves the previous values (loadFromFile only overwrites
 *      keys that are present, SimulationConfig is a process singleton)
 *  The setup-time refusals (together with boxWallsEnabled_ or periodicX_/Y_, an axis other than z,
 *  a non-positive radius) abort in setupELTerminalVelocity and need the MPI setup; they are
 *  covered by the FeatFloWer runs, not here.
 *
 *  No world/MPI setup - mirrors pe_angular_dof_mask_config_test. Decks are written under
 *  ./_cylinder_wall_config_test/.
 */
//=================================================================================================

#include <pe/config/SimulationConfig.h>

#include <boost/filesystem.hpp>

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

const boost::filesystem::path deckDir("_cylinder_wall_config_test");

std::string writeDeck(const std::string& name, const std::string& body) {
  const boost::filesystem::path p = deckDir / (name + ".json");
  std::ofstream out(p.string());
  out << body;
  return p.string();
}

// Returns the exception message if loadFromFile threw, "" if it did not throw.
std::string loadThrows(const std::string& file) {
  try {
    SimulationConfig::loadFromFile(file);
  } catch (const std::exception& e) {
    return e.what();
  }
  return "";
}

}  // namespace

int main() {
  boost::filesystem::create_directories(deckDir);
  SimulationConfig& config = SimulationConfig::getInstance();

  // --- 1. pristine defaults and a deck without the keys ---------------------------------------
  expect(!config.getCylinderWallEnabled(), "pristine default is false");
  expect(config.getCylinderWallRadius() < real(0), "pristine radius is negative (unset)");
  expect(loadThrows(writeDeck("absent", "{ \"zWallsEnabled_\": false }")).empty(),
         "deck without the keys loads");
  expect(!config.getCylinderWallEnabled(), "absent key -> false");

  // --- 2. explicit values ---------------------------------------------------------------------
  expect(loadThrows(writeDeck("on", "{ \"cylinderWallEnabled_\": true, \"cylinderWallRadius_\": 0.01385 }")).empty(),
         "enabled deck loads");
  expect(config.getCylinderWallEnabled(), "true -> getCylinderWallEnabled() == true");
  expect(config.getCylinderWallRadius() == real(0.01385), "radius read");

  expect(loadThrows(writeDeck("absent_again", "{ \"zWallsEnabled_\": false }")).empty(),
         "a later deck without the keys loads");
  expect(config.getCylinderWallEnabled(), "absent key leaves the previous value (true)");
  expect(config.getCylinderWallRadius() == real(0.01385), "absent key leaves the previous radius");

  expect(loadThrows(writeDeck("off", "{ \"cylinderWallEnabled_\": false }")).empty(),
         "cylinderWallEnabled_ = false loads");
  expect(!config.getCylinderWallEnabled(), "false -> getCylinderWallEnabled() == false");

  // --- 3. setters mirror the keys -------------------------------------------------------------
  config.setCylinderWallEnabled(true);
  expect(config.getCylinderWallEnabled(), "setCylinderWallEnabled(true)");
  config.setCylinderWallRadius(real(0.5));
  expect(config.getCylinderWallRadius() == real(0.5), "setCylinderWallRadius(0.5)");
  config.setCylinderWallEnabled(false);

  if (failures == 0) {
    std::cout << "pe_cylinder_wall_config_test: all checks passed\n";
    return 0;
  }
  std::cerr << "pe_cylinder_wall_config_test: " << failures << " check(s) failed\n";
  return 1;
}
