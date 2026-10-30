//=================================================================================================
/*!
 *  \file tests/interface/pe_box_walls_config_test.cpp
 *  \brief Contract test for the optional boxWallsEnabled_ deck key (EL terminal-velocity setup).
 *
 *  Json contract (src/config/SimulationConfig.cpp):
 *    - absent            -> getBoxWallsEnabled() == false (existing EL decks unchanged)
 *    - true / false      -> the value
 *    - a later deck without the key leaves the previous value (loadFromFile only overwrites
 *      keys that are present, SimulationConfig is a process singleton)
 *  The setup-time refusal (boxWallsEnabled_ together with periodicX_/Y_/Z_ or zWallsEnabled_
 *  aborts in setupELTerminalVelocity, pe/interface/setup/el_terminal_velocity.h) needs the MPI
 *  setup and is covered by the FeatFloWer twins, not here.
 *
 *  No world/MPI setup - mirrors pe_angular_dof_mask_config_test. Decks are written under
 *  ./_box_walls_config_test/.
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

const boost::filesystem::path deckDir("_box_walls_config_test");

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

  // --- 1. pristine default and a deck without the key ----------------------------------------
  expect(!config.getBoxWallsEnabled(), "pristine default is false");
  expect(loadThrows(writeDeck("absent", "{ \"zWallsEnabled_\": false }")).empty(),
         "deck without boxWallsEnabled_ loads");
  expect(!config.getBoxWallsEnabled(), "absent key -> false");

  // --- 2. explicit values ---------------------------------------------------------------------
  expect(loadThrows(writeDeck("on", "{ \"boxWallsEnabled_\": true }")).empty(),
         "boxWallsEnabled_ = true loads");
  expect(config.getBoxWallsEnabled(), "true -> getBoxWallsEnabled() == true");

  expect(loadThrows(writeDeck("absent_again", "{ \"zWallsEnabled_\": false }")).empty(),
         "a later deck without the key loads");
  expect(config.getBoxWallsEnabled(), "absent key leaves the previous value (true)");

  expect(loadThrows(writeDeck("off", "{ \"boxWallsEnabled_\": false }")).empty(),
         "boxWallsEnabled_ = false loads");
  expect(!config.getBoxWallsEnabled(), "false -> getBoxWallsEnabled() == false");

  // --- 3. setter mirrors the key --------------------------------------------------------------
  config.setBoxWallsEnabled(true);
  expect(config.getBoxWallsEnabled(), "setBoxWallsEnabled(true)");
  config.setBoxWallsEnabled(false);
  expect(!config.getBoxWallsEnabled(), "setBoxWallsEnabled(false)");

  if (failures == 0) {
    std::cout << "pe_box_walls_config_test: all checks passed\n";
    return 0;
  }
  std::cerr << "pe_box_walls_config_test: " << failures << " check(s) failed\n";
  return 1;
}
