//=================================================================================================
/*!
 *  \file tests/interface/pe_el_restitution_config_test.cpp
 *  \brief Json contract of the restitution switch (restitutionEnabled_, restitutionThreshold_,
 *         restitutionScheme_, restitutionIterations_), its application to the collision system
 *         (applyOptionalRestitutionParams()) and the EL_CONTACT_PARAMS start-up echo line.
 *
 *  Json contract (src/config/SimulationConfig.cpp):
 *    - absent  -> off, 0, "post-pass", 4 (the solver defaults: existing decks unchanged)
 *    - present -> the values; invalid values (threshold < 0, unknown scheme, iterations < 1) throw
 *      std::invalid_argument and leave the previous value
 *    - a later deck without the keys leaves the previous values (SimulationConfig is a process
 *      singleton and loadFromFile only overwrites keys that are present)
 *  Applier: the collision system's switch follows the config (with the solver's switch present);
 *  with restitutionEnabled_ true and no switch it throws (checked only under such a solver).
 *  Echo: the exact line for json friction 0.15 / restitution 0.9 (pair values mu = 0.3, e = 0.9),
 *  with walls and without, switch on and off.
 *
 *  No world stepping, no MPI. Decks are written under ./_el_restitution_config_test/.
 */
//=================================================================================================

#include <pe/core.h>
#include <pe/config/SimulationConfig.h>
#include <pe/interface/support/contact_params_echo.h>
#include <pe/interface/support/optional_collision_params.h>

#include <boost/filesystem.hpp>

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <type_traits>

using namespace pe;

namespace {

int failures = 0;

void expect(bool ok, const std::string& what) {
  if (!ok) {
    std::cerr << "FAILED: " << what << "\n";
    ++failures;
  }
}

const boost::filesystem::path deckDir("_el_restitution_config_test");

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

typedef std::remove_reference<decltype(*theCollisionSystem())>::type CollisionSystemType;

// Applier and echo against the configured collision system (template: the branch for a solver
// without the switch must not be compiled against one with it, and vice versa).
template <typename CS>
void checkApplierAndEcho(CS& cs, SimulationConfig& config, MaterialID particle, const ContactEchoWalls& walls) {
  if constexpr (HasRestitutionSwitch<CS>::value) {
    applyOptionalRestitutionParams(cs, config);
    expect(cs.getRestitution() && !cs.getRestitutionPostPass() && cs.getRestitutionIterations() == 6 &&
           cs.getRestitutionThreshold() == real(0.001), "applier: the solver switch follows the config");
    const std::string on = elContactParamsLine(cs, particle, walls);
    std::cout << on << "\n";
    expect(on == "EL_CONTACT_PARAMS pair=particle-particle mu_static=0.3 mu_dynamic=0.3 e=0.9"
                 " | pair=particle-wall wall=zwalls mu_static=0.3 mu_dynamic=0.3 e=0.9"
                 " | restitution=on threshold=0.001 scheme=in-solve iterations=6",
           "echo line with the switch on: " + on);

    // back to the defaults: the deck keys off
    config.setRestitutionEnabled(false);
    config.setRestitutionThreshold(real(0));
    config.setRestitutionScheme("post-pass");
    config.setRestitutionIterations(4);
    applyOptionalRestitutionParams(cs, config);
    expect(!cs.getRestitution() && cs.getRestitutionPostPass() && cs.getRestitutionIterations() == 4 &&
           cs.getRestitutionThreshold() == real(0), "applier: defaults restore the solver defaults");
    const std::string off = elContactParamsLine(cs, particle, ContactEchoWalls());
    std::cout << off << "\n";
    expect(off == "EL_CONTACT_PARAMS pair=particle-particle mu_static=0.3 mu_dynamic=0.3 e=0.9"
                  " | pair=particle-wall wall=none | restitution=off threshold=0 scheme=post-pass iterations=4",
           "echo line with the switch off and no walls: " + off);
  } else {
    // A solver without the switch: refused when the deck asks for restitution, unsupported echo
    bool threw = false;
    try { applyOptionalRestitutionParams(cs, config); } catch (const std::runtime_error&) { threw = true; }
    expect(threw, "applier refuses restitutionEnabled_ under a solver without the switch");
    const std::string line = elContactParamsLine(cs, particle, walls);
    std::cout << line << "\n";
    expect(line.find("restitution=unsupported") != std::string::npos, "echo marks the switch unsupported");
  }

}

}  // namespace

int main() {
  boost::filesystem::create_directories(deckDir);
  SimulationConfig& config = SimulationConfig::getInstance();

  // --- 1. pristine defaults and a deck without the keys ---------------------------------------
  expect(!config.getRestitutionEnabled(), "pristine restitutionEnabled_ is false");
  expect(config.getRestitutionThreshold() == real(0), "pristine threshold is 0");
  expect(config.getRestitutionScheme() == "post-pass", "pristine scheme is post-pass");
  expect(config.getRestitutionIterations() == 4, "pristine iterations is 4");
  expect(loadThrows(writeDeck("absent", "{ \"restitution_\": 0.9 }")).empty(), "deck without the keys loads");
  expect(!config.getRestitutionEnabled(), "absent key -> false (restitution_ alone does not switch it on)");

  // --- 2. explicit values ---------------------------------------------------------------------
  expect(loadThrows(writeDeck("on", "{ \"restitutionEnabled_\": true, \"restitutionThreshold_\": 0.001,"
                                    " \"restitutionScheme_\": \"in-solve\", \"restitutionIterations_\": 6 }")).empty(),
         "deck with the keys loads");
  expect(config.getRestitutionEnabled(), "restitutionEnabled_ true read");
  expect(config.getRestitutionThreshold() == real(0.001), "restitutionThreshold_ read");
  expect(config.getRestitutionScheme() == "in-solve", "restitutionScheme_ read");
  expect(config.getRestitutionIterations() == 6, "restitutionIterations_ read");

  // A later deck without the keys: the four fields are resolved per deck, absent = the defaults
  // (a deck must never inherit an enabled switch from an earlier deck of the same process).
  expect(loadThrows(writeDeck("absent_again", "{ \"zWallsEnabled_\": true }")).empty(), "a later deck without the keys loads");
  expect(!config.getRestitutionEnabled() && config.getRestitutionThreshold() == real(0) &&
         config.getRestitutionScheme() == "post-pass" && config.getRestitutionIterations() == 4,
         "absent keys in a later deck -> off, 0, post-pass, 4 (not the previous deck's values)");

  // --- 3. validation (all four validated together: a refused deck changes none of them) -------
  const std::string onDeck("{ \"restitutionEnabled_\": true, \"restitutionThreshold_\": 0.001,"
                           " \"restitutionScheme_\": \"in-solve\", \"restitutionIterations_\": 6 }");
  const auto backToOn = [&]() { return loadThrows(writeDeck("on_again", onDeck)).empty(); };
  const auto stillOn = [&]() {
    return config.getRestitutionEnabled() && config.getRestitutionThreshold() == real(0.001) &&
           config.getRestitutionScheme() == "in-solve" && config.getRestitutionIterations() == 6;
  };
  struct Bad { const char* name; const char* body; };
  const Bad bad[] = {
    { "bad_threshold",  "{ \"restitutionEnabled_\": false, \"restitutionThreshold_\": -0.1 }" },
    { "bad_scheme",     "{ \"restitutionEnabled_\": false, \"restitutionScheme_\": \"postpass\" }" },
    { "bad_iterations", "{ \"restitutionEnabled_\": false, \"restitutionIterations_\": 0 }" },
  };
  for (const Bad& b : bad) {
    expect(backToOn(), std::string("reload of the enabled deck before ") + b.name);
    const std::string msg = loadThrows(writeDeck(b.name, b.body));
    expect(!msg.empty(), std::string(b.name) + " throws");
    expect(stillOn(), std::string(b.name) + ": the refused deck leaves all four values unchanged");
  }
  expect(backToOn(), "reload of the enabled deck before the applier");

  // --- 4. applier and echo --------------------------------------------------------------------
  const MaterialID particle = createMaterial("el_rc_particle", real(860), real(0.9), real(0.15), real(0.15),
                                             real(0.2), real(80), real(100), real(10), real(11));
  const MaterialID wall = createMaterial("el_rc_wall", real(1), real(0.9), real(0.15), real(0.15),
                                         real(0.2), real(80), real(100), real(10), real(11));
  ContactEchoWalls walls;
  walls.push_back(std::make_pair(std::string("zwalls"), wall));

  checkApplierAndEcho(*theCollisionSystem(), config, particle, walls);

  if (failures == 0) {
    std::cout << "pe_el_restitution_config_test: all checks passed\n";
    return 0;
  }
  std::cerr << "pe_el_restitution_config_test: " << failures << " check(s) failed\n";
  return 1;
}
