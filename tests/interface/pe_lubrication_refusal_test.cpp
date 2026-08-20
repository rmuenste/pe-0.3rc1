// REFUSAL GUARD: lubricationEnabled_ must never silently do nothing.
//
// Setting the json switch under a solver that does not invoke the lubrication stage used
// to be a no-op that still looked like a working configuration: AABBs changed, detection
// tagged pre-contact pairs, and not one newton of lubrication force was ever applied.
// D2.2 §3.5 requires that to be a hard error. This test is the only thing standing
// between that requirement and a silent regression.
//
// Unlike the other lubrication tests, this one links the SHIPPED library (pe_static) with
// whatever pe_CONSTRAINT_SOLVER the tree is configured for -- it is deliberately not given
// a purpose-built stage-capable variant, because the configuration it has to police is the
// default one a user actually gets. It is currently also the only test that exercises the
// shipped default config at all.
//
// The assertion is therefore on the guard's CONTRACT rather than on a hard-coded outcome:
//
//     applyOptionalLubricationParams throws  <=>  lubricationEnabled_ && !stage-capable
//
// which is meaningful under both a neutral default (throws) and a stage-capable one
// (accepts), instead of silently degenerating into a tautology in one of them.

#include <pe/core.h>
#include <pe/config/SimulationConfig.h>
#include <pe/core/lubrication/Params.h>
#include <pe/interface/setup_optional_collision_params.h>

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <type_traits>

using namespace pe;

namespace {

int failures = 0;

void expect(bool ok, const char* what) {
  if (!ok) {
    std::cerr << "FAILED: " << what << "\n";
    ++failures;
  }
}

// Returns true if the call threw.
template <typename CS>
bool throwsFor(CS& cs, SimulationConfig& config, bool enabled) {
  config.setLubricationEnabled(enabled);
  try {
    applyOptionalLubricationParams(cs, config);
    return false;
  } catch (const std::runtime_error&) {
    return true;
  }
}

}  // namespace

int main() {
  auto& cs = *theCollisionSystem();
  using CS = std::remove_reference<decltype(cs)>::type;
  constexpr bool stageCapable = HasLubricationStage<CS>::value;

  SimulationConfig& config = SimulationConfig::getInstance();

  std::cout << "active pe_CONSTRAINT_SOLVER is stage-capable: "
            << (stageCapable ? "yes" : "no") << "\n";

  // Shipped default must be OFF, or the guard would fire on an untouched deck.
  expect(config.getLubricationEnabled() == false,
         "shipped SimulationConfig default: lubricationEnabled_ is false");

  // Disabled: must ALWAYS be accepted, whatever the solver.
  expect(!throwsFor(cs, config, false),
         "lubricationEnabled_ = false is accepted under any solver");
  expect(lubrication::isEnabled() == false,
         "lubricationEnabled_ = false leaves the master switch off");

  // Enabled: accepted iff the active solver actually runs the stage.
  const bool threw = throwsFor(cs, config, true);
  expect(threw == !stageCapable,
         "lubricationEnabled_ = true throws exactly when the solver cannot honor it");

  if (stageCapable) {
    std::cout << "  enabled=true -> accepted (solver runs the stage)\n";
    expect(lubrication::isEnabled() == true,
           "stage-capable solver: the switch actually arms the add-on");
  } else {
    std::cout << "  enabled=true -> REFUSED (solver does not run the stage)\n";
    // A refusal must be a refusal: the store must not be left half-armed, or the
    // caller could swallow the exception and still get the AABB/detection side
    // effects without any force.
    expect(lubrication::isEnabled() == false,
           "refusal leaves the master switch OFF, not half-armed");
    expect(lubrication::aabbPadding(real(0.0015)) == real(0),
           "refusal leaves AABB padding at zero");
  }

  // Restore the default so nothing downstream inherits an armed store.
  config.setLubricationEnabled(false);

  if (failures == 0) {
    std::cout << "pe-lubrication-refusal: all checks passed\n";
    return EXIT_SUCCESS;
  }
  std::cerr << "pe-lubrication-refusal: " << failures << " check(s) failed\n";
  return EXIT_FAILURE;
}
