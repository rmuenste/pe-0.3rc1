//=================================================================================================
/*!\file pe/interface/setup/serial_common.h
 *  \brief Prelude of every serial-mode (PE_SERIAL_MODE) case setup.
 *
 *  Pulls in the engine, output and configuration headers a serial setupXxxSerial(int) is
 *  written against, plus the checkpoint-resume helpers the resumable setups share. Each
 *  CFD rank runs its own complete PE instance in this mode, so a setup creates ALL bodies on
 *  every rank and restricts output to the representative rank (cfdRank == 1).
 */
//=================================================================================================
#ifndef _PE_INTERFACE_SETUP_SERIAL_COMMON_H_
#define _PE_INTERFACE_SETUP_SERIAL_COMMON_H_

#include <pe/core.h>
#include <pe/core/rigidbody/Sphere.h>
#include <pe/core/rigidbody/Box.h>
#include <pe/core/rigidbody/Capsule.h>
#include <pe/core/rigidbody/TriangleMesh.h>
#include <pe/core/rigidbody/Plane.h>
#include <pe/core/TimeStep.h>
#include <pe/core/CollisionSystemID.h>
#include <pe/core/Materials.h>
#include <pe/vtk/Writer.h>
#include <pe/vtk/DistanceMapWriter.h>
#include <pe/util/logging/Logger.h>
#include <pe/config/SimulationConfig.h>
#include <pe/util/Checkpointer.h>
#include <pe/interface/support/geometry_utils.h>
#include <pe/interface/support/optional_collision_params.h>
#include <pe/interface/support/serial_step_features.h>
#include <pe/interface/support/el_optional_api.h>
#include <pe/interface/fbm_gate_probe.h>
#include <string>
#include <type_traits>
#include <utility>
#include <deque>
#include <map>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>
#include <sys/types.h>
#include <iostream>

namespace pe {

inline unsigned int effectiveCheckpointSpacing(const SimulationConfig& config) {
  // Trigger::triggerAll() runs per substep; scale spacing to keep main-step cadence.
  return config.getPointerspacing() * config.getSubsteps();
}

/**
 * @brief Translates the deck's resume expectation keys into a ResumeExpectation.
 *
 * Every key is opt-in, so a deck that does not mention them yields an empty expectation and the
 * resume behaves exactly as before this guard existed.
 */
inline ResumeExpectation resumeExpectationFromConfig(const SimulationConfig& config) {
  ResumeExpectation expectation;

  if (config.getResumeExpectedTimeSet()) {
    expectation.hasTime = true;
    expectation.time = config.getResumeExpectedTime();
  }
  expectation.toleranceSteps = config.getResumeTimeToleranceSteps();

  if (config.getResumeExpectedStep() >= 0) {
    expectation.hasStep = true;
    expectation.step = static_cast<uint64_t>(config.getResumeExpectedStep());
  }

  if (!config.getResumeExpectedTag().empty()) {
    expectation.hasTag = true;
    expectation.tag = config.getResumeExpectedTag();
  }

  return expectation;
}

/**
 * @brief Loads the configured resume checkpoint and enforces the deck's expectations on it.
 *
 * This is the single resume entry point for the serial setups: it restores the checkpoint's
 * material table before instantiating bodies, rejects a truncated or mismatched checkpoint, and
 * refuses to continue when the driver's expected time/step/tag does not match what the
 * checkpoint carries -- the sibling of the pe/CFD step size mismatch guard.
 *
 * @return The resumed checkpoint's metadata, so the setup can report it.
 */
inline CheckpointMetadata resumeFromConfiguredCheckpoint(const SimulationConfig& config) {
  const CheckpointMetadata metadata =
      readCheckpoint(config.getCheckpointPath(), config.getResumeCheckpointFile());
  validateResumeExpectation(metadata, resumeExpectationFromConfig(config),
                            config.getResumeCheckpointFile());
  return metadata;
}

}  // namespace pe

#endif  // _PE_INTERFACE_SETUP_SERIAL_COMMON_H_
