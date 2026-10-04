//=================================================================================================
/*!
 *  \file tools/contact_viewer/SimControls.h
 *  \brief Simulation controls shared by the contact viewer modes
 *
 *  Run / pause / step state, the time step, the solver knobs, gravity (applied as a force), the
 *  mouse spring (Ctrl + left-drag) and the keyboard shortcuts. Both modes drive the same engine
 *  singleton, so there is one set of these settings; each mode keeps its own Clock.
 */
//=================================================================================================

#ifndef _PE_TOOLS_CONTACT_VIEWER_SIM_CONTROLS_H_
#define _PE_TOOLS_CONTACT_VIEWER_SIM_CONTROLS_H_

#include <string>
#include <vector>

#include <pe/core.h>

#include "glm/glm.hpp"
#include "polyscope/structure.h"

namespace simctl {

//! Settings shared by both modes.
struct Controls {
   bool   running       = false;
   int    queuedSteps   = 0;       //!< Step requests, consumed by takeFrameSteps().
   int    stepsPerClick = 1;
   int    stepsPerFrame = 4;
   double dt            = 2.0e-3;
   //! Gravity is applied by the viewer as a force m g on every dynamic body before each step:
   //! the Euler-Lagrange solver (the repo default) leaves body forces such as gravity to the
   //! outer driver and ignores World::setGravity(). The world gravity stays 0 so that solvers
   //! that do honour it do not apply it twice.
   double gravityZ      = -9.81;
   // Solver knobs. HardContactEulerLagrange has setters but no getters: the values start at the
   // engine's constructor defaults and are pushed by applySolverKnobs().
   double erp             = 0.7;
   int    maxIterations   = 100;
   double relaxationParam = 0.9;
   int    relaxationModel = 1;   //!< ApproximateInelasticCoulombContactByDecoupling
   bool   splitImpulse    = false;   //!< Position correction by pseudo velocities instead of the Baumgarte term
   // Mouse spring: angular frequency [1/s] and damping ratio.
   double springOmega     = 20.0;
   double springZeta      = 1.0;
};

Controls& controls();

//! Simulated time of one mode's world.
struct Clock {
   double      time  = 0.0;
   long        steps = 0;
   std::string error;   //!< Set on an exception or a non-finite position; stops stepping.

   void reset() { time = 0.0; steps = 0; error.clear(); }
};

//! Pushes the solver knobs to the collision system.
void applySolverKnobs();

//! Run / Pause / Step / Reset buttons, step counts and the dt widgets, drawn into the current
//! ImGui window. Returns true if Reset was clicked.
bool drawStepControls();

//! Gravity and solver knobs as a collapsing header in the current ImGui window.
void drawSolverControls();

//! Keyboard shortcuts (space: run / pause, n: step, r: reset). Returns true on r.
bool processKeys();

//! Number of steps to run this frame (stepsPerFrame while running, plus queued steps); consumes
//! the queued steps.
int takeFrameSteps();

//! Advances the world by \a n steps: gravity on every non-fixed body of \a bodies, the mouse
//! spring, World::simulationStep(). Exceptions and non-finite body positions are reported in
//! \a clock.error and stop the run; nothing is stepped while an error is pending.
void advance( int n, const std::vector<pe::BodyID>& bodies, Clock& clock );

//! Ctrl + left-drag on a body: a mass-scaled damped spring F = m ( w^2 (target - x) - 2 zeta w v )
//! pulls its center of mass towards an anchor that follows the mouse on a camera-facing plane.
class MouseSpring {
 public:
   struct Target {
      polyscope::Structure* mesh;
      pe::BodyID            body;
   };

   //! Per frame, before stepping: starts a drag on a Ctrl + click on a non-fixed target, moves the
   //! anchor while the button is held, ends it on release. \a lineRadius is the drawn spring's.
   void process( const std::vector<Target>& targets, double lineRadius );

   //! Adds the spring force to the dragged body; called before every step by advance().
   void apply() const;

   void end();

   pe::BodyID body() const { return body_; }

 private:
   pe::BodyID body_   = nullptr;
   glm::vec3  target_ = glm::vec3( 0.0f );
   glm::vec3  planeN_ = glm::vec3( 0.0f );
};

MouseSpring& mouseSpring();

} // namespace simctl

#endif
