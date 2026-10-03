//=================================================================================================
/*!
 *  \file tools/contact_viewer/sim_controls.cpp
 *  \brief Simulation controls shared by the contact viewer modes
 */
//=================================================================================================

#include <pe/system/WarningDisable.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <type_traits>

#include <pe/core.h>

#include "polyscope/curve_network.h"
#include "polyscope/pick.h"
#include "polyscope/polyscope.h"

#include "ShapeMeshes.h"
#include "SimControls.h"

using namespace pe;


namespace {

using CollisionSystemType = std::remove_reference<decltype( *theCollisionSystem() )>::type;

const char* const kRelaxationModels[] = {
   "inelastic frictionless", "approx. Coulomb (decoupling)", "approx. Coulomb (orth. projections)",
   "Coulomb (decoupling)", "Coulomb (orth. projections)", "generalized max. dissipation" };

} // namespace


namespace simctl {

Controls& controls()
{
   static Controls c;
   return c;
}


MouseSpring& mouseSpring()
{
   static MouseSpring s;
   return s;
}


void applySolverKnobs()
{
   const Controls& c = controls();
   CollisionSystemID cs = theCollisionSystem();
   cs->setErrorReductionParameter( static_cast<real>( c.erp ) );
   cs->setMaxIterations( static_cast<size_t>( std::max( 1, c.maxIterations ) ) );
   cs->setRelaxationParameter( static_cast<real>( c.relaxationParam ) );
   cs->setRelaxationModel( static_cast<typename CollisionSystemType::RelaxationModel>( c.relaxationModel ) );
   cs->setSplitImpulse( c.splitImpulse );
}


bool drawStepControls()
{
   Controls& c = controls();
   bool reset = false;

   if( ImGui::Button( c.running ? "Pause" : "Run", ImVec2( 70, 0 ) ) )
      c.running = !c.running;
   ImGui::SameLine();
   if( ImGui::Button( "Step" ) )
      c.queuedSteps += std::max( 1, c.stepsPerClick );
   ImGui::SameLine();
   if( ImGui::Button( "Reset" ) )
      reset = true;
   ImGui::SameLine();
   ImGui::TextDisabled( "(space / n / r)" );

   ImGui::SetNextItemWidth( 120 );
   ImGui::InputInt( "steps per Step click", &c.stepsPerClick );
   c.stepsPerClick = std::max( 1, c.stepsPerClick );
   ImGui::SliderInt( "steps / frame", &c.stepsPerFrame, 1, 64 );
   // Time step: live (applies to the next step, also mid-run). Log slider for coarse changes,
   // halve/double for bisecting a stability limit, presets for the common values; Ctrl+click
   // the slider to type an exact value.
   const double dtLo = 1.0e-5, dtHi = 5.0e-2;
   ImGui::SliderScalar( "dt [s]", ImGuiDataType_Double, &c.dt, &dtLo, &dtHi, "%.3e", ImGuiSliderFlags_Logarithmic );
   if( ImGui::IsItemHovered( ImGuiHoveredFlags_DelayShort ) )
      ImGui::SetTooltip( "Takes effect on the next step, also while running. Ctrl+click to type a value." );
   if( ImGui::Button( "dt / 2" ) ) c.dt *= 0.5;
   ImGui::SameLine();
   if( ImGui::Button( "dt x 2" ) ) c.dt *= 2.0;
   const double presetsDt[] = { 1.0e-4, 5.0e-4, 1.0e-3, 2.0e-3, 5.0e-3, 1.0e-2 };
   for( double p : presetsDt ) {
      char label[16];
      std::snprintf( label, sizeof( label ), "%g", p );
      ImGui::SameLine();
      if( ImGui::Button( label ) ) c.dt = p;
   }
   c.dt = std::max( dtLo, std::min( c.dt, dtHi ) );
   ImGui::TextDisabled( "simulated time per frame: %.3e s (%d x dt)", c.stepsPerFrame * c.dt, c.stepsPerFrame );

   return reset;
}


void drawSolverControls()
{
   Controls& c = controls();
   if( !ImGui::CollapsingHeader( "World / solver (live)", ImGuiTreeNodeFlags_DefaultOpen ) )
      return;

   const double gLo = -30.0, gHi = 0.0;
   ImGui::SliderScalar( "gravity z", ImGuiDataType_Double, &c.gravityZ, &gLo, &gHi, "%.3f" );
   if( ImGui::IsItemHovered( ImGuiHoveredFlags_DelayShort ) )
      ImGui::SetTooltip( "Applied by the viewer as a force m g per step." );

   bool changed = false;
   const double zero = 0.0, one = 1.0;
   changed |= ImGui::SliderScalar( "error reduction", ImGuiDataType_Double, &c.erp, &zero, &one, "%.3f" );
   changed |= ImGui::SliderInt( "max iterations", &c.maxIterations, 1, 2000, "%d", ImGuiSliderFlags_Logarithmic );
   changed |= ImGui::SliderScalar( "relaxation", ImGuiDataType_Double, &c.relaxationParam, &zero, &one, "%.3f" );
   changed |= ImGui::Combo( "friction model", &c.relaxationModel, kRelaxationModels, 6 );
   changed |= ImGui::Checkbox( "split impulse (position correction)", &c.splitImpulse );
   if( ImGui::IsItemHovered( ImGuiHoveredFlags_DelayShort ) )
      ImGui::SetTooltip( "Penetration is removed by pseudo velocities that move the positions but are not kept as body\n"
                         "velocity. Off: the Baumgarte term, erp x depth / dt of separation velocity that stays in the\n"
                         "body (an overlapping start launches it, a landing body bounces at restitution 0)." );
   if( changed )
      applySolverKnobs();

   const double wLo = 1.0, wHi = 300.0, zHi = 2.0;
   ImGui::SliderScalar( "drag spring [1/s]", ImGuiDataType_Double, &c.springOmega, &wLo, &wHi, "%.3g", ImGuiSliderFlags_Logarithmic );
   if( ImGui::IsItemHovered( ImGuiHoveredFlags_DelayShort ) )
      ImGui::SetTooltip( "Stiffness of the Ctrl + left-drag spring as an angular frequency; the force scales with\n"
                         "the body mass, so the feel is the same for every body." );
   ImGui::SliderScalar( "drag damping ratio", ImGuiDataType_Double, &c.springZeta, &zero, &zHi, "%.2f" );
}


bool processKeys()
{
   Controls& c = controls();
   const ImGuiIO& io = ImGui::GetIO();
   if( io.WantCaptureKeyboard )
      return false;
   if( ImGui::IsKeyPressed( ImGuiKey_Space, false ) )
      c.running = !c.running;
   if( ImGui::IsKeyPressed( ImGuiKey_N, true ) )
      c.queuedSteps += std::max( 1, c.stepsPerClick );
   return ImGui::IsKeyPressed( ImGuiKey_R, false );
}


int takeFrameSteps()
{
   Controls& c = controls();
   const int n = ( c.running ? c.stepsPerFrame : 0 ) + c.queuedSteps;
   c.queuedSteps = 0;
   return n;
}


void advance( int n, const std::vector<BodyID>& bodies, Clock& clock )
{
   Controls& c = controls();
   if( n <= 0 || !clock.error.empty() )
      return;

   WorldID world = theWorld();
   try {
      for( int i = 0; i < n; ++i ) {
         for( BodyID b : bodies )
            if( !b->isFixed() )
               b->addForce( b->getMass() * Vec3( 0.0, 0.0, c.gravityZ ) );
         mouseSpring().apply();
         world->simulationStep( static_cast<real>( c.dt ) );
         clock.time += c.dt;
         ++clock.steps;
      }
   }
   catch( const std::exception& e ) {
      clock.error = std::string( "simulationStep: " ) + e.what();
      c.running   = false;
   }
   for( BodyID b : bodies ) {
      const Vec3& p = b->getPosition();
      if( !std::isfinite( p[0] ) || !std::isfinite( p[1] ) || !std::isfinite( p[2] ) ) {
         clock.error = "non-finite body position: simulation stopped (Reset to rebuild)";
         c.running   = false;
         break;
      }
   }
}


void MouseSpring::process( const std::vector<Target>& targets, double lineRadius )
{
   ImGuiIO& io = ImGui::GetIO();

   if( body_ == nullptr ) {
      if( io.KeyCtrl && !io.WantCaptureMouse && ImGui::IsMouseClicked( 0 ) ) {
         const std::pair<polyscope::Structure*, size_t> hit =
            polyscope::pick::pickAtScreenCoords( glm::vec2( io.MousePos.x, io.MousePos.y ) );
         for( const Target& t : targets ) {
            if( hit.first != t.mesh || t.body->isFixed() )
               continue;
            body_   = t.body;
            target_ = viewer::toGlm( t.body->getPosition() );
            planeN_ = glm::normalize( polyscope::view::getCameraWorldPosition() - target_ );
            polyscope::state::doDefaultMouseInteraction = false;   // camera stays put during drag
         }
      }
      return;
   }

   bool stillTargeted = false;
   for( const Target& t : targets )
      stillTargeted = stillTargeted || t.body == body_;
   if( !ImGui::IsMouseDown( 0 ) || !stillTargeted ) {
      end();   // release: the momentum the spring imparted stays with the body
      return;
   }

   // Slide the anchor on the camera-facing plane through the grab point.
   const glm::vec3 org = polyscope::view::getCameraWorldPosition();
   const glm::vec3 dir = polyscope::view::screenCoordsToWorldRay( glm::vec2( io.MousePos.x, io.MousePos.y ) );
   const float denom = glm::dot( dir, planeN_ );
   if( std::abs( denom ) > 1.0e-6f ) {
      const float t = glm::dot( target_ - org, planeN_ ) / denom;
      if( t > 0.0f )
         target_ = org + t * dir;
   }
   const std::vector<glm::vec3> pts{ viewer::toGlm( body_->getPosition() ), target_ };
   polyscope::registerCurveNetworkLine( "mouse spring", pts )->setRadius( lineRadius, /*isRelative=*/false );
}


void MouseSpring::apply() const
{
   if( body_ == nullptr )
      return;
   const Controls& c = controls();
   const Vec3 target( target_.x, target_.y, target_.z );
   const real w = static_cast<real>( c.springOmega );
   const real z = static_cast<real>( c.springZeta );
   body_->addForce( body_->getMass() * ( w * w * ( target - body_->getPosition() ) - real(2) * z * w * body_->getLinearVel() ) );
}


void MouseSpring::end()
{
   body_ = nullptr;
   polyscope::removeStructure( "mouse spring", /*errorIfAbsent=*/false );
   polyscope::state::doDefaultMouseInteraction = true;
}

} // namespace simctl
