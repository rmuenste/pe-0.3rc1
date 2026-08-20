//=================================================================================================
/*!
 *  \file tools/live_viewer/live_viewer.cpp
 *  \brief Interactive Polyscope viewer for small lubrication scenarios
 *
 *  Live viewer for the HardContactLubricated solver stack: a small column of spheres
 *  settles onto a ground plane while all runtime lubrication knobs (pe::lubrication::*,
 *  see doc/technical-notes/lubrication-production-design.md) are exposed as ImGui widgets
 *  through a declarative parameter registry (ParamRegistry.h). ImPlot panes show the probe
 *  sphere's gap/velocity history plus the analytic Kroupa wall-normal resistance for the
 *  current knob values, so model parameters can be tuned with immediate visual feedback.
 *
 *  Requires pe_CONSTRAINT_SOLVER == pe::response::HardContactLubricated (the current
 *  default in pe/config/Collisions.h).
 */
//=================================================================================================

#include <pe/system/WarningDisable.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <vector>

#include <pe/core.h>
#include <pe/core/lubrication/Params.h>
#include <pe/support.h>

#include "glm/glm.hpp"
#include "polyscope/curve_network.h"
#include "polyscope/pick.h"
#include "polyscope/point_cloud.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

#include "implot.h"

#include "ParamRegistry.h"

using namespace pe;


namespace {

const double kPi = 3.14159265358979323846;


//=================================================================================================
//
//  SCENARIO AND SIMULATION STATE
//
//=================================================================================================

//! Parameters that define the initial world; edits apply on Reset, not mid-run.
struct Scenario {
   int    numSpheres  = 5;
   double radius      = 0.01;   // [m]
   double spacing     = 2.5;    // center-to-center spacing of the column, in radii
   double startHeight = 0.05;   // gap between lowest sphere surface and the plane [m]
   double initialVz   = -0.05;  // [m/s]
};

Scenario scenario;   // values staged in the GUI
Scenario active;     // values the current world was built with

bool   running       = false;
bool   stepQueued    = false;   // single-step request, consumed at the top of the next frame
int    stepsPerFrame = 4;
double dt            = 5.0e-4;
double simTime       = 0.0;

// Scenario default error reduction, applied once at startup. Softer than the engine's
// native 0.7 default: lubrication runs settle better with little constraint stabilization.
// The GUI slider reads/writes the live engine value from here on.
const double kScenarioErp = 0.05;

std::vector<SphereID> spheres;

// Probe time series (lowest sphere of the column)
std::vector<float> tBuf, gapBuf, vzBuf;

// Mouse-spring drag state (Ctrl + left-drag on a sphere)
int       dragIndex   = -1;              // index into spheres[] while dragging, else -1
glm::vec3 dragTarget( 0.0f );            // spring anchor in world space
glm::vec3 dragPlaneN( 0.0f );            // camera-facing drag plane normal
double    springOmega = 30.0;            // spring angular frequency [1/s]
double    springZeta  = 1.0;             // damping ratio

// Mirror data pushed to Polyscope
std::vector<glm::vec3>  spherePositions;
polyscope::PointCloud*  sphereCloud = nullptr;

std::vector<viewer::ParamGroup> liveParams;


//=================================================================================================
//
//  PE -> POLYSCOPE MIRROR
//
//=================================================================================================

void updateSpherePositions()
{
   for( std::size_t i = 0; i < spheres.size(); ++i ) {
      const Vec3 p = spheres[i]->getPosition();
      spherePositions[i] = glm::vec3( static_cast<float>( p[0] ),
                                      static_cast<float>( p[1] ),
                                      static_cast<float>( p[2] ) );
   }
   if( sphereCloud != nullptr )
      sphereCloud->updatePointPositions( spherePositions );
}


//! Pattern for mirroring boxes/capsules: register a unit-size surface mesh once via
//! polyscope::registerSurfaceMesh and call mesh->setTransform( bodyTransform( body, extents ) )
//! every frame. Unused by the default sphere scenario but kept as the extension template.
[[maybe_unused]] glm::mat4 bodyTransform( ConstBodyID body, const Vec3& scale )
{
   const Vec3& p = body->getPosition();
   const Rot3& R = body->getRotation();
   glm::mat4 T( 1.0f );
   for( int c = 0; c < 3; ++c )
      for( int r = 0; r < 3; ++r )
         T[c][r] = static_cast<float>( R( r, c ) * scale[c] );
   T[3] = glm::vec4( static_cast<float>( p[0] ),
                     static_cast<float>( p[1] ),
                     static_cast<float>( p[2] ), 1.0f );
   return T;
}


//=================================================================================================
//
//  MOUSE SPRING (Ctrl + left-drag on a sphere)
//
//=================================================================================================

void endDrag()
{
   dragIndex = -1;
   polyscope::removeStructure( "mouse spring", /*errorIfAbsent=*/false );
   polyscope::state::doDefaultMouseInteraction = true;
}


//! Mass-scaled damped spring toward the drag anchor: F = m ( w^2 (target - x) - 2 zeta w v ).
//! Parameterizing by frequency/damping ratio keeps the feel identical across body masses.
//! Must be called before each simulationStep; PE clears the force accumulator per step.
void applyMouseSpring()
{
   if( dragIndex < 0 || dragIndex >= static_cast<int>( spheres.size() ) )
      return;
   SphereID s = spheres[dragIndex];
   const Vec3 x = s->getPosition();
   const Vec3 v = s->getLinearVel();
   const Vec3 target( static_cast<real>( dragTarget.x ),
                      static_cast<real>( dragTarget.y ),
                      static_cast<real>( dragTarget.z ) );
   const real m = s->getMass();
   const real w = static_cast<real>( springOmega );
   const real z = static_cast<real>( springZeta );
   s->addForce( m * ( w * w * ( target - x ) - real(2) * z * w * v ) );
}


void processMouseDrag()
{
   ImGuiIO& io = ImGui::GetIO();

   if( dragIndex < 0 ) {
      if( io.KeyCtrl && !io.WantCaptureMouse && ImGui::IsMouseClicked( 0 ) ) {
         const glm::vec2 sc( io.MousePos.x, io.MousePos.y );
         const std::pair<polyscope::Structure*, size_t> hit = polyscope::pick::pickAtScreenCoords( sc );
         if( hit.first == sphereCloud && hit.second < spheres.size() ) {
            dragIndex = static_cast<int>( hit.second );
            const Vec3 p = spheres[dragIndex]->getPosition();
            dragTarget = glm::vec3( static_cast<float>( p[0] ),
                                    static_cast<float>( p[1] ),
                                    static_cast<float>( p[2] ) );
            dragPlaneN = glm::normalize( polyscope::view::getCameraWorldPosition() - dragTarget );
            polyscope::state::doDefaultMouseInteraction = false;   // camera stays put during drag
         }
      }
      return;
   }

   if( !ImGui::IsMouseDown( 0 ) || dragIndex >= static_cast<int>( spheres.size() ) ) {
      endDrag();   // release: the momentum the spring imparted stays with the body
      return;
   }

   // Slide the anchor on the camera-facing plane through the grab point.
   const glm::vec3 org = polyscope::view::getCameraWorldPosition();
   const glm::vec3 dir = polyscope::view::screenCoordsToWorldRay( glm::vec2( io.MousePos.x, io.MousePos.y ) );
   const float denom = glm::dot( dir, dragPlaneN );
   if( std::abs( denom ) > 1.0e-6f ) {
      const float t = glm::dot( dragTarget - org, dragPlaneN ) / denom;
      if( t > 0.0f )
         dragTarget = org + t * dir;
   }

   const Vec3 p = spheres[dragIndex]->getPosition();
   const std::vector<glm::vec3> pts{ glm::vec3( static_cast<float>( p[0] ),
                                                static_cast<float>( p[1] ),
                                                static_cast<float>( p[2] ) ),
                                     dragTarget };
   polyscope::registerCurveNetworkLine( "mouse spring", pts );
}


//=================================================================================================
//
//  SCENE CONSTRUCTION
//
//=================================================================================================

void buildScene()
{
   endDrag();

   WorldID world = theWorld();
   world->clear();
   spheres.clear();
   tBuf.clear();
   gapBuf.clear();
   vzBuf.clear();
   simTime = 0.0;
   running = false;
   active  = scenario;

   // Materials survive World::clear(); create them exactly once.
   static const MaterialID planeMaterial  = createMaterial( "viewer_plane" , 2.5, 0.0, 0.5 , 0.3 , 0.25, 150, 1000  , 10, 11 );
   static const MaterialID sphereMaterial = createMaterial( "viewer_sphere", 7.0, 0.2, 0.45, 0.35, 0.22, 300, 5.0e4 , 20, 25 );

   unsigned int id( 0U );

   PlaneID plane = createPlane( id++, 0.0, 0.0, 1.0, 0.0, planeMaterial );
   plane->setFixed( true );

   const real r = static_cast<real>( active.radius );
   for( int i = 0; i < active.numSpheres; ++i ) {
      const real z = r + static_cast<real>( active.startHeight )
                   + static_cast<real>( i ) * static_cast<real>( active.spacing ) * r;
      SphereID s = createSphere( id++, 0.0, 0.0, z, r, sphereMaterial );
      s->setLinearVel( 0.0, 0.0, static_cast<real>( active.initialVz ) );
      spheres.push_back( s );
   }

   // (Re-)register the Polyscope mirror. Per-point radii so a polydisperse
   // scenario needs no viewer changes. The old cloud must not receive the new
   // positions (the sizes differ on reset); registerPointCloud replaces it.
   sphereCloud = nullptr;
   spherePositions.assign( spheres.size(), glm::vec3( 0.0f ) );
   updateSpherePositions();
   sphereCloud = polyscope::registerPointCloud( "spheres", spherePositions );
   std::vector<double> radii( spheres.size(), active.radius );
   auto* radiusQty = sphereCloud->addScalarQuantity( "radius", radii );
   sphereCloud->setPointRadiusQuantity( radiusQty, /*autoScale=*/false );

   // Auto-computed extents degenerate for a single point (zero length scale ->
   // NaN camera -> white screen) and drift as bodies fall; size them to the
   // scenario instead and re-home the camera on every scene rebuild.
   const double columnTop = active.startHeight
                          + active.radius * ( 2.0 + ( active.numSpheres - 1 ) * active.spacing );
   const float ext = static_cast<float>( std::max( columnTop, 8.0 * active.radius ) );
   polyscope::options::automaticallyComputeSceneExtents = false;
   polyscope::state::lengthScale = ext;
   polyscope::state::boundingBox =
      std::tuple<glm::vec3, glm::vec3>{ glm::vec3( -ext, -ext, 0.0f ),
                                        glm::vec3(  ext,  ext, ext  ) };
   polyscope::view::resetCameraToHomeView();
}


void sampleProbe()
{
   if( spheres.empty() )
      return;
   const Vec3 p = spheres.front()->getPosition();
   const Vec3 v = spheres.front()->getLinearVel();
   tBuf.push_back  ( static_cast<float>( simTime ) );
   gapBuf.push_back( static_cast<float>( p[2] - active.radius ) );
   vzBuf.push_back ( static_cast<float>( v[2] ) );
}


//=================================================================================================
//
//  PARAMETER REGISTRY
//
//=================================================================================================

void setupParamRegistry()
{
   namespace lub = pe::lubrication;
   WorldID world = theWorld();

   viewer::ParamGroup fluid{ "Fluid / world" };
   fluid.reals.push_back( { "viscosity", 1.0e-6f, 1.0f, true,
      [world]() { return static_cast<double>( world->getViscosity() ); },
      [world]( double v ) { world->setViscosity( static_cast<real>( v ) ); },
      "Dynamic viscosity of the surrounding fluid" } );
   fluid.reals.push_back( { "damping", 0.9f, 1.0f, false,
      [world]() { return static_cast<double>( world->getDamping() ); },
      [world]( double v ) { world->setDamping( static_cast<real>( v ) ); } } );
   fluid.reals.push_back( { "gravity z", -20.0f, 0.0f, false,
      [world]() { return static_cast<double>( world->getGravity()[2] ); },
      [world]( double v ) {
         const Vec3 g = world->getGravity();
         world->setGravity( g[0], g[1], static_cast<real>( v ) );
      } } );

   viewer::ParamGroup model{ "Lubrication model" };
   model.bools.push_back( { "enabled",
      []() { return lub::isEnabled(); },
      []( bool b ) { lub::setEnabled( b ); },
      "Master switch for lubrication contacts/forces" } );
   model.enums.push_back( { "force model", { "Kroupa 2016", "legacy" },
      []() { return lub::getModel(); },
      []( int m ) { lub::setModel( m ); } } );
   model.enums.push_back( { "scheme", { "semi-implicit", "explicit-capped" },
      []() { return lub::getScheme(); },
      []( int s ) { lub::setScheme( s ); } } );
   model.bools.push_back( { "tangential terms",
      []() { return lub::getTangential(); },
      []( bool b ) { lub::setTangential( b ); },
      "Sliding force + sliding torque" } );
   model.bools.push_back( { "twisting torque",
      []() { return lub::getTwisting(); },
      []( bool b ) { lub::setTwisting( b ); } } );
   model.bools.push_back( { "slip correction f*",
      []() { return lub::getSlipCorrection(); },
      []( bool b ) { lub::setSlipCorrection( b ); },
      "Vinogradova slip correction" } );
   model.bools.push_back( { "resist separation",
      []() { return lub::getResistSeparation(); },
      []( bool b ) { lub::setResistSeparation( b ); },
      "Apply normal suction on separating motion, not only approach" } );
   model.bools.push_back( { "wall terms",
      []() { return lub::getWallTerms(); },
      []( bool b ) { lub::setWallTerms( b ); },
      "Dedicated resistance set for sphere-plane pairs" } );

   viewer::ParamGroup cutoffs{ "Cutoffs / regularization" };
   cutoffs.reals.push_back( { "eps_c (critical)", 1.0e-5f, 1.0e-1f, true,
      []() { return static_cast<double>( lub::getEpsCritical() ); },
      []( double v ) { lub::setEpsCritical( static_cast<real>( v ) ); },
      "Saturation distance / slip length scale: eps_c = h_c / a_ref" } );
   cutoffs.reals.push_back( { "cutoff factor", 0.0f, 1.0f, false,
      []() { return static_cast<double>( lub::getCutoffFactor() ); },
      []( double v ) { lub::setCutoffFactor( static_cast<real>( v ) ); },
      "Relative outer cutoff eps_cut = h_cut / a_ref; 0 = legacy absolute threshold" } );
   cutoffs.reals.push_back( { "legacy threshold", 1.0e-6f, 1.0e-2f, true,
      []() { return static_cast<double>( lub::getLubricationThreshold() ); },
      []( double v ) { lub::setLubricationThreshold( static_cast<real>( v ) ); },
      "Absolute outer cutoff, active only when cutoff factor == 0" } );
   cutoffs.reals.push_back( { "contact hysteresis", 1.0e-8f, 1.0e-3f, true,
      []() { return static_cast<double>( lub::getContactHysteresisDelta() ); },
      []( double v ) { lub::setContactHysteresisDelta( static_cast<real>( v ) ); } } );
   cutoffs.reals.push_back( { "lubrication hysteresis", 1.0e-8f, 1.0e-3f, true,
      []() { return static_cast<double>( lub::getLubricationHysteresisDelta() ); },
      []( double v ) { lub::setLubricationHysteresisDelta( static_cast<real>( v ) ); } } );
   cutoffs.reals.push_back( { "min eps (gap clamp)", 1.0e-10f, 1.0e-4f, true,
      []() { return static_cast<double>( theCollisionSystem()->getMinEpsLub() ); },
      []( double v ) { theCollisionSystem()->setMinEpsLub( static_cast<real>( v ) ); },
      "Minimal lubrication gap regularization (minEpsLub)" } );
   cutoffs.bools.push_back( { "AABB inflation",
      []() { return lub::getAabbInflation(); },
      []( bool b ) { lub::setAabbInflation( b ); },
      "Grow AABBs by the cutoff so coarse detection sees pre-contact pairs" } );

   viewer::ParamGroup solver{ "Solver" };
   solver.reals.push_back( { "error reduction", 0.0f, 1.0f, false,
      []() { return static_cast<double>( theCollisionSystem()->getErrorReductionParameter() ); },
      []( double v ) { theCollisionSystem()->setErrorReductionParameter( static_cast<real>( v ) ); } } );

   viewer::ParamGroup spring{ "Mouse spring" };
   spring.reals.push_back( { "frequency [1/s]", 1.0f, 300.0f, true,
      []() { return springOmega; },
      []( double v ) { springOmega = v; },
      "Spring stiffness as angular frequency; force scales with body mass" } );
   spring.reals.push_back( { "damping ratio", 0.0f, 2.0f, false,
      []() { return springZeta; },
      []( double v ) { springZeta = v; },
      "1 = critically damped drag, <1 springy, >1 sluggish" } );

   liveParams = { fluid, model, cutoffs, solver, spring };
}


//=================================================================================================
//
//  PLOTS
//
//=================================================================================================

//! Analytic Kroupa sphere-wall normal resistance for the current knob values, evaluated at
//! the probe's current approach speed (design note, wall set):
//!   |F_nl^w| = 6 pi eta R |v| ( 1/eps - ln(eps)/5 - eps ln(eps)/21 ),  eps = max( h/R, eps_c )
void buildAnalyticCurve( std::vector<float>& hs, std::vector<float>& fs )
{
   namespace lub = pe::lubrication;
   hs.clear();
   fs.clear();

   const double eta  = static_cast<double>( theWorld()->getViscosity() );
   const double R    = active.radius;
   const double vRef = vzBuf.empty() ? std::abs( active.initialVz )
                                     : std::max( 1.0e-12, static_cast<double>( std::abs( vzBuf.back() ) ) );
   const double epsC = static_cast<double>( lub::getEpsCritical() );
   const double hCut = static_cast<double>( lub::lubricationCutoff( static_cast<real>( R ) ) );
   if( R <= 0.0 || hCut <= 0.0 )
      return;

   const double hMin = std::max( 1.0e-8 * R, 0.05 * epsC * R );
   const int    n    = 200;
   for( int i = 0; i < n; ++i ) {
      const double h   = hMin * std::pow( hCut / hMin, i / static_cast<double>( n - 1 ) );
      const double eps = std::max( h / R, epsC );
      const double f   = 6.0 * kPi * eta * R * vRef
                       * ( 1.0 / eps - std::log( eps ) / 5.0 - eps * std::log( eps ) / 21.0 );
      hs.push_back( static_cast<float>( h ) );
      fs.push_back( static_cast<float>( std::max( f, 0.0 ) ) );
   }
}


void drawPlots()
{
   ImGui::Begin( "Plots" );
   const int n = static_cast<int>( tBuf.size() );

   if( ImPlot::BeginPlot( "gap vs time", ImVec2( -1, 200 ) ) ) {
      ImPlot::SetupAxes( "t [s]", "gap [m]" );
      if( n > 0 )
         ImPlot::PlotLine( "probe gap", tBuf.data(), gapBuf.data(), n );
      ImPlot::EndPlot();
   }

   if( ImPlot::BeginPlot( "v_z vs time", ImVec2( -1, 200 ) ) ) {
      ImPlot::SetupAxes( "t [s]", "v_z [m/s]" );
      if( n > 0 )
         ImPlot::PlotLine( "probe v_z", tBuf.data(), vzBuf.data(), n );
      ImPlot::EndPlot();
   }

   static std::vector<float> hCurve, fCurve;
   buildAnalyticCurve( hCurve, fCurve );
   if( ImPlot::BeginPlot( "analytic |F_n| vs gap (wall term)", ImVec2( -1, 220 ) ) ) {
      ImPlot::SetupAxes( "h [m]", "|F_n| [N]" );
      ImPlot::SetupAxisScale( ImAxis_X1, ImPlotScale_Log10 );
      ImPlot::SetupAxisScale( ImAxis_Y1, ImPlotScale_Log10 );
      if( !hCurve.empty() )
         ImPlot::PlotLine( "Kroupa wall", hCurve.data(), fCurve.data(),
                           static_cast<int>( hCurve.size() ) );
      ImPlot::EndPlot();
   }

   ImGui::End();
}


//=================================================================================================
//
//  PER-FRAME CALLBACK
//
//=================================================================================================

void guiCallback()
{
   WorldID world = theWorld();

   processMouseDrag();

   // Advance the simulation. A queued single step is executed here so it renders
   // exactly one frame later - imperceptible, and it keeps step/draw ordering trivial.
   int steps = 0;
   if( running ) {
      steps = stepsPerFrame;
   }
   else if( stepQueued ) {
      steps = 1;
      stepQueued = false;
   }
   for( int i = 0; i < steps; ++i ) {
      applyMouseSpring();
      world->simulationStep( static_cast<real>( dt ) );
      simTime += dt;
      sampleProbe();
   }
   if( steps > 0 )
      updateSpherePositions();

   ImGui::Begin( "Simulation" );
   if( ImGui::Button( running ? "Pause" : "Run" ) )
      running = !running;
   ImGui::SameLine();
   if( ImGui::Button( "Step" ) )
      stepQueued = true;
   ImGui::SameLine();
   if( ImGui::Button( "Reset" ) )
      buildScene();
   ImGui::SliderInt( "steps / frame", &stepsPerFrame, 1, 64 );
   ImGui::InputDouble( "dt", &dt, 0.0, 0.0, "%.3e" );
   ImGui::Text( "t = %.4f s | samples: %d | contacts: %d",
                simTime, static_cast<int>( tBuf.size() ),
                static_cast<int>( theCollisionSystem()->getNumberOfContacts() ) );
   if( dragIndex >= 0 )
      ImGui::TextDisabled( "dragging sphere %d", dragIndex );
   else
      ImGui::TextDisabled( "Ctrl + left-drag a sphere to pull it" );
   ImGui::End();

   ImGui::Begin( "Parameters (live)" );
   viewer::drawParamGroups( liveParams );
   ImGui::End();

   ImGui::Begin( "Scenario (applied on Reset)" );
   ImGui::SliderInt( "spheres", &scenario.numSpheres, 1, 50 );
   ImGui::InputDouble( "radius",       &scenario.radius,      0.0, 0.0, "%.4g" );
   ImGui::InputDouble( "spacing [R]",  &scenario.spacing,     0.0, 0.0, "%.3g" );
   ImGui::InputDouble( "start height", &scenario.startHeight, 0.0, 0.0, "%.4g" );
   ImGui::InputDouble( "initial v_z",  &scenario.initialVz,   0.0, 0.0, "%.4g" );
   if( ImGui::Button( "Reset with these values" ) )
      buildScene();
   ImGui::End();

   drawPlots();
}

} // namespace


//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================

int main( int, char** )
{
   // Engine defaults mirroring examples/basic_lubrication.
   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -9.81 );
   world->setViscosity( 8.37e-5 );
   world->setLiquidDensity( 1.0 );
   world->setLiquidSolid( true );
   world->setDamping( 1.0 );

   lubrication::setEnabled( true );
   theCollisionSystem()->setErrorReductionParameter( static_cast<real>( kScenarioErp ) );

   polyscope::options::programName = "PE live viewer";
   // The callback opens its own windows; skip the empty wrapper window Polyscope
   // would otherwise create for it.
   polyscope::options::openImGuiWindowForUserCallback = false;
   polyscope::view::setUpDir( polyscope::UpDir::ZUp );
   polyscope::options::groundPlaneMode       = polyscope::GroundPlaneMode::TileReflection;
   polyscope::options::groundPlaneHeightMode = polyscope::GroundPlaneHeightMode::Manual;
   polyscope::options::groundPlaneHeight     = 0.0f;   // coincides with the PE plane at z=0
   polyscope::init();
   ImPlot::CreateContext();

   setupParamRegistry();
   buildScene();

   polyscope::state::userCallback = guiCallback;
   polyscope::show();

   ImPlot::DestroyContext();
   return EXIT_SUCCESS;
}
