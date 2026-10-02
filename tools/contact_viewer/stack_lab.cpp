//=================================================================================================
/*!
 *  \file tools/contact_viewer/stack_lab.cpp
 *  \brief Stack Lab mode of the contact viewer: run / pause / step a small simulation
 *
 *  A small world (up to a few dozen bodies) on a fixed ground plane at z = 0, stepped with the
 *  configured collision system (pe_CONSTRAINT_SOLVER). Scenarios stress resting contact: box
 *  towers, pyramids, brick walls, mixed-shape stacks, a box on a ramp and shapes dropped onto
 *  the ground. After each rendered frame the narrow phase is re-run over all body pairs into a
 *  recording container (the collision system clears its own contacts at the end of a step), so
 *  the overlay shows the contacts of the current post-step configuration. Diagnostics: kinetic
 *  energy, the solver's maximum penetration and contact count, and the drift of the top body.
 */
//=================================================================================================

#include <pe/system/WarningDisable.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <random>
#include <string>
#include <vector>

#include <pe/core.h>
#include <pe/core/detection/fine/MaxContacts.h>

#include "glm/glm.hpp"
#include "polyscope/curve_network.h"
#include "polyscope/pick.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

#include "implot.h"

#include "ContactOverlay.h"
#include "ShapeMeshes.h"
#include "SimControls.h"
#include "StackLab.h"

using namespace pe;
using pe::detection::fine::MaxContacts;


namespace {

const double kPi       = 3.14159265358979323846;
const double kDegToRad = kPi / 180.0;


//=================================================================================================
//
//  SCENARIO AND SIMULATION STATE
//
//=================================================================================================

// Appended kinds keep the indices (--preset k) of the existing ones stable.
enum ScenarioKind { kTower, kPyramid, kWall, kMixed, kRamp, kDrop, kCylinderStack, kSquareWall, kNumScenarios };
const char* const kScenarioNames[kNumScenarios] = {
   "box tower", "triangular wall", "brick wall", "mixed-shape stack", "box on a ramp", "drop shapes on the ground",
   "upright cylinder stack", "square wall (N x N)" };

enum DropShape { kDropSphere, kDropBox, kDropCapsule, kDropCylinder, kDropEllipsoid, kNumDropShapes };
const char* const kDropShapeNames[kNumDropShapes] = { "sphere", "box", "capsule", "cylinder", "ellipsoid" };

//! Parameters that define the initial world; edits apply on Reset, not mid-run.
struct Scenario {
   int    kind        = kTower;
   int    count       = 6;       // tower/stack height, pyramid base, wall rows, number of drops
   int    wallColumns = 4;
   double size        = 1.0;     // box edge length s; every other dimension scales with it
   double gap         = 0.0;     // initial vertical gap between stacked bodies, in s
   double sideGap     = 0.0;     // gap between neighbours in a row of the triangular / square wall, in s
   double jitter      = 0.0;     // random lateral offset per body, in s
   double yawJitter   = 0.0;     // random yaw per body [deg]
   int    seed        = 1;
   double rampAngle   = 20.0;    // [deg]
   int    dropShape   = kDropBox;
   double dropHeight  = 1.0;     // clearance above the ground, in s
   double dropTilt[2] = { 20.0, 10.0 };  // Euler x, y [deg]
   double friction    = 0.4;     // pair friction coefficient mu of every contact
   double restitution = 0.0;
   double density     = 1.0;
};

Scenario staged;   // values edited in the GUI
Scenario active;   // values the current world was built with

simctl::Clock simClock;   // simulated time of this mode's world; stepping settings are shared (SimControls.h)

//! One rendered body; the ground plane is drawn by Polyscope's ground and has no entry.
struct SimBody {
   BodyID                  body;
   polyscope::SurfaceMesh* mesh;
   glm::vec3               color;
};
std::vector<SimBody> simBodies;
std::vector<BodyID>  contactBodies;   // every body including the ground plane, for the contact pass

// Contact overlay (narrow phase re-run over all pairs after the frame's steps)
viewer::ContactLog     contactLog;
viewer::OverlayOptions overlay;
bool                   showContacts = true;
bool                   colorBySpeed = false;
double                 speedScale   = 2.0;   // speed mapped to full red [m/s]

// Diagnostics, sampled once per rendered frame that advanced the simulation
int  topIndex = -1;   // index into simBodies of the initially highest dynamic body
Vec3 topStart;
std::vector<double> tBuf, keBuf, penBuf, solverContactsBuf, overlayContactsBuf, driftBuf;

//=================================================================================================
//
//  PE -> POLYSCOPE MIRROR
//
//=================================================================================================

glm::vec3 bodyColor( std::size_t i )
{
   // Golden-ratio hue walk: neighbours in a stack get clearly different, muted colors.
   const double h = std::fmod( 0.08 + 0.618033988749895 * static_cast<double>( i ), 1.0 );
   const double s = 0.45, v = 0.90;
   const double hh = h * 6.0;
   const int    k  = static_cast<int>( hh ) % 6;
   const double f  = hh - std::floor( hh );
   const double p = v * ( 1 - s ), q = v * ( 1 - s * f ), t = v * ( 1 - s * ( 1 - f ) );
   double r = v, g = t, b = p;
   switch( k ) {
      case 1: r = q; g = v; b = p; break;
      case 2: r = p; g = v; b = t; break;
      case 3: r = p; g = q; b = v; break;
      case 4: r = t; g = p; b = v; break;
      case 5: r = v; g = p; b = q; break;
      default: break;
   }
   return glm::vec3( static_cast<float>( r ), static_cast<float>( g ), static_cast<float>( b ) );
}


void updateMirror()
{
   for( SimBody& sb : simBodies ) {
      sb.mesh->setTransform( viewer::bodyTransform( sb.body ) );
      if( colorBySpeed && !sb.body->isFixed() ) {
         const float t = static_cast<float>( std::min( 1.0, static_cast<double>( sb.body->getLinearVel().length() ) / speedScale ) );
         sb.mesh->setSurfaceColor( ( 1.0f - t ) * glm::vec3( 0.30f, 0.45f, 0.85f ) + t * glm::vec3( 0.95f, 0.20f, 0.15f ) );
      }
      else {
         sb.mesh->setSurfaceColor( sb.color );
      }
   }
}


//! Narrow phase over all body pairs of the current configuration, AABB-prefiltered.
void updateContacts()
{
   contactLog.clear();
   if( showContacts ) {
      try {
         for( std::size_t i = 0; i < contactBodies.size(); ++i )
            for( std::size_t j = i + 1; j < contactBodies.size(); ++j ) {
               BodyID a = contactBodies[i], b = contactBodies[j];
               if( a->isFixed() && b->isFixed() )
                  continue;
               if( a->getType() != planeType && b->getType() != planeType
                   && !a->getAABB().overlaps( b->getAABB(), contactThreshold ) )
                  continue;
               MaxContacts::collide( a, b, contactLog );
            }
      }
      catch( const std::exception& e ) {
         simClock.error = std::string( "contact overlay: " ) + e.what();
      }
   }
   viewer::drawContactOverlay( "contacts", contactLog, overlay );
}


double kineticEnergy()
{
   double e = 0.0;
   for( const SimBody& sb : simBodies ) {
      if( sb.body->isFixed() )
         continue;
      const Vec3& v = sb.body->getLinearVel();
      const Vec3& w = sb.body->getAngularVel();
      e += 0.5 * static_cast<double>( sb.body->getMass() * ( trans( v ) * v ) );
      e += 0.5 * static_cast<double>( trans( w ) * ( sb.body->getInertia() * w ) );
   }
   return e;
}


void sampleDiagnostics()
{
   if( tBuf.size() > 60000 ) {   // keep the plots responsive in long runs: drop the older half
      for( std::vector<double>* buf : { &tBuf, &keBuf, &penBuf, &solverContactsBuf, &overlayContactsBuf, &driftBuf } )
         buf->erase( buf->begin(), buf->begin() + static_cast<long>( buf->size() / 2 ) );
   }
   tBuf.push_back( simClock.time );
   keBuf.push_back( kineticEnergy() );
   penBuf.push_back( static_cast<double>( theCollisionSystem()->getMaximumPenetration() ) );
   solverContactsBuf.push_back( static_cast<double>( theCollisionSystem()->getNumberOfContacts() ) );
   overlayContactsBuf.push_back( static_cast<double>( contactLog.entries.size() ) );
   driftBuf.push_back( topIndex >= 0
      ? static_cast<double>( ( simBodies[topIndex].body->getPosition() - topStart ).length() ) : 0.0 );
}


//=================================================================================================
//
//  SCENE CONSTRUCTION
//
//=================================================================================================

pe::id_t      nextId = 0;
MaterialID    material;
std::mt19937  rng;

double uniform( double halfWidth )
{
   if( halfWidth <= 0.0 )
      return 0.0;
   return std::uniform_real_distribution<double>( -halfWidth, halfWidth )( rng );
}


void addBody( BodyID b, const Quat& q = Quat() )
{
   b->setOrientation( q );
   contactBodies.push_back( b );
   const viewer::ShapeMesh m = viewer::makeBodyMesh( b );
   const std::string name = "body " + std::to_string( simBodies.size() );
   polyscope::SurfaceMesh* mesh = polyscope::registerSurfaceMesh( name, m.vertices, m.faces );
   const glm::vec3 color = b->isFixed() ? glm::vec3( 0.55f, 0.55f, 0.58f ) : bodyColor( simBodies.size() );
   mesh->setSurfaceColor( color );
   mesh->setEdgeWidth( b->getType() == boxType ? 1.0 : 0.0 );
   mesh->setSmoothShade( b->getType() != boxType );
   simBodies.push_back( SimBody{ b, mesh, color } );
}


//! Stacked bodies share this layout: body i rests at height 0.5 s + i (s + gap) with a random
//! lateral offset and yaw.
Vec3 stackPosition( int i, double s )
{
   return Vec3( uniform( active.jitter * s ), uniform( active.jitter * s ),
                0.5 * s + i * s * ( 1.0 + active.gap ) );
}

Quat yawJitter()
{
   return Quat( 0.0, 0.0, uniform( active.yawJitter ) * kDegToRad );
}


void buildTower( double s )
{
   for( int i = 0; i < active.count; ++i )
      addBody( createBox( ++nextId, stackPosition( i, s ), Vec3( s, s, s ), material ), yawJitter() );
}


//! Row \a r (0 = bottom) of \a n unit boxes (edge s), centred on x = 0 and touching up to
//! sideGap. Each box's x extent after its yaw jitter is used for the layout, so neighbours never
//! start overlapping (an overlap would be corrected with a velocity and launch the boxes); the
//! lateral jitter acts across the wall (y) only.
void buildWallRow( int r, int n, double s )
{
   std::vector<double> yaw( n ), half( n );
   double width = active.sideGap * s * ( n - 1 );
   for( int k = 0; k < n; ++k ) {
      yaw[k]  = uniform( active.yawJitter ) * kDegToRad;
      half[k] = 0.5 * s * ( std::abs( std::cos( yaw[k] ) ) + std::abs( std::sin( yaw[k] ) ) );
      width  += 2.0 * half[k];
   }
   double x = -0.5 * width;
   for( int k = 0; k < n; ++k ) {
      x += half[k];
      const Vec3 p( x, uniform( active.jitter * s ), 0.5 * s + r * s * ( 1.0 + active.gap ) );
      addBody( createBox( ++nextId, p, Vec3( s, s, s ), material ), Quat( 0.0, 0.0, yaw[k] ) );
      x += half[k] + active.sideGap * s;
   }
}


//! Triangular wall: count boxes in the bottom row, one fewer per row, one on top; every row is
//! centred, so its boxes sit over the joints of the row below.
void buildTriangularWall( double s )
{
   for( int r = 0; r < active.count; ++r )
      buildWallRow( r, active.count - r, s );
}


//! Square wall: count x count boxes, columns stacked straight on top of each other.
void buildSquareWall( double s )
{
   for( int r = 0; r < active.count; ++r )
      buildWallRow( r, active.count, s );
}


void buildWall( double s )
{
   // Bricks 2s x s x s; odd rows are offset by half a brick.
   const double pitch = 2.0 * s * 1.02;
   for( int r = 0; r < active.count; ++r )
      for( int c = 0; c < active.wallColumns; ++c ) {
         Vec3 p = stackPosition( r, s );
         p[0] += ( c - 0.5 * ( active.wallColumns - 1 ) ) * pitch + ( r % 2 ) * 0.5 * pitch;
         addBody( createBox( ++nextId, p, Vec3( 2.0 * s, s, s ), material ), yawJitter() );
      }
}


void buildMixed( double s )
{
   // Every body is s tall; capsules and cylinders lie on their side (axis = body x).
   for( int i = 0; i < active.count; ++i ) {
      const Vec3 p = stackPosition( i, s );
      BodyID b;
      switch( i % 8 ) {
         case 1:  b = createCylinder ( ++nextId, p, 0.5 * s, s, material ); break;
         case 3:  b = createCapsule  ( ++nextId, p, 0.5 * s, s, material ); break;
         case 5:  b = createEllipsoid( ++nextId, p, 0.75 * s, 0.5 * s, 0.5 * s, material ); break;
         case 7:  b = createSphere   ( ++nextId, p, 0.5 * s, material ); break;
         default: b = createBox      ( ++nextId, p, Vec3( 1.5 * s, 1.5 * s, s ), material ); break;
      }
      addBody( b, yawJitter() );
   }
}


void buildCylinderStack( double s )
{
   // Box, upright cylinder, upright cylinder, box, ...: every cylinder stands flat on a box or on
   // another cylinder's cap and carries a box or a cylinder. Exercises the face manifolds of
   // box-cylinder and cylinder-cylinder. All bodies are s tall; the axis (body x) points up.
   for( int i = 0; i < active.count; ++i ) {
      const Vec3 p = stackPosition( i, s );
      if( i % 3 == 0 ) {
         addBody( createBox( ++nextId, p, Vec3( 1.2 * s, 1.2 * s, s ), material ), yawJitter() );
      }
      else {
         addBody( createCylinder( ++nextId, p, 0.5 * s, s, material ),
                  Quat( 0.0, 0.5 * kPi, uniform( active.yawJitter ) * kDegToRad ) );
      }
   }
}


void buildRamp( double s )
{
   // A fixed plank tilted about y (its +x end down), normal n = ( sin a, 0, cos a ); the test
   // boxes rest on its upper half, aligned with it.
   const double a     = active.rampAngle * kDegToRad;
   const double thick = 0.2 * s;
   const Quat   q( 0.0, a, 0.0 );
   const Vec3   center( 0.0, 0.0, 4.0 * s * std::sin( a ) + thick + 0.05 * s );
   BoxID ramp = createBox( ++nextId, center, Vec3( 8.0 * s, 3.0 * s, thick ), material );
   ramp->setFixed( true );
   addBody( ramp, q );

   const Rot3 R( q.toRotationMatrix() );
   for( int i = 0; i < std::max( 1, active.count / 2 ); ++i ) {
      const Vec3 local( -2.5 * s + i * 1.2 * s, 0.0, 0.5 * thick + 0.5 * s + active.gap * s );
      addBody( createBox( ++nextId, center + R * local, Vec3( s, s, s ), material ), q );
   }
}


void buildDrop( double s )
{
   for( int i = 0; i < active.count; ++i ) {
      const Vec3 p( ( i - 0.5 * ( active.count - 1 ) ) * 2.0 * s + uniform( active.jitter * s ),
                    uniform( active.jitter * s ),
                    ( active.dropHeight + 1.0 ) * s );
      BodyID b;
      switch( active.dropShape ) {
         case kDropSphere:    b = createSphere   ( ++nextId, p, 0.5 * s, material ); break;
         case kDropCapsule:   b = createCapsule  ( ++nextId, p, 0.3 * s, s, material ); break;
         case kDropCylinder:  b = createCylinder ( ++nextId, p, 0.5 * s, s, material ); break;
         case kDropEllipsoid: b = createEllipsoid( ++nextId, p, 0.6 * s, 0.4 * s, 0.25 * s, material ); break;
         default:             b = createBox      ( ++nextId, p, Vec3( s, s, s ), material ); break;
      }
      addBody( b, Quat( active.dropTilt[0] * kDegToRad, active.dropTilt[1] * kDegToRad,
                        uniform( active.yawJitter ) * kDegToRad ) );
   }
}


void frameCamera()
{
   // Bounds of the rendered bodies; the ground plane is infinite and does not count.
   Vec3 lo( 1e30, 1e30, 0.0 ), hi( -1e30, -1e30, active.size );
   for( const SimBody& sb : simBodies ) {
      const RigidBody::AABB& bb = sb.body->getAABB();
      for( int k = 0; k < 3; ++k ) {
         lo[k] = std::min( lo[k], bb[k] );
         hi[k] = std::max( hi[k], bb[k+3] );
      }
   }
   const Vec3   c = 0.5 * ( lo + hi );
   const double r = std::max( static_cast<double>( ( hi - lo ).length() ) * 0.5, 1.5 * active.size );

   polyscope::options::automaticallyComputeSceneExtents = false;
   polyscope::state::lengthScale = static_cast<float>( 2.0 * r );
   polyscope::state::boundingBox = std::tuple<glm::vec3, glm::vec3>{
      glm::vec3( static_cast<float>( c[0] - r ), static_cast<float>( c[1] - r ), 0.0f ),
      glm::vec3( static_cast<float>( c[0] + r ), static_cast<float>( c[1] + r ), static_cast<float>( 2.0 * r ) ) };
   // Distance chosen so the bounding sphere fits the default vertical field of view (45 deg)
   // with some margin; tall towers were cut off with a fixed factor.
   const glm::vec3 target( static_cast<float>( c[0] ), static_cast<float>( c[1] ), static_cast<float>( c[2] ) );
   const float     dist = static_cast<float>( 1.3 * r / std::sin( 0.5 * 45.0 * kDegToRad ) );
   polyscope::view::lookAt( target + dist * glm::normalize( glm::vec3( 1.2f, -2.0f, 0.8f ) ), target );
}


void buildScene()
{
   simctl::mouseSpring().end();
   contactLog.clear();   // holds pointers into the world being cleared
   simBodies.clear();
   contactBodies.clear();
   polyscope::removeAllStructures();

   WorldID world = theWorld();
   world->clear();
   world->setGravity( 0.0, 0.0, 0.0 );   // gravity is applied as a force (simctl::Controls::gravityZ)

   active = staged;
   simctl::controls().running     = false;
   simctl::controls().queuedSteps = 0;
   simClock.reset();
   for( std::vector<double>* buf : { &tBuf, &keBuf, &penBuf, &solverContactsBuf, &overlayContactsBuf, &driftBuf } )
      buf->clear();
   rng.seed( static_cast<unsigned>( active.seed ) );
   nextId = 0;

   // pe combines pair friction additively (Materials.cpp): each body gets mu / 2. Materials
   // cannot be edited after creation, so every reset registers a fresh anonymous one.
   const real halfMu = static_cast<real>( 0.5 * active.friction );
   material = createMaterial( static_cast<real>( active.density ), static_cast<real>( active.restitution ),
                              halfMu, halfMu, real(0.25), real(300), real(1e5), real(10), real(10) );

   PlaneID ground = createPlane( ++nextId, 0.0, 0.0, 1.0, 0.0, material );
   contactBodies.push_back( ground );

   const double s = active.size;
   switch( active.kind ) {
      case kPyramid:    buildTriangularWall( s ); break;
      case kSquareWall: buildSquareWall( s );     break;
      case kWall:    buildWall( s );    break;
      case kMixed:   buildMixed( s );   break;
      case kCylinderStack: buildCylinderStack( s ); break;
      case kRamp:    buildRamp( s );    break;
      case kDrop:    buildDrop( s );    break;
      default:       buildTower( s );   break;
   }

   topIndex = -1;
   for( std::size_t i = 0; i < simBodies.size(); ++i )
      if( !simBodies[i].body->isFixed()
          && ( topIndex < 0 || simBodies[i].body->getPosition()[2] > simBodies[topIndex].body->getPosition()[2] ) )
         topIndex = static_cast<int>( i );
   if( topIndex >= 0 )
      topStart = simBodies[topIndex].body->getPosition();

   overlay.normalLength = 0.3 * s;
   overlay.pointRadius  = 0.03 * s;

   updateMirror();
   updateContacts();
   frameCamera();
}


//=================================================================================================
//
//  STEPPING
//
//=================================================================================================

void step( int n )
{
   if( n <= 0 || !simClock.error.empty() )
      return;
   std::vector<BodyID> bodies;
   for( const SimBody& sb : simBodies )
      bodies.push_back( sb.body );
   simctl::advance( n, bodies, simClock );
   updateMirror();
   updateContacts();
   sampleDiagnostics();
}


//=================================================================================================
//
//  GUI
//
//=================================================================================================

void drawSimulationWindow()
{
   ImGui::Begin( "Simulation" );

   if( simctl::drawStepControls() )
      buildScene();

   ImGui::Text( "t = %.4f s   steps: %ld   bodies: %d", simClock.time, simClock.steps, static_cast<int>( simBodies.size() ) );
   ImGui::Text( "solver: %d contacts, max penetration %.3e",
                static_cast<int>( theCollisionSystem()->getNumberOfContacts() ),
                static_cast<double>( theCollisionSystem()->getMaximumPenetration() ) );
   if( !simClock.error.empty() )
      ImGui::TextColored( ImVec4( 1.0f, 0.3f, 0.3f, 1.0f ), "%s", simClock.error.c_str() );
   if( simctl::mouseSpring().body() != nullptr )
      ImGui::TextDisabled( "dragging body %lu", static_cast<unsigned long>( simctl::mouseSpring().body()->getID() ) );
   else
      ImGui::TextDisabled( "Ctrl + left-drag a body to pull it" );

   simctl::drawSolverControls();

   if( ImGui::CollapsingHeader( "Display", ImGuiTreeNodeFlags_DefaultOpen ) ) {
      bool redraw = false;
      redraw |= ImGui::Checkbox( "contacts (narrow phase re-run)", &showContacts );
      if( ImGui::IsItemHovered( ImGuiHoveredFlags_DelayShort ) )
         ImGui::SetTooltip( "MaxContacts::collide() over all AABB-overlapping pairs of the current configuration.\n"
                            "Red = penetrating, yellow = within contactThreshold; normals point from g2 to g1." );
      const char* colorModes[] = { "sign of dist", "contact type" };
      redraw |= ImGui::Combo( "contact color", &overlay.colorBy, colorModes, 2 );
      const double zero = 0.0, big = 1.0e3;
      redraw |= ImGui::DragScalar( "normal length", ImGuiDataType_Double, &overlay.normalLength, 0.002f, &zero, &big, "%.4g" );
      redraw |= ImGui::DragScalar( "marker radius", ImGuiDataType_Double, &overlay.pointRadius, 0.0005f, &zero, &big, "%.4g" );
      redraw |= ImGui::Checkbox( "color bodies by speed", &colorBySpeed );
      if( colorBySpeed )
         redraw |= ImGui::DragScalar( "red at speed [m/s]", ImGuiDataType_Double, &speedScale, 0.01f, &zero, &big, "%.3g" );
      if( redraw ) {
         updateMirror();
         updateContacts();
      }
      if( ImGui::Button( "re-frame camera" ) )
         frameCamera();
   }
   ImGui::End();
}


void drawScenarioWindow()
{
   ImGui::Begin( "Scenario (applied on Reset)" );
   Scenario& s = staged;
   ImGui::Combo( "scenario", &s.kind, kScenarioNames, kNumScenarios );

   const char* countLabel = "tower height";
   switch( s.kind ) {
      case kPyramid:    countLabel = "bottom row boxes";         break;
      case kSquareWall: countLabel = "boxes per row and column"; break;
      case kWall:    countLabel = "rows";         break;
      case kMixed:
      case kCylinderStack: countLabel = "stack height"; break;
      case kRamp:    countLabel = "boxes (x2)";   break;
      case kDrop:    countLabel = "bodies";       break;
      default: break;
   }
   ImGui::SliderInt( countLabel, &s.count, 1, 30 );
   if( s.kind == kWall )
      ImGui::SliderInt( "columns", &s.wallColumns, 1, 10 );
   if( s.kind == kRamp ) {
      const double angleLo = 0.0, angleHi = 60.0;
      ImGui::SliderScalar( "ramp angle [deg]", ImGuiDataType_Double, &s.rampAngle, &angleLo, &angleHi, "%.2f" );
      const double tanA = std::tan( s.rampAngle * kDegToRad );
      ImGui::TextDisabled( "tan(angle) = %.3f  vs  mu = %.3f  ->  boxes should %s", tanA, s.friction,
                           tanA > s.friction ? "SLIDE" : "STICK" );
   }
   if( s.kind == kDrop ) {
      ImGui::Combo( "shape", &s.dropShape, kDropShapeNames, kNumDropShapes );
      ImGui::InputDouble( "drop clearance [s]", &s.dropHeight, 0.0, 0.0, "%.3g" );
      ImGui::InputDouble( "tilt x [deg]", &s.dropTilt[0], 0.0, 0.0, "%.3g" );
      ImGui::InputDouble( "tilt y [deg]", &s.dropTilt[1], 0.0, 0.0, "%.3g" );
   }

   ImGui::InputDouble( "box size s", &s.size, 0.0, 0.0, "%.4g" );
   s.size = std::max( 1.0e-4, s.size );
   ImGui::InputDouble( "initial gap [s]", &s.gap, 0.0, 0.0, "%.4g" );
   if( s.kind == kPyramid || s.kind == kSquareWall ) {
      ImGui::InputDouble( "side gap [s]", &s.sideGap, 0.0, 0.0, "%.4g" );
      s.sideGap = std::max( 0.0, s.sideGap );
      if( ImGui::IsItemHovered( ImGuiHoveredFlags_DelayShort ) )
         ImGui::SetTooltip( "Gap between neighbours in a row; 0 = touching. The layout uses each box's width\n"
                            "after its yaw jitter, so neighbours never start overlapping." );
   }
   ImGui::InputDouble( "lateral jitter [s]", &s.jitter, 0.0, 0.0, "%.4g" );
   if( ( s.kind == kPyramid || s.kind == kSquareWall ) && ImGui::IsItemHovered( ImGuiHoveredFlags_DelayShort ) )
      ImGui::SetTooltip( "For the walls: across the wall (y) only." );
   ImGui::InputDouble( "yaw jitter [deg]", &s.yawJitter, 0.0, 0.0, "%.4g" );
   ImGui::InputInt( "random seed", &s.seed );

   ImGui::SeparatorText( "material" );
   ImGui::InputDouble( "friction mu (pair)", &s.friction, 0.0, 0.0, "%.3g" );
   ImGui::InputDouble( "restitution", &s.restitution, 0.0, 0.0, "%.3g" );
   ImGui::InputDouble( "density", &s.density, 0.0, 0.0, "%.4g" );

   if( ImGui::Button( "Reset with these values" ) )
      buildScene();
   ImGui::End();
}


void drawPlots()
{
   ImGui::Begin( "Diagnostics" );
   const int n = static_cast<int>( tBuf.size() );
   const ImPlotAxisFlags fit = ImPlotAxisFlags_AutoFit;

   if( ImPlot::BeginPlot( "kinetic energy", ImVec2( -1, 170 ) ) ) {
      ImPlot::SetupAxes( "t [s]", "E_kin [J]", fit, fit );
      if( n > 0 ) ImPlot::PlotLine( "E_kin", tBuf.data(), keBuf.data(), n );
      ImPlot::EndPlot();
   }
   if( ImPlot::BeginPlot( "penetration", ImVec2( -1, 170 ) ) ) {
      ImPlot::SetupAxes( "t [s]", "max penetration", fit, fit );
      if( n > 0 ) ImPlot::PlotLine( "solver", tBuf.data(), penBuf.data(), n );
      ImPlot::EndPlot();
   }
   if( ImPlot::BeginPlot( "contact count", ImVec2( -1, 170 ) ) ) {
      ImPlot::SetupAxes( "t [s]", "contacts", fit, fit );
      if( n > 0 ) {
         ImPlot::PlotStairs( "solver", tBuf.data(), solverContactsBuf.data(), n );
         ImPlot::PlotStairs( "overlay (post-step)", tBuf.data(), overlayContactsBuf.data(), n );
      }
      ImPlot::EndPlot();
   }
   if( ImPlot::BeginPlot( "top body drift", ImVec2( -1, 170 ) ) ) {
      ImPlot::SetupAxes( "t [s]", "|x - x0|", fit, fit );
      if( n > 0 ) ImPlot::PlotLine( "top body", tBuf.data(), driftBuf.data(), n );
      ImPlot::EndPlot();
   }
   ImGui::TextDisabled( "a resting stack: E_kin -> 0, flat contact count, drift ~ 0" );
   ImGui::End();
}


} // namespace


//=================================================================================================
//
//  MODE INTERFACE
//
//=================================================================================================

namespace stacklab {

void activate()
{
   polyscope::options::groundPlaneMode       = polyscope::GroundPlaneMode::TileReflection;
   polyscope::options::groundPlaneHeightMode = polyscope::GroundPlaneHeightMode::Manual;
   polyscope::options::groundPlaneHeight     = 0.0f;   // coincides with the pe ground plane
   simctl::applySolverKnobs();
   buildScene();
}


void loadScenario( int index )
{
   staged.kind = std::max( 0, std::min( index, kNumScenarios - 1 ) );
   buildScene();
}


void advance( int steps )
{
   step( steps );
}


void frame()
{
   if( simctl::processKeys() )
      buildScene();

   std::vector<simctl::MouseSpring::Target> targets;
   for( const SimBody& sb : simBodies )
      targets.push_back( simctl::MouseSpring::Target{ sb.mesh, sb.body } );
   simctl::mouseSpring().process( targets, 0.01 * active.size );

   // Queued single steps run here, so step/draw ordering stays trivial.
   step( simctl::takeFrameSteps() );

   drawSimulationWindow();
   drawScenarioWindow();
   drawPlots();
}


bool smokeTest()
{
   bool ok = true;
   const int steps = 500;
   std::printf( "\nStack Lab: %d steps of dt = %.1e per scenario\n", steps, simctl::controls().dt );
   for( int k = 0; k < kNumScenarios; ++k ) {
      loadScenario( k );
      const int n = ( k == kDrop ) ? 3 * steps : steps;   // the drop needs time to land and settle
      for( int i = 0; i < n / 50; ++i ) {
         step( 50 );
         polyscope::frameTick();
         if( k == kDrop && topIndex >= 0 && i % 3 == 2 )
            std::printf( "   t %.2f  top z %+.4f  v_z %+.4f  overlay contacts %d\n", simClock.time,
                         static_cast<double>( simBodies[topIndex].body->getPosition()[2] ),
                         static_cast<double>( simBodies[topIndex].body->getLinearVel()[2] ),
                         static_cast<int>( contactLog.entries.size() ) );
      }
      std::printf( "scenario %d %-28s bodies %2d  E_kin %.3e  max pen %.3e  solver contacts %3d  overlay %3d  top drift %.3e%s%s\n",
                   k, kScenarioNames[k], static_cast<int>( simBodies.size() ), kineticEnergy(),
                   static_cast<double>( theCollisionSystem()->getMaximumPenetration() ),
                   static_cast<int>( theCollisionSystem()->getNumberOfContacts() ),
                   static_cast<int>( contactLog.entries.size() ), driftBuf.empty() ? 0.0 : driftBuf.back(),
                   simClock.error.empty() ? "" : "  ERROR: ", simClock.error.c_str() );
      if( !simClock.error.empty() )
         ok = false;
   }

   // The walls must never start overlapping, also with touching neighbours and random yaw: an
   // initial overlap is corrected with a velocity and launches the boxes.
   {
      const Scenario saved = staged;
      bool touching = true;
      for( int kind : { static_cast<int>( kPyramid ), static_cast<int>( kSquareWall ) } ) {
         staged           = Scenario();
         staged.kind      = kind;
         staged.count     = 5;
         staged.sideGap   = 0.0;
         staged.yawJitter = 10.0;
         staged.jitter    = 0.05;
         buildScene();
         real deepest = 0;
         for( const viewer::ContactLog::Entry& c : contactLog.entries )
            deepest = std::min( deepest, c.dist );
         std::printf( "%s, side gap 0, yaw jitter 10 deg: %d bodies, deepest initial contact %.3e -> %s\n",
                      kScenarioNames[kind], static_cast<int>( simBodies.size() ), static_cast<double>( deepest ),
                      deepest >= real(-1e-9) ? "no overlap" : "OVERLAP" );
         touching = touching && deepest >= real(-1e-9);
      }
      ok = ok && touching;
      staged = saved;
   }

   // Cylinders dropped onto the ground: before collideCylinderPlane() was implemented they fell
   // through it. After 3 s they must rest on it (centre at least r = 0.5 s above the ground).
   const Scenario saved = staged;
   staged.kind      = kDrop;
   staged.dropShape = kDropCylinder;
   buildScene();
   step( 1500 );
   double lowest = 1.0e30;
   for( const SimBody& sb : simBodies )
      lowest = std::min( lowest, static_cast<double>( sb.body->getPosition()[2] ) );
   const bool resting = simClock.error.empty() && lowest > 0.49 * active.size && kineticEnergy() < 1.0e-6;
   std::printf( "cylinder drop: lowest centre z %.4f (r = %.2f), E_kin %.3e -> %s\n", lowest,
                0.5 * active.size, kineticEnergy(), resting ? "resting on the ground" : "FAILED" );
   ok = ok && resting;
   staged = saved;
   buildScene();

   return ok;
}

} // namespace stacklab
