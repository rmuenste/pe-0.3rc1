//=================================================================================================
/*!
 *  \file tools/contact_viewer/pair_lab.cpp
 *  \brief Pair Lab mode of the contact viewer: static narrow-phase inspection of two bodies
 *
 *  Two bodies, no time stepping. Every pose or shape edit re-runs
 *  pe::detection::fine::MaxContacts::collide() on the pair into a recording container
 *  (ContactOverlay.h) and mirrors the result: contact points, normals, a contact table, the
 *  reversed-dispatch-order comparison, the support-point witnesses along each contact normal,
 *  and a one-degree-of-freedom sweep of contact count / minimum distance.
 */
//=================================================================================================

#include <pe/system/WarningDisable.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <pe/core.h>
#include <pe/core/detection/fine/MaxContacts.h>

#include "glm/glm.hpp"
#include "polyscope/curve_network.h"
#include "polyscope/pick.h"
#include "polyscope/point_cloud.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

#include "implot.h"

#include "ContactOverlay.h"
#include "PairLab.h"
#include "ShapeMeshes.h"

using namespace pe;
using pe::detection::fine::MaxContacts;


namespace {

const double kPi      = 3.14159265358979323846;
const double kDegToRad = kPi / 180.0;


//=================================================================================================
//
//  PAIR DESCRIPTION
//
//=================================================================================================

enum ShapeKind { kSphere, kBox, kCapsule, kCylinder, kEllipsoid, kPlane, kNumShapes };

const char* const kShapeNames[kNumShapes] = { "sphere", "box", "capsule", "cylinder", "ellipsoid", "plane" };
const char* const kBodyNames[2]           = { "body A", "body B" };
const char* const kDofNames[6]            = { "pos x", "pos y", "pos z", "rot x [deg]", "rot y [deg]", "rot z [deg]" };

//! Shape, size and pose of one body. The orientation is stored as pe's Euler angles
//! (Quat( xangle, yangle, zangle ): rotations applied in the order x, y, z), in degrees.
struct BodySpec {
   int    kind        = kBox;
   double radius      = 0.5;                  // sphere, capsule, cylinder
   double length      = 1.0;                  // capsule (cylinder part) and cylinder, along body x
   double lengths[3]  = { 1.0, 1.0, 1.0 };    // box side lengths
   double semiAxes[3] = { 0.5, 0.25, 0.15 };  // ellipsoid
   double pos[3]      = { 0.0, 0.0, 0.0 };
   double eulerDeg[3] = { 0.0, 0.0, 0.0 };

   BodySpec& at( double x, double y, double z )      { pos[0] = x; pos[1] = y; pos[2] = z; return *this; }
   BodySpec& rotated( double x, double y, double z ) { eulerDeg[0] = x; eulerDeg[1] = y; eulerDeg[2] = z; return *this; }

   //! Pose degree of freedom \a k: 0-2 position, 3-5 Euler angles.
   double& dof( int k ) { return k < 3 ? pos[k] : eulerDeg[k-3]; }
};

BodySpec sphereSpec( double r )                          { BodySpec s; s.kind = kSphere;   s.radius = r; return s; }
BodySpec capsuleSpec( double r, double l )               { BodySpec s; s.kind = kCapsule;  s.radius = r; s.length = l; return s; }
BodySpec cylinderSpec( double r, double l )              { BodySpec s; s.kind = kCylinder; s.radius = r; s.length = l; return s; }
BodySpec planeSpec()                                     { BodySpec s; s.kind = kPlane; return s; }
BodySpec boxSpec( double lx, double ly, double lz )      { BodySpec s; s.kind = kBox; s.lengths[0] = lx; s.lengths[1] = ly; s.lengths[2] = lz; return s; }
BodySpec ellipsoidSpec( double a, double b, double c )   { BodySpec s; s.kind = kEllipsoid; s.semiAxes[0] = a; s.semiAxes[1] = b; s.semiAxes[2] = c; return s; }

struct Preset {
   const char* name;
   BodySpec    a;
   BodySpec    b;
};

//! Configurations that stress the contact manifold generation. All penetrate by about 0.01.
const std::vector<Preset>& presets()
{
   static const std::vector<Preset> list{
      { "box on box: face-face (stack)",      boxSpec( 1, 1, 1 ), boxSpec( 1, 1, 1 ).at( 0.0, 0.0, 0.99 ) },
      { "box on box: face-face, offset + yaw", boxSpec( 1, 1, 1 ), boxSpec( 1, 1, 1 ).at( 0.3, 0.2, 0.99 ).rotated( 0, 0, 30 ) },
      { "box on box: edge-edge (crossed)",     boxSpec( 1, 1, 1 ).rotated( 45, 0, 0 ),
                                               boxSpec( 1, 1, 1 ).at( 0.0, 0.0, 1.4042 ).rotated( 0, 45, 0 ) },
      { "box on box: corner-face",             boxSpec( 1, 1, 1 ),
                                               boxSpec( 1, 1, 1 ).at( 0.0, 0.0, 1.3560 ).rotated( 45, -35.2644, 0 ) },
      { "capsule lying on box face",           boxSpec( 1, 1, 1 ), capsuleSpec( 0.25, 1.0 ).at( 0.0, 0.0, 0.74 ) },
      { "sphere on box edge",                  boxSpec( 1, 1, 1 ), sphereSpec( 0.5 ).at( 0.85, 0.0, 0.85 ) },
      { "box on plane",                        planeSpec(), boxSpec( 1, 1, 1 ).at( 0.0, 0.0, 0.49 ) },
      { "cylinder flat on plane",              planeSpec(), cylinderSpec( 0.5, 1.0 ).at( 0.0, 0.0, 0.49 ).rotated( 0, 90, 0 ) },
      { "cylinder rim on plane",               planeSpec(), cylinderSpec( 0.5, 1.0 ).at( 0.0, 0.0, 0.673 ).rotated( 0, 30, 0 ) },
      { "cylinder lying on plane",             planeSpec(), cylinderSpec( 0.5, 1.0 ).at( 0.0, 0.0, 0.49 ) },
      { "ellipsoid on box face, off-centre",   ellipsoidSpec( 0.5, 0.25, 0.15 ), boxSpec( 1, 1, 1 ).at( 0.99, 0.1, 0.0 ) },
      { "ellipsoid pair, tilted",              ellipsoidSpec( 0.5, 0.25, 0.25 ),
                                               ellipsoidSpec( 0.5, 0.25, 0.25 ).at( 0.85, 0.3, 0.0 ).rotated( 0, 0, 30 ) },
   };
   return list;
}


//=================================================================================================
//
//  STATE
//
//=================================================================================================

BodySpec specs[2] = { presets()[0].a, presets()[0].b };
BodyID   bodies[2] = { nullptr, nullptr };
int      presetIndex = 0;

polyscope::SurfaceMesh* meshes[2]      = { nullptr, nullptr };
glm::mat4               meshPose[2]    = { glm::mat4( 1.0f ), glm::mat4( 1.0f ) };  // last transform pushed
bool                    gizmo[2]       = { false, false };
const glm::vec3         kBodyColors[2] = { glm::vec3( 0.35f, 0.55f, 0.85f ), glm::vec3( 0.90f, 0.60f, 0.30f ) };

viewer::ContactLog     contactLog;       // collide( A, B )
viewer::ContactLog     swappedLog;       // collide( B, A )
viewer::OverlayOptions overlay;
std::string            collideError;
int                    selected      = -1;    // row of contactLog highlighted in table and view
bool                   showWitnesses = true;
bool                   dirty         = true;  // pose/shape/option changed: re-run collide + overlay

//! Agreement of collide( A, B ) with collide( B, A ), contacts matched by nearest position.
struct SwapCheck {
   bool   countMatch = true;
   double dPos       = 0.0;
   double dDist      = 0.0;
   double dNormal    = 0.0;   // |n1 - n2| with both normals oriented from B towards A
};
SwapCheck swapCheck;
const double kSwapTolerance = 1.0e-8;

// One-degree-of-freedom sweep
int    sweepBody    = 1;
int    sweepDof     = 2;
double sweepMin     = 0.9;
double sweepMax     = 1.1;
int    sweepSamples = 201;
bool   sweepAuto    = true;
std::vector<double> sweepX, sweepCount, sweepMinDist;


//=================================================================================================
//
//  BODIES
//
//=================================================================================================

BodyID createBody( const BodySpec& s, pe::id_t uid )
{
   // Materials survive World::clear(); create it exactly once.
   static const MaterialID material = createMaterial( "contact_viewer", 1.0, 0.0, 0.3, 0.3, 0.25, 200, 1000, 10, 11 );

   const Vec3 origin( 0, 0, 0 );
   switch( s.kind ) {
      case kSphere:    return createSphere   ( uid, origin, static_cast<real>( s.radius ), material );
      case kBox:       return createBox      ( uid, origin, Vec3( s.lengths[0], s.lengths[1], s.lengths[2] ), material );
      case kCapsule:   return createCapsule  ( uid, origin, static_cast<real>( s.radius ), static_cast<real>( s.length ), material );
      case kCylinder:  return createCylinder ( uid, origin, static_cast<real>( s.radius ), static_cast<real>( s.length ), material );
      case kEllipsoid: return createEllipsoid( uid, origin, static_cast<real>( s.semiAxes[0] ), static_cast<real>( s.semiAxes[1] ),
                                               static_cast<real>( s.semiAxes[2] ), material );
      default:         return createPlane    ( uid, Vec3( 0, 0, 1 ), origin, material );
   }
}


viewer::ShapeMesh createMesh( const BodySpec& s )
{
   switch( s.kind ) {
      case kSphere:    return viewer::makeEllipsoidMesh( s.radius, s.radius, s.radius );
      case kBox:       return viewer::makeBoxMesh( s.lengths[0], s.lengths[1], s.lengths[2] );
      case kCapsule:   return viewer::makeCapsuleMesh( s.radius, s.length );
      case kCylinder:  return viewer::makeCylinderMesh( s.radius, s.length );
      case kEllipsoid: return viewer::makeEllipsoidMesh( s.semiAxes[0], s.semiAxes[1], s.semiAxes[2] );
      default:         return viewer::makePlaneMesh( 2.5 );
   }
}


Quat specOrientation( const BodySpec& s )
{
   return Quat( static_cast<real>( s.eulerDeg[0] * kDegToRad ),
                static_cast<real>( s.eulerDeg[1] * kDegToRad ),
                static_cast<real>( s.eulerDeg[2] * kDegToRad ) );
}


//! Polyscope v2.3 exposes the per-structure transform gizmo only through the structure's own
//! options menu (Structure::transformGizmo is protected). A derived class may form a pointer
//! to the inherited member, which is all that is needed to toggle it from the Pair Lab window.
struct GizmoAccess : polyscope::Structure {
   static polyscope::TransformationGizmo polyscope::Structure::* member() { return &GizmoAccess::transformGizmo; }
};

void setGizmoEnabled( polyscope::Structure* structure, bool enabled )
{
   ( structure->*GizmoAccess::member() ).enabled = enabled;
}


//! Pushes the staged pose of body \a i to the engine only (used inside the sweep loop).
void setBodyPose( int i )
{
   bodies[i]->setPosition( Vec3( specs[i].pos[0], specs[i].pos[1], specs[i].pos[2] ) );
   bodies[i]->setOrientation( specOrientation( specs[i] ) );
}


void applyPose( int i )
{
   setBodyPose( i );
   meshPose[i] = viewer::bodyTransform( bodies[i] );
   meshes[i]->setTransform( meshPose[i] );
   dirty = true;
}


//! Shape or size changed: recreate both bodies and their mirror meshes.
void rebuildScene()
{
   contactLog.clear();    // the logs hold pointers into the world being cleared
   swappedLog.clear();
   theWorld()->clear();

   for( int i = 0; i < 2; ++i ) {
      bodies[i] = createBody( specs[i], static_cast<pe::id_t>( i + 1 ) );

      const viewer::ShapeMesh mesh = createMesh( specs[i] );
      meshes[i] = polyscope::registerSurfaceMesh( kBodyNames[i], mesh.vertices, mesh.faces );
      meshes[i]->setSurfaceColor( kBodyColors[i] );
      meshes[i]->setTransparency( 0.45f );   // contacts live inside the overlap region
      meshes[i]->setEdgeWidth( ( specs[i].kind == kBox || specs[i].kind == kPlane ) ? 1.0 : 0.0 );
      setGizmoEnabled( meshes[i], gizmo[i] );
      applyPose( i );
   }
}


void selectPreset( int index )
{
   presetIndex = std::max( 0, std::min( index, static_cast<int>( presets().size() ) - 1 ) );
   specs[0]    = presets()[presetIndex].a;
   specs[1]    = presets()[presetIndex].b;
   selected    = -1;
   rebuildScene();
}


//! Inverse of Quat( xangle, yangle, zangle ) for a column-major transform whose upper 3x3
//! block may carry a scale (the Polyscope gizmo also scales): RotationMatrix::getEulerAnglesXYZ
//! on the normalized columns.
void poseFromTransform( const glm::mat4& T, BodySpec& s )
{
   double R[3][3];
   for( int c = 0; c < 3; ++c ) {
      const double len = std::sqrt( static_cast<double>( T[c][0]*T[c][0] + T[c][1]*T[c][1] + T[c][2]*T[c][2] ) );
      for( int r = 0; r < 3; ++r )
         R[r][c] = ( len > 0.0 ) ? T[c][r] / len : ( r == c ? 1.0 : 0.0 );
   }

   const double cy = std::sqrt( R[0][0]*R[0][0] + R[1][0]*R[1][0] );
   double x, y, z;
   if( cy > 1.0e-6 ) {
      x = std::atan2(  R[2][1], R[2][2] );
      y = std::atan2( -R[2][0], cy );
      z = std::atan2(  R[1][0], R[0][0] );
   }
   else {
      x = std::atan2( -R[1][2], R[1][1] );
      y = std::atan2( -R[2][0], cy );
      z = 0.0;
   }
   s.eulerDeg[0] = x / kDegToRad;
   s.eulerDeg[1] = y / kDegToRad;
   s.eulerDeg[2] = z / kDegToRad;
   for( int k = 0; k < 3; ++k )
      s.pos[k] = T[3][k];
}


//! A gizmo drag moves the mesh, not the body: adopt the mesh transform as the staged pose.
void readBackGizmos()
{
   for( int i = 0; i < 2; ++i ) {
      if( !gizmo[i] || meshes[i] == nullptr )
         continue;
      const glm::mat4 T = meshes[i]->getTransform();
      float diff = 0.0f;
      for( int c = 0; c < 4; ++c )
         for( int r = 0; r < 4; ++r )
            diff = std::max( diff, std::abs( T[c][r] - meshPose[i][c][r] ) );
      if( diff > 1.0e-6f ) {
         poseFromTransform( T, specs[i] );
         applyPose( i );   // also strips the scale the gizmo may have introduced
      }
   }
}


//=================================================================================================
//
//  CONTACT GENERATION
//
//=================================================================================================

void runCollide( BodyID b1, BodyID b2, viewer::ContactLog& log )
{
   log.clear();
   try {
      MaxContacts::collide( b1, b2, log );
   }
   catch( const std::exception& e ) {
      collideError = e.what();
   }
}


//! Contact normal oriented from body B towards body A, whichever of the two is g1.
Vec3 normalTowardsA( const viewer::ContactLog::Entry& c )
{
   return c.g1 == bodies[0] ? c.normal : -c.normal;
}


void compareDispatchOrders()
{
   swapCheck = SwapCheck();
   swapCheck.countMatch = ( contactLog.entries.size() == swappedLog.entries.size() );
   if( swappedLog.entries.empty() )
      return;

   for( const viewer::ContactLog::Entry& c : contactLog.entries ) {
      const viewer::ContactLog::Entry* best = nullptr;
      real bestSq = std::numeric_limits<real>::max();
      for( const viewer::ContactLog::Entry& o : swappedLog.entries ) {
         const real sq = ( o.pos - c.pos ).sqrLength();
         if( sq < bestSq ) { bestSq = sq; best = &o; }
      }
      swapCheck.dPos    = std::max( swapCheck.dPos,    static_cast<double>( std::sqrt( bestSq ) ) );
      swapCheck.dDist   = std::max( swapCheck.dDist,   static_cast<double>( std::abs( best->dist - c.dist ) ) );
      swapCheck.dNormal = std::max( swapCheck.dNormal, static_cast<double>( ( normalTowardsA( *best ) - normalTowardsA( c ) ).length() ) );
   }
}


bool hasSupport( pe::GeomID g )
{
   return g->getType() != planeType;   // RigidBody::support() is undefined for the infinite plane
}


//! Signed gap between the two bodies' support points along the contact normal: the extent of
//! g1 towards g2 minus the extent of g2 towards g1. For a correct normal of two convex bodies
//! this equals the contact distance; a mismatch means normal and depth are inconsistent.
real supportGap( const viewer::ContactLog::Entry& c )
{
   return trans( c.normal ) * ( c.g1->support( -c.normal ) - c.g2->support( c.normal ) );
}


void drawWitnesses()
{
   std::vector<glm::vec3>                 nodes, colors;
   std::vector<std::array<std::size_t,2>> edges;

   if( showWitnesses ) {
      for( std::size_t k = 0; k < contactLog.entries.size(); ++k ) {
         const viewer::ContactLog::Entry& c = contactLog.entries[k];
         if( ( selected >= 0 && static_cast<int>( k ) != selected ) || !hasSupport( c.g1 ) || !hasSupport( c.g2 ) )
            continue;
         edges.push_back( { nodes.size(), nodes.size() + 1 } );
         nodes.push_back ( viewer::toGlm( c.g1->support( -c.normal ) ) );
         nodes.push_back ( viewer::toGlm( c.g2->support(  c.normal ) ) );
         colors.push_back( kBodyColors[ c.g1 == bodies[0] ? 0 : 1 ] );
         colors.push_back( kBodyColors[ c.g2 == bodies[0] ? 0 : 1 ] );
      }
   }

   if( nodes.empty() ) {
      polyscope::removeStructure( "support witnesses", /*errorIfAbsent=*/false );
      polyscope::removeStructure( "support witness points", /*errorIfAbsent=*/false );
      return;
   }
   polyscope::registerCurveNetwork( "support witnesses", nodes, edges )
      ->setRadius( 0.25 * overlay.pointRadius, /*isRelative=*/false )
      ->setColor( glm::vec3( 0.15f, 0.15f, 0.15f ) );
   polyscope::PointCloud* cloud = polyscope::registerPointCloud( "support witness points", nodes );
   cloud->setPointRadius( 0.7 * overlay.pointRadius, /*isRelative=*/false );
   cloud->addColorQuantity( "body", colors )->setEnabled( true );
}


void drawSelection()
{
   if( selected < 0 || selected >= static_cast<int>( contactLog.entries.size() ) ) {
      polyscope::removeStructure( "selected contact", /*errorIfAbsent=*/false );
      return;
   }
   const std::vector<glm::vec3> point{ viewer::toGlm( contactLog.entries[selected].pos ) };
   polyscope::registerPointCloud( "selected contact", point )
      ->setPointRadius( 1.6 * overlay.pointRadius, /*isRelative=*/false )
      ->setPointColor( glm::vec3( 1.0f, 1.0f, 1.0f ) );
}


void runSweep()
{
   sweepX.clear();
   sweepCount.clear();
   sweepMinDist.clear();
   if( sweepSamples < 2 || !( sweepMax > sweepMin ) )
      return;

   BodySpec& spec = specs[sweepBody];
   const double saved = spec.dof( sweepDof );
   viewer::ContactLog log;

   for( int k = 0; k < sweepSamples; ++k ) {
      const double x = sweepMin + ( sweepMax - sweepMin ) * k / ( sweepSamples - 1 );
      spec.dof( sweepDof ) = x;
      setBodyPose( sweepBody );
      runCollide( bodies[0], bodies[1], log );

      double minDist = std::numeric_limits<double>::quiet_NaN();   // gaps in the curve = no contact
      for( const viewer::ContactLog::Entry& c : log.entries )
         minDist = std::isnan( minDist ) ? static_cast<double>( c.dist ) : std::min( minDist, static_cast<double>( c.dist ) );
      sweepX.push_back( x );
      sweepCount.push_back( static_cast<double>( log.entries.size() ) );
      sweepMinDist.push_back( minDist );
   }

   spec.dof( sweepDof ) = saved;
   setBodyPose( sweepBody );
}


void refresh()
{
   collideError.clear();
   runCollide( bodies[0], bodies[1], contactLog );
   runCollide( bodies[1], bodies[0], swappedLog );
   compareDispatchOrders();
   if( selected >= static_cast<int>( contactLog.entries.size() ) )
      selected = -1;

   viewer::drawContactOverlay( "contacts", contactLog, overlay );
   drawSelection();
   drawWitnesses();
   if( sweepAuto )
      runSweep();
   dirty = false;
}


//=================================================================================================
//
//  TEST CASE EXPORT
//
//=================================================================================================

std::string createCall( const BodySpec& s, int uid )
{
   std::ostringstream o;
   o << std::setprecision( 17 );
   const char* type[kNumShapes] = { "Sphere", "Box", "Capsule", "Cylinder", "Ellipsoid", "Plane" };
   const char  var = ( uid == 1 ) ? 'a' : 'b';
   o << "   " << type[s.kind] << "ID " << var << " = create" << type[s.kind] << "( " << uid << ", ";
   if( s.kind == kPlane )
      o << "Vec3( 0, 0, 1 ), ";
   o << "Vec3( " << s.pos[0] << ", " << s.pos[1] << ", " << s.pos[2] << " ), ";
   switch( s.kind ) {
      case kSphere:    o << s.radius << ", "; break;
      case kBox:       o << "Vec3( " << s.lengths[0] << ", " << s.lengths[1] << ", " << s.lengths[2] << " ), "; break;
      case kCapsule:
      case kCylinder:  o << s.radius << ", " << s.length << ", "; break;
      case kEllipsoid: o << s.semiAxes[0] << ", " << s.semiAxes[1] << ", " << s.semiAxes[2] << ", "; break;
      default: break;
   }
   o << "mat );\n";
   if( s.eulerDeg[0] != 0.0 || s.eulerDeg[1] != 0.0 || s.eulerDeg[2] != 0.0 )
      o << "   " << var << "->setOrientation( Quat( real(" << s.eulerDeg[0] * kDegToRad << "), real("
        << s.eulerDeg[1] * kDegToRad << "), real(" << s.eulerDeg[2] * kDegToRad << ") ) );   // Euler x, y, z [rad]\n";
   return o.str();
}


//! The current pair as a snippet for a tests/interface contact test, with the contacts the
//! narrow phase generated right now as comments (observed values, not verified expectations).
std::string testCaseSnippet()
{
   std::ostringstream o;
   o << std::setprecision( 17 );
   o << "   // Pair Lab: " << kShapeNames[specs[0].kind] << " (a) vs " << kShapeNames[specs[1].kind] << " (b)\n";
   o << createCall( specs[0], 1 ) << createCall( specs[1], 2 );
   o << "   ContactLog log;\n   MaxContacts::collide( a, b, log );\n";
   o << "   // observed: " << contactLog.entries.size() << " contact(s), normal points from g2 to g1\n";
   for( std::size_t k = 0; k < contactLog.entries.size(); ++k ) {
      const viewer::ContactLog::Entry& c = contactLog.entries[k];
      o << "   //   [" << k << "] " << viewer::kindName( c.kind ) << ", g1 = " << ( c.g1 == bodies[0] ? 'a' : 'b' )
        << ", dist " << c.dist << ", pos (" << c.pos[0] << ", " << c.pos[1] << ", " << c.pos[2]
        << "), normal (" << c.normal[0] << ", " << c.normal[1] << ", " << c.normal[2] << ")\n";
   }
   return o.str();
}


//=================================================================================================
//
//  GUI
//
//=================================================================================================

bool dragDoubles( const char* label, double* v, int n, float speed, double lo, double hi, const char* fmt )
{
   return ImGui::DragScalarN( label, ImGuiDataType_Double, v, n, speed, &lo, &hi, fmt );
}


//! Returns 1 for a pose edit, 2 for a shape/size edit (needs a scene rebuild).
int drawBodyControls( int i )
{
   BodySpec& s = specs[i];
   int change = 0;
   ImGui::PushID( i );

   if( ImGui::Combo( "shape", &s.kind, kShapeNames, kNumShapes ) )
      change = 2;
   switch( s.kind ) {
      case kSphere:
         if( dragDoubles( "radius", &s.radius, 1, 0.002f, 1.0e-3, 10.0, "%.4f" ) ) change = 2;
         break;
      case kBox:
         if( dragDoubles( "side lengths", s.lengths, 3, 0.002f, 1.0e-3, 10.0, "%.4f" ) ) change = 2;
         break;
      case kCapsule:
      case kCylinder:
         if( dragDoubles( "radius", &s.radius, 1, 0.002f, 1.0e-3, 10.0, "%.4f" ) ) change = 2;
         if( dragDoubles( "length (body x)", &s.length, 1, 0.002f, 1.0e-3, 10.0, "%.4f" ) ) change = 2;
         break;
      case kEllipsoid:
         if( dragDoubles( "semi-axes", s.semiAxes, 3, 0.002f, 1.0e-3, 10.0, "%.4f" ) ) change = 2;
         break;
      default:
         ImGui::TextDisabled( "normal = body z axis" );
         break;
   }

   if( dragDoubles( "position", s.pos, 3, 0.002f, -10.0, 10.0, "%.5f" ) )
      change = std::max( change, 1 );
   if( dragDoubles( "rot x,y,z [deg]", s.eulerDeg, 3, 0.2f, -180.0, 180.0, "%.3f" ) )
      change = std::max( change, 1 );
   if( ImGui::IsItemHovered( ImGuiHoveredFlags_DelayShort ) )
      ImGui::SetTooltip( "pe Euler angles: Quat( x, y, z ), applied in the order x, y, z.\nCtrl+click to type a value; hold Alt while dragging for fine steps." );

   if( ImGui::Checkbox( "gizmo", &gizmo[i] ) )
      setGizmoEnabled( meshes[i], gizmo[i] );
   ImGui::SameLine();
   if( ImGui::Button( "reset rotation" ) ) {
      s.rotated( 0, 0, 0 );
      change = std::max( change, 1 );
   }

   ImGui::PopID();
   return change;
}


void drawPairWindow()
{
   ImGui::Begin( "Pair Lab" );

   if( ImGui::BeginCombo( "preset", presets()[presetIndex].name ) ) {
      for( int k = 0; k < static_cast<int>( presets().size() ); ++k )
         if( ImGui::Selectable( presets()[k].name, k == presetIndex ) )
            selectPreset( k );
      ImGui::EndCombo();
   }
   if( ImGui::Button( "swap A <-> B" ) ) {
      std::swap( specs[0], specs[1] );
      selected = -1;
      rebuildScene();
   }

   for( int i = 0; i < 2; ++i ) {
      if( !ImGui::CollapsingHeader( kBodyNames[i], ImGuiTreeNodeFlags_DefaultOpen ) )
         continue;
      const int change = drawBodyControls( i );
      if( change == 2 )
         rebuildScene();
      else if( change == 1 )
         applyPose( i );
   }

   if( ImGui::CollapsingHeader( "Overlay", ImGuiTreeNodeFlags_DefaultOpen ) ) {
      const char* colorModes[] = { "sign of dist", "contact type" };
      dirty |= ImGui::Combo( "color by", &overlay.colorBy, colorModes, 2 );
      dirty |= dragDoubles( "normal length", &overlay.normalLength, 1, 0.002f, 0.0, 1.0e6, "%.4g" );
      dirty |= ImGui::Checkbox( "scale normals by |dist|", &overlay.scaleByDist );
      dirty |= dragDoubles( "marker radius", &overlay.pointRadius, 1, 0.0005f, 1.0e-4, 1.0, "%.4f" );
      dirty |= ImGui::Checkbox( "support witnesses", &showWitnesses );
      if( ImGui::IsItemHovered( ImGuiHoveredFlags_DelayShort ) )
         ImGui::SetTooltip( "g1->support( -n ) and g2->support( n ) per contact (selected contact only when one is\n"
                            "selected), joined by a segment. Not available for planes. Against a flat face the\n"
                            "support point is not unique and pe returns a corner of that face." );
   }

   if( ImGui::Button( "copy as test case" ) ) {
      const std::string snippet = testCaseSnippet();
      ImGui::SetClipboardText( snippet.c_str() );
      std::cout << snippet << std::flush;
   }
   if( ImGui::IsItemHovered( ImGuiHoveredFlags_DelayShort ) )
      ImGui::SetTooltip( "C++ snippet of this pair to the clipboard and stdout" );

   ImGui::End();
}


void drawContactsWindow()
{
   ImGui::Begin( "Contacts" );

   ImGui::Text( "collide( A, B ): %d contact(s)   |   contactThreshold = %.3g",
                static_cast<int>( contactLog.entries.size() ), static_cast<double>( contactThreshold ) );
   if( !collideError.empty() )
      ImGui::TextColored( ImVec4( 1.0f, 0.3f, 0.3f, 1.0f ), "collide() threw: %s", collideError.c_str() );

   const bool swapOk = swapCheck.countMatch && swapCheck.dPos <= kSwapTolerance
                    && swapCheck.dDist <= kSwapTolerance && swapCheck.dNormal <= kSwapTolerance;
   ImGui::TextColored( swapOk ? ImVec4( 0.3f, 0.9f, 0.4f, 1.0f ) : ImVec4( 1.0f, 0.3f, 0.3f, 1.0f ),
                       "collide( B, A ): %d contact(s), %s", static_cast<int>( swappedLog.entries.size() ),
                       swapOk ? "agrees" : "DIFFERS" );
   if( !contactLog.entries.empty() && !swappedLog.entries.empty() )
      ImGui::Text( "  max |d pos| %.3e   |d dist| %.3e   |d normal| %.3e", swapCheck.dPos, swapCheck.dDist, swapCheck.dNormal );

   const ImGuiTableFlags flags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit;
   if( !contactLog.entries.empty() && ImGui::BeginTable( "contacts", 6, flags ) ) {
      ImGui::TableSetupColumn( "#" );
      ImGui::TableSetupColumn( "type" );
      ImGui::TableSetupColumn( "g1" );
      ImGui::TableSetupColumn( "dist" );
      ImGui::TableSetupColumn( "position" );
      ImGui::TableSetupColumn( "normal (g2 -> g1)" );
      ImGui::TableHeadersRow();
      for( int k = 0; k < static_cast<int>( contactLog.entries.size() ); ++k ) {
         const viewer::ContactLog::Entry& c = contactLog.entries[k];
         ImGui::TableNextRow();
         ImGui::TableNextColumn();
         char label[16];
         std::snprintf( label, sizeof( label ), "%d", k );
         if( ImGui::Selectable( label, k == selected, ImGuiSelectableFlags_SpanAllColumns ) ) {
            selected = ( k == selected ) ? -1 : k;
            dirty = true;
         }
         ImGui::TableNextColumn(); ImGui::TextUnformatted( viewer::kindName( c.kind ) );
         ImGui::TableNextColumn(); ImGui::TextUnformatted( c.g1 == bodies[0] ? "A" : "B" );
         ImGui::TableNextColumn(); ImGui::Text( "%+.6e", static_cast<double>( c.dist ) );
         ImGui::TableNextColumn(); ImGui::Text( "%+.5f %+.5f %+.5f", static_cast<double>( c.pos[0] ), static_cast<double>( c.pos[1] ), static_cast<double>( c.pos[2] ) );
         ImGui::TableNextColumn(); ImGui::Text( "%+.5f %+.5f %+.5f", static_cast<double>( c.normal[0] ), static_cast<double>( c.normal[1] ), static_cast<double>( c.normal[2] ) );
      }
      ImGui::EndTable();
   }

   if( selected >= 0 && selected < static_cast<int>( contactLog.entries.size() ) ) {
      const viewer::ContactLog::Entry& c = contactLog.entries[selected];
      ImGui::Separator();
      ImGui::Text( "contact %d: dist %+.12e", selected, static_cast<double>( c.dist ) );
      if( hasSupport( c.g1 ) && hasSupport( c.g2 ) ) {
         const double gap = static_cast<double>( supportGap( c ) );
         ImGui::Text( "support gap along n: %+.12e   (gap - dist = %+.3e)", gap, gap - static_cast<double>( c.dist ) );
         if( ImGui::IsItemHovered( ImGuiHoveredFlags_DelayShort ) )
            ImGui::SetTooltip( "n . ( g1->support( -n ) - g2->support( n ) ). Equals dist when normal and depth of a\n"
                               "convex pair are consistent; a multi-point manifold reports it for the deepest point." );
      }
   }
   else if( !contactLog.entries.empty() ) {
      ImGui::TextDisabled( "click a row or a contact marker for details" );
   }

   ImGui::End();
}


void drawSweepWindow()
{
   ImGui::Begin( "Sweep" );

   bool changed = false;
   changed |= ImGui::Combo( "body", &sweepBody, kBodyNames, 2 );
   changed |= ImGui::Combo( "degree of freedom", &sweepDof, kDofNames, 6 );
   changed |= ImGui::InputDouble( "from", &sweepMin, 0.0, 0.0, "%.6g" );
   changed |= ImGui::InputDouble( "to", &sweepMax, 0.0, 0.0, "%.6g" );
   changed |= ImGui::SliderInt( "samples", &sweepSamples, 11, 2001 );
   if( ImGui::Button( "centre range on current value" ) ) {
      const double half = 0.5 * ( sweepMax - sweepMin );
      const double x    = specs[sweepBody].dof( sweepDof );
      sweepMin = x - half;
      sweepMax = x + half;
      changed  = true;
   }
   ImGui::Checkbox( "auto-refresh", &sweepAuto );
   ImGui::SameLine();
   if( ImGui::Button( "run" ) || ( changed && sweepAuto ) )
      runSweep();

   // The drag line is the scrubber: moving it poses the body at that parameter value.
   double       cursor = specs[sweepBody].dof( sweepDof );
   bool         moved  = false;
   const int    n      = static_cast<int>( sweepX.size() );
   const ImVec4 cursorColor( 1.0f, 0.8f, 0.1f, 1.0f );

   if( ImPlot::BeginPlot( "contact count", ImVec2( -1, 180 ) ) ) {
      ImPlot::SetupAxes( kDofNames[sweepDof], "contacts", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit );
      if( n > 0 )
         ImPlot::PlotStairs( "count", sweepX.data(), sweepCount.data(), n );
      moved |= ImPlot::DragLineX( 0, &cursor, cursorColor );
      ImPlot::EndPlot();
   }
   if( ImPlot::BeginPlot( "minimum dist", ImVec2( -1, 220 ) ) ) {
      ImPlot::SetupAxes( kDofNames[sweepDof], "min dist", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit );
      if( n > 0 )
         ImPlot::PlotLine( "min dist", sweepX.data(), sweepMinDist.data(), n );
      moved |= ImPlot::DragLineX( 0, &cursor, cursorColor );
      ImPlot::EndPlot();
   }
   ImGui::TextDisabled( "drag the yellow line to scrub the pose; gaps in min dist = no contact" );

   if( moved ) {
      specs[sweepBody].dof( sweepDof ) = cursor;
      applyPose( sweepBody );
   }

   ImGui::End();
}


//! A click on a contact marker in the 3D view selects its table row.
void processPick()
{
   const std::pair<polyscope::Structure*, size_t> hit = polyscope::pick::getSelection();
   if( hit.first == nullptr || !polyscope::hasPointCloud( "contacts" ) || hit.first != polyscope::getPointCloud( "contacts" ) )
      return;
   if( static_cast<int>( hit.second ) != selected && hit.second < contactLog.entries.size() ) {
      selected = static_cast<int>( hit.second );
      dirty    = true;
   }
   polyscope::pick::resetSelection();   // consumed; the overlay is re-registered on every refresh
}

} // namespace


//=================================================================================================
//
//  MODE INTERFACE
//
//=================================================================================================

namespace pairlab {

void activate()
{
   theWorld()->setGravity( 0.0, 0.0, 0.0 );   // the pair is posed by hand and never stepped
   polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::None;   // pe planes are meshes here

   // The bodies stay within a few units of the origin; fixed extents keep the camera and the
   // absolute marker sizes stable while shapes are edited.
   polyscope::options::automaticallyComputeSceneExtents = false;
   polyscope::state::lengthScale = 3.0f;
   polyscope::state::boundingBox =
      std::tuple<glm::vec3, glm::vec3>{ glm::vec3( -1.5f, -1.5f, -1.5f ), glm::vec3( 1.5f, 1.5f, 1.5f ) };

   rebuildScene();   // keeps the pair as it was posed before a mode switch
   // Oblique view: the axis-aligned home view hides one dimension of box-box manifolds.
   polyscope::view::lookAt( glm::vec3( 2.6f, -3.2f, 2.0f ), glm::vec3( 0.0f, 0.0f, 0.4f ) );
}


void loadPreset( int index )
{
   selectPreset( index );
}


void frame()
{
   readBackGizmos();
   processPick();

   drawPairWindow();
   if( dirty )
      refresh();
   drawContactsWindow();
   drawSweepWindow();
}


bool smokeTest()
{
   bool ok = true;

   for( int k = 0; k < static_cast<int>( presets().size() ); ++k ) {
      selectPreset( k );
      polyscope::frameTick();
      std::printf( "preset %2d  %-40s %d contact(s), swapped %d, %s%s\n", k, presets()[k].name,
                   static_cast<int>( contactLog.entries.size() ), static_cast<int>( swappedLog.entries.size() ),
                   swapCheck.countMatch ? "counts agree" : "COUNTS DIFFER",
                   collideError.empty() ? "" : ( " ERROR: " + collideError ).c_str() );
      for( const viewer::ContactLog::Entry& c : contactLog.entries )
         std::printf( "            %-12s dist %+.6e  pos (%+.4f %+.4f %+.4f)  n (%+.4f %+.4f %+.4f)\n",
                      viewer::kindName( c.kind ), static_cast<double>( c.dist ),
                      static_cast<double>( c.pos[0] ), static_cast<double>( c.pos[1] ), static_cast<double>( c.pos[2] ),
                      static_cast<double>( c.normal[0] ), static_cast<double>( c.normal[1] ), static_cast<double>( c.normal[2] ) );
      // An empty preset is reported, not failed: this checks the viewer, and the contact
      // routines themselves are covered by tests/interface (collideCylinderPlane() is a stub).
      if( contactLog.entries.empty() )
         std::printf( "            note: no contact generated\n" );
      if( !collideError.empty() )
         ok = false;
      if( static_cast<int>( sweepX.size() ) != sweepSamples ) {
         std::printf( "  FAILED: sweep produced %d of %d samples\n", static_cast<int>( sweepX.size() ), sweepSamples );
         ok = false;
      }
   }

   // Every shape pair through the dispatch and a full GUI frame, deeply overlapping: the
   // printed matrix of contact counts shows at a glance which pairs generate nothing.
   int pairs = 0;
   std::printf( "\ncontacts per pair (row = A, column = B)\n%10s", "" );
   for( int b = 0; b < kNumShapes; ++b )
      std::printf( "%10s", kShapeNames[b] );
   for( int a = 0; a < kNumShapes; ++a ) {
      std::printf( "\n%10s", kShapeNames[a] );
      for( int b = 0; b < kNumShapes; ++b ) {
         specs[0] = BodySpec();
         specs[1] = BodySpec().at( 0.2, 0.05, 0.12 ).rotated( 20, -15, 40 );   // overlaps A for every shape
         specs[0].kind = a;
         specs[1].kind = b;
         selected = 0;
         rebuildScene();
         polyscope::frameTick();
         ++pairs;
         std::printf( "%10d", static_cast<int>( contactLog.entries.size() ) );
         if( !collideError.empty() ) {
            std::printf( "\n  FAILED: %s vs %s threw: %s\n", kShapeNames[a], kShapeNames[b], collideError.c_str() );
            ok = false;
         }
      }
   }
   std::printf( "\n%d shape pairs dispatched\n", pairs );

   // Gizmo read-back: mesh transform -> Euler angles must invert Quat( x, y, z ).
   double worst = 0.0;
   const double eulers[][3] = { { 0, 0, 0 }, { 30, 0, 0 }, { 0, 40, 0 }, { 0, 0, 50 }, { 45, -35.2644, 0 },
                                { -120, 60, 170 }, { 10, -80, -95 } };
   for( const auto& e : eulers ) {
      specs[1].at( 0.3, -0.2, 0.9 ).rotated( e[0], e[1], e[2] );
      applyPose( 1 );
      BodySpec back;
      poseFromTransform( meshPose[1], back );
      for( int k = 0; k < 3; ++k ) {
         worst = std::max( worst, std::abs( back.eulerDeg[k] - e[k] ) );
         worst = std::max( worst, std::abs( back.pos[k] - specs[1].pos[k] ) );
      }
   }
   std::printf( "pose round trip: worst deviation %.3e (deg / length units)\n", worst );
   if( worst > 1.0e-3 ) {
      std::printf( "  FAILED: gizmo read-back does not invert the pe Euler convention\n" );
      ok = false;
   }

   return ok;
}

} // namespace pairlab
