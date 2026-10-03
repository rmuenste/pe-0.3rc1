//=================================================================================================
/*!
 *  \file tools/contact_viewer/pair_lab.cpp
 *  \brief Pair Lab mode of the contact viewer: narrow-phase inspection of two bodies
 *
 *  Two bodies over an optional ground plane. Every pose or shape edit re-runs
 *  pe::detection::fine::MaxContacts::collide() on the pair into a recording container
 *  (ContactOverlay.h) and mirrors the result: contact points, normals, a contact table, the
 *  reversed-dispatch-order comparison, the support-point witnesses along each contact normal,
 *  and a one-degree-of-freedom sweep of contact count / minimum distance.
 *
 *  The posed pair (plus per-body "fixed" flags and initial velocities) is also the initial state
 *  of a simulation driven by the shared controls (SimControls.h): run, pause, step, Reset back
 *  to the posed state, Ctrl + left-drag. While time is past zero the pose editors and the sweep
 *  are locked (they act on the posed state); the contact analysis follows the current state.
 */
//=================================================================================================

#include <pe/system/WarningDisable.h>

#include <algorithm>
#include <array>
#include <chrono>
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
#ifdef PE_USE_CGAL
#include <pe/core/detection/fine/DistanceMap.h>
#endif

#include "glm/glm.hpp"
#include "polyscope/curve_network.h"
#include "polyscope/pick.h"
#include "polyscope/point_cloud.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/volume_grid.h"

#include "implot.h"

#include "ContactOverlay.h"
#include "PairLab.h"
#include "ShapeMeshes.h"
#include "SimControls.h"

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

enum ShapeKind { kSphere, kBox, kCapsule, kCylinder, kEllipsoid, kPlane, kMesh, kNumShapes };

const char* const kShapeNames[kNumShapes] = { "sphere", "box", "capsule", "cylinder", "ellipsoid", "plane", "mesh" };
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
   // Initial state of the simulation (planes are always fixed).
   bool   fixed       = false;
   double vel[3]      = { 0.0, 0.0, 0.0 };
   double angVel[3]   = { 0.0, 0.0, 0.0 };   // [rad/s]
   // Triangle mesh: a torus (hole axis = body z) with a DistanceMap (needs CGAL).
   double torusMajor   = 0.6;
   double torusMinor   = 0.25;
   int    torusSegs[2] = { 48, 24 };
   bool   distanceMap  = true;
   int    dmResolution = 50;
   int    dmTolerance  = 5;

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
BodySpec torusSpec( double R, double r )                 { BodySpec s; s.kind = kMesh; s.torusMajor = R; s.torusMinor = r; return s; }

struct Preset {
   const char* name;
   BodySpec    a;
   BodySpec    b;
   double      groundLift = 0.0;   //!< > 0: the ground is placed this far above the pair's lowest point (manual height)
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
      { "cylinder standing on box",            boxSpec( 1, 1, 1 ), cylinderSpec( 0.3, 1.0 ).at( 0.1, 0.0, 0.99 ).rotated( 0, 90, 0 ) },
      { "cylinder lying on box",               boxSpec( 1, 1, 1 ), cylinderSpec( 0.25, 0.8 ).at( 0.0, 0.1, 0.74 ) },
      { "box on cylinder cap",                 cylinderSpec( 1.0, 1.0 ).rotated( 0, 90, 0 ), boxSpec( 0.4, 0.4, 0.4 ).at( 0.1, 0.2, 0.69 ) },
      { "cylinders stacked coaxially",         cylinderSpec( 0.5, 1.0 ).rotated( 0, 90, 0 ),
                                               cylinderSpec( 0.5, 1.0 ).at( 0.05, 0.0, 0.99 ).rotated( 0, 90, 0 ) },
      { "parallel lying cylinders",            cylinderSpec( 0.5, 1.0 ), cylinderSpec( 0.5, 1.0 ).at( 0.3, 0.0, 0.99 ) },
      { "cylinder standing on lying cylinder", cylinderSpec( 0.5, 2.0 ), cylinderSpec( 0.4, 1.0 ).at( 0.2, 0.0, 0.99 ).rotated( 0, 90, 0 ) },
      { "capsule on cylinder cap",             cylinderSpec( 0.5, 1.0 ).rotated( 0, 90, 0 ), capsuleSpec( 0.2, 1.5 ).at( 0.0, 0.1, 0.69 ) },
      // Mesh (torus R 1, r 0.3) with a DistanceMap: the primitive rests on top of the tube at x = R.
      { "sphere on torus tube",                torusSpec( 1.0, 0.3 ), sphereSpec( 0.2 ).at( 1.0, 0.0, 0.49 ) },
      { "box on torus tube",                   torusSpec( 1.0, 0.3 ), boxSpec( 0.4, 0.4, 0.4 ).at( 1.0, 0.0, 0.49 ) },
      { "capsule across torus tube",           torusSpec( 1.0, 0.3 ), capsuleSpec( 0.15, 0.5 ).at( 1.0, 0.0, 0.44 ).rotated( 0, 0, 90 ) },
      { "cylinder standing on torus tube",     torusSpec( 1.0, 0.3 ), cylinderSpec( 0.2, 0.6 ).at( 1.0, 0.0, 0.59 ).rotated( 0, 90, 0 ) },
      { "ellipsoid on torus tube",             torusSpec( 1.0, 0.3 ), ellipsoidSpec( 0.3, 0.2, 0.15 ).at( 1.0, 0.0, 0.44 ) },
      { "sphere in torus hole (no contact)",   torusSpec( 1.0, 0.3 ), sphereSpec( 0.5 ) },
      // Demonstrations of the open issues in contact-issues.md ("ISSUE" presets).
      // Overlap sampling: a 5 x 5 box over a torus with a 0.2 thick tube. Only the part of the box
      // inside the mesh's bounding box is sampled, at the grid spacing, so the 0.087 wide contact
      // band is found at every position (with a pitch of 5 / 24 = 0.21 it was missed).
      { "large box on small torus (overlap sampling)",      torusSpec( 0.35, 0.1 ), boxSpec( 5, 5, 0.4 ).at( 0.0, 0.0, 0.29 ) },
      // Two patches: the box rests on the tube at x = -1 and x = +1 with parallel normals. The
      // connectivity clustering keeps them apart (four contacts on each); a fixed clustering
      // radius of the box's size used to merge them into one cluster with three contacts.
      { "box bridging the torus (two patches)",             torusSpec( 1.0, 0.3 ), boxSpec( 2.6, 0.4, 0.4 ).at( 0.0, 0.0, 0.49 ) },
      // Plane-mesh: the torus rests on the ground, which is placed 0.01 into it; the clustering
      // gives the resting patch and its outline (six ground contacts, 52 before clustering).
      { "torus on the ground (plane-mesh manifold)",        torusSpec( 1.0, 0.3 ), sphereSpec( 0.2 ).at( 0.0, 0.0, 1.2 ), 0.01 },
      // Position correction: 0.05 penetration becomes a separation velocity erp * 0.05 / dt
      // (35 m/s at dt = 1e-3) that stays in the body. Untick "start from a touching state" in the
      // Simulation window and press Run.
      { "ISSUE: box on box 0.05 deep (correction launch)",  boxSpec( 1, 1, 1 ), boxSpec( 1, 1, 1 ).at( 0.0, 0.0, 0.95 ) },
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

polyscope::VolumeGrid*  dmGrids[2]      = { nullptr, nullptr };   // signed distance field display
bool                    showDistanceMap = false;
double                  dmBuildSeconds[2] = { 0.0, 0.0 };
std::string             dmInfo[2];

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

// Ground plane: a fixed pe plane (uid 3) drawn as Polyscope's ground; by default kept just under
// the posed pair while the pair is edited.
bool    groundOn     = true;
bool    groundAuto   = true;
double  groundHeight = 0.0;
PlaneID ground       = nullptr;
viewer::ContactLog groundLog;     // collide( A, ground ), collide( B, ground )

// Simulation from the posed state
simctl::Clock simClock;
// The presets penetrate by 0.01 (the static analysis shows real depths). The solver turns
// penetration into a separation velocity erp * depth / dt that stays in the body, so a penetrating
// initial state launches the bodies (7 m/s for 0.01 at dt = 1e-3). When set, the first step from
// the posed state first translates the bodies apart; the posed state itself is unchanged.
bool   startTouching   = true;
double removedPenetration = 0.0;   // largest initial penetration removed at the start of the run
std::vector<double> tBuf, keBuf, minDistBuf, countBuf;

bool simulating() { return simClock.steps > 0; }

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

//! Pair friction is additive (0.3 + 0.3 = 0.6), restitution 0.
MaterialID pairMaterial()
{
   // Materials survive World::clear(); create it exactly once.
   static const MaterialID material = createMaterial( "contact_viewer", 1.0, 0.0, 0.3, 0.3, 0.25, 200, 1000, 10, 11 );
   return material;
}


BodyID createMeshBody( const BodySpec& s, pe::id_t uid, int slot )
{
   Vertices vertices;
   IndicesLists faces;
   viewer::makeTorus( s.torusMajor, s.torusMinor, std::max( 3, s.torusSegs[0] ), std::max( 3, s.torusSegs[1] ), vertices, faces );
   TriangleMeshID m = createTriangleMesh( uid, Vec3( 0, 0, 0 ), vertices, faces, pairMaterial(), /*convex=*/false );

   dmBuildSeconds[slot] = 0.0;
   dmInfo[slot]         = "no DistanceMap";
#ifdef PE_USE_CGAL
   if( s.distanceMap ) {
      const auto t0 = std::chrono::steady_clock::now();
      m->enableDistanceMapAcceleration( std::max( 4, s.dmResolution ), std::max( 0, s.dmTolerance ) );
      dmBuildSeconds[slot] = std::chrono::duration<double>( std::chrono::steady_clock::now() - t0 ).count();
      if( m->hasDistanceMap() ) {
         const DistanceMap* dm = m->getDistanceMap();
         char buf[160];
         std::snprintf( buf, sizeof( buf ), "DistanceMap %d x %d x %d, spacing %.4g, built in %.2f s",
                        dm->getNx(), dm->getNy(), dm->getNz(), static_cast<double>( dm->getSpacing() ), dmBuildSeconds[slot] );
         dmInfo[slot] = buf;
      }
      else {
         dmInfo[slot] = "DistanceMap creation FAILED (mesh not closed?)";
      }
   }
#else
   dmInfo[slot] = "built without CGAL: no DistanceMap, GJK/EPA treats the mesh as convex";
#endif
   return m;
}


BodyID createBody( const BodySpec& s, pe::id_t uid )
{
   const MaterialID material = pairMaterial();
   if( s.kind == kMesh )
      return createMeshBody( s, uid, static_cast<int>( uid ) - 1 );

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


viewer::ShapeMesh createMesh( const BodySpec& s, BodyID body )
{
   switch( s.kind ) {
      case kMesh:      return viewer::makeBodyMesh( body );
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


//! (Re-)registers the signed distance field of body \a i as a Polyscope volume grid in the
//! mesh's body frame (the grid follows the body through its transform), with the zero
//! isosurface shown; Polyscope's slice planes cut through the scalar field.
void registerDistanceMapGrid( int i )
{
   const std::string name = std::string( kBodyNames[i] ) + " distance map";
   polyscope::removeStructure( name, /*errorIfAbsent=*/false );
   dmGrids[i] = nullptr;
#ifdef PE_USE_CGAL
   if( !showDistanceMap || specs[i].kind != kMesh || bodies[i] == nullptr )
      return;
   TriangleMeshID m = static_body_cast<TriangleMesh>( bodies[i] );
   if( !m->hasDistanceMap() )
      return;
   const DistanceMap* dm = m->getDistanceMap();
   const Vec3 o = dm->getOrigin();
   const real h = dm->getSpacing();
   const glm::uvec3 dims( dm->getNx(), dm->getNy(), dm->getNz() );
   const glm::vec3  lo = viewer::toGlm( o );
   const glm::vec3  hi = viewer::toGlm( o + Vec3( ( dm->getNx() - 1 ) * h, ( dm->getNy() - 1 ) * h, ( dm->getNz() - 1 ) * h ) );
   // Same node layout as Polyscope (x fastest, then y, then z), so the data is passed as is.
   const std::vector<float> sdf( dm->getSdfData().begin(), dm->getSdfData().end() );
   dmGrids[i] = polyscope::registerVolumeGrid( name, dims, lo, hi );
   polyscope::VolumeGridNodeScalarQuantity* q = dmGrids[i]->addNodeScalarQuantity( "signed distance", sdf );
   q->setIsosurfaceLevel( 0.0f );
   q->setIsosurfaceVizEnabled( true );
   q->setGridcubeVizEnabled( false );
   q->setEnabled( true );
   dmGrids[i]->setTransform( viewer::bodyTransform( bodies[i] ) );
#endif
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
   if( dmGrids[i] != nullptr )
      dmGrids[i]->setTransform( meshPose[i] );
   dirty = true;
}


//! Lowest point of the pair (planes excluded, exact via the support function); 0 without a
//! finite body.
double lowestPoint()
{
   double z = 0.0;
   bool   any = false;
   for( int i = 0; i < 2; ++i ) {
      if( specs[i].kind == kPlane )
         continue;
      const double zi = static_cast<double>( bodies[i]->support( Vec3( 0, 0, -1 ) )[2] );
      z   = any ? std::min( z, zi ) : zi;
      any = true;
   }
   return z;
}


void updateGroundDisplay()
{
   polyscope::options::groundPlaneMode       = ( ground != nullptr ) ? polyscope::GroundPlaneMode::TileReflection
                                                                     : polyscope::GroundPlaneMode::None;
   polyscope::options::groundPlaneHeightMode = polyscope::GroundPlaneHeightMode::Manual;
   polyscope::options::groundPlaneHeight     = static_cast<float>( groundHeight );
}


//! Moves the ground to groundHeight (just under the pair when groundAuto).
void placeGround()
{
   if( ground == nullptr )
      return;
   if( groundAuto )
      groundHeight = lowestPoint();
   ground->setPosition( Vec3( 0.0, 0.0, groundHeight ) );
   updateGroundDisplay();
}


//! Shape, size, initial state or ground changed, or back to the posed state: recreate the world
//! (both bodies, the ground and their mirror meshes) at t = 0.
void rebuildScene()
{
   simctl::mouseSpring().end();
   simctl::controls().running     = false;
   simctl::controls().queuedSteps = 0;
   simClock.reset();
   for( std::vector<double>* buf : { &tBuf, &keBuf, &minDistBuf, &countBuf } )
      buf->clear();

   contactLog.clear();    // the logs hold pointers into the world being cleared
   swappedLog.clear();
   groundLog.clear();
   theWorld()->clear();
   ground = nullptr;

   for( int i = 0; i < 2; ++i ) {
      bodies[i] = createBody( specs[i], static_cast<pe::id_t>( i + 1 ) );

      const viewer::ShapeMesh mesh = createMesh( specs[i], bodies[i] );
      meshes[i] = polyscope::registerSurfaceMesh( kBodyNames[i], mesh.vertices, mesh.faces );
      meshes[i]->setSurfaceColor( kBodyColors[i] );
      meshes[i]->setTransparency( 0.45f );   // contacts live inside the overlap region
      meshes[i]->setEdgeWidth( ( specs[i].kind == kBox || specs[i].kind == kPlane ) ? 1.0 : 0.0 );
      meshes[i]->setSmoothShade( specs[i].kind == kMesh );
      setGizmoEnabled( meshes[i], gizmo[i] );
      registerDistanceMapGrid( i );
      applyPose( i );

      if( specs[i].kind != kPlane ) {
         if( specs[i].fixed ) {
            bodies[i]->setFixed( true );
         }
         else {
            bodies[i]->setLinearVel( Vec3( specs[i].vel[0], specs[i].vel[1], specs[i].vel[2] ) );
            bodies[i]->setAngularVel( Vec3( specs[i].angVel[0], specs[i].angVel[1], specs[i].angVel[2] ) );
         }
      }
   }

   if( groundOn ) {
      ground = createPlane( 3, Vec3( 0, 0, 1 ), Vec3( 0, 0, 0 ), pairMaterial() );
      placeGround();
   }
   updateGroundDisplay();
}


//! PE -> Polyscope while simulating.
void updateMirror()
{
   for( int i = 0; i < 2; ++i ) {
      meshPose[i] = viewer::bodyTransform( bodies[i] );
      meshes[i]->setTransform( meshPose[i] );
      if( dmGrids[i] != nullptr )
         dmGrids[i]->setTransform( meshPose[i] );
   }
}


//! The gizmo edits the posed state, so it is hidden while time is past zero.
void syncGizmos()
{
   for( int i = 0; i < 2; ++i )
      if( meshes[i] != nullptr )
         setGizmoEnabled( meshes[i], gizmo[i] && !simulating() );
}


void selectPreset( int index )
{
   presetIndex = std::max( 0, std::min( index, static_cast<int>( presets().size() ) - 1 ) );
   const Preset& preset = presets()[presetIndex];
   specs[0]    = preset.a;
   specs[1]    = preset.b;
   selected    = -1;
   groundAuto  = ( preset.groundLift <= 0.0 );
   rebuildScene();
   if( !groundAuto ) {
      groundHeight = lowestPoint() + preset.groundLift;
      placeGround();
      dirty = true;
   }
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


//! Whether support( n ) is a meaningful contact witness: not for the infinite plane (undefined)
//! and not for a triangle mesh (the farthest vertex of a possibly non-convex mesh).
bool hasSupport( pe::GeomID g )
{
   return g->getType() != planeType && g->getType() != triangleMeshType;
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
   if( !simulating() )
      placeGround();
   runCollide( bodies[0], bodies[1], contactLog );
   runCollide( bodies[1], bodies[0], swappedLog );
   compareDispatchOrders();
   if( selected >= static_cast<int>( contactLog.entries.size() ) )
      selected = -1;

   groundLog.clear();
   if( ground != nullptr ) {
      viewer::ContactLog log;
      for( int i = 0; i < 2; ++i ) {
         if( specs[i].kind == kPlane )
            continue;
         runCollide( bodies[i], ground, log );
         groundLog.entries.insert( groundLog.entries.end(), log.entries.begin(), log.entries.end() );
      }
   }

   viewer::drawContactOverlay( "contacts", contactLog, overlay );
   // Ground contacts in their own look (green, smaller), so they are not taken for A-B contacts.
   viewer::OverlayOptions groundOverlay = overlay;
   groundOverlay.fixedColor  = true;
   groundOverlay.pointRadius = 0.6 * overlay.pointRadius;
   viewer::drawContactOverlay( "ground contacts", groundLog, groundOverlay );
   drawSelection();
   drawWitnesses();
   if( sweepAuto && !simulating() )
      runSweep();
   dirty = false;
}


double kineticEnergy()
{
   double e = 0.0;
   for( int i = 0; i < 2; ++i ) {
      if( bodies[i]->isFixed() )
         continue;
      const Vec3& v = bodies[i]->getLinearVel();
      const Vec3& w = bodies[i]->getAngularVel();
      e += 0.5 * static_cast<double>( bodies[i]->getMass() * ( trans( v ) * v ) );
      e += 0.5 * static_cast<double>( trans( w ) * ( bodies[i]->getInertia() * w ) );
   }
   return e;
}


//! Movable body of a contact to translate: B unless B cannot move.
int movableIndex( int preferred )
{
   const int other = 1 - preferred;
   if( specs[preferred].kind != kPlane && !bodies[preferred]->isFixed() ) return preferred;
   if( specs[other].kind     != kPlane && !bodies[other]->isFixed() )     return other;
   return -1;
}


//! Removes the initial penetration of the posed state (see startTouching): lifts bodies out of the
//! ground and translates the movable body of the pair along the deepest A-B contact normal by the
//! deepest depth, a few rounds so that one correction cannot undo the other.
void separateInitialState()
{
   removedPenetration = 0.0;
   for( int round = 0; round < 4; ++round ) {
      bool moved = false;

      if( ground != nullptr )
         for( int i = 0; i < 2; ++i ) {
            if( specs[i].kind == kPlane || bodies[i]->isFixed() )
               continue;
            const double depth = groundHeight - static_cast<double>( bodies[i]->support( Vec3( 0, 0, -1 ) )[2] );
            if( depth > 0.0 ) {
               bodies[i]->translate( Vec3( 0.0, 0.0, depth ) );
               removedPenetration = std::max( removedPenetration, depth );
               moved = true;
            }
         }

      viewer::ContactLog log;
      runCollide( bodies[0], bodies[1], log );
      const viewer::ContactLog::Entry* deepest = nullptr;
      for( const viewer::ContactLog::Entry& c : log.entries )
         if( c.dist < real(0) && ( deepest == nullptr || c.dist < deepest->dist ) )
            deepest = &c;
      if( deepest != nullptr ) {
         // The normal points from g2 towards g1: g1 moves along +n, g2 along -n.
         const int g1 = ( deepest->g1 == bodies[0] ) ? 0 : 1;
         const int i  = movableIndex( 1 );
         if( i >= 0 ) {
            const real depth = -deepest->dist;
            bodies[i]->translate( ( i == g1 ? depth : -depth ) * deepest->normal );
            removedPenetration = std::max( removedPenetration, static_cast<double>( depth ) );
            moved = true;
         }
      }

      if( !moved )
         break;
   }
}


//! Advances the pair (and records the history) by \a n steps.
void step( int n )
{
   if( n <= 0 || !simClock.error.empty() )
      return;
   if( simClock.steps == 0 && startTouching )
      separateInitialState();
   std::vector<BodyID> dynamic;
   for( int i = 0; i < 2; ++i )
      if( specs[i].kind != kPlane )
         dynamic.push_back( bodies[i] );
   simctl::advance( n, dynamic, simClock );
   updateMirror();

   // A-B contacts of the new state for the history (refresh() redraws the overlay).
   viewer::ContactLog log;
   runCollide( bodies[0], bodies[1], log );
   double minDist = std::numeric_limits<double>::quiet_NaN();
   for( const viewer::ContactLog::Entry& c : log.entries )
      minDist = std::isnan( minDist ) ? static_cast<double>( c.dist ) : std::min( minDist, static_cast<double>( c.dist ) );
   if( tBuf.size() > 60000 )
      for( std::vector<double>* buf : { &tBuf, &keBuf, &minDistBuf, &countBuf } )
         buf->erase( buf->begin(), buf->begin() + static_cast<long>( buf->size() / 2 ) );
   tBuf.push_back( simClock.time );
   keBuf.push_back( kineticEnergy() );
   minDistBuf.push_back( minDist );
   countBuf.push_back( static_cast<double>( log.entries.size() ) );
   dirty = true;
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
   const char* type[kNumShapes] = { "Sphere", "Box", "Capsule", "Cylinder", "Ellipsoid", "Plane", "TriangleMesh" };
   const char  var = ( uid == 1 ) ? 'a' : 'b';
   if( s.kind == kMesh ) {
      o << "   // torus R " << s.torusMajor << ", r " << s.torusMinor << ", " << s.torusSegs[0] << " x " << s.torusSegs[1]
        << " segments: see makeTorus() in tests/interface/pe_primitive_mesh_distancemap_test.cpp\n";
      o << "   TriangleMeshID " << var << " = createTriangleMesh( " << uid << ", Vec3( " << s.pos[0] << ", " << s.pos[1] << ", "
        << s.pos[2] << " ), vertices, faces, mat, false );\n";
      if( s.distanceMap )
         o << "   " << var << "->enableDistanceMapAcceleration( " << s.dmResolution << ", " << s.dmTolerance << " );\n";
      if( s.eulerDeg[0] != 0.0 || s.eulerDeg[1] != 0.0 || s.eulerDeg[2] != 0.0 )
         o << "   " << var << "->setOrientation( Quat( real(" << s.eulerDeg[0] * kDegToRad << "), real("
           << s.eulerDeg[1] * kDegToRad << "), real(" << s.eulerDeg[2] * kDegToRad << ") ) );   // Euler x, y, z [rad]\n";
      return o.str();
   }
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
   // The current state: the posed one at t = 0, the simulated one afterwards.
   BodySpec current[2] = { specs[0], specs[1] };
   for( int i = 0; i < 2; ++i )
      poseFromTransform( viewer::bodyTransform( bodies[i] ), current[i] );

   std::ostringstream o;
   o << std::setprecision( 17 );
   o << "   // Pair Lab: " << kShapeNames[specs[0].kind] << " (a) vs " << kShapeNames[specs[1].kind] << " (b)";
   if( simulating() )
      o << ", simulated state at t = " << simClock.time << " (" << simClock.steps << " steps)";
   o << "\n";
   o << createCall( current[0], 1 ) << createCall( current[1], 2 );
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
      case kMesh: {
         // Mesh parameters rebuild the body (and its DistanceMap, which takes a moment): applied
         // on Enter, not while typing.
         const ImGuiInputTextFlags enter = ImGuiInputTextFlags_EnterReturnsTrue;
         if( ImGui::InputDouble( "torus major radius R", &s.torusMajor, 0.0, 0.0, "%.4g", enter ) ) change = 2;
         if( ImGui::InputDouble( "torus minor radius r", &s.torusMinor, 0.0, 0.0, "%.4g", enter ) ) change = 2;
         if( ImGui::InputInt2( "segments (major, minor)", s.torusSegs, enter ) ) change = 2;
         s.torusMajor = std::max( 1.0e-3, s.torusMajor );
         s.torusMinor = std::max( 1.0e-3, std::min( s.torusMinor, s.torusMajor - 1.0e-3 ) );
         if( ImGui::Checkbox( "distance map", &s.distanceMap ) ) change = 2;
         if( ImGui::IsItemHovered( ImGuiHoveredFlags_DelayShort ) )
            ImGui::SetTooltip( "Contacts with this mesh come from its signed distance field (any convex primitive,\n"
                               "plane, other meshes); without it the mesh goes through GJK/EPA as if it were convex." );
         if( s.distanceMap ) {
            if( ImGui::InputInt( "DM resolution (cells)", &s.dmResolution, 1, 10, enter ) ) change = 2;
            if( ImGui::InputInt( "DM tolerance (padding cells)", &s.dmTolerance, 1, 1, enter ) ) change = 2;
         }
         ImGui::TextDisabled( "%s", dmInfo[i].c_str() );
         ImGui::TextDisabled( "hole axis = body z; Enter applies a value" );
         break;
      }
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

   if( s.kind != kPlane ) {
      if( ImGui::Checkbox( "fixed", &s.fixed ) ) change = 2;
      if( ImGui::IsItemHovered( ImGuiHoveredFlags_DelayShort ) )
         ImGui::SetTooltip( "Immovable in the simulation (infinite mass), e.g. a base or a ramp." );
      if( !s.fixed ) {
         if( dragDoubles( "initial velocity", s.vel, 3, 0.01f, -100.0, 100.0, "%.3f" ) ) change = 2;
         if( dragDoubles( "initial ang. vel. [rad/s]", s.angVel, 3, 0.02f, -100.0, 100.0, "%.3f" ) ) change = 2;
      }
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

   if( simulating() )
      ImGui::TextColored( ImVec4( 1.0f, 0.8f, 0.2f, 1.0f ), "t = %.4f s: the pose is locked, Reset (r) returns to it", simClock.time );
   ImGui::BeginDisabled( simulating() );
   for( int i = 0; i < 2; ++i ) {
      if( !ImGui::CollapsingHeader( kBodyNames[i], ImGuiTreeNodeFlags_DefaultOpen ) )
         continue;
      const int change = drawBodyControls( i );
      if( change == 2 )
         rebuildScene();
      else if( change == 1 )
         applyPose( i );
   }

   if( ImGui::CollapsingHeader( "Ground plane", ImGuiTreeNodeFlags_DefaultOpen ) ) {
      if( ImGui::Checkbox( "ground plane", &groundOn ) )
         rebuildScene();
      if( groundOn ) {
         ImGui::SameLine();
         if( ImGui::Checkbox( "keep under the pair", &groundAuto ) )
            dirty = true;
         if( ImGui::IsItemHovered( ImGuiHoveredFlags_DelayShort ) )
            ImGui::SetTooltip( "The ground follows the lowest point of the posed pair (touching, no gap)." );
         ImGui::BeginDisabled( groundAuto );
         if( dragDoubles( "height", &groundHeight, 1, 0.002f, -100.0, 100.0, "%.4f" ) )
            dirty = true;
         ImGui::EndDisabled();
      }
   }
   ImGui::EndDisabled();

   if( ImGui::CollapsingHeader( "Overlay", ImGuiTreeNodeFlags_DefaultOpen ) ) {
      const char* colorModes[] = { "sign of dist", "contact type" };
      dirty |= ImGui::Combo( "color by", &overlay.colorBy, colorModes, 2 );
      dirty |= dragDoubles( "normal length", &overlay.normalLength, 1, 0.002f, 0.0, 1.0e6, "%.4g" );
      dirty |= ImGui::Checkbox( "scale normals by |dist|", &overlay.scaleByDist );
      dirty |= dragDoubles( "marker radius", &overlay.pointRadius, 1, 0.0005f, 1.0e-4, 1.0, "%.4f" );
      if( ImGui::Checkbox( "distance map (SDF grid)", &showDistanceMap ) )
         for( int i = 0; i < 2; ++i )
            registerDistanceMapGrid( i );
      if( ImGui::IsItemHovered( ImGuiHoveredFlags_DelayShort ) )
         ImGui::SetTooltip( "The mesh's signed distance field as a Polyscope volume grid in the body frame: the zero\n"
                            "isosurface (the surface as the DistanceMap sees it); add a slice plane (View menu) to\n"
                            "look at the field inside." );
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
   if( ground != nullptr )
      ImGui::TextDisabled( "ground contacts (A, B): %d (drawn green, smaller)", static_cast<int>( groundLog.entries.size() ) );
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
   if( simulating() ) {
      ImGui::TextDisabled( "The sweep varies the posed state: Reset (r) to use it." );
      ImGui::End();
      return;
   }

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


void drawSimulationWindow()
{
   ImGui::Begin( "Simulation" );

   if( simctl::drawStepControls() )
      rebuildScene();
   ImGui::Text( "t = %.4f s   steps: %ld", simClock.time, simClock.steps );
   ImGui::TextDisabled( "starts from the posed pair; Reset returns to it" );
   ImGui::BeginDisabled( simulating() );
   ImGui::Checkbox( "start from a touching state", &startTouching );
   ImGui::EndDisabled();
   if( ImGui::IsItemHovered( ImGuiHoveredFlags_DelayShort ) )
      ImGui::SetTooltip( "The solver turns penetration into a separation velocity (error reduction x depth / dt)\n"
                         "that stays in the body: a penetrating pose launches the bodies. When set, the first\n"
                         "step moves the bodies apart first; the posed state is unchanged. Untick to see the\n"
                         "raw solver response." );
   if( simulating() && startTouching && removedPenetration > 0.0 )
      ImGui::TextDisabled( "initial penetration removed: %.4g", removedPenetration );
   if( !simClock.error.empty() )
      ImGui::TextColored( ImVec4( 1.0f, 0.3f, 0.3f, 1.0f ), "%s", simClock.error.c_str() );
   if( simctl::mouseSpring().body() != nullptr )
      ImGui::TextDisabled( "dragging body %s", simctl::mouseSpring().body() == bodies[0] ? "A" : "B" );
   else
      ImGui::TextDisabled( "Ctrl + left-drag a body to pull it" );

   simctl::drawSolverControls();

   if( ImGui::CollapsingHeader( "History", ImGuiTreeNodeFlags_DefaultOpen ) ) {
      const int n = static_cast<int>( tBuf.size() );
      const ImPlotAxisFlags fit = ImPlotAxisFlags_AutoFit;
      if( ImPlot::BeginPlot( "A-B minimum dist", ImVec2( -1, 150 ) ) ) {
         ImPlot::SetupAxes( "t [s]", "min dist", fit, fit );
         if( n > 0 ) ImPlot::PlotLine( "min dist", tBuf.data(), minDistBuf.data(), n );
         ImPlot::EndPlot();
      }
      if( ImPlot::BeginPlot( "A-B contacts / kinetic energy", ImVec2( -1, 150 ) ) ) {
         ImPlot::SetupAxes( "t [s]", "contacts", fit, fit );
         ImPlot::SetupAxis( ImAxis_Y2, "E_kin [J]", fit | ImPlotAxisFlags_AuxDefault );
         if( n > 0 ) {
            ImPlot::PlotStairs( "contacts", tBuf.data(), countBuf.data(), n );
            ImPlot::SetAxes( ImAxis_X1, ImAxis_Y2 );
            ImPlot::PlotLine( "E_kin", tBuf.data(), keBuf.data(), n );
         }
         ImPlot::EndPlot();
      }
      ImGui::TextDisabled( "gaps in min dist = no A-B contact" );
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
   theWorld()->setGravity( 0.0, 0.0, 0.0 );   // gravity is applied as a force (simctl::Controls::gravityZ)
   simctl::applySolverKnobs();

   // The bodies stay within a few units of the origin; fixed extents keep the camera and the
   // absolute marker sizes stable while shapes are edited.
   polyscope::options::automaticallyComputeSceneExtents = false;
   polyscope::state::lengthScale = 3.0f;
   polyscope::state::boundingBox =
      std::tuple<glm::vec3, glm::vec3>{ glm::vec3( -1.5f, -1.5f, -1.5f ), glm::vec3( 1.5f, 1.5f, 1.5f ) };

   rebuildScene();   // keeps the pair as it was posed before a mode switch (at t = 0)
   // Oblique view: the axis-aligned home view hides one dimension of box-box manifolds.
   polyscope::view::lookAt( glm::vec3( 2.6f, -3.2f, 2.0f ), glm::vec3( 0.0f, 0.0f, 0.4f ) );
}


void loadPreset( int index )
{
   selectPreset( index );
}


void advance( int steps )
{
   step( steps );
   refresh();
}


void frame()
{
   if( simctl::processKeys() )
      rebuildScene();
   if( !simulating() )
      readBackGizmos();
   processPick();

   std::vector<simctl::MouseSpring::Target> targets;
   for( int i = 0; i < 2; ++i )
      if( specs[i].kind != kPlane )
         targets.push_back( simctl::MouseSpring::Target{ meshes[i], bodies[i] } );
   simctl::mouseSpring().process( targets, 0.01 );

   // Queued single steps run here, so step/draw ordering stays trivial.
   step( simctl::takeFrameSteps() );
   syncGizmos();

   drawPairWindow();
   if( dirty )
      refresh();
   drawContactsWindow();
   drawSweepWindow();
   drawSimulationWindow();
}


bool smokeTest()
{
   bool ok = true;

   for( int k = 0; k < static_cast<int>( presets().size() ); ++k ) {
      selectPreset( k );
      polyscope::frameTick();
      std::printf( "preset %2d  %-50s %d contact(s), swapped %d, ground %d, %s%s\n", k, presets()[k].name,
                   static_cast<int>( contactLog.entries.size() ), static_cast<int>( swappedLog.entries.size() ),
                   static_cast<int>( groundLog.entries.size() ),
                   swapCheck.countMatch ? "counts agree" : "COUNTS DIFFER",
                   collideError.empty() ? "" : ( " ERROR: " + collideError ).c_str() );
      for( const viewer::ContactLog::Entry& c : contactLog.entries )
         std::printf( "            %-12s dist %+.6e  pos (%+.4f %+.4f %+.4f)  n (%+.4f %+.4f %+.4f)\n",
                      viewer::kindName( c.kind ), static_cast<double>( c.dist ),
                      static_cast<double>( c.pos[0] ), static_cast<double>( c.pos[1] ), static_cast<double>( c.pos[2] ),
                      static_cast<double>( c.normal[0] ), static_cast<double>( c.normal[1] ), static_cast<double>( c.normal[2] ) );
      // An empty preset is reported, not failed: this checks the viewer, and the contact
      // routines themselves are covered by tests/interface.
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

#ifdef PE_USE_CGAL
   // Mesh with a DistanceMap: a sphere in the torus hole must get no contact (the convex hull
   // would contain it), a sphere on the tube one with the tube's upward normal.
   {
      int hole = -1, tube = -1;
      for( int k = 0; k < static_cast<int>( presets().size() ); ++k ) {
         if( std::string( presets()[k].name ) == "sphere in torus hole (no contact)" ) hole = k;
         if( std::string( presets()[k].name ) == "sphere on torus tube" )              tube = k;
      }
      selectPreset( hole );
      polyscope::frameTick();
      const bool holeOk = static_body_cast<TriangleMesh>( bodies[0] )->hasDistanceMap() && contactLog.entries.empty();
      std::printf( "torus (%s): sphere in the hole -> %d contact(s) %s\n", dmInfo[0].c_str(),
                   static_cast<int>( contactLog.entries.size() ), holeOk ? "ok" : "FAILED" );
      showDistanceMap = true;   // the SDF grid through a frame as well
      selectPreset( tube );
      polyscope::frameTick();
      bool tubeOk = !contactLog.entries.empty() && dmGrids[0] != nullptr;
      for( const viewer::ContactLog::Entry& c : contactLog.entries )
         tubeOk = tubeOk && c.dist < real(0) && ( c.g1 == bodies[1] ? c.normal[2] : -c.normal[2] ) > real(0.99);
      std::printf( "torus: sphere on the tube -> %d contact(s), SDF grid %s -> %s\n", static_cast<int>( contactLog.entries.size() ),
                   dmGrids[0] != nullptr ? "registered" : "missing", tubeOk ? "ok" : "FAILED" );
      showDistanceMap = false;
      ok = ok && holeOk && tubeOk;

      // The large box on the small torus must be supported wherever it is: the contact band of
      // the tube is narrower than the former sample pitch of 5 / 24.
      int large = -1;
      for( int k = 0; k < static_cast<int>( presets().size() ); ++k )
         if( std::string( presets()[k].name ) == "large box on small torus (overlap sampling)" ) large = k;
      selectPreset( large );
      bool largeOk = true;
      for( double x : { 0.0, 0.05, 0.1, 0.15, 0.2 } ) {
         specs[1].pos[0] = x;
         applyPose( 1 );
         polyscope::frameTick();
         largeOk = largeOk && !contactLog.entries.empty();
         std::printf( "large box on small torus at x = %.2f: %d contact(s)\n", x, static_cast<int>( contactLog.entries.size() ) );
      }
      ok = ok && largeOk;
   }
#endif

   // Simulation from a posed state: the cylinder standing on the box, over the ground. After 2 s
   // both must rest (box on the ground, cylinder on the box), and Reset must restore the pose.
   int standing = 0;
   for( int k = 0; k < static_cast<int>( presets().size() ); ++k )
      if( std::string( presets()[k].name ) == "cylinder standing on box" )
         standing = k;
   groundOn   = true;
   groundAuto = true;
   selectPreset( standing );
   polyscope::frameTick();
   const double groundZ = groundHeight;
   simctl::controls().running = true;          // a few GUI frames in the running state
   for( int i = 0; i < 5; ++i )
      polyscope::frameTick();
   simctl::controls().running = false;
   for( int i = 0; i < 20 && simClock.steps < 1000; ++i ) {
      step( std::min( 50L, 1000L - simClock.steps ) );
      polyscope::frameTick();
   }
   const double boxBottom = static_cast<double>( bodies[0]->support( Vec3( 0, 0, -1 ) )[2] );
   const double boxTop    = static_cast<double>( bodies[0]->support( Vec3( 0, 0,  1 ) )[2] );
   const double cylBottom = static_cast<double>( bodies[1]->support( Vec3( 0, 0, -1 ) )[2] );
   const bool   rests     = simClock.error.empty() && kineticEnergy() < 1.0e-6
                         && std::abs( boxBottom - groundZ ) < 1.0e-3 && cylBottom > boxTop - 1.0e-3;
   std::printf( "\nsimulation (%s, %ld steps): box bottom %.5f (ground %.5f), cylinder bottom %.5f (box top %.5f), E_kin %.3e, %d A-B contacts -> %s\n",
                presets()[standing].name, simClock.steps, boxBottom, groundZ, cylBottom, boxTop, kineticEnergy(),
                static_cast<int>( contactLog.entries.size() ), rests ? "resting" : "FAILED" );
   ok = ok && rests;

   // The corner-face preset penetrates by 0.01: without separation the top box is launched at
   // erp * depth / dt; with it (default) it must not rise.
   int cornerFace = 0;
   for( int k = 0; k < static_cast<int>( presets().size() ); ++k )
      if( std::string( presets()[k].name ) == "box on box: corner-face" )
         cornerFace = k;
   double launch[2];
   for( int touching = 0; touching < 2; ++touching ) {
      startTouching = ( touching == 1 );
      selectPreset( cornerFace );
      step( 1 );
      launch[touching] = static_cast<double>( bodies[1]->getLinearVel()[2] );
   }
   const double expected = simctl::controls().erp * 0.01 / simctl::controls().dt;
   // With the split impulse the raw (penetrating) start must not launch the box either.
   simctl::controls().splitImpulse = true;
   simctl::applySolverKnobs();
   startTouching = false;
   selectPreset( cornerFace );
   step( 1 );
   const double launchSplit = static_cast<double>( bodies[1]->getLinearVel()[2] );
   simctl::controls().splitImpulse = false;
   simctl::applySolverKnobs();
   const bool launchOk = std::abs( launch[0] - expected ) < 0.05 * expected && launch[1] <= 0.0 && std::abs( launchSplit ) < 0.05;
   std::printf( "corner-face start: v_z of the top box %+.3f raw (erp * depth / dt = %.3f), %+.4f with separation, %+.4f with split impulse -> %s\n",
                launch[0], expected, launch[1], launchSplit, launchOk ? "ok" : "FAILED" );
   ok = ok && launchOk;
   startTouching = true;
   selectPreset( standing );

   rebuildScene();   // Reset
   const Vec3 p1 = bodies[1]->getPosition();
   const bool restored = simClock.steps == 0 && p1[0] == specs[1].pos[0] && p1[1] == specs[1].pos[1] && p1[2] == specs[1].pos[2];
   std::printf( "reset: %s\n", restored ? "posed state restored" : "FAILED" );
   ok = ok && restored;

   return ok;
}

} // namespace pairlab
