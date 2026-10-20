//=================================================================================================
/*!
 *  \file tools/table_tennis/table_tennis.cpp
 *  \brief Interactive Polyscope table tennis demo: serves with velocity and spin, ghost serve
 *
 *  An ITTF-size table (2.74 x 1.525 m, top at 0.76 m) with the net as a box block, a 40 mm /
 *  2.7 g hollow ball (0.4 mm shell, Sphere::setShellThickness()), and a GUI that launches a serve from an adjustable position with an adjustable
 *  velocity and spin. Flight aerodynamics (quadratic drag and the Magnus force) are applied by
 *  the demo as external forces; the bounce is the HardContactSemiImplicitTimesteppingSolvers
 *  solver with Newton restitution and Coulomb friction, so backspin reverses the ball at the
 *  bounce when the spin is strong enough: the "ghost serve" preset bounces once on the opponent's
 *  side and comes back over the net.
 *
 *  Units: SI, z up, the table's length along x, the server at x < 0.
 *
 *  Usage: pe_table_tennis [--smoke]
 *                         [--serve vx vy vz wx wy wz [--pos x y z] [--cor e] [--mu m] [--dt s]
 *                          [--shell t_m] [--mass kg] [--time T] [--no-drag] [--no-magnus] [--quiet]]
 *    --smoke   headless self check (Polyscope's mock OpenGL backend)
 *    --screenshot <file.png> [--at T] [--preset k] [--top]   render preset k (default 0, the
 *              ghost serve) at time T (default 1.0 s), optionally from above, to an image and exit
 *    --serve   headless batch run of one serve (spin in revolutions per second): prints the
 *              bounce / net-crossing events and a RESULT line, no window
 *    --toss    headless batch run of a toss & serve: --toss-pos x y z --toss-speed u --contact-z z
 *              --bat speed azimuth elevation tilt yaw (m/s, deg) --wrist rate --rubber e mu
 *              --swing rate radius phase axisYaw axisPitch (pendulum swing about an arm pivot;
 *              --bat is then the arm's motion) --handle-up
 */
//=================================================================================================

#include <pe/system/WarningDisable.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <fstream>
#include <string>
#include <type_traits>
#include <vector>

#include <pe/core.h>

#include "polyscope/curve_network.h"
#include "polyscope/point_cloud.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

#include "implot.h"

#include "ShapeMeshes.h"

using namespace pe;


namespace {

using CollisionSystemType = std::remove_reference<decltype( *theCollisionSystem() )>::type;

//=================================================================================================
//  GEOMETRY (ITTF)
//=================================================================================================

const double kPi            = 3.14159265358979323846;
const double kTableLength   = 2.74;
const double kTableWidth    = 1.525;
const double kTableHeight   = 0.76;     // top surface
const double kTopThickness  = 0.03;
const double kNetHeight     = 0.1525;
const double kNetWidth      = 1.83;     // overhangs the table by 15.25 cm on each side
const double kNetThickness  = 0.01;
const double kNetTop        = kTableHeight + kNetHeight;
const double kBallRadius    = 0.02;
const double kG             = 9.81;


//=================================================================================================
//  PARAMETERS
//=================================================================================================

struct ServeParams {
   double pos[3]  = { -1.75, 0.0, 1.05 };   // ghost serve preset (see README.md): behind the end line (x = -1.37), 29 cm above the table
   double vel[3]  = {  2.9, 0.0, 2.25 };    // m/s, a lobbed launch
   double spin[3] = {  0.0, -50.0, 0.0 };   // revolutions per second (pro backspin serves reach ~55); omega_y < 0 is backspin for +x travel
};

struct AeroParams {
   bool   drag      = true;
   bool   magnus    = true;
   double rhoAir    = 1.204;   // kg/m^3
   double cd        = 0.40;    // drag coefficient of a smooth sphere at Re ~ 1e4..1e5
   double liftScale = 1.0;     // scales C_L = 1 / ( 2 + v / ( r omega ) )
   double spinDecay = 0.0;     // spin decay time constant [s], 0 = none
};

struct ContactParams {
   double cor = 0.90;   // ITTF drop test: 0.305 m -> ~0.25 m
   double mu  = 0.30;
   double corRubber = 0.80;   // ball on rubber (toss & serve mode)
   double muRubber  = 0.80;
};

struct BallParams {
   double mass           = 0.0027;   // kg (ITTF: 2.7 g)
   double shellThickness = 0.0004;   // m; a real ball is a ~0.4 mm shell, 0 = solid sphere
};

struct SolverParams {
   double dt            = 2.0e-4;
   int    stepsPerFrame = 40;
   int    maxIterations = 50;
   double erp           = 0.7;
   bool   splitImpulse  = true;
   bool   restitution   = true;
   bool   restitutionPostPass = false;   // Box3D scheme: inelastic solve, then a capped bounce pass
   int    restitutionIterations = 2;
};

//! Toss & serve mode: the ball is tossed straight up from the open palm and a bat on a
//! prescribed straight stroke meets it on the way down at the contact height.
struct BatParams {
   double toss[3]      = { -1.75, 0.0, 0.90 };   // release point (palm), behind the end line
   double tossSpeed    = 2.0;     // m/s up: rises 20 cm (the rule asks for at least 16)
   double contactZ     = 1.02;    // m, ball centre height where the bat meets it (falling, 8 cm below the apex)
   double batSpeed     = 2.0;     // m/s along the stroke
   double strokeAz     = 0.0;     // deg, stroke direction about z (0 = toward the net, +x)
   double strokeEl     = 15.0;    // deg, stroke direction above the horizontal
   double bladeTilt    = 30.0;    // deg, blade normal above the horizontal (open face > 0)
   double bladeYaw     = 0.0;     // deg, blade normal about z
   double wristRate    = 0.0;     // rad/s, bat rotation about z during the stroke (sidespin)
   double preTime      = 0.15;    // s, stroke starts this long before contact
   double postTime     = 0.06;    // s, stroke continues this long after contact, then stops
   // Pendulum swing: the bat hangs from a pivot (wrist / forearm) and swings on a circle of
   // radius swingRadius about the swing axis at swingRate, with the handle pointing at the
   // pivot; batSpeed / strokeAz / strokeEl are then the arm's translation, added on top.
   // swingRate 0 is the plain straight stroke above.
   double swingRate      = 0.0;    // rad/s; positive swings right-to-left for a server facing the net (axis +x)
   double swingRadius    = 0.30;   // m, pivot to bat centre
   double swingPhase     = 0.0;    // deg, arm angle at contact measured from the lowest point along the swing (< 0: still descending, catching the bottom of the ball)
   double swingAxisYaw   = 0.0;    // deg, swing axis about z (0: axis = +x, the swing plane faces the net)
   double swingAxisPitch = 0.0;    // deg, swing axis above the horizontal (> 0: the plane leans back toward the server)
   bool   handleUp       = false;  // straight stroke: roll the bat so that the handle points up in the blade plane
};

//! Receive mode: a second bat held still on the receiver's side, a block without a stroke. By
//! default it is placed by a dry run of the serve: its face is put against the incoming ball at
//! the point where the ball, after its bounce on the receiver's side, is highest (or earlier on
//! the rise), then tilted and yawed by the angles below; the handle points to the receiver's
//! right. The receiver faces the incoming ball, so for a ghost serve, whose ball comes back toward
//! the net after the bounce, the bat is reached over the table between the ball and the net.
struct ReceiverParams {
   double tilt         = 0.0;    // deg, blade normal above the horizontal: 0 vertical face, > 0 open (face up), < 0 closed
   double yaw          = 0.0;    // deg, blade normal about z relative to the incoming direction (> 0 turns the face toward the receiver's right)
   double riseFraction = 1.0;    // where on the rise after the bounce the ball is met: 1 = apex, 0.5 = halfway in time
   bool   autoPlace    = true;   // place by dry run on every serve; off: the ball is met at pos, coming along +x
   double pos[3]       = { 0.9, 0.0, 0.92 };   // ball centre at the intercept when autoPlace is off
};

struct Preset {
   const char* name;
   ServeParams serve;
};

struct BatPreset {
   const char* name;
   BatParams   bat;
};

// Found with the batch sweep in README.md ("Toss & serve"). The defaults of BatParams are the
// flat serve: ball leaves at (3.9, 0, 1.2) m/s with ~3 rev/s, 6.5 cm over the tape, lands 0.42 m
// past the net.
BatParams backspinBat()
{
   // Short heavy backspin: a higher toss dropped onto a nearly horizontal blade that moves
   // tangentially at 7 m/s. Ball (3.1, 0, 2.4) m/s with 33 rev/s backspin (sticking-limited:
   // bat speed / 2.5 of tangential velocity on a thin shell), 9 cm over the tape, lands 0.34 m
   // past the net and walks back toward it on the following bounces.
   BatParams b;
   b.toss[2]   = 0.85;
   b.tossSpeed = 2.5;
   b.contactZ  = 0.95;
   b.batSpeed  = 7.0;
   b.strokeEl  = 0.0;
   b.bladeTilt = 85.0;
   return b;
}

BatParams pendulumBat()
{
   // Pendulum: the same toss, the bat brushes across the ball toward the server's left (stroke
   // azimuth -30 deg), blade 60 deg open and yawed 60 deg so the face still meets the ball
   // tangentially. Ball (3.4, -0.1, 1.8) m/s with spin (-21, -24, 18) rev/s (side-backspin on a
   // tilted axis, 37 rev/s total), 8 cm over the tape, lands 0.36 m past the net and 0.33 m to
   // the left after curving, then kicks further sideways on the bounce.
   BatParams b = backspinBat();
   b.batSpeed  = 7.5;
   b.strokeAz  = -30.0;
   b.strokeEl  = 5.0;
   b.bladeTilt = 60.0;
   b.bladeYaw  = 60.0;
   return b;
}

BatParams pendulumRightHanded()
{
   // Right-handed pendulum: handle up, the bat hangs from the wrist and swings right to left
   // (about +x, 16 rad/s on a 0.30 m arm: 4.8 m/s at the bat) while the arm moves forward at
   // 3.5 m/s; contact at the lowest point of the swing, blade 60 deg open and yawed 20 deg to
   // the right so that the ball, pushed left by the brush, still goes down the table. Served
   // from the right side (y = -0.5). Ball (3.1, 1.3, 1.9) m/s, spin (21, -10, -14) rev/s: roll
   // from the bottom brush, backspin from the forward arm under the open face, sidespin from the
   // yawed face; 8.5 cm over the tape, lands 0.30 m past the net near the centre line and kicks
   // right on every bounce.
   BatParams b = backspinBat();
   b.toss[1]        = -0.5;
   b.batSpeed       = 3.5;
   b.strokeEl       = 0.0;
   b.bladeTilt      = 60.0;
   b.bladeYaw       = -20.0;
   b.swingRate      = 16.0;
   b.swingRadius    = 0.30;
   b.swingPhase     = 0.0;
   return b;
}

BatParams reversePendulumRightHanded()
{
   // Reverse pendulum: the same forehand swing with the wrist snapping the other way, left to
   // right (negative swing rate), served from the left side with the blade yawed left. The
   // sim's bat is symmetric, so it is the exact mirror (y -> -y) of the pendulum: spin
   // (-21, -10, 14) rev/s, the ball breaks and kicks to the server's left.
   BatParams b = pendulumRightHanded();
   b.toss[1]   = 0.5;
   b.bladeYaw  = 20.0;
   b.swingRate = -16.0;
   return b;
}

const BatPreset kBatPresets[] = {
   { "Flat serve (toss + bat)", BatParams() },
   { "Backspin serve (brush, toss + bat)", backspinBat() },
   { "Pendulum serve (right-handed swing, toss + bat)", pendulumRightHanded() },
   { "Reverse pendulum serve (right-handed swing, toss + bat)", reversePendulumRightHanded() },
   { "Forward brush, side-backspin (straight stroke)", pendulumBat() },
};
const int kNumBatPresets = static_cast<int>( sizeof( kBatPresets ) / sizeof( kBatPresets[0] ) );

enum Mode { kModeLaunch = 0, kModeToss = 1, kModeReceive = 2 };

// Bat geometry (ITTF: blade about 150 x 157 mm, 6 mm; rubber + sponge up to 4 mm per side)
const double kBladeRadius    = 0.0775;
const double kBladeThickness = 0.006;
const double kRubberThickness = 0.002;
const double kBatHalfThickness = 0.5 * kBladeThickness + kRubberThickness;
const double kHandleRadius   = 0.014;
const double kHandleLength   = 0.10;

//! ITTF service rule, the part a launch point can violate: the ball must be behind the
//! server's end line (not over the playing surface) and above the level of the playing
//! surface when struck. Returns an empty string for a legal launch point.
const char* serveLegalityAt( const Vec3& p )
{
   if( p[0] + kBallRadius > -0.5 * kTableLength ) return "illegal: the ball must be behind the end line (x < -1.37 m)";
   if( p[2] - kBallRadius < kTableHeight )         return "illegal: the ball must be above the playing surface (z > 0.78 m)";
   return "";
}

const char* serveLegality( const ServeParams& p )
{
   return serveLegalityAt( Vec3( p.pos[0], p.pos[1], p.pos[2] ) );
}

//! Toss mode: the point where the bat strikes the ball must be legal, and the toss high enough.
const char* tossLegality( const BatParams& b )
{
   if( b.tossSpeed * b.tossSpeed / ( 2.0 * kG ) < 0.16 ) return "illegal: the toss must rise at least 16 cm";
   if( b.contactZ > b.toss[2] + b.tossSpeed * b.tossSpeed / ( 2.0 * kG ) - 0.005 ) return "contact height above the toss apex";
   if( b.contactZ < b.toss[2] ) return "illegal: the ball must be struck on its way down, above the release height";
   return serveLegalityAt( Vec3( b.toss[0], b.toss[1], b.contactZ ) );
}



// The ghost serve values come from the batch sweep documented in README.md.
const Preset kPresets[] = {
   { "Ghost serve (backspin, comes back)", { { -1.75, 0.0, 1.05 }, { 2.9, 0.0, 2.25 }, { 0.0, -50.0, 0.0 } } },
   { "Flat serve (no spin)",               { { -1.75, 0.0, 1.05 }, { 4.0, 0.0, 1.00 }, { 0.0,   0.0, 0.0 } } },   // lands 0.40 m past the net, 7 cm over the tape
   { "Topspin serve (fast)",               { { -1.75, 0.0, 1.05 }, { 6.0, 0.0, -1.00 }, { 0.0,  40.0, 0.0 } } },  // lands 0.88 m past the net, 7 cm over the tape
   { "Sidespin serve (curves back)",       { { -1.75, 0.3, 1.05 }, { 4.0, -0.9, 1.00 }, { 0.0, -20.0, 40.0 } } }, // lands 0.53 m past the net near the centre line
   // Banana serve, best attempt so far: a slammed-down first bounce at the end line and side
   // edge, a high lob that passes 5 cm outside the table's width at the net line and curves
   // back to land 1.2 m deep. See README.md for why it is a lob in this flight model.
   { "Banana serve (right, high lob)",     { { -1.50, -0.66, 1.05 }, { 3.0, -1.0, -6.00 }, { 0.0, 30.0, 80.0 } } },
   { "Banana serve (left, high lob)",      { { -1.50, 0.66, 1.05 }, { 3.0, 1.0, -6.00 }, { 0.0, 30.0, -80.0 } } },
};
const int kNumPresets = static_cast<int>( sizeof( kPresets ) / sizeof( kPresets[0] ) );


//=================================================================================================
//  EVENTS
//=================================================================================================

enum EventKind { kBounceServer, kBounceOpponent, kNetCrossForward, kNetCrossBack, kNetHit, kEdgeOrNetTop, kFloor, kBatHit, kReceiverHit };

const char* eventName( EventKind k )
{
   switch( k ) {
      case kBounceServer:    return "bounce, server side";
      case kBounceOpponent:  return "bounce, opponent side";
      case kNetCrossForward: return "crosses the net (forward)";
      case kNetCrossBack:    return "crosses the net (back!)";
      case kNetHit:          return "hits the net";
      case kEdgeOrNetTop:    return "bounce off edge / net top";
      case kFloor:           return "floor";
      case kBatHit:          return "struck by the bat";
      case kReceiverHit:     return "struck by the receiving bat";
   }
   return "?";
}

struct Event {
   double    time;
   EventKind kind;
   Vec3      pos;
};


//=================================================================================================
//  SIMULATION
//=================================================================================================

struct Sim {
   WorldID             world      = nullptr;
   SphereID            ball       = nullptr;
   MaterialID          matBall    = 0;
   MaterialID          matTable   = 0;
   std::vector<BodyID> fixedBodies;

   ServeParams   serve;
   BatParams     bat;
   int           mode = kModeLaunch;
   UnionID       batBody   = nullptr;
   std::vector<BodyID> batParts;
   MaterialID    matBlade  = 0;
   MaterialID    matRubber = 0;
   MaterialID    matHandle = 0;
   double        contactTime = 0.0;    // toss mode: when the bat meets the ball
   Vec3          contactPoint;          // toss mode: ball centre at contact
   Vec3          batNormal;             // toss mode: blade normal (from the bat toward the ball)
   Vec3          batVel;                // toss mode: stroke velocity
   bool          batHit    = false;
   Vec3          afterHitVel, afterHitSpin;   // ball state right after the bat contact
   ReceiverParams receiver;
   bool          launchReceive = false;   // launch mode: add the receiving bat too
   UnionID       recBody = nullptr;
   std::vector<BodyID> recParts;
   bool          recPlaced = false;
   Vec3          recBallCentre;    // ball centre at the planned intercept (before the table clearance shift)
   Vec3          recFaceCentre;    // ball centre at the intercept as placed
   Vec3          recNormal;        // face normal, toward the incoming ball
   Vec3          recIncoming;      // horizontal unit direction of the incoming ball at the intercept
   std::string   recNote;          // placement notes
   bool          dryRun = false;   // placement run: receiver parked, no readouts
   std::vector<double> dryT;
   std::vector<Vec3>   dryPos, dryVel;
   bool          recHit = false;
   Vec3          afterRecVel, afterRecSpin;   // ball state right after the receiving bat
   double        recMaxHeight = 0.0;          // ball bottom above the table after the receive
   BallParams    ball_;        // staged in the GUI, applied by createBall() on the next serve / reset
   BallParams    ballApplied;
   AeroParams    aero;
   ContactParams contact;
   SolverParams  solver;

   double      time     = 0.0;
   long        steps    = 0;
   bool        running  = false;
   bool        served   = false;
   bool        settled  = false;
   bool        floored  = false;
   int         sampleEvery = 2;
   std::string error;
   Vec3        prevPos, prevVel;

   std::vector<Event>     events;
   std::vector<glm::vec3> trace;
   std::vector<glm::vec3> bouncePts;
   std::vector<double>    traceX, traceZ;
   double                 maxSpeed = 0.0;
   double                 maxHeight = 0.0;   // of the ball's bottom above the table top
   std::ofstream          dump;              // --dump: one CSV row per step (batch mode)
};

Sim& sim()
{
   static Sim s;
   return s;
}

//! The server strikes the ball with the bat (toss modes) instead of a prescribed launch.
bool usesBat()
{
   const Sim& s = sim();
   return s.mode == kModeToss || s.mode == kModeReceive;
}

//! The receiving bat is in play.
bool receiving()
{
   const Sim& s = sim();
   return s.mode == kModeReceive || ( s.mode == kModeLaunch && s.launchReceive );
}


const char* currentLegality()
{
   const Sim& s = sim();
   return usesBat() ? tossLegality( s.bat ) : serveLegality( s.serve );
}


void applyContactParams()
{
   Sim& s = sim();
   const real e( s.contact.cor ), mu( s.contact.mu );
   Material::setRestitution    ( s.matBall, s.matTable, e );
   Material::setStaticFriction ( s.matBall, s.matTable, mu );
   Material::setDynamicFriction( s.matBall, s.matTable, mu );
   if( s.matRubber != 0 ) {
      const real er( s.contact.corRubber ), mur( s.contact.muRubber );
      Material::setRestitution    ( s.matBall, s.matRubber, er );
      Material::setStaticFriction ( s.matBall, s.matRubber, mur );
      Material::setDynamicFriction( s.matBall, s.matRubber, mur );
      Material::setRestitution    ( s.matBall, s.matBlade, real( 0.5 ) );
      Material::setStaticFriction ( s.matBall, s.matBlade, real( 0.3 ) );
      Material::setDynamicFriction( s.matBall, s.matBlade, real( 0.3 ) );
      Material::setRestitution    ( s.matBall, s.matHandle, real( 0.5 ) );
      Material::setStaticFriction ( s.matBall, s.matHandle, real( 0.3 ) );
      Material::setDynamicFriction( s.matBall, s.matHandle, real( 0.3 ) );
   }
}


void applySolverParams()
{
   Sim& s = sim();
   CollisionSystemID cs = theCollisionSystem();
   cs->setMaxIterations( static_cast<size_t>( std::max( 1, s.solver.maxIterations ) ) );
   cs->setErrorReductionParameter( static_cast<real>( std::max( 0.0, std::min( 1.0, s.solver.erp ) ) ) );
   cs->setSplitImpulse( s.solver.splitImpulse );
   cs->setRestitution( s.solver.restitution );
   cs->setRestitutionPostPass( s.solver.restitutionPostPass );
   cs->setRestitutionIterations( static_cast<size_t>( std::max( 1, s.solver.restitutionIterations ) ) );
}


MaterialID makeMaterial( const char* name, double density )
{
   // density, cor, csf, cdf, poisson, young, stiffness, dampingN, dampingT
   return createMaterial( name, real( density ), real( 0.9 ), real( 0.15 ), real( 0.15 ),
                          real( 0.25 ), real( 300 ), real( 1e5 ), real( 10 ), real( 10 ) );
}


polyscope::SurfaceMesh* ballMesh = nullptr;
bool mirrorReady = false;

//! (Re)creates the ball from the staged ball parameters: a sphere of the ITTF radius whose
//! material density gives the requested mass for the requested wall thickness (0 = solid),
//! then Sphere::setShellThickness() for the shell inertia (2/3 m r^2 limit instead of 2/5).
void createBall()
{
   Sim& s = sim();
   Vec3 pos( s.serve.pos[0], s.serve.pos[1], s.serve.pos[2] );
   if( s.ball ) {
      pos = s.ball->getPosition();
      destroy( s.ball );
      s.ball = nullptr;
   }
   const double R = kBallRadius;
   const double t = std::max( 0.0, std::min( s.ball_.shellThickness, R ) );
   const double r = R - t;
   const double volume = 4.0 / 3.0 * kPi * ( t > 0.0 && t < R ? R * R * R - r * r * r : R * R * R );
   const double mass = std::max( 1e-6, s.ball_.mass );
   static int ballMaterials = 0;
   const std::string matName = "tt_ball_" + std::to_string( ballMaterials++ );
   s.matBall = makeMaterial( matName.c_str(), mass / volume );
   applyContactParams();
   s.ball = createSphere( 1000, pos, real( R ), s.matBall );
   if( t > 0.0 && t < R )
      s.ball->setShellThickness( real( t ) );
   s.ballApplied = s.ball_;
   s.ballApplied.shellThickness = t;
   s.prevPos = s.ball->getPosition();
   s.prevVel = s.ball->getLinearVel();
   if( mirrorReady ) {
      const viewer::ShapeMesh bm = viewer::makeBodyMesh( s.ball );
      ballMesh = polyscope::registerSurfaceMesh( "ball", bm.vertices, bm.faces );
      ballMesh->setSurfaceColor( glm::vec3( 1.0f, 0.62f, 0.10f ) );
      ballMesh->setSmoothShade( true );
   }
}


//! Blade normal and stroke velocity from the angles.
Vec3 bladeNormal( const BatParams& b )
{
   const double t = b.bladeTilt * kPi / 180.0, y = b.bladeYaw * kPi / 180.0;
   return Vec3( std::cos( t ) * std::cos( y ), std::cos( t ) * std::sin( y ), std::sin( t ) );
}

Vec3 strokeVelocity( const BatParams& b )
{
   const double e = b.strokeEl * kPi / 180.0, a = b.strokeAz * kPi / 180.0;
   return b.batSpeed * Vec3( std::cos( e ) * std::cos( a ), std::cos( e ) * std::sin( a ), std::sin( e ) );
}

//! Rotation taking the body-frame x axis (cylinder axis) onto \a n.
Quat orientationFor( const Vec3& n )
{
   const Vec3 x( 1, 0, 0 );
   Vec3 axis( x % n );
   const double sinA = axis.length(), cosA = trans( x ) * n;
   if( sinA < 1e-9 ) {
      if( cosA > 0 ) return Quat();
      return Quat( Vec3( 0, 0, 1 ), real( kPi ) );
   }
   axis /= real( sinA );
   return Quat( axis, real( std::atan2( sinA, cosA ) ) );
}

//! The bat: a union of the blade (heavy, so that the prescribed motion is not disturbed by the
//! ball), one rubber disc per side and a handle. Cylinder axes run along x; the blade normal is
//! the union's x axis. The union is a dynamic body whose pose and velocity are prescribed before
//! every step (a fixed body cannot carry a velocity in this build), its mass of about a tonne
//! makes the ball's reaction on it negligible.
void createBat()
{
   Sim& s = sim();
   if( s.batBody ) { destroy( s.batBody ); s.batBody = nullptr; s.batParts.clear(); }
   if( s.matBlade == 0 ) {
      s.matBlade  = makeMaterial( "tt_blade", 1.0e7 );    // the mass of the bat sits in the blade
      s.matRubber = makeMaterial( "tt_rubber", 1.0e3 );
      s.matHandle = makeMaterial( "tt_handle", 1.0e3 );   // light, so the centre of mass stays in the blade
   }
   const Vec3 pos( s.bat.toss[0] - 0.4, s.bat.toss[1], s.bat.toss[2] );
   UnionID bat = nullptr;
   pe_CREATE_UNION( u, 3000 )
   {
      CylinderID blade = createCylinder( 3001, pos, real( kBladeRadius ), real( kBladeThickness ), s.matBlade );
      CylinderID r1 = createCylinder( 3002, pos + Vec3( 0.5 * kBladeThickness + 0.5 * kRubberThickness, 0, 0 ), real( kBladeRadius ), real( kRubberThickness ), s.matRubber );
      CylinderID r2 = createCylinder( 3003, pos - Vec3( 0.5 * kBladeThickness + 0.5 * kRubberThickness, 0, 0 ), real( kBladeRadius ), real( kRubberThickness ), s.matRubber );
      CylinderID handle = createCylinder( 3004, pos - Vec3( 0, 0, kBladeRadius + 0.5 * kHandleLength - 0.01 ), real( kHandleRadius ), real( kHandleLength ), s.matHandle );
      handle->rotate( Vec3( 0, 1, 0 ), real( 0.5 * kPi ) );   // axis along z
      s.batParts = { blade, r1, r2, handle };
      bat = u;
   }
   s.batBody = bat;
   applyContactParams();
}


//! Swing frame: axis a, the in-plane "down" direction d0 (radial at the lowest point) and the
//! direction of motion at the lowest point e1 = a x d0.
void swingFrame( const BatParams& b, Vec3& a, Vec3& d0, Vec3& e1 )
{
   const double y = b.swingAxisYaw * kPi / 180.0, p = b.swingAxisPitch * kPi / 180.0;
   a = Vec3( std::cos( p ) * std::cos( y ), std::cos( p ) * std::sin( y ), std::sin( p ) );
   Vec3 down( 0, 0, -1 );
   down -= ( trans( down ) * a ) * a;
   if( down.length() < 1e-9 ) down = Vec3( 0, -1, 0 );
   d0 = down / down.length();
   e1 = a % d0;
}

//! Radial unit vector of the arm at swing angle theta (rad, from the lowest point).
Vec3 swingRadial( const Vec3& d0, const Vec3& e1, double theta )
{
   return std::cos( theta ) * d0 + std::sin( theta ) * e1;
}

//! Orientation with the blade normal \a n and the handle (body -z) along \a handleDir
//! (projected into the blade plane); minimal rotation when no handle direction is given.
Quat batOrientation( const Vec3& n, const Vec3* handleDir )
{
   const Quat q0 = orientationFor( n );
   if( !handleDir ) return q0;
   Vec3 h( *handleDir );
   h -= ( trans( h ) * n ) * n;
   if( h.length() < 1e-6 ) return q0;
   h /= h.length();
   const Vec3 h0( q0.rotate( Vec3( 0, 0, -1 ) ) );   // where the handle points after q0
   const double angle = std::atan2( trans( n ) * ( h0 % h ), trans( h0 ) * h );
   return Quat( n, real( angle ) ) * q0;
}


//! Pose and velocity of the bat at simulated time \a t of the current serve.
void driveBat( double t )
{
   Sim& s = sim();
   if( !s.batBody ) return;
   const BatParams& b = s.bat;
   // Re-plan the contact from the ball's actual state until the hit: the toss is slowed by
   // drag, and at bat speeds of several m/s a few milliseconds move the face centre by
   // centimetres, i.e. the difference between the face and the rim.
   if( s.served && !s.batHit && t >= 0.0 ) {
      const Vec3 p( s.ball->getPosition() ), v( s.ball->getLinearVel() );
      const double dz = p[2] - s.contactPoint[2];
      const double disc = v[2] * v[2] + 2.0 * kG * dz;
      if( disc >= 0.0 ) {
         const double tau = ( v[2] + std::sqrt( disc ) ) / kG;   // time until the falling ball reaches contactZ
         if( tau >= 0.0 ) {
            s.contactTime = t + tau;
            s.contactPoint[0] = p[0] + v[0] * tau;
            s.contactPoint[1] = p[1] + v[1] * tau;
         }
      }
   }
   const double t0 = s.contactTime - b.preTime, t1 = s.contactTime + b.postTime;
   const double tc = std::max( t0, std::min( t, t1 ) );
   const double tau = tc - s.contactTime;
   const bool moving = t >= t0 && t <= t1;
   const Vec3 centreAtContact( s.contactPoint - real( kBallRadius + kBatHalfThickness ) * s.batNormal );
   Vec3 centre( centreAtContact + real( tau ) * s.batVel );
   Vec3 vel( s.batVel );
   Vec3 w( 0, 0, 0 );
   Quat q;
   if( b.swingRate != 0.0 ) {
      // Pendulum: pivot P = contact-time centre - rho r(theta_c), translating with the arm.
      Vec3 a, d0, e1;
      swingFrame( b, a, d0, e1 );
      const double thetaC = b.swingPhase * kPi / 180.0, theta = thetaC + b.swingRate * tau;
      const Vec3 pivotC( centreAtContact - real( b.swingRadius ) * swingRadial( d0, e1, thetaC ) );
      centre = pivotC + real( tau ) * s.batVel + real( b.swingRadius ) * swingRadial( d0, e1, theta );
      vel    = s.batVel + real( b.swingRate * b.swingRadius ) * ( -std::sin( theta ) * d0 + std::cos( theta ) * e1 );
      w      = real( b.swingRate ) * a;
      const Vec3 toPivot( -swingRadial( d0, e1, thetaC ) );   // handle along the arm at contact
      const Quat qC = batOrientation( s.batNormal, &toPivot );
      q = Quat( a, real( b.swingRate * tau ) ) * qC;
   }
   else {
      Vec3 n( s.batNormal );
      if( b.wristRate != 0.0 ) {
         // Rotate the normal about z by wristRate * (t - contactTime) (zero rotation at contact).
         const double ang = b.wristRate * tau;
         n = Vec3( std::cos( ang ) * n[0] - std::sin( ang ) * n[1], std::sin( ang ) * n[0] + std::cos( ang ) * n[1], n[2] );
         w = Vec3( 0, 0, b.wristRate );
      }
      const Vec3 up( 0, 0, 1 );
      q = batOrientation( n, b.handleUp ? &up : nullptr );
   }
   // setPosition() places the union's centre of mass; orient first, then shift so that the
   // blade's centre (the first part) is at the required point.
   s.batBody->setOrientation( q );
   const Vec3 bladeOffset( s.batParts[0]->getPosition() - s.batBody->getPosition() );
   s.batBody->setPosition( centre - bladeOffset );
   s.batBody->setLinearVel( moving ? vel : Vec3( 0, 0, 0 ) );
   s.batBody->setAngularVel( moving ? w : Vec3( 0, 0, 0 ) );
}


//! Toss kinematics: contact time at which the falling ball passes contactZ (no drag).
void planStroke()
{
   Sim& s = sim();
   const BatParams& b = s.bat;
   const double apex = b.toss[2] + b.tossSpeed * b.tossSpeed / ( 2.0 * kG );
   const double drop = std::max( 0.0, apex - b.contactZ );
   s.contactTime  = b.tossSpeed / kG + std::sqrt( 2.0 * drop / kG );
   s.contactPoint = Vec3( b.toss[0], b.toss[1], std::min( b.contactZ, apex ) );
   s.batNormal    = bladeNormal( b );
   s.batVel       = strokeVelocity( b );
}


//! The receiving bat: the same union as the server's bat, but fixed (it is never moved during
//! a step), so the solver treats it as an infinitely heavy, resting surface. The face toward the
//! ball is the first rubber (body +x side).
void createReceiver()
{
   Sim& s = sim();
   if( s.recBody ) { destroy( s.recBody ); s.recBody = nullptr; s.recParts.clear(); }
   const Vec3 pos( 0.0, 6.0, 1.0 );   // parked
   UnionID bat = nullptr;
   pe_CREATE_UNION( u, 4000 )
   {
      CylinderID blade = createCylinder( 4001, pos, real( kBladeRadius ), real( kBladeThickness ), s.matBlade );
      CylinderID r1 = createCylinder( 4002, pos + Vec3( 0.5 * kBladeThickness + 0.5 * kRubberThickness, 0, 0 ), real( kBladeRadius ), real( kRubberThickness ), s.matRubber );
      CylinderID r2 = createCylinder( 4003, pos - Vec3( 0.5 * kBladeThickness + 0.5 * kRubberThickness, 0, 0 ), real( kBladeRadius ), real( kRubberThickness ), s.matRubber );
      CylinderID handle = createCylinder( 4004, pos - Vec3( 0, 0, kBladeRadius + 0.5 * kHandleLength - 0.01 ), real( kHandleRadius ), real( kHandleLength ), s.matHandle );
      handle->rotate( Vec3( 0, 1, 0 ), real( 0.5 * kPi ) );   // axis along z
      s.recParts = { blade, r1, r2, handle };
      bat = u;
   }
   bat->setFixed( true );
   s.recBody = bat;
   s.recPlaced = false;
}


//! Face normal of the receiving bat for the incoming horizontal direction \a d (unit vector):
//! against the ball, then tilted above the horizontal and yawed about z.
Vec3 receiverNormal( const ReceiverParams& r, const Vec3& d )
{
   const Vec3 b( -d );                       // toward the incoming ball
   const Vec3 h( Vec3( 0, 0, 1 ) % b );      // horizontal, to the receiver's left
   const double t = r.tilt * kPi / 180.0, y = r.yaw * kPi / 180.0;
   return std::cos( t ) * ( std::cos( y ) * b - std::sin( y ) * h ) + std::sin( t ) * Vec3( 0, 0, 1 );
}


//! Blade centre for a ball centred at \a ballCentre touching the middle of a face with normal
//! \a n. If the blade would dip into the table top the bat is raised just enough (\a raise), so
//! the ball then meets the face below its middle.
Vec3 receiverBladeCentre( const Vec3& ballCentre, const Vec3& n, double& raise )
{
   const Vec3 centre0( ballCentre - real( kBallRadius + kBatHalfThickness ) * n );
   const double nz = std::max( -1.0, std::min( 1.0, static_cast<double>( n[2] ) ) );
   const double dropBelowCentre = kBladeRadius * std::sqrt( 1.0 - nz * nz ) + kBatHalfThickness * std::fabs( nz );
   const bool overTable = std::fabs( centre0[0] ) < 0.5 * kTableLength + kBladeRadius && std::fabs( centre0[1] ) < 0.5 * kTableWidth + kBladeRadius;
   const double lowest = centre0[2] - dropBelowCentre;
   raise = ( overTable && lowest < kTableHeight + 0.005 ) ? kTableHeight + 0.005 - lowest : 0.0;
   return centre0 + Vec3( 0, 0, raise );
}


//! True if a ball centred at \a p would touch the blade disc (centre \a c, normal \a n) within
//! \a margin.
bool touchesBlade( const Vec3& p, const Vec3& c, const Vec3& n, double margin )
{
   const Vec3 r( p - c );
   const double axial = std::fabs( trans( r ) * n );
   const Vec3 radial( r - real( trans( r ) * n ) * n );
   return axial < kBallRadius + kBatHalfThickness + margin && radial.length() < kBladeRadius + kBallRadius + margin;
}


//! Puts the receiving bat so that a ball centred at \a ballCentre touches the middle of the face
//! whose normal is \a n (raised if needed, see receiverBladeCentre()); the handle points to the
//! receiver's right (z x d, projected into the blade plane).
void poseReceiver( const Vec3& ballCentre, const Vec3& n, const Vec3& d )
{
   Sim& s = sim();
   if( !s.recBody ) return;
   double raise = 0.0;
   const Vec3 centre( receiverBladeCentre( ballCentre, n, raise ) );
   char note[160] = "";
   if( raise > 0.0 )
      std::snprintf( note, sizeof( note ), "raised %.1f cm so that the blade clears the table%s", 100.0 * raise,
                     raise > kBladeRadius - kBallRadius ? " (the ball may pass under the face)" : "" );
   Vec3 handleDir( Vec3( 0, 0, 1 ) % d );
   if( handleDir.length() < 1e-6 ) handleDir = Vec3( 0, 1, 0 );
   const Quat q = batOrientation( n, &handleDir );
   s.recBody->setOrientation( q );
   const Vec3 bladeOffset( s.recParts[0]->getPosition() - s.recBody->getPosition() );
   s.recBody->setPosition( centre - bladeOffset );
   s.recBallCentre = ballCentre;
   s.recFaceCentre = ballCentre + Vec3( 0, 0, raise );
   s.recNormal     = n;
   s.recIncoming   = d;
   s.recNote       = note;
}


//! Re-applies the angles to the planned intercept (live angle changes).
void reposeReceiver()
{
   Sim& s = sim();
   if( s.recPlaced )
      poseReceiver( s.recBallCentre, receiverNormal( s.receiver, s.recIncoming ), s.recIncoming );
}


void parkReceiver()
{
   Sim& s = sim();
   if( !s.recBody ) return;
   s.recBody->setOrientation( Quat() );
   const Vec3 bladeOffset( s.recParts[0]->getPosition() - s.recBody->getPosition() );
   s.recBody->setPosition( Vec3( 0.0, 6.0, 1.0 ) - bladeOffset );
   s.recPlaced = false;
}


bool ballParamsChanged()
{
   const Sim& s = sim();
   return s.ball_.mass != s.ballApplied.mass || s.ball_.shellThickness != s.ballApplied.shellThickness;
}


void buildScene()
{
   Sim& s = sim();
   s.world = theWorld();
   s.world->setGravity( 0, 0, -kG );

   s.matTable = makeMaterial( "tt_table", 700.0 );
   applySolverParams();

   pe::id_t id = 1;
   PlaneID floor = createPlane( id++, 0, 0, 1, 0, s.matTable );
   s.fixedBodies.push_back( floor );

   BoxID top = createBox( id++, Vec3( 0, 0, kTableHeight - 0.5 * kTopThickness ),
                          Vec3( kTableLength, kTableWidth, kTopThickness ), s.matTable );
   top->setFixed( true );
   s.fixedBodies.push_back( top );

   BoxID net = createBox( id++, Vec3( 0, 0, kTableHeight + 0.5 * kNetHeight ),
                          Vec3( kNetThickness, kNetWidth, kNetHeight ), s.matTable );
   net->setFixed( true );
   s.fixedBodies.push_back( net );

   // Cosmetic legs.
   const double legH = kTableHeight - kTopThickness, legW = 0.05, inset = 0.12;
   for( int ix = -1; ix <= 1; ix += 2 ) {
      for( int iy = -1; iy <= 1; iy += 2 ) {
         BoxID leg = createBox( id++, Vec3( ix * ( 0.5 * kTableLength - inset ), iy * ( 0.5 * kTableWidth - inset ), 0.5 * legH ),
                                Vec3( legW, legW, legH ), s.matTable );
         leg->setFixed( true );
         s.fixedBodies.push_back( leg );
      }
   }

   createBall();
   createBat();
   createReceiver();
   planStroke();
   driveBat( -1.0 );
}


void placeBall()
{
   Sim& s = sim();
   if( ballParamsChanged() )
      createBall();
   if( usesBat() )
      s.ball->setPosition( real( s.bat.toss[0] ), real( s.bat.toss[1] ), real( s.bat.toss[2] ) );
   else
      s.ball->setPosition( real( s.serve.pos[0] ), real( s.serve.pos[1] ), real( s.serve.pos[2] ) );
   s.ball->setLinearVel( 0, 0, 0 );
   s.ball->setAngularVel( 0, 0, 0 );
   s.prevPos = s.ball->getPosition();
   s.prevVel = s.ball->getLinearVel();
}


void clearRun()
{
   Sim& s = sim();
   s.time = 0.0;
   s.steps = 0;
   s.served = false;
   s.settled = false;
   s.floored = false;
   s.error.clear();
   s.events.clear();
   s.trace.clear();
   s.bouncePts.clear();
   s.traceX.clear();
   s.traceZ.clear();
   s.maxSpeed = 0.0;
   s.maxHeight = 0.0;
   s.batHit = false;
   s.recHit = false;
   s.recMaxHeight = 0.0;
}


void placeReceiver();

//! Ball back to the serve position at rest; the run is cleared. In receive mode the receiving
//! bat is (re)placed for the current serve.
void resetBall()
{
   Sim& s = sim();
   s.running = false;
   if( receiving() ) placeReceiver(); else parkReceiver();
   clearRun();
   placeBall();
   if( usesBat() ) { planStroke(); driveBat( -1.0 ); }
}


//! Launches the ball from the serve position with the serve velocity and spin (or tosses it
//! and starts the stroke).
void startServe()
{
   Sim& s = sim();
   clearRun();
   placeBall();
   if( usesBat() ) {
      planStroke();
      driveBat( 0.0 );
      s.ball->setLinearVel( 0, 0, real( s.bat.tossSpeed ) );
      s.ball->setAngularVel( 0, 0, 0 );
   }
   else {
      s.ball->setLinearVel( real( s.serve.vel[0] ), real( s.serve.vel[1] ), real( s.serve.vel[2] ) );
      s.ball->setAngularVel( real( 2.0 * kPi * s.serve.spin[0] ), real( 2.0 * kPi * s.serve.spin[1] ), real( 2.0 * kPi * s.serve.spin[2] ) );
   }
   s.prevPos = s.ball->getPosition();
   s.prevVel = s.ball->getLinearVel();
   s.served = true;
   s.running = true;
   s.trace.push_back( viewer::toGlm( s.prevPos ) );
   s.traceX.push_back( s.prevPos[0] );
   s.traceZ.push_back( s.prevPos[2] );
}


//! Quadratic drag and Magnus lift on the ball, as external forces for the next step.
void applyAerodynamics()
{
   Sim& s = sim();
   const Vec3 v( s.ball->getLinearVel() );
   const Vec3 w( s.ball->getAngularVel() );
   const double speed = v.length();
   const double area  = kPi * kBallRadius * kBallRadius;
   if( speed > 1e-9 ) {
      Vec3 f;
      if( s.aero.drag )
         f -= real( 0.5 * s.aero.rhoAir * s.aero.cd * area * speed ) * v;
      const double wmag = w.length();
      if( s.aero.magnus && wmag > 1e-9 ) {
         // C_L = 1 / ( 2 + v / ( r omega ) ) (Nathan 2008 form), F = 1/2 rho A C_L v^2 (omega x v) / ( |omega| |v| ).
         const double spinRatio = kBallRadius * wmag / speed;
         const double cl = s.aero.liftScale / ( 2.0 + 1.0 / spinRatio );
         f += real( 0.5 * s.aero.rhoAir * area * cl * speed / wmag ) * ( w % v );
      }
      s.ball->addForce( f );
   }
   if( s.aero.spinDecay > 0.0 && w.length() > 1e-9 )
      s.ball->addTorque( real( -1.0 / s.aero.spinDecay ) * ( s.ball->getInertia() * w ) );
}


void recordStep()
{
   Sim& s = sim();
   const Vec3 p( s.ball->getPosition() );
   const Vec3 v( s.ball->getLinearVel() );
   const double bottom = p[2] - kBallRadius;

   if( s.dryRun ) {
      s.dryT.push_back( s.time );
      s.dryPos.push_back( p );
      s.dryVel.push_back( v );
   }

   // Receiving bat: a velocity jump next to its face.
   if( receiving() && !s.dryRun && s.recPlaced && !s.recHit && ( v - s.prevVel ).length() > 0.5 &&
       ( p - s.recFaceCentre ).length() < kBladeRadius + kBallRadius + 0.03 ) {
      s.recHit = true;
      s.afterRecVel = v;
      s.afterRecSpin = s.ball->getAngularVel() / real( 2.0 * kPi );
      s.events.push_back( { s.time, kReceiverHit, p } );
      s.prevPos = p;
      s.prevVel = v;
      return;
   }
   if( s.recHit )
      s.recMaxHeight = std::max( s.recMaxHeight, bottom - kTableHeight );

   // Bat contact: a velocity jump while the ball is not near the table or the floor.
   const bool nearTable = bottom - kTableHeight < 0.03 && std::fabs( p[0] ) <= 0.5 * kTableLength + 0.05 && std::fabs( p[1] ) <= 0.5 * kTableWidth + 0.05;
   if( usesBat() && !s.batHit && !nearTable && bottom > 0.05 && ( v - s.prevVel ).length() > 0.5 ) {
      s.batHit = true;
      s.afterHitVel = v;
      s.afterHitSpin = s.ball->getAngularVel() / real( 2.0 * kPi );
      s.events.push_back( { s.time, kBatHit, p } );
      s.prevPos = p;
      s.prevVel = v;
      return;
   }

   if( s.prevPos[0] < 0 && p[0] >= 0 )      s.events.push_back( { s.time, kNetCrossForward, p } );
   else if( s.prevPos[0] > 0 && p[0] <= 0 ) s.events.push_back( { s.time, kNetCrossBack, p } );

   if( s.prevVel[2] < -0.1 && v[2] > 0 ) {   // impacts slower than 0.1 m/s are the ball settling / dribbling
      const bool overTable = std::fabs( p[0] ) <= 0.5 * kTableLength + kBallRadius && std::fabs( p[1] ) <= 0.5 * kTableWidth + kBallRadius;
      if( overTable && std::fabs( bottom - kTableHeight ) < 0.02 ) {
         s.events.push_back( { s.time, p[0] < 0 ? kBounceServer : kBounceOpponent, p } );
         s.bouncePts.push_back( viewer::toGlm( Vec3( p[0], p[1], real( kTableHeight ) ) ) );
      }
      else if( bottom < 0.02 ) {
         s.bouncePts.push_back( viewer::toGlm( Vec3( p[0], p[1], 0 ) ) );
      }
      else {
         s.events.push_back( { s.time, kEdgeOrNetTop, p } );
         s.bouncePts.push_back( viewer::toGlm( p ) );
      }
   }

   if( std::fabs( p[0] ) < 0.06 && bottom < kNetTop && bottom > kTableHeight - 0.01 &&
       ( ( s.prevVel[0] > 0.1 && v[0] <= 0 ) || ( s.prevVel[0] < -0.1 && v[0] >= 0 ) ) )
      s.events.push_back( { s.time, kNetHit, p } );

   if( !s.floored && bottom < 0.02 ) {
      s.floored = true;
      s.events.push_back( { s.time, kFloor, p } );
   }

   s.maxSpeed  = std::max( s.maxSpeed, static_cast<double>( v.length() ) );
   s.maxHeight = std::max( s.maxHeight, bottom - kTableHeight );
   if( s.dump.is_open() && !s.dryRun ) {
      const Vec3 w( s.ball->getAngularVel() );
      s.dump << s.time << ',' << p[0] << ',' << p[1] << ',' << p[2] << ',' << v[0] << ',' << v[1] << ',' << v[2]
             << ',' << w[0] / ( 2 * kPi ) << ',' << w[1] / ( 2 * kPi ) << ',' << w[2] / ( 2 * kPi ) << '\n';
   }

   if( s.steps % s.sampleEvery == 0 ) {
      s.trace.push_back( viewer::toGlm( p ) );
      s.traceX.push_back( p[0] );
      s.traceZ.push_back( p[2] );
   }

   const bool outOfBounds = std::fabs( p[0] ) > 8.0 || std::fabs( p[1] ) > 5.0;
   if( outOfBounds || ( s.floored && bottom < 0.005 && v.length() < 0.05 ) ) {
      s.settled = true;
      s.running = false;
   }

   s.prevPos = p;
   s.prevVel = v;
}


void advance( int n )
{
   Sim& s = sim();
   if( !s.error.empty() || s.settled ) return;
   try {
      for( int i = 0; i < n; ++i ) {
         if( usesBat() ) driveBat( s.time );
         applyAerodynamics();
         s.world->simulationStep( real( s.solver.dt ) );
         s.time += s.solver.dt;
         ++s.steps;
         recordStep();
         const Vec3& p = s.ball->getPosition();
         if( !std::isfinite( p[0] ) || !std::isfinite( p[1] ) || !std::isfinite( p[2] ) ) {
            s.error = "non-finite ball position";
            s.running = false;
            return;
         }
         if( s.settled ) return;
      }
   }
   catch( const std::exception& e ) {
      s.error = e.what();
      s.running = false;
   }
}


//! Places the receiving bat for the current serve. Automatic placement runs the serve once with
//! the bat parked and considers every sample between the first bounce on the receiver's side
//! and the next bounce (or the floor) as an intercept: the face against the ball's horizontal
//! direction, tilted and yawed as requested. A candidate is valid if the bat so placed does not
//! stand in the ball's own path before that moment (a ghost serve that comes back toward the net
//! can only be met behind the net over the server's side, where a receiver legally reaches
//! over) and keeps the blade out of the net. Among the valid candidates the highest is the
//! apex; the bat is put at the requested fraction of the time from the bounce to it.
void placeReceiver()
{
   Sim& s = sim();
   parkReceiver();
   s.recNote.clear();
   if( !s.receiver.autoPlace ) {
      const Vec3 d( 1, 0, 0 );
      poseReceiver( Vec3( s.receiver.pos[0], s.receiver.pos[1], s.receiver.pos[2] ), receiverNormal( s.receiver, d ), d );
      s.recPlaced = true;
      return;
   }

   // Dry run until the first bounce on the receiver's side is followed by another bounce, the
   // floor or an edge (net crossings do not end the window).
   const bool wasRunning = s.running;
   s.dryRun = true;
   s.dryT.clear(); s.dryPos.clear(); s.dryVel.clear();
   startServe();
   double tb = -1.0, te = -1.0;
   while( s.running && s.time < 4.0 ) {
      advance( 200 );
      tb = te = -1.0;
      for( const Event& e : s.events ) {
         if( tb < 0.0 ) { if( e.kind == kBounceOpponent ) tb = e.time; }
         else if( e.kind == kBounceOpponent || e.kind == kBounceServer || e.kind == kFloor || e.kind == kEdgeOrNetTop || e.kind == kNetHit ) { te = e.time; break; }
      }
      if( te >= 0.0 ) break;
   }
   s.dryRun = false;
   s.running = wasRunning;
   if( te < 0.0 ) te = s.time + 1.0;

   if( tb < 0.0 ) {
      s.recNote = "the serve does not bounce on the receiver's side: receiving bat parked";
      clearRun();
      placeBall();
      return;
   }

   auto horizontal = []( const Vec3& v ) {
      Vec3 d( v[0], v[1], 0.0 );
      if( d.length() < 0.05 ) return Vec3( 1, 0, 0 );
      return Vec3( d / d.length() );
   };
   // Obstruction of candidate i: -1 if the blade would straddle the net plane, otherwise the
   // number of earlier samples (more than 10 ms before t_i) at which the ball would touch the
   // blade from behind or at the rim; touching the face earlier is just an earlier receive.
   auto obstruction = [&]( int i ) {
      const Vec3 n( receiverNormal( s.receiver, horizontal( s.dryVel[i] ) ) );
      double raise = 0.0;
      const Vec3 c( receiverBladeCentre( s.dryPos[i], n, raise ) );
      const double nx = std::max( -1.0, std::min( 1.0, static_cast<double>( n[0] ) ) );
      const double extentX = kBladeRadius * std::sqrt( 1.0 - nx * nx ) + kBatHalfThickness * std::fabs( nx );
      if( std::fabs( c[0] ) - extentX < 0.5 * kNetThickness + 0.005 ) return -1;
      int touches = 0;
      for( size_t j = 0; j < s.dryT.size() && s.dryT[j] < s.dryT[i] - 0.01; ++j ) {
         if( !touchesBlade( s.dryPos[j], c, n, 0.005 ) ) continue;
         const Vec3 r( s.dryPos[j] - c );
         const double axial = trans( r ) * n;
         const Vec3 radial( r - real( axial ) * n );
         const bool onTheFace = axial > 0.0 && radial.length() < kBladeRadius - 0.005;
         if( !onTheFace ) ++touches;
      }
      return touches;
   };
   auto valid = [&]( int i ) { return obstruction( i ) == 0; };

   int apex = -1, apexAny = -1, apexOwnSide = -1;   // own side: over the receiver's half, clear of the net
   const int stride = 4;
   for( int i = 0; i < static_cast<int>( s.dryT.size() ); i += stride ) {
      if( s.dryT[i] <= tb || s.dryT[i] > te ) continue;
      const int o = obstruction( i );
      if( o < 0 ) continue;
      if( apexAny < 0 || s.dryPos[i][2] > s.dryPos[apexAny][2] ) apexAny = i;
      if( s.dryPos[i][0] > 0.1 && ( apexOwnSide < 0 || s.dryPos[i][2] > s.dryPos[apexOwnSide][2] ) ) apexOwnSide = i;
      if( o == 0 && ( apex < 0 || s.dryPos[i][2] > s.dryPos[apex][2] ) ) apex = i;
   }
   if( apexOwnSide >= 0 ) apexAny = apexOwnSide;
   if( apexAny < 0 ) {
      s.recNote = "no place for the bat after the bounce on the receiver's side: receiving bat parked";
      clearRun();
      placeBall();
      return;
   }
   std::string note;
   const bool obstructed = apex < 0;
   if( obstructed ) {
      // The ball comes back up through the column it came down in (near-vertical rebound of a
      // heavy backspin serve): a held bat is always in the serve's way. Take the apex anyway;
      // the serve will strike the back of the bat, and the verdict says so.
      apex = apexAny;
      note = "no held bat can meet this ball without standing in the serve's own path: placed at the apex, the serve will strike the back of the bat; ";
   }
   const double f = std::max( 0.05, std::min( 1.0, s.receiver.riseFraction ) );
   const double tTarget = tb + f * ( s.dryT[apex] - tb );
   int pick = apex;
   for( int i = 0; i < static_cast<int>( s.dryT.size() ); i += stride )
      if( s.dryT[i] > tb && s.dryT[i] <= s.dryT[apex] && ( obstructed || valid( i ) ) && std::fabs( s.dryT[i] - tTarget ) < std::fabs( s.dryT[pick] - tTarget ) ) pick = i;
   if( s.dryPos[pick][0] < 0.0 ) note += "the ball comes back toward the net: met behind the net over the server's side (reaching over); ";

   const Vec3 d( horizontal( s.dryVel[pick] ) );
   poseReceiver( s.dryPos[pick], receiverNormal( s.receiver, d ), d );
   s.recNote = note + s.recNote;
   s.recPlaced = true;
   clearRun();
   placeBall();
}


//! Serve: in receive mode the receiving bat is placed first (dry run), then the serve starts.
void launchServe()
{
   if( receiving() ) placeReceiver(); else parkReceiver();
   startServe();
}


//! Ghost serve verdict from the event list: [server-side bounce] -> forward net crossing -> one
//! opponent-side bounce -> backward net crossing, with no net hit / floor / second bounce before.
struct Verdict {
   bool        ghost = false;
   std::string text;
};

Verdict finish( Verdict v, bool ghost )
{
   v.ghost = ghost;
   return v;
}

Verdict evaluate()
{
   const Sim& s = sim();
   Verdict v;
   int stage = 0;   // 0: before the net, 1: crossed, 2: bounced on the opponent's side, 3: came back
   double backHeight = 0.0;
   bool ownSideFirst = false, received = false;
   for( const Event& e : s.events ) {
      if( e.kind == kReceiverHit ) { received = true; break; }   // what follows is the return
      switch( e.kind ) {
         case kBounceServer:
            if( stage == 0 ) { ownSideFirst = true; break; }
            if( stage == 3 ) { v.text = "ghost serve: came back and bounced on the server's side"; return finish( v, true ); }
            v.text = "double bounce on the server's side";
            return v;
         case kNetCrossForward:
            if( stage == 0 ) { stage = 1; break; }
            v.text = "crossed forward again";
            return v;
         case kBounceOpponent:
            if( stage == 1 ) { stage = 2; break; }
            if( stage == 2 ) { v.text = "bounced twice on the opponent's side: no return"; return v; }
            v.text = "bounced on the opponent's side without crossing the net first";
            return v;
         case kNetCrossBack:
            if( stage == 2 ) { stage = 3; backHeight = e.pos[2] - kBallRadius - kNetTop; break; }
            v.text = "came back without a bounce on the opponent's side";
            return v;
         case kNetHit:
            v.text = stage >= 2 ? "came back but hit the net" : "hit the net";
            return v;
         case kEdgeOrNetTop:
            v.text = "clipped an edge / the net top";
            return v;
         case kFloor:
            if( stage == 3 ) { v.text = "ghost serve: came back over the net, then fell off the table"; return finish( v, true ); }
            v.text = stage == 2 ? "did not come back (floor)" : "missed the table";
            return v;
         case kBatHit:
         case kReceiverHit:
            break;
      }
   }
   switch( stage ) {
      case 0: v.text = ownSideFirst ? "bounced on the server's side, heading to the net" : "in flight toward the net"; break;
      case 1: v.text = "over the net, not bounced yet"; break;
      case 2: v.text = received ? "bounced on the receiver's side and was blocked" : "bounced on the opponent's side, waiting for the return"; break;
      case 3: {
         char buf[160];
         std::snprintf( buf, sizeof( buf ), "GHOST SERVE: came back over the net, %.1f cm above it", 100.0 * backHeight );
         v.text = buf;
         v.ghost = true;
         break;
      }
   }
   if( !ownSideFirst && stage >= 1 ) v.text += " (no own-side bounce first: illegal serve)";
   return v;
}

//! A regular (non-ghost) serve is good if it bounces on the server's side, crosses the net and
//! bounces on the opponent's side, without touching the net before that. Returns the net
//! clearance of the forward crossing (ball bottom above the tape), or -1 if the serve is not good.
double goodServeClearance()
{
   const Sim& s = sim();
   int stage = 0;
   double clearance = -1.0;
   for( const Event& e : s.events ) {
      if( e.kind == kNetHit || e.kind == kEdgeOrNetTop || e.kind == kFloor ) return -1.0;
      if( stage == 0 && e.kind == kBounceServer ) stage = 1;
      else if( stage == 1 && e.kind == kNetCrossForward ) { stage = 2; clearance = e.pos[2] - kBallRadius - kNetTop; }
      else if( stage == 2 && e.kind == kBounceOpponent ) return clearance;
      else if( e.kind == kBounceServer || e.kind == kBounceOpponent || e.kind == kNetCrossForward || e.kind == kNetCrossBack ) return -1.0;
   }
   return -1.0;
}


//! Banana serve: a good serve whose first bounce is near a side edge and which crosses the net
//! line outside the table's width (|y| > 0.7625 m; beyond the net posts, |y| > 0.915 m, it goes
//! around the net) before the sidespin curves it back onto the opponent's side.
struct Banana {
   bool   banana      = false;
   bool   aroundNet   = false;
   double firstBounceY = 0.0;
   double crossY      = 0.0;
   double landX       = 0.0;
   double landY       = 0.0;
};

Banana bananaCheck()
{
   const Sim& s = sim();
   Banana b;
   if( goodServeClearance() < 0.0 ) return b;
   int stage = 0;
   for( const Event& e : s.events ) {
      if( stage == 0 && e.kind == kBounceServer )       { b.firstBounceY = e.pos[1]; stage = 1; }
      else if( stage == 1 && e.kind == kNetCrossForward ) { b.crossY = e.pos[1]; stage = 2; }
      else if( stage == 2 && e.kind == kBounceOpponent )  { b.landX = e.pos[0]; b.landY = e.pos[1]; stage = 3; break; }
   }
   if( stage < 3 ) return b;
   b.aroundNet = std::fabs( b.crossY ) > 0.5 * kNetWidth;
   // Ball centre outside the table's width at the net line, both bounces with the centre over
   // the table, the first one near a side edge.
   b.banana = std::fabs( b.crossY ) > 0.5 * kTableWidth && std::fabs( b.firstBounceY ) > 0.45
           && std::fabs( b.firstBounceY ) < 0.5 * kTableWidth && std::fabs( b.landY ) < 0.5 * kTableWidth;
   return b;
}


//! What became of the receiver's block: the events after the receiving-bat contact.
struct Receive {
   bool        hit     = false;
   int         outcome = 0;   // 0 no contact / in flight, 1 good return, 2 into the net, 3 over the net but off the table, 4 dumped on the receiver's side, 5 off before the net, 6 clipped, 7 struck before the bounce
   double      netClearance = -1.0;   // of the return crossing
   double      landX = 0.0, landY = 0.0;
   std::string text;
};

Receive receiveVerdict()
{
   const Sim& s = sim();
   Receive r;
   bool crossed = false, reachedOver = false, bounced = false;   // reached over: the block happened over the server's side
   for( const Event& e : s.events ) {
      if( !r.hit ) {
         bounced |= e.kind == kBounceOpponent;
         if( e.kind == kReceiverHit ) {
            r.hit = true;
            reachedOver = e.pos[0] < 0.0;
            if( !bounced ) { r.outcome = 7; r.text = "the bat stood in the serve's path: struck before the ball bounced (no receive)"; return r; }
         }
         continue;
      }
      char buf[200];
      switch( e.kind ) {
         case kNetHit:
            r.outcome = 2; r.text = "return: into the net"; return r;
         case kNetCrossBack:
            crossed = true; r.netClearance = e.pos[2] - kBallRadius - kNetTop; break;
         case kNetCrossForward:
            if( reachedOver ) { r.outcome = 4; r.text = "return: back over the net to the receiver's own side"; return r; }
            break;
         case kBounceServer:
            if( crossed || reachedOver ) {
               r.outcome = 1; r.landX = e.pos[0]; r.landY = e.pos[1];
               if( crossed ) std::snprintf( buf, sizeof( buf ), "return: over the net (%.1f cm clearance), landed %.2f m from the end line at y = %+.2f", 100.0 * r.netClearance, 0.5 * kTableLength + e.pos[0], e.pos[1] );
               else std::snprintf( buf, sizeof( buf ), "return (reached over the net): landed %.2f m from the end line at y = %+.2f", 0.5 * kTableLength + e.pos[0], e.pos[1] );
               r.text = buf; return r;
            }
            break;
         case kBounceOpponent:
            r.outcome = 4; r.text = "return: bounced on the receiver's own side"; return r;
         case kFloor:
            r.outcome = ( crossed || reachedOver ) ? 3 : 5;
            if( crossed ) std::snprintf( buf, sizeof( buf ), "return: over the net (%.1f cm clearance) but off the table (long / wide)", 100.0 * r.netClearance );
            else if( reachedOver ) std::snprintf( buf, sizeof( buf ), "return (reached over the net): off the table (long / wide)" );
            else std::snprintf( buf, sizeof( buf ), "return: off the table without reaching the net" );
            r.text = buf; return r;
         case kEdgeOrNetTop:
            r.outcome = 6; r.text = "return: clipped the net top / an edge"; return r;
         default:
            break;
      }
   }
   r.text = r.hit ? ( crossed ? "return: over the net, in flight" : "return: in flight" ) : "the ball did not reach the receiving bat";
   return r;
}


//! evaluate() plus the launch-point legality.
Verdict evaluateWithLegality()
{
   Verdict v = evaluate();
   const char* illegal = currentLegality();
   if( *illegal ) v.text = std::string( illegal ) + "; " + v.text;
   return v;
}

//=================================================================================================
//  POLYSCOPE MIRROR
//=================================================================================================

glm::vec3 fixedColor( size_t i )
{
   if( i == 0 ) return glm::vec3( 0.80f, 0.80f, 0.78f );   // floor
   if( i == 1 ) return glm::vec3( 0.08f, 0.30f, 0.62f );   // table top
   if( i == 2 ) return glm::vec3( 0.20f, 0.20f, 0.22f );   // net
   return glm::vec3( 0.45f, 0.45f, 0.48f );                // legs
}


void registerScene()
{
   Sim& s = sim();
   for( size_t i = 0; i < s.fixedBodies.size(); ++i ) {
      BodyID b = s.fixedBodies[i];
      const viewer::ShapeMesh m = viewer::makeBodyMesh( b, 3.0 );
      polyscope::SurfaceMesh* mesh = polyscope::registerSurfaceMesh( "fixed " + std::to_string( i ), m.vertices, m.faces );
      mesh->setTransform( viewer::bodyTransform( b ) );
      mesh->setSurfaceColor( fixedColor( i ) );
      mesh->setEdgeWidth( i == 0 ? 0.0 : 1.0 );
      mesh->setSmoothShade( false );
   }
   mirrorReady = true;
   const viewer::ShapeMesh bm = viewer::makeBodyMesh( s.ball );
   ballMesh = polyscope::registerSurfaceMesh( "ball", bm.vertices, bm.faces );
   ballMesh->setSurfaceColor( glm::vec3( 1.0f, 0.62f, 0.10f ) );
   ballMesh->setSmoothShade( true );

   // Court-size extents and the home view.
   polyscope::options::automaticallyComputeSceneExtents = false;
   polyscope::state::lengthScale = 3.0f;
   polyscope::state::boundingBox =
      std::tuple<glm::vec3, glm::vec3>{ glm::vec3( -2.0f, -1.5f, 0.0f ), glm::vec3( 2.0f, 1.5f, 1.6f ) };
   // Side view: the serve's arc over the net lies in the x-z plane.
   polyscope::view::lookAt( glm::vec3( -0.3f, -4.2f, 1.5f ), glm::vec3( -0.3f, 0.0f, 0.85f ) );
}


std::vector<polyscope::SurfaceMesh*> batMeshes;

void registerBat()
{
   Sim& s = sim();
   batMeshes.clear();
   for( size_t i = 0; i < s.batParts.size(); ++i ) {
      const viewer::ShapeMesh m = viewer::makeBodyMesh( s.batParts[i] );
      polyscope::SurfaceMesh* mesh = polyscope::registerSurfaceMesh( "bat " + std::to_string( i ), m.vertices, m.faces );
      mesh->setSurfaceColor( i == 0 ? glm::vec3( 0.75f, 0.60f, 0.40f ) : i == 1 ? glm::vec3( 0.85f, 0.10f, 0.10f ) : i == 2 ? glm::vec3( 0.05f, 0.05f, 0.05f ) : glm::vec3( 0.55f, 0.40f, 0.25f ) );
      mesh->setSmoothShade( true );
      batMeshes.push_back( mesh );
   }
}


std::vector<polyscope::SurfaceMesh*> recMeshes;

void registerReceiver()
{
   Sim& s = sim();
   recMeshes.clear();
   for( size_t i = 0; i < s.recParts.size(); ++i ) {
      const viewer::ShapeMesh m = viewer::makeBodyMesh( s.recParts[i] );
      polyscope::SurfaceMesh* mesh = polyscope::registerSurfaceMesh( "receiver " + std::to_string( i ), m.vertices, m.faces );
      mesh->setSurfaceColor( i == 0 ? glm::vec3( 0.75f, 0.60f, 0.40f ) : i == 1 ? glm::vec3( 0.85f, 0.10f, 0.10f ) : i == 2 ? glm::vec3( 0.05f, 0.05f, 0.05f ) : glm::vec3( 0.55f, 0.40f, 0.25f ) );
      mesh->setSmoothShade( true );
      recMeshes.push_back( mesh );
   }
}


void updateMirror()
{
   Sim& s = sim();
   ballMesh->setTransform( viewer::bodyTransform( s.ball ) );
   if( batMeshes.size() != s.batParts.size() ) registerBat();
   for( size_t i = 0; i < batMeshes.size(); ++i ) {
      batMeshes[i]->setTransform( viewer::bodyTransform( s.batParts[i] ) );
      batMeshes[i]->setEnabled( usesBat() );
   }
   if( recMeshes.size() != s.recParts.size() ) registerReceiver();
   for( size_t i = 0; i < recMeshes.size(); ++i ) {
      recMeshes[i]->setTransform( viewer::bodyTransform( s.recParts[i] ) );
      recMeshes[i]->setEnabled( receiving() && s.recPlaced );
   }

   if( s.trace.size() >= 2 ) {
      polyscope::CurveNetwork* c = polyscope::registerCurveNetworkLine( "trace", s.trace );
      c->setColor( glm::vec3( 0.95f, 0.35f, 0.10f ) );
      c->setRadius( 0.004, /*isRelative=*/false );
   }
   else
      polyscope::removeStructure( "trace", /*errorIfAbsent=*/false );

   if( !s.bouncePts.empty() ) {
      polyscope::PointCloud* p = polyscope::registerPointCloud( "bounces", s.bouncePts );
      p->setPointColor( glm::vec3( 0.9f, 0.1f, 0.1f ) );
      p->setPointRadius( 0.015, /*isRelative=*/false );
   }
   else
      polyscope::removeStructure( "bounces", /*errorIfAbsent=*/false );

   std::vector<glm::vec3> servePt{ glm::vec3( static_cast<float>( s.serve.pos[0] ), static_cast<float>( s.serve.pos[1] ), static_cast<float>( s.serve.pos[2] ) ) };
   polyscope::PointCloud* sp = polyscope::registerPointCloud( "serve position", servePt );
   sp->setPointColor( glm::vec3( 0.2f, 0.8f, 0.2f ) );
   sp->setPointRadius( 0.012, /*isRelative=*/false );
}


//=================================================================================================
//  GUI
//=================================================================================================

bool inputVec3( const char* label, double* v, const char* fmt = "%.3f" )
{
   ImGui::SetNextItemWidth( 260 );
   return ImGui::InputScalarN( label, ImGuiDataType_Double, v, 3, nullptr, nullptr, fmt );
}


void drawReceiverControls()
{
   Sim& s = sim();
   if( !ImGui::TreeNodeEx( "Receiving bat (held still)", ImGuiTreeNodeFlags_DefaultOpen ) )
      return;
   const double tiltLo = -80.0, tiltHi = 80.0, yawLo = -60.0, yawHi = 60.0, fLo = 0.1, fHi = 1.0;
   bool angleChanged = false;
   angleChanged |= ImGui::SliderScalar( "blade angle [deg] (0 vertical, > 0 open)", ImGuiDataType_Double, &s.receiver.tilt, &tiltLo, &tiltHi, "%.0f" );
   if( ImGui::IsItemHovered( ImGuiHoveredFlags_DelayShort ) )
      ImGui::SetTooltip( "Face normal above the horizontal. Open the face against backspin, close it against topspin.\nChanges apply at once, also mid-flight." );
   angleChanged |= ImGui::SliderScalar( "blade yaw [deg] (> 0: to the receiver's right)", ImGuiDataType_Double, &s.receiver.yaw, &yawLo, &yawHi, "%.0f" );
   if( angleChanged ) reposeReceiver();
   if( ImGui::Checkbox( "place automatically (dry run of the serve)", &s.receiver.autoPlace ) ) resetBall();
   if( s.receiver.autoPlace ) {
      ImGui::SliderScalar( "meet the ball at (1 = apex of the rise)", ImGuiDataType_Double, &s.receiver.riseFraction, &fLo, &fHi, "%.2f" );
      if( ImGui::IsItemHovered( ImGuiHoveredFlags_DelayShort ) )
         ImGui::SetTooltip( "Fraction of the time from the bounce on the receiver's side to the apex. Applied on the next Serve / Reset." );
   }
   else {
      if( inputVec3( "ball centre at the block [m]", s.receiver.pos ) ) resetBall();
      ImGui::TextDisabled( "the ball is assumed to arrive along +x" );
   }
   if( s.recPlaced )
      ImGui::TextDisabled( "face at (%.2f %.2f %.2f), %.0f cm above the table, normal (%.2f %.2f %.2f)", s.recFaceCentre[0], s.recFaceCentre[1], s.recFaceCentre[2],
                           100.0 * ( s.recFaceCentre[2] - kBallRadius - kTableHeight ), s.recNormal[0], s.recNormal[1], s.recNormal[2] );
   else
      ImGui::TextDisabled( "receiving bat parked" );
   if( !s.recNote.empty() )
      ImGui::TextWrapped( "%s", s.recNote.c_str() );
   if( s.recHit )
      ImGui::Text( "after the receive: v (%.2f %.2f %.2f) |v| %.2f m/s, spin (%.0f %.0f %.0f) rev/s, rose %.0f cm",
                   s.afterRecVel[0], s.afterRecVel[1], s.afterRecVel[2], s.afterRecVel.length(),
                   s.afterRecSpin[0], s.afterRecSpin[1], s.afterRecSpin[2], 100.0 * s.recMaxHeight );
   ImGui::TreePop();
}


void drawServeWindow()
{
   Sim& s = sim();
   ImGui::SetNextWindowPos( ImVec2( 10, 10 ), ImGuiCond_FirstUseEver );
   ImGui::SetNextWindowSize( ImVec2( 400, 0 ), ImGuiCond_FirstUseEver );
   ImGui::Begin( "Serve" );

   int requested = s.mode;
   ImGui::RadioButton( "Launch (ball velocity + spin)", &requested, kModeLaunch );
   ImGui::SameLine();
   ImGui::RadioButton( "Toss & serve (bat)", &requested, kModeToss );
   ImGui::RadioButton( "Toss, serve & receive (held bat on the other side)", &requested, kModeReceive );
   if( requested != s.mode ) { s.mode = requested; resetBall(); }
   if( s.mode == kModeLaunch && ImGui::Checkbox( "receiving bat (held still) on the other side", &s.launchReceive ) )
      resetBall();
   ImGui::Separator();

   if( usesBat() ) {
      static int batPresetIndex = 0;
      ImGui::SetNextItemWidth( 260 );
      if( ImGui::BeginCombo( "preset##bat", kBatPresets[batPresetIndex].name ) ) {
         for( int i = 0; i < kNumBatPresets; ++i )
            if( ImGui::Selectable( kBatPresets[i].name, i == batPresetIndex ) ) { batPresetIndex = i; s.bat = kBatPresets[i].bat; }
         ImGui::EndCombo();
      }
      inputVec3( "toss release [m]", s.bat.toss );
      ImGui::SetNextItemWidth( 120 );
      ImGui::InputDouble( "toss speed [m/s]", &s.bat.tossSpeed, 0.0, 0.0, "%.2f" );
      ImGui::SameLine();
      ImGui::TextDisabled( "rises %.0f cm", 100.0 * s.bat.tossSpeed * s.bat.tossSpeed / ( 2.0 * kG ) );
      ImGui::SetNextItemWidth( 120 );
      ImGui::InputDouble( "contact height [m]", &s.bat.contactZ, 0.0, 0.0, "%.3f" );
      ImGui::SetNextItemWidth( 120 );
      ImGui::InputDouble( "bat speed [m/s]", &s.bat.batSpeed, 0.0, 0.0, "%.2f" );
      ImGui::SetNextItemWidth( 120 );
      ImGui::InputDouble( "stroke azimuth [deg]", &s.bat.strokeAz, 0.0, 0.0, "%.1f" );
      ImGui::SetNextItemWidth( 120 );
      ImGui::InputDouble( "stroke elevation [deg]", &s.bat.strokeEl, 0.0, 0.0, "%.1f" );
      ImGui::SetNextItemWidth( 120 );
      ImGui::InputDouble( "blade tilt [deg] (open > 0)", &s.bat.bladeTilt, 0.0, 0.0, "%.1f" );
      ImGui::SetNextItemWidth( 120 );
      ImGui::InputDouble( "blade yaw [deg]", &s.bat.bladeYaw, 0.0, 0.0, "%.1f" );
      ImGui::SetNextItemWidth( 120 );
      ImGui::InputDouble( "wrist rate [rad/s] about z", &s.bat.wristRate, 0.0, 0.0, "%.1f" );
      ImGui::Checkbox( "handle up (straight stroke)", &s.bat.handleUp );
      if( ImGui::TreeNodeEx( "Pendulum swing (0 rate = straight stroke)", ImGuiTreeNodeFlags_DefaultOpen ) ) {
         ImGui::SetNextItemWidth( 120 );
         ImGui::InputDouble( "swing rate [rad/s] (+: right to left)", &s.bat.swingRate, 0.0, 0.0, "%.1f" );
         ImGui::SetNextItemWidth( 120 );
         ImGui::InputDouble( "swing radius [m]", &s.bat.swingRadius, 0.0, 0.0, "%.2f" );
         ImGui::SetNextItemWidth( 120 );
         ImGui::InputDouble( "arm angle at contact [deg] (<0: before the bottom)", &s.bat.swingPhase, 0.0, 0.0, "%.1f" );
         ImGui::SetNextItemWidth( 120 );
         ImGui::InputDouble( "swing axis yaw [deg]", &s.bat.swingAxisYaw, 0.0, 0.0, "%.1f" );
         ImGui::SetNextItemWidth( 120 );
         ImGui::InputDouble( "swing axis pitch [deg]", &s.bat.swingAxisPitch, 0.0, 0.0, "%.1f" );
         ImGui::TextDisabled( "bat speed / azimuth / elevation above are then the arm's motion; tip speed %.1f m/s", std::fabs( s.bat.swingRate ) * s.bat.swingRadius );
         ImGui::TreePop();
      }
      if( const char* illegal = tossLegality( s.bat ); *illegal )
         ImGui::TextColored( ImVec4( 1.0f, 0.45f, 0.2f, 1.0f ), "%s", illegal );
      else
         ImGui::TextDisabled( "toss and contact point legal; contact at t = %.3f s", s.contactTime );
      if( s.batHit )
         ImGui::Text( "after the bat: v (%.2f %.2f %.2f) |v| %.2f m/s, spin (%.0f %.0f %.0f) rev/s",
                      s.afterHitVel[0], s.afterHitVel[1], s.afterHitVel[2], s.afterHitVel.length(),
                      s.afterHitSpin[0], s.afterHitSpin[1], s.afterHitSpin[2] );
      planStroke();
   }

   if( receiving() ) drawReceiverControls();

   static int presetIndex = 0;
   if( usesBat() ) goto buttons;
   {
   ImGui::SetNextItemWidth( 260 );
   if( ImGui::BeginCombo( "preset", kPresets[presetIndex].name ) ) {
      for( int i = 0; i < kNumPresets; ++i ) {
         if( ImGui::Selectable( kPresets[i].name, i == presetIndex ) ) {
            presetIndex = i;
            s.serve = kPresets[i].serve;
         }
      }
      ImGui::EndCombo();
   }

   inputVec3( "position [m]", s.serve.pos );
   if( ImGui::IsItemHovered( ImGuiHoveredFlags_DelayShort ) )
      ImGui::SetTooltip( "Ball center. The server's end line is at x = -1.37, the table top at z = 0.76." );
   inputVec3( "velocity [m/s]", s.serve.vel );
   inputVec3( "spin [rev/s]", s.serve.spin, "%.1f" );
   if( ImGui::IsItemHovered( ImGuiHoveredFlags_DelayShort ) )
      ImGui::SetTooltip( "Angular velocity. For a ball travelling +x: omega_y < 0 is backspin, > 0 topspin, omega_z sidespin." );
   const double wmag = 2.0 * kPi * std::sqrt( s.serve.spin[0] * s.serve.spin[0] + s.serve.spin[1] * s.serve.spin[1] + s.serve.spin[2] * s.serve.spin[2] );
   const double vmag = std::sqrt( s.serve.vel[0] * s.serve.vel[0] + s.serve.vel[1] * s.serve.vel[1] + s.serve.vel[2] * s.serve.vel[2] );
   ImGui::TextDisabled( "|v| = %.2f m/s, |omega| = %.0f rad/s, surface speed %.2f m/s", vmag, wmag, wmag * kBallRadius );
   if( const char* illegal = serveLegality( s.serve ); *illegal )
      ImGui::TextColored( ImVec4( 1.0f, 0.45f, 0.2f, 1.0f ), "%s", illegal );
   else
      ImGui::TextDisabled( "launch point legal: behind the end line, above the table" );
   }

buttons:
   ImGui::Spacing();
   ImGui::PushStyleColor( ImGuiCol_Button, ImVec4( 0.15f, 0.55f, 0.20f, 1.0f ) );
   const bool serveClicked = ImGui::Button( "Serve", ImVec2( 110, 32 ) );
   ImGui::PopStyleColor();
   ImGui::SameLine();
   if( ImGui::Button( s.running ? "Pause" : "Run", ImVec2( 70, 32 ) ) )
      s.running = !s.running;
   ImGui::SameLine();
   if( ImGui::Button( "Step", ImVec2( 60, 32 ) ) )
      advance( s.solver.stepsPerFrame );
   ImGui::SameLine();
   const bool resetClicked = ImGui::Button( "Reset", ImVec2( 70, 32 ) );
   ImGui::TextDisabled( "keys: s serve, space run/pause, r reset" );
   ImGui::End();

   if( serveClicked ) launchServe();
   if( resetClicked ) resetBall();
}


const double kZero = 0.0, kOne = 1.0;

void drawPhysicsWindow()
{
   Sim& s = sim();
   ImGui::SetNextWindowPos( ImVec2( 10, 300 ), ImGuiCond_FirstUseEver );
   ImGui::SetNextWindowSize( ImVec2( 400, 0 ), ImGuiCond_FirstUseEver );
   ImGui::Begin( "Physics (live)" );

   if( ImGui::CollapsingHeader( "Aerodynamics", ImGuiTreeNodeFlags_DefaultOpen ) ) {
      ImGui::Checkbox( "drag", &s.aero.drag );
      ImGui::SameLine();
      ImGui::Checkbox( "Magnus", &s.aero.magnus );
      ImGui::SetNextItemWidth( 120 );
      ImGui::InputDouble( "C_d", &s.aero.cd, 0.0, 0.0, "%.2f" );
      ImGui::SetNextItemWidth( 120 );
      ImGui::InputDouble( "lift scale", &s.aero.liftScale, 0.0, 0.0, "%.2f" );
      if( ImGui::IsItemHovered( ImGuiHoveredFlags_DelayShort ) )
         ImGui::SetTooltip( "C_L = scale / ( 2 + v / ( r omega ) )" );
      ImGui::SetNextItemWidth( 120 );
      ImGui::InputDouble( "air density [kg/m^3]", &s.aero.rhoAir, 0.0, 0.0, "%.3f" );
      ImGui::SetNextItemWidth( 120 );
      ImGui::InputDouble( "spin decay tau [s] (0 = off)", &s.aero.spinDecay, 0.0, 0.0, "%.2f" );
   }
   if( ImGui::CollapsingHeader( "Ball (applied on next serve / reset)", ImGuiTreeNodeFlags_DefaultOpen ) ) {
      double massG = 1000.0 * s.ball_.mass, shellMm = 1000.0 * s.ball_.shellThickness;
      ImGui::SetNextItemWidth( 120 );
      if( ImGui::InputDouble( "mass [g]", &massG, 0.0, 0.0, "%.2f" ) ) s.ball_.mass = std::max( 0.01, massG ) / 1000.0;
      ImGui::SetNextItemWidth( 120 );
      if( ImGui::InputDouble( "shell thickness [mm] (0 = solid)", &shellMm, 0.0, 0.0, "%.2f" ) )
         s.ball_.shellThickness = std::max( 0.0, std::min( shellMm, 1000.0 * kBallRadius ) ) / 1000.0;
      if( ImGui::IsItemHovered( ImGuiHoveredFlags_DelayShort ) )
         ImGui::SetTooltip( "Hollow sphere: same collision geometry, shell mass and inertia\n(I -> 2/3 m r^2 for a thin wall instead of 2/5 m r^2 for a solid ball)." );
      const double factor = s.ball->getInertia()[0] / ( s.ball->getMass() * kBallRadius * kBallRadius );
      ImGui::TextDisabled( "current ball: %.2f g, %s, I / (m r^2) = %.3f%s", 1000.0 * s.ball->getMass(),
                           s.ball->isHollow() ? "hollow" : "solid", factor, ballParamsChanged() ? "  (changes pending)" : "" );
   }
   if( ImGui::CollapsingHeader( "Contact (ball / table)", ImGuiTreeNodeFlags_DefaultOpen ) ) {
      ImGui::SliderScalar( "restitution e", ImGuiDataType_Double, &s.contact.cor, &kZero, &kOne, "%.2f" );
      ImGui::SliderScalar( "friction mu", ImGuiDataType_Double, &s.contact.mu, &kZero, &kOne, "%.2f" );
      ImGui::SliderScalar( "rubber restitution", ImGuiDataType_Double, &s.contact.corRubber, &kZero, &kOne, "%.2f" );
      ImGui::SliderScalar( "rubber friction", ImGuiDataType_Double, &s.contact.muRubber, &kZero, &kOne, "%.2f" );
   }
   if( ImGui::CollapsingHeader( "Solver", ImGuiTreeNodeFlags_DefaultOpen ) ) {
      const double dtLo = 2.0e-5, dtHi = 2.0e-3;
      ImGui::SliderScalar( "dt [s]", ImGuiDataType_Double, &s.solver.dt, &dtLo, &dtHi, "%.2e", ImGuiSliderFlags_Logarithmic );
      ImGui::SliderInt( "steps / frame", &s.solver.stepsPerFrame, 1, 200 );
      ImGui::SliderInt( "max iterations", &s.solver.maxIterations, 1, 200 );
      ImGui::SliderScalar( "erp", ImGuiDataType_Double, &s.solver.erp, &kZero, &kOne, "%.2f" );
      ImGui::Checkbox( "split impulse", &s.solver.splitImpulse );
      if( ImGui::IsItemHovered( ImGuiHoveredFlags_DelayShort ) )
         ImGui::SetTooltip( "On (default): penetration is removed by a position solve that adds no momentum.\nOff: the Baumgarte term pushes the ball out at erp * depth / dt, an artificial rebound." );
      ImGui::SameLine();
      ImGui::Checkbox( "restitution", &s.solver.restitution );
      if( s.solver.restitution ) {
         ImGui::Checkbox( "capped post-pass (Box3D scheme)", &s.solver.restitutionPostPass );
         if( ImGui::IsItemHovered( ImGuiHoveredFlags_DelayShort ) )
            ImGui::SetTooltip( "Off: the Newton target -e v_n is part of the contact solve.\nOn: the solve stays inelastic and a separate pass adds the bounce,\nwith the restitution impulse capped at e times the compression impulse (Poisson),\nso several contacts impacting at once cannot create energy. Single contacts are unchanged." );
         if( s.solver.restitutionPostPass )
            ImGui::SliderInt( "post-pass sweeps", &s.solver.restitutionIterations, 1, 10 );
      }
   }
   ImGui::End();
   applyContactParams();
   applySolverParams();
}


void drawStatusWindow()
{
   Sim& s = sim();
   ImGui::SetNextWindowPos( ImVec2( 10, 560 ), ImGuiCond_FirstUseEver );
   ImGui::SetNextWindowSize( ImVec2( 400, 0 ), ImGuiCond_FirstUseEver );
   ImGui::Begin( "Status" );
   const Vec3 p( s.ball->getPosition() ), v( s.ball->getLinearVel() ), w( s.ball->getAngularVel() );
   ImGui::Text( "t = %.3f s  (%ld steps)", s.time, s.steps );
   ImGui::Text( "ball  x %+.3f  y %+.3f  z %+.3f  (%.1f cm above the table)", p[0], p[1], p[2], 100.0 * ( p[2] - kBallRadius - kTableHeight ) );
   ImGui::Text( "v     %+.2f %+.2f %+.2f  |v| %.2f m/s", v[0], v[1], v[2], v.length() );
   ImGui::Text( "spin  %+.1f %+.1f %+.1f rev/s", w[0] / ( 2 * kPi ), w[1] / ( 2 * kPi ), w[2] / ( 2 * kPi ) );
   ImGui::Text( "max speed %.2f m/s, max height %.2f m above the table", s.maxSpeed, s.maxHeight );
   if( !s.error.empty() )
      ImGui::TextColored( ImVec4( 1, 0.3f, 0.3f, 1 ), "error: %s", s.error.c_str() );

   const Verdict verdict = evaluateWithLegality();
   if( s.served ) {
      if( verdict.ghost ) ImGui::TextColored( ImVec4( 0.3f, 1.0f, 0.3f, 1 ), "%s", verdict.text.c_str() );
      else ImGui::TextWrapped( "%s", verdict.text.c_str() );
      const Banana b = bananaCheck();
      if( b.banana )
         ImGui::TextColored( ImVec4( 1.0f, 0.85f, 0.2f, 1 ), "BANANA SERVE: crossed the net line %s at y = %+.2f m (table edge %.2f), landed at y = %+.2f m",
                             b.aroundNet ? "around the net" : "outside the table width", b.crossY, 0.5 * kTableWidth, b.landY );
      if( receiving() ) {
         const Receive r = receiveVerdict();
         const ImVec4 col = r.outcome == 1 ? ImVec4( 0.3f, 1.0f, 0.3f, 1 ) : r.outcome == 0 ? ImVec4( 0.7f, 0.7f, 0.7f, 1 ) : ImVec4( 1.0f, 0.5f, 0.3f, 1 );
         ImGui::TextColored( col, "%s", r.text.c_str() );
      }
   }
   else
      ImGui::TextDisabled( "no serve yet" );

   ImGui::Separator();
   ImGui::Text( "events" );
   const size_t first = s.events.size() > 14 ? s.events.size() - 14 : 0;
   for( size_t i = first; i < s.events.size(); ++i ) {
      const Event& e = s.events[i];
      ImGui::BulletText( "%.3f s  %s  at x %+.2f, y %+.2f, z %.2f", e.time, eventName( e.kind ), e.pos[0], e.pos[1], e.pos[2] );
   }
   ImGui::End();
}


void drawSideView()
{
   Sim& s = sim();
   ImGui::SetNextWindowPos( ImVec2( 420, 10 ), ImGuiCond_FirstUseEver );
   ImGui::SetNextWindowSize( ImVec2( 560, 260 ), ImGuiCond_FirstUseEver );
   ImGui::Begin( "Side view (x-z)" );
   if( ImPlot::BeginPlot( "##side", ImVec2( -1, -1 ), ImPlotFlags_Equal ) ) {
      ImPlot::SetupAxes( "x [m]", "z [m]" );
      ImPlot::SetupAxesLimits( -2.0, 2.0, 0.0, 1.6, ImPlotCond_Once );
      const double tx[2] = { -0.5 * kTableLength, 0.5 * kTableLength }, tz[2] = { kTableHeight, kTableHeight };
      const double nx[2] = { 0.0, 0.0 }, nz[2] = { kTableHeight, kNetTop };
      ImPlot::SetNextLineStyle( ImVec4( 0.2f, 0.5f, 1.0f, 1 ), 3.0f );
      ImPlot::PlotLine( "table", tx, tz, 2 );
      ImPlot::SetNextLineStyle( ImVec4( 0.6f, 0.6f, 0.6f, 1 ), 3.0f );
      ImPlot::PlotLine( "net", nx, nz, 2 );
      if( !s.traceX.empty() ) {
         ImPlot::SetNextLineStyle( ImVec4( 1.0f, 0.5f, 0.1f, 1 ), 2.0f );
         ImPlot::PlotLine( "ball", s.traceX.data(), s.traceZ.data(), static_cast<int>( s.traceX.size() ) );
      }
      ImPlot::EndPlot();
   }
   ImGui::End();
}


void guiCallback()
{
   Sim& s = sim();
   ImGuiIO& io = ImGui::GetIO();
   if( !io.WantTextInput ) {
      if( ImGui::IsKeyPressed( ImGuiKey_Space, false ) ) s.running = !s.running;
      if( ImGui::IsKeyPressed( ImGuiKey_S, false ) )     launchServe();
      if( ImGui::IsKeyPressed( ImGuiKey_R, false ) )     resetBall();
   }

   drawServeWindow();
   drawPhysicsWindow();
   drawStatusWindow();
   drawSideView();

   if( s.running )
      advance( s.solver.stepsPerFrame );
   updateMirror();
}


//=================================================================================================
//  HEADLESS MODES
//=================================================================================================

void printEvents()
{
   const Sim& s = sim();
   for( const Event& e : s.events )
      std::printf( "  %.3f s  %-28s x %+.3f  y %+.3f  z %.3f\n", e.time, eventName( e.kind ), e.pos[0], e.pos[1], e.pos[2] );
}


//! Runs the current serve to rest (or \a maxTime) without any window; prints events and a RESULT line.
int runBatch( double maxTime, bool quiet )
{
   Sim& s = sim();
   launchServe();
   while( s.running && s.time < maxTime )
      advance( 1000 );
   const Verdict v = evaluateWithLegality();
   if( !quiet ) printEvents();
   if( !s.error.empty() ) std::printf( "error: %s\n", s.error.c_str() );
   double backHeight = -1.0;
   for( const Event& e : s.events )
      if( e.kind == kNetCrossBack ) { backHeight = e.pos[2] - kBallRadius - kNetTop; break; }
   const double good = goodServeClearance();
   const Banana b = bananaCheck();
   if( usesBat() )
      std::printf( "BAT hit=%d t_contact=%.3f vBall=(%.2f %.2f %.2f) |v|=%.2f spin=(%.0f %.0f %.0f)\n", s.batHit ? 1 : 0, s.contactTime,
                   s.afterHitVel[0], s.afterHitVel[1], s.afterHitVel[2], s.afterHitVel.length(), s.afterHitSpin[0], s.afterHitSpin[1], s.afterHitSpin[2] );
   if( receiving() ) {
      const Receive r = receiveVerdict();
      std::printf( "RECEIVE placed=%d hit=%d face=(%.3f %.3f %.3f) normal=(%.2f %.2f %.2f) vBall=(%.2f %.2f %.2f) spin=(%.0f %.0f %.0f) rose=%.3f outcome=%d netClearance=%.3f land=(%.3f %.3f) note=\"%s\" return=\"%s\"\n",
                   s.recPlaced ? 1 : 0, r.hit ? 1 : 0, s.recFaceCentre[0], s.recFaceCentre[1], s.recFaceCentre[2], s.recNormal[0], s.recNormal[1], s.recNormal[2],
                   s.afterRecVel[0], s.afterRecVel[1], s.afterRecVel[2], s.afterRecSpin[0], s.afterRecSpin[1], s.afterRecSpin[2], s.recMaxHeight,
                   r.outcome, r.netClearance, r.landX, r.landY, s.recNote.c_str(), r.text.c_str() );
   }
   std::printf( "RESULT ghost=%d legal=%d good=%d netClearance=%.3f banana=%d around=%d crossY=%.3f firstY=%.3f landX=%.3f landY=%.3f events=%zu t=%.3f maxSpeed=%.2f backClearance=%.3f ball=%s verdict=\"%s\"\n",
                v.ghost ? 1 : 0, *currentLegality() ? 0 : 1, good >= 0.0 ? 1 : 0, good, b.banana ? 1 : 0, b.aroundNet ? 1 : 0,
                b.crossY, b.firstBounceY, b.landX, b.landY, s.events.size(), s.time, s.maxSpeed, backHeight,
                s.ball->isHollow() ? "hollow" : "solid", v.text.c_str() );
   return s.error.empty() ? EXIT_SUCCESS : EXIT_FAILURE;
}


bool smokeTest()
{
   Sim& s = sim();
   bool ok = true;
   s.serve = kPresets[0].serve;
   launchServe();
   for( int frame = 0; frame < 400 && s.running; ++frame )
      polyscope::frameTick();
   printEvents();
   const Verdict v = evaluate();
   std::printf( "verdict: %s\n", v.text.c_str() );
   bool crossed = false, bounced = false;
   for( const Event& e : s.events ) {
      crossed |= e.kind == kNetCrossForward;
      bounced |= e.kind == kBounceOpponent;
   }
   if( !s.error.empty() ) { std::printf( "FAIL: %s\n", s.error.c_str() ); ok = false; }
   if( !crossed ) { std::printf( "FAIL: the ghost preset did not cross the net\n" ); ok = false; }
   if( !bounced ) { std::printf( "FAIL: the ghost preset did not bounce on the opponent's side\n" ); ok = false; }
   if( !v.ghost ) { std::printf( "FAIL: the ghost preset did not come back over the net\n" ); ok = false; }
   if( *serveLegality( s.serve ) ) { std::printf( "FAIL: the ghost preset launches from an illegal point: %s\n", serveLegality( s.serve ) ); ok = false; }

   // Every other preset: legal launch point, own-side bounce, over the net, opponent-side bounce.
   for( int i = 1; i < kNumPresets; ++i ) {
      s.serve = kPresets[i].serve;
      launchServe();
      for( int frame = 0; frame < 400 && s.running; ++frame )
         polyscope::frameTick();
      const double clearance = goodServeClearance();
      std::printf( "preset \"%s\": %s, net clearance %.3f m\n", kPresets[i].name, clearance >= 0.0 ? "good serve" : "NOT a good serve", clearance );
      if( *serveLegality( s.serve ) ) { std::printf( "FAIL: preset \"%s\" launches from an illegal point\n", kPresets[i].name ); ok = false; }
      if( clearance < 0.0 ) { std::printf( "FAIL: preset \"%s\" does not land on the opponent's side over the net\n", kPresets[i].name ); ok = false; }
   }
   s.serve = kPresets[0].serve;

   // Toss & serve presets: legal toss and contact point, bat hit, good serve.
   s.mode = kModeToss;
   for( int i = 0; i < kNumBatPresets; ++i ) {
      s.bat = kBatPresets[i].bat;
      resetBall();
      launchServe();
      for( int frame = 0; frame < 400 && s.running; ++frame )
         polyscope::frameTick();
      const double clearance = goodServeClearance();
      std::printf( "bat preset \"%s\": hit %d, %s, net clearance %.3f m, ball after the bat |v| %.2f m/s\n", kBatPresets[i].name,
                   s.batHit ? 1 : 0, clearance >= 0.0 ? "good serve" : "NOT a good serve", clearance, s.afterHitVel.length() );
      if( *tossLegality( s.bat ) ) { std::printf( "FAIL: bat preset \"%s\" is illegal: %s\n", kBatPresets[i].name, tossLegality( s.bat ) ); ok = false; }
      if( !s.batHit ) { std::printf( "FAIL: bat preset \"%s\": the bat missed the ball\n", kBatPresets[i].name ); ok = false; }
      if( clearance < 0.0 ) { std::printf( "FAIL: bat preset \"%s\" is not a good serve\n", kBatPresets[i].name ); ok = false; }
   }

   // Receive mode: every bat preset reaches the held receiving bat; so does every launch preset
   // with the receiver added.
   s.mode = kModeReceive;
   s.receiver = ReceiverParams();
   for( int i = 0; i < kNumBatPresets; ++i ) {
      s.bat = kBatPresets[i].bat;
      launchServe();
      for( int frame = 0; frame < 400 && s.running; ++frame )
         polyscope::frameTick();
      const Receive r = receiveVerdict();
      std::printf( "receive, bat preset \"%s\": placed %d, face %.0f cm above the table, receiver hit %d, %s\n", kBatPresets[i].name,
                   s.recPlaced ? 1 : 0, 100.0 * ( s.recFaceCentre[2] - kBallRadius - kTableHeight ), r.hit ? 1 : 0, r.text.c_str() );
      if( !s.recPlaced ) { std::printf( "FAIL: receiving bat not placed for \"%s\": %s\n", kBatPresets[i].name, s.recNote.c_str() ); ok = false; }
      if( !r.hit ) { std::printf( "FAIL: the ball missed the receiving bat for \"%s\"\n", kBatPresets[i].name ); ok = false; }
   }
   s.mode = kModeLaunch;
   s.launchReceive = true;
   for( int i = 0; i < kNumPresets; ++i ) {
      s.serve = kPresets[i].serve;
      launchServe();
      for( int frame = 0; frame < 400 && s.running; ++frame )
         polyscope::frameTick();
      const Receive r = receiveVerdict();
      std::printf( "receive, preset \"%s\": placed %d, face %.0f cm above the table, receiver hit %d, %s\n", kPresets[i].name,
                   s.recPlaced ? 1 : 0, 100.0 * ( s.recFaceCentre[2] - kBallRadius - kTableHeight ), r.hit ? 1 : 0, r.text.c_str() );
      if( !s.recPlaced ) { std::printf( "FAIL: receiving bat not placed for \"%s\": %s\n", kPresets[i].name, s.recNote.c_str() ); ok = false; }
      if( !r.hit ) { std::printf( "FAIL: the ball missed the receiving bat for \"%s\"\n", kPresets[i].name ); ok = false; }
   }
   s.launchReceive = false;
   s.serve = kPresets[0].serve;
   s.mode = kModeLaunch;
   resetBall();
   return ok;
}

} // namespace


//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================

int main( int argc, char** argv )
{
   Sim& s = sim();
   bool smoke = false, batch = false, quiet = false, topView = false, wantReceive = false;
   double maxTime = 6.0, shotTime = 1.0;
   std::string shot;
   int shotPreset = 0;
   auto num = [&]( int& i ) { return std::atof( argv[++i] ); };
   for( int i = 1; i < argc; ++i ) {
      const std::string arg( argv[i] );
      if( arg == "--smoke" ) smoke = true;
      else if( arg == "--screenshot" && i + 1 < argc ) shot = argv[++i];
      else if( arg == "--at" && i + 1 < argc ) shotTime = num( i );
      else if( arg == "--preset" && i + 1 < argc ) shotPreset = std::atoi( argv[++i] );
      else if( arg == "--top" ) topView = true;
      else if( arg == "--mode-toss" ) s.mode = kModeToss;
      else if( arg == "--mode-receive" ) s.mode = kModeReceive;
      else if( arg == "--post-pass" ) s.solver.restitutionPostPass = true;
      else if( arg == "--receive" ) wantReceive = true;
      else if( arg == "--receiver" && i + 3 < argc ) { wantReceive = true; s.receiver.tilt = num( i ); s.receiver.yaw = num( i ); s.receiver.riseFraction = num( i ); }
      else if( arg == "--receiver-pos" && i + 3 < argc ) { wantReceive = true; s.receiver.autoPlace = false; for( int k = 0; k < 3; ++k ) s.receiver.pos[k] = num( i ); }
      else if( arg == "--serve" && i + 6 < argc ) {
         batch = true;
         for( int k = 0; k < 3; ++k ) s.serve.vel[k]  = num( i );
         for( int k = 0; k < 3; ++k ) s.serve.spin[k] = num( i );
      }
      else if( arg == "--pos" && i + 3 < argc ) { for( int k = 0; k < 3; ++k ) s.serve.pos[k] = num( i ); }
      else if( arg == "--toss" ) { batch = true; s.mode = kModeToss; }
      else if( arg == "--toss-pos" && i + 3 < argc ) { for( int k = 0; k < 3; ++k ) s.bat.toss[k] = num( i ); }
      else if( arg == "--toss-speed" && i + 1 < argc ) s.bat.tossSpeed = num( i );
      else if( arg == "--contact-z" && i + 1 < argc ) s.bat.contactZ = num( i );
      else if( arg == "--bat" && i + 5 < argc ) { s.bat.batSpeed = num( i ); s.bat.strokeAz = num( i ); s.bat.strokeEl = num( i ); s.bat.bladeTilt = num( i ); s.bat.bladeYaw = num( i ); }
      else if( arg == "--wrist" && i + 1 < argc ) s.bat.wristRate = num( i );
      else if( arg == "--swing" && i + 5 < argc ) { s.bat.swingRate = num( i ); s.bat.swingRadius = num( i ); s.bat.swingPhase = num( i ); s.bat.swingAxisYaw = num( i ); s.bat.swingAxisPitch = num( i ); }
      else if( arg == "--handle-up" ) s.bat.handleUp = true;
      else if( arg == "--rubber" && i + 2 < argc ) { s.contact.corRubber = num( i ); s.contact.muRubber = num( i ); }
      else if( arg == "--cor" && i + 1 < argc )  s.contact.cor = num( i );
      else if( arg == "--shell" && i + 1 < argc ) s.ball_.shellThickness = num( i );
      else if( arg == "--dump" && i + 1 < argc ) { s.dump.open( argv[++i] ); s.dump << "t,x,y,z,vx,vy,vz,wx_rps,wy_rps,wz_rps\n"; }
      else if( arg == "--mass" && i + 1 < argc )  s.ball_.mass = num( i );
      else if( arg == "--mu" && i + 1 < argc )   s.contact.mu = num( i );
      else if( arg == "--dt" && i + 1 < argc )   s.solver.dt = num( i );
      else if( arg == "--time" && i + 1 < argc ) maxTime = num( i );
      else if( arg == "--lift" && i + 1 < argc ) s.aero.liftScale = num( i );
      else if( arg == "--cd" && i + 1 < argc )   s.aero.cd = num( i );
      else if( arg == "--no-drag" )   s.aero.drag = false;
      else if( arg == "--no-magnus" ) s.aero.magnus = false;
      else if( arg == "--quiet" )     quiet = true;
      else {
         std::fprintf( stderr, "usage: %s [--smoke] [--screenshot <file.png> [--at T] [--preset k] [--top] [--mode-toss | --mode-receive]] [--post-pass] [--toss ...] [--serve vx vy vz wx wy wz [--pos x y z] [--cor e] [--mu m] [--shell t_m] [--mass kg] [--dt s] [--time T] [--lift k] [--cd c] [--no-drag] [--no-magnus] [--dump file.csv] [--quiet]] [--receive] [--receiver tilt yaw riseFraction] [--receiver-pos x y z]\n", argv[0] );
         return EXIT_FAILURE;
      }
   }

   if( wantReceive ) {
      if( s.mode == kModeLaunch ) s.launchReceive = true;
      else s.mode = kModeReceive;
   }

   buildScene();

   if( batch )
      return runBatch( maxTime, quiet );

   polyscope::options::programName = "PE table tennis";
   polyscope::options::openImGuiWindowForUserCallback = false;
   polyscope::options::transparencyMode = polyscope::TransparencyMode::Pretty;
   polyscope::view::setUpDir( polyscope::UpDir::ZUp );
   polyscope::init( smoke ? "openGL_mock" : "" );
   ImPlot::CreateContext();

   registerScene();
   updateMirror();
   polyscope::state::userCallback = guiCallback;

   bool ok = true;
   if( smoke ) {
      ok = smokeTest();
      std::printf( "\nsmoke test %s\n", ok ? "passed" : "FAILED" );
   }
   else if( !shot.empty() ) {
      if( usesBat() ) { if( shotPreset >= 0 && shotPreset < kNumBatPresets ) s.bat = kBatPresets[shotPreset].bat; }
      else if( shotPreset >= 0 && shotPreset < kNumPresets ) s.serve = kPresets[shotPreset].serve;
      if( usesBat() ) resetBall();
      if( topView )
         polyscope::view::lookAt( glm::vec3( -0.01f, -0.3f, 7.0f ), glm::vec3( 0.0f, 0.0f, 0.76f ) );
      launchServe();
      for( int frame = 0; frame < 2000 && s.running && s.time < shotTime; ++frame )
         polyscope::frameTick();
      s.running = false;
      for( int i = 0; i < 3; ++i )   // settle the overlay and the camera
         polyscope::frameTick();
      polyscope::screenshot( shot, /*transparentBG=*/false );
      std::printf( "screenshot at t = %.3f s: %s\n", s.time, shot.c_str() );
   }
   else
      polyscope::show();

   ImPlot::DestroyContext();
   return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
