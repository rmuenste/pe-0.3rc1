//=================================================================================================
/*!
 *  \file tools/impact_lab/impact_lab.cpp
 *  \brief Impact lab: the two restitution schemes of the semi-implicit hard-contact solver,
 *         side by side on the impacts where they differ.
 *
 *  HardContactSemiImplicitTimesteppingSolvers bounces impacting contacts in one of two ways:
 *  the original scheme puts the Newton target -e gdot_n into the relaxation itself
 *  (setRestitutionPostPass( false )), the default since the capped post-pass landed runs the
 *  relaxation inelastic and adds the bounce in a few extra sweeps, with the separating part of
 *  each contact's impulse capped at e times its compression impulse and clamped at zero. Single
 *  contacts give the same answer under both. This tool runs small scenes with several contacts
 *  impacting in the same step under both schemes, records the trajectories and energies, and
 *  plays them back together: the selected scheme solid, the other one transparent.
 *
 *  Usage: pe_impact_lab [--scenario k] [--sweeps n] [--smoke | --batch | --screenshot file.png [--at T]]
 *
 *  Needs pe_CONSTRAINT_SOLVER = pe::response::HardContactSemiImplicitTimesteppingSolvers.
 */
//=================================================================================================

#include <pe/system/WarningDisable.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include <pe/core.h>

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

#include "implot.h"

#include "ShapeMeshes.h"

using namespace pe;

namespace {

const double kPi = 3.14159265358979323846;
const double kG  = 9.81;

enum Scheme { kInSolve = 0, kPostPass = 1 };
const char* kSchemeName[2]  = { "Newton target in the solve (original)", "capped post-pass (default)" };
const char* kSchemeShort[2] = { "in-solve Newton", "capped post-pass" };

enum Scenario { kFlatLanding = 0, kSpinningBox, kWedge, kChain, kPile, kNumScenarios };

struct ScenarioInfo {
   const char* name;
   const char* text;
   double e, mu, dt, duration;
   bool gravity;
};

const ScenarioInfo kScenarios[kNumScenarios] = {
   { "Flat landing",
     "A 0.4 x 0.2 x 0.1 m box lands exactly flat on its four corners at 1 m/s without spin. It must come up at e |v| without rotating. "
     "The post-pass is a fixed number of sweeps: with two, the corners swept first take more of the bounce and the box comes up slower and turning; four keep it symmetric.",
     0.8, 0.0, 1e-4, 1.0, true },
   { "Spinning box",
     "The same box with angular velocity about the horizontal y axis at touchdown. The leading edge impacts, the trailing edge is separating but its constraint still takes part in the solve. "
     "The Newton target in the solve returns more kinetic energy than e^2 of the incoming, at e = 1 more than came in. The capped pass cannot.",
     1.0, 0.0, 1e-4, 1.0, true },
   { "Wedge",
     "A 5 cm ball in a V of two fixed planes, touching both faces with an oblique velocity of 2 m/s. One face is hit hard, the other only through the rebound. No gravity.",
     0.9, 0.0, 1e-4, 0.6, false },
   { "Chain",
     "N touching balls, the first one arrives at 1 m/s (Newton's cradle). Both schemes share the impulse along the chain and neither reproduces the single outgoing ball; the Newton sweep keeps the energy, the capped pass loses some. No gravity.",
     1.0, 0.0, 1e-4, 0.6, false },
   { "Pile",
     "Boxes dropped in a loose column onto the ground with e = 0.8 and friction 0.3. Watch the mechanical energy: it may only fall. A box landing on two others at once is the kind of impact where the in-solve scheme can add energy.",
     0.8, 0.3, 5e-4, 3.0, true },
};

struct Params {
   int    scenario      = kSpinningBox;
   double e             = 1.0;
   double mu            = 0.0;
   double dt            = 1e-4;
   double duration      = 1.0;
   double spin          = 8.0;     // spinning box: angular velocity about y at touchdown [rad/s]
   double halfAngle     = 30.0;    // wedge: opening half-angle [deg]
   double incidence     = 50.0;    // wedge: incidence angle off the wedge axis [deg]
   int    chainBalls    = 3;
   int    pileBoxes     = 12;
   int    sweeps        = 4;       // post-pass sweeps
   int    maxIterations = 50;
   bool   splitImpulse  = true;
};

struct Frame {
   double t;
   std::vector<glm::mat4> pose;   // one per dynamic body
};

struct Run {
   bool valid = false;
   std::string error;
   std::vector<viewer::ShapeMesh> shapes;   // dynamic bodies, in body order
   std::vector<Frame> frames;
   std::vector<double> t, ke, me;           // recorded energies
   double me0 = 0;
   double impactBefore = -1, impactAfter = -1;   // kinetic energy before and after the first step with contact impulses
   double maxMeRatio = 0;                        // max mechanical energy / initial
   double spinAfter = 0;                         // |omega| of body 0 after the first impact
   Vec3 velAfter;                                // v of body 0 after the first impact
   std::vector<double> chainVel;                 // chain: v_x of every ball after the impact
   size_t armed = 0, capped = 0, clamped = 0;
   double clampedImpulse = 0, restitutionImpulse = 0;
};

struct Lab {
   Params p;
   Run run[2];
   std::vector<viewer::ShapeMesh> fixedShapes;
   std::vector<glm::mat4> fixedPoses;
   int    shown     = kPostPass;   // scheme drawn solid
   bool   showOther = true;        // the other scheme transparent
   bool   playing   = false;
   double time      = 0;
   double speed     = 0.2;         // playback speed relative to real time
   std::vector<polyscope::SurfaceMesh*> meshes[2];
   std::vector<polyscope::SurfaceMesh*> fixedMeshes;
   MaterialID mat = 0;
   bool gui = false;
};

Lab& lab() { static Lab L; return L; }

void loadScenarioDefaults( Params& p )
{
   const ScenarioInfo& info = kScenarios[p.scenario];
   p.e = info.e; p.mu = info.mu; p.dt = info.dt; p.duration = info.duration;
}


//=================================================================================================
//  SCENES
//=================================================================================================

struct Scene {
   std::vector<BodyID> dynamic;
   std::vector<BodyID> fixed;
};

Scene buildScene( const Params& p )
{
   Lab& L = lab();
   WorldID world = theWorld();
   world->clear();
   const ScenarioInfo& info = kScenarios[p.scenario];
   world->setGravity( 0, 0, info.gravity ? real( -kG ) : real( 0 ) );
   if( L.mat == 0 ) {
      // density, cor, csf, cdf, poisson, young, stiffness, dampingN, dampingT
      L.mat = createMaterial( real( 1000 ), real( 0.5 ), real( 0.3 ), real( 0.3 ), real( 0.25 ), real( 300 ), real( 1e5 ), real( 10 ), real( 10 ) );
   }
   Material::setRestitution    ( L.mat, L.mat, real( p.e ) );
   Material::setStaticFriction ( L.mat, L.mat, real( p.mu ) );
   Material::setDynamicFriction( L.mat, L.mat, real( p.mu ) );

   Scene s;
   pe::id_t id = 1;
   switch( p.scenario ) {
      case kFlatLanding:
      case kSpinningBox: {
         s.fixed.push_back( createPlane( id++, 0, 0, 1, 0, L.mat ) );
         const Vec3 len( real( 0.4 ), real( 0.2 ), real( 0.1 ) );
         BoxID box = createBox( id++, Vec3( 0, 0, real( 0.5 ) * len[2] - real( 1e-6 ) ), len, L.mat );
         box->setLinearVel( Vec3( 0, 0, real( -1 ) ) );
         if( p.scenario == kSpinningBox )
            box->setAngularVel( Vec3( 0, real( p.spin ), 0 ) );
         s.dynamic.push_back( box );
         break;
      }
      case kWedge: {
         const double a = p.halfAngle * kPi / 180.0, th = p.incidence * kPi / 180.0, r = 0.05, speed = 2.0;
         s.fixed.push_back( createPlane( id++, Vec3( real(  std::cos( a ) ), 0, real( std::sin( a ) ) ), real( 0 ), L.mat ) );
         s.fixed.push_back( createPlane( id++, Vec3( real( -std::cos( a ) ), 0, real( std::sin( a ) ) ), real( 0 ), L.mat ) );
         const Vec3 v( real( speed * std::sin( th ) ), 0, real( -speed * std::cos( th ) ) );
         // Centre touching both faces (1 um into each) so that both contacts exist in the first
         // step; approaching from a distance, the two faces are reached one step apart and the
         // impacts become sequential single-contact ones, on which the schemes agree.
         const Vec3 apex( 0, 0, real( r / std::sin( a ) - 1e-6 / std::sin( a ) ) );
         SphereID ball = createSphere( id++, apex, real( r ), L.mat );
         ball->setLinearVel( v );
         s.dynamic.push_back( ball );
         break;
      }
      case kChain: {
         const real r( 0.03 );
         for( int i = 0; i < p.chainBalls; ++i ) {
            SphereID ball = createSphere( id++, Vec3( 2 * r * real( i ), 0, real( 0.3 ) ), r, L.mat );
            if( i == 0 ) ball->setLinearVel( Vec3( 1, 0, 0 ) );
            s.dynamic.push_back( ball );
         }
         break;
      }
      case kPile: {
         s.fixed.push_back( createPlane( id++, 0, 0, 1, 0, L.mat ) );
         std::mt19937 rng( 7 );
         std::uniform_real_distribution<double> U( -1.0, 1.0 );
         for( int i = 0; i < p.pileBoxes; ++i ) {
            BoxID box = createBox( id++, Vec3( real( 0.06 * U( rng ) ), real( 0.06 * U( rng ) ), real( 0.3 + 0.35 * i ) ),
                                   Vec3( real( 0.2 ), real( 0.2 ), real( 0.2 ) ), L.mat );
            Vec3 axis( 1, real( 0.5 * U( rng ) ), real( 0.5 * U( rng ) ) );
            axis.normalize();
            box->rotate( axis, real( 0.3 * U( rng ) ) );
            s.dynamic.push_back( box );
         }
         break;
      }
      default:
         break;
   }
   return s;
}


void applySolver( const Params& p, int scheme )
{
   CollisionSystemID cs = theCollisionSystem();
   cs->setMaxIterations( static_cast<size_t>( std::max( 1, p.maxIterations ) ) );
   cs->setSplitImpulse( p.splitImpulse );
   cs->setRestitution( true );
   cs->setRestitutionPostPass( scheme == kPostPass );
   cs->setRestitutionIterations( static_cast<size_t>( std::max( 1, p.sweeps ) ) );
}


double kinetic( const std::vector<BodyID>& bodies )
{
   double ke = 0;
   for( BodyID b : bodies ) {
      const Vec3& v = b->getLinearVel();
      const Vec3& w = b->getAngularVel();
      ke += 0.5 * b->getMass() * ( trans( v ) * v ) + 0.5 * ( trans( w ) * ( b->getInertia() * w ) );
   }
   return ke;
}

double potential( const std::vector<BodyID>& bodies, double g )
{
   double pe = 0;
   for( BodyID b : bodies )
      pe += b->getMass() * g * b->getPosition()[2];
   return pe;
}


//=================================================================================================
//  SIMULATION
//=================================================================================================

Run simulate( const Params& p, int scheme )
{
   Lab& L = lab();
   Run r;
   applySolver( p, scheme );
   Scene sc = buildScene( p );
   const ScenarioInfo& info = kScenarios[p.scenario];
   const double g = info.gravity ? kG : 0.0;

   for( BodyID b : sc.dynamic ) r.shapes.push_back( viewer::makeBodyMesh( b, 1.5 ) );
   if( scheme == kInSolve ) {
      L.fixedShapes.clear(); L.fixedPoses.clear();
      for( BodyID b : sc.fixed ) { L.fixedShapes.push_back( viewer::makeBodyMesh( b, 1.5 ) ); L.fixedPoses.push_back( viewer::bodyTransform( b ) ); }
   }

   const int steps = std::max( 1, static_cast<int>( std::lround( p.duration / p.dt ) ) );
   const int every = std::max( 1, steps / 600 );
   WorldID world = theWorld();
   CollisionSystemID cs = theCollisionSystem();

   auto snapshot = [&]( double t ) {
      Frame f; f.t = t;
      for( BodyID b : sc.dynamic ) f.pose.push_back( viewer::bodyTransform( b ) );
      r.frames.push_back( f );
   };
   auto record = [&]( double t, double ke, double me ) { r.t.push_back( t ); r.ke.push_back( ke ); r.me.push_back( me ); };

   double ke = kinetic( sc.dynamic ), me = ke + potential( sc.dynamic, g );
   r.me0 = me;
   r.maxMeRatio = 1;
   snapshot( 0 ); record( 0, ke, me );

   std::vector<Vec3> vPrev( sc.dynamic.size() ), wPrev( sc.dynamic.size() );
   bool impacted = false;
   double kePrev = ke;
   try {
      for( int i = 1; i <= steps; ++i ) {
         for( size_t k = 0; k < sc.dynamic.size(); ++k ) { vPrev[k] = sc.dynamic[k]->getLinearVel(); wPrev[k] = sc.dynamic[k]->getAngularVel(); }
         world->simulationStep( real( p.dt ) );
         const double t = i * p.dt;

         // A step carried a contact impulse if some body's velocity change is not the gravity increment.
         bool contact = false;
         for( size_t k = 0; k < sc.dynamic.size() && !contact; ++k ) {
            const Vec3 dv( sc.dynamic[k]->getLinearVel() - vPrev[k] - Vec3( 0, 0, real( -g * p.dt ) ) );
            const Vec3 dw( sc.dynamic[k]->getAngularVel() - wPrev[k] );
            contact = dv.length() > 1e-6 || dw.length() > 1e-6;
         }
         ke = kinetic( sc.dynamic ); me = ke + potential( sc.dynamic, g );
         if( !std::isfinite( ke ) ) { r.error = "non-finite state"; break; }
         if( !impacted && contact ) {
            r.impactBefore = kePrev;
            r.impactAfter  = ke;
            impacted = true;
            r.spinAfter = sc.dynamic[0]->getAngularVel().length();
            r.velAfter  = sc.dynamic[0]->getLinearVel();
            for( BodyID b : sc.dynamic ) r.chainVel.push_back( b->getLinearVel()[0] );
         }
         kePrev = ke;
         r.maxMeRatio = std::max( r.maxMeRatio, r.me0 > 0 ? me / r.me0 : 0.0 );

         const RestitutionPassStats& st = cs->getRestitutionPassStats();
         r.armed += st.armed; r.capped += st.capped; r.clamped += st.clamped;
         r.clampedImpulse += st.clampedImpulse; r.restitutionImpulse += st.restitutionImpulse;

         if( i % every == 0 || i == steps ) { snapshot( t ); record( t, ke, me ); }
      }
   }
   catch( const std::exception& ex ) {
      r.error = ex.what();
   }
   r.valid = r.error.empty();
   return r;
}


void runBoth()
{
   Lab& L = lab();
   for( int k = 0; k < 2; ++k ) L.run[k] = simulate( L.p, k );
   L.time = 0;
}


//=================================================================================================
//  MIRROR
//=================================================================================================

glm::vec3 schemeColor( int k )
{
   return k == kInSolve ? glm::vec3( 0.92f, 0.41f, 0.20f ) : glm::vec3( 0.11f, 0.69f, 0.48f );
}

void setCamera()
{
   const Lab& L = lab();
   switch( L.p.scenario ) {
      case kPile:  polyscope::view::lookAt( glm::vec3( 0.0f, -4.5f, 2.0f ), glm::vec3( 0.0f, 0.0f, 0.9f ) ); break;
      case kChain: polyscope::view::lookAt( glm::vec3( 0.15f, -0.8f, 0.42f ), glm::vec3( 0.15f, 0.0f, 0.3f ) ); break;
      case kWedge: polyscope::view::lookAt( glm::vec3( 0.0f, -1.0f, 0.4f ), glm::vec3( 0.0f, 0.0f, 0.2f ) ); break;
      default:     polyscope::view::lookAt( glm::vec3( 0.25f, -1.3f, 0.55f ), glm::vec3( 0.0f, 0.0f, 0.15f ) ); break;
   }
}

void registerMeshes()
{
   Lab& L = lab();
   if( !L.gui ) return;
   polyscope::removeAllStructures();
   L.fixedMeshes.clear();
   for( size_t i = 0; i < L.fixedShapes.size(); ++i ) {
      polyscope::SurfaceMesh* m = polyscope::registerSurfaceMesh( "wall " + std::to_string( i ), L.fixedShapes[i].vertices, L.fixedShapes[i].faces );
      m->setTransform( L.fixedPoses[i] );
      m->setSurfaceColor( glm::vec3( 0.62f, 0.65f, 0.70f ) );
      m->setTransparency( 0.5f );
      L.fixedMeshes.push_back( m );
   }
   for( int k = 0; k < 2; ++k ) {
      L.meshes[k].clear();
      const Run& r = L.run[k];
      for( size_t i = 0; i < r.shapes.size(); ++i ) {
         polyscope::SurfaceMesh* m = polyscope::registerSurfaceMesh( std::string( kSchemeShort[k] ) + " body " + std::to_string( i ), r.shapes[i].vertices, r.shapes[i].faces );
         m->setSurfaceColor( schemeColor( k ) );
         m->setEdgeWidth( 1.0 );
         L.meshes[k].push_back( m );
      }
   }
   setCamera();
}

void showFrame( double t )
{
   Lab& L = lab();
   if( !L.gui ) return;
   for( int k = 0; k < 2; ++k ) {
      const Run& r = L.run[k];
      if( !r.valid || r.frames.empty() ) continue;
      size_t idx = static_cast<size_t>( std::upper_bound( r.frames.begin(), r.frames.end(), t,
                                        []( double tt, const Frame& f ) { return tt < f.t; } ) - r.frames.begin() );
      if( idx > 0 ) --idx;
      const bool solid = ( k == L.shown );
      for( size_t i = 0; i < L.meshes[k].size() && i < r.frames[idx].pose.size(); ++i ) {
         L.meshes[k][i]->setTransform( r.frames[idx].pose[i] );
         L.meshes[k][i]->setEnabled( solid || L.showOther );
         L.meshes[k][i]->setTransparency( solid ? 1.0f : 0.3f );
      }
   }
}


//=================================================================================================
//  GUI
//=================================================================================================

const char* kScenarioNames[kNumScenarios] = { "Flat landing", "Spinning box", "Wedge", "Chain", "Pile" };

void drawControls()
{
   Lab& L = lab();
   Params& p = L.p;
   ImGui::SetNextWindowPos( ImVec2( 10, 10 ), ImGuiCond_FirstUseEver );
   ImGui::SetNextWindowSize( ImVec2( 420, 0 ), ImGuiCond_FirstUseEver );
   ImGui::Begin( "Impact lab" );

   if( ImGui::Combo( "scenario", &p.scenario, kScenarioNames, kNumScenarios ) )
      loadScenarioDefaults( p );
   ImGui::PushTextWrapPos( 0 );
   ImGui::TextDisabled( "%s", kScenarios[p.scenario].text );
   ImGui::PopTextWrapPos();
   ImGui::Separator();

   const double kZero = 0, kOne = 1;
   switch( p.scenario ) {
      case kSpinningBox: { const double lo = 0, hi = 15; ImGui::SliderScalar( "spin at touchdown [rad/s]", ImGuiDataType_Double, &p.spin, &lo, &hi, "%.1f" ); break; }
      case kWedge: {
         const double aLo = 15, aHi = 80, tLo = 0, tHi = 75;
         ImGui::SliderScalar( "half-angle [deg]", ImGuiDataType_Double, &p.halfAngle, &aLo, &aHi, "%.0f" );
         ImGui::SliderScalar( "incidence [deg]", ImGuiDataType_Double, &p.incidence, &tLo, &tHi, "%.0f" );
         break; }
      case kChain: ImGui::SliderInt( "balls", &p.chainBalls, 2, 8 ); break;
      case kPile:  ImGui::SliderInt( "boxes", &p.pileBoxes, 2, 30 ); break;
      default: break;
   }
   ImGui::SliderScalar( "restitution e", ImGuiDataType_Double, &p.e, &kZero, &kOne, "%.2f" );
   ImGui::SliderScalar( "friction mu", ImGuiDataType_Double, &p.mu, &kZero, &kOne, "%.2f" );
   const double dtLo = 2e-5, dtHi = 2e-3, durLo = 0.1, durHi = 6.0;
   ImGui::SliderScalar( "dt [s]", ImGuiDataType_Double, &p.dt, &dtLo, &dtHi, "%.1e", ImGuiSliderFlags_Logarithmic );
   ImGui::SliderScalar( "duration [s]", ImGuiDataType_Double, &p.duration, &durLo, &durHi, "%.1f" );
   ImGui::SliderInt( "post-pass sweeps", &p.sweeps, 1, 10 );
   if( ImGui::IsItemHovered( ImGuiHoveredFlags_DelayShort ) )
      ImGui::SetTooltip( "setRestitutionIterations(): Box3D uses 2, pe's default is 4.\nOnly the capped post-pass uses it." );
   ImGui::SliderInt( "max iterations", &p.maxIterations, 1, 200 );
   ImGui::Checkbox( "split impulse", &p.splitImpulse );

   ImGui::Separator();
   if( ImGui::Button( "Run both schemes" ) ) { runBoth(); registerMeshes(); L.playing = true; }
   ImGui::SameLine();
   if( ImGui::Button( L.playing ? "Pause" : "Play" ) ) L.playing = !L.playing;
   ImGui::SameLine();
   if( ImGui::Button( "Restart" ) ) { L.time = 0; L.playing = true; }
   const double tMax = L.run[0].valid && !L.run[0].t.empty() ? L.run[0].t.back() : p.duration;
   ImGui::SliderScalar( "time [s]", ImGuiDataType_Double, &L.time, &kZero, &tMax, "%.3f" );
   const double spLo = 0.02, spHi = 1.0;
   ImGui::SliderScalar( "playback speed", ImGuiDataType_Double, &L.speed, &spLo, &spHi, "%.2f x", ImGuiSliderFlags_Logarithmic );

   ImGui::Separator();
   ImGui::TextUnformatted( "drawn solid:" );
   ImGui::RadioButton( kSchemeName[kInSolve], &L.shown, kInSolve );
   ImGui::RadioButton( kSchemeName[kPostPass], &L.shown, kPostPass );
   ImGui::Checkbox( "show the other scheme transparent", &L.showOther );
   ImGui::TextDisabled( "orange: in-solve Newton, green: capped post-pass" );
   ImGui::TextDisabled( "space: play/pause, R: restart, Enter: run" );
   ImGui::End();
}


void drawResults()
{
   Lab& L = lab();
   ImGui::SetNextWindowPos( ImVec2( 440, 10 ), ImGuiCond_FirstUseEver );
   ImGui::SetNextWindowSize( ImVec2( 640, 560 ), ImGuiCond_FirstUseEver );
   ImGui::Begin( "Results" );

   if( ImGui::BeginTable( "summary", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg ) ) {
      ImGui::TableSetupColumn( "" );
      ImGui::TableSetupColumn( kSchemeShort[kInSolve] );
      ImGui::TableSetupColumn( kSchemeShort[kPostPass] );
      ImGui::TableHeadersRow();
      auto row = [&]( const char* label, auto fn ) {
         ImGui::TableNextRow(); ImGui::TableNextColumn(); ImGui::TextUnformatted( label );
         for( int k = 0; k < 2; ++k ) { ImGui::TableNextColumn(); fn( L.run[k] ); }
      };
      row( "kinetic energy after / before the first contact step", []( const Run& r ) {
         if( r.impactAfter >= 0 && r.impactBefore > 0 ) {
            const double q = r.impactAfter / r.impactBefore;
            if( q > 1.0 + 1e-6 ) ImGui::TextColored( ImVec4( 0.9f, 0.3f, 0.2f, 1 ), "%.4f  (energy created)", q );
            else ImGui::Text( "%.4f", q );
         } else ImGui::TextDisabled( "n/a" ); } );
      row( "e^2 (single-contact value)", [&]( const Run& ) { ImGui::Text( "%.4f", L.p.e * L.p.e ); } );
      row( "body 0 velocity after [m/s]", []( const Run& r ) { if( r.impactAfter >= 0 ) ImGui::Text( "(%.3f, %.3f, %.3f)", r.velAfter[0], r.velAfter[1], r.velAfter[2] ); else ImGui::TextDisabled( "n/a" ); } );
      row( "body 0 |omega| after [rad/s]", []( const Run& r ) { if( r.impactAfter >= 0 ) ImGui::Text( "%.4f", r.spinAfter ); else ImGui::TextDisabled( "n/a" ); } );
      row( "max mechanical energy / initial", []( const Run& r ) {
         if( r.maxMeRatio > 1.0 + 1e-6 ) ImGui::TextColored( ImVec4( 0.9f, 0.3f, 0.2f, 1 ), "%.4f", r.maxMeRatio ); else ImGui::Text( "%.4f", r.maxMeRatio ); } );
      if( L.p.scenario == kChain )
         row( "ball velocities after [m/s]", []( const Run& r ) {
            std::string s; char buf[32];
            for( double v : r.chainVel ) { std::snprintf( buf, sizeof( buf ), "%s%.3f", s.empty() ? "" : ", ", v ); s += buf; }
            ImGui::TextUnformatted( s.empty() ? "n/a" : s.c_str() ); } );
      row( "post-pass: armed visits / capped", []( const Run& r ) { if( r.armed ) ImGui::Text( "%zu / %zu", r.armed, r.capped ); else ImGui::TextDisabled( "-" ); } );
      row( "post-pass: clamp hits, impulse [N s]", []( const Run& r ) {
         if( r.armed == 0 ) ImGui::TextDisabled( "-" );
         else if( r.clamped ) ImGui::Text( "%zu, %.3e", r.clamped, r.clampedImpulse );
         else ImGui::TextUnformatted( "0 (clamp never engaged)" ); } );
      row( "post-pass: restitution impulse [N s]", []( const Run& r ) { if( r.armed ) ImGui::Text( "%.4f", r.restitutionImpulse ); else ImGui::TextDisabled( "-" ); } );
      ImGui::EndTable();
   }
   for( int k = 0; k < 2; ++k )
      if( !L.run[k].error.empty() ) ImGui::TextColored( ImVec4( 0.9f, 0.3f, 0.2f, 1 ), "%s: %s", kSchemeShort[k], L.run[k].error.c_str() );

   const bool mechanical = kScenarios[L.p.scenario].gravity;
   if( ImPlot::BeginPlot( mechanical ? "mechanical energy" : "kinetic energy", ImVec2( -1, -1 ) ) ) {
      ImPlot::SetupAxes( "t [s]", mechanical ? "E_kin + E_pot [J]" : "E_kin [J]" );
      for( int k = 0; k < 2; ++k ) {
         const Run& r = L.run[k];
         if( !r.valid ) continue;
         const glm::vec3 c = schemeColor( k );
         ImPlot::SetNextLineStyle( ImVec4( c.r, c.g, c.b, 1 ), 2.0f );
         const std::vector<double>& y = mechanical ? r.me : r.ke;
         ImPlot::PlotLine( kSchemeShort[k], r.t.data(), y.data(), static_cast<int>( r.t.size() ) );
      }
      double now = L.time;
      ImPlot::SetNextLineStyle( ImVec4( 0.5f, 0.5f, 0.5f, 1 ), 1.0f );
      ImPlot::PlotInfLines( "now", &now, 1 );
      ImPlot::EndPlot();
   }
   ImGui::End();
}


void guiCallback()
{
   Lab& L = lab();
   ImGuiIO& io = ImGui::GetIO();
   if( !io.WantTextInput ) {
      if( ImGui::IsKeyPressed( ImGuiKey_Space, false ) ) L.playing = !L.playing;
      if( ImGui::IsKeyPressed( ImGuiKey_R, false ) ) { L.time = 0; L.playing = true; }
      if( ImGui::IsKeyPressed( ImGuiKey_Enter, false ) ) { runBoth(); registerMeshes(); L.playing = true; }
   }
   drawControls();
   drawResults();
   if( L.playing ) {
      const double tMax = L.run[0].valid && !L.run[0].t.empty() ? L.run[0].t.back() : L.p.duration;
      L.time += io.DeltaTime * L.speed;
      if( L.time > tMax ) L.time = 0;   // loop
   }
   showFrame( L.time );
}


//=================================================================================================
//  HEADLESS
//=================================================================================================

void printSummary()
{
   const Lab& L = lab();
   std::printf( "%-13s", kScenarioNames[L.p.scenario] );
   for( int k = 0; k < 2; ++k ) {
      const Run& r = L.run[k];
      if( !r.valid ) { std::printf( "  %s: %s", kSchemeShort[k], r.error.c_str() ); continue; }
      const double q = ( r.impactAfter >= 0 && r.impactBefore > 0 ) ? r.impactAfter / r.impactBefore : -1;
      std::printf( "  | %-16s KE ratio %7.4f  |omega'| %7.4f  maxE/E0 %7.4f", kSchemeShort[k], q, r.spinAfter, r.maxMeRatio );
      if( k == kPostPass ) std::printf( "  armed %zu capped %zu clamped %zu (%.2e)", r.armed, r.capped, r.clamped, r.clampedImpulse );
   }
   std::printf( "\n" );
}

bool batch( bool smoke )
{
   Lab& L = lab();
   bool ok = true;
   for( int sc = 0; sc < kNumScenarios; ++sc ) {
      L.p.scenario = sc;
      loadScenarioDefaults( L.p );
      if( smoke && sc == kPile ) { L.p.duration = 1.0; L.p.pileBoxes = 6; }
      runBoth();
      printSummary();
      for( int k = 0; k < 2; ++k ) if( !L.run[k].valid ) ok = false;
      if( smoke ) {
         const Run& a = L.run[kInSolve];
         const Run& b = L.run[kPostPass];
         if( sc == kSpinningBox ) {
            const double qa = a.impactAfter / a.impactBefore, qb = b.impactAfter / b.impactBefore;
            if( a.impactAfter < 0 || b.impactAfter < 0 ) { std::printf( "FAIL: spinning box, no impact measured\n" ); ok = false; }
            if( !( qa > 1.02 ) ) { std::printf( "FAIL: spinning box, in-solve scheme should create energy at e = 1 (got %.4f)\n", qa ); ok = false; }
            if( !( qb <= 1.0 + 1e-6 ) ) { std::printf( "FAIL: spinning box, capped post-pass created energy (%.4f)\n", qb ); ok = false; }
         }
         if( sc == kFlatLanding && !( b.spinAfter < 0.01 ) ) { std::printf( "FAIL: flat landing, post-pass left a spin of %.4f rad/s\n", b.spinAfter ); ok = false; }
         if( sc == kFlatLanding && !( std::fabs( b.velAfter[2] - L.p.e ) < 0.02 * L.p.e ) ) { std::printf( "FAIL: flat landing, post-pass rebound %.4f\n", b.velAfter[2] ); ok = false; }
      }
   }
   return ok;
}

} // namespace


int main( int argc, char** argv )
{
   Lab& L = lab();
   bool smoke = false, batchMode = false;
   std::string shot;
   double shotTime = 0.2;
   for( int i = 1; i < argc; ++i ) {
      const std::string arg( argv[i] );
      if( arg == "--smoke" ) smoke = true;
      else if( arg == "--batch" ) batchMode = true;
      else if( arg == "--scenario" && i + 1 < argc ) { L.p.scenario = std::max( 0, std::min( kNumScenarios - 1, std::atoi( argv[++i] ) ) ); }
      else if( arg == "--sweeps" && i + 1 < argc ) L.p.sweeps = std::max( 1, std::atoi( argv[++i] ) );
      else if( arg == "--screenshot" && i + 1 < argc ) shot = argv[++i];
      else if( arg == "--at" && i + 1 < argc ) shotTime = std::atof( argv[++i] );
      else {
         std::fprintf( stderr, "usage: %s [--scenario 0..4] [--sweeps n] [--smoke | --batch | --screenshot file.png [--at T]]\n", argv[0] );
         return EXIT_FAILURE;
      }
   }
   loadScenarioDefaults( L.p );

   if( smoke || batchMode ) {
      const bool ok = batch( smoke );
      if( smoke ) std::printf( "\nsmoke test %s\n", ok ? "passed" : "FAILED" );
      return ok ? EXIT_SUCCESS : EXIT_FAILURE;
   }

   polyscope::options::programName = "PE impact lab";
   polyscope::options::openImGuiWindowForUserCallback = false;
   polyscope::options::transparencyMode = polyscope::TransparencyMode::Pretty;
   polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::None;
   polyscope::options::automaticallyComputeSceneExtents = false;
   polyscope::view::setUpDir( polyscope::UpDir::ZUp );
   polyscope::init();
   ImPlot::CreateContext();
   L.gui = true;

   runBoth();
   registerMeshes();
   showFrame( 0 );
   polyscope::state::userCallback = guiCallback;

   if( !shot.empty() ) {
      L.time = shotTime;
      L.playing = false;
      for( int i = 0; i < 3; ++i ) polyscope::frameTick();
      polyscope::screenshot( shot, /*transparentBG=*/false );
      std::printf( "screenshot at t = %.3f s: %s\n", L.time, shot.c_str() );
   }
   else {
      L.playing = true;
      polyscope::show();
   }

   ImPlot::DestroyContext();
   return EXIT_SUCCESS;
}
