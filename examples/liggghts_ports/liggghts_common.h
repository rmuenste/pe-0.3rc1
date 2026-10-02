//=================================================================================================
/*!
 *  \file liggghts_common.h
 *  \brief Helpers shared by the pe ports of the LIGGGHTS-PUBLIC tutorials.
 *
 *  Everything a LIGGGHTS input script gets for free and pe does not provide:
 *  particle insertion (insert/pack, insert/stream), size distributions with mass fractions,
 *  a thermo-style screen output, and a small ASCII STL reader for insertion faces.
 */
//=================================================================================================
#ifndef _PE_EXAMPLES_LIGGGHTS_COMMON_H_
#define _PE_EXAMPLES_LIGGGHTS_COMMON_H_

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <pe/core.h>
#include <pe/support.h>
#include <pe/vtk.h>
#include <pe/util.h>
#include <pe/util/timing/WcTimer.h>

namespace lp {

using namespace pe;

//*************************************************************************************************
// Command line: --dt, --tend, --friction, --out, --no-vtk, plus free key=value pairs
//*************************************************************************************************
struct Args {
   real        dt;          // 0 means "use the case default"
   real        tEnd;
   real        friction;    // per material; <0 means "use the case default"
   real        erp;         // error reduction parameter of the hard-contact solver (default 0.5)
   std::string out;
   bool        vtk;
   std::string caseName;
   Args() : dt(0), tEnd(0), friction(-1), erp(0.5), out("./paraview"), vtk(true) {}
};

inline Args parseArgs( int argc, char* argv[], const std::string& usage )
{
   Args a;
   for( int i=1; i<argc; ++i ) {
      const std::string s( argv[i] );
      if( s == "--no-vtk" ) a.vtk = false;
      else if( s == "--dt" && i+1 < argc ) a.dt = std::atof( argv[++i] );
      else if( s == "--tend" && i+1 < argc ) a.tEnd = std::atof( argv[++i] );
      else if( s == "--friction" && i+1 < argc ) a.friction = std::atof( argv[++i] );
      else if( s == "--erp" && i+1 < argc ) a.erp = std::atof( argv[++i] );
      else if( s == "--out" && i+1 < argc ) a.out = argv[++i];
      else if( s == "--case" && i+1 < argc ) a.caseName = argv[++i];
      else { std::cout << usage << std::endl; std::exit( s == "--help" || s == "-h" ? 0 : 1 ); }
   }
   return a;
}

//*************************************************************************************************
// Particle sizes
//*************************************************************************************************
inline real sphereMass( real density, real r ) { return density * real(4.0/3.0) * M_PI * r*r*r; }

/*! LIGGGHTS "fix particledistribution/discrete" weights are MASS fractions. Returns the number
 *  fraction of the first of two radii. */
inline real numberFraction0( real massFrac0, real massFrac1, real r0, real r1, real density )
{
   const real n0( massFrac0 / sphereMass( density, r0 ) );
   const real n1( massFrac1 / sphereMass( density, r1 ) );
   return n0 / ( n0 + n1 );
}

//*************************************************************************************************
// Overlap check against pending candidates and all spheres already in the world
//*************************************************************************************************
struct Candidate { Vec3 pos; real r; };

inline bool overlaps( const Vec3& pos, real r, const std::vector<Candidate>& pending, WorldID world )
{
   for( std::vector<Candidate>::const_iterator c=pending.begin(); c!=pending.end(); ++c )
      if( ( c->pos - pos ).sqrLength() < sq( c->r + r ) ) return true;
   for( World::Bodies::CastIterator<Sphere> s=world->begin<Sphere>(); s!=world->end<Sphere>(); ++s )
      if( ( s->getPosition() - pos ).sqrLength() < sq( s->getRadius() + r ) ) return true;
   return false;
}

//*************************************************************************************************
// Region samplers (LIGGGHTS "region block" / "region cylinder z")
//*************************************************************************************************
struct Region {
   virtual ~Region() {}
   //! Random point for a particle of radius r; shrink>0 keeps the whole particle inside (all_in yes)
   virtual Vec3 sample( real shrink ) const = 0;
};

struct BlockRegion : public Region {
   Vec3 lo, hi;
   BlockRegion( const Vec3& l, const Vec3& h ) : lo(l), hi(h) {}
   virtual Vec3 sample( real s ) const {
      return Vec3( rand<real>( lo[0]+s, hi[0]-s ), rand<real>( lo[1]+s, hi[1]-s ), rand<real>( lo[2]+s, hi[2]-s ) );
   }
};

struct CylinderZRegion : public Region {
   real cx, cy, radius, zlo, zhi;
   CylinderZRegion( real x, real y, real r, real z0, real z1 ) : cx(x), cy(y), radius(r), zlo(z0), zhi(z1) {}
   virtual Vec3 sample( real s ) const {
      const real rr( ( radius - s ) * std::sqrt( rand<real>( 0.0, 1.0 ) ) );
      const real phi( rand<real>( 0.0, 2.0*M_PI ) );
      return Vec3( cx + rr*std::cos(phi), cy + rr*std::sin(phi), rand<real>( zlo+s, zhi-s ) );
   }
};

//*************************************************************************************************
// Minimal ASCII STL reader (insertion faces) and area-weighted point sampling on the facets
//*************************************************************************************************
struct Tri { Vec3 a, b, c; real area; };

inline std::vector<Tri> readAsciiSTL( const std::string& file, real scale = 1.0 )
{
   std::vector<Tri> tris;
   std::ifstream in( file.c_str() );
   if( !in ) throw std::runtime_error( "Cannot open STL file " + file );
   std::string tok;
   std::vector<Vec3> v;
   while( in >> tok ) {
      if( tok == "vertex" ) {
         real x, y, z; in >> x >> y >> z;
         v.push_back( Vec3( x, y, z ) * scale );
         if( v.size() == 3 ) {
            Tri t; t.a = v[0]; t.b = v[1]; t.c = v[2];
            t.area = real(0.5) * ( ( t.b - t.a ) % ( t.c - t.a ) ).length();
            tris.push_back( t );
            v.clear();
         }
      }
   }
   return tris;
}

inline Vec3 randomPointOnTriangles( const std::vector<Tri>& tris )
{
   real total( 0 );
   for( size_t i=0; i<tris.size(); ++i ) total += tris[i].area;
   real pick( rand<real>( 0.0, total ) );
   size_t i( 0 );
   for( ; i+1<tris.size(); ++i ) { if( pick < tris[i].area ) break; pick -= tris[i].area; }
   real u( rand<real>( 0.0, 1.0 ) ), w( rand<real>( 0.0, 1.0 ) );
   if( u + w > 1 ) { u = 1 - u; w = 1 - w; }
   return tris[i].a + ( tris[i].b - tris[i].a ) * u + ( tris[i].c - tris[i].a ) * w;
}

//*************************************************************************************************
// Insertion helpers
//*************************************************************************************************
/*! LIGGGHTS fix insert/pack with "particles_in_region N": N particles of radius radiusFn() into a
 *  region; allIn=true shrinks the centre range by the radius; maxAttempt positions per particle.
 *  Returns the number of particles actually created. */
template< typename RadiusFn >
inline unsigned int insertPack( WorldID world, unsigned int& id, unsigned int n, RadiusFn radiusFn,
                                const Region& region, bool allIn, unsigned int maxAttempt,
                                const Vec3& vel, MaterialID mat, unsigned int maxFailures = 50 )
{
   std::vector<Candidate> pending;
   unsigned int failures( 0 );
   while( pending.size() < n && failures < maxFailures ) {
      const real r( radiusFn() );
      bool found( false ); Vec3 pos;
      for( unsigned int a=0; a<maxAttempt && !found; ++a ) {
         pos = region.sample( allIn ? r : real(0) );
         found = !overlaps( pos, r, pending, world );
      }
      if( !found ) { ++failures; continue; }
      Candidate c; c.pos = pos; c.r = r; pending.push_back( c );
   }
   for( size_t i=0; i<pending.size(); ++i ) {
      SphereID s = createSphere( ++id, pending[i].pos, pending[i].r, mat );
      s->setLinearVel( vel );
   }
   return static_cast<unsigned int>( pending.size() );
}

/*! LIGGGHTS fix insert/stream. The face is extruded by \a extrude along its normal, on the side
 *  the particles come from (the side opposite to the velocity). Centres only ("all_in no").
 *  LIGGGHTS moves freshly inserted particles kinematically with the normal component of the
 *  insertion velocity (no gravity, no contacts) until they cross the face, and only then
 *  releases them with the full insertion velocity. With kinematic=true the same is done here:
 *  the spheres are created with collisions disabled and their velocity is pinned to the normal
 *  component after every step by releaseStream(), which also frees the ones that crossed. */
struct StreamFace {
   std::vector<Tri> tris;
   Vec3 normal;      // unit normal pointing to the insertion side (opposite to the velocity)
   real offset;      // normal . x = offset on the face plane
   Vec3 velocity;    // full insertion velocity
   Vec3 kinVelocity; // (velocity . normal) * normal
   StreamFace( const std::vector<Tri>& t, const Vec3& vel ) : tris(t), velocity(vel) {
      normal = ( ( t[0].b - t[0].a ) % ( t[0].c - t[0].a ) ).getNormalized();
      if( trans(normal) * vel > 0 ) normal = -normal;
      offset = trans(normal) * t[0].a;
      kinVelocity = normal * ( trans(normal) * vel );
   }
   //! true once a point has crossed the face (moved to the velocity side)
   bool crossed( const Vec3& p ) const { return trans(normal) * p - offset < real(0); }
};

template< typename RadiusFn >
inline unsigned int insertStream( WorldID world, unsigned int& id, unsigned int n, RadiusFn radiusFn,
                                  const StreamFace& face, real extrude, unsigned int maxAttempt,
                                  MaterialID mat, bool kinematic, std::vector<SphereID>& unreleased,
                                  unsigned int maxFailures = 50 )
{
   std::vector<Candidate> pending;
   unsigned int failures( 0 );
   while( pending.size() < n && failures < maxFailures ) {
      const real r( radiusFn() );
      bool found( false ); Vec3 pos;
      for( unsigned int a=0; a<maxAttempt && !found; ++a ) {
         pos = randomPointOnTriangles( face.tris ) + face.normal * rand<real>( 0.0, extrude );
         found = !overlaps( pos, r, pending, world );
      }
      if( !found ) { ++failures; continue; }
      Candidate c; c.pos = pos; c.r = r; pending.push_back( c );
   }
   for( size_t i=0; i<pending.size(); ++i ) {
      SphereID s = createSphere( ++id, pending[i].pos, pending[i].r, mat );
      if( kinematic ) {
         // Deliberately not created as a fixed body: the fixed flag of a registered body is not
         // meant to be toggled, so the velocity is simply pinned to the kinematic value after every
         // step in releaseStream(); the gravity the solver adds to the position update within one
         // step (g*dt^2) is negligible.
         s->setCollisionEnabled( false );
         s->setLinearVel( face.kinVelocity );
         unreleased.push_back( s );
      }
      else s->setLinearVel( face.velocity );
   }
   return static_cast<unsigned int>( pending.size() );
}

//! true if sphere s overlaps any other sphere that takes part in collision detection
inline bool overlapsEnabled( SphereID s, WorldID world )
{
   for( World::Bodies::CastIterator<Sphere> o=world->begin<Sphere>(); o!=world->end<Sphere>(); ++o ) {
      if( *o == s || !o->isCollisionEnabled() ) continue;
      if( ( o->getPosition() - s->getPosition() ).sqrLength() < sq( o->getRadius() + s->getRadius() ) ) return true;
   }
   return false;
}

/*! Releases kinematic stream particles that have crossed the face. Returns the number released.
 *  A particle is only released once it does not overlap an already released neighbour: while it
 *  glides with collisions disabled, released neighbours drift sideways under it, and the hard
 *  contact solver would remove such an overlap in a single step (velocity jump of the order
 *  erp * depth / dt, i.e. 10 m/s for 5 mm at dt 2.5e-4). LIGGGHTS has the same overlaps but its
 *  Hertz springs resolve them gently. */
inline unsigned int releaseStream( std::vector<SphereID>& unreleased, const StreamFace& face, WorldID world )
{
   unsigned int released( 0 );
   std::vector<SphereID> keep;
   for( size_t i=0; i<unreleased.size(); ++i ) {
      SphereID s = unreleased[i];
      if( face.crossed( s->getPosition() ) && !overlapsEnabled( s, world ) ) {
         s->setCollisionEnabled( true );
         s->setLinearVel( face.velocity );
         ++released;
      }
      else {
         s->setLinearVel( face.kinVelocity );   // undo the gravity increment of the last step
         s->setAngularVel( 0.0, 0.0, 0.0 );
         keep.push_back( s );
      }
   }
   unreleased.swap( keep );
   return released;
}

//*************************************************************************************************
// Thermo-style output: step, time, atoms, ke (translational, like LIGGGHTS "ke"), rke, vmax
//*************************************************************************************************
struct Thermo {
   real ke, rke, vmax, zmin, zmax;
   unsigned int n;
   void measure( WorldID world ) {
      ke = rke = vmax = 0; zmin = 1e30; zmax = -1e30; n = 0;
      for( World::Bodies::CastIterator<Sphere> s=world->begin<Sphere>(); s!=world->end<Sphere>(); ++s ) {
         const real v2( s->getLinearVel().sqrLength() );
         const real m( s->getMass() ), r( s->getRadius() );
         ke  += real(0.5) * m * v2;
         rke += real(0.5) * real(0.4) * m * r*r * s->getAngularVel().sqrLength();
         vmax = std::max( vmax, std::sqrt( v2 ) );
         zmin = std::min( zmin, s->getPosition()[2] ); zmax = std::max( zmax, s->getPosition()[2] );
         ++n;
      }
      for( World::Bodies::CastIterator<Union> u=world->begin<Union>(); u!=world->end<Union>(); ++u ) {
         const real v2( u->getLinearVel().sqrLength() );
         ke  += real(0.5) * u->getMass() * v2;
         const Vec3 w( u->getAngularVel() );
         rke += real(0.5) * ( trans(w) * ( u->getInertia() * w ) );
         vmax = std::max( vmax, std::sqrt( v2 ) );
         zmin = std::min( zmin, u->getPosition()[2] ); zmax = std::max( zmax, u->getPosition()[2] );
         ++n;
      }
   }
   static void header() {
      std::cout << std::setw(8) << "step" << std::setw(10) << "time" << std::setw(8) << "atoms"
                << std::setw(14) << "ke" << std::setw(14) << "rke" << std::setw(10) << "vmax"
                << std::setw(10) << "zmin" << std::setw(10) << "zmax" << "\n";
   }
   void print( unsigned int step, real time ) const {
      std::cout << std::setw(8) << step << std::setw(10) << std::fixed << std::setprecision(4) << time
                << std::setw(8) << n
                << std::setw(14) << std::scientific << std::setprecision(5) << ke
                << std::setw(14) << rke
                << std::setw(10) << std::fixed << std::setprecision(4) << vmax
                << std::setw(10) << zmin << std::setw(10) << zmax << "\n" << std::flush;
   }
};

inline unsigned int stepsFor( real t, real dt ) { return static_cast<unsigned int>( std::floor( t/dt + 0.5 ) ); }

} // namespace lp

#endif
