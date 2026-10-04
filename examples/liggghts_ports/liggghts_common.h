//=================================================================================================
/*!
 *  \file liggghts_common.h
 *  \brief Helpers shared by the pe ports of the LIGGGHTS-PUBLIC tutorials.
 *
 *  Everything a LIGGGHTS input script gets for free and pe does not provide:
 *  particle insertion (insert/pack, insert/stream), size distributions with mass fractions,
 *  a thermo-style screen output, and a small ASCII STL reader for insertion faces.
 *
 *  Parallel runs (MPI builds): the ports converted so far (conveyor, cylinder_pack) run the same
 *  source serially and under MPI. The helpers in the "Parallel runs" section give them a slab
 *  decomposition, an insertion that draws identical candidates on every process (same seed, same
 *  random calls) against the spheres of all processes and creates each particle on its owner
 *  only, and a thermo output reduced over all processes and printed by the root. With one process
 *  everything degrades to the serial behaviour. insert_stream, mesh_gran, moving_mesh_gran and
 *  multisphere are still serial.
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
#if HAVE_MPI
#include <mpi.h>
#include <pe/core/MPITrait.h>
#endif

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
   bool        splitImpulse;   // --split-impulse: position correction by pseudo velocities (CollisionSystem::setSplitImpulse)
   std::string out;
   bool        vtk;
   std::string caseName;
   Args() : dt(0), tEnd(0), friction(-1), erp(0.5), splitImpulse(false), out("./paraview"), vtk(true) {}
};

inline Args parseArgs( int argc, char* argv[], const std::string& usage )
{
   Args a;
   for( int i=1; i<argc; ++i ) {
      const std::string s( argv[i] );
      if( s == "--no-vtk" ) a.vtk = false;
      else if( s == "--split-impulse" ) a.splitImpulse = true;
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

//*************************************************************************************************
// Parallel runs (MPI builds; with one process everything degrades to the serial behaviour)
//*************************************************************************************************
//! Exit code of a port that refuses to run under an unintended solver (CTest: SKIP_RETURN_CODE).
const int kSkipExitCode = 77;

inline bool isRoot() { return MPISettings::rank() == MPISettings::root(); }
inline int  numProcesses() { return MPISettings::size(); }

//! MPI_Init / MPI_Finalize around main() (nothing without MPI).
struct MpiScope {
   MpiScope( int& argc, char**& argv ) {
#if HAVE_MPI
      MPI_Init( &argc, &argv );
#else
      (void)argc; (void)argv;
#endif
   }
   ~MpiScope() {
#if HAVE_MPI
      MPI_Finalize();
#endif
   }
};

//! std::cout on the root process, nothing elsewhere: rout() << ... .
struct RootOut {
   template< typename T > RootOut& operator<<( const T& v ) { if( isRoot() ) std::cout << v; return *this; }
   RootOut& operator<<( std::ostream& (*m)( std::ostream& ) ) { if( isRoot() ) std::cout << m; return *this; }
   RootOut& operator<<( std::ios_base& (*m)( std::ios_base& ) ) { if( isRoot() ) std::cout << m; return *this; }
};
inline RootOut rout() { return RootOut(); }

/*! 1-D slab decomposition of the whole space along  axis (0 = x, 1 = y, 2 = z): process k owns
 *  the slab [lo + k w, lo + (k+1) w) with w = (hi - lo) / size; the first slab has no lower and
 *  the last no upper bound, so no particle can ever be outside every domain (particles leaving
 *  [lo, hi] stay owned by the end processes). Each slab is connected to its two neighbours, so w
 *  must exceed the largest particle diameter plus the contact threshold. Nothing with one
 *  process. */
inline void decomposeSlabs( int axis, real lo, real hi )
{
#if HAVE_MPI
   const int np( MPISettings::size() ), me( MPISettings::rank() );
   if( np <= 1 ) return;
   Vec3 n( 0, 0, 0 ); n[axis] = real(1);
   const real w( ( hi - lo ) / np );
   // HalfSpace( normal, d ) is the set normal . x >= d
   const auto lower = [&]( int k ) { return HalfSpace(  n,  lo + k*w ); };         // x_axis >= lo + k w
   const auto upper = [&]( int k ) { return HalfSpace( -n, -( lo + (k+1)*w ) ); }; // x_axis <= lo + (k+1) w
   const auto define = [&]( int k, bool local ) {
      if( k == 0 )           { if( local ) defineLocalDomain( upper( 0 ) ); else connect( k, upper( 0 ) ); }
      else if( k == np - 1 ) { if( local ) defineLocalDomain( lower( k ) ); else connect( k, lower( k ) ); }
      else                   { if( local ) defineLocalDomain( intersect( lower( k ), upper( k ) ) );
                               else        connect( k, intersect( lower( k ), upper( k ) ) ); }
   };
   define( me, true );
   if( me > 0 )      define( me - 1, false );
   if( me < np - 1 ) define( me + 1, false );
#else
   (void)axis; (void)lo; (void)hi;
#endif
}

//! Shadow copies and migrations for bodies created outside a simulation step.
inline void synchronizeIfParallel( WorldID world )
{
#if HAVE_MPI
   if( MPISettings::size() > 1 ) world->synchronize();
#else
   (void)world;
#endif
}

//! Positions and radii of the spheres of ALL processes (this process's own bodies, gathered).
inline void gatherSpheres( WorldID world, std::vector<Candidate>& all )
{
   std::vector<real> local;
   for( World::Bodies::CastIterator<Sphere> s=world->begin<Sphere>(); s!=world->end<Sphere>(); ++s ) {
      if( s->isRemote() ) continue;
      const Vec3& p( s->getPosition() );
      local.push_back( p[0] ); local.push_back( p[1] ); local.push_back( p[2] ); local.push_back( s->getRadius() );
   }
   std::vector<real> global;
#if HAVE_MPI
   if( MPISettings::size() > 1 ) {
      const int np( MPISettings::size() );
      std::vector<int> counts( np ), displs( np );
      int mine( static_cast<int>( local.size() ) );
      MPI_Allgather( &mine, 1, MPI_INT, &counts[0], 1, MPI_INT, MPISettings::comm() );
      int total( 0 );
      for( int i=0; i<np; ++i ) { displs[i] = total; total += counts[i]; }
      global.resize( total );
      MPI_Allgatherv( mine > 0 ? &local[0] : 0, mine, MPITrait<real>::getType(),
                      total > 0 ? &global[0] : 0, &counts[0], &displs[0], MPITrait<real>::getType(), MPISettings::comm() );
   }
   else global.swap( local );
#else
   global.swap( local );
#endif
   all.clear();
   for( size_t i=0; i+3<global.size(); i+=4 ) {
      Candidate c; c.pos = Vec3( global[i], global[i+1], global[i+2] ); c.r = global[i+3];
      all.push_back( c );
   }
}

//! Number of spheres of all processes.
inline unsigned int globalSphereCount( WorldID world )
{
   unsigned int n( 0 );
   for( World::Bodies::CastIterator<Sphere> s=world->begin<Sphere>(); s!=world->end<Sphere>(); ++s )
      if( !s->isRemote() ) ++n;
#if HAVE_MPI
   if( MPISettings::size() > 1 ) {
      unsigned int total( 0 );
      MPI_Allreduce( &n, &total, 1, MPI_UNSIGNED, MPI_SUM, MPISettings::comm() );
      n = total;
   }
#endif
   return n;
}

//! Overlap of (pos, r) with a pending candidate or an existing sphere (list from gatherSpheres()).
inline bool overlaps( const Vec3& pos, real r, const std::vector<Candidate>& pending, const std::vector<Candidate>& existing )
{
   for( std::vector<Candidate>::const_iterator c=pending.begin(); c!=pending.end(); ++c )
      if( ( c->pos - pos ).sqrLength() < sq( c->r + r ) ) return true;
   for( std::vector<Candidate>::const_iterator c=existing.begin(); c!=existing.end(); ++c )
      if( ( c->pos - pos ).sqrLength() < sq( c->r + r ) ) return true;
   return false;
}

//! Serial-only variant against this process's world (used by the ports that are still serial).
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
/*! LIGGGHTS fix insert/pack. Parallel: every process draws the same candidates (same seed and the
 *  same random calls, checked against the spheres of all processes) and creates the ones it owns;
 *  the returned count and the id counter are the same everywhere. */
template< typename RadiusFn >
inline unsigned int insertPack( WorldID world, unsigned int& id, unsigned int n, RadiusFn radiusFn,
                                const Region& region, bool allIn, unsigned int maxAttempt,
                                const Vec3& vel, MaterialID mat, unsigned int maxFailures = 50 )
{
   std::vector<Candidate> existing;
   gatherSpheres( world, existing );
   std::vector<Candidate> pending;
   unsigned int failures( 0 );
   while( pending.size() < n && failures < maxFailures ) {
      const real r( radiusFn() );
      bool found( false ); Vec3 pos;
      for( unsigned int a=0; a<maxAttempt && !found; ++a ) {
         pos = region.sample( allIn ? r : real(0) );
         found = !overlaps( pos, r, pending, existing );
      }
      if( !found ) { ++failures; continue; }
      Candidate c; c.pos = pos; c.r = r; pending.push_back( c );
   }
   for( size_t i=0; i<pending.size(); ++i ) {
      ++id;
      if( !world->ownsPoint( pending[i].pos ) ) continue;
      SphereID s = createSphere( id, pending[i].pos, pending[i].r, mat );
      s->setLinearVel( vel );
   }
   synchronizeIfParallel( world );
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
//! Reduced over all processes in measure(); header() and print() write on the root only.
struct Thermo {
   real ke, rke, vmax;
   Vec3 lo, hi;          // bounding box of the body centres
   unsigned int n;
   real zmin() const { return lo[2]; }
   real zmax() const { return hi[2]; }
   void measure( WorldID world ) {
      ke = rke = vmax = 0; lo = Vec3( 1e30, 1e30, 1e30 ); hi = -lo; n = 0;
      for( World::Bodies::CastIterator<Sphere> s=world->begin<Sphere>(); s!=world->end<Sphere>(); ++s ) {
         if( s->isRemote() ) continue;
         const real v2( s->getLinearVel().sqrLength() );
         const real m( s->getMass() ), r( s->getRadius() );
         ke  += real(0.5) * m * v2;
         rke += real(0.5) * real(0.4) * m * r*r * s->getAngularVel().sqrLength();
         vmax = std::max( vmax, std::sqrt( v2 ) );
         extend( s->getPosition() );
         ++n;
      }
      for( World::Bodies::CastIterator<Union> u=world->begin<Union>(); u!=world->end<Union>(); ++u ) {
         if( u->isRemote() ) continue;
         const real v2( u->getLinearVel().sqrLength() );
         ke  += real(0.5) * u->getMass() * v2;
         const Vec3 w( u->getAngularVel() );
         rke += real(0.5) * ( trans(w) * ( u->getInertia() * w ) );
         vmax = std::max( vmax, std::sqrt( v2 ) );
         extend( u->getPosition() );
         ++n;
      }
#if HAVE_MPI
      if( MPISettings::size() > 1 ) {
         real sums[3] = { ke, rke, real(n) }, sumsAll[3];
         real maxs[4] = { vmax, hi[0], hi[1], hi[2] }, maxsAll[4];
         real mins[3] = { lo[0], lo[1], lo[2] }, minsAll[3];
         MPI_Allreduce( sums, sumsAll, 3, MPITrait<real>::getType(), MPI_SUM, MPISettings::comm() );
         MPI_Allreduce( maxs, maxsAll, 4, MPITrait<real>::getType(), MPI_MAX, MPISettings::comm() );
         MPI_Allreduce( mins, minsAll, 3, MPITrait<real>::getType(), MPI_MIN, MPISettings::comm() );
         ke = sumsAll[0]; rke = sumsAll[1]; n = static_cast<unsigned int>( sumsAll[2] + real(0.5) );
         vmax = maxsAll[0]; hi = Vec3( maxsAll[1], maxsAll[2], maxsAll[3] ); lo = Vec3( minsAll[0], minsAll[1], minsAll[2] );
      }
#endif
   }
   void extend( const Vec3& p ) {
      for( int k=0; k<3; ++k ) { lo[k] = std::min( lo[k], p[k] ); hi[k] = std::max( hi[k], p[k] ); }
   }
   static void header() {
      rout() << std::setw(8) << "step" << std::setw(10) << "time" << std::setw(8) << "atoms"
             << std::setw(14) << "ke" << std::setw(14) << "rke" << std::setw(10) << "vmax"
             << std::setw(10) << "zmin" << std::setw(10) << "zmax" << "\n";
   }
   void print( unsigned int step, real time ) const {
      rout() << std::setw(8) << step << std::setw(10) << std::fixed << std::setprecision(4) << time
             << std::setw(8) << n
             << std::setw(14) << std::scientific << std::setprecision(5) << ke
             << std::setw(14) << rke
             << std::setw(10) << std::fixed << std::setprecision(4) << vmax
             << std::setw(10) << zmin() << std::setw(10) << zmax() << "\n" << std::flush;
   }
};

inline unsigned int stepsFor( real t, real dt ) { return static_cast<unsigned int>( std::floor( t/dt + 0.5 ) ); }

} // namespace lp

#endif
