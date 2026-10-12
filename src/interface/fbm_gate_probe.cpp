//=================================================================================================
/*!
 * \file src/interface/fbm_gate_probe.cpp
 * \brief Layer-2 FBM classification gate: body construction and C entry points.
 *
 * See pe/interface/fbm_gate_probe.h for the construction this implements and why.
 */
//=================================================================================================

#include <pe/interface/fbm_gate_probe.h>
#include <pe/config/SimulationConfig.h>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <sstream>

using namespace pe;
using namespace pe::detection::coarse::hhg;   // hierarchyFactor

namespace fbmgate {

Construction& construction() {
   static Construction c;
   return c;
}

const char* shapeName( int shape ) {
   switch( shape ) {
      case SHAPE_SPHERE:    return "sphere";
      case SHAPE_BOX:       return "box";
      case SHAPE_CAPSULE:   return "capsule";
      case SHAPE_ELLIPSOID: return "ellipsoid";
      case SHAPE_CYLINDER:  return "cylinder";
      default:              return "?";
   }
}

// pe::id_t spelled out: with `using namespace pe` in scope, a bare id_t is ambiguous against the
// POSIX ::id_t that the system headers bring in, and GCC then mis-parses this declaration.
BodyID createProbeShape( pe::id_t uid, int shape, const Vec3& pos, real ex, MaterialID material ) {
   BodyID body = 0;
   switch( shape ) {
      case SHAPE_SPHERE:
         body = createSphere( uid, pos, ex, material, true );
         break;
      case SHAPE_BOX:
         body = createBox( uid, pos, Vec3( 2*ex, 2*ex, 2*ex ), material, true );
         break;
      case SHAPE_CAPSULE:
         // created x-axis-aligned (Capsule.h); cylindrical part ex, two caps of radius ex/2
         body = createCapsule( uid, pos, ex/2, ex, material, true );
         break;
      case SHAPE_ELLIPSOID:
         body = createEllipsoid( uid, pos, ex, ex/2, ex/2, material, true );
         break;
      case SHAPE_CYLINDER:
         // created x-axis-aligned (Cylinder.h); NOT fixed - the queries skip fixed cylinders
         body = createCylinder( uid, pos, ex/2, 2*ex, material, true );
         break;
      default:
         throw std::invalid_argument( "createProbeShape: unknown shape" );
   }
   body->setLinearVel( 0, 0, 0 );
   body->setAngularVel( 0, 0, 0 );

   // The construction stands on the x-extent of the REAL AABB, padding included. Check it
   // rather than trust the axis convention of each shape.
   const real ext      = body->getAABB()[3] - body->getAABB()[0];
   const real expected = 2*ex + 2*contactThreshold;
   if( std::fabs( ext - expected ) > real(1e-9) ) {
      std::ostringstream msg;
      msg << "createProbeShape(" << shapeName( shape ) << "): AABB x-extent " << ext
          << " differs from the expected 2*ex + 2*contactThreshold = " << expected
          << " - this shape is not x-aligned or not padded the way the construction assumes";
      throw std::runtime_error( msg.str() );
   }
   return body;
}

void createGateProbeBodies( MaterialID material, int& spacerCount, int& probeCount ) {
   Construction& gc = construction();
   gc.ct      = contactThreshold;
   gc.rSpacer = real(0.06);
   gc.delta   = real(1e-5);
   gc.span    = ( 2*gc.rSpacer + 2*gc.ct ) * std::sqrt( hierarchyFactor );
   gc.ex      = gc.span/2 - gc.ct - gc.delta;
   gc.planes.clear();
   for( int k = 1; k*gc.span < real(1); ++k ) gc.planes.push_back( k*gc.span );

   // Spacers FIRST (see the header): they set the span and trip the activation threshold.
   const int nSpacers = 40;
   for( int i = 0; i < nSpacers; ++i ) {
      SphereID s = createSphere( 9000 + i, Vec3( 2 + real(0.2)*i, 2, 2 ), gc.rSpacer, material, true );
      s->setLinearVel( 0, 0, 0 );
      s->setAngularVel( 0, 0, 0 );
      gc.spacers.push_back( s );
      ++spacerCount;
   }

   // Probes, parked outside the mesh until a stage places them.
   const int nProbes = 30;
   for( int j = 0; j < nProbes; ++j ) {
      Probe p;
      p.shape = j % NUM_SHAPES;
      p.home  = Vec3( 3 + real(0.3)*j, 3, 3 );
      p.body  = createProbeShape( 100 + j, p.shape, p.home, gc.ex, material );
      gc.probes.push_back( p );
      ++probeCount;
   }
   gc.built = true;
}

namespace {

bool isRepresentativeRank() {
   return SimulationConfig::getInstance().getCfdRank() == 1;
}

// Move a body so that its AABB minimum along x lands on targetMinX, with the centre at (yc, zc).
// The correction uses the body's REAL AABB minimum, read after creation (and after any rotation)
// - the Layer-1 lesson: assuming a half-extent mis-places every shape whose AABB is not 2*ex.
void placeByAabbMinX( BodyID body, real targetMinX, real yc, real zc ) {
   body->setPosition( body->getPosition()[0], yc, zc );
   const real aabbMinX = body->getAABB()[0];
   const Vec3 q = body->getPosition();
   body->setPosition( q[0] + ( targetMinX - aabbMinX ), q[1], q[2] );
}

void park( const Probe& p ) {
   p.body->setPosition( p.home );
}

} // anonymous namespace
} // namespace fbmgate

using namespace fbmgate;

extern "C" int fbm_gate_grid_active_() {
   return theCollisionSystem()->getCoarseDetector().isGridActive() ? 1 : 0;
}

extern "C" int fbm_gate_body_count_() {
   return static_cast<int>( theCollisionSystem()->getBodyStorage().size() );
}

extern "C" int fbm_gate_grid_spans_( double* spans, const int* nmax ) {
   std::vector<real> s;
   theCollisionSystem()->getCoarseDetector().getCellSpans( s );
   const int n = static_cast<int>( s.size() );
   for( int i = 0; i < n && i < *nmax; ++i ) spans[i] = s[i];

   // The tripwire. The whole construction assumes the first grid's span is the one the spacers
   // set. If pe's hierarchyFactor, its AABB padding or the activation re-filing order ever
   // change, this is where the harness must fail rather than silently test nothing.
   const Construction& gc = construction();
   if( gc.built ) {
      if( n == 0 ) {
         std::fprintf( stderr, "fbm_gate_grid_spans_: no grid exists - the activation threshold "
                               "has not tripped (body count %d)\n", fbm_gate_body_count_() );
         return -1;
      }
      if( std::fabs( s[0] - gc.span ) > real(1e-12) ) {
         std::fprintf( stderr, "fbm_gate_grid_spans_: first grid span %.17g differs from the "
                               "computed (2*r_spacer + 2*ct)*sqrt(hierarchyFactor) = %.17g\n",
                       double(s[0]), double(gc.span) );
         return -1;
      }
   }
   return n;
}

extern "C" void fbm_gate_place_stage_( const int* stage, const int* shapeFilter, int* nPlaced ) {
   Construction& gc = construction();
   *nPlaced = 0;
   if( !gc.built ) throw std::logic_error( "fbm_gate_place_stage_: construction not built" );

   const int    nPlanes = static_cast<int>( gc.planes.size() );   // 5 for r_spacer = 0.06
   // Both rows on Q2 DOF planes of the level-2 mesh (y = m/36), so a DOF lies on every
   // probe's axis: 0.25 = 9/36, 0.75 = 27/36.
   const real   yRow[2] = { real(0.25), real(0.75) };
   const real   zRow[3] = { real(1)/6, real(1)/2, real(5)/6 };     // one per rank slab
   const real   zCut[2] = { real(1)/3, real(2)/3 };                // the subdomain planes

   for( size_t j = 0; j < gc.probes.size(); ++j ) {
      const Probe& p = gc.probes[j];
      const bool inPlay = ( *shapeFilter < 0 || *shapeFilter == p.shape );
      if( !inPlay || *stage == STAGE_PARK ) { park( p ); continue; }

      // The lattice plane must NOT alias the shape. Probe j has shape j % NUM_SHAPES, and
      // NUM_SHAPES == nPlanes == 5, so k = j % nPlanes would pin every shape to one plane -
      // and the plane k = 4 (5 span = 0.85) has its far boundary 6 span = 1.02 outside the
      // domain, where no DOF can be missed. The first run showed exactly that: every shape
      // differed except the cylinder, which happened to own plane 4. Adding the row index
      // j / nPlanes (0..5) walks each shape over every plane while (k, iy, iz) stays a
      // bijection onto the 5 x 2 x 3 slots: for a fixed row the shape index still runs 0..4.
      const int jj = static_cast<int>( j );
      const int k  = ( jj % nPlanes + jj / nPlanes ) % nPlanes;
      switch( *stage ) {
         case STAGE_LATTICE: {
            const int iy = ( static_cast<int>( j ) / nPlanes ) % 2;
            const int iz = ( static_cast<int>( j ) / nPlanes ) / 2;
            if( iz > 2 ) { park( p ); continue; }
            placeByAabbMinX( p.body, gc.planes[k] - gc.delta, yRow[iy], zRow[iz] );
            ++*nPlaced;
            break;
         }
         case STAGE_OVERLAP: {
            // pairs (2m, 2m+1): the first sits delta below plane k, the second is shifted by
            // 4/3 ex so the two overlap over one third of a diameter across the plane
            const int m  = static_cast<int>( j ) / 2;
            const int kk = m % nPlanes;
            const int iz = ( m / nPlanes ) % 3;
            const real minX = gc.planes[kk] - gc.delta + ( ( j % 2 ) ? real(4)/3 * gc.ex : real(0) );
            placeByAabbMinX( p.body, minX, yRow[0], zRow[iz] );
            ++*nPlaced;
            break;
         }
         case STAGE_PARTITION: {
            const int iy = ( static_cast<int>( j ) / nPlanes ) % 2;
            const int ic = ( static_cast<int>( j ) / nPlanes ) / 2;
            if( ic > 1 ) { park( p ); continue; }
            placeByAabbMinX( p.body, gc.planes[k] - gc.delta, yRow[iy], zCut[ic] );
            ++*nPlaced;
            break;
         }
         default:
            throw std::invalid_argument( "fbm_gate_place_stage_: unknown stage" );
      }
   }
}

extern "C" void fbm_gate_nudge_( const double* d ) {
   // Deliberately no simulationStep() and no findContacts(): the index must stay as it is until
   // the driver decides whether to refresh it. That staleness IS the thing under test.
   Construction& gc = construction();
   const Vec3 dv( d[0], d[1], d[2] );
   for( size_t j = 0; j < gc.probes.size(); ++j ) {
      BodyID b = gc.probes[j].body;
      b->setPosition( b->getPosition() + dv );
   }
}

extern "C" void fbm_gate_probe_report_() {
   if( !isRepresentativeRank() ) return;
   const Construction& gc = construction();
   std::printf( "FBM_GATE_CONSTRUCTION span= %.10g ex= %.10g delta= %.3g ct= %.3g planes=",
                double(gc.span), double(gc.ex), double(gc.delta), double(gc.ct) );
   for( size_t k = 0; k < gc.planes.size(); ++k ) std::printf( " %.10g", double(gc.planes[k]) );
   std::printf( "\n" );
   for( size_t j = 0; j < gc.probes.size(); ++j ) {
      const Probe& p = gc.probes[j];
      const Vec3 q = p.body->getPosition();
      const real minX = p.body->getAABB()[0], maxX = p.body->getAABB()[3];
      std::printf( "FBM_GATE_PROBE uid= %llu sid= %llu shape= %-9s pos= %.8g %.8g %.8g "
                   "aabb_x= %.10g %.10g cell_x= %ld\n",
                   (unsigned long long) p.body->getID(), (unsigned long long) p.body->getSystemID(),
                   shapeName( p.shape ), double(q[0]), double(q[1]), double(q[2]),
                   double(minX), double(maxX), long( std::floor( minX / gc.span ) ) );
   }
}
