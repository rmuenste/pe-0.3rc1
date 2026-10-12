//=================================================================================================
/*!
 * \file pe/interface/fbm_gate_probe.h
 * \brief Body construction and C entry points for the Layer-2 FBM classification gate.
 *
 * The gate (FeatFloWer application q2p1_hashgrid_test, built with ENABLE_FBM_QUERY_GATE) asks
 * whether pe's accelerated point query agrees with the exhaustive one on EVERY Q2 DOF of a real
 * partitioned mesh. Everything here exists so that the FeatFloWer side can stay thin: the
 * geometry the test depends on - the HashGrid cell span, pe's contact threshold, each body's
 * real AABB after creation - is knowable only inside pe.
 *
 * THE CONSTRUCTION THE GATE DEPENDS ON
 *
 * A body is filed in the grid whose cell span is sqrt(hierarchyFactor) = sqrt(2) times its own
 * AABB (HashGrids.h, addGrid). A monodisperse population therefore fills only 70.7 % of its
 * cell, straddles at most two adjacent cells, and the 27-cell stencil of getBodiesNearPoint()
 * always covers the cell it was filed in: no stale-index miss is geometrically possible. The
 * miss needs a body whose AABB fills a WHOLE span, which only happens when SMALLER bodies set
 * that span. Hence two populations:
 *
 *   spacers - small spheres, created FIRST, parked outside the mesh. They set the span and trip
 *             pe's 32-body activation threshold. Creation order is load-bearing: when the
 *             threshold trips, only the first 32 pending bodies are re-filed, and the first of
 *             them creates the grid.
 *   probes  - one body per shape, sized so its AABB is span - 2*delta along x, placed with its
 *             AABB minimum delta below a cell-boundary plane. A nudge of more than delta without
 *             re-filing is the defect; the same nudge followed by refreshSpatialIndex() is the
 *             verdict.
 *
 * Probes are sized by their x HALF-EXTENT, never by a bounding radius: a cylinder inscribed in a
 * bounding sphere has an x-extent of only sqrt(3)/2 of the sphere's diameter and would not fill
 * the span - the exact artefact that made the Layer-1 cylinder look immune. createProbeShape()
 * asserts the real AABB x-extent of every body it makes, so an axis convention it did not expect
 * fails loudly instead of testing nothing.
 *
 * No probe is ever fixed: both point queries skip fixed cylinders and fixed triangle meshes.
 */
//=================================================================================================
#ifndef _PE_INTERFACE_FBM_GATE_PROBE_H_
#define _PE_INTERFACE_FBM_GATE_PROBE_H_

#include <pe/core.h>
#include <vector>

namespace fbmgate {

enum Shape { SHAPE_SPHERE = 0, SHAPE_BOX, SHAPE_CAPSULE, SHAPE_ELLIPSOID, SHAPE_CYLINDER, NUM_SHAPES };

//! Stage codes shared with the Fortran driver (hashgrid_gate_driver.f90).
enum Stage {
   STAGE_PARK      = 0,  //!< every probe back to its parked position outside the mesh
   STAGE_LATTICE   = 1,  //!< probes on the 5 x 2 x 3 lattice, AABB minima delta below the planes
   STAGE_OVERLAP   = 2,  //!< probe pairs overlapping by a third of a diameter across a plane
   STAGE_PARTITION = 3   //!< probes centred on the z = 1/3 and z = 2/3 subdomain cuts
};

struct Probe {
   pe::BodyID body;
   int        shape;
   pe::Vec3   home;      //!< parked position, outside [0,1]^3
};

struct Construction {
   pe::real rSpacer = 0, ct = 0, delta = 0, span = 0, ex = 0;
   std::vector<pe::BodyID> spacers;
   std::vector<Probe>      probes;
   std::vector<pe::real>   planes;   //!< k * span for every k with k * span < 1
   bool built = false;
};

Construction& construction();

//! One non-fixed body of the given shape whose AABB x-extent is exactly 2*ex (+ pe's padding).
pe::BodyID createProbeShape( pe::id_t uid, int shape, const pe::Vec3& pos, pe::real ex,
                             pe::MaterialID material );

//! Spacers first, then parked probes. Returns counts through the two references.
void createGateProbeBodies( pe::MaterialID material, int& spacerCount, int& probeCount );

const char* shapeName( int shape );

} // namespace fbmgate

// C entry points for the Fortran driver (source/src_particles/dem_query.f90).
extern "C" {
   int  fbm_gate_grid_active_();                               //!< 1 if pe's grid is active on this rank
   int  fbm_gate_body_count_();                                //!< bodies in this rank's storage
   int  fbm_gate_grid_spans_( double* spans, const int* nmax ); //!< cell spans; -1 if the first one is not the computed span
   void fbm_gate_place_stage_( const int* stage, const int* shapeFilter, int* nPlaced );
   void fbm_gate_nudge_( const double* d );                   //!< translate every probe, no re-filing
   void fbm_gate_probe_report_();                             //!< one line per probe, rank 1 only
}

#endif
