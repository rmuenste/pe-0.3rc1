//=================================================================================================
/*!
 *  \file tests/interface/pe_coupling_detection_test.cpp
 *  \brief Compile-level test of the solver-feature detection of pe::coupling
 *
 *  Copyright (C) 2026 Raphael Münster
 *
 *  This file is part of pe.
 *
 *  pe is free software: you can redistribute it and/or modify it under the terms of the GNU
 *  General Public License as published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 *
 *  pe is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 *  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  General Public License for more details.
 *
 *  The coupling implementation calls every solver-dependent member through a dispatch template
 *  (src/interface/coupling/SolverFeatures.h, src/interface/PointOwnership.h), so a collision
 *  system or coarse detector that lacks the member still compiles. That only holds if the
 *  dispatch really is a template on the type: an `if constexpr` inside a non-template function
 *  would still check the member call. This translation unit instantiates every dispatch
 *  function with stand-in types that have none of the members -- it would not compile if a
 *  dispatch were not discarded -- and checks the answers at compile time and at run time:
 *    - Bare: no member at all: every detector false, every try*() returns false, gravity and
 *      buoyancy BodyForce::unknown (not "applied by the driver"), the point query throws;
 *    - DeclaresFalse: declares appliesGravity / appliesBuoyancy = false: appliedByDriver;
 *    - Full: has every member (recording the calls): every detector true, the calls arrive.
 */
//=================================================================================================

#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include <src/interface/PointOwnership.h>
#include <src/interface/coupling/SolverFeatures.h>

#include "pe_coupling_test_util.h"

using coupling_test::expect;
using coupling_test::expectThrows;
namespace pd = pe::coupling::detail;
namespace pid = pe::interface::detail;


//*************************************************************************************************
/*!\brief A collision system / coarse detector stand-in without any of the members.
 */
struct Bare
{};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief A stand-in that declares its body-force semantics as "not applied by the solver".
 */
struct DeclaresFalse
{
   static constexpr bool appliesGravity  = false;
   static constexpr bool appliesBuoyancy = false;
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief A stand-in with every member, recording what it receives.
 */
struct Full
{
   static constexpr bool appliesGravity  = true;
   static constexpr bool appliesBuoyancy = true;

   void setErrorReductionParameter( pe::real erp ) { erp_ = erp; }
   void setMaxIterations( std::size_t n ) { iterations_ = n; }
   void setRelaxationParameter( pe::real f ) { relaxation_ = f; }
   void setSplitImpulse( bool b ) { split_ = b; }
   bool getSplitImpulse() const { return split_; }
   bool isSyncRequired() const { return true; }
   void setPredictionDiagnostics( bool b ) { diagnostics_ = b; }
   bool getPredictionDiagnostics() const { return diagnostics_; }
   void refreshSpatialIndex() { ++refreshes_; }
   bool isGridActive() const { return true; }
   void getCellSpans( std::vector<pe::real>& spans ) const { spans.assign( 1, pe::real( 0.5 ) ); }
   template< typename Container >
   void getBodiesNearPoint( pe::real, pe::real, pe::real, Container& bodies ) const
   { bodies.clear(); ++queries_; }

   pe::real    erp_ = 0, relaxation_ = 0;
   std::size_t iterations_ = 0;
   bool        split_ = false, diagnostics_ = true;
   int         refreshes_ = 0;
   mutable int queries_ = 0;
};
//*************************************************************************************************


// Compile-time answers
static_assert( !pd::HasErrorReduction<Bare>::value && !pd::HasMaxIterations<Bare>::value &&
               !pd::HasContactRelaxation<Bare>::value && !pd::HasSplitImpulse<Bare>::value &&
               !pd::HasSyncQuery<Bare>::value && !pd::HasPredictionDiagnostics<Bare>::value &&
               !pd::HasSpatialIndexRefresh<Bare>::value && !pd::HasGridActivity<Bare>::value &&
               !pd::HasCellSpans<Bare>::value && !pd::DeclaresGravity<Bare>::value &&
               !pd::DeclaresBuoyancy<Bare>::value && !pid::HasPointQuery<Bare>::value &&
               !pid::HasShadowStorage<Bare>::value, "Bare has no member" );
static_assert( pd::HasErrorReduction<Full>::value && pd::HasMaxIterations<Full>::value &&
               pd::HasContactRelaxation<Full>::value && pd::HasSplitImpulse<Full>::value &&
               pd::HasSyncQuery<Full>::value && pd::HasPredictionDiagnostics<Full>::value &&
               pd::HasSpatialIndexRefresh<Full>::value && pd::HasGridActivity<Full>::value &&
               pd::HasCellSpans<Full>::value && pid::HasPointQuery<Full>::value,
               "Full has every member" );
static_assert( pd::DeclaresGravity<DeclaresFalse>::value, "a declared false is detected" );


int main()
{
   // Bare: every dispatch compiles and reports "absent"
   {
      Bare bare;
      bool flag( false );
      expect( !pd::trySetErrorReduction( bare, 0.5 ), "Bare: no error reduction" );
      expect( !pd::trySetMaxIterations( bare, 5 ), "Bare: no iteration limit" );
      expect( !pd::trySetContactRelaxation( bare, 0.5 ), "Bare: no contact relaxation" );
      expect( !pd::trySetSplitImpulse( bare, true ), "Bare: no split impulse" );
      expect( !pd::tryGetSplitImpulse( bare, flag ), "Bare: no split impulse query" );
      expect( !pd::syncRequired( bare ), "Bare: no synchronization" );
      expect( !pd::trySetPredictionDiagnostics( bare, false, flag ), "Bare: no diagnostics" );
      pd::refreshSpatialIndex( bare );
      expect( !pd::gridActive( bare ), "Bare: no grid" );
      expect( pd::pointQueryBound( bare ) == std::numeric_limits<pe::real>::infinity(),
              "Bare: no hashing bound" );
      expect( pd::gravitySemantics<Bare>() == pe::coupling::BodyForce::unknown,
              "Bare: gravity unknown, not 'applied by the driver'" );
      expect( pd::buoyancySemantics<Bare>() == pe::coupling::BodyForce::unknown,
              "Bare: buoyancy unknown" );
      for( const pe::coupling::Setting s : { pe::coupling::Setting::gravity,
                                             pe::coupling::Setting::fluidDensity,
                                             pe::coupling::Setting::errorReduction,
                                             pe::coupling::Setting::maxIterations,
                                             pe::coupling::Setting::contactRelaxation,
                                             pe::coupling::Setting::splitImpulse } )
         expect( !pd::settingSupportedBy<Bare>( s ), "Bare: setting unsupported" );
      expect( pd::settingSupportedBy<Bare>( pe::coupling::Setting::fluidViscosity ),
              "Bare: viscosity always supported" );

      std::vector<pe::BodyID> candidates;
      const double pos[3] = { 0.0, 0.0, 0.0 };
      expectThrows<std::runtime_error>( [&]() { pid::bodiesNearPoint( bare, pos, candidates ); },
                                        "Bare: the point query throws", "no point query" );
      int visited( 0 );
      pid::forEachShadowCopy( bare, [&]( pe::BodyID ) { ++visited; } );
      expect( visited == 0 && pid::findShadowCopy( bare, 0 ) == nullptr, "Bare: no shadows" );
   }

   // The refusal text claims driver ownership only for a declared false
   {
      const std::string unknown( pd::bodyForceReason( pe::coupling::BodyForce::unknown,
                                                      "gravity" ) );
      const std::string driver( pd::bodyForceReason( pe::coupling::BodyForce::appliedByDriver,
                                                     "gravity" ) );
      expect( unknown.find( "driver" ) == std::string::npos &&
              unknown.find( "does not declare" ) != std::string::npos,
              "unknown semantics: the message does not claim the driver owns the force" );
      expect( driver.find( "driver owns" ) != std::string::npos,
              "declared false: the message says the driver owns the force" );
   }

   // DeclaresFalse: driver-owned, and the settings are refused
   expect( pd::gravitySemantics<DeclaresFalse>() == pe::coupling::BodyForce::appliedByDriver,
           "DeclaresFalse: gravity applied by the driver" );
   expect( !pd::settingSupportedBy<DeclaresFalse>( pe::coupling::Setting::gravity ),
           "DeclaresFalse: gravity setting refused" );

   // Full: the calls arrive
   {
      Full full;
      bool flag( false );
      expect( pd::trySetErrorReduction( full, 0.25 ) && full.erp_ == pe::real( 0.25 ),
              "Full: error reduction set" );
      expect( pd::trySetMaxIterations( full, 7 ) && full.iterations_ == 7, "Full: iterations" );
      expect( pd::trySetContactRelaxation( full, 0.75 ) && full.relaxation_ == pe::real( 0.75 ),
              "Full: relaxation" );
      expect( pd::trySetSplitImpulse( full, true ) && pd::tryGetSplitImpulse( full, flag ) &&
              flag, "Full: split impulse" );
      expect( pd::syncRequired( full ), "Full: synchronization query" );
      expect( pd::trySetPredictionDiagnostics( full, false, flag ) && flag &&
              !full.diagnostics_, "Full: diagnostics switched, previous state returned" );
      pd::refreshSpatialIndex( full );
      expect( full.refreshes_ == 1 && pd::gridActive( full ), "Full: refresh and grid" );
      expect( pd::pointQueryBound( full ) ==
              std::ldexp( pe::real( 0.5 ), std::numeric_limits<std::size_t>::digits - 2 ),
              "Full: bound from the smallest cell span" );
      expect( pd::gravitySemantics<Full>() == pe::coupling::BodyForce::appliedBySolver,
              "Full: gravity applied by the solver" );
      std::vector<pe::BodyID> candidates;
      const double pos[3] = { 0.0, 0.0, 0.0 };
      pid::bodiesNearPoint( full, pos, candidates );
      expect( full.queries_ == 1, "Full: point query used" );
   }

   return coupling_test::report( "pe_coupling_detection_test" );
}
