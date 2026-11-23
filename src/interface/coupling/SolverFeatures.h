//=================================================================================================
/*!
 *  \file src/interface/coupling/SolverFeatures.h
 *  \brief Compile-time detection of the solver-dependent features used by pe::coupling
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
 *  You should have received a copy of the GNU General Public License along with pe. If not,
 *  see <http://www.gnu.org/licenses/>.
 *
 *  Internal header of the coupling implementation. The coupling API is compiled once, against
 *  whatever pe_CONSTRAINT_SOLVER selects, so every solver-dependent call goes through a
 *  std::void_t detector and an `if constexpr` dispatch, the pattern of
 *  pe/interface/support/el_optional_api.h:
 *
 *  - member functions (contact parameters, split impulses, the hash-grid queries) are detected
 *    structurally;
 *  - the body-force semantics cannot be detected from the type, so a collision system
 *    declares them explicitly (static constexpr bool appliesGravity / appliesBuoyancy, like
 *    hasLubricationStage); a system without the declaration is reported as BodyForce::unknown.
 *
 *  Every dispatch function is a template on the collision-system or detector type: inside a
 *  template the discarded branch of `if constexpr` is not instantiated, so a type that lacks a
 *  member really compiles (tests/interface/pe_coupling_detection_test.cpp checks that with
 *  stand-in types).
 */
//=================================================================================================

#ifndef _PE_SRC_INTERFACE_COUPLING_SOLVERFEATURES_H_
#define _PE_SRC_INTERFACE_COUPLING_SOLVERFEATURES_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>
#include <pe/core.h>
#include <pe/interface/coupling/Types.h>


namespace pe {

namespace coupling {

namespace detail {

//=================================================================================================
//
//  TYPE DEFINITIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The collision system of the configured pe world.
 */
typedef CollisionSystem<Config>  ActiveCollisionSystem;
//*************************************************************************************************




//=================================================================================================
//
//  DETECTION TRAITS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Whether the collision system declares its gravity semantics (appliesGravity).
 */
template< typename CS, typename = void >
struct DeclaresGravity : std::false_type
{};

template< typename CS >
struct DeclaresGravity< CS, std::void_t< decltype( CS::appliesGravity ) > > : std::true_type
{};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Whether the collision system declares its buoyancy semantics (appliesBuoyancy).
 */
template< typename CS, typename = void >
struct DeclaresBuoyancy : std::false_type
{};

template< typename CS >
struct DeclaresBuoyancy< CS, std::void_t< decltype( CS::appliesBuoyancy ) > > : std::true_type
{};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Whether the collision system has setErrorReductionParameter( real ).
 */
template< typename CS, typename = void >
struct HasErrorReduction : std::false_type
{};

template< typename CS >
struct HasErrorReduction< CS, std::void_t<
   decltype( std::declval<CS&>().setErrorReductionParameter( real() ) ) > > : std::true_type
{};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Whether the collision system has setMaxIterations( size_t ).
 */
template< typename CS, typename = void >
struct HasMaxIterations : std::false_type
{};

template< typename CS >
struct HasMaxIterations< CS, std::void_t<
   decltype( std::declval<CS&>().setMaxIterations( std::size_t() ) ) > > : std::true_type
{};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Whether the collision system has setRelaxationParameter( real ).
 */
template< typename CS, typename = void >
struct HasContactRelaxation : std::false_type
{};

template< typename CS >
struct HasContactRelaxation< CS, std::void_t<
   decltype( std::declval<CS&>().setRelaxationParameter( real() ) ) > > : std::true_type
{};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Whether the collision system has setSplitImpulse( bool ) and getSplitImpulse().
 */
template< typename CS, typename = void >
struct HasSplitImpulse : std::false_type
{};

template< typename CS >
struct HasSplitImpulse< CS, std::void_t<
   decltype( std::declval<CS&>().setSplitImpulse( bool() ) ),
   decltype( std::declval<const CS&>().getSplitImpulse() ) > > : std::true_type
{};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Whether the collision system has the collective query isSyncRequired().
 */
template< typename CS, typename = void >
struct HasSyncQuery : std::false_type
{};

template< typename CS >
struct HasSyncQuery< CS, std::void_t<
   decltype( std::declval<const CS&>().isSyncRequired() ) > > : std::true_type
{};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Whether the collision system has the switch of its per-step diagnostic block
 *        (setPredictionDiagnostics( bool ) / getPredictionDiagnostics()).
 */
template< typename CS, typename = void >
struct HasPredictionDiagnostics : std::false_type
{};

template< typename CS >
struct HasPredictionDiagnostics< CS, std::void_t<
   decltype( std::declval<CS&>().setPredictionDiagnostics( bool() ) ),
   decltype( std::declval<const CS&>().getPredictionDiagnostics() ) > > : std::true_type
{};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Whether the coarse detector has refreshSpatialIndex().
 */
template< typename CD, typename = void >
struct HasSpatialIndexRefresh : std::false_type
{};

template< typename CD >
struct HasSpatialIndexRefresh< CD, std::void_t<
   decltype( std::declval<CD&>().refreshSpatialIndex() ) > > : std::true_type
{};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Whether the coarse detector has isGridActive().
 */
template< typename CD, typename = void >
struct HasGridActivity : std::false_type
{};

template< typename CD >
struct HasGridActivity< CD, std::void_t<
   decltype( std::declval<const CD&>().isGridActive() ) > > : std::true_type
{};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Whether the coarse detector has getCellSpans( std::vector<real>& ).
 */
template< typename CD, typename = void >
struct HasCellSpans : std::false_type
{};

template< typename CD >
struct HasCellSpans< CD, std::void_t< decltype( std::declval<const CD&>().getCellSpans(
   std::declval< std::vector<real>& >() ) ) > > : std::true_type
{};
//*************************************************************************************************




//=================================================================================================
//
//  DISPATCH FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief The dispatch functions are templates on the collision-system / detector type, so the
 *        discarded `if constexpr` branch is never instantiated and a type without the member
 *        compiles (a non-template function would still check the member call).
 */
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The gravity semantics a collision system declares.
 */
template< typename CS >
inline BodyForce gravitySemantics()
{
   if constexpr( DeclaresGravity<CS>::value )
      return CS::appliesGravity ? BodyForce::appliedBySolver : BodyForce::appliedByDriver;
   else
      return BodyForce::unknown;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The buoyancy semantics a collision system declares.
 */
template< typename CS >
inline BodyForce buoyancySemantics()
{
   if constexpr( DeclaresBuoyancy<CS>::value )
      return CS::appliesBuoyancy ? BodyForce::appliedBySolver : BodyForce::appliedByDriver;
   else
      return BodyForce::unknown;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Why a body-force setting is unavailable, worded by the declared semantics.
 *
 * \param semantics The semantics of the solver (gravitySemantics() / buoyancySemantics()).
 * \param force The name of the force for the message ("gravity", "buoyancy").
 * \return The reason; it claims that the driver owns the force only if the solver says so.
 */
inline std::string bodyForceReason( BodyForce semantics, const char* force )
{
   if( semantics == BodyForce::unknown )
      return std::string( "it does not declare whether it applies " ) + force +
             ", so it is unknown which side owns it";
   return std::string( "it does not apply " ) + force + "; the driver owns that body force";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether a collision system type supports a world setting.
 *
 * \param setting The setting.
 * \return \a true if the setting is available.
 */
template< typename CS >
inline bool settingSupportedBy( Setting setting )
{
   switch( setting ) {
      case Setting::gravity:           return gravitySemantics<CS>() == BodyForce::appliedBySolver;
      case Setting::fluidDensity:      return buoyancySemantics<CS>() == BodyForce::appliedBySolver;
      case Setting::fluidViscosity:    return true;
      case Setting::errorReduction:    return HasErrorReduction<CS>::value;
      case Setting::maxIterations:     return HasMaxIterations<CS>::value;
      case Setting::contactRelaxation: return HasContactRelaxation<CS>::value;
      case Setting::splitImpulse:      return HasSplitImpulse<CS>::value;
   }
   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the configured pe supports a world setting.
 */
inline bool settingSupported( Setting setting )
{
   return settingSupportedBy<ActiveCollisionSystem>( setting );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Sets the error reduction parameter if the collision system has one.
 *
 * \return \a true if it was set.
 */
template< typename CS >
inline bool trySetErrorReduction( CS& cs, real erp )
{
   if constexpr( HasErrorReduction<CS>::value ) {
      cs.setErrorReductionParameter( erp );
      return true;
   }
   else {
      (void)cs; (void)erp;
      return false;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Sets the iteration limit if the collision system has one.
 *
 * \return \a true if it was set.
 */
template< typename CS >
inline bool trySetMaxIterations( CS& cs, std::size_t iterations )
{
   if constexpr( HasMaxIterations<CS>::value ) {
      cs.setMaxIterations( iterations );
      return true;
   }
   else {
      (void)cs; (void)iterations;
      return false;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Sets the contact relaxation parameter if the collision system has one.
 *
 * \return \a true if it was set.
 */
template< typename CS >
inline bool trySetContactRelaxation( CS& cs, real relaxation )
{
   if constexpr( HasContactRelaxation<CS>::value ) {
      cs.setRelaxationParameter( relaxation );
      return true;
   }
   else {
      (void)cs; (void)relaxation;
      return false;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Switches split impulses if the collision system has them.
 *
 * \return \a true if it was set.
 */
template< typename CS >
inline bool trySetSplitImpulse( CS& cs, bool enable )
{
   if constexpr( HasSplitImpulse<CS>::value ) {
      cs.setSplitImpulse( enable );
      return true;
   }
   else {
      (void)cs; (void)enable;
      return false;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Reads the split-impulse switch if the collision system has one.
 *
 * \param enabled Output: the switch (unchanged if there is none).
 * \return \a true if the collision system has split impulses.
 */
template< typename CS >
inline bool tryGetSplitImpulse( const CS& cs, bool& enabled )
{
   if constexpr( HasSplitImpulse<CS>::value ) {
      enabled = cs.getSplitImpulse();
      return true;
   }
   else {
      (void)cs; (void)enabled;
      return false;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The collective query whether a synchronization is required (false if absent).
 *
 * Collective in a parallel MPI run: every process must call it.
 */
template< typename CS >
inline bool syncRequired( const CS& cs )
{
   if constexpr( HasSyncQuery<CS>::value ) {
      return cs.isSyncRequired();
   }
   else {
      (void)cs;
      return false;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Switches the solver's per-step diagnostic block, if it has the switch.
 *
 * \param enable The new state.
 * \param previous Output: the state before (unchanged if there is no switch).
 * \return \a true if the collision system has the switch.
 */
template< typename CS >
inline bool trySetPredictionDiagnostics( CS& cs, bool enable, bool& previous )
{
   if constexpr( HasPredictionDiagnostics<CS>::value ) {
      previous = cs.getPredictionDiagnostics();
      cs.setPredictionDiagnostics( enable );
      return true;
   }
   else {
      (void)cs; (void)enable; (void)previous;
      return false;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Re-files the bodies of the coarse detector (no-op if it has no refresh).
 */
template< typename CD >
inline void refreshSpatialIndex( CD& cd )
{
   if constexpr( HasSpatialIndexRefresh<CD>::value )
      cd.refreshSpatialIndex();
   else
      (void)cd;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Whether the coarse detector's grid is active (false if it has no grid).
 */
template< typename CD >
inline bool gridActive( const CD& cd )
{
   if constexpr( HasGridActivity<CD>::value ) {
      return cd.isGridActive();
   }
   else {
      (void)cd;
      return false;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The largest coordinate magnitude the coarse detector's point query can hash safely.
 *
 * \return The bound, or +infinity if the detector hashes no points (inactive grid, no grid).
 *
 * The hash grid converts |x| / cellSpan to size_t; the bound keeps that below a quarter of the
 * size_t range (2^62 for a 64-bit size_t, 2^30 for a 32-bit one) for the smallest cell span of
 * the hierarchy, far from the conversion overflow.
 */
template< typename CD >
inline real pointQueryBound( const CD& cd )
{
   if constexpr( HasCellSpans<CD>::value && HasGridActivity<CD>::value ) {
      if( !cd.isGridActive() )
         return std::numeric_limits<real>::infinity();
      std::vector<real> spans;
      cd.getCellSpans( spans );
      if( spans.empty() )
         return std::numeric_limits<real>::infinity();
      const real minSpan( *std::min_element( spans.begin(), spans.end() ) );
      return std::ldexp( minSpan, std::numeric_limits<std::size_t>::digits - 2 );
   }
   else {
      (void)cd;
      return std::numeric_limits<real>::infinity();
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the name of a setting as used in error messages.
 *
 * \param setting The setting.
 * \return The name.
 */
inline const char* settingName( Setting setting )
{
   switch( setting ) {
      case Setting::gravity:           return "gravity";
      case Setting::fluidDensity:      return "fluidDensity";
      case Setting::fluidViscosity:    return "fluidViscosity";
      case Setting::errorReduction:    return "errorReduction";
      case Setting::maxIterations:     return "maxIterations";
      case Setting::contactRelaxation: return "contactRelaxation";
      case Setting::splitImpulse:      return "splitImpulse";
   }
   return "unknown";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns pe_CONSTRAINT_SOLVER as configured, e.g. "pe::response::HardContactAndFluid".
 *
 * \return The solver name.
 */
std::string constraintSolverName();
//*************************************************************************************************

} // namespace detail

} // namespace coupling

} // namespace pe

#endif
