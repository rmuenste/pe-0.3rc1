//=================================================================================================
/*!
 *  \file pe/core/response/ShortRangeRepulsion.h
 *  \brief Short-range repulsion solver (Pan et al. 2002, Eq. 2.1)
 *
 *  Implements a soft pre-contact repulsive force that activates within a security zone of
 *  width ρ ahead of actual particle–particle or particle–wall contact:
 *
 *    F = (ρ - d) / ρ)² / ε_p   (particle–particle, when d ≤ ρ)
 *    F = (ρ - d) / ρ)² / ε_w   (particle–wall, when d ≤ ρ)
 *
 *  where d is the gap between surfaces (positive = separated) and ρ is the security-zone
 *  half-width.  Reference: T.-W. Pan, D.D. Joseph, R. Bai, R. Glowinski, V. Sarin,
 *  "Fluidization of 1204 spheres", J. Fluid Mech. 451 (2002) 169–191, Eq. 2.1.
 *
 *  Paper parameters (§4):  ε_p = ε_w = 5×10⁻⁷,  ρ = h_v = 0.06858 cm.
 */
//=================================================================================================

#ifndef _PE_CORE_RESPONSE_SHORTRANGEREPULSION_H_
#define _PE_CORE_RESPONSE_SHORTRANGEREPULSION_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/domaindecomp/Domain.h>
#include <pe/core/MPISection.h>
#include <pe/core/MPISettings.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <pe/util/constraints/SameType.h>
#include <pe/util/logging/DebugSection.h>
#include <pe/util/NonCopyable.h>
#include <pe/util/NullType.h>


namespace pe {

namespace response {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Short-range repulsion solver (Pan et al. 2002, Eq. 2.1).
 * \ingroup collision_response
 *
 * Force-based solver that applies a soft repulsive force within a security zone ρ to prevent
 * particle–particle and particle–wall overlap without hard contact resolution.
 */
template< typename C              // Type of the configuration
        , typename U1=NullType    // First unused auxiliary template parameter
        , typename U2=NullType >  // Second unused auxiliary template parameter
class ShortRangeRepulsion : private NonCopyable
{
public:
   //**Type definitions****************************************************************************
   typedef C                                      Config;          //!< Type of the configuration.
   typedef ShortRangeRepulsion<C,U1,U2>           This;            //!< Type of this instance.
   typedef typename Config::BodyType              BodyType;        //!< Type of the rigid bodies.
   typedef typename Config::BodyID                BodyID;          //!< Handle for a rigid body.
   typedef typename Config::ConstBodyID           ConstBodyID;     //!< Handle for a constant rigid body.
   typedef typename Config::ContactType           ContactType;     //!< Type of the contacts.
   typedef typename Config::ContactID             ContactID;       //!< Handle for a contact.
   typedef typename Config::ConstContactID        ConstContactID;  //!< Handle for a constant contact.
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit ShortRangeRepulsion( const Domain& domain,
                                 real eps_p = real(5e-7),
                                 real eps_w = real(5e-7),
                                 real rho   = real(6.858e-4),
                                 real gamma = real(0) );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   ~ShortRangeRepulsion();
   //@}
   //**********************************************************************************************

   //**Solver functions****************************************************************************
   /*!\name Solver functions */
   //@{
   inline real resolveContact( ContactID c ) const;
   //@}
   //**********************************************************************************************

   //**Configuration setters***********************************************************************
   /*!\name Configuration setters/getters */
   //@{
   inline void setEpsP ( real v ) { eps_p_ = v; }
   inline void setEpsW ( real v ) { eps_w_ = v; }
   inline void setRho  ( real v ) { rho_   = v; }
   inline void setGamma( real v ) { gamma_ = v; }
   inline real getEpsP () const   { return eps_p_; }
   inline real getEpsW () const   { return eps_w_; }
   inline real getRho  () const   { return rho_;   }
   inline real getGamma() const   { return gamma_; }
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   const Domain& domain_;  //!< The local process domain.
   real eps_p_;            //!< Particle–particle stiffness parameter ε_p.
   real eps_w_;            //!< Particle–wall stiffness parameter ε_w.
   real rho_;              //!< Security zone width ρ.
   real gamma_;            //!< Normal velocity damping coefficient (0 = disabled).
   //@}
   //**********************************************************************************************

   //**Compile time checks*************************************************************************
   /*! \cond PE_INTERNAL */
   pe_CONSTRAINT_MUST_BE_SAME_TYPE( U1, NullType );
   pe_CONSTRAINT_MUST_BE_SAME_TYPE( U2, NullType );
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for ShortRangeRepulsion.
 *
 * \param domain   The local process domain.
 * \param eps_p    Particle–particle stiffness (default: 5×10⁻⁷).
 * \param eps_w    Particle–wall stiffness (default: 5×10⁻⁷).
 * \param rho      Security zone width (default: 6.858×10⁻⁴, i.e. 0.06858 cm).
 * \param gamma    Normal velocity damping coefficient (default: 0 = disabled).
 */
template< typename C, typename U1, typename U2 >
ShortRangeRepulsion<C,U1,U2>::ShortRangeRepulsion( const Domain& domain,
                                                   real eps_p, real eps_w,
                                                   real rho, real gamma )
   : domain_( domain )
   , eps_p_ ( eps_p )
   , eps_w_ ( eps_w )
   , rho_   ( rho   )
   , gamma_ ( gamma )
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for ShortRangeRepulsion.
 */
template< typename C, typename U1, typename U2 >
ShortRangeRepulsion<C,U1,U2>::~ShortRangeRepulsion()
{}
//*************************************************************************************************




//=================================================================================================
//
//  SOLVER FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Resolves the given contact by applying a short-range repulsive force.
 *
 * \param c The contact to resolve (may be pre-contact with positive gap up to ρ).
 * \return The "overlap" (ρ − gap) used for the force, or 0 if outside the security zone.
 *
 * Applies the Pan et al. (2002) Eq. 2.1 repulsive force:
 *
 *   F = (overlap/ρ)² / ε   where  overlap = ρ − gap
 *
 * The force is applied only when gap ≤ ρ.  The normal is oriented body2 → body1.
 * An optional damping term (γ) reduces the force when bodies are approaching.
 */
template< typename C, typename U1, typename U2 >
inline real ShortRangeRepulsion<C,U1,U2>::resolveContact( ContactID c ) const
{
   // gap > ρ: outside security zone, no force
   const real gap = c->getDistance();
   if( gap > rho_ ) {
      pe_LOG_DEBUG_SECTION( log ) {
         log << "Rejecting contact outside security zone (gap=" << gap << " > rho=" << rho_ << ").\n";
      }
      return real(0);
   }

   BodyID b1( c->getBody1() );
   BodyID b2( c->getBody2() );

   pe_INTERNAL_ASSERT( !b1->isFixed() || !b2->isFixed(),
                       "Invalid contact between two fixed objects." );

   /* Possible contact types
    *
    * L: Local body
    * G: Global body
    * R: Remote body
    * +---+---+---+---+
    * |   | L | G | R |
    * +---+---+---+---+
    * | L | + | + | * |
    * +---+---+---+---+
    * | G | + | ~ | - |
    * +---+---+---+---+
    * | R | * | - | # |
    * +---+---+---+---+
    *
    *  + Accept contact unconditionally
    *  - Reject contact unconditionally
    *  * Accept contact if we own the contact point
    *  # Accept contact if we own the contact point and the owners of the involved bodies are not the same
    *  ~ Accept contact only on root process
    */

   if( !b1->isRemote() && !b2->isRemote() ) {
      // local-local, local-global, global-global contacts
      if( b1->isGlobal() && b2->isGlobal() ) {
         // Resolve global-global contacts only on root process
         if( MPISettings::rank() != MPISettings::root() ) {
            pe_LOG_DEBUG_SECTION( log ) {
               log << "Rejecting global-global contact " << c << " on non-root process.\n";
            }
            return real(0);
         }
      }
   }
   else {
      // local-remote, global-remote or remote-remote contact
      if( b1->isGlobal() || b2->isGlobal() ) {
         pe_LOG_DEBUG_SECTION( log ) {
            log << "Rejecting global-remote contact " << c << ".\n";
         }
         return real(0);
      }
      else if( b1->isRemote() && b2->isRemote() ) {
         if( b1->getOwnerRank() == b2->getOwnerRank() ) {
            pe_LOG_DEBUG_SECTION( log ) {
               log << "Rejecting remote-remote contact (same owner): " << c << ".\n";
            }
            return real(0);
         }
         else if( !domain_.ownsPoint( c->getPosition() ) ) {
            pe_LOG_DEBUG_SECTION( log ) {
               log << "Rejecting remote-remote contact (not owned): " << c << ".\n";
            }
            return real(0);
         }
      }
      else {
         if( !domain_.ownsPoint( c->getPosition() ) ) {
            pe_LOG_DEBUG_SECTION( log ) {
               log << "Rejecting remote-local contact (not owned): " << c << ".\n";
            }
            return real(0);
         }
      }
   }

   // overlap = ρ - gap  (> 0 inside security zone, > ρ if penetrating)
   const real overlap = rho_ - gap;

   // Choose stiffness: wall (fixed body) or particle–particle
   // b2->isFixed() indicates a wall (plane or fixed boundary)
   const real eps = b2->isFixed() ? eps_w_ : eps_p_;

   // Pan et al. (2002) Eq. 2.1: F = (overlap/ρ)² / ε
   real fmag = ( overlap * overlap ) / ( eps * rho_ * rho_ );

   // Optional normal velocity damping (standard spring-dashpot model):
   //   approaching (relVelN > 0): extra upward force resists approach → add to fmag
   //   receding   (relVelN < 0): less upward force resists separation → subtract from fmag
   // Clamp to zero to prevent attractive (inward) forces.
   if( gamma_ > real(0) ) {
      // relVelN > 0 when bodies are approaching (PE sign convention: getNormalRelVel() < 0)
      const real relVelN = -c->getNormalRelVel();
      fmag += gamma_ * relVelN;
      if( fmag < real(0) ) fmag = real(0);
   }

   // Global position of contact point
   const Vec3 gpos( c->getPosition() );

   // Normal runs body2 → body1; apply equal-and-opposite forces
   const Vec3 fN( fmag * c->getNormal() );

   b1->addForceAtPos(  fN, gpos );
   b2->addForceAtPos( -fN, gpos );

   return overlap;
}
//*************************************************************************************************

} // namespace response

} // namespace pe

#endif
