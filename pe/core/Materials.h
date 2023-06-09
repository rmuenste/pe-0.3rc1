//=================================================================================================
/*!
 *  \file pe/core/Materials.h
 *  \brief Header file for materials
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *
 *  This file is part of pe.
 *
 *  pe is free software: you can redistribute it and/or modify it under the terms of the GNU
 *  General Public License as published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 *
 *  pe is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 *  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along with pe. If not,
 *  see <http://www.gnu.org/licenses/>.
 */
//=================================================================================================

#ifndef _PE_CORE_MATERIALS_H_
#define _PE_CORE_MATERIALS_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <string>
#include <vector>
#include <pe/core/Types.h>
#include <pe/math/MatrixMxN.h>
#include <pe/math/shims/Invert.h>
#include <pe/system/Precision.h>
#include <pe/util/Assert.h>
#include <pe/util/constraints/SameSize.h>


namespace pe {

//=================================================================================================
//
//  CLASS MATERIAL
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup materials Materials
 * \ingroup core
 */
/*!\brief Rigid body material.
 * \ingroup materials
 *
 * A material specifies the properties of a rigid body: the density of the body, the coefficient
 * of restitution and the coefficients of static and dynamic friction.\n
 * The \b pe engine provides several predefined materials that can be directly used:
 *
 * - iron
 * - copper
 * - granite
 * - oak
 * - fir
 *
 * In order to create a new custom material use the createMaterial() function:

   \code
   // Creating a new material using the following material properties:
   // - name/identifier: myMaterial
   // - density: 2.54
   // - coefficient of restitution: 0.8
   // - coefficient of static friction: 0.1
   // - coefficient of dynamic friction: 0.05
   // - Poisson's ratio: 0.2
   // - Young's modulus: 80.0
   // - Contact stiffness: 100
   // - dampingN: 10
   // - dampingT: 11
   MaterialID myMaterial = createMaterial( "myMaterial", 2.54, 0.8, 0.1, 0.05, 0.2, 80, 100, 10, 11 );
   \endcode

 * The following functions can be used to acquire a specific MaterialID or to get a specific
 * property of a material:

   \code
   // Searching a material
   MaterialID myMaterial = Material::find( "myMaterial" );

   // Getting the density, coefficient of restitution, coefficient of static and
   // dynamic friction, Poisson's ratio and Young's modulus of the material
   real density = Material::getDensity( myMaterial );
   real cor     = Material::getRestitution( myMaterial );
   real csf     = Material::getStaticFriction( myMaterial );
   real cdf     = Material::getDynamicFriction( myMaterial );
   real poisson = Material::getPoissonRatio( myMaterial );
   real young   = Material::getYoungModulus( myMaterial ):
   \endcode
 */
class PE_PUBLIC Material
{
private:
   //**Type definitions****************************************************************************
   typedef Materials::size_type  SizeType;  //!< Size type of the Material class.
   //**********************************************************************************************

public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit inline Material( const std::string& name, real density, real cor,
                             real csf, real cdf, real poisson, real young, real stiffness,
                             real dampingN, real dampingT );
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline const std::string& getName()            const;
   inline real               getDensity()         const;
   inline real               getRestitution()     const;
   inline real               getStaticFriction()  const;
   inline real               getDynamicFriction() const;
   inline real               getPoissonRatio()    const;
   inline real               getYoungModulus()    const;
   inline real               getStiffness()       const;
   inline real               getDampingN()        const;
   inline real               getDampingT()        const;

   static        MaterialID         find( const std::string& name );
   static        std::vector<MaterialID> findPrefix( const std::string& prefix );
   static inline const std::string& getName( MaterialID material );
   static inline real               getDensity( MaterialID material );
   static inline real               getRestitution( MaterialID material );
   static inline real               getRestitution( MaterialID material1, MaterialID material2 );
   static inline real               getStaticFriction( MaterialID material );
   static inline real               getStaticFriction( MaterialID material1, MaterialID material2 );
   static inline real               getDynamicFriction( MaterialID material );
   static inline real               getDynamicFriction( MaterialID material1, MaterialID material2 );
   static inline real               getPoissonRatio( MaterialID material );
   static inline real               getYoungModulus( MaterialID material );
   static inline real               getYoungModulus( MaterialID material1, MaterialID material2 );
   static inline real               getStiffness( MaterialID material );
   static inline real               getStiffness( MaterialID material1, MaterialID material2 );
   static inline real               getDampingN( MaterialID material );
   static inline real               getDampingN( MaterialID material1, MaterialID material2 );
   static inline real               getDampingT( MaterialID material );
   static inline real               getDampingT( MaterialID material1, MaterialID material2 );
   //@}
   //**********************************************************************************************

   //**Set functions*******************************************************************************
   /*!\name Set functions */
   //@{
   static inline void setRestitution( MaterialID material1, MaterialID material2, real cor );
   static inline void setStaticFriction( MaterialID material1, MaterialID material2, real csf );
   static inline void setDynamicFriction( MaterialID material1, MaterialID material2, real cdf );
   //@}
   //**********************************************************************************************

private:
   //**Setup functions*****************************************************************************
   /*!\name Setup functions */
   //@{
   static bool activateMaterials();
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   std::string name_;  //!< The name of the material.
   real density_;      //!< The density of the material.
   real restitution_;  //!< The coefficient of restitution (COR) of a self-similar collision \f$ [0..1] \f$.
                       /*!< The COR represents the energy dissipated during a collision between
                            self-similar bodies, that is bodies with similar materials. A value of
                            0 corresponds to completely inelastic collision where all energy is
                            dissipated, a value of 1 corresponds to a completely elastic collision
                            where no energy is lost. The COR is assumed to be rate-independent. The
                            COR is often determined experimentally by measuring the pre- and
                            post-impact relative velocities:
                            \f[ C_R = \frac{V_{2,after}-V_{1,after}}{V_{2,before}-V_{1,before}} \f]
                            During a collision, the COR values of the two colliding
                            rigid bodies can be used by the collision response mechanism to
                            determine the restitution factor of the contact point. */
   real static_;       //!< The coefficient of static friction (CSF) \f$ [0..\infty) \f$.
                       /*!< The CSF is a dimensionless, non-negative quantity representing the
                            amount of static friction between two touching rigid bodies. Static
                            friction occurs in case the relative tangential velocity between the
                            two bodies is 0. Then the force magnitudes of the normal and friction
                            force are related by an inequality:
                            \f[ |\vec{f_t}| \leq \mu_s |\vec{f_n}| \f]
                            The direction of the friction must oppose acceleration if sliding is
                            imminent and is unresticted otherwise. */
   real dynamic_;      //!< The coefficient of dynamic friction (CDF) \f$ [0..\infty) \f$.
                       /*!< The CDF is a dimensionless, non-negative quantity representing the
                            amount of dynamic friction between two touching rigid bodies. Dynamic
                            friction occurs in case the relative tangential velocity between the
                            two bodies is greater than 0. Then the force magnitudes of the normal
                            and friction force are related by an inequality:
                            \f[ |\vec{f_t}| = -\mu_d |\vec{f_n}| \frac{\vec{v_t}}{|\vec{v_t}|} \f] */
   real poisson_;      //!< The Poisson's ratio for the material \f$ [-1..0.5] \f$.
                       /*!< When a material is compressed in one direction, it usually tends to
                            expand in the other two directions perpendicular to the direction of
                            compression. This effect is called Poisson effect. In this context, the
                            Poisson's ratio is the ratio of the contraction or transverse strain
                            (perpendicular to the applied load) to the extension or axial strain
                            (in the direction of the applied load). For stable, isotropic, linear
                            elastic materials this ratio cannot be less than -1.0 nor greater than
                            0.5 due to the requirement that Young's modulus has positive values. */
   real young_;        //!< The Young's modulus for the material \f$ (0..\infty) \f$.
                       /*!< The Young's modulus is a measure for the stiffness of an isotropic
                            elastic material. It is defined as the ratio of the uniaxial stress
                            over the uniaxial strain in the range of stress in which Hooke's law
                            holds. The SI unit for Young's modulus is \f$ Pa \f$ or \f$ N/m^2 \f$. */
   real stiffness_;    //!< The stiffness of the contact region \f$ (0..\infty) \f$.
                       /*!< Rigid body theory assumes that the deformation during contact is
                            localized to the contact region. This local compliance can be modelled
                            simplified as a spring-damper. The spring constant corresponds to this
                            parameter. */
   real dampingN_;     //!< The damping at the contact region in normal direction \f$ [0..\infty) \f$.
                       /*!< Rigid body theory assumes that the deformation during contact is
                            localized to the contact region. This local compliance in normal
                            direction can be modelled simplified as a spring-damper. The viscous
                            damping coefficient corresponds to this parameter. */
   real dampingT_;     //!< The damping at the contact region in tangential direction \f$ [0..\infty) \f$.
                       /*!< Friction counteracts the tangential relative velocity and thus can be
                            modelled as a viscous damper with a limited damping force. The viscous
                            damping coefficient corresponds to this parameter.*/

   static Materials materials_;      //!< Vector for the registered materials.
   static MatN corTable_;            //!< Table for the coefficients of restitution.
   static MatN csfTable_;            //!< Table for the coefficients of static friction.
   static MatN cdfTable_;            //!< Table for the coefficients of dynamic friction.
   static bool materialsActivated_;  //!< Helper variable for the automatic registration process.
   static unsigned int anonymousMaterials_;   //!< Counter for the amount of anonymous materials.
   //@}
   //**********************************************************************************************

   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   friend MaterialID createMaterial( const std::string& name, real density, real cor,
                                     real csf, real cdf, real poisson, real young,
                                     real stiffness, real dampingN, real dampingT );
   friend MaterialID createMaterial( real density, real cor, real csf, real cdf,
                                     real poisson, real young,
                                     real stiffness, real dampingN, real dampingT );
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The constructor of the Material class.
 *
 * \param name The name of the material.
 * \param density The density of the material \f$ (0..\infty) \f$.
 * \param cor The coefficient of restitution (COR) of the material \f$ [0..1] \f$.
 * \param csf The coefficient of static friction (CSF) of the material \f$ [0..\infty) \f$.
 * \param cdf The coefficient of dynamic friction (CDF) of the material \f$ [0..\infty) \f$.
 * \param poisson The Poisson's ratio of the material \f$ [-1..0.5] \f$.
 * \param young The Young's modulus of the material \f$ (0..\infty) \f$.
 * \param stiffness The stiffness in normal direction of the material's contact region.
 * \param dampingN The damping coefficient in normal direction of the material's contact region.
 * \param dampingT The damping coefficient in tangential direction of the material's contact region.
 */
inline Material::Material( const std::string& name, real density, real cor,
                           real csf, real cdf, real poisson, real young,
                           real stiffness, real dampingN, real dampingT )
   : name_       ( name )       // The name of the material
   , density_    ( density )    // The density of the material
   , restitution_( cor )        // The coefficient of restitution of the material
   , static_     ( csf )        // The coefficient of static friction of the material
   , dynamic_    ( cdf )        // The coefficient of dynamic friction of the material
   , poisson_    ( poisson )    // The Poisson's ratio for the material
   , young_      ( young )      // The Young's modulus for the material
   , stiffness_  ( stiffness )  // The stiffness in normal direction of the material's contact region.
   , dampingN_   ( dampingN )   // The damping coefficient in normal direction of the material's contact region.
   , dampingT_   ( dampingT )   // The damping coefficient in tangential direction of the material's contact region.
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the name of the material.
 *
 * \return The name of the material.
 */
inline const std::string& Material::getName() const
{
   return name_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the density of the material.
 *
 * \return The density of the material.
 */
inline real Material::getDensity() const
{
   return density_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the coefficient of restitution of the material.
 *
 * \return The coefficient of restitution of the material.
 */
inline real Material::getRestitution() const
{
   return restitution_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the coefficient of static friction of the material.
 *
 * \return The coefficient of static friction of the material.
 */
inline real Material::getStaticFriction() const
{
   return static_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the coefficient of dynamic friction of the material.
 *
 * \return The coefficient of dynamic friction of the material.
 */
inline real Material::getDynamicFriction() const
{
   return dynamic_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the Poisson's ratio of the material.
 *
 * \return The Poisson's ratio of the material.
 */
inline real Material::getPoissonRatio() const
{
   return poisson_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the Young's modulus of the material.
 *
 * \return The Young's modulus of the material.
 */
inline real Material::getYoungModulus() const
{
   return young_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the stiffness in normal direction of the material's contact region.
 *
 * \return The stiffness in normal direction of the material's contact region.
 */
inline real Material::getStiffness() const
{
   return stiffness_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the damping coefficient in normal direction of the material's contact region.
 *
 * \return The damping coefficient in normal direction of the material's contact region.
 */
inline real Material::getDampingN() const
{
   return dampingN_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the damping coefficient in tangential direction of the material's contact region.
 *
 * \return The damping coefficient in tangential direction of the material's contact region.
 */
inline real Material::getDampingT() const
{
   return dampingT_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the name of the given material.
 * \ingroup materials
 *
 * \param material The material to be queried.
 * \return The name of the given material.
 */
inline const std::string& Material::getName( MaterialID material )
{
   pe_USER_ASSERT( material < materials_.size(), "Invalid material ID" );
   return materials_[material].getName();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the density of the given material.
 * \ingroup materials
 *
 * \param material The material to be queried.
 * \return The density of the given material.
 */
inline real Material::getDensity( MaterialID material )
{
   pe_USER_ASSERT( material < materials_.size(), "Invalid material ID" );
   return materials_[material].getDensity();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the coefficient of restitution of the given material.
 * \ingroup materials
 *
 * \param material The material to be queried.
 * \return The coefficient of restitution of the given material.
 */
inline real Material::getRestitution( MaterialID material )
{
   pe_USER_ASSERT( material < materials_.size(), "Invalid material ID" );
   return materials_[material].getRestitution();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the composite coefficient of restitution for a collision between two rigid bodies.
 * \ingroup materials
 *
 * \param material1 The material of the first colliding rigid body.
 * \param material2 The material of the second colliding rigid body.
 * \return The resulting composite coefficient of restitution of the collision.
 */
inline real Material::getRestitution( MaterialID material1, MaterialID material2 )
{
   pe_USER_ASSERT( material1 < materials_.size(), "Invalid material ID" );
   pe_USER_ASSERT( material2 < materials_.size(), "Invalid material ID" );
   return corTable_( material1, material2 );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the coefficient of static friction of the given material.
 * \ingroup materials
 *
 * \param material The material to be queried.
 * \return The coefficient of static friction of the given material.
 */
inline real Material::getStaticFriction( MaterialID material )
{
   pe_USER_ASSERT( material < materials_.size(), "Invalid material ID" );
   return materials_[material].getStaticFriction();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the coefficient of static friction for a collision between two rigid bodies.
 * \ingroup materials
 *
 * \param material1 The material of the first colliding rigid body.
 * \param material2 The material of the second colliding rigid body.
 * \return The resulting coefficient of static friction of the collision.
 */
inline real Material::getStaticFriction( MaterialID material1, MaterialID material2 )
{
   pe_USER_ASSERT( material1 < materials_.size(), "Invalid material ID" );
   pe_USER_ASSERT( material2 < materials_.size(), "Invalid material ID" );
   return csfTable_( material1, material2 );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the coefficient of dynamic friction of the given material.
 * \ingroup materials
 *
 * \param material The material to be queried.
 * \return The coefficient of dynamic friction of the given material.
 */
inline real Material::getDynamicFriction( MaterialID material )
{
   pe_USER_ASSERT( material < materials_.size(), "Invalid material ID" );
   return materials_[material].getDynamicFriction();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the coefficient of dynamic friction for a collision between two rigid bodies.
 * \ingroup materials
 *
 * \param material1 The material of the first colliding rigid body.
 * \param material2 The material of the second colliding rigid body.
 * \return The resulting coefficient of dynamic friction of the collision.
 */
inline real Material::getDynamicFriction( MaterialID material1, MaterialID material2 )
{
   pe_USER_ASSERT( material1 < materials_.size(), "Invalid material ID" );
   pe_USER_ASSERT( material2 < materials_.size(), "Invalid material ID" );
   return cdfTable_( material1, material2 );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the Poisson's ratio of the given material.
 * \ingroup materials
 *
 * \param material The material to be queried.
 * \return The Poisson's ratio of the given material.
 */
inline real Material::getPoissonRatio( MaterialID material )
{
   pe_USER_ASSERT( material < materials_.size(), "Invalid material ID" );
   return materials_[material].getPoissonRatio();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the Young's modulus of the given material.
 * \ingroup materials
 *
 * \param material The material to be queried.
 * \return The Young's modulus of the given material.
 */
inline real Material::getYoungModulus( MaterialID material )
{
   pe_USER_ASSERT( material < materials_.size(), "Invalid material ID" );
   return materials_[material].getYoungModulus();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the (effective) Young's modulus for a collision between two rigid bodies.
 * \ingroup materials
 *
 * \param material1 The material of the first colliding rigid body.
 * \param material2 The material of the second colliding rigid body.
 * \return The resulting (effective) Young's modulus of the collision.
 *
 * This function returns the effective Young's modulus for a collision between two rigid bodies.
 * The effective Young's modulus is calculated as

          \f[ \frac{1}{E_{eff}} = \frac{1 - \nu_1^2}{E_1} + \frac{1 - \nu_2^2}{E_2}, \f]

 * where \f$ E_1 \f$ and \f$ E_2 \f$ are the Young's modulus for the first and second material,
 * respectively, and \f$ \nu_1 \f$ and \f$ \nu_2 \f$ are the Poisson's ratio for the materials.
 */
inline real Material::getYoungModulus( MaterialID material1, MaterialID material2 )
{
   pe_USER_ASSERT( material1 < materials_.size(), "Invalid material ID" );
   pe_USER_ASSERT( material2 < materials_.size(), "Invalid material ID" );

   const real nu1( getPoissonRatio( material1 ) );
   const real nu2( getPoissonRatio( material2 ) );
   const real y1 ( getYoungModulus( material1 ) );
   const real y2 ( getYoungModulus( material2 ) );

   const real tmp1( y2 * ( real(1) - nu1*nu1 ) );
   const real tmp2( y1 * ( real(1) - nu2*nu2 ) );

   return ( ( y1*y2 ) / ( tmp1 + tmp2 ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the stiffness in normal direction of the material's contact region.
 * \ingroup materials
 *
 * \param material The material to be queried.
 * \return The stiffness in normal direction of the contact region of the given material.
 */
inline real Material::getStiffness( MaterialID material )
{
   pe_USER_ASSERT( material < materials_.size(), "Invalid material ID" );
   return materials_[material].getStiffness();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the stiffness in normal direction of the contact between two materials.
 * \ingroup materials
 *
 * \param material1 The material of the first colliding rigid body.
 * \param material2 The material of the second colliding rigid body.
 * \return The stiffness in normal direction of the contact between two materials.
 *
 * Rigid body theory assumes that deformation during contact is localized to the contact region.
 * Therefore the contact region is often modelled simplified as a spring-damper. When two bodies
 * are in contact the spring-dampers are serially connected and thus the contact stiffness can
 * be expressed as the series connection of two springs: \f$ k_*^{-1} = k_1^{-1} + k_2^{-1}\f$.
 */
inline real Material::getStiffness( MaterialID material1, MaterialID material2 )
{
   pe_USER_ASSERT( material1 < materials_.size(), "Invalid material ID" );
   pe_USER_ASSERT( material2 < materials_.size(), "Invalid material ID" );

   return inv( inv( getStiffness( material1 ) ) + inv( getStiffness( material2 ) ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the damping coefficient in normal direction of the material's contact region.
 * \ingroup materials
 *
 * \param material The material to be queried.
 * \return The damping in normal direction of the contact region of the given material.
 */
inline real Material::getDampingN( MaterialID material )
{
   pe_USER_ASSERT( material < materials_.size(), "Invalid material ID" );
   return materials_[material].getDampingN();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the damping in normal direction of the contact between two materials.
 * \ingroup materials
 *
 * \param material1 The material of the first colliding rigid body.
 * \param material2 The material of the second colliding rigid body.
 * \return The damping in normal direction of the contact between two materials.
 *
 * Rigid body theory assumes that deformation during contact is localized to the contact region.
 * Therefore the contact region is often modelled simplified as a spring-damper. When two bodies
 * are in contact the spring-dampers are serially connected and thus the contact damping can
 * be expressed as the series connection of two viscous dampers: \f$ c_*^{-1} = c_1^{-1} + c_2^{-1}\f$.
 */
inline real Material::getDampingN( MaterialID material1, MaterialID material2 )
{
   pe_USER_ASSERT( material1 < materials_.size(), "Invalid material ID" );
   pe_USER_ASSERT( material2 < materials_.size(), "Invalid material ID" );

   return inv( inv( getDampingN( material1 ) ) + inv( getDampingN( material2 ) ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the damping coefficient in tangential direction of the material's contact region.
 * \ingroup materials
 *
 * \param material The material to be queried.
 * \return The damping in tangential direction of the contact region of the given material.
 */
inline real Material::getDampingT( MaterialID material )
{
   pe_USER_ASSERT( material < materials_.size(), "Invalid material ID" );
   return materials_[material].getDampingT();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the damping in tangential direction of the contact between two materials.
 * \ingroup materials
 *
 * \param material1 The material of the first colliding rigid body.
 * \param material2 The material of the second colliding rigid body.
 * \return The damping in tangential direction of the contact between two materials.
 *
 * Rigid body theory assumes that deformation during contact is localized to the contact region.
 * Therefore the contact region is often modelled simplified as a spring-damper. When two bodies
 * are in contact the spring-dampers are serially connected and thus the contact damping can
 * be expressed as the series connection of two viscous dampers: \f$ c_*^{-1} = c_1^{-1} + c_2^{-1}\f$.
 */
inline real Material::getDampingT( MaterialID material1, MaterialID material2 )
{
   pe_USER_ASSERT( material1 < materials_.size(), "Invalid material ID" );
   pe_USER_ASSERT( material2 < materials_.size(), "Invalid material ID" );

   return inv( inv( getDampingT( material1 ) ) + inv( getDampingT( material2 ) ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the coefficient of restitution between material \a material1 and \a material2.
 * \ingroup materials
 *
 * \param material1 The material of the first colliding rigid body.
 * \param material2 The material of the second colliding rigid body.
 * \param cor The coefficient of restitution between \a material1 and \a material2.
 * \return void
 */
inline void Material::setRestitution( MaterialID material1, MaterialID material2, real cor )
{
   pe_USER_ASSERT( material1 < materials_.size(), "Invalid material ID" );
   pe_USER_ASSERT( material2 < materials_.size(), "Invalid material ID" );
   corTable_( material1, material2 ) = cor;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the coefficient of static friction between material \a material1 and \a material2.
 * \ingroup materials
 *
 * \param material1 The material of the first colliding rigid body.
 * \param material2 The material of the second colliding rigid body.
 * \param csf The coefficient of static friction between \a material1 and \a material2.
 * \return void
 */
inline void Material::setStaticFriction( MaterialID material1, MaterialID material2, real csf )
{
   pe_USER_ASSERT( material1 < materials_.size(), "Invalid material ID" );
   pe_USER_ASSERT( material2 < materials_.size(), "Invalid material ID" );
   csfTable_( material1, material2 ) = csf;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the coefficient of dynamic friction between material \a material1 and \a material2.
 * \ingroup materials
 *
 * \param material1 The material of the first colliding rigid body.
 * \param material2 The material of the second colliding rigid body.
 * \param cdf The coefficient of dynamic friction between \a material1 and \a material2.
 * \return void
 */
inline void Material::setDynamicFriction( MaterialID material1, MaterialID material2, real cdf )
{
   pe_USER_ASSERT( material1 < materials_.size(), "Invalid material ID" );
   pe_USER_ASSERT( material2 < materials_.size(), "Invalid material ID" );
   cdfTable_( material1, material2 ) = cdf;
}
//*************************************************************************************************




//=================================================================================================
//
//  MATERIAL IRON
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Specification of the material iron.
 * \ingroup materials
 *
 * The Iron class represents the material iron. It is implemented as a veneer class for the
 * Material base class to set the properties of iron:
 *
 * - Name: "iron"
 * - Density: \f$ 7.874 \frac{kg}{dm^3} \f$
 * - Coefficient of restitution: 0.5
 * - Coefficient of static friction: 0.1
 * - Coefficient of dynamic friction: 0.1
 * - Poisson's ratio: 0.24
 * - Young's modulus: 200 GPa
 * - Stiffness: \f$ ~200 \frac{N}{m} \f$
 * - Normal Damping: \f$ 0 \frac{Ns}{m} \f$
 * - Tangential Damping: \f$ 0 \frac{Ns}{m} \f$
 *
 * Since several parameters are not unitless they might not match the scaling of the simulation.
 * In that case custom materials must be created. Also even though the stiffness is proportional
 * to Young's modulus the proportionality constant depends on other parameters such as the shape of
 * the contact region or the radii of the objects. Thus if the simulation does rely on the value of
 * the stiffness the user must supply an appropriate stiffness coefficient. Since no published
 * values were available for the damping coefficients they are deactivated.
 *
 * The iron material is automatically registered and can be directly used by the predefined
 * constant specifier \a iron:

   \code
   // Creating an iron sphere
   SphereID sphere = createSphere( 1, 0.0, 0.0, 0.0, iron );
   \endcode
 */
class PE_PUBLIC Iron : public Material
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit inline Iron();
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default constructor for the Iron class.
 */
inline Iron::Iron()
   : Material( "iron", static_cast<real>( 7.874 ), static_cast<real>( 0.5 ), static_cast<real>( 0.1 ), static_cast<real>( 0.1 ), static_cast<real>( 0.24 ), static_cast<real>( 200 ), static_cast<real>( 200 ), static_cast<real>( 0 ), static_cast<real>( 0 ) )  // Initialization of the Material base class
{
   pe_CONSTRAINT_MUST_HAVE_SAME_SIZE( Material, Iron );
}
//*************************************************************************************************




//=================================================================================================
//
//  MATERIAL COPPER
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Specification of the material copper.
 * \ingroup materials
 *
 * The Copper class represents the material copper. It is implemented as a veneer class for
 * the Material base class to set the properties of iron:
 *
 * - Name: "copper"
 * - Density: \f$ 8.92 \frac{kg}{dm^3} \f$
 * - Coefficient of restitution: 0.5
 * - Coefficient of static friction: 0.1
 * - Coefficient of dynamic friction: 0.1
 * - Poisson's ratio: 0.33
 * - Young's modulus: 117 GPa
 * - Stiffness: \f$ ~117 \frac{N}{m} \f$
 * - Normal Damping: \f$ 0 \frac{Ns}{m} \f$
 * - Tangential Damping: \f$ 0 \frac{Ns}{m} \f$
 *
 * Since several parameters are not unitless they might not match the scaling of the simulation.
 * In that case custom materials must be created. Also even though the stiffness is proportional
 * to Young's modulus the proportionality constant depends on other parameters such as the shape of
 * the contact region or the radii of the objects. Thus if the simulation does rely on the value of
 * the stiffness the user must supply an appropriate stiffness coefficient. Since no published
 * values were available for the damping coefficients they are deactivated.
 *
 * The copper material is automatically registered and can be directly used by the predefined
 * constant specifier \a copper:

   \code
   // Creating a copper sphere
   SphereID sphere = createSphere( 1, 0.0, 0.0, 0.0, copper );
   \endcode
 */
class PE_PUBLIC Copper : public Material
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit inline Copper();
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default constructor for the Copper class.
 */
inline Copper::Copper()
   : Material( "copper", static_cast<real>( 8.92 ), static_cast<real>( 0.5 ), static_cast<real>( 0.1 ), static_cast<real>( 0.1 ), static_cast<real>( 0.33 ), static_cast<real>( 117 ), static_cast<real>( 117 ), static_cast<real>( 0 ), static_cast<real>( 0 ) )  // Initialization of the Material base class
{
   pe_CONSTRAINT_MUST_HAVE_SAME_SIZE( Material, Copper );
}
//*************************************************************************************************




//=================================================================================================
//
//  MATERIAL GRANITE
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Specification of the material granite.
 * \ingroup materials
 *
 * The Granite class represents the material granite. It is implemented as a veneer class for
 * the Material base class to set the properties of granite:
 *
 * - Name: "granite"
 * - Density: \f$ 2.80 \frac{kg}{dm^3} \f$
 * - Coefficient of restitution: 0.5
 * - Coefficient of static friction: 0.1
 * - Coefficient of dynamic friction: 0.1
 * - Poisson's ratio: 0.25
 * - Young's modulus: 55 GPa
 * - Stiffness: \f$ ~55 \frac{N}{m} \f$
 * - Normal Damping: \f$ 0 \frac{Ns}{m} \f$
 * - Tangential Damping: \f$ 0 \frac{Ns}{m} \f$
 *
 * Since several parameters are not unitless they might not match the scaling of the simulation.
 * In that case custom materials must be created. Also even though the stiffness is proportional
 * to Young's modulus the proportionality constant depends on other parameters such as the shape of
 * the contact region or the radii of the objects. Thus if the simulation does rely on the value of
 * the stiffness the user must supply an appropriate stiffness coefficient. Since no published
 * values were available for the damping coefficients they are deactivated.
 *
 * The granite material is automatically registered and can be directly used by the predefined
 * constant specifier \a granite:

   \code
   // Creating a granite sphere
   SphereID sphere = createSphere( 1, 0.0, 0.0, 0.0, granite );
   \endcode
 */
class PE_PUBLIC Granite : public Material
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit inline Granite();
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default constructor for the Granite class.
 */
inline Granite::Granite()
   : Material( "granite", static_cast<real>( 2.8 ), static_cast<real>( 0.5 ), static_cast<real>( 0.1 ), static_cast<real>( 0.1 ), static_cast<real>( 0.25 ), static_cast<real>( 55 ), static_cast<real>( 55 ), static_cast<real>( 0 ), static_cast<real>( 0 ) )  // Initialization of the Material base class
{
   pe_CONSTRAINT_MUST_HAVE_SAME_SIZE( Material, Granite );
}
//*************************************************************************************************




//=================================================================================================
//
//  MATERIAL OAK
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Specification of the material oak.
 * \ingroup materials
 *
 * The Oak class represents the material oak wood. It is implemented as a veneer class for the
 * Material base class to set the properties of oak wood:
 *
 * - Name: "oak"
 * - Density: \f$ 0.8 \frac{kg}{dm^3} \f$
 * - Coefficient of restitution: 0.5
 * - Coefficient of static friction: 0.1
 * - Coefficient of dynamic friction: 0.1
 * - Poisson's ratio: 0.35
 * - Young's modulus: 11 GPa
 * - Stiffness: \f$ ~11 \frac{N}{m} \f$
 * - Normal Damping: \f$ 0 \frac{Ns}{m} \f$
 * - Tangential Damping: \f$ 0 \frac{Ns}{m} \f$
 *
 * Since several parameters are not unitless they might not match the scaling of the simulation.
 * In that case custom materials must be created. Also even though the stiffness is proportional
 * to Young's modulus the proportionality constant depends on other parameters such as the shape of
 * the contact region or the radii of the objects. Thus if the simulation does rely on the value of
 * the stiffness the user must supply an appropriate stiffness coefficient. Since no published
 * values were available for the damping coefficients they are deactivated.
 *
 * The oak wood material is automatically registered and can be directly used by the predefined
 * constant specifier \a oak:

   \code
   // Creating an oak wood sphere
   SphereID sphere = createSphere( 1, 0.0, 0.0, 0.0, oak );
   \endcode
 */
class PE_PUBLIC Oak : public Material
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit inline Oak();
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default constructor for the Oak class.
 */
inline Oak::Oak()
   : Material( "oak", static_cast<real>( 0.8 ), static_cast<real>( 0.5 ), static_cast<real>( 0.1 ), static_cast<real>( 0.1 ), static_cast<real>( 0.35 ), static_cast<real>( 11 ), static_cast<real>( 11 ), static_cast<real>( 0 ), static_cast<real>( 0 ) )  // Initialization of the Material base class
{
   pe_CONSTRAINT_MUST_HAVE_SAME_SIZE( Material, Oak );
}
//*************************************************************************************************




//=================================================================================================
//
//  MATERIAL FIR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Specification of the material fir.
 * \ingroup materials
 *
 * The Fir class represents the material fir wood. It is implemented as a veneer class for the
 * Material base class to set the properties of fir wood:
 *
 * - Name: "fir"
 * - Density: \f$ 0.5 \frac{kg}{dm^3} \f$
 * - Coefficient of restitution: 0.5
 * - Coefficient of static friction: 0.1
 * - Coefficient of dynamic friction: 0.1
 * - Poisson's ratio: 0.34
 * - Young's modulus: 13 GPa
 * - Stiffness: \f$ ~13 \frac{N}{m} \f$
 * - Normal Damping: \f$ 0 \frac{Ns}{m} \f$
 * - Tangential Damping: \f$ 0 \frac{Ns}{m} \f$
 *
 * Since several parameters are not unitless they might not match the scaling of the simulation.
 * In that case custom materials must be created. Also even though the stiffness is proportional
 * to Young's modulus the proportionality constant depends on other parameters such as the shape of
 * the contact region or the radii of the objects. Thus if the simulation does rely on the value of
 * the stiffness the user must supply an appropriate stiffness coefficient. Since no published
 * values were available for the damping coefficients they are deactivated.
 *
 * The fir wood material is automatically registered and can be directly used by the predefined
 * constant specifier \a fir:

   \code
   // Creating a fir wood sphere
   SphereID sphere = createSphere( 1, 0.0, 0.0, 0.0, fir );
   \endcode
 */
class PE_PUBLIC Fir : public Material
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit inline Fir();
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default constructor for the Fir class.
 */
inline Fir::Fir()
   : Material( "fir", static_cast<real>( 0.5 ), static_cast<real>( 0.5 ), static_cast<real>( 0.1 ), static_cast<real>( 0.1 ), static_cast<real>( 0.34 ), static_cast<real>( 13 ), static_cast<real>( 13 ), static_cast<real>( 0 ), static_cast<real>( 0 ) )  // Initialization of the Material base class
{
   pe_CONSTRAINT_MUST_HAVE_SAME_SIZE( Material, Fir );
}
//*************************************************************************************************




//=================================================================================================
//
//  MATERIAL CONSTANTS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief ID for the material iron.
 * \ingroup materials
 *
 * This material can be used to create iron rigid bodies. Iron has the following material
 * properties:
 *
 * - Name: "iron"
 * - Density: \f$ 7.874 \frac{kg}{dm^3} \f$
 * - Restitution factor: 0.5
 */
const MaterialID iron = 0;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief ID for the material copper.
 * \ingroup materials
 *
 * This material can be used to create copper rigid bodies. Copper has the following material
 * properties:
 *
 * - Name: "copper"
 * - Density: \f$ 8.92 \frac{kg}{dm^3} \f$
 * - Coefficient of restitution: 0.5
 * - Coefficient of static friction: 0.1
 * - Coefficient of dynamic friction: 0.1
 */
const MaterialID copper = 1;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief ID for the material granite.
 * \ingroup materials
 *
 * This material can be used to create granite rigid bodies.
 */
const MaterialID granite = 2;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief ID for the material oak wood.
 * \ingroup materials
 *
 * This material can be used to create rigid bodies made from oak wood.
 */
const MaterialID oak = 3;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief ID for the material fir wood.
 * \ingroup materials
 *
 * This material can be used to create rigid bodies made from fir wood.
 */
const MaterialID fir = 4;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief ID for an invalid material.
 * \ingroup materials
 *
 * This MaterialID is returned by the getMaterial() function in case no material with the
 * specified name is returned. This value should not be used to create rigid bodies or in
 * any other function!
 */
const MaterialID invalid_material = static_cast<MaterialID>( -1 );
//*************************************************************************************************




//=================================================================================================
//
//  MATERIAL FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Material functions */
//@{
PE_PUBLIC MaterialID createMaterial( const std::string& name, real density, real cor,
                           real csf, real cdf, real poisson, real young, real stiffness, real dampingN, real dampingT );
PE_PUBLIC MaterialID createMaterial( real density, real cor, real csf, real cdf,
                           real poisson, real young, real stiffness, real dampingN, real dampingT );
//@}
//*************************************************************************************************

} // namespace pe

#endif
