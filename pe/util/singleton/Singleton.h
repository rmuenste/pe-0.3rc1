//=================================================================================================
/*!
 *  \file pe/util/singleton/Singleton.h
 *  \brief Header file for the Singleton class
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
// //=================================================================================================

#ifndef _PE_UTIL_SINGLETON_SINGLETON_H_
#define _PE_UTIL_SINGLETON_SINGLETON_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <boost/shared_ptr.hpp>
#include <pe/util/constraints/DerivedFrom.h>
#include <pe/util/NonCopyable.h>
#include <pe/util/NullType.h>
#include <pe/util/Suffix.h>
#include <pe/util/TypeList.h>


namespace pe {

//=================================================================================================
//
//  ::pe NAMESPACE FORWARD DECLARATIONS
//
//=================================================================================================

template< typename > class Dependency;
template< typename T, typename TL, bool C > struct HasCyclicDependency;




//=================================================================================================
//
//  CLASS HASCYCLICDEPENDENCYHELPER
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Auxiliary helper struct for the HasCyclicDependency class template.
 * \ingroup singleton
 *
 * Helper template class for the HasCyclicDependency template class to resolve all lifetime
 * dependencies represented by means of a dependency type list.
 */
template< typename TL                      // Type list of checked lifetime dependencies
        , typename D                       // Type list of lifetime dependencies to check
        , size_t   N = Length<D>::value >  // Length of the dependency type list
struct HasCyclicDependencyHelper;
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Specialization of the HasCyclicDependencyHelper class template.
 * \ingroup singleton
 *
 * This specialization of the HasCyclicDependencyHelper class is selected in case the given
 * dependency type list is empty. In this case no cyclic lifetime dependency could be detected.
 */
template< typename TL   // Type list of checked lifetime dependencies
        , size_t   N >  // Length of the dependency type list
struct HasCyclicDependencyHelper<TL,NullType,N>
{
   enum { value = 0 };
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Specialization of the HasCyclicDependencyHelper class template.
 * \ingroup singleton
 *
 * This specialization of the HasCyclicDependencyHelper class is selected in case the length
 * of the given type list is 1.
 */
template< typename TL   // Type list of checked lifetime dependencies
        , typename D >  // Type list of lifetime dependencies to check
struct HasCyclicDependencyHelper<TL,D,1>
{
   typedef typename TypeAt<D,0>::Result  D1;

   enum { value = HasCyclicDependency<D1,TL,Contains<TL,D1>::value>::value };
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Specialization of the HasCyclicDependencyHelper class template.
 * \ingroup singleton
 *
 * This specialization of the HasCyclicDependencyHelper class is selected in case the length
 * of the given type list is 2.
 */
template< typename TL   // Type list of checked lifetime dependencies
        , typename D >  // Type list of lifetime dependencies to check
struct HasCyclicDependencyHelper<TL,D,2>
{
   typedef typename TypeAt<D,0>::Result  D1;
   typedef typename TypeAt<D,1>::Result  D2;

   enum { value = HasCyclicDependency<D1,TL,Contains<TL,D1>::value>::value ||
                  HasCyclicDependency<D2,TL,Contains<TL,D2>::value>::value };
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Specialization of the HasCyclicDependencyHelper class template.
 * \ingroup singleton
 *
 * This specialization of the HasCyclicDependencyHelper class is selected in case the length
 * of the given type list is 3.
 */
template< typename TL   // Type list of checked lifetime dependencies
        , typename D >  // Type list of lifetime dependencies to check
struct HasCyclicDependencyHelper<TL,D,3>
{
   typedef typename TypeAt<D,0>::Result  D1;
   typedef typename TypeAt<D,1>::Result  D2;
   typedef typename TypeAt<D,2>::Result  D3;

   enum { value = HasCyclicDependency<D1,TL,Contains<TL,D1>::value>::value ||
                  HasCyclicDependency<D2,TL,Contains<TL,D2>::value>::value ||
                  HasCyclicDependency<D3,TL,Contains<TL,D3>::value>::value };
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Specialization of the HasCyclicDependencyHelper class template.
 * \ingroup singleton
 *
 * This specialization of the HasCyclicDependencyHelper class is selected in case the length
 * of the given type list is 4.
 */
template< typename TL   // Type list of checked lifetime dependencies
        , typename D >  // Type list of lifetime dependencies to check
struct HasCyclicDependencyHelper<TL,D,4>
{
   typedef typename TypeAt<D,0>::Result  D1;
   typedef typename TypeAt<D,1>::Result  D2;
   typedef typename TypeAt<D,2>::Result  D3;
   typedef typename TypeAt<D,3>::Result  D4;

   enum { value = HasCyclicDependency<D1,TL,Contains<TL,D1>::value>::value ||
                  HasCyclicDependency<D2,TL,Contains<TL,D2>::value>::value ||
                  HasCyclicDependency<D3,TL,Contains<TL,D3>::value>::value ||
                  HasCyclicDependency<D4,TL,Contains<TL,D4>::value>::value };
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Specialization of the HasCyclicDependencyHelper class template.
 * \ingroup singleton
 *
 * This specialization of the HasCyclicDependencyHelper class is selected in case the length
 * of the given type list is 5.
 */
template< typename TL   // Type list of checked lifetime dependencies
        , typename D >  // Type list of lifetime dependencies to check
struct HasCyclicDependencyHelper<TL,D,5>
{
   typedef typename TypeAt<D,0>::Result  D1;
   typedef typename TypeAt<D,1>::Result  D2;
   typedef typename TypeAt<D,2>::Result  D3;
   typedef typename TypeAt<D,3>::Result  D4;
   typedef typename TypeAt<D,4>::Result  D5;

   enum { value = HasCyclicDependency<D1,TL,Contains<TL,D1>::value>::value ||
                  HasCyclicDependency<D2,TL,Contains<TL,D2>::value>::value ||
                  HasCyclicDependency<D3,TL,Contains<TL,D3>::value>::value ||
                  HasCyclicDependency<D4,TL,Contains<TL,D4>::value>::value ||
                  HasCyclicDependency<D5,TL,Contains<TL,D5>::value>::value };
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Specialization of the HasCyclicDependencyHelper class template.
 * \ingroup singleton
 *
 * This specialization of the HasCyclicDependencyHelper class is selected in case the length
 * of the given type list is 6.
 */
template< typename TL   // Type list of checked lifetime dependencies
        , typename D >  // Type list of lifetime dependencies to check
struct HasCyclicDependencyHelper<TL,D,6>
{
   typedef typename TypeAt<D,0>::Result  D1;
   typedef typename TypeAt<D,1>::Result  D2;
   typedef typename TypeAt<D,2>::Result  D3;
   typedef typename TypeAt<D,3>::Result  D4;
   typedef typename TypeAt<D,4>::Result  D5;
   typedef typename TypeAt<D,5>::Result  D6;

   enum { value = HasCyclicDependency<D1,TL,Contains<TL,D1>::value>::value ||
                  HasCyclicDependency<D2,TL,Contains<TL,D2>::value>::value ||
                  HasCyclicDependency<D3,TL,Contains<TL,D3>::value>::value ||
                  HasCyclicDependency<D4,TL,Contains<TL,D4>::value>::value ||
                  HasCyclicDependency<D5,TL,Contains<TL,D5>::value>::value ||
                  HasCyclicDependency<D6,TL,Contains<TL,D6>::value>::value };
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Specialization of the HasCyclicDependencyHelper class template.
 * \ingroup singleton
 *
 * This specialization of the HasCyclicDependencyHelper class is selected in case the length
 * of the given type list is 7.
 */
template< typename TL   // Type list of checked lifetime dependencies
        , typename D >  // Type list of lifetime dependencies to check
struct HasCyclicDependencyHelper<TL,D,7>
{
   typedef typename TypeAt<D,0>::Result  D1;
   typedef typename TypeAt<D,1>::Result  D2;
   typedef typename TypeAt<D,2>::Result  D3;
   typedef typename TypeAt<D,3>::Result  D4;
   typedef typename TypeAt<D,4>::Result  D5;
   typedef typename TypeAt<D,5>::Result  D6;
   typedef typename TypeAt<D,6>::Result  D7;

   enum { value = HasCyclicDependency<D1,TL,Contains<TL,D1>::value>::value ||
                  HasCyclicDependency<D2,TL,Contains<TL,D2>::value>::value ||
                  HasCyclicDependency<D3,TL,Contains<TL,D3>::value>::value ||
                  HasCyclicDependency<D4,TL,Contains<TL,D4>::value>::value ||
                  HasCyclicDependency<D5,TL,Contains<TL,D5>::value>::value ||
                  HasCyclicDependency<D6,TL,Contains<TL,D6>::value>::value ||
                  HasCyclicDependency<D7,TL,Contains<TL,D7>::value>::value };
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Specialization of the HasCyclicDependencyHelper class template.
 * \ingroup singleton
 *
 * This specialization of the HasCyclicDependencyHelper class is selected in case the length
 * of the given type list is 8.
 */
template< typename TL   // Type list of checked lifetime dependencies
        , typename D >  // Type list of lifetime dependencies to check
struct HasCyclicDependencyHelper<TL,D,8>
{
   typedef typename TypeAt<D,0>::Result  D1;
   typedef typename TypeAt<D,1>::Result  D2;
   typedef typename TypeAt<D,2>::Result  D3;
   typedef typename TypeAt<D,3>::Result  D4;
   typedef typename TypeAt<D,4>::Result  D5;
   typedef typename TypeAt<D,5>::Result  D6;
   typedef typename TypeAt<D,6>::Result  D7;
   typedef typename TypeAt<D,7>::Result  D8;

   enum { value = HasCyclicDependency<D1,TL,Contains<TL,D1>::value>::value ||
                  HasCyclicDependency<D2,TL,Contains<TL,D2>::value>::value ||
                  HasCyclicDependency<D3,TL,Contains<TL,D3>::value>::value ||
                  HasCyclicDependency<D4,TL,Contains<TL,D4>::value>::value ||
                  HasCyclicDependency<D5,TL,Contains<TL,D5>::value>::value ||
                  HasCyclicDependency<D6,TL,Contains<TL,D6>::value>::value ||
                  HasCyclicDependency<D7,TL,Contains<TL,D7>::value>::value ||
                  HasCyclicDependency<D8,TL,Contains<TL,D8>::value>::value };
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS HASCYCLICDEPENDENCY
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Class template for the detection of cyclic lifetime dependencies.
 * \ingroup singleton
 *
 * This class template checks the given type \a T for cyclic lifetime dependencies. In case a
 * cyclic lifetime dependency is detected, the \a value member enumeration is set to 1. Otherwise
 * it is set to 0.
 */
template< typename T                      // The type to be checked for cyclic lifetime dependencies
        , typename TL                     // Type list of checked lifetime dependencies
        , bool C=Contains<TL,T>::value >  // Flag to indicate whether T is contained in TL
struct HasCyclicDependency
{
   typedef typename Append<TL,T>::Result  ETL;
   enum { value = HasCyclicDependencyHelper<ETL,typename T::Dependencies>::value };
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Specialization of the HasCyclicDependency class template.
 * \ingroup singleton
 *
 * This specialization of the HasCyclicDependency class is selected in case the given type \a T
 * is contained in the given lifetime dependency type list \a TL. In this case a cyclic lifetime
 * dependency was detected and the \a value member enumeration is set to 1.
 */
template< typename T     // The type to be checked for cyclic lifetime dependencies
        , typename TL >  // Type list of checked lifetime dependencies
struct HasCyclicDependency<T,TL,true>
{
   enum { value = 1 };
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS CYCLIC_LIFETIME_DEPENDENCY_TEST
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Compile time constraint wrapper class.
 * \ingroup singleton
 *
 * Helper class for the pe::CYCLIC_LIFETIME_DEPENDENCY_DETECTED class template. This class is
 * used as a wrapper for the instantiation of the pe::CYCLIC_LIFETIME_DEPENDENCY_DETECTED
 * constraint class. It serves the purpose to force the instantiation of either the defined
 * specialization or the undefined basic template during the compilation. In case the compile
 * time condition is met, the type pe::CYCLIC_LIFETIME_DEPENDENCY_TEST<1> is defined.
 */
template< int > struct CYCLIC_LIFETIME_DEPENDENCY_TEST {};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  DETECT_CYCLIC_LIFETIME_DEPENDENCY CONSTRAINT
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Compile time constraint.
 * \ingroup singleton
 *
 * Helper template class for the compile time constraint enforcement. Based on the compile time
 * constant expression used for the template instantiation, either the undefined basic template
 * or the specialization is selected. If the undefined basic template is selected, a compilation
 * error is created.
 */
template< bool > struct CYCLIC_LIFETIME_DEPENDENCY_DETECTED;
template<> struct CYCLIC_LIFETIME_DEPENDENCY_DETECTED<false> { enum { value = 1 }; };
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constraint on the data type.
 * \ingroup singleton
 *
 * In case the given data type \a T is not an integral data type, a compilation error is created.
 */
#define pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY(T) \
   typedef \
      pe::CYCLIC_LIFETIME_DEPENDENCY_TEST< \
         pe::CYCLIC_LIFETIME_DEPENDENCY_DETECTED< pe::HasCyclicDependency<T,pe::NullType>::value >::value > \
      pe_JOIN( DETECT_CYCLIC_LIFETIME_DEPENDENCY_TYPEDEF, __LINE__ )
//*************************************************************************************************




//=================================================================================================
//
//  BEFRIEND_SINGLETON MACRO
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Friendship declaration for the Singleton class template.
 * \ingroup singleton
 *
 * This macro has to be used in order to declare the Singleton functionality as friend of the
 * class deriving from Singleton.
 */
#define pe_BEFRIEND_SINGLETON \
   template< typename, typename, typename, typename, typename, typename, typename, typename, typename > friend class pe::Singleton; \
   template< typename, typename, bool > friend struct pe::HasCyclicDependency; \
   template< typename > friend class pe::Dependency;
//*************************************************************************************************




//=================================================================================================
//
//  CLASS SINGLETON
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup singleton Singleton
 * \ingroup util
 *
 * \section motivation Motivation
 *
 * The singleton design pattern is one of the most popular and most important design patterns
 * available. It can be used to ensures that a specific class has only exactly one instance,
 * and provides a global access point to this instance [1,2]. Additionally, via the singleton
 * pattern it is possible to manage the lifetime of objects, and especially the lifetime
 * dependencies between several objects.\n
 *
 * In the \b pe framework the singleton pattern is realized by the Singleton class template.
 * Classes that are supposed to be implemented in terms of the singleton pattern only have
 * to derive from this class in order to gain all necessary characteristics of a singleton:
 *
 *  - non-copyability via the NonCopyable base class
 *  - a single point of access via the instance() member function
 *  - explicit specification of lifetime dependencies; this feature provides a controlled
 *    order of destruction of all singleton objects depending on a non-cyclic dependency
 *    tree [3,4]
 *  - compile time detection of cyclic lifetime dependencies
 *
 * The only precondition on classes deriving from the Singleton class is the availability of
 * a default constructor. In case it is not possible to instantiate the class via a default
 * constructor, i.e., in case the class has only constructors that require at least a single
 * argument, the \b pe Singleton implementation cannot be used!
 *
 *
 * \section usage Usage of the Singleton
 *
 * In order to make a specific class a singleton, two modifications have to be applied to this
 * class:
 *  -# The class has to derive (publicly or non-publicly) from the Singleton class. In case the
 *     class derives publicly the instance() member function, which the class inherits from the
 *     Singleton class, is publicly accessible and provides a point of access to the singleton
 *     instance. In case the class derives non-publicly, the instance() function is not publicly
 *     accessible and therefore the class has to provide another point of access to the singleton
 *     instance.\n
 *     The first template parameter has to be the class itself. The following template parameters
 *     define lifetime dependencies of this class, i.e., specify on which singleton instances the
 *     class depends. It is possible to specify up to 8 lifetime dependencies. The example below
 *     demonstrates this for the World class, which is solely depending on the Logger class,
 *     which represents the core of the \b pe logging functionality.
 *  -# The class needs to befriend the Singleton via the pe::pe_BEFRIEND_SINGLETON macro. This
 *     macro provides a convenient way to express this friendship relation and works both in
 *     case the class derives publicly or non-publicly from the Singleton class. This friendship
 *     is necessary since in order to guarantee the uniqueness of the singleton instance the
 *     constructor of the deriving class must be declared in a non-public section of the class
 *     definition. However, in order for the Singleton class to provide the instance() function,
 *     the constructor must be accessible. This is achieved by the pe::pe_BEFRIEND_SINGLETON
 *     macro. The following example demonstrates this by means of the World class:

   \code
   class World : private Singleton<World,Logger>
   {
    private:
      World();

      ...
      pe_BEFRIEND_SINGLETON;
      ...
   };
   \endcode

 * \section references References
 *
 * [1]: E. Gamma, R. Helm, R.E. Johnson, J.M. Vlissides: Design Patterns, Addison-Wesley
 *      Professional Computing Series, 2008, ISBN: 978-0-201-63361-0\n
 * [2]: S. Meyers: Effective C++, Third Edition, Addison-Wesley Professional Computing Series,
 *      2008, ISBN: 978-0-321-33487-9\n
 * [3]: A. Alexandrescu: Modern C++ Design, Generic Programming and Design Patterns Applied,
 *      Addison-Wesley, 2001, ISBN: 978-0201704310\n
 * [4]: E. Gabrilovich: Controlling the Destruction Order of Singleton Objects, Dr. Dobbs
 *      (www.drdobbs.com), 1999\n
 */
/*!\brief Base class for all lifetime managed singletons.
 * \ingroup singleton
 *
 * The Singleton class represents the base class for all lifetime managed singletons of the
 * \b pe framework. Classes, which are supposed to be implemented in terms of the singleton
 * pattern, only have to derive from this class in order to gain all basic characteristics
 * of a singleton:
 *
 *  - non-copyability via the NonCopyable base class
 *  - explicit specification of lifetime dependencies; this feature provides a controlled
 *    order of destruction of all singleton objects depending on a non-cyclic dependency
 *    tree
 *  - compile time detection of cyclic lifetime dependencies
 *
 * The only prerequisite for classes deriving from the Singleton class template is the existence
 * of a default constructor. In case no default constructor is available, the \b pe singleton
 * functionality cannot be used!\n
 * Due to the use of the Singleton base class, lifetime dependencies between classes can be
 * expressed very conveniently. The following example demonstrates this by means of the World
 * class, which defines a lifetime dependency on the Logger class, which represents the core
 * of the \b pe logging functionality:

   \code
   // Definition of the World class
   // The world is implemented in terms of the singleton pattern by 
   class World : private Singleton<World,Logger>
   {
    private:
      World();

      ...
      pe_BEFRIEND_SINGLETON;
      ...
   };
   \endcode

 * TODO: Finalize the documentation
 */
template< typename T                // Type of the singleton (CRTP pattern)
        , typename D1 = NullType    // Type of the first lifetime dependency
        , typename D2 = NullType    // Type of the second lifetime dependency
        , typename D3 = NullType    // Type of the third lifetime dependency
        , typename D4 = NullType    // Type of the fourth lifetime dependency
        , typename D5 = NullType    // Type of the fifth lifetime dependency
        , typename D6 = NullType    // Type of the sixth lifetime dependency
        , typename D7 = NullType    // Type of the seventh lifetime dependency
        , typename D8 = NullType >  // Type of the eighth lifetime dependency
class Singleton : private NonCopyable
{
public:
   //**Type definitions****************************************************************************
   //! Type of this Singleton instance.
   typedef Singleton<T,D1,D2,D3,D4,D5,D6,D7,D8>  SingletonType;

   //! Type list of all lifetime dependencies.
   typedef pe_TYPELIST_8( D1, D2, D3, D4, D5, D6, D7, D8 )  Dependencies;
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\brief Constructor for the Singleton class.
   //
   // In case a cyclic lifetime dependency is detected, a compilation error is created.
   */
   explicit Singleton()
      : dependency1_( D1::instance() )  // Handle to the first lifetime dependency
      , dependency2_( D2::instance() )  // Handle to the second lifetime dependency
      , dependency3_( D3::instance() )  // Handle to the third lifetime dependency
      , dependency4_( D4::instance() )  // Handle to the fourth lifetime dependency
      , dependency5_( D5::instance() )  // Handle to the fifth lifetime dependency
      , dependency6_( D6::instance() )  // Handle to the sixth lifetime dependency
      , dependency7_( D7::instance() )  // Handle to the seventh lifetime dependency
      , dependency8_( D8::instance() )  // Handle to the eighth lifetime dependency
   {
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( T, SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D1, typename D1::SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D2, typename D2::SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D3, typename D3::SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D4, typename D4::SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D5, typename D5::SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D6, typename D6::SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D7, typename D7::SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D8, typename D8::SingletonType );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D1 );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D2 );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D3 );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D4 );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D5 );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D6 );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D7 );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D8 );
   }
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\brief Destructor for the Singleton class.
   */
   ~Singleton()
   {}
   //**********************************************************************************************

public:
   //**Instance function***************************************************************************
   /*!\name Instance function */
   //@{
   static boost::shared_ptr<T> instance()
   {
      static boost::shared_ptr<T> object( new T() );
      return object;
   }
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   boost::shared_ptr<D1> dependency1_;  //!< Handle to the first lifetime dependency.
   boost::shared_ptr<D2> dependency2_;  //!< Handle to the second lifetime dependency.
   boost::shared_ptr<D3> dependency3_;  //!< Handle to the third lifetime dependency.
   boost::shared_ptr<D4> dependency4_;  //!< Handle to the fourth lifetime dependency.
   boost::shared_ptr<D5> dependency5_;  //!< Handle to the fifth lifetime dependency.
   boost::shared_ptr<D6> dependency6_;  //!< Handle to the sixth lifetime dependency.
   boost::shared_ptr<D7> dependency7_;  //!< Handle to the seventh lifetime dependency.
   boost::shared_ptr<D8> dependency8_;  //!< Handle to the eighth lifetime dependency.
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  SINGLETON SPECIALIZATION (7 LIFETIME DEPENDENCIES)
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Specialization of the Singleton class for 7 lifetime dependencies.
 * \ingroup singleton
 *
 * This specialization of the Singleton class template is used in case 7 lifetime dependencies
 * are specified.
 */
template< typename T     // Type of the singleton (CRTP pattern)
        , typename D1    // Type of the first lifetime dependency
        , typename D2    // Type of the second lifetime dependency
        , typename D3    // Type of the third lifetime dependency
        , typename D4    // Type of the fourth lifetime dependency
        , typename D5    // Type of the fifth lifetime dependency
        , typename D6    // Type of the sixth lifetime dependency
        , typename D7 >  // Type of the eighth lifetime dependency
class Singleton<T,D1,D2,D3,D4,D5,D6,D7,NullType> : private NonCopyable
{
public:
   //**Type definitions****************************************************************************
   //! Type of this Singleton instance.
   typedef Singleton<T,D1,D2,D3,D4,D5,D6,D7,NullType>  SingletonType;

   //! Type list of all lifetime dependencies.
   typedef pe_TYPELIST_7( D1, D2, D3, D4, D5, D6, D7 )  Dependencies;
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\brief Constructor for the Singleton class.
   //
   // In case a cyclic lifetime dependency is detected, a compilation error is created.
   */
   explicit Singleton()
      : dependency1_( D1::instance() )  // Handle to the first lifetime dependency
      , dependency2_( D2::instance() )  // Handle to the second lifetime dependency
      , dependency3_( D3::instance() )  // Handle to the third lifetime dependency
      , dependency4_( D4::instance() )  // Handle to the fourth lifetime dependency
      , dependency5_( D5::instance() )  // Handle to the fifth lifetime dependency
      , dependency6_( D6::instance() )  // Handle to the sixth lifetime dependency
      , dependency7_( D7::instance() )  // Handle to the seventh lifetime dependency
   {
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( T, SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D1, typename D1::SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D2, typename D2::SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D3, typename D3::SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D4, typename D4::SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D5, typename D5::SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D6, typename D6::SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D7, typename D7::SingletonType );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D1 );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D2 );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D3 );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D4 );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D5 );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D6 );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D7 );
   }
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\brief Destructor for the Singleton class.
   */
   ~Singleton()
   {}
   //**********************************************************************************************

public:
   //**Instance function***************************************************************************
   /*!\name Instance function */
   //@{
   static boost::shared_ptr<T> instance()
   {
      static boost::shared_ptr<T> object( new T() );
      return object;
   }
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   boost::shared_ptr<D1> dependency1_;  //!< Handle to the first lifetime dependency.
   boost::shared_ptr<D2> dependency2_;  //!< Handle to the second lifetime dependency.
   boost::shared_ptr<D3> dependency3_;  //!< Handle to the third lifetime dependency.
   boost::shared_ptr<D4> dependency4_;  //!< Handle to the fourth lifetime dependency.
   boost::shared_ptr<D5> dependency5_;  //!< Handle to the fifth lifetime dependency.
   boost::shared_ptr<D6> dependency6_;  //!< Handle to the sixth lifetime dependency.
   boost::shared_ptr<D7> dependency7_;  //!< Handle to the seventh lifetime dependency.
   //@}
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  SINGLETON SPECIALIZATION (6 LIFETIME DEPENDENCIES)
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Specialization of the Singleton class for 6 lifetime dependencies.
 * \ingroup singleton
 *
 * This specialization of the Singleton class template is used in case 6 lifetime dependencies
 * are specified.
 */
template< typename T     // Type of the singleton (CRTP pattern)
        , typename D1    // Type of the first lifetime dependency
        , typename D2    // Type of the second lifetime dependency
        , typename D3    // Type of the third lifetime dependency
        , typename D4    // Type of the fourth lifetime dependency
        , typename D5    // Type of the fifth lifetime dependency
        , typename D6 >  // Type of the eighth lifetime dependency
class Singleton<T,D1,D2,D3,D4,D5,D6,NullType,NullType> : private NonCopyable
{
public:
   //**Type definitions****************************************************************************
   //! Type of this Singleton instance.
   typedef Singleton<T,D1,D2,D3,D4,D5,D6,NullType,NullType>  SingletonType;

   //! Type list of all lifetime dependencies.
   typedef pe_TYPELIST_6( D1, D2, D3, D4, D5, D6 )  Dependencies;
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\brief Constructor for the Singleton class.
   //
   // In case a cyclic lifetime dependency is detected, a compilation error is created.
   */
   explicit Singleton()
      : dependency1_( D1::instance() )  // Handle to the first lifetime dependency
      , dependency2_( D2::instance() )  // Handle to the second lifetime dependency
      , dependency3_( D3::instance() )  // Handle to the third lifetime dependency
      , dependency4_( D4::instance() )  // Handle to the fourth lifetime dependency
      , dependency5_( D5::instance() )  // Handle to the fifth lifetime dependency
      , dependency6_( D6::instance() )  // Handle to the sixth lifetime dependency
   {
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( T, SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D1, typename D1::SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D2, typename D2::SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D3, typename D3::SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D4, typename D4::SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D5, typename D5::SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D6, typename D6::SingletonType );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D1 );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D2 );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D3 );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D4 );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D5 );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D6 );
   }
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\brief Destructor for the Singleton class.
   */
   ~Singleton()
   {}
   //**********************************************************************************************

public:
   //**Instance function***************************************************************************
   /*!\name Instance function */
   //@{
   static boost::shared_ptr<T> instance()
   {
      static boost::shared_ptr<T> object( new T() );
      return object;
   }
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   boost::shared_ptr<D1> dependency1_;  //!< Handle to the first lifetime dependency.
   boost::shared_ptr<D2> dependency2_;  //!< Handle to the second lifetime dependency.
   boost::shared_ptr<D3> dependency3_;  //!< Handle to the third lifetime dependency.
   boost::shared_ptr<D4> dependency4_;  //!< Handle to the fourth lifetime dependency.
   boost::shared_ptr<D5> dependency5_;  //!< Handle to the fifth lifetime dependency.
   boost::shared_ptr<D6> dependency6_;  //!< Handle to the sixth lifetime dependency.
   //@}
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  SINGLETON SPECIALIZATION (5 LIFETIME DEPENDENCIES)
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Specialization of the Singleton class for 5 lifetime dependencies.
 * \ingroup singleton
 *
 * This specialization of the Singleton class template is used in case 5 lifetime dependencies
 * are specified.
 */
template< typename T     // Type of the singleton (CRTP pattern)
        , typename D1    // Type of the first lifetime dependency
        , typename D2    // Type of the second lifetime dependency
        , typename D3    // Type of the third lifetime dependency
        , typename D4    // Type of the fourth lifetime dependency
        , typename D5 >  // Type of the fifth lifetime dependency
class Singleton<T,D1,D2,D3,D4,D5,NullType,NullType,NullType> : private NonCopyable
{
public:
   //**Type definitions****************************************************************************
   //! Type of this Singleton instance.
   typedef Singleton<T,D1,D2,D3,D4,D5,NullType,NullType,NullType>  SingletonType;

   //! Type list of all lifetime dependencies.
   typedef pe_TYPELIST_5( D1, D2, D3, D4, D5 )  Dependencies;
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\brief Constructor for the Singleton class.
   //
   // In case a cyclic lifetime dependency is detected, a compilation error is created.
   */
   explicit Singleton()
      : dependency1_( D1::instance() )  // Handle to the first lifetime dependency
      , dependency2_( D2::instance() )  // Handle to the second lifetime dependency
      , dependency3_( D3::instance() )  // Handle to the third lifetime dependency
      , dependency4_( D4::instance() )  // Handle to the fourth lifetime dependency
      , dependency5_( D5::instance() )  // Handle to the fifth lifetime dependency
   {
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( T, SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D1, typename D1::SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D2, typename D2::SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D3, typename D3::SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D4, typename D4::SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D5, typename D5::SingletonType );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D1 );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D2 );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D3 );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D4 );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D5 );
   }
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\brief Destructor for the Singleton class.
   */
   ~Singleton()
   {}
   //**********************************************************************************************

public:
   //**Instance function***************************************************************************
   /*!\name Instance function */
   //@{
   static boost::shared_ptr<T> instance()
   {
      static boost::shared_ptr<T> object( new T() );
      return object;
   }
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   boost::shared_ptr<D1> dependency1_;  //!< Handle to the first lifetime dependency.
   boost::shared_ptr<D2> dependency2_;  //!< Handle to the second lifetime dependency.
   boost::shared_ptr<D3> dependency3_;  //!< Handle to the third lifetime dependency.
   boost::shared_ptr<D4> dependency4_;  //!< Handle to the fourth lifetime dependency.
   boost::shared_ptr<D5> dependency5_;  //!< Handle to the fifth lifetime dependency.
   //@}
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  SINGLETON SPECIALIZATION (4 LIFETIME DEPENDENCIES)
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Specialization of the Singleton class for 4 lifetime dependencies.
 * \ingroup singleton
 *
 * This specialization of the Singleton class template is used in case 4 lifetime dependencies
 * are specified.
 */
template< typename T     // Type of the singleton (CRTP pattern)
        , typename D1    // Type of the first lifetime dependency
        , typename D2    // Type of the second lifetime dependency
        , typename D3    // Type of the third lifetime dependency
        , typename D4 >  // Type of the fourth lifetime dependency
class Singleton<T,D1,D2,D3,D4,NullType,NullType,NullType,NullType> : private NonCopyable
{
public:
   //**Type definitions****************************************************************************
   //! Type of this Singleton instance.
   typedef Singleton<T,D1,D2,D3,D4,NullType,NullType,NullType,NullType>  SingletonType;

   //! Type list of all lifetime dependencies.
   typedef pe_TYPELIST_4( D1, D2, D3, D4 )  Dependencies;
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\brief Constructor for the Singleton class.
   //
   // In case a cyclic lifetime dependency is detected, a compilation error is created.
   */
   explicit Singleton()
      : dependency1_( D1::instance() )  // Handle to the first lifetime dependency
      , dependency2_( D2::instance() )  // Handle to the second lifetime dependency
      , dependency3_( D3::instance() )  // Handle to the third lifetime dependency
      , dependency4_( D4::instance() )  // Handle to the fourth lifetime dependency
   {
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( T, SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D1, typename D1::SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D2, typename D2::SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D3, typename D3::SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D4, typename D4::SingletonType );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D1 );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D2 );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D3 );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D4 );
   }
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\brief Destructor for the Singleton class.
   */
   ~Singleton()
   {}
   //**********************************************************************************************

public:
   //**Instance function***************************************************************************
   /*!\name Instance function */
   //@{
   static boost::shared_ptr<T> instance()
   {
      static boost::shared_ptr<T> object( new T() );
      return object;
   }
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   boost::shared_ptr<D1> dependency1_;  //!< Handle to the first lifetime dependency.
   boost::shared_ptr<D2> dependency2_;  //!< Handle to the second lifetime dependency.
   boost::shared_ptr<D3> dependency3_;  //!< Handle to the third lifetime dependency.
   boost::shared_ptr<D4> dependency4_;  //!< Handle to the fourth lifetime dependency.
   //@}
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  SINGLETON SPECIALIZATION (3 LIFETIME DEPENDENCIES)
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Specialization of the Singleton class for 3 lifetime dependencies.
 * \ingroup singleton
 *
 * This specialization of the Singleton class template is used in case 3 lifetime dependencies
 * are specified.
 */
template< typename T     // Type of the singleton (CRTP pattern)
        , typename D1    // Type of the first lifetime dependency
        , typename D2    // Type of the second lifetime dependency
        , typename D3 >  // Type of the third lifetime dependency
class Singleton<T,D1,D2,D3,NullType,NullType,NullType,NullType,NullType> : private NonCopyable
{
public:
   //**Type definitions****************************************************************************
   //! Type of this Singleton instance.
   typedef Singleton<T,D1,D2,D3,NullType,NullType,NullType,NullType,NullType>  SingletonType;

   //! Type list of all lifetime dependencies.
   typedef pe_TYPELIST_3( D1, D2, D3 )  Dependencies;
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\brief Constructor for the Singleton class.
   //
   // In case a cyclic lifetime dependency is detected, a compilation error is created.
   */
   explicit Singleton()
      : dependency1_( D1::instance() )  // Handle to the first lifetime dependency
      , dependency2_( D2::instance() )  // Handle to the second lifetime dependency
      , dependency3_( D3::instance() )  // Handle to the third lifetime dependency
   {
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( T, SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D1, typename D1::SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D2, typename D2::SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D3, typename D3::SingletonType );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D1 );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D2 );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D3 );
   }
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\brief Destructor for the Singleton class.
   */
   ~Singleton()
   {}
   //**********************************************************************************************

public:
   //**Instance function***************************************************************************
   /*!\name Instance function */
   //@{
   static boost::shared_ptr<T> instance()
   {
      static boost::shared_ptr<T> object( new T() );
      return object;
   }
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   boost::shared_ptr<D1> dependency1_;  //!< Handle to the first lifetime dependency.
   boost::shared_ptr<D2> dependency2_;  //!< Handle to the second lifetime dependency.
   boost::shared_ptr<D3> dependency3_;  //!< Handle to the third lifetime dependency.
   //@}
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  SINGLETON SPECIALIZATION (2 LIFETIME DEPENDENCIES)
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Specialization of the Singleton class for 2 lifetime dependencies.
 * \ingroup singleton
 *
 * This specialization of the Singleton class template is used in case 2 lifetime dependencies
 * are specified.
 */
template< typename T     // Type of the singleton (CRTP pattern)
        , typename D1    // Type of the first lifetime dependency
        , typename D2 >  // Type of the second lifetime dependency
class Singleton<T,D1,D2,NullType,NullType,NullType,NullType,NullType,NullType> : private NonCopyable
{
public:
   //**Type definitions****************************************************************************
   //! Type of this Singleton instance.
   typedef Singleton<T,D1,D2,NullType,NullType,NullType,NullType,NullType,NullType>  SingletonType;

   //! Type list of all lifetime dependencies.
   typedef pe_TYPELIST_2( D1, D2 )  Dependencies;
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\brief Constructor for the Singleton class.
   //
   // In case a cyclic lifetime dependency is detected, a compilation error is created.
   */
   explicit Singleton()
      : dependency1_( D1::instance() )  // Handle to the first lifetime dependency
      , dependency2_( D2::instance() )  // Handle to the second lifetime dependency
   {
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( T, SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D1, typename D1::SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D2, typename D2::SingletonType );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D1 );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D2 );
   }
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\brief Destructor for the Singleton class.
   */
   ~Singleton()
   {}
   //**********************************************************************************************

public:
   //**Instance function***************************************************************************
   /*!\name Instance function */
   //@{
   static boost::shared_ptr<T> instance()
   {
      static boost::shared_ptr<T> object( new T() );
      return object;
   }
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   boost::shared_ptr<D1> dependency1_;  //!< Handle to the first lifetime dependency.
   boost::shared_ptr<D2> dependency2_;  //!< Handle to the second lifetime dependency.
   //@}
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  SINGLETON SPECIALIZATION (1 LIFETIME DEPENDENCY)
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Specialization of the Singleton class for a single lifetime dependency.
 * \ingroup singleton
 *
 * This specialization of the Singleton class template is used in case a single lifetime
 * dependency is specified.
 */
template< typename T     // Type of the singleton (CRTP pattern)
        , typename D1 >  // Type of the lifetime dependency
class Singleton<T,D1,NullType,NullType,NullType,NullType,NullType,NullType,NullType> : private NonCopyable
{
public:
   //**Type definitions****************************************************************************
   //! Type of this Singleton instance.
   typedef Singleton<T,D1,NullType,NullType,NullType,NullType,NullType,NullType,NullType>  SingletonType;

   //! Type list of all lifetime dependencies.
   typedef pe_TYPELIST_1( D1 )  Dependencies;
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\brief Constructor for the Singleton class.
   //
   // In case a cyclic lifetime dependency is detected, a compilation error is created.
   */
   explicit Singleton()
      : dependency1_( D1::instance() )  // Handle to the lifetime dependency
   {
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( T, SingletonType );
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( D1, typename D1::SingletonType );
      pe_DETECT_CYCLIC_LIFETIME_DEPENDENCY( D1 );
   }
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\brief Destructor for the Singleton class.
   */
   ~Singleton()
   {}
   //**********************************************************************************************

public:
   //**Instance function***************************************************************************
   /*!\name Instance function */
   //@{
   static boost::shared_ptr<T> instance()
   {
      static boost::shared_ptr<T> object( new T() );
      return object;
   }
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   boost::shared_ptr<D1> dependency1_;  //!< Handle to the lifetime dependency.
   //@}
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  SINGLETON SPECIALIZATION (0 LIFETIME DEPENDENCIES)
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Specialization of the Singleton class for no lifetime dependencies.
 * \ingroup singleton
 *
 * This specialization of the Singleton class template is used in case no lifetime dependencies
 * are specified.
 */
template< typename T >  // Type of the singleton (CRTP pattern)
class Singleton<T,NullType,NullType,NullType,NullType,NullType,NullType,NullType,NullType> : private NonCopyable
{
public:
   //**Type definitions****************************************************************************
   //! Type of this Singleton instance.
   typedef Singleton<T,NullType,NullType,NullType,NullType,NullType,NullType,NullType,NullType>  SingletonType;

   //! Type list of all lifetime dependencies.
   typedef NullType  Dependencies;
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\brief Constructor for the Singleton class.
   //
   // In case a cyclic lifetime dependency is detected, a compilation error is created.
   */
   explicit Singleton()
   {
      pe_CONSTRAINT_MUST_BE_DERIVED_FROM( T, SingletonType );
   }
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\brief Destructor for the Singleton class.
   */
   ~Singleton()
   {}
   //**********************************************************************************************

public:
   //**Instance function***************************************************************************
   /*!\name Instance function */
   //@{
   static boost::shared_ptr<T> instance()
   {
      static boost::shared_ptr<T> object( new T() );
      return object;
   }
   //@}
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************

} // namespace pe

#endif
