//=================================================================================================
/*!
 *  \file src/vtk/Writer.cpp
 *  \brief VTK file writer for the VTK visualization
 *
 *  Copyright (C) 2012 Simon Bogner
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


//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <boost/filesystem/operations.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <boost/version.hpp>
#include <pe/core/MPI.h>
#include <pe/core/rigidbody/Sphere.h>
#include <pe/core/rigidbody/Ellipsoid.h>
#include <pe/core/rigidbody/Box.h>
#include <pe/core/rigidbody/TriangleMesh.h>
#include <pe/core/Serialization.h>
#include <pe/core/TimeStep.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <pe/util/Assert.h>
#include <pe/util/ColorMacros.h>
#include <pe/util/Logging.h>
#include <pe/util/Time.h>
#include <pe/vtk/Writer.h>
#include <pe/vtk/Base64Writer.h>
#include <pe/core/rigidbody/Capsule.h>
#include <vector>


namespace pe {

namespace vtk {

//=================================================================================================
//
//  DEFINITION AND INITIALIZATION OF THE STATIC MEMBER VARIABLES
//
//=================================================================================================

bool Writer::active_( false );
boost::mutex Writer::instanceMutex_;

Vec3 evalSphere(real radius, real phi, real theta) 
{
  return Vec3(
              radius * sin(phi),
              radius * cos(phi) * sin(theta), 
              radius * cos(phi) * cos(theta)
  );
}

//===================================================================================================
// In this function we produce a hemisphere with center at (0,0,0), a top on the x-axis
// and a radius of s. The hemisphere has 65 Vertices and 112 triangles
std::pair< std::vector<Vec3>, std::vector<Vector3<size_t> > >
triangulateSphere(real s) {
  //-Pi/2 to Pi/2
  real phi;
  //0 to 2Pi
  real theta;
  //points on the sphere
  //x=x0+r*cos(theta)*cos(phi)
  //y=y0+r*cos(theta)*sin(phi)
  //z=z0+r*sin(theta)

  std::vector< Vec3 > vVertices;
  std::vector< Vector3<size_t> > vFaces;

  int lat = 8;
  int longi = 8;

  real dphi   = M_PI / (real)longi;
  real dtheta = M_PI / (real)lat;
  real halfpi = M_PI / 2.0;

  Vec3 vTop = evalSphere(s, halfpi, 0);
  Vec3 vBottom = evalSphere(s, -halfpi, 0);
  vVertices.push_back(vTop);

  phi = halfpi - dphi;
  for( int j = 1; j < longi-3; j++ )
  {

    theta = 0.0f;
    for( int i = 0; i < 2 * lat; i++ )
    {
      Vec3 vNext = evalSphere(s, phi, theta);
      vVertices.push_back(vNext);
      theta += dtheta;
    }//end for i
    phi -= dphi;
  }//end for j

  int lat2 = 2 * lat;
  //add upper triangle fan
  for( int i = 0; i < lat2; i++ )
  {
    Vector3<size_t> verts;
    verts[0] = 0;
    verts[1] = 1 + i;
    verts[2] = 1 + ( i + 1 ) % lat2;
    vFaces.push_back(verts);
  }

  //add body
  for( int i = 0; i < longi - 5; i++ )
  {
    int index = 1 + i * lat2;
    for( int j = 0; j < lat2; j++ )
    {
      Vector3<size_t> verts;
      verts[0] = index + j;
      verts[1] = index + lat2 + j;
      verts[2] = index + ( j + 1 ) % lat2;

      vFaces.push_back(verts);
      verts[0] = index + ( j + 1 ) % lat2;
      verts[1] = index + lat2 + j;
      verts[2] = index + lat2 + ( j + 1 ) % lat2;

      vFaces.push_back(verts);
    }
  }

  std::pair < std::vector<Vec3>, std::vector<Vector3<size_t> > > mypair;
  mypair.first = vVertices;
  mypair.second = vFaces;
  return mypair;

}


//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The default constructor of the Writer class.
 *
 * \param filename File name for the VTK visualization file.
 * \param spacing Spacing between two visualized time steps \f$ [1..\infty) \f$.
 * \exception std::invalid_argument Invalid file name.
 */
Writer::Writer( const std::string& filename, unsigned int spacing, unsigned int tstart, unsigned int tend,
                bool binary, bool writeEmptyFiles )
   : Visualization()                // Initialization of the Visualization base object
   , Dependency<logging::Logger>()  // Initialization of the logger lifetime dependency
   , tspacing_ ( spacing )           // Spacing between two visualized time steps
   , tstart_ ( tstart )
   , tend_ ( tend )
   , steps_   ( 0 )                 // Time step counter between two time steps
   , counter_ ( 0 )                 // Visualization counter for the number of visualized time steps
   , filename_(filename)            // The output directory for the VTK files
   , binary_(binary)
   , writeEmptyFiles_(writeEmptyFiles)
   , spheres_ ()                    // Registered spheres
   , boxes_ ()						// Registered boxes
   , capsules_ ()					// Registered capsules
   , cylinders_ ()					// Registered cylinders
   , planes_ ()						// Registered planes
   , meshes_ ()						// Registered meshes
   , springs_ ()					// Registered springs
{
   using namespace boost::filesystem;

   // Setting the active flag
   pe_INTERNAL_ASSERT( !active_, "Multiple constructor calls for a singleton class" );
   active_ = true;

   path p(filename_);
   if( exists(p) ) {
      if( !is_directory(p) ) {
         std::ostringstream oss;
         oss << "VTK output path exists but is not a directory: '" << p.string() << "'";
         throw std::runtime_error( oss.str() );
      }
      //std::cerr << "vtk::Writer::Writer(): Directory exists: "<<p<<". - Files in this directory may be overwritten!\n";
   }
   else {
      const bool created = create_directories(p);
      if( !created ) {
         std::ostringstream oss;
         oss << "Failed to create VTK output directory: '" << p.string()
             << "'. Ensure the parent directory exists and is writable.";
         throw std::runtime_error( oss.str() );
      }
   }

   // Adding the registered visible spheres
   for( Visualization::Spheres::Iterator s=beginSpheres(); s!=endSpheres(); ++s )
      addSphere( *s );

   // Adding the registered visible spheres
   for( Visualization::Boxes::Iterator s=beginBoxes(); s!=endBoxes(); ++s )
      addBox( *s );

   // Adding the registered visible capsules
   for( Visualization::Capsules::Iterator s=beginCapsules(); s!=endCapsules(); ++s )
      addCapsule( *s );

   // Adding the registered visible capsules
   for( Visualization::Meshes::Iterator s=beginMeshes(); s!=endMeshes(); ++s )
      addMesh( *s );

   const bool includeSpheres  = writeEmptyFiles_ || !spheres_.isEmpty();
   const bool includeBoxes    = writeEmptyFiles_ || !boxes_.isEmpty();
   const bool includeCapsules = writeEmptyFiles_ || !capsules_.isEmpty();
   const bool includeMeshes   = writeEmptyFiles_ || !meshes_.isEmpty();
   const int partsPerProc = static_cast<int>(includeSpheres) + static_cast<int>(includeBoxes) +
                            static_cast<int>(includeCapsules) + static_cast<int>(includeMeshes);

   if(MPISettings::rank()==MPISettings::root()) {

      // Write the pvd collector file (Paraview) for all time steps
      std::stringstream pvdfile;
      pvdfile << filename_<<"/collector.pvd";
      std::ofstream pvd( pvdfile.str().c_str(), std::ofstream::out | std::ofstream::trunc );
      if( !pvd.is_open() )
         throw std::invalid_argument( "Invalid file name: " );

      // write header
      pvd << "<?xml version=\"1.0\"?>" << std::endl;
      pvd << "<!-- This pvd file references all the written time steps. -->\n";
      pvd << "<VTKFile type=\"Collection\" version=\"0.1\" byte_order=\"LittleEndian\">" << std::endl;
      pvd << "<Collection>" << std::endl;

      // write filenames of datasets
      int timeCount=0;  // for numbering of the files
      for(unsigned int t=tstart_; t<=tend_; t+=tspacing_,timeCount++)
      {
         for(int proc=0; proc<MPISettings::size(); proc++)
         {
            int part = proc * partsPerProc;
            if( includeSpheres ) {
               pvd<< "<DataSet timestep=\"" <<t<<
                     "\" part=\"" << part++ <<
                     "\" file=\"" << proc << "/spheres" << timeCount <<".vtu\"/>\n";
            }
            if( includeBoxes ) {
               pvd<< "<DataSet timestep=\"" <<t<<
                     "\" part=\"" << part++ <<
                     "\" file=\"" << proc << "/boxes" << timeCount <<".vtu\"/>\n";
            }
            if( includeCapsules ) {
               pvd<< "<DataSet timestep=\"" <<t<<
                     "\" part=\"" << part++ <<
                     "\" file=\"" << proc << "/capsules" << timeCount <<".vtu\"/>\n";
            }
            if( includeMeshes ) {
               pvd<< "<DataSet timestep=\"" <<t<<
                     "\" part=\"" << part++ <<
                     "\" file=\"" << proc << "/meshes" << timeCount <<".vtu\"/>\n";
            }

            //TODO: Write other entries
         }
      }

      // close tags
      pvd << "</Collection>" << std::endl;
      pvd << "</VTKFile>" << std::endl;
      pvd.close();
   }


   if(MPISettings::rank()==MPISettings::root()) {

      // Write the visit collector file (Visit) for all time steps
      std::stringstream visitfile;
      visitfile << filename_<<"/collector.visit";
      std::ofstream visit( visitfile.str().c_str(), std::ofstream::out | std::ofstream::trunc );
      if( !visit.is_open() )
         throw std::invalid_argument( "Invalid file name: " );

      // write header
      visit << "!NBLOCKS " << partsPerProc * MPISettings::size() << std::endl;

      // write filenames of datasets
      int timeCount=0;  // for numbering of the files
      for(unsigned int t=tstart_; t<=tend_; t+=tspacing_,timeCount++)
      {
         for(int proc=0; proc<MPISettings::size(); proc++)
         {
            if( includeSpheres ) {
               visit<< proc << "/spheres" << timeCount <<".vtu\n";
            }
            if( includeBoxes ) {
               visit<< proc << "/boxes" << timeCount <<".vtu\n";
            }
            if( includeCapsules ) {
               visit<< proc << "/capsules" << timeCount <<".vtu\n";
            }
            if( includeMeshes ) {
               visit<< proc << "/meshes" << timeCount <<".vtu\n";
            }

            //TODO: Write other entries
         }
      }

      // close tags
      visit << "</Collection>" << std::endl;
      visit << "</VTKFile>" << std::endl;
      visit.close();
   }



   // Logging the successful setup of the VTK writer
   pe_LOG_PROGRESS_SECTION( log ) {
      log << "Successfully initialized the VTK writer instance";
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The destructor of the Writer class.
 */
Writer::~Writer()
{
   // Logging the successful destruction of the VTK writer
   pe_LOG_PROGRESS_SECTION( log ) {
      log << "Successfully destroyed the VTK writer instance";
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the spacing of the VTK visualization.
 *
 * \param spacing Spacing between two visualized time steps \f$ [1..\infty) \f$.
 * \exception std::invalid_argument Invalid spacing value.
 */
void Writer::setSpacing( unsigned int spacing )
{
   // Checking the spacing value
   if( spacing == 0 )
      throw std::invalid_argument( "Invalid spacing value" );

   tspacing_ = spacing;
}
//*************************************************************************************************




//=================================================================================================
//
//  ADD FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Registering a single sphere for the VTK visualization.
 *
 * \param sphere The sphere to be registered.
 * \return void
 */
void Writer::addSphere( ConstSphereID sphere )
{
   spheres_.pushBack( sphere );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registering a single sphere for the POV-Ray visualization.
 *
 * \param sphere The sphere to be registered.
 * \return void
 */
void Writer::addEllipsoid( ConstEllipsoidID ell )
{
   return;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registering a single box for the VTK visualization.
 *
 * \param box The box to be registered.
 * \return void
 */
void Writer::addBox( ConstBoxID box )
{
   // The Writer is not able to visualize boxes. Therefore the box doesn't
   // have to be registered.
	boxes_.pushBack( box );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registering a single capsule for the VTK visualization.
 *
 * \param capsule The capsule to be registered.
 * \return void
 */
void Writer::addCapsule( ConstCapsuleID capsule )
{
   // The Writer is not able to visualize capsules. Therefore the capsule doesn't
   // have to be registered.
  capsules_.pushBack(capsule);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registering a single cylinder for the VTK visualization.
 *
 * \param cylinder The cylinder to be registered.
 * \return void
 */
void Writer::addCylinder( ConstCylinderID /*cylinder*/ )
{
   // The Writer is not able to visualize cylinders. Therefore the cylinder doesn't
   // have to be registered.
   return;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registering a single cylinder for the VTK visualization.
 *
 * \param cylinder The cylinder to be registered.
 * \return void
 */
void Writer::addInnerCylinder( ConstInnerCylinderID /*cylinder*/ )
{
   // The Writer is not able to visualize cylinders. Therefore the cylinder doesn't
   // have to be registered.
   return;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registering a single plane for the VTK visualization.
 *
 * \param plane The plane to be registered.
 * \return void
 */
void Writer::addPlane( ConstPlaneID /*plane*/ )
{
   // The Writer is not able to visualize planes. Therefore the plane doesn't
   // have to be registered.
   return;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registering a single triangle mesh for the VTK visualization.
 *
 * \param mesh The triangle mesh to be registered.
 * \return void
 */
void Writer::addMesh( ConstTriangleMeshID mesh )
{
   // The Writer is not able to visualize triangle meshes. Therefore the mesh doesn't
   // have to be registered.
   meshes_.pushBack(mesh);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registering a single triangle mesh for the VTK visualization.
 *
 * \param mesh The triangle mesh to be registered.
 * \return void
 */
void Writer::addInnerMesh( ConstInnerMeshID mesh )
{
   // The Writer is not able to visualize triangle meshes. Therefore the mesh doesn't
   // have to be registered.
   return;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registering a single spring for the VTK visualization.
 *
 * \param spring The spring to be registered.
 * \return void
 */
void Writer::addSpring( ConstSpringID /*spring*/ )
{
   // The Writer is not able to visualize springs. Therefore the spring doesn't
   // have to be registered.
   return;
}
//*************************************************************************************************




//=================================================================================================
//
//  REMOVE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Removing a single sphere from the VTK visualization.
 *
 * \param sphere The sphere to be removed.
 * \return void
 */
void Writer::removeSphere( ConstSphereID sphere )
{
   for( Spheres::Iterator pos=spheres_.begin(); pos!=spheres_.end(); ++pos ) {
      if( *pos == sphere ) {
         spheres_.erase( pos );
         return;
      }
   }
   pe_INTERNAL_ASSERT( false, "Sphere is not registered for the VTK visualization" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a single sphere from the POV-Ray visualization.
 *
 * \param sphere The sphere to be removed.
 * \return void
 */
void Writer::removeEllipsoid( ConstEllipsoidID ell )
{
   return;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a single box from the VTK visualization.
 *
 * \param box The box to be removed.
 * \return void
 */
void Writer::removeBox( ConstBoxID box )
{
	   for( Boxes::Iterator pos=boxes_.begin(); pos!=boxes_.end(); ++pos ) {
	      if( *pos == box ) {
	         boxes_.erase( pos );
	         return;
	      }
	   }
	   pe_INTERNAL_ASSERT( false, "Box is not registered for the VTK visualization" );
   return;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a single capsule from the VTK visualization.
 *
 * \param capsule The capsule to be removed.
 * \return void
 */
void Writer::removeCapsule( ConstCapsuleID capsule )
{
   // The Writer is not able to visualize capsules. Therefore the capsule doesn't
   // have to be deregistered.
	 for( Capsules::Iterator pos=capsules_.begin(); pos!=capsules_.end(); ++pos ) {
	    if( *pos == capsule ) {
	       capsules_.erase( pos );
	       return;
	    }
	 }
	 pe_INTERNAL_ASSERT( false, "Capsule is not registered for the VTK visualization" );
   return;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a single cylinder from the VTK visualization.
 *
 * \param cylinder The cylinder to be removed.
 * \return void
 */
void Writer::removeCylinder( ConstCylinderID /*cylinder*/ )
{
   // The Writer is not able to visualize cylinders. Therefore the cylinder doesn't
   // have to be deregistered.
   return;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a single cylinder from the VTK visualization.
 *
 * \param cylinder The cylinder to be removed.
 * \return void
 */
void Writer::removeInnerCylinder( ConstInnerCylinderID /*cylinder*/ )
{
   // The Writer is not able to visualize cylinders. Therefore the cylinder doesn't
   // have to be deregistered.
   return;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a single plane from the VTK visualization.
 *
 * \param plane The plane to be removed.
 * \return void
 */
void Writer::removePlane( ConstPlaneID /*plane*/ )
{
   // The Writer is not able to visualize planes. Therefore the planes doesn't
   // have to be deregistered.
   return;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a single triangle mesh from the VTK visualization.
 *
 * \param mesh The triangle mesh to be removed.
 * \return void
 */
void Writer::removeMesh( ConstTriangleMeshID mesh )
{
   // The Writer is not able to visualize triangle meshes. Therefore the mesh doesn't
   // have to be deregistered.
	for( Meshes::Iterator pos=meshes_.begin(); pos!=meshes_.end(); ++pos ) {
	   if( *pos == mesh ) {
	      meshes_.erase( pos );
	      return;
	   }
	}
	pe_INTERNAL_ASSERT( false, "Mesh is not registered for the VTK visualization" );
   return;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a single triangle mesh from the VTK visualization.
 *
 * \param mesh The triangle mesh to be removed.
 * \return void
 */
void Writer::removeInnerMesh( ConstInnerMeshID mesh )
{
   // The Writer is not able to visualize triangle meshes. Therefore the mesh doesn't
   // have to be deregistered.
   return;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a single spring from the VTK visualization.
 *
 * \param spring The spring to be removed.
 * \return void
 */
void Writer::removeSpring( ConstSpringID /*spring*/ )
{
   // The Writer is not able to visualize springs. Therefore the spring doesn't
   // have to be deregistered.
   return;
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Visualizing the current state of the registered rigid bodies.
 *
 * \return void
 *
 * This function is automatically called every time step. It extends the VTK visualization
 * file by the current state of all registered spheres.
 */
void Writer::trigger()
{

   // Skipping the visualization for intermediate time steps
   if(( ++steps_ < tspacing_ ) && (counter_ != 0)) return;

   // Adjusing the counters
   steps_ = 0;

   std::cerr << "VTK trigger filename_: '" << filename_ << "'"
             << " c_str: " << static_cast<const void*>(filename_.c_str())
             << " size: " << filename_.size()
             << " rank: " << MPISettings::rank()
             << std::endl;

   if( writeEmptyFiles_ || !spheres_.isEmpty() ) {
      std::ostringstream spherefile;
      spherefile << filename_ << "/spheres" << counter_ << ".vtu";
      writeSpheres( spherefile.str().c_str() );
   }

   if( writeEmptyFiles_ || !boxes_.isEmpty() ) {
      std::ostringstream boxfile;
      boxfile << filename_ << "/boxes" << counter_ << ".vtu";
      writeBoxes( boxfile.str().c_str() );
   }

   if( writeEmptyFiles_ || !capsules_.isEmpty() ) {
      std::ostringstream capsulefile;
      capsulefile << filename_ << "/capsules" << counter_ << ".vtu";
      writeCapsules( capsulefile.str().c_str() );
   }

   if( writeEmptyFiles_ || !meshes_.isEmpty() ) {
      std::ostringstream meshfile;
      meshfile << filename_ << "/meshes" << counter_ << ".vtu";
      writeMeshes( meshfile.str().c_str() );
   }

   ++counter_;
}
//*************************************************************************************************




//*************************************************************************************************
/*!\brief Visualizing the current state of the registered spheres.
 *
 * \return void
 *
 * This function creates a new file containing the sphere data of the current timestep
 */
void Writer::writeSpheres(const boost::filesystem::path& filename) const
{
	   using namespace boost::filesystem;
	   using boost::lexical_cast;

	   std::cerr << "VTK writeSpheres path: " << filename.string() << std::endl;

	   // Checking if the function is called inside an exclusive section
	   if( MPISettings::size() > 1 && ExclusiveSection::isActive() ) {
	      throw std::runtime_error( "Invalid function call inside exclusive section" );
	   }

	   // Determining the directory and the filename for the POV-Ray visualization
	   const path directory( filename.parent_path() );
	   const path file     ( filename.filename()    );
	   std::cerr << "VTK writeSpheres dir: " << directory.string()
	             << " file: " << file.string()
	             << " rank: " << MPISettings::rank()
	             << " size: " << MPISettings::size()
	             << std::endl;

	   // Checking the directory and the filename
	   if( !directory.empty() && !exists( directory ) )
	      throw std::runtime_error( "Directory for VTK-Ray files does not exist" );
	   if( file.empty() )
	      throw std::runtime_error( "Invalid file name" );

	   // Generation of a single VTK-Ray file
	   // In case the 'singleFile' flag is set to 'true' all processes append their local, finite
	   // rigid bodies to the main POV-Ray file 'filename'. This task is performed serially in
	   // ascending order.
	   if( false /*singleFile_  || MPISettings::size() == 1 */)
	   {
	      pe_SERIALIZATION
	      {
	    	  throw std::runtime_error( "src/vtk/Writer.cpp::writeSpheres(): not yet implemented");

	         // Opening the output file
	         std::ofstream out( filename.string().c_str(), std::ofstream::out | std::ostream::app );
	         if( !out.is_open() ) {
	            std::ostringstream oss;
	            oss << " Error opening VTK-Ray file '" << filename << "' !\n";
	            throw std::runtime_error( oss.str() );
	         }

	         // Writing the registered spheres
	         for( Spheres::ConstIterator s=spheres_.begin(); s!=spheres_.end(); ++s )
	         {

	         }

	         // Closing the output file
	         out.close();
	      }
	   }

	   // Generation of multiple VTK-Ray files
	   // In case the 'singleFile' flag is set to 'false' each process creates an individual
	   // POV-Ray file (containing their local rigid bodies) that is included in the main POV-Ray
	   // file 'filename'.
	   else
	   {
	      // Creating the process-specific extended directory
	      path extended( directory );
	      extended /= lexical_cast<std::string>( MPISettings::rank() );
	      if( !exists( extended ) )
	         create_directory( extended );
	      extended /= file;
	      std::cerr << "VTK writeSpheres extended path: " << extended.string() << std::endl;

	      // Opening the output file
	      std::ofstream out( extended.string().c_str(), std::ofstream::out | std::ostream::trunc );
	      if( !out.is_open() ) {
	         std::ostringstream oss;
	         oss << " Error opening VTK-Ray file '" << filename << "' !\n";
	         throw std::runtime_error( oss.str() );
	      }

         if(binary_)
         {
            //std::cout << "writing spheres binary";
            writeSphereDataBinary(out);
         }
         else
         {
            //std::cout << "writing spheres ascii";
            writeSphereDataAscii(out);
         }

	      // Closing the output file
	      out.close();
	   }
}
//*************************************************************************************************



void Writer::writeSphereDataBinary(std::ostream& out) const {
   Base64Writer b64(out);

   unsigned int numSpheres = boost::numeric_cast<unsigned int>( spheres_.size() );
   unsigned int data_size_tensor = (4 * sizeof(float) * (numSpheres * 9)
         + 2) / 3;
   unsigned int data_size_vec = (4 * sizeof(float) * (numSpheres * 3) + 2)
         / 3;
   unsigned int data_size_scalar = (4 * sizeof(float) * (numSpheres * 1)
         + 2) / 3;

   // write grid
   out << "<?xml version=\"1.0\"?>\n";
   out
         << "<VTKFile type=\"UnstructuredGrid\" version=\"0.1\" byte_order=\"LittleEndian\">\n";
   out << " <UnstructuredGrid>\n";
   out << "  <Piece NumberOfPoints=\"" << spheres_.size()
         << "\" NumberOfCells=\"" << spheres_.size() << "\">\n";
   out << "   <Points>\n";
   out << "    <DataArray type=\"" << "Float32" << "\" NumberOfComponents=\""
         << 3 << "\" format=\"binary\">\n";

   // write array size first
   b64 << data_size_vec;
   b64.flush();

   // Write the sphere positions
   for (Spheres::ConstIterator s = spheres_.begin(); s != spheres_.end(); ++s) {
      b64 << (float)s->getPosition()[0];
      b64 << (float)s->getPosition()[1];
      b64 << (float)s->getPosition()[2];
   }
   b64.flush();

   // Declare grid as point cloud
   out << "    </DataArray>\n";
   out << "   </Points>\n";
   out << "   <Cells>\n";
   out << "    <DataArray type=\"Int32\" Name=\"connectivity\">\n";
   for (unsigned int i = 0; i < spheres_.size(); i++)
      out << " " << i << "\n";
   out << "    </DataArray>\n";
   out << "    <DataArray type=\"Int32\" Name=\"offsets\">\n";
   for (unsigned int i = 0; i < spheres_.size(); i++)
      out << " " << i + 1 << "\n";
   out << "    </DataArray>\n";
   out << "    <DataArray type=\"UInt8\" Name=\"types\">\n";
   for (unsigned int i = 0; i < spheres_.size(); i++)
      out << " " << 1 << "\n";
   out << "    </DataArray>\n";
   out << "   </Cells>\n";

   // Data at each point
   out << "   <PointData Scalars=\"scalars\" Vectors=\"vectors\">\n";

   // write IDs
   out << "    <DataArray type=\"" << "UInt32" << "\" Name=\"" << "ID"
         << "\" NumberOfComponents=\"" << 1 << "\" format=\"binary\">\n";
   b64 << data_size_scalar;
   b64.flush();
   for (Spheres::ConstIterator s = spheres_.begin(); s != spheres_.end(); ++s) {
      b64 << s->getID();
   } //end for all Spheres
   b64.flush();
   out << "    </DataArray>\n";

   // write Radii
   out << "    <DataArray type=\"" << "Float32" << "\" Name=\"" << "Radius"
         << "\" NumberOfComponents=\"" << 1 << "\" format=\"binary\">\n";
   b64 << data_size_scalar;
   b64.flush();
   for (Spheres::ConstIterator s = spheres_.begin(); s != spheres_.end(); ++s) {
      b64 << (float)s->getRadius();
   } //end for all Spheres
   b64.flush();
   out << "    </DataArray>\n";

   // write Mass
   out << "    <DataArray type=\"" << "Float32" << "\" Name=\"" << "Mass"
         << "\" NumberOfComponents=\"" << 1 << "\" format=\"binary\">\n";
   b64 << data_size_scalar;
   b64.flush();
   for (Spheres::ConstIterator s = spheres_.begin(); s != spheres_.end(); ++s) {
      b64 << (float)s->getMass();
   } //end for all Spheres
   b64.flush();
   out << "    </DataArray>\n";

   // write Orientation
   out << "    <DataArray type=\"" << "Float32" << "\" Name=\""
         << "Orientation" << "\" NumberOfComponents=\"" << 9
         << "\" format=\"binary\">\n";
   b64 << data_size_tensor;
   b64.flush();
   for (Spheres::ConstIterator s = spheres_.begin(); s != spheres_.end(); ++s) {
      const pe::Rot3& rot = s->getRotation();
      //Vector3<Real> o = calcEulerAngles(rot);
      b64 << (float)rot[0];
      b64 << (float)rot[1];
      b64 << (float)rot[2];
      b64 << (float)rot[3];
      b64 << (float)rot[4];
      b64 << (float)rot[5];
      b64 << (float)rot[6];
      b64 << (float)rot[7];
      b64 << (float)rot[8];
   } //end for all Spheres
   b64.flush();
   out << "    </DataArray>\n";

   // write Euler angles
   out << "    <DataArray type=\"" << "Float32" << "\" Name=\""
         << "Euler Rotation" << "\" NumberOfComponents=\"" << 3
         << "\" format=\"binary\">\n";
   b64 << data_size_vec;
   b64.flush();
   for (Spheres::ConstIterator s = spheres_.begin(); s != spheres_.end(); ++s) {
      const pe::Vec3& rot = s->getRotation().getEulerAnglesXYZ();
      //Vector3<Real> o = calcEulerAngles(rot);
      b64 << (float)rot[0];
      b64 << (float)rot[1];
      b64 << (float)rot[2];
      //out << "\t" << o[0] << "\t" << o[1] << "\t" << o[2] << "\n";
   } //end for all Boxes
   b64.flush();
   out << "    </DataArray>\n";

   // write Net Force
   out << "    <DataArray type=\"" << "Float32" << "\" Name=\"" << "Net Force"
         << "\" NumberOfComponents=\"" << 3 << "\" format=\"binary\">\n";
   b64 << data_size_vec;
   b64.flush();
   for (Spheres::ConstIterator s = spheres_.begin(); s != spheres_.end(); ++s) {
      const Vec3 &f = s->getForce();
      b64 << (float)f[0];
      b64 << (float)f[1];
      b64 << (float)f[2];
   } //end for all Spheres
   b64.flush();
   out << "    </DataArray>\n";

   // write Velocsy
   out << "    <DataArray type=\"" << "Float32" << "\" Name=\"" << "Velocity"
         << "\" NumberOfComponents=\"" << 3 << "\" format=\"binary\">\n";
   b64 << data_size_vec;
   b64.flush();
   for (Spheres::ConstIterator s = spheres_.begin(); s != spheres_.end(); ++s) {
      const Vec3 &v = s->getLinearVel();
      b64 << (float)v[0];
      b64 << (float)v[1];
      b64 << (float)v[2];
   } //end for all Spheres
   b64.flush();
   out << "    </DataArray>\n";

   // write angular Velocity
   out << "    <DataArray type=\"" << "Float32" << "\" Name=\""
         << "Angular Velocity" << "\" NumberOfComponents=\"" << 3
         << "\" format=\"binary\">\n";
   b64 << data_size_vec;
   b64.flush();
   for (Spheres::ConstIterator s = spheres_.begin(); s != spheres_.end(); ++s) {
      const Vec3 &v = s->getAngularVel();
      b64 << (float)v[0];
      b64 << (float)v[1];
      b64 << (float)v[2];
   } //end for all Spheres
   b64.flush();
   out << "    </DataArray>\n";

   out << "   </PointData>\n";
   out << "   <CellData>  </CellData>\n";
   out << "  </Piece>\n";
   out << " </UnstructuredGrid>\n";
   out << "</VTKFile>\n";
}





void Writer::writeSphereDataAscii(std::ostream& out) const
{
   // write grid
   out << "<?xml version=\"1.0\"?>\n";
   out << "<VTKFile type=\"UnstructuredGrid\" version=\"0.1\" byte_order=\"LittleEndian\">\n";
   out << " <UnstructuredGrid>\n";
   out << "  <Piece NumberOfPoints=\"" << spheres_.size() <<
          "\" NumberOfCells=\"" << spheres_.size() << "\">\n";
   out << "   <Points>\n";
   out << "    <DataArray type=\"" << "Float32" <<
          "\" NumberOfComponents=\"" << 3 <<
          "\" format=\"ascii\">\n";

     // Write the sphere positions
     for( Spheres::ConstIterator s=spheres_.begin(); s!=spheres_.end(); ++s )
     {
       out << "\t" << s->getPosition()[0] << "\t" << s->getPosition()[1] << "\t" << s->getPosition()[2] << "\n";
     }

     // Declare grid as point cloud
     out << "    </DataArray>\n";
     out << "   </Points>\n";
     out << "   <Cells>\n";
     out << "    <DataArray type=\"Int32\" Name=\"connectivity\">\n";
     for (unsigned int i = 0; i < spheres_.size(); i++)
        out << " " << i << "\n";
     out << "    </DataArray>\n";
     out << "    <DataArray type=\"Int32\" Name=\"offsets\">\n";
     for (unsigned int i = 0; i < spheres_.size(); i++)
        out << " " << i + 1 << "\n";
     out << "    </DataArray>\n";
     out << "    <DataArray type=\"UInt8\" Name=\"types\">\n";
     for (unsigned int i = 0; i < spheres_.size(); i++)
        out << " " << 1 << "\n";
     out << "    </DataArray>\n";
     out << "   </Cells>\n";

     // Data at each point
     out << "   <PointData Scalars=\"scalars\" Vectors=\"vectors\">\n";

     // write IDs
     out << "    <DataArray type=\"" << "UInt64" <<
            "\" Name=\"" << "ID" <<
            "\" NumberOfComponents=\"" << 1 <<
            "\" format=\"ascii\">\n";
     for( Spheres::ConstIterator s=spheres_.begin(); s!=spheres_.end(); ++s )
     {
        out << "\t" << s->getSystemID() << "\n";
     }      //end for all Spheres
     out << "    </DataArray>\n";

     // write Radii
     out << "    <DataArray type=\"" << "Float32" <<
           "\" Name=\"" << "Radius" <<
           "\" NumberOfComponents=\"" << 1 <<
           "\" format=\"ascii\">\n";
     for( Spheres::ConstIterator s=spheres_.begin(); s!=spheres_.end(); ++s )
     {
        out << "\t" << s->getRadius() << "\n";
     }      //end for all Spheres
     out << "    </DataArray>\n";

     // write Mass
     out << "    <DataArray type=\"" << "Float32" <<
            "\" Name=\"" << "Mass" <<
            "\" NumberOfComponents=\"" << 1 <<
            "\" format=\"ascii\">\n";
     for( Spheres::ConstIterator s=spheres_.begin(); s!=spheres_.end(); ++s )
     {
        out << "\t" << s->getMass() << "\n";
     }      //end for all Spheres
     out << "    </DataArray>\n";

     // write Orientation
     out << "    <DataArray type=\"" << "Float32" <<
            "\" Name=\"" << "Orientation" <<
            "\" NumberOfComponents=\"" << 9 <<
            "\" format=\"ascii\">\n";
     for( Spheres::ConstIterator s=spheres_.begin(); s!=spheres_.end(); ++s )
     {
        const pe::Rot3& rot = s->getRotation();
        //Vector3<Real> o = calcEulerAngles(rot);
        out << "\t" << rot[0] << " " << rot[1] << " " << rot[2] <<
               "\t" << rot[3] << " " << rot[4] << " " << rot[5] <<
               "\t" << rot[6] << " " << rot[7] << " " << rot[8] <<"\n";
     }      //end for all Spheres
     out << "    </DataArray>\n";

     // write Euler angles
     out << "    <DataArray type=\"" << "Float32" <<
            "\" Name=\"" << "Euler Rotation" <<
            "\" NumberOfComponents=\"" << 3 <<
            "\" format=\"ascii\">\n";
     for( Spheres::ConstIterator s=spheres_.begin(); s!=spheres_.end(); ++s )
     {
        const pe::Vec3& rot = s->getRotation().getEulerAnglesXYZ();
        //Vector3<Real> o = calcEulerAngles(rot);
        out << "\t" << rot[0] << "\t" << rot[1] << "\t" << rot[2] << "\n";
        //out << "\t" << o[0] << "\t" << o[1] << "\t" << o[2] << "\n";
     }      //end for all Boxes
     out << "    </DataArray>\n";

     // write Net Force
     out << "    <DataArray type=\"" << "Float32" <<
           "\" Name=\"" << "Net Force" <<
           "\" NumberOfComponents=\"" << 3 <<
           "\" format=\"ascii\">\n";
     for( Spheres::ConstIterator s=spheres_.begin(); s!=spheres_.end(); ++s )
     {
        const Vec3 &f = s->getForce();
        out << "\t" << f[0] << "\t" << f[1] << "\t" << f[2] << "\n";
     }      //end for all Spheres
     out << "    </DataArray>\n";

     // write Velocsy
     out << "    <DataArray type=\"" << "Float32" <<
           "\" Name=\"" << "Velocity" <<
           "\" NumberOfComponents=\"" << 3 <<
           "\" format=\"ascii\">\n";
     for( Spheres::ConstIterator s=spheres_.begin(); s!=spheres_.end(); ++s )
     {
        const Vec3 &v = s->getLinearVel();
        out << "\t" << v[0] << "\t" << v[1] << "\t" << v[2] << "\n";
     }      //end for all Spheres
     out << "    </DataArray>\n";

     // write angular Velocity
     out << "    <DataArray type=\"" << "Float32" <<
           "\" Name=\"" << "Angular Velocity" <<
           "\" NumberOfComponents=\"" << 3 <<
           "\" format=\"ascii\">\n";
     for( Spheres::ConstIterator s=spheres_.begin(); s!=spheres_.end(); ++s )
     {
        const Vec3 &v = s->getAngularVel();
        out << "\t" << v[0] << "\t" << v[1] << "\t" << v[2] << "\n";
     }      //end for all Spheres
     out << "    </DataArray>\n";

     out << "   </PointData>\n";
     out << "   <CellData>  </CellData>\n";
     out << "  </Piece>\n";
     out << " </UnstructuredGrid>\n";
     out << "</VTKFile>\n";
}




//*************************************************************************************************
/*!\brief Visualizing the current state of the registered spheres.
 *
 * \return void
 *
 * This function creates a new file containing the sphere data of the current timestep
 */
void Writer::writeBoxes(const boost::filesystem::path& filename) const
{
	   using namespace boost::filesystem;
	   using boost::lexical_cast;

	   std::cerr << "VTK writeBoxes path: " << filename.string() << std::endl;

	   // Checking if the function is called inside an exclusive section
	   if( MPISettings::size() > 1 && ExclusiveSection::isActive() ) {
	      throw std::runtime_error( "Invalid function call inside exclusive section" );
	   }

      // Determining the directory and the filename for the POV-Ray visualization
      const path directory( filename.parent_path() );
      const path file     ( filename.filename()    );

      // Checking the directory and the filename
      if( !directory.empty() && !exists( directory ) )
         throw std::runtime_error( "Directory for VTK-Ray files does not exist" );
      if( file.empty() )
         throw std::runtime_error( "Invalid file name" );

      // Generation of a single VTK-Ray file
      // In case the 'singleFile' flag is set to 'true' all processes append their local, finite
      // rigid bodies to the main POV-Ray file 'filename'. This task is performed serially in
      // ascending order.
      if( false && /*singleFile || */ MPISettings::size() == 1 )
      {
         pe_SERIALIZATION
         {
           throw std::runtime_error( "src/vtk/Writer.cpp::writeSpheres(): not yet implemented");

            // Opening the output file
            std::ofstream out( filename.string().c_str(), std::ofstream::out | std::ostream::app );
            if( !out.is_open() ) {
               std::ostringstream oss;
               oss << " Error opening VTK-Ray file '" << filename << "' !\n";
               throw std::runtime_error( oss.str() );
            }

            // Writing the registered boxes
            for( Boxes::ConstIterator s=boxes_.begin(); s!=boxes_.end(); ++s )
            {

            }

            // Closing the output file
            out.close();
         }
      }

      // Generation of multiple VTK-Ray files
      // In case the 'singleFile' flag is set to 'false' each process creates an individual
      // POV-Ray file (containing their local rigid bodies) that is included in the main POV-Ray
      // file 'filename'.
      else
      {
         // Creating the process-specific extended directory
         path extended( directory );
         extended /= lexical_cast<std::string>( MPISettings::rank() );
         if( !exists( extended ) )
            create_directory( extended );
         extended /= file;

         // Opening the output file
         std::ofstream out( extended.string().c_str(), std::ofstream::out | std::ostream::trunc );
         if( !out.is_open() ) {
            std::ostringstream oss;
            oss << " Error opening VTK-Ray file '" << filename << "' !\n";
            throw std::runtime_error( oss.str() );
         }

         if(binary_) {
            //std::cout << "writing boxes binary";
            writeBoxDataBinary(out);
         }
         else {
            //std::cout << "writing boxes ascii";
            writeBoxDataAscii(out);
         }

         // Closing the output file
         out.close();
      }
}
//*************************************************************************************************



//*************************************************************************************************
void Writer::writeBoxDataAscii(std::ostream& out) const {
   // write grid
   out << "<?xml version=\"1.0\"?>\n";
   out << "<VTKFile type=\"UnstructuredGrid\" version=\"0.1\" byte_order=\"LittleEndian\">\n";
   out << " <UnstructuredGrid>\n";
   out << "  <Piece NumberOfPoints=\"" << boxes_.size() * 8 <<
        "\" NumberOfCells=\"" << boxes_.size() * 6<< "\">\n";
   out << "   <Points>\n";
   out << "    <DataArray type=\"" << "Float32" <<
        "\" NumberOfComponents=\"" << 3 <<
        "\" format=\"ascii\">\n";

     // Write the box positions
     for( Boxes::ConstIterator s=boxes_.begin(); s!=boxes_.end(); ++s )
     {
       Vec3 length = s->getLengths();
       std::vector<Vec3> points;
       points.push_back(Vec3(- 0.5 * length[0], - 0.5 * length[1], - 0.5 * length[2]));
       points.push_back(Vec3(+ 0.5 * length[0], - 0.5 * length[1], - 0.5 * length[2]));
       points.push_back(Vec3(+ 0.5 * length[0], + 0.5 * length[1], - 0.5 * length[2]));
       points.push_back(Vec3(- 0.5 * length[0], + 0.5 * length[1], - 0.5 * length[2]));

       points.push_back(Vec3(- 0.5 * length[0], - 0.5 * length[1], + 0.5 * length[2]));
       points.push_back(Vec3(+ 0.5 * length[0], - 0.5 * length[1], + 0.5 * length[2]));
       points.push_back(Vec3(+ 0.5 * length[0], + 0.5 * length[1], + 0.5 * length[2]));
       points.push_back(Vec3(- 0.5 * length[0], + 0.5 * length[1], + 0.5 * length[2]));

       const pe::Rot3& rot = s->getRotation();
       for (std::vector<Vec3>::size_type idx = 0; idx < points.size(); ++idx) {
         points[idx] = (rot * points[idx]) + s->getPosition();
       }

       out << "\t" << points[0][0] << "\t" << points[0][1] << "\t" << points[0][2] << "\n";
       out << "\t" << points[1][0] << "\t" << points[1][1] << "\t" << points[1][2] << "\n";
       out << "\t" << points[2][0] << "\t" << points[2][1] << "\t" << points[2][2] << "\n";
       out << "\t" << points[3][0] << "\t" << points[3][1] << "\t" << points[3][2] << "\n";
                                                                      
       out << "\t" << points[4][0] << "\t" << points[4][1] << "\t" << points[4][2] << "\n";
       out << "\t" << points[5][0] << "\t" << points[5][1] << "\t" << points[5][2] << "\n";
       out << "\t" << points[6][0] << "\t" << points[6][1] << "\t" << points[6][2] << "\n";
       out << "\t" << points[7][0] << "\t" << points[7][1] << "\t" << points[7][2] << "\n";
     }

     // Declare grid as point cloud
     out << "    </DataArray>\n";
     out << "   </Points>\n";
     out << "   <Cells>\n";
     out << "    <DataArray type=\"Int32\" Name=\"connectivity\">\n";
     unsigned int voff = 0;
     for (unsigned int i = 0; i < boxes_.size(); i++) {
        // face bottom 
        out << " " << 0 + voff << " " << 1 + voff << " " << 2 + voff << " " << 3 + voff << "\n";
        // face top 
        out << " " << 4 + voff << " " << 5 + voff << " " << 6 + voff << " " << 7 + voff << "\n";
        // face right 
        out << " " << 1 + voff << " " << 2 + voff << " " << 6 + voff << " " << 5 + voff << "\n";
        // face left 
        out << " " << 0 + voff << " " << 4 + voff << " " << 7 + voff << " " << 3 + voff << "\n";
        // face front 
        out << " " << 0 + voff << " " << 1 + voff << " " << 5 + voff << " " << 4 + voff << "\n";
        // face back 
        out << " " << 2 + voff << " " << 3 + voff << " " << 7 + voff << " " << 6 + voff << "\n";
        voff += 8;
     }
     out << "    </DataArray>\n";
     out << "    <DataArray type=\"Int32\" Name=\"offsets\">\n";
     voff = 0;
     for (unsigned int i = 0; i < boxes_.size(); i++)
       for(unsigned int j = 0; j < 6; ++j) {
          out << " " << (i * 24) + j * 4 + 4 << "\n";
       }
     out << "    </DataArray>\n";
     out << "    <DataArray type=\"UInt8\" Name=\"types\">\n";
     // Every box has 6 faces
     for (unsigned int i = 0; i < boxes_.size(); i++)
       for(unsigned int j = 0; j < 6; ++j) 
         out << " " << 9 << "\n";
     out << "    </DataArray>\n";
     out << "   </Cells>\n";

//     // Data at each point
//     out << "   <PointData Scalars=\"scalars\" Vectors=\"vectors\">\n";
//
//     // write IDs
//     out << "    <DataArray type=\"" << "UInt32" <<
//            "\" Name=\"" << "ID" <<
//            "\" NumberOfComponents=\"" << 1 <<
//            "\" format=\"ascii\">\n";
//     for( Boxes::ConstIterator s=boxes_.begin(); s!=boxes_.end(); ++s )
//     {
//        out << "\t" << s->getID() << "\n";
//     }      //end for all Boxes
//     out << "    </DataArray>\n";
//
//     // write Radii
//     out << "    <DataArray type=\"" << "Float32" <<
//           "\" Name=\"" << "Lengths" <<
//           "\" NumberOfComponents=\"" << 3 <<
//           "\" format=\"ascii\">\n";
//     for( Boxes::ConstIterator s=boxes_.begin(); s!=boxes_.end(); ++s )
//     {
//        out << "\t" << s->getLengths()[0] << "\t" << s->getLengths()[1] << "\t" << s->getLengths()[2] << "\n";
//     }      //end for all Boxes
//     out << "    </DataArray>\n";
//
//     // write Mass
//     out << "    <DataArray type=\"" << "Float32" <<
//            "\" Name=\"" << "Mass" <<
//            "\" NumberOfComponents=\"" << 1 <<
//            "\" format=\"ascii\">\n";
//     for( Boxes::ConstIterator s=boxes_.begin(); s!=boxes_.end(); ++s )
//     {
//        out << "\t" << s->getMass() << "\n";
//     }      //end for all Boxes
//     out << "    </DataArray>\n";
//
//     // write Orientation
//     out << "    <DataArray type=\"" << "Float32" <<
//            "\" Name=\"" << "Orientation" <<
//            "\" NumberOfComponents=\"" << 9 <<
//            "\" format=\"ascii\">\n";
//     for( Boxes::ConstIterator s=boxes_.begin(); s!=boxes_.end(); ++s )
//     {
//        const pe::Rot3& rot = s->getRotation();
//        //Vector3<Real> o = calcEulerAngles(rot);
//        out << "\t" << rot[0] << " " << rot[1] << " " << rot[2] <<
//               "\t" << rot[3] << " " << rot[4] << " " << rot[5] <<
//               "\t" << rot[6] << " " << rot[7] << " " << rot[8] <<"\n";
//        //out << "\t" << o[0] << "\t" << o[1] << "\t" << o[2] << "\n";
//     }      //end for all Boxes
//     out << "    </DataArray>\n";
//
//     // write Euler angles
//     out << "    <DataArray type=\"" << "Float32" <<
//            "\" Name=\"" << "Euler Rotation" <<
//            "\" NumberOfComponents=\"" << 3 <<
//            "\" format=\"ascii\">\n";
//     for( Boxes::ConstIterator s=boxes_.begin(); s!=boxes_.end(); ++s )
//     {
//        const pe::Vec3& rot = s->getRotation().getEulerAnglesXYZ();
//        //Vector3<Real> o = calcEulerAngles(rot);
//        out << "\t" << rot[0] << "\t" << rot[1] << "\t" << rot[2] << "\n";
//        //out << "\t" << o[0] << "\t" << o[1] << "\t" << o[2] << "\n";
//     }      //end for all Boxes
//     out << "    </DataArray>\n";
//
//     // write Net Force
//     out << "    <DataArray type=\"" << "Float32" <<
//           "\" Name=\"" << "Net Force" <<
//           "\" NumberOfComponents=\"" << 3 <<
//           "\" format=\"ascii\">\n";
//     for( Boxes::ConstIterator s=boxes_.begin(); s!=boxes_.end(); ++s )
//     {
//        const Vec3 &f = s->getForce();
//        out << "\t" << f[0] << "\t" << f[1] << "\t" << f[2] << "\n";
//     }      //end for all Boxes
//     out << "    </DataArray>\n";
//
//     // write Velocsy
//     out << "    <DataArray type=\"" << "Float32" <<
//           "\" Name=\"" << "Velocity" <<
//           "\" NumberOfComponents=\"" << 3 <<
//           "\" format=\"ascii\">\n";
//     for( Boxes::ConstIterator s=boxes_.begin(); s!=boxes_.end(); ++s )
//     {
//        const Vec3 &v = s->getLinearVel();
//        out << "\t" << v[0] << "\t" << v[1] << "\t" << v[2] << "\n";
//     }      //end for all Boxes
//     out << "    </DataArray>\n";
//
//     // write angular Velocity
//     out << "    <DataArray type=\"" << "Float32" <<
//           "\" Name=\"" << "Angular Velocity" <<
//           "\" NumberOfComponents=\"" << 3 <<
//           "\" format=\"ascii\">\n";
//     for( Boxes::ConstIterator s=boxes_.begin(); s!=boxes_.end(); ++s )
//     {
//        const Vec3 &v = s->getAngularVel();
//        out << "\t" << v[0] << "\t" << v[1] << "\t" << v[2] << "\n";
//     }      //end for all Boxes
//     out << "    </DataArray>\n";
//
//     out << "   </PointData>\n";
     out << "   <PointData>  </PointData>\n";
     out << "   <CellData>  </CellData>\n";
     out << "  </Piece>\n";
     out << " </UnstructuredGrid>\n";
     out << "</VTKFile>\n";
}



void Writer::writeBoxDataBinary(std::ostream& out) const {
   Base64Writer b64(out);

   // write grid
   out << "<?xml version=\"1.0\"?>\n";
   out << "<VTKFile type=\"UnstructuredGrid\" version=\"0.1\" byte_order=\"LittleEndian\">\n";
   out << " <UnstructuredGrid>\n";
   out << "  <Piece NumberOfPoints=\"" << boxes_.size() * 8
         << "\" NumberOfCells=\"" << boxes_.size() * 6 << "\">\n";
   out << "   <Points>\n";
   out << "    <DataArray type=\"" << "Float32" << "\" NumberOfComponents=\""
         << 3 << "\" format=\"binary\">\n";

   // write array size first
   unsigned int numPoints = boost::numeric_cast<unsigned int>( boxes_.size() * 8 );
   unsigned int data_size_points = (4 * sizeof(float) * (numPoints * 3) + 2) / 3;
   b64 << data_size_points;
   b64.flush();

   // Write the box vertices (8 per box)
   for( Boxes::ConstIterator s=boxes_.begin(); s!=boxes_.end(); ++s )
   {
      Vec3 length = s->getLengths();
      std::vector<Vec3> points;
      points.push_back(Vec3(- 0.5 * length[0], - 0.5 * length[1], - 0.5 * length[2]));
      points.push_back(Vec3(+ 0.5 * length[0], - 0.5 * length[1], - 0.5 * length[2]));
      points.push_back(Vec3(+ 0.5 * length[0], + 0.5 * length[1], - 0.5 * length[2]));
      points.push_back(Vec3(- 0.5 * length[0], + 0.5 * length[1], - 0.5 * length[2]));

      points.push_back(Vec3(- 0.5 * length[0], - 0.5 * length[1], + 0.5 * length[2]));
      points.push_back(Vec3(+ 0.5 * length[0], - 0.5 * length[1], + 0.5 * length[2]));
      points.push_back(Vec3(+ 0.5 * length[0], + 0.5 * length[1], + 0.5 * length[2]));
      points.push_back(Vec3(- 0.5 * length[0], + 0.5 * length[1], + 0.5 * length[2]));

      const pe::Rot3& rot = s->getRotation();
      for (std::vector<Vec3>::size_type idx = 0; idx < points.size(); ++idx) {
         points[idx] = (rot * points[idx]) + s->getPosition();
         b64 << (float)points[idx][0];
         b64 << (float)points[idx][1];
         b64 << (float)points[idx][2];
      }
   }
   b64.flush();

   // Declare grid as point cloud
   out << "    </DataArray>\n";
   out << "   </Points>\n";
   out << "   <Cells>\n";
   out << "    <DataArray type=\"Int32\" Name=\"connectivity\">\n";
   unsigned int voff = 0;
   for (unsigned int i = 0; i < boxes_.size(); i++) {
      // face bottom
      out << " " << 0 + voff << " " << 1 + voff << " " << 2 + voff << " " << 3 + voff << "\n";
      // face top
      out << " " << 4 + voff << " " << 5 + voff << " " << 6 + voff << " " << 7 + voff << "\n";
      // face right
      out << " " << 1 + voff << " " << 2 + voff << " " << 6 + voff << " " << 5 + voff << "\n";
      // face left
      out << " " << 0 + voff << " " << 4 + voff << " " << 7 + voff << " " << 3 + voff << "\n";
      // face front
      out << " " << 0 + voff << " " << 1 + voff << " " << 5 + voff << " " << 4 + voff << "\n";
      // face back
      out << " " << 2 + voff << " " << 3 + voff << " " << 7 + voff << " " << 6 + voff << "\n";
      voff += 8;
   }
   out << "    </DataArray>\n";
   out << "    <DataArray type=\"Int32\" Name=\"offsets\">\n";
   for (unsigned int i = 0; i < boxes_.size(); i++)
      for(unsigned int j = 0; j < 6; ++j) {
         out << " " << (i * 24) + j * 4 + 4 << "\n";
      }
   out << "    </DataArray>\n";
   out << "    <DataArray type=\"UInt8\" Name=\"types\">\n";
   for (unsigned int i = 0; i < boxes_.size(); i++)
      for(unsigned int j = 0; j < 6; ++j)
         out << " " << 9 << "\n";
   out << "    </DataArray>\n";
   out << "   </Cells>\n";

   out << "   <PointData>  </PointData>\n";
   out << "   <CellData>  </CellData>\n";
   out << "  </Piece>\n";
   out << " </UnstructuredGrid>\n";
   out << "</VTKFile>\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Visualizing the current state of the registered spheres.
 *
 * \return void
 *
 * This function creates a new file containing the sphere data of the current timestep
 */
void Writer::writeCapsules(const boost::filesystem::path& filename) const
{
	   using namespace boost::filesystem;
	   using boost::lexical_cast;

	   std::cerr << "VTK writeCapsules path: " << filename.string() << std::endl;

	   // Checking if the function is called inside an exclusive section
	   if( MPISettings::size() > 1 && ExclusiveSection::isActive() ) {
	      throw std::runtime_error( "Invalid function call inside exclusive section" );
	   }

      // Determining the directory and the filename for the POV-Ray visualization
      const path directory( filename.parent_path() );
      const path file     ( filename.filename()    );

      // Checking the directory and the filename
      if( !directory.empty() && !exists( directory ) )
         throw std::runtime_error( "Directory for VTK-Ray files does not exist" );
      if( file.empty() )
         throw std::runtime_error( "Invalid file name" );

      // Generation of a single VTK-Ray file
      // In case the 'singleFile' flag is set to 'true' all processes append their local, finite
      // rigid bodies to the main POV-Ray file 'filename'. This task is performed serially in
      // ascending order.
      if( false && /*singleFile || */ MPISettings::size() == 1 )
      {
         pe_SERIALIZATION
         {
           throw std::runtime_error( "src/vtk/Writer.cpp::writeSpheres(): not yet implemented");

            // Opening the output file
            std::ofstream out( filename.string().c_str(), std::ofstream::out | std::ostream::app );
            if( !out.is_open() ) {
               std::ostringstream oss;
               oss << " Error opening VTK-Ray file '" << filename << "' !\n";
               throw std::runtime_error( oss.str() );
            }

            // Writing the registered boxes
            for( Boxes::ConstIterator s=boxes_.begin(); s!=boxes_.end(); ++s )
            {

            }

            // Closing the output file
            out.close();
         }
      }

      // Generation of multiple VTK-Ray files
      // In case the 'singleFile' flag is set to 'false' each process creates an individual
      // POV-Ray file (containing their local rigid bodies) that is included in the main POV-Ray
      // file 'filename'.
      else
      {
         // Creating the process-specific extended directory
         path extended( directory );
         extended /= lexical_cast<std::string>( MPISettings::rank() );
         if( !exists( extended ) )
            create_directory( extended );
         extended /= file;

         // Opening the output file
         std::ofstream out( extended.string().c_str(), std::ofstream::out | std::ostream::trunc );
         if( !out.is_open() ) {
            std::ostringstream oss;
            oss << " Error opening VTK-Ray file '" << filename << "' !\n";
            throw std::runtime_error( oss.str() );
         }

         if(binary_) {
            //std::cout << "writing boxes binary";
            writeCapsuleDataBinary(out);
         }
         else {
            //std::cout << "writing boxes ascii";
            writeCapsuleDataAscii(out);
         }

         // Closing the output file
         out.close();
      }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Visualizing the current state of the registered spheres.
 *
 * \return void
 *
 * This function creates a new file containing the sphere data of the current timestep
 */
void Writer::writeMeshes(const boost::filesystem::path& filename) const
{
	   using namespace boost::filesystem;
	   using boost::lexical_cast;

	   std::cerr << "VTK writeMeshes path: " << filename.string() << std::endl;

	   // Checking if the function is called inside an exclusive section
	   if( MPISettings::size() > 1 && ExclusiveSection::isActive() ) {
	      throw std::runtime_error( "Invalid function call inside exclusive section" );
	   }

      // Determining the directory and the filename for the POV-Ray visualization
      const path directory( filename.parent_path() );
      const path file     ( filename.filename()    );

      // Checking the directory and the filename
      if( !directory.empty() && !exists( directory ) )
         throw std::runtime_error( "Directory for VTK-Ray files does not exist" );
      if( file.empty() )
         throw std::runtime_error( "Invalid file name" );

      // Generation of a single VTK-Ray file
      // In case the 'singleFile' flag is set to 'true' all processes append their local, finite
      // rigid bodies to the main POV-Ray file 'filename'. This task is performed serially in
      // ascending order.
      if( false && /*singleFile || */ MPISettings::size() == 1 )
      {
         pe_SERIALIZATION
         {
           throw std::runtime_error( "src/vtk/Writer.cpp::writeMeshes(): not yet implemented");

            // Opening the output file
            std::ofstream out( filename.string().c_str(), std::ofstream::out | std::ostream::app );
            if( !out.is_open() ) {
               std::ostringstream oss;
               oss << " Error opening VTK-Ray file '" << filename << "' !\n";
               throw std::runtime_error( oss.str() );
            }

            // Writing the registered boxes
            for( Boxes::ConstIterator s=boxes_.begin(); s!=boxes_.end(); ++s )
            {

            }

            // Closing the output file
            out.close();
         }
      }

      // Generation of multiple VTK-Ray files
      // In case the 'singleFile' flag is set to 'false' each process creates an individual
      // POV-Ray file (containing their local rigid bodies) that is included in the main POV-Ray
      // file 'filename'.
      else
      {
         // Creating the process-specific extended directory
         path extended( directory );
         extended /= lexical_cast<std::string>( MPISettings::rank() );
         if( !exists( extended ) )
            create_directory( extended );
         extended /= file;

         // Opening the output file
         std::ofstream out( extended.string().c_str(), std::ofstream::out | std::ostream::trunc );
         if( !out.is_open() ) {
            std::ostringstream oss;
            oss << " Error opening VTK-Ray file '" << filename << "' !\n";
            throw std::runtime_error( oss.str() );
         }

         if(binary_) {
            //std::cout << "writing meshes binary";
            writeMeshDataBinary(out);
         }
         else {
            //std::cout << "writing meshes ascii";
            writeMeshDataAscii(out);
         }

         // Closing the output file
         out.close();
      }
}
//*************************************************************************************************

//*************************************************************************************************
void Writer::writeCapsuleDataAscii(std::ostream& out) const {
   // write grid
   int verticalsegments = 2;
   int pointsoncircle   = 16;
   out << "<?xml version=\"1.0\"?>\n";
   out << "<VTKFile type=\"UnstructuredGrid\" version=\"0.1\" byte_order=\"LittleEndian\">\n";
   out << " <UnstructuredGrid>\n";
   out << "  <Piece NumberOfPoints=\"" << capsules_.size() * 2 * pointsoncircle + capsules_.size() * (2. * 65) <<
        "\" NumberOfCells=\"" << capsules_.size() * (2 * pointsoncircle + 224)  << "\">\n";
   out << "   <Points>\n";
   out << "    <DataArray type=\"" << "Float32" <<
        "\" NumberOfComponents=\"" << 3 <<
        "\" format=\"ascii\">\n";

     // The connectivity of the faces is the same for all capsules
     std::vector< Vector3<size_t> > faces;

     //add the body of the cylinder
     for(int j=0;j<pointsoncircle;j++)
     {
       Vector3<size_t> verts;

       // first triangle
       verts[0] = j;
       verts[1] = (j + 1) % pointsoncircle;
       verts[2] = pointsoncircle + j;

       // second triangle
       faces.push_back(verts);
       verts[0] = pointsoncircle + j;
       verts[1] = pointsoncircle + (j + 1) % pointsoncircle;
       verts[2] = (j + 1) % pointsoncircle;
       faces.push_back(verts);
     }

     // Write the capsule triangulation vertices
     unsigned int count(0);
     for( Capsules::ConstIterator s=capsules_.begin(); s != capsules_.end(); ++s, ++count )
     {
       
       Vec3 center = s->getPosition();
       
       std::pair < std::vector<Vec3>, std::vector<Vector3<size_t> > > mypair = triangulateSphere(s->getRadius());

       // 65 vertices
       std::vector<Vec3>& hemisVerts = mypair.first;
       // 112 cells
       std::vector<Vector3<size_t> >& hemisFaces = mypair.second;

       std::vector<Vec3> points;
       
       // The height and the radius
       real height  = s->getLength();
       real height2 = s->getLength() * 0.5;
       real rad     = s->getRadius();

       real dalpha = 2.0 * M_PI/(real)pointsoncircle;

       Vec3 u(0, 0, 1);
       Vec3 vTop    = (height2 * u);
       Vec3 vBottom =-(height2 * u);

       real dheight = (2.0 * height2)/real(verticalsegments+1);
       real currentheight = height2;
       real alpha = 0.0;

       //create the top vertices on the cylinder
       for(int i=0;i<pointsoncircle;i++)
       {
         Vec3 next = Vec3( height2 , rad * cos(alpha), rad * sin(alpha));
         points.push_back(next);
         alpha+=dalpha;
       }

       //create the bottom vertices on the cylinder
       for(int i=0;i<pointsoncircle;i++)
       {
         Vec3 next = Vec3(-height2 , rad * cos(alpha), rad * sin(alpha));
         points.push_back(next);
         alpha+=dalpha;
       }

       const pe::Rot3& rot = s->getRotation();
       for (std::vector<Vec3>::size_type idx = 0; idx < points.size(); ++idx) {
         points[idx] = (rot * points[idx]) + s->getPosition();
         out << "\t" << points[idx][0] << "\t" << points[idx][1] << "\t" << points[idx][2] << "\n";
       }

       for (std::vector<Vec3>::size_type idx = 0; idx < hemisVerts.size(); ++idx) {
         Vec3 point = (rot * (hemisVerts[idx] + Vec3(height2,0,0))) + s->getPosition();
         out << "\t" << point[0] << "\t" << point[1] << "\t" << point[2] << "\n";
       }

       for (std::vector<Vec3>::size_type idx = 0; idx < hemisVerts.size(); ++idx) {
         Vec3 point = (rot * (-hemisVerts[idx] - Vec3(height2,0,0))) + s->getPosition();
         out << "\t" << point[0] << "\t" << point[1] << "\t" << point[2] << "\n";
       }

     }

     // Write each capsule as a set of faces 
     out << "    </DataArray>\n";
     out << "   </Points>\n";
     out << "   <Cells>\n";
     out << "    <DataArray type=\"Int32\" Name=\"connectivity\">\n";
     unsigned int voff = 0;
     for (unsigned int i = 0; i < capsules_.size(); i++) {

       ConstCapsuleID s = capsules_[i];
       std::pair < std::vector<Vec3>, std::vector<Vector3<size_t> > > mypair = triangulateSphere(s->getRadius());

       // 65 vertices
       std::vector<Vec3>& hemisVerts = mypair.first;
       // 112 cells
       std::vector<Vector3<size_t> >& hemisFaces = mypair.second;

       // Connectivity of the cylinder faces 
       for( unsigned int j = 0; j < faces.size(); j++ ) {
          out << " " << faces[j][0] + voff << " " << faces[j][1] + voff << " " << faces[j][2] + voff << "\n";
       }
       voff += 2 * pointsoncircle;
       // Connectivity of the 1st hemisphere faces 
       for( unsigned int j = 0; j < hemisFaces.size(); j++ ) {
          out << " " << hemisFaces[j][0] + voff << " " << hemisFaces[j][1] + voff << " " << hemisFaces[j][2] + voff << "\n";
       }
       voff += hemisVerts.size();;

       // Connectivity of the 2nd hemisphere faces 
       for( unsigned int j = 0; j < hemisFaces.size(); j++ ) {
          out << " " << hemisFaces[j][0] + voff << " " << hemisFaces[j][1] + voff << " " << hemisFaces[j][2] + voff << "\n";
       }
       voff += hemisVerts.size();;
     }
     out << "    </DataArray>\n";
     out << "    <DataArray type=\"Int32\" Name=\"offsets\">\n";
     voff = 0;
     for( unsigned int i = 0; i < capsules_.size(); i++ ) {
       for( unsigned int j = 0; j < faces.size() + 224; j++ ) {
         voff += 3;
         out << " " << voff << "\n";
       }
     }
     out << "    </DataArray>\n";
     out << "    <DataArray type=\"UInt8\" Name=\"types\">\n";

     // Every capsule has 2 * pointsoncircle triangles 
     for (unsigned int i = 0; i < capsules_.size(); i++)
       for( unsigned int j = 0; j < faces.size() + 224; j++ ) {
         out << " " << 5 << "\n";
       }
     out << "    </DataArray>\n";
     out << "   </Cells>\n";

//     // Data at each point
//     out << "   <PointData Scalars=\"scalars\" Vectors=\"vectors\">\n";
//
//     // write IDs
//     out << "    <DataArray type=\"" << "UInt32" <<
//            "\" Name=\"" << "ID" <<
//            "\" NumberOfComponents=\"" << 1 <<
//            "\" format=\"ascii\">\n";
//     for( Boxes::ConstIterator s=boxes_.begin(); s!=boxes_.end(); ++s )
//     {
//        out << "\t" << s->getID() << "\n";
//     }      //end for all Boxes
//     out << "    </DataArray>\n";
//
//     // write Radii
//     out << "    <DataArray type=\"" << "Float32" <<
//           "\" Name=\"" << "Lengths" <<
//           "\" NumberOfComponents=\"" << 3 <<
//           "\" format=\"ascii\">\n";
//     for( Boxes::ConstIterator s=boxes_.begin(); s!=boxes_.end(); ++s )
//     {
//        out << "\t" << s->getLengths()[0] << "\t" << s->getLengths()[1] << "\t" << s->getLengths()[2] << "\n";
//     }      //end for all Boxes
//     out << "    </DataArray>\n";
//
//     // write Mass
//     out << "    <DataArray type=\"" << "Float32" <<
//            "\" Name=\"" << "Mass" <<
//            "\" NumberOfComponents=\"" << 1 <<
//            "\" format=\"ascii\">\n";
//     for( Boxes::ConstIterator s=boxes_.begin(); s!=boxes_.end(); ++s )
//     {
//        out << "\t" << s->getMass() << "\n";
//     }      //end for all Boxes
//     out << "    </DataArray>\n";
//
//     // write Orientation
//     out << "    <DataArray type=\"" << "Float32" <<
//            "\" Name=\"" << "Orientation" <<
//            "\" NumberOfComponents=\"" << 9 <<
//            "\" format=\"ascii\">\n";
//     for( Boxes::ConstIterator s=boxes_.begin(); s!=boxes_.end(); ++s )
//     {
//        const pe::Rot3& rot = s->getRotation();
//        //Vector3<Real> o = calcEulerAngles(rot);
//        out << "\t" << rot[0] << " " << rot[1] << " " << rot[2] <<
//               "\t" << rot[3] << " " << rot[4] << " " << rot[5] <<
//               "\t" << rot[6] << " " << rot[7] << " " << rot[8] <<"\n";
//        //out << "\t" << o[0] << "\t" << o[1] << "\t" << o[2] << "\n";
//     }      //end for all Boxes
//     out << "    </DataArray>\n";
//
//     // write Euler angles
//     out << "    <DataArray type=\"" << "Float32" <<
//            "\" Name=\"" << "Euler Rotation" <<
//            "\" NumberOfComponents=\"" << 3 <<
//            "\" format=\"ascii\">\n";
//     for( Boxes::ConstIterator s=boxes_.begin(); s!=boxes_.end(); ++s )
//     {
//        const pe::Vec3& rot = s->getRotation().getEulerAnglesXYZ();
//        //Vector3<Real> o = calcEulerAngles(rot);
//        out << "\t" << rot[0] << "\t" << rot[1] << "\t" << rot[2] << "\n";
//        //out << "\t" << o[0] << "\t" << o[1] << "\t" << o[2] << "\n";
//     }      //end for all Boxes
//     out << "    </DataArray>\n";
//
//     // write Net Force
//     out << "    <DataArray type=\"" << "Float32" <<
//           "\" Name=\"" << "Net Force" <<
//           "\" NumberOfComponents=\"" << 3 <<
//           "\" format=\"ascii\">\n";
//     for( Boxes::ConstIterator s=boxes_.begin(); s!=boxes_.end(); ++s )
//     {
//        const Vec3 &f = s->getForce();
//        out << "\t" << f[0] << "\t" << f[1] << "\t" << f[2] << "\n";
//     }      //end for all Boxes
//     out << "    </DataArray>\n";
//
//     // write Velocsy
//     out << "    <DataArray type=\"" << "Float32" <<
//           "\" Name=\"" << "Velocity" <<
//           "\" NumberOfComponents=\"" << 3 <<
//           "\" format=\"ascii\">\n";
//     for( Boxes::ConstIterator s=boxes_.begin(); s!=boxes_.end(); ++s )
//     {
//        const Vec3 &v = s->getLinearVel();
//        out << "\t" << v[0] << "\t" << v[1] << "\t" << v[2] << "\n";
//     }      //end for all Boxes
//     out << "    </DataArray>\n";
//
//     // write angular Velocity
//     out << "    <DataArray type=\"" << "Float32" <<
//           "\" Name=\"" << "Angular Velocity" <<
//           "\" NumberOfComponents=\"" << 3 <<
//           "\" format=\"ascii\">\n";
//     for( Boxes::ConstIterator s=boxes_.begin(); s!=boxes_.end(); ++s )
//     {
//        const Vec3 &v = s->getAngularVel();
//        out << "\t" << v[0] << "\t" << v[1] << "\t" << v[2] << "\n";
//     }      //end for all Boxes
//     out << "    </DataArray>\n";
//
//     out << "   </PointData>\n";
     out << "   <PointData>  </PointData>\n";
     out << "   <CellData>  </CellData>\n";
     out << "  </Piece>\n";
     out << " </UnstructuredGrid>\n";
     out << "</VTKFile>\n";
}
//*************************************************************************************************




//*************************************************************************************************
void Writer::writeCapsuleDataBinary(std::ostream& out) const {
   Base64Writer b64(out);
   int pointsoncircle   = 16;

   // write grid
   out << "<?xml version=\"1.0\"?>\n";
   out << "<VTKFile type=\"UnstructuredGrid\" version=\"0.1\" byte_order=\"LittleEndian\">\n";
   out << " <UnstructuredGrid>\n";
   unsigned int numPoints = 0;
   unsigned int numCells = 0;
   for( Capsules::ConstIterator s=capsules_.begin(); s != capsules_.end(); ++s )
   {
      std::pair< std::vector<Vec3>, std::vector<Vector3<size_t> > > mypair = triangulateSphere(s->getRadius());
      const unsigned int hemisVerts = boost::numeric_cast<unsigned int>( mypair.first.size() );
      const unsigned int hemisFaces = boost::numeric_cast<unsigned int>( mypair.second.size() );
      numPoints += 2 * pointsoncircle + 2 * hemisVerts;
      numCells += 2 * pointsoncircle + 2 * hemisFaces;
   }
   out << "  <Piece NumberOfPoints=\"" << numPoints
         << "\" NumberOfCells=\"" << numCells << "\">\n";
   out << "   <Points>\n";
   out << "    <DataArray type=\"" << "Float32" << "\" NumberOfComponents=\""
         << 3 << "\" format=\"binary\">\n";

   // write array size first
   unsigned int data_size_points = (4 * sizeof(float) * (numPoints * 3) + 2) / 3;
   b64 << data_size_points;
   b64.flush();

   // The connectivity of the faces is the same for all capsules
   std::vector< Vector3<size_t> > faces;

   //add the body of the cylinder
   for(int j=0;j<pointsoncircle;j++)
   {
     Vector3<size_t> verts;

     // first triangle
     verts[0] = j;
     verts[1] = (j + 1) % pointsoncircle;
     verts[2] = pointsoncircle + j;

     // second triangle
     faces.push_back(verts);
     verts[0] = pointsoncircle + j;
     verts[1] = pointsoncircle + (j + 1) % pointsoncircle;
     verts[2] = (j + 1) % pointsoncircle;
     faces.push_back(verts);
   }

   for( Capsules::ConstIterator s=capsules_.begin(); s != capsules_.end(); ++s )
   {
      std::pair< std::vector<Vec3>, std::vector<Vector3<size_t> > > mypair = triangulateSphere(s->getRadius());
      std::vector<Vec3>& hemisVerts = mypair.first;

      std::vector<Vec3> points;

      // The height and the radius
      real height2 = s->getLength() * 0.5;
      real rad     = s->getRadius();

      real dalpha = 2.0 * M_PI/(real)pointsoncircle;
      real alpha = 0.0;

      //create the top vertices on the cylinder
      for(int i=0;i<pointsoncircle;i++)
      {
        Vec3 next = Vec3( height2 , rad * cos(alpha), rad * sin(alpha));
        points.push_back(next);
        alpha+=dalpha;
      }

      //create the bottom vertices on the cylinder
      for(int i=0;i<pointsoncircle;i++)
      {
        Vec3 next = Vec3(-height2 , rad * cos(alpha), rad * sin(alpha));
        points.push_back(next);
        alpha+=dalpha;
      }

      const pe::Rot3& rot = s->getRotation();
      for (std::vector<Vec3>::size_type idx = 0; idx < points.size(); ++idx) {
        points[idx] = (rot * points[idx]) + s->getPosition();
        b64 << (float)points[idx][0];
        b64 << (float)points[idx][1];
        b64 << (float)points[idx][2];
      }

      for (std::vector<Vec3>::size_type idx = 0; idx < hemisVerts.size(); ++idx) {
        Vec3 point = (rot * (hemisVerts[idx] + Vec3(height2,0,0))) + s->getPosition();
        b64 << (float)point[0];
        b64 << (float)point[1];
        b64 << (float)point[2];
      }

      for (std::vector<Vec3>::size_type idx = 0; idx < hemisVerts.size(); ++idx) {
        Vec3 point = (rot * (-hemisVerts[idx] - Vec3(height2,0,0))) + s->getPosition();
        b64 << (float)point[0];
        b64 << (float)point[1];
        b64 << (float)point[2];
      }
   }
   b64.flush();

   // Declare grid as point cloud
   out << "    </DataArray>\n";
   out << "   </Points>\n";
   out << "   <Cells>\n";
   out << "    <DataArray type=\"Int32\" Name=\"connectivity\">\n";
   unsigned int voff = 0;
   for (unsigned int i = 0; i < capsules_.size(); i++) {
      ConstCapsuleID s = capsules_[i];
      std::pair< std::vector<Vec3>, std::vector<Vector3<size_t> > > mypair = triangulateSphere(s->getRadius());
      std::vector<Vector3<size_t> >& hemisFaces = mypair.second;
      std::vector<Vec3>& hemisVerts = mypair.first;

      // Connectivity of the cylinder faces
      for( unsigned int j = 0; j < faces.size(); j++ ) {
         out << " " << faces[j][0] + voff << " " << faces[j][1] + voff << " " << faces[j][2] + voff << "\n";
      }
      voff += 2 * pointsoncircle;

      // Connectivity of the 1st hemisphere faces
      for( unsigned int j = 0; j < hemisFaces.size(); j++ ) {
         out << " " << hemisFaces[j][0] + voff << " " << hemisFaces[j][1] + voff << " " << hemisFaces[j][2] + voff << "\n";
      }
      voff += hemisVerts.size();

      // Connectivity of the 2nd hemisphere faces
      for( unsigned int j = 0; j < hemisFaces.size(); j++ ) {
         out << " " << hemisFaces[j][0] + voff << " " << hemisFaces[j][1] + voff << " " << hemisFaces[j][2] + voff << "\n";
      }
      voff += hemisVerts.size();
   }
   out << "    </DataArray>\n";
   out << "    <DataArray type=\"Int32\" Name=\"offsets\">\n";
   unsigned int offset = 0;
   for( unsigned int i = 0; i < capsules_.size(); i++ ) {
      ConstCapsuleID s = capsules_[i];
      std::pair< std::vector<Vec3>, std::vector<Vector3<size_t> > > mypair = triangulateSphere(s->getRadius());
      std::vector<Vector3<size_t> >& hemisFaces = mypair.second;
      const unsigned int cellsPerCapsule = boost::numeric_cast<unsigned int>(faces.size() + 2 * hemisFaces.size());
      for( unsigned int j = 0; j < cellsPerCapsule; j++ ) {
         offset += 3;
         out << " " << offset << "\n";
      }
   }
   out << "    </DataArray>\n";
   out << "    <DataArray type=\"UInt8\" Name=\"types\">\n";
   for (unsigned int i = 0; i < capsules_.size(); i++) {
      ConstCapsuleID s = capsules_[i];
      std::pair< std::vector<Vec3>, std::vector<Vector3<size_t> > > mypair = triangulateSphere(s->getRadius());
      std::vector<Vector3<size_t> >& hemisFaces = mypair.second;
      const unsigned int cellsPerCapsule = boost::numeric_cast<unsigned int>(faces.size() + 2 * hemisFaces.size());
      for( unsigned int j = 0; j < cellsPerCapsule; j++ ) {
         out << " " << 5 << "\n";
      }
   }
   out << "    </DataArray>\n";
   out << "   </Cells>\n";

   out << "   <PointData>  </PointData>\n";
   out << "   <CellData>  </CellData>\n";
   out << "  </Piece>\n";
   out << " </UnstructuredGrid>\n";
   out << "</VTKFile>\n";
}
//*************************************************************************************************


//*************************************************************************************************
void Writer::writeMeshDataAscii(std::ostream& out) const {

   int numPoints = 0;
   int numCells = 0;

   // Loop to get the total number of vertices and cells
   for (Meshes::ConstIterator m = meshes_.begin(); m != meshes_.end(); ++m) {
     numPoints += m->getBFVertices().size(); 
     numCells += m->getFaceIndices().size(); 
   }

   out << "<?xml version=\"1.0\"?>\n";
   out << "<VTKFile type=\"UnstructuredGrid\" version=\"0.1\" byte_order=\"LittleEndian\">\n";
   out << " <UnstructuredGrid>\n";
   out << "  <Piece NumberOfPoints=\"" << numPoints <<
        "\" NumberOfCells=\"" << numCells  << "\">\n";
   out << "   <Points>\n";
   out << "    <DataArray type=\"" << "Float32" <<
        "\" NumberOfComponents=\"" << 3 <<
        "\" format=\"ascii\">\n";

   std::vector<int> offsets;
   for (Meshes::ConstIterator m = meshes_.begin(); m != meshes_.end(); ++m) {
      Vec3 pos = m->getPosition();
      Quat q = m->getQuaternion();
      for (size_t i(0); i < m->getBFVertices().size(); ++i) {
        Vec3 v = m->getBFVertices()[i];
        //rotation
        v = q.rotate(v);
        //translation
        v += pos;
        out << "\t" << v[0] << "\t" << v[1] << "\t" << v[2] << "\n";

      }

     offsets.push_back(m->getBFVertices().size());
   }

   out << "    </DataArray>\n";
   //writeDoubles(outfile, coordinates, "Coordinates", 3);
   out << "  </Points>\n";

   // Write the cells
   out << "      <Cells>\n";
   out << "    <DataArray type=\"Int32\" Name=\"connectivity\">\n";
   int voff = 0;
   for (size_t i = 0; i < meshes_.size(); ++i) {
      ConstTriangleMeshID m = meshes_[i];
      for (size_t j(0); j < m->getFaceIndices().size(); ++j) {
        out << " " << m->getFaceIndices()[j][0] + voff << " " << m->getFaceIndices()[j][1] + voff << " " << m->getFaceIndices()[j][2] + voff << "\n";
      }

      voff += m->getBFVertices().size();
   }
   out << "    </DataArray>\n";

   // the offset array  is used to specify the starting index of each cell in the connectivity array 
   out << "    <DataArray type=\"Int32\" Name=\"offsets\">\n";
   int triCount = 3;
   for(int i(0); i < numCells; ++i) {
      out << " " << triCount << "\n";
      triCount += 3;
   }

   out << "    </DataArray>\n";

   out << "    <DataArray type=\"UInt8\" Name=\"types\">\n";
   for(int i(0); i < numCells; ++i) {
      out << " " << 5 << "\n";
   }
   out << "    </DataArray>\n";

   out << "      </Cells>\n";

   out << "    </Piece>\n";
   out << "  </UnstructuredGrid>\n";
   out << "</VTKFile>\n";

}
//*************************************************************************************************


//*************************************************************************************************
void Writer::writeMeshDataBinary(std::ostream& out) const {
   Base64Writer b64(out);

   unsigned int numPoints = 0;
   unsigned int numCells = 0;

   for (Meshes::ConstIterator m = meshes_.begin(); m != meshes_.end(); ++m) {
     numPoints += boost::numeric_cast<unsigned int>( m->getBFVertices().size() );
     numCells += boost::numeric_cast<unsigned int>( m->getFaceIndices().size() );
   }

   out << "<?xml version=\"1.0\"?>\n";
   out << "<VTKFile type=\"UnstructuredGrid\" version=\"0.1\" byte_order=\"LittleEndian\">\n";
   out << " <UnstructuredGrid>\n";
   out << "  <Piece NumberOfPoints=\"" << numPoints
        << "\" NumberOfCells=\"" << numCells  << "\">\n";
   out << "   <Points>\n";
   out << "    <DataArray type=\"" << "Float32" <<
        "\" NumberOfComponents=\"" << 3 <<
        "\" format=\"binary\">\n";

   unsigned int data_size_points = (4 * sizeof(float) * (numPoints * 3) + 2) / 3;
   b64 << data_size_points;
   b64.flush();

   for (Meshes::ConstIterator m = meshes_.begin(); m != meshes_.end(); ++m) {
      Vec3 pos = m->getPosition();
      Quat q = m->getQuaternion();
      for (size_t i(0); i < m->getBFVertices().size(); ++i) {
        Vec3 v = m->getBFVertices()[i];
        v = q.rotate(v);
        v += pos;
        b64 << (float)v[0];
        b64 << (float)v[1];
        b64 << (float)v[2];
      }
   }
   b64.flush();

   out << "    </DataArray>\n";
   out << "  </Points>\n";

   out << "      <Cells>\n";
   out << "    <DataArray type=\"Int32\" Name=\"connectivity\">\n";
   int voff = 0;
   for (size_t i = 0; i < meshes_.size(); ++i) {
      ConstTriangleMeshID m = meshes_[i];
      for (size_t j(0); j < m->getFaceIndices().size(); ++j) {
        out << " " << m->getFaceIndices()[j][0] + voff << " " << m->getFaceIndices()[j][1] + voff << " " << m->getFaceIndices()[j][2] + voff << "\n";
      }

      voff += m->getBFVertices().size();
   }
   out << "    </DataArray>\n";

   out << "    <DataArray type=\"Int32\" Name=\"offsets\">\n";
   int triCount = 0;
   for(int i(0); i < static_cast<int>(numCells); ++i) {
      triCount += 3;
      out << " " << triCount << "\n";
   }

   out << "    </DataArray>\n";

   out << "    <DataArray type=\"UInt8\" Name=\"types\">\n";
   for(int i(0); i < static_cast<int>(numCells); ++i) {
      out << " " << 5 << "\n";
   }
   out << "    </DataArray>\n";

   out << "      </Cells>\n";

   out << "    </Piece>\n";
   out << "  </UnstructuredGrid>\n";
   out << "</VTKFile>\n";
}
//*************************************************************************************************

//   virtual void writeMeshDataAscii(std::ostream& out) const;
//   virtual void writeMeshDataBinary(std::ostream& out) const;

//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the current state of an VTK writer.
 *
 * \param os Reference to the output stream.
 * \return void
 */
void Writer::print( std::ostream& os ) const
{
   os << " VTK file                     : '" << filename_ << "'\n"
      << " Number of time steps         : "  << counter_  << "\n"
      << " Number of registered spheres : "  << spheres_.size() << "\n"
      << " Number of registered boxes   : "  << boxes_.size() << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  VTK WRITER SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Activation of the VTK writer.
 * \ingroup vtk
 *
 * \param filename File name for the VTK visualization file.
 * \param spacing Spacing between two visualized time steps \f$ [1..\infty) \f$.
 * \return Handle to the active VTK writer.
 * \exception std::invalid_argument Invalid spacing value.
 * \exception std::invalid_argument Invalid file name.
 *
 * This function activates the VTK writer for an VTK visualization. The first call to
 * this function will activate the VTK writer and return the handle to the active writer,
 * subsequent calls will ignore the parameters and only return the handle to the VTK writer.
 */
WriterID activateWriter( const std::string& filename, unsigned int spacing, unsigned int tstart, unsigned int tend,
                         bool binary, bool writeEmptyFiles )
{
   boost::mutex::scoped_lock lock( Writer::instanceMutex_ );
   if( filename.empty() ) {
      throw std::invalid_argument( "VTK writer requires a non-empty output path" );
   }
   static WriterID dx( new Writer( filename, spacing, tstart, tend, binary, writeEmptyFiles ) );
   return dx;
}
//*************************************************************************************************




//=================================================================================================
//
//  OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for VTK writers.
 * \ingroup vtk
 *
 * \param os Reference to the output stream.
 * \param dx Reference to a constant VTK writer object.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, const Writer& dx )
{
   os << "--" << pe_BROWN << "VTK FILE WRITER" << pe_OLDCOLOR
      << "------------------------------------------------------------\n";
   dx.print( os );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for VTK writer handles.
 * \ingroup vtk
 *
 * \param os Reference to the output stream.
 * \param dx VTK writer handle.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, const WriterID& dx )
{
   os << "--" << pe_BROWN << "VTK FILE WRITER" << pe_OLDCOLOR
      << "------------------------------------------------------------\n";
   dx->print( os );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for constant VTK writer handles.
 * \ingroup vtk
 *
 * \param os Reference to the output stream.
 * \param dx Constant VTK writer handle.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, const ConstWriterID& dx )
{
   os << "--" << pe_BROWN << "VTK FILE WRITER" << pe_OLDCOLOR
      << "------------------------------------------------------------\n";
   dx->print( os );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************

} // namespace vtk

} // namespace pe
