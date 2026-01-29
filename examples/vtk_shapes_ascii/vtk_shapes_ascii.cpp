//=================================================================================================
/*!
 *  \file vtk_shapes_ascii.cpp
 *  \brief Example emitting one VTK ASCII frame for each supported shape
 */
//=================================================================================================

//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>

//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iostream>
#include <string>
#include <pe/core.h>
#include <pe/support.h>
#include <pe/vtk.h>

using namespace pe;

//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================

int main( int /*argc*/, char* /*argv*/[] )
{
   const unsigned int timesteps ( 0 );
   const unsigned int visspacing( 1 );
   unsigned int id( 0 );

   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -0.4 );

   // Ground plane to keep objects near origin.
   createPlane( id++, 0.0, 0.0, 1.0, 0.0, granite );

   // One of each VTK-supported shape.
   createSphere( ++id, -0.5, 0.0, 0.8, 0.1, granite );
   createBox( ++id, 0.5, 0.0, 0.8, 0.2, 0.1, 0.15, iron );
   createCapsule( ++id, 0.0, 0.5, 0.8, 0.08, 0.4, oak );

   const std::string meshFile(std::string(VTK_SHAPES_ASCII_SRC_DIR) + "/cube.obj");
   createTriangleMesh( ++id, Vec3(0.0, -0.6, 0.8), meshFile, granite, true, true,
                       Vec3(1.0, 1.0, 1.0), false, false );

   // Binary VTK output.
   vtk::WriterID vtkw = vtk::activateWriter( "./paraview_vtk_shapes_binary", visspacing, 0, timesteps, true );
   (void)vtkw;

   std::cout << "Running single simulation step for VTK ASCII output..." << std::endl;
   world->simulationStep( 0.004 );

   return 0;
}
//*************************************************************************************************
