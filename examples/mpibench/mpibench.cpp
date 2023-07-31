//=================================================================================================
/*!
 *  \file MPINano.cpp
 *  \brief Example file for the pe physics engine
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2012 Tobias Preclik
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

#include <pe/engine.h>
#include <pe/support.h>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <vector>
#include <pe/vtk.h>
#include <boost/filesystem.hpp>
#include <pe/util/Checkpointer.h>
#include <pe/interface/decompose.h>
#include <random>
#include <algorithm>
using namespace pe;
using namespace pe::timing;
using namespace pe::povray;
using boost::filesystem::path;




// Assert statically that only the FFD solver or a hard contact solver is used since parameters are tuned for them.
#define pe_CONSTRAINT_MUST_BE_EITHER_TYPE(A, B, C) typedef \
   pe::CONSTRAINT_TEST< \
      pe::CONSTRAINT_MUST_BE_SAME_TYPE_FAILED< \
         pe::IsSame<A,B>::value | pe::IsSame<A,C>::value \
      >::value > \
   pe_JOIN( CONSTRAINT_MUST_BE_SAME_TYPE_TYPEDEF, __LINE__ );

typedef Configuration< pe_COARSE_COLLISION_DETECTOR, pe_FINE_COLLISION_DETECTOR, pe_BATCH_GENERATOR, response::HardContactSemiImplicitTimesteppingSolvers>::Config TargetConfig2;
typedef Configuration< pe_COARSE_COLLISION_DETECTOR, pe_FINE_COLLISION_DETECTOR, pe_BATCH_GENERATOR, response::HardContactAndFluid>::Config TargetConfig3;
pe_CONSTRAINT_MUST_BE_EITHER_TYPE(Config, TargetConfig2, TargetConfig3);



//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================



//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns a random angle between \f$ [-\frac{\pi}{20}..\frac{\pi}{20}] \f$.
 *
 * \return The random angle (radian measure).
 */
real angle()
{
   return rand<real>( -M_PI/real(20), M_PI/real(20) );
}
//*************************************************************************************************

real generateRandomNumber() {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dis(-0.01, 0.01);
  return dis(gen);
}

//*************************************************************************************************
/*!\brief Main function for the mpinano example.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return Success of the simulation.
 *
 * Particles in a non-periodic box with non-zero initial velocities.
 */
int main( int argc, char** argv )
{
   /////////////////////////////////////////////////////
   // MPI Initialization

   MPI_Init( &argc, &argv );


   /////////////////////////////////////////////////////
   // Simulation parameters

   const real   spacing (real(0.5)*0.1 + 0.0025 );  // Initial spacing in-between two spheres
   const real   spacingx   (  1.5  );  // Initial spacing inbetween two spherical particles
   const real   spacingy   (  1.5  );  // Initial spacing inbetween two spherical particles
   const real   spacingz   (  0.6  );  // Initial spacing inbetween two spherical particles
   const real   velocity  (  0.02 );  // Initial maximum velocity of the spherical particles

   const size_t timesteps ( 1000 );  // Total number of time steps
   const real   stepsize  ( 0.005 );  // Size of a single time step

   const size_t seed      ( 12345 );  // Seed for the random number generation

   bool   povray    ( false );        // Switches the POV-Ray visualization on and off
   bool   vtk( true );
   const size_t visspacing(    10 );  // Number of time steps inbetween two POV-Ray files

   const bool   strong    ( false );  // Compile time switch between strong and weak scaling

   const bool spheres     ( false );  // Switch between spheres and granular particles
   bool resume     ( false );

   //*************************************************************************************************
   // The Checkpointer
   path                 checkpoint_path( "checkpoints/" );            // The path where to store the checkpointing data
   Checkpointer checkpointer = Checkpointer(checkpoint_path, visspacing, 0, timesteps);
   std::string startFile;

   /////////////////////////////////////////////////////
   // Initial setups
   real radiusx = 4 * 0.1;
   real sphereRad = Vec3(0.5 * radiusx, 0.5 * radiusx, 0.5 * radiusx).length();

   // Parsing the command line arguments
   CommandLineInterface& cli = CommandLineInterface::getInstance();
   cli.getDescription().add_options()
      ("processes", value< std::vector<int> >()->multitoken()->required(), "number of processes in x-, y- and z-dimension")
      ("startFile", value< std::string >(), "name of the start .peb file")
   ;
   cli.parse( argc, argv );
   cli.evaluateOptions();
   variables_map& vm = cli.getVariablesMap();

   if( vm.count( "no-povray" ) > 0 )
      povray = false;

   if( vm.count( "startFile" ) ) {
     startFile = vm["startFile"].as<std::string>();
     resume = true;
   }

   const int px( vm[ "processes" ].as< std::vector<int> >()[0] );
   const int py( vm[ "processes" ].as< std::vector<int> >()[1] );
   const int pz( vm[ "processes" ].as< std::vector<int> >()[2] );

   if( px <= 0 ) {
      std::cerr << " Invalid number of processes in x-dimension!\n";
      pe::exit( EXIT_FAILURE );
   }
   if( py <= 0 ) {
      std::cerr << " Invalid number of processes in y-dimension!\n";
      pe::exit( EXIT_FAILURE );
   }
   if( pz <= 0 ) {
      std::cerr << " Invalid number of processes in z-dimension!\n";
      pe::exit( EXIT_FAILURE );
   }

   if( MPISettings::size() < px*py*pz ) {
      std::cerr << " Number of available processes is smaller than the number of processes specified on the command line." << std::endl;
      pe::exit(EXIT_FAILURE);
   }

   setSeed( seed );  // Setup of the random number generation

   // Creates the material "myMaterial" with the following material properties:
   //  - material density               : 2.54
   //  - coefficient of restitution     : 0.8
   //  - coefficient of static friction : 0.1
   //  - coefficient of dynamic friction: 0.05
   //  - Poisson's ratio                : 0.2
   //  - Young's modulus                : 80
   //  - Contact stiffness              : 100
   //  - dampingN                       : 10
   //  - dampingT                       : 11
   //MaterialID myMaterial = createMaterial( "myMaterial", 2.54, 0.8, 0.1, 0.05, 0.2, 80, 100, 10, 11 );
   MaterialID elastic = createMaterial( "elastic", 1.0, 1.0, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5 );

   WorldID     world     = theWorld();
   world->setGravity( 0.0, 0.0,-1.0 );
   world->setDamping(0.50);
   world->setViscosity(3e-3);
   MPISystemID mpisystem = theMPISystem();


   /////////////////////////////////////////////////////
   // Setup of the MPI processes: 3D Regular Domain Decomposition

   int dims   [] = { px   , py   , pz    };
   int periods[] = { false, false, false };

   int rank;           // Rank of the neighboring process
   int center[3];      // Definition of the coordinates array 'center'
   MPI_Comm cartcomm;  // The new MPI communicator with Cartesian topology

   MPI_Cart_create( MPI_COMM_WORLD, 3, dims, periods, false, &cartcomm );
   if( cartcomm == MPI_COMM_NULL ) {
      MPI_Finalize();
      return 0;
   }

   mpisystem->setComm( cartcomm );
   MPI_Cart_coords( cartcomm, mpisystem->getRank(), 3, center );

   int my_cart_rank;
   MPI_Cart_rank(cartcomm, center, &my_cart_rank);

//===========================================================================================================

   const real dx( 0.4 );
   const real dy( 0.4 );
   const real dz( 0.4 );

   real bx =-0.4;
   real by =-0.4;
   real bz = 0.0;

   decomposeDomain(center, bx, by, bz, dx, dy, dz, px, py, pz);

//===========================================================================================================

//#ifndef NDEBUG
   // Checking the process setup
   mpisystem->checkProcesses();
//#endif

   // Setup of the VTK visualization
   if( vtk ) {
      vtk::WriterID vtkw = vtk::activateWriter( "./paraview", visspacing, 0, timesteps, false);
   }

//===========================================================================================================

   // Starting the time measurement for the setup
   WcTimer setupTime;
   setupTime.start();

//=========================================================================================
   MaterialID myMaterial = createMaterial("Bench", 1.0, 0.0, 0.1, 0.05, 0.2, 80, 100, 10, 11);

   int idx = 0;
   real h = 0.0075;
   real radius2 = 0.02;
//   47
//   real radius2 = 0.012;
//   Vec3 position( 0.25,-0.25, 0.15);
//   const double initialRadius = 0.22;
//   const double distance = 2.3 * radius2;
//   const double angDistance = 2.05 * radius2;
//   const int numIterations = 8;
//   const double radiusIncrement = radius2 + 0.42 * distance;
//   int hZ[] = {16, 16, 16, 16, 16, 16, 16, 16, 16};

   Vec3 position( 0.25,-0.25, 0.15);
   const double initialRadius = 0.222;
   const double distance = 2.3 * radius2;
   const double angDistance = 2.125 * radius2;
   const int numIterations = 3;
   const double radiusIncrement = radius2 + 0.52 * distance;
   int hZ[] = {9, 8, 7, 19, 19, 19, 19, 19, 19, 19, 19};
   
   int count(0);
   if(!resume) {
     for (int iteration = 0; iteration < numIterations; ++iteration) {

       double radius = initialRadius + iteration * radiusIncrement;
       int numPoints = static_cast<int>(2 * M_PI * radius / angDistance);
       double angleIncrement = 2 * M_PI / numPoints;        
       real pos_z = radius2 + 0.1 * h;
       real zinc = 2. * radius2 + 0.001;

   pe_EXCLUSIVE_SECTION( 0 ) {
     std::cout << "\n--" << "numPoints" << numPoints << std::endl;
   }
        int height = hZ[iteration];
        int j = 0;
        for (; j < height; ++j) {
          for (int i = 0; i < numPoints; ++i) {
            double angle = i * angleIncrement;
            position[0] = radius * std::cos(angle);
            position[1] = radius * std::sin(angle);
            position[2] = pos_z + j * zinc;
              if (world->ownsPoint( position )) {
                SphereID sphere = createSphere(idx, position, radius2, myMaterial, true);

//                real x = generateRandomNumber(); 
//                real y = generateRandomNumber(); 
//                sphere->setLinearVel(x, y, 0);
                ++idx;      
              }
            count++;
          }
        }        
      }
    }
    else {
      checkpointer.read( "../start.1" );
      int i(0);
      for (; i < theCollisionSystem()->getBodyStorage().size(); i++) {
         World::SizeType widx = static_cast<World::SizeType>(i);
         BodyID body = world->getBody(static_cast<unsigned int>(widx));
         if(body->getType() == sphereType) {
            body->setAngularVel(Vec3(0,0,0));
            body->setLinearVel(Vec3(0,0,0));
         }
         else {
            std::cout << "Found type: " << body->getType() << std::endl;
         }
      }

    }

  //=========================================================================================
  CylinderID cyl(0);
  pe_GLOBAL_SECTION
  {
    cyl = createCylinder( 10011, 0.0, 0.0, 0.2, 0.20, 0.4, iron );
    cyl->setFixed(true);
    cyl->rotate(0.0, M_PI/2.0 , 0.0);

    InnerCylinderID cyl2(0);
    cyl2 = createInnerCylinder( 10012, 0.0, 0.0, 0.2, 0.40, 0.4, iron );
    cyl2->setFixed(true);
    cyl2->rotate(0.0, M_PI/2.0, 0.0);
  }
  //=========================================================================================

   theCollisionSystem()->setErrorReductionParameter(0.1);
   // Synchronization of the MPI processes
   world->synchronize();

   // Ending the time measurement for the setup
   setupTime.end();

   const real cylRad1 = 0.2;  
   const real cylRad2 = 0.4;  
   const real cylLength  = 0.4;

   real domainVol = M_PI * std::pow(cylRad2, 2) * cylLength;
   real cylVol = M_PI * std::pow(cylRad1, 2) * cylLength;
   domainVol -= cylVol;

   //=========================================================================================  

   // Calculating the total number of particles and primitives
   unsigned long particlesTotal ( 0 );
   unsigned long primitivesTotal( 0 );
   unsigned long bla = idx;
   int numBodies =  theCollisionSystem()->getBodyStorage().size();
   unsigned long bodiesUpdate = static_cast<unsigned long>(numBodies);
   MPI_Reduce( &bla, &particlesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm );
   MPI_Reduce( &bodiesUpdate, &primitivesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm );

   if (resume)
     particlesTotal = primitivesTotal;

   real partVol = 4./3. * M_PI * std::pow(radius2, 3);

   pe_EXCLUSIVE_SECTION( 0 ) {
     std::cout << "\n--" << "SIMULATION SETUP"
       << "--------------------------------------------------------------\n"
       << " Total number of MPI processes           = " << px * py * pz << "\n"
       << " Total particles          = " << particlesTotal << "\n"
       << " particle volume          = " << partVol << "\n"
       << " Domain volume            = " << domainVol << "\n"
       << " Volume fraction[%]       = " << (particlesTotal * partVol)/domainVol * 100.0 << "\n"
       << " Total objects            = " << primitivesTotal << "\n" << std::endl;
      std::cout << "--------------------------------------------------------------------------------\n" << std::endl;
   }


   /////////////////////////////////////////////////////
   // Setup timing results

   double localTime( setupTime.total() );
   double globalMin( 0.0 );
   double globalMax( 0.0 );
   MPI_Reduce( &localTime, &globalMin, 1, MPI_DOUBLE, MPI_MIN, 0, cartcomm );
   MPI_Reduce( &localTime, &globalMax, 1, MPI_DOUBLE, MPI_MAX, 0, cartcomm );

   /////////////////////////////////////////////////////
   // Simulation loop

   WcTimer simTime;
   simTime.start();
   //world->run( timesteps, stepsize );

   for( unsigned int timestep=0; timestep <= timesteps; ++timestep ) {
     pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "\r Time step " << timestep << " of " << timesteps << "   " << std::flush;
     }
     world->simulationStep( stepsize );

     if(timestep % visspacing == 0) {
       // Write out checkpoint
       checkpointer.setPath( checkpoint_path );
       checkpointer.write( "checkpoint");
     }
   }



   simTime.end();



   /////////////////////////////////////////////////////
   // Simulation timing results

   localTime  = simTime.total();
   MPI_Reduce( &localTime, &globalMin, 1, MPI_DOUBLE, MPI_MIN, 0, cartcomm );
   MPI_Reduce( &localTime, &globalMax, 1, MPI_DOUBLE, MPI_MAX, 0, cartcomm );

   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "\n--" << pe_BROWN << "SIMULATION RESULTS" << pe_OLDCOLOR << "----------------------------------------------------------\n"
                << " Minimum total WC-Time = " << globalMin << "\n"
                << " Maximum total WC-Time = " << globalMax << "\n"
                << "------------------------------------------------------------------------------\n" << std::endl;
   }


   /////////////////////////////////////////////////////
   // MPI Finalization
   MPI_Barrier(MPI_COMM_WORLD);
   MPI_Finalize();
}
//*************************************************************************************************
