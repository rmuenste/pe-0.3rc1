//=================================================================================================
/*!
 *  \file BoxStack.cpp
 *  \brief Example file for the pe physics engine
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


//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes 1 Termin im Nov, 1 Termin im Dez.
// 4 Dokumente, Master/Bacherlor Tech/math  + Wima Master/Bachelor
// Hinweisblätter
//*************************************************************************************************

#include <cstring>
#include <iostream>
#include <sstream>
#include <pe/core.h>
#include <pe/support.h>
#include <pe/vtk.h>
#include <pe/util.h>
#include <pe/core/rigidbody/TriangleMesh.h>
#include <random>
#include <algorithm>


#include <fstream>    // For file handling
#include <vector>     // For std::vector
#include <string>     // For std::string

using namespace pe;
using namespace pe::povray;


//*************************************************************************************************
// Irrlicht includes
//*************************************************************************************************

#include <pe/irrlicht.h>
#if HAVE_IRRLICHT
using namespace pe::irrlicht;
#endif




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

real sign()
{
   real val = rand<real>( -M_PI/real(20), M_PI/real(20) );
   return (val > 0) ? 1.0 : -1.0; 
}


std::vector<Vec3> readVectorsFromFile(const std::string& fileName) {
    std::vector<Vec3> vectors;
    std::ifstream file(fileName);
    
    // Check if the file was successfully opened
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file " << fileName << std::endl;
        return vectors; // Return an empty vector in case of error
    }
    
    std::string line;
    
    // Read the file line by line
    while (std::getline(file, line)) {
        std::stringstream ss(line);  // Create a string stream from the line
        float x, y, z;
        
        // Parse the line for three float values
        if (ss >> x >> y >> z) {
            // Create a Vec3 object and add it to the vector
            vectors.emplace_back(x, y, z);
        }
    }
    
    file.close();  // Close the file
    return vectors;
}

//=================================================================================================
// GenerateRandomPositions
//=================================================================================================
// Function to generate random positions within a cubic domain
std::vector<Vec3> generateRandomPositions(real L, real diameter, real volumeFraction, real eps) {

    WorldID world = theWorld();

    std::vector<Vec3> positions;

    real cellSize = diameter + eps;

    real partVol = 4./3. * M_PI * std::pow(0.5 * diameter, 3);
    real domainVol = L * L * L;

    std::cout << "Trying to generate volume fraction:  " << volumeFraction * 100.0 << std::endl;

    // Calculate the number of cells along one side of the cubic grid
    int gridSize = static_cast<int>(L / cellSize);

    // Calculate the total number of cells in the grid
    int totalCells = gridSize * gridSize * gridSize;

    // Calculate the maximum volume fraction possible for the current configuration
    real maxPhi = ((totalCells * partVol) / domainVol);

    if (volumeFraction > maxPhi) {
      std::cout << "User defined volume fraction: " << volumeFraction << " is too high for the current configuration (max: )" << maxPhi << std::endl;
      std::exit(EXIT_FAILURE);
    }

    // Initialize a vector to keep track of visited cells
    std::vector<bool> cellVisited(totalCells, false);

    // Initialize random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<real> dis(-cellSize / 2.0, cellSize / 2.0);

    // Generate random positions until the grid is full or the volume fraction is reached
    //while (positions.size() < totalCells && (static_cast<real>(positions.size()) / totalCells) < volumeFraction) {
    while (positions.size() < totalCells && (partVol * positions.size() / domainVol) < volumeFraction) {
    //while (positions.size() < totalCells && (static_cast<real>(positions.size()) * partVol) / domainVol < volumeFraction) {
        // Generate random cell indices
        int x = std::uniform_int_distribution<int>(0, gridSize - 1)(gen);
        int y = std::uniform_int_distribution<int>(0, gridSize - 1)(gen);
        int z = std::uniform_int_distribution<int>(0, gridSize - 1)(gen);

        int cellIndex = x + y * gridSize + z * gridSize * gridSize;
        
        // Calculate the position of the cell center
        real posX = (x + 0.5) * cellSize;
        real posY = (y + 0.5) * cellSize;
        real posZ = (z + 0.5) * cellSize;

        Vec3 gpos(posX, posY, posZ);

        // Check if the cell has not been visited
        //if (!cellVisited[cellIndex] && world->ownsPoint( gpos ) ) {
        if (!cellVisited[cellIndex]) {
            // Mark the cell as visited
            cellVisited[cellIndex] = true;

            // Calculate the position of the cell center
            double posX = (x + 0.5) * cellSize;
            double posY = (y + 0.5) * cellSize;
            double posZ = (z + 0.5) * cellSize;

            // Create a Vec3 object for the position and add it to the positions vector
            positions.push_back(Vec3(posX, posY, posZ));
        }
    real solidFraction = (partVol * positions.size() / domainVol) * 100.0;
//    std::cout << MPISettings::rank() << ")local fraction:  " << (static_cast<real>(positions.size()) / totalCells) << " of " << volumeFraction << std::endl;
//    std::cout << MPISettings::rank() << ")local:  " << positions.size() << " of " << totalCells << std::endl;
    }

    real solidFraction = (partVol * positions.size() / domainVol) * 100.0;
    std::cout << MPISettings::rank() << ")Volume fraction:  " << solidFraction << std::endl;
    std::cout << MPISettings::rank() << ")local:  " << positions.size() << std::endl;
    return positions;
}

// Function to load planes from file and create HalfSpace instances
void loadPlanesAndCreateHalfSpaces(const std::string& filename, std::vector<HalfSpace> &halfSpaces) {
    std::ifstream file(filename);
    std::string line;
    
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << "\n";
        return;
    }

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        double px, py, pz, nx, ny, nz;
        
        // Read the plane's point and normal from the line
        if (!(iss >> px >> py >> pz >> nx >> ny >> nz)) {
            std::cerr << "Error: Malformed line: " << line << "\n";
            continue;
        }

        // Create a Vec3 for the normal vector
        Vec3 normal(nx, ny, nz);
        
        // Calculate the distance from the origin using the point-normal formula
        double dO = std::abs(nx * px + ny * py + nz * pz) / normal.length();
        
        // Create the HalfSpace instance
        //halfSpaces.emplace_back(normal, dO);
        //HalfSpace(normal.normalize(), Vec3(px, py, pz));
        halfSpaces.emplace_back(normal.normalize(), Vec3(px, py, pz));
    }
    
    file.close();
}

std::vector<Vec3> generatePointsAlongCenterline(std::vector<Vec3> &vecOfEdges) {

    // Step 1: Measure the total length of the curve
    double curve_length = 0.0;
    std::vector<double> edge_lengths;
    std::vector<Vec3> wayPoints;

    // User-defined parameters
    real sphereRadius = 0.05;  // Radius of each sphere
    real dt = 0.05;           // Distance from the sphere surface to the circle center
    int num_steps = 21;      // Number of divisions along the curve
    std::vector<Vec3> sphere_positions;

    size_t num_edges = vecOfEdges.size() - 1;

    for (size_t i = 0; i < num_edges; ++i) {
        Vec3 v1 = vecOfEdges[i];
        Vec3 v2 = vecOfEdges[i + 1];
        double edge_length = (v2 - v1).length();
        edge_lengths.push_back(edge_length);
        curve_length += edge_length;
    }

    std::cout << "Curve length: " << curve_length << std::endl;

    // Step 2: Set ds (step size)
    double ds = curve_length / real(num_steps);

    // Step 3: Compute cumulative lengths to help find the edge containing each step
    std::vector<double> cumulative_lengths;
    cumulative_lengths.push_back(0.0);  // Starting point

    for (size_t i = 0; i < edge_lengths.size(); ++i) {
        cumulative_lengths.push_back(cumulative_lengths.back() + edge_lengths[i]);
    }

    // Step 4: Traverse the curve in increments of ds
    for (double s = ds; s <= curve_length-ds; s += ds) {
        // Find the edge that contains the current distance s
        size_t edge_index = 0;
        while (edge_index < num_edges && s > cumulative_lengths[edge_index + 1]) {
            ++edge_index;
        }

        if (edge_index >= num_edges) {
            break;  // Reached the end of the curve
        }

        // Compute the parameter t along the current edge
        double edge_start = cumulative_lengths[edge_index];
        double edge_end = cumulative_lengths[edge_index + 1];
        double t = (s - edge_start) / (edge_end - edge_start);

        // Calculate the point along the edge at distance s
        Vec3 v1 = vecOfEdges[edge_index];
        Vec3 v2 = vecOfEdges[edge_index + 1];
        Vec3 point_on_edge = v1 + (v2 - v1) * t;

        // Step 5: Take a user-defined action at the point
        wayPoints.push_back(point_on_edge);

        //-------------------- User Action Start --------------------
        // Generate a ring of spheres around the current edge at point_on_edge
        Vec3 edge_direction = v2 - v1;
        Vec3 someVector(1.0, 0., 0.);
        if ( std::abs( trans(edge_direction) * someVector ) > 0.999) 
          someVector = Vec3(0.0, 1.0, 0.0);

        // Compute orthogonal vectors u and v in the plane perpendicular to edge_direction
        Vec3 u = (edge_direction % someVector).getNormalized();
        Vec3 v = (edge_direction % u).getNormalized();

        // Compute circle radius
        real circle_radius = sphereRadius + dt;
        real circumference = 2. * M_PI * circle_radius;

        std::cout << "circumference = " << circumference << std::endl;
        // Compute maximum number of spheres without overlap
        int max_spheres = int(circumference / (2. * sphereRadius)) - 1;

        if (max_spheres < 1)
          max_spheres = 1;

        std::cout << "max_spheres = " << max_spheres << std::endl;

        // Compute exact angle step
        real theta_step = 2. * M_PI / max_spheres;

        // Place spheres around the circle
        for(int i(0); i < max_spheres; ++i) {
          real theta = i * theta_step;
          Vec3 sphere_offset = (std::cos(theta) * u + std::sin(theta) * v) * circle_radius;
          sphere_positions.push_back(Vec3(point_on_edge + sphere_offset));
        }
    }

    real minDist = std::numeric_limits<real>::max();
    for (size_t i = 1; i < wayPoints.size(); ++i) {
        real dist = (wayPoints[i-1] - wayPoints[i]).length();
        if (minDist > dist) minDist = dist;
        std::cout << "Distance between [" << i-1 << ", " << i << "] = " << (wayPoints[i-1] - wayPoints[i]).length() << std::endl;
    }
    std::cout << "Minimal distance: " << minDist  << " => minRadius = " << minDist * 0.5 << std::endl;

    return sphere_positions;

}


//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Main function for the box stack example.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return Success of the simulation.
 */
int main( int argc, char* argv[] )
{
   // Time parameters
   const size_t initsteps     (  2000 );  // Initialization steps with closed outlet door
   const size_t timesteps     ( 11 );  // Number of time steps for the flowing granular media
   const real   stepsize      ( 0.01 );  // Size of a single time step

   // Visualization variables
   const size_t visspacing    (   10 );  // Number of time steps in-between two POV-Ray files
   bool povray  ( true );
   bool irrlicht( false );
   bool vtk( true );

   setSeed( 12345 );  // Setup of the random number generation

   // Fixed simulation parameters
   const real L(0.1);

   const real LX(0.1);
   const real LY(0.1);
   const real LZ(0.04);

   // Parsing the command line arguments
   CommandLineInterface& cli = CommandLineInterface::getInstance();
   cli.getDescription().add_options()
     ( "file", value<std::string>()->default_value(""), "obj mesh file to be loaded" );
   cli.parse( argc, argv );
   cli.evaluateOptions();
   variables_map& vm = cli.getVariablesMap();
   if( vm.count( "no-povray" ) > 0 )
      povray = false;
   if( vm.count( "no-irrlicht" ) > 0 )
      irrlicht = false;
   if( vm.count( "no-vtk" ) > 0 )
      vtk = false;

   std::string fileName(vm["file"].as<std::string>());

   if( fileName.empty() ) {
     std::cout << "Need to enter a file via the --file command line parameter. Setting a default value." << std::endl;
     fileName = std::string("archimedes.obj");
   }

   // Simulation world setup
   WorldID world = theWorld();
   world->setGravity( 0.0,-1.0, 0.0 );

   real simViscosity( 8.37e-5 );
   real simRho( 1.0 );
   world->setViscosity( simViscosity );
   world->setLiquidDensity( simRho );
 
   // Particle Bench Config 
   real slipLength( 0.01 );
   real minEps( 0.01 );
   world->setLiquidSolid(true);
   world->setDamping( 0.98 );

   std::vector<Vec3> edges = readVectorsFromFile("vertices.txt");
   std::vector<Vec3> spherePositions = generatePointsAlongCenterline(edges);
//   std::vector<HalfSpace> halfSpaces;
//   loadPlanesAndCreateHalfSpaces("my_planes.txt", halfSpaces);
//
//   Vec3 p2(-0.52232, -2.5464, 0.13586);
//   Vec3 p3( 0.37061, -2.573, 0.091369);
//   Vec3 p4( 1.5182, -2.1098, 0.047162);
//   for (const auto& hs : halfSpaces) {
//      hs.print(std::cout, "\t");
//      std::cout << "Contains point " << Vec3(-1.5486, -2.0881, 0.18783) << " " << hs.containsPoint(Vec3(-1.5486, -2.0881, 0.18783)) << std::endl;
//      std::cout << "Contains point " << p2 << " " << hs.containsPoint(p2) << std::endl;
//      std::cout << "Contains point " << p3 << " " << hs.containsPoint(p3) << std::endl;
//      std::cout << "Contains point " << p4 << " " << hs.containsPoint(p4) << std::endl;
//   }
//
//   return 0;
 
   // Setup of the VTK visualization
   if( vtk ) {
      vtk::WriterID vtkw = vtk::activateWriter( "./paraview", visspacing, 0, timesteps, false);
   }

   //======================================================================================== 
   // The way we atm include lubrication by increasing contact threshold
   // has problems: the particles get distributed to more domain bc the threshold AABB
   // is much larger than the particle actually is.
   // We can even run into the "registering distant domain" error when the AABB of the 
   // particle is close in size to the size of a domain part!
   //======================================================================================== 
   int id = 1;
 
   //======================================================================================== 
   // Here is how to create some random positions on a grid up to a certain
   // volume fraction.
   //======================================================================================== 
   bool resume = false;
   real epsilon = 2e-4;
   real targetVolumeFraction = 0.35;
   real radius2 = 0.01 - epsilon;

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
   MaterialID sphere = createMaterial( "sphere", 1.0, 0.5, 0.1, 0.05, 0.3, 300, 1e6, 1e5, 2e5 );
   MaterialID archi  = createMaterial( "archimedes", 1.0, 0.5, 0.1, 0.05, 0.3, 300, 1e6, 1e5, 2e5 );
   int planeId = 99999;
   //======================================================================================== 
   Vec3 archimedesPos(0.0274099, -2.56113, 0.116155);
   TriangleMeshID archimedes = createTriangleMesh(++id, Vec3(0, 0, 0.0), fileName, archi, true, true, Vec3(1.0,1.0,1.0), false, false);
   archimedes->setPosition(archimedesPos);
   archimedes->setFixed(true);

//   Vec3 spherePos   (-1.9, 0.4, 0.1);
//   Vec3 spherePos2  ( 0.42236, -0.300538, 0.08);
//   //Vec3 spherePos (-2.15, 0.65, 0.1);
//   //Vec3 spherePos ( 0.0,-0.2, 0.1);
//   //Vec3 spherePos (-1.9,-2.2, 0.1);
//   real sphereRad = 0.1;
//   SphereID s1 = createSphere( id++, spherePos, sphereRad, sphere );
//   SphereID s2 = createSphere( id++, spherePos2, sphereRad, sphere );


   std::vector<Vec3> allPositions;
   int numPositions;
 
   //======================================================================================== 
   // The positions are created randomly on the root process and then bcasts 
   // to the other processes.
   //======================================================================================== 
   //allPositions = generateRandomPositions(LX, 2.0 * radius2, targetVolumeFraction, epsilon); 
   //numPositions = allPositions.size();
   real sphereRad = 0.05;  // Radius of each sphere
   for (auto spherePos: spherePositions) {
     createSphere( id++, spherePos, sphereRad, sphere );
   }

 
   unsigned int particlesTotal = 0;
   for (int j(0); j < theCollisionSystem()->getBodyStorage().size(); j++) {
     World::SizeType widx = static_cast<World::SizeType>(j);
     BodyID body = world->getBody(static_cast<unsigned int>(widx));
     if(body->getType() == sphereType) {
        particlesTotal++;
     }
   }
   real domainVol = LX * LY * LZ;
   real partVol = 4./3. * M_PI * std::pow(radius2, 3);
   real phi = (particlesTotal * partVol)/domainVol * 100.0;
 
   std::string resOut = (resume) ? " resuming " : " not resuming ";
 
   pe_EXCLUSIVE_SECTION( 0 ) {
     std::cout << "\n--" << "SIMULATION SETUP"
       << "--------------------------------------------------------------\n"
       << " Total timesteps                         = " << timesteps << "\n"
       << " Timestep size                           = " << stepsize << "\n"
       << " Total particles                         = " << particlesTotal << "\n"
       << " particle volume                         = " << partVol << "\n"
       << " Fluid Viscosity                         = " << simViscosity << "\n"
       << " Fluid Density                           = " << simRho << "\n"
       << " Lubrication h_c                         = " << slipLength << "\n"
       << " Lubrication threshold                   = " << lubricationThreshold << "\n"
       << " Contact threshold                       = " << contactThreshold << "\n"
       << " eps_init                                = " << lubricationThreshold / radius2 << "\n"
       << " Domain volume                           = " << LX * LY * LZ << "\n"
       << " Resume                                  = " << resOut  << "\n"
       << " Volume fraction[%]                      = " << phi << "\n" << std::endl;
     std::cout << "--------------------------------------------------------------------------------\n" << std::endl;
   }

   // Simulation loop
   std::cout << "\n--" << pe_BROWN << "RIGID BODY SIMULATION" << pe_OLDCOLOR
             << "---------------------------------------------------------" << std::endl;

   for( unsigned int timestep=0; timestep <= timesteps; ++timestep ) {
      std::cout << "\r Time step " << timestep << " of " << timesteps << "   " << std::flush;
      world->simulationStep( stepsize );
      //std::cout << "\r Sphere pos " << s1->getPosition() << std::endl;
   }

   std::cout << "\n--------------------------------------------------------------------------------\n"
             << std::endl;


   return 0;
}
//*************************************************************************************************
