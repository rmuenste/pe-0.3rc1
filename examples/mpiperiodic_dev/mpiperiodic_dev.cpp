
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
#include <sstream>
#include <random>
#include <algorithm>
#include <pe/core/Types.h>
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

//*************************************************************************************************


//*************************************************************************************************
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
//*************************************************************************************************


//real sphereRad = 0.0125;  // Radius of each sphere
real sphereRad = 0.00625;  // Radius of each sphere 0.00625
//*************************************************************************************************
std::vector<Vec3> generatePointsAlongCenterline(std::vector<Vec3> &vecOfEdges) {

    // Step 1: Measure the total length of the curve
    double curve_length = 0.0;
    std::vector<double> edge_lengths;
    std::vector<Vec3> wayPoints;
    int num_rings = 10;

    // User-defined parameters
    real sphereRadius = sphereRad;  // Radius of each sphere
    real dt = 2. * sphereRad;           // Distance from the sphere surface to the circle center
    int num_steps = 188;      // Number of divisions along the curve
    std::vector<Vec3> sphere_positions;

    size_t num_edges = vecOfEdges.size() - 1;

    for (size_t i = 0; i < num_edges; ++i) {
        Vec3 v1 = vecOfEdges[i];
        Vec3 v2 = vecOfEdges[i + 1];
        double edge_length = (v2 - v1).length();
        edge_lengths.push_back(edge_length);
        curve_length += edge_length;
    }

    //std::cout << "Curve length: " << curve_length << std::endl;

    // Step 2: Set ds (step size)
    double ds = curve_length / real(num_steps);

    // Step 3: Compute cumulative lengths to help find the edge containing each step
    std::vector<double> cumulative_lengths;
    cumulative_lengths.push_back(0.0);  // Starting point

    for (size_t i = 0; i < edge_lengths.size(); ++i) {
        cumulative_lengths.push_back(cumulative_lengths.back() + edge_lengths[i]);
    }

    // Step 4: Traverse the curve in increments of ds
    for (double s = ds + 0.2 * ds; s <= curve_length-ds; s += ds) {
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

        for (int j(0); j < num_rings; ++j) {

          // Compute circle radius
          real circle_radius = sphereRadius + dt + j * (2. * sphereRadius + 0.75 * dt);

          real circumference = 2. * M_PI * circle_radius;
          
          //std::cout << "circumference = " << circumference << std::endl;
          // Compute maximum number of spheres without overlap
          int max_spheres = int(circumference / (2. * sphereRadius)) - 1;

          if (max_spheres < 1)
             max_spheres = 1;

          //std::cout << "max_spheres = " << max_spheres << std::endl;

          // Compute exact angle step
          real theta_step = 2. * M_PI / max_spheres;

          // Place spheres around the circle
          for(int i(0); i < max_spheres; ++i) {
             real theta = i * theta_step;
             Vec3 sphere_offset = (std::cos(theta) * u + std::sin(theta) * v) * circle_radius;
             sphere_positions.push_back(Vec3(point_on_edge + sphere_offset));
          }

        }


    }

    real minDist = std::numeric_limits<real>::max();
    for (size_t i = 1; i < wayPoints.size(); ++i) {
        real dist = (wayPoints[i-1] - wayPoints[i]).length();
        if (minDist > dist) minDist = dist;
        //std::cout << "Distance between [" << i-1 << ", " << i << "] = " << (wayPoints[i-1] - wayPoints[i]).length() << std::endl;
    }
    //std::cout << "Minimal distance: " << minDist  << " => minRadius = " << minDist * 0.5 << std::endl;

    return sphere_positions;

}

// Function to load planes from file and create HalfSpace instances
void loadPlanesAndCreateHalfSpaces(const std::string &filename, std::vector<HalfSpace> &halfSpaces)
{
   std::ifstream file(filename);
   std::string line;

   if (!file.is_open())
   {
      std::cerr << "Error: Could not open file " << filename << "\n";
      return;
   }
   int counter = 0;
   while (std::getline(file, line))
   {
      std::istringstream iss(line);
      double px, py, pz, nx, ny, nz;

      // Read the plane's point and normal from the line
      if (!(iss >> px >> py >> pz >> nx >> ny >> nz))
      {
         std::cerr << "Error: Malformed line: " << line << "\n";
         continue;
      }

      // Create a Vec3 for the normal vector
      Vec3 normal(nx, ny, nz);
      if (normal[0] > 0.0)
      {
         normal = -normal;
      }
      Vec3 point(px, py, pz);

      bool originOutside = (trans(-point) * normal < 0.0);
      //*  - > 0: The global origin is outside the half space\n
      //*  - < 0: The global origin is inside the half space\n
      //*  - = 0: The global origin is on the surface of the half space

      // Calculate the distance from the origin using the point-normal formula
      double dO = std::abs(nx * px + ny * py + nz * pz) / normal.length();
      if (!originOutside)
      {
         dO = -dO;
      }

      // Create the HalfSpace instance
      halfSpaces.emplace_back(normal, dO);
      counter++;
   }

   file.close();
}

// Function to load planes from file and create HalfSpace instances
void loadPlanesYAndCreateHalfSpaces(const std::string &filename, std::vector<HalfSpace> &halfSpaces)
{
   std::ifstream file(filename);
   std::string line;

   if (!file.is_open())
   {
      std::cerr << "Error: Could not open file " << filename << "\n";
      return;
   }
   int counter = 0;
   while (std::getline(file, line))
   {
      std::istringstream iss(line);
      double px, py, pz, nx, ny, nz;

      // Read the plane's point and normal from the line
      if (!(iss >> px >> py >> pz >> nx >> ny >> nz))
      {
         std::cerr << "Error: Malformed line: " << line << "\n";
         continue;
      }

      // Create a Vec3 for the normal vector
      Vec3 normal(nx, ny, nz);
      if (normal[1] < 0.0)
      {
         normal = -normal;
      }
      Vec3 point(px, py, pz);

      bool originOutside = (trans(-point) * normal < 0.0);
      //*  - > 0: The global origin is outside the half space\n
      //*  - < 0: The global origin is inside the half space\n
      //*  - = 0: The global origin is on the surface of the half space

      // Calculate the distance from the origin using the point-normal formula
      double dO = std::abs(nx * px + ny * py + nz * pz) / normal.length();
      if (!originOutside)
      {
         dO = -dO;
      }

      // Create the HalfSpace instance
      halfSpaces.emplace_back(normal, dO);
      counter++;
   }

   file.close();
}


//*************************************************************************************************
//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================



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
   // Simulation parameters

   // Particle parameters
   const bool   spheres ( true   );  // Switch between spheres and granular particles
   const real   radius  ( 0.005  );  // The radius of spheres of the granular media
   const real   spacing ( 0.001  );  // Initial spacing in-between two spheres
   const real   velocity( 0.0025 );  // Initial maximum velocity of the spheres

   // Time parameters
   const size_t initsteps     (  2000 );  // Initialization steps with closed outlet door
   const size_t timesteps     ( 2 );  // Number of time steps for the flowing granular media
   const real   stepsize      ( 0.0005 );  // Size of a single time step

   // Process parameters
   const int processesX( 34 );  // Number of processes in x-direction
   const int processesZ( 2 );  // Number of processes in z-direction

   // Random number generator parameters
   const size_t seed( 12345 );

   // Verbose mode
   const bool verbose( false );  // Switches the output of the simulation on and off

   // Visualization
   bool povray( false );  // Switches the POV-Ray visualization on and off
   bool   vtk( true );

   // Visualization parameters
   const bool   colorProcesses( false );  // Switches the processes visualization on and off
   const bool   animation     (  true );  // Switches the animation of the POV-Ray camera on and off
   const size_t visspacing    (   1 );  // Number of time steps in-between two POV-Ray files
   const size_t colorwidth    (    51 );  // Number of particles in x-dimension with a specific color


   /////////////////////////////////////////////////////
   // MPI Initialization

   MPI_Init( &argc, &argv );

   /////////////////////////////////////////////////////
   // Initial setups

   // Configuration of the simulation world
   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -0.0 );
   world->setViscosity( 373e-3 );

   // Configuration of the MPI system
   MPISystemID mpisystem = theMPISystem();

   // Checking the total number of MPI processes
   if( processesX < 2 || processesZ < 1 || processesX*processesZ != mpisystem->getSize() ) {
      std::cerr << pe_RED
                << "\n Invalid number of MPI processes!\n\n"
                << processesX*processesZ << " : " << mpisystem->getSize()
                << pe_OLDCOLOR;
      return EXIT_FAILURE;
   }

   // Setup of the random number generation
   setSeed( seed );

   // Creating the material for the particles
   MaterialID granular = createMaterial( "granular", 1.0, 0.04, 0.1, 0.1, 0.3, 300, 1e6, 1e5, 2e5 );

   // Fixed simulation parameters
   const real L(0.1);

   // Cube domain
   const real lx( L );                         // Size of the domain in x-direction
   const real ly( L );                         // Size of the domain in y-direction
   const real lz( L );                         // Size of the domain in z-direction
   const real space( real(2)*radius+spacing );  // Space initially required by a single particle

   /////////////////////////////////////////////////////
   // Setup of the MPI processes: Periodic 2D Regular Domain Decomposition

   const real lpx( lx / processesX );  // Size of a process subdomain in x-direction
   const real lpz( lz / processesZ );  // Size of a process subdomain in z-direction

   int dims   [] = { processesX, processesZ };
   int periods[] = { false     , false      };

   int rank;           // Rank of the neighboring process
   int center[2];      // Definition of the coordinates array 'center'
   MPI_Comm cartcomm;  // The new MPI communicator with Cartesian topology

   MPI_Cart_create( MPI_COMM_WORLD, 2, dims, periods, false, &cartcomm );
   mpisystem->setComm( cartcomm );
   MPI_Cart_coords( cartcomm, mpisystem->getRank(), 2, center );

   std::vector<HalfSpace> halfSpaces;
   std::vector<HalfSpace> halfSpacesY;

   loadPlanesAndCreateHalfSpaces("planesX68.txt", halfSpaces);
   loadPlanesYAndCreateHalfSpaces("planesY68.txt", halfSpacesY);

   int west[] = {center[0] - 1, center[1]};
   int east[] = {center[0] + 1, center[1]};

   int south[] = {center[0], center[1] - 1};
   int north[] = {center[0], center[1] + 1};

   int southwest[] = { center[0]-1, center[1]-1 };
   int southeast[] = { center[0]+1, center[1]-1 };

   int northwest[] = { center[0]-1, center[1]+1 };
   int northeast[] = { center[0]+1, center[1]+1 };

   std::vector<HalfSpace> mySpaces;

   if (west[0] < 0)
   {
      HalfSpace hs_y = halfSpacesY[center[0]];  
      if(center[1] != 0)
      {
        HalfSpace hsy_temp = halfSpacesY[center[0]];
        hs_y = HalfSpace(-hsy_temp.getNormal(), -hsy_temp.getDisplacement());
      }

      mySpaces.push_back(halfSpaces[center[0]]);
      mySpaces.push_back(hs_y);

      defineLocalDomain(intersect(
               halfSpaces[center[0]],
               hs_y
      ));
   }
   else
   {
      // -x of halfSpaces[mpisystem->getRank()] and +x of halfSpaces[mpisystem->getRank()-1]
      HalfSpace hs = halfSpaces[center[0] - 1];
      HalfSpace hs_flip = HalfSpace(-hs.getNormal(), -hs.getDisplacement());
      
      HalfSpace hs_y = halfSpacesY[center[0]];  
      if(center[1] != 0)
      {
        HalfSpace hsy_temp = halfSpacesY[center[0]];
        hs_y = HalfSpace(-hsy_temp.getNormal(), -hsy_temp.getDisplacement());
      }

      mySpaces.push_back(halfSpaces[center[0]]);
      mySpaces.push_back(hs_flip);
      mySpaces.push_back(hs_y);

      defineLocalDomain(intersect(
          halfSpaces[center[0]],
          hs_flip,
          hs_y
         ));
   }
   //===================================================================================

   pe_LOG_INFO_SECTION(log) 
   {
     log << "Center: " << center[0] << " " << center[1] << "\n";
   }

   //===================================================================================
   // Connecting the west neighbor
   if (west[0] > 0)
   {
      MPI_Cart_rank(cartcomm, west, &rank);

      HalfSpace hs1 = halfSpaces[west[0]];
      HalfSpace hs = halfSpaces[west[0] - 1];

      HalfSpace hs_flip = HalfSpace(-hs.getNormal(), -hs.getDisplacement());

      HalfSpace hs_y = halfSpacesY[west[0]];  
      if(west[1] != 0)
      {
        HalfSpace hsy_temp = halfSpacesY[west[0]];
        hs_y = HalfSpace(-hsy_temp.getNormal(), -hsy_temp.getDisplacement());
      }

      connect(rank,
              intersect(
              hs1,
              hs_flip,
              hs_y
              ));
   }

   if (west[0] == 0)
   {
      MPI_Cart_rank(cartcomm, west, &rank);

      HalfSpace hs1 = halfSpaces[west[0]];

      HalfSpace hs_y = halfSpacesY[west[0]];  
      if(west[1] != 0)
      {
        HalfSpace hsy_temp = halfSpacesY[west[0]];
        hs_y = HalfSpace(-hsy_temp.getNormal(), -hsy_temp.getDisplacement());
      }

      connect(rank,intersect(
              hs1,
              hs_y
              )
      );
   }

   //===================================================================================
   // Connecting the east neighbor
   if (east[0] < processesX)
   {
      MPI_Cart_rank(cartcomm, east, &rank);

      HalfSpace hs1 = halfSpaces[east[0]];
      HalfSpace hs = halfSpaces[east[0] - 1];

      HalfSpace hs_flip = HalfSpace(-hs.getNormal(), -hs.getDisplacement());

      HalfSpace hs_y = halfSpacesY[east[0]];  
      if(east[1] != 0)
      {
        HalfSpace hsy_temp = halfSpacesY[east[0]];
        hs_y = HalfSpace(-hsy_temp.getNormal(), -hsy_temp.getDisplacement());
      }

      connect(rank,
              intersect(
              hs1,
              hs_flip,
              hs_y
              ));
   }

   //===================================================================================
   // Connecting the south neighbor
   if( south[1] >= 0 ) {

      MPI_Cart_rank( cartcomm, south, &rank );

      HalfSpace hs_y = halfSpacesY[center[0]];  

      if(mySpaces.size() == 2) {
         connect( rank, intersect(
            mySpaces[0],
            hs_y
         ));
      }
      else {
         connect( rank, intersect(
            mySpaces[0],
            mySpaces[1],
            hs_y
         ));
      }
   }

   //===================================================================================
   // Connecting the north neighbor
   if( north[1] < processesZ ) {
      MPI_Cart_rank( cartcomm, north, &rank );

      HalfSpace hs_y = halfSpacesY[center[0]];  
      HalfSpace hsy_temp = halfSpacesY[center[0]];
      hs_y = HalfSpace(-hsy_temp.getNormal(), -hsy_temp.getDisplacement());

      if(mySpaces.size() == 2) {
         connect( rank, intersect(
            mySpaces[0],
            hs_y
         ));
      }
      else {
         connect( rank, intersect(
            mySpaces[0],
            mySpaces[1],
            hs_y
         ));
      }
   }

   //===================================================================================
   // Connecting the south-west neighbor
   if( southwest[0] >= 0 && southwest[1] >= 0 ) {
      MPI_Cart_rank( cartcomm, southwest, &rank );

      HalfSpace hs1 = halfSpaces[west[0]];
      HalfSpace hs_y = halfSpacesY[west[0]];  
      if (west[0] > 0)
      {
         HalfSpace hs = halfSpaces[west[0] - 1];
         HalfSpace hs_flip = HalfSpace(-hs.getNormal(), -hs.getDisplacement());
         connect( rank, intersect(
            hs1,
            hs_flip,
            hs_y
         ));
      }
      else {
         connect( rank, intersect(
            hs1,
            hs_y
         ));
      }

   }
 
   //===================================================================================
   // Connecting the south-east neighbor
   if( southeast[0] < processesX && southeast[1] >= 0 ) {
      MPI_Cart_rank( cartcomm, southeast, &rank );

      HalfSpace hs_y = halfSpacesY[east[0]];  

      HalfSpace hs1 = halfSpaces[east[0]];
      HalfSpace hs = halfSpaces[east[0] - 1];

      HalfSpace hs_flip = HalfSpace(-hs.getNormal(), -hs.getDisplacement());

      connect( rank, intersect(
         hs1,
         hs_flip,
         hs_y
      ));
   }

   //===================================================================================
   // Connecting the north-west neighbor
   if( northwest[0] >= 0 && northwest[1] < processesZ ) {
      MPI_Cart_rank( cartcomm, northwest, &rank );

      HalfSpace hs1 = halfSpaces[west[0]];

      HalfSpace hs_y = halfSpacesY[west[0]];  
      HalfSpace hsy_temp = halfSpacesY[west[0]];
      hs_y = HalfSpace(-hsy_temp.getNormal(), -hsy_temp.getDisplacement());

      if (west[0] > 0)
      {
         HalfSpace hs = halfSpaces[west[0] - 1];
         HalfSpace hs_flip = HalfSpace(-hs.getNormal(), -hs.getDisplacement());
         connect( rank, intersect(
            hs1,
            hs_flip,
            hs_y
         ));
      }
      else {
         connect( rank, intersect(
            hs1,
            hs_y
         ));
      }
   }
 
   //===================================================================================
   // Connecting the north-west neighbor
   if( northeast[0] < processesX && northeast[1] < processesZ ) {
      MPI_Cart_rank( cartcomm, northeast, &rank );

      HalfSpace hs_y = halfSpacesY[east[0]];  
      HalfSpace hsy_temp = halfSpacesY[east[0]];
      hs_y = HalfSpace(-hsy_temp.getNormal(), -hsy_temp.getDisplacement());

      HalfSpace hs1 = halfSpaces[east[0]];
      HalfSpace hs = halfSpaces[east[0] - 1];

      HalfSpace hs_flip = HalfSpace(-hs.getNormal(), -hs.getDisplacement());

      connect( rank, intersect(
         hs1,
         hs_flip,
         hs_y
      ) );
   }

   //Checking the process setup
   theMPISystem()->checkProcesses();

   // Create a custom material for the benchmark
   MaterialID elastic = createMaterial("elastic", 1.0, 0.1, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5);
   int idx = 0;
   std::vector<Vec3> edges = readVectorsFromFile("vertices.txt");
   std::vector<Vec3> spherePositions = generatePointsAlongCenterline(edges);
   for (auto spherePos: spherePositions) {
     if (world->ownsPoint(spherePos))
     {
       createSphere( idx++, spherePos, sphereRad, elastic );
     }
   }


   // Synchronization of the MPI processes
   world->synchronize();

   // Setup of the VTK visualization
   if( vtk ) {
      vtk::WriterID vtkw = vtk::activateWriter( "./paraview", visspacing, 0, timesteps, false);
   }

   //=================================================================================
   // Calculating the total number of particles and primitives
   unsigned long particlesTotal(0);
   unsigned long primitivesTotal(0);
   unsigned long bla = idx;

   int numBodies(0);
   int numTotal(0);
   unsigned int j(0);
   for (; j < theCollisionSystem()->getBodyStorage().size(); j++)
   {
      World::SizeType widx = static_cast<World::SizeType>(j);
      BodyID body = world->getBody(static_cast<unsigned int>(widx));
      if (body->getType() == sphereType)
      {
         numBodies++;
         numTotal++;
      }
      else
      {
         numTotal++;
      }
   }

   unsigned long bodiesUpdate = static_cast<unsigned long>(numBodies);
   unsigned long bodiesTotal = static_cast<unsigned long>(numTotal);
   MPI_Reduce(&bodiesUpdate, &particlesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm);
   MPI_Reduce(&bodiesTotal, &primitivesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm);

   real domainVol = 0.604;
   real partVol = 4. / 3. * M_PI * std::pow(sphereRad, 3);

   pe_EXCLUSIVE_SECTION(0)
   {
      std::cout << "\n--" << "SIMULATION SETUP"
                << "--------------------------------------------------------------\n"
                << " Total number of MPI processes           = " << 68 << "\n"
                << " Simulation stepsize dt                  = " << TimeStep::size() << "\n"
                << " Total number of particles               = " << particlesTotal << "\n"
                << " particle volume                         = " << partVol << "\n"
                << " Total number of objects                 = " << primitivesTotal << "\n"
                << " Gravity constant                        = " << world->getGravity() << "\n"
                << " Lubrication threshold                   = " << lubricationThreshold << "\n"
                << " Contact threshold                       = " << contactThreshold << "\n"
                << " Domain cube side length                 = " << L << "\n"
                << " Domain volume                           = " << domainVol << "\n"
                << " Volume fraction[%]                      = " << (particlesTotal * partVol) / domainVol * 100.0 << "\n"
                << " Total objects                           = " << primitivesTotal << "\n"
                << std::endl;
      std::cout << "--------------------------------------------------------------------------------\n"
                << std::endl;
   }



  pe_EXCLUSIVE_SECTION( 0 ) {
    std::cout << "\n--" << "SIMULATION SETUP"
      << "--------------------------------------------------------------\n"
      << " Total number of MPI processes           = " << processesX * processesZ << "\n"
      << " Total timesteps                         = " << timesteps << "\n"
      << " Timestep size                           = " << stepsize << "\n" << std::endl;
    std::cout << "--------------------------------------------------------------------------------\n" << std::endl;
  }

  for( unsigned int timestep=0; timestep <= timesteps; ++timestep ) {
    pe_EXCLUSIVE_SECTION( 0 ) {
     std::cout << "\r Time step " << timestep+1 << " of " << timesteps << "   " << std::flush;
    }
    world->simulationStep( stepsize );
   }
   pe_EXCLUSIVE_SECTION( 0 ) {
     std::cout << std::endl;
   }
   /////////////////////////////////////////////////////
   // MPI Finalization
   MPI_Finalize();

}    

//*************************************************************************************************
