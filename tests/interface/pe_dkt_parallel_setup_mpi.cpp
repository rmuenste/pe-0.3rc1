//=================================================================================================
/*!
 *  \file tests/interface/pe_dkt_parallel_setup_mpi.cpp
 *  \brief Contract test of the parallel-mode DKT / walled-column setup
 *         (setupDraftKissTumbBench in pe/interface/setup/dkt.h).
 *
 *  Two ranks, processesX_/Y_/Z_ = 1 x 1 x 2 on the campaign column [-3,3] x [-3,3] x [0,24]
 *  passed as the CFD domain. The json deck and the xyz file are written by the test; the three
 *  spheres sit in the lower slab (z = 5, z = 10) and the upper slab (z = 20).
 *
 *  Checked, reduced over both ranks: every sphere of the file exists exactly once as a local
 *  (owned) body, on the rank whose slab holds its start position, with its file index as user
 *  id and the json radius; the two materials exist with the serial half's values ("ground"
 *  density 1, "Bench" density / restitution / friction from the deck); the ground plane is the
 *  only plane (no side walls); gravity, fluid properties and the step size follow the deck.
 *
 *  The refusals of the setup (process grid vs communicator, degenerate box, zmin != 0, resume_,
 *  empty xyz file, a start position outside the box) end in MPI_Abort and are documented in
 *  the setup header; they are not exercised here.
 */
//=================================================================================================

#include <pe/interface/setup/dkt.h>

#include <pe/core.h>
#include <pe/core/Materials.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include <mpi.h>

namespace {

bool near( double lhs, double rhs, double tol = 1.0e-12 )
{
   return std::abs( lhs - rhs ) <= tol * std::max( 1.0, std::abs( rhs ) );
}

void writeFile( const std::string& name, const std::string& body )
{
   std::ofstream out( name );
   out << body;
}

}  // namespace

int main( int argc, char** argv )
{
   MPI_Init( &argc, &argv );

   int rank = 0;
   int size = 0;
   MPI_Comm_rank( MPI_COMM_WORLD, &rank );
   MPI_Comm_size( MPI_COMM_WORLD, &size );
   if( size != 2 ) {
      if( rank == 0 )
         std::cerr << "test requires exactly two MPI ranks\n";
      MPI_Abort( MPI_COMM_WORLD, 2 );
   }

   // Deck and positions, written by rank 0 before anyone reads them
   const double radius = 0.5;
   const double rhoP   = 1.14;
   if( rank == 0 ) {
      writeFile( "example.json",
         "{\n"
         "  \"processesX_\": 1, \"processesY_\": 1, \"processesZ_\": 2,\n"
         "  \"stepsize_\": 0.005, \"timesteps_\": 10,\n"
         "  \"benchRadius_\": 0.5, \"particleDensity_\": 1.14,\n"
         "  \"restitution_\": 0.0, \"staticFriction_\": 0.1, \"dynamicFriction_\": 0.05,\n"
         "  \"fluidDensity_\": 1.0, \"fluidViscosity_\": 0.0047,\n"
         "  \"gravity_\": [0.0, 0.0, -9.81],\n"
         "  \"vtk_\": false, \"useCheckpointer_\": false, \"resume_\": false,\n"
         "  \"xyzFilePath_\": \"particles.xyz\"\n"
         "}\n" );
      writeFile( "particles.xyz",
         "0.0 0.0 5.0\n"
         "0.05 0.0 10.0\n"
         "-1.0 1.0 20.0\n" );
   }
   MPI_Barrier( MPI_COMM_WORLD );

   pe::setupDraftKissTumbBench( MPI_COMM_WORLD, -3.0, 3.0, -3.0, 3.0, 0.0, 24.0 );

   bool valid = true;
   const auto check = [&valid, rank]( bool ok, const char* what ) {
      if( !ok ) {
         std::cerr << "rank " << rank << " FAILED: " << what << "\n";
         valid = false;
      }
   };

   // --- owned spheres: on the rank whose slab holds them, file index as user id --------------
   int ownedSpheres = 0;
   int ownedPlanes  = 0;
   int remoteBodies = 0;
   pe::WorldID world = pe::theWorld();
   for( pe::World::Iterator it = world->begin(); it != world->end(); ++it ) {
      pe::BodyID body = *it;
      if( body->isRemote() ) {
         ++remoteBodies;
         continue;
      }
      if( body->getType() == pe::sphereType ) {
         ++ownedSpheres;
         pe::SphereID sphere = pe::static_body_cast<pe::Sphere>( body );
         const pe::Vec3 pos = sphere->getPosition();
         const int expectedRank = pos[2] < 12.0 ? 0 : 1;
         check( expectedRank == rank, "sphere owned by the rank of its slab" );
         check( near( sphere->getRadius(), radius ), "sphere radius from the deck" );
         const pe::id_t uid = sphere->getID();
         check( ( uid == 0 && near( pos[2], 5.0 ) ) || ( uid == 1 && near( pos[2], 10.0 ) ) ||
                ( uid == 2 && near( pos[2], 20.0 ) ), "user id = index in the xyz file" );
         check( sphere->getMaterial() == pe::Material::find( "Bench" ), "sphere carries the Bench material" );
      }
      else if( body->getType() == pe::planeType ) {
         ++ownedPlanes;
         check( near( body->getPosition()[2], 0.0 ), "the only plane is the ground at z = 0" );
      }
   }
   check( ownedSpheres == ( rank == 0 ? 2 : 1 ), "two spheres in the lower slab, one in the upper" );
   check( ownedPlanes == 1, "exactly one plane per rank (global ground plane, no side walls)" );

   int totalSpheres = 0;
   MPI_Allreduce( &ownedSpheres, &totalSpheres, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD );
   check( totalSpheres == 3, "every sphere of the file exists exactly once" );

   // --- materials in the serial order with the serial values --------------------------------
   const pe::MaterialID ground = pe::Material::find( "ground" );
   const pe::MaterialID bench  = pe::Material::find( "Bench" );
   check( ground != pe::invalid_material && bench != pe::invalid_material, "both materials registered" );
   if( ground != pe::invalid_material && bench != pe::invalid_material ) {
      check( ground < bench, "\"ground\" registered before \"Bench\"" );
      check( near( pe::Material::getDensity( ground ), 1.0 ), "ground density 1 (serial value)" );
      check( near( pe::Material::getDensity( bench ), rhoP ), "Bench density from the deck" );
      check( near( pe::Material::getRestitution( bench ), 0.0 ), "Bench restitution from the deck" );
      check( near( pe::Material::getStaticFriction( bench ), 0.1 ), "Bench static friction from the deck" );
      check( near( pe::Material::getDynamicFriction( bench ), 0.05 ), "Bench dynamic friction from the deck" );
   }

   // --- world and step size follow the deck -------------------------------------------------
   check( near( world->getGravity()[2], -9.81 ), "gravity from the deck" );
   // (the world has no liquid-density getter; the viscosity and gravity stand for the deck values)
   check( near( world->getViscosity(), 0.0047 ), "fluid viscosity from the deck" );
   check( near( pe::TimeStep::size(), 0.005 ), "step size from the deck" );

   const int localFailure = valid ? 0 : 1;
   int failures = 0;
   MPI_Allreduce( &localFailure, &failures, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD );
   if( rank == 0 ) {
      if( failures == 0 )
         std::cout << "pe_dkt_parallel_setup_mpi: all checks passed on both ranks\n";
      else
         std::cerr << "pe_dkt_parallel_setup_mpi: " << failures << " rank(s) reported failures\n";
   }

   MPI_Finalize();
   return failures == 0 ? 0 : 1;
}
