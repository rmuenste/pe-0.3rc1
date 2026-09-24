//=================================================================================================
/*!
 *  \file tools/contact_viewer/ShapeMeshes.h
 *  \brief Body-frame surface meshes of the pe primitives for the Polyscope mirror
 *
 *  Every generator returns the mesh in the body frame at the actual dimensions, following the
 *  pe conventions: capsule and cylinder axes run along the body-frame x axis, the plane's
 *  default normal is the body-frame z axis. The mirror then only needs the rigid transform of
 *  the body (bodyTransform()); a size change re-registers the mesh. Generating at the actual
 *  size instead of scaling a unit mesh keeps the capsule caps spherical.
 */
//=================================================================================================

#ifndef _PE_TOOLS_CONTACT_VIEWER_SHAPE_MESHES_H_
#define _PE_TOOLS_CONTACT_VIEWER_SHAPE_MESHES_H_

#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

#include <pe/core.h>

#include "glm/glm.hpp"

namespace viewer {

//! Polygon mesh; boxes and planes use quads so that the drawn edges are the real edges of the
//! shape instead of a triangulation.
struct ShapeMesh {
   std::vector<glm::vec3>                 vertices;
   std::vector<std::vector<std::size_t>>  faces;
};

const double kMeshPi = 3.14159265358979323846;


//! Rigid transform (rotation + translation) of a pe body as a column-major glm matrix.
inline glm::mat4 bodyTransform( pe::ConstBodyID body )
{
   const pe::Vec3& p = body->getPosition();
   const pe::Rot3& R = body->getRotation();
   glm::mat4 T( 1.0f );
   for( int c = 0; c < 3; ++c )
      for( int r = 0; r < 3; ++r )
         T[c][r] = static_cast<float>( R( r, c ) );
   T[3] = glm::vec4( static_cast<float>( p[0] ),
                     static_cast<float>( p[1] ),
                     static_cast<float>( p[2] ), 1.0f );
   return T;
}


inline glm::vec3 toGlm( const pe::Vec3& v )
{
   return glm::vec3( static_cast<float>( v[0] ), static_cast<float>( v[1] ), static_cast<float>( v[2] ) );
}


//! Latitude/longitude rings between two poles on the x axis. \a ring( i ) returns the axial
//! position and the radius of ring i (1 <= i < rings); used for ellipsoids and capsules.
template< typename RingFn >
inline ShapeMesh makeRevolvedMesh( float xPoleLow, float xPoleHigh, int rings, int segments, RingFn ring )
{
   ShapeMesh m;
   m.vertices.push_back( glm::vec3( xPoleLow, 0.0f, 0.0f ) );
   for( int i = 1; i < rings; ++i ) {
      const std::array<float,2> xr = ring( i );
      for( int j = 0; j < segments; ++j ) {
         const double phi = 2.0 * kMeshPi * j / segments;
         m.vertices.push_back( glm::vec3( xr[0], xr[1] * static_cast<float>( std::cos( phi ) ),
                                                 xr[1] * static_cast<float>( std::sin( phi ) ) ) );
      }
   }
   m.vertices.push_back( glm::vec3( xPoleHigh, 0.0f, 0.0f ) );

   const std::size_t low  = 0;
   const std::size_t high = m.vertices.size() - 1;
   const auto at = [segments]( int i, int j ) {
      return static_cast<std::size_t>( 1 + ( i - 1 ) * segments + ( j % segments ) );
   };
   for( int j = 0; j < segments; ++j ) {
      m.faces.push_back( { low, at( 1, j + 1 ), at( 1, j ) } );
      m.faces.push_back( { high, at( rings - 1, j ), at( rings - 1, j + 1 ) } );
   }
   for( int i = 1; i < rings - 1; ++i )
      for( int j = 0; j < segments; ++j ) {
         m.faces.push_back( { at( i, j ), at( i, j + 1 ), at( i + 1, j + 1 ) } );
         m.faces.push_back( { at( i, j ), at( i + 1, j + 1 ), at( i + 1, j ) } );
      }
   return m;
}


//! Ellipsoid with semi-axes (a, b, c); a sphere is the a = b = c case.
inline ShapeMesh makeEllipsoidMesh( double a, double b, double c, int rings = 24, int segments = 48 )
{
   ShapeMesh m = makeRevolvedMesh( -1.0f, 1.0f, rings, segments, [rings]( int i ) {
      const double theta = kMeshPi * i / rings;   // from the -x pole
      return std::array<float,2>{ static_cast<float>( -std::cos( theta ) ),
                                  static_cast<float>(  std::sin( theta ) ) };
   } );
   for( glm::vec3& v : m.vertices )
      v *= glm::vec3( static_cast<float>( a ), static_cast<float>( b ), static_cast<float>( c ) );
   return m;
}


//! Capsule of cylinder part length \a length (pe convention) and cap radius \a radius.
inline ShapeMesh makeCapsuleMesh( double radius, double length, int capRings = 12, int segments = 48 )
{
   const int rings = 2 * capRings;   // rings 1..capRings on the low cap, the rest on the high cap
   const double h = 0.5 * length;
   return makeRevolvedMesh( static_cast<float>( -h - radius ), static_cast<float>( h + radius ), rings, segments,
      [=]( int i ) {
         // Both caps carry an equator ring (i == capRings and i == capRings + 1 would coincide
         // on a sphere); the quad strip between them is the cylinder part.
         const bool   low   = ( i <= capRings );
         const double theta = low ? 0.5 * kMeshPi * i / capRings
                                  : 0.5 * kMeshPi * ( 1.0 + static_cast<double>( i - capRings - 1 ) / ( capRings - 1 ) );
         const double x     = ( low ? -h : h ) - radius * std::cos( theta );
         return std::array<float,2>{ static_cast<float>( x ), static_cast<float>( radius * std::sin( theta ) ) };
      } );
}


inline ShapeMesh makeCylinderMesh( double radius, double length, int segments = 48 )
{
   ShapeMesh m;
   const float h = static_cast<float>( 0.5 * length );
   for( int side = 0; side < 2; ++side )
      for( int j = 0; j < segments; ++j ) {
         const double phi = 2.0 * kMeshPi * j / segments;
         m.vertices.push_back( glm::vec3( side == 0 ? -h : h,
                                          static_cast<float>( radius * std::cos( phi ) ),
                                          static_cast<float>( radius * std::sin( phi ) ) ) );
      }
   const std::size_t cLow  = m.vertices.size();
   m.vertices.push_back( glm::vec3( -h, 0.0f, 0.0f ) );
   const std::size_t cHigh = m.vertices.size();
   m.vertices.push_back( glm::vec3(  h, 0.0f, 0.0f ) );

   const std::size_t n = static_cast<std::size_t>( segments );
   for( std::size_t j = 0; j < n; ++j ) {
      const std::size_t k = ( j + 1 ) % n;
      m.faces.push_back( { j, k, n + k } );
      m.faces.push_back( { j, n + k, n + j } );
      m.faces.push_back( { cLow, k, j } );
      m.faces.push_back( { cHigh, n + j, n + k } );
   }
   return m;
}


inline ShapeMesh makeBoxMesh( double lx, double ly, double lz )
{
   ShapeMesh m;
   const float hx = static_cast<float>( 0.5 * lx ), hy = static_cast<float>( 0.5 * ly ), hz = static_cast<float>( 0.5 * lz );
   for( int i = 0; i < 8; ++i )
      m.vertices.push_back( glm::vec3( ( i & 1 ) ? hx : -hx, ( i & 2 ) ? hy : -hy, ( i & 4 ) ? hz : -hz ) );
   m.faces = { { 0, 2, 3, 1 },   // -z
               { 4, 5, 7, 6 },   // +z
               { 0, 1, 5, 4 },   // -y
               { 2, 6, 7, 3 },   // +y
               { 0, 4, 6, 2 },   // -x
               { 1, 3, 7, 5 } }; // +x
   return m;
}


//! Finite patch of the infinite plane: a square of half size \a halfSize in the body-frame
//! z = 0 plane (the pe plane's default normal is +z).
inline ShapeMesh makePlaneMesh( double halfSize )
{
   ShapeMesh m;
   const float s = static_cast<float>( halfSize );
   m.vertices = { glm::vec3( -s, -s, 0.0f ), glm::vec3( s, -s, 0.0f ), glm::vec3( s, s, 0.0f ), glm::vec3( -s, s, 0.0f ) };
   m.faces    = { { 0, 1, 2, 3 } };
   return m;
}

//! Mesh of an existing pe body (any primitive except the infinite plane, which gets a patch
//! of half size \a planeHalfSize).
inline ShapeMesh makeBodyMesh( pe::BodyID body, double planeHalfSize = 2.5 )
{
   using namespace pe;
   switch( body->getType() ) {
      case sphereType: {
         const double r = static_cast<double>( static_body_cast<Sphere>( body )->getRadius() );
         return makeEllipsoidMesh( r, r, r );
      }
      case boxType: {
         const Vec3& l = static_body_cast<Box>( body )->getLengths();
         return makeBoxMesh( l[0], l[1], l[2] );
      }
      case capsuleType: {
         CapsuleID c = static_body_cast<Capsule>( body );
         return makeCapsuleMesh( c->getRadius(), c->getLength() );
      }
      case cylinderType: {
         CylinderID c = static_body_cast<Cylinder>( body );
         return makeCylinderMesh( c->getRadius(), c->getLength() );
      }
      case ellipsoidType: {
         const Vec3 r = static_body_cast<Ellipsoid>( body )->getRadius();
         return makeEllipsoidMesh( r[0], r[1], r[2] );
      }
      default:
         return makePlaneMesh( planeHalfSize );
   }
}

} // namespace viewer

#endif
