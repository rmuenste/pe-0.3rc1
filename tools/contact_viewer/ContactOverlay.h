//=================================================================================================
/*!
 *  \file tools/contact_viewer/ContactOverlay.h
 *  \brief Recording contact container and its Polyscope overlay
 *
 *  ContactLog is the minimal container interface MaxContacts::collide() expects (the same
 *  trick as tests/interface/pe_ellipsoid_contact_test.cpp): the narrow phase runs directly on
 *  two bodies and its output is recorded, no collision system or solver involved.
 *  drawContactOverlay() mirrors a log as a point cloud plus world-length normal vectors.
 */
//=================================================================================================

#ifndef _PE_TOOLS_CONTACT_VIEWER_CONTACT_OVERLAY_H_
#define _PE_TOOLS_CONTACT_VIEWER_CONTACT_OVERLAY_H_

#include <cmath>
#include <string>
#include <vector>

#include <pe/core.h>

#include "glm/glm.hpp"
#include "polyscope/point_cloud.h"
#include "polyscope/polyscope.h"

#include "ShapeMeshes.h"

namespace viewer {

struct ContactLog
{
   enum Kind { vertexFace, edgeEdge, lubrication };

   struct Entry {
      pe::GeomID g1;
      pe::GeomID g2;
      pe::Vec3   pos;
      pe::Vec3   normal;   //!< Points from g2 towards g1 (pe convention).
      pe::real   dist;     //!< Signed distance, negative = penetration.
      Kind       kind;
   };
   std::vector<Entry> entries;

   void addVertexFaceContact( pe::GeomID g1, pe::GeomID g2, const pe::Vec3& gpos, const pe::Vec3& normal, pe::real dist ) {
      entries.push_back( Entry{ g1, g2, gpos, normal, dist, vertexFace } );
   }
   void addEdgeEdgeContact( pe::GeomID g1, pe::GeomID g2, const pe::Vec3& gpos, const pe::Vec3& normal,
                            const pe::Vec3&, const pe::Vec3&, pe::real dist ) {
      entries.push_back( Entry{ g1, g2, gpos, normal, dist, edgeEdge } );
   }
   void addLubricationContact( pe::GeomID g1, pe::GeomID g2, const pe::Vec3& gpos, const pe::Vec3& normal,
                               pe::real dist, pe::real = pe::real(1) ) {
      entries.push_back( Entry{ g1, g2, gpos, normal, dist, lubrication } );
   }
   void clear() { entries.clear(); }
};


inline const char* kindName( ContactLog::Kind kind )
{
   switch( kind ) {
      case ContactLog::vertexFace:  return "vertex-face";
      case ContactLog::edgeEdge:    return "edge-edge";
      case ContactLog::lubrication: return "lubrication";
   }
   return "?";
}


struct OverlayOptions {
   enum ColorBy { bySign, byKind };
   int    colorBy      = bySign;
   double normalLength = 0.3;    //!< World length of the drawn normals, or the scale of |dist|.
   bool   scaleByDist  = false;  //!< Normal length = normalLength * |dist| instead of fixed.
   double pointRadius  = 0.025;  //!< World radius of the contact markers.
};


inline glm::vec3 contactColor( const ContactLog::Entry& c, int colorBy )
{
   if( colorBy == OverlayOptions::byKind ) {
      switch( c.kind ) {
         case ContactLog::vertexFace:  return glm::vec3( 0.20f, 0.55f, 0.95f );
         case ContactLog::edgeEdge:    return glm::vec3( 0.95f, 0.55f, 0.10f );
         case ContactLog::lubrication: return glm::vec3( 0.30f, 0.80f, 0.45f );
      }
   }
   // Penetrating contacts red, contacts within contactThreshold of touching yellow.
   return c.dist < pe::real(0) ? glm::vec3( 0.90f, 0.15f, 0.15f ) : glm::vec3( 0.95f, 0.85f, 0.15f );
}


//! (Re-)registers the point cloud \a name for \a log; removes it when the log is empty, since
//! Polyscope structures without elements have no well-defined extents.
inline void drawContactOverlay( const std::string& name, const ContactLog& log, const OverlayOptions& opt )
{
   if( log.entries.empty() ) {
      polyscope::removeStructure( name, /*errorIfAbsent=*/false );
      return;
   }

   std::vector<glm::vec3> points, normals, colors;
   for( const ContactLog::Entry& c : log.entries ) {
      const double len = opt.scaleByDist ? opt.normalLength * std::abs( static_cast<double>( c.dist ) )
                                         : opt.normalLength;
      points.push_back ( toGlm( c.pos ) );
      normals.push_back( toGlm( c.normal ) * static_cast<float>( len ) );
      colors.push_back ( contactColor( c, opt.colorBy ) );
   }

   polyscope::PointCloud* cloud = polyscope::registerPointCloud( name, points );
   cloud->setPointRadius( opt.pointRadius, /*isRelative=*/false );
   cloud->addColorQuantity( "color", colors )->setEnabled( true );
   // AMBIENT vectors are drawn at their world length, so the arrow length is what the GUI says.
   cloud->addVectorQuantity( "normal (g2 -> g1)", normals, polyscope::VectorType::AMBIENT )
        ->setVectorRadius( 0.4 * opt.pointRadius, /*isRelative=*/false )
        ->setVectorColor( glm::vec3( 0.1f, 0.1f, 0.1f ) )
        ->setEnabled( true );
}

} // namespace viewer

#endif
