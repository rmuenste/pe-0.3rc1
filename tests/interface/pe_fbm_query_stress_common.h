// Shared comparison core for the FBM classification query stress tests.
//
// Both the serial and the MPI harness ask the same question - does the accelerated
// (HashGrid) query agree with the exhaustive one on the SAME frozen state? - and differ
// only in what world they build around it. Everything that decides a verdict lives here
// so the two cannot drift apart.
#pragma once

#include <pe/core.h>
#include <pe/interface/object_queries.h>
#include <cstdio>
#include <cmath>
#include <string>
#include <vector>

namespace fbmstress {

struct Stats {
   long queries = 0, accelHits = 0, baseHits = 0, hitMiss = 0, idDiff = 0, inprDiff = 0;
   std::vector<std::string> examples;
   long bad() const { return hitMiss + idDiff; }
};

inline bool gridActive() {
   return pe::theCollisionSystem()->getCoarseDetector().isGridActive();
}

inline Stats sweep(const std::vector<pe::Vec3>& pts) {
   Stats s;
   for (size_t i = 0; i < pts.size(); ++i) {
      double pos[3] = { pts[i][0], pts[i][1], pts[i][2] };
      int ia = 0, ib = 0;
      short int ba[8] = {}, bb[8] = {};
      const bool ra = ::pointInsideParticlesAccelerated(0, &ia, pos, ba);
      const bool rb = ::pointInsideParticles(0, &ib, pos, bb);
      const unsigned long long ida = ::ByteArrayToUint64(ba);
      const unsigned long long idb = ::ByteArrayToUint64(bb);
      ++s.queries; s.accelHits += ra; s.baseHits += rb;
      bool bad = false;
      if (ra != rb)              { ++s.hitMiss;  bad = true; }
      else if (ra && ida != idb) { ++s.idDiff;   bad = true; }
      if (ra && rb && ia != ib)  { ++s.inprDiff; }
      if (bad && s.examples.size() < 6) {
         char buf[256];
         std::snprintf(buf, sizeof buf,
            "point (%.12g, %.12g, %.12g): accel hit=%d id=%llu | baseline hit=%d id=%llu",
            pts[i][0], pts[i][1], pts[i][2], int(ra), ida, int(rb), idb);
         s.examples.push_back(buf);
      }
   }
   return s;
}

// Points just inside and just outside a sphere of influence around c, plus its centre.
inline void addSurfacePoints(std::vector<pe::Vec3>& pts, const pe::Vec3& c, pe::real r, pe::real eps) {
   static const double d[7][3] = {{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1},{0.577,0.577,0.577}};
   for (int k = 0; k < 7; ++k) {
      const pe::Vec3 u(pe::real(d[k][0]), pe::real(d[k][1]), pe::real(d[k][2]));
      pts.push_back(c + u * (r - eps));
      pts.push_back(c + u * (r + eps));
   }
   pts.push_back(c);
}

inline void addLattice(std::vector<pe::Vec3>& pts, const pe::Vec3& lo, const pe::Vec3& hi, int n) {
   for (int i = 0; i < n; ++i) for (int j = 0; j < n; ++j) for (int k = 0; k < n; ++k)
      pts.push_back(pe::Vec3(lo[0] + (hi[0]-lo[0])*i/(n-1.0),
                             lo[1] + (hi[1]-lo[1])*j/(n-1.0),
                             lo[2] + (hi[2]-lo[2])*k/(n-1.0)));
}

} // namespace fbmstress
