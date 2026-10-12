// Layer-1 stress test for the FBM classification query.
//
// Compares pointInsideParticlesAccelerated() against pointInsideParticles() on the
// SAME frozen world state. The two are the accelerated (HashGrid) and the exhaustive
// classification used by the FBM alpha field; everything downstream (KVEL candidate
// sets, forces) consumes their result, so a force comparison cannot detect a
// classification miss - only this comparison can.
//
// Each stage builds a body configuration, brings the world to a state, and sweeps a
// point set. A stage FAILS on any hit/miss or full-system-ID disagreement, except the
// ownership stage, which characterises rather than asserts.
//
// Build (non-CGAL): see report/fbm_query_stress.build.sh

#include <pe/core.h>
#include <pe/interface/object_queries.h>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>
#include <vector>
using namespace pe;

#include "pe_fbm_query_stress_common.h"
using namespace fbmstress;

// Report one stage. Stages that characterise rather than assert pass mustPass = false.
static int report(const char* stage, const Stats& s, bool mustPass, bool wantGrid) {
   const bool active = gridActive();
   const long bad = s.bad();
   std::printf("[%-26s] grid=%-3s queries=%-7ld accelHits=%-7ld baseHits=%-7ld "
               "hit/miss=%-5ld idDiff=%-5ld inprDiff=%-5ld -> %s\n",
               stage, active ? "ON" : "off", s.queries, s.accelHits, s.baseHits,
               s.hitMiss, s.idDiff, s.inprDiff,
               bad == 0 ? "ok" : (mustPass ? "FAIL" : "differs (characterised)"));
   for (size_t i = 0; i < s.examples.size(); ++i) std::printf("    %s\n", s.examples[i].c_str());
   if (wantGrid && !active) { std::printf("    ERROR: this stage requires an ACTIVE grid; it tested nothing\n"); return 1; }
   return (mustPass && bad) ? 1 : 0;
}

// Background bodies purely to push the body count over pe's gridActivationThreshold (32).
static void background(int n = 40, real x0 = 60) {
   for (int i = 0; i < n; ++i)
      createSphere(9000 + i, Vec3(x0 + 3*i, 60, 60), real(0.4), granite)->setFixed(true);
}

// ------------------------------------------------------------------- stages
static int stageA() {                      // monodisperse, after rehash and after motion
   theWorld()->clear(); background();
   std::vector<SphereID> b;
   for (int i = 0; i < 12; ++i) {
      SphereID s = createSphere(100 + i, Vec3(2.0 + 1.7*(i%4), 2.0 + 1.7*(i/4), 2.0), real(0.5), granite);
      s->setLinearVel(real(0.3)*((i%3)-1), real(0.2)*((i%2)-0.5), 0);
      b.push_back(s);
   }
   theWorld()->simulationStep(real(0.001));
   std::vector<Vec3> pts; addLattice(pts, Vec3(0,0,0), Vec3(8,8,4), 12);
   for (size_t i = 0; i < b.size(); ++i) addSurfacePoints(pts, b[i]->getPosition(), real(0.5), real(1e-6));
   int rc = report("A monodisperse, settled", sweep(pts), true, true);
   theWorld()->simulationStep(real(0.01));    // integrate: positions move, cells do not
   std::vector<Vec3> pts2;
   for (size_t i = 0; i < b.size(); ++i) addSurfacePoints(pts2, b[i]->getPosition(), real(0.5), real(1e-6));
   addLattice(pts2, Vec3(0,0,0), Vec3(8,8,4), 12);
   rc |= report("A after integration", sweep(pts2), true, true);
   return rc;
}

static int stageB() {                      // polydisperse, adversarial cell placement
   theWorld()->clear(); background();
   const real h = (real(0.8) + 2*contactThreshold) * std::sqrt(real(2));   // a grid span
   std::vector<SphereID> b; std::vector<real> rr;
   for (int i = 0; i < 8; ++i) {
      const real r = h/2 - contactThreshold - real(0.00005) * (1 + i);     // just below half a span
      const real oldmin = h * (1 + i) - real(0.00005);                     // AABB min just below a boundary
      SphereID s = createSphere(200 + i, Vec3(oldmin + r + contactThreshold, 0.5 + 2*i, 0.5), r, granite);
      s->setLinearVel((4*contactThreshold + real(0.0004)) / real(0.001), 0, 0);  // 0.035 % of a diameter
      b.push_back(s); rr.push_back(r);
   }
   theWorld()->simulationStep(real(0.001));
   std::vector<Vec3> pts;
   for (size_t i = 0; i < b.size(); ++i) {
      addSurfacePoints(pts, b[i]->getPosition(), rr[i], real(1e-8));
      for (int k = -2; k <= 2; ++k)                                        // points straddling cell boundaries
         pts.push_back(Vec3(h*(1+i) + k*real(1e-5), b[i]->getPosition()[1], b[i]->getPosition()[2]));
   }
   // control: no refresh after the motion, so the stale cells are expected to show
   return report("B polydisperse [control]", sweep(pts), false, true);
}

static int stageC() {                      // every shape the query switches on
   theWorld()->clear(); background();
   std::vector<Vec3> pts;
   SphereID    sp = createSphere   (300, Vec3( 2, 2, 2), real(0.6), granite);
   BoxID       bx = createBox      (301, Vec3( 6, 2, 2), real(1.0), real(0.8), real(0.6), granite);
   CapsuleID   cp = createCapsule  (302, Vec3(10, 2, 2), real(0.4), real(1.4), granite);
   CylinderID  cy = createCylinder (303, Vec3(14, 2, 2), real(0.5), real(1.2), granite);
   EllipsoidID el = createEllipsoid(304, Vec3(18, 2, 2), real(0.8), real(0.5), real(0.35), granite);
   UnionID     un = createTetrasphere(305, Vec3(22, 2, 2), real(0.5), granite, true);
   bx->rotate(real(0.3), real(0.4), real(0.5));                 // body-frame containment under rotation
   cy->rotate(real(0.7), real(0.1), real(0.2));
   el->rotate(real(0.2), real(0.9), real(0.4));
   // the exhaustive path skips FIXED cylinders and meshes; include one of each to pin the policy down
   CylinderID cyFixed = createCylinder(306, Vec3(26, 2, 2), real(0.5), real(1.2), granite); cyFixed->setFixed(true);
   BoxID      bxFixed = createBox     (307, Vec3(30, 2, 2), real(1.0), real(1.0), real(1.0), granite); bxFixed->setFixed(true);
   theWorld()->simulationStep(real(0.001));
   const Vec3 cs[8] = { sp->getPosition(), bx->getPosition(), cp->getPosition(), cy->getPosition(),
                        el->getPosition(), un->getPosition(), cyFixed->getPosition(), bxFixed->getPosition() };
   const real rs[8] = { real(0.6), real(1.0), real(0.9), real(0.8), real(0.8), real(1.0), real(0.8), real(0.9) };
   for (int i = 0; i < 8; ++i) { addSurfacePoints(pts, cs[i], rs[i], real(1e-7)); addLattice(pts, cs[i] - Vec3(1.2,1.2,1.2), cs[i] + Vec3(1.2,1.2,1.2), 9); }
   int rc = report("C multi-shape, rotated", sweep(pts), true, true);
   theWorld()->simulationStep(real(0.02));
   std::vector<Vec3> pts2;
   for (int i = 0; i < 8; ++i) addSurfacePoints(pts2, cs[i], rs[i], real(1e-7));
   rc |= report("C multi-shape after motion", sweep(pts2), true, true);
   return rc;
}

static int stageE() {                      // overlap ownership: characterise, do not assert
   theWorld()->clear(); background();
   SphereID a = createSphere(400, Vec3(real(1.55), real(0.5), real(0.5)), real(0.4), granite); a->setFixed(true);
   SphereID b = createSphere(401, Vec3(real(1.40), real(0.5), real(0.5)), real(0.4), granite); b->setFixed(true);
   BoxID    c = createBox   (402, Vec3(real(5.00), real(0.5), real(0.5)), real(1.0), real(1.0), real(1.0), granite); c->setFixed(true);
   SphereID d = createSphere(403, Vec3(real(5.30), real(0.5), real(0.5)), real(0.4), granite); d->setFixed(true);
   theWorld()->simulationStep(real(0.001));
   std::vector<Vec3> pts;
   for (int k = 0; k <= 20; ++k) pts.push_back(Vec3(real(1.40) + k*real(0.0075), real(0.5), real(0.5)));
   for (int k = 0; k <= 20; ++k) pts.push_back(Vec3(real(5.00) + k*real(0.0150), real(0.5), real(0.5)));
   return report("E overlap ownership", sweep(pts), false, true);
}

static int stageF(unsigned seed) {         // randomised: shapes, sizes, positions, motion
   theWorld()->clear(); background();
   std::srand(seed);
   auto rnd = [](real lo, real hi) { return lo + (hi - lo) * (real(std::rand()) / real(RAND_MAX)); };
   std::vector<Vec3> centres; std::vector<real> radii;
   for (int i = 0; i < 24; ++i) {
      const Vec3 p(rnd(-6, 12), rnd(-6, 12), rnd(-3, 6));
      const real r = rnd(real(0.15), real(0.9));
      switch (i % 5) {
         case 0: createSphere   (500+i, p, r, granite); break;
         case 1: createBox      (500+i, p, r, r*real(1.3), r*real(0.7), granite); break;
         case 2: createCapsule  (500+i, p, r*real(0.6), r*real(1.5), granite); break;
         case 3: createEllipsoid(500+i, p, r, r*real(0.6), r*real(1.4), granite); break;
         default: createCylinder(500+i, p, r*real(0.7), r*real(1.1), granite); break;
      }
      BodyID bd = theCollisionSystem()->getBodyStorage().at(theCollisionSystem()->getBodyStorage().size()-1);
      bd->rotate(rnd(0, 3), rnd(0, 3), rnd(0, 3));
      bd->setLinearVel(rnd(-2, 2), rnd(-2, 2), rnd(-2, 2));
      centres.push_back(p); radii.push_back(r * real(1.6));
   }
   int rc = 0;
   for (int step = 0; step < 4; ++step) {
      theWorld()->simulationStep(real(0.002));
      std::vector<Vec3> pts;
      for (size_t i = 0; i < centres.size(); ++i) {
         BodyID bd = theCollisionSystem()->getBodyStorage().at(i + 40);
         addSurfacePoints(pts, bd->getPosition(), radii[i], real(1e-7));
      }
      addLattice(pts, Vec3(-7,-7,-4), Vec3(13,13,7), 11);
      char name[64]; std::snprintf(name, sizeof name, "F random seed=%u step=%d", seed, step);
      rc |= report(name, sweep(pts), true, true);
   }
   return rc;
}


// Adversarial placement generalised over shapes: the background bodies set the grid
// span, each test body is sized to nearly fill one cell, its AABB minimum is placed
// just below a cell boundary, and it then moves by a fraction of a per cent of its own
// size. Query points sit just inside the advancing surface. This is the review's
// sphere construction, applied to every shape the classification switches on.
static int stageG() {
   theWorld()->clear();
   const real bgSize = real(0.8);                       // background AABB edge -> grid span
   for (int i = 0; i < 40; ++i)
      createSphere(9000+i, Vec3(60 + 3*i, 60, 60), bgSize/2, granite)->setFixed(true);
   const real h  = (bgSize + 2*contactThreshold) * std::sqrt(real(2));   // the span pe will pick
   const real ex = h/2 - contactThreshold - real(0.00005);               // half-extent, just under
   const real disp = 4*contactThreshold + real(0.0004);                  // the small motion
   const char* names[5] = { "sphere", "box", "capsule", "ellipsoid", "cylinder" };
   int rc = 0;
   for (int k = 0; k < 5; ++k) {
      theWorld()->clear();
      for (int i = 0; i < 40; ++i)
         createSphere(9000+i, Vec3(60 + 3*i, 60, 60), bgSize/2, granite)->setFixed(true);
      const real oldmin = h - real(0.00005);            // AABB min just below the boundary at h
      const Vec3 c(oldmin + ex + contactThreshold, real(0.5), real(0.5));
      BodyID body = 0;
      switch (k) {
         case 0: body = createSphere   (700, c, ex, granite); break;
         case 1: body = createBox      (700, c, 2*ex, 2*ex, 2*ex, granite); break;
         case 2: body = createCapsule  (700, c, ex*real(0.45), ex*real(1.1), granite); break;
         case 3: body = createEllipsoid(700, c, ex, ex*real(0.9), ex*real(0.8), granite); break;
         default:body = createCylinder (700, c, ex, 2*ex, granite); break;  // radius ex -> AABB fills the cell span, like the other shapes
      }
      // put the body's REAL AABB minimum just below the cell boundary: assuming the
      // half-extent is ex mis-places shapes whose AABB is not 2*ex wide (the cylinder)
      {
         const real aabbMinX = body->getAABB()[0];
         const Vec3 q = body->getPosition();
         body->setPosition(q[0] + (oldmin - aabbMinX), q[1], q[2]);
      }
      body->setLinearVel(disp / real(0.001), 0, 0);
      theWorld()->simulationStep(real(0.001));
      std::vector<Vec3> pts;
      const Vec3 p = body->getPosition();
      const real halfX = (body->getAABB()[3] - body->getAABB()[0]) / 2;
      for (int j = 0; j <= 40; ++j)                     // a line through the advancing surface
         pts.push_back(Vec3(p[0] + halfX*real(0.999) - j*halfX*real(0.05), p[1], p[2]));
      for (int j = -3; j <= 3; ++j)                     // and points straddling the next boundary
         pts.push_back(Vec3(2*h + j*real(1e-5), p[1], p[2]));
      char name[64]; std::snprintf(name, sizeof name, "G adversarial %s [control]", names[k]);
      rc |= report(name, sweep(pts), false, true);   // documents the defect; stage R is the verdict
   }
   return rc;
}


#if PE_USE_CGAL
// Triangle meshes, with and without the DistanceMap acceleration. The query skips
// FIXED meshes in both paths, so every mesh here is mobile; a fixed one is included
// to confirm both paths agree on that policy.
static int stageD() {
   theWorld()->clear();
   for (int i = 0; i < 40; ++i)
      createSphere(9000+i, Vec3(60 + 3*i, 60, 60), real(0.4), granite)->setFixed(true);
   TriangleMeshID tet = createRegularTetrahedron(800, Vec3(2, 2, 2), real(0.8), granite);
   TriangleMeshID box = createTriangulatedBox   (801, Vec3(6, 2, 2), Vec3(real(1.2), real(0.9), real(0.7)), granite);
   TriangleMeshID fix = createRegularTetrahedron(802, Vec3(10, 2, 2), real(0.8), granite); fix->setFixed(true);
   box->enableDistanceMapAcceleration(32, 3);          // resolution, tolerance
   std::printf("    DistanceMap: tetrahedron=%d triangulated box=%d (box is the accelerated one)\n",
               int(tet->hasDistanceMap()), int(box->hasDistanceMap()));
   theWorld()->simulationStep(real(0.001));
   std::vector<Vec3> pts;
   addSurfacePoints(pts, tet->getPosition(), real(0.9), real(1e-6));
   addSurfacePoints(pts, box->getPosition(), real(0.8), real(1e-6));
   addSurfacePoints(pts, fix->getPosition(), real(0.9), real(1e-6));
   addLattice(pts, Vec3(0.5, 0.5, 0.5), Vec3(11.5, 3.5, 3.5), 12);
   int rc = report("D meshes + DistanceMap", sweep(pts), true, true);
   tet->rotate(real(0.5), real(0.3), real(0.2));
   box->rotate(real(0.2), real(0.6), real(0.1));
   theWorld()->simulationStep(real(0.002));
   std::vector<Vec3> pts2;
   addSurfacePoints(pts2, tet->getPosition(), real(0.9), real(1e-6));
   addSurfacePoints(pts2, box->getPosition(), real(0.8), real(1e-6));
   rc |= report("D meshes after rotation", sweep(pts2), true, true);
   return rc;
}
#endif


#if PE_USE_CGAL
// DistanceMap against the exact mesh test, on identical geometry. An approximate
// method is not expected to be bit-identical: the question is whether it agrees
// everywhere except a thin band around the surface, and how wide that band is.
static int stageD2() {
   theWorld()->clear();
   for (int i = 0; i < 40; ++i)
      createSphere(9000+i, Vec3(60 + 3*i, 60, 60), real(0.4), granite)->setFixed(true);
   const Vec3 L(real(1.2), real(0.9), real(0.7));
   TriangleMeshID exact = createTriangulatedBox(810, Vec3(2, 2, 2), L, granite);
   TriangleMeshID dmap  = createTriangulatedBox(811, Vec3(8, 2, 2), L, granite);
   dmap->enableDistanceMapAcceleration(32, 3);
   std::printf("    exact mesh hasDistanceMap=%d | accelerated mesh hasDistanceMap=%d\n",
               int(exact->hasDistanceMap()), int(dmap->hasDistanceMap()));
   theWorld()->simulationStep(real(0.001));
   // identical relative sample sets around each body, including a fine radial scan
   long disagree = 0, total = 0; real bandMin = real(1e30), bandMax = 0;
   for (int ix = -28; ix <= 28; ++ix)
   for (int iy = -22; iy <= 22; ++iy)
   for (int iz = -18; iz <= 18; ++iz) {
      // offset so no sample lands exactly on a face: ties there are arbitrary, not error
      const Vec3 rel(ix*real(0.03) + real(0.0017), iy*real(0.03) + real(0.0011), iz*real(0.03) + real(0.0007));
      double pa[3] = { exact->getPosition()[0]+rel[0], exact->getPosition()[1]+rel[1], exact->getPosition()[2]+rel[2] };
      double pb[3] = { dmap ->getPosition()[0]+rel[0], dmap ->getPosition()[1]+rel[1], dmap ->getPosition()[2]+rel[2] };
      int ia=0, ib=0; short int ba[8]={}, bb[8]={};
      const bool ra = pointInsideParticles(0, &ia, pa, ba);
      const bool rb = pointInsideParticles(0, &ib, pb, bb);
      ++total;
      if (ra != rb) {
         ++disagree;
         // distance from the box surface, in the body frame, for the disagreeing point
         const real dx = std::max(real(0), std::fabs(rel[0]) - L[0]/2);
         const real dy = std::max(real(0), std::fabs(rel[1]) - L[1]/2);
         const real dz = std::max(real(0), std::fabs(rel[2]) - L[2]/2);
         const real outside = std::sqrt(dx*dx + dy*dy + dz*dz);
         const real inside  = std::min(std::min(L[0]/2 - std::fabs(rel[0]), L[1]/2 - std::fabs(rel[1])), L[2]/2 - std::fabs(rel[2]));
         const real band = (outside > 0) ? outside : std::max(real(0), inside);
         bandMin = std::min(bandMin, band); bandMax = std::max(bandMax, band);
      }
   }
   std::printf("[%-26s] grid=%-3s points=%-7ld disagree=%-6ld (%.3f %%) band from surface: %.4g .. %.4g\n",
               "D2 DistanceMap vs exact", gridActive() ? "ON" : "off", total, disagree,
               100.0*disagree/double(total), disagree ? double(bandMin) : 0.0, double(bandMax));
   const real cell = std::max(std::max(L[0], L[1]), L[2]) / real(32);
   std::printf("    DistanceMap resolution 32 over the longest edge %.3g -> cell %.4g; "
               "disagreement confined within %.4g of the surface is consistent with that sampling\n",
               double(L[0]), double(cell), double(bandMax));
   return 0;                                   // characterisation, not an assertion
}
#endif


// The inpr contract: both queries write the SOLID FLAG through inpr, and the identity
// through the byte array. They must write the same flag for the same point. The
// accelerated path used to write bytes[0] + 1 - the most significant byte of the
// system id plus one - which equals 1 only while that byte is zero, i.e. below id 2^56.
static int stageH() {
   theWorld()->clear();
   for (int i = 0; i < 40; ++i)
      createSphere(9000+i, Vec3(60 + 3*i, 60, 60), real(0.4), granite)->setFixed(true);
   SphereID s = createSphere(900, Vec3(2, 2, 2), real(0.7), granite);
   theWorld()->simulationStep(real(0.001));
   double p[3] = { s->getPosition()[0], s->getPosition()[1], s->getPosition()[2] };
   int ia = -1, ib = -1; short int ba[8] = {}, bb[8] = {};
   const bool ra = pointInsideParticlesAccelerated(0, &ia, p, ba);
   const bool rb = pointInsideParticles(0, &ib, p, bb);
   const bool ok = (ra && rb && ia == ib);
   std::printf("[%-26s] grid=%-3s accelerated: hit=%d inpr=%d id=%llu | exhaustive: hit=%d inpr=%d id=%llu -> %s\n",
               "H inpr contract", gridActive() ? "ON" : "off", int(ra), ia,
               (unsigned long long)ByteArrayToUint64(ba), int(rb), ib,
               (unsigned long long)ByteArrayToUint64(bb), ok ? "ok" : "FAIL");
   if (!ok) std::printf("    the two paths disagree about the solid flag for a point inside a body\n");
   return ok ? 0 : 1;
}


// The same adversarial constructions, but with the spatial index refreshed after motion and
// before the queries. If the stale cell assignment is the cause, these must all pass.
static int stageR() {
   theWorld()->clear();
   const real bgSize = real(0.8);
   for (int i = 0; i < 40; ++i)
      createSphere(9000+i, Vec3(60 + 3*i, 60, 60), bgSize/2, granite)->setFixed(true);
   const real h  = (bgSize + 2*contactThreshold) * std::sqrt(real(2));
   const real ex = h/2 - contactThreshold - real(0.00005);
   const real disp = 4*contactThreshold + real(0.0004);
   const char* names[5] = { "sphere", "box", "capsule", "ellipsoid", "cylinder" };
   int rc = 0;
   for (int k = 0; k < 5; ++k) {
      theWorld()->clear();
      for (int i = 0; i < 40; ++i)
         createSphere(9000+i, Vec3(60 + 3*i, 60, 60), bgSize/2, granite)->setFixed(true);
      const real oldmin = h - real(0.00005);
      const Vec3 c(oldmin + ex + contactThreshold, real(0.5), real(0.5));
      BodyID body = 0;
      switch (k) {
         case 0: body = createSphere   (700, c, ex, granite); break;
         case 1: body = createBox      (700, c, 2*ex, 2*ex, 2*ex, granite); break;
         case 2: body = createCapsule  (700, c, ex*real(0.45), ex*real(1.1), granite); break;
         case 3: body = createEllipsoid(700, c, ex, ex*real(0.9), ex*real(0.8), granite); break;
         default:body = createCylinder (700, c, ex, 2*ex, granite); break;  // radius ex -> AABB fills the cell span, like the other shapes
      }
      // put the body's REAL AABB minimum just below the cell boundary: assuming the
      // half-extent is ex mis-places shapes whose AABB is not 2*ex wide (the cylinder)
      {
         const real aabbMinX = body->getAABB()[0];
         const Vec3 q = body->getPosition();
         body->setPosition(q[0] + (oldmin - aabbMinX), q[1], q[2]);
      }
      body->setLinearVel(disp / real(0.001), 0, 0);
      theWorld()->simulationStep(real(0.001));
      theCollisionSystem()->getCoarseDetector().refreshSpatialIndex();   // <- the fix under test
      std::vector<Vec3> pts;
      const Vec3 p = body->getPosition();
      const real halfX = (body->getAABB()[3] - body->getAABB()[0]) / 2;
      for (int j = 0; j <= 40; ++j) pts.push_back(Vec3(p[0] + halfX*real(0.999) - j*halfX*real(0.05), p[1], p[2]));
      for (int j = -3; j <= 3; ++j) pts.push_back(Vec3(2*h + j*real(1e-5), p[1], p[2]));
      char name[64]; std::snprintf(name, sizeof name, "R refreshed %s", names[k]);
      rc |= report(name, sweep(pts), true, true);
   }
   return rc;
}

int main(int argc, char** argv) {
   theWorld()->setGravity(0, 0, 0);
   theWorld()->setDamping(1);
   std::printf("pe gridActivationThreshold is 32 bodies; every stage asserts the grid is ACTIVE.\n\n");
   int rc = 0;
   rc |= stageA();
   rc |= stageB();
   rc |= stageC();
   rc |= stageE();
   rc |= stageG();
   rc |= stageH();
   rc |= stageR();
#if PE_USE_CGAL
   rc |= stageD();
   rc |= stageD2();
#else
   std::printf("[D meshes + DistanceMap  ] SKIPPED: this pe was built without CGAL\n");
#endif
   const unsigned seeds[] = { 1, 7, 2026 };
   for (int i = 0; i < 3; ++i) rc |= stageF(seeds[i]);
   std::printf("\n%s\n", rc ? "RESULT: at least one must-pass stage FAILED" : "RESULT: all must-pass stages ok");
   return rc;
}
