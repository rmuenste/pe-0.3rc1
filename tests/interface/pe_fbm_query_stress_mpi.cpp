// Parallel (MPI) stress test for the FBM classification query.
//
// The serial harness answers "do the two queries agree for a point and a set of bodies".
// In parallel the question gains three axes that no serial stage can reach:
//
//   * shadow copies - the exhaustive path walks bodystorage AND ghostBodies(); the
//     accelerated path returns whatever the grid holds. The two only diverge once
//     shadows exist, so a body straddling a subdomain cut is the interesting case.
//   * the synchronize() seam - a shadow that arrives after findContacts() waits in
//     bodiesToAdd_; getBodiesNearPoint() looks there (pe #42), but where the index
//     refresh sits relative to that arrival decides whether the two agree.
//   * migration - a body crossing the cut changes owner and representation, and a
//     stale index entry then attributes a point to a body the rank no longer owns.
//
// Every verdict is reduced over all ranks: a mismatch anywhere fails the run, and no
// rank's silence is taken for agreement.
//
// Run: mpirun -np 2 ./fbm_query_stress_mpi     (also valid with 4)

#include <mpi.h>
#include "pe_fbm_query_stress_common.h"

using namespace pe;
using namespace fbmstress;

static int rank_ = 0, nranks_ = 1;
static real CUT = real(0);                 // the subdomain cut plane, x = CUT
static real g_lo = real(0), g_hi = real(0); // this rank's own x-extent, for body placement
static bool g_refresh = false;             // call refreshSpatialIndex() after motion?
static bool g_mustPass = false;            // pass 0 is the control: it documents the defect
static const char* g_tag = "";             // label suffix, so both passes are distinguishable

// Every stage steps through this, so the two passes differ only in the refresh.
static void advance(real dt) {
   theWorld()->simulationStep(dt);
   if (g_refresh) theCollisionSystem()->getCoarseDetector().refreshSpatialIndex();
}

// ---------------------------------------------------------------- reporting
// Reduce a stage over all ranks. mustPass stages fail the run on any disagreement.
static int reportMPI(const char* stage, const Stats& s, bool mustPass, bool wantGrid) {
   long local[6]  = { s.queries, s.accelHits, s.baseHits, s.hitMiss, s.idDiff, s.inprDiff };
   long global[6] = {};
   MPI_Allreduce(local, global, 6, MPI_LONG, MPI_SUM, MPI_COMM_WORLD);
   int localGrid = gridActive() ? 1 : 0, gridMin = 0;
   MPI_Allreduce(&localGrid, &gridMin, 1, MPI_INT, MPI_MIN, MPI_COMM_WORLD);

   // every rank prints its own line, so a single bad rank cannot hide behind the total
   for (int r = 0; r < nranks_; ++r) {
      MPI_Barrier(MPI_COMM_WORLD);
      if (r == rank_ && s.bad() > 0) {
         std::printf("    [rank %d] %s: hit/miss=%ld idDiff=%ld\n", rank_, stage, s.hitMiss, s.idDiff);
         for (size_t i = 0; i < s.examples.size(); ++i) std::printf("      %s\n", s.examples[i].c_str());
         std::fflush(stdout);
      }
   }
   MPI_Barrier(MPI_COMM_WORLD);
   const long bad = global[3] + global[4];
   if (rank_ == 0) {
      std::printf("[%-28s] grid(all)=%-3s queries=%-7ld accelHits=%-6ld baseHits=%-6ld "
                  "hit/miss=%-5ld idDiff=%-5ld -> %s\n",
                  stage, gridMin ? "ON" : "off", global[0], global[1], global[2],
                  global[3], global[4], bad == 0 ? "ok" : (mustPass ? "FAIL" : "differs"));
      if (wantGrid && !gridMin)
         std::printf("    ERROR: a rank has no active grid; that rank tested nothing\n");
      std::fflush(stdout);
   }
   if (wantGrid && !gridMin) return 1;
   return (mustPass && bad) ? 1 : 0;
}

// ------------------------------------------------------------ decomposition
// Two subdomains split at x = CUT; with 4 ranks, four slabs.
static void setupDomain() {
   const real width = real(20);
   if (nranks_ == 2) {
      CUT = real(0);
      if (rank_ == 0) { defineLocalDomain(HalfSpace(Vec3(-1,0,0), -CUT)); connect(1, HalfSpace(Vec3(+1,0,0), CUT)); g_lo = CUT - real(10); g_hi = CUT; }
      else            { defineLocalDomain(HalfSpace(Vec3(+1,0,0),  CUT)); connect(0, HalfSpace(Vec3(-1,0,0), -CUT)); g_lo = CUT; g_hi = CUT + real(10); }
   } else {
      const real lo = -width/2 + rank_*(width/nranks_);
      const real hi = lo + width/nranks_;
      CUT = hi;
      if (rank_ == 0)            defineLocalDomain(HalfSpace(Vec3(-1,0,0), -hi));
      else if (rank_ == nranks_-1) defineLocalDomain(HalfSpace(Vec3(+1,0,0), lo));
      else                       defineLocalDomain(intersect(HalfSpace(Vec3(+1,0,0), lo), HalfSpace(Vec3(-1,0,0), -hi)));
      if (rank_ > 0)           connect(rank_-1, HalfSpace(Vec3(-1,0,0), -lo));
      if (rank_ < nranks_-1)   connect(rank_+1, HalfSpace(Vec3(+1,0,0),  hi));
      g_lo = lo; g_hi = hi;
   }
}

// Enough bodies for pe to build its grid ON EVERY RANK. The threshold of 32 is per
// rank, not global: background bodies spread across the cut leave each side below it
// and the grid never switches on - the first run of this harness did exactly that and
// passed every stage while testing nothing, which is why the grid assertion exists.
static void background() {
   // A 4 x 4 x 3 lattice of 48 fixed spheres INSIDE this rank's own slab, parked far from
   // the query region. The 32-body threshold is per rank: bodies spread across the cut, or
   // a lattice wider than a slab, leave ranks below it and the grid never switches on - the
   // first version of this harness did that at 2 ranks and the 4-rank run did it again.
   const real span = g_hi - g_lo;
   const real dx   = (span > real(5)) ? real(1.0) : span / real(5);   // fit 4 columns in the slab
   const real x0   = (g_lo + g_hi)/2 - real(1.5)*dx;
   int uid = 9000;
   for (int i = 0; i < 4; ++i)
   for (int j = 0; j < 4; ++j)
   for (int k = 0; k < 3; ++k) {
      const Vec3 p(x0 + i*dx, real(38) + j*real(1.0), real(38) + k*real(1.0));
      if (theCollisionSystem()->getDomain().ownsPoint(p))
         createSphere(uid++, p, real(0.4), granite)->setFixed(true);
   }
}

// --------------------------------------------------------------- the stages
static int stageM1() {                      // bodies far from the cut: the serial question, in parallel
   theWorld()->clear(); background();
   for (int i = 0; i < 10; ++i) {
      const Vec3 p(real(-8) + real(1.7)*i, real(1), real(1));
      if (theCollisionSystem()->getDomain().ownsPoint(p)) createSphere(100+i, p, real(0.5), granite);
   }
   advance(real(0.001));
   std::vector<Vec3> pts;
   addLattice(pts, Vec3(-10,-1,-1), Vec3(10,3,3), 12);
   char n[64]; std::snprintf(n, sizeof n, "M1 away from the cut%s", g_tag); return reportMPI(n, sweep(pts), g_mustPass, true);
}

static int stageM2() {                      // a body straddling the cut -> shadow copy next door
   theWorld()->clear(); background();
   const Vec3 c(CUT, real(1), real(1));     // centred exactly on the cut plane
   if (theCollisionSystem()->getDomain().ownsPoint(c)) createSphere(200, c, real(1.2), granite);
   advance(real(0.001));
   std::vector<Vec3> pts;
   addSurfacePoints(pts, c, real(1.2), real(1e-6));
   for (int k = -20; k <= 20; ++k)          // a line crossing the cut through the body
      pts.push_back(Vec3(CUT + k*real(0.08), real(1), real(1)));
   char n[64]; std::snprintf(n, sizeof n, "M2 body on the cut%s", g_tag); return reportMPI(n, sweep(pts), g_mustPass, true);
}

static int stageM3() {                      // query between synchronize() and the next findContacts
   theWorld()->clear(); background();
   const Vec3 start(CUT - real(2.5), real(1), real(1));
   if (theCollisionSystem()->getDomain().ownsPoint(start)) {
      SphereID s = createSphere(300, start, real(1.0), granite);
      s->setLinearVel(real(500), 0, 0);     // crosses into the neighbour within one step
   }
   advance(real(0.005));
   std::vector<Vec3> pts;
   for (int k = -30; k <= 30; ++k) pts.push_back(Vec3(CUT + k*real(0.12), real(1), real(1)));
   char n[64]; std::snprintf(n, sizeof n, "M3 after the sync seam%s", g_tag); return reportMPI(n, sweep(pts), g_mustPass, true);
}

static int stageM4() {                      // migration: ownership changes sides
   theWorld()->clear(); background();
   const Vec3 start(CUT - real(1.5), real(1), real(1));
   if (theCollisionSystem()->getDomain().ownsPoint(start)) {
      SphereID s = createSphere(400, start, real(0.8), granite);
      s->setLinearVel(real(300), 0, 0);
   }
   for (int step = 0; step < 3; ++step) advance(real(0.002));
   std::vector<Vec3> pts;
   for (int k = -30; k <= 30; ++k) pts.push_back(Vec3(CUT + k*real(0.15), real(1), real(1)));
   char n[64]; std::snprintf(n, sizeof n, "M4 after migration%s", g_tag); return reportMPI(n, sweep(pts), g_mustPass, true);
}

static int stageM5() {                      // every shape, straddling the cut, rotated
   theWorld()->clear(); background();
   const char* names[5] = { "sphere", "box", "capsule", "ellipsoid", "cylinder" };
   int rc = 0;
   for (int k = 0; k < 5; ++k) {
      theWorld()->clear(); background();
      const Vec3 c(CUT, real(1), real(1));
      BodyID b = 0;
      if (theCollisionSystem()->getDomain().ownsPoint(c)) {
         switch (k) {
            case 0: b = createSphere   (500, c, real(1.0), granite); break;
            case 1: b = createBox      (500, c, real(1.8), real(1.4), real(1.2), granite); break;
            case 2: b = createCapsule  (500, c, real(0.5), real(1.6), granite); break;
            case 3: b = createEllipsoid(500, c, real(1.1), real(0.8), real(0.6), granite); break;
            default:b = createCylinder (500, c, real(0.7), real(1.5), granite); break;
         }
         if (b) b->rotate(real(0.3), real(0.5), real(0.2));
      }
      advance(real(0.001));
      std::vector<Vec3> pts;
      addSurfacePoints(pts, c, real(1.3), real(1e-6));
      for (int j = -24; j <= 24; ++j) pts.push_back(Vec3(CUT + j*real(0.07), real(1), real(1)));
      addLattice(pts, c - Vec3(1.6,1.6,1.6), c + Vec3(1.6,1.6,1.6), 9);
      char name[64]; std::snprintf(name, sizeof name, "M5 %s on the cut%s", names[k], g_tag);
      rc |= reportMPI(name, sweep(pts), g_mustPass, true);
   }
   return rc;
}

int main(int argc, char** argv) {
   MPI_Init(&argc, &argv);
   MPI_Comm_rank(MPI_COMM_WORLD, &rank_);
   MPI_Comm_size(MPI_COMM_WORLD, &nranks_);
   theWorld()->setGravity(0, 0, 0);
   theWorld()->setDamping(1);
   setupDomain();
   if (rank_ == 0) {
      std::printf("parallel FBM query stress: %d ranks, cut plane at x = %g\n", nranks_, double(CUT));
      std::printf("every stage is reduced over all ranks; a mismatch on any rank fails the run\n\n");
      std::fflush(stdout);
   }
   int rc = 0;
   for (int pass = 0; pass < 2; ++pass) {
      g_refresh = (pass == 1);
      g_mustPass = g_refresh;
      g_tag = g_refresh ? " [refreshed]" : "";
      if (rank_ == 0) std::printf("--- pass %d: %s ---\n", pass, g_refresh ? "refreshSpatialIndex() after motion" : "no refresh (control)");
      rc |= stageM1();
      rc |= stageM2();
      rc |= stageM3();
      rc |= stageM4();
      rc |= stageM5();
      if (rank_ == 0) std::printf("\n");
   }
   int worst = 0;
   MPI_Allreduce(&rc, &worst, 1, MPI_INT, MPI_MAX, MPI_COMM_WORLD);
   if (rank_ == 0) std::printf("\n%s\n", worst ? "RESULT: at least one must-pass stage FAILED" : "RESULT: all must-pass stages ok");
   MPI_Finalize();
   return worst;
}
