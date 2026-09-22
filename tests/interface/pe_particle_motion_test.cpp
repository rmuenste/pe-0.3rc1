//=================================================================================================
/*!
 *  \file tests/interface/pe_particle_motion_test.cpp
 *  \brief Pins the D6.4 particleMotion_ modes "translationOnly" and "free" of the DNS-drag
 *         xyz path, next to the pre-existing "fixed" and "rotationOnly".
 *
 *  Config contract (pe/config/SimulationConfig.h, src/config/SimulationConfig.cpp):
 *    - setParticleMotion / json particleMotion_ accept exactly fixed, rotationOnly,
 *      translationOnly, free; any other value -> std::invalid_argument naming the key.
 *  Setup guard (pe/interface/sim_setup_serial_features.h):
 *    - angularDofMask_ under free -> accepted (mask honoured);
 *    - angularDofMask_ under translationOnly -> std::invalid_argument naming BOTH
 *      "angularDofMask_" and "translationOnly";
 *    - effectiveAngularDofMask: (0,0,0) for translationOnly, the deck mask otherwise.
 *  Body level (applyParticleMotionLocks, the one routine every creation path and the resume
 *  loop go through), for a sphere AND an ellipsoid after applyFluidForces with a nonzero force
 *  and torque:
 *    - translationOnly: v advanced (bit-identical to an unlocked body), w == (0,0,0) exactly;
 *    - free: v and w both advanced, bit-identical to an unlocked body; not fixed;
 *    - rotationOnly (control): v == 0 exactly, w advanced;
 *    - fixed (control): isFixed().
 *
 *  SimulationConfig is a process singleton whose loadFromFile() only overwrites present keys;
 *  the deck cases therefore set particleMotion_ explicitly in every deck. Decks are written
 *  under ./_particle_motion_test/. Serial world, no MPI.
 */
//=================================================================================================

#include <pe/core.h>
#include <pe/config/SimulationConfig.h>
#include <pe/interface/sim_setup_serial_features.h>

#include <boost/filesystem.hpp>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>

using namespace pe;

namespace {

int failures = 0;

void expect(bool ok, const std::string& what) {
  if (!ok) {
    std::printf("FAIL: %s\n", what.c_str());
    ++failures;
  }
}

// Exact (bit-for-bit) comparison; no tolerance on purpose.
bool same(const Vec3& a, const Vec3& b) {
  return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
}

const boost::filesystem::path deckDir("_particle_motion_test");

std::string writeDeck(const std::string& name, const std::string& body) {
  const boost::filesystem::path p = deckDir / (name + ".json");
  std::ofstream out(p.string());
  out << body;
  return p.string();
}

// Returns the invalid_argument message if loadFromFile threw one, "" if it did not throw.
std::string loadThrows(const std::string& file) {
  try {
    SimulationConfig::loadFromFile(file);
  } catch (const std::invalid_argument& e) {
    return e.what();
  }
  return "";
}

std::string setterThrows(SimulationConfig& config, const std::string& motion) {
  try {
    config.setParticleMotion(motion);
  } catch (const std::invalid_argument& e) {
    return e.what();
  }
  return "";
}

std::string guardThrows(const SimulationConfig& config) {
  try {
    checkAngularDofMaskRequiresRotationOnly(config);
  } catch (const std::invalid_argument& e) {
    return e.what();
  }
  return "";
}

//------------------------------------------------------------------------------------------------
// Config / guard contract
//------------------------------------------------------------------------------------------------
void configChecks() {
  boost::filesystem::create_directories(deckDir);
  SimulationConfig& config = SimulationConfig::getInstance();

  // --- setter: the four values, and only those ------------------------------------------------
  const char* accepted[] = {"fixed", "rotationOnly", "translationOnly", "free"};
  for (const char* m : accepted) {
    expect(setterThrows(config, m).empty(), std::string("setter accepts '") + m + "'");
    expect(config.getParticleMotion() == m, std::string("setter stores '") + m + "'");
  }
  const char* rejected[] = {"Free", "translation", "spin", "", "translationonly"};
  for (const char* m : rejected) {
    config.setParticleMotion("fixed");
    const std::string msg = setterThrows(config, m);
    expect(!msg.empty() && msg.find("particleMotion_") != std::string::npos,
           std::string("setter rejects '") + m + "' with a message naming particleMotion_");
    expect(config.getParticleMotion() == "fixed",
           std::string("rejected '") + m + "' leaves the previous value in place");
  }

  // --- json: the four values parse, an unknown value throws ----------------------------------
  for (const char* m : accepted) {
    const std::string deck = writeDeck(std::string("motion_") + m,
                                       std::string("{ \"particleMotion_\": \"") + m + "\" }");
    expect(loadThrows(deck).empty(), std::string("deck particleMotion_ = ") + m + " loads");
    expect(config.getParticleMotion() == m, std::string("deck particleMotion_ = ") + m + " is read");
  }
  {
    config.setParticleMotion("fixed");
    const std::string msg = loadThrows(writeDeck("motion_bogus", "{ \"particleMotion_\": \"wobble\" }"));
    expect(!msg.empty() && msg.find("particleMotion_") != std::string::npos,
           "deck particleMotion_ = wobble throws invalid_argument naming particleMotion_");
    expect(config.getParticleMotion() == "fixed", "rejected deck leaves particleMotion_ at fixed");
  }

  // --- guard: mask under free accepted and honoured ---------------------------------------------
  expect(loadThrows(writeDeck("free_mask", "{ \"particleMotion_\": \"free\", "
                                           "\"angularDofMask_\": [1, 0, 1] }")).empty(),
         "free + angularDofMask_ [1,0,1] loads");
  expect(config.hasAngularDofMask(), "free + mask: hasAngularDofMask() == true");
  expect(guardThrows(config).empty(), "guard accepts angularDofMask_ under free");
  expect(same(effectiveAngularDofMask(config), Vec3(1, 0, 1)),
         "effective mask under free is the deck mask (1,0,1)");

  // --- guard: mask under translationOnly refused, message names both ------------------------
  config.setParticleMotion("translationOnly");
  {
    const std::string msg = guardThrows(config);
    expect(!msg.empty(), "guard refuses angularDofMask_ under translationOnly");
    expect(msg.find("angularDofMask_") != std::string::npos, "refusal names angularDofMask_");
    expect(msg.find("translationOnly") != std::string::npos, "refusal names translationOnly");
  }
  expect(same(effectiveAngularDofMask(config), Vec3(0, 0, 0)),
         "effective mask under translationOnly is (0,0,0) regardless of the deck mask");

  // --- guard: mask under fixed still refused; rotationOnly still accepted -----------------------
  config.setParticleMotion("fixed");
  expect(!guardThrows(config).empty(), "guard still refuses angularDofMask_ under fixed");
  config.setParticleMotion("rotationOnly");
  expect(guardThrows(config).empty(), "guard still accepts angularDofMask_ under rotationOnly");
  expect(same(effectiveAngularDofMask(config), Vec3(1, 0, 1)),
         "effective mask under rotationOnly is the deck mask");
}

//------------------------------------------------------------------------------------------------
// Body-level behaviour
//------------------------------------------------------------------------------------------------
const Vec3 w0(real(0.4), real(-0.3), real(0.2));
const Vec3 v0(real(0.3), real(-0.2), real(0.1));
const Vec3 torque(real(0.7), real(-1.3), real(2.1));
const Vec3 force(real(0.5), real(0.25), real(-0.125));
const Vec3 deckMask(1, 1, 1);

template <typename ID>
void load(ID b) {
  b->setLinearVel(v0);
  b->setAngularVel(w0);
  b->addForce(force);
  b->addTorque(torque);
}

template <typename ID>
struct Set {
  ID plain;            // no locks at all (reference)
  ID free;             // particleMotion_ = free
  ID translationOnly;  // particleMotion_ = translationOnly
  ID rotationOnly;     // particleMotion_ = rotationOnly (control)
  ID fixed;            // particleMotion_ = fixed (control)
};

template <typename ID>
void bodyChecks(const Set<ID>& s, const std::string& shape, real dt) {
  applyParticleMotionLocks(s.free, "free", deckMask);
  applyParticleMotionLocks(s.translationOnly, "translationOnly", deckMask);
  applyParticleMotionLocks(s.rotationOnly, "rotationOnly", deckMask);
  applyParticleMotionLocks(s.fixed, "fixed", deckMask);

  expect(!s.free->isFixed(), shape + ": free is not fixed");
  expect(!s.translationOnly->isFixed(), shape + ": translationOnly is not fixed");
  expect(!s.rotationOnly->isFixed(), shape + ": rotationOnly is not fixed");
  expect(s.fixed->isFixed(), shape + ": fixed is fixed");

  expect(same(s.free->getLinearDofMask(), Vec3(1, 1, 1)), shape + ": free linear mask (1,1,1)");
  expect(same(s.free->getAngularDofMask(), Vec3(1, 1, 1)), shape + ": free angular mask (1,1,1)");
  expect(same(s.translationOnly->getLinearDofMask(), Vec3(1, 1, 1)),
         shape + ": translationOnly linear mask (1,1,1)");
  expect(same(s.translationOnly->getAngularDofMask(), Vec3(0, 0, 0)),
         shape + ": translationOnly angular mask (0,0,0)");
  expect(same(s.rotationOnly->getLinearDofMask(), Vec3(0, 0, 0)),
         shape + ": rotationOnly linear mask (0,0,0)");

  load(s.plain);
  load(s.free);
  load(s.translationOnly);
  load(s.rotationOnly);
  s.plain->applyFluidForces(dt);
  s.free->applyFluidForces(dt);
  s.translationOnly->applyFluidForces(dt);
  s.rotationOnly->applyFluidForces(dt);

  // reference actually moved
  expect(!same(s.plain->getLinearVel(), v0), shape + ": unlocked v advanced");
  expect(!same(s.plain->getAngularVel(), w0), shape + ": unlocked w advanced");
  expect(s.plain->getAngularVel()[0] != real(0) && s.plain->getAngularVel()[1] != real(0) &&
             s.plain->getAngularVel()[2] != real(0),
         shape + ": unlocked w has no zero component (so a zero below is the mask, not luck)");

  // translationOnly: v free, w zero
  expect(same(s.translationOnly->getLinearVel(), s.plain->getLinearVel()),
         shape + ": translationOnly v == unlocked v bit-for-bit");
  expect(same(s.translationOnly->getAngularVel(), Vec3(0, 0, 0)),
         shape + ": translationOnly w == (0,0,0) exactly after applyFluidForces");

  // free: both free
  expect(same(s.free->getLinearVel(), s.plain->getLinearVel()),
         shape + ": free v == unlocked v bit-for-bit");
  expect(same(s.free->getAngularVel(), s.plain->getAngularVel()),
         shape + ": free w == unlocked w bit-for-bit");

  // rotationOnly control: v zero, w free
  expect(same(s.rotationOnly->getLinearVel(), Vec3(0, 0, 0)),
         shape + ": rotationOnly v == (0,0,0) exactly");
  expect(same(s.rotationOnly->getAngularVel(), s.plain->getAngularVel()),
         shape + ": rotationOnly w == unlocked w bit-for-bit");

  // the mask is re-applied by move() too (world axes): translationOnly stays spin-free
  load(s.translationOnly);
  s.translationOnly->move(dt);
  expect(same(s.translationOnly->getAngularVel(), Vec3(0, 0, 0)),
         shape + ": translationOnly w == (0,0,0) exactly after move");

  // unknown mode is refused at the body level too
  bool threw = false;
  try {
    applyParticleMotionLocks(s.plain, "wobble", deckMask);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  expect(threw, shape + ": applyParticleMotionLocks refuses an unknown mode");
}

}  // namespace

int main() {
  configChecks();

  WorldID world = theWorld();
  world->setGravity(real(0), real(0), real(0));
  world->setDamping(real(1));
  MaterialID mat = createMaterial("particle_motion_test", real(1.25), real(0.1), real(0.05),
                                  real(0.05), real(0.2), real(80), real(100), real(10), real(11));
  const real dt = real(0.01);

  {
    const real r = real(1.5);
    Set<SphereID> s;
    s.plain = createSphere(1, Vec3(0, 0, 0), r, mat);
    s.free = createSphere(2, Vec3(20, 0, 0), r, mat);
    s.translationOnly = createSphere(3, Vec3(40, 0, 0), r, mat);
    s.rotationOnly = createSphere(4, Vec3(60, 0, 0), r, mat);
    s.fixed = createSphere(5, Vec3(80, 0, 0), r, mat);
    bodyChecks(s, "sphere", dt);
  }
  {
    const real a = real(0.2404), b = real(0.7211), c = real(0.7211);  // the D6.4 oblate body
    Set<EllipsoidID> s;
    s.plain = createEllipsoid(11, Vec3(0, 50, 0), a, b, c, mat);
    s.free = createEllipsoid(12, Vec3(20, 50, 0), a, b, c, mat);
    s.translationOnly = createEllipsoid(13, Vec3(40, 50, 0), a, b, c, mat);
    s.rotationOnly = createEllipsoid(14, Vec3(60, 50, 0), a, b, c, mat);
    s.fixed = createEllipsoid(15, Vec3(80, 50, 0), a, b, c, mat);
    bodyChecks(s, "ellipsoid", dt);
  }

  if (failures == 0) {
    std::printf("pe-particle-motion: all checks passed\n");
    return EXIT_SUCCESS;
  }
  std::printf("pe-particle-motion: %d check(s) failed\n", failures);
  return EXIT_FAILURE;
}
