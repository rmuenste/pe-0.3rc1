#include <pe/interface/object_queries.h>
#include <src/interface/coupling_body_access.h>

#include <iostream>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <vector>

// Existing interface functions/state that are not declared in object_queries.h.
void setRemoteObjByIdx(int, int*, int*, double*, double[3], double[3]);
bool mapLocalToSystem(int, int);
extern std::map<int, boost::uint64_t> fbmMapRemote;

namespace {
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

template<class F>
void expectOutOfRange(F call) {
  try { call(); }
  catch (const std::out_of_range&) { return; }
  throw std::runtime_error("Expected std::out_of_range");
}

void makeShadow(pe::BodyID body) {
  // Construct the separate-storage layout without requiring MPI or stepping.
  pe::theWorld()->remove(body);
  body->setRemote(true);
  pe::theCollisionSystem()->getBodyShadowCopyStorage().add(body);
}
}

int main() {
  try {
    using namespace pe;
    const MaterialID mat = createMaterial("remote_query_test", 1, 0.1, 0.05, 0.05,
                                          0.2, 80, 100, 10, 11);
    SphereID owned = createSphere(1, Vec3(0, 0, 0), 1, mat);
    SphereID first = createSphere(2, Vec3(4, 0, 0), 1, mat);
    makeShadow(first);
    // Boxes are excluded from the CFD particle index map.
    makeShadow(createBox(3, Vec3(8, 0, 0), Vec3(1, 1, 1), mat));
    SphereID last = createSphere(4, Vec3(12, 0, 0), 1, mat);
    makeShadow(last);

    const auto ghosts = pe::interface::detail::ghostBodies();
    require(ghosts.size() == 3, "Ghost count includes owned bodies");
    require(ghosts.at(0) == first && ghosts.at(2) == last, "Ghost order changed");
    require(ghosts.find(first->getSystemID()) == first, "System ID lookup failed");
    require(ghosts.find(std::numeric_limits<pe::id_t>::max()) == nullptr,
            "Missing ghost ID must return nullptr");
    std::vector<BodyID> visited;
    ghosts.forEach([&](BodyID body) { visited.push_back(body); });
    require(visited.size() == 3 && visited[0] == first && visited[2] == last &&
                visited[1]->getType() == boxType,
            "Traversal must visit each ghost once, in order, excluding owned bodies");
    require(ghosts.particleIndices() == std::vector<std::size_t>({0, 2}),
            "Particle filter or order changed");
    expectOutOfRange([&] { ghosts.at(3); });
    expectOutOfRange([&] { ghosts.at(static_cast<std::size_t>(-1)); });
    int exportedIndices[3] = {-1, -1, -1};
    getRemoteParticlesIndexMap(exportedIndices);
    require(exportedIndices[0] == 0 && exportedIndices[1] == 2 && exportedIndices[2] == -1,
            "Legacy particle-map output changed");

    int localID = 0, uniqueID = 0;
    double time = 0, pos[3] = {}, vel[3] = {};
    auto get = [&](int i) { getRemoteObjByIdx(i, &localID, &uniqueID, &time, pos, vel); };
    auto set = [&](int i) { setRemoteObjByIdx(i, &localID, &uniqueID, &time, pos, vel); };

    // A valid shadow index can exceed the owned-body count.
    get(2);
    require(pos[0] == 12, "Getter rejected or read the wrong shadow");
    pos[0] = 13;
    vel[0] = 2;
    set(2);
    require(last->getPosition()[0] == 13 && last->getLinearVel()[0] == 2,
            "Setter did not update the requested shadow");

    // Conversely, the owned-body count can exceed the shadow-body count.
    for (int i = 0; i < 4; ++i)
      createSphere(10 + i, Vec3(20 + 4*i, 0, 0), 1, mat);
    for (int i : {-1, 3, std::numeric_limits<int>::max()}) {
      expectOutOfRange([&] { get(i); });
      expectOutOfRange([&] { set(i); });
    }

    fbmMapRemote[7] = last->getSystemID();
    require(mapLocalToSystem(1, 7), "Filtered particle index must select the second sphere");
    require(!mapLocalToSystem(0, 7), "Different body IDs must not match");
    const std::size_t entries = fbmMapRemote.size();
    require(!mapLocalToSystem(0, 99), "Missing vertex must not match");
    require(fbmMapRemote.size() == entries, "Lookup inserted a missing vertex");
    for (int i : {-1, 2, 3, std::numeric_limits<int>::max()})
      expectOutOfRange([&] { mapLocalToSystem(i, 7); });

    // The force setter must still resolve by system ID and leave kinematics alone.
    particleData_t particle = {};
    uint64toByteArray(last->getSystemID(), particle.bytes);
    particle.force[0] = 3;
    setRemPartStruct(&particle);
    require(last->getForce()[0] == 3 && last->getLinearVel()[0] == 2 &&
                last->getPosition()[0] == 13 && owned->getForce()[0] == 0,
            "System-ID force update targeted the wrong body or changed kinematics");
    getRemPartStructByIdx(2, &particle);
    require(particle.position[0] == 13 && particle.velocity[0] == 2,
            "Ghost particle export changed");

    // Single-rank force processing must update owned bodies and ghosts once.
    const real dt = 0.1;
    TimeStep::stepsize(dt);
    owned->setForce(Vec3(3, 0, 0));
    synchronizeForces();
    require(std::fabs(owned->getLinearVel()[0] - dt * 3 / owned->getMass()) < 1e-12 &&
                std::fabs(last->getLinearVel()[0] - (2 + dt * 3 / last->getMass())) < 1e-12,
            "Force traversal skipped or duplicated an owned body or ghost");
    require(owned->getForce()[0] == 0 && last->getForce()[0] == 0,
            "Force application must clear forces as before");

    uint64toByteArray(std::numeric_limits<pe::id_t>::max(), particle.bytes);
    bool missingIDRejected = false;
    try { setRemPartStruct(&particle); }
    catch (const std::logic_error&) { missingIDRejected = true; }
    require(missingIDRejected, "Missing force-update ID must be rejected");

    theWorld()->clear();
    require(ghosts.size() == 0 && ghosts.particleIndices().empty(), "Empty ghost access failed");
    require(ghosts.find(1) == nullptr, "Empty ghost lookup must return nullptr");
    std::size_t emptyVisits = 0;
    ghosts.forEach([&](BodyID) { ++emptyVisits; });
    require(emptyVisits == 0, "Empty traversal called its callback");
    expectOutOfRange([&] { get(0); });
    expectOutOfRange([&] { set(0); });
    expectOutOfRange([&] { mapLocalToSystem(0, 7); });
    return 0;
  }
  catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
