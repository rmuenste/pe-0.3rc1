#ifndef _PE_INTERFACE_SIM_SETUP_SERIAL_FEATURES_H_
#define _PE_INTERFACE_SIM_SETUP_SERIAL_FEATURES_H_

#include <pe/config/SimulationConfig.h>
#include <pe/core.h>
#include <pe/core/detection/fine/DistanceMap.h>
#include <pe/core/rigidbody/Sphere.h>
#include <pe/core/rigidbody/TriangleMesh.h>
#include <pe/util/logging/Logger.h>
#include <algorithm>
#include <deque>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <vector>

namespace pe {

struct CenterlinePosition {
  Vec3 closestPoint;
  real radialDistance;
  real lengthAlongCurve;
  real percentageAlongCurve;
  int segmentIndex;
};

inline real calculateTotalCenterlineLength(const std::vector<Vec3>& centerline) {
  real totalLength = 0.0;
  for (size_t i = 0; i + 1 < centerline.size(); ++i) {
    totalLength += (centerline[i + 1] - centerline[i]).length();
  }
  return totalLength;
}

inline CenterlinePosition calculateCenterlinePosition(
    const Vec3& particlePos,
    const std::vector<Vec3>& centerline,
    real totalLength) {
  if (centerline.size() < 2) {
    return CenterlinePosition{Vec3(0, 0, 0), 0.0, 0.0, 0.0, -1};
  }

  real minDistance = std::numeric_limits<real>::max();
  Vec3 closestPoint;
  real lengthAlongCurve = 0.0;
  real cumulativeLength = 0.0;
  int closestSegmentIndex = -1;

  for (size_t i = 0; i + 1 < centerline.size(); ++i) {
    const Vec3& v0 = centerline[i];
    const Vec3& v1 = centerline[i + 1];
    Vec3 edgeVec = v1 - v0;
    real edgeLength = edgeVec.length();

    if (edgeLength < 1e-10) {
      continue;
    }

    Vec3 toParticle = particlePos - v0;
    real t = trans(toParticle) * edgeVec / (edgeLength * edgeLength);
    t = std::max(real(0), std::min(real(1), t));

    Vec3 pointOnSegment = v0 + edgeVec * t;
    real distance = (particlePos - pointOnSegment).length();

    if (distance < minDistance) {
      minDistance = distance;
      closestPoint = pointOnSegment;
      lengthAlongCurve = cumulativeLength + (t * edgeLength);
      closestSegmentIndex = static_cast<int>(i);
    }

    cumulativeLength += edgeLength;
  }

  real percentageAlongCurve =
      (totalLength > 0.0) ? (lengthAlongCurve / totalLength) * 100.0 : 0.0;

  return CenterlinePosition{
      closestPoint,
      minDistance,
      lengthAlongCurve,
      percentageAlongCurve,
      closestSegmentIndex};
}

struct SerialStepContext {
  WorldID world;
  SimulationConfig& config;
  bool isRepresentative;
  int timestep;
  real fullStepSize;
  int substeps;
};

class SerialStepFeature {
 public:
  virtual ~SerialStepFeature() {}
  virtual void afterMainStep(const SerialStepContext& ctx) = 0;
};

class NullSerialStepFeature : public SerialStepFeature {
 public:
  virtual void afterMainStep(const SerialStepContext&) {}
};

class DomainEscapeReinsertionFeature : public SerialStepFeature {
 private:
  struct QueuedParticle {
    real radius;
    MaterialID material;

    QueuedParticle(real r, MaterialID mat) : radius(r), material(mat) {}
  };

  std::vector<QueuedParticle> particleQueue_;

  TriangleMeshID findBoundaryMesh() const {
    for (auto it = theCollisionSystem()->getBodyStorage().begin();
         it != theCollisionSystem()->getBodyStorage().end(); ++it) {
      BodyID body = *it;
      if (body->getType() != triangleMeshType) {
        continue;
      }

      TriangleMeshID mesh = static_body_cast<TriangleMesh>(body);
#ifdef PE_USE_CGAL
      if (mesh->hasDistanceMap()) {
        return mesh;
      }
#else
      (void)mesh;
#endif
    }

    return TriangleMeshID();
  }

  bool checkOverlap(const Vec3& testPos, real testRadius, real safetyMargin) const {
    for (auto it = theCollisionSystem()->getBodyStorage().begin();
         it != theCollisionSystem()->getBodyStorage().end(); ++it) {
      BodyID otherBody = *it;
      if (otherBody->getType() != sphereType) {
        continue;
      }

      SphereID otherSphere = static_body_cast<Sphere>(otherBody);
      real centerDist = (testPos - otherBody->getPosition()).length();
      real minDist = testRadius + otherSphere->getRadius() + safetyMargin;

      if (centerDist < minDist) {
        return true;
      }
    }

    return false;
  }

  int reinsertQueuedParticles(const SerialStepContext& ctx, const std::vector<Vec3>& safePositions) {
    int reinsertedFromQueue = 0;
    const real safetyMargin = ctx.config.getSerialReinsertionSafetyMargin();
    size_t initialQueueSize = particleQueue_.size();

    for (size_t i = 0; i < initialQueueSize && !safePositions.empty();) {
      QueuedParticle& qp = particleQueue_[i];
      bool inserted = false;

      for (size_t posIdx = 0; posIdx < safePositions.size(); ++posIdx) {
        const Vec3& safePos = safePositions[posIdx];
        if (checkOverlap(safePos, qp.radius, safetyMargin)) {
          continue;
        }

        size_t newID = ctx.world->size();
        SphereID newSphere = createSphere(newID, safePos, qp.radius, qp.material);
        newSphere->setLinearVel(Vec3(0.0, 0.0, 0.0));
        newSphere->setAngularVel(Vec3(0.0, 0.0, 0.0));

        ++reinsertedFromQueue;
        particleQueue_.erase(particleQueue_.begin() + i);
        inserted = true;

        if (ctx.isRepresentative) {
          std::cout << "  -> Reinserted queued particle at position ("
                    << safePos[0] << ", " << safePos[1] << ", " << safePos[2]
                    << ")" << std::endl;
        }
        break;
      }

      if (!inserted) {
        ++i;
      }
    }

    return reinsertedFromQueue;
  }

  std::vector<BodyID> collectEscapedParticles(TriangleMeshID boundaryMesh) const {
    std::vector<BodyID> escapedParticles;

#ifdef PE_USE_CGAL
    if (!boundaryMesh || !boundaryMesh->hasDistanceMap()) {
      return escapedParticles;
    }

    const DistanceMap* dm = boundaryMesh->getDistanceMap();

    for (auto it = theCollisionSystem()->getBodyStorage().begin();
         it != theCollisionSystem()->getBodyStorage().end(); ++it) {
      BodyID body = *it;
      if (body->getType() != sphereType) {
        continue;
      }

      Vec3 posWorld = body->getPosition();
      Vec3 posLocal = boundaryMesh->pointFromWFtoBF(posWorld);
      real distance = dm->interpolateDistance(posLocal[0], posLocal[1], posLocal[2]);

      if (distance < 0.0) {
        escapedParticles.push_back(body);
      }
    }
#else
    (void)boundaryMesh;
#endif

    return escapedParticles;
  }

  void handleEscapedParticles(const SerialStepContext& ctx,
                              const std::vector<Vec3>& safePositions,
                              const std::vector<BodyID>& escapedParticles,
                              int& reinsertedEscaped,
                              int& destroyedCount) {
    const real safetyMargin = ctx.config.getSerialReinsertionSafetyMargin();

    for (size_t escapedIdx = 0; escapedIdx < escapedParticles.size(); ++escapedIdx) {
      BodyID body = escapedParticles[escapedIdx];
      SphereID sphere = static_body_cast<Sphere>(body);
      Vec3 posWorld = body->getPosition();
      real sphereRadius = sphere->getRadius();

      if (ctx.isRepresentative) {
        std::cout << "WARNING: Particle " << body->getSystemID()
                  << " has escaped the domain at world position ("
                  << posWorld[0] << ", " << posWorld[1] << ", " << posWorld[2]
                  << ")" << std::endl;
      }

      bool reinserted = false;
      for (size_t posIdx = 0; posIdx < safePositions.size(); ++posIdx) {
        const Vec3& safePos = safePositions[posIdx];
        if (checkOverlap(safePos, sphereRadius, safetyMargin)) {
          continue;
        }

        body->setPosition(safePos);
        body->setLinearVel(Vec3(0.0, 0.0, 0.0));
        body->setAngularVel(Vec3(0.0, 0.0, 0.0));
        ++reinsertedEscaped;
        reinserted = true;

        if (ctx.isRepresentative) {
          std::cout << "  -> Reinserted particle " << body->getSystemID()
                    << " at safe position (" << safePos[0] << ", "
                    << safePos[1] << ", " << safePos[2] << ")" << std::endl;
        }
        break;
      }

      if (reinserted) {
        continue;
      }

      if (ctx.isRepresentative) {
        std::cout << "  -> All safe positions occupied, destroying particle "
                  << body->getSystemID() << " and adding to queue" << std::endl;
      }

      particleQueue_.push_back(QueuedParticle(sphereRadius, sphere->getMaterial()));

      World::Iterator destroyIt = ctx.world->begin();
      for (; destroyIt != ctx.world->end(); ++destroyIt) {
        if (*destroyIt == body) {
          break;
        }
      }

      if (destroyIt != ctx.world->end()) {
        ctx.world->destroy(destroyIt);
        ++destroyedCount;
      }
    }
  }

 public:
  virtual void afterMainStep(const SerialStepContext& ctx) {
    if (!ctx.config.getSerialEnableEscapeReinsertion()) {
      return;
    }

#ifndef PE_USE_CGAL
    (void)ctx;
    return;
#else
    TriangleMeshID boundaryMesh = findBoundaryMesh();
    if (!boundaryMesh || !boundaryMesh->hasDistanceMap()) {
      return;
    }

    const std::vector<Vec3>& safePositions = ctx.config.getSerialReinsertionSafePositions();
    int reinsertedFromQueue = reinsertQueuedParticles(ctx, safePositions);
    std::vector<BodyID> escapedParticles = collectEscapedParticles(boundaryMesh);
    int escapedCount = static_cast<int>(escapedParticles.size());
    int reinsertedEscaped = 0;
    int destroyedCount = 0;

    handleEscapedParticles(
        ctx, safePositions, escapedParticles, reinsertedEscaped, destroyedCount);

    if ((escapedCount > 0 || reinsertedFromQueue > 0 || !particleQueue_.empty()) &&
        ctx.isRepresentative) {
      std::cout << "PARTICLE MANAGEMENT at timestep " << ctx.timestep << ":\n"
                << "  Escaped: " << escapedCount << "\n"
                << "  Reinserted (escaped): " << reinsertedEscaped << "\n"
                << "  Reinserted (queued): " << reinsertedFromQueue << "\n"
                << "  Destroyed: " << destroyedCount << "\n"
                << "  Queue size: " << particleQueue_.size() << std::endl;
    }
#endif
  }
};

class StuckParticleDiagnosticsFeature : public SerialStepFeature {
 private:
  struct ParticlePositionHistory {
    std::map<size_t, Vec3> positions;
  };

  std::deque<ParticlePositionHistory> positionHistory_;

  ParticlePositionHistory recordCurrentPositions() const {
    ParticlePositionHistory currentHistory;

    for (auto it = theCollisionSystem()->getBodyStorage().begin();
         it != theCollisionSystem()->getBodyStorage().end(); ++it) {
      BodyID body = *it;
      if (body->getType() == sphereType) {
        currentHistory.positions[body->getSystemID()] = body->getPosition();
      }
    }

    return currentHistory;
  }

  void updateHistory(const ParticlePositionHistory& currentHistory, int window) {
    positionHistory_.push_back(currentHistory);
    while (static_cast<int>(positionHistory_.size()) > window) {
      positionHistory_.pop_front();
    }
  }

  void logStuckDiagnostic(const SerialStepContext& ctx,
                          BodyID body,
                          const Vec3& oldPos,
                          const Vec3& currentPos,
                          real displacement,
                          int detectionWindow) const {
    CenterlinePosition clPos = calculateCenterlinePosition(
        currentPos,
        ctx.config.getCenterlineVertices(),
        ctx.config.getTotalCenterlineLength());

    pe_LOG_INFO_SECTION(log) {
      log << "STUCK PARTICLE DIAGNOSTIC at timestep " << ctx.timestep << ":\n";
      log << "  Particle (ID=" << body->getSystemID() << ") STUCK: displacement = "
          << displacement << " over " << detectionWindow << " timesteps\n";
      log << "    Position " << detectionWindow << " steps ago: ("
          << oldPos[0] << ", " << oldPos[1] << ", " << oldPos[2] << ")\n";
      log << "    Current position: ("
          << currentPos[0] << ", " << currentPos[1] << ", " << currentPos[2] << ")\n";
      log << "  Centerline position:\n";
      log << "    Length along centerline: " << clPos.lengthAlongCurve
          << " / " << ctx.config.getTotalCenterlineLength() << "\n";
      log << "    Percentage along centerline: " << clPos.percentageAlongCurve << "%\n";
      log << "    Radial distance from centerline: " << clPos.radialDistance << "\n";

      real tubeRadius = 0.25;
      real wallProximity = tubeRadius - clPos.radialDistance;
      log << "    Distance from tube wall: " << wallProximity;
      if (wallProximity < ctx.config.getSerialStuckWallDistanceThreshold()) {
        log << " [NEAR WALL]";
      }
      log << "\n";

      if (clPos.percentageAlongCurve < 25.0) {
        log << "    [IN FIRST 25% OF CENTERLINE]\n";
      }

      Vec3 velocity = body->getLinearVel();
      log << "  Velocity information:\n";
      log << "    Current velocity: ("
          << velocity[0] << ", " << velocity[1] << ", " << velocity[2] << ")\n";
      log << "    Velocity magnitude: " << velocity.length() << "\n";

      int contactCount = 0;
      int boundaryContacts = 0;
      int particleContacts = 0;
      real maxPenetration = 0.0;
      for (auto contactIt = body->beginContacts(); contactIt != body->endContacts(); ++contactIt) {
        ++contactCount;
        real penetration = -contactIt->getDistance();
        if (penetration > maxPenetration) {
          maxPenetration = penetration;
        }

        BodyID otherBody =
            (contactIt->getBody1() == body) ? contactIt->getBody2() : contactIt->getBody1();
        if (otherBody->getType() == triangleMeshType) {
          ++boundaryContacts;
        } else if (otherBody->getType() == sphereType) {
          ++particleContacts;
        }
      }

      log << "  Contact information:\n";
      log << "    Total active contacts: " << contactCount << "\n";
      log << "    Contacts with boundary: " << boundaryContacts << "\n";
      log << "    Contacts with particles: " << particleContacts << "\n";
      if (contactCount > 0) {
        log << "    Maximum penetration depth: " << maxPenetration << "\n";
        if (maxPenetration > 1e-5) {
          log << "    WARNING: Significant penetration detected!\n";
        }
      }
    }
  }

  void updateStuckFlags(const SerialStepContext& ctx,
                        const ParticlePositionHistory& oldHistory,
                        const ParticlePositionHistory& currentHistory,
                        int detectionWindow,
                        real displacementThreshold) const {
    for (std::map<size_t, Vec3>::const_iterator currentIt = currentHistory.positions.begin();
         currentIt != currentHistory.positions.end(); ++currentIt) {
      size_t particleID = currentIt->first;
      const Vec3& currentPos = currentIt->second;

      std::map<size_t, Vec3>::const_iterator oldIt = oldHistory.positions.find(particleID);
      if (oldIt == oldHistory.positions.end()) {
        continue;
      }

      const Vec3& oldPos = oldIt->second;
      real displacement = (currentPos - oldPos).length();
      if (displacement >= displacementThreshold) {
        continue;
      }

      for (auto bodyIt = theCollisionSystem()->getBodyStorage().begin();
           bodyIt != theCollisionSystem()->getBodyStorage().end(); ++bodyIt) {
        BodyID body = *bodyIt;
        if (body->getSystemID() != particleID) {
          continue;
        }

        body->isStuck_ = true;
        if (ctx.isRepresentative) {
          logStuckDiagnostic(ctx, body, oldPos, currentPos, displacement, detectionWindow);
        }
        break;
      }
    }

    for (auto it = theCollisionSystem()->getBodyStorage().begin();
         it != theCollisionSystem()->getBodyStorage().end(); ++it) {
      BodyID body = *it;
      if (body->getType() != sphereType || !body->isStuck_) {
        continue;
      }

      size_t bodyID = body->getSystemID();
      bool stillStuck = false;
      std::map<size_t, Vec3>::const_iterator currentIt = currentHistory.positions.find(bodyID);
      if (currentIt != currentHistory.positions.end()) {
        std::map<size_t, Vec3>::const_iterator oldIt = oldHistory.positions.find(bodyID);
        if (oldIt != oldHistory.positions.end()) {
          real displacement = (currentIt->second - oldIt->second).length();
          stillStuck = displacement < displacementThreshold;
        }
      }

      if (!stillStuck) {
        body->isStuck_ = false;
      }
    }
  }

  void logStuckSummary(const SerialStepContext& ctx) const {
    if (!ctx.isRepresentative) {
      return;
    }

    size_t stuckCount = 0;
    size_t totalCount = 0;
    int nearWallCount = 0;
    int earlyRegionCount = 0;
    real avgRadialDistance = 0.0;
    real avgPercentage = 0.0;
    const real nearWallRadiusThreshold = 0.25 - ctx.config.getSerialStuckWallDistanceThreshold();

    for (auto it = theCollisionSystem()->getBodyStorage().begin();
         it != theCollisionSystem()->getBodyStorage().end(); ++it) {
      BodyID body = *it;
      if (body->getType() != sphereType) {
        continue;
      }

      ++totalCount;
      if (!body->isStuck_) {
        continue;
      }

      ++stuckCount;
      CenterlinePosition clPos = calculateCenterlinePosition(
          body->getPosition(),
          ctx.config.getCenterlineVertices(),
          ctx.config.getTotalCenterlineLength());

      avgRadialDistance += clPos.radialDistance;
      avgPercentage += clPos.percentageAlongCurve;
      if (clPos.radialDistance > nearWallRadiusThreshold) {
        ++nearWallCount;
      }
      if (clPos.percentageAlongCurve < 25.0) {
        ++earlyRegionCount;
      }
    }

    if (stuckCount == 0) {
      return;
    }

    avgRadialDistance /= stuckCount;
    avgPercentage /= stuckCount;
    real stuckPercentage =
        (totalCount > 0)
            ? (static_cast<real>(stuckCount) / static_cast<real>(totalCount)) * 100.0
            : 0.0;

    pe_LOG_INFO_SECTION(log) {
      log << "STUCK PARTICLE SUMMARY at timestep " << ctx.timestep << ":\n";
      log << "  Total sphere particles: " << totalCount << "\n";
      log << "  Stuck particles: " << stuckCount << " (" << stuckPercentage << "%)\n";
      log << "  Centerline distribution:\n";
      log << "    Avg percentage along centerline: " << avgPercentage << "%\n";
      log << "    In first 25% of centerline: " << earlyRegionCount
          << " (" << (earlyRegionCount * 100.0 / stuckCount) << "%)\n";
      log << "  Radial distribution:\n";
      log << "    Avg radial distance: " << avgRadialDistance << "\n";
      log << "    Near wall (r > " << nearWallRadiusThreshold << "): " << nearWallCount
          << " (" << (nearWallCount * 100.0 / stuckCount) << "%)\n";
    }
  }

 public:
  virtual void afterMainStep(const SerialStepContext& ctx) {
    if (!ctx.config.getSerialEnableStuckDiagnostics()) {
      return;
    }

    if (!ctx.isRepresentative) {
      return;
    }

    if (ctx.config.getCenterlineVertices().size() < 2 ||
        ctx.config.getTotalCenterlineLength() <= 0.0) {
      return;
    }

    const int detectionWindow = ctx.config.getSerialStuckDetectionWindow();
    const real displacementThreshold = ctx.config.getSerialStuckDisplacementThreshold();
    if (detectionWindow <= 0) {
      return;
    }

    ParticlePositionHistory currentHistory = recordCurrentPositions();
    updateHistory(currentHistory, detectionWindow);
    if (static_cast<int>(positionHistory_.size()) < detectionWindow) {
      return;
    }

    const ParticlePositionHistory& oldHistory = positionHistory_.front();
    updateStuckFlags(
        ctx, oldHistory, currentHistory, detectionWindow, displacementThreshold);
    logStuckSummary(ctx);
  }
};

class SerialStepFeatureSet {
 private:
  bool escapeEnabled_;
  bool stuckEnabled_;
  std::unique_ptr<SerialStepFeature> escapeFeature_;
  std::unique_ptr<SerialStepFeature> stuckFeature_;

  void ensureFeatures(const SimulationConfig& config) {
    if (!escapeFeature_ || escapeEnabled_ != config.getSerialEnableEscapeReinsertion()) {
      escapeEnabled_ = config.getSerialEnableEscapeReinsertion();
      if (escapeEnabled_) {
        escapeFeature_.reset(new DomainEscapeReinsertionFeature());
      } else {
        escapeFeature_.reset(new NullSerialStepFeature());
      }
    }

    if (!stuckFeature_ || stuckEnabled_ != config.getSerialEnableStuckDiagnostics()) {
      stuckEnabled_ = config.getSerialEnableStuckDiagnostics();
      if (stuckEnabled_) {
        stuckFeature_.reset(new StuckParticleDiagnosticsFeature());
      } else {
        stuckFeature_.reset(new NullSerialStepFeature());
      }
    }
  }

 public:
  SerialStepFeatureSet()
      : escapeEnabled_(false), stuckEnabled_(false), escapeFeature_(), stuckFeature_() {}

  void afterMainStep(const SerialStepContext& ctx) {
    ensureFeatures(ctx.config);
    escapeFeature_->afterMainStep(ctx);
    stuckFeature_->afterMainStep(ctx);
  }
};

inline SerialStepFeatureSet& serialStepFeatureSet() {
  static SerialStepFeatureSet featureSet;
  return featureSet;
}

}  // namespace pe

#endif
