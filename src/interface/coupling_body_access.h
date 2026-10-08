#ifndef PE_INTERFACE_INTERNAL_COUPLING_BODY_ACCESS_H
#define PE_INTERFACE_INTERNAL_COUPLING_BODY_ACCESS_H

#include <pe/core/CollisionSystem.h>
#include <pe/core/rigidbody/RigidBody.h>

#include <cstddef>
#include <stdexcept>
#include <vector>

namespace pe {
namespace interface {
namespace detail {

// Internal CFD access to ghost replicas. This initial backend requires a
// collision system with mutable, separate shadow storage. It does not route
// updates or synchronize; callers retain responsibility for those operations.
// Indices and handles must not be retained across synchronization/removal.
template<class System>
class GhostBodyAccess {
 public:
  explicit GhostBodyAccess(System& system) : system_(system) {}

  std::size_t size() const {
    return system_.getBodyShadowCopyStorage().size();
  }

  BodyID at(std::size_t index) const {
    if (index >= size()) {
      throw std::out_of_range("CFD ghost body index out of range");
    }
    return system_.getBodyShadowCopyStorage().at(index);
  }

  // Returns nullptr when this rank has no ghost with the requested system ID.
  BodyID find(id_t systemID) const {
    auto& bodies = system_.getBodyShadowCopyStorage();
    const auto found = bodies.find(systemID);
    return found == bodies.end() ? nullptr : *found;
  }

  // Visits each ghost once in storage order. The callback may modify body
  // state, but must not add/remove bodies or synchronize during traversal.
  template<class Function>
  void forEach(Function function) const {
    for (BodyID body : system_.getBodyShadowCopyStorage()) {
      function(body);
    }
  }

  // Preserve the existing Fortran remote-particle map's filtering and order.
  // This is deliberately distinct from all-ghost enumeration (size/at).
  std::vector<std::size_t> particleIndices() const {
    std::vector<std::size_t> indices;
    std::size_t index = 0;
    forEach([&](ConstBodyID body) {
      const auto type = body->getType();
      if (type == sphereType || type == capsuleType || type == ellipsoidType ||
          (type == cylinderType && !body->isFixed()) || type == triangleMeshType) {
        indices.push_back(index);
      }
      ++index;
    });
    return indices;
  }

 private:
  System& system_;
};

inline GhostBodyAccess<CollisionSystem<Config>> ghostBodies() {
  return GhostBodyAccess<CollisionSystem<Config>>(*theCollisionSystem());
}

}  // namespace detail
}  // namespace interface
}  // namespace pe

#endif
