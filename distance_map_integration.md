# DistanceMap Class Integration Plan

This document outlines the steps to fully integrate the `DistanceMap` class into the `pe` library.

### 1. Namespace Integration

- **Action:** Move the entire `DistanceMap` class definition into the `pe` namespace.
- **Benefit:** This is the standard practice for library components and allows for unqualified name lookup (e.g., `real` instead of `pe::real`).

### 2. Data Type Unification

- **Action:** Replace the primitive `double` type with the library's `pe::real` typedef for all variables and function parameters representing physical quantities.
- **Affected Files:**
    - `DistanceMap.h` (core class)
    - `cgal_box.cpp` (example usage)
    - `VtkOutput.h` (export helpers)
- **Benefit:** Ensures data type consistency across the entire `pe` library.

### 3. Header Management

- **Action:** Ensure that all modified files include the necessary `pe` headers, such as `<pe/config/Precision.h>`, to make `pe::real` available.

### 4. Mitigate Name Clashes

- **Action:** Analyze and mitigate potential name clashes with third-party dependencies, especially CGAL. The recommended approach is to use the Pimpl (Pointer to Implementation) idiom to completely hide CGAL types and headers from the public `DistanceMap.h` header file.
- **Benefit:** Prevents polluting the `pe` namespace, improves compile times, and creates a stable, encapsulated API.

---

### 5. Recommended Implementation Strategy: Pimpl Idiom

To address the name clash issue, the **Pimpl (Pointer to Implementation)** idiom is the recommended approach. It hides all third-party dependencies (like CGAL) from the library's public-facing headers.

#### How It Works

**1. Public Header (`DistanceMap.h`)**

This file becomes a thin wrapper. It does **not** include any CGAL headers. It only forward-declares an internal implementation class and holds a private pointer (usually a `std::unique_ptr`) to it.

```cpp
// In pe/DistanceMap.h
#include <pe/config/Precision.h>
#include <memory> // For std::unique_ptr

namespace pe {
  class DistanceMap {
  public:
    DistanceMap(/* constructor args */);
    ~DistanceMap(); // Important! Needs to be in .cpp

    // Public interface is clean of CGAL types
    real interpolateDistance(real x, real y, real z) const;
    // ... other public methods

  private:
    class Impl; // Forward-declare the implementation class
    std::unique_ptr<Impl> _pimpl; // Pointer to implementation
  };
}
```

**2. Implementation File (`DistanceMap.cpp`)**

This file contains the private `Impl` class definition and all the CGAL-related logic and headers.

```cpp
// In pe/DistanceMap.cpp
#include "DistanceMap.h"

// All heavy includes are isolated here!
#include <CGAL/Surface_mesh.h>
#include <CGAL/AABB_tree.h>
// ... other CGAL headers

// The implementation class is defined only in the .cpp
class pe::DistanceMap::Impl {
public:
    // CGAL types are freely used here
    CGAL::Surface_mesh<...> _mesh;
    CGAL::AABB_tree<...> _tree;

    void computeSdf() { /* ... */ }
    // ... all the real logic
};

// Constructor, destructor, and methods are defined here
pe::DistanceMap::DistanceMap(/*...*/) : _pimpl{std::make_unique<Impl>()} {
    // ... initialize the implementation ...
}
pe::DistanceMap::~DistanceMap() = default;

pe::real pe::DistanceMap::interpolateDistance(real x, real y, real z) const {
    // Forward the call to the implementation
    return _pimpl->interpolateDistance(x, y, z);
}
```