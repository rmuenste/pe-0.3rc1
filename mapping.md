# Fortran to C++ Function Mapping

This document tracks the current interface surface defined in
`source/src_particles/dem_query.f90` and the C++ entry points in `libs/pe`.
It is intentionally split into:
- live mappings that still exist in the active code path
- legacy or orphaned entries that are present in the tree but no longer form a
  complete Fortran -> C++ path

## Function Mapping Chain

The mapping follows this pattern:
**Fortran Interface** -> **extern "C" Function** -> **C++ Implementation**

## Particle Count Functions

### getTotalParticles
- **Fortran Interface**: `getTotalParticles()` (line 35 in `dem_query.f90`)
- **extern "C" Function**: `getTotalParticles()` (`./pe/interface/c_interface_queries.h`)
- **C++ Implementation**: `getTotalParts()` (`./src/interface/object_queries.cpp`)

### getNumParticles
- **Fortran Interface**: `getNumParticles()` (line 46 in `dem_query.f90`)
- **extern "C" Function**: `getNumParticles()` (`./pe/interface/c_interface_queries.h`)
- **C++ Implementation**: `getNumParts()` (`./src/interface/object_queries.cpp`)

### getNumRemParticles
- **Fortran Interface**: `getNumRemParticles()` (line 56 in `dem_query.f90`)
- **extern "C" Function**: `getNumRemParticles()` (`./pe/interface/c_interface_queries.h`)
- **C++ Implementation**: `getNumRemParts()` (`./src/interface/object_queries.cpp`)

## Particle Access Functions

### getParticle2
- **Fortran Interface**: `getParticle2(idx, particle)` (line 70 in `dem_query.f90`)
- **extern "C" Function**: `getParticle2()` (`./pe/interface/c_interface_particle_getset.h`)
- **C++ Implementation**: `getPartStructByIdx()` (`./src/interface/object_queries.cpp`)

### setParticle2
- **Fortran Interface**: `setParticle2(particle)` (line 83 in `dem_query.f90`)
- **extern "C" Function**: `setParticle2()` (`./pe/interface/c_interface_particle_getset.h`)
- **C++ Implementation**: `setPartStruct()` (`./src/interface/object_queries.cpp`)

## Remote Particle Functions

### getRemoteParticle2
- **Fortran Interface**: `getRemoteParticle2(idx, particle)` (line 107 in `dem_query.f90`)
- **extern "C" Function**: `getRemoteParticle2()` (`./pe/interface/c_interface_particle_getset.h`)
- **C++ Implementation**: `getRemPartStructByIdx()` (`./src/interface/object_queries.cpp`)

### setRemoteParticle2
- **Fortran Interface**: `setRemoteParticle2(particle)` (line 123 in `dem_query.f90`)
- **extern "C" Function**: `setRemoteParticle2()` (`./pe/interface/c_interface_particle_getset.h`)
- **C++ Implementation**: `setRemPartStruct()` (`./src/interface/object_queries.cpp`)

## Geometry and Type Queries

### isSphere
- **Fortran Interface**: `isSphere(idx)` -> bound to `isTypeSphere` (line 163 in `dem_query.f90`)
- **extern "C" Function**: `isTypeSphere()` (`./pe/interface/c_interface_queries.h`)
- **C++ Implementation**: `isSphereType()` (`./src/interface/object_queries.cpp`)

### getParticleRadius
- **Fortran Interface**: `getParticleRadius(idx)` (line 173 in `dem_query.f90`)
- **extern "C" Function**: `getParticleRadius()` (`./pe/interface/c_interface_queries.h`)
- **C++ Implementation**: `getObjRadius()` (`./src/interface/object_queries.cpp`)

### pointInsideObject
- **Fortran Interface**: `pointInsideObject(idx, pos)` (line 139 in `dem_query.f90`)
- **extern "C" Function**: `pointInsideObject()` (`./pe/interface/c_interface_particle_fbm.h`)
- **C++ Implementation**: `isInsideObject()` (`./src/interface/object_queries.cpp`)

### pointInsideRemObject
- **Fortran Interface**: `pointInsideRemObject(idx, pos)` (line 151 in `dem_query.f90`)
- **extern "C" Function**: `pointInsideRemObject()` (`./pe/interface/c_interface_particle_fbm.h`)
- **C++ Implementation**: `isInsideRemObject()` (`./src/interface/object_queries.cpp`)

## Mapping and Utility Functions

### check_rem_id
- **Fortran Interface**: `check_rem_id(fbmid, id)` (line 163 in `dem_query.f90`)
- **extern "C" Function**: `check_rem_id()` (`./pe/interface/c_interface_queries.h`)
- **C++ Implementation**: `checkRemoteFBM()` (`./src/interface/object_queries.cpp`)

### rem_particles_index_map
- **Fortran Interface**: `rem_particles_index_map(idxMap)` (line 234 in `dem_query.f90`)
- **extern "C" Function**: `rem_particles_index_map()` (`./pe/interface/c_interface_particle_fbm.h`)
- **C++ Implementation**: `getRemoteParticlesIndexMap()` (`./src/interface/object_queries.cpp`)

### particles_index_map
- **Fortran Interface**: `particles_index_map(idxMap)` (line 241 in `dem_query.f90`)
- **extern "C" Function**: `particles_index_map()` (`./pe/interface/c_interface_particle_fbm.h`)
- **C++ Implementation**: `getParticlesIndexMap()` (`./src/interface/object_queries.cpp`)

### get_bytes
- **Fortran Interface**: `get_bytes(bytes)` (line 191 in `dem_query.f90`)
- **extern "C" Function**: not found
- **C++ Implementation**: no live implementation found

## Legacy / Removed Entries

These names still appear in comments, wrappers, or internal helpers, but they no
longer form a complete active Fortran -> C++ interface path.

### getParticle
- Removed from the current `dem_query.f90` interface surface.
- `getLocalParticle()` still contains a call to `getParticle` at
  [dem_query.f90:353](</home/rafa/code/FeatFloWer/FeatFloWer/source/src_particles/dem_query.f90:353>),
  so that call site should be updated or removed.

### getRemoteParticle
- Removed from the current `dem_query.f90` interface surface.
- The old C++ implementation comment remains in
  `./src/interface/object_queries.cpp`, but there is no live Fortran binding.

### setForces
- Removed from the current `dem_query.f90` interface surface.

### setRemoteForces
- Removed from the current `dem_query.f90` interface surface.
- The wrapper still exists in `./pe/interface/c_interface_particle_getset.h`,
  but it is no longer backed by a live Fortran binding.

### setParticle
- Removed from the current `dem_query.f90` interface surface.

### map_local_to_system
- Removed from the current `dem_query.f90` interface surface.

### map_local_to_system2
- Removed from the current `dem_query.f90` interface surface.

### pointInsideParticles(int* inpr, double pos[3])
- Declaration still exists in `./pe/interface/object_queries.h`, but there is no
  active C++ implementation or call path.

### particleMapping / remoteParticleMapping
- Internal helpers in `./src/interface/object_queries.cpp`.
- No remaining public callers after removing `map_particles_` and
  `rem_map_particles_`.

### setRemoteObjByIdx
- Internal helper in `./src/interface/object_queries.cpp`.
- No live callers remain.

## Summary

### Live mappings: 14
- getTotalParticles, getNumParticles, getNumRemParticles
- getParticle2, setParticle2
- getRemoteParticle2, setRemoteParticle2
- pointInsideObject, pointInsideRemObject
- isSphere/isTypeSphere, getParticleRadius
- check_rem_id, rem_particles_index_map, particles_index_map

### Legacy or removed entries
- getParticle, getRemoteParticle, setForces, setRemoteForces, setParticle
- map_local_to_system, map_local_to_system2
- pointInsideParticles(int* inpr, double pos[3])
- particleMapping, remoteParticleMapping
- setRemoteObjByIdx
- get_bytes

## Interface Headers

- `./pe/interface/c_interface_queries.h` - particle count and type queries
- `./pe/interface/c_interface_particle_getset.h` - particle get/set operations
- `./pe/interface/c_interface_particle_fbm.h` - FBM-related functions and mappings
- `./pe/interface/object_queries.h` - C++ declarations
