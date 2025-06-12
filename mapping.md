# Fortran to C++ Function Mapping

This document maps the Fortran interface functions defined in `source/src_particles/dem_query.f90` to their C++ implementations through the extern "C" interface layer.

## Function Mapping Chain

The mapping follows this pattern:
**Fortran Interface** → **extern "C" Function** → **C++ Implementation**

---

## Particle Count Functions

### getTotalParticles
- **Fortran Interface**: `getTotalParticles()` (line 35 in dem_query.f90)
- **extern "C" Function**: `getTotalParticles()` (libs/pe/pe/interface/c_interface_queries.h:16)
- **C++ Implementation**: `getTotalParts()` (libs/pe/src/interface/object_queries.cpp:592-613)

### getNumParticles
- **Fortran Interface**: `getNumParticles()` (line 46 in dem_query.f90)
- **extern "C" Function**: `getNumParticles()` (libs/pe/pe/interface/c_interface_queries.h:25)
- **C++ Implementation**: `getNumParts()` (libs/pe/src/interface/object_queries.cpp:621-648)

### getNumRemParticles
- **Fortran Interface**: `getNumRemParticles()` (line 56 in dem_query.f90)
- **extern "C" Function**: `getNumRemParticles()` (libs/pe/pe/interface/c_interface_queries.h:35)
- **C++ Implementation**: `getNumRemParts()` (libs/pe/src/interface/object_queries.cpp:654-658)

---

## Particle Access Functions

### getParticle
- **Fortran Interface**: `getParticle(idx, lidx, uidx, time, pos, vel)` (line 65 in dem_query.f90)
- **extern "C" Function**: `getParticle()` (libs/pe/pe/interface/c_interface_particle_getset.h:59)
- **C++ Implementation**: `getObjByIdx()` (libs/pe/src/interface/object_queries.cpp:756-788)

### getParticle2
- **Fortran Interface**: `getParticle2(idx, particle)` (line 80 in dem_query.f90)
- **extern "C" Function**: `getParticle2()` (libs/pe/pe/interface/c_interface_particle_getset.h:78)
- **C++ Implementation**: `getPartStructByIdx()` (libs/pe/src/interface/object_queries.cpp:1018-1058)

### setParticle2
- **Fortran Interface**: `setParticle2(particle)` (line 92 in dem_query.f90)
- **extern "C" Function**: `setParticle2()` (libs/pe/pe/interface/c_interface_particle_getset.h:89)
- **C++ Implementation**: `setPartStruct()` (libs/pe/src/interface/object_queries.cpp:1066-1102)

### setParticle
- **Fortran Interface**: `setParticle(idx, lidx, uidx, time, pos, vel)` (line 141 in dem_query.f90)
- **extern "C" Function**: Not found in interface headers (missing mapping)
- **C++ Implementation**: `setObjByIdx()` (libs/pe/src/interface/object_queries.cpp:848-873)

---

## Remote Particle Functions

### getRemoteParticle
- **Fortran Interface**: `getRemoteParticle(idx, lidx, uidx, time, pos, vel)` (line 103 in dem_query.f90)
- **extern "C" Function**: `getRemoteParticle()` (libs/pe/pe/interface/c_interface_particle_getset.h:116)
- **C++ Implementation**: `getRemoteObjByIdx()` (libs/pe/src/interface/object_queries.cpp:802-834)

### getRemoteParticle2
- **Fortran Interface**: `getRemoteParticle2(idx, particle)` (line 118 in dem_query.f90)
- **extern "C" Function**: `getRemoteParticle2()` (libs/pe/pe/interface/c_interface_particle_getset.h:135)
- **C++ Implementation**: `getRemPartStructByIdx()` (libs/pe/src/interface/object_queries.cpp:1157-1211)

### setRemoteParticle2
- **Fortran Interface**: `setRemoteParticle2(particle)` (line 130 in dem_query.f90)
- **extern "C" Function**: `setRemoteParticle2()` (libs/pe/pe/interface/c_interface_particle_getset.h:100)
- **C++ Implementation**: `setRemPartStruct()` (libs/pe/src/interface/object_queries.cpp:1108-1147)

---

## Force Setting Functions

### setForces
- **Fortran Interface**: `setForces(idx, lidx, uidx, force, torque)` (line 156 in dem_query.f90)
- **extern "C" Function**: `setForces()` (libs/pe/pe/interface/c_interface_particle_getset.h:17)
- **C++ Implementation**: `setForcesByIdx()` (libs/pe/src/interface/object_queries.cpp:924-941)

### setRemoteForces
- **Fortran Interface**: `setRemoteForces(idx, lidx, uidx, force, torque)` (line 170 in dem_query.f90)
- **extern "C" Function**: `setRemoteForces()` (libs/pe/pe/interface/c_interface_particle_getset.h:36)
- **C++ Implementation**: `setRemoteForcesByIdx()` (libs/pe/src/interface/object_queries.cpp:954-971)

---

## Geometric Query Functions

### pointInsideObject
- **Fortran Interface**: `pointInsideObject(idx, pos)` (line 204 in dem_query.f90)
- **extern "C" Function**: `pointInsideObject()` (libs/pe/pe/interface/c_interface_particle_fbm.h:7)
- **C++ Implementation**: `isInsideObject()` (libs/pe/src/interface/object_queries.cpp:533-552)

### pointInsideRemObject
- **Fortran Interface**: `pointInsideRemObject(idx, pos)` (line 215 in dem_query.f90)
- **extern "C" Function**: `pointInsideRemObject()` (libs/pe/pe/interface/c_interface_particle_fbm.h:20)
- **C++ Implementation**: `isInsideRemObject()` (libs/pe/src/interface/object_queries.cpp:561-583)

---

## Type and Property Query Functions

### isSphere (isTypeSphere)
- **Fortran Interface**: `isSphere(idx)` → bound to `isTypeSphere` (line 184 in dem_query.f90)
- **extern "C" Function**: `isTypeSphere()` (libs/pe/pe/interface/c_interface_queries.h:58)
- **C++ Implementation**: `isSphereType()` (libs/pe/src/interface/object_queries.cpp:696-712)

### getParticleRadius
- **Fortran Interface**: `getParticleRadius(idx)` (line 194 in dem_query.f90)
- **extern "C" Function**: `getParticleRadius()` (libs/pe/pe/interface/c_interface_queries.h:45)
- **C++ Implementation**: `getObjRadius()` (libs/pe/src/interface/object_queries.cpp:979-1008)

---

## Mapping and Utility Functions

### check_rem_id
- **Fortran Interface**: `check_rem_id(fbmid, id)` (line 226 in dem_query.f90)
- **extern "C" Function**: `check_rem_id()` (libs/pe/pe/interface/c_interface_queries.h:4)
- **C++ Implementation**: `checkRemoteFBM()` (libs/pe/src/interface/object_queries.cpp:468-486)

### rem_particles_index_map
- **Fortran Interface**: `rem_particles_index_map(idxMap)` (line 234 in dem_query.f90)
- **extern "C" Function**: `rem_particles_index_map()` (libs/pe/pe/interface/c_interface_particle_fbm.h:44)
- **C++ Implementation**: `getRemoteParticlesIndexMap()` (libs/pe/src/interface/object_queries.cpp:154-193)

### particles_index_map
- **Fortran Interface**: `particles_index_map(idxMap)` (line 241 in dem_query.f90)
- **extern "C" Function**: `particles_index_map()` (libs/pe/pe/interface/c_interface_particle_fbm.h:77)
- **C++ Implementation**: `getParticlesIndexMap()` (libs/pe/src/interface/object_queries.cpp:115-145)

### get_bytes
- **Fortran Interface**: `get_bytes(bytes)` (line 248 in dem_query.f90)
- **extern "C" Function**: Not found in interface headers (missing extern "C" wrapper)
- **C++ Implementation**: Likely related to `uint64toByteArray()` (object_queries.h:335)

### map_local_to_system
- **Fortran Interface**: `map_local_to_system(lidx, vidx)` (line 255 in dem_query.f90)
- **extern "C" Function**: Not found in interface headers (missing extern "C" wrapper)
- **C++ Implementation**: Unknown - implementation not located

### map_local_to_system2
- **Fortran Interface**: `map_local_to_system2(lidx, vidx)` (line 263 in dem_query.f90)
- **extern "C" Function**: Not found in interface headers (missing extern "C" wrapper)
- **C++ Implementation**: Unknown - implementation not located

---

## Summary

### Complete Mappings: 15 functions
These functions have complete Fortran → extern "C" → C++ mappings:
- getTotalParticles, getNumParticles, getNumRemParticles
- getParticle, getParticle2, setParticle2
- getRemoteParticle, getRemoteParticle2, setRemoteParticle2
- setForces, setRemoteForces
- pointInsideObject, pointInsideRemObject
- isSphere/isTypeSphere, getParticleRadius
- check_rem_id, rem_particles_index_map, particles_index_map

### Incomplete Mappings: 4 functions
These functions have Fortran interfaces but missing extern "C" wrappers:
- setParticle (has C++ implementation but no extern "C" wrapper found)
- get_bytes (implementation unclear)
- map_local_to_system (implementation not found)
- map_local_to_system2 (implementation not found)

### Primary Implementation File
**All C++ implementations are in**: `libs/pe/src/interface/object_queries.cpp`

### Interface Header Files
- `libs/pe/pe/interface/c_interface_queries.h` - Particle count and type queries
- `libs/pe/pe/interface/c_interface_particle_getset.h` - Particle get/set operations
- `libs/pe/pe/interface/c_interface_particle_fbm.h` - FBM-related functions and mappings
- `libs/pe/pe/interface/object_queries.h` - C++ function declarations