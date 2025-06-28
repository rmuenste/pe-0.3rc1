# MPI Data Communication Analysis

## Marshalling and Unmarshalling Overview

The PE physics engine uses a template-based marshalling system for serializing rigid body data for MPI communication. The marshalling functions are defined in `pe/core/Marshalling.h`.

## General Marshalling Architecture

### Template-Based Design
```cpp
template< typename Buffer, typename T >
inline void marshal( Buffer& buffer, const T& obj );

template< typename Buffer, typename T >
inline void unmarshal( Buffer& buffer, T& obj );
```

The system uses template specializations for different data types, allowing type-safe serialization/deserialization.

## Rigid Body Base Data Structure

### RigidBody Marshalling (`pe/core/Marshalling.h:184-198`)
```cpp
template< typename Buffer >
inline void marshal( Buffer& buffer, const RigidBody& obj ) {
   buffer << obj.getSystemID();        // System-wide unique ID
   buffer << obj.getID();              // User-defined ID  
   buffer << obj.isVisible();          // Visibility flag
   buffer << obj.isFixed();            // Fixed/movable flag
   marshal( buffer, obj.getPosition() );     // 3D position vector
   if( obj.hasSuperBody() )
      marshal( buffer, obj.getRelPosition() ); // Relative position if part of union
   marshal( buffer, obj.getQuaternion() );   // Orientation quaternion
   if( !obj.hasSuperBody() ) {
      marshal( buffer, obj.getLinearVel() );  // Linear velocity
      marshal( buffer, obj.getAngularVel() ); // Angular velocity
   }
}
```

### Data Fields Transmitted:
1. **System ID** (id_t) - Global unique identifier
2. **User ID** (id_t) - User-defined identifier
3. **Visibility flag** (bool) - Rendering visibility
4. **Fixed flag** (bool) - Whether body can move
5. **Position** (Vec3) - Global 3D coordinates
6. **Relative Position** (Vec3) - Only for union sub-bodies
7. **Quaternion** (Quat) - Orientation
8. **Linear Velocity** (Vec3) - Only for top-level bodies
9. **Angular Velocity** (Vec3) - Only for top-level bodies

## Triangle Mesh MPI Data Communication

### Triangle Mesh Marshalling (`pe/core/Marshalling.h:486-495`)
```cpp
template< typename Buffer >
inline void marshal( Buffer& buffer, const TriangleMesh& obj ) {
   marshal( buffer, static_cast<const GeomPrimitive&>( obj ) );  // Base class data
   buffer << obj.getBFVertices().size();                        // Number of vertices
   for( size_t i = 0; i < obj.getBFVertices().size(); ++i )
      marshal( buffer, obj.getBFVertices()[i] );                // Each vertex (Vec3)
   buffer << obj.getFaceIndices().size();                       // Number of faces
   for( size_t i = 0; i < obj.getFaceIndices().size(); ++i )
      marshal( buffer, obj.getFaceIndices()[i] );               // Each face (Vec3 of indices)
}
```

### Triangle Mesh Unmarshalling (`pe/core/Marshalling.h:507-524`)
```cpp
template< typename Buffer >
inline void unmarshal( Buffer& buffer, TriangleMesh::Parameters& objparam, bool hasSuperBody ) {
   unmarshal( buffer, static_cast<GeomPrimitive::Parameters&>( objparam ), hasSuperBody );
   
   size_t numVertices;
   buffer >> numVertices;
   objparam.vertices_.resize( numVertices );
   
   for( size_t i = 0; i < objparam.vertices_.size(); ++i )
      unmarshal( buffer, objparam.vertices_[i] );
   
   size_t numFaces;
   buffer >> numFaces;
   objparam.faceIndices_.resize( numFaces );
   
   for( size_t i = 0; i < objparam.faceIndices_.size(); ++i )
      unmarshal( buffer, objparam.faceIndices_[i] );
}
```

### Triangle Mesh Data Structure Over MPI:

1. **Base GeomPrimitive Data** (inherited):
   - All RigidBody fields (ID, position, orientation, etc.)
   - Material properties

2. **Vertex Data**:
   - **Count**: `size_t numVertices` - Number of vertices
   - **Vertices**: Array of `Vec3` - 3D coordinates of each vertex in body-fixed frame
   - **Format**: `[x₁, y₁, z₁, x₂, y₂, z₂, ..., xₙ, yₙ, zₙ]`

3. **Face Data**:
   - **Count**: `size_t numFaces` - Number of triangular faces
   - **Face Indices**: Array of `Vec3` - Indices into vertex array (stored as Vec3 but contains integers)
   - **Format**: `[v₁ᵢ, v₁ⱼ, v₁ₖ, v₂ᵢ, v₂ⱼ, v₂ₖ, ..., vₙᵢ, vₙⱼ, vₙₖ]`

### Memory Layout Example:
```
Triangle Mesh MPI Packet:
┌─────────────────┬─────────────────┬─────────────────┬─────────────────┐
│ RigidBody Data  │ GeomPrim Data   │ Vertex Data     │ Face Index Data │
├─────────────────┼─────────────────┼─────────────────┼─────────────────┤
│ - System ID     │ - Material      │ - Vertex Count  │ - Face Count    │
│ - User ID       │                 │ - Vertex Array  │ - Index Array   │
│ - Position      │                 │   [Vec3...]     │   [Vec3...]     │
│ - Orientation   │                 │                 │                 │
│ - Velocities    │                 │                 │                 │
│ - Flags         │                 │                 │                 │
└─────────────────┴─────────────────┴─────────────────┴─────────────────┘
```

## Dynamic Marshalling System

### Dynamic Type Resolution (`pe/core/Marshalling.h:212-246`)
```cpp
template< typename Buffer >
inline void marshalDynamically( Buffer& buffer, const RigidBody& obj ) {
   const GeomType geomType( obj.getType() );
   marshal( buffer, geomType );  // Type identifier first
   
   switch( geomType ) {
      case sphereType:
         marshal( buffer, static_cast<const Sphere&>( obj ) );
         break;
      case triangleMeshType:
         marshal( buffer, static_cast<const TriangleMesh&>( obj ) );
         break;
      // ... other types
   }
}
```

This allows the receiver to reconstruct the correct object type from the serialized data.

## MPI Communication Process in synchronize()

### 1. Assembly Phase (`pe/core/collisionsystem/HardContactSemiImplicitTimesteppingSolvers.h:3102-3344`)
- Iterate through all local bodies
- Determine which processes need updates/copies/removals
- Marshal appropriate notifications into process-specific send buffers

### 2. Communication Phase (`pe/core/collisionsystem/HardContactSemiImplicitTimesteppingSolvers.h:3355-3372`) 
```cpp
if( lastSyncTag_ == mpitagHCTSSynchronizePositionsAndVelocities2 ) {
   communicate( mpitagHCTSSynchronizePositionsAndVelocities );
   lastSyncTag_ = mpitagHCTSSynchronizePositionsAndVelocities;
}
else {
   communicate( mpitagHCTSSynchronizePositionsAndVelocities2 );
   lastSyncTag_ = mpitagHCTSSynchronizePositionsAndVelocities2;
}
```
- Uses alternating MPI tags to avoid race conditions
- Bulk MPI communication between all process pairs

### 3. Parsing Phase (`pe/core/collisionsystem/HardContactSemiImplicitTimesteppingSolvers.h:3392-3651`)
- Process received buffers from all neighboring processes
- Unmarshal notifications and take appropriate actions
- Create/update/remove shadow copies as needed

## Key Design Features

1. **Efficient Serialization**: Template-based approach minimizes overhead
2. **Type Safety**: Dynamic marshalling preserves object types across processes
3. **Selective Communication**: Only necessary data is transmitted based on geometric relationships
4. **Consistency**: Registration system ensures all processes have required shadow copies
5. **Memory Management**: Proper cleanup of shadow copies when no longer needed

## Performance Considerations

1. **Batch Communication**: All notifications for a process are batched into single MPI message
2. **Geometric Filtering**: Only bodies intersecting process domains are communicated
3. **Incremental Updates**: Update notifications only send position/velocity, not full geometry
4. **Memory Pooling**: Reuse of send/receive buffers across synchronization calls