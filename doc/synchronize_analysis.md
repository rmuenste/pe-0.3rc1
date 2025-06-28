# Analysis of CollisionSystem::synchronize() Function

## Function Location
`pe/core/collisionsystem/HardContactSemiImplicitTimesteppingSolvers.h:3089`

## Overview
The `synchronize()` function is responsible for synchronizing rigid bodies across MPI processes in the PE physics engine. It handles body ownership changes, position updates, and maintains shadow copies of remote bodies.

## Notification Logic Analysis

### 1. When Removal Notifications are Sent

**Condition**: `pe/core/collisionsystem/HardContactSemiImplicitTimesteppingSolvers.h:3175-3185`
```cpp
else if( body->isRegistered( *process ) ) {
   // In case the rigid body no longer intersects the remote process nor interacts with it but is registered,
   // send removal notification.
   
   pe_LOG_DEBUG_SECTION( log ) {
      log << "Sending removal notification for body " << b->getSystemID() << " to process " << process->getRank() << ".\n";
   }
   marshal( buffer, notificationType<RigidBodyRemovalNotification>() );
   marshal( buffer, RigidBodyRemovalNotification( *b ) );
   body->deregisterProcess( *process );
}
```

**Triggers**:
- Body no longer intersects with a remote process (`!process->intersectsWith( b )`)
- Body is still registered with that process (`body->isRegistered( *process )`)
- Body remains locally owned (owner hasn't changed)

**Purpose**: Clean up shadow copies that are no longer needed by remote processes.

### 2. When Body Migration Notifications are Sent

**Condition 1 - Local Migration**: `pe/core/collisionsystem/HardContactSemiImplicitTimesteppingSolvers.h:3325-3334`
```cpp
// Send migration notification to new owner
{
   Process::SendBuff& buffer( p->getSendBuffer() );
   
   marshal( buffer, notificationType<RigidBodyMigrationNotification>() );
   marshal( buffer, RigidBodyMigrationNotification( *b, reglist ) );
}
```

**Triggers**:
- Body position update causes it to leave current process domain (`!domain_.ownsPoint( gpos )`)
- New owner is found among neighboring processes (`owner.first >= 0` and `owner.second != NULL`)

**Condition 2 - Remote Migration**: `pe/core/collisionsystem/HardContactSemiImplicitTimesteppingSolvers.h:3310-3312`
```cpp
marshal( buffer, notificationType<RigidBodyRemoteMigrationNotification>() );
marshal( buffer, RigidBodyRemoteMigrationNotification( *b, p->getRank() ) );
```

**Triggers**:
- Same as local migration but sent to processes that have shadow copies (excluding new owner)
- Notifies them that the body ownership changed to a different process

## 3. How Removal and Migration Actions are Handled

### Removal Action Handling (`pe/core/collisionsystem/HardContactSemiImplicitTimesteppingSolvers.h:3598-3621`)
```cpp
case rigidBodyRemovalNotification: {
   // Remove shadow copy as prompted.
   BodyID b( *bodystorageShadowCopies_.find( objparam.sid_ ) );
   
   bodystorageShadowCopies_.remove( b );
   detector_.remove( b );
   
   delete b;
   break;
}
```

**Actions**:
1. Find shadow copy by system ID
2. Remove from shadow copy storage
3. Remove from collision detector
4. Delete the body object

### Migration Action Handling

**Local Migration** (`pe/core/collisionsystem/HardContactSemiImplicitTimesteppingSolvers.h:3527-3568`):
```cpp
case rigidBodyMigrationNotification: {
   BodyID b( *bodystorageShadowCopies_.find( objparam.sid_ ) );
   
   bodystorageShadowCopies_.remove( b );
   bodystorage_.add( b );
   b->manager_ = theDefaultManager();
   
   b->setOwner( myRank, 0 );
   b->setRemote( false );
   b->clearProcesses();
   // Register processes from registration list
}
```

**Actions**:
1. Move body from shadow copy storage to main storage
2. Set local ownership
3. Update registration list with processes that need shadow copies

**Remote Migration** (`pe/core/collisionsystem/HardContactSemiImplicitTimesteppingSolvers.h:3570-3596`):
```cpp
case rigidBodyRemoteMigrationNotification: {
   BodyID b( *bodystorageShadowCopies_.find( objparam.sid_ ) );
   
   // Update owner information
   b->setOwner( objparam.to_, *it );  // or b->setOwner( objparam.to_, 0 ) for distant processes
}
```

**Actions**:
1. Update owner rank of the shadow copy
2. Keep shadow copy in shadow storage (ownership changed but still remote)

## 4. Other Notification Types and Their Purposes

### RigidBodyUpdateNotification (`pe/core/collisionsystem/HardContactSemiImplicitTimesteppingSolvers.h:3499-3525`)
- **Purpose**: Update position, orientation, and velocities of existing shadow copies
- **Sent when**: Body intersects with process and is already registered
- **Data**: Position, orientation quaternion, linear and angular velocities

### RigidBodyCopyNotification (`pe/core/collisionsystem/HardContactSemiImplicitTimesteppingSolvers.h:3401-3497`)
- **Purpose**: Create new shadow copies on remote processes
- **Sent when**: Body intersects with process but is not registered yet
- **Data**: Complete body geometry and physical properties (dynamically marshalled)

### RigidBodyDeletionNotification (`pe/core/collisionsystem/HardContactSemiImplicitTimesteppingSolvers.h:3623-3645`)
- **Purpose**: Remove shadow copies when body is deleted (not just moved)
- **Sent when**: Body flows out of simulation domain or is explicitly deleted
- **Difference from removal**: Deletion means body no longer exists, removal means process no longer needs it

## Key Design Patterns

1. **Two-Phase Process**: 
   - Assembly phase: Determine what notifications to send
   - Communication phase: Exchange messages via MPI
   - Parsing phase: Process received notifications

2. **Registration System**: Bodies track which processes need their shadow copies

3. **Shadow Copy Management**: Separate storage for locally-owned vs. remote bodies

4. **Intersection-Based Distribution**: Bodies are distributed based on geometric intersection with process domains

5. **Migration Protocol**: Complex handoff procedure with registration list transfer to maintain consistency