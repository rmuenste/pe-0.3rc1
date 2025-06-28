# Batch Generator Documentation

## Overview

The Batch Generator is a critical component of the PE (Physics Engine) collision system responsible for organizing contacts into independent groups that can be processed in parallel. Located in the `pe/core/batches` directory, this system provides the foundation for thread-level parallelism within the physics engine, operating as a deeper layer of parallelism beneath the top-level MPI domain decomposition.

## Purpose and Role in Physics Engine

### Multi-Level Parallelization Strategy

The PE physics engine employs a hierarchical parallelization approach:

1. **Top Level - Domain Decomposition (MPI)**: Distributes the simulation domain across multiple processes
2. **Mid Level - Batch Parallelization**: Groups contacts into independent batches for thread-level parallelism
3. **Low Level - Contact Resolution**: Processes individual contacts within each batch

### Core Objective

The batch generator's primary purpose is to identify independent groups of contacts that can be solved simultaneously without affecting each other. This independence is crucial for:

- **Thread Safety**: Ensuring multiple threads can work on different batches without data races
- **Numerical Stability**: Preventing conflicts in constraint solving
- **Performance Optimization**: Maximizing parallel efficiency by minimizing synchronization points

## Configuration

### Batch Generator Selection

The batch generator is configured in `pe/config/Collisions.h` as one of the four template parameters of the central configuration:

```cpp
#define pe_BATCH_GENERATOR pe::batches::UnionFind
```

Available options:
- `pe::batches::SingleBatch` - Creates a single batch containing all contacts
- `pe::batches::UnionFind` - Creates independent batches using Union-Find algorithm

### Integration with Collision System

The batch generator is integrated into the collision system configuration:

```cpp
template< template<typename> class CD,    // Coarse Detector
          typename FD,                    // Fine Detector  
          template<typename> class BG,    // Batch Generator
          template<typename,typename,typename> class CR >  // Constraint Resolver
struct CollisionConfig;
```

## Architecture Overview

### Core Components

#### 1. BatchVector Container
```cpp
template< typename C, typename D = NoDelete, typename G = OptimalGrowth >
class BatchVector
{
public:
   typedef C                     Contact;   // Type of contacts
   typedef ContactVector<C,D,G>  Batch;     // Type of a single batch
   
   // Container operations
   Batch& operator[](size_t index);
   void resize(size_t n);
   void clear();
   size_t size() const;
};
```

**Features:**
- **Template-based**: Supports different contact types and memory policies
- **Dynamic Sizing**: Automatically adjusts to accommodate variable batch counts
- **Efficient Access**: Direct indexing for fast batch retrieval
- **Memory Management**: Configurable deletion and growth policies

#### 2. Batch Generator Interface

All batch generators implement a common interface:

```cpp
template<typename Config>
class BatchGeneratorInterface
{
public:
   template<typename Contacts, typename Batches>
   void generateBatches(Contacts& contacts, Batches& batches);
};
```

## Batch Generation Algorithms

### 1. SingleBatch Algorithm

#### Purpose
The SingleBatch algorithm provides a baseline implementation that creates a single batch containing all contacts. This approach:
- **Disables Parallelization**: All contacts are processed sequentially
- **Simplifies Debugging**: Eliminates batch-related complexity
- **Serves as Reference**: Provides correctness baseline for comparison

#### Implementation
```cpp
template<typename C>
class SingleBatch : private NonCopyable
{
public:
   template<typename Contacts, typename Batches>
   void generateBatches(Contacts& contacts, Batches& batches);
};
```

#### Algorithm Flow
1. **Clear Batches**: Remove any existing batch data
2. **Early Exit**: Return immediately if no contacts exist
3. **Set Indices**: Assign sequential indices to all contacts
4. **Create Single Batch**: Place all contacts in batch[0]

#### Time Complexity
- **O(n)** where n is the number of contacts
- **Space Complexity**: O(1) additional space (excluding contact storage)

#### Use Cases
- **Testing and Validation**: Verifying solver correctness
- **Single-threaded Execution**: When parallelization is not beneficial
- **Debugging**: Isolating batch-related issues

### 2. UnionFind Algorithm

#### Purpose
The UnionFind algorithm creates truly independent batches by analyzing the contact graph structure. It identifies connected components where each component represents a group of bodies that are directly or indirectly connected through contacts.

#### Theoretical Foundation

**Contact Graph Theory:**
- **Vertices**: Rigid bodies in the simulation
- **Edges**: Contacts between bodies
- **Connected Component**: A maximal set of bodies connected through contact chains
- **Independence**: Bodies in different components cannot influence each other

#### Implementation
```cpp
template<typename C>
class UnionFind : private NonCopyable
{
public:
   typedef typename Config::BodyType       BodyType;
   typedef typename Config::BodyID         BodyID;
   typedef typename BodyType::ConstNodeID  ConstNodeID;
   
   template<typename Contacts, typename Batches>
   void generateBatches(Contacts& contacts, Batches& batches);
};
```

#### Algorithm Details

##### Phase 1: Root Identification
```cpp
for (auto& contact : contacts) {
   const BodyID b1 = contact.getBody1();
   const BodyID b2 = contact.getBody2();
   
   // Determine the root of the connected component
   ConstNodeID root = b1->isFixed() ? b2->getRoot() : b1->getRoot();
   
   // Assign batch ID to new roots
   if (roots.find(root) == roots.end()) {
      root->batch_ = batchCount++;
      roots.insert(root);
   }
}
```

**Key Insights:**
- **Fixed Body Handling**: Fixed bodies don't contribute to batch determination
- **Root Node Usage**: Utilizes Union-Find data structure already present in rigid bodies
- **Batch Assignment**: Each unique root gets a distinct batch identifier

##### Phase 2: Contact Distribution
```cpp
for (auto& contact : contacts) {
   const BodyID b1 = contact.getBody1();
   const BodyID b2 = contact.getBody2();
   
   ConstNodeID root = b1->isFixed() ? b2->getRoot() : b1->getRoot();
   Batch& batch = batches[root->batch_];
   
   contact.setIndex(batch.size());
   batch.pushBack(contact);
}
```

**Operations:**
- **Batch Lookup**: Uses pre-computed batch IDs from Phase 1
- **Index Assignment**: Sets within-batch index for each contact
- **Contact Insertion**: Adds contact to appropriate batch

#### Algorithm Properties

##### Time Complexity
- **Phase 1**: O(n) where n is number of contacts
- **Phase 2**: O(n) for contact distribution
- **Overall**: O(n) linear time complexity

##### Space Complexity
- **Auxiliary Storage**: O(k) where k is number of unique roots
- **Batch Storage**: O(n) for contact references
- **Overall**: O(n + k) space complexity

##### Correctness Guarantees
- **Independence**: Bodies in different batches cannot influence each other
- **Completeness**: All contacts are assigned to exactly one batch
- **Consistency**: Contact-body relationships are preserved

#### Contact Graph Analysis

##### Connected Components
```
Example Contact Graph:
Bodies: A, B, C, D, E, F
Contacts: A-B, B-C, D-E

Connected Components:
- Component 1: {A, B, C} → Batch 0
- Component 2: {D, E}   → Batch 1  
- Component 3: {F}      → No contacts (not in any batch)
```

##### Fixed Body Interactions
```
Special Case - Fixed Bodies:
Bodies: Fixed_Ground, A, B, C
Contacts: Ground-A, Ground-B, A-C

Result:
- Ground is fixed → doesn't determine batching
- Component: {A, B, C} → Single Batch 0
- All ground contacts in same batch due to A-C connection
```

## Parallelization Benefits

### Thread-Level Parallelism

#### Independent Processing
```cpp
// Conceptual parallel processing
#pragma omp parallel for
for (size_t i = 0; i < batches.size(); ++i) {
   solveBatch(batches[i]);  // Safe parallel execution
}
```

#### Performance Characteristics
- **Ideal Speedup**: Up to N threads where N = number of batches
- **Load Balancing**: Depends on contact distribution across batches
- **Synchronization**: Minimal - only between batch boundaries

### Memory Access Patterns

#### Cache Efficiency
- **Spatial Locality**: Contacts within batches processed together
- **Temporal Locality**: Related body data accessed consecutively
- **Memory Bandwidth**: Reduced cache misses due to improved locality

#### NUMA Considerations
- **Thread Affinity**: Batches can be assigned to specific NUMA nodes
- **Memory Allocation**: Body data can be co-located with processing threads
- **Communication Overhead**: Minimized cross-node memory access

## Integration with Physics Pipeline

### Collision System Workflow

1. **Coarse Detection**: Identifies potentially colliding body pairs
2. **Fine Detection**: Generates detailed contact information
3. **Batch Generation**: ← **THIS COMPONENT** ← Groups contacts for parallel processing
4. **Constraint Solving**: Resolves contacts within each batch
5. **Integration**: Updates body positions and velocities

### Data Flow

```
Input:  ContactVector<Contact> contacts
        ↓
Process: BatchGenerator.generateBatches()
        ↓
Output: BatchVector<ContactVector<Contact>> batches
```

### Solver Integration

#### Constraint Resolution
```cpp
class ConstraintSolver {
   template<typename Batch>
   void solveBatch(Batch& batch) {
      // Process all contacts in this batch
      // Safe to parallelize across different batches
   }
};
```

#### Response Calculation
- **Force Computation**: Each batch computes forces independently
- **Impulse Application**: Applied separately per batch
- **Convergence Criteria**: Evaluated per batch with global synchronization

## Performance Analysis

### Batch Distribution Metrics

#### Optimal Characteristics
- **Uniform Size**: Batches of similar size for load balancing
- **Minimal Count**: Fewer batches → less parallelization overhead
- **Maximal Independence**: More batches → better parallelization potential

#### Performance Indicators
```cpp
// Typical batch analysis metrics
size_t totalContacts = contacts.size();
size_t batchCount = batches.size();
double averageBatchSize = totalContacts / batchCount;
double loadBalance = computeLoadBalanceMetric(batches);
double parallelEfficiency = computeParallelEfficiency(batchCount, threadCount);
```

### Scalability Considerations

#### Thread Scaling
- **Optimal Thread Count**: Usually equals number of batches
- **Over-subscription**: More threads than batches → diminishing returns
- **Under-utilization**: Fewer threads than batches → unused parallelism

#### Problem Size Dependencies
- **Large Systems**: More contacts → potentially more batches
- **Dense Systems**: Highly connected → fewer batches
- **Sparse Systems**: Loosely connected → many small batches

## Advanced Features

### Debugging and Visualization

#### Batch Logging
```cpp
pe_LOG_DEBUG_SECTION(log) {
   if (!batches.isEmpty()) {
      log << "   Batch generation...\n";
      for (size_t i = 0; i < batches.size(); ++i) {
         Batch& batch = batches[i];
         log << "      Batch " << i+1 << " = (";
         for (size_t j = 0; j < batch.size(); ++j)
            log << " " << batch[j]->getID();
         log << " )\n";
      }
   }
}
```

#### Analysis Tools
- **Batch Size Distribution**: Histogram of batch sizes
- **Contact Connectivity**: Graph analysis of contact networks
- **Performance Profiling**: Timing analysis per batch

### Memory Management

#### Batch Container Optimization
```cpp
template< typename C,                    // Contact type
          typename D = NoDelete,         // Deletion policy  
          typename G = OptimalGrowth >   // Growth policy
class BatchVector;
```

**Policies:**
- **NoDelete**: Contacts managed externally
- **OptimalGrowth**: Geometric growth for amortized O(1) insertion

#### Contact Lifecycle
1. **Creation**: Contacts created by fine detection
2. **Batching**: References distributed to batches
3. **Processing**: Contacts processed within batches
4. **Cleanup**: Batch container cleared, contacts may persist

## Extensibility and Customization

### Custom Batch Generators

#### Interface Requirements
```cpp
template<typename Config>
class CustomBatchGenerator : private NonCopyable {
public:
   typedef Config ConfigType;
   
   template<typename Contacts, typename Batches>
   void generateBatches(Contacts& contacts, Batches& batches);
   
   // Compile-time verification
   pe_CONSTRAINT_MUST_BE_SAME_TYPE(CustomBatchGenerator<Config>, 
                                   typename Config::BatchGenerator);
};
```

#### Implementation Guidelines
- **Independence**: Ensure batches are truly independent
- **Efficiency**: Minimize computational overhead
- **Robustness**: Handle edge cases (no contacts, single contact, etc.)
- **Logging**: Provide debugging information

### Specialized Algorithms

#### Potential Extensions
- **Graph Coloring**: Alternative approach to contact independence
- **Spatial Partitioning**: Batch based on spatial proximity
- **Temporal Coherence**: Leverage previous time step batch structure
- **Adaptive Batching**: Dynamic batch size optimization

## Relationship to Other Components

### Dependencies

#### Input Dependencies
- **Contact Generation**: Requires contacts from fine detection
- **Body Management**: Uses rigid body hierarchy and Union-Find structure
- **Configuration**: Depends on collision system configuration

#### Output Dependencies
- **Constraint Solvers**: Provides batched contacts for resolution
- **Parallel Execution**: Enables thread-level parallelism
- **Performance Systems**: Affects overall simulation performance

### Integration Points

#### MPI Integration
- **Domain Boundaries**: Batches respect process boundaries
- **Communication**: Batch structure affects inter-process communication
- **Load Balancing**: Batch distribution impacts domain decomposition

#### Memory Systems
- **Allocation**: Batch containers use configurable memory policies
- **Caching**: Batch structure affects cache performance
- **NUMA**: Influences memory placement strategies

## Conclusion

The Batch Generator represents a sophisticated approach to enabling thread-level parallelism in physics simulation. By intelligently partitioning contacts into independent groups, it provides the foundation for efficient parallel processing while maintaining simulation accuracy and numerical stability.

### Key Benefits

1. **Parallelization**: Enables effective thread-level parallelism
2. **Independence**: Guarantees mathematical correctness through contact independence
3. **Performance**: Improves cache locality and reduces synchronization overhead
4. **Scalability**: Provides clear path for performance scaling with additional threads
5. **Flexibility**: Supports multiple algorithms for different simulation scenarios

### Design Excellence

The batch generator exemplifies several important software engineering principles:
- **Template-based Design**: Provides flexibility while maintaining performance
- **Algorithm Abstraction**: Clean separation between interface and implementation
- **Performance Focus**: Linear time complexity with minimal memory overhead
- **Debugging Support**: Comprehensive logging and analysis capabilities
- **Extensibility**: Clear path for custom algorithm implementation

This component is essential for achieving high-performance physics simulation in multi-threaded environments, providing the critical bridge between contact detection and constraint resolution while enabling effective utilization of modern multi-core processors.