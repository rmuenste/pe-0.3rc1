# Vec3 and Mat3 Documentation

This document describes the usage of the `Vec3` (3D Vector) and `Mat3` (3x3 Matrix) classes within the Physics Engine (pe) codebase. These classes are the core mathematical primitives used for rigid body dynamics, collision detection, and geometry.

**Headers:**
- `pe/math/Vector3.h`
- `pe/math/Matrix3x3.h`
- `pe/math.h` (Includes both)

**Namespace:** `pe`

**Type Definitions:**
These types are typically aliases for `real` precision (float or double) defined in `pe/system/Precision.h`.
```cpp
typedef Vector3<real> Vec3;
typedef Matrix3x3<real> Mat3;
```

---

## Vec3 (Vector3)

The `Vec3` class represents a 3-dimensional vector $(x, y, z)$.

### 1. Construction

```cpp
using namespace pe;

// Default constructor (initializes to 0, 0, 0)
Vec3 a; 

// Initialize all components to the same value
Vec3 b( 1.0 ); // (1.0, 1.0, 1.0)

// Component-wise initialization
Vec3 c( 1.0, 2.0, 3.0 ); // (x=1, y=2, z=3)

// Copy constructor
Vec3 d( c );

// From C-array
real data[3] = { 4.0, 5.0, 6.0 };
Vec3 e( data );
```

### 2. Element Access

Elements can be accessed using the subscript operator `[]`. Indices are 0 (x), 1 (y), 2 (z).

```cpp
Vec3 v( 1, 2, 3 );
real x = v[0]; // 1.0
v[1] = 5.0;    // Sets y to 5.0
```

### 3. Arithmetic Operations

Standard operators are overloaded for intuitive math.

```cpp
Vec3 a(1, 2, 3), b(4, 5, 6);

// Vector addition/subtraction
Vec3 sum = a + b;
Vec3 diff = a - b;

// Scalar multiplication/division
Vec3 scaled = a * 2.0;
Vec3 div = a / 2.0;

// Unary negation
Vec3 neg = -a;

// Component-wise multiplication
Vec3 compProd = a * b; // (a.x*b.x, a.y*b.y, a.z*b.z)

// Cross Product (operator %)
Vec3 crossProd = a % b; 
```

### 4. Dot Product
The dot product is achieved by transposing the first vector (conceptually) using `trans()`.

```cpp
real dot = trans(a) * b;
```

### 5. Utility Functions

```cpp
Vec3 v( 3, 0, 0 );

// Length
real len = v.length();       // 3.0
real sqLen = v.sqrLength();  // 9.0

// Normalization
v.normalize();               // Modifies v in-place to length 1
Vec3 n = v.getNormalized();  // Returns a new normalized vector

// Perpendicular vector (arbitrary perp vector)
Vec3 p = v.perp();

// Min/Max element
real minVal = v.min();
real maxVal = v.max();

// Reset/Clear (sets to 0)
v.reset();
```

### 6. Global Functions

```cpp
// Absolute values
Vec3 absV = abs(v);   // For integer types
Vec3 fabsV = fabs(v); // For floating point types

// Check for NaN
bool is_nan = isnan(v);

// Check if zero
bool is_zero = isDefault(v);
```

---

## Mat3 (Matrix3x3)

The `Mat3` class represents a 3x3 matrix. Storage is row-major (usually, check specific access patterns), but access is (row, col).

### 1. Construction

```cpp
using namespace pe;

// Default constructor (all zeros)
Mat3 A;

// Initialize all elements to value
Mat3 B( 1.0 ); 

// Diagonal initialization
Mat3 C( 1.0, 2.0, 3.0 ); 
/* 
   1 0 0
   0 2 0
   0 0 3
*/

// Full initialization (Row-wise: xx, xy, xz, yx, yy, yz, zx, zy, zz)
Mat3 D( 1, 0, 0,
        0, 1, 0,
        0, 0, 1 );

// From column vectors
Vec3 c1(1,0,0), c2(0,1,0), c3(0,0,1);
Mat3 E( c1, c2, c3 );
```

### 2. Element Access

```cpp
Mat3 M;

// 2D Access (row, column)
M(0, 0) = 5.0; // Set element at row 0, col 0
real val = M(1, 2);

// 1D Linear Access (0-8)
M[0] = 1.0; // Same as M(0,0)
M[8] = 9.0; // Same as M(2,2)
```

### 3. Arithmetic Operations

```cpp
Mat3 A, B;

// Matrix addition/subtraction
Mat3 Sum = A + B;
Mat3 Diff = A - B;

// Scalar multiplication/division
Mat3 Scaled = A * 2.0;

// Matrix-Matrix Multiplication
Mat3 Prod = A * B;

// Matrix-Vector Multiplication
Vec3 v(1, 1, 1);
Vec3 result = A * v;     // Standard M * v
Vec3 resultT = v * A;    // Treated as v^T * M (returns transpose vector conceptually)

// Matrix-Scalar Product (M * M)
Mat3 Sqr = sq(A);
```

### 4. Advanced Operations

```cpp
Mat3 M( ... );

// Determinant
real det = M.getDeterminant();

// Transpose
Mat3 MT = trans(M);      // Global function returning new matrix
M.transpose();           // Modifies M in-place

// Inverse
Mat3 M_inv = inv(M);     // Global function returning new matrix
M.invert();              // Modifies M in-place (throws/asserts if singular)

// Linear System Solver (Ax = b)
// Uses Cholesky if applicable or optimized inversion
Vec3 x = M.solve( b );

// Cholesky Decomposition (returns L where M = L * L^T)
Mat3 L = M.getCholesky();
```

### 5. Utility Functions

```cpp
// Column access
Vec3 col0 = M.col(0);

// Checks
bool isDiag = M.isDiagonal();
bool isSym = M.isSymmetric();
bool isSing = M.isSingular(); // det approx 0

// Scaling
M.scale( 2.0 ); // In-place scalar multiplication
```

### 6. Special Matrix-Vector Products

Special operators are available for cross-product related matrix operations (skew-symmetric).

```cpp
Vec3 r(1, 0, 0);
Mat3 M( ... );

// Skew-symmetric multiplication: M * r_cross
// Equivalent to M * SkewMatrix(r)
Mat3 R1 = M % r; 

// Skew-symmetric multiplication: r_cross * M
// Equivalent to SkewMatrix(r) * M
Mat3 R2 = r % M;
```
