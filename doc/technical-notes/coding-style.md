# PE Coding Style

How PE C++ code is named, laid out, formatted and documented.

**Source of the rules.** PE had no written style guide. The rules below are inferred from
`pe/config`, `pe/math`, `pe/util`, `pe/system` (and `src/util`), which follow one style almost
without exception: by indentation and parenthesis style, 331 of 332 headers in those four folders
and all 386 headers under `pe/core` match it. The naming table comes from the "Naming convention"
section of `tutorial/Tutorial00.h`. This is the **canonical style for new code**.

Newer code in a few areas uses a different style. See
[Code that deviates from the canonical style](#code-that-deviates-from-the-canonical-style)
for where it is and how to handle it.

> **Canonical examples.** When in doubt, copy the shape of an existing file:
> `pe/util/NonCopyable.h` (minimal class), `pe/util/MemoryPool.h` (class template with
> out-of-class definitions), `pe/math/Vector3.h` (full math type), `pe/config/HashGrids.h`
> (configuration constants), `src/util/Random.cpp` (implementation file).

## Quick reference

| Element | Convention | Example |
|---|---|---|
| Classes, structs, typedefs | `LikeThis` | `MemoryPool`, `Vec3`, `ElementType` |
| Functions (member and free) | `likeThis` | `getNormalized()`, `createSphere()` |
| Member data (static and non-static) | `likeThis_` | `freeList_`, `seed_` |
| Mathematical constants | `M_LIKE_THIS` | `M_PI`, `M_LN2` |
| Macros | `pe_LIKE_THIS` | `pe_INTERNAL_ASSERT`, `pe_CONSTRAINT_SOLVER` |
| Enumerators | `likeThis` | `sphereType`, `littleEndian` |
| Configuration constants | `likeThis` | `xCellCount`, `hierarchyFactor` |
| Template parameters | short `CamelCase` | `Type`, `TF`, `VT`, `T1`, `Blocksize` |
| Namespaces | lowercase | `pe`, `pe::response`, `pe::lubrication` |
| Handle types | `<Name>ID`, `Const<Name>ID` | `BodyID`, `ConstBodyID`, `SphereID` |
| Header files | one class per file, named after it | `pe/math/Vector3.h` |
| Directories | lowercase | `pe/core/rigidbody/` |
| Include guards | `_PE_<DIR>_<FILE>_H_` | `_PE_UTIL_MEMORYPOOL_H_` |
| Indentation | 3 spaces, no tabs | |
| Line length | 100 columns (banners are 99) | |
| Control statements | `if( x ) {` | `for( size_t i=0; i<n; ++i ) {` |

Objects are created with a `create...()` function that returns an ID and released with
`destroy()` (e.g. `createSphere()` / `destroy()`, see `tutorial/Tutorial00.h`).

## File layout

Every header and source file has the same skeleton:

```cpp
//=================================================================================================
/*!
 *  \file pe/util/MemoryPool.h
 *  \brief Header file for the memory pool class
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *
 *  This file is part of pe.
 *  ... GPL v3 notice ...
 */
//=================================================================================================

#ifndef _PE_UTIL_MEMORYPOOL_H_
#define _PE_UTIL_MEMORYPOOL_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <vector>
#include <pe/util/Assert.h>
#include <pe/util/NonCopyable.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Memory pool for small objects.
 * \ingroup util
 * ...
 */
template< typename Type, size_t Blocksize >
class MemoryPool : private NonCopyable
{
   ...
};
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

...

} // namespace pe

#endif
```

- **File header.** The `\file` path is the include path (`pe/...` for headers, `src/...` for
  sources), followed by a one-line `\brief`, the copyright line and the GPL notice. New files
  carry their author's copyright line (e.g. `Copyright (C) 2025 Raphael Münster`).
  The header block may also hold a longer explanation of the file's purpose
  (see `pe/core/lubrication/LubricationModel.h`).
- **Includes** go under the `// Includes` banner, written with angle brackets and full paths
  (`#include <pe/util/Types.h>`, never `"Types.h"`): first standard headers, then third-party
  (Boost), then `pe/`, each group roughly alphabetical. Platform-specific includes such as
  `<pe/system/WarningDisable.h>` get their own banner above that
  (`// Platform/compiler-specific includes`). Never re-sort includes mechanically: some are
  order dependent (`pe/system/Precision.h` includes `pe/config/Precision.h` *inside*
  `namespace pe`).
- **Namespaces** are not indented and are closed with `} // namespace pe`.

## Banners and vertical rhythm

Banners give the code its distinctive look and are part of the style, not decoration.

| Banner | Width | Purpose |
|---|---|---|
| `//====...` with `//  TITLE IN CAPITALS` between two `//` lines | 99 chars | Major section: `CLASS DEFINITION`, `CONSTRUCTORS`, `OPERATORS`, `UTILITY FUNCTIONS`, `GLOBAL OPERATORS`, `TYPE DEFINITIONS`, ... |
| `//****...` (full line) | 99 chars | Frames **every** top-level entity: each class, function definition, typedef and constant gets a frame line above its Doxygen comment and one below its closing brace. |
| `//**Group name****...` (indented 3) | to column 100 | Group header inside a class body, closed by a plain `//****...` line. |

Blank lines: **two** between consecutive framed entities, **four** before a `//====` section
banner, **two** after the include guard and after the includes block.

## Formatting

### Indentation and line length

- 3 spaces per level, no tabs. Access specifiers (`public:`, `private:`) sit at column 0 of the
  class; members are indented 3.
- 100 columns. Wrap long declarations by moving the declarator to an indented continuation
  line rather than exceeding the limit:

  ```cpp
  inline const Vector3< typename MathTrait<Type,Other>::MultType, false >
     rotate( const Vector3<Other,TF>& v ) const;
  ```

### Braces

- Class, struct and function bodies: opening brace on its own line.
- Control statements, `enum`s and `union`s: opening brace on the same line.
- `else` starts a new line after the closing brace (`}` / `else {`), never `} else {`.
- A single-statement body may omit the braces, on the next line or (for very short guards)
  on the same line:

  ```cpp
  if( len == Type(0) )
     return *this;

  if( ( ptr1 - ptr2 ) % sizeof(FreeObject) != 0 ) return false;
  ```

### Spaces

- **No space before `(`, a space inside it** for control statements, function declarations
  and calls with arguments: `if( x )`, `while( it != end )`, `f( a, b )`, `void free( void* rawMemory )`.
  Empty argument lists stay `f()`.
- Short scalar constructions and casts are usually written compactly: `Type(0)`, `real(2)`,
  `sizeof(Type)`, `std::time(0)`, `reinterpret_cast<const byte*>(ptr)`.
- Arithmetic and comparison operators are usually written **compactly** inside loop headers,
  subscripts and dense formulas, and spaced where that improves readability:
  `for( size_t i=0; i<Blocksize-1; ++i )`, `rawMemory_[i+1]`,
  `lhs[0]+rhs[0]`, but `toRelease >= it->rawMemory_`.
- Templates: `template< typename Type, size_t Blocksize >` (spaces inside the angle brackets of
  the template head), but no spaces in template-ids: `MemoryPool<Type,Blocksize>`,
  `Vector3<Type,TF>`. Long template heads put one parameter per line with **leading commas**
  and a trailing comment for each:

  ```cpp
  template< typename T1  // Data type of the left-hand side vector
          , typename T2  // Data type of the right-hand side vector
          , bool TF >    // Transposition flag
  ```

- Pointers and references bind to the type: `Type* ptr`, `const Vector3& v`.

### Declarations and initialization

- Prefer direct initialization for locals: `const Type len( std::sqrt( ... ) );`.
- `const`-qualify everything that does not change, including locals.
- Declarations inside a class group are **aligned in columns** (return types, names,
  `const`, trailing `//!<` comments); keep that table aligned when adding a member:

  ```cpp
                             inline size_t        size()          const;
                             inline Vector3&      set( Type x, Type y, Type z );
  template< typename Other > inline Vector3&      scale( Other scalar );
  ```

- Constructor initializer lists put one initializer per line with a **leading comma**,
  indented 3 from the function, optionally aligned and commented:

  ```cpp
  inline Timer::Timer()
     : counter_( 0   )
     , start_  ( 0.0 )
  ```

## Class structure

A class declaration is a sequence of commented groups, in this order where present:

1. `Type definitions` (`typedef`s; `This`, `ElementType`, ...)
2. `Constructors`, `Destructor` (write "No explicitly declared destructor." if there is none)
3. `Operators`
4. `Get functions` / `Set functions` / `Utility functions` / other functional groups
5. `Member variables`
6. `Compile time checks` (`pe_CONSTRAINT_...` wrapped in `/*! \cond PE_INTERNAL */ ... /*! \endcond */`)

`public:` groups come before `private:` ones. Each group looks like this:

```cpp
   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline size_t size() const;
   inline void   reset();
   //@}
   //**********************************************************************************************
```

In headers, member functions are **declared** in the class and **defined** after it, one
framed definition per function, grouped under section banners mirroring the class groups
(`CONSTRUCTORS`, `OPERATORS`, `UTILITY FUNCTIONS`, ...). All functions defined in headers
are marked `inline`. Trivial one-liners (empty constructors) may be defined in-class.

## Doxygen

- Entity comments start with `/*!\brief` (no space), continue with ` * ` lines, and use
  `\ingroup <group>`, `\param <name> <text>`, `\return <text>` (`\return void` for void
  functions), and `\a name` to refer to parameters.
- Members and enumerators are documented with trailing `//!<` comments.
- Code examples use `\code` / `\endcode` with the lines **not** prefixed by `*`:

  ```cpp
   * This operator represents the addition of two 3-dimensional vectors:

     \code
     pe::Vec3 a, b, c;
     c = a + b;
     \endcode

   * The operator returns ...
  ```

- Internal-only material is wrapped in `/*! \cond PE_INTERNAL */ ... /*! \endcond */`.
- Formulas use `\f$ ... \f$`.

## Language idioms

The legacy code is C++98 era; the project now builds as C++17 (`CMAKE_CXX_STANDARD 17`).

- Use the floating-point type `real` (from `pe/system/Precision.h`), never a hard-coded
  `double`, and the vector/matrix types `Vec3`, `Mat3`, `Quat`.
- Use `size_t` for sizes and indices.
- Runtime checks: `pe_INTERNAL_ASSERT( expr, msg )` for internal invariants and
  `pe_USER_ASSERT( expr, msg )` for misuse of the API (`pe/util/Assert.h`). Compile-time
  checks: the `pe_CONSTRAINT_*` macros (`pe/util/constraints/`).
- Errors that callers must handle are thrown as `std::runtime_error` /
  `std::invalid_argument` with a descriptive message.
- Make classes non-copyable by deriving privately from `NonCopyable`.
- In files written in the legacy style, keep using `typedef` for member types. C++11/17
  features (`if constexpr`, `std::void_t`, `auto` for iterator types, `nullptr`) are fine in
  new code where they make it clearer; `pe/interface/support/el_optional_api.h` is the established
  pattern for detecting optional collision-system APIs.

## Code that deviates from the canonical style

Much of the newer code follows the canonical style (e.g. `pe/core/lubrication/` and
`pe/util/CheckpointMetadata.h`, both added in 2026). A noticeable part does not. It uses a
Google/LLVM-like style:

| | Canonical | Deviating |
|---|---|---|
| Indentation | 3 spaces | 2 spaces (some 4) |
| Control statements | `if( x )` | `if (x)` |
| Function braces | own line | same line |
| Template heads | `template< typename T >` | `template <typename T>` / `template<class T>` |
| Entity framing | `//****` frames + `/*!\brief` | plain `//` or `//!` comments, few banners |
| Include guards | `_PE_DIR_FILE_H_` | `PE_INTERFACE_..._H`, `_FC2_..._HPP_`, `_PE_SETUP_ATC_H_` |
| File names | `CamelCase.h` | `snake_case.h` (`setup/atc.h`, `support/el_optional_api.h`) |
| Namespace close | `} // namespace pe` | `}  // namespace pe` |

Where it lives, by the same indentation/parenthesis measure:

| Location | Files in deviating style |
|---|---|
| `pe/interface/` | nearly all: the case setups under `setup/` and the helpers under `support/` |
| `src/interface/` | 5 of 8 |
| `tests/` | 14 of 35 |
| `src/core/` | `detection/fine/DistanceMap.cpp`, `domaindecomp/TriMeshDopBoundary.cpp` |
| `pe/config/` | `SimulationConfig.h` |

Rules for working with it:

1. **New files use the canonical style**, including new files in `pe/interface/` and
   `tests/`.
2. **When editing an existing file, match that file's style.** Never mix the two styles
   within one file.
3. **Do not reformat as a side effect.** Converting a deviating file to the canonical style
   is fine, but only as a separate, formatting-only commit.
4. Names fixed by an external ABI are exempt: the Fortran entry points `commf2c_*` (see
   [pe-interface-serial-mode.md](pe-interface-serial-mode.md)) keep their lowercase,
   underscore-separated names.

## clang-format

The repository root contains a `.clang-format` that approximates the canonical style
(clang-format 18 or newer). It gets indentation, braces, `if( x )` / `f( a, b )` spacing,
pointer binding, initializer lists and comment alignment right, but **cannot reproduce**:

- hand-aligned declaration tables (it only aligns consecutive declarations its own way),
- compact operators (`i<n`, `v_[i+1]`): it always spaces binary operators,
- compact casts and scalar constructions (`Type(0)` becomes `Type( 0 )`),
- template-ids without a space after the comma (`Vector3<Type,TF>`),
- leading-comma template parameter lists,
- `} // namespace pe` with one space (it writes two).

Applied to whole legacy files it rewrites about 15% of their lines. Therefore:

- **Format only what you changed:** `git clang-format` (staged changes) or
  `git clang-format --diff` to preview.
- Use `clang-format -i` only on files you create from scratch.
- Protect hand-aligned blocks you add with `// clang-format off` / `// clang-format on`.
- Do not run it on files in the deviating style (it would convert only the touched lines and
  leave the file mixed).

## Related notes

- [new-collision-system-implementation-guide.md](new-collision-system-implementation-guide.md):
  structure of collision-system specializations.
- [collision-detection-conventions.md](collision-detection-conventions.md),
  [plane-conventions.md](plane-conventions.md): physics/API conventions, as opposed to the
  code style described here.
