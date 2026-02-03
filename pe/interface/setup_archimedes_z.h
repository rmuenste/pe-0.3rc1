#pragma once

#include <iostream>

using namespace pe::povray;

//===================================================================================
// Setup for the Archimedes case - Z direction subdivision
//
// DEPRECATED: This function has been deprecated and its implementation removed.
// The Archimedes setup functionality should be reimplemented in application code
// if needed.
//===================================================================================

#if defined(__GNUC__) || defined(__clang__)
__attribute__((deprecated("setupArchimedesZ is deprecated and has no implementation")))
#elif defined(_MSC_VER)
__declspec(deprecated("setupArchimedesZ is deprecated and has no implementation"))
#endif
void setupArchimedesZ(MPI_Comm ex0);


//===================================================================================
// Setup for the Archimedes case - Z direction subdivision
// DEPRECATED: Empty stub function - do not use
//===================================================================================
void setupArchimedesZ(MPI_Comm ex0)
{
   std::cerr << "WARNING: setupArchimedesZ() is deprecated and has no implementation.\n"
             << "Please reimplement this functionality in your application code.\n";
}