add_executable(mpidem mpidem.cpp)
target_include_directories(mpidem PUBLIC ${FC_CORE_INCLUDE_DIRS})
target_link_libraries(mpidem)

# Dependencies
IF( LIBRARY_TYPE STREQUAL STATIC )
   ADD_DEPENDENCIES( mpidem      pe_static )
ELSE()
   ADD_DEPENDENCIES( mpidem      pe_shared )
ENDIF()
