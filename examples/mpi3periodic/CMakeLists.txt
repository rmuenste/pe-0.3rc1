add_executable(mpi3periodic mpi3periodic.cpp)
target_include_directories(mpi3periodic PUBLIC ${FC_CORE_INCLUDE_DIRS})
target_link_libraries(mpi3periodic)

# Dependencies
IF( LIBRARY_TYPE STREQUAL STATIC )
   ADD_DEPENDENCIES( mpi3periodic      pe_static )
ELSE()
   ADD_DEPENDENCIES( mpi3periodic      pe_shared )
ENDIF()
