add_executable(mpiperiodic mpiperiodic.cpp)
target_include_directories(mpiperiodic PUBLIC ${FC_CORE_INCLUDE_DIRS})
target_link_libraries(mpiperiodic)

# Dependencies
IF( LIBRARY_TYPE STREQUAL STATIC )
   ADD_DEPENDENCIES( mpiperiodic      pe_static )
ELSE()
   ADD_DEPENDENCIES( mpiperiodic      pe_shared )
ENDIF()
