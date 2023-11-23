add_executable(mpiperiodic_dev mpiperiodic_dev.cpp)
target_include_directories(mpiperiodic_dev PUBLIC ${FC_CORE_INCLUDE_DIRS})
target_link_libraries(mpiperiodic_dev)

# Dependencies
IF( LIBRARY_TYPE STREQUAL STATIC )
   ADD_DEPENDENCIES( mpiperiodic_dev      pe_static )
ELSE()
   ADD_DEPENDENCIES( mpiperiodic_dev      pe_shared )
ENDIF()
