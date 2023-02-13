add_executable(mpiboxes mpiboxes.cpp)
target_include_directories(mpiboxes PUBLIC ${FC_CORE_INCLUDE_DIRS})
target_link_libraries(mpiboxes)

# Dependencies
IF( LIBRARY_TYPE STREQUAL STATIC )
   ADD_DEPENDENCIES( mpiboxes      pe_static )
ELSE()
   ADD_DEPENDENCIES( mpiboxes      pe_shared )
ENDIF()
