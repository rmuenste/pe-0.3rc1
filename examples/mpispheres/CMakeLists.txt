add_executable(mpispheres mpispheres.cpp)
target_include_directories(mpispheres PUBLIC ${FC_CORE_INCLUDE_DIRS})
target_link_libraries(mpispheres)

# Dependencies
IF( LIBRARY_TYPE STREQUAL STATIC )
   ADD_DEPENDENCIES( mpispheres      pe_static )
ELSE()
   ADD_DEPENDENCIES( mpispheres      pe_shared )
ENDIF()
