add_executable(triangle_mesh triangle_mesh.cpp)
target_include_directories(triangle_mesh PUBLIC ${FC_CORE_INCLUDE_DIRS})
target_link_libraries(triangle_mesh)

# Dependencies
IF( LIBRARY_TYPE STREQUAL STATIC )
   ADD_DEPENDENCIES( triangle_mesh      pe_static )
ELSE()
   ADD_DEPENDENCIES( triangle_mesh      pe_shared )
ENDIF()