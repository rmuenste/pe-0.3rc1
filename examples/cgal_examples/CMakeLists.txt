add_executable(cgal_box cgal_box.cpp VtkOutput.cpp)
add_executable(mesh_collision_test mesh_collision_test.cpp VtkOutput.cpp)
add_executable(mesh_simulation mesh_simulation.cpp VtkOutput.cpp)
add_executable(mesh_containspoint_test mesh_containspoint_test.cpp)

find_package(GMP REQUIRED)
target_link_libraries(cgal_box  PRIVATE ${GMP_LIBRARIES})
target_link_libraries(mesh_collision_test PRIVATE ${GMP_LIBRARIES})
target_link_libraries(mesh_simulation PRIVATE ${GMP_LIBRARIES})
target_link_libraries(mesh_containspoint_test PRIVATE ${GMP_LIBRARIES})

message(STATUS "CGAL include directory: ${CGAL_INCLUDE_DIR}")

target_include_directories(cgal_box PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS} ${CGAL_INCLUDE_DIR})
target_include_directories(mesh_collision_test PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS} ${CGAL_INCLUDE_DIR})
target_include_directories(mesh_simulation PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS} ${CGAL_INCLUDE_DIR})
target_include_directories(mesh_containspoint_test PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS} ${CGAL_INCLUDE_DIR})

target_link_libraries(cgal_box PRIVATE 
  $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
)

target_link_libraries(mesh_collision_test PRIVATE 
  $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
)

target_link_libraries(mesh_simulation PRIVATE 
  $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
)

target_link_libraries(mesh_containspoint_test PRIVATE 
  $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
)

# Add CGAL dependency
#add_dependencies(cgal_box cgal)