add_executable(lubrication_sphere_plane
  sphere_plane_gap.cpp
)
target_include_directories(lubrication_sphere_plane PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
target_link_libraries(lubrication_sphere_plane PRIVATE
  $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
)

add_executable(lubrication_sphere_sphere
  sphere_sphere_gap.cpp
)
target_include_directories(lubrication_sphere_sphere PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
target_link_libraries(lubrication_sphere_sphere PRIVATE
  $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
)
