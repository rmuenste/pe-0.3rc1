add_executable(decomposition_geometry decomposition_geometry.cpp)

target_include_directories(decomposition_geometry PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})

# Based on the type of the main library we choose static or shared linked libraries
target_link_libraries(decomposition_geometry PRIVATE 
  $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
)