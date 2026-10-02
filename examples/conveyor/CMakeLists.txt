# Conveyor belt example: pe re-implementation of the LIGGGHTS-PUBLIC tutorial
# examples/LIGGGHTS/Tutorials_public/conveyor (hard-contact solver, serial).
add_executable(conveyor conveyor.cpp)
target_include_directories(conveyor PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
target_link_libraries(conveyor PRIVATE
  $<$<STREQUAL:${PE_LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${PE_LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${PE_LIBRARY_TYPE},SHARED>:pe_shared>
)
