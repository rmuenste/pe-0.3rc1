# pe ports of LIGGGHTS-PUBLIC tutorials (see LIGGGHTS-PUBLIC/md_docs/RUNBOOK.md)
function(lp_add_example target src)
  add_executable(${target} ${src})
  target_include_directories(${target} PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(${target} PRIVATE
    $<$<STREQUAL:${PE_LIBRARY_TYPE},STATIC>:pe_static>
    $<$<STREQUAL:${PE_LIBRARY_TYPE},BOTH>:pe_static>
    $<$<STREQUAL:${PE_LIBRARY_TYPE},SHARED>:pe_shared>
  )
endfunction()

lp_add_example(lp_insert_stream insert_stream.cpp)
lp_add_example(lp_cylinder_pack cylinder_pack.cpp)
lp_add_example(lp_multisphere   multisphere.cpp)

if(PE_USE_CGAL)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/mesh_gran.cpp)
    lp_add_example(lp_mesh_gran mesh_gran.cpp)
  endif()
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/moving_mesh_gran.cpp)
    lp_add_example(lp_moving_mesh_gran moving_mesh_gran.cpp)
  endif()
endif()

# Self-checking runs of the parallel-capable ports (exit code: particle count, container bounds,
# finite energies; 77 = unintended solver, reported as skipped). Serial, and under MPI with 2
# and 4 processes (slab decomposition along the cylinder axis).
# Each with the Baumgarte term and with the split-impulse position correction ("-split").
foreach(variant "" "-split")
  if(variant STREQUAL "-split")
    set(extra --split-impulse)
  else()
    set(extra)
  endif()
  add_test(NAME pe-example-cylinder-pack-serial${variant} COMMAND lp_cylinder_pack --case noCohesion --no-vtk --tend 0.25 ${extra})
  set_tests_properties(pe-example-cylinder-pack-serial${variant} PROPERTIES
    LABELS "integration;serial;example;liggghts" SKIP_RETURN_CODE 77 TIMEOUT 900)
  if(PE_USE_MPI AND MPI_FOUND)
    foreach(np 2 4)
      add_test(NAME pe-example-cylinder-pack-mpi${np}${variant}
        COMMAND ${MPIEXEC_EXECUTABLE} ${MPIEXEC_NUMPROC_FLAG} ${np} ${MPIEXEC_PREFLAGS}
                $<TARGET_FILE:lp_cylinder_pack> ${MPIEXEC_POSTFLAGS} --case noCohesion --no-vtk --tend 0.25 ${extra})
      set_tests_properties(pe-example-cylinder-pack-mpi${np}${variant} PROPERTIES
        LABELS "integration;mpi;example;liggghts" SKIP_RETURN_CODE 77 TIMEOUT 900 PROCESSORS ${np})
    endforeach()
  endif()
endforeach()
