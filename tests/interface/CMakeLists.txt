set(PE_INTERFACE_SERIAL_FIXTURE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/fixtures/serial")
set(PE_INTERFACE_SERIAL_WORK_ROOT
    "${CMAKE_CURRENT_BINARY_DIR}/serial-runs")

add_executable(pe_interface_smoke_serial
  pe_interface_smoke_serial.cpp
)

target_include_directories(pe_interface_smoke_serial PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(pe_interface_smoke_serial PRIVATE pe_static)

# Pure-arithmetic unit test for the Euler-Lagrange semi-implicit drag update.
# Needs only the header-only helper + Vec3; no world/MPI setup, so it is a plain
# add_test rather than a --case of the smoke harness.
add_executable(pe_el_semi_implicit_drag_test
  pe_el_semi_implicit_drag_test.cpp
)
target_link_libraries(pe_el_semi_implicit_drag_test PRIVATE pe_static)
add_test(
  NAME pe-el-semi-implicit-drag
  COMMAND pe_el_semi_implicit_drag_test
)

# Pure-arithmetic unit test for the pairwise lubrication model (Kroupa et al. 2016).
# Header-only physics + Vec3; no world/MPI setup, so it is a plain add_test rather
# than a --case of the smoke harness.
add_executable(pe_lubrication_model_test
  pe_lubrication_model_test.cpp
)
target_link_libraries(pe_lubrication_model_test PRIVATE pe_static)
add_test(
  NAME pe-lubrication-model
  COMMAND pe_lubrication_model_test
)

# Bit-for-bit regression gate: lubricationModel_="legacy" + lubricationCutoffFactor_=0
# must reproduce the pre-Kroupa-2016 solver trajectories exactly. Needs a build with
# pe_CONSTRAINT_SOLVER == pe::response::HardContactLubricated (see the test file header);
# under any other solver the test self-reports as SKIPPED via exit code 77.
add_executable(pe_lubrication_legacy_regression_test
  pe_lubrication_legacy_regression_test.cpp
)
target_link_libraries(pe_lubrication_legacy_regression_test PRIVATE pe_static)
add_test(
  NAME pe-lubrication-legacy-regression
  COMMAND pe_lubrication_legacy_regression_test
)
set_tests_properties(pe-lubrication-legacy-regression PROPERTIES SKIP_RETURN_CODE 77)

# Level 1 lubrication validation (Kroupa-2016 model through the full solver pipeline).
# Both need the HardContactLubricated solver and self-report as SKIPPED otherwise.
add_executable(pe_lubrication_two_sphere_approach_test
  pe_lubrication_two_sphere_approach_test.cpp
)
target_link_libraries(pe_lubrication_two_sphere_approach_test PRIVATE pe_static)
add_test(
  NAME pe-lubrication-two-sphere-approach
  COMMAND pe_lubrication_two_sphere_approach_test
)
set_tests_properties(pe-lubrication-two-sphere-approach PROPERTIES SKIP_RETURN_CODE 77)

add_executable(pe_lubrication_bounce_stokes_test
  pe_lubrication_bounce_stokes_test.cpp
)
target_link_libraries(pe_lubrication_bounce_stokes_test PRIVATE pe_static)
add_test(
  NAME pe-lubrication-bounce-stokes
  COMMAND pe_lubrication_bounce_stokes_test
)
set_tests_properties(pe-lubrication-bounce-stokes PROPERTIES SKIP_RETURN_CODE 77)

function(pe_add_interface_serial_test test_name case_name)
  add_test(
    NAME ${test_name}
    COMMAND pe_interface_smoke_serial
      --case ${case_name}
      --fixtures "${PE_INTERFACE_SERIAL_FIXTURE_DIR}"
      --work-root "${PE_INTERFACE_SERIAL_WORK_ROOT}"
  )
endfunction()

pe_add_interface_serial_test(
  pe-interface-serial-atc-external
  atc-external
)
pe_add_interface_serial_test(
  pe-interface-serial-atc-resume-roundtrip
  atc-resume-roundtrip
)
if(PE_USE_CGAL)
  pe_add_interface_serial_test(
    pe-interface-serial-atc-cgal-distancemap
    atc-cgal-distancemap
  )
  pe_add_interface_serial_test(
    pe-interface-serial-span-complex-distancemaps
    span-complex-distancemaps
  )
endif()
pe_add_interface_serial_test(
  pe-interface-serial-fluidization-srr-fresh
  fluidization-srr-fresh
)
pe_add_interface_serial_test(
  pe-interface-serial-fluidization-srr-resume-roundtrip
  fluidization-srr-resume-roundtrip
)

if(MPI AND CGAL)
  add_executable(pe_frozen_field_trace_mpi
    pe_frozen_field_trace_mpi.cpp
  )
  target_link_libraries(pe_frozen_field_trace_mpi PRIVATE pe_static MPI::MPI_CXX)

  set(PE_FROZEN_FIELD_FIXTURE_DIR
      "${CMAKE_CURRENT_SOURCE_DIR}/fixtures/frozen-field")

  foreach(mesh_format IN ITEMS off obj)
    add_test(
      NAME pe-interface-frozen-field-${mesh_format}
      COMMAND
        ${MPIEXEC_EXECUTABLE} ${MPIEXEC_NUMPROC_FLAG} 2
        ${MPIEXEC_PREFLAGS}
        $<TARGET_FILE:pe_frozen_field_trace_mpi>
        ${mesh_format}
        "${PE_FROZEN_FIELD_FIXTURE_DIR}/cube.obj"
        ${MPIEXEC_POSTFLAGS}
    )
    set_tests_properties(pe-interface-frozen-field-${mesh_format} PROPERTIES
      WORKING_DIRECTORY "${PE_FROZEN_FIELD_FIXTURE_DIR}"
    )
  endforeach()
endif()
