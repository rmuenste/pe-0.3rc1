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

# The tests below drive the shared lubrication stage through a real collision pipeline, so
# they need a stage-capable pe_CONSTRAINT_SOLVER. That CANNOT be selected per target with
# target_compile_definitions -- pe_CONSTRAINT_SOLVER feeds the global Config typedef and
# non-template library units are compiled against it, so mixing solvers between library
# and consumer yields undefined references. Each test therefore links a whole library
# variant instead, and so runs and must pass in every build configuration.
#
# pe_static_lubstage_fluid / _plain are the same library built with a stage-capable
# pe_CONSTRAINT_SOLVER (HardContactAndFluid / the plain hard-contact solver). See the
# top-level CMakeLists.txt for why this has to be a whole library variant rather than a
# per-target compile definition, and why two are needed.

# Bit-for-bit regression gate: lubricationModel_="legacy" + lubricationCutoffFactor_=0
# must reproduce the pre-Kroupa-2016 trajectories exactly.
add_executable(pe_lubrication_legacy_regression_test
  pe_lubrication_legacy_regression_test.cpp
)
target_link_libraries(pe_lubrication_legacy_regression_test PRIVATE pe_static_lubstage_fluid)
add_test(
  NAME pe-lubrication-legacy-regression
  COMMAND pe_lubrication_legacy_regression_test
)

# Level 1 lubrication validation (Kroupa-2016 model through the full solver pipeline).
# Both link a stage-capable library variant, so neither can skip.
add_executable(pe_lubrication_two_sphere_approach_test
  pe_lubrication_two_sphere_approach_test.cpp
)
target_link_libraries(pe_lubrication_two_sphere_approach_test PRIVATE pe_static_lubstage_plain)
add_test(
  NAME pe-lubrication-two-sphere-approach
  COMMAND pe_lubrication_two_sphere_approach_test
)

add_executable(pe_lubrication_bounce_stokes_test
  pe_lubrication_bounce_stokes_test.cpp
)
target_link_libraries(pe_lubrication_bounce_stokes_test PRIVATE pe_static_lubstage_fluid)
add_test(
  NAME pe-lubrication-bounce-stokes
  COMMAND pe_lubrication_bounce_stokes_test
)

# Contract tests for the checkpoint identity sidecar: metadata roundtrip, legacy (sidecar-less)
# acceptance, truncation rejection for both the sidecar and the `.peb`, material table restore,
# and the resume mismatch guard. Links the shipped pe_static -- the checkpoint path is
# solver-independent, so this must hold in the default configuration users get.
add_executable(pe_checkpoint_metadata_test
  pe_checkpoint_metadata_test.cpp
)
target_link_libraries(pe_checkpoint_metadata_test PRIVATE pe_static)
add_test(
  NAME pe-checkpoint-metadata
  COMMAND pe_checkpoint_metadata_test
)

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

# Serial-mode coverage for the lubrication add-on (D2.2 §3.7). Drives the shared stage
# through the CAMPAIGN solver (HardContactAndFluid) in a non-MPI build, with lubrication
# enabled purely at runtime, and checks the pipeline force against LubricationModel.h
# evaluated directly -- the model as its own oracle, isolating plumbing from physics.
# Covers sphere-sphere and sphere-wall, and that the switch is exactly inert when off.
add_executable(pe_lubrication_serial_coverage_test
  pe_lubrication_serial_coverage_test.cpp
)
target_link_libraries(pe_lubrication_serial_coverage_test PRIVATE pe_static_lubstage_fluid)
add_test(
  NAME pe-lubrication-serial-coverage
  COMMAND pe_lubrication_serial_coverage_test
)

# AABB neutrality gate: lubrication OFF must mean no bounding-box inflation at all.
# Bounding-box size feeds the HashGrids broad phase, so a stray padding term is a silent
# trajectory change in every DNS case. Runs in its own process so the parameter store is
# pristine, and links the campaign solver -- the exact "HCAF + defaults + off" case.
add_executable(pe_lubrication_aabb_neutrality_test
  pe_lubrication_aabb_neutrality_test.cpp
)
target_link_libraries(pe_lubrication_aabb_neutrality_test PRIVATE pe_static_lubstage_fluid)
add_test(
  NAME pe-lubrication-aabb-neutrality
  COMMAND pe_lubrication_aabb_neutrality_test
)

# Refusal guard (D2.2 §3.5): lubricationEnabled_ must never be a silent no-op.
# Deliberately links the SHIPPED pe_static -- not a stage-capable variant -- because the
# configuration it polices is the default one a user actually gets. It asserts the guard's
# contract (throws iff enabled && !stage-capable), so it stays meaningful under both a
# neutral and a stage-capable default rather than degenerating into a tautology.
# Currently also the only test that exercises the shipped default configuration.
add_executable(pe_lubrication_refusal_test
  pe_lubrication_refusal_test.cpp
)
target_link_libraries(pe_lubrication_refusal_test PRIVATE pe_static)
add_test(
  NAME pe-lubrication-refusal
  COMMAND pe_lubrication_refusal_test
)
