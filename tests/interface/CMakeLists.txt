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

# Exercise CFD shadow queries with unequal owned/shadow counts and filtered indices.
add_executable(pe_remote_query_test pe_remote_query_test.cpp)
target_link_libraries(pe_remote_query_test PRIVATE pe_static_lubstage_fluid)
add_test(NAME pe-remote-query COMMAND pe_remote_query_test)
set_tests_properties(pe-remote-query PROPERTIES LABELS "unit;serial;interface")

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
set_tests_properties(pe-el-semi-implicit-drag PROPERTIES
  LABELS "unit;serial"
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
set_tests_properties(pe-lubrication-model PROPERTIES
  LABELS "unit;serial;lubrication"
)

# Pure-arithmetic unit test for modelKroupaDeficit (gate-G2 deficit form): subtraction
# identity, continuous onset at the activation gap, full-model inertness, hCut=0
# fail-safe. Same harness class as pe_lubrication_model_test.
add_executable(pe_lubrication_deficit_test
  pe_lubrication_deficit_test.cpp
)
target_link_libraries(pe_lubrication_deficit_test PRIVATE pe_static)
add_test(
  NAME pe-lubrication-deficit
  COMMAND pe_lubrication_deficit_test
)
set_tests_properties(pe-lubrication-deficit PROPERTIES
  LABELS "unit;serial;lubrication"
)

# Pins the solid-ellipsoid inertia tensor (torque-path review defect D-1: I_zz carried
# a 1/4 factor where the solid-ellipsoid value is 1/5 on all three principal axes).
# Serial world, no MPI.
add_executable(pe_ellipsoid_inertia_test
  pe_ellipsoid_inertia_test.cpp
)
target_link_libraries(pe_ellipsoid_inertia_test PRIVATE pe_static)
add_test(
  NAME pe-ellipsoid-inertia
  COMMAND pe_ellipsoid_inertia_test
)
set_tests_properties(pe-ellipsoid-inertia PROPERTIES
  LABELS "unit;serial;rigidbody"
)

# Pins the per-axis angular DOF mask (RigidBody::setAngularDofMask), the angular mirror of
# the linear mask: a sphere and an ellipsoid under mask (0,1,0) keep w_y bit-for-bit and get
# w_x = w_z = 0 exactly after applyFluidForces and after move; mask (1,1,1) is bit-identical
# to no mask; the linear mask and v are untouched. Serial world, no MPI.
add_executable(pe_angular_dof_mask_test
  pe_angular_dof_mask_test.cpp
)
target_link_libraries(pe_angular_dof_mask_test PRIVATE pe_static)
add_test(
  NAME pe-angular-dof-mask
  COMMAND pe_angular_dof_mask_test
)
set_tests_properties(pe-angular-dof-mask PROPERTIES
  LABELS "unit;serial;rigidbody"
)

# Contract test for the optional angularDofMask_ deck key: absent -> (1,1,1); [0,1,0] ->
# (0,1,0); wrong shape / non-0-or-1 entries -> std::invalid_argument naming the key; and the
# setup-time guard (checkAngularDofMaskRequiresRotationOnly) refuses the key under any
# particleMotion_ other than rotationOnly. Config singleton only, no world/MPI; writes decks
# into ./_angmask_config_test/.
add_executable(pe_angular_dof_mask_config_test
  pe_angular_dof_mask_config_test.cpp
)
target_link_libraries(pe_angular_dof_mask_config_test PRIVATE pe_static)
add_test(
  NAME pe-angular-dof-mask-config
  COMMAND pe_angular_dof_mask_config_test
)
set_tests_properties(pe-angular-dof-mask-config PROPERTIES
  LABELS "unit;serial;interface"
)

# Contract test for the optional boxWallsEnabled_ deck key of the EL terminal-velocity setup:
# absent -> false, true/false read, a later deck without the key keeps the value. Config singleton
# only, no world/MPI; writes decks into ./_box_walls_config_test/.
add_executable(pe_box_walls_config_test
  pe_box_walls_config_test.cpp
)
target_link_libraries(pe_box_walls_config_test PRIVATE pe_static)
add_test(
  NAME pe-box-walls-config
  COMMAND pe_box_walls_config_test
)
set_tests_properties(pe-box-walls-config PROPERTIES
  LABELS "unit;serial;interface"
)

# Contract + body-level test for the two D6.4 particleMotion_ modes. Config: the four values
# fixed / rotationOnly / translationOnly / free parse (setter and json), an unknown value throws
# std::invalid_argument; angularDofMask_ is accepted under free and refused under
# translationOnly with a message naming both. Body level (sphere AND ellipsoid, via
# applyParticleMotionLocks + applyFluidForces): translationOnly leaves v free and zeros w
# exactly; free leaves both free and bit-identical to an unlocked body; rotationOnly and fixed
# unchanged as controls. Serial world, no MPI; writes decks into ./_particle_motion_test/.
add_executable(pe_particle_motion_test
  pe_particle_motion_test.cpp
)
target_link_libraries(pe_particle_motion_test PRIVATE pe_static)
add_test(
  NAME pe-particle-motion
  COMMAND pe_particle_motion_test
)
set_tests_properties(pe-particle-motion PROPERTIES
  LABELS "unit;serial;interface;rigidbody"
)

# Pins the ellipsoid support mapping (was a sphere of the semi-major radius), the exact
# bounding box of a rotated ellipsoid (was the positive axis tips only) and the radial depth.
# Serial world, no MPI; pure geometry, no solver.
add_executable(pe_ellipsoid_support_test
  pe_ellipsoid_support_test.cpp
)
target_link_libraries(pe_ellipsoid_support_test PRIVATE pe_static)
add_test(
  NAME pe-ellipsoid-support
  COMMAND pe_ellipsoid_support_test
)
set_tests_properties(pe-ellipsoid-support PROPERTIES
  LABELS "unit;serial;rigidbody"
)

# Narrow-phase contact generation for ellipsoid pairs through MaxContacts::collide() with a
# recording contact container (ellipsoid-ellipsoid, -sphere, -box via GJK/EPA; -plane analytic).
# Serial world, no MPI; the solver is not involved. Timeout guards against a GJK/EPA stall.
add_executable(pe_ellipsoid_contact_test
  pe_ellipsoid_contact_test.cpp
)
target_link_libraries(pe_ellipsoid_contact_test PRIVATE pe_static)
add_test(
  NAME pe-ellipsoid-contact
  COMMAND pe_ellipsoid_contact_test
)
set_tests_properties(pe-ellipsoid-contact PROPERTIES
  LABELS "unit;serial;rigidbody"
  TIMEOUT 300
)

# Cylinder-plane contact generation (MaxContacts::collideCylinderPlane, formerly an empty
# stub): flat-on-cap / lying / rim cases against analytic values, the contactThreshold
# boundary, and a random sweep against CylinderBase::support(). Recording contact container,
# no solver; serial world, no MPI.
add_executable(pe_cylinder_plane_contact_test
  pe_cylinder_plane_contact_test.cpp
)
target_link_libraries(pe_cylinder_plane_contact_test PRIVATE pe_static)
add_test(
  NAME pe-cylinder-plane-contact
  COMMAND pe_cylinder_plane_contact_test
)
set_tests_properties(pe-cylinder-plane-contact PROPERTIES
  LABELS "unit;serial;rigidbody"
  TIMEOUT 120
)

# Multi-point contact manifolds for box-cylinder, cylinder-cylinder and capsule-cylinder
# (MaxContacts::addFeatureManifold on top of GJK/EPA): standing / lying / overhanging / stacked
# configurations against analytic contact sets, single-contact cases kept, and a small-tilt sweep
# checking the deepest manifold point against the exact support depth. Serial world, no MPI.
add_executable(pe_cylinder_manifold_test
  pe_cylinder_manifold_test.cpp
)
target_link_libraries(pe_cylinder_manifold_test PRIVATE pe_static)
add_test(
  NAME pe-cylinder-manifold
  COMMAND pe_cylinder_manifold_test
)
set_tests_properties(pe-cylinder-manifold PROPERTIES
  LABELS "unit;serial;rigidbody"
  TIMEOUT 120
)

# Hard-contact solvers: a fixed body must not inherit the stale velocity correction of a previous
# body in its dv_/dw_ slot (after World::clear() or when BodyStorage::remove() swaps it into a
# destroyed body's slot). Boxes resting on a fixed ramp must stick in all three cases.
# Serial world, no MPI.
add_executable(pe_fixed_body_stale_correction_test
  pe_fixed_body_stale_correction_test.cpp
)
target_link_libraries(pe_fixed_body_stale_correction_test PRIVATE pe_static)
add_test(
  NAME pe-fixed-body-stale-correction
  COMMAND pe_fixed_body_stale_correction_test
)
set_tests_properties(pe-fixed-body-stale-correction PROPERTIES
  LABELS "unit;serial;rigidbody"
  TIMEOUT 120
)

# Split-impulse position correction of HardContactEulerLagrange (setSplitImpulse): an
# overlapping box is not launched, a dropped box does not bounce at restitution 0, the pseudo
# motion propagates through touching contacts, and a resting tower stays at rest. Each case also
# runs with the switch off to pin the Baumgarte behaviour it replaces. Serial, no MPI.
add_executable(pe_split_impulse_test
  pe_split_impulse_test.cpp
)
target_link_libraries(pe_split_impulse_test PRIVATE pe_static)
add_test(
  NAME pe-split-impulse
  COMMAND pe_split_impulse_test
)
set_tests_properties(pe-split-impulse PROPERTIES
  LABELS "unit;serial;rigidbody"
  SKIP_RETURN_CODE 77
  TIMEOUT 120
)

# Box-box contacts with (nearly) parallel edges (MaxContacts::collideBoxBox): edge-pair axes of
# parallel edges used to produce O(1) garbage that replaced the contact of overlapping boxes by
# a positive-distance edge/edge contact. Random near-parallel configurations checked against GJK,
# plus a clean edge-versus-face approach. Recording contact container, no solver; serial, no MPI.
add_executable(pe_box_box_parallel_edges_test
  pe_box_box_parallel_edges_test.cpp
)
target_link_libraries(pe_box_box_parallel_edges_test PRIVATE pe_static)
add_test(
  NAME pe-box-box-parallel-edges
  COMMAND pe_box_box_parallel_edges_test
)
set_tests_properties(pe-box-box-parallel-edges PROPERTIES
  LABELS "unit;serial;rigidbody"
  TIMEOUT 120
)

# Primitive-mesh contacts on a DistanceMap (MaxContacts::collideTMeshWithDistanceMap): sphere,
# box, capsule, cylinder and ellipsoid against a non-convex torus, checked against the analytic
# torus distance; the hole case shows what the DistanceMap adds over GJK/EPA. Needs CGAL; the
# executable is a no-op without it. Serial world, no MPI.
if(PE_USE_CGAL)
  add_executable(pe_primitive_mesh_distancemap_test
    pe_primitive_mesh_distancemap_test.cpp
  )
  target_link_libraries(pe_primitive_mesh_distancemap_test PRIVATE pe_static)
  add_test(
    NAME pe-primitive-mesh-distancemap
    COMMAND pe_primitive_mesh_distancemap_test
  )
  set_tests_properties(pe-primitive-mesh-distancemap PROPERTIES
    LABELS "unit;serial;rigidbody;cgal"
    TIMEOUT 300
  )
endif()

# Checkpoint write/read round trip for the DNS/DKT resume path: a moving sphere and a
# rotated, spinning ellipsoid survive with position, orientation and both velocities intact;
# the sidecar carries the driver identity. Ellipsoids were not checkpointed before this.
# Serial world, no MPI; writes into ./_ckpt_roundtrip_test/.
add_executable(pe_checkpoint_roundtrip_test
  pe_checkpoint_roundtrip_test.cpp
)
target_link_libraries(pe_checkpoint_roundtrip_test PRIVATE pe_static)
add_test(
  NAME pe-checkpoint-roundtrip
  COMMAND pe_checkpoint_roundtrip_test
)
set_tests_properties(pe-checkpoint-roundtrip PROPERTIES
  LABELS "unit;serial;checkpoint"
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
set_tests_properties(pe-lubrication-legacy-regression PROPERTIES
  LABELS "integration;serial;lubrication"
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
set_tests_properties(pe-lubrication-two-sphere-approach PROPERTIES
  LABELS "integration;serial;lubrication"
)

add_executable(pe_lubrication_bounce_stokes_test
  pe_lubrication_bounce_stokes_test.cpp
)
target_link_libraries(pe_lubrication_bounce_stokes_test PRIVATE pe_static_lubstage_fluid)
add_test(
  NAME pe-lubrication-bounce-stokes
  COMMAND pe_lubrication_bounce_stokes_test
)
set_tests_properties(pe-lubrication-bounce-stokes PROPERTIES
  LABELS "integration;serial;lubrication"
)

# Dynamics smoke test: a tilted prolate spheroid dropped on a plane settles broadside at
# height B; a second one dropped crosswise onto it is deflected without interpenetration. Needs a solver that applies gravity
# itself, so it links the plain hard-contact variant (the shipped HardContactEulerLagrange
# default leaves body forces to the CFD driver).
add_executable(pe_ellipsoid_settle_test
  pe_ellipsoid_settle_test.cpp
)
target_link_libraries(pe_ellipsoid_settle_test PRIVATE pe_static_lubstage_plain)
add_test(
  NAME pe-ellipsoid-settle
  COMMAND pe_ellipsoid_settle_test
)
set_tests_properties(pe-ellipsoid-settle PROPERTIES
  LABELS "integration;serial;rigidbody"
  TIMEOUT 600
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
set_tests_properties(pe-checkpoint-metadata PROPERTIES
  LABELS "integration;serial;checkpoint"
)

# Two-rank counterpart. The collective parts of the checkpoint write -- the rank-invariant scratch
# filename MPI_File_open demands, and the cross-rank body-count reduction -- cannot be reached by
# any serial test, and a mistake in either is silent in a serial-only suite.
if(PE_USE_MPI AND MPI_FOUND)
  add_executable(pe_checkpoint_metadata_mpi
    pe_checkpoint_metadata_mpi.cpp
  )
  target_link_libraries(pe_checkpoint_metadata_mpi PRIVATE pe_static MPI::MPI_CXX)
  add_test(
    NAME pe-checkpoint-metadata-mpi
    COMMAND
      ${MPIEXEC_EXECUTABLE} ${MPIEXEC_NUMPROC_FLAG} 2
      ${MPIEXEC_PREFLAGS}
      $<TARGET_FILE:pe_checkpoint_metadata_mpi>
      ${MPIEXEC_POSTFLAGS}
  )
  set_tests_properties(pe-checkpoint-metadata-mpi PROPERTIES
    LABELS "integration;mpi;checkpoint"
  )
endif()

# Failure propagation is a collective contract: timeout each two-rank scenario so a
# regression fails CTest instead of blocking the entire checkpoint suite.
if(PE_USE_MPI AND MPI_FOUND)
  add_executable(pe_checkpoint_failure_mpi pe_checkpoint_failure_mpi.cpp)
  target_link_libraries(pe_checkpoint_failure_mpi PRIVATE pe_static MPI::MPI_CXX)
  foreach(scenario IN ITEMS directory open binary-open rename sidecar preserve metadata missing material body submit complete close read)
    add_test(NAME pe-checkpoint-failure-${scenario}
      COMMAND ${MPIEXEC_EXECUTABLE} ${MPIEXEC_NUMPROC_FLAG} 2 ${MPIEXEC_PREFLAGS}
        $<TARGET_FILE:pe_checkpoint_failure_mpi> ${scenario} ${MPIEXEC_POSTFLAGS})
    set_tests_properties(pe-checkpoint-failure-${scenario} PROPERTIES
      LABELS "integration;mpi;checkpoint" TIMEOUT 60 PROCESSORS 2)
  endforeach()
endif()

# A checkpoint written by two ranks (MPI-IO) read by a serial program of the same build, which
# must take the stream path without MPI_Init. The write is the fixture of the read.
if(PE_USE_MPI AND MPI_FOUND)
  add_executable(pe_checkpoint_parallel_file_serial_read
    pe_checkpoint_parallel_file_serial_read.cpp
  )
  target_link_libraries(pe_checkpoint_parallel_file_serial_read PRIVATE pe_static MPI::MPI_CXX)
  set(PE_TWO_RANK_CHECKPOINT_DIR "${CMAKE_CURRENT_BINARY_DIR}/two-rank-checkpoint")
  add_test(
    NAME pe-checkpoint-two-rank-write
    COMMAND
      ${MPIEXEC_EXECUTABLE} ${MPIEXEC_NUMPROC_FLAG} 2
      ${MPIEXEC_PREFLAGS}
      $<TARGET_FILE:pe_checkpoint_parallel_file_serial_read>
      ${MPIEXEC_POSTFLAGS}
      --write "${PE_TWO_RANK_CHECKPOINT_DIR}"
  )
  set_tests_properties(pe-checkpoint-two-rank-write PROPERTIES
    LABELS "integration;mpi;checkpoint"
    FIXTURES_SETUP pe-two-rank-checkpoint
  )
  add_test(
    NAME pe-checkpoint-two-rank-serial-read
    COMMAND pe_checkpoint_parallel_file_serial_read --read "${PE_TWO_RANK_CHECKPOINT_DIR}"
  )
  set_tests_properties(pe-checkpoint-two-rank-serial-read PROPERTIES
    LABELS "integration;serial;checkpoint"
    FIXTURES_REQUIRED pe-two-rank-checkpoint
    SKIP_RETURN_CODE 77
  )
endif()

function(pe_add_interface_serial_test test_name case_name)
  add_test(
    NAME ${test_name}
    COMMAND pe_interface_smoke_serial
      --case ${case_name}
      --fixtures "${PE_INTERFACE_SERIAL_FIXTURE_DIR}"
      --work-root "${PE_INTERFACE_SERIAL_WORK_ROOT}"
  )
  set(test_labels integration serial interface ${ARGN})
  set_tests_properties(${test_name} PROPERTIES
    LABELS "${test_labels}"
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
    cgal
  )
  pe_add_interface_serial_test(
    pe-interface-serial-span-complex-distancemaps
    span-complex-distancemaps
    cgal
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

if(PE_USE_MPI AND MPI_FOUND AND PE_USE_CGAL)
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
      LABELS "integration;mpi;cgal;interface"
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
set_tests_properties(pe-lubrication-serial-coverage PROPERTIES
  LABELS "integration;serial;lubrication"
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
set_tests_properties(pe-lubrication-aabb-neutrality PROPERTIES
  LABELS "integration;serial;lubrication"
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
set_tests_properties(pe-lubrication-refusal PROPERTIES
  LABELS "integration;serial;lubrication"
)

# Stale velocity-correction regression: a FIXED body whose per-step storage index is recycled
# from a departed mobile body (BodyStorage::remove is swap-with-last) must not inherit that
# body's dv_/dw_ entry. initializeVelocityCorrections() used to assign dv/dw only for awake
# non-fixed bodies while the vectors are resize()d, never cleared, so every contact against the
# wall was sized as if it moved at the ghost velocity (impulses no body absorbs). The solver
# only sees v_ + dv_, so the defect and the fix are solver-generic; one source is registered
# against each hard-contact pipeline: the campaign solver (HardContactAndFluid), the neutral
# semi-implicit solver, and the shipped default (HardContactEulerLagrange via pe_static).
# Body forces are applied explicitly so the same scenario drives all three. Serial world, no MPI.
add_executable(pe_stale_velocity_correction_fluid_test
  pe_stale_velocity_correction_test.cpp
)
target_link_libraries(pe_stale_velocity_correction_fluid_test PRIVATE pe_static_lubstage_fluid)
add_test(
  NAME pe-stale-velocity-correction-fluid
  COMMAND pe_stale_velocity_correction_fluid_test
)
set_tests_properties(pe-stale-velocity-correction-fluid PROPERTIES
  LABELS "integration;serial;rigidbody"
)

add_executable(pe_stale_velocity_correction_plain_test
  pe_stale_velocity_correction_test.cpp
)
target_link_libraries(pe_stale_velocity_correction_plain_test PRIVATE pe_static_lubstage_plain)
add_test(
  NAME pe-stale-velocity-correction-plain
  COMMAND pe_stale_velocity_correction_plain_test
)
set_tests_properties(pe-stale-velocity-correction-plain PROPERTIES
  LABELS "integration;serial;rigidbody"
)

add_executable(pe_stale_velocity_correction_test
  pe_stale_velocity_correction_test.cpp
)
target_link_libraries(pe_stale_velocity_correction_test PRIVATE pe_static)
add_test(
  NAME pe-stale-velocity-correction
  COMMAND pe_stale_velocity_correction_test
)
set_tests_properties(pe-stale-velocity-correction PROPERTIES
  LABELS "integration;serial;rigidbody"
)

# Split impulse against the other two hard-contact pipelines (the default pipeline's run is
# pe-split-impulse above): the same source linked against the fluid-coupled and the semi-implicit
# library variants. 77 = configured system without the switch (skipped).
add_executable(pe_split_impulse_fluid_test
  pe_split_impulse_test.cpp
)
target_link_libraries(pe_split_impulse_fluid_test PRIVATE pe_static_lubstage_fluid)
add_test(
  NAME pe-split-impulse-fluid
  COMMAND pe_split_impulse_fluid_test
)
set_tests_properties(pe-split-impulse-fluid PROPERTIES
  LABELS "unit;serial;rigidbody"
  SKIP_RETURN_CODE 77
  TIMEOUT 120
)

add_executable(pe_split_impulse_plain_test
  pe_split_impulse_test.cpp
)
target_link_libraries(pe_split_impulse_plain_test PRIVATE pe_static_lubstage_plain)
add_test(
  NAME pe-split-impulse-plain
  COMMAND pe_split_impulse_plain_test
)
set_tests_properties(pe-split-impulse-plain PROPERTIES
  LABELS "unit;serial;rigidbody"
  SKIP_RETURN_CODE 77
  TIMEOUT 120
)

# Hollow (shell) spheres: Sphere::setShellThickness() mass and inertia, createHollowSphere(),
# fixed bodies, and the checkpoint round trip of the wall thickness (file format 0.3). Serial.
add_executable(pe_hollow_sphere_test
  pe_hollow_sphere_test.cpp
)
target_link_libraries(pe_hollow_sphere_test PRIVATE pe_static)
add_test(
  NAME pe-hollow-sphere
  COMMAND pe_hollow_sphere_test
)
set_tests_properties(pe-hollow-sphere PROPERTIES
  LABELS "unit;serial;rigidbody"
  TIMEOUT 60
)

# Newton restitution of the semi-implicit hard-contact solver (setRestitution()): ITTF drop
# test, inelastic default, rest without micro-bounces, sliding backspin impact. Linked against
# the semi-implicit library variant; 77 = configured solver without the switch (skipped).
add_executable(pe_hcsits_restitution_test
  pe_hcsits_restitution_test.cpp
)
target_link_libraries(pe_hcsits_restitution_test PRIVATE pe_static_lubstage_plain)
add_test(
  NAME pe-hcsits-restitution
  COMMAND pe_hcsits_restitution_test
)
set_tests_properties(pe-hcsits-restitution PROPERTIES
  LABELS "unit;serial;rigidbody"
  SKIP_RETURN_CODE 77
  TIMEOUT 120
)

# Restitution of the Euler-Lagrange solver (HardContactEulerLagrange::setRestitution(), port of
# the HCSITS switch above; shared post-pass in pe/core/collisionsystem/RestitutionPostPass.h):
# head-on impacts (Newton law, momentum, energy e^2, equal and unequal masses), drop on the floor
# (apex ratio e^2, rest), the impact law with the EL drag active, oblique impact with friction, a
# resting pile in a box (stays at rest; switch off bitwise reproducible), simultaneous contacts.
# Linked against the default library (the EL solver); 77 = another configured solver (skipped).
add_executable(pe_el_restitution_test
  pe_el_restitution_test.cpp
)
target_link_libraries(pe_el_restitution_test PRIVATE pe_static)
add_test(
  NAME pe-el-restitution
  COMMAND pe_el_restitution_test
)
set_tests_properties(pe-el-restitution PROPERTIES
  LABELS "unit;serial;rigidbody"
  SKIP_RETURN_CODE 77
  TIMEOUT 120
)

# Restitution at the scale of the case-11 bed (0.87 mm spheres, EL drag, v_rest 0.01 m/s, default
# stop rule): a defluidised pile comes to rest with zero delivered rebounds, impacts in the fluidised
# range keep their rebound. EL solver (default library); 77 = another configured solver.
add_executable(pe_el_restitution_rest_test pe_el_restitution_rest_test.cpp)
target_link_libraries(pe_el_restitution_rest_test PRIVATE pe_static)
add_test(NAME pe-el-restitution-rest COMMAND pe_el_restitution_rest_test)
set_tests_properties(pe-el-restitution-rest PROPERTIES
  LABELS "integration;serial;rigidbody"
  SKIP_RETURN_CODE 77
  TIMEOUT 170
)

# Restitution post-pass with several contacts per body (support contacts of a chain and of a pile,
# frictional unloading, a positive gap below the contact threshold, a remote impact, an orthogonal
# impact on a loaded floor contact) and its stop rule (pe-restitution-stoprule), on both solvers that have the
# switch: the EL solver (default library) and the semi-implicit one (plain variant).
foreach(variant "" "_plain")
  if(variant STREQUAL "_plain")
    set(support_lib pe_static_lubstage_plain)
  else()
    set(support_lib pe_static)
  endif()
  string(REPLACE "_" "-" variant_name "${variant}")
  add_executable(pe_restitution_stoprule${variant}_test pe_restitution_stoprule_test.cpp)
  target_link_libraries(pe_restitution_stoprule${variant}_test PRIVATE ${support_lib})
  add_test(NAME pe-restitution-stoprule${variant_name} COMMAND pe_restitution_stoprule${variant}_test)
  set_tests_properties(pe-restitution-stoprule${variant_name} PROPERTIES
    LABELS "unit;serial;rigidbody"
    SKIP_RETURN_CODE 77
    TIMEOUT 120
  )
  add_executable(pe_restitution_support${variant}_test pe_restitution_support_test.cpp)
  target_link_libraries(pe_restitution_support${variant}_test PRIVATE ${support_lib})
  add_test(NAME pe-restitution-support${variant_name} COMMAND pe_restitution_support${variant}_test)
  set_tests_properties(pe-restitution-support${variant_name} PROPERTIES
    LABELS "unit;serial;rigidbody"
    SKIP_RETURN_CODE 77
    TIMEOUT 120
  )
endforeach()

# Unit tests of the post-pass on a mock collision system (RestitutionPass called directly): skip
# certificate, Poisson ceiling through unload and reload, tangential stop decision, wall scale.
add_executable(pe_restitution_pass_unit_test pe_restitution_pass_unit_test.cpp)
target_link_libraries(pe_restitution_pass_unit_test PRIVATE pe_static)
add_test(NAME pe-restitution-pass-unit COMMAND pe_restitution_pass_unit_test)
set_tests_properties(pe-restitution-pass-unit PROPERTIES
  LABELS "unit;serial;rigidbody"
  TIMEOUT 60
)

# Json contract of restitutionEnabled_ / restitutionThreshold_ / restitutionScheme_ /
# restitutionIterations_ (per-deck defaults, validation), the applier (applyOptionalRestitutionParams)
# and the EL_CONTACT_PARAMS start-up echo line (pe/interface/support/contact_params_echo.h); also
# against HardContactAndFluid, which has no switch (the applier must refuse an enable). And the
# setups without restitution support refusing an explicit restitutionEnabled_ = true (one serial EL
# and one serial DNS setup, under the EL solver and under HardContactAndFluid). These tests write
# decks, so they need the json loader.
if(PE_USE_JSON)
  foreach(variant "" "_fluid")
    if(variant STREQUAL "_fluid")
      set(restitution_cfg_lib pe_static_lubstage_fluid)
    else()
      set(restitution_cfg_lib pe_static)
    endif()
    string(REPLACE "_" "-" variant_name "${variant}")
    add_executable(pe_el_restitution_config${variant}_test pe_el_restitution_config_test.cpp)
    target_link_libraries(pe_el_restitution_config${variant}_test PRIVATE ${restitution_cfg_lib})
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/el_restitution_config${variant})
    add_test(NAME pe-el-restitution-config${variant_name} COMMAND pe_el_restitution_config${variant}_test
             WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/el_restitution_config${variant})
    set_tests_properties(pe-el-restitution-config${variant_name} PROPERTIES LABELS "unit;serial;interface")

    add_executable(pe_el_restitution_setup${variant}_test pe_el_restitution_setup_test.cpp)
    target_link_libraries(pe_el_restitution_setup${variant}_test PRIVATE ${restitution_cfg_lib})
    foreach(setup frozen dnsdrag inherit)
      file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/el_restitution_setup${variant}_${setup})
      add_test(NAME pe-el-restitution-setup${variant_name}-${setup}
               COMMAND pe_el_restitution_setup${variant}_test ${setup}
               WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/el_restitution_setup${variant}_${setup})
      set_tests_properties(pe-el-restitution-setup${variant_name}-${setup} PROPERTIES LABELS "unit;serial;interface")
    endforeach()
  endforeach()
endif()

# Static against dynamic coefficient of friction (Contact.cpp): a sliding contact must take the
# dynamic coefficient, a resting one the static coefficient. Box sliding on a plane, box resting
# on and sliding down an incline whose slope lies between the two coefficients, box on a fixed box.
add_executable(pe_contact_friction_coefficient_test
  pe_contact_friction_coefficient_test.cpp
)
target_link_libraries(pe_contact_friction_coefficient_test PRIVATE pe_static_lubstage_plain)
add_test(
  NAME pe-contact-friction-coefficient
  COMMAND pe_contact_friction_coefficient_test
)
set_tests_properties(pe-contact-friction-coefficient PROPERTIES
  LABELS "unit;serial;rigidbody"
  TIMEOUT 120
)

# Kinematic (moving fixed) bodies and surface velocities. pe_kinematic_body_test: a fixed body
# keeps a prescribed velocity (MOBILE_INFINITE, CMake option PE_MOBILE_INFINITE), no contacts
# between fixed bodies, RigidBody::setTargetPose() lands on the target, bodies on a moving
# platform are woken and carried. pe_surface_velocity_test: RigidBody::setRelSurfaceVel() belts
# (box, rotated, plane, union, ceiling) carry what lies on them, no effect on the normal
# direction, checkpoint round trip. Each source is linked against the configured library and the
# two solver variants, so every hard-contact pipeline runs it. 77 = configured system without the
# feature (skipped). Serial, no MPI.
foreach(kinematic_test kinematic_body surface_velocity)
  string(REPLACE "_" "-" kinematic_test_name ${kinematic_test})
  foreach(variant "" "_fluid" "_plain")
    if(variant STREQUAL "_fluid")
      set(kinematic_lib pe_static_lubstage_fluid)
    elseif(variant STREQUAL "_plain")
      set(kinematic_lib pe_static_lubstage_plain)
    else()
      set(kinematic_lib pe_static)
    endif()
    string(REPLACE "_" "-" variant_name "${variant}")
    add_executable(pe_${kinematic_test}${variant}_test pe_${kinematic_test}_test.cpp)
    target_link_libraries(pe_${kinematic_test}${variant}_test PRIVATE ${kinematic_lib})
    add_test(
      NAME pe-${kinematic_test_name}${variant_name}
      COMMAND pe_${kinematic_test}${variant}_test
    )
    set_tests_properties(pe-${kinematic_test_name}${variant_name} PROPERTIES
      LABELS "unit;serial;rigidbody"
      SKIP_RETURN_CODE 77
      TIMEOUT 120
    )
  endforeach()
endforeach()

# HashGrids::getBodiesNearPoint must also return bodies added since the last findContacts()
# (bodiesToAdd_), both with the grid inactive and active, so that FBM point queries see a
# particle inserted between simulation steps. Serial, no MPI.
add_executable(pe_hashgrid_pending_body_test
  pe_hashgrid_pending_body_test.cpp
)
target_link_libraries(pe_hashgrid_pending_body_test PRIVATE pe_static)
add_test(
  NAME pe-hashgrid-pending-body
  COMMAND pe_hashgrid_pending_body_test
)
set_tests_properties(pe-hashgrid-pending-body PROPERTIES
  LABELS "unit;serial;detection"
  TIMEOUT 120
)

# ---------------------------------------------------------------------------------------
# FBM classification query: the accelerated (HashGrid) point query against the exhaustive
# one, on the same frozen state. Everything downstream of this query - FeatFloWer's KVEL
# candidate sets, its alpha field, its forces - consumes the ANSWER, so a force comparison
# cannot detect a classification miss; only this comparison can. Single- and two-body cases
# cannot either: pe builds no grid below gridActivationThreshold (32 bodies), so every
# stage here asserts the grid is ACTIVE and reports a stage that ran without one as having
# tested nothing.
#
# Stages marked [control] run without refreshSpatialIndex() and are expected to show the
# stale-cell miss; they do not fail the test. The refreshed stage is the verdict.
#
# CGAL-dependent mesh stages compile only when pe was built with CGAL; pe_cgal is linked
# PUBLIC into the library target, so the definition and include dirs arrive automatically
# and the test TU cannot disagree with the library about the TriangleMesh layout.
add_executable(pe_fbm_query_stress pe_fbm_query_stress.cpp)
target_link_libraries(pe_fbm_query_stress PRIVATE pe_static_lubstage_fluid)
add_test(NAME pe-fbm-query-stress COMMAND pe_fbm_query_stress)

# The test presets select by label (test-serial takes "serial", test-cgal takes "cgal"),
# so a test without one of those labels is built by CI and never run. The mesh and
# DistanceMap stages exist only in a CGAL build, which is what earns the second label.
set(fbm_query_labels "integration;serial;fbm;query")
if(PE_USE_CGAL)
  list(APPEND fbm_query_labels "cgal")
endif()
set_tests_properties(pe-fbm-query-stress PROPERTIES LABELS "${fbm_query_labels}")

# The parallel counterpart. Shadow copies, the synchronize() seam and migration across a
# subdomain cut cannot be reached by any serial stage. Verdicts reduce over all ranks, so
# no rank's silence is taken for agreement.
if(PE_USE_MPI AND MPI_FOUND)
  add_executable(pe_fbm_query_stress_mpi pe_fbm_query_stress_mpi.cpp)
  target_link_libraries(pe_fbm_query_stress_mpi PRIVATE pe_static_lubstage_fluid MPI::MPI_CXX)

  # Open MPI refuses to start more ranks than the host has slots, so the 4-rank case
  # cannot run on a two-core CI runner. That decomposition is not optional - a body
  # lattice wider than a rank's slab is a defect only 4 ranks can reach, and it already
  # caught one - so the test oversubscribes rather than being dropped.
  #
  # It asks through the environment rather than through a --oversubscribe preflag,
  # because the flag has to be spelled for the right launcher: srun has no such option,
  # and a configure-time probe of `mpiexec --version` recognised the local Open MPI but
  # not the one on the CI runner. These MCA variables are what the flag sets anyway
  # (rmaps_base_oversubscribe for Open MPI 4, the PRRTE mapping policy for 5), they are
  # read by the launcher the ranks actually start under, and an MPI that does not know
  # them ignores them. MPIEXEC_PREFLAGS stays honoured for whoever needs a real flag.
  set(fbm_query_mpi_oversubscribe
    "OMPI_MCA_rmaps_base_oversubscribe=1"
    "PRTE_MCA_rmaps_default_mapping_policy=:oversubscribe"
  )

  foreach(nranks 2 4)
    add_test(
      NAME pe-fbm-query-stress-mpi-${nranks}
      COMMAND
        ${MPIEXEC_EXECUTABLE} ${MPIEXEC_NUMPROC_FLAG} ${nranks}
        ${MPIEXEC_PREFLAGS}
        $<TARGET_FILE:pe_fbm_query_stress_mpi>
        ${MPIEXEC_POSTFLAGS}
    )
    # PROCESSORS keeps ctest from co-scheduling the 4-rank run beside another test on
    # a machine that has barely enough cores for it alone.
    set_tests_properties(pe-fbm-query-stress-mpi-${nranks} PROPERTIES
      LABELS "integration;mpi;fbm;query"
      PROCESSORS ${nranks}
      ENVIRONMENT "${fbm_query_mpi_oversubscribe}"
    )
  endforeach()
endif()

# Contract test of the parallel-mode DKT / walled-column setup (pe/interface/setup/dkt.h):
# two ranks, a 1 x 1 x 2 cut of the campaign column handed in as the CFD domain, spheres from
# the xyz file. Checks that the parallel half builds the serial half's world: every sphere once,
# on the rank of its slab, serial materials and order, the ground plane as the only plane, deck
# values in the world. The setup's refusals end in MPI_Abort and are not exercised here.
if(PE_USE_MPI AND MPI_FOUND)
  add_executable(pe_dkt_parallel_setup_mpi pe_dkt_parallel_setup_mpi.cpp)
  target_link_libraries(pe_dkt_parallel_setup_mpi PRIVATE pe_static_lubstage_fluid MPI::MPI_CXX)
  add_test(
    NAME pe-dkt-parallel-setup-mpi
    COMMAND
      ${MPIEXEC_EXECUTABLE} ${MPIEXEC_NUMPROC_FLAG} 2
      ${MPIEXEC_PREFLAGS}
      $<TARGET_FILE:pe_dkt_parallel_setup_mpi>
      ${MPIEXEC_POSTFLAGS}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  )
  set_tests_properties(pe-dkt-parallel-setup-mpi PROPERTIES
    LABELS "integration;mpi;interface"
    PROCESSORS 2
    ENVIRONMENT "${fbm_query_mpi_oversubscribe}"
  )
endif()

# Restitution of the EL solver across a process boundary: two ranks, the EL terminal-velocity
# setup with restitutionEnabled_ from the deck; a pair and a coupled chain straddling the cut against
# the same inside one domain, the relaxation recurrence, the refusal of a mobile global body. Own
# working directory (the test writes example.json, as the DKT setup test does in its own). The test
# decides on the COMPILED solver (another one may come in through PE_PREPROCESSOR_FLAGS or
# CMAKE_CXX_FLAGS) and returns 77 (skipped) unless it is HardContactEulerLagrange; needs json.
if(PE_USE_MPI AND MPI_FOUND AND PE_USE_JSON)
  add_executable(pe_el_restitution_mpi pe_el_restitution_mpi.cpp)
  target_link_libraries(pe_el_restitution_mpi PRIVATE pe_static MPI::MPI_CXX)
  file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/el_restitution_mpi)
  add_test(
    NAME pe-el-restitution-mpi
    COMMAND
      ${MPIEXEC_EXECUTABLE} ${MPIEXEC_NUMPROC_FLAG} 2
      ${MPIEXEC_PREFLAGS}
      $<TARGET_FILE:pe_el_restitution_mpi>
      ${MPIEXEC_POSTFLAGS}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/el_restitution_mpi
  )
  set_tests_properties(pe-el-restitution-mpi PROPERTIES
    LABELS "integration;mpi;rigidbody;interface"
    PROCESSORS 2
    SKIP_RETURN_CODE 77
    ENVIRONMENT "${fbm_query_mpi_oversubscribe}"
  )
endif()

# Restitution of the semi-implicit solver on two ranks: a pair and a chain with its rows treated on
# different ranks through the convergence tail of the post-pass (momentum, exact send filter), and
# the refusal of a mobile global body on both ranks. Plain library variant; 77 = no stop rule.
if(PE_USE_MPI AND MPI_FOUND)
  add_executable(pe_hcsits_restitution_mpi pe_hcsits_restitution_mpi.cpp)
  target_link_libraries(pe_hcsits_restitution_mpi PRIVATE pe_static_lubstage_plain MPI::MPI_CXX)
  add_test(
    NAME pe-hcsits-restitution-mpi
    COMMAND
      ${MPIEXEC_EXECUTABLE} ${MPIEXEC_NUMPROC_FLAG} 2
      ${MPIEXEC_PREFLAGS}
      $<TARGET_FILE:pe_hcsits_restitution_mpi>
      ${MPIEXEC_POSTFLAGS}
  )
  set_tests_properties(pe-hcsits-restitution-mpi PROPERTIES
    LABELS "integration;mpi;rigidbody"
    PROCESSORS 2
    SKIP_RETURN_CODE 77
    ENVIRONMENT "${fbm_query_mpi_oversubscribe}"
  )
endif()

# Review round 3: the impact scale of a body is the same on every rank that holds it (one-treater
# chain against the split chain under the production stop rule), and the registry of non-fixed
# global bodies (fix + destroy, clear + repopulate, registered fixed then unfixed; restitution off),
# both solvers on two ranks.
if(PE_USE_MPI AND MPI_FOUND)
  foreach(variant "" "_plain")
    if(variant STREQUAL "_plain")
      set(r3_lib pe_static_lubstage_plain)
    else()
      set(r3_lib pe_static)
    endif()
    string(REPLACE "_" "-" variant_name "${variant}")
    add_executable(pe_restitution_scale${variant}_mpi pe_restitution_scale_mpi.cpp)
    target_link_libraries(pe_restitution_scale${variant}_mpi PRIVATE ${r3_lib} MPI::MPI_CXX)
    add_test(NAME pe-restitution-scale-mpi${variant_name}
      COMMAND ${MPIEXEC_EXECUTABLE} ${MPIEXEC_NUMPROC_FLAG} 2 ${MPIEXEC_PREFLAGS}
              $<TARGET_FILE:pe_restitution_scale${variant}_mpi> ${MPIEXEC_POSTFLAGS})
    set_tests_properties(pe-restitution-scale-mpi${variant_name} PROPERTIES
      LABELS "integration;mpi;rigidbody" PROCESSORS 2 SKIP_RETURN_CODE 77
      ENVIRONMENT "${fbm_query_mpi_oversubscribe}")
    add_executable(pe_global_registry${variant}_mpi pe_global_registry_mpi.cpp)
    target_link_libraries(pe_global_registry${variant}_mpi PRIVATE ${r3_lib} MPI::MPI_CXX)
    foreach(mode fixdestroy clear unfix)
      add_test(NAME pe-global-registry-mpi${variant_name}-${mode}
        COMMAND ${MPIEXEC_EXECUTABLE} ${MPIEXEC_NUMPROC_FLAG} 2 ${MPIEXEC_PREFLAGS}
                $<TARGET_FILE:pe_global_registry${variant}_mpi> ${MPIEXEC_POSTFLAGS} ${mode})
      set_tests_properties(pe-global-registry-mpi${variant_name}-${mode} PROPERTIES
        LABELS "integration;mpi;rigidbody" PROCESSORS 2 TIMEOUT 120
        ENVIRONMENT "${fbm_query_mpi_oversubscribe}")
    endforeach()
  endforeach()
endif()

# Contract test for the optional cylinderWallEnabled_ / cylinderWallRadius_ deck keys of the EL
# terminal-velocity setup (fixed inner cylinder as the particle-side container wall): defaults,
# explicit values, a later deck without the keys keeps the values. Config singleton only.
add_executable(pe_cylinder_wall_config_test
  pe_cylinder_wall_config_test.cpp
)
target_link_libraries(pe_cylinder_wall_config_test PRIVATE pe_static)
add_test(
  NAME pe-cylinder-wall-config
  COMMAND pe_cylinder_wall_config_test
)
set_tests_properties(pe-cylinder-wall-config PROPERTIES
  LABELS "unit;serial;interface"
)
