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
