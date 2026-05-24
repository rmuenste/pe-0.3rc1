# PE Interface Smoke Tests

This directory contains lightweight standalone tests for PE setup functions that
are normally reached from FeatFloWer. The first runner is serial-only:

- Binary: `pe_interface_smoke_serial`
- Fixtures: `fixtures/serial`
- CTest prefix: `pe-interface-serial-*`

Each CTest case starts a separate process. The runner copies the serial fixtures
to a case-local run directory under the build tree, renames the selected config
to `example.json`, changes into that directory, and calls the real serial setup
function.

From a parent FeatFloWer build configured with `USE_PE=ON`,
`USE_PE_SERIAL_MODE=ON`, and `BUILD_TESTING=ON`:

```bash
cmake --build build-el-frozen-trace-gcc14 --target pe_interface_smoke_serial
ctest --test-dir build-el-frozen-trace-gcc14 -R pe-interface-serial --output-on-failure
```

Future parallel interface coverage should live next to this runner as a separate
`pe_interface_smoke_parallel` target with fixtures under `fixtures/parallel` and
CTest names using the `pe-interface-parallel-*` prefix.
