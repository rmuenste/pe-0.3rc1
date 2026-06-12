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

From a standalone PE checkout, only PE's normal dependencies plus testing are
needed:

```bash
cmake -S . -B build-interface-tests \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=ON \
  -DPE_USE_JSON=ON \
  -DPE_USE_EIGEN=ON

cmake --build build-interface-tests --target pe_interface_smoke_serial
ctest --test-dir build-interface-tests -R pe-interface-serial --output-on-failure
```

When PE is configured with `-DPE_USE_CGAL=ON`, the serial test set also includes
`pe-interface-serial-atc-cgal-distancemap` and
`pe-interface-serial-span-complex-distancemaps`. The ATC case enables the ATC
domain boundary DistanceMap and verifies that `DistanceMapWriter` writes the
configured VTI file. The SpanComplex case calls `setupSpanComplexSerial`, builds
DistanceMaps for the tool, chip, and workpiece OBJ fixtures, and verifies the
three generated maps. These CGAL cases are intentionally absent from CTest when
`PE_USE_CGAL=OFF`.

```bash
cmake -S . -B build-interface-tests-cgal \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=ON \
  -DPE_USE_JSON=ON \
  -DPE_USE_EIGEN=ON \
  -DPE_USE_CGAL=ON

cmake --build build-interface-tests-cgal --target pe_interface_smoke_serial
ctest --test-dir build-interface-tests-cgal \
  -R pe-interface-serial-atc-cgal-distancemap \
  --output-on-failure
```

The generated VTI file is written in the case-local run directory, for example:

```text
build-interface-tests-cgal/tests/interface/serial-runs/atc-cgal-distancemap/atc_boundary_distancemap.vti
```

## Adding Tests

Add new interface coverage as a new CTest case, not as another setup call inside
an existing runner invocation. PE setup functions mutate process-global
singletons, so each scenario needs its own process.

1. Add or reuse fixture files under `fixtures/serial`.
2. Add a `pe_add_interface_serial_test(<ctest-name>, <case-name>)` entry in
   `tests/interface/CMakeLists.txt`. Wrap it in the relevant feature gate, such
   as `if(PE_USE_CGAL)`, when the case requires an optional dependency.
3. Update `pe_interface_smoke_serial.cpp` so `main()` maps `<case-name>` to the
   fixture config copied to `example.json`.
4. Implement a focused helper that calls exactly one setup function and checks
   the expected bodies, config flags, files, or roundtrip behavior.
5. Dispatch `<case-name>` to that helper in `main()`.

Future parallel interface coverage should live next to this runner as a separate
`pe_interface_smoke_parallel` target with fixtures under `fixtures/parallel` and
CTest names using the `pe-interface-parallel-*` prefix.
