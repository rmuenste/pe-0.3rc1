# PE code analysis — experimental v0.1

**Read-only baseline tooling** for PE. Does not change library code, impose CI gates,
or reformat legacy files. Keep reports under `build/` (ignored by Git).

## Requirements

- CMake >= 3.21 and Ninja for the existing presets
- A configured PE build with its normal C++ dependencies
- Clang / clang-tidy installed (prefer a matching major version)
- Python >= 3.9
- Optional: Clang C++ compiler for AST-based function metrics (required for metrics)

PE's existing `CMakePresets.json` already exports `compile_commands.json`.
No production CMake changes are needed.

## Configuration

From the repository root:

```sh
cmake --preset serial
cmake --build --preset build-serial -j2
```

This generates `build/presets/serial/compile_commands.json`. If you have a
working compiler/dependency environment but cannot use the preset, configure
an equivalent standalone build with `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`.
For clang AST analysis, **configure the build with clang++ as C++ compiler**
(e.g. on a fresh build directory with `-DCMAKE_CXX_COMPILER=clang++`).
Reusing a GCC compilation database with clang may fail on GCC-only flags.

## clang-tidy diagnostics

```sh
python3 tools/code-analysis/run_tidy.py \
  --build build/presets/serial \
  --output build/code-analysis/tidy-serial.json
```

By default, only compiled translation units under `src/` are analyzed.
Use `--prefix src/interface/` for a smaller pilot. The runner stores raw
stdout/stderr with each file's exit code. A nonzero clang-tidy exit code may
also be caused by a compile/configuration error and must not be interpreted
automatically as a code defect. clang-tidy warnings alone do not generally
produce a nonzero exit code without warnings-as-errors. Summaries therefore
must not imply that zero failed files means zero warnings.

## Experimental AST metrics

```sh
python3 tools/code-analysis/metrics.py \
  --build build/presets/serial \
  --prefix src/interface/ \
  --limit 2 \
  --output build/code-analysis/metrics-pilot.json
```

This uses `clang++ -Xclang -ast-dump=json -fsyntax-only` with the actual
compilation command, counts control-flow decision nodes, and records source
spans. The initial algorithm is intentionally simple:

`cyclomatic_approx = 1 + if + loops + (case/default excluding default) +
catch + ternary + logical &&/||`

It is **not** a formal control-flow graph, and does not compute Sonar's
cognitive complexity. In particular, templates, conditional compilation,
macros, and exception handling require further validation. The output field
is deliberately called `cyclomatic_approx`. The pilot should be checked
against manually evaluated functions before use in a repository-wide report.

AST JSON can be large for template-heavy PE files. Start with `--limit 1`
or `--limit 2`; the runner enforces timeout and maximum AST output size,
and records errors instead of silently omitting files. It stores full AST
temporarily, not in Git. Default scope excludes header-only definitions that
are not explicitly identified as originating in the TU main file.

## Future work

- Confirm parsing on serial, MPI, and CGAL builds separately
- Replace AST heuristic with a dedicated Clang LibTooling CFG visitor
- Add full cognitive complexity, coupling and dependency graphs
- Generate Git revision comparisons and visualize only after validating metrics
- Add documentation of clang-tidy baseline warnings and triage rules
