# PE code analysis — experimental v0.1

**Read-only baseline tooling** for PE. Does not change library code, impose CI gates,
or reformat legacy files. Keep reports under `build/` (ignored by Git).

## Requirements

- CMake >= 3.21 and Ninja for the existing presets
- A configured PE build with its normal C++ dependencies
- Clang / clang-tidy installed (prefer a matching major version)
- Python >= 3.9
- Optional: Clang C++ compiler for AST-based function metrics (required for metrics)

PE's `CMakePresets.json` exports `compile_commands.json` in every preset.

## Configuration

From the repository root:

```sh
cmake --preset clang-serial
cmake --build --preset build-clang-serial -j4
```

This keeps the analysis build apart from any GCC build, and generates
`build/presets/clang-serial/compile_commands.json`. Configuring is enough for the
analysis itself; building additionally confirms that PE compiles with Clang.
Reusing a GCC compilation database with clang may fail on GCC-only flags.

The `clang-mpi` preset does the same for MPI, in `build/presets/clang-mpi`. Analyze
it separately: its results differ from serial. It cannot fully build with Clang yet,
because `pe/interface/setup/creep.h` and `part_bench.h` pass a `shared_ptr` through a
variadic function; `cfd_entry_points.cpp` is reported as failed until that is fixed.

## clang-tidy diagnostics

```sh
python3 tools/code-analysis/run_tidy.py \
  --build build/presets/clang-serial \
  --output build/code-analysis/tidy-serial.json
```

By default, only compiled translation units under `src/` are analyzed.
Use `--prefix src/interface/` for a smaller pilot. The runner stores raw
stdout/stderr with each file's exit code. A nonzero clang-tidy exit code may
also be caused by a compile/configuration error and must not be interpreted
automatically as a code defect. clang-tidy warnings alone do not generally
produce a nonzero exit code without warnings-as-errors. Summaries therefore
must not imply that zero failed files means zero warnings. A timed-out file has
`timed_out: true` and no diagnostics at all, because clang-tidy prints at exit.

PE compiles most sources for three library variants that differ in
`pe_CONSTRAINT_SOLVER`. clang-tidy would analyze a file once per compile command,
so the runner analyzes only the first command (the `pe_static` variant) and records
the number of commands it skipped. `--all-variants` analyzes all of them; this takes
longer and reports the deduplicated union.

## Experimental AST metrics

```sh
python3 tools/code-analysis/metrics.py \
  --build build/presets/clang-serial \
  --prefix src/interface/ \
  --limit 2 \
  --output build/code-analysis/metrics-pilot.json
```

This uses `clang++ -Xclang -ast-dump=json -fsyntax-only` with the actual
compilation command, counts control-flow decision nodes, and records source
spans. `--limit` defaults to 2 translation units; `--limit 0` analyzes all of them.
For a full run over `src/`, omit `--prefix`:

```sh
python3 tools/code-analysis/metrics.py \
  --build build/presets/clang-serial \
  --limit 0 \
  --output build/code-analysis/metrics-full-serial.json
```

Only the configure step is needed. On a desktop machine the full serial run took about
21 minutes for 178 files, with a peak of about 4.6 GB of memory. `src/core/BodyReader.cpp`
contains a single top-level declaration of about 101 MB, close to the `--max-ast-mb`
default of 100; raise it (e.g. `--max-ast-mb 150`) if that file reports `ast_too_large`.

The initial algorithm is intentionally simple:

`cyclomatic_approx = 1 + if + loops + (case/default excluding default) +
catch + ternary + logical &&/||`

It is **not** a formal control-flow graph, and does not compute Sonar's
cognitive complexity. In particular, templates, conditional compilation,
macros, and exception handling require further validation. The output field
is deliberately called `cyclomatic_approx`. The pilot should be checked
against manually evaluated functions before use in a repository-wide report.

AST JSON is large: a typical PE translation unit dumps about 3.5 GB, nearly all of
it from headers. The runner reads the dump as a stream and parses only top-level
declarations that contain a main-file location, so it needs no temporary AST file and
far less memory than the dump size. `--max-ast-mb` limits one such declaration, not
the dump; exceeding it, the timeout, a Clang error or an unexpected JSON layout
is recorded per file instead of silently omitting the file. The splitter relies on
Clang's two-space pretty-printing and reports `json_error` if that layout changes.

**Scope.** Only definitions that start in the TU's main file are reported, so functions
and templates defined in `pe/` headers are not covered by running over `src/`. Compiler-
generated (implicit) members are excluded. A template is reported once, as written.
Code disabled by the preprocessor is invisible: `frozen_field_trace.cpp` has one
function in a serial build, because the rest needs MPI and CGAL. Decisions from macro
expansions count towards the function using the macro, and lambda bodies are not
counted towards any enclosing function.

## Future work

- Confirm parsing on serial, MPI, and CGAL builds separately
- Replace AST heuristic with a dedicated Clang LibTooling CFG visitor
- Add full cognitive complexity, coupling and dependency graphs
- Generate Git revision comparisons and visualize only after validating metrics
- Add documentation of clang-tidy baseline warnings and triage rules
