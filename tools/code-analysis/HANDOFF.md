# Handoff: local-agent verification of PE code analysis v0.1

Repository: `rmuenste/pe-0.3rc1`
Branch: `tooling/clang-analysis-v0.1`
Status: experimental, read-only; **do not merge before local validation**.

## Scope delivered

- `run_tidy.py`: compile-database-driven clang-tidy wrapper; per-TU raw diagnostics and exit statuses.
- `.clang-tidy`: conservative bug-prone, analyzer, performance check selection.
- `metrics.py`: first-pass Clang AST function inventory, size and decision-count approximation.
- `README.md`: usage instructions and limitations.
- No changes to PE production C++ / CMake preset defaults.

## Validate locally

1. Check out the branch and inspect diff against master.
2. Confirm tool versions: `cmake --version`, `clang++ --version`,
   `clang-tidy --version`, `python3 --version`.
3. Configure a **fresh Clang** standalone build with PE's normal dependencies,
   preferably matching serial preset options and with compile commands exported.
   Do not overwrite a working GCC build.
4. Confirm `compile_commands.json` contains PE `src/` translation units.
5. Run `python3 -m py_compile tools/code-analysis/*.py`.
6. Run `python3 -m unittest discover -s tools/code-analysis/tests -v`.
7. Run clang-tidy pilot on `src/interface/`, inspect at least 10 findings.
   Distinguish build failures, third-party warnings, false positives, actionable
   PE findings; save output under `build/code-analysis/`.
8. Run metrics AST pilot with `--limit 2`. Inspect generated JSON and manually
   verify at least three functions, including nested `if` and a loop.
9. Expand to selected `src/core/` files, then broader coverage only after
   confirming memory/time limits. Report failures and omitted template/header code.
10. Try MPI configuration if MPI is available, separately from serial, and
    report configuration-specific differences.

## Acceptance checks

- No C++ runtime or build behavior changes.
- Analysis does not edit C++ files.
- Exit status / parser failures visible in reports, not hidden.
- Report identifies Clang/compiler versions and Git SHA in the accompanying notes.
- Metrics are described as approximations, not certified complexity scores.
- Output stays under ignored build directory.
- Provide short summary of warnings by rule, meaningful findings, and recommended
  follow-up fixes, but do not mass-apply clang-tidy fixes.

## Known limits

- Full PE compilation and clang-tidy could not be verified in the ChatGPT
  GitHub editing environment, so this is NOT a validated baseline yet.
- Optional dependency configurations change analyzed ASTs.
- C++ macro/template expansion can double-count or omit definitions.
- clang-tidy nonzero exit codes conflate tool failures and warnings-as-errors.
- Raw clang-tidy diagnostic text is retained, rather than parsed into a normalized
  warnings schema, because emitted format varies with Clang versions.
- The AST visitor is a prototype, not a full Clang CFG implementation.

## Request to local agent

Please validate the scripts, fix portability or parser problems in this branch,
and submit a concise report with exact commands, versions, failure counts,
confirmed metrics examples, and any recommended changes before merge.
