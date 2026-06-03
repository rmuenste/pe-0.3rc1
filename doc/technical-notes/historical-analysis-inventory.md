# Historical and Analysis Note Inventory

This inventory tracks technical notes that may still contain useful analysis but
should not be treated as canonical current-state documentation without review.

## Second-Pass Review Candidates

| File | Why it needs review |
|------|---------------------|
| `ffd-solver.md` | Focused analysis of FFD internals; likely useful, but should be checked against current FFD configuration names before becoming canonical. |
| `short-range-rep-forces.md` | Narrow experimental/numerical analysis; keep only if this experiment remains relevant. |

## Review Policy

- Keep these files in place until a second cleanup pass decides whether to
  polish, archive, or delete each one.
- If retained as historical notes, add a short header that names the analyzed
  solver, experiment, or date range.
- Prefer stable file/function references over source line numbers during any
  cleanup.
