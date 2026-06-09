# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

> **Single source of truth.** All substantive guidance — build system, dependencies,
> architecture, repo structure, examples, and design deep-dives — lives in `AGENTS.md`
> and the technical notes it links under `doc/technical-notes/`. `AGENTS.md` is imported
> below so Claude Code shares exactly the same guidance as other agents and the two
> cannot drift. **Update `AGENTS.md` / the technical notes, not this file.** Keep
> CLAUDE.md limited to Claude-Code-specific behavior (if any).

@AGENTS.md

## Finding things

- Start at `doc/technical-notes/README.md` — the index that routes to every deep-dive
  (build, architecture, collision detection, solvers, lubrication, MPI, DistanceMap,
  and the PE serial-mode / CFD coupling interface).
- API quick references also live there, e.g. `vec3-mat3-documentation.md` (`Vec3`/`Mat3`)
  and `rigid-body-documentation.md` (`RigidBody`, including the `rotate*` methods).

## Claude Code specifics

Nothing Claude-Code-only at the moment. Add any Claude-Code-specific instructions here;
keep shared, tool-agnostic guidance in `AGENTS.md`.
