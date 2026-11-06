#!/usr/bin/env python3
"""Run clang-tidy on PE's compilation database without modifying source code."""
import argparse
import concurrent.futures
import json
import pathlib
import re
import shutil
import subprocess
import sys
from datetime import datetime, timezone


def load_sources(database, root, prefixes):
    entries = json.loads(pathlib.Path(database).read_text(encoding="utf-8"))
    found = {}
    for item in entries:
        source = (pathlib.Path(item["directory"]) / item["file"]).resolve()
        try:
            relative = source.relative_to(root).as_posix()
        except ValueError:
            continue
        if source.suffix not in (".c", ".cc", ".cpp", ".cxx"):
            continue
        if not any(relative.startswith(prefix) for prefix in prefixes):
            continue
        if source.is_file():
            found[relative] = source
    return sorted(found.items())


def run_one(item, tidy, build, config, timeout):
    relative, source = item
    cmd = [tidy, "-p", str(build), "--config-file=" + str(config),
           "--quiet", str(source)]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, errors="replace",
                              timeout=timeout, check=False)
        return {"file": relative, "exit_code": proc.returncode,
                "stdout": proc.stdout, "stderr": proc.stderr, "timed_out": False}
    except subprocess.TimeoutExpired as exc:
        return {"file": relative, "exit_code": None,
                "stdout": (exc.stdout or b"").decode(errors="replace") if isinstance(exc.stdout, bytes) else (exc.stdout or ""),
                "stderr": (exc.stderr or b"").decode(errors="replace") if isinstance(exc.stderr, bytes) else (exc.stderr or ""),
                "timed_out": True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=pathlib.Path, default=pathlib.Path(__file__).resolve().parents[2])
    parser.add_argument("--build", type=pathlib.Path, required=True, help="configured CMake build directory")
    parser.add_argument("--output", type=pathlib.Path, default=pathlib.Path("build/code-analysis/tidy.json"))
    parser.add_argument("--clang-tidy", default="clang-tidy")
    parser.add_argument("--config", type=pathlib.Path, default=pathlib.Path(__file__).with_name(".clang-tidy"))
    parser.add_argument("--prefix", action="append", default=None,
                        help="source prefix relative to root (repeatable; default src/)")
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--timeout", type=int, default=180, help="per-file timeout in seconds")
    args = parser.parse_args()
    root, build = args.root.resolve(), args.build.resolve()
    database = build / "compile_commands.json"
    if not database.is_file():
        parser.error("missing compilation database: " + str(database))
    tidy = shutil.which(args.clang_tidy)
    if tidy is None:
        parser.error("clang-tidy executable not found: " + args.clang_tidy)
    config = args.config.resolve()
    if not config.is_file():
        parser.error("missing .clang-tidy: " + str(config))
    if args.jobs < 1 or args.timeout < 1:
        parser.error("--jobs and --timeout must be positive")
    sources = load_sources(database, root, args.prefix or ["src/"])
    if not sources:
        parser.error("no eligible translation units in compilation database")
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        results = list(pool.map(lambda item: run_one(item, tidy, build, config, args.timeout), sources))
    # Keep raw diagnostics: clang-tidy text output is not a stable machine interface.
    result = {"schema_version": 1, "tool": "clang-tidy",
              "generated_utc": datetime.now(timezone.utc).isoformat(),
              "root": str(root), "build": str(build), "config": str(config),
              "files_total": len(results),
              "files_failed": sum(r["exit_code"] != 0 for r in results),
              "files_timed_out": sum(r["timed_out"] for r in results),
              "results": results}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(f"clang-tidy: {result['files_total']} files; {result['files_failed']} nonzero/timeouts; {args.output}")
    return 0 if result["files_failed"] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
