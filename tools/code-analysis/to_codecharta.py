#!/usr/bin/env python3
"""Convert metrics.py output to a CSV for CodeCharta's `ccsh csvimport`.

Every function becomes one leaf, at the synthetic path
`<file>/<function>@<line_start>`, so CodeCharta shows files as folders of functions.
The line suffix keeps overloads apart. A `/` in a function name (e.g. `operator/`)
would split the path, so it is replaced by `∕` (U+2215).

Files whose analysis failed, and files with no measured functions, have no leaves and
would vanish from the map; they are listed on stderr instead.
"""

import argparse
import csv
import json
import pathlib
import sys

COLUMNS = ["path", "cyclomatic_approx", "lines", "decision_nodes", "functions"]
SLASH_SUBSTITUTE = "\u2215"


def function_path(file, function):
    name = function["name"].replace("/", SLASH_SUBSTITUTE)
    return f"{file}/{name}@{function['line_start']}"


def rows(report, prefixes=None):
    """Return (CSV rows, {"failed": [...], "empty": [...]}) for the selected files."""
    skipped = {"failed": [], "empty": []}
    out = []
    for entry in report["files"]:
        file = entry["file"]
        if prefixes and not any(file.startswith(p) for p in prefixes):
            continue
        if entry["status"] != "ok":
            skipped["failed"].append(f"{file} ({entry['status']})")
            continue
        if not entry["functions"]:
            skipped["empty"].append(file)
            continue
        for function in entry["functions"]:
            out.append({
                "path": function_path(file, function),
                "cyclomatic_approx": function["cyclomatic_approx"],
                "lines": function["lines"],
                "decision_nodes": function["decision_nodes"],
                "functions": 1,
            })
    return out, skipped


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--input", type=pathlib.Path, required=True,
                        help="JSON report written by metrics.py")
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--prefix", action="append", default=None,
                        help="only include files starting with this path (repeatable)")
    args = parser.parse_args(argv)

    report = json.loads(args.input.read_text())
    out, skipped = rows(report, args.prefix)
    paths = [row["path"] for row in out]
    if len(set(paths)) != len(paths):
        print("error: duplicate function paths", file=sys.stderr)
        return 1

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=COLUMNS)
        writer.writeheader()
        writer.writerows(out)

    print(f"{len(out)} functions written to {args.output}", file=sys.stderr)
    for file in skipped["failed"]:
        print(f"not shown, analysis failed: {file}", file=sys.stderr)
    for file in skipped["empty"]:
        print(f"not shown, no functions: {file}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
