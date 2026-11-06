#!/usr/bin/env python3
"""Experimental per-function metric inventory from Clang JSON AST dumps."""
import argparse
import json
import pathlib
import shlex
import shutil
import subprocess
import sys
import tempfile
from datetime import datetime, timezone

from run_tidy import load_sources

FUNCTION_KINDS = {
    "FunctionDecl", "CXXMethodDecl", "CXXConstructorDecl",
    "CXXDestructorDecl", "CXXConversionDecl",
}
DECISIONS = {
    "IfStmt", "ForStmt", "CXXForRangeStmt", "WhileStmt", "DoStmt",
    "CaseStmt", "CXXCatchStmt", "ConditionalOperator", "BinaryConditionalOperator",
}


def decision_count(node):
    """Count decisions inside a function body, but not inside nested lambdas."""
    if not isinstance(node, dict) or node.get("kind") == "LambdaExpr":
        return 0
    kind = node.get("kind")
    count = int(kind in DECISIONS)
    if kind == "BinaryOperator" and node.get("opcode") in ("&&", "||"):
        count += 1
    for child in node.get("inner", []):
        count += decision_count(child)
    return count


def span(node):
    begin = node.get("range", {}).get("begin", {})
    end = node.get("range", {}).get("end", {})
    start = begin.get("expansionLoc", begin).get("line")
    stop = end.get("expansionLoc", end).get("line")
    if not isinstance(start, int) or not isinstance(stop, int) or stop < start:
        return None
    return start, stop


def same_file(node, source):
    # Clang often omits main-file names; explicit header paths should be rejected.
    loc = node.get("loc", {})
    begin = node.get("range", {}).get("begin", {})
    path = loc.get("file") or begin.get("file")
    if path:
        return pathlib.Path(path).resolve() == source
    return True


def extract_functions(ast, source):
    results = []
    source = pathlib.Path(source).resolve()
    def visit(node):
        if not isinstance(node, dict):
            return
        if node.get("kind") in FUNCTION_KINDS and same_file(node, source):
            body = next((child for child in node.get("inner", [])
                         if child.get("kind") in ("CompoundStmt", "CXXTryStmt")), None)
            loc = span(node)
            if body is not None and loc is not None:
                begin, end = loc
                decisions = decision_count(body)
                results.append({
                    "name": node.get("name", "<anonymous>"),
                    "kind": node.get("kind"),
                    "line_start": begin, "line_end": end,
                    "lines": end - begin + 1,
                    "decision_nodes": decisions,
                    "cyclomatic_approx": decisions + 1,
                })
                # Avoid counting recursive declaration descendants a second time.
                return
        for child in node.get("inner", []):
            visit(child)
    visit(ast)
    # ASTs can contain multiple instantiations of the same source definition.
    unique = {(item["name"], item["kind"], item["line_start"], item["line_end"]): item
              for item in results}
    return sorted(unique.values(), key=lambda i: (i["line_start"], i["name"]))


def clang_command(entry, compiler):
    if "arguments" in entry:
        args = list(entry["arguments"])
    else:
        args = shlex.split(entry["command"])
    if not args:
        raise ValueError("empty compilation command")
    output = [compiler, "-Xclang", "-ast-dump=json", "-fsyntax-only"]
    skip_next = {"-o", "-MF", "-MT", "-MQ", "-MJ"}
    i = 1
    while i < len(args):
        arg = args[i]
        if arg in skip_next:
            i += 2
            continue
        if arg in ("-c", "-MD", "-MMD", "-MP"):
            i += 1
            continue
        if arg.startswith(("-o", "-MF", "-MT", "-MQ", "-MJ")) and arg not in skip_next:
            i += 1
            continue
        output.append(arg)
        i += 1
    return output


def analyze_one(entry, relative, source, compiler, timeout, max_bytes):
    command = clang_command(entry, compiler)
    with tempfile.TemporaryFile() as stdout, tempfile.TemporaryFile() as stderr:
        try:
            proc = subprocess.run(command, cwd=entry["directory"], stdout=stdout,
                                  stderr=stderr, timeout=timeout, check=False)
        except subprocess.TimeoutExpired:
            return {"file": relative, "status": "timeout", "functions": [],
                    "error": f"Clang exceeded {timeout}s timeout"}
        size = stdout.tell()
        stderr.seek(0)
        diagnostic = stderr.read(12000).decode("utf-8", errors="replace")
        if proc.returncode != 0:
            return {"file": relative, "status": "clang_error", "functions": [],
                    "exit_code": proc.returncode, "error": diagnostic}
        if size > max_bytes:
            return {"file": relative, "status": "ast_too_large", "functions": [],
                    "ast_bytes": size, "error": f"AST JSON exceeds {max_bytes} byte limit"}
        try:
            stdout.seek(0)
            ast = json.load(stdout)
        except (ValueError, UnicodeDecodeError) as exc:
            return {"file": relative, "status": "json_error", "functions": [],
                    "error": str(exc)}
    functions = extract_functions(ast, source)
    return {"file": relative, "status": "ok", "functions": functions,
            "function_count": len(functions), "ast_bytes": size}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=pathlib.Path,
                        default=pathlib.Path(__file__).resolve().parents[2])
    parser.add_argument("--build", type=pathlib.Path, required=True)
    parser.add_argument("--output", type=pathlib.Path,
                        default=pathlib.Path("build/code-analysis/metrics.json"))
    parser.add_argument("--clang", default="clang++")
    parser.add_argument("--prefix", action="append", default=None)
    parser.add_argument("--limit", type=int, default=2,
                        help="max TUs to analyze (default 2); use 0 for all")
    parser.add_argument("--timeout", type=int, default=120)
    parser.add_argument("--max-ast-mb", type=int, default=100)
    args = parser.parse_args()
    if args.limit < 0 or args.timeout < 1 or args.max_ast_mb < 1:
        parser.error("invalid limit/timeout/max-ast-mb")
    root, build = args.root.resolve(), args.build.resolve()
    database_path = build / "compile_commands.json"
    if not database_path.is_file():
        parser.error("missing compilation database: " + str(database_path))
    compiler = shutil.which(args.clang)
    if compiler is None:
        parser.error("Clang executable not found: " + args.clang)
    database = json.loads(database_path.read_text(encoding="utf-8"))
    eligible = load_sources(database_path, root, args.prefix or ["src/"])
    if args.limit:
        eligible = eligible[:args.limit]
    # Choose the first matching CMake entry; a TU can occur under multiple targets.
    by_source = {}
    for entry in database:
        source = (pathlib.Path(entry["directory"]) / entry["file"]).resolve()
        by_source.setdefault(source, entry)
    results = []
    for relative, source in eligible:
        try:
            result = analyze_one(by_source[source], relative, source, compiler,
                                 args.timeout, args.max_ast_mb * 1024 * 1024)
        except (OSError, ValueError) as exc:
            result = {"file": relative, "status": "error", "functions": [],
                      "error": str(exc)}
        results.append(result)
        print(f"{relative}: {result['status']} ({len(result['functions'])} functions)",
              file=sys.stderr)
    report = {"schema_version": 1, "metric": "cyclomatic_approx",
              "generated_utc": datetime.now(timezone.utc).isoformat(),
              "root": str(root), "build": str(build),
              "clang": compiler, "files_total": len(results),
              "files_failed": sum(item["status"] != "ok" for item in results),
              "files": results}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Saved {args.output}; {report['files_failed']} failures")
    return 1 if report["files_failed"] else 0


if __name__ == "__main__":
    sys.exit(main())
