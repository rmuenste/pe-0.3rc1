#!/usr/bin/env python3
"""Experimental per-function metric inventory from Clang JSON AST dumps."""
import argparse
import bisect
import json
import pathlib
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
import threading
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


# Clang pretty-prints AST JSON with a fixed two-space indent, so the declarations
# directly under TranslationUnitDecl can be split without parsing the whole dump.
TU_INNER_START = b'\n  "inner": [\n'
TOP_LEVEL_SEPARATOR = b"\n    },\n"
TOP_LEVEL_END = b"\n    }\n  ]\n}"
# A location's own file always follows its offset. This excludes "includedFrom".
LOCATION_FILE = re.compile(rb'"offset": \d+,\s+"file": "((?:[^"\\]|\\.)*)"')
RESOLVED_FILE = "_resolved_file"


class AstDeclTooLarge(Exception):
    """A declaration that must be parsed in memory exceeds the configured limit."""

    def __init__(self, size):
        super().__init__(size)
        self.size = size


def decision_count(node):
    """Count decisions inside a function body, but not inside nested lambdas."""
    count = 0
    stack = [node]
    while stack:
        current = stack.pop()
        if not isinstance(current, dict) or current.get("kind") == "LambdaExpr":
            continue
        kind = current.get("kind")
        count += int(kind in DECISIONS)
        if kind == "BinaryOperator" and current.get("opcode") in ("&&", "||"):
            count += 1
        stack.extend(current.get("inner", []))
    return count


def annotate_locations(root, current_file=None):
    """Record the file of every source location and return the last file seen.

    Clang emits a location's "file" only when it differs from the previously printed
    location, so its absence does not imply the main file. Python's JSON parser keeps
    key order, which allows the dump to be replayed in the order Clang wrote it.
    """
    stack = [root]
    while stack:
        value = stack.pop()
        if isinstance(value, list):
            stack.extend(reversed(value))
        elif isinstance(value, dict):
            if "offset" in value or "line" in value or "file" in value:
                current_file = value.get("file", current_file)
                value[RESOLVED_FILE] = current_file
            stack.extend(reversed([child for key, child in value.items()
                                   if key != "includedFrom"]))
    return current_file


def span(node, newline_offsets=None):
    begin = node.get("range", {}).get("begin", {})
    end = node.get("range", {}).get("end", {})
    start = begin.get("expansionLoc", begin).get("line")
    stop = end.get("expansionLoc", end).get("line")
    if newline_offsets is not None:
        if not isinstance(start, int) and isinstance(begin.get("offset"), int):
            start = bisect.bisect_left(newline_offsets, begin["offset"]) + 1
        if not isinstance(stop, int) and isinstance(end.get("offset"), int):
            stop = bisect.bisect_left(newline_offsets, end["offset"]) + 1
    if not isinstance(start, int) or not isinstance(stop, int) or stop < start:
        return None
    return start, stop


def location_file(node):
    """Return the file a declaration starts in, after annotate_locations()."""
    begin = node.get("range", {}).get("begin", {})
    begin = begin.get("expansionLoc", begin)
    if RESOLVED_FILE in begin:
        return begin[RESOLVED_FILE]
    loc = node.get("loc", {})
    return loc.get("expansionLoc", loc).get(RESOLVED_FILE)


class MainFileMatcher:
    """Compare Clang's file spellings with the main file, relative to the compile directory."""

    def __init__(self, source, base=None):
        self.source = pathlib.Path(source).resolve()
        self.base = pathlib.Path(base) if base is not None else pathlib.Path.cwd()
        self.cache = {}

    def __call__(self, path):
        if path is None:
            return False
        if path not in self.cache:
            self.cache[path] = (self.base / path).resolve() == self.source
        return self.cache[path]


def collect_functions(ast, is_main_file, newline_offsets, results):
    """Append definitions written in the main file; return definitions without a usable span."""
    skipped = 0
    stack = [ast]
    while stack:
        node = stack.pop()
        if not isinstance(node, dict):
            continue
        children = node.get("inner", [])
        if node.get("kind") in FUNCTION_KINDS and is_main_file(location_file(node)):
            if node.get("isImplicit"):
                continue
            body = next((child for child in children
                         if child.get("kind") in ("CompoundStmt", "CXXTryStmt")), None)
            if body is not None:
                loc = span(node, newline_offsets)
                if loc is None:
                    skipped += 1
                    continue
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
                continue
        stack.extend(reversed(children))
    return skipped


def unique_functions(results):
    # A template's pattern precedes its instantiations, and is the code as written.
    unique = {}
    for item in results:
        key = (item["name"], item["kind"], item["line_start"], item["line_end"])
        unique.setdefault(key, item)
    return sorted(unique.values(), key=lambda i: (i["line_start"], i["name"]))


def newline_offsets_of(source):
    data = pathlib.Path(source).read_bytes()
    return [i for i, value in enumerate(data) if value == 10]


def extract_functions(ast, source, base=None):
    """Extract main-file function metrics from a fully parsed AST."""
    is_main_file = MainFileMatcher(source, base)
    annotate_locations(ast)
    results = []
    collect_functions(ast, is_main_file, newline_offsets_of(is_main_file.source), results)
    return unique_functions(results)


def iter_top_level_decls(stream, read_size=1 << 20):
    """Yield the JSON text of each declaration directly under TranslationUnitDecl."""
    buffer = bytearray()
    while TU_INNER_START not in buffer:
        block = stream.read(read_size) if len(buffer) < 1 << 16 else b""
        if not block:
            raise ValueError("unrecognized AST JSON layout: no top-level declaration list")
        buffer += block
    start = buffer.index(TU_INNER_START)
    if b'"kind": "TranslationUnitDecl"' not in buffer[:start]:
        raise ValueError("unrecognized AST JSON layout: no TranslationUnitDecl")
    del buffer[:start + len(TU_INNER_START)]
    search_from = 0
    while True:
        position = buffer.find(TOP_LEVEL_SEPARATOR, search_from)
        if position >= 0:
            end = position + len(TOP_LEVEL_SEPARATOR) - 2  # keep the closing brace
            yield buffer[:end]
            del buffer[:position + len(TOP_LEVEL_SEPARATOR)]
            search_from = 0
            continue
        block = stream.read(read_size)
        if not block:
            break
        search_from = max(0, len(buffer) - len(TOP_LEVEL_SEPARATOR) + 1)
        buffer += block
    tail = bytes(buffer).rstrip()
    if not tail.endswith(TOP_LEVEL_END):
        raise ValueError("truncated AST JSON: declaration list is not closed")
    yield tail[:-len(TOP_LEVEL_END)] + b"\n    }"


def parse_ast_stream(stream, source, base=None, max_decl_bytes=100 * 1024 * 1024):
    """Extract main-file functions from AST JSON, parsing only declarations that need it.

    A real PE translation unit dumps several GB, nearly all from headers. Declarations
    with no main-file location are only scanned to keep track of the current file.
    """
    is_main_file = MainFileMatcher(source, base)
    newline_offsets = newline_offsets_of(is_main_file.source)
    current_file = None
    results = []
    stats = {"ast_bytes": 0, "decls_total": 0, "decls_parsed": 0,
             "largest_parsed_decl_bytes": 0, "functions_without_span": 0}
    for decl in iter_top_level_decls(stream):
        stats["ast_bytes"] += len(decl)
        stats["decls_total"] += 1
        files = [json.loads(b'"' + match.group(1) + b'"')
                 for match in LOCATION_FILE.finditer(decl)]
        # Clang always prints the file when entering the main file, so which header
        # was current does not matter here.
        if not is_main_file(current_file) and not any(map(is_main_file, files)):
            current_file = None
            continue
        if len(decl) > max_decl_bytes:
            raise AstDeclTooLarge(len(decl))
        stats["decls_parsed"] += 1
        stats["largest_parsed_decl_bytes"] = max(stats["largest_parsed_decl_bytes"], len(decl))
        node = json.loads(decl)
        current_file = annotate_locations(node, current_file)
        stats["functions_without_span"] += collect_functions(
            node, is_main_file, newline_offsets, results)
    return unique_functions(results), stats


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
    outcome, failure = None, None
    with tempfile.TemporaryFile() as stderr:
        proc = subprocess.Popen(command, cwd=entry["directory"], stdout=subprocess.PIPE,
                                stderr=stderr)
        timed_out = threading.Event()
        def expire():
            timed_out.set()
            proc.kill()
        timer = threading.Timer(timeout, expire)
        timer.start()
        try:
            outcome = parse_ast_stream(proc.stdout, source, entry["directory"], max_bytes)
        except AstDeclTooLarge as exc:
            failure = {"status": "ast_too_large", "ast_bytes": exc.size,
                       "error": f"Main-file declaration exceeds {max_bytes} byte limit"}
        except (ValueError, UnicodeDecodeError, RecursionError) as exc:
            failure = {"status": "json_error", "error": str(exc)}
        finally:
            timer.cancel()
            if failure is not None:
                proc.kill()  # the rest of the dump is no longer wanted
            proc.stdout.close()
            returncode = proc.wait()
        stderr.seek(0)
        diagnostic = stderr.read(12000).decode("utf-8", errors="replace")
    if timed_out.is_set():
        return {"file": relative, "status": "timeout", "functions": [],
                "error": f"Clang or AST parsing exceeded {timeout}s timeout"}
    if failure is not None:
        return {"file": relative, "functions": [], **failure}
    if returncode != 0:
        return {"file": relative, "status": "clang_error", "functions": [],
                "exit_code": returncode, "error": diagnostic}
    functions, stats = outcome
    return {"file": relative, "status": "ok", "functions": functions,
            "function_count": len(functions), **stats}


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
    parser.add_argument("--max-ast-mb", type=int, default=100,
                        help="largest single main-file top-level declaration to parse in "
                             "memory; the total AST size is not limited")
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
