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
# Template declarations list their instantiations after the code as written.
TEMPLATE_DECL = re.compile(
    rb'\n( *)"kind": "(ClassTemplateDecl|FunctionTemplateDecl|VarTemplateDecl)",\n')
INSTANTIATION_KINDS = {
    b"ClassTemplateDecl": {b"ClassTemplateSpecializationDecl"},
    b"VarTemplateDecl": {b"VarTemplateSpecializationDecl"},
}
TEMPLATED_FUNCTION_KINDS = {kind.encode() for kind in FUNCTION_KINDS} | {b"CXXDeductionGuideDecl"}


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


class FileScope:
    """Decide from Clang's spelling of a path whether a file's definitions are collected.

    The main file always is. With a root, so is every file under root/<prefix> for one
    of the header prefixes, unless it is in `exclude`: headers analyzed in an earlier
    translation unit, and the main files of other translation units. Paths are
    resolved relative to the compile directory; a collected file maps to its resolved
    Path, any other file to None.
    """

    def __init__(self, source, base=None, root=None, header_prefixes=(), exclude=()):
        self.source = pathlib.Path(source).resolve()
        self.base = pathlib.Path(base) if base is not None else pathlib.Path.cwd()
        self.root = pathlib.Path(root).resolve() if root is not None else None
        self.header_prefixes = tuple(header_prefixes)
        self.exclude = exclude
        self.cache = {}

    def __call__(self, path):
        if path is None:
            return None
        if path not in self.cache:
            self.cache[path] = self._classify((self.base / path).resolve())
        return self.cache[path]

    def _classify(self, resolved):
        if resolved == self.source:
            return resolved
        if self.root is None or not self.header_prefixes or resolved in self.exclude:
            return None
        try:
            relative = resolved.relative_to(self.root).as_posix()
        except ValueError:
            return None
        return resolved if relative.startswith(self.header_prefixes) else None


class NewlineOffsets(dict):
    """Newline byte offsets per file, to turn offset-only locations into line numbers."""

    def __missing__(self, path):
        data = pathlib.Path(path).read_bytes()
        self[path] = [i for i, value in enumerate(data) if value == 10]
        return self[path]


def collect_functions(ast, scope, newline_offsets, results):
    """Append (file, metrics) for definitions in collected files.

    Return the number of definitions without a usable span.
    """
    skipped = 0
    stack = [ast]
    while stack:
        node = stack.pop()
        if not isinstance(node, dict):
            continue
        children = node.get("inner", [])
        file = scope(location_file(node)) if node.get("kind") in FUNCTION_KINDS else None
        if file is not None:
            if node.get("isImplicit"):
                continue
            body = next((child for child in children
                         if child.get("kind") in ("CompoundStmt", "CXXTryStmt")), None)
            if body is not None:
                loc = span(node, newline_offsets[file])
                if loc is None:
                    skipped += 1
                    continue
                begin, end = loc
                decisions = decision_count(body)
                results.append((file, {
                    "name": node.get("name", "<anonymous>"),
                    "kind": node.get("kind"),
                    "line_start": begin, "line_end": end,
                    "lines": end - begin + 1,
                    "decision_nodes": decisions,
                    "cyclomatic_approx": decisions + 1,
                }))
                # Avoid counting recursive declaration descendants a second time.
                continue
        stack.extend(reversed(children))
    return skipped


def unique_functions(results):
    # Instantiations share the span of the template as written. They are pruned from
    # streamed dumps; otherwise the pattern comes first for templates in one file.
    unique = {}
    for item in results:
        key = (item["name"], item["kind"], item["line_start"], item["line_end"])
        unique.setdefault(key, item)
    return sorted(unique.values(), key=lambda i: (i["line_start"], i["name"]))


def extract_functions(ast, source, base=None):
    """Extract main-file function metrics from a fully parsed AST."""
    scope = FileScope(source, base)
    annotate_locations(ast)
    results = []
    collect_functions(ast, scope, NewlineOffsets(), results)
    return unique_functions([item for _, item in results])


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


def instantiation_ranges(decl):
    """Return sorted (start, stop) byte ranges of template instantiations in decl.

    Under a ClassTemplateDecl or VarTemplateDecl every specialization child is an
    implicit instantiation, or a reference to an explicit specialization that Clang
    prints in full where it is written. Under a FunctionTemplateDecl the same holds
    for every function after the first, which is the template as written. A child
    object of a template at indent I starts with a line of I + 4 spaces and "{", and
    ends at the next line of exactly I + 4 spaces and "}".
    """
    ranges = []
    for match in TEMPLATE_DECL.finditer(decl):
        position = match.start()
        # The ranges never overlap: a template inside a dropped range is skipped here.
        index = bisect.bisect_right(ranges, (position, len(decl) + 1)) - 1
        if index >= 0 and position < ranges[index][1]:
            continue  # a template inside an instantiation that is dropped anyway
        indent = len(match.group(1)) - 2
        end = decl.find(b"\n" + b" " * indent + b"}", match.end())
        end = len(decl) if end < 0 else end
        child_open = b"\n" + b" " * (indent + 4) + b"{\n"
        child_close = b"\n" + b" " * (indent + 4) + b"}"
        dropped_kinds = INSTANTIATION_KINDS.get(bytes(match.group(2)))
        seen_pattern = False
        cursor = match.end()
        while True:
            start = decl.find(child_open, cursor, end)
            if start < 0:
                break
            stop = decl.find(child_close, start + len(child_open), end)
            if stop < 0:
                raise ValueError("unrecognized AST JSON layout: unclosed template child")
            stop += len(child_close)
            kind_at = decl.find(b'"kind": "', start, stop) + len(b'"kind": "')
            kind = bytes(decl[kind_at:decl.find(b'"', kind_at)])
            if dropped_kinds is not None:
                drop = kind in dropped_kinds
            else:
                drop = kind in TEMPLATED_FUNCTION_KINDS and seen_pattern
                seen_pattern = seen_pattern or kind in TEMPLATED_FUNCTION_KINDS
            if drop:
                bisect.insort(ranges, (start + 1, stop))  # from the indent before "{" to "}"
            cursor = stop
    return ranges


def prune_instantiations(decl):
    """Replace template instantiations in a declaration's JSON text by placeholders.

    The rest of the text stays byte-identical. A placeholder keeps the last file a
    dropped instantiation printed, because Clang omits "file" in following locations
    that are in that same file.
    """
    ranges = instantiation_ranges(decl)
    if not ranges:
        return decl
    parts, cursor = [], 0
    for start, stop in ranges:
        parts.append(decl[cursor:start])
        last = None
        for last in LOCATION_FILE.finditer(decl, start, stop):
            pass
        parts.append(b'{"file": "' + last.group(1) + b'"}' if last else b"{}")
        cursor = stop
    parts.append(decl[cursor:])
    return b"".join(parts)


def parse_ast_stream(stream, source, base=None, max_decl_bytes=100 * 1024 * 1024,
                     root=None, header_prefixes=(), exclude=frozenset(),
                     newline_offsets=None):
    """Extract function metrics from AST JSON, parsing only declarations that need it.

    Return (main-file functions, {header Path: functions}, stats). Without a root only
    the main file is collected; see FileScope for headers. The header mapping has an
    entry, possibly empty, for every collected header that occurs in the dump.

    A real PE translation unit dumps several GB. Declarations without a location in a
    collected file are only scanned to keep track of the current file; the others are
    parsed after their template instantiations are pruned. max_decl_bytes limits one
    such declaration, counted without spaces.
    """
    scope = FileScope(source, base, root, header_prefixes, exclude)
    newline_offsets = NewlineOffsets() if newline_offsets is None else newline_offsets
    current_file = None
    results, headers = [], {}
    stats = {"ast_bytes": 0, "decls_total": 0, "decls_parsed": 0,
             "largest_parsed_decl_bytes": 0, "functions_without_span": 0}
    for decl in iter_top_level_decls(stream):
        stats["ast_bytes"] += len(decl)
        stats["decls_total"] += 1
        files = [json.loads(b'"' + match.group(1) + b'"')
                 for match in LOCATION_FILE.finditer(decl)]
        collected = {scope(name) for name in files} - {None}
        if scope(current_file) is None and not collected:
            # Clang prints a location's file whenever it changes, so the next
            # declaration is in a collected file only if it prints one.
            current_file = None
            continue
        for file in collected - {scope.source}:
            headers.setdefault(file, [])
        decl = prune_instantiations(decl)
        # Deep expressions are indented by up to thousands of spaces, which the JSON
        # parser skips; the rest is what a declaration costs in memory.
        size = len(decl) - decl.count(b" ")
        if size > max_decl_bytes:
            raise AstDeclTooLarge(size)
        stats["decls_parsed"] += 1
        stats["largest_parsed_decl_bytes"] = max(stats["largest_parsed_decl_bytes"], size)
        node = json.loads(decl)
        current_file = annotate_locations(node, current_file)
        stats["functions_without_span"] += collect_functions(
            node, scope, newline_offsets, results)
    main = [item for file, item in results if file == scope.source]
    for file, item in results:
        if file != scope.source:
            headers.setdefault(file, []).append(item)
    return (unique_functions(main),
            {file: unique_functions(items) for file, items in headers.items()}, stats)


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


def analyze_one(entry, relative, source, compiler, timeout, max_bytes, **scope):
    """Analyze one translation unit; `scope` is passed on to parse_ast_stream().

    A successful result has the internal key "headers", {header Path: functions}.
    """
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
            outcome = parse_ast_stream(proc.stdout, source, entry["directory"], max_bytes,
                                       **scope)
        except AstDeclTooLarge as exc:
            failure = {"status": "ast_too_large", "ast_bytes": exc.size,
                       "error": f"A declaration to parse exceeds the {max_bytes} byte limit "
                                "(counted without spaces)"}
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
    functions, headers, stats = outcome
    return {"file": relative, "status": "ok", "functions": functions,
            "function_count": len(functions), **stats,
            "header_files": len(headers),
            "header_functions": sum(map(len, headers.values())),
            "headers": headers}


HEADER_SUFFIXES = (".h", ".hh", ".hpp", ".hxx", ".inl", ".ipp", ".tpp")


def unanalyzed_headers(root, prefixes, analyzed):
    """Return root-relative header files under the prefixes that no TU covered."""
    found = []
    for prefix in prefixes:
        base = root / prefix
        if base.is_dir():
            found += [path for path in base.rglob("*")
                      if path.suffix in HEADER_SUFFIXES and path.is_file()]
    return sorted({path.relative_to(root).as_posix() for path in found
                   if path.resolve() not in analyzed})


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
                        help="largest single top-level declaration to parse in memory, "
                             "counted without spaces and after pruning template "
                             "instantiations; the total AST size is not limited")
    parser.add_argument("--header-prefix", action="append", default=None,
                        help="also report definitions in headers under this path "
                             "(repeatable; default pe/ and src/)")
    parser.add_argument("--no-headers", action="store_true",
                        help="report definitions in the analyzed .cpp files only")
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
    header_prefixes = [] if args.no_headers else (args.header_prefix or ["pe/", "src/"])
    # A header is reported from the first translation unit that includes it; later
    # ones skip it. Main files of other TUs are never reported as headers.
    analyzed, headers = set(), []
    newline_offsets = NewlineOffsets()
    results = []
    for relative, source in eligible:
        try:
            result = analyze_one(by_source[source], relative, source, compiler,
                                 args.timeout, args.max_ast_mb * 1024 * 1024,
                                 root=root, header_prefixes=header_prefixes,
                                 exclude=analyzed | set(by_source),
                                 newline_offsets=newline_offsets)
        except (OSError, ValueError) as exc:
            result = {"file": relative, "status": "error", "functions": [],
                      "error": str(exc)}
        for path, functions in result.pop("headers", {}).items():
            analyzed.add(path)
            headers.append({"file": path.relative_to(root).as_posix(), "kind": "header",
                            "status": "ok", "analyzed_in": relative,
                            "functions": functions, "function_count": len(functions)})
        results.append({"file": relative, "kind": "source", **result})
        print(f"{relative}: {result['status']} ({len(result['functions'])} functions, "
              f"{result.get('header_functions', 0)} in {result.get('header_files', 0)} "
              f"new headers)", file=sys.stderr)
    headers.sort(key=lambda item: item["file"])
    failed = sum(item["status"] != "ok" for item in results)
    report = {"schema_version": 2, "metric": "cyclomatic_approx",
              "generated_utc": datetime.now(timezone.utc).isoformat(),
              "root": str(root), "build": str(build), "clang": compiler,
              "header_prefixes": header_prefixes,
              "translation_units": len(results), "translation_units_failed": failed,
              "headers": len(headers),
              "functions": sum(len(item["functions"]) for item in results + headers),
              "headers_not_analyzed": unanalyzed_headers(root, header_prefixes, analyzed),
              "files": results + headers}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Saved {args.output}; {report['functions']} functions in {len(results)} "
          f"sources and {len(headers)} headers; {failed} failures; "
          f"{len(report['headers_not_analyzed'])} headers not analyzed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
