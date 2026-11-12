"""Unit tests for the pure source-selection and AST metrics helpers."""
import io
import json
import pathlib
import shutil
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from metrics import (AstDeclTooLarge, analyze_one, clang_command, decision_count,
                     extract_functions, parse_ast_stream, prune_instantiations)
from run_tidy import load_sources, write_first_variant_database


def function_node(name, loc, begin, end, decisions=0):
    return {"kind": "FunctionDecl", "name": name, "loc": loc,
            "range": {"begin": begin, "end": end},
            "inner": [{"kind": "CompoundStmt",
                       "inner": [{"kind": "IfStmt", "inner": []}] * decisions}]}


def dump_tu(decls):
    """Serialize declarations with the same layout as clang's -ast-dump=json."""
    tu = {"kind": "TranslationUnitDecl", "loc": {}, "inner": decls}
    return json.dumps(tu, indent=2).encode()


class MetricsTests(unittest.TestCase):
    def test_decision_count(self):
        node = {"kind": "CompoundStmt", "inner": [
            {"kind": "IfStmt", "inner": [
                {"kind": "BinaryOperator", "opcode": "&&", "inner": []},
                {"kind": "ForStmt", "inner": []}
            ]},
            {"kind": "LambdaExpr", "inner": [{"kind": "IfStmt"}]}
        ]}
        self.assertEqual(decision_count(node), 3)

    def test_extract_function(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = pathlib.Path(tmp) / "sample.cpp"
            path.write_text("int f(int a) { if(a) return 1; return 0; }\n")
            node = {"kind": "TranslationUnitDecl", "inner": [
                {"kind": "FunctionDecl", "name": "f",
                 "loc": {"file": str(path)},
                 "range": {"begin": {"line": 1}, "end": {"line": 1}},
                 "inner": [{"kind": "CompoundStmt", "inner": [
                     {"kind": "IfStmt", "inner": []}
                 ]}]}
            ]}
            result = extract_functions(node, path)
            self.assertEqual(result[0]["cyclomatic_approx"], 2)
            self.assertEqual(result[0]["lines"], 1)

    def test_offset_locations_without_line_numbers(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = pathlib.Path(tmp) / "single.cpp"
            code = "int f(int a) { if(a) return 1; return 0; }\n"
            path.write_text(code)
            node = {"kind": "TranslationUnitDecl", "inner": [
                {"kind": "FunctionDecl", "name": "f",
                 "loc": {"file": str(path), "line": 1},
                 "range": {"begin": {"offset": 0}, "end": {"offset": 42}},
                 "inner": [{"kind": "CompoundStmt", "inner": [
                     {"kind": "IfStmt", "inner": []}
                 ]}]}
            ]}
            result = extract_functions(node, path)
            self.assertEqual(len(result), 1)
            self.assertEqual(result[0]["cyclomatic_approx"], 2)

    def test_command(self):
        entry = {"arguments": ["c++", "-Iinclude", "-O2", "-MD", "-MF",
                               "a.d", "-c", "src/a.cpp", "-o", "a.o"]}
        cmd = clang_command(entry, "clang++")
        self.assertEqual(cmd[:4], ["clang++", "-Xclang", "-ast-dump=json", "-fsyntax-only"])
        self.assertIn("src/a.cpp", cmd)
        self.assertNotIn("-c", cmd)
        self.assertNotIn("-o", cmd)
        self.assertNotIn("a.d", cmd)

    def test_load_sources_filters_build_and_duplicates(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp)
            (root / "src").mkdir()
            (root / "build").mkdir()
            source = root / "src" / "a.cpp"
            source.write_text("int a;\n")
            entries = [
                {"directory": str(root), "file": "src/a.cpp"},
                {"directory": str(root), "file": "src/a.cpp"},
                {"directory": str(root), "file": "build/generated.cpp"}
            ]
            db = root / "compile_commands.json"
            db.write_text(json.dumps(entries))
            result = load_sources(db, root.resolve(), ["src/"])
            self.assertEqual(len(result), 1)
            self.assertEqual(result[0][0], "src/a.cpp")

    def test_first_variant_database_keeps_first_command_only(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp).resolve()
            source = root / "a.cpp"
            other = root / "b.cpp"
            source.write_text("int a;\n")
            other.write_text("int b;\n")
            entries = [
                {"directory": str(root), "file": "a.cpp", "command": "first"},
                {"directory": str(root), "file": "a.cpp", "command": "second"},
                {"directory": str(root), "file": "b.cpp", "command": "unselected"},
            ]
            db = root / "compile_commands.json"
            db.write_text(json.dumps(entries))
            output = root / "filtered"
            output.mkdir()
            counts = write_first_variant_database(db, [("a.cpp", source)], output)
            filtered = json.loads((output / "compile_commands.json").read_text())
            self.assertEqual([item["command"] for item in filtered], ["first"])
            self.assertEqual(counts, {source: 2})


class LocationTrackingTests(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.dir = pathlib.Path(tmp.name).resolve()
        self.main = self.dir / "main.cpp"
        self.main.write_text("int m1() {}\nint m2() {}\n")
        self.header = str(self.dir / "header.h")

    def decls(self):
        """Clang's location compression: "file" appears only when the file changes."""
        main = str(self.main)
        return [
            function_node("h1", {"offset": 900, "file": self.header, "line": 90},
                          {"offset": 900}, {"offset": 910}, decisions=1),
            function_node("h2", {"offset": 950, "line": 95},
                          {"offset": 950}, {"offset": 960}, decisions=1),
            function_node("m1", {"offset": 0, "file": main, "line": 1},
                          {"offset": 0}, {"offset": 10}, decisions=2),
            function_node("m2", {"offset": 12, "line": 2},
                          {"offset": 12}, {"offset": 22}),
        ]

    def test_header_functions_without_file_key_are_rejected(self):
        ast = {"kind": "TranslationUnitDecl", "inner": self.decls()}
        result = extract_functions(ast, self.main)
        self.assertEqual([item["name"] for item in result], ["m1", "m2"])
        self.assertEqual(result[0]["cyclomatic_approx"], 3)
        self.assertEqual(result[1]["line_start"], 2)

    def test_included_from_is_not_a_location(self):
        # A header included by the main file names it only in includedFrom.
        included = {"offset": 900, "file": self.header, "line": 90,
                    "includedFrom": {"file": str(self.main)}}
        ast = {"kind": "TranslationUnitDecl", "inner": [
            function_node("h1", included, {"offset": 900}, {"offset": 910})]}
        self.assertEqual(extract_functions(ast, self.main), [])

    def test_stream_parses_only_main_file_declarations(self):
        stream = io.BytesIO(dump_tu(self.decls()))
        functions, _, stats = parse_ast_stream(stream, self.main)
        self.assertEqual([item["name"] for item in functions], ["m1", "m2"])
        self.assertEqual(stats["decls_total"], 4)
        self.assertEqual(stats["decls_parsed"], 2)

    def test_stream_matches_full_parse(self):
        decls = self.decls()
        expected = extract_functions({"kind": "TranslationUnitDecl", "inner": self.decls()},
                                     self.main)
        functions, _, _ = parse_ast_stream(io.BytesIO(dump_tu(decls)), self.main)
        self.assertEqual(functions, expected)

    def test_stream_reports_truncated_dump(self):
        data = dump_tu(self.decls())
        with self.assertRaises(ValueError):
            parse_ast_stream(io.BytesIO(data[:len(data) // 2]), self.main)

    def test_stream_rejects_unknown_layout(self):
        with self.assertRaises(ValueError):
            parse_ast_stream(io.BytesIO(b'{"kind": "TranslationUnitDecl"}'), self.main)

    def test_stream_size_limit_applies_to_collected_files_only(self):
        stream = io.BytesIO(dump_tu(self.decls()[:2]))
        functions, _, stats = parse_ast_stream(stream, self.main, max_decl_bytes=1)
        self.assertEqual(functions, [])
        self.assertEqual(stats["decls_parsed"], 0)
        with self.assertRaises(AstDeclTooLarge):
            parse_ast_stream(io.BytesIO(dump_tu(self.decls())), self.main, max_decl_bytes=1)


class HeaderScopeTests(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = pathlib.Path(tmp.name).resolve()
        (self.root / "pe").mkdir()
        self.main = self.root / "main.cpp"
        self.main.write_text("int m1() {}\nint m2() {}\n")
        self.header = self.root / "pe" / "header.h"
        self.header.write_text(("x" * 9 + "\n") * 100)  # line n starts at offset 10 (n - 1)
        self.other = self.root / "other.h"

    def stream(self):
        return io.BytesIO(dump_tu([
            function_node("h1", {"offset": 900, "file": str(self.header), "line": 91},
                          {"offset": 900}, {"offset": 910}, decisions=1),
            function_node("x", {"offset": 5, "file": str(self.other), "line": 1},
                          {"offset": 5}, {"offset": 8}),
            # No "file": still other.h, which is not collected.
            function_node("y", {"offset": 950, "line": 96}, {"offset": 950}, {"offset": 960}),
            function_node("m1", {"offset": 0, "file": str(self.main), "line": 1},
                          {"offset": 0}, {"offset": 10}),
            # No "file" after the main file: back in the header only if printed.
            function_node("h2", {"offset": 950, "file": str(self.header), "line": 96},
                          {"offset": 950}, {"offset": 960}, decisions=2),
        ]))

    def test_headers_under_prefix_are_collected(self):
        functions, headers, stats = parse_ast_stream(
            self.stream(), self.main, root=self.root, header_prefixes=("pe/",))
        self.assertEqual([item["name"] for item in functions], ["m1"])
        self.assertEqual(list(headers), [self.header])
        self.assertEqual([(item["name"], item["line_start"], item["cyclomatic_approx"])
                          for item in headers[self.header]], [("h1", 91, 2), ("h2", 96, 3)])
        # x is parsed as well: it could have started in the header printed before it.
        self.assertEqual(stats["decls_parsed"], 4)

    def test_without_prefixes_only_the_main_file_is_collected(self):
        functions, headers, _ = parse_ast_stream(self.stream(), self.main, root=self.root)
        self.assertEqual([item["name"] for item in functions], ["m1"])
        self.assertEqual(headers, {})

    def test_excluded_headers_are_skipped(self):
        functions, headers, stats = parse_ast_stream(
            self.stream(), self.main, root=self.root, header_prefixes=("pe/",),
            exclude={self.header})
        self.assertEqual([item["name"] for item in functions], ["m1"])
        self.assertEqual(headers, {})
        self.assertEqual(stats["decls_parsed"], 2)  # m1, and h2 after the main file


def template_dump(children, file="/x/a.h"):
    decl = {"id": "0x1", "kind": "ClassTemplateDecl",
            "loc": {"offset": 1, "file": file, "line": 1}, "inner": children}
    return dump_tu([{"id": "0x0", "kind": "NamespaceDecl", "inner": [decl]}])


class PruneTests(unittest.TestCase):
    def test_class_template_instantiations_become_placeholders(self):
        pattern = {"id": "0x2", "kind": "CXXRecordDecl", "name": "Box"}
        instance = {"id": "0x3", "kind": "ClassTemplateSpecializationDecl",
                    "loc": {"offset": 7, "file": "/x/b.h", "line": 2},
                    "inner": [{"id": "0x4", "kind": "CXXMethodDecl"}]}
        tu = json.loads(prune_instantiations(template_dump([pattern, instance])))
        children = tu["inner"][0]["inner"][0]["inner"]
        self.assertEqual(children, [pattern, {"file": "/x/b.h"}])

    def test_function_template_keeps_the_first_function(self):
        functions = [{"id": f"0x{i}", "kind": "FunctionDecl", "name": "f"} for i in range(3)]
        param = {"id": "0x9", "kind": "TemplateTypeParmDecl"}
        decl = {"id": "0x1", "kind": "FunctionTemplateDecl", "inner": [param] + functions}
        tu = json.loads(prune_instantiations(dump_tu([decl])))
        self.assertEqual(tu["inner"][0]["inner"], [param, functions[0], {}, {}])

    def test_text_without_templates_is_unchanged(self):
        data = dump_tu([{"id": "0x1", "kind": "FunctionDecl", "name": "f"}])
        self.assertEqual(prune_instantiations(data), data)


@unittest.skipUnless(shutil.which("clang++"), "clang++ is not installed")
class ClangIntegrationTests(unittest.TestCase):
    """Check hand-counted values against a real Clang AST."""

    HEADER = """\
inline int headerOne(int a) { if(a) return 1; return 0; }
inline int headerTwo(int a) { if(a) return 1; return 0; }
template <typename T> struct Wrapped { T twice(T a) { return a > 0 ? a + a : a; } };
"""
    SOURCE = """\
#include "fixture.h"
#define BRANCH(x) if(x) return 1

int straight(int a) {
   return a;
}

int nested(int a, int b) {
   for(int i = 0; i < a; ++i) {
      if(i > b) {
         if(i == 3 && b == 4) return i;
      }
   }
   return 0;
}

int switcher(int a) {
   switch(a) {
      case 1: return 1;
      case 2: return 2;
      default: return a > 0 ? 3 : 4;
   }
}

int catcher() {
   try { return 1; }
   catch(int) { return 2; }
   catch(...) { return 3; }
}

int withMacro(int a) {
   BRANCH(a);
   return 0;
}

int withLambda(int a) {
   auto f = [](int x) { if(x) return 1; return 0; };
   return f(a);
}

template <typename T> T templated(T a) {
   if(a) return a;
   return T();
}
template <typename T> int discarded(T a) {
   if constexpr(sizeof(T) > 100) {
      if(a) return 1;
   }
   return 0;
}
int useTemplate() { return templated<int>(1) + static_cast<int>(templated<long>(2)) + discarded(1); }

struct Implicit { ~Implicit() = default; int member(int a) const { return a ? 1 : 0; } };
Implicit implicitInstance;

template <typename T> struct Holder { int out(T a); };
template <typename T> int Holder<T>::out(T a) {
   if constexpr(sizeof(T) > 100) {
      if(a) return 1;
   }
   return 0;
}
int useHolder() { return Holder<int>().out(1) + Wrapped<int>().twice(2); }
"""

    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        directory = pathlib.Path(tmp.name).resolve()
        (directory / "fixture.h").write_text(self.HEADER)
        self.source = directory / "fixture.cpp"
        self.source.write_text(self.SOURCE)
        entry = {"directory": str(directory),
                 "arguments": ["clang++", "-std=c++17", "-c", str(self.source), "-o", "a.o"]}
        self.result = analyze_one(entry, "fixture.cpp", self.source,
                                  shutil.which("clang++"), 60, 1 << 20)
        self.assertEqual(self.result["status"], "ok", self.result.get("error"))
        self.by_name = {item["name"]: item for item in self.result["functions"]}

    def test_header_functions_are_excluded(self):
        self.assertNotIn("headerOne", self.by_name)
        self.assertNotIn("headerTwo", self.by_name)

    def test_headers_are_collected_with_a_prefix(self):
        directory = self.source.parent
        entry = {"directory": str(directory),
                 "arguments": ["clang++", "-std=c++17", "-c", str(self.source)]}
        result = analyze_one(entry, "fixture.cpp", self.source, shutil.which("clang++"),
                             60, 1 << 20, root=directory, header_prefixes=["fixture.h"])
        self.assertEqual(result["status"], "ok", result.get("error"))
        self.assertEqual(result["functions"], self.result["functions"])
        header = result["headers"][directory / "fixture.h"]
        self.assertEqual([(item["name"], item["line_start"], item["cyclomatic_approx"])
                          for item in header],
                         [("headerOne", 1, 2), ("headerTwo", 2, 2), ("twice", 3, 2)])

    def test_hand_counted_complexity(self):
        expected = {
            "straight": 1,    # no decisions
            "nested": 5,      # for + if + if + &&
            "switcher": 4,    # case + case + ternary; default is not counted
            "catcher": 3,     # catch + catch
            "withMacro": 2,   # if from the macro expansion
            "withLambda": 1,  # the lambda body is excluded by design
            "templated": 2,   # if, counted once despite two instantiations
            "discarded": 3,   # as written; the instantiation drops the inner if
            "useTemplate": 1,
            "member": 2,      # ternary
            "out": 3,         # as written, although its instantiation precedes it
        }
        actual = {name: self.by_name[name]["cyclomatic_approx"] for name in expected}
        self.assertEqual(actual, expected)

    def test_line_spans(self):
        self.assertEqual(self.by_name["straight"]["line_start"], 4)
        self.assertEqual(self.by_name["straight"]["line_end"], 6)
        self.assertEqual(self.by_name["nested"]["line_start"], 8)
        self.assertEqual(self.by_name["nested"]["lines"], 8)

    def test_templates_are_deduplicated_and_implicit_members_excluded(self):
        names = [item["name"] for item in self.result["functions"]]
        self.assertEqual(names.count("templated"), 1)
        # implicitInstance forces Clang to define the implicit default constructor.
        self.assertNotIn("Implicit", names)

    def test_clang_failure_is_reported(self):
        broken = self.source.with_name("broken.cpp")
        broken.write_text("int broken( {\n")
        entry = {"directory": str(self.source.parent),
                 "arguments": ["clang++", "-c", str(broken)]}
        result = analyze_one(entry, "broken.cpp", broken, shutil.which("clang++"), 60, 1 << 20)
        self.assertEqual(result["status"], "clang_error")
        self.assertIn("error", result["error"])


if __name__ == "__main__":
    unittest.main()
