"""Unit tests for the pure source-selection and AST metrics helpers."""
import json
import pathlib
import sys
import tempfile
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from metrics import clang_command, decision_count, extract_functions
from run_tidy import load_sources


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


if __name__ == "__main__":
    unittest.main()
