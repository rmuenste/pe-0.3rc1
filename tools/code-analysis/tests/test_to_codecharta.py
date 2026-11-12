import csv
import io
import json
import pathlib
import sys
import tempfile
import unittest
from contextlib import redirect_stderr

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from to_codecharta import function_path, main, rows


def function(name, line, cc=1, lines=3):
    return {"name": name, "line_start": line, "lines": lines,
            "decision_nodes": cc - 1, "cyclomatic_approx": cc}


REPORT = {"files": [
    {"file": "src/a/A.cpp", "status": "ok",
     "functions": [function("f", 10, cc=4), function("f", 20), function("operator/", 30)]},
    {"file": "src/a/Empty.cpp", "status": "ok", "functions": []},
    {"file": "src/b/B.cpp", "status": "timeout", "functions": []},
    {"file": "src/b/C.cpp", "status": "ok", "functions": [function("g", 5)]},
    {"file": "pe/b/D.h", "kind": "header", "status": "ok", "functions": [function("h", 7)]},
    {"file": "pe/b/Fwd.h", "kind": "header", "status": "ok", "functions": []},
]}


class ToCodeChartaTests(unittest.TestCase):
    def test_overloads_get_distinct_paths(self):
        out, _ = rows(REPORT)
        self.assertIn("src/a/A.cpp/f@10", [r["path"] for r in out])
        self.assertIn("src/a/A.cpp/f@20", [r["path"] for r in out])

    def test_slash_in_name_does_not_split_path(self):
        path = function_path("src/a/A.cpp", function("operator/", 30))
        self.assertEqual(path.count("/"), 3)
        self.assertTrue(path.endswith("operator\u2215@30"))

    def test_failed_and_empty_files_are_reported_not_dropped_silently(self):
        out, skipped = rows(REPORT)
        self.assertEqual(skipped["failed"], ["src/b/B.cpp (timeout)"])
        self.assertEqual(skipped["empty"], ["src/a/Empty.cpp"])
        self.assertEqual(skipped["empty_headers"], ["pe/b/Fwd.h"])
        self.assertEqual(len(out), 5)
        self.assertIn("pe/b/D.h/h@7", [r["path"] for r in out])

    def test_prefix_filter(self):
        out, skipped = rows(REPORT, ["src/b/"])
        self.assertEqual([r["path"] for r in out], ["src/b/C.cpp/g@5"])
        self.assertEqual(skipped["empty"], [])

    def test_csv_columns_and_values(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = pathlib.Path(tmp)
            (tmp / "m.json").write_text(json.dumps(REPORT))
            with redirect_stderr(io.StringIO()) as err:
                self.assertEqual(main(["--input", str(tmp / "m.json"),
                                       "--output", str(tmp / "m.csv")]), 0)
            with (tmp / "m.csv").open() as handle:
                first = next(csv.DictReader(handle))
        self.assertEqual(first, {"path": "src/a/A.cpp/f@10", "cyclomatic_approx": "4",
                                 "lines": "3", "decision_nodes": "3", "functions": "1"})
        self.assertIn("analysis failed: src/b/B.cpp (timeout)", err.getvalue())
        self.assertIn("no functions: 1 headers", err.getvalue())

    def test_duplicate_paths_fail(self):
        report = {"files": [{"file": "x.cpp", "status": "ok",
                             "functions": [function("f", 1), function("f", 1)]}]}
        with tempfile.TemporaryDirectory() as tmp:
            tmp = pathlib.Path(tmp)
            (tmp / "m.json").write_text(json.dumps(report))
            with redirect_stderr(io.StringIO()):
                self.assertEqual(main(["--input", str(tmp / "m.json"),
                                       "--output", str(tmp / "m.csv")]), 1)


if __name__ == "__main__":
    unittest.main()
