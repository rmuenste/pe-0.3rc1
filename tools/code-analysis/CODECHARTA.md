# Viewing metrics in CodeCharta

[CodeCharta](https://codecharta.com) (MaibornWolff, BSD-3) shows code as a 3D city.
Here each **function** is a building and each source file or header a district of its
functions:

| Visual | Metric |
|---|---|
| Footprint (`area`) | `lines` of the function |
| Height and colour (`height`, `color`) | `cyclomatic_approx` |

Folders also carry the sum of a `functions` column, i.e. their function count. All
values come from `metrics.py` and keep its limits (see `README.md`): they are
approximations. Definitions in `src/` files and in the `pe/` and `src/` headers they
include are covered; the map shows them under `src/` and `pe/`.

## Requirements

- Docker. Both CodeCharta parts run as containers, nothing else is installed:
  `codecharta/codecharta-analysis:2.3.0` (the `ccsh` importer) and
  `codecharta/codecharta-visualization:2.9.1` (the web viewer, served by nginx).
  These are the versions the setup was tested with.
- Python >= 3.9.
- A `metrics.py` report. Clang and a PE build are needed only to produce the report,
  so it can be made on another machine and copied over.

## 1. Produce the report (build machine)

```sh
cmake --preset clang-serial
python3 tools/code-analysis/metrics.py \
  --build build/presets/clang-serial \
  --limit 0 \
  --output build/code-analysis/metrics-full-serial.json
```

The JSON is about 430 kB. To view it on another machine, copy it into the same
place in a checkout of this branch there, for example:

```sh
mkdir -p build/code-analysis
scp buildhost:path/to/pe/build/code-analysis/metrics-full-serial.json build/code-analysis/
```

## 2. Build the map and start the viewer (viewing machine)

From the repository root:

```sh
tools/code-analysis/codecharta.sh build/code-analysis/metrics-full-serial.json
```

The script

1. converts the report to CSV with `to_codecharta.py`,
2. runs `ccsh csvimport` in Docker to write `build/code-analysis/codecharta/<name>.cc.json`,
3. starts the viewer container `pe-codecharta-viewer` on `127.0.0.1:9000`, with the map
   directory mounted read-only, and prints the URL to open, e.g.

   ```
   http://127.0.0.1:9000/?file=pe/metrics-full-serial.cc.json&area=lines&height=cyclomatic_approx&color=cyclomatic_approx
   ```

Stop the viewer with `docker stop pe-codecharta-viewer`.

Options:

| Option | Effect |
|---|---|
| `--prefix src/core/rigidbody/` | only files under this path (repeatable) |
| `--name rigidbody` | map name; default is the report's basename. Use a distinct name per `--prefix` run, or it replaces the full map |
| `--no-viewer` | only build the map |
| `CODECHARTA_PORT=9100` | viewer port |
| `CODECHARTA_DIR=...` | output directory (default `build/code-analysis/codecharta`) |

A smaller map is easier to read at first:

```sh
tools/code-analysis/codecharta.sh build/code-analysis/metrics-full-serial.json \
  --prefix src/core/rigidbody/ --name rigidbody
```

The viewer can show several maps; add them with its file-open button, or change
`file=` in the URL.

### Viewing over SSH

If the report and Docker are only on a remote machine, run the script there and
forward the port from the local machine, then open the URL locally:

```sh
ssh -L 9000:127.0.0.1:9000 buildhost
```

## What the map leaves out

`to_codecharta.py` prints these on stderr instead of dropping them silently:

- files whose analysis failed (`timeout`, `ast_too_large`, `clang_error`, `json_error`);
- source files without measured functions. In a serial build these are `Settings.cpp`,
  `SimSettings.cpp`, `TimeStep.cpp` and `Random.cpp` (static data only),
  `RectilinearGrid.cpp` (needs MPI) and `delaunay_output.cpp` (needs CGAL);
- the number of headers without function definitions (declarations only).

Headers that no analyzed `.cpp` file includes are not in the map at all; the report
lists them in `headers_not_analyzed`.

## Path format

Function nodes use the synthetic path `<file>/<function>@<line_start>`, e.g.
`src/core/rigidbody/Box.cpp/Box@76`; the line keeps overloads apart. Names are not
class-qualified. A `/` in a name (`operator/`, `operator/=`) is replaced by `∕`
(U+2215) so it does not split the path.
