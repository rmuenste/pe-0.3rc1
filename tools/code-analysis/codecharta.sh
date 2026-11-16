#!/usr/bin/env bash
# Build a CodeCharta map from a metrics.py report and serve it on localhost.
# Needs python3 and Docker; no Clang or PE build. See CODECHARTA.md.
#
#   tools/code-analysis/codecharta.sh METRICS_JSON [--name NAME] [--no-viewer] [--prefix PATH]...
#
# The map is written to <CODECHARTA_DIR>/<NAME>.cc.json; NAME defaults to the input's
# basename, so give each --prefix run its own --name to keep the full map.
# Environment: CODECHARTA_DIR (output directory, default build/code-analysis/codecharta),
#              CODECHARTA_PORT (viewer port on 127.0.0.1, default 9000).
set -euo pipefail

ANALYSIS_IMAGE=codecharta/codecharta-analysis:2.3.0
VIEWER_IMAGE=codecharta/codecharta-visualization:2.9.1
CONTAINER=pe-codecharta-viewer

usage() {
   sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'
   exit "${1:-0}"
}

[[ $# -ge 1 ]] || usage 2
[[ $1 == -h || $1 == --help ]] && usage 0

metrics=$1
shift
[[ -f $metrics ]] || { echo "error: no such file: $metrics" >&2; exit 2; }

viewer=1
name=$(basename "$metrics" .json)
convert_args=()
while [[ $# -gt 0 ]]; do
   case $1 in
      --no-viewer) viewer=0 ;;
      --name)      [[ $# -ge 2 ]] || usage 2; name=$2; shift ;;
      --prefix)    [[ $# -ge 2 ]] || usage 2; convert_args+=(--prefix "$2"); shift ;;
      -h|--help)   usage 0 ;;
      *)           echo "error: unknown option: $1" >&2; usage 2 ;;
   esac
   shift
done

script_dir=$(cd "$(dirname "$0")" && pwd)
out_dir=${CODECHARTA_DIR:-build/code-analysis/codecharta}
port=${CODECHARTA_PORT:-9000}
mkdir -p "$out_dir"
out_dir=$(cd "$out_dir" && pwd)
[[ $name =~ ^[A-Za-z0-9._-]+$ ]] || { echo "error: --name must be a plain file name" >&2; exit 2; }

python3 "$script_dir/to_codecharta.py" --input "$metrics" --output "$out_dir/$name.csv" \
   ${convert_args[@]+"${convert_args[@]}"}

rm -f "$out_dir/$name.cc.json"
docker run --rm --user "$(id -u):$(id -g)" -v "$out_dir:/data" -w /data "$ANALYSIS_IMAGE" \
   ccsh csvimport --path-column-name=path --not-compressed \
   --output-file="$name.cc.json" "$name.csv"
[[ -s $out_dir/$name.cc.json ]] || { echo "error: ccsh wrote no $name.cc.json" >&2; exit 1; }
echo "map: $out_dir/$name.cc.json"

[[ $viewer -eq 1 ]] || exit 0

docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
docker run -d --rm --name "$CONTAINER" -p "127.0.0.1:$port:80" \
   -v "$out_dir:/usr/share/nginx/html/pe:ro" "$VIEWER_IMAGE" >/dev/null
echo "viewer: http://127.0.0.1:$port/?file=pe/$name.cc.json&area=lines&height=cyclomatic_approx&color=cyclomatic_approx"
echo "stop it with: docker stop $CONTAINER"
