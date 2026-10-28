#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# The remote (CUDA) render step on Kaggle (D44, design reconstruction.md#first-reconstruction-loop): the Kaggle
# CLI runs in a pinned container; the token comes from the host's ~/.kaggle
# (access_token, or a legacy kaggle.json), mounted read-only. Nothing is
# installed on the host.
#
#   infra/scripts/kaggle.sh whoami                 check the token, print the user
#   infra/scripts/kaggle.sh smoke OUT              [SMOKE-TEST]: install gsplat on a Kaggle
#                                                  GPU, render 1000 random Gaussians, pull
#                                                  remote.json (versions, install mode) to OUT
#   infra/scripts/kaggle.sh bench OUT              Phase 6 (reconstruction.md §1): gsplat 1.5.3 train/render ms per
#                                                  iteration vs N, resolution, packed; SH3; the append path
#   infra/scripts/kaggle.sh train-smoke OUT        Phase 6: the live trainer on a synthetic corridor (exact depth
#                                                  and albedo): per-KF cost, held-out PSNR, GPU re-anchoring check
#   infra/scripts/kaggle.sh train DATA_DIR OUT     Phase 6: GPU training on real spools (spool_train_kernel.py):
#                                                  DATA_DIR = spools/<agent>/ + scene/ + dataset.json, uploaded
#                                                  as one private dataset (data.tar); pulls remote.json +
#                                                  splat_<budget>.tar.gz (budgets: ARIADNE_BUDGETS JSON)
#   infra/scripts/kaggle.sh render PKG_DIR OUT     upload a render package as a private
#                                                  dataset, run the render kernel on a GPU,
#                                                  pull renders.npz + remote.json to OUT
#   infra/scripts/kaggle.sh narration CUES OUT     demo-film narration (infra/eval/narration_ming.py, Ming US_James)
#                                                  on a T4; pulls remote.json + narration.tar.gz to OUT
#
# Everything on Kaggle is PRIVATE (dataset and kernels). Kernels time out after
# KAGGLE_TIMEOUT_MIN (default 60) of polling; a failed kernel's log is pulled too.
# ---------------------------------------------------------------------------
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
IMAGE=ariadne-kaggle:2.2.4
BASE=python@sha256:f77ac9e44ae96ef2c90b8053ea08c31f8be030f824196b0ae4db6d462c84e51f
WORK="$REPO_ROOT/build/kaggle"
TIMEOUT_MIN="${KAGGLE_TIMEOUT_MIN:-60}"
# CLI 2.x ignores enable_gpu alone: the accelerator must be named (kagglesdk: NvidiaTeslaT4 | NvidiaTeslaP100).
ACCEL="${KAGGLE_ACCELERATOR:-NvidiaTeslaT4}"

die() { echo "kaggle.sh: $*" >&2; exit 1; }
[ -f "$HOME/.kaggle/access_token" ] || [ -f "$HOME/.kaggle/kaggle.json" ] \
  || die "no ~/.kaggle/access_token or kaggle.json (kaggle.com -> Settings -> API -> Create New Token)"

image() {
  docker image inspect "$IMAGE" >/dev/null 2>&1 && return
  printf 'FROM %s\nRUN pip install --no-cache-dir --root-user-action=ignore kaggle==2.2.4\n' "$BASE" \
    | docker build -q -t "$IMAGE" - >/dev/null
}
kg() {   # kaggle CLI as the invoking user; repo build dir mounted at /w
  docker run --rm -u "$(id -u):$(id -g)" -e HOME=/home/u -e KAGGLE_CONFIG_DIR=/home/u/.kaggle \
    -v "$HOME/.kaggle:/home/u/.kaggle:ro" -v "$WORK:/w" -w /w "$IMAGE" kaggle "$@"
}
user() {   # the CLI resolves the account from the token
  local u
  u="$(kg config view 2>/dev/null | sed -n 's/^- username: //p')"
  [ -n "$u" ] && [ "$u" != None ] || die "token does not resolve to a Kaggle user"
  echo "$u"
}

push_and_wait() {   # $1 kernel dir (under $WORK), $2 slug, $3 OUT
  local dir="$1" slug="$2" out="$3" u st i
  u="$(user)"
  kg kernels push -p "/w/$dir" --accelerator "$ACCEL"
  for ((i = 0; i < TIMEOUT_MIN * 2; i++)); do
    sleep 30
    st="$(kg kernels status "$u/$slug" 2>&1 | tail -1)"
    echo "  $(date -u +%H:%M:%S) $st"
    case "$st" in
      *[Cc][Oo][Mm][Pp][Ll][Ee][Tt][Ee]*) break ;;
      *[Ee][Rr][Rr][Oo][Rr]*|*[Cc][Aa][Nn][Cc][Ee][Ll]*) kg kernels output "$u/$slug" -p "/w/$dir/out" || true
                                          die "kernel $slug failed; log in $WORK/$dir/out" ;;
    esac
  done
  case "$st" in *[Cc][Oo][Mm][Pp][Ll][Ee][Tt][Ee]*) ;; *) die "kernel $slug timed out after ${TIMEOUT_MIN} min";; esac
  mkdir -p "$WORK/$dir/out"
  kg kernels output "$u/$slug" -p "/w/$dir/out"
  mkdir -p "$out"
  cp "$WORK/$dir/out/"*.json "$out/"
  if ls "$WORK/$dir/out/"*.npz >/dev/null 2>&1; then cp "$WORK/$dir/out/"*.npz "$out/"; fi
  if ls "$WORK/$dir/out/"*.tar.gz >/dev/null 2>&1; then cp "$WORK/$dir/out/"*.tar.gz "$out/"; fi
  echo "pulled to $out"
}

kernel_dir() {   # $1 name, $2 slug, $3 dataset id or "", $4 prelude line, [$5 kernel source, default render_kernel.py]
  local d="$WORK/$1" u
  u="$(user)"
  rm -rf "$d"; mkdir -p "$d"
  { echo "$4"; cat "$REPO_ROOT/python/reconstruction/kaggle/${5:-render_kernel.py}"; } > "$d/kernel.py"
  python3 - "$d" "$u/$2" "$2" "$3" <<'EOF'
import json, sys
d, kid, title, ds = sys.argv[1:5]
json.dump({"id": kid, "title": title, "code_file": "kernel.py", "language": "python", "kernel_type": "script",
           "is_private": True, "enable_gpu": True, "enable_internet": True, "machine_shape": "NvidiaTeslaT4",
           "dataset_sources": [ds] if ds else [], "competition_sources": [], "kernel_sources": []},
          open(f"{d}/kernel-metadata.json", "w"), indent=2)
EOF
}

# The kernel imports ariadne_recon: embed the package (python sources only) and unpack it before main().
embed_pkg() {   # $1 kernel.py
    python3 - "$1" "$REPO_ROOT/python/reconstruction" <<'PYEOF'
import base64, io, sys, tarfile
k, root = sys.argv[1:3]
buf = io.BytesIO()
with tarfile.open(fileobj=buf, mode="w:gz") as t:
    t.add(f"{root}/ariadne_recon", arcname="ariadne_recon",
          filter=lambda i: None if "__pycache__" in i.name else i)
src = open(k).read()
head = ("import base64 as _b, io as _io, sys as _s, tarfile as _t\n"
        f"_t.open(fileobj=_io.BytesIO(_b.b64decode('{base64.b64encode(buf.getvalue()).decode()}')), mode='r:gz')"
        ".extractall('/kaggle/working/pkg')\n_s.path.insert(0, '/kaggle/working/pkg')\n")
open(k, "w").write(head + src)
PYEOF
}
dataset_up() {   # $1 dataset slug (under the user), $2 dir under $WORK holding the files + dataset-metadata.json
  local ds="$(user)/$1" st i
  if kg datasets status "$ds" >/dev/null 2>&1; then kg datasets version -p "/w/$2" -m "$(date -u +%FT%TZ)"
  else kg datasets create -p "/w/$2"; fi
  for ((i = 0; i < 80; i++)); do
    st="$(kg datasets status "$ds" 2>&1 | tail -1)"; echo "  dataset: $st"
    case "$st" in *ready*) return 0 ;; esac; sleep 15
  done
  die "dataset $ds not ready"
}

mkdir -p "$WORK"
image
case "${1:-}" in
  whoami) kg config view; echo "user: $(user)" ;;
  smoke)
    out="${2:?OUT dir}"
    kernel_dir smoke ariadne-recon-smoke-gpu "" "import os; os.environ['SMOKE'] = '1'"
    push_and_wait smoke ariadne-recon-smoke-gpu "$out" ;;
  bench)
    out="${2:?OUT dir}"
    kernel_dir bench ariadne-recon-bench-gpu "" "# Ariadne Phase 6 T4 microbenchmark (generated by infra/scripts/kaggle.sh)" bench_kernel.py
    push_and_wait bench ariadne-recon-bench-gpu "$out" ;;
  train-smoke)
    out="${2:?OUT dir}"
    kernel_dir train ariadne-recon-train-smoke-gpu "" "# Ariadne Phase 6 trainer smoke (generated by infra/scripts/kaggle.sh)" train_smoke_kernel.py
    embed_pkg "$WORK/train/kernel.py"
    push_and_wait train ariadne-recon-train-smoke-gpu "$out" ;;
  train)
    data="${2:?DATA_DIR (spools/, scene/, dataset.json)}"; out="${3:?OUT dir}"
    [ -d "$data/spools" ] && [ -f "$data/scene/scene.json" ] || die "$data needs spools/<agent>/ and scene/scene.json"
    u="$(user)"
    rm -rf "$WORK/trainds"; mkdir -p "$WORK/trainds"
    tar -C "$data" -cf "$WORK/trainds/data.tar" --dereference spools scene $( [ -f "$data/dataset.json" ] && echo dataset.json )
    printf '{"title": "ariadne-spools", "id": "%s/ariadne-spools", "licenses": [{"name": "CC0-1.0"}]}\n' "$u" \
      > "$WORK/trainds/dataset-metadata.json"
    dataset_up ariadne-spools trainds
    budgets="${ARIADNE_BUDGETS:-}"; [ -n "$budgets" ] || budgets='{"batch": 100, "live": 15}'   # D198's two rows
    kernel_dir trainspool ariadne-recon-train-gpu "$u/ariadne-spools" \
      "import os; os.environ.setdefault('ARIADNE_BUDGETS', '$budgets')" spool_train_kernel.py
    embed_pkg "$WORK/trainspool/kernel.py"
    push_and_wait trainspool ariadne-recon-train-gpu "$out" ;;
  render)
    pkg="${2:?render package dir}"; out="${3:?OUT dir}"
    u="$(user)"; ds="$u/ariadne-render-pkg"
    rm -rf "$WORK/pkg"; mkdir -p "$WORK/pkg"
    cp "$pkg/render_package.npz" "$pkg/render_package.json" "$WORK/pkg/"
    printf '{"title": "ariadne-render-pkg", "id": "%s", "licenses": [{"name": "CC0-1.0"}]}\n' "$ds" \
      > "$WORK/pkg/dataset-metadata.json"
    if kg datasets status "$ds" >/dev/null 2>&1; then
      kg datasets version -p /w/pkg -m "package $(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["sha256"][:12])' "$pkg/render_package.json")"
    else
      kg datasets create -p /w/pkg
    fi
    for ((i = 0; i < 40; i++)); do
      st="$(kg datasets status "$ds" 2>&1 | tail -1)"; echo "  dataset: $st"
      case "$st" in *ready*) break ;; esac; sleep 15
    done
    case "$st" in *ready*) ;; *) die "dataset $ds not ready" ;; esac
    kernel_dir render ariadne-recon-render-gpu "$ds" "# Ariadne 1.7 render kernel (generated by infra/scripts/kaggle.sh)"
    push_and_wait render ariadne-recon-render-gpu "$out" ;;
  narration)   # CUES.jsonl OUT: infra/eval/narration_ming.py on a T4 (the AWS runner's voice and checks)
    cues="${2:?cues.jsonl}"; out="${3:?OUT dir}"
    u="$(user)"
    rm -rf "$WORK/cues"; mkdir -p "$WORK/cues"; cp "$cues" "$WORK/cues/cues.jsonl"
    printf '{"title": "ariadne-narration-cues", "id": "%s/ariadne-narration-cues", "licenses": [{"name": "CC0-1.0"}]}\n' "$u" \
      > "$WORK/cues/dataset-metadata.json"
    dataset_up ariadne-narration-cues cues
    kernel_dir narration ariadne-narration-gpu "$u/ariadne-narration-cues" ""
    { echo "NARRATION_MING_B64 = '$(base64 -w0 "$REPO_ROOT/infra/eval/narration_ming.py")'"
      cat "$REPO_ROOT/infra/eval/narration_kaggle_kernel.py"; } > "$WORK/narration/kernel.py"
    TIMEOUT_MIN="${KAGGLE_TIMEOUT_MIN:-90}" push_and_wait narration ariadne-narration-gpu "$out" ;;
  *) sed -n '2,27p' "$0"; exit 2 ;;
esac
