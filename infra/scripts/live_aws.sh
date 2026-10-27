#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Live runs on AWS GPU (design LIVE_RUNTIME_L1 D155): one g4dn.xlarge SPOT (T4, 4 vCPU: the whole D84 quota) through
# infra/scripts/spinup.sh, with its three brakes (shutdown timer on the instance, trap here, sweep).
#
#   infra/scripts/live_aws.sh push                 local images -> ECR, tag live-<image id 12>; prints the refs
#   infra/scripts/live_aws.sh payload SCENE        build outputs + the live scene -> S3 (SHA256SUMS); prints the URI
#   infra/scripts/live_aws.sh smoke                [SMOKE-TEST] NVIDIA EGL in ariadne/sim on g4dn (~15 min)
#   infra/scripts/live_aws.sh run SCENE END_S      payload + images -> live.sh run + replay x2 on the instance
#   infra/scripts/live_aws.sh batch JOBS MAX_MIN   lockstep jobs, one after another on one instance (D165; 4.3 gate):
#       JOBS: one job per line "PRESET SEED END_S PLANNER_CONFIG|-" (- = the 1.8 placeholder row); every job runs
#       with ARIADNE_LOCKSTEP=1 and ARIADNE_SHADOW_MAP_CONFIG=configs/mission/mission_v2.yaml. Scenes must be composed
#       locally first (live.sh compose PRESET SEED 0.15). FIRST, an EGL reproducibility check: ci_tiny seed 0, 20 s,
#       twice, lockstep_repro_check.py; the batch STOPS if it fails. Each job's slim result (everything but
#       inputs.mcap, ~10 GB per 200 s run, and sockets) goes to S3 as soon as it ends, so a spot interruption loses
#       one job only. MAX_MIN <= 240 (spinup's cap): split long gates into several batches.
#
# The build outputs are made locally (the 2-job dev container) and shipped, not rebuilt on 4 vCPUs:
#   build/docker/cpp/runtime/ariadne_live, build/sim-plugins/*.so, build/ros2/install/, SCENE/.
# The instance gets the working tree from spinup.sh (tracked + untracked-not-ignored) and the payload on top, so
# every relative path in live.sh is the same as on the dev host. Results come back to build/runs/<run-id>/output/.
# ---------------------------------------------------------------------------
set -Eeuo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
AWS="$REPO_ROOT/infra/scripts/aws.sh"
REGION=eu-north-1
log() { printf '[live_aws %s] %s\n' "$(date -u +%H:%M:%S)" "$*" >&2; }

account() { "$AWS" aws sts get-caller-identity --query Account --output text | tr -d '\r'; }
bucket() {
  if [ -n "${ARIADNE_RUNNER_JSON:-}" ]; then
    python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["artifact_bucket"])' "$ARIADNE_RUNNER_JSON"
  else
    "$AWS" tf main output -json runner | python3 -c 'import json,sys; print(json.load(sys.stdin)["artifact_bucket"])'
  fi
}

# Profile/cap knobs (D219): LIVE_AWS_PROFILE=cpux86 runs the same flow on the c7i CPU profile (llvmpipe render,
# proven deterministic across machines at D168 M1); LIVE_AWS_MAX_MIN raises cmd_run's instance cap for long runs.
PROFILE="${LIVE_AWS_PROFILE:-gpu}"
RUN_MAX_MIN="${LIVE_AWS_MAX_MIN:-60}"
INSTANCE_ARGS=()
if [[ -n "${LIVE_AWS_INSTANCE_TYPE:-}" ]]; then
  INSTANCE_ARGS=(--type "$LIVE_AWS_INSTANCE_TYPE")
fi
# LIVE_AWS_AZ pins the spot AZ (spinup.sh --az): the cheapest AZ can be out of g5 capacity.
[[ -n "${LIVE_AWS_AZ:-}" ]] && INSTANCE_ARGS+=(--az "$LIVE_AWS_AZ")

# local image -> ECR repo
declare -A REPO=([ariadne/sim:dev]=ariadne/ariadne-sim [ariadne/core-dev:dev]=ariadne/ariadne-core [ariadne-console:1]=ariadne/ariadne-console [ariadne-orchestrator:1]=ariadne/ariadne-orchestrator [ariadne-recon:1]=ariadne/ariadne-gpu)

ref_of() {   # ECR reference of a local image, tagged by its own image id (content-addressed, not "latest")
  local img="$1" reg="$2" id
  id="$(docker image inspect "$img" --format '{{.Id}}' | cut -d: -f2 | cut -c1-12)"
  echo "$reg/${REPO[$img]}:live-$id"
}

cmd_push() {
  local reg; reg="$(account).dkr.ecr.$REGION.amazonaws.com"
  "$AWS" aws ecr get-login-password --region "$REGION" | tr -d '\r' | docker login --username AWS --password-stdin "$reg" >/dev/null
  local img
  for img in "${!REPO[@]}"; do
    local ref; ref="$(ref_of "$img" "$reg")"
    docker tag "$img" "$ref"
    log "push $img -> $ref"
    docker push -q "$ref" >/dev/null
    echo "$img $ref $(docker image inspect "$img" --format '{{.Id}}')"
  done
}

cmd_payload() {   # SCENE... (one or more scene directories)
  local id stage
  id="$(date -u +%Y%m%dT%H%M%SZ)"
  stage="$REPO_ROOT/build/live_payload/$id"
  mkdir -p "$stage/p"
  local extra=()
  # D221: the collab worker binary + the ONNX models ride the payload when present (.cache is gitignored, so the
  # tree sync never ships them; the collab run needs them at the configured relative paths).
  [ -f "$REPO_ROOT/build/docker/cpp/runtime/ariadne_collab_worker" ] && extra+=(build/docker/cpp/runtime/ariadne_collab_worker)
  [ -d "$REPO_ROOT/.cache/models" ] && extra+=(.cache/models)
  [ -d "$REPO_ROOT/build/ort/lib" ] && extra+=(build/ort/lib)   # ONNX Runtime of an ARIADNE_WITH_ORT build
  (cd "$REPO_ROOT" && tar chf - build/docker/cpp/runtime/ariadne_live build/sim-plugins/libariadne_kinematic_mover.so \
     build/ros2/install "${extra[@]}" "$@") | tar xf - -C "$stage/p"
  (cd "$stage/p" && find . -type f ! -name SHA256SUMS -print0 | sort -z | xargs -0 sha256sum > SHA256SUMS)
  tar czf "$stage/payload.tar.gz" -C "$stage" p
  local uri="s3://$(bucket)/runs/live-payload-$id/payload.tar.gz"
  "$AWS" aws s3 cp "build/live_payload/$id/payload.tar.gz" "$uri" --only-show-errors
  log "payload $(du -h "$stage/payload.tar.gz" | cut -f1) -> $uri"
  echo "$uri"
}

# The instance-side prologue: docker + NVIDIA runtime present, images pulled and tagged with their LOCAL names, so
# live.sh runs unchanged.
prologue() {   # $1 = "sim core console" refs file contents (img ref id per line)
  cat <<EOF
set -euo pipefail
R=\$ARIADNE_RESULTS
REG=$(account).dkr.ecr.$REGION.amazonaws.com
command -v docker >/dev/null || { apt-get update -qq && apt-get install -y -qq docker.io; }
nvidia-smi > \$R/nvidia-smi.txt 2>&1 || true
docker info --format '{{json .Runtimes}}' > \$R/docker_runtimes.json
lscpu > \$R/lscpu.txt; free -m > \$R/free.txt
aws ecr get-login-password --region $REGION | docker login --username AWS --password-stdin \$REG >/dev/null
while read -r img ref id; do
  docker pull -q "\$ref" >/dev/null && docker tag "\$ref" "\$img"
  got=\$(docker image inspect "\$img" --format '{{.Id}}'); [ "\$got" = "\$id" ] || { echo "digest mismatch \$img"; exit 3; }
  echo "\$img \$ref \$got" >> \$R/images.txt
done <<'REFS'
$1
REFS
EOF
  if [ -n "${ARIADNE_RECON_GPU_URI:-}" ]; then
    cat <<EOF
command -v zstd >/dev/null || { apt-get update -qq && apt-get install -y -qq zstd; }
aws s3 cp '${ARIADNE_RECON_GPU_URI}' /tmp/ariadne-recon-gpu.tar.zst --only-show-errors
aws s3 cp '${ARIADNE_RECON_GPU_URI}.sha256' /tmp/ariadne-recon-gpu.sha256 --only-show-errors
expected=\$(awk '{print \$1}' /tmp/ariadne-recon-gpu.sha256)
echo "\$expected  /tmp/ariadne-recon-gpu.tar.zst" | sha256sum -c -
zstd -dc /tmp/ariadne-recon-gpu.tar.zst | docker load > \$R/recon_gpu_load.txt
docker tag ariadne/recon-gpu:build ariadne-recon-gpu:1
docker image inspect ariadne-recon-gpu:1 --format '{{.Id}}' > \$R/recon_gpu_image.txt
docker run --rm --gpus all ariadne-recon-gpu:1 python -c 'import torch, gsplat; from gsplat import csrc; assert torch.cuda.is_available(); print(torch.__version__, gsplat.__version__, torch.cuda.get_device_name(0), csrc.__file__)' > \$R/recon_gpu_preflight.txt
EOF
  fi
}

cmd_smoke() {
  local refs; refs="$(cmd_push)"
  local script="$REPO_ROOT/build/live_payload/smoke-$(date -u +%Y%m%dT%H%M%SZ).sh"
  mkdir -p "$(dirname "$script")"
  { prologue "$refs"; cat <<'EOF'
# [SMOKE-TEST] the sim image's headless EGL under the NVIDIA runtime (never run before: llvmpipe only, SMOKE §4).
ARIADNE_SMOKE_OUT_HOST=$R/sim-smoke ARIADNE_SMOKE_TIMEOUT=120 infra/scripts/sim.sh > $R/sim_smoke.log 2>&1; echo "sim_smoke_exit=$?" >> $R/sim_smoke.log
docker run --rm --gpus all ariadne/sim:dev bash -c 'ls -la /usr/share/glvnd/egl_vendor.d/; ldconfig -p | grep -iE "libEGL_nvidia|libnvidia-egl" | head' > $R/egl_vendor.txt 2>&1 || true
EOF
  } > "$script"
  "$REPO_ROOT/infra/scripts/spinup.sh" run --profile "$PROFILE" "${INSTANCE_ARGS[@]}" --max-minutes 25 -- "$(cat "$script")"
}

cmd_run() {
  local scene="$1" end_s="$2"
  local source_commit; source_commit="$(git -C "$REPO_ROOT" rev-parse HEAD)"
  local source_dirty; source_dirty="$([ -n "$(git -C "$REPO_ROOT" status --porcelain -- cpp configs ros2 simulator infra)" ] && echo true || echo false)"
  local refs; refs="$(cmd_push)"
  local uri="${ARIADNE_LIVE_PAYLOAD_URI:-}"
  [ -n "$uri" ] || uri="$(cmd_payload "$scene")"
  local script="$REPO_ROOT/build/live_payload/run-$(date -u +%Y%m%dT%H%M%SZ).sh"
  local knobs="" v   # live.sh's optional CPU tuning knobs, forwarded only when set here
  for v in ARIADNE_CPUSET_RT ARIADNE_CPUSET_SIM ARIADNE_CPUSET_RECON ARIADNE_CPUSET_MAIN ARIADNE_CPUSET_ESTIMATOR \
           ARIADNE_CPUSET_ESTIMATOR_0 ARIADNE_CPUSET_ESTIMATOR_1 ARIADNE_CPUSET_COLLAB ARIADNE_RT_CV_THREADS ARIADNE_WORKER_CV_THREADS; do
    if [ -n "${!v:-}" ]; then knobs+="$v=${!v} "; fi
  done
  { prologue "$refs"; cat <<EOF
export ARIADNE_GIT_COMMIT=$source_commit ARIADNE_GIT_DIRTY=$source_dirty ARIADNE_RT_MEM=${ARIADNE_RT_MEM:-6g}
aws s3 cp $uri - | tar xz -C /tmp
(cd /tmp/p && sha256sum -c --quiet SHA256SUMS) && echo "payload sha256 OK" > \$R/payload_check.txt
cp -a /tmp/p/. . && cp /tmp/p/SHA256SUMS \$R/
mkdir -p build/live/runs
if [ '${ARIADNE_RT_GPU:-0}' = 1 ]; then   # the GPU runtime image: core-dev + CUDA 12.8 runtime + cuDNN 9
  docker build -q -f infra/docker/core-gpu.Dockerfile -t ariadne/core-gpu:dev . > \$R/core_gpu_image.txt 2>&1
fi
if env ARIADNE_LOCKSTEP=${ARIADNE_LOCKSTEP:-} ARIADNE_LIVE_CONFIG=${ARIADNE_LIVE_CONFIG:-} ARIADNE_PLANNER_CONFIG=${ARIADNE_PLANNER_CONFIG:-} \
ARIADNE_PERCEPTION_CONFIG=${ARIADNE_PERCEPTION_CONFIG:-} ARIADNE_SLAM_CONFIG=${ARIADNE_SLAM_CONFIG:-} ARIADNE_IMU_NOISE_CONFIG=${ARIADNE_IMU_NOISE_CONFIG:-} ARIADNE_MISSION_CONFIG=${ARIADNE_MISSION_CONFIG:-} \
ARIADNE_SHADOW_MAP_CONFIG=${ARIADNE_SHADOW_MAP_CONFIG:-} \
ARIADNE_SPLAT=${ARIADNE_SPLAT:-0} ARIADNE_RECON_IMAGE=${ARIADNE_RECON_IMAGE:-ariadne-recon:1} \
ARIADNE_RECON_BACKEND=${ARIADNE_RECON_BACKEND:-cpu_init} ARIADNE_RECON_ITERS=${ARIADNE_RECON_ITERS:-} ARIADNE_RECON_BUDGET_MS=${ARIADNE_RECON_BUDGET_MS:-50} ARIADNE_RECON_DIAGNOSTICS=${ARIADNE_RECON_DIAGNOSTICS:-0} \
ARIADNE_RT_MEM=${ARIADNE_RT_MEM:-6g} ARIADNE_PROFILE=${ARIADNE_PROFILE:-} ARIADNE_SAMPLER=${ARIADNE_SAMPLER:-} \
ARIADNE_RT_GPU=${ARIADNE_RT_GPU:-0} \$( [ '${ARIADNE_RT_GPU:-0}' = 1 ] && echo ARIADNE_CORE_IMAGE=ariadne/core-gpu:dev ) \
$knobs  infra/scripts/live.sh run $scene build/live/runs $end_s > \$R/live_run.log 2>&1; then live_rc=0; else live_rc=\$?; fi
run=\$(find build/live/runs -mindepth 1 -maxdepth 1 -type d -name 'live-*' -printf '%T@ %p\\n' | sort -nr | head -1 | cut -d' ' -f2-)
if [ -n "\$run" ]; then
  if [ '${ARIADNE_SKIP_REPLAY:-0}' = '1' ]; then
    replay_rc=0; echo 'replay skipped by explicit live profiling setting' > \$R/replay.log
  elif infra/scripts/live.sh replay \$run $scene > \$R/replay.log 2>&1; then replay_rc=0; else replay_rc=\$?; fi
  if [ '${ARIADNE_ARCHIVE_RECORDING:-inline}' = separate ]; then
    recording=\$(find "\$run" -name inputs.mcap -type f | head -1)
    if [ -n "\$recording" ]; then
      recording_uri="\${ARIADNE_RESULTS_URI%/output/results}/recordings/inputs.mcap"
      sha256sum "\$recording" > \$R/recording.sha256
      aws s3 cp "\$recording" "\$recording_uri" --only-show-errors
      printf '%s\\n' "\$recording_uri" > \$R/recording_uri.txt
    else
      echo 'mission failed before creating a recording' > \$R/recording_error.txt
    fi
    tar czf \$R/run.tar.gz --exclude='*.sock' --exclude='inputs.mcap' \$run
  else
    tar czf \$R/run.tar.gz --exclude='*.sock' \$run
  fi
else
  replay_rc=125
  echo 'no live run directory was created' > \$R/replay.log
fi
printf 'live_exit=%s\\nreplay_exit=%s\\n' "\$live_rc" "\$replay_rc" > \$R/mission_exit_status.txt
test "\$live_rc" -eq 0 -a "\$replay_rc" -eq 0
EOF
  } > "$script"
  "$REPO_ROOT/infra/scripts/spinup.sh" run --profile "$PROFILE" "${INSTANCE_ARGS[@]}" --max-minutes "$RUN_MAX_MIN" -- "$(cat "$script")"
}

cmd_replay_s3() {   # S3_RUN_TARBALL SCENE - replay an EXISTING recording in-region (no local download; collab60d)
  # ARIADNE_REPLAY_RECORDING_URI: the run's inputs.mcap when it was archived apart (ARIADNE_ARCHIVE_RECORDING=separate);
  # ARIADNE_PROFILE / ARIADNE_REPLAY_PASSES are passed to live.sh replay.
  local tarball="$1" scene="$2" rec="${ARIADNE_REPLAY_RECORDING_URI:-}"
  local refs; refs="$(cmd_push)"
  local uri="${ARIADNE_LIVE_PAYLOAD_URI:-}"   # as cmd_run: reuse an uploaded payload across capacity retries
  [ -n "$uri" ] || uri="$(cmd_payload "$scene")"
  local script="$REPO_ROOT/build/live_payload/replays3-$(date -u +%Y%m%dT%H%M%SZ).sh"
  { prologue "$refs"; cat <<EOF
aws s3 cp $uri - | tar xz -C /tmp
(cd /tmp/p && sha256sum -c --quiet SHA256SUMS) && echo "payload sha256 OK" > \$R/payload_check.txt
cp -a /tmp/p/. .
mkdir -p /tmp/runsrc build/live/runs
aws s3 cp $tarball - | tar xz -C /tmp/runsrc
src=\$(ls -d /tmp/runsrc/build/live/runs/live-* | tail -1)
rm -rf "\$src/replay1" "\$src/replay2" "\$src"/replay*.log "\$src/replay_check.json"
if [ -n '$rec' ]; then   # beside the run's manifest, where live.sh replay looks for it
  aws s3 cp '$rec' "\$(dirname "\$(ls -d "\$src"/*/manifest.yaml | head -1)")/inputs.mcap" --only-show-errors
fi
cp -a "\$src" build/live/runs/
run=build/live/runs/\$(basename "\$src")
ARIADNE_PROFILE=${ARIADNE_PROFILE:-0} ARIADNE_REPLAY_PASSES=${ARIADNE_REPLAY_PASSES:-2} ARIADNE_REPLAY_PROFILE_ONLY=${ARIADNE_REPLAY_PROFILE_ONLY:-0} ARIADNE_RT_MEM=\${ARIADNE_RT_MEM:-20g} infra/scripts/live.sh replay "\$run" $scene > \$R/replay.log 2>&1 || echo "replay_check_failed" >> \$R/replay.log
cp "\$run/replay_check.json" \$R/ 2>/dev/null || true
tar czf \$R/replays.tar.gz "\$run"/replay1 "\$run"/replay2 "\$run"/replay*.log 2>/dev/null || true
EOF
  } > "$script"
  "$REPO_ROOT/infra/scripts/spinup.sh" run --profile "$PROFILE" "${INSTANCE_ARGS[@]}" --max-minutes "$RUN_MAX_MIN" -- "$(cat "$script")"
}

cmd_batch() {
  local jobs="$1" max_min="$2"
  [ -f "$REPO_ROOT/$jobs" ] || { log "no job file $jobs"; exit 2; }
  local scenes=(build/live/scenes/ci_tiny_0) preset seed end_s cfg
  while read -r preset seed end_s cfg; do
    { [ -z "$preset" ] || [ "${preset:0:1}" = "#" ]; } && continue
    [ -f "$REPO_ROOT/build/live/scenes/${preset}_${seed}/mission_live.sdf" ] || { log "compose ${preset} ${seed} first"; exit 2; }
    [ "$cfg" = "-" ] || [ -f "$REPO_ROOT/$cfg" ] || { log "no planner config $cfg"; exit 2; }
    [ $(( ${end_s%.*} % 5 )) = 0 ] || { log "END_S $end_s: lockstep needs a multiple of the 5 s replan"; exit 2; }
    scenes+=("build/live/scenes/${preset}_${seed}")
  done < "$REPO_ROOT/$jobs"
  local refs; refs="$(cmd_push)"
  local uri; uri="$(cmd_payload $(printf '%s\n' "${scenes[@]}" | sort -u))"
  local script="$REPO_ROOT/build/live_payload/batch-$(date -u +%Y%m%dT%H%M%SZ).sh"
  { prologue "$refs"; cat <<BATCH
aws s3 cp $uri - | tar xz -C /tmp
(cd /tmp/p && sha256sum -c --quiet SHA256SUMS) && echo "payload sha256 OK" > \$R/payload_check.txt
cp -a /tmp/p/. . && cp /tmp/p/SHA256SUMS \$R/
cp $jobs \$R/jobs.txt
BATCH
  cat <<'BATCH'
export ARIADNE_LOCKSTEP=1 ARIADNE_SHADOW_MAP_CONFIG=configs/mission/mission_v2.yaml
slim() {   # a run directory without its recording -> $R/$2.tar.gz, uploaded at once
  tar czf $R/$2.tar.gz --exclude='*.sock' --exclude='inputs.mcap' "$1"
  if [ -n "${ARIADNE_RESULTS_URI:-}" ]; then aws s3 cp $R/$2.tar.gz $ARIADNE_RESULTS_URI/$2.tar.gz --only-show-errors || true; fi
}
# 1. EGL reproducibility (lockstep is verified on llvmpipe only): the same seed twice, compared; stop if it differs.
for i in 1 2; do infra/scripts/live.sh run build/live/scenes/ci_tiny_0 build/live/repro 20 > $R/repro_run$i.log 2>&1 || true; done
if ! python3 python/analysis/lockstep_repro_check.py $(ls -d build/live/repro/live-*/ | head -1) \
     $(ls -d build/live/repro/live-*/ | tail -1) $R/repro_check.json > $R/repro_check.txt 2>&1; then
  echo "EGL repro check FAILED: batch stopped" | tee -a $R/batch.log
  slim build/live/repro repro
  exit 4
fi
echo "EGL repro check PASS" | tee -a $R/batch.log
slim build/live/repro repro
# 2. The jobs, in file order.
n=0
while read -r preset seed end_s cfg; do
  { [ -z "$preset" ] || [ "${preset:0:1}" = "#" ]; } && continue
  n=$((n + 1)); name=$(printf 'job%02d_%s_%s_%s' $n "$preset" "$seed" "$(basename "${cfg%.yaml}")")
  t0=$(date +%s)
  if [ "$cfg" = "-" ]; then unset ARIADNE_PLANNER_CONFIG; else export ARIADNE_PLANNER_CONFIG="$cfg"; fi
  rc=0
  infra/scripts/live.sh run build/live/scenes/${preset}_${seed} build/live/batch/$name $end_s > $R/$name.log 2>&1 || rc=$?
  echo "$name exit $rc wall_s $(( $(date +%s) - t0 ))" | tee -a $R/batch.log
  slim build/live/batch/$name $name
  if [ -n "${ARIADNE_RESULTS_URI:-}" ]; then aws s3 cp $R/batch.log $ARIADNE_RESULTS_URI/batch.log --only-show-errors || true; fi
done < $R/jobs.txt
BATCH
  } > "$script"
  "$REPO_ROOT/infra/scripts/spinup.sh" run --profile "$PROFILE" "${INSTANCE_ARGS[@]}" --max-minutes "$max_min" -- "$(cat "$script")"
}

case "${1:-}" in
  push) shift; cmd_push "$@" ;;
  payload) shift; cmd_payload "$@" ;;
  smoke) shift; cmd_smoke "$@" ;;
  run) shift; cmd_run "$@" ;;
  replay-s3) shift; cmd_replay_s3 "$@" ;;
  batch) shift; cmd_batch "$@" ;;
  *) sed -n '2,23p' "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac
