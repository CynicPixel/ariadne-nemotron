#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# The live closed-loop run (design docs/design/LIVE_RUNTIME_L1.md, D150-D155).
#
#   infra/scripts/live.sh build                    sim side: the mover plugin + ros2/ (colcon) in ariadne/sim
#   infra/scripts/live.sh compose PRESET SEED RTF  scene + mission_live.* -> build/live/scenes/<preset>_<seed>
#   infra/scripts/live.sh run SCENE OUT [END_S]    one live run -> OUT/<run>/  (+ sim/console logs)
#                                                  config: ARIADNE_LIVE_CONFIG (default configs/mission/live_v1.yaml)
#                                                  ARIADNE_LOCKSTEP=1: lockstep (D165), recorded in the run's config
#                                                  ARIADNE_PLANNER_CONFIG=configs/mission/mission_v2.yaml: A1's Phase 4
#                                                  planner (copied into the run as planner_config.yaml; replay uses it)
#                                                  ARIADNE_SHADOW_MAP_CONFIG=configs/mission/mission_v2.yaml: a Phase 4
#                                                  TSDF on the same keyframes -> snapshots/p4slab_<ms>.png (any planner)
#   infra/scripts/live.sh replay RUN_DIR SCENE     the recording, replayed twice, compared (replay_check.json)
#
# Three containers per run, named ariadne-live-<id>-{rt,sim,console}. Only those are ever stopped, by the exact
# container ids `docker run -d` printed for them (other sessions share the docker daemon):
#   rt       ariadne/core-dev  ariadne_live --link <run>/link.sock  (listens first)
#   sim      ariadne/sim       ros2 launch ariadne_sim live_world.launch.py: gz sim PAUSED + RosGzBridge +
#                              RuntimeLink in one component container; unpaused here once <run>/link.ready exists
#   console  ariadne-console   ARIADNE_LIVE=<run>:<scene> uvicorn, 127.0.0.1:${ARIADNE_CONSOLE_PORT:-8080};
#                              a probe client keeps a WebSocket open so frames are served (and logged) during the run
#   orch     ariadne-orchestrator  only with `chooser: llm` (AGENTIC_7 D156): the Bedrock tool loop + MCP servers,
#                              listening on <run>/chooser.sock BEFORE the runtime starts (it connects, waiting <= 60 s).
#                              Credentials: the current session exported to a mode-600 env file that is deleted as
#                              soon as docker has read it (aws.sh's pattern for Terraform); short-lived, never logged.
# Render: NVIDIA EGL when the docker NVIDIA runtime exists (the AWS g4dn host), else Mesa llvmpipe (slow; smoke only).
# Parallelism: the builds use 2 jobs (the dev host crashed above that).
# ---------------------------------------------------------------------------
set -Eeuo pipefail
trap 'echo "[live] FAILED (exit $?) at line $LINENO: $BASH_COMMAND" >&2' ERR

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SIM_IMAGE="${ARIADNE_SIM_IMAGE:-ariadne/sim:dev}"
CORE_IMAGE="${ARIADNE_CORE_IMAGE:-ariadne/core-dev:dev}"
CONSOLE_IMAGE="${ARIADNE_CONSOLE_IMAGE:-ariadne-console:1}"
ORCH_IMAGE="${ARIADNE_ORCH_IMAGE:-ariadne-orchestrator:1}"
JOBS="${ARIADNE_BUILD_JOBS:-2}"
U="$(id -u):$(id -g)"
log() { printf '[live %s] %s\n' "$(date -u +%H:%M:%S)" "$*" >&2; }

render_args() {
  if docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q '"nvidia"'; then
    echo "--gpus all -e ARIADNE_RENDER_MODE=nvidia-egl"
  else
    echo "-e ARIADNE_RENDER_MODE=mesa-llvmpipe -e LIBGL_ALWAYS_SOFTWARE=1 -e GALLIUM_DRIVER=llvmpipe"
  fi
}

sim_run() {   # one-shot command in the sim image with ROS sourced, repo at /work
  docker run --rm -u "$U" -v "$REPO_ROOT":/work -w /work "$SIM_IMAGE" bash -c "source /opt/ros/jazzy/setup.bash && $*"
}

cmd_build() {
  log "sim plugins (mover live mode) -> build/sim-plugins"
  sim_run "cmake -S simulator/plugins -B build/sim-plugins -DCMAKE_BUILD_TYPE=Release >/dev/null && \
           cmake --build build/sim-plugins --parallel $JOBS"
  log "ros2 packages -> build/ros2/install"
  sim_run "colcon --log-base build/ros2/log build --base-paths ros2 --build-base build/ros2/build \
           --install-base build/ros2/install --parallel-workers $JOBS --event-handlers console_direct- \
           --cmake-args -DCMAKE_BUILD_TYPE=Release"
}

cmd_compose() {
  local preset="$1" seed="$2" rtf="$3" out="build/live/scenes/${1}_${2}"
  log "compose $preset seed $seed (real-time factor $rtf) -> $out"
  sim_run "cd python && python3 -m dataset_tools.worldgen generate --preset ../configs/worldgen/$preset.yaml \
           --seed $seed --out ../$out --rig ../${ARIADNE_COMPOSE_RIG:-configs/sim/rig_stereo_v1.yaml} --live-rtf $rtf"
}

cmd_demo_mission() {   # P8 D216: recompose mission_live.* only, for the user's chosen entrances and target
  local preset="$1" seed="$2" agents="${3:-}" target="${4:-}" rtf="${5:-1}" out="build/live/scenes/${1}_${2}"
  log "demo mission $preset seed $seed agents ${agents:-all} target ${target:-generated} -> $out"
  sim_run "cd python && python3 -m dataset_tools.worldgen demo-mission --preset ../configs/worldgen/$preset.yaml \
           --seed $seed --out ../$out --rig ../${ARIADNE_COMPOSE_RIG:-configs/sim/rig_stereo_v1.yaml} --live-rtf $rtf \
           ${agents:+--agents $agents} ${target:+--target-xy $target}"
}

CONTAINERS=()
stop_mine() {
  local c
  for c in "${CONTAINERS[@]}"; do docker stop -t 10 "$c" >/dev/null 2>&1 || true; done
  [ -z "$CRED_PID" ] || kill "$CRED_PID" 2>/dev/null || true
  [ -z "$CRED_DIR" ] || rm -rf "$CRED_DIR"
}

CRED_DIR="" CRED_PID=""
start_orchestrator() {   # -> appends its container id; returns once <run>/chooser.sock exists
  local id="$1" run="$2" awscli=(aws)
  command -v aws >/dev/null || awscli=("$REPO_ROOT/infra/scripts/aws.sh" aws)   # the dev host has no aws install
  # Credentials (2026-10-23: a one-time export of the session's keys expired ~15 min into a run and the orchestrator
  # died): the session is re-exported every 4 min into a mode-700 directory; the container reads it through a
  # credential_process profile, so botocore refreshes before each expiry. Keys never reach a command line or a log.
  CRED_DIR="$(mktemp -d)"; chmod 700 "$CRED_DIR"
  if ! "${awscli[@]}" configure export-credentials --format process > "$CRED_DIR/creds.json" 2>/dev/null; then
    rm -rf "$CRED_DIR"; log "no AWS credentials for Bedrock (infra/scripts/aws.sh login)"; exit 1
  fi
  printf '[default]\ncredential_process = cat /creds/creds.json\nregion = %s\n' "${AWS_REGION:-eu-north-1}" > "$CRED_DIR/config"
  ( while sleep 240; do
      "${awscli[@]}" configure export-credentials --format process > "$CRED_DIR/creds.tmp" 2>/dev/null \
        && mv "$CRED_DIR/creds.tmp" "$CRED_DIR/creds.json"
    done ) &
  CRED_PID=$!
  CONTAINERS+=("$(docker run -d --name "ariadne-live-$id-orch" --memory=1g --memory-swap=1g -u "$U" -e HOME=/tmp \
    -e AWS_CONFIG_FILE=/creds/config -v "$CRED_DIR":/creds:ro -v "$REPO_ROOT":/work -w /work/python "$ORCH_IMAGE" \
    python -m orchestration.orchestrator --socket "/work/$run/chooser.sock" --dir "/work/$run/decisions")")
  orch="${CONTAINERS[-1]}"
  local t0; t0=$(date +%s)
  until [ -S "$REPO_ROOT/$run/chooser.sock" ]; do
    docker inspect -f '{{.State.Running}}' "$orch" | grep -q true || { log "orchestrator exited"; docker logs "$orch" 2>&1 | tail -20 >&2; exit 1; }
    [ $(( $(date +%s) - t0 )) -lt 60 ] || { log "orchestrator not listening after 60 s"; exit 1; }
    sleep 1
  done
  log "orchestrator: $(docker logs "$orch" 2>&1 | tail -1)"
}

cmd_run() {
  local scene="$1" out="$2" end_s="${3:-}"
  [ -f "$REPO_ROOT/$scene/mission_live.sdf" ] || { log "no $scene/mission_live.sdf (live.sh compose)"; exit 2; }
  local id; id="$(date -u +%Y%m%dT%H%M%SZ)-$(head -c2 /dev/urandom | od -An -tx1 | tr -d ' \n')"
  local run="$out/live-$id"
  mkdir -p "$REPO_ROOT/$run"
  local live_cfg="${ARIADNE_LIVE_CONFIG:-configs/mission/live_v1.yaml}" rt sim con orch="" recon=""
  local perception_cfg="${ARIADNE_PERCEPTION_CONFIG:-configs/perception/sim_v1.yaml}"
  local slam_cfg="${ARIADNE_SLAM_CONFIG:-configs/slam/sim_v1.yaml}"
  local imu_noise_cfg="${ARIADNE_IMU_NOISE_CONFIG:-}"
  local mission_cfg="${ARIADNE_MISSION_CONFIG:-configs/mission/mission_v1.yaml}"
  # The config this run used, always recorded beside it: replay reads it (chooser, gate, end_s).
  sed -e "${end_s:+s/^end_s: .*/end_s: $end_s/}" -e "${ARIADNE_LOCKSTEP:+s/^lockstep: .*/lockstep: true/}" \
    "$REPO_ROOT/$live_cfg" > "$REPO_ROOT/$run/live_config.yaml"
  live_cfg="$run/live_config.yaml"
  cp "$REPO_ROOT/$perception_cfg" "$REPO_ROOT/$run/perception_config.yaml"
  cp "$REPO_ROOT/$slam_cfg" "$REPO_ROOT/$run/slam_config.yaml"
  cp "$REPO_ROOT/$mission_cfg" "$REPO_ROOT/$run/mission_config.yaml"
  perception_cfg="$run/perception_config.yaml"
  slam_cfg="$run/slam_config.yaml"
  local imu_noise_arg=()
  if [ -n "$imu_noise_cfg" ]; then
    cp "$REPO_ROOT/$imu_noise_cfg" "$REPO_ROOT/$run/imu_noise_config.yaml"
    imu_noise_arg=(--imu-noise-config "$run/imu_noise_config.yaml")
  fi
  mission_cfg="$run/mission_config.yaml"
  local planner=()
  if [ -n "${ARIADNE_PLANNER_CONFIG:-}" ]; then
    cp "$REPO_ROOT/$ARIADNE_PLANNER_CONFIG" "$REPO_ROOT/$run/planner_config.yaml"
    planner=(--planner-config "$run/planner_config.yaml")
  fi
  if [ -n "${ARIADNE_SHADOW_MAP_CONFIG:-}" ]; then   # one map for every row (snapshots/p4slab_<ms>.png)
    cp "$REPO_ROOT/$ARIADNE_SHADOW_MAP_CONFIG" "$REPO_ROOT/$run/shadow_map_config.yaml"
    planner+=(--shadow-map-config "$run/shadow_map_config.yaml")
  fi
  {
    printf 'source_perception: %s\n' "${ARIADNE_PERCEPTION_CONFIG:-configs/perception/sim_v1.yaml}"
    printf 'source_slam: %s\n' "${ARIADNE_SLAM_CONFIG:-configs/slam/sim_v1.yaml}"
    printf 'source_mission: %s\n' "${ARIADNE_MISSION_CONFIG:-configs/mission/mission_v1.yaml}"
    printf 'source_live: %s\n' "${ARIADNE_LIVE_CONFIG:-configs/mission/live_v1.yaml}"
    sha256sum "$REPO_ROOT/$run/perception_config.yaml" "$REPO_ROOT/$run/slam_config.yaml" \
      "$REPO_ROOT/$run/mission_config.yaml" "$REPO_ROOT/$run/live_config.yaml"
    [ ! -f "$REPO_ROOT/$run/imu_noise_config.yaml" ] || sha256sum "$REPO_ROOT/$run/imu_noise_config.yaml"
    for cfg in planner_config.yaml shadow_map_config.yaml collab_config.yaml; do
      [ ! -f "$REPO_ROOT/$run/$cfg" ] || sha256sum "$REPO_ROOT/$run/$cfg"
    done
    [ ! -f "$REPO_ROOT/${ARIADNE_COLLAB_CONFIG:-configs/collab/collab_live_v1.yaml}" ] || \
      sha256sum "$REPO_ROOT/${ARIADNE_COLLAB_CONFIG:-configs/collab/collab_live_v1.yaml}"
    for model in .cache/models/eigenplaces_sim_ref/eigenplaces_r50_256_640x360.onnx \
      .cache/models/aliked_lg_sim_ref/aliked_n16_k1024_1280x720.onnx \
      .cache/models/aliked_lg_sim_ref/lightglue_aliked.onnx; do
      [ ! -f "$REPO_ROOT/$model" ] || sha256sum "$REPO_ROOT/$model"
    done
  } > "$REPO_ROOT/$run/config_manifest.txt"
  trap 'stop_mine' EXIT
  local world; world="$(grep -o '<world name="[^"]*"' "$REPO_ROOT/$scene/mission_live.sdf" | head -1 | cut -d'"' -f2)"
  log "run $id: scene $scene, world '$world', end ${end_s:-config} s, config $live_cfg, render: $(render_args | cut -d' ' -f1-2)"

  # 0. chooser llm: the orchestrator listens first.
  local chooser=()
  if grep -q '^chooser: *llm' "$REPO_ROOT/$live_cfg"; then
    start_orchestrator "$id" "$run"
    chooser=(--chooser-socket "$run/chooser.sock")
  fi

  # 0b. The splat recon service (A2, D231; tee D220), only with ARIADNE_SPLAT=1: it LISTENS before the runtime
  # connects. cpu_init backend (numpy/scipy/cv2 - A2's ariadne-recon image, never the console image). The tee is a
  # silent no-op when this did not start, so gate runs are untouched.
  local recon_env=()
  # A pre-set ARIADNE_RECON_SOCK (container path) reaches the runtime as-is: the mock/test path - A2's
  # service-side mock and my tee smoke both listen outside this script.
  [ -n "${ARIADNE_RECON_SOCK:-}" ] && recon_env=(-e ARIADNE_RECON_SOCK="$ARIADNE_RECON_SOCK")
  if [ "${ARIADNE_SPLAT:-0}" = "1" ]; then
    local recon_backend="${ARIADNE_RECON_BACKEND:-cpu_init}"
    local recon_gpu=() recon_train=()
    if [ "$recon_backend" = gpu ]; then
      recon_gpu=(--gpus all)
      if [ -n "${ARIADNE_RECON_ITERS:-}" ]; then
        recon_train=(--iters "$ARIADNE_RECON_ITERS")
      else
        recon_train=(--budget-ms "${ARIADNE_RECON_BUDGET_MS:-50}")
      fi
      [ "${ARIADNE_RECON_DIAGNOSTICS:-0}" != 1 ] || recon_train+=(--diagnostics)
    elif [ "$recon_backend" != cpu_init ]; then
      log "unsupported reconstruction backend: $recon_backend"; exit 2
    fi
    CONTAINERS+=("$(docker run -d --name "ariadne-live-$id-recon" "${recon_gpu[@]}" \
      --memory="${ARIADNE_RECON_MEM:-4g}" --memory-swap="${ARIADNE_RECON_MEM:-4g}" \
      -u "$U" -e HOME=/tmp -e OMP_NUM_THREADS=2 -e MKL_NUM_THREADS=2 \
      -v "$REPO_ROOT":/work -w /work/python "${ARIADNE_RECON_IMAGE:-ariadne-recon:1}" \
      python -m ariadne_recon.live.service --listen "/work/$run/recon.sock" --out "/work/$run/splat" \
      --scene "/work/$scene" --backend "$recon_backend" --training-image rgb "${recon_train[@]}")")
    recon="${CONTAINERS[-1]}"
    local t0r; t0r=$(date +%s)
    until [ -S "$REPO_ROOT/$run/recon.sock" ]; do
      docker inspect -f '{{.State.Running}}' "$recon" | grep -q true || {
        docker logs "$recon" > "$REPO_ROOT/$run/recon.log" 2>&1; log 'reconstruction service exited during startup'; exit 1;
      }
      [ $(( $(date +%s) - t0r )) -lt 120 ] || { log 'reconstruction socket startup timed out'; exit 1; }
      sleep 1
    done
    recon_env=(-e ARIADNE_RECON_SOCK="/work/$run/recon.sock")
  fi

  # 1. The runtime listens first.
  local runtime_commit="${ARIADNE_GIT_COMMIT:-$(git -C "$REPO_ROOT" rev-parse HEAD 2>/dev/null || echo "payload:$(basename "$REPO_ROOT")")}"
  local runtime_dirty="${ARIADNE_GIT_DIRTY:-$([ -n "$(git -C "$REPO_ROOT" status --porcelain -- cpp configs ros2 simulator 2>/dev/null)" ] && echo true || echo false)}"
  local runtime_image; runtime_image="$(docker image inspect "$CORE_IMAGE" --format '{{.Id}}')"
  python3 "$REPO_ROOT/infra/scripts/live_identity.py" record --run "$REPO_ROOT/$run" \
    --binary "$REPO_ROOT/build/docker/cpp/runtime/ariadne_live" --image "$runtime_image" \
    --commit "$runtime_commit" --dirty "$runtime_dirty"
  CONTAINERS+=("$(docker run -d --name "ariadne-live-$id-rt" --memory="${ARIADNE_RT_MEM:-6g}" --memory-swap="${ARIADNE_RT_MEM:-6g}" -u "$U" -v "$REPO_ROOT":/work -w /work "${recon_env[@]}" \
    -e ARIADNE_GIT_COMMIT="$runtime_commit" \
    -e ARIADNE_GIT_DIRTY="$runtime_dirty" \
    -e ARIADNE_IMAGE_DIGEST="$runtime_image" \
    "$CORE_IMAGE" build/docker/cpp/runtime/ariadne_live --scene "$scene" \
      --perception-config "$perception_cfg" --slam-config "$slam_cfg" "${imu_noise_arg[@]}" \
      --mission-config "$mission_cfg" --live-config "$live_cfg" \
      --out "$run" --link "$run/link.sock" "${chooser[@]}" "${planner[@]}")")
  rt="${CONTAINERS[-1]}"

  # 2. gz (paused) + bridge + RuntimeLink.
  local latest_stereo=0
  if grep -q '^mode: *demo' "$REPO_ROOT/$live_cfg"; then latest_stereo=1; fi
  # shellcheck disable=SC2046
  CONTAINERS+=("$(docker run -d --name "ariadne-live-$id-sim" -u "$U" $(render_args) -v "$REPO_ROOT":/work -w /work \
    -e GZ_SIM_SYSTEM_PLUGIN_PATH=/work/build/sim-plugins -e HOME=/tmp -e ARIADNE_LIVE_LATEST="$latest_stereo" \
    -e GZ_TRANSPORT_RCVHWM=0 -e GZ_TRANSPORT_SNDHWM=0 \
    "$SIM_IMAGE" bash -c "source /opt/ros/jazzy/setup.bash && source build/ros2/install/setup.bash && \
      ros2 launch ariadne_sim live_world.launch.py scene_dir:=/work/$scene run_dir:=/work/$run")")
  sim="${CONTAINERS[-1]}"

  # 3. Unpause once every topic is matched end to end.
  local t0; t0=$(date +%s)
  until [ -f "$REPO_ROOT/$run/link.ready" ]; do
    [ $(( $(date +%s) - t0 )) -lt 300 ] || { log "link not ready after 300 s"; docker logs "$sim" 2>&1 | tail -40 >&2; exit 1; }
    docker inspect -f '{{.State.Running}}' "$sim" | grep -q true || { log "sim container exited"; docker logs "$sim" 2>&1 | tail -40 >&2; exit 1; }
    sleep 2
  done
  log "link ready: $(cat "$REPO_ROOT/$run/link.ready")"
  # D167 timers: gz's achieved RTF straight from the world stats topic, and the GL renderer string (D155: on g4dn
  # this must name the NVIDIA device; llvmpipe here means the GPU was NOT used).
  # The tap writes a start marker first, so an exec that never ran is distinguishable from a silent gz topic.
  docker exec -d "$sim" sh -c "echo \"tap started $(date -u +%H:%M:%S)\" > /work/$run/gz_stats.err; . /opt/ros/jazzy/setup.sh; exec gz topic -e -t /world/$world/stats --json-output >> /work/$run/gz_stats.jsonl 2>> /work/$run/gz_stats.err" \
    || echo "docker exec for the stats tap failed" > "$REPO_ROOT/$run/gz_stats.err"
  docker exec "$sim" sh -c ". /opt/ros/jazzy/setup.sh; timeout 3 gz topic -l" > "$REPO_ROOT/$run/gz_topics.txt" 2>&1 || true
  for _ in 1 2 3 4 5 6; do   # ogre2.log appears when the render engine initializes: the probe raced it on g4dn (A1)
    if docker exec "$sim" sh -c "grep -m4 -i 'GL_RENDERER\|GL_VENDOR\|Device Name\|RenderSystem Name' /tmp/.gz/rendering/ogre2.log 2>/dev/null" \
         > "$REPO_ROOT/$run/renderer.txt" 2>&1 && [ -s "$REPO_ROOT/$run/renderer.txt" ]; then break; fi
    echo "renderer: ogre2.log not found at probe time" > "$REPO_ROOT/$run/renderer.txt"
    sleep 5
  done
  log "renderer: $(head -1 "$REPO_ROOT/$run/renderer.txt" 2>/dev/null)"
  if grep -q '^lockstep: *true' "$REPO_ROOT/$live_cfg"; then
    log "lockstep (D165): the runtime runs the world barrier to barrier"
  else
    docker exec "$sim" bash -c "source /opt/ros/jazzy/setup.bash && gz service -s /world/$world/control \
      --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 10000 --req 'pause: false'" >&2
    log "world running"
  fi

  # 4. Console, fed live, with a probe client so frames are served during the run.
  local port="${ARIADNE_CONSOLE_PORT:-8080}"
  CONTAINERS+=("$(docker run -d --name "ariadne-live-$id-console" -u "$U" -v "$REPO_ROOT":/work -w /work/python \
    -p "127.0.0.1:$port:8080" -e ARIADNE_LIVE="/work/$run:/work/$scene" \
    -e ARIADNE_SPLAT_DIR="/work/$run/splat" -e ARIADNE_SCENE_DIR="/work/$scene" "$CONSOLE_IMAGE" \
    uvicorn operator_console.server:app --host 0.0.0.0 --port 8080 --log-level warning)")
  con="${CONTAINERS[-1]}"
  docker exec -d "$con" python3 -c "
import asyncio, websockets
async def main():
    await asyncio.sleep(3)
    async with websockets.connect('ws://127.0.0.1:8080/ws/telemetry') as ws:
        async for _ in ws: pass
asyncio.run(main())" || log "console probe client not started (websockets missing?)"

  # 5. Wait for the runtime (it ends at end_s of sim time), then stop the rest.
  # D224: an external sampler (A2's profiling) beside the run - started after the containers, stopped with them.
  SAMPLER_PID=""
  if [ -n "${ARIADNE_SAMPLER:-}" ]; then
    bash "$ARIADNE_SAMPLER" "$REPO_ROOT/$run/sampler" > "$REPO_ROOT/$run/sampler.log" 2>&1 &
    SAMPLER_PID=$!
  fi
  local rc; rc="$(docker wait "$rt")"
  docker logs "$rt" > "$REPO_ROOT/$run/runtime.log" 2>&1 || true
  docker logs "$sim" > "$REPO_ROOT/$run/sim.log" 2>&1 || true
  if [ -n "$recon" ]; then
    local recon_rc
    recon_rc="$(timeout "${ARIADNE_RECON_DRAIN_S:-300}" docker wait "$recon")" || recon_rc=124
    docker logs "$recon" > "$REPO_ROOT/$run/recon.log" 2>&1 || true
    [ "$recon_rc" = 0 ] || { log "reconstruction exit $recon_rc"; rc=1; }
  fi
  sleep 3   # the console's last poll sees results.yaml and ends its stream
  docker logs "$con" > "$REPO_ROOT/$run/console.log" 2>&1 || true
  [ -z "$orch" ] || docker logs "$orch" > "$REPO_ROOT/$run/orchestrator.log" 2>&1 || true
  printf '%s\n' "${CONTAINERS[@]}" > "$REPO_ROOT/$run/container_ids.txt"
  [ -z "${SAMPLER_PID:-}" ] || kill "$SAMPLER_PID" 2>/dev/null || true
  stop_mine
  for c in "${CONTAINERS[@]}"; do docker rm "$c" >/dev/null 2>&1 || true; done
  CONTAINERS=()
  log "runtime exit $rc; run dir $run"
  [ "$rc" = 0 ]
}

cmd_replay() {
  local run="$1" scene="$2" i
  local cfg; cfg="$( [ -f "$REPO_ROOT/$run/live_config.yaml" ] && echo "$run/live_config.yaml" || echo configs/mission/live_v1.yaml)"
  local perception_cfg="$run/perception_config.yaml" slam_cfg="$run/slam_config.yaml" mission_cfg="$run/mission_config.yaml"
  [ -f "$REPO_ROOT/$perception_cfg" ] || perception_cfg="configs/perception/sim_v1.yaml"
  [ -f "$REPO_ROOT/$slam_cfg" ] || slam_cfg="configs/slam/sim_v1.yaml"
  [ -f "$REPO_ROOT/$mission_cfg" ] || mission_cfg="configs/mission/mission_v1.yaml"
  local imu_noise_arg=()
  if [ -f "$REPO_ROOT/$run/imu_noise_config.yaml" ]; then imu_noise_arg=(--imu-noise-config "$run/imu_noise_config.yaml"); fi
  local live_dir; live_dir="$(cd "$REPO_ROOT/$run" && ls -d */inputs.mcap | head -1 | xargs dirname)"
  # The recorded inputs beyond the sensors: operator directives always; the LLM's choices when chooser is llm (the
  # model is never called in replay, AGENTIC_7 D161).
  local extra=(--directives "$run/$live_dir/directives.jsonl")
  grep -q '^chooser: *llm' "$REPO_ROOT/$cfg" && extra+=(--decisions "$run/$live_dir/trace.jsonl")
  [ -f "$REPO_ROOT/$run/planner_config.yaml" ] && extra+=(--planner-config "$run/planner_config.yaml")
  [ -f "$REPO_ROOT/$run/shadow_map_config.yaml" ] && extra+=(--shadow-map-config "$run/shadow_map_config.yaml")
  python3 "$REPO_ROOT/infra/scripts/live_identity.py" replay --run "$REPO_ROOT/$run" \
    --binary "$REPO_ROOT/build/docker/cpp/runtime/ariadne_live" \
    --image "$(docker image inspect "$CORE_IMAGE" --format '{{.Id}}')"
  for i in 1 2; do
    docker run --rm --memory="${ARIADNE_RT_MEM:-4g}" --memory-swap="${ARIADNE_RT_MEM:-4g}" -u "$U" -v "$REPO_ROOT":/work -w /work \
      --env-file "$REPO_ROOT/$run/replay_identity.env" "$CORE_IMAGE" build/docker/cpp/runtime/ariadne_live \
      --scene "$scene" --perception-config "$perception_cfg" --slam-config "$slam_cfg" "${imu_noise_arg[@]}" \
      --mission-config "$mission_cfg" --live-config "$cfg" \
      --out "$run/replay$i" --replay "$run/$live_dir/inputs.mcap" "${extra[@]}" \
      > "$REPO_ROOT/$run/replay$i.log" 2>&1
  done
  python3 "$REPO_ROOT/python/analysis/live_replay_check.py" "$REPO_ROOT/$run"
}

case "${1:-}" in
  build) shift; cmd_build "$@" ;;
  compose) shift; cmd_compose "$@" ;;
  demo-mission) shift; cmd_demo_mission "$@" ;;
  run) shift; cmd_run "$@" ;;
  replay) shift; cmd_replay "$@" ;;
  *) sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac
