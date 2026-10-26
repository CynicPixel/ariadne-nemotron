#!/usr/bin/env bash
# 1500 simulated seconds through the direct, non-lockstep, multi-agent demo path.
# Run on the g4dn GPU runner after the existing GPU slot is free.
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
unset ARIADNE_LOCKSTEP
export LIVE_AWS_PROFILE=gpu
export LIVE_AWS_MAX_MIN=240
export ARIADNE_ROOT_GB=240
export ARIADNE_LIVE_CONFIG=configs/mission/live_demo_final_profile_1500.yaml
export ARIADNE_PERCEPTION_CONFIG=configs/perception/sim_v1_rad_flat.yaml
export ARIADNE_SLAM_CONFIG=configs/slam/sim_v2_loops.yaml
export ARIADNE_IMU_NOISE_CONFIG=configs/sim/imu_noise_v1.yaml
export ARIADNE_PLANNER_CONFIG=configs/mission/mission_v2_direct1500.yaml
export ARIADNE_SHADOW_MAP_CONFIG=configs/mission/mission_v2_direct1500.yaml
export ARIADNE_MISSION_CONFIG=configs/mission/mission_v1.yaml
export ARIADNE_SAMPLER=infra/eval/direct_demo_profile_sampler.sh
export ARIADNE_SPLAT=1
export ARIADNE_RECON_IMAGE=ariadne-recon-gpu:1
export ARIADNE_RECON_BACKEND=gpu
export ARIADNE_RECON_BUDGET_MS=50
export ARIADNE_RECON_MEM=4g
export ARIADNE_RT_MEM=10g
export ARIADNE_RECON_DRAIN_S=300
export ARIADNE_ARCHIVE_RECORDING=separate
export ARIADNE_SKIP_REPLAY=1
export ARIADNE_RECON_GPU_URI=s3://ariadne-836505524290-eun1/runs/images/ariadne-recon-gpu-a827de5514fe.tar.zst
export ARIADNE_RUNNER_JSON="$ROOT/build/live_payload/runner.json"
SCENE=build/live/scenes/maze_short_6
if [[ ! -f "$SCENE/mission_live.sdf" ]] || ! rg -q '<real_time_factor>0(\.0+)?</real_time_factor>' "$SCENE/mission_live.sdf"; then
  infra/scripts/live.sh compose maze_short 6 0.0
fi
if ! rg -q '<real_time_factor>0(\.0+)?</real_time_factor>' "$SCENE/mission_live.sdf"; then
  echo "scene is not composed unthrottled (RTF 0): $SCENE/mission_live.sdf" >&2
  exit 2
fi
PAYLOAD_URI="${ARIADNE_LIVE_PAYLOAD_URI:-}"
if [[ -z "$PAYLOAD_URI" ]]; then
  PAYLOAD_URI="$(infra/scripts/live_aws.sh payload "$SCENE")"
fi
export ARIADNE_LIVE_PAYLOAD_URI="$PAYLOAD_URI"
EVIDENCE=evaluation/runs/direct_demo_profile_1500
mkdir -p "$EVIDENCE"
printf 'payload_uri: %s\n' "$PAYLOAD_URI" > "$EVIDENCE/staged_payload.txt"
rc=0
infra/scripts/live_aws.sh run "$SCENE" 1500 || rc=$?
latest="$(find build/runs -mindepth 1 -maxdepth 1 -type d -name '*-gpu-*' -printf '%T@ %p\n' | sort -nr | head -1 | cut -d' ' -f2-)"
if [[ -n "$latest" && -d "$latest/output" ]]; then
  mkdir -p "$EVIDENCE/aws_output"
  cp -a "$latest/output/." "$EVIDENCE/aws_output/"
  printf '%s\n' "$latest" > "$EVIDENCE/aws_run_dir.txt"
fi
exit "$rc"
