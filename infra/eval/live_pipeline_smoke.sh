#!/usr/bin/env bash
# Actual free-running two-agent integration gate; no replay substituted for live.
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
END_S="${1:-30}"
[[ "$END_S" =~ ^[1-9][0-9]*$ ]] && (( END_S <= 200 )) || { echo 'smoke horizon must be 1..200 simulated seconds' >&2; exit 2; }
git diff --quiet HEAD -- cpp configs infra ros2 python/reconstruction tests || { echo 'commit code/config changes before launching' >&2; exit 2; }
export LIVE_AWS_PROFILE=gpu LIVE_AWS_MAX_MIN=45 ARIADNE_ROOT_GB=240
# Approved Stockholm G/VT Spot quota: 8 vCPUs (AWS support, 2026-10-27).
export LIVE_AWS_INSTANCE_TYPE="${LIVE_AWS_INSTANCE_TYPE:-g4dn.2xlarge}"
export SPINUP_AVOID_AZ=eu-north-1c
export ARIADNE_RUNNER_JSON="$ROOT/build/live_payload/runner.json"
unset ARIADNE_LOCKSTEP ARIADNE_RECON_ITERS ARIADNE_LIVE_PAYLOAD_URI
export ARIADNE_LIVE_CONFIG=configs/mission/live_demo_final_profile_1500.yaml
export ARIADNE_PERCEPTION_CONFIG=configs/perception/sim_v1_rad_flat.yaml
export ARIADNE_SLAM_CONFIG=configs/slam/sim_v2_loops.yaml
export ARIADNE_IMU_NOISE_CONFIG=configs/sim/imu_noise_v1.yaml
export ARIADNE_MISSION_CONFIG=configs/mission/mission_v1.yaml
export ARIADNE_PLANNER_CONFIG=configs/mission/mission_v2_direct1500.yaml
export ARIADNE_SPLAT=1 ARIADNE_RECON_BACKEND=gpu ARIADNE_RECON_IMAGE=ariadne-recon-gpu:1
export ARIADNE_RECON_BUDGET_MS=50 ARIADNE_RECON_MEM=4g ARIADNE_RT_MEM=10g
export ARIADNE_RECON_GPU_URI=s3://ariadne-836505524290-eun1/runs/images/ariadne-recon-gpu-a827de5514fe.tar.zst
export ARIADNE_RECON_DRAIN_S=60 ARIADNE_SKIP_REPLAY=1 ARIADNE_ARCHIVE_RECORDING=separate
export ARIADNE_PROFILE=1
export ARIADNE_SAMPLER=infra/eval/direct_demo_profile_sampler.sh
SCENE=build/live/scenes/maze_short_6
for f in "$ARIADNE_PERCEPTION_CONFIG" "$ARIADNE_SLAM_CONFIG" "$ARIADNE_IMU_NOISE_CONFIG" \
         build/docker/cpp/runtime/ariadne_live build/docker/cpp/runtime/ariadne_collab_worker; do test -f "$f"; done
rg -q '<real_time_factor>0(\.0+)?</real_time_factor>' "$SCENE/mission_live.sdf"
mkdir -p evaluation/runs/live_pipeline_final
infra/scripts/live_aws.sh run "$SCENE" "$END_S"
