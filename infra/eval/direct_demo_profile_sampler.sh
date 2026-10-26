#!/usr/bin/env bash
# Host-side sampler for the 1500 s live demo profile. Keeps CPU/thread/perf samples
# from the shared project sampler and adds GPU, disk, and memory snapshots.
set -Eeuo pipefail
OUT="${1:?usage: direct_demo_profile_sampler.sh OUTDIR}"
mkdir -p "$OUT"
SAMPLER_PERF_ARGS='-a -F 99' infra/eval/perf_sampler.sh "$OUT/system" &
CPU_PID=$!
GPU_PID="" IO_PID="" NET_PID=""
stop() {
  trap - EXIT TERM INT
  for p in "$GPU_PID" "$IO_PID" "$NET_PID" "$CPU_PID"; do
    [ -z "$p" ] || kill -TERM "$p" 2>/dev/null || true
  done
  for p in "$GPU_PID" "$IO_PID" "$NET_PID" "$CPU_PID"; do
    [ -z "$p" ] || wait "$p" 2>/dev/null || true
  done
}
trap stop EXIT TERM INT
for _ in $(seq 1 30); do
  command -v iostat >/dev/null && command -v sar >/dev/null && break
  sleep 1
done
if command -v iostat >/dev/null; then iostat -xz 1 > "$OUT/iostat.txt" 2>&1 & IO_PID=$!; fi
if command -v sar >/dev/null; then sar -n DEV 1 > "$OUT/network.txt" 2>&1 & NET_PID=$!; fi
(
  while :; do
    printf '\n== %s ==\n' "$(date -u +%FT%TZ)"
    nvidia-smi --query-gpu=utilization.gpu,utilization.memory,memory.used,power.draw --format=csv,noheader 2>&1 || true
    df -h / | tail -1
    free -m | awk 'NR==2 {print "memory_MiB used=" $3 " available=" $7}'
    sleep 2
  done
) > "$OUT/gpu_disk_mem.txt" 2>&1 &
GPU_PID=$!
wait "$CPU_PID"
