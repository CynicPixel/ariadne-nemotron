# F5 live GPU recording runbook (FM)

Sources:
- upstream M (live path, 2026-10-28 messages);
- FM memory gotchas from the 2026-10-27 console runs.

Every step marked **check** must be confirmed in the logs before recording counts.

## 0. Preconditions

- **Base line:** the fork contains **`ce351fb`** (integ) / `3dad564` (perf). This fixes ORT CUDA silently falling back to CPU.
  - Cause: `Session(Env(), path, Options(true))` appended the CUDA provider before `Ort::Env` existed; argument evaluation order is unspecified.
  - Symptom without the fix: every ALIKED/LightGlue session logs "Attempt to use DefaultLogger but none has been registered" and runs on CPU.
- **Clearances:** the user approves the AWS spend in-session; M clears the GPU launch (the spot quota is shared, 8 vCPU).
- **Instance:** **g4dn.xlarge** (T4, 4 vCPU, 16 GB). Only this size reliably launches in eu-north-1; g5.2xlarge / g4dn.2xlarge spot capacity scores 1/10.
- **Key:** the Nebius key is passed at run time only. Never put it in the worktree DIR (spinup tars DIR into S3), the payload, a job file or a commit.

## 1. ORT build and payload staging (local)

```bash
# configure + build in ariadne/core-dev:dev, worktree at /work
cmake -S /work -B /work/build/docker-ort -DCMAKE_BUILD_TYPE=Release -DARIADNE_WITH_ORT=ON \
      -DFETCHCONTENT_BASE_DIR=/work/build/fetchcontent-ort      # pulls onnxruntime-linux-x64-gpu_cuda12-1.31.0.tgz
cmake --build /work/build/docker-ort --target ariadne_live ariadne_collab_worker
# test targets: gtest discovery runs the binaries ->
#   LD_LIBRARY_PATH=/work/build/fetchcontent-ort/onnxruntime-src/lib
```

Stage what the `live_aws.sh` payload reads:
- Both binaries go to `build/docker/cpp/runtime/`.
- `build/ort/lib/` holds `libonnxruntime.so*`, `libonnxruntime_providers_cuda.so` and `libonnxruntime_providers_shared.so`, from
  `build/fetchcontent-ort/onnxruntime-src/lib`. A symlink is fine, because the payload uses `tar -h`. The payload includes `build/ort/lib` only
  if that directory exists (`live_aws.sh:81-84`).
- `python/operator_console/static/dist` must be built locally (it is gitignored, so the snapshot otherwise lacks `main.js`).

## 2. Run environment

| var | value | why |
|---|---|---|
| `ARIADNE_RT_GPU` | `1` | gives the runtime container the GPU (core-gpu image = core-dev + CUDA 12.8 runtime + cuDNN 9, built on the instance from `infra/docker/core-gpu.Dockerfile`) |
| `ARIADNE_LIVE_CONFIG` | `configs/mission/live_demo_final_profile_1500_ortcuda.yaml`, or the fork's Nemotron variant of it | `collab.dnn_backend ort_cuda` |
| `ARIADNE_RT_MEM` | **`10g`** | smoke run 7b34cf was OOM-killed (137) at 6g ~2 min into the mission, on the CPU fallback (rt went 2.8 → 4.9 GB with collab workers inside the rt cgroup). Rerun 046ee9 with ORT CUDA peaked at 3.9 GiB, so 10g is ample |
| `ARIADNE_ROOT_GB` | `150` | the 75 GB root fills with four images + the recon zst + the payload |
| `ARIADNE_RECON_BACKEND` | `cpu_init` (under the console) | with `gpu` the console serves chunks from `<run>/trained` and the live pane 404s |
| `ARIADNE_LLM_PROVIDER` | `nebius` (fork) | skips the AWS credential export in `start_orchestrator`; the key comes via `--env-file` at run time |

## 3. Checks in the logs

- **check:** the runtime log says `hard-pair ... via onnxruntime_cuda`. If it says `onnxruntime_cpu (cuda unavailable: …)`, it fell back to CPU, and the take does not count.
- **check:** the sim log shows GL_VENDOR NVIDIA. EGL on the T4 was confirmed in 7b34cf.
- **check:** trace decisions show `source: llm` with `orchestrator.provider = nebius-token-factory` and the Token Factory model ID.
- **check:** no 137 exit. Record peak rt memory from the samplers.
- **check:** `hypotheses.jsonl` and `verifier.jsonl` exist, and at least one status change appears in the take.
- **Known exit bug (open upstream, 2026-10-28):** after the mission ends and `results.yaml` is written, `ariadne_live` can die at
  process exit with glibc "corrupted double-linked list" (exit 139), and `live.sh` then marks the run FAILED. Until M's fix is
  merged, judge a take by its artifacts (`results.yaml`, `trace.jsonl`, the jsonl logs, the recording), not by the rt exit code.

## 3a. Expected performance on g4dn.xlarge (M's rerun 046ee9, `3dad564`, classical chooser, 100 s)

- **ORT CUDA:** all 6 ALIKED/LightGlue sessions (parent + 2 workers) run via `onnxruntime_cuda`. Worker `ms_aliked` is 7.7–9.6 s for the whole run (CPU: ~167 s in a7f27b).
- **RTF: 0.42 end to end**, including startup. 4 vCPU is CPU-starved.
  - 38/43 frames per agent; track_mean 3.3–3.6 s.
  - 13 replans; 22 commands sent, 4 expired.
  - Command age p50 9.9 s (max 13.6 s); plan_fork mean 10.2 s (max 42 s).
- **Largest remaining CPU consumer:** EigenPlaces on cv::dnn CPU in each collab worker (`ms_place` 136–144 s per worker).
- **Implication for the take:** the console plays at sim time. At RTF below 1, any sped-up footage must be labelled with its factor
  on screen. The Nemotron decision latency adds to the command age above.

## 4. Relaunch etiquette

The spot quota frees a few minutes after termination; an immediate relaunch gets `MaxSpotInstanceCountExceeded`. Wait before
retrying, and never write your own retry loop. Tell M about each launch and each termination.
