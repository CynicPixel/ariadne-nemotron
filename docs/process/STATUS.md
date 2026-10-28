# Status (newest first; times IST)

## 2026-10-28 ~10:00 — S0 sync done (FM): base line merged, worktrees opened, bootstrap started

- **Base line:** upstream M delivered `integ/nemotron-base` at `140957f` = main `0ca523c` + `perf/live-decision-path` `fd63fcc`.
  M's checks: build clean with ARIADNE_WITH_ORT=ON; `ctest -R 'LiveLink|Egress|HardPair'` 23/23 (models env set); planner and
  worker suites 75/75; recon `test_live.py` 18/18. Merged into the fork's main as **S0 (`8c0d1b4`)**, no conflicts.
- **Session worktrees** (F0 user rule: FP and FV talk only to FM; only FM talks to upstream M, live path only):
  - `/data/code/ariadne-nemotron-fp`, branch `fp/nemotron-layer` (FP);
  - `/data/code/ariadne-nemotron-fv`, branch `fv/rebrand` (FV).
  
  Both fast-forwarded to `8c0d1b4`.
- **F0.4 bootstrap started:**
  - copies of Ariadne's `.cache/{ccache-docker,fetchcontent-docker,models,fixtures}`;
  - the runtime built with the existing `ariadne/core-dev:dev` image (`ARIADNE_SKIP_DEV_BUILD=1`; `core.Dockerfile` unchanged since 2026-09-29, so no shared image is rebuilt);
  - `ctest -R 'LiveLink|Egress'`;
  - `live.sh build`;
  - console esbuild.
  
  Timing goes here when it finishes.
- **Fork gotchas from M:** ORT builds need `LD_LIBRARY_PATH` at the onnxruntime lib for gtest discovery. The GPU live path needs `ARIADNE_RT_GPU=1` and
  the `_ortcuda` live config. The ORT payload recipe for F5 has been asked of M.

## 2026-10-28 09:00 — Fork opened (FM): F0.1 done, D300–D349 claimed

- **Fork:** `/data/code/ariadne-nemotron`, cloned from the local Ariadne at `0ca523c`. Full history kept (D300).
  - `upstream` = `/data/code/Ariadne`, push `DISABLED`.
  - `origin` = `git@github.com:CynicPixel/ariadne-nemotron.git` (private, created by the user; empty, reachable).
- **D-number claims:** FM D300–D309, FP D310–D329, FV D330–D339; D340–D349 reserved. Upstream M acknowledged.
- **Base line:** upstream M builds `integ/nemotron-base` = main + `perf/live-decision-path` (`fd63fcc`, which includes
  `perf/live-step3`). M's dry run shows 8 conflicted files, mostly from `fix/live-pipeline-final` vs main. ETA 12:00
  once the build and `ctest -R 'LiveLink|Egress|HardPair'` are green.
- **Compute:** FM queues every fork job and asks M first. The slotq band is **60+**, job files named `6N-fm-<name>.job`. The Nebius key
  is never in DIR, the snapshot or S3. M clears the eval spend with the user.
- **GPU:** Stockholm g5.2xlarge / g4dn.2xlarge spot capacity scores 1/10 (6 h of failed retries, per M); only g4dn.xlarge
  launches. The 16-vCPU on-demand G case is still open. The F5 recording is planned on g4dn.xlarge.
- **Blocked:**
  - F0.5 waits on the user's Nebius signup and key.
  - F0.4 and G1 wait on the base line.
  - F1 Python work proceeds on main meanwhile (no file overlap with the perf diff).
