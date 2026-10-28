# Status (newest first; times IST)

## 2026-10-29 02:50 — F3.1–F3.4 done on fv/rebrand (FV); F3.5 waits on FP's hypotheses fixture

- **F3.1 scrub gate (`tools/scrub/`, D330):**
  - `check.sh` is clean on `fv/rebrand`. It checks the denylist (text, raw bytes, PDF text), that the deleted directories stay
    deleted, that no living file references them, and secrets (token shapes plus the literal local key values).
  - `--release` blocks `TODO(FM)` in `README.md` and `docs/submission/`. `--history` scans all refs.
  - `apply.sh` is idempotent: the second run printed nothing.
  - Negative tests: every planted fake secret and the planted denylist string were caught. In a throwaway repo, a committed-then-deleted
    key was caught by `--history`.
- **History secret scan (before G3):** all refs, 790 commits, 931,328 added lines: **no secrets.**
- **F3.2:** deleted `submission/`, `docs/report/`, `infra/cool/` and `python/video/scenes/` (81 files, including the old film).
  The recovery point is `7496988`.
- **F3.3:**
  - `console_site.py`: the paper copy is removed and the paper link is `None`. It was edited, never run.
  - `runs_index.py` and `./ariadne runs` now write `build/runs/runs.csv`.
  - `ariadne`: the hosted-frontend note is removed; `cloud cool` is removed; `usage()` now reads up to the rule line.
  - `demo.py`: no default film URL, no `/paper` route, and the GitHub link points to this fork.
  - `make_gifs.py` writes to `build/video/gifs`.
  - The account ID stays in the launchers, per FM.
- **F3.4:**
  - `README.md` is rewritten. It covers Nemotron on Token Factory (model IDs from D301), a Mermaid architecture diagram,
    inherited results labelled pre-Nemotron, and a zero-credential quick start.
  - Five `TODO(FM)` markers remain: the video, the bench's pick, the confirmation of FP's implementation, the F4 results, and
    the Nemotron run and bench commands.
  - `evaluation/runs/README.md` adds the lineage note.
  - The wider architecture, algorithms and validation phrasing is limited to the denylisted lines (D330: bare COOL stays).
- **Tests:** `python/tests/console/test_demo.py` 8/8 passed (scratch venv). `py_compile` and `bash -n` pass on every touched
  script, and the three mission YAMLs parse.
  - `ariadne_live.cpp` changed only in a comment, so it is not rebuilt here.
- **Video:** the beat audit is in `docs/process/VIDEO_NOTES.md`.
  - Beat 11's footer carries the old competition's name, and beat 8 is Bedrock-labelled.
  - Rendered frames are not yet checked.
  - `docs/me/Ariadne_thumbnail.png` carries a faint third-party stock-logo watermark and is kept out of the README.
- **Open:** F3.5 waits on FP's `hypotheses.jsonl` fixture and the schema section.

## 2026-10-29 02:30 — G0 passed (FM): live Nemotron on Token Factory

- **Key:** the user's key is in place (outside the repo, 0600).
- **Models:** `GET /v1/models` lists 5 NVIDIA models; IDs, regions and prices are in D301.
- **Live calls:** Super-VL with an image and an `assign` tool, and Super 120B with the tool, both returned HTTP 200 in about 1.1 s with a valid tool call and 0 reasoning tokens. Cost < USD 0.001.
- **Budget:** Limited credit budget. FP builds a ledger cap into the provider (hard USD 40, warn USD 25).
- **Sessions:** FP and FV started at ~02:15. Recording: Kaggle GPU as a stretch (go/no-go 29 Oct 18:00), local recording guaranteed. Narration: same TTS as the Ariadne film, new script.

## 2026-10-29 01:30 — Internal G0 no-go time passed (FM)

- **No Nebius key** at `~/.config/ariadne/nebius.env` as of 01:26. The plan's hard no-go (29 Oct 00:00) has passed without G0.
  The hackathon deadline (30 Oct 22:30) still stands. Whether to continue on a compressed plan is the user's call.
- **FP and FV never started:** `fp/nemotron-layer` and `fv/rebrand` are 0 commits ahead of main. F1, F2 and F3 have not started.
- **Upstream:** `integ/nemotron-base` is at `b2b02b2` (the crash reporter also catches SIGABRT/SIGBUS). There is no exit-139 fix yet.
  - M's GPU poller retries g5.2xlarge/g4dn.2xlarge every 20 min for 12 h. When it lands, it runs `b2b02b2` with ORT CUDA vs CPU DNN back to back.
  - S2 stays at 12:00 today, without the backtrace.
- **GPU capacity (M, 01:35):** g4dn.xlarge is also at placement score 1/10 in all three eu-north-1 AZs. All 60 attempts on 28 Oct
  17:33–20:33 failed for capacity. No GPU quota is reserved for the fork. Until capacity returns, an F5 recording on AWS GPU is not available.

## 2026-10-28 10:15 — GPU rerun figures from upstream M (FM)

- **M's g4dn.xlarge rerun 046ee9** (`3dad564`, classical chooser, 100 s, RT_MEM=10g):
  - ORT CUDA confirmed in all 6 sessions.
  - Peak rt memory 3.9 GiB.
  - **RTF 0.42 end to end; the box is CPU-bound.** Per M's correction, this is not a gain: the starved estimators did less work
    (38–43 frames vs 106–111 in ec5893).
  - CPU cores: `ariadne_live` 1.36, gz sim 0.96, ROS bridge 0.67, collab workers 0.55.
  - EigenPlaces on GPU would free ≤0.5 core (unmeasured, queued upstream).
  - A lower camera rate is the likely lever, but it is unmeasured.
- **Open upstream bug:** exit 139 ("corrupted double-linked list") at `ariadne_live` exit, after `results.yaml` is written. Artifacts are complete,
  but `live.sh` reports FAILED. The runbook now says to judge takes by artifacts until the fix is merged.
- **F5 risk:** the plan's "RTF 1" recording is not reachable on g4dn.xlarge today. Options are with the user (labelled speed-up,
  or an upstream EigenPlaces GPU path before S2). Still blocked: F0.5 (no Nebius key at 10:11).

## 2026-10-28 09:20 — F0.4 bootstrap done; S0.1 sync (FM)

- **Bootstrap: 7.5 min wall** (copying caches 83 s, runtime build + `ctest -R 'LiveLink|Egress'` 17/17 at 415 s, `live.sh build` 40 s,
  console esbuild 2 s).
  - First attempt failed: Ariadne's `.cache/fetchcontent-docker/*-subbuild` caches embed host paths. Fix: keep `*-src`, drop
    `*-subbuild`/`*-build`.
  - Artifacts: `build/docker/cpp/runtime/ariadne_live` (non-ORT, for local Mesa wiring), `build/sim-plugins`,
    `build/ros2/install`, `python/operator_console/static/dist`.
- **S0.1:** merged upstream `ce351fb` (= perf `3dad564`: ORT env created before the CUDA provider). Without it every `_ortcuda` run
  silently falls back to CPU. Incremental rebuild clean; LiveLink|Egress|HardPair pass (the model-env HardPair cases SKIP).
- **M's g4dn.xlarge smoke 7b34cf:** classical chooser; OOM-killed at RT_MEM=6g ~2 min in. The F5 runbook uses 10g
  (`docs/process/GPU_RECORDING_RUNBOOK.md`).

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
