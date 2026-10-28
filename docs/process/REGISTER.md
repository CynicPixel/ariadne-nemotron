# Completion register

**Rules:**
- One row per hard gate, invariant and plan item.
- A row changes only together with its evidence: a D-ID, a STATUS entry or a run directory.

**States:** `done` · `partial` · `deferred → X` · `not started` · `blocked (by)` · `FAIL / NEGATIVE` · `proposed (branch)`.

**Owners:**
- FM: fork admin, syncs, compute queue, GPU recording, Devpost.
- FP: provider, bench, hypotheses, verifier, eval.
- FV: scrub, README, console, video, copy.
- U: the user.

## Track hard gates (Nebius × NVIDIA, Physical AI)

| # | item | state | evidence / missing | owner |
|---|---|---|---|---|
| H1 | Runtime call to Nebius Token Factory | partial | G0 (D301); in-loop host calls: bench `evaluation/runs/nemotron/bench_20261028T211611Z` (30 decisions), serve() smokes (D319); live-runtime run pending G1 | FM/FP |
| H2 | ≥ 1 NVIDIA open model that materially drives robot behaviour | partial | Lightning picks validated by the C++ gate (bench 20/20 valid); Super-VL proposes hypotheses (D319); needs G1/G2 trace with `source: orchestrator` | FP |
| H3 | Public repo, OSS license visible in About | partial | private repo created 28 Oct; MIT in tree; flip public at G3 | U/FV |
| H4 | README: setup, how Nemotron and Token Factory are used | partial | README rewritten on fv/rebrand; 5 `TODO(FM)` (video, bench pick, impl check, F4, run cmd); `check.sh --release` blocks them | FV |
| H5 | Public YouTube video < 3:00, ≥ 60 s live modules, narrated Nebius/NVIDIA role | not started | F6.1 | FV/U |
| H6 | Project text + track = Physical AI | not started | F6.2 | FV |
| H7 | Additional Info form answered truthfully from measured data | not started | F6.2 | FM/U |
| H8 | Testing access through 15 Dec (zero-credential quickstart + bench) | not started | F3.4, F1.5 | FV/FP |
| H9 | Submitted (not a draft) before 30 Oct 22:30 IST | not started | F6.3 | U |

## Invariants (CLAUDE.md §2)

| # | item | state | evidence / missing | owner |
|---|---|---|---|---|
| I1 | No secret in tracked files, snapshots, S3 or job files | partial | tree + full history (790 commits) clean, D330; snapshots/S3/job files not covered by `check.sh` | all |
| I2 | No sim GT in model inputs | partial | [VERIFIED by code reading, 2026-10-29] request poses/`flown_m` from the estimator's `#frame` stream (`ariadne_live.cpp` deliver_frame L1344, L1561; `a1_planner.hpp:150` "estimated odometry path"); slab/map/egress from keyframe depth; structural risk from the observed traversability map (GT zones calibrate it offline only, D112); hypothesis layer reads request + slab only (`test_layer_inputs_are_request_and_slab_only`). [ASSUMPTION] `brief.T_W_O` is the briefed start pose (`ariadne_live.cpp:1855`, `:2286`). Pending: a live G2 request audit | FP |
| I3 | Nemotron claims ← logged live calls; Bedrock-era runs labelled | partial | lineage note `evaluation/runs/README.md`, README labels; F4 manifests missing | FP/FV |
| I4 | Egress at ρ_responder | done | inherited (Ariadne D-entries) | — |
| I5 | Old-competition denylist clean (`check.sh`) | done (fv/rebrand) | `check.sh` clean; D330; re-run after each sync | FV |

## Phase items

| # | item | state | evidence / missing | owner |
|---|---|---|---|---|
| F0.1 | Fork + remotes | done | STATUS 2026-10-28 | FM |
| F0.2 | Scaffolding | done | 5a8de3b, 6f3c25a, dd10b3f | FM |
| F0.3 | `integ/nemotron-base` | done | 140957f (S0) + ce351fb (S0.1); STATUS 2026-10-28 | M |
| F0.4 | Bootstrap build/models/frontend | done | 7.5 min; LiveLink+Egress 17/17; STATUS 2026-10-28 09:20 | FM |
| F0.5 | Nebius go/no-go + model IDs | done | D301; `evaluation/runs/nemotron/g0_20261028T205925Z/` | U/FM |
| F1.1–F1.6 | Nemotron provider, bench, tests | partial | provider/ledger/live.sh (284bff1); bench done: `bench_20261028T211611Z`, pick Lightning + Super fallback (D319), Nano NEGATIVE 1/10; 44/44 orchestration tests on host (0d41536; 39/39 in nemotron-1 at 3ecbe70); G1 live run pending | FP |
| F2.1–F2.7 | Evidence-seeking layer | partial | F2.1 slab field (b6f9978; FM rebuilt + ctest green); F2.2/F2.3 hypotheses + verifier (D315/D316); F2.4 VL proposer + MCP tools (D319, 860f284, 0d41536); F2.5 traces + `evidence_report`; F2.6 not started; F2.7 cut; G2 live chain pending | FP |
| F3.1–F3.4 | Scrub, deletions, references, README + lineage | done (fv/rebrand, not merged) | D330; STATUS 2026-10-29 02:50 | FV |
| F3.5 | Console hypotheses panel | blocked (FP fixture) | needs `hypotheses.jsonl` fixture + schema | FV |
| F4.1–F4.2 | Matched evaluation | not started | — | FP/FM |
| F5.1–F5.3 | Last sync + GPU recording | not started | — | FM |
| F6.1–F6.3 | Video, Devpost, submit | not started | — | FV/FM/U |
