# Decision-tier bench, 2026-10-29 02:46–03:05 IST (FP, D318)

`python -m orchestration.bench --corpus evaluation/runs/live-agentic/live_loop/llm_live_run5_maze_short_90s --n 10
--est-in 32000 --est-out 500`, on a host venv (openai 3.40.0, mcp 2.3.0), with live Token Factory calls. The commit and the
config sha are in `manifest.yaml`; per-request rows are in `results.jsonl`.

**Manifest caveat.** `manifest.yaml`'s `git_commit: 3ecbe70` is wrong. The bench read HEAD when it *wrote* the manifest, after
`main` had been merged into the worktree mid-run. The code that ran is `eef1d84` (bench + config at that commit; the bench's
code paths are unchanged in 3ecbe70). `git_dirty_python_configs: true` is from untracked F2 files the bench does not
import. Fixed: the bench now reads the commit before the run and ignores untracked files.

## What was replayed

- **Requests:** 10 of the 18 chooser requests that `ariadne_live` sent in the 2026-10-23 run `llm_live_run5`, evenly spread over
  t = 5–85 s. Only the requests are replayed. The recorded answers came from the Bedrock-era Nova model; they are reference only.
- **Model setup:** each model ran alone, with no fallback. It was text-only (the `--text-only` MCP server) at temperature 0, with
  `max_tokens` 512 and `enable_thinking:false`. Approvals were stubbed `approved`; no candidate in these requests needed one.
- **Scoring:** the C++ gate's rules (range, approval, exactly-once by cluster).

## Result [VERIFIED, this run]

| model | answered | valid agent decisions | p50 / p95 latency | calls/decision | prompt tok/decision | USD (10 req) |
|---|---|---|---|---|---|---|
| `nvidia/nemotron-3-super-120b-a12b` | 10/10 | 20/20 | 7.95 / 26.9 s | 3.7 | 87.7k | 0.270 |
| `nvidia/Nemotron-3_5-Lightning` | 10/10 | 20/20 | 6.42 / 8.13 s | 6.1 | 111k | 0.067 |
| `nvidia/NVIDIA-Nemotron-3-Nano-30B-A3B` | **1/10** | 2/2 | 27.0 / 45.9 s | 9.8 | 46.3k | 0.031 |

- **Nano (negative result):** on 9/10 requests it never produced a parsed tool call. Every turn ended `finish_reason: stop` until
  the 10-call cap, so the answer was null and C++ would fall back to classical. It is not used.
- **Super:** two decisions (t65000ms, t85000ms) each had one turn cut at `max_tokens` 512 with no tool call. Super writes
  free-text deliberation into `content` even with thinking off. Both still assigned on a later turn.
- **Reasoning tokens:** 0 for every model.
- **Cost:** the bench spent USD 0.3758 (ledger 0.0076 → 0.4111). The pre-run estimate was USD 0.14. It was low because
  late-mission requests are larger (a `list_candidates` result re-sent on every turn, up to ~200k prompt tokens per Super
  decision). The bench's default per-request estimate is now 120k/1k.
- **Reference agreement:** with the classical choice, Lightning 13/20 and Super 11/20; with the recorded Nova choice, Lightning
  16/20 and Super 12/20. These are not accuracy measures.

## Decision-tier pick (D319)

- **Primary:** `nvidia/Nemotron-3_5-Lightning`. It is as valid as Super here (20/20), has a 3.3× lower p95 and is 4× cheaper per
  decision (~USD 0.0067). It is served in eu-north1.
- **Transport-error fallback:** `nvidia/nemotron-3-super-120b-a12b`, in a different region (us-central1).

n = 10 requests from one recorded run on one scene. This is descriptive, not a significance claim.
