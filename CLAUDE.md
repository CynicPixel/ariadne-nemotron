# ariadne-nemotron — Implementation Repo Guidelines (Claude)

**This repo is ariadne-nemotron:** Ariadne's simulated multi-agent reconnaissance stack, with an
evidence-seeking agentic layer built on **NVIDIA Nemotron served by Nebius Token Factory**. Ariadne
itself is collaborative stereo-inertial SLAM, Gaussian-splat reconstruction and risk-weighted egress
routing for agents in GPS-denied, comms-constrained collapsed structures.
**Target: Nebius × NVIDIA Global AI Hackathon, Physical AI track.** Submissions close
**30 Oct 2026 22:30 IST**; the internal submit target is **30 Oct 18:00 IST**.

> **Provenance chain:** research memo (local, untracked) → plan
> (`docs/process/PHASED_PLAN.md`) → design (`docs/design/nemotron-evidence-loop.md`) →
> **code (here)**. Ariadne's architecture (`docs/architecture.md`) stays the source of truth for
> everything below the agentic layer. When code and doc disagree, the doc wins unless you surface
> the conflict and get a decision.

---

## 0. Teaching protocol (brief, applies to every code-producing turn)

The maintainer is an experienced systems engineer: **teach the domain, not the language.**
1. Before code: one short paragraph on what the piece is, why it exists, and its data in → out.
2. Explain each new concept the first time it appears (tool-calling loops, structured outputs,
   reasoning tokens, MoE serving, occupancy verification).
3. No unexplained magic: non-obvious lines, config keys and flags get a one-line what+why.
4. After code: how to run it, what correct output looks like, how to tell it broke.

## 1. Locked decisions (do NOT re-litigate; `docs/decisions.md` D300+ records why)

| Area | Decision |
|---|---|
| Model platform | **Nebius Token Factory**, via the OpenAI-compatible API. Model IDs come from `GET /v1/models` and live in `configs/llm/nemotron.yaml`, **never** in code constants. |
| Model tiers | **Hypothesis tier:** Nemotron 3.5 Super VL, async, over images (keyframes + map). **Decision tier:** Nemotron Super / Lightning / Nano on each replan, reasoning off; the bench picks which. |
| Safety boundary | Unchanged C++ gate (`ariadne_live.cpp` chooser block): index-in-set, exactly-once, approval above the risk threshold, operator overrides. Routes come only from `MissionPlanner::Route`. The model never certifies geometry. |
| Hypothesis status | Set only by the deterministic slab verifier (`python/orchestration/verifier.py`), from observed occupancy. Never from simulator truth, never from the model's own words. |
| Sim + estimation | Ariadne as-is: ROS 2 Jazzy + Gazebo Harmonic, OpenCV 5, from-scratch VIO/collab/planning. No Isaac migration. |
| Compute | AWS EC2 stays the simulator's compute (spot g4dn.xlarge is what launches). Token Factory is the inference path. |
| Upstream | Shared git history with Ariadne. `upstream` = the local `/data/code/Ariadne`, **push DISABLED**. The fork pulls `integ/nemotron-base` at sync points; Ariadne identifiers (`ariadne_live`, `ARIADNE_*`, image names) stay unchanged so merges stay clean. |
| Legacy provider | Bedrock remains as an optional `providers/bedrock.py` comparator only. |

## 2. Invariants — must not regress

1. **Hard gates of the track:** a live runtime call to Token Factory running an NVIDIA open model
   that materially drives robot behaviour; a public repo with the MIT license visible; a public
   YouTube video **under 3:00** with ≥ 60 s of the live application.
2. **No secret in any tracked file, snapshot, S3 payload or job file.** The key lives in
   `~/.config/ariadne/nebius.env` (0600) and is passed at run time only.
3. **No simulator ground truth in any model input.** GT is used only by evaluators, and to
   pre-screen scenes (recorded as a decision).
4. **Every Nemotron claim is backed by a logged live call** (the trace `orchestrator` block:
   provider, model, latency, tokens). Pre-fork Bedrock-era runs under `evaluation/runs/` are never
   presented as Nemotron results.
5. **Egress claims use ρ_responder (0.40 m),** never ρ_agent.
6. **Honesty:** negative results are reported, never buried. No invented metrics, instance
   types or significance claims. Count unsafe *proposals rejected* separately from unsafe
   *actions executed*.
7. **`tools/scrub/check.sh` passes** before any push.

## 3. Repo structure and ownership

Ariadne's layout (`docs/architecture.md` §3) holds. The dependency rule holds: core ← domain libs
← runtime ← adapters; no SDK type in an algorithm library. Fork-owned paths, which upstream never
edits:
`python/orchestration/{providers/,hypotheses.py,verifier.py,bench.py}`, `configs/llm/`,
`tools/scrub/`, `docs/process/`, `docs/submission/`, and the console hypotheses panel.
**Shared hot files** get minimal, localized edits only: the `ariadne_live.cpp` slab block,
`live.sh` `start_orchestrator`, `orchestrator.py`, `mcp_server.py`.

| Session | Owns | D-block |
|---|---|---|
| FM | fork admin, upstream syncs, compute queue, live GPU recording, Devpost form | D300–D309 |
| FP | Nemotron provider, bench, slab export, hypotheses, verifier, MCP tools, eval analysis | D310–D329 |
| FV | scrub + gate, README/docs, console panel, video, Devpost copy | D330–D339 |

D340–D349 are reserved.

## 4. Execution model

Follow `docs/process/PHASED_PLAN.md` (F0 → F6, gates G0–G6 in IST). A phase item is checked off
only together with its evidence (`docs/process/REGISTER.md` row + a STATUS entry or run
directory). Upstream syncs follow the plan's sync protocol: fetch upstream, merge
`upstream/integ/nemotron-base`, run `tools/scrub/apply.sh && tools/scrub/check.sh`, run the targeted tests, log it in
STATUS. Resolve a hot-file conflict by taking upstream's side and re-applying the scrub.

**Upstream contact (user rule, 2026-10-28):** consult **no** existing Ariadne session except upstream
M, and M **only for live-path matters**: the integration base line, runtime perf, and GPU launches on
the shared quota. FP and FV never message upstream sessions; anything they need from upstream goes
through FM.

**Compute:** the two-job cap is machine-wide (fork + upstream). **FM is the single queue owner**;
before starting a job, FM checks machine load and running containers itself. Long runs go to the
background. GPU launches are cleared with M first. The user approves any AWS spend in-session.

## 5. Conventions

- **Git:**
  - Commit only your own work, naming files explicitly. **No Co-Authored-By trailer**, ever.
  - Push only to `origin` (CynicPixel/ariadne-nemotron); never to `upstream`.
  - Never touch GitHub CynicPixel/Ariadne, and never redeploy the upstream hosted console site (`infra/scripts/console_site.py`).
- **Manifests or it didn't happen:** every run records git commit, image digests, scene/seed,
  configs and the LLM provider/model IDs.
- **Honesty tags:** `[VERIFIED]`, `[ASSUMPTION]`, `[PROXY]`, `[UNVERIFIED]`. No guessed API
  names: verify against the live endpoint.
- **Times:** IST in messages to the maintainer and sessions; UTC in artifacts.
- **Verify before claiming done:** run it and report the actual output. A skipped step or a red test is
  stated plainly.
- **Context Mode:** use it (`ctx_execute`/`ctx_batch_execute`) for large outputs: logs, run dirs,
  API listings.

## 6. Source-of-truth corpus (link, don't duplicate)

- Plan: `docs/process/PHASED_PLAN.md` · register: `docs/process/REGISTER.md` · status:
  `docs/process/STATUS.md`
- Agentic-layer design: `docs/design/nemotron-evidence-loop.md` (and `docs/design/agentic.md`
  for the inherited chooser)
- Decisions: `docs/decisions.md` (D300+ are the fork's)
- Architecture and algorithms: `docs/architecture.md`, `docs/algorithms.md`

## graphify

This project has a knowledge graph at graphify-out/ (generated locally, gitignored).
- For codebase questions, run `graphify query "<question>"` first when
  `graphify-out/graph.json` exists. Use `graphify path "<A>" "<B>"` and `graphify explain "<concept>"`
  for focused questions.
- After modifying code, run `graphify update .` (AST-only, no API cost).
