# Nemotron evidence-seeking loop (FP; design, decisions D310–D329)

Status: preflight + decisions filled 2026-10-29 03:10 IST (FP), before code. Plan items: `docs/process/PHASED_PLAN.md` F1–F2.

## 1. Preflight

**Inherited chooser** (`docs/design/agentic.md` D156–D164). Per replan, `ariadne_live` sends one joint request line over
`<run>/chooser.sock`. `orchestrator.py` writes it to `decisions/<did>.json` and runs a tool loop against two MCP stdio
servers (`mcp_server.py`, full and `--text-only`). The answer is `{decision_id, assignments, orchestrator}`.

The C++ gate (`ariadne_live.cpp` chooser block, ~L2372–2605) re-validates every assignment per agent:
- index in range, else fallback;
- an above-threshold pick needs the approval file `approvals/<did>-a<k>-c<i>.json` with verdict `approved`;
- exactly-once by cluster.

It reads only `agent`, `candidate`, `approval_id` and `reason` per entry. The whole answer line is embedded in
`trace.jsonl` as `choice.answer`, so extra fields (`hypothesis_id`, `evidence_goal`, the `orchestrator` log) are tolerated
and archived [VERIFIED by reading the parser: `YAML::Load(ans)["assignments"]`, per-key lookups].

**Map facts.** The operator map is `maps.slab`. It is a `CV_8U` grid in the `occupancy_grid` convention (0 unknown, 1 free,
2 occupied), row = y index (north), col = x index (east), and cell centre = `origin + (idx + 0.5)·cell`
(`frontier_mission.hpp:77`). It is already written raw (unflipped, unscaled) at every replan to
`snapshots/slab_<ms>.png` (L2205), before the request is built. `map_png` is a 3× scaled, north-up rendering with
overlays, for the model only, and is never used by the verifier.

Cluster ids and candidate indices are renumbered at every replan. The runtime itself re-associates by goal within
`refused_radius_m` (0.5 m) for `current_task`.

**Live endpoint facts (D301, G0).** Base `https://api.tokenfactory.nebius.com/v1`, OpenAI-compatible, served by vLLM:
- Super-VL accepts an `image_url` data URI in user content together with `tools` and `tool_choice:auto`;
  `reasoning_effort:"none"` gives 0 reasoning tokens.
- Super 120B with `chat_template_kwargs.enable_thinking:false` gives 0 reasoning tokens and emits short `content` alongside
  `tool_calls`.
- Image tokens are billed as prompt tokens (a map ≈ 262).

Not yet verified at G0, now closed by the fixture recording (§4): `tool_choice:"required"`, and the reasoning-off flags for
Lightning, Nano and VL.

**Literature pointers** (from agentic.md §0; no new survey under the deadline):
- L3MVN and LFG: the LLM ranks frontiers, and a classical planner keeps feasibility.
- Co-NavGPT: joint multi-robot assignment over a merged map.
- Hypothesis-driven active perception, in the sense of choosing views to discriminate between hypotheses about unobserved
  structure. This loop instantiates it with a deterministic, conservative verifier rather than a learned belief.

## 2. Decisions

- **D310 Provider interface.** `providers/base.py` defines `Provider.run_turn(...)`, which runs one tool loop with that
  vendor's message format and returns whether `assign` recorded a choice. It declares `transport_errors`.
  - `bedrock.py` is the inherited Converse loop, moved out unchanged.
  - `nebius.py` uses the OpenAI SDK (`max_retries=1`, per-call timeout 45 s, well under `llm.timeout_s` 120 s).
  - `orchestrator.py` keeps the socket server and `decide()`. It walks the configured chain and falls back to the next
    model **only** on a transport error (connection, timeout, 429, 5xx). A wrong or empty answer, `finish_reason:length` or
    BudgetExceeded is never a fallback trigger. The C++ side then falls back to the classical choice.
- **D311 Config, not constants.** `configs/llm/nemotron.yaml` (and `bedrock.yaml`, which reproduces the D84 constants) holds:
  - the chain: model, modality, `max_tokens`, `extra_body`, temperature;
  - prices in $/1M tokens;
  - the budget caps.
  
  Text-only models are paired with the `--text-only` MCP server. Model IDs are D301's, verbatim. `--llm-config` selects the
  file, and the orchestrator image reads it from the bind-mounted repo.
- **D312 Spend ledger (FM requirement).**
  - **Ledger:** append-only `NEBIUS_SPEND_LEDGER` (default `~/.config/ariadne/nebius_spend.jsonl`), one line per call:
    `ts_utc, model, prompt_tokens, completion_tokens, est_usd, purpose`.
  - **Pricing:** `est_usd` uses the config's prices. An unknown model is priced at the most expensive known in/out price.
  - **Caps:** before every call the ledger is summed. At or above `budget.usd_hard` (40) the call raises `BudgetExceeded`;
    at `usd_warn` (25) a warning is logged once per process.
  - **Writes:** `O_APPEND` under `flock`, so the bench and a live run can both write.
  - **In the container:** `live.sh` touches the host file (0600) and bind-mounts only that file, because `HOME=/tmp` in the
    container would otherwise give a fresh, empty ledger every run.
- **D313 Adapter rules (Nemotron on vLLM).**
  - **Decision tier:** reasoning off (`enable_thinking:false`, per model in config) and `max_tokens` 512.
  - **Hypothesis tier:** may reason, capped at 2048.
  - **`tool_choice`:** `auto`, plus the inherited nudge ("Call a tool; finish with assign.") when a turn has no tool call.
  - **Images:** tool results are text-only `role:tool` messages. Images from `observe_image` follow in one `role:user`
    message as data URIs, and only for image-capable models.
  - **Assistant turns** are re-appended as `{role, content, tool_calls}`. Vendor `reasoning*` fields are dropped.
  - **Logging:** the `orchestrator` log keeps the Bedrock keys and adds `provider`, `reasoning_tokens`, `est_usd` and
    `finish`.
- **D314 Slab in the request.** The request gains `slab{path, t_s, res_m, origin_xy_m, rows, cols}`. `path` is the
  existing `snapshots/slab_<ms>.png` of the *same* replan, written before the request. There is no second image write,
  which departs from the plan's `<did>_slab.png`.
  - **Guard (FM condition):** the orchestrator refuses to verify against a slab whose `t_s` differs from the request's
    `t_s`, or whose file stem ms is greater than the decision ms. Evidence can never come from a later replan than the
    model saw.
  - **Reading:** the verifier reads the PNG with Pillow and numpy, which are pinned in the orchestrator image.
- **D315 Hypothesis record.** A hypothesis claims that *the frontier at centroid (x, y) opens onto a passage a responder can
  use (0.40 m clearance)*.
  - **Anchor:** the candidate that seeded it, kept as `{decision_id, agent, candidate, cluster, goal_xy_m, centroid_xy_m}`.
  - **Re-anchoring:** at every later replan the hypothesis is re-anchored to the candidate whose `centroid_xy_m` lies within
    `match_radius_m` (1.0 m) of the stored centroid. `goal_xy_m` is the standoff point, which moves as the map grows, so
    the centroid is the stable key. Cluster ids are never used as a key.
  - **Seeding:** deterministic, from request data only, with no ground truth. Each agent's top `seed_top_k` (2) ranked
    candidates seed a hypothesis unless one already covers that centroid.
  - **VL proposer (F2.4, optional):** may add `rationale` and `prior` to a seeded hypothesis. It never sets status.
  - **Record shape:** §3.1.
- **D316 Verifier rule** (`verifier.py`, deterministic, slab-only).
  - **Region:** a disk of radius `R` = 1.5 m around the hypothesis centroid. The entry side is the half facing the anchor's
    `goal_xy_m`; the far side is the cells with projection ≥ 0.5 R beyond the centroid along goal→centroid.
  - **Clearance:** a free cell is *responder-clear* when no occupied cell lies within ρ_responder = 0.40 m.
  - **Two maps over the disk:**
    - pessimistic: only observed-free cells may be clear;
    - optimistic: unknown cells count as free; clearance still only from observed occupied cells.
  - **Status from the two maps:**

    | condition | status |
    |---|---|
    | a clear 8-connected path from the entry side to the far side exists in the pessimistic map | **confirmed** |
    | no such path exists even in the optimistic map (observed obstacles alone block a responder) | **refuted** |
    | in the pessimistic map, clear cells connected to the entry side reach within 0.5 R of the centroid | **supported** |
    | none of the above | **unresolved** |

  - **Expiry:** a hypothesis with no matching candidate for 3 consecutive replans while still unresolved or supported becomes
    **expired**.
  - **Terminal states:** confirmed, refuted and expired are terminal. A later contradicting verdict is logged in
    `verifier.jsonl` as `contradiction`, never silently flipped.
  - The verifier never reads model text or simulator truth.
- **D317 Traces.** `hypotheses.jsonl` (one line per create, re-anchor or status change; append-only) and `verifier.jsonl`
  (one line per hypothesis per replan) go in the run dir, both carrying `decision_id` and `t_s` to join with
  `trace.jsonl`.
  - The live state for one decision is `decisions/<did>.hypotheses.json`, written by the orchestrator before the turn starts. It is per decision, so concurrent requests never race.
  - `list_hypotheses` (MCP) reads that file.
  - `assign` takes optional per-entry `hypothesis_id` (it must exist and be open) and `evidence_goal` (≤ 200 chars), and
    records them in the answer.
- **D318 Bench** (`python -m orchestration.bench`).
  - **Replay:** copies only `t*ms.json` from a recorded corpus into a scratch dir and rebases its paths.
  - **Approval stub:** pre-writes `approved` verdicts for every `needs_approval` candidate.
  - **Per model:** runs `decide()` over N requests (default 10).
  - **Scoring:** the C++ gate's rules (range, exactly-once, approval).
  - **Cost:** prints an estimate from the config's prices before running, and refuses if `ledger + estimate ≥ usd_hard`.
  - Agreement with the recorded Bedrock-era choice is reported as reference only, never as accuracy.

- **D319 Decision tier and VL proposer.** The bench picks Lightning, with Super on transport errors. The VL proposer is on:
  async, validated, logged per call, seed fallback, hit rate over terminal outcomes only. Details are in `docs/decisions.md` D319.
  Extra events in `hypotheses.jsonl`:
  - `vl_annotated`: a VL prediction attached to an existing open hypothesis;
  - `vl_call`: one per VL call, with `hypothesis_id: null`, `outcome` ok|no_tool_call|zero_valid|error, its tokens and its
    latency. The console panel should skip lines without a `hypothesis_id`.
  
  A VL-created record has `provenance.source: "vl"` with `rationale`, `prior`, `predicts` and `seen_decision_id`, plus the
  `orchestrator` block.

## 3. Acceptance (register F1/F2, gates G1/G2)

### 3.1 `hypotheses.jsonl` record (FV console panel)

One JSON object per line, append-only, one line per event. The panel keeps the last record per `hypothesis_id`.

```json
{"t_utc": "2026-10-29T03:10:00.123Z", "sim_t_s": 20.0, "decision_id": "t20000ms", "event": "created|reanchored|status",
 "hypothesis_id": "h003", "agent_id": 0,
 "anchor": {"kind": "candidate", "decision_id": "t20000ms", "candidate": 1, "cluster": 5600757264945488847,
            "goal_xy_m": [2.75, 3.75], "centroid_xy_m": [2.75, 5.25]},
 "claim": "the frontier at (2.75, 5.25) opens onto a passage a responder can use (0.40 m clearance)",
 "evidence_goal": "drive to the standoff and look past the centroid", "status": "unresolved",
 "status_source": "verifier|none", "verifier": {"slab": "snapshots/slab_20000.png", "slab_t_s": 20.0, "reason": "..."},
 "provenance": {"source": "seed|vl", "keyframe_path": null, "map_png": "decisions/t20000ms_map.png"},
 "orchestrator": {"provider": "nebius", "model": "...", "latency_ms": 0, "tokens_in": 0, "tokens_out": 0}}
```

- `status` ∈ `unresolved | supported | confirmed | refuted | expired`.
- `status_source` is `verifier` for every status event and `none` on creation.
- `orchestrator` is present only on VL-proposer records.
- The fixture is `python/tests/orchestration/fixtures/hypotheses_sample.jsonl`.

## 4. Tests

These are the offline tests in `python/tests/orchestration/` (42; host venv and `ariadne-orchestrator:nemotron-1`):
- `test_orchestrator_stub.py`: the Bedrock path, unchanged behaviour.
- `test_nebius_provider.py`: covers
  - the recorded Token Factory loops (Super, Lightning, Nano; 2026-10-29) through `decide()` and the real MCP servers;
  - transport fallback, budget refusal, no-tool and `length` turns, bad JSON arguments, `<think>` text and image routing;
  - the ledger (sums, unknown-model price, cap, warning once, unreadable line, env path);
  - bench hygiene and gate scoring.
- `test_verifier_hypotheses.py`: covers
  - the verdicts on synthetic slabs, the status transitions and the slab guard;
  - corpus geometry;
  - a layer replay over a recorded run (confirmed and refuted both occur), and that only the request and slab are inputs;
  - the MCP `list_hypotheses` and `assign` fields;
  - the VL proposer (async fold-in, validation, `vl_call` logging, seed fallback, hit rate).
## 5. Cost (tokens per decision and per run, from the bench)

Bench `bench_20261028T211611Z` (10 recorded requests per model):

| model | mean prompt / completion tokens per decision | USD per decision |
|---|---|---|
| Lightning | 111k / 440 | 0.0067 |
| Super | 88k / 778 | 0.027 |

- **VL proposer:** ~11k / 0.3k per replan (one live call), USD ~0.0036.
- **A 90 s run** (18 replans) on Lightning + VL is about USD 0.19.
- **Prompt size:** prompt tokens are dominated by `list_candidates`, which is re-sent on every turn.
## 6. Result
