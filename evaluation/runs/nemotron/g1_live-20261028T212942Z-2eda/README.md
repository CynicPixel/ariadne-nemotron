# G1 live run: evidence-loop analysis (FP)

- **Run:** `live-20261028T212942Z-2eda`, run by FM on 2026-10-29 ~03:00 IST at code `860f284`.
- **Setup:** `maze_short_6`, `live_llm_v1`, `mission_v2` planner, `configs/llm/nemotron.yaml` (Lightning decisions, Super-VL hypotheses), 90 sim s.
- **Run dir:** on the dev host (FM archives it). This directory holds only `python -m orchestration.evidence_report` output, made at commit `bb57a92`.

## Findings [VERIFIED from the run's own logs]

**Decisions:**
- 36 agent decisions: 32 `source: orchestrator`, 4 `fallback`. Every model call was `nvidia/Nemotron-3_5-Lightning` via Nebius, 0 provider fallbacks.
- The 4 fallbacks are 2 decisions, t15000ms and t30000ms. Each hit the 10-call cap: `assign` was rejected three times because it
  named a hypothesis the verifier had just closed (h001, h007). 860f284 rejected the whole assignment for a stale id. 0d41536 records
  the move and drops the id, so these become valid.

**Hypotheses:**
- 21 hypotheses: 15 VL-created, 6 seed; 5 seed hypotheses VL-annotated.
- 16/16 VL calls `ok`.
- Final statuses: 14 confirmed, 3 refuted, 3 supported, 1 unresolved.
- First verified resolution at t = 10 s.
- One contradiction, logged and not applied: h018 was confirmed at 80 s, then a refuted verdict came at 90 s after a map revision.

**VL hit rate** (terminal outcomes only): 12/17. On the predictive subset, resolved after creation, it is 5/8. The small n is
descriptive only.

**G2 chain** (hypothesis → targeted decision → gate accept → motion → newer slab → status change → different next pick): 15 of 23
targeted decisions complete it, 5 of them through a refutation.

Worked example, h010 ("the frontier at (2.37, 1.68) opens onto a passage a responder can use (0.40 m clearance)"):
- Seeded at 35 s. Lightning targets it for agent 0 at 35, 40, 45, 50 and 55 s; every pick is accepted by the C++ gate and sent (`go`).
- Agent 0 moves from (0.56, 3.15) to (3.06, 3.76).
- In the verifier disk, observed occupied cells rise from 59 to 284 and the known fraction from 0.46 to 0.78.
- At 60 s, `refuted`: "observed obstacles close every crossing at 0.40 m clearance".
- At that same 60 s replan (the verifier runs before the turn), Lightning sends agent 0 to candidate 26 at (−0.23, 3.07), the other
  way. The report's "next pick" column compares the following decision (65 s, 3.3 m away).

**Cost:** Lightning 138 calls, 3.10M / 11.6k tokens, USD 0.189; VL USD 0.060.

**Caveat:** end_to_end_rtf 0.122 on Mesa with other host load (FM). Latency figures are wall-clock under that load.
