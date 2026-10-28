# G2 live run: evidence-loop analysis (FP)

- **Run:** `live-20261028T214744Z-b93e`, run by FM on 2026-10-29 at code `f785424`. Scene `maze_short_6`, `live_llm_v1`, `mission_v2`,
  `configs/llm/nemotron.yaml`, **120 sim s**.
- **FM reports:** runtime exit 0, no OOM (this covers G1's 120 s OOM clause), RTF 0.129 on Mesa.
- **Files here:** `evidence_report.{md,json}` (`python -m orchestration.evidence_report`, at commit `f785424` + the fixes below).
  The `t95000ms/` case is the request, the hypotheses view, the recorded choice and both agents' trace records.

## Findings [VERIFIED from the run's own logs]

**Decisions and hypotheses:**
- 48 agent decisions: 45 `orchestrator`, 3 `fallback`. Every model call was `nvidia/Nemotron-3_5-Lightning` via Nebius.
- 27 hypotheses (17 VL, 10 seed; 5 VL-annotated). Final: 21 confirmed, 5 refuted, 1 supported.
- VL calls: 22 ok, 1 zero_valid. VL hit rate over terminal outcomes 14/21; on the predictive subset (resolved after creation) 6/9.
- G2 chain: 5 of 14 targeted decisions complete it, 3 of them through a refutation. h013: targeted for agent 0 at 60/65/70 s,
  refuted at 75 s, after which agent 0's next pick is 2.28 m away.

**Contradictions (53 events, 3 distinct):**
- h001, h002 and h003 were confirmed at 10–15 s. From 35–40 s on, every replan gave a `refuted` verdict.
- **Cause: real map revisions, not a logging bug.** In those regions, 12–17 cells observed *free* turn *occupied* between
  consecutive slabs (across the whole map, 4–37 such flips per replan). The maze corridors are not much wider than 2 × 0.40 m,
  so a few new occupied cells close a crossing.
- A stricter confirmation rule did not remove them: replayed on these slabs, it required every cell within ρ to be observed
  free, and h001–h003 were still confirmed first.
- **Fix (after this run):** a contradiction is logged once per change of the contradicting verdict, never per replan.
  `contradicted` is surfaced in `list_hypotheses` and the console view. Status keeps the first conclusive verdict.
- **Wording:** "confirmed" means *confirmed on the map at that time*; later map revisions can contradict it, and when they do,
  it is shown.

**t95000ms: an unsafe proposal rejected by the gate** (counted separately from executed actions, invariant 6). FM's reading
is correct:
- h020 is anchored to agent 0's candidate **0**: centroid (2.25, 8.75), risk 0, bottleneck 0.47 m, which is also the
  classical choice.
- Lightning's reason described that frontier ("highest-ranked unresolved supported hypothesis h020 (frontier at 2.25,8.75)
  … low risk 0"), but it sent index **53**: risk 0.575 > 0.5, bottleneck 0.29 m, centroid (2.93, 6.37), `needs_approval`.
- The record has `hypothesis_anchored: false`.
- The C++ gate refused ("above the risk threshold without an approval"), and agent 0 executed classical choice 0.
- **Fix:** `assign` now refuses a `hypothesis_id` paired with a candidate other than the one it is anchored to, and names the
  right index. It also tells the model to call `request_human_approval` before an above-threshold pick, instead of recording
  a pick the gate would refuse. The gate is unchanged and stays the authority.

**t120000ms (47.7 s, the max latency): degenerate output.**
- Lightning returned `H\n\n\n…` until `max_tokens` on 9 turns at ~66k context per call; no tool call, then a classical
  fallback.
- **Fix:** a cut-off turn with no tool call is never re-sent. One terse retry follows, and a second in a row ends the turn,
  so the fallback comes in seconds.

**Latency:**
- The second-slowest decision, t95000ms (34.8 s), was 4 calls at 160k prompt tokens. Agent 0 had 106 candidates, so
  `list_candidates` dominates the context.
- p50 is 8.05 s under host load. Not to be published without the RTF/load caveat.
