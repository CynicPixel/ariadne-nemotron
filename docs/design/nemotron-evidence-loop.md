# Nemotron evidence-seeking loop (FP; design, decisions D310–D329)

Status: skeleton (FM, 2026-10-28). FP fills in Preflight and Decisions **before** writing code (design-preflight rule).
Plan items: `docs/process/PHASED_PLAN.md` F1–F2.

## 1. Preflight
- Inherited chooser: `docs/design/agentic.md`; `python/orchestration/{orchestrator,mcp_server,fake_orchestrator}.py`;
  the C++ gate is the `ariadne_live.cpp` chooser block (request ~L2125–2184, validation ~L2226–2340, `ChooserLink` ~L550–600).
- Live endpoint facts (from F0.5, D301): model IDs from `/v1/models`, base host, tool-calling and structured-output behaviour,
  reasoning-token behaviour, image input in tool loops.
- Literature/SOTA pointers for semantic active perception and hypothesis-driven exploration.

## 2. Decisions
- Provider interface; tiering (hypothesis = VL, async; decision = fast, reasoning off); the fallback chain.
- Hypothesis record schema; anchoring to planner cluster/candidate IDs (no free-form pixel boxes).
- Verifier rules and thresholds (slab-only, ρ_responder 0.40 m).
- Scene pre-screen protocol (GT used for scene choice only).

## 3. Acceptance (register F1/F2, gates G1/G2)
## 4. Tests
## 5. Cost (tokens per decision and per run, from the bench)
## 6. Result
