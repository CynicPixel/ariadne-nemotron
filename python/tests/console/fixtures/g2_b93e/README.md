# G2 run lines (console regression fixture)

Real lines, copied unchanged from FM's G2 live run `live-20261028T214744Z-b93e` (fork main, 2026-10-29). They are kept in the
run's own layout: `hypotheses.jsonl` sits in the outer run directory, and the runtime's `trace.jsonl` sits in its mission
subdirectory.
- `hypotheses.jsonl`: every event up to sim t = 95 s, including the run-level `vl_call` records (`hypothesis_id` null).
- `trace.jsonl`: the choice records at t = 90 s and t = 95 s.
  - Agent 0 carries `answer = {..., "orchestrator": {...}}`; at 95 s that is a `fallback` (the gate refused a pick above the risk
    threshold without approval).
  - Agent 1 carries `answer = "see agent 0's record"`, the string that made `/api/hypotheses` return HTTP 500.

The evidence report for the run itself is FP's, under `evaluation/runs/nemotron/`.
