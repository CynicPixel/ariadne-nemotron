| policy | run | end_s | free_cells_60s | free_cells_end | egress_found_t_s | flown_m_end | decisions_by_source | no_answer | proposals_rejected_by_gate | unsafe_actions_executed | approval_requests | llm_latency_p50_s | llm_latency_p95_s | llm_input_tokens | llm_est_usd | first_verified_resolution_t_s | chains_complete | vl_hit_rate |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| B0 | live-20261028T221048Z-d01b | 90 | 17616 | 18984 | - | [7.757737118, 8.040343449] | {'classical': 36} | {} | {} | 0 | 0 | - | - | 0 | 0 | - | - | - |
| B2 | live-20261028T222153Z-5da7 | 90 | 17364 | 19061 | - | [9.331983388, 7.261917733] | {'orchestrator': 36} | {} | {} | 0 | 0 | 5.6 | 9.77 | 1167673 | 0.072 | - | - | - |
| B4 | live-20261028T223308Z-9e2d | 90 | 18462 | 22668 | - | [8.93357608, 8.747015783] | {'orchestrator': 34, 'fallback': 2} | {'timeout or no orchestrator': 2} | {} | 0 | 3 | 7.99 | 73 | 2527291 | 0.154 | 10 | 9 | 6/12 (later 3/5) |

Per policy (median over runs; n shown):

- B0: n = 1; free cells at end median 18984; egress found in 0/1; proposals rejected by the gate 0; unsafe actions executed 0 (above threshold, no approval on record); approval requests 0 (answer-window crossings -> fallback 0)
- B2: n = 1; free cells at end median 19061; egress found in 0/1; proposals rejected by the gate 0; unsafe actions executed 0 (above threshold, no approval on record); approval requests 0 (answer-window crossings -> fallback 0)
- B4: n = 1; free cells at end median 22668; egress found in 0/1; proposals rejected by the gate 0; unsafe actions executed 0 (above threshold, no approval on record); approval requests 3 (answer-window crossings -> fallback 1)

Operator approval requests (unattended runs: no operator, each request blocks the tool's 60 s window; at RTF ~0.125 that is ~7.5 sim s with the agents on their previous command):

- B4 live-20261028T223308Z-9e2d: t80000ms agent 1 candidate 13, decision latency 74.689 s, answered: ok
- B4 live-20261028T223308Z-9e2d: t85000ms agent 0 candidate 37, decision latency 72.994 s, answered: ok
- B4 live-20261028T223308Z-9e2d: t90000ms agent 1 candidate 44, decision latency None s, FALLBACK (answer window crossed)

Caveats:
- Descriptive only: few runs, one scene (maze_short_6, repeats differ by free-running timing and model sampling); no
  significance claims.
- Primary metrics are in SIM time (t_s, map cells, distance flown). Every run is on a CPU-only Mesa dev box at
  ~0.12-0.13 real-time factor; wall-clock LLM latencies come from that box under load and are not deployment figures.
- "confirmed" / "refuted" mean confirmed / refuted on the map at that time (the deterministic slab verifier at 0.40 m
  responder clearance); later map revisions can contradict a settled status, and the trace records when they do
  (`contradiction` events, `contradicted` in the hypothesis view).
- Unsafe proposals rejected by the C++ gate are counted apart from unsafe actions executed (invariant 6); a missing
  answer (no assignment, timeout) is neither.
