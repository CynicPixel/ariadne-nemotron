# Lightning bench with the one-turn inspection nudge (FP, 2026-10-29 ~05:45 IST)

This is the same corpus and the same 10 requests as `bench_20261028T211611Z`, run on `nvidia/Nemotron-3_5-Lightning` only, at commit
`bb57a92`. The one change is the first user prompt, which now ends: "Request everything you need in ONE turn (several tool calls at
once), then call assign."

**Why.** In the G1 live run (`live-20261028T212942Z-2eda`), Lightning spent ~7 turns on single-tool calls before assigning.
Each turn re-sent the growing history, so decisions cost 100–200k prompt tokens.

**Result [VERIFIED, this run]** (Lightning, the same 10 recorded requests):

| | calls/decision | mean prompt tokens | p50 / p95 latency | USD (10 req) | valid agent decisions |
|---|---|---|---|---|---|
| before (`bench_20261028T211611Z`) | 6.1 | 111k | 6.42 / 8.13 s | 0.067 | 20/20 |
| with the nudge (this run) | 2.8 | 60.5k | 4.26 / 6.68 s | 0.037 | 20/20 |

n = 10 requests from one recorded run. This is descriptive. Reference agreement with the classical choice was 15/20 (13/20 before);
that is not an accuracy measure.
