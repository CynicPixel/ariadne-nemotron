# Evidence-loop report: `live-20261028T223308Z-9e2d`

- Agent decisions: 36, by source {'orchestrator': 34, 'fallback': 2}
- LLM: ['nvidia/Nemotron-3_5-Lightning']; 75 calls; latency p50 7.989 s, max 74.689 s; tokens 2527291/9844; est. USD 0.154
- Hypotheses: 25 ({'seed': 13, 'vl': 12}; 2 VL-annotated); final {'confirmed': 16, 'refuted': 3, 'supported': 6}; first verified resolution at t = 10 s
- VL calls: 17 {'ok': 15, 'zero_valid': 2}, est. USD 0.07171; hit rate 6/12 (confirmed/refuted only; expired/unresolved/supported excluded); resolved after creation 3/5
- G2 chains: 9 targeted, 9 complete, 0 with a refutation

| hyp | decision | agent | gate | motion | new slab | status change | next pick | complete |
|---|---|---|---|---|---|---|---|---|
| h001 | t5000ms | 0 | True | True | slab_10000.png | 15 s confirmed | 20 s, 1.3 m | True |
| h002 | t5000ms | 1 | True | True | slab_10000.png | 10 s confirmed | 15 s, 1.16 m | True |
| h001 | t10000ms | 0 | True | True | slab_15000.png | 15 s confirmed | 20 s, 2.04 m | True |
| h006 | t25000ms | 1 | True | True | slab_30000.png | 40 s confirmed | 45 s, 1.1 m | True |
| h011 | t45000ms | 0 | True | True | slab_50000.png | 55 s confirmed | 60 s, 1.31 m | True |
| h012 | t45000ms | 1 | True | True | slab_50000.png | 50 s confirmed | 55 s, 1.47 m | True |
| h011 | t50000ms | 0 | True | True | slab_55000.png | 55 s confirmed | 60 s, 1.58 m | True |
| h020 | t70000ms | 1 | True | True | slab_75000.png | 85 s supported | 90 s, 1.97 m | True |
| h020 | t75000ms | 1 | True | True | slab_80000.png | 85 s supported | 90 s, 1.97 m | True |
