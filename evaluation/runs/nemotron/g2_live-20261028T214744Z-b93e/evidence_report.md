# Evidence-loop report: `live-20261028T214744Z-b93e`

- Agent decisions: 48, by source {'orchestrator': 45, 'fallback': 3}
- LLM: ['nvidia/Nemotron-3_5-Lightning']; 98 calls; latency p50 8.052 s, max 47.721 s; tokens 3472448/16312; est. USD 0.21226
- Hypotheses: 27 ({'seed': 10, 'vl': 17}; 5 VL-annotated); final {'confirmed': 21, 'refuted': 5, 'supported': 1}; first verified resolution at t = 10 s
- VL calls: 23 {'ok': 22, 'zero_valid': 1}, est. USD 0.09921; hit rate 14/21 (confirmed/refuted only; expired/unresolved/supported excluded); resolved after creation 6/9
- G2 chains: 14 targeted, 5 complete, 3 with a refutation

| hyp | decision | agent | gate | motion | new slab | status change | next pick | complete |
|---|---|---|---|---|---|---|---|---|
| h001 | t5000ms | 0 | True | True | slab_10000.png | 15 s confirmed | 20 s, 0.35 m | False |
| h002 | t5000ms | 1 | True | True | slab_10000.png | 10 s confirmed | 15 s, 1.02 m | True |
| h001 | t10000ms | 0 | True | True | slab_15000.png | 15 s confirmed | 20 s, 0.72 m | True |
| h013 | t60000ms | 0 | True | True | slab_65000.png | 75 s refuted | 80 s, 2.28 m | True |
| h016 | t60000ms | 1 | True | True | slab_65000.png | 75 s confirmed | 80 s, 0.41 m | False |
| h013 | t65000ms | 0 | True | True | slab_70000.png | 75 s refuted | 80 s, 2.28 m | True |
| h016 | t65000ms | 1 | True | True | slab_70000.png | 75 s confirmed | 80 s, 0.41 m | False |
| h013 | t70000ms | 0 | True | True | slab_75000.png | 75 s refuted | 80 s, 2.28 m | True |
| h016 | t70000ms | 1 | True | True | slab_75000.png | 75 s confirmed | 80 s, 0.41 m | False |
| h020 | t95000ms | 0 | False | True | slab_100000.png | 120 s confirmed | - | False |
| h020 | t100000ms | 0 | True | True | slab_105000.png | 120 s confirmed | - | False |
| h020 | t105000ms | 0 | True | True | slab_110000.png | 120 s confirmed | - | False |
| h020 | t110000ms | 0 | True | True | slab_115000.png | 120 s confirmed | - | False |
| h020 | t115000ms | 0 | True | True | slab_120000.png | 120 s confirmed | - | False |
