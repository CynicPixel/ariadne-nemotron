# Evidence-loop report: `live-20261028T212942Z-2eda`

- Agent decisions: 36, by source {'orchestrator': 32, 'fallback': 4}
- LLM: ['nvidia/Nemotron-3_5-Lightning']; 138 calls; latency p50 10.726500000000001 s, max 13.857 s; tokens 3095177/11647; est. USD 0.18851
- Hypotheses: 21 ({'seed': 6, 'vl': 15}; 5 VL-annotated); final {'confirmed': 14, 'refuted': 3, 'supported': 3, 'unresolved': 1}; first verified resolution at t = 10 s
- VL calls: 16 {'ok': 16}, est. USD 0.05954; hit rate 12/17 (confirmed/refuted only; expired/unresolved/supported excluded); resolved after creation 5/8
- G2 chains: 23 targeted, 15 complete, 5 with a refutation

| hyp | decision | agent | gate | motion | new slab | status change | next pick | complete |
|---|---|---|---|---|---|---|---|---|
| h001 | t5000ms | 0 | True | True | slab_10000.png | 15 s confirmed | 20 s, 0.26 m | False |
| h002 | t5000ms | 1 | True | True | slab_10000.png | 15 s confirmed | 20 s, 1.74 m | True |
| h001 | t10000ms | 0 | True | True | slab_15000.png | 15 s confirmed | 20 s, 0.12 m | False |
| h002 | t10000ms | 1 | True | True | slab_15000.png | 15 s confirmed | 20 s, 1.4 m | True |
| h006 | t20000ms | 0 | True | True | slab_25000.png | 80 s confirmed | 85 s, 5.35 m | True |
| h006 | t25000ms | 0 | True | True | slab_30000.png | 80 s confirmed | 85 s, 5.32 m | True |
| h010 | t35000ms | 0 | True | True | slab_40000.png | 60 s refuted | 65 s, 1.91 m | True |
| h010 | t40000ms | 0 | True | True | slab_45000.png | 60 s refuted | 65 s, 2.5 m | True |
| h010 | t45000ms | 0 | True | True | slab_50000.png | 60 s refuted | 65 s, 3.32 m | True |
| h010 | t50000ms | 0 | True | True | slab_55000.png | 60 s refuted | 65 s, 2.45 m | True |
| h010 | t55000ms | 0 | True | True | slab_60000.png | 60 s refuted | 65 s, 3.32 m | True |
| h006 | t60000ms | 0 | True | True | slab_65000.png | 80 s confirmed | 85 s, 5.56 m | True |
| h006 | t65000ms | 0 | True | True | slab_70000.png | 80 s confirmed | 85 s, 5.57 m | True |
| h006 | t70000ms | 0 | True | True | slab_75000.png | 80 s confirmed | 85 s, 2.46 m | True |
| h018 | t70000ms | 1 | True | True | slab_75000.png | 80 s confirmed | 85 s, 2.43 m | True |
| h006 | t75000ms | 0 | True | True | slab_80000.png | 80 s confirmed | 85 s, 2.48 m | True |
| h018 | t75000ms | 1 | True | True | slab_80000.png | 80 s confirmed | 85 s, 1.96 m | True |
| h019 | t80000ms | 0 | True | True | slab_85000.png | - | - | False |
| h015 | t80000ms | 1 | True | True | slab_85000.png | - | - | False |
| h019 | t85000ms | 0 | True | True | slab_90000.png | - | - | False |
| h021 | t85000ms | 1 | True | True | slab_90000.png | 90 s unresolved | - | False |
| h019 | t90000ms | 0 | True | True | - | - | - | False |
| h021 | t90000ms | 1 | True | True | - | - | - | False |
