| model | requests | answered | agent_decisions | valid | holds | out_of_set | gate_rejected | same_cluster | latency_p50_s | latency_p95_s | calls_mean | input_tokens_mean | output_tokens_mean | reasoning_tokens_total | est_usd_total | errors | ref_agree_classical | ref_agree_recorded_bedrock |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `nvidia/nemotron-3-super-120b-a12b` | 10 | 10 | 20 | 20 | 0 | 0 | 0 | 0 | 7.95 | 26.9 | 3.7 | 8.77e+04 | 778 | 0 | 0.27 | 2 | 11 | 12 |
| `nvidia/Nemotron-3_5-Lightning` | 10 | 10 | 20 | 20 | 0 | 0 | 0 | 0 | 6.42 | 8.13 | 6.1 | 1.11e+05 | 440 | 0 | 0.0674 | 0 | 13 | 16 |
| `nvidia/NVIDIA-Nemotron-3-Nano-30B-A3B` | 10 | 1 | 2 | 2 | 0 | 0 | 0 | 0 | 27 | 45.9 | 9.8 | 4.63e+04 | 1407 | 0 | 0.0312 | 0 | 2 | 2 |

`ref_agree_*` count agent decisions equal to the classical / recorded Bedrock-era choice: reference only, not accuracy.
