# Evaluation evidence

Every claimed number in the report and the README points to a directory here. A run directory holds what the run
wrote: its manifest (git commit, image digests, OpenCV build hash, configs, seeds), raw and processed metrics, and
usually a README with the verdict and the command that produced it.

## Layout: `runs/<subsystem>/<what>[_<date>][_<aws id>]/`

| subsystem | contents |
|---|---|
| `gates/` | end-to-end replay gates: the phase-1 thin pipeline, the phase-2 estimator gate, the AWS pre-merge gates (`premerge_aws/<aws run id>/`) |
| `vio/` | full EuRoC stereo-inertial runs on AWS |
| `performance/` | pipeline profile (per-stage timings, thread scaling, x86 vs Graviton) and DNN timings |
| `arm64/` | native Graviton4 builds and test suites (stock OpenCV 5.0.0) |
| `perception/radiometric/` | dark-world lighting, restoration rows, real-data (Hilti) check |
| `retrieval/` | EigenPlaces place recognition on EuRoC and across simulated agents |
| `robust-backend/` | SE-Sync certificate on benchmark graphs, outlier ramp, loop-closure noise model |
| `collaboration/` | known-entry gate sweep, byte table, wire coding, recorded live runs with the gate off and on |
| `mapping/` | volumetric map, traversability, risk calibration against ground truth |
| `planning/` | next-best-view gate, replay and ablation; CBBA allocation replay; route comparison |
| `egress/` | responder-width egress routes and hazard replanning against ground truth |
| `reconstruction/` | Gaussian-splat GPU training and renders, T4 benchmarks, live splat service |
| `live-agentic/` | live closed-loop runs, Bedrock-vs-classical chooser pairs, the final integrated stack, the 250 s demo recording |

## Run ids

AWS runs are named `YYYYMMDDTHHMMSSZ-<profile>-<6 hex>` (profile `cpux86` = c7i.4xlarge, `cpuarm` = c8g.4xlarge,
`gpu` = g4dn.xlarge; all spot). Inputs and full outputs stay in S3 at `s3://<artifact bucket>/runs/<id>/`; the
committed directories hold the evidence subset. `build/runs/runs.csv` (written by `./ariadne runs`) lists every AWS run with its instance,
duration, cost, exit code, committed evidence directory and S3 path (regenerate with `infra/scripts/runs_index.py`).

Recorded artifacts (manifests, results, logs) are kept byte-for-byte as the run wrote them, so paths inside them use
the names the repository had at the time (for example `docs/design/NBV_4.3.md` is now `docs/design/next-best-view.md`;
`evaluation/runs/collab_d178` is now `evaluation/runs/collaboration/known_entry_gate`).
