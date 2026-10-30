# Run evidence: which runs back which claims

| directories | what they are | the language model in the loop |
|---|---|---|
| `nemotron/` | the Nemotron evaluation (G0 calls, the decision-tier bench, the G1/G2 live runs, the F4 matched comparison) | NVIDIA Nemotron on Nebius Token Factory. Each run records provider, model ID, latency and tokens per call. |
| everything else: `arm64/`, `collaboration/`, `egress/`, `gates/`, `live-agentic/`, `live_pipeline_final/`, `mapping/`, `perception/`, `performance/`, `planning/`, `reconstruction/`, `retrieval/`, `robust-backend/`, `vio/` | earlier development runs (up to 2026-10-27) | none, or a different LLM provider (Amazon Bedrock) in `live-agentic/` |

Rules that follow from this:
- **Earlier development runs are never presented as Nemotron results.** Their estimation, collaboration, mapping, planning,
  egress and reconstruction numbers describe the same stack the Nemotron runs use, so they may be cited for those subsystems.
  Any decision or chooser number among them (`live-agentic/`) used a different LLM provider and is labelled so wherever it is
  cited.
- **A Nemotron claim needs a run under `nemotron/`** with a manifest (git commit, image digests, scene and seed, configs, provider
  and model IDs) and the logged live calls behind it.
- **Run files are kept byte for byte as the run wrote them.** Paths inside them are as they were at the time, including paths
  since removed from the tree; the commit each run names still has them. `tools/scrub/` never rewrites this directory, and
  `tools/scrub/check.sh` still holds it to the denylist and the secret scan.

The run-id format and directory layout are described in [`../README.md`](../README.md).
