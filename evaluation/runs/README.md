# Run evidence: lineage

This fork shares Ariadne's history and was cut at `0ca523c` ([D300](../../docs/decisions.md)). That puts the run directories here in
two groups.

| directories | era | the language model in the loop |
|---|---|---|
| `nemotron/` | **this fork**, from 2026-10-29 | NVIDIA Nemotron on Nebius Token Factory. Each run records provider, model ID, latency and tokens per call. |
| everything else: `arm64/`, `collaboration/`, `egress/`, `gates/`, `live-agentic/`, `live_pipeline_final/`, `mapping/`, `perception/`, `performance/`, `planning/`, `reconstruction/`, `retrieval/`, `robust-backend/`, `vio/` | **pre-fork Ariadne** (committed up to 2026-10-27) | none, or the earlier Amazon Bedrock chooser (Nova 2 Lite, DeepSeek fallback) in the agentic runs |

Rules that follow from this:
- **Pre-fork runs are never presented as Nemotron results.** Their estimation, collaboration, mapping, planning, egress and
  reconstruction numbers still describe the stack this fork runs on, so they may be cited as *inherited*. Any decision or
  chooser number among them (for example `live-agentic/`) is Bedrock-era, and is labelled so wherever it is cited.
- **A Nemotron claim needs a run under `nemotron/`** with a manifest (git commit, image digests, scene and seed, configs, provider
  and model IDs) and the logged live calls behind it.
- **Run files are kept byte for byte as the run wrote them.** Paths inside them are as they were at the time, including paths
  this fork has since removed from the tree; the commit each run names still has them. `tools/scrub/` never rewrites this
  directory, and `tools/scrub/check.sh` still holds it to the denylist and the secret scan.

The run-id format and directory layout are described in [`../README.md`](../README.md).
