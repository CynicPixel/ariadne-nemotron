# syntax=docker/dockerfile:1.7
# ---------------------------------------------------------------------------
# ariadne-orchestrator (arch §25, design agentic.md D156-D160; fork D310-D317): the chooser's tool loop (Nemotron on
# Nebius Token Factory via the OpenAI SDK; Bedrock kept as a comparator) and the MCP server over the deterministic
# core's decision requests, plus the slab verifier (numpy + Pillow read the raw slab PNG). Pins: base by digest (the
# console's), every package exact. Tag: ariadne-orchestrator:nemotron-1.
# The repo is bind-mounted at /work (read-only); the run directory is mounted writable (decisions/, approvals/).
# Credentials come from the environment only: NEBIUS_API_KEY via `docker run --env-file` (live.sh), or the AWS chain
# for Bedrock. Nothing secret is baked into the image.
# ---------------------------------------------------------------------------
FROM python@sha256:f77ac9e44ae96ef2c90b8053ea08c31f8be030f824196b0ae4db6d462c84e51f
RUN pip install --no-cache-dir --root-user-action=ignore boto3==1.43.118 mcp==2.3.0 pytest==9.1.1 \
        openai==3.40.0 httpx2==2.16.0 PyYAML==6.0.3 numpy==2.5.4 pillow==13.0.0
ENV PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=/work/python
WORKDIR /work
