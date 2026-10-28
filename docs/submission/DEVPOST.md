# Devpost project text (F6.2, draft)

Track: **Physical AI**. Every number below must come from a run under `evaluation/runs/` (Nemotron numbers: `evaluation/runs/nemotron/`
only). FM fills the form answers in `FORM_ANSWERS.md`.

---

## Tagline

Robot teams that seek evidence before people go in: NVIDIA Nemotron on Nebius Token Factory chooses, observed geometry decides.

## Inspiration

After a structural collapse, the spaces that most need searching are the ones people cannot safely enter: dark, cluttered, with no
GPS and almost no radio. Robots can go first, but a map alone does not say where to look next, and a language model alone should
never be trusted to say where a person can walk.

## What it does

A team of simulated stereo-camera robots maps a collapsed structure together, and hands incident command an egress route checked at
responder width (0.40 m).

On top of that stack, the planner's most promising frontiers become open hypotheses, for example "this frontier leads to open space
a responder can pass". On every replan, NVIDIA Nemotron, served by Nebius Token Factory, reads the planner's candidate tasks and the
open hypotheses through MCP tools. It assigns each robot a candidate and can name the hypothesis that move should test.

What the model never does:
- **It never invents a goal.** A C++ gate accepts only the planner's own candidates, each used exactly once, and holds risky ones
  for a human.
- **It never decides a hypothesis.** A deterministic verifier sets status from the occupancy the robots observed: unresolved,
  supported, confirmed, refuted or expired.
- **It never computes a route.** Routes come from the planner.
- **It never sees simulator ground truth.**

Every call is logged with model ID, latency and tokens. Spend is capped by a ledger. If the model fails, the robots continue on the
classical chooser.

## How we built it

- **Base: Ariadne.** It is our own stack in C++17 on OpenCV 5. We wrote the stereo-inertial VIO, the robust collaborative pose
  graph that refuses aliased loops, TSDF/ESDF mapping, CBBA allocation and D* Lite egress. Gaussian-splat reconstruction uses
  gsplat, and the world runs in Gazebo Harmonic under ROS 2 Jazzy.
- **Nemotron provider.** It talks to Token Factory through its OpenAI-compatible API.
  - Model IDs, prices and caps live in `configs/llm/nemotron.yaml`, copied from the endpoint's own `/v1/models` listing.
  - Decision tier: `nvidia/nemotron-3-super-120b-a12b`, with `nvidia/Nemotron-3_5-Lightning` as transport fallback, reasoning
    off. <!-- TODO(FM): the bench's final pick and order -->
  - `nvidia/Nemotron-3.5-Super-VL` was verified live on an occupancy-map image at G0. <!-- TODO(FM): say whether the VL tier ran in the recorded runs (vl_proposer) -->
- **Evidence loop.**
  - Hypotheses are anchored to planner candidates and re-associated by centroid across replans.
  - The verifier tests responder-clearance crossings on the observed occupancy slab.
  - `hypotheses.jsonl` and `verifier.jsonl` line up with the decision trace. <!-- TODO(FM): add "and the operator console
    shows them live" only if the F3.5 panel ships -->

- **Evaluation.**
  - A replay bench picks the decision model.
  - A matched evaluation compares a classical baseline, Nemotron without hypotheses and Nemotron with hypotheses, per seed.
  <!-- TODO(FM): seeds, scenes, duration -->
- **Compute.** The simulator runs on AWS EC2 or a local GPU. Inference is on Token Factory.

## Results

<!-- TODO(FM): the F4 table, descriptive and per seed: time to first verified resolution, egress-found time, fallback / invalid /
out-of-set rates, gate rejections (proposals rejected, counted apart from actions executed), latency p50/p95, tokens, cost.
Negative or mixed results are stated as they are. -->

## Challenges we ran into

- **Keeping the model honest about geometry.** In the first live call, the VL model's free-text reason cited a robot position the
  prompt never contained. That is why model text is never evidence, and hypothesis status comes only from the verifier.
- **Model IDs that differ between sources** (casing, `3_5` vs `3.5`). The config takes IDs only from the live endpoint's own listing.
- **A fixed budget.** The credit is finite, so a ledger prices every call and refuses all calls at a hard cap.
- **GPU capacity for the live recording.** <!-- TODO(FM): what happened, stated plainly (F5) -->

## Accomplishments that we're proud of

- A language model in a robot loop that can be wrong without being dangerous: every proposal is checked, every claim is verified
  from observed geometry, and every failure is counted.
- An open, reproducible stack. Every number links to a run with a manifest, and replays are digest-checked.

## What we learned

<!-- TODO(FM): one or two lessons from the F4 numbers, e.g. where hypotheses changed decisions and where they did not. -->

## What's next

- Turning the optional VL annotation into a tested hypothesis proposer, judged against the deterministic seeding on the same
  verifier.
- Real-robot trials. Everything here is simulated.

## Built with

NVIDIA Nemotron · Nebius Token Factory · OpenAI Python SDK · MCP · C++17 · OpenCV 5 · ROS 2 Jazzy · Gazebo Harmonic · Python ·
FastAPI · gsplat · Docker · AWS EC2

## Links

- Code: https://github.com/CynicPixel/ariadne-nemotron (MIT)
- Video: <!-- TODO(FM): YouTube link -->
