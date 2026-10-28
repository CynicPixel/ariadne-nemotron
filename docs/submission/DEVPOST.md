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

On top of that stack, Nemotron's vision model (Nemotron 3.5 Super VL) looks at the map the robots built and their camera keyframes,
and proposes hypotheses about the planner's candidate goals, for example "this frontier opens onto a passage a responder can use".
On every replan, NVIDIA Nemotron, served by Nebius Token Factory, reads the planner's candidate tasks and the
open hypotheses through MCP tools. It assigns each robot a candidate and can name the hypothesis that move should test.

What the model never does:
- **It never invents a goal.** A C++ gate accepts only the planner's own candidates, each used exactly once, and holds risky ones
  for a human.
- **It never decides a hypothesis.** A deterministic verifier sets status from the occupancy the robots observed: unresolved,
  supported, confirmed, refuted or expired. Confirmed means confirmed on the map at that time: later map revisions can contradict
  it, and the operator console shows when they do.
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
  - Decision tier: `nvidia/Nemotron-3_5-Lightning`, chosen by a replay bench over recorded decisions, with
    `nvidia/nemotron-3-super-120b-a12b` only on transport errors; reasoning off.
  - Hypothesis tier: `nvidia/Nemotron-3.5-Super-VL` proposes up to four hypotheses per replan through a tool call, asynchronously,
    from the runtime's own map render and the keyframes. Deterministic seeding from the planner is the fallback.
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

First live run with the full loop (G1; one run, 90 simulated seconds, descriptive; slower than real time on a loaded host):
36 decisions, 32 from Nemotron Lightning and 4 classical fallbacks (all 4 from a stale-hypothesis rejection, fixed since);
21 hypotheses, 15 proposed by the VL model; 14 confirmed, 3 refuted, 3 supported, 1 unresolved at the end. In 15 of 23
hypothesis-targeted decisions the whole chain closed: targeted move, gate accept, motion, new observation, status change, a
different next pick. Model cost for the run: about USD 0.25.
[Run report](https://github.com/CynicPixel/ariadne-nemotron/tree/main/evaluation/runs/nemotron/g1_live-20261028T212942Z-2eda)

<!-- TODO(FM): the F4 table, descriptive and per seed: time to first verified resolution, egress-found time, fallback / invalid /
out-of-set rates, gate rejections (proposals rejected, counted apart from actions executed), latency p50/p95, tokens, cost.
Negative or mixed results are stated as they are. -->

## Challenges we ran into

- **Keeping the model honest about geometry.** In the first live call, the VL model's free-text reason cited a robot position the
  prompt never contained. That is why model text is never evidence, and hypothesis status comes only from the verifier.
- **A model can explain one goal and send another.** In our second live run (G2, t = 95 s), Lightning's reason described a
  low-risk frontier, but the candidate index it sent pointed elsewhere: risk 0.575 against a 0.5 threshold, through a 0.29 m
  bottleneck. The C++ gate checks the chosen candidate's own risk, not the model's words. It refused the pick without operator
  approval, and the robot took the classical choice instead. <!-- TODO(FM): link evaluation/runs/nemotron/g2_live-20261028T214744Z-b93e/
  (decisions/t95000ms.json, t95000ms.choice.json, the t=95 trace records) once FP commits it; check the facts against it -->
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

- Measuring the VL proposer against deterministic seeding on the same verifier, over more seeds than a hackathon allows
  (in G1 its terminal hit rate was 12 of 17, a small sample).
- Real-robot trials. Everything here is simulated.

## Built with

NVIDIA Nemotron · Nebius Token Factory · OpenAI Python SDK · MCP · C++17 · OpenCV 5 · ROS 2 Jazzy · Gazebo Harmonic · Python ·
FastAPI · gsplat · Docker · AWS EC2

## Links

- Code: https://github.com/CynicPixel/ariadne-nemotron (MIT)
- Video: <!-- TODO(FM): YouTube link -->
