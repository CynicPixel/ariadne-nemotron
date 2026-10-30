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
  - `hypotheses.jsonl` and `verifier.jsonl` line up with the decision trace. The operator console's hypotheses panel shows them
    live, with both models' IDs and latencies, and replays them in mission time for a finished run.

- **Evaluation.**
  - A replay bench picks the decision model.
  - A matched evaluation compares a classical baseline, Nemotron without hypotheses and Nemotron with hypotheses, per seed.
  - One scene (`maze_short_6`), 90 s per run, one run per policy (n = 1), on a CPU-only dev box at about 0.13× real time.
- **Compute.** The simulator runs on AWS EC2 or a local GPU. Inference is on Token Factory.

## Results

First live run with the full loop (G1; one run, 90 simulated seconds, descriptive; slower than real time on a loaded host):
36 decisions, 32 from Nemotron Lightning and 4 classical fallbacks (all 4 from a stale-hypothesis rejection, fixed since);
21 hypotheses, 15 proposed by the VL model; 14 confirmed, 3 refuted, 3 supported, 1 unresolved at the end. In 15 of 23
hypothesis-targeted decisions the whole chain closed: targeted move, gate accept, motion, new observation, status change, a
different next pick. Model cost for the run: about USD 0.25.
[Run report](https://github.com/CynicPixel/ariadne-nemotron/tree/main/evaluation/runs/nemotron/g1_live-20261028T212942Z-2eda)

Matched comparison, one 90-second run per policy on one scene (n = 1 each: descriptive, no ranking claim):
- **Evidence loop (B4):** Nemotron 3.5 Lightning made 34 of 36 agent decisions via Nebius Token Factory. Super-VL and the
  planner's top candidates seeded 25 hypotheses; the slab verifier (0.40 m responder clearance) confirmed 16 and refuted 3,
  the first at t = 10 s. All 9 hypothesis-targeted decisions completed the hypothesis → decision → gate → motion → verdict →
  next-pick chain.
- **Safety:** the C++ gate executed 0 unsafe actions in all three runs. Nemotron asked the (absent) operator to approve 3 risky
  picks; each waited the full 60 s window, and one crossed the 120 s answer window and fell back to the classical choice.
- **Coverage:** mapped free cells at 90 s were 18,984 (classical), 19,061 (Nemotron) and 22,668 (Nemotron + evidence loop).
  **No run found a responder-width egress route within 90 s.**
- **Cost:** decision-tier LLM USD 0.072 (Nemotron) and USD 0.154 (evidence loop) per 90 s run, plus USD 0.072 for the VL tier.
- "Confirmed" means confirmed on the map at that time; later map revisions can contradict it, and the console shows when they do.
  [F4 evidence](https://github.com/CynicPixel/ariadne-nemotron/tree/main/evaluation/runs/nemotron/f4_20261028T221048Z)

## Challenges we ran into

- **Keeping the model honest about geometry.** In the first live call, the VL model's free-text reason cited a robot position the
  prompt never contained. That is why model text is never evidence, and hypothesis status comes only from the verifier.
- **A model can explain one goal and send another.** In our second live run (G2, t = 95 s), Lightning's reason described a
  low-risk frontier, but the candidate index it sent pointed elsewhere: risk 0.575 against a 0.5 threshold, through a 0.29 m
  bottleneck. The C++ gate checks the chosen candidate's own risk, not the model's words. It refused the pick without operator
  approval, and the robot took the classical choice instead.
  [G2 evidence](https://github.com/CynicPixel/ariadne-nemotron/tree/main/evaluation/runs/nemotron/g2_live-20261028T214744Z-b93e)
- **Model IDs that differ between sources** (casing, `3_5` vs `3.5`). The config takes IDs only from the live endpoint's own listing.
- **A fixed budget.** The credit is finite, so a ledger prices every call and refuses all calls at a hard cap.
- **No GPU capacity for the live recording.** AWS spot placement for g4dn.xlarge scored 1/10 in our region during the window,
  so every live run ran CPU-only on a dev box at about 0.13× real time. The video's console footage is a 1× replay of a recorded
  run, labelled as such.

## Accomplishments that we're proud of

- A language model in a robot loop that can be wrong without being dangerous: every proposal is checked, every claim is verified
  from observed geometry, and every failure is counted.
- An open, reproducible stack. Every number links to a run with a manifest, and replays are digest-checked.

## What we learned

- **A model's stated reason is not evidence.** Twice the model's text disagreed with the facts: a VL reason at G0 cited a robot
  position the prompt never contained, and in G2 a decision's reason described one goal while its index pointed at another.
  Both were caught only because the gate and the verifier read the real numbers, not the words.
- **"Confirmed" is a statement about a map that keeps changing.** In G2, map revisions later contradicted three settled
  hypotheses. We now surface the contradiction instead of hiding it or flipping the status.
- **Human approval has a cost in a live loop.** With no operator present in F4, each of three risky picks blocked for 60 s, and
  one decision missed its answer window.

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
