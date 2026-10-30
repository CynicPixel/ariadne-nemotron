# Project Ariadne

**Tagline:** Autonomous multi-agent collaborative visual SLAM and 3D reconstruction for communication-constrained disaster environments.

Have you heard of the Greek myth of Theseus and the Minotaur? An often overlooked detail is that Theseus could only make it back out of the Labyrinth because of the thread that marked his path, given to him by Princess Ariadne. The sword killed the monster. The thread is what got him home.

> A collapsed building is a labyrinth nobody has surveyed.

Power is out. Dust hangs in the air and scatters in the sparse light entering through the broken windows, the collapsed roof. Corridors have become voids between concrete and furniture. Someone may be trapped inside, and every hour the survival window narrows. The responders who go in are working from a guess. A map that arrives on day four is an archive. A map that arrives in hour three is a decision.

What incident command actually needs is a thread. A route wide enough for a person with a pack. A route that does not run under a slab that might come down. A way to reach the target and a way back out. Camera feeds and occupancy grids do not answer that question. **Our project is built to.**

---

## What we built

Ariadne sends a small team of agents into the space. Each carries a stereo camera, an inertial sensor, and its own light source. They cannot see each other for most of the mission. Radio does not get through rubble, so every byte costs something and every message has to earn its place. The agents cooperate anyway.

### Three coupled problems

**State estimation.** A stereo-inertial sliding window estimator written from scratch on OpenCV 5 primitives. On-manifold IMU preintegration, analytic Jacobians, Schur complement to eliminate landmarks and solve the reduced camera system. Each agent has a metric trajectory and honest covariance before it ever talks to a peer.

**What to trust.** When a link opens, agents trade compact place-recognition descriptors. Candidates are ranked by how much connectivity they add to the joint pose graph divided by the bytes they cost:

$$\text{priority}(e) = \frac{\Delta\lambda_2(L)}{\text{bytes}(e)}$$

Only the winners get their local features requested. Surviving matches pass a degeneracy check, pairwise consistency maximization, and graduated non-convexity with a truncated least squares objective:

$$w_i^{(k+1)} = \min\left(1,\ \frac{\mu\, c_i}{\lVert \bar{r}_i^{(k)} \rVert}\right)$$

before they are allowed to move the shared map. A single confident false match in a corridor of identical concrete is enough to fold a map in half, so trust is expensive and it is earned in layers.

**Where to go.** A voxel map with two distance fields. One treats unobserved space as open. The other, the one that drives the responder route, treats it as blocked. Because:

$$c(x,y) = \lVert x-y \rVert \left[1 + w_u(1-\text{support}) + w_r\, r + w_c\, \frac{\max(0,\ \rho_{\text{safe}} - \text{clearance})}{\rho_{\text{safe}}}\right]$$

Every legal edge costs at least its geometric length, so the heuristic stays consistent and the search never claims a shortcut through a wall. Route confidence is the fifth percentile of support along the path, which means one weakly observed segment can sink an otherwise confident route.

---

## What comes out the other end

An incremental search from every traversable exit keeps a valid path available as the map changes. A Gaussian splat reconstruction runs alongside so an incident commander can look into the space from angles no agent ever stood at. NVIDIA Nemotron, served on Nebius Token Factory, sits above the planner and picks among tasks the deterministic planner has already proved feasible, with every decision traced and high-risk actions gated behind operator approval. Nemotron's vision model reads the agents' own map and keyframes and proposes hypotheses (does this gap lead somewhere a responder could follow?); a geometry verifier confirms or refutes each one from what the agents actually observed. If the link to command goes down, agents continue within their pre-approved envelope rather than stopping dead.

The clearance decisions stay with the geometry. The splat is for looking. The route is for moving.

---

## The thread

The myth has one thread. The system has three.

> **Observations.** What the agents actually saw.
>
> **Verified geometry.** What survived the trust gauntlet.
>
> **The path home.** What a responder can actually walk.

Ariadne produces a shared model of a space nobody could safely enter, and a route through it that leads a responder in and back out again.

## What the experiments showed

Ariadne is still a research prototype, but the individual systems have been put through benchmarks, controlled ablations, and recorded multi-agent missions. A few results from the evaluation:

- **Visual SLAM:** Mean trajectory error of 6.6 cm across 11 EuRoC sequences, matching or outperforming published ORB-SLAM3 results on 7/11. ALIKED + LightGlue loop verification achieved a pose AUC@10° of 0.607, compared with 0.284 for a matched ORB baseline.
- **Collaborative mapping:** In matched perceptual-aliasing experiments, the known-entry consistency gate reduced merged-map error from 3.78 m to ≤1.5 cm (seed-mean), without rejecting any of 560 tested true loop closures. In a recorded gate-off mission, a false association displaced an agent by 15.3 m; with the gate enabled on the same scene and seed, all agents remained within 7.5 cm.
- **Robust estimation:** The pose-graph backend reproduced SE-Sync benchmark objectives and obtained optimality certificates on 7 public benchmark graphs under the tested chordal objective. Its solver also agreed with GTSAM on 100/100 synthetic graphs.
- **Exploration and egress:** Distributed task allocation reached 99.4% of the exact optimum on evaluated instances. All 12/12 responder-width routes passed simulated ground-truth validation, while incremental D* Lite agreed with fresh shortest-path searches across 180 obstacle updates.
- **3D reconstruction:** A three-agent reconstruction trained on AWS g4dn produced 2.59 million Gaussians, with 24.36 dB held-out-view PSNR on a procedurally generated maze.
- **Agentic orchestration:** In a 120 s live run, Nemotron 3.5 Lightning on Nebius Token Factory made 45 of 48 agent task decisions that executed. The deterministic gate refused one high-risk pick the model sent without operator approval, and the classical planner covered the remaining two. Across three matched 90 s runs, 0 unsafe actions were executed, and the geometry verifier confirmed 16 and refuted 3 hypotheses in the evidence-loop run.
- **Reproducibility:** The native Graviton4 build passed 708/708 C++ tests, with replay digests identical across x86 and arm64.

These results come from public datasets, controlled simulations, and recorded cloud executions—not physical disaster deployments. Some comparisons use published reference values rather than matched reruns, and known-entry localization assumes the agents' relative starting configuration is available. The Nemotron runs are single runs per policy on a CPU-only dev box, slower than real time; they are descriptive, not statistics.

The full methodology, ablations, unsuccessful experiments, and reproducible run records are available in the [evaluation results](https://github.com/CynicPixel/ariadne-nemotron/blob/main/docs/results/index.md), the [Nemotron run records](https://github.com/CynicPixel/ariadne-nemotron/tree/main/evaluation/runs/nemotron), and the [source repository](https://github.com/CynicPixel/ariadne-nemotron).

## Built With

aliked · nvidia-nemotron · nebius-token-factory · amazon-ec2 · amazon-ecr · amazon-web-services · c++ · cuda · docker · eigen · eigenplaces · gazebo · gsplat · lightglue · mcp · numpy · opencv · orb · particle · python · pytorch · ros-2 · scipy · terraform

## Try it out

- [GitHub Repo](https://github.com/CynicPixel/ariadne-nemotron)
- [Demo video](https://youtu.be/aoIGG4V3038)
