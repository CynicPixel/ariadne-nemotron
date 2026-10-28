# Ariadne documentation map

This is the reading order for engineers and for coding agents building context on the code. Each document is the
source of truth for its area. Code comments cite it as `<doc>.md §N`, the architecture as `arch §N`, and decisions
by their id (`D178`).

## 1. System level

| document | what it holds |
|---|---|
| [`architecture.md`](architecture.md) | **The architecture**, cited as "arch §N": sensor rig, every subsystem's design, data contracts, the AWS deployment, the evaluation protocol. §0.2 records the locked decisions and why each was made. |
| [`algorithms.md`](algorithms.md) | Algorithm research behind each choice: the literature, published benchmark numbers, licences, and the alternatives that were rejected. |
| [`decisions.md`](decisions.md) | The engineering decision log, D1–D235. Each entry gives context, the decision, the evidence (run ids, tests) and the consequences, negative results included. |
| [`build.md`](build.md) | Build, test, run and deploy instructions, pinned dependencies, repository layout. |
| [`validation.md`](validation.md) | Environment and dependency verification: OpenCV 5 module availability, simulator and GPU rendering, AWS services, determinism across x86 and Graviton. |

## 2. Design, by subsystem (`design/`)

Order follows the data path, from the camera to the route home.

| subsystem | document |
|---|---|
| Visual front end: ORB/FAST + pyramidal LK tracking, XFeat (cv::dnn), SGBM/WLS stereo, lost-track re-association | [`design/perception.md`](design/perception.md) |
| Radiometric front end: flat field, variance-stabilising denoise, exposure normalisation; degraded (dark, dusty) environments | [`design/radiometric.md`](design/radiometric.md) |
| Local SLAM: windowed bundle adjustment (Schur-complement LM), keyframe policy, landmark lifecycle, lost-frame bridge, estimator build plan | [`design/local-slam.md`](design/local-slam.md) |
| Inertial layer: attitude-only and full Forster preintegration in the window, initialisation, gravity priors | [`design/inertial.md`](design/inertial.md) |
| Sparse SE(3) pose-graph solver: chordal initialisation, Levenberg–Marquardt | [`design/pose-graph.md`](design/pose-graph.md) |
| Robust back end: degeneracy gate, PCM, GNC-TLS, SE-Sync-style optimality certificate | [`design/robust-backend.md`](design/robust-backend.md) |
| Place recognition: EigenPlaces (cv::dnn), descriptor compression, retrieval across agents | [`design/retrieval.md`](design/retrieval.md) |
| Multi-agent collaboration: two-stage exchange, inter-agent verification (ALIKED + LightGlue), elected broker, merging | [`design/collaboration.md`](design/collaboration.md) |
| Collaboration inside the live runtime: asynchronous, event-driven, worker processes | [`design/live-collaboration.md`](design/live-collaboration.md) |
| Radio model: RF link with wall attenuation, partitions, per-size packet loss | [`design/comms.md`](design/comms.md) |
| Spectral (λ₂) prioritisation of feature requests under a byte budget | [`design/spectral-selection.md`](design/spectral-selection.md) |
| Local-feature wire coding (int8/int4/binary) on real verification pairs | [`design/wire-coding.md`](design/wire-coding.md) |
| Volumetric mapping: voxel-hash TSDF, tri-state occupancy, incremental ESDF, frontiers | [`design/volumetric.md`](design/volumetric.md) |
| Traversability at robot and responder width; structural-risk proxy | [`design/traversability-risk.md`](design/traversability-risk.md) |
| Next-best-view: candidate generation, ray-cast information gain, energy-reserve rule | [`design/next-best-view.md`](design/next-best-view.md) |
| Task allocation: event-triggered CBBA and broker sequential greedy | [`design/allocation.md`](design/allocation.md) |
| Egress: responder-width lattice, multi-exit D* Lite, incremental replanning | [`design/egress.md`](design/egress.md) |
| Exploration scheduling (TARE-style) and visibility-graph motion (FAR-style) | [`design/exploration.md`](design/exploration.md) |
| Mission layer: occupancy, frontier selection, egress route | [`design/mission.md`](design/mission.md) |
| Gaussian-splat reconstruction: keyframe-anchored submaps, GPU training, live service | [`design/reconstruction.md`](design/reconstruction.md) |
| Live closed-loop runtime: Gazebo ↔ ROS 2 bridge ↔ estimator, lockstep, multi-agent pipeline, consoles | [`design/live-runtime.md`](design/live-runtime.md) |
| Agentic layer: Bedrock + MCP chooser, C++ validation, risk gate, decision trace | [`design/agentic.md`](design/agentic.md) |
| Procedural worlds, kinematic mover, MCAP recording | [`design/simulation.md`](design/simulation.md) |
| Evaluation harness and the end-to-end replay gate | [`design/evaluation-harness.md`](design/evaluation-harness.md) |

## 3. Results

| document | what it holds |
|---|---|
| [`results/index.md`](results/index.md) | **The best result per subsystem**, each with its comparator, evidence path and scope, plus the list of things not to claim (failed gates and negative results). |
| [`results/planning.md`](results/planning.md), [`results/radiometric.md`](results/radiometric.md), [`results/reconstruction.md`](results/reconstruction.md), [`results/wire-coding.md`](results/wire-coding.md) | Per-area result write-ups |
| [`failure-cases.md`](failure-cases.md) | Failure cases and limitations |
| [`../evaluation/README.md`](../evaluation/README.md) | The run-evidence layout and run ids |

The original proposal is [`proposal.pdf`](proposal.pdf).
