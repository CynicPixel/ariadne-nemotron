# ARIADNE — End-to-End Technical Architecture & Scoped Implementation Plan

**Project:** Ariadne — collaborative visual reconstruction and risk-weighted egress mapping for autonomous agents in GPS-denied, communication-constrained, visually degraded post-disaster structures.

**Purpose of this document:** Convert `algorithms.md(1).md` into an implementation-ready architecture, module boundary specification, algorithm selection, simulator/deployment stack, staged development program, test strategy, evaluation harness, and AWS/COOL deployment design.

**Design stance:** ambitious by design. The implementation should use the strongest technically defensible methods identified in the algorithm research, reimplemented from the papers where appropriate. No algorithm is excluded merely because OpenCV 5 does not expose a one-call API: OpenCV 5 is the primitive substrate; Ariadne owns the system-level implementations.

**Important:** This document is an implementation specification, not a claim that every component is already implemented. Each algorithm is tied to a validation test and a staged integration checkpoint.

**Document version:** v2. This revision merges the execution-layer gap fill into the master architecture: every
module that previously specified *what* without *how* now carries implementation-precision detail, the inertial
layer and the volumetric mapping layer exist as first-class sections, and the eight internal inconsistencies
recorded in §0.2 are resolved in place. There is no separate addendum; this document is the single source of
truth for implementation.

**Evidence tags used throughout:**

- `[VERIFIED web 2026-08-30]` — checked against a live primary source during the v2 revision.
- `[PAPER]` — sourced from the cited paper. The expression given here is a faithful *sketch*, not a transcription;
  sign and convention errors are the classic transcription failure.
- `[DESIGN]` — an Ariadne design decision, internally consistent but not externally sourced.
- `[SMOKE-TEST]` — must be confirmed against the installed binary on day 1; do not trust this document for it.

**Standing instruction to build agents:** for every equation tagged `[PAPER]`, open the paper, transcribe the
exact expression, and land the corresponding unit test from §37 in the same commit. Every `[SMOKE-TEST]` item is a
day-1 task whose result is written into `docs/validation.md` and into the run-manifest schema *before* any
dependent code is written.

---

# 0. Executive architectural decision

Ariadne should be built as a **vision-primary, stereo-inertial, multi-agent system with classical geometric state estimation and learned visual primitives**, not as an end-to-end learned SLAM stack.

The canonical runtime path is:

```text
Gazebo Harmonic / ROS 2 Jazzy simulation
        |
        v
per-agent stereo RGB + IMU + co-mounted-light streams
        |
        v
Radiometric calibration + low-light/noise normalization
   (clean render; degradation applied by a seeded operator at consumption time)
        |
        v
IMU preintegration -> motion prediction + gravity direction
        |
        v
LK sparse tracking every frame; XFeat extraction at keyframes
   (ALIKED + LightGlue only on hard inter-agent candidate pairs, never at frame rate)
        |
        v
stereo geometry + PnP/essential geometry + local landmark map
        |
        v
windowed SE(3) bundle adjustment (reprojection + inertial factors)
        |
        v
local pose graph + uncertainty
        |
        +-----------------------------+
        |                             |
        v                             v
EigenPlaces global retrieval      local map / frontier state
        |                             |
        v                             v
candidate inter-agent matches     active perception candidate generation
        |                             |
        v                             v
USAC -> geometric verification -> spectral communication prioritization
        |                             |
        +--------------+--------------+
                       |
                       v
              PCM pre-filter
                       |
                       v
                GNC robust PGO
                       |
                       v
         elected-broker global map state
                       |
             +---------+---------+
             |                   |
             v                   v
      reconstruction       traversability /
      / Gaussian maps      uncertainty volume
             |                   |
             +---------+---------+
                       |
                       v
       risk-aware multi-agent task allocation
                       |
                       v
               next observations
                       |
                       v
             continuous closed loop

Parallel operational plane:
    human approval / failure supervisor / observability / byte accounting
```

## 0.1 Why this is the chosen shape

The algorithm catalogue converges independently on the same architectural pattern from several directions:

- SVO/ORB-SLAM3-style systems motivate inexpensive frame-to-frame tracking plus expensive keyframe processing.
- Photo-SLAM separates classical localization from photorealistic mapping.
- EC3R-SLAM separates a lightweight sparse tracker from expensive feed-forward dense reconstruction.
- MAGS-SLAM demonstrates the emerging feasibility of **RGB-only multi-agent Gaussian SLAM**, using local maps and compact summaries.
- Swarm-SLAM demonstrates that communication-aware sparse collaboration can be more efficient than repeatedly running distributed back-end optimization.
- SubT experience shows that autonomy and disconnection tolerance matter more than assuming a perfect network.

These are not interchangeable systems; Ariadne synthesizes their strongest mechanisms while preserving a deliberately classical, inspectable geometric core.

References:
- XFeat: https://arxiv.org/abs/2404.19174
- LightGlue: https://arxiv.org/abs/2306.13643
- Photo-SLAM: https://arxiv.org/abs/2311.16728
- RTG-SLAM: https://arxiv.org/abs/2404.19706
- EC3R-SLAM: https://arxiv.org/abs/2510.02080
- MAGS-SLAM: https://arxiv.org/abs/2605.10760
- Swarm-SLAM: https://arxiv.org/abs/2301.06230
- OpenCV 5: https://github.com/opencv/opencv/wiki/OpenCV-5

---

## 0.2 Revision record — internal inconsistencies resolved in v2

Eight defects in the first architecture pass were blocking or misleading for implementation. Each is now resolved
in the body of this document; the register is retained because the *reasoning* is load-bearing and because a build
agent reading a single section should be able to see why it says what it says.

**C1 — The IMU was claimed but never implemented (was blocking).** §0 declared Ariadne "stereo-inertial", Plane A
produced IMU streams, §12.1 defined a `GRAVITY_PRIOR` edge, and Correction B listed gravity priors as a benefit of
the stereo rig — but no section specified preintegration, bias states, gravity alignment, or how an inertial
residual entered BA or the pose graph. The IMU was produced by the simulator and consumed by nothing, and the
`GRAVITY_PRIOR` edge had no producer. **Resolved:** §10 (Inertial layer), staged as Iteration 3.5, with a reduced
fallback mode (§10.7) that still produces the gravity prior and motion prediction if that iteration slips. Under
no circumstances may "stereo-inertial" appear in any write-up while the estimator is vision-only; a reviewer
may reject work that misrepresents its capabilities.

**C2 — `Sim3 or SE3` in the loop-edge struct was not a decision (was blocking for the protocol).** A message
schema cannot contain "or". With stereo as the primary rig (Correction B), scale is observable per agent and
inter-agent edges are SE(3). Sim(3) survives in exactly two places, neither of which is the production wire
format: the monocular ablation, and Umeyama trajectory alignment during evaluation. **Resolved:** §4.3 —
`LoopEdge.measurement` is `SE3` with `Matrix6d information`; the monocular ablation uses a separate `Sim3LoopEdge`
that never crosses the production serializer.

**C3 — OpenCV 5 ground truth moved and is re-baselined `[VERIFIED web 2026-08-30]`.** OpenCV 5.0 is released
(announcement early June 2026). Facts that change Ariadne's plan: C++17 minimum, Python 2 and the legacy C API
removed; `calib3d` split into geometry/calibration/stereo/point-cloud functionality with **USAC as the default
robust-estimation backend** for homography, essential/fundamental and all PnP solvers, multi-threaded; a new
multi-view (N-camera) calibration pipeline; `ptcloud`-area mesh/point-cloud I/O, ICP and **dense RGB-D fusion with
TSDF, HashTSDF and ColorTSDF** (which §20 builds on rather than writing a fuser from scratch); a new DNN engine
covering >80% of the ONNX spec with an ONNX Runtime integration path; and **Universal Intrinsics 2.0** targeting
SSE/AVX2/AVX-512/NEON/SVE/RVV from one vector codebase with reported 3–4x speedups on common Arm operations such
as resize and warp. That last item is directly load-bearing for the COOL narrative (§38.1): the stock baseline is
*already* Arm-optimized, so a COOL delta measured on top of it is a strong and honest result — and where the delta
is small on some ops, report it. SURF, BRIEF, FREAK and DAISY moved to `opencv_contrib`.
Sources: https://docs.opencv.org/5.0/ , https://github.com/opencv/opencv/wiki/OpenCV-5 ,
https://github.com/opencv/opencv/wiki/OpenCV-4-to-5-migration , https://opencv.org/opencv-5/

Two `[SMOKE-TEST]` items this revision could not resolve from secondary sources:

1. *Module naming discrepancy.* The docs and wiki describe the split as `geometry`/`calib`/`stereo`/`ptcloud`;
   several third-party write-ups describe a `3d` module covering "geometry, I/O, ICP, and parts of SLAM". These
   may be pre-release vs release layouts, or the same code described loosely. Day 1:
   `find $PREFIX/include/opencv2 -maxdepth 1` and record the actual header set in the run manifest. Every
   reference in this document to "the geometry module" means "wherever `findEssentialMat`, `recoverPose`,
   `solvePnPRansac` and `triangulatePoints` actually live" — the migration guide confirms those names are
   unchanged.
2. *PyPI wheel availability.* As of the June 2026 release coverage the OpenCV 5 pip wheel was announced but not
   yet visible on PyPI. The containers in §27 therefore **build OpenCV 5 + contrib from a pinned git tag** as the
   authoritative path; a wheel, if now present, is a convenience for the Python analysis environment only. This
   also removes wheel-content risk for contrib modules on arm64.

**C4 — The COOL workload had to survive the restoration ablation (strategic).** §38.1 and Iteration 14 anchored
the COOL benchmark on the restoration front-end, while §6.2's standing caveat simultaneously commits to ablating restoration
and accepting a negative result. If the ablation concludes that restoration hurts geometry — a live possibility
per the low-light findings in the catalogue — the benchmarked workload would be a stage the project had just
removed. **Resolved:** §38.1 defines the COOL workload as the **always-on per-frame pipeline** (undistort/rectify,
pyramid construction, `calcOpticalFlowPyrLK`, format conversion, flat-field multiply, VST transform, DNN-input
resize, descriptor quantization/serialization). Restoration stages are benchmarked as an *additional*, clearly
labeled configuration.

**C5 — Novelty framing must match the sensor decision (report-level).** Correction B (stereo primary) is right,
but it forfeits the "monocular multi-agent gap" claim unless framed carefully. The honest claim: Ariadne is
**vision-only** — no LiDAR, no depth sensor, stereo RGB + IMU + carried light — which is exactly the low-cost
degraded-conditions gap named by the SubT survey; the monocular ablation then quantifies what scale observability
buys. The phrase "monocular-first" must not appear as a description of Ariadne anywhere.

**C6 — Determinism: Gazebo camera rendering is not a reproducible function (architectural).** §35 requires
identical image streams across baselines and §36 requires figure-level reproducibility, but a live Gazebo run is
not bit-deterministic across executions (GPU rendering, scheduling, transport timing). **Resolved — and this is
the single most important architectural addition in v2:** simulation is split from consumption by
**record/replay** (§5.9). Gazebo runs once per (scene, trajectory, seed) and produces a clean recording; all
perception/SLAM/collaboration runs consume recordings deterministically; all visual degradation is applied at
consumption time by a seeded operator (§5.6). Consequences: identical pixels across every baseline and ablation
*by construction*; degradation parameter sweeps never re-run Gazebo; the GPU node renders once while the
Graviton/COOL node runs the many-replica sweeps, which is also the correct economics; and "clean ground truth
beneath every degradation" becomes a property of the pipeline rather than a promise. Closed-loop active-perception
experiments still run live (§5.10) and are exempt from bit-determinism, reported over seeds instead.

**C7 — Contrib dependence must be explicit (build system).** `xphoto`, `ximgproc` (DisparityWLSFilter, §9.3),
`optflow` and `quality` (the PSNR/SSIM harness) are contrib modules. Every container in §27 builds `opencv` +
`opencv_contrib` at the same pinned tag with an explicit `-DOPENCV_EXTRA_MODULES_PATH`, and the run manifest
records a hash of `cv::getBuildInformation()`.

**C8 — Per-frame ALIKED ambiguity.** The §0 diagram said "XFeat/ALIKED keypoints + sparse tracking" per frame
while §7.2 correctly reserved ALIKED for hard pairs. The per-frame path is: LK tracking on existing keypoints,
XFeat only at keyframes and when track count drops below threshold. **ALIKED never runs at frame rate** — at CPU
frame rates it is ~5 FPS class per the catalogue's benchmark table and cannot live in the 20–30 Hz path. The
diagram in §0 is corrected accordingly.

---

# 1. Audit of `algorithms.md`

## 1.1 What the algorithm catalogue gets exactly right

### Keep: OpenCV 5 as a real computational substrate

The catalogue's central architectural claim is correct: OpenCV 5 is not merely a preprocessing dependency. Current OpenCV 5 documentation exposes learned local features and modern matching interfaces, and OpenCV 5 has reorganized the former `calib3d` stack into geometry/calibration/stereo/point-cloud functionality. The official OpenCV material confirms ALIKED, DISK, LightGlue, ANN search, and the reworked geometric stack.

Sources:
- https://github.com/opencv/opencv/wiki/OpenCV-5
- https://docs.opencv.org/5.0/main_modules/features_main.html
- https://docs.opencv.org/doc/doxygen/html/d8/dc3/classcv_1_1LightGlueMatcher.html
- https://docs.opencv.org/5.0/main_modules/features_annoy.html
- https://docs.opencv.org/5.0/main_modules/ptcloud.html

Implementation consequence: make `cv::*` primitives visible at the module boundary and log them in the run manifest.

---

### Keep: XFeat as the primary local feature engine

The algorithm catalogue's choice of XFeat is particularly well motivated for Ariadne:

- lightweight,
- compact 64-D descriptors,
- explicitly designed for resource-limited devices,
- strong localization/pose behavior relative to lightweight alternatives,
- useful semi-dense matching mode,
- good fit for a byte-constrained fleet.

XFeat's paper describes a large-resolution/low-channel design aimed at efficient correspondence and reports real-time operation on inexpensive CPUs.

Source:
- https://arxiv.org/abs/2404.19174

Implementation consequence:

```text
tracking:
    XFeat descriptors only when needed

keyframe:
    XFeat sparse features
    optional XFeat semi-dense refinement

inter-agent candidate:
    XFeat candidate descriptors
    ALIKED + LightGlue verification for selected hard pairs
```

Do not replace XFeat with a heavier generic descriptor simply because a benchmark has a better raw matching score. Ariadne's objective includes communication cost.

---

### Keep: ALIKED + LightGlue as the difficult-pair verifier

OpenCV 5 exposes ALIKED and LightGlue-compatible interfaces. The official LightGlue documentation describes contextual matching rather than plain descriptor distance. LightGlue's own paper emphasizes adaptive computation as image-pair difficulty changes.

Sources:
- https://arxiv.org/abs/2306.13643
- https://docs.opencv.org/doc/doxygen/html/d8/dc3/classcv_1_1LightGlueMatcher.html
- https://docs.opencv.org/5.0/main_modules/features_main.html

Implementation consequence:

- XFeat is optimized for compactness and fast repeated use.
- ALIKED + LightGlue is the expensive verification path for the relatively small number of candidate pairs selected by the communication policy.
- This is not an API limitation workaround; it is deliberate algorithmic specialization.

---

### Keep: EigenPlaces for global place retrieval, with a domain-adaptation experiment

EigenPlaces was specifically designed to improve viewpoint robustness in visual place recognition by training on clusters of different views of the same place. That is highly relevant to agents seeing the same corridor from different trajectories/heights.

Source:
- https://arxiv.org/abs/2308.10832

However, the algorithm catalogue correctly flags a real risk: standard VPR training domains are not equivalent to dark, texture-poor collapsed interiors.

Implementation consequence:

1. Start with pretrained EigenPlaces R50 256-D.
2. Export to ONNX.
3. Run through OpenCV DNN.
4. Store/transmit fp16 or int8 representation as an optional communication compression experiment.
5. Fine-tune on synthetic Ariadne views only after baseline retrieval is characterized.
6. Measure retrieval quality separately from descriptor size.

Use `cv::ANNIndex` for retrieval/index persistence.

> **As built (2.1, D62, approved 2026-10-21: the exact index is the default; `cv::ANNIndex` is the persistence/scale ablation).** `perception/place/`:
> - `PlaceRecognizer` runs EigenPlaces R50-256 through `cv::dnn`, one Net per thread;
> - `PlaceIndex` has two backends, `exact` (one gemm, the default) and `annoy` (`cv::ANNIndex`, rebuilt from the
>   full matrix when stale, over-fetch then filter, exact rescoring).
>
> Retrieval on EuRoC: pooled R@1 0.626 (376×240) / 0.677 (752×480) against 0.444 for mean-bit (perception.md
> §11.6). Default inference size 376×240 (2026-10-20; §11.7). No consumer in 2.1; 2.5 consumes the loop-candidate query.

---

### Keep: Swarm-SLAM communication architecture

This is the strongest direct analogue for Ariadne's communication layer. Swarm-SLAM is explicitly designed around sparse collaborative SLAM and introduces an inter-robot loop-closure prioritization mechanism.

Source:
- https://arxiv.org/abs/2301.06230

Adopt:

- global descriptor broadcast,
- candidate selection before expensive local-descriptor exchange,
- communication bookkeeping,
- sparse exchange,
- elected-broker global optimization,
- anchor/reference-frame propagation,
- evaluation in bytes versus map quality.

The algorithm catalogue reports that Swarm-SLAM's real-world experiment exchanged 94.95 MB over a heterogeneous three-robot setup and reports large reductions in communication relative to distributed alternatives. That becomes an excellent benchmark target, but do not copy the exact metric envelope across different sensors/tasks; use it as architectural evidence.

---

### Keep: GNC as the primary robust backend mechanism

GNC is unusually attractive because it is a wrapper around a weighted least-squares solver rather than a replacement optimizer.

Source:
- https://arxiv.org/abs/1909.08605

Implementation:

```text
initialize robust parameter μ
while not converged:
    solve weighted PGO
    compute whitened residuals
    update loop-edge weights analytically
    decrease μ
```

Use robustification **only on candidate loop-closure/inter-agent edges**. Do not indiscriminately robustify trusted odometry edges.

The GNC paper reports robustness to high outlier ratios and compares favorably with RANSAC and specialized robust methods. These are published results, not an Ariadne guarantee; Ariadne must measure its own operating regime.

---

### Keep: SE-Sync-style certificate as a verification artifact, not runtime solver

The certificate is the right "depth per line of code" component. The runtime does not need to become a full certifiably optimal solver.

Implementation:

- solve Ariadne's practical PGO with GNC,
- construct the rotation connection matrix / certificate,
- estimate the smallest eigenvalue with a Lanczos or shifted power-iteration procedure,
- report whether the sufficient certificate condition is satisfied.

Never label the map "provably globally optimal" unless the certificate condition actually holds. Otherwise report the numerical certificate as inconclusive.

---

### Keep: layered loop-closure rejection

Use:

```text
retrieval
 -> cheap candidate filters
 -> USAC / 2-view geometry
 -> degeneracy test
 -> PCM consistency
 -> robust PGO/GNC
 -> post-fit residual + certificate
```

Crucially, **not every candidate should reach PCM or PGO**.

The algorithm catalogue identifies a blind spot: pairwise-consistency methods become less meaningful if the underlying relative-geometry hypotheses are themselves geometrically degenerate. Therefore:

```text
detect degeneracy BEFORE trusting a loop hypothesis
```

This makes the loop closure stack more honest than "run robust optimization on everything."

---

## 1.2 Corrections / decisions that should override simplistic readings of the catalogue

### Correction A: do not build a fully distributed optimizer just because the problem is multi-agent

Swarm-SLAM's own architecture strongly motivates the opposite at Ariadne's scale: use sparse inter-agent communication and an elected broker to run the expensive global solve at rendezvous/contact events.

Reason:

- few agents,
- intermittent connectivity,
- expensive dense optimizer state,
- explicit byte budget,
- need for deterministic convergence behavior,
- easy debugging.

A decentralized optimizer can still be implemented as an experimental baseline, but should not be the production path.

---

### Correction B: "monocular-first" is not the best-performing physical configuration

The literature gap around multi-agent monocular 3DGS is valuable for novelty, but Ariadne's operational output is a **metric traversability/egress route**, which strongly favors metric sensing.

Therefore the primary simulated agent should be:

```text
stereo RGB
+
co-mounted controllable illumination
+
IMU
```

Use monocular mode as a benchmark/ablation. This gives:

- metric depth,
- better degeneracy handling,
- gravity priors,
- stronger egress geometry,
- more defensible risk distance estimates,
- a direct OpenCV `stereo` / `ptcloud` workload.

MAGS-SLAM should remain the literature comparator showing why an RGB-only version is interesting, rather than forcing the entire project into monocular scale ambiguity.

### Novelty framing after this correction (C5)

Choosing stereo forfeits the "multi-agent monocular gap" claim, so the framing must be replaced rather than
quietly retained. The claim Ariadne can honestly make is that the system is **vision-only** — no LiDAR, no depth
sensor; stereo RGB + IMU + a carried light — which is precisely the low-cost degraded-conditions gap named by the
SubT survey. The monocular ablation (§31.8) then *quantifies what scale observability buys*, which is a better
result than asserting the gap.

**Do not write "monocular-first" anywhere as a description of Ariadne.** The phrase in this heading refers to the
rejected option, and it is the only place it may appear.

### Correction C: NFL-BA belongs primarily in the reconstruction/photometric layer

The NFL-BA paper is compelling evidence that co-located lights create systematic near-field effects and that modeling the light geometry improves mapping/tracking.

Source:
- https://arxiv.org/abs/2412.13176

Do not make every frame's geometric tracking depend on a full photometric near-field optimization. Ariadne's primary tracking remains geometric.

Instead:

1. calibrate the light,
2. compensate obvious falloff/radiometric effects before feature extraction,
3. use near-field illumination terms in dense reconstruction and keyframe photometric refinement,
4. use the illumination model as an additional depth/quality cue where justified,
5. measure whether it improves geometry in the actual simulator.

This preserves runtime reliability while exploiting the strongest physics contribution.

---

### Correction D: 3DGS must be decoupled from localization

Photo-SLAM's key architectural point is highly relevant: photorealistic mapping can be maintained alongside a geometric localization system rather than forcing pose estimation to depend upon differentiable rendering.

Source:
- https://arxiv.org/abs/2311.16728

RTG-SLAM is likewise useful as a design source for Gaussian lifecycle/stability ideas.

Source:
- https://arxiv.org/abs/2404.19706

Ariadne's geometry is therefore authoritative. The Gaussian representation is a consumer of validated cameras and depths.

---

### Correction E: do not make VGGT the tracker

The algorithm catalogue flags the memory wall correctly. VGGT-family models are better treated as expensive submap processors/verifiers, not as per-frame tracking.

VGGT-SLAM 2.0 specifically studies efficient incremental submap alignment and reports onboard robot operation.

Source:
- https://arxiv.org/abs/2601.19887

EC3R-SLAM independently reinforces the architectural pattern of a lightweight tracker plus expensive feed-forward mapping.

Source:
- https://arxiv.org/abs/2510.02080

Ariadne therefore uses classical geometric tracking and can optionally use a feed-forward model for **submap initialization, verification or offline accuracy comparison**.

---

# 2. System-of-systems architecture

## 2.1 Major planes

Ariadne has six logical planes.

### Plane A — Sensor and simulation

Produces timestamped:

- left RGB,
- right RGB,
- IMU,
- light-control state,
- ground-truth pose,
- ground-truth depth,
- ground-truth normals,
- ground-truth traversability,
- communication channel state.

### Plane B — Agent perception/state estimation

Produces:

- radiometrically normalized frames (albedo-proxy, §6.2),
- keypoints/descriptors,
- tracked landmarks,
- stereo depths + per-pixel σ_Z (§9.6),
- preintegrated inertial factors, gravity priors, motion prediction (§10),
- exposure and lamp actuator state (§6.5),
- local camera states,
- uncertainty,
- keyframes,
- local pose graph.

### Plane C — Collaboration

Produces:

- global descriptors,
- candidate loop edges,
- verified relative poses,
- communication decisions,
- synchronized subgraphs,
- global reference-frame transforms.

### Plane D — Reconstruction

Consumes validated keyframes and camera geometry and produces:

- local point clouds,
- local Gaussian submaps,
- merged Gaussian scene,
- confidence/coverage volumes,
- held-out-view renders.

### Plane D½ — Volumetric world model

Sits between reconstruction and planning, and exists because Plane E consumes free-space structure that the
Gaussian representation cannot answer queries about (§20). Produces:

- per-agent TSDF and tri-state occupancy,
- ESDF / clearance,
- frontier clusters,
- traversability at both `ρ_agent` and `ρ_responder`,
- a structural-risk proxy,
- a coarse global layer for cross-agent deconfliction.

### Plane E — Active mission intelligence

Consumes:

- occupancy / clearance / frontiers from Plane D½,
- uncertainty,
- coverage,
- traversability,
- communication connectivity,
- agent energy state,
- time remaining,
- existing task assignments.

Produces:

- candidate observation poses,
- task assignments,
- rendezvous tasks,
- revisit requests,
- communication actions,
- human escalation requests.

### Plane F — Operator / evaluation / reliability

Provides:

- live map,
- agent health,
- communication statistics,
- uncertainty,
- route confidence,
- audit traces,
- failure replay,
- evaluation metrics,
- restart/checkpoint behavior.

---

# 3. Repository structure

Use a monorepo with strict dependency direction.

```text
ariadne/
├── CMakeLists.txt
├── pyproject.toml
├── README.md
├── LICENSES/
├── docs/
│   ├── ARCHITECTURE.md
│   ├── algorithms.md
│   ├── IMPLEMENTATION.md
│   ├── EVALUATION.md
│   ├── PROTOCOL.md
│   ├── SIMULATION.md
│   ├── validation.md      # day-1 [SMOKE-TEST] resolutions; gates dependent code
│   └── failure-cases.md
│
├── configs/
│   ├── agent.yaml
│   ├── fleet.yaml
│   ├── camera.yaml
│   ├── illumination.yaml
│   ├── comms.yaml
│   ├── planner.yaml
│   ├── reconstruction.yaml
│   └── benchmark/*.yaml
│
├── third_party/
│   └── reference_only/
│
├── opencv/
│   └── patches/
│
├── cpp/
│   ├── core/
│   │   ├── math/
│   │   ├── geometry/
│   │   ├── uncertainty/
│   │   ├── time/
│   │   └── serialization/
│   │
│   ├── perception/
│   │   ├── radiometric/
│   │   ├── denoise/
│   │   ├── features/
│   │   ├── tracking/
│   │   ├── stereo/
│   │   ├── exposure_control/      # §6.5 active exposure + lamp co-control
│   │   └── keyframes/             # (loop degeneracy gate moved to slam/robust/, D70)
│   │
│   ├── slam/
│   │   ├── imu/                   # §10 preintegration, bias states, gravity prior
│   │   ├── landmarks/
│   │   ├── local_map/
│   │   ├── bundle_adjustment/
│   │   ├── pose_graph/            # §13 PGO, chordal init (D70)
│   │   ├── robust/                # §14.4–§14.8 degeneracy gate, PCM, GNC-TLS, certificate (D70)
│   │   └── loop_closure/          # intra-agent loop candidates (D73)
│   │
│   ├── collaboration/
│   │   ├── retrieval/
│   │   ├── candidate_selection/
│   │   ├── protocol/
│   │   ├── broker/                # (PCM and GNC run in slam/robust/ on intra- and inter-agent edges, D70)
│   │   └── reference_frames/
│   │
│   ├── reconstruction/
│   │   ├── pointcloud/
│   │   ├── gaussian/
│   │   ├── submap/
│   │   ├── fusion/
│   │   └── rendering/
│   │
│   ├── planning/
│   │   ├── volumetric/            # §20 TSDF, occupancy, ESDF, frontiers, traversability
│   │   ├── frontier/
│   │   ├── nbv/
│   │   ├── risk/
│   │   ├── assignment/
│   │   ├── route/
│   │   └── rendezvous/
│   │
│   └── runtime/
│       ├── agent/
│       ├── supervisor/
│       ├── metrics/
│       └── logging/
│
├── python/
│   ├── training/
│   ├── dataset_tools/
│   │   └── worldgen/              # §5.5 procedural world + hidden GT layer
│   ├── reconstruction/            # §18.2 gsplat service (PyTorch, out of the C++ core)
│   ├── orchestration/             # §25.2 MCP server + decision trace
│   ├── analysis/
│   ├── visualization/
│   └── operator_console/
│
├── ros2/
│   ├── ariadne_interfaces/
│   ├── ariadne_sim/
│   ├── ariadne_bridge/
│   ├── ariadne_agent/
│   └── ariadne_operator/
│
├── simulator/
│   ├── worlds/
│   ├── models/
│   ├── sensors/
│   ├── illumination_plugin/
│   ├── dust_plugin/
│   └── comms_plugin/
│
├── evaluation/
│   ├── datasets/
│   ├── scenarios/
│   ├── metrics/
│   ├── baselines/
│   ├── ablations/
│   └── reports/
│
├── infra/
│   ├── docker/
│   ├── aws/
│   │   ├── terraform/
│   │   ├── ecs/
│   │   ├── ec2/
│   │   ├── s3/
│   │   └── ecr/
│   └── scripts/
│
└── tests/
    ├── unit/
    ├── numerical/
    ├── integration/
    ├── protocol/
    ├── simulation/
    ├── regression/
    └── end_to_end/
```

### Dependency rule

```text
core
 ↑
perception / slam / collaboration / reconstruction / planning
 ↑
runtime
 ↑
ROS2 adapters / simulator / AWS services
```

Never let a simulator message type leak into the SLAM core.

Never let an AWS SDK type leak into the algorithm core.

This keeps:

- local deterministic testing easy,
- replay independent of ROS,
- CPU unit tests independent of GPUs,
- AWS deployment interchangeable,
- algorithm benchmarking possible from files alone.

---

# 4. Canonical data model

## 4.1 Frame

```cpp
struct Frame {
    AgentId agent;
    FrameId id;
    Timestamp t;

    cv::Mat left;
    cv::Mat right;

    ImuWindow imu_window;

    PhotometricState photo;
    IlluminationState illumination;

    GroundTruthOptional gt;
};
```

## 4.2 Keyframe

```cpp
struct Keyframe {
    KeyframeId id;
    AgentId agent;
    Timestamp t;

    SE3 T_world_cam;
    Matrix6d pose_cov;

    // Inertial state (§10). Valid when imu_mode == full; in attitude_only mode
    // velocity and accelerometer bias are absent and gravity_dir is still populated.
    Vector3d velocity_world;
    Vector3d bias_gyro;
    Vector3d bias_accel;
    Vector3d gravity_dir;           // unit vector in this agent's frame W_i
    ImuPreintegration preint_from_prev;

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    StereoDepth depth;
    PointCloud local_cloud;

    GlobalDescriptor global_descriptor;

    std::vector<LandmarkId> landmark_ids;

    double motion_cost;
    double energy_cost;
    double timestamp_age;

    UncertaintySummary uncertainty;

    // D56: was this pose measured (PnP against the map) or predicted while tracking was lost (bridged / coasted)?
    // Consumers weight or exclude predicted keyframes by this; segment counts loss onsets.
    PoseSource pose_source;
    uint32_t segment;
};
```

## 4.3 Loop closure edge

```cpp
struct LoopEdge {
    KeyframeId a;
    KeyframeId b;

    SE3 measurement;                // C2: SE(3), not "Sim3 or SE3". Stereo makes
    Matrix6d information;           // scale observable per agent; a wire schema
                                    // cannot contain "or".

    MatchStatistics matches;
    GeometryStatistics geometry;

    EdgeStatus status;

    double retrieval_score;
    double pcm_score;
    double gnc_weight;

    bool degeneracy_flag;
};
```

Sim(3) survives in exactly two places, neither of which is the production wire format:

```cpp
// 1. Monocular ablation only. Never crosses the production serializer.
struct Sim3LoopEdge {
    KeyframeId a, b;
    Sim3 measurement;
    Matrix7d information;
    // ... same statistics/status fields as LoopEdge
};
```

2. Trajectory evaluation, where ATE alignment uses Umeyama Sim(3)/SE(3) alignment (Umeyama, TPAMI 1991) exactly as
   the `evo` toolchain does. This is an offline metric path, not a runtime type.

## 4.4 Communication accounting

Every message crosses exactly one serializer.

```cpp
enum class MessageClass {
    HEARTBEAT,
    GLOBAL_DESCRIPTOR,
    LOCAL_DESCRIPTOR,
    MATCH_REQUEST,
    MATCH_RESPONSE,
    POSE_EDGE,
    POSE_GRAPH,
    SUBMAP_SUMMARY,
    SUBMAP_PATCH,
    TASK_BID,
    TASK_CONSENSUS,
    HUMAN_REQUEST,
    ACK,
};
```

Accounting:

```cpp
struct ByteRecord {
    Timestamp t;
    AgentId sender;
    AgentId receiver;
    MessageClass type;

    uint64_t payload_bytes;
    uint64_t wire_bytes;
    uint64_t cumulative_bytes;
};
```

The byte metric must count bytes at serialization, not "logical payload size."

---

# 5. Simulation architecture

## 5.1 Simulator choice and pinned versions

### Chosen stack

**ROS 2 Jazzy + Gazebo Harmonic.**

This is the clean current pairing for a modern ROS 2 simulation stack; the official ROS 2 documentation states that Jazzy's recommended Gazebo release is Harmonic.

Sources:
- https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Gazebo/Simulation-Gazebo.html
- https://docs.ros.org/en/iron/Releases/Release-Jazzy-Jalisco.html

### Exact versions `[VERIFIED web 2026-08-30]`

ROS 2 **Jazzy** (Ubuntu 24.04) officially pairs with Gazebo **Harmonic**, and the Jazzy vendor packages vendor the
Harmonic libraries (gazebosim.org ROS/Gazebo vendor-package and installation docs). Harmonic component versions:

```text
gz-sim8   sdformat14   gz-transport13   gz-msgs10   gz-sensors8   gz-rendering8
```

Install `ros-jazzy-ros-gz` for the bridge and sim launch integration. Pin all of — ROS distro, gz-harmonic apt
package versions, `ros_gz` version — in the run manifest (§28.2). A simulation stack whose versions are not in the
manifest cannot produce a reproducible figure.

### Do not use PX4 for the baseline simulation

Ariadne does not need flight dynamics, motor models, actuator simulation or autopilot fidelity for its research question. PX4 adds integration surface without increasing the quality of the vision/coordination experiment.

Agents are abstract camera-bearing mobile platforms.

Represent motion as:

```text
state = SE(3) pose + velocity + energy + communication state
```

The simulator should command trajectories directly.

A future physical deployment adapter may map the same waypoints to a real robot.

---

## 5.2 Why Gazebo rather than an elaborate robotics stack

Gazebo provides:

- controllable cameras,
- stereo rendering,
- configurable lighting,
- sensor timestamps,
- ROS 2 integration,
- deterministic worlds,
- headless operation,
- straightforward AWS deployment.

The project should use Gazebo primarily as a **sensor and world-state generator**.

---

## 5.3 Agent model SDF — stereo + IMU + lamp

One `<model>` per agent, namespaced `ariadne/agent_<k>`.

**Stereo.** Two `<sensor type="camera">` on one rigid link with `<pose>` offsets equal to the baseline, identical
`<update_rate>`, resolution 1280×720, `<clip near>` 0.05. Hardware-synchronous capture is emulated with gz-sensors
**triggered cameras** (`<triggered>true</triggered>` plus a shared `<trigger_topic>` driven by a fixed-rate
publisher). `[SMOKE-TEST: confirm triggered-camera support in the installed gz-sensors8; the fallback is
same-update-rate free-running cameras plus timestamp pairing, which is acceptable because both render at the same
sim step.]` Set `<camera><distortion>` to a mild plumb-bob so the undistort path is genuinely exercised rather
than being a no-op (Harmonic supports camera distortion `[VERIFIED — Harmonic release features list]`).

**Noise — the split that makes record/replay work.** Configure gz's built-in Gaussian *camera* noise to **zero**
and apply all sensor degradation in the seeded degradation operator of §5.6. This is precisely what makes C6's
clean-signal recording design work: the bag holds the clean render, and degradation is a reproducible function
applied at consumption.

IMU noise, by contrast, **is** configured in SDF (`<imu><noise>` gyro/accel densities plus bias random walk),
because inertial noise must be *integrated by the estimator*, not overlaid on images after the fact. Choose
consumer-MEMS-class values and record them: they are the σ inputs to §10.2's preintegration covariance.

**Lamp.** `<light type="spot">` attached to the camera link — inner/outer cone 25°/40°, quadratic attenuation,
`<cast_shadows>true</cast_shadows>`. Runtime dimming and aiming go through the world's `/world/<name>/light_config`
service with `gz::msgs::Light` `[VERIFIED web 2026-08-30, gz-sim light_config tutorial]`, wrapped by a small
`ariadne_sim` node exposing `set_lamp(power)` to the controller in §6.5. Spot rather than point: it matches a real
headlamp lobe and gives ogre2 well-supported shadow mapping. Shadow texture size is tunable — raise it if shadow
aliasing pollutes features. World ambient light is set to near-zero for dark scenarios.

The lamp's calibrated parameters (intensity P, pose `T_C_L`, cone profile, and whether the `cosθ` term is
calibrated or folded into P) are the inputs to the falloff normalization of §6.2 Stage 3 and to the albedo-proxy
reconstruction path of §19.1.

**Ground truth.** GT pose from gz's pose publisher; GT depth from a `<sensor type="depth_camera">` co-located with
the left camera. GT depth is **recorded but never visible to perception** — it is consumed only by the evaluation
harness and by the dust model in §5.6, where using true depth is the correct physics and is only possible because
GT depth rides in the recording.

---

## 5.4 Bridge, topics, and the simulated clock

`ros_gz_bridge parameter_bridge` driven from a YAML config. Per agent: left/right `gz.msgs.Image →
sensor_msgs/Image`, `camera_info`, `gz.msgs.IMU → sensor_msgs/Imu`, GT pose, GT depth. World-level: `/clock`
(`gz.msgs.Clock → rosgraph_msgs/Clock`). Every ROS node runs `use_sim_time:=true`. All ROS topics live under
`/ariadne/agent_<k>/…`.

**Time synchronization contract.** IMU and camera timestamps derive from this same simulated clock, so no online
camera–IMU time-offset estimation is implemented. `t_offset_cam_imu` exists in the calibration struct, is 0 in
simulation, and would be estimated offline if a real rig ever appears. This is a stated simplification, and it is
stated in the report rather than left implicit (§10.6).

---

## 5.5 World generation

Generate parametrically rather than hand-building one showcase world.

World primitives:

- intact rooms,
- collapsed rooms,
- partially collapsed corridors,
- rubble piles,
- beams,
- concrete slabs,
- pipes,
- furniture,
- repeated structural members,
- voids/tunnels,
- dead ends,
- narrow passages.

Each scene stores a hidden ground-truth layer:

```text
mesh
occupancy
signed distance
traversability
clearance
risk
surface normals
material IDs
semantic zones
```

### Generation pipeline

`python/dataset_tools/worldgen`, in five stages:

1. Sample a procedural floorplan from a rooms/corridors grammar.
2. Apply collapse operators — slab drop with rotation, wall shear, beam scatter, rubble piles as
   convex-decomposed heaps, void carving — with tunable severity, plus **repetition operators** that deliberately
   clone structural elements to manufacture perceptual aliasing for the Iteration 7 laboratory.
3. Assemble meshes with `trimesh`; assign low-texture concrete PBR materials.
4. Emit SDF via Jinja templates against sdformat14. (Harmonic also ships a Python SDFormat API; use whichever is
   less friction, but pin the choice.)
5. Bake the hidden GT layer: watertight mesh → voxel occupancy → signed distance (mesh-to-sdf) → GT
   traversability/clearance **at both ρ values** (§20.5) → GT egress routes by running §22's planner on the GT
   volume → stored as `scene_gt.npz` beside the world.

A scene is fully identified by `(grammar seed, severity vector)`. Two numbers reproduce a world.

---

## 5.6 Visual degradation model

### Darkness

Use physically motivated co-mounted illumination.

For a point-light approximation:

```text
E(x) ∝ P / r(x)^2
```

with directional/angle effects added only if evidence shows they matter.

NFL-BA explicitly studies co-located near-field lighting and derives an inverse-distance-based formulation for its effects.

Source:
- https://arxiv.org/abs/2412.13176

### Sensor response

Model:

```text
I_sensor = gain * Poisson(I_photon)
           + Gaussian(read_noise)
           + exposure effects
           + lens/vignetting
```

Use a Poisson-Gaussian noise model.

### Lens / flat-field

Store:

```text
K(x) = vignetting + pixel response field
```

and calibrate it offline.

### Exposure

Each simulated camera has:

- exposure time,
- analog gain,
- digital gain,
- clipping threshold.

An adaptive exposure controller becomes part of the mission system, not a rendering cheat.

### Dust

Use a simplified participating-medium model:

```text
I_obs(x) = J(x) * t(x) + A(x) * (1 - t(x))
```

where:

```text
t(x) = exp(-β * depth)
```

augmenting it with a spatially varying local particulate field.

Do not claim full physical atmospheric simulation. The objective is to create controlled, parameterized visual degradations.

### The degradation operator `D` — consumption-time and seeded

This is the mechanism that makes C6's determinism guarantee real. All of the above is implemented as a single
deterministic operator

```text
D(image, params, seed, frame_id) -> degraded image
```

applied when frames are **read from the recording** (and identically in live mode). The chain runs in sensor
order:

```text
scene radiance scale (darkness level)
  -> Poisson shot noise      (per-pixel, seeded by (seed, frame_id, pixel-block))
  -> read noise
  -> PRNU / vignette field
  -> exposure / gain
  -> quantization / clipping
  -> optional dust compositing:  I = J·t + A·(1−t),  t = exp(−β·depth_gt)
                                 plus a Perlin local particulate field
```

The dust stage uses the **recorded GT depth** (§5.3), which is the correct physics and is only possible because GT
depth rides in the recording.

Parameter vectors are named configs (`dark2_dust1`, …) referenced by run manifests, never inline constants.

> **As built (Phase 5, D130).** The sim render is sRGB-encoded (SMOKE §4u). D gains `input_transfer: srgb`, which
> decodes before stage 1; the default stays `linear`, so pinned digests hold. The dust stage's daylight form above is
> wrong for a headlamp (algorithms.md §10.2): the replacement, round-trip attenuation plus a near-field backscatter
> integral with the lamp offset as a parameter, is D139.

**Division of labour — this line matters.** The lamp's *optical* effect (falloff, shadows, cone profile) is real
Gazebo rendering and is **not** part of `D`. `D` degrades the *sensor*; the renderer degrades the *scene*.

**Exposure control in replay.** In replay mode the exposure controller of §6.5 acts on `D`'s exposure parameters,
closing the control loop without re-rendering. Document this honestly: replayed exposure control changes *sensing*
but not *light transport*. It is a stated, measured approximation — Iteration 9 quantifies the live-vs-replay
difference rather than assuming it away.

---

## 5.7 Communication simulation

Each agent has a communication graph `G(t)`.

For pair `(i,j)`:

```text
bandwidth(t) = B_ij
latency(t)    = L_ij
packet_loss   = P_ij
connected     = C_ij
```

Make communication depend on:

- Euclidean range,
- wall/rubble penetration count,
- structural material,
- randomized shadow zones,
- explicit partitions.

Provide reproducible scenario seeds.

Modes:

```text
ideal
range-limited
attenuated
partitioned
intermittent
store-and-forward
```

### Physical layer vs accounting layer `[DESIGN]`

These are two distinct components, and conflating them would destroy replayability.

**Physical layer:** gz-sim's **RFComms system plugin** — a log-distance path-loss model ported from the SubT
competition infrastructure, with `max_range`, `fading_exponent`, `l0`, `sigma` and radio `capacity` SDF parameters
`[VERIFIED web 2026-08-30, gazebosim.org RFComms API]`. Its SubT lineage is itself citable realism.

**Accounting / policy layer:** RFComms carries only a per-pair *link-state estimate*. Actual Ariadne payloads flow
through the **Ariadne comms broker node**, which:

- enforces bandwidth, latency and loss sampled from the RFComms link state,
- injects partitions from scenario scripts,
- is the single point where `ByteRecord`s (§4.4) are written.

Rationale: byte accounting and fault injection must be deterministic and replayable (C6), and gz-transport
delivery is not. The wall/rubble penetration count above is computed in the broker by ray-casting the GT mesh
between agents and adding a per-wall attenuation term to the RFComms-derived loss.

> **As built (3.3, D102, 2026-10-22).** `collaboration/comms/`: RFComms' equations (gz-sim8) are reproduced in the
> broker rather than read from the live plugin, since replay is the substrate. The wall term counts solid blocks
> crossed. `loss_p` is the per-packet marginal over shadowing, computed for the 1500 B MTU (a stated seam limit).
> `CommsNode` owns the one ledger. Not yet carrying protocol traffic (needs 3.2's sim integration).

---

## 5.8 Agent motion

No physics-accurate locomotion is required.

Represent each agent as:

```text
SE3 kinematic body
camera rig
illumination rig
collision envelope
battery state
communication transceiver
```

Motion commands:

```text
goto(position, orientation)
follow(path)
orbit(target)
rendezvous(peer)
return_to_safe_zone()
hold()
```

Obstacle avoidance is solved over a geometric configuration space rather than actuator simulation.

### Implementation in Gazebo

No physics locomotion. A **kinematic mover** sets the model pose along commanded trajectories with velocity-limited
SE(3) interpolation, checking the collision envelope against the GT mesh itself. Implement it as a gz system plugin
(preferred, for rate stability) or by driving `/world/<w>/set_pose` services from the agent runtime.

---

## 5.9 Record / replay — the determinism substrate

**This is the architectural consequence of C6 and everything downstream depends on it.**

Gazebo runs **once** per `(scene, trajectory, seed)` and writes a recording. Every perception, SLAM,
collaboration, reconstruction and ablation run afterwards consumes that recording. Degradation is applied at read
time by the seeded operator of §5.6.

**Format:** rosbag2 **MCAP** (the default storage since ROS 2 Iron), one bag per `(scene, trajectory, seed)`,
containing clean images, IMU, GT streams, tf, lamp state and camera infos. Bags are the S3 `datasets/` payload
(§28.1), and every downstream figure's manifest names its bag checksums.

Live closed-loop runs record the same topics **plus** all Ariadne decision and telemetry topics, which is what
makes §42 item 17 ("a complete trace can be replayed offline") true rather than aspirational.

What this buys:

- identical pixels across every baseline and ablation, by construction rather than by discipline;
- degradation parameter sweeps that never re-run Gazebo;
- the GPU node renders once while the Graviton/COOL node runs the many-replica sweeps — which is also the correct
  cost structure (§5.10);
- "clean ground truth beneath every degradation" as a property of the pipeline.

**Exemption, stated explicitly:** closed-loop active-perception experiments must run live, because the trajectory
depends on the estimate. Those runs are *not* bit-deterministic and are reported over seeds with variance, never
as single-run numbers. Every figure declares which regime produced it.

---

## 5.10 Headless operation and run topology

**Headless rendering `[VERIFIED web 2026-08-30]`.** Gazebo supports EGL headless rendering with ogre2 via

```bash
gz sim -s -r --headless-rendering world.sdf
```

on a GPU instance without X (gazebosim.org headless-rendering tutorial, which itself walks through the AWS GPU
setup). Camera sensors require the render engine, so **rendering-bearing sim runs live on the g5/g6 node and
Graviton runs no Gazebo at all**. The sim image carries the NVIDIA container runtime with EGL libraries (§27.1).

**Batch evaluation topology:**

```text
Gazebo (GPU, once per scene/trajectory)
    -> MCAP recording
    -> S3
    -> N deterministic consumer replicas
         - Graviton / COOL : CPU pipeline sweeps
         - GPU node        : reconstruction
```

This is simultaneously the sweep economics and the COOL multi-agent throughput benchmark, where "agents per
instance" means concurrent replay pipelines per `m8g.4xlarge`.

**Live mode topology:** the full ROS 2 graph — agents, broker and console — used for active perception, the
failure campaign, and the demo video.

---

# 6. Per-agent perception

## 6.1 Pipeline

```text
raw stereo frames  (clean from the bag; degradation operator D applied here, §5.6)
   |
timestamp synchronization  (shared sim clock, §5.4)
   |
radiometric normalization  (flat-field -> VST denoise -> falloff, §6.2)
   |
   +---------------------------+
   |                           |
   v                           v
GEOMETRY PATH             RETRIEVAL PATH          (the split of §6.3)
calibrated corrections    + CLAHE / contrast
   |                        normalization
quality estimation             |
   |                           v
LK tracking every frame    EigenPlaces descriptor
XFeat at keyframes             |
   |                           v
stereo correspondence      §17 place recognition
   |
depth + uncertainty
   |
pose estimation  (IMU-predicted, §10.5)
```

ALIKED and LightGlue do not appear in this diagram because they never run at frame rate; they are invoked only on
hard inter-agent candidate pairs (§7.2).

---

## 6.2 Radiometric normalization

Four stages, in this order. Each has a concrete implementation, not a placeholder.

### Stage 1 — flat-field

Correct vignetting and fixed spatial gain. A per-camera gain image `K(x)` is calibrated offline in simulation from
a uniform-albedo wall sweep; the online correction is **one `cv::multiply`** by the precomputed reciprocal, stored
fp16. This is deliberately the cheapest possible stage because it runs on every frame from every agent and is part
of the COOL benchmark workload (§38.1).

### Stage 2 — Poisson–Gaussian noise handling

Estimate per-`(exposure, gain)` noise parameters — α (photon gain) and σ (read noise) — **offline** from static
captures, following Foi, Trimeche, Katkovnik & Egiazarian, *Practical Poissonian-Gaussian noise modeling and
fitting for single-image raw-data*, TIP 2008 `[PAPER]`.

Online:

```text
generalized Anscombe transform:   f(I) = (2/α)·sqrt(αI + (3/8)α² + σ²)
        -> denoise in the stabilized domain
        -> exact unbiased inverse (Mäkitalo & Foi, TIP 2013)  [PAPER]
```

Start with `cv::fastNlMeansDenoising` at small `h`; also try `medianBlur` 3×3 for salt noise from dust.

**Use the exact unbiased inverse, not the naive algebraic one.** The algebraic inverse is biased at low photon
counts, and that bias lands *exactly* in the dark regions Ariadne exists to handle. Gate the whole stage on
measured noise level per §6.4.

> **As built (2.2, D66/D67/D69, 2026-10-21).** Stages 1–2 run per camera **before rectification**, on the raw frames
> after D (`RadiometricSource`, between the source and the rectifier). D is applied in `SimSource` (opt-in,
> `--degradation`, per-camera seeds, D66). Deviations from the text above: α/σ are **fitted online** from the first
> 8 frame pairs (simplified Foi 2008), not offline captures; the gate is ON when the stabilised σ_z is in [0.8, 1.25]
> **and** gray-path SNR at the median level is **< 30**; the default denoiser is a Gaussian (≈ 14 ms/image total),
> NLM (≈ 360 ms/image) is an ablation. Off by default; measured-neutral on the one segment run (D69).

### Stage 3 — light falloff

Given calibrated lamp intensity P, lamp pose `T_C_L` (§5.3), and the current depth estimate `D(x)`, compute `r(x)`
and divide by `P/r²` — optionally times `cosθ`, depending on what the lamp calibration of §5.3 establishes:

```text
I_corrected(x) =
    f(I(x), r(x), cos(theta), exposure_state)
```

For initial implementation use inverse-square attenuation.

The output is the **albedo-proxy image**. It is consumed by (a) feature extraction, when the ablation says it
helps, and (b) the reconstruction input path of §19.1, where it is load-bearing rather than optional — the
albedo-proxy is what makes 3DGS under a carried light photometrically coherent at all.

### Stage 4 — exposure / gain normalization

Maintain photometric state — exposure time `t`, analog gain `g` — from the controller in §6.5, and normalize
working images to a canonical `(t₀, g₀)` before **any** cross-frame photometric comparison.

### Standing caveat — avoid aggressive enhancement

The algorithm catalogue correctly identifies a critical empirical risk: enhancement can improve image appearance and even increase matching counts while hurting actual loop closure / reconstruction.

Therefore every restoration stage is tested on geometry, not only visual quality.

Metrics:

- feature repeatability,
- match precision,
- match recall,
- relative pose error,
- track length,
- ATE,
- RPE,
- loop closure precision,
- reconstruction error.

---

## 6.3 Retrieval-path vs geometry-path preprocessing split `[DESIGN]`

The catalogue's empirical finding — enhancement can help appearance matching while hurting geometry — is usually
treated as a dilemma. It is not. It is an asymmetry, and the architecture should exploit it structurally:

- The **geometry path** (LK, stereo, triangulation, XFeat for correspondence) receives **only calibrated
  corrections**: flat-field, VST-domain denoise, falloff normalization. Nothing that is not invertible physics.
- The **retrieval path** (EigenPlaces input, §17.1) may *additionally* receive CLAHE / contrast normalization,
  because retrieval is an appearance decision whose output never enters a residual. A retrieval descriptor that is
  wrong costs a rejected candidate; a geometry pixel that is wrong costs a biased pose.

The retrieval-path enhancement is ablated separately — one additional row in the illumination ablation matrix
(§31.5). This is a cheap experiment with a genuinely informative outcome either way.

> **As built (3.1, D92).** `PlaceRecognizerParams.enhance: none | clahe` (clip 2.0, 8×8; Lab L for RGB) runs on a copy
> inside the recogniser, so the geometry path is untouched. The default is `none`.
>
> On EuRoC (297 queries), CLAHE changes R@1 by ≤ 0.013, n.s., under clean and two darkened-proxy conditions. The
> darkened proxy turned out *easier* than clean, so it does not answer the dark question. The §31.5 sim row does, and
> it is still open (retrieval.md §5).

---

## 6.4 Low-light denoising

Start with classical methods using OpenCV.

Primary:

```text
variance-stabilizing transform
 -> denoise
 -> inverse transform
```

Implement generalized Anscombe handling for the Poisson-Gaussian model.

OpenCV operations used around it:

- `cv::GaussianBlur`,
- `cv::fastNlMeansDenoising`,
- `cv::medianBlur`,
- `cv::normalize`,
- `cv::SVD`.

Do not make the transform mandatory if the raw image is already above a noise threshold. The decision is driven by measured noise statistics.

---

## 6.5 Active exposure and lamp co-control

New module: `cpp/perception/exposure_control/`.

This is the CERBERUS-named unfixed failure from the catalogue, and it is a genuine novelty axis for Ariadne. It
must therefore be a **real controller** in the mission system, not a rendering setting chosen once per scenario.

**Metric.** The gradient-information score of Shim, Lee & Kweon, *Auto-adjusting camera exposure for outdoor
robotics using gradient information*, IROS 2014 `[PAPER]`. Comparator: Zhang, Forster & Scaramuzza, *Active
exposure control for robust visual odometry in HDR environments*, ICRA 2017 `[PAPER]`.

**Controller.** At keyframe rate, evaluate the metric on the current frame together with its saturation and shadow
fractions, then take a gradient-descent step on log-exposure with a **slew-rate limit of ≤ 0.5 stop/frame** so the
controller cannot destabilize LK tracking. Hard constraints bound the search: `t_max` from motion blur at the
predicted angular rate (§10.5), `t_min` from the noise model (§6.2 Stage 2).

**Lamp channel.** Lamp power is a *second actuator* optimizing the same objective plus an energy cost term. In
simulation it is actuated through the Gazebo light service wrapped in §5.3. Both actuators are logged in the
decision trace — "overexposure event → controller response" is Failure E of the Iteration 16 campaign and a demo
beat in §43.

In replay mode the controller acts on the degradation operator's exposure parameters (§5.6), which closes the loop
without re-rendering at the cost of a stated approximation: sensing changes, light transport does not. Iteration 9
measures that gap directly.

> **As built (5.3, D132, 2026-10-22; not yet wired into a runner).** `perception/exposure_control/`. The metric is
> Shim et al. eq. 1, with Zhang's gradient percentile as the comparator row (algorithms.md §9.4: Shim is "not robust").
> **Approved deviation (2026-10-23, D147):** the lamp is not optimised with an energy cost term. One
> brightness decision is spent in a fixed order (lamp → exposure → gain), the limiting case of a small energy cost. In
> replay, lamp power is D's `lamp_power`, a radiance scale: exact for ambient 0 and a decoded render (SMOKE §4u).
> **Wiring (D147):**
> - The controller runs at keyframes and commands the source through `StereoSource::SetPhotometric`.
> - Every frame reports the state it was actually exposed with: applied values, `applied_ns`, and clamp. A live sink
>   (gz `light_config`, owned by the live runtime) implements the same contract.
> - LK applies the known brightness ratio between frames (known-gain KLT).
> - Rows run with `radiometric: flat_only`.

---

# 7. Feature extraction

## 7.1 Feature interface

```cpp
class LocalFeatureEngine {
public:
    // Keypoints only: the tracker replenishes every frame and never uses
    // descriptors, so it must not pay for them (D34, approved 2026-09-29).
    virtual std::vector<cv::KeyPoint> detect(const cv::Mat& gray, const cv::Mat& mask,
                                             int max_keypoints) = 0;
    // Keyframes: keypoints and row-aligned descriptors.
    virtual FeatureSet detectAndCompute(const cv::Mat& gray, const cv::Mat& mask) = 0;
};
```

Implement:

```text
XFeatEngine
ALIKEDEngine
DISKEngine
SIFT_Engine
ORBEngine
```

All implement the same contract.

---

## 7.2 Primary configuration

### Online frame tracking

- tracked XFeat keypoints,
- pyramidal optical flow,
- forward-backward consistency,
- image-gradient quality,
- spatial uniformity filter.

No neural extractor runs at frame rate. The per-frame cost is LK plus filtering.

### Keyframe creation

Run full XFeat extraction. XFeat also runs off-cadence when the tracked-point count drops below threshold, which
is the recovery path out of `DEGRADED` (§8.2).

### Hard inter-agent candidate

Run:

```text
ALIKED
    +
LightGlue
```

and/or XFeat semi-dense refinement.

**ALIKED never runs at frame rate.** At CPU frame rates it is ~5 FPS class per the catalogue's benchmark table,
against a 20–30 Hz requirement. It is a verifier for expensive, rare, high-value decisions — inter-agent
candidates that survived retrieval and cheap geometric screening — and nothing else (C8).

> **As built (D60, 2.1, `docs/design/perception.md`).** EuRoC v2 ships **ORB + lost-mode re-association**, not
> XFeat.
> - **Re-association:** when fewer than 30 tracks hold a landmark, tracks holding none are described and matched to
>   healthy (MEASURED-born) window landmarks: mutual NN, ratio 0.75, Hamming ≤ 50. The matches are verified by seeded
>   PnP and relabelled onto the landmark id.
> - **XFeat as specified here is built and config-selectable:** `features.engine: xfeat` (one cached forward per
>   image, §7.3 grid selection after the mask), `tracker.replenish: keyframe` with an off-cadence
>   `replenish_min_tracks`, and this section's image-gradient quality as `replenish_min_eig_quality`, a Shi-Tomasi
>   gate on new tracks.
> - Measured on EuRoC, XFeat cuts lost frames hardest but regresses ATE on V2 and MH_02, even with re-association off.
> - "XFeat primary" stands for the learned paths. The VO front-end choice is revisited when that regression is
>   removed.

---

## 7.3 Keypoint selection

Do not keep the top N points globally.

Use spatial bins:

```text
image -> MxN cells

per cell:
    retain top-k by score/reliability
```

This reduces catastrophic concentration in one textured patch.

Also track:

```text
gradient energy
cornerness
descriptor reliability
distance-from-lamp
saturation
dust confidence
```

Feed these into feature weights.

### Keypoint budget arithmetic `[DESIGN]`

Targets, to be tuned by profiling rather than trusted from this document:

```text
tracked points per frame        : 1000–1500, budgeted across an 8x6 grid
XFeat raw detections at keyframe: max 4096
    -> stored after filtering   : <= 1200
    -> eligible for transmission: <= 800
```

**These are manifest fields, not constants.** The byte table in §15.7 is computed from the transmission cap, and
the spectral policy's cost model (§16) is computed from that table — so changing a cap changes the headline
communication metric. A cap that lives in a header file rather than in the run manifest makes every reported byte
count unfalsifiable.

---

## 7.4 DNN execution contract `[SMOKE-TEST for each model]`

Every learned component enters through `cv::dnn` under a fixed contract. Getting these details wrong produces
silently degraded descriptors rather than errors, which is the worst possible failure mode, so each line below is
a day-1 smoke test against the installed binary and the exported ONNX file.

**Threading.** One `cv::dnn::Net` instance **per thread**; `Net` is not treated as thread-safe. The feature worker
pool owns N nets loaded from the same ONNX file.

**Per-model input/output contracts:**

| Model | Input | Output | Notes |
|---|---|---|---|
| XFeat | grayscale `N×1×H×W` float in [0,1] | 64-D float descriptors | fix inference size at 640×640 (the class default) initially; measure keypoint localization error vs full-res before changing |
| DISK | dims must be positive multiples of 16 | — | a pad-and-crop wrapper owns this constraint |
| ALIKED | RGB `[1,3,H,W]` | sub-pixel keypoints, 128-D | hard pairs only (§7.2) |
| EigenPlaces | 224×224 (or the repo's inference resolution — record it) | L2-normalized 256-D | ResNet-50 backbone + head exported at opset 17; ImageNet normalization baked into the export or applied in the wrapper |

**LightGlue.** `cv::LightGlueMatcher` requires `setPairInfo(qKpts, tKpts, qSize, tSize)` *before* `match()`. Wrap
it so it drops into the same `DescriptorMatcher` seam as brute-force and ANN matching, so the matcher is an
ablation switch rather than a structural commitment.

> **As built (2.1, D63, approved 2026-10-21; perception.md §12, SMOKE §4s).**
> `perception::HardPairMatcher` wraps `cv::ALIKED` + `cv::LightGlueMatcher` on Ariadne's own export from upstream:
> ALIKED-n16 at K = 1024 (2048 is the ablation), native 752×480, LightGlue with all 9 layers. The wrapper owns three
> conventions:
> - the image size must equal the export's (it throws otherwise; sim needs its own export);
> - upstream's aspect-preserving keypoint normalisation, passed through an empty `Size()`; OpenCV's per-axis rule is
>   the ablation;
> - one instance per thread.
>
> Cost about 0.5 s per extraction and 0.6 s per match (shared CPU), so it serves hard pairs only (C8).
>
> **Sim rig (D90, 2026-10-22, SMOKE §4t).** 1280×720 RGB exports for EigenPlaces (1280×720, 640×360) and
> ALIKED-n16 (K = 1024, 1280×720) are exact against upstream on recorded sim frames. The RGB path is
> `PlaceRecognizer::DescribeRgb` / `HardPairMatcher::ExtractRgb`, which take RGB order (`StereoFrame::left_rgb`).
> **EigenPlaces' sim default is 640×360** (settled 2026-10-22 by the sim retrieval evaluation,
> `retrieval.md#sim-retrieval`): on 61 revisit queries of the clean ci_tiny recording both sizes give R@1 0.705, and
> 1280×720 costs 4×.
>
> **Threading as built (D91).** Every wrapper keeps its networks in `dnn::PerThread<T>`, so one wrapper serves a
> pool. **Second rule:** with several callers, OpenCV's own pool is set to 1 thread, so the parallelism comes from
> the callers. Concurrent ENGINE_NEW forwards sharing the default pool differ at the 1e-5 level; at 1 thread they are
> byte-identical (`test_dnn_contract`).
>
> **Measured on EuRoC loop candidates** (4815 retrieval candidates, the 1.6 verifier's essential matrix): pose
> AUC@10° 0.607 vs the ORB placeholder's 0.284, better on 11/11 flights. But 4 confident wrong verifications show
> that an E-inlier count alone is not a loop gate, so the PnP and degeneracy gates stay downstream (§12.6).

**Fallback if the new DNN engine rejects an operator.** The sanctioned path is the ONNX Runtime integration that
OpenCV 5 itself ships `[VERIFIED web 2026-08-30, docs.opencv.org/5.0]`. This keeps the "runs through OpenCV" claim
intact. **Record which engine actually executed each model in the run manifest** — this is exactly the kind of
detail that quietly differs between the dev box and the benchmark instance.

---

# 8. Frame-to-frame tracking

## 8.1 Primary tracker

Use `calcOpticalFlowPyrLK` for inexpensive inter-frame tracking.

OpenCV is responsible for:

- pyramid generation,
- LK flow,
- image operations.

Ariadne owns:

- correspondence filtering,
- robust motion estimation,
- track lifecycle,
- state covariance,
- keyframe insertion.

---

## 8.2 Tracking update

For tracked points:

```text
p_t-1 -> p_t
```

perform:

1. forward-backward check,
2. image-bound check,
3. residual threshold,
4. gradient direction consistency,
5. stereo consistency if available,
6. RANSAC/USAC relative motion,
7. covariance estimate.

If track support collapses:

```text
state = DEGRADED
```

and immediately increase the active-perception value of a recovery observation.

> **As built (D56, `docs/design/local-slam.md#lost-frames`).** DEGRADED is `VoStatus::kLost`: PnP has failed, and the pose is
> PREDICTED.
> - Rotation is measured from the surviving 2-D tracks against an anchor frame: seeded USAC H/E with ORB-SLAM's
>   score rule.
> - Translation is the last measured velocity, decaying as exp(−t/τ).
> - Each frame and keyframe carries `pose_source` (measured / bridged / coasted) and a segment counter (loss onsets).
> - 2.4's gyro replaces the rotation source. Keyframes are still made while lost: suppressing them raised the losses.

---

## 8.3 Keyframe policy

Insert a keyframe when a weighted combination of:

```text
translation
rotation
time
tracked-feature loss
parallax
new scene coverage
uncertainty
```

exceeds threshold.

A practical score:

```text
S =
    w_t * normalized_translation
  + w_r * normalized_rotation
  + w_p * parallax
  + w_f * feature_loss
  + w_u * uncertainty_growth
```

---

# 9. Stereo geometry

## 9.1 Sensor

Primary simulated camera:

```text
global-shutter stereo pair
baseline: configurable, default 8–12 cm
resolution: 1280x720
frame rate: 20–30 Hz
```

Use a baseline large enough for metre-scale rooms while retaining overlap in narrow voids.

Do not assume a single optimal baseline. Evaluate:

```text
4 cm
8 cm
12 cm
20 cm
```

against range/depth error.

---

## 9.2 Calibration

Use OpenCV:

- `calibrateCamera`,
- `stereoCalibrate`,
- `stereoRectify`,
- `initUndistortRectifyMap`.

Store calibration immutable per sensor rig.

**Update 2026-10-17 (D54, §49 ritual):** the stored calibration stays immutable. On top of it, each run may apply
**one** disclosed online correction to the right rectification map.

- **What is fitted:** the residual row misalignment `dy = a + b x + c y` of the rectified pair, estimated from
  round-trip-consistent stereo LK matches pooled across the first keyframes.
- **How it is applied:** folded into the right map as the new camera matrix `A·P2`. P2, Q and the ideal rig are
  untouched.
- **Why:** EuRoC's V2 recordings sit ≈ 1.7 px off their published calibration, and a 1 px row gate then rejected 97%
  of correct matches (`docs/design/local-slam.md#window-audit`).
- **Disclosure:** the estimate (a, b, c, match count, keyframe) is the manifest field `stereo_row_refinement` and a
  `results.yaml` line. Estimates that exceed the search bound are refused loudly.
- **Exposure:** before stereo LK and SGBM, the right image's global gain and bias are matched to the left's
  (per-camera auto-exposure; EuRoC's listed known issue).

---

## 9.3 Disparity

Primary:

```text
StereoSGBM
```

OpenCV 5 provides stereo correspondence APIs.

### Configuration `[DESIGN, standard]`

```text
blockSize          : 5–7
P1                 : 8  · channels · blockSize²
P2                 : 32 · channels · blockSize²
uniquenessRatio    : 10–15
speckleWindowSize  : 100–200
speckleRange       : 2
disp12MaxDiff      : 1
mode               : MODE_SGBM_3WAY        (speed)
```

Left–right consistency comes from a right-matcher pass filtered with `cv::ximgproc::DisparityWLSFilter` (contrib —
see C7). **That filter also yields a confidence map; use it rather than inventing a confidence heuristic.**

Subpixel: SGBM outputs fixed-point disparity in 1/16 px. Carry `σ_d = 0.25–0.5 px` forward into §9.5.

Source:
- https://docs.opencv.org/5.0/main_modules/ptcloud_rgbd.html

> **As built (D61, 2.1).** `stereo.confidence: wls` fills `StereoDepth.confidence` (u8, 0 where invalid) from
> `DisparityWLSFilter::getConfidenceMap()`.
> - It uses a right-matcher pass and the generic filter, with the native SGBM ROI and radius. NOT
>   `createDisparityWLSFilter(sgbm)`, which would mutate our SGBM.
> - The left disparity is unchanged unless `wls_filtered_disparity`.
> - Default `none` (placeholder): nothing consumes the map yet, and it costs a second SGBM pass (~23 ms) plus the
>   filter (~14 ms) per stereo pass at 752×480.
> - Quality against EuRoC laser-scan GT (AUC, Hu & Mordohai protocol) is **unmeasured**; the design is in
>   perception.md §10.6.

---

## 9.4 Confined-space disparity arithmetic

`numDisparities` sets the **minimum observable range**, and in a collapsed structure that bound is a mission
constraint, not a tuning detail:

```text
Z_min = f·B / d_max
```

At 1280×720 with HFOV 90° (f ≈ 640 px) and B = 10 cm, `numDisparities = 256` gives `Z_min ≈ 0.25 m`. A narrow void
demands `Z_min ≤ 0.3 m`, so **256 is the floor at this baseline** — and the B = 20 cm ablation config (§9.1) must
either raise `numDisparities` or accept `Z_min ≈ 0.5 m`, which is a real blind zone at exactly the ranges the
project cares about.

Encode this as a **config validator that refuses inconsistent `(f, B, numDisparities, Z_min)` combinations**. The
failure mode this prevents is a silent blind zone that looks like sparse depth rather than like a misconfiguration.

---

## 9.5 Depth uncertainty

Approximate stereo depth:

```text
Z = fB / d
```

and propagate disparity uncertainty:

```text
σ_Z ≈ |∂Z/∂d| σ_d
     = fB/d² * σ_d
```

Equivalently, in terms of the subpixel disparity noise carried out of §9.3:

```text
σ_Z = Z²/(f·B) · σ_d
```

Use this uncertainty later in:

- landmark weights,
- point-cloud fusion,
- active perception,
- traversability confidence.

---

## 9.6 Depth output contract to downstream

The stereo stage emits, at keyframe resolution:

```cpp
struct StereoDepth {
    cv::Mat disparity;   // CV_16S, SGBM native fixed point (1/16 px) -- NOT fp16 (D34)
    cv::Mat Z;           // fp16, computed in fp32 from the fixed-point disparity
    cv::Mat sigma_Z;     // fp16
    cv::Mat confidence;  // u8, from DisparityWLSFilter
    cv::Mat valid;       // mask
};
```

Disparity is not fp16 (D34, approved 2026-09-29): fp16 spacing is 0.125 px above 128 px, coarser than SGBM's
1/16 px for every depth under 0.5 m at f = 640 px, B = 0.10 m, which is the confined-space range. `CV_16S` is exact
at the same 2 bytes. Consumers that carve free space along depth rays use only depth with dense local support
(`DenseSupportMask`, D34): SGBM reports surfaces nearer than its search range as sparse phantom far depths. The
gate is a mitigation, not a guarantee: on EuRoC MH_01 it catches 87% of them (D35), so free-space carving from
stereo additionally needs a near-field check.

Two consumers depend on this contract precisely:

- **TSDF integration (§20.1)** consumes `Z`, `sigma_Z` and `valid`, weighting each sample by `1/σ_Z²`.
- **Landmark triangulation (§11.2)** cross-checks LK-tracked points against stereo `Z`. §11.2's "inconsistent
  stereo depth" test is exactly `|Z_tri − Z_stereo| > 3·σ`.

---

# 10. Inertial layer

New module: `cpp/slam/inertial/`. This section exists because v1 claimed a stereo-inertial system and specified no
inertial estimator anywhere (C1).

## 10.1 Scope decision

Full stereo-VIO is not required for Ariadne's research question, but **three inertial products are**:

1. **A gravity direction per agent**, which yields the `GRAVITY_PRIOR` pose-graph edge (§12.1) and removes 2 of
   the rotational gauge DOF in the inter-agent alignment problem.
2. **Motion prediction between frames**, which seeds LK search windows and makes the tracker robust to fast
   rotation — the failure mode that kills vision-only trackers in corridors.
3. **Scale and robustness reinforcement** of stereo odometry across texture-poor intervals.

The design below delivers all three with entirely standard machinery. It is not a research contribution and
should not be presented as one; it is the piece without which the sensor claim is false.

## 10.2 Preintegration `[PAPER]`

Implement on-manifold IMU preintegration exactly per Forster, Carlone, Dellaert & Scaramuzza, *On-Manifold
Preintegration for Real-Time Visual-Inertial Odometry*, T-RO 2017 (arXiv:1512.02363; earlier RSS 2015).

Between keyframes i and j, integrate gyro and accelerometer samples into preintegrated `ΔR_ij`, `Δv_ij`, `Δp_ij`
with first-order bias Jacobians

```text
∂ΔR/∂b_g ,  ∂Δv/∂b_g ,  ∂Δv/∂b_a ,  ∂Δp/∂b_g ,  ∂Δp/∂b_a
```

and the 9×9 preintegration covariance propagated per sample. The per-sample noise densities come from the SDF IMU
configuration of §5.3 — the simulator and the estimator must be reading the same numbers, and the manifest records
them once.

State per keyframe becomes:

```text
T_WB ∈ SE(3)     (body = IMU frame; camera via the fixed T_BC extrinsic)
v_W  ∈ R³
b_g, b_a ∈ R³    (random-walk between keyframes)
```

**Residuals:** the standard three-block inertial residual — rotation via `Log(ΔR̃_ijᵀ R_iᵀ R_j)`, velocity, and
position with gravity `g_W` — plus a bias random-walk residual between consecutive keyframes.

These enter the windowed BA of §11 as factors alongside the reprojection factors. **The Schur elimination
structure is unchanged**, because inertial factors touch only camera-block variables, never landmarks. This is
the property that makes the inertial layer cheap to add to an existing BA rather than a rewrite of it.

Transcribe residual and Jacobian expressions from the paper's equations, not from this document, and validate
every Jacobian with the finite-difference harness of §11.4.

> **As built (D59, `docs/design/inertial.md` §4).** The window keeps `T_W_C` (left perturbation) and reaches the
> body through the fixed `T_C_B`; the same δ left-perturbs `T_W_B`, and the residual Jacobians were re-derived for it.
> The accelerometer sample is rotated by the mid-interval orientation (VINS-Mono), not Forster's start-of-interval one
> (which left 1.8 mm/s at the truth). Inertial blocks are a separate segment of the reduced system
> (`[poses | v,b_g,b_a | landmarks]`). The IMU does not observe velocity, so an inertial chain ties keyframes only out
> of a group with two pose-determined keyframes; in a dark window window[0]'s velocity is held instead.
> Discrete noise is density²/Δt; on EuRoC the datasheet densities are inflated ×3 (gyro) and ×9 (accel), measured
> (§4.1). Every block is finite-difference tested. window[0]'s bias prior keeps the previous prior's information
> (random-walk-propagated) and deflates only the window's added information (Λ_carried + (Λ_marg − Λ_carried)/κ);
> deflating the whole marginal let unobserved b_a run away in dark windows. Estimating gravity's direction per window
> (a 4-DoF gauge) was built and is a recorded negative result (inertial.md §4.8).

## 10.3 Initialization `[DESIGN, standard practice]`

Agents start stationary for a configurable 1–2 s. During that window:

```text
b_g            = mean gyro
gravity dir    = mean accel, normalized
R_WB           initialized so g_W = (0,0,-9.81) maps to the measured direction
                 (yaw unobservable -> fixed to 0 per agent, reconciled later by
                  inter-agent registration, §15.5)
b_a            = residual after gravity removal, with a loose prior
                 (weakly observable at rest — do not pretend otherwise)
```

Stereo provides metric scale from the first pair, so **none of the monocular visual-inertial alignment machinery
is needed**. This is one of the concrete payoffs of Correction B.

> **As built (D59, inertial.md §4.3), superseding the stationary-start recipe above.** EuRoC's Vicon-room flights
> do not start at rest, and attitude_only showed the gravity direction is better taken from motion. Initialisation
> runs after N tracked keyframes, on their window-refined (metric) poses: b_g from the keyframe rotations, then v_k, g
> and b_a as a linear MAP (ORB-SLAM3's inertial-only MAP with the scale known), g re-solved on the 9.81 sphere,
> refused on |g| or χ²/dof. **The world is not rotated**: gravity is estimated in the VO world (the first keyframe's
> camera) and held fixed, because the graph, keyframe records and the gate depend on that world. b_a stays weakly
> observable exactly as warned above (a gravity tilt θ looks like a bias of 9.81 θ); a loose prior decides it.

## 10.4 Gravity prior edge — the producer for §12.1

For each keyframe, after windowed BA, emit a **unary `GRAVITY_PRIOR` factor** on the roll/pitch components of
`T_WB`, with information taken from the current velocity/bias covariance.

In the broker's global PGO these priors constrain all agents' maps to a shared gravity direction, shrinking the
inter-agent alignment problem from 6 DOF to **4 (yaw + translation)** whenever both agents carry valid priors.
This measurably improves chordal initialization conditioning in corridor scenes, and Iteration 7 tests exactly
that.

## 10.5 Motion prediction

Between frames, propagate `T` and `v` with raw gyro/accel — no preintegration is needed at frame rate — to predict
keypoint locations for LK. Where prediction and track disagree beyond threshold, flag `DEGRADED` per §8.2. The
same predicted angular rate bounds the exposure controller's `t_max` (§6.5).

## 10.6 Time synchronization contract

IMU and camera timestamps come from the same simulated clock (§5.4), so **no online time-offset estimation is
implemented**. `t_offset_cam_imu` exists in the calibration struct, is 0 in simulation, and would be estimated
offline if a real rig ever appears. Document this as a stated simplification rather than leaving a reader to
discover that a real-hardware path is missing.

## 10.7 Fallback mode — if Iteration 3.5 slips

```text
imu_mode: attitude_only
```

A complementary filter (e.g. Mahony) producing gravity direction and rotation prediction only; no velocity or bias
states in BA. Products (1) and (2) of §10.1 survive — so the `GRAVITY_PRIOR` edge keeps a producer and the tracker
keeps its prediction — and product (3) is lost.

**The run manifest records which mode produced every result.** This is what makes the fallback honest rather than
a quiet downgrade, and it is why the sensor claim in the report can be checked against any individual figure.

> **As built (D58, 2.4 step 1).** `attitude_only` ships for EuRoC v2 (`slam/inertial/attitude.hpp`):
> - **Rotation prediction:** the bias-corrected gyro integral, with the bias estimated from VO rotations on tracked
>   frames. Mahony alone cannot see the yaw-axis bias.
> - **Gravity:** Mahony's explicit filter (Theorem 5.1, kP = 1, kI = 0.3) supplies the direction, carried on every
>   keyframe record as `gravity_dir_C`.
> - **While lost,** the gyro replaces the D56 visual bridge's rotation; translation keeps the bridge's decaying velocity.
> - **§10.5 LK seeding** is a tested seam, off by default.
> - **Not yet built:** the `GRAVITY_PRIOR` edges are produced by 2.4 (plan: "gravity-prior edges"; corrected D59)
>   and consumed by the pose-graph solver (2.5); the sim IMU noise operator is 2.2's degradation wiring. The manifest
>   records `imu_mode`.

---

# 11. Local map and SLAM backend

## 11.1 State

Per agent:

```text
poses:
    T_W_Ck ∈ SE(3)

landmarks:
    X_j ∈ R³

extrinsics:
    fixed

camera intrinsics:
    fixed

optional light parameters:
    fixed/calibrated

optional exposure:
    per-keyframe

inertial (when imu_mode == full, §10):
    v_W_k ∈ R³
    b_g, b_a ∈ R³ per keyframe
```

---

## 11.2 Local landmark creation

Triangulate matched observations.

Reject if:

- insufficient parallax,
- negative depth,
- depth outside sensor interval,
- excessive reprojection residual,
- inconsistent stereo depth,
- high uncertainty.

**Update 2026-10-17 (D54, §49 ritual):** there is no fixed far-depth cap. ORB-SLAM2 (§III-A) classifies points
beyond ~40 baselines as "far" and keeps them, and our 60-baseline discard had misread that: it cut 36–38% of
candidates in the EuRoC machine hall. The "high uncertainty" test (σ_Z / Z ≤ `max_relative_sigma`) is the depth
bound: Z ≤ max_relative_sigma · fB / σ_d, ≈ 9.6 m on EuRoC. Multi-view triangulation of points beyond it is not
implemented.

---

## 11.3 Bundle adjustment

Implement from scratch.

Residual:

```text
r_ij = π(T_i X_j) - z_ij
```

State update:

```text
T_i <- exp(δξ_i) T_i
```

Use **one perturbation convention globally**.

Recommended:

```text
left multiplicative SE(3) perturbation
```

Do not mix left/right updates between Jacobian routines.

### Robust kernel

Huber on reprojection residuals inside BA, `δ = 1.5–2.5 px`.

Scope matters here: **loop-closure robustness lives in the pose graph** (§12.1, §14.6), not in BA. BA-level Huber
handles residual mismatches from tracking noise; it is not a defence against wrong data association, and treating
it as one hides exactly the failures §14 exists to catch.

### Gauge

Fix the oldest in-window pose. Never solve the free-gauge system inside the window and "handle it later".

### Inertial factors

When `imu_mode == full`, the preintegrated factors of §10.2 join the same normal equations. They touch only camera
blocks, so §11.5's elimination structure is unchanged.

---

## 11.4 Jacobians

Analytic Jacobians.

Use OpenCV `projectPoints` as a derivative/check oracle when possible, but keep Ariadne's own implementation authoritative.

Validate numerical Jacobians with finite differences:

```text
relative error < configured tolerance
```

Test:

- translation,
- rotation,
- point coordinates,
- near-zero depth,
- wide-FOV rays.

---

## 11.5 Schur complement

Partition normal equations:

```text
H_cc H_cl
H_lc H_ll
```

Eliminate landmarks:

```text
S = H_cc - H_cl H_ll^-1 H_lc
```

Solve camera system, back-substitute landmarks.

This is the core performance mechanism for windowed BA.

---

## 11.6 Solver

Use Levenberg-Marquardt:

```text
(H + λD) δ = -g
```

with trust/radius-like update logic.

**Update 2026-10-16 (D51, §49 ritual):** the Phase 2 solvers (pose graph, then windowed BA) run on OpenCV 5's
`cv::detail::LevMarqBase` driver, with Ariadne's residuals, Jacobians and sparse algebra behind its `LevMarqBackend`
interface. The driver damps with Marquardt's `D = diag(H)` (clamped) and adapts λ by step quality. The schedule below
remains the Phase 1 two-frame stub's, kept byte-identical. All driver settings are config + manifest fields; design
`docs/design/pose-graph.md`.

**Update 2026-10-19 (D70):** the pose graph runs `pgo_v2`, which has the diagonal clamp OFF. OpenCV clamps the damping
`λ·diag(H)` itself to an absolute [1e-6, 1e32], whereas Ceres clamps `diag(JᵀJ)`; the floor throttled the final
Gauss–Newton steps on parking-garage. The windowed BA keeps the clamp (`pgo_v1` defaults). Design
`docs/design/robust-backend.md` §2.

Use Eigen sparse Cholesky internally.

Reference-check with:

- Ceres or GTSAM for small graphs,
- not runtime.

### Damping schedule `[DESIGN]`

```text
λ₀        = 1e-4 · max(diag H)
on reject : λ *= 2
on accept : λ *= 1/3
terminate : relative cost change < 1e-6, or 15 iterations
```

### Covariance recovery

The pose covariance of the newest keyframe comes from the corresponding block of `S⁻¹`, where `S` is the Schur
system of §11.5. **Compute it by solving `S·X = E_block`, not by inverting `S`.**

This one quantity feeds three downstream consumers, which is why it is specified here rather than left implicit:
the keyframe `UncertaintySummary` (§4.2), the keyframe-insertion score (§8.3), and the information-gain terms of
the active planner (§21.3).

---

## 11.7 Window exit and marginalization policy

v1 specified Schur elimination and LM but never said what happens when a keyframe leaves the window — a silent gap
with real consistency consequences. The two defensible options are:

**(a)** proper marginalization with first-estimate Jacobians, or
**(b)** a prior-free window plus pose-graph odometry edges.

The catalogue documents at length why (a) is a consistency minefield: inconsistent linearization points, spurious
information gain, and failures that appear as quiet overconfidence rather than as divergence.

**Decision `[DESIGN]`: option (b).** When a keyframe exits the window:

- its final windowed pose, and the relative pose plus joint covariance to its predecessor, are frozen into an
  `ODOMETRY` pose-graph edge (composed covariance via the adjoint transport in `cpp/core/uncertainty`);
- its landmarks with fewer than 2 in-window observations are retired to the static local map — still usable for
  relocalization, no longer optimized;
- **no marginal prior is retained inside the window.**

This trades some estimator optimality for the elimination of an entire class of subtle inconsistency bugs, and the
pose graph recovers the correlations at the level Ariadne actually consumes them. State the trade explicitly in
the report; it is a defensible engineering decision, not an oversight, and it should be presented as one.

---

# 12. Local pose graph

## 12.1 Edge types

```text
ODOMETRY
STEREO_ODOMETRY
LOCAL_LOOP
INTER_AGENT_LOOP
ANCHOR
GRAVITY_PRIOR
```

Only loop-type edges are robustified by default. `ODOMETRY` edges always carry GNC weight 1 (§14.6).

`ODOMETRY` edges have two producers: frame-to-frame tracking, and window exit per §11.7. `GRAVITY_PRIOR` edges are
produced by §10.4 — under both `full` and `attitude_only` inertial modes, so this edge type is never orphaned.

---

## 12.2 Residual

For edge `i -> j`:

```text
r_ij = Log( Z_ij^-1 * X_i^-1 * X_j )
```

Use a 6-vector tangent residual.

---

# 13. Rotation initialization

Do **not** throw a cold nonlinear optimizer at the whole pose graph.

Pipeline:

```text
1. rotation synchronization / chordal relaxation
2. translation initialization
3. joint SE(3) refinement
4. GNC loop-edge robustification
```

The rationale follows the catalogue's PGO analysis: rotation synchronization provides a more stable initialization.

## 13.1 Chordal rotation synchronization `[PAPER]`

Martinec & Pajdla (CVPR 2007) linear rotation synchronization, as surveyed for SLAM by Carlone, Tron, Daniilidis &
Dellaert, *Initialization techniques for 3D pose graph optimization*, ICRA 2015.

```text
1. Stack relative-rotation constraints  R_j ≈ R_ij R_i
   into a sparse linear system over the 9 rotation entries per node
   (3 independent right-hand-side solves, with one anchor rotation fixed).

2. Project each 3x3 block back to SO(3) by SVD:

       R = U · diag(1, 1, det(U Vᵀ)) · Vᵀ

3. Translations by linear least squares with rotations fixed.

4. Joint SE(3) refinement.
```

**The determinant correction in step 2 is mandatory.** Without it the projection can return a reflection, and the
catalogue flags this as a recurring transcription failure. It is one line and it is not optional.

---

# 14. Robust loop closure stack

## 14.1 Stage 0 — candidate retrieval

Global descriptor search.

Return top K:

```text
K = 10–50
```

depending on fleet size.

Apply temporal exclusion windows.

---

## 14.2 Stage 1 — 2D correspondence

For candidate:

```text
XFeat BF/ANN
```

or hard candidate:

```text
ALIKED + LightGlue
```

Metrics:

- mutual match ratio,
- spatial spread,
- descriptor distance,
- confidence distribution.

---

## 14.3 Stage 2 — geometric verification

Use OpenCV:

```text
findEssentialMat
recoverPose
findFundamentalMat
solvePnPRansac
```

Use current OpenCV robust estimation infrastructure.

Reject:

- low inlier count,
- low parallax,
- unstable pose,
- physically impossible scale,
- poor cheirality,
- spatially concentrated inliers.

---

## 14.4 Stage 3 — degeneracy detector

Compute geometry conditioning:

```text
Hessian eigenvalues
triangulation angle distribution
inlier spatial covariance
rotation/translation observability
corridor score
```

Define:

```text
degenerate =
    λ_min / λ_max < τ_condition
    OR
    insufficient parallax
    OR
    near-collinear inlier distribution
```

Ariadne does not merely reject a false closure; it can **decline to attempt the closure** when the scene does not provide enough information.

This is specifically motivated by the SubT-derived observation in `algorithms.md(1).md` that robust backends cannot rescue an invalid/degenerate hypothesis indefinitely.

**Update 2026-10-19 (D71, §49 ritual):** `λ_min / λ_max < τ_condition` on the full Hessian mixes metres and radians,
and its verdict depends on the frame. The gate instead bounds the worst-direction 1σ of the translation marginal [m]
and of the rotation marginal [rad] separately. Both come from the loop's covariance in the pose-graph residual tangent,
with B's stereo depth noise propagated, and the within-block ratios are reported. Parallax becomes "too few inliers
with a stereo triangulation angle ≥ θ_min". Collinearity means the **3D** inliers lie on a line; collinear pixels
alone (a planar fan through the camera centre) are well conditioned with stereo depth. Pixel spread, planarity and
the corridor score are reported, not gated. Implemented in `cpp/slam/robust/degeneracy.{hpp,cpp}`; design
`docs/design/robust-backend.md` §4; algorithms.md §7.8a.

**Update 2026-10-21 (D73 part 2b):** on real data (EuRoC) the PnP-Hessian loop covariance is about 100–1000×
overconfident. Correlated depth and calibration errors do not average down with the inlier count; COVINS-G (arXiv
2301.07147) reports the same for analytic loop covariances.
- **The edge information** is the calibrated model var = k σ²_PnP + floor², per translation and rotation block.
  Fitted on EuRoC MH_01 + V1_01; held-out V1_02 gives mean NEES 5.24.
- **The gate's criteria** still read the PnP σ.
- **Calibration is per sensor and environment.** The sim needs its own.

---

## 14.5 Stage 4 — PCM

Build a pairwise consistency matrix for candidate loop closures.

For closures `e_i`, `e_j`:

```text
compose(
    e_i,
    odometry path,
    inverse(e_j)
)
```

and test against a covariance-aware threshold.

Do not rely on exact maximum clique for large candidate sets.

Use:

```text
greedy maximal consistent subset
```

or branch-and-bound only for very small sets.

### Consistency statistic `[PAPER]`

Mangelson, Dominic, Eustice & Vasudevan, *Pairwise Consistent Measurement Set Maximization for Robust Multi-robot
Map Merging*, ICRA 2018. For candidate inter-agent closures `z_ik` and `z_jl`:

```text
c = || Log( z_ik⁻¹ · x̂_i,j · z_jl · x̂_l,k ) ||²_Σ
```

with `Σ` composed along the path by adjoint transport. The pair is consistent if

```text
c < χ²₆(0.95) = 12.59        (expose the quantile as config, not a literal)
```

Implement exact max-clique only behind a flag, for candidate sets of ≤ 25.

### Ordering constraint — inviolable

The catalogue's blind-spot warning about PCM is that a set of **self-consistent wrong closures** passes pairwise
consistency cleanly. Repeated structure produces exactly such sets, and Ariadne deliberately manufactures repeated
structure in Iteration 7.

This is precisely why the degeneracy gate of §14.4 runs **before** PCM, not after it: the gate refuses hypotheses
the scene cannot support, so they never reach the stage that would confirm them in each other's company. **Keep
that ordering inviolable.** A refactor that reorders these stages for convenience silently removes the defence.

**Update 2026-10-19 (D72, §49 ritual):**
- **The pair threshold is no longer the fixed χ²₆(0.95).** A maximum clique keeps a true loop only if all of its
  n − 1 pairs pass. At 5 % per pair, measured recall was 0.84–0.86, worst 0.60–0.70 on 30-loop groups. The default
  bounds false rejection per loop instead, at α_loop/(n − 1) per pair with α_loop = 0.001, derived from the
  recall ≥ 0.95 requirement. The fixed quantile stays as config, as the ablation baseline.
- **Group rules:** a group with one candidate passes, tagged untested. Two or more candidates with a maximum clique
  of 1 pass nothing.
- **Clique:** the default is the greedy CLIQUEHEU heuristic. Exact Bron–Kerbosch runs behind a flag for ≤ 25.
- **Odometry legs** come from the odometry-only chain, with shared edges counted once.
- Implemented in `cpp/slam/robust/pcm.{hpp,cpp}`; design `robust-backend.md` §5.

---

## 14.6 Stage 5 — GNC

Run GNC over surviving candidate loop edges.

### Update equations `[PAPER — re-derive signs from the paper before shipping]`

Yang, Antonante, Tzoumas & Carlone, *Graduated Non-Convexity for Robust Spatial Perception*, RA-L 2020
(arXiv:1909.08605). With residual `r_i`, inlier threshold `ε` and control parameter `μ`, the GNC-TLS weights have
a closed form:

```text
w_i = 0                                    if r_i² ≥ ((μ+1)/μ)·ε²
w_i = 1                                    if r_i² ≤ (μ/(μ+1))·ε²
w_i = (ε / r_i)·sqrt(μ(μ+1)) − μ           otherwise
```

Threshold and schedule:

```text
ε²  = χ²₆(0.99) · σ²        (6-DOF edge residuals)
μ₀  = ε² / (2·r_max² − ε²)  (r_max = largest initial residual)
μ   *= 1.4 each outer iteration, until the surrogate matches TLS
```

Each outer iteration is one weighted PGO solve using the existing LM solver (§11.6). Odometry edges are held at
`w = 1` throughout.

Track and expose:

```text
initial residual
final residual
weight trajectory      (per edge, per outer iteration)
edge acceptance
edge rejection
```

This creates an excellent judge-visible trace: the per-edge weight trajectory shows a false closure being
*demoted*, which is far more convincing than a final inlier/outlier label.

**Update 2026-10-20 (D73 part 1):** implemented as specified. The residual is whitened, `r² = rᵀΩr` with `ε² =
χ²₆(0.99) = 16.81`, so σ² is absorbed into Ω. Weights, μ₀ and the ×1.4 schedule match GTSAM 4.3.0's `GncOptimizer`,
which uses barcSq = ε²/2 because its error carries ½. Two additions:
- **Component anchoring.** When rejections split the active graph, each component is solved with its own anchor, in
  `SolvePoseGraph` itself: pgo_v2 has no damping floor to hide a singular system.
- **Limitation, measured.** A lone loop between two otherwise unconnected agents fits exactly, so GNC cannot reject
  it. PCM's no-consensus rule is the defence when several such loops disagree; a single one passes as "untested".
- Implemented in `cpp/slam/robust/gnc.{hpp,cpp}`; design `robust-backend.md` §6.

---

## 14.7 Optional Stage 6 — DCS / switchable constraints

Implement DCS as a lightweight additional robust-loss experiment.

Do not run every robust mechanism simultaneously by default.

Benchmark:

```text
Huber
Cauchy
DCS
Switchable Constraints
GNC
GNC + PCM
GNC + degeneracy gate
```

---

## 14.8 Rotation optimality certificate `[PAPER — convention-sensitive]`

This is the verification artifact promised in §1.1 ("SE-Sync-style certificate as a verification artifact, not a
runtime solver"), made executable. It answers a question no amount of residual inspection can: *was this the
global optimum, or a good-looking local one?*

Follow the duality line of Carlone, Rosen et al. (*Lagrangian duality in 3D SLAM*, IROS 2015), SE-Sync (Rosen,
Carlone, Bandeira & Leonard, IJRR 2019, arXiv:1612.07386) and Eriksson et al. (*Rotation Averaging and Strong
Duality*, CVPR 2018).

Given the connection Laplacian `Q̃` of the rotation subproblem and the solved rotations `R̂`:

```text
1. Build the block-diagonal dual variable Λ:

       Λ_i = sym( Σ_j Q̃_ij R̂_j R̂_iᵀ )

2. Form the certificate matrix:

       S = Λ − Q̃

3. Verify S ⪰ 0 by computing λ_min with shifted Lanczos on the sparse matrix.
```

**The sign convention differs across these papers.** Lock it against whichever paper's formulation is actually
transcribed, and unit-test on a 5-node graph with a known globally optimal solution before trusting a single
result from it.

Report `certified` or `not certified (λ_min = …)`. **Never claim global optimality on a `not certified` result** —
the certificate is one-directional, and a failed certificate means "unknown", not "wrong". Roughly 300 lines plus
tests.

**As built (D74, 2026-10-22; §49 update).** `cpp/slam/robust/certificate.{hpp,cpp}`, behind
`loop_closure.certificate` (off).
- **What is certified:** SE-Sync's isotropic chordal surrogate of the active edge set, refined to a stationary point
  first. It is not the runtime anisotropic Log cost.
- **The sign:** SE-Sync's, `S = M − Λ` on the translation-explicit data matrix. The `Λ − Q̃` above is the other sign;
  the noise-free unit test fixes it.
- **The test:** shifted Lanczos is replaced by SE-Sync's practice, a Cholesky of S with one translation grounded and a
  relative η = 1e-10‖M‖ added to the rotation diagonal only. That is exactly the reduced test `Q̃ − Λ + ηI ≻ 0`; a
  full `S + ηI` over-certifies. Lanczos only reports λ_min on failure.
- **What a certified result carries:** the bound f − f* ≤ |gap| + 3nη.

---

# 15. Distributed collaboration protocol

## 15.1 Principle

Never continuously stream maps.

Transmit:

```text
small global descriptors
      ↓
candidate request
      ↓
local descriptors / 2D features
      ↓
verified pose constraint
      ↓
small pose graph
```

This is the key communication reduction.

---

## 15.2 Message classes

### Heartbeat

```text
agent_id
timestamp
pose summary
battery
link state
keyframe watermark
```

### Global descriptor

```text
keyframe_id
timestamp
descriptor
thumbnail hash
quality
```

### Candidate request

```text
request_id
query_keyframe
candidate_keyframe
requested_feature_type
```

### Feature response

```text
keypoints
descriptors
camera metadata
optional 3D landmarks
```

### Pose edge

```text
keyframe_i
keyframe_j
SE3                      (C2: never "SE3/Sim3" — a schema cannot contain "or")
information (Matrix6d)
frame_id of each pose    (§15.5; the deserializer rejects untagged poses)
verification stats
```

### Pose graph batch

Compressed graph:

```text
nodes
edges
covariances
frame IDs
reference anchor
```

---

## 15.3 Communication bookkeeping

Maintain per-peer:

```text
sent_global_descriptor_set
sent_local_feature_set
sent_pose_edge_set
sent_submap_set
```

Every message is idempotent.

Duplicate messages are ignored.

Out-of-order messages are stored until prerequisites arrive.

### Store-and-forward queue policy

A per-peer outbox with **strict priority classes**:

```text
1. heartbeat
2. pose edges + pose-graph batches
3. global descriptors
4. feature responses
5. submap summaries
```

Within a class: FIFO.

TTLs: global descriptors expire at 120 s (a stale retrieval is cheap to redo); **edges and pose graphs never
expire — they are the product**. Under a link budget, classes 4 and 5 are the only ones the spectral policy of
§16 is entitled to cancel.

Idempotency, concretely: every message carries `(sender, class, monotonic_seq)`; receivers keep a per-sender
high-water mark plus a small out-of-order buffer; duplicates are acknowledged and dropped.

---

## 15.4 Broker election with epochs `[DESIGN, follows Swarm-SLAM's centralized-at-rendezvous choice]`

Ariadne adopts Swarm-SLAM's elected broker, which leaves one question v1 never answered: **what happens under a
partition?**

- Connected components are discovered from heartbeats (§15.2). Within a component, **the broker is the lowest
  AgentId** — deterministic, requiring no voting protocol (this matches Swarm-SLAM's own rule; RA-L 2024,
  arXiv:2301.06230).
- Every broker solve is stamped with an **epoch**: `(component_member_set_hash, monotonic_counter)`. An agent
  accepts a returned global estimate only if that epoch's member set contains it; stale epochs are discarded.
- **Two simultaneous brokers in two partitions are correct under this rule.** Each optimizes its own component,
  and the merge on reconnection is simply a new epoch over the union. This is the property that distinguishes a
  partition-tolerant design from one that merely survives partitions by luck.
- **Broker state is soft.** Everything the broker needs — local subgraphs, verified edges — is re-sendable from
  the agents' own bookkeeping. Broker loss therefore costs recomputation, never consistency, which is what makes
  Iteration 5's exit condition ("consistent or explicitly incomplete, never silent corruption") achievable rather
  than aspirational.

### Gauge handling in the broker solve

Fix the anchor agent's first pose: translation plus yaw when gravity priors are present (§10.4), full rotation
when they are not. **Never solve the free-gauge system and handle it afterwards.**

---

## 15.5 Reference-frame algebra

Each agent `i` owns a frame `W_i` — its startup pose, gravity-aligned. The global frame `G` is defined as `W_a` of
the current **anchor agent** `a`.

Anchor propagation: on the first inter-agent verification within a component, the anchor is the lowest AgentId
present. On later merges the lower *anchor* id wins, and the losing side's agents update

```text
T_G_Wi  <-  T_Ganchor_Gold · T_Gold_Wi
```

**Every pose that crosses the wire is tagged with the frame id it is expressed in**, and the deserializer rejects
untagged poses or poses in an unknown frame. This turns the most insidious class of multi-agent bug — a correct
pose interpreted in the wrong frame — into a deserialization error.

All Gaussians and routes are stored in **keyframe-local or `W_i` frames** and composed with the current `T_G_Wi`
at read time. Consequently, anchor changes and PGO updates never rewrite stored geometry (§18.6).

---

## 15.6 Communication simulation split

The physical link model and the payload/accounting path are separate components; see §5.7. RFComms supplies link
state; the Ariadne comms broker enforces it and is the single writer of `ByteRecord`s.

---

## 15.7 Wire format and byte accounting `[DESIGN]`

**Serializer: FlatBuffers** — schema-versioned, zero-copy read, and deterministic size accounting at the single
serialization boundary §4.4 requires. Plus zstd level 3 on class 4–5 payloads only; descriptors are high-entropy,
so **measure and report the actual ratio rather than assuming one**.

```text
wire_bytes = serialized + compression + 28 B emulated UDP/IP header per datagram
```

The header term is included so the metric is honest about framing rather than flattering.

Quantization at the boundary:

| Payload | Encoding | Size |
|---|---|---|
| Global descriptor (256-D) | fp16 | 512 B + ~48 B envelope |
| Keypoint | u,v as uint16 (1/4 px), score u8, level u8 | 6 B |
| XFeat descriptor | int8 with per-keyframe scale (ablated vs fp16) | 64 B (128 B fp16) |
| Keyframe feature response, 800 kp | keypoints + int8 desc + K, T hint | ≈ 57 KB |
| SE(3) pose edge | 7×f32 pose + 21×f32 upper-tri info + ids/stats | ≈ 150 B |
| Pose-graph batch, n nodes / m edges | nodes 28 B + edges 150 B | e.g. 200 kf + 40 edges ≈ 12 KB |

**These numbers reproduce the catalogue's central claim from first principles**: front-end descriptor traffic
dominates back-end traffic by orders of magnitude. One feature response ≈ 380 pose edges. That ratio is the entire
justification for the spectral prioritization of §16 — it is cheap to send the *answer* and expensive to send the
*evidence*, so choosing which evidence to request is the whole game.

The int8-descriptor ablation belongs with the compression experiment in §17.2, reported with matching
precision/recall curves rather than as a byte saving alone.

The sizes above depend on the keypoint caps of §7.3, which is why both live in the run manifest.

---

# 16. Spectral communication prioritization

This is the **flagship collaboration algorithm**.

## 16.1 Candidate graph

Construct a hypothetical graph using:

```text
existing odometry graph
+
already fixed inter-agent edges
+
candidate edges weighted by retrieval similarity
```

For each budget-limited candidate set:

```text
maximize λ₂(L)
subject to number_of_selected_edges <= B
```

where `L` is the weighted graph Laplacian.

---

## 16.2 Algorithm

Start with a connectivity-enforcing greedy set.

Then iteratively:

```text
compute Fiedler vector v2

for each candidate e:
    gain_e = v2^T L_e v2

select highest gain / communication_cost

update graph
recompute v2
```

For small candidate sets, compare with exact combinatorial enumeration.

For larger sets, use greedy/local refinement.

### Solver `[DESIGN]`

`λ₂` and the Fiedler vector on the candidate Laplacian come from **LOBPCG or shifted-inverse Lanczos** on the
sparse `L`, **warm-started from the previous Fiedler vector**. The graph changes by exactly one edge per iteration
of the greedy loop, so warm starts converge in a few iterations and the inner eigensolve stops being the
bottleneck it would otherwise be.

### Marginal gain, cheaply

For a candidate `e` with endpoints `(a, b)`:

```text
gain_e = κ_e · (v₂[a] − v₂[b])²
```

This equals `v₂ᵀ L_e v₂` for a weighted edge Laplacian — cheaper and clearer than forming `L_e` at all.

### Cost model

Selection maximizes `gain_e / cost_e`, where `cost_e` is the **predicted wire bytes** of verifying `e`, taken from
the table in §15.7: feature-response size, minus keyframes already sent per the bookkeeping of §15.3, with
vertex-cover sharing when one keyframe serves several candidates.

Because the policy optimizes against *predicted* bytes, the prediction must be validated: Iteration 6 fails if
predicted and actual wire bytes diverge by more than 15%.

### Connectivity bootstrap

If a candidate's endpoints lie in **different connected components** of the fixed graph, its gain is `+∞` class.
Connectivity comes first: `λ₂` is not a meaningful objective on a disconnected graph, and Swarm-SLAM makes the
same point. Only once the component is connected does the algorithm optimize algebraic connectivity within it.

### Testing

An exact enumeration oracle for `|candidates| ≤ 15` is the correctness reference (Iteration 6).

> **As built (3.4, D170; merged c720e77).**
> - **Fiedler space:** the full candidate graph, solved by block inverse iteration (grounded sparse LDLT,
>   warm-started, a degenerate cluster handled as one).
> - **Selection:** connectivity bootstrap, then greedy on `gain / marginal bytes` with a shared keyframe cost, then
>   1-swap refinement (`refine_passes` 2).
> - **Per-instance bound:** a dual bound (MAC) over the shared-cost LP (`cv::solveLP`). Measured valid 60/60 against
>   the oracle, and loose (1.3–6.4×).
> - **Graph:** Swarm-SLAM's construction, verified from the cslam source: ids min..max per agent (D171), with fixed
>   odometry and accepted edges at κ = 1.
> - **Integration:** selection runs behind `NodeHooks::select` with a token bucket of predicted bytes. Unset = the
>   "similarity" (request-all) baseline; `SelectBySimilarity` = "greedy".
> - **Pricing:** the fetch cost is the exact codec payload times a calibrated ratio, plus framing.
> - **Not built:** the vertex-cover *direction* choice of Giamou et al. Its residual bytes are reported on the sim
>   runs.
> - **Budget defaults (20 kB/s, 200 kB):** placeholders until D171's derived levels.

---

## 16.3 Communication budget

Budget can be:

```text
bytes
messages
energy
time
```

Primary benchmark:

```text
bytes
```

because this is the cleanest headline resource.

Secondary:

```text
bytes + latency
```

---

# 17. Place-recognition stack

## 17.1 Global retrieval

Primary:

```text
EigenPlaces R50, 256-D
```

via ONNX/OpenCV DNN, under the execution contract of §7.4: ResNet-50 backbone + head exported at opset 17, fixed
224×224 input (or the repo's inference resolution — record which), ImageNet normalization baked into the export or
applied in the wrapper, output L2-normalized.

Its input comes from the **retrieval path** of §6.3, which may carry appearance enhancement the geometry path is
forbidden.

Index:

```text
cv::ANNIndex
```

Sources:
- https://arxiv.org/abs/2308.10832
- https://docs.opencv.org/5.0/main_modules/features_annoy.html

---

## 17.2 Compression experiment

Compare:

```text
fp32 256-D
fp16 256-D
int8 256-D
PCA->128 fp16
PCA->256 int8
```

Plot:

```text
recall@1 / recall@5
vs
bytes/keyframe
```

This is an Ariadne-specific contribution because representation quality and communication cost are jointly optimized.

> **Measured (3.6, D93, EuRoC, retrieval.md §4).**
> - fp16 and int8 with a per-vector scale (258 B) leave every query's top-k hit unchanged against fp32 (1024 B), at
>   376×240 and at 752×480.
> - PCA truncation helps nothing at 376×240 and loses up to 0.037 R@1 at 752×480.
>
> The wire candidate is int8 at 258 B. It needs a `wire.fbs` change (3.2), since `GlobalDescriptor` is `[float]`.

---

## 17.3 Aliasability diagnostics

Measure descriptor distributions for:

```text
same-place pairs
different-place pairs
```

Estimate overlap.

Report:

```text
ROC
precision-recall
false-closure probability
```

Do not hide the building's perceptual ambiguity behind a single aggregate score.

---

# 18. Dense reconstruction / 3D Gaussian representation

## 18.1 Canonical data flow

```text
validated keyframe poses
        +
stereo depth
        +
RGB
        |
        v
local point cloud
        |
        v
Gaussian initialization
        |
        v
submap optimization
        |
        v
submap-level merge
        |
        v
global Gaussian scene
```

---

## 18.2 Use `gsplat` as the implementation base

The algorithm catalogue found `gsplat` to be a clean Apache-2.0 implementation path, while many 3DGS repos pull restrictive Inria rasterizers transitively.

Source:
- https://github.com/nerfstudio-project/gsplat

For Ariadne:

- geometry/state ownership remains Ariadne,
- Gaussian kernels/rasterization can be provided by a dedicated reconstruction package,
- all camera poses supplied by the validated geometric backend.

### Process boundary `[DESIGN]`

Reconstruction is a **Python service** (`python/reconstruction/`) — PyTorch + gsplat (nerfstudio-project/gsplat,
rasterization backend) — fed by `KeyframeBundle` records over gRPC or spooled files. It is a separate process, not
a library linked into the C++ core, which is Correction D carried to its implementation conclusion.

```text
KeyframeBundle = {
    kf_id, agent,
    frame_id_of_pose,        // the pose-graph VERSION this was exported at
    T_WiC, K,
    rgb,                     // falloff-normalized albedo-proxy, §19.1
    depth + σ (fp16),
    exposure_state, lamp_state,
    mask
}
```

Poses carry the **pose-graph version** they were exported at. The service subscribes to PGO updates and
*re-anchors* (§18.6) rather than retraining, which is what keeps reconstruction off the localization critical
path.

### Compute placement

gsplat training runs on the g5/g6 node (§26). Target budget from the feasibility audit: ~30 min per room-scale
run on a `g5.xlarge` spot instance. The service checkpoints submaps to S3 so the console can stream `.ply` /
`.splat` exports (§29).

> **Interim deviation (D44, 2026-10-15):** the account's GPU quota is 0 in every region checked (re-requests
> escalated by AWS). Until it lands, the gsplat render step runs on Kaggle (CUDA) through the same service code,
> fed a small render package; everything before it (spool, fusion, initialization) runs locally on CPU. Manifests
> record `splat_backend`. The realtime demo still requires a live GPU, so this deviation does not cover the demo.

---

## 18.3 Gaussian state

```cpp
struct Gaussian {
    Vec3 mean;
    Mat3 covariance;
    Vec3 color;
    float opacity;

    float stability;
    float uncertainty;
    KeyframeId source;
    SubmapId submap;
};
```

---

## 18.4 Stable / unstable Gaussian lifecycle

Inspired by RTG-SLAM:

```text
NEW
  ↓
UNSTABLE
  ↓
SUPPORTED
  ↓
STABLE
```

Delete or merge Gaussians if:

- unsupported,
- repeatedly inconsistent,
- extremely transparent,
- redundant with another Gaussian,
- inconsistent under new geometry.

Source:
- https://arxiv.org/abs/2404.19706

---

## 18.5 Occupancy-aware fusion

For overlapping submaps:

1. transform into global coordinates,
2. voxelize occupancy,
3. group overlapping Gaussians,
4. compare geometric support,
5. compare photometric support,
6. merge if compatible,
7. retain competing hypotheses if uncertainty remains high.

This follows the broad direction demonstrated by MAGS-SLAM for geometry/appearance-aware collaborative Gaussian fusion.

Source:
- https://arxiv.org/abs/2605.10760

Do not allow photometric appearance to overwrite geometrically inconsistent structure.

**Status of this mechanism.** Occupancy-aware fusion is the *measured alternative*, not the default. The default
overlap policy is the simpler nearest-chunk pruning of §18.7, because it is a published baseline with a known
behaviour; this fusion scheme must beat it on measurement before it becomes the shipping path.

---

## 18.6 Submap anchoring so PGO never invalidates splats

Each submap is anchored to a **reference keyframe**, and all of its Gaussians live in that keyframe's frame.

A PGO update therefore changes only the anchor's global pose, and **the Gaussians move rigidly for free**. No
retraining, no re-fitting, no invalidation.

Intra-submap drift correction is deliberately *not* attempted. Instead submaps are kept short — 20–40 keyframes,
with a new submap started on a distance or rotation threshold, or immediately after any loop closure that bends
the local trajectory beyond a residual threshold. Within a submap, drift is small enough to ignore; across
submaps, the pose graph handles it. This is the concrete form of the decoupling asserted in Correction D, and it
composes directly with the frame algebra of §15.5.

---

## 18.7 Merge and appearance, pinned to verified mechanisms

- **Overlap.** Nearest-chunk pruning from *Hierarchical 3D Gaussians* (Kerbl et al., SIGGRAPH 2024) as the
  baseline: a Gaussian outside its own submap's cell that is nearer another submap's cell is deleted. Measure this
  before adopting anything more elaborate (§18.5).
- **Exposure.** A per-keyframe 3×4 affine color transform, initialized to identity, and **enabled only after
  ~5000 optimization steps** so geometry stabilizes first. The catalogue flags skipping that delay as the classic
  failure: the transform absorbs geometric error into color and the reconstruction converges to something that
  renders well and is geometrically wrong. With the §19.1 normalization in place, this transform's correct job is
  small — absorbing residual auto-exposure differences between agents.
- **Never voxel-average Gaussians during merge** (the Fed3DGS negative result). If union growth becomes a real
  problem, the documented remedy is Fed3DGS-style distillation with a replay camera bank.

---

# 19. Photometric reconstruction under co-moving illumination

## 19.1 The coherence problem, and the default resolution

State the problem explicitly, because it invalidates the naive pipeline rather than merely degrading it:

**3DGS reconstructs a static radiance field. Under carried illumination, the scene's radiance is a function of the
observer.** Naively-trained splats are therefore photometrically incoherent — the same surface is bright in one
view and dark in another for reasons the model cannot represent — and held-out PSNR measures the wrong thing
entirely.

**Resolution, adopted as the default pipeline: reconstruct an albedo-proxy field, not a radiance field.**

- **Training images** are the falloff-normalized outputs of §6.2 Stage 3, which divide out `P·cosθ/r²` using the
  calibrated lamp and the current depth. The Gaussian color then approximates albedo up to a global scale.
- **Display**: the console renders either the albedo-proxy directly (honest, slightly flat) or *relights* it with
  a virtual lamp using the same forward model (photorealistic and physically consistent). Both are legitimate;
  label which is shown.
- **Held-out evaluation** renders **with the light model applied at the held-out camera's lamp pose**, so
  PSNR/SSIM compare like with like. Without this, the headline reconstruction metric is not measuring
  reconstruction quality.

With this in place, the `L_light` term of §19.3 optimizes the *residual* near-field effects that normalization
missed, rather than carrying the entire illumination burden — which is what it would otherwise be asked to do,
and would fail at. The GLOW-style specular downweighting of §19.4 applies to the normalized images unchanged.

---

## 19.2 Illumination model

For a Gaussian/surface sample at `x` and light position `l`:

```text
r = ||x - l||

E ∝ 1/r²
```

optionally:

```text
E ∝ cos(theta) / r²
```

Use the measured light calibration to determine whether the cosine term materially improves fit.

---

## 19.3 NFL-aware loss

Implement a practical loss:

```text
L_total =
    λ_photo L_photo
  + λ_depth L_depth
  + λ_normal L_normal
  + λ_light L_light
  + λ_reg L_reg
```

where `L_light` explains intensity variation induced by the changing light-surface distance.

Source:
- https://arxiv.org/abs/2412.13176

---

## 19.4 Specular robustness

GLOW specifically identifies dynamic specular highlights as a problem for co-located flashlight captures and introduces a surface-angle-weighted radiometric loss.

Source:
- https://arxiv.org/abs/2511.22857

Implement the underlying idea:

```text
weight_specular =
    f(view_direction, light_direction, surface_normal)
```

downweight pixels likely to be highlight-dominated.

Do not attempt a full neural inverse renderer for the baseline.

---

# 20. Volumetric mapping, ESDF, frontiers, and traversability

New module: `cpp/planning/volumetric/`.

This section exists because §21 and §22 consume occupancy, clearance, frontiers and traversability that nothing
else in the architecture produces. The Gaussian representation of §18 is a *rendering* product; it is not a
queryable free-space structure, and a planner cannot ask it whether a corridor is passable.

## 20.1 TSDF

Integrate keyframe depth (§9.6) into OpenCV 5 **HashTSDF** — dense RGB-D fusion with TSDF, HashTSDF and ColorTSDF
ships in the 5.0 release `[VERIFIED web 2026-08-30]` — at **8–10 cm voxels**, per agent, in that agent's frame
`W_i`. Sample weights are `1/σ_Z²` from the stereo uncertainty.

`[SMOKE-TEST]` the exact class names and API in the installed build. If the API proves awkward for incremental
multi-block use, the fallback is a from-scratch voxel-hash TSDF (~600 lines) **with the OpenCV version retained as
the correctness oracle** — the fallback is a performance decision, never an excuse to skip cross-validation.

> **As built (D110, 2026-10-23).** The fallback is taken: `cv::Volume` exposes no voxel or weight read (SMOKE §4u),
> so 4.1 is a from-scratch voxel-hash TSDF, with HashTSDF as a test oracle. **The voxel is 5 cm, not 8–10 cm:** on
> both scenes' GT, a one-cell wall bleed closes agent 1's entrance at ρ_responder for 8 and 10 cm voxels. Unknown is
> weight 0 exactly (§20.2). **§20.3 as built:** clearance is centre to the nearest point of a blocked cube (the GT's
> convention), by exact windowed min-convolutions. FH's parabolas are centre-to-centre, so they do not give it.
> Design: `docs/design/volumetric.md`.

## 20.2 Occupancy

A tri-state derived from TSDF weight and sign:

```text
free  |  occupied  |  unknown        (unknown = weight 0)
```

The distinction between "observed free" and "unknown" is load-bearing everywhere downstream — it is the difference
between a route that is known to exist and one that merely has not been contradicted.

## 20.3 ESDF for clearance

A 3D Euclidean distance transform over the occupied set, via three-pass separable Felzenszwalb–Huttenlocher 1D
lower-envelope transforms (*Distance Transforms of Sampled Functions*, Theory of Computing 2012) `[PAPER]`.

**Recompute incrementally per updated block, never globally.** A global recompute per keyframe is the obvious
implementation and it will not hold frame rate.

## 20.4 Frontiers

Free voxels 6-adjacent to unknown, clustered by connected components, each cluster summarized by:

```text
centroid  |  normal  |  visible-unknown volume estimate
```

These are candidate generator A of §21.2.

## 20.5 Traversability and risk

```text
traversable(x)  <=>  ESDF(x) >= ρ
```

where **ρ is the consumer cross-section radius, and it is a first-class parameter with two values**:

```text
ρ_agent      = 0.15–0.25 m    (agent motion planning)
ρ_responder  = 0.40 m default, config   (the egress product)
```

> **As built (D111, 2026-10-23).** Egress traversability at ρ_responder = an observed column (D45's rule, with
> clearance to non-free columns ≥ ρ) **and** a standing-body capsule on the occupied-only 3D field. A capsule on the
> unknown-as-blocked field would be stricter, but the rig cannot observe a standing body's volume: on the evidence
> spool it closes the target. The structural risk proxy is built, and **uncalibrated** until the GT-zone scenes exist.

**Every egress claim in the report and in the demo is computed at `ρ_responder`.** An agent-passable void is not a
human-passable route, and conflating the two is the single easiest embarrassment to preempt — a judge asking "can
a person actually fit through that?" should meet a parameter, not a silence.

**Structural risk** is an overhang score from TSDF normals plus an unsupported-mass heuristic, calibrated against
the GT-labeled zones in the simulated scenes. **Label it a proxy everywhere it appears** — it belongs on the
honesty list of §44. Ariadne does not do structural engineering, and a risk number that looks like a load
calculation invites exactly the wrong interpretation.

## 20.6 Global coarse layer

A 0.5 m downsampled occupancy/uncertainty volume over the global frame `G`, used by:

- the TARE-style global exploration stage (§21.5),
- cross-agent coverage deconfliction.

Exchanged as run-length-encoded submap summaries at message class 5 (§15.3) — the lowest priority class, and
therefore the first thing the spectral policy cancels under budget pressure, which is correct: coarse coverage is
the most deferrable product Ariadne exchanges.

---

# 21. Active perception / next-best-view

This is the major addition that was still missing as a completed domain in `algorithms.md(1).md`.

## 21.1 Design principle

A candidate action is valuable when it improves:

- geometric uncertainty,
- coverage,
- map connectivity,
- place-recognition ambiguity,
- reconstruction quality,
- egress validity,

minus:

- travel cost,
- expected energy cost,
- communication cost,
- mission time,
- structural risk.

---

## 21.2 Candidate generation

Generate candidate viewpoints from:

### A. Frontier surfaces

From the frontier clusters of §20.4 (centroid, normal, visible-unknown volume).

### B. Uncertainty hotspots

Regions with:

```text
high depth variance
high pose influence
low observation count
high route relevance
```

### C. Connectivity targets

Candidate views that can bridge two disconnected map regions.

### D. Alias resolution targets

Locations where visually similar regions need a discriminative observation.

### E. Egress uncertainty

Views that can validate a route's:

- clearance,
- obstruction,
- structural confidence.

---

## 21.3 Information gain approximation

Do not calculate an expensive full posterior over the entire scene.

Approximate:

```text
IG(action) =
    Δcoverage
  + α Δentropy
  + β Δpose_connectivity
  + γ Δegress_confidence
  + δ Δloop_disambiguation
```

### Computable form of each term

Each of these is a concrete raycast or lookup against structures that now have producers — none is a placeholder:

| Term | Computation |
|---|---|
| `Δcoverage` | expected newly-observed unknown voxels, by raycasting the candidate frustum through the occupancy map of §20.2 with a sparse bundle of 200–500 rays |
| `Δentropy` | over the same rays, with `p(occupied)` from TSDF weight |
| `Δpose_connectivity` | the **spectral gain** (§16.2) of the inter-agent edge this view is predicted to enable |
| `Δegress_confidence` | predicted removal of low-confidence voxels on the current best route — raycast intersection with the route corridor (§22) |
| `Δloop_disambiguation` | for alias-flagged place pairs (§17.3), the value of a viewpoint predicted to be discriminative, i.e. one from which the two hypotheses show different geometry |

`Δpose_connectivity` is the term that makes "fly toward a peer's trajectory" competitive with "go explore", and it
is what connects the active planner to the collaboration layer rather than leaving them as two independent
systems.

### Normalization

Normalize each term by its **running 90th percentile across the current candidate set**. This is self-normalizing
and introduces no magic constants — which matters because these five terms have incommensurable units and any
fixed scaling would be an unfalsifiable tuning choice.

---

## 21.4 Risk-aware utility

Use:

```text
U(a) =
    G(a)
  - λd C_motion
  - λb C_bandwidth
  - λe C_energy
  - λt C_time
  - λr C_risk
```

This preserves the original research hypothesis in the proposal.

**C_time as built (D117, 2026-10-24; next-best-view.md §15).**
- C_time is the duration of the mover's own command profile (`core/trajectory/command_planner.hpp` `PlanCommand`)
  over the exact command the loop will send for the view, from the agent's current heading. It covers every turn in
  place, every clearance-limited move, and the final turn.
- D114 collapsed motion, energy and time into one path term ("all ∝ path"). That holds only for a rig that yaws while
  it translates. The kinematic rig turns in place: the 4.3 gate measured 45 % of Ariadne's mission time spent turning.
- With `nbv.cost_time: exact_command`, λ_t's third of the path weight therefore prices C_time instead:
  `U = G − [(λ_d⁰ + λ_e(t))/3]·C_path/p90 − [λ_t(t)/3]·C_time/p90 − C_b/p90 − C_r/p90`. The weights still sum to 1 at
  t = 0.
- C_time is a [PROXY]: it uses the planner's clearance, not GT.
- Views may also be offered at their arrival heading (D118, `nbv.view_yaw: target+arrival`).
- Ablation rows: next-best-view.md §15.5.

### λ scheduling — "the weights are not fixed", made real

```text
λ_e(t) = λ_e⁰ · (1 + (E_used / E_total)³)
```

Energy cost inflates **cubically** toward the endurance limit; `λ_t` behaves the same way against the mission
clock. Early in a mission the agent is exploratory; late in a mission the same utility function is conservative,
without any mode switch.

### The hard rule that sits outside the utility function

```text
if  E_remaining < 1.3 × E(return path):
        candidate set := { return_to_safe_zone, hold }
```

This is a **hard constraint, not a term**, and it is logged as a supervisor event (§24). The reasoning is
deliberate: a utility function is a tuning surface, and no tuning error — however large a weight, however wrong a
gain estimate — may be permitted to strand an agent. Anything that can strand an agent must be unreachable from
the optimizer, not merely expensive within it.

---

## 21.5 TARE-inspired multi-resolution exploration

TARE uses dense local planning with sparse global representation.

Source:
- https://www.cmu-exploration.com/tare-planner
- https://github.com/caochao39/tare_planner

Ariadne adaptation:

```text
global layer:
    coarse occupancy / topology graph      (§20.6, 0.5 m)
    low-resolution uncertainty

local layer:
    dense voxel map
    detailed candidate viewpoints
    collision / clearance evaluation
```

This is a strong fit because Ariadne's world can be large while local camera planning needs precision.

---

## 21.6 FAR-style route validation

FAR Planner uses a dynamically updated visibility graph and rapid replanning.

Source:
- https://arxiv.org/abs/2110.09460
- https://github.com/MichaelFYang/far_planner

Adaptation:

```text
surface/clearance observations
 -> free-space boundary extraction
 -> dynamic visibility graph
 -> fast route query
```

Use this for operational route planning, not as the only exploration policy.

---

## 21.7 Multi-agent assignment

For a small team:

1. each agent computes local candidate utility,
2. tasks are spatially clustered,
3. agents bid on tasks,
4. consensus resolves duplicate ownership,
5. assignments update on new information.

CBBA is a strong baseline because it has a mature decentralized auction/consensus formulation.

Source:
- https://pure.kaist.ac.kr/en/publications/consensus-based-decentralized-auctions-for-robust-task-allocation/

### Concrete configuration `[PAPER]`

CBBA (Choi, Brunet & How, *Consensus-Based Decentralized Auctions for Robust Task Allocation*, T-RO 2009),
restricted to **bundle size 1–2**: Ariadne's tasks are short-horizon views, so long bundles buy little and
complicate consensus considerably.

**Event-triggered**, not consensus-synchronized per frame — the right structure for a communication-constrained
fleet, where a per-frame consensus round would spend the very byte budget the collaboration layer exists to
conserve. Bid rounds run only on:

```text
new task cluster  |  completed task  |  topology change
```

Consensus messages ride the class-2 priority of §15.3.

Broker-connected components may shortcut to a **sequential greedy auction at the broker**. Both paths are logged
identically, which makes the "auction vs broker-greedy" ablation row nearly free — and it is a genuinely
interesting row, because it measures what decentralization costs when centralization happens to be available.

A current 2026 line of work also motivates group-level bidding when tasks are spatially correlated; see the recent grouping auction-consensus work:
- https://arxiv.org/abs/2608.15884

Treat that as an experimental extension, not a hard dependency.

---

## 21.8 Best practical planner

The production planner should be a hybrid:

```text
candidate generation:
    uncertainty/frontier/connectivity

candidate scoring:
    EIG approximation + mission costs

task grouping:
    spatial clustering

assignment:
    auction/consensus

within-task path:
    FAR-style dynamic visibility graph / A* fallback

global exploration scheduling:
    TARE-style coarse-to-fine representation
```

This is more defensible than a pure learned policy because every decision is inspectable.

---

# 22. Route / egress subsystem

## 22.1 Representation

Construct a spatial cost field:

```text
C(x) =
    w_clearance C_clearance
  + w_uncertainty C_uncertainty
  + w_risk C_structural
  + w_traversability C_traversability
```

### Executable form

**Graph:** a 26-connected lattice over free voxels (§20.2) with `ESDF ≥ ρ_responder` (§20.5).

> **As built (D111):** a 2.5D 8-connected lattice over the egress-traversable columns of §20.5's capsule rule. A
> responder walks on the floor, and a 3D lattice would route through the air above props. The 3D obstacle test lives
> in the capsule.

**Edge cost:**

```text
c(x→y) = ||x−y|| · ( 1
                   + w_u · U(x,y)
                   + w_r · Risk(x,y)
                   + w_c · max(0, ρ_safe − ESDF) / ρ_safe )
```

with `U` derived from observation count and TSDF weight.

**Planner: D\* Lite** (Koenig & Likhachev, AAAI 2002) `[PAPER]`. Incremental replanning is the right match for a
regime where the voxels change underneath the plan. The FAR-style visibility graph (§21.6) remains the
*exploration-time* router and the comparator row in §31.6 — the two are complements, not competitors.

**The ρ discipline:** every route in the egress product, every egress metric, and every route shown in the demo is
computed at `ρ_responder`, never at `ρ_agent`. The agent may reach places the route may not claim.

---

## 22.2 Route confidence

For a route `R`:

```text
confidence(R) =
    aggregation of local support/confidence along R
```

Use a conservative aggregation such as a low quantile or harmonic-type combination rather than a simple mean.

Reason: one catastrophic low-confidence segment should matter more than twenty excellent ones.

**Concretely:** the 5th percentile of per-voxel support along the route.

### Egress metrics emitted per mission

Added to the Mission block of §30.2:

1. **Time-to-first-valid-route** — the first route whose every voxel is *observed* free at `ρ_responder`. Not
   "not known to be blocked": observed.
2. **Route validity vs GT traversability**, from the `scene_gt.npz` layer baked in §5.5.
3. **Route-confidence calibration** — predicted confidence against realized validity across runs. This is a
   genuinely novel little plot and a cheap one: it answers whether Ariadne's stated confidence *means* anything,
   which is the question a rescue operator would actually ask.

---

## 22.3 Dynamic replanning

Replan whenever:

```text
new obstacle discovered
route confidence falls
structural risk estimate changes
agent becomes unavailable
communication changes
```

D* Lite is an academically well-founded incremental replanning baseline.

Source:
- https://aaai.org/papers/00476-aaai02-072-d-lite/

The choice between the two route mechanisms is already settled in §22.1 and is not reopened here: **D\* Lite is
the egress planner** over the voxel lattice, and the FAR-style visibility graph is the exploration-time router
(§21.6). They operate on different structures at different rates and are complements, not alternatives.

What §31.6 measures is therefore not "which one wins" but the cost of the split — replan latency and path quality
for each, on the same scenarios:

```text
D* Lite                    (egress, incremental, ρ_responder lattice)
FAR-style visibility graph (exploration-time routing)
```

---

# 23. Human authority / risk gate

Autonomy must remain bounded.

A proposed action has:

```text
risk_score
expected_information_gain
distance
energy
confidence
reason
```

If:

```text
risk_score > autonomous_threshold
```

generate:

```text
HUMAN_APPROVAL_REQUEST
```

with:

- map snapshot,
- agent position,
- intended action,
- evidence,
- uncertainty,
- predicted benefit,
- risk reason,
- alternative action.

If communication to command is absent:

```text
approved envelope -> continue
outside envelope  -> hold/defer/backtrack
```

---

# 24. Failure supervisor

## 24.1 State machine

Each component has:

```text
HEALTHY
DEGRADED
FAILED
RECOVERING
```

Monitor:

- tracker inlier count,
- BA condition number,
- pose covariance,
- CPU time,
- GPU memory,
- message queues,
- heartbeat freshness,
- battery,
- Gaussian count,
- map growth rate.

---

## 24.2 Last-safe-state checkpointing

Persist:

```text
last valid keyframe
last valid local pose graph
reference transform
task state
communication watermarks
```

On failure:

```text
stop component
restore checkpoint
discard in-flight state
replay deterministic inputs
restart
```

This follows the failure-recovery lessons captured from SubT deployments in the algorithm catalogue.

---

# 25. Agentic orchestration layer

## 25.1 Keep LLM outside the numerical core

LLM/agent reasoning should never own:

- camera pose estimation,
- Jacobian computation,
- PGO numerical solve,
- raw image correspondence,
- safety thresholds without a deterministic rule layer.

The agentic layer owns:

- task selection,
- mission-level negotiation,
- explanation,
- exception handling,
- invocation of tools,
- human escalation.

### The rule, enforced in code rather than by convention

```text
the deterministic planner computes the candidate action set and the safety envelope;
the LLM may only CHOOSE AMONG presented candidates and compose escalation packets.
```

An out-of-set response falls back to the deterministic choice and **logs the event** — and that log line is itself
good failure-handling evidence, because it demonstrates the guardrail firing rather than merely asserting that one
exists.

Cadence: 0.2–1 Hz. The classical-planner ablation of §25.3 then becomes literally "skip the choose step", which is
why the ablation is nearly free.

---

## 25.2 MCP tool surface

**Placement:** run the orchestrator LLM through **Amazon Bedrock** (any strong tool-use model, invoked via the
converse/tool-use API). This makes the agentic layer part of the "meaningful AWS component" story rather than an
external dependency bolted onto it.

The MCP server lives in `python/orchestration/` and exposes exactly the tool list below. **Every tool call, and
the vision-derived inputs that motivated it, is appended to the decision trace** — JSONL in the run artifact,
which is the decision-trace deliverable.

Expose:

```text
observe_image
estimate_pose
compute_depth
query_uncertainty
retrieve_place_candidates
verify_loop
request_peer_observation
allocate_task
request_rendezvous
query_route
request_human_approval
```

The runtime trace must show:

```text
vision output
    ->
agent decision
    ->
tool call/action
```

This is the agentic-behaviour requirement the design is held to.

Requirements of the original brief:
- OpenCV 5 and AWS are hard requirements.
- The agentic layer requires vision output to influence a subsequent plan/tool/action/approval.
- The project must provide a trace and evaluation of failure handling/human control.

Source:
- the earlier challenge brief (not part of this repository)

---

## 25.3 Classical planner ablation

Run exactly the same environment with:

```text
A. deterministic Ariadne planner
B. LLM/MCP orchestrated planner
```

If B wins:

```text
quantify why
```

If B does not:

```text
say so
```

Credible engineering matters more than decorative agent terminology.

---

# 26. AWS architecture

## 26.1 Split CPU and GPU workloads

The architecture should explicitly separate:

### Graviton / COOL

Run:

- per-frame OpenCV processing,
- radiometric processing,
- filtering,
- feature preprocessing,
- XFeat/ALIKED where supported,
- geometric verification,
- evaluation benchmarks.

### x86 GPU

Run:

- Gazebo rendering when GPU acceleration is useful,
- Gaussian optimization/rendering,
- expensive feed-forward model experiments,
- batch evaluation.

This is the correct use of a heterogeneous cloud environment.

---

## 26.2 Primary AWS topology

```text
                           +----------------------+
                           |        S3            |
                           | datasets / artifacts |
                           +----------+-----------+
                                      |
                +---------------------+----------------------+
                |                                            |
                v                                            v
     +----------------------+                       +----------------------+
     | Graviton4 EC2        |                       | GPU EC2              |
     | m8g.4xlarge          |                       | g5/g6 family         |
     | COOL + OpenCV 5      |                       | Gazebo + 3DGS        |
     +----------+-----------+                       +----------+-----------+
                |                                               |
                +-------------------+---------------------------+
                                    |
                                    v
                         +----------------------+
                         | Agent orchestrator   |
                         | ECS/EC2 service      |
                         +----------+-----------+
                                    |
                    +---------------+----------------+
                    |                                |
                    v                                v
              Operator API                    Evaluation jobs
                    |                                |
                    v                                v
             Web operator UI                   S3 metrics
```

---

## 26.3 COOL node

AWS Marketplace currently documents COOL on AWS Graviton4 and recommends m8g.4xlarge or larger for performance.

Source:
- https://aws.amazon.com/marketplace/pp/prodview-fdvbfiewzuehs
- https://aws.amazon.com/about-aws/whats-new/2024/09/amazon-ec2-c8g-m8g-instances/
- https://docs.aws.amazon.com/ec2/latest/instancetypes/gp.html

Implementation:

```text
Amazon Linux/Ubuntu ARM64
+
COOL AMI
+
Ariadne ARM64 container
```

The final benchmark must compare:

```text
stock OpenCV 5 on same Graviton instance
vs
COOL OpenCV on same Graviton instance
```

Do not compare a Graviton CPU against a desktop GPU and call it a COOL speedup.

---

## 26.4 GPU node

Use:

```text
g5.xlarge
```

or a later AWS GPU family available at submission time.

Workloads:

- Gazebo rendering,
- Gaussian mapping,
- larger ONNX models,
- training/fine-tuning.

Make the GPU node disposable.

---

# 27. Containerization

## 27.1 Images

| Image | Base | Contents | Runs on |
|---|---|---|---|
| `ariadne-core` (amd64 + arm64, one Dockerfile via buildx) | `ubuntu:24.04` | gcc-13, CMake, Eigen 3.4, **OpenCV 5 + contrib built from a pinned git tag**, FlatBuffers, zstd, ONNX models, ariadne cpp | dev, CI, Graviton (stock baseline) |
| `ariadne-cool` (arm64) | COOL AMI toolchain, layered per vendor instructions | same ariadne cpp, linked / `LD_LIBRARY_PATH`-switched against `/opt/cool` OpenCV | `m8g.4xlarge` |
| `ariadne-sim` (amd64) | `ros:jazzy` | gz-harmonic, `ros-jazzy-ros-gz`, worlds, EGL libs, degradation operator | g5/g6 |
| `ariadne-gpu` (amd64) | `nvidia/cuda:12.x-runtime-ubuntu24.04` | PyTorch (pinned cu12x), gsplat (pinned), reconstruction service, OpenCV 5 py | g5/g6 |
| `ariadne-console` (amd64) | `python:3.12-slim` | FastAPI + WebSocket API, static Three.js frontend | t4g/t3.small or ECS |

Every image builds `opencv` + `opencv_contrib` at the **same pinned tag** with an explicit
`-DOPENCV_EXTRA_MODULES_PATH` (C7), and records a hash of `cv::getBuildInformation()` in the run manifest.

### COOL benchmark discipline

```text
same binary  |  same instance  |  same inputs
the ONLY varied element: which OpenCV the loader resolves
                         (stock build vs /opt/cool)
```

Pin `cv::setNumThreads` and thread affinity identically across both arms. Report:

```text
p50 / p95 per-op and per-pipeline latency
frames per second
utilization (perf stat)
cost per frame = (instance $/hr) / (3600 · fps)
```

and sweep the concurrent agent-pipeline count to the knee. This satisfies the COOL rubric's evidence list and
honours the Arm-vs-Arm rule of §26.3.

Given Universal Intrinsics 2.0 (C3), **also report stock OpenCV 5 Arm numbers against OpenCV 4.x once**, as
context for how far the baseline itself moved. Without that context a COOL delta is uninterpretable — the reader
cannot tell an optimization from a version bump.

---

## 27.2 Build and CI

GitHub Actions. amd64 jobs on hosted runners; arm64 jobs on GitHub's Arm hosted runners if available to the
account `[SMOKE-TEST]`, otherwise a self-hosted runner on the Graviton box — registered once, doubling as the
benchmark machine, with benchmark jobs serialized behind a lock so a CI build never contaminates a timing run.

The OpenCV 5 compile is cached via ccache plus a prebuilt `opencv-base` layer, rebuilt only when the pinned tag
changes.

Push to ECR. **Images are referenced by digest, never by tag, in run manifests** — a tag is not a version.

---

## 27.3 Infrastructure as code

Terraform modules:

```text
network   : one VPC, public subnet, security groups
storage   : the S3 buckets of §28.1 + lifecycle (runs/ -> IA at 30 d)
registry  : ECR
compute   : launch templates for m8g.4xlarge, g5.xlarge/g6.xlarge
            (spot with on-demand fallback)
sweeps    : SQS queue + a small ASG of workers whose user-data pulls a
            manifest message, runs the container, uploads artifacts,
            terminates on idle
budget    : AWS Budgets alarm at 50% and 80% of credits
```

**No EKS, no Kubernetes.** The sweep pattern is SQS plus containers, which one person can operate under
competition time pressure. Orchestration complexity that needs a second operator is a liability, not an asset.

---

## 27.4 Base design

### CPU image

```dockerfile
FROM ubuntu:24.04

C++17
CMake
Eigen
OpenCV 5
ONNX models
Ariadne core
```

OpenCV 5 requires C++17 according to the current OpenCV 5 documentation.

Source:
- https://github.com/opencv/opencv/wiki/OpenCV-5

### ARM image

Same source tree, linked against `/opt/cool` OpenCV environment.

### GPU image

NVIDIA runtime with:

```text
CUDA
PyTorch
gsplat
OpenCV 5
Ariadne reconstruction
```

---

# 28. AWS storage and orchestration

## 28.1 S3 layout

```text
s3://ariadne/
    datasets/
    scenes/
    checkpoints/
    runs/
        YYYY/MM/DD/run_id/
            config.yaml
            manifest.json
            metrics.parquet
            trajectories/
            maps/
            logs/
            video/
```

---

## 28.2 Run manifest

Every run records:

```yaml
git_commit:
opencv_version:
cool_version:
cuda_version:
scene:
seed:
agent_count:
camera_config:
illumination_config:
communication_config:
planner_config:
feature_engine:
place_recognition:
loop_backend:
reconstruction_backend:
```

This is critical for reproducibility.

> **As built (D60, 2.1).** Front-end entries recorded under the manifest's `extra` map:
> - `reassociation` (`off` or `lost_window/latest_keyframe[/admit_bridged]`);
> - `replenish_cadence` (`every_frame` | `keyframe`);
> - for `feature_engine: xfeat`: `xfeat_dnn_engine` (`opencv_dnn_graph` | `gemm_reference`), `xfeat_input` (e.g.
>   `resize:736x480`), `xfeat_weights`, `xfeat_weights_checksum` (sha256 from the export's provenance file) and
>   `xfeat_upstream_commit`.
>
> `results.yaml` gains `reassociation:` and `xfeat:` lines when they are enabled.

---

## 28.3 Event transport

For the hackathon prototype, avoid an elaborate distributed cloud bus.

Use:

```text
ROS2 / local runtime
+
gRPC or WebSocket for service boundaries
+
S3 for artifacts
```

Optionally use:

```text
SQS
```

only for asynchronous cloud job orchestration.

Do not put every camera frame through SQS.

---

# 29. Operator console

## 29.1 Main view

```text
3D reconstruction
+
agent trajectories
+
current tasks
+
communication links
+
uncertainty field
+
egress route
```

---

## 29.2 Incident timeline

Show:

```text
22:41 Agent 2 enters low-confidence corridor
22:42 retrieval finds 12 candidates
22:42 spectral selector selects 2
22:42 PCM rejects 1
22:42 GNC rejects 1 false closure
22:43 Agent 2 requests peer observation
22:44 peer rendezvous succeeds
22:44 route confidence rises
```

This is much more compelling than simply displaying a 3D viewer.

---

## 29.3 Implementation

**Hosted-demo amendment (2026-10-26):** the implemented session lifecycle, fixed
four-pane layout, controller lease, camera/feature packet contract, observed-point
fallback, recorded playback and post-run training are specified in
[`design/live-runtime.md#web-demo-console`](design/live-runtime.md#web-demo-console). Demo supervision keeps
one request pending through human review while classical planning continues; it
revalidates delayed answers against the current map. Ground truth is an explicitly
separate preview. Hardware performance and publication remain release gates.

**Backend:** FastAPI — WebSocket telemetry at 2–10 Hz, REST for approvals.
**Frontend:** Three.js with a Gaussian-splat web renderer (e.g. the GaussianSplats3D library), streaming submap
exports from S3 (§18.2).

> **Update 2026-10-23 (D197, §49 ritual, user-approved):** the renderer is **Spark** (sparkjs, MIT). GaussianSplats3D's
> README declares it unmaintained and points to Spark. Spark sorts many `SplatMesh` objects together, each with its
> own transform, which is what keyframe-anchored submaps need (D190): one mesh per submap, and re-anchoring is a new
> transform, never a re-download.

Panels, per §29.1–29.2: raw vs restored feeds side by side, budget gauges, negotiation timeline, uncertainty
overlay, the thread, and the approval modal wired to `HUMAN_APPROVAL_REQUEST` (§23).

This is the "working web endpoint" submission artifact. **Host it live for judging week** on one small instance —
a console that only runs on the developer's laptop is not a deliverable.

---

# 30. Evaluation harness

## 30.1 Ground-truth evaluation

Every simulation provides:

```text
GT pose
GT mesh
GT depth
GT traversability
GT route
GT communication graph
```

---

## 30.2 Core metrics

### Localization

```text
ATE RMSE
RPE
rotation error
translation error
```

### Registration

```text
inter-agent relative pose error
accepted/rejected closure precision
false closure rate
```

### Mapping

```text
Chamfer distance
completeness
coverage
held-out-view PSNR
SSIM
LPIPS
```

### Communication

```text
total bytes
bytes/keyframe
bytes/verified loop
bytes/map-area-reconstructed
descriptor bytes
local feature bytes
pose graph bytes
```

### Mission

```text
time to first usable map
time to first valid egress route      (all voxels OBSERVED free at ρ_responder, §22.2)
route validity vs GT traversability
route confidence
route-confidence calibration          (predicted confidence vs realized validity, §22.2)
distance travelled
energy proxy
```

---

# 31. Ablation matrix

Run a formal matrix.

## 31.1 Collaboration

```text
solo
independent multi-agent
greedy similarity
greedy coverage
Ariadne spectral
Ariadne spectral + vertex-cover exchange
```

## 31.2 Robustness

```text
no robustification
Huber
DCS
Switchable Constraints
GNC
PCM + GNC
degeneracy gate + PCM + GNC
```

## 31.3 Features

```text
ORB
SIFT
XFeat
ALIKED
XFeat + LightGlue
ALIKED + LightGlue
```

## 31.4 Place recognition

```text
DBoW-style
EigenPlaces-128
EigenPlaces-256
MegaLoc reference
AnyLoc reference
```

> **As built (2.1, D62).** Index axis: `exact` (one gemm, the default) vs `annoy` (`cv::ANNIndex`, trees/seed in the
> manifest), the scale ablation. EigenPlaces-256 is built at 376×240 (default) and 752×480 (perception.md §11).

## 31.5 Illumination

```text
ambient normal
dark
dark + fixed light
dark + co-moving light
dark + co-moving light + radiometric correction
dark + co-moving light + NFL-aware reconstruction
dark + co-moving light + retrieval-path enhancement only   (§6.3 asymmetry test)
dark + co-moving light + albedo-proxy reconstruction        (§19.1 default)
fixed exposure vs active exposure + lamp co-control         (§6.5)
```

## 31.6 Planner

```text
random
greedy frontier
coverage utility
TARE-inspired
risk-aware information utility
Ariadne multi-agent planner
```

Route query comparator: `D* Lite` (§22.1, production) vs FAR-style visibility graph (§21.6).

## 31.7 Allocation

```text
broker-greedy sequential auction
CBBA event-triggered, bundle size 1
CBBA event-triggered, bundle size 2
```

Nearly free to run, because both paths are logged identically (§21.7).

## 31.8 Sensor configuration

```text
stereo + IMU (full)             (production, §10)
stereo + IMU (attitude_only)    (§10.7 fallback)
stereo, vision-only
monocular + Sim3 inter-agent    (C2/C5: quantifies what scale observability buys)
stereo baseline sweep: 4 / 8 / 12 / 20 cm    (§9.1, subject to the §9.4 validator)
```

## 31.9 Descriptor compression

```text
XFeat descriptor fp16 vs int8 with per-keyframe scale       (§15.7)
global descriptor fp32 / fp16 / int8 / PCA variants         (§17.2)
```

Reported as precision/recall against bytes, never as a byte saving alone.

---

# 32. SOTA/reference comparison table

| Function | Primary Ariadne | Reference/comparator | Why |
|---|---|---|---|
| Local features | XFeat | ALIKED, DISK, ORB | XFeat gives compact efficient features |
| Hard matching | ALIKED + LightGlue | BF | contextual adaptive matching |
| Frame tracking | LK + geometric filtering | SVO-style | cheap repeated tracking |
| Stereo | StereoSGBM | learned stereo | CPU/OpenCV-native metric geometry |
| Local BA | custom Schur LM | Ceres/GTSAM oracle | technical depth |
| PGO | custom GNC | GTSAM | robust multi-agent backend |
| Retrieval | EigenPlaces 256-D | MegaLoc/AnyLoc | balance accuracy/bandwidth |
| C-SLAM | spectral + sparse exchange | Swarm-SLAM | direct communication objective |
| Loop robustness | degeneracy + PCM + GNC | PCM only | avoids degenerate hypotheses |
| Reconstruction | 3DGS / gsplat | Photo-SLAM / RTG-SLAM ideas | decoupled robust geometry + photorealism |
| Multi-agent 3DGS | local Gaussian submaps | MAGS-SLAM | current monocular/multi-agent frontier |
| Exploration | TARE-inspired | TARE | multiresolution |
| Egress route | D* Lite over the ESDF lattice | FAR-style visibility graph | incremental replanning as voxels change |
| Exploration routing | FAR-style visibility graph | A* fallback | fast queries over a sparse graph |
| Allocation | event-triggered auction/consensus | CBBA | decentralized conflict resolution |
| Illumination | calibrated co-moving-light model | NFL-BA / GLOW | physically appropriate |
| Low light | classical photometric normalization | learned enhancement | geometry-first |
| Certification | SE-Sync-style certificate | no certificate | independent optimizer validation |

---

# 33. Scoped development program

The development process is intentionally **cross-cutting and synchronized**. Do not build one giant subsystem for weeks and only then discover that its assumptions do not interface with the others.

Each iteration adds a vertical slice through the whole system.

---

## Iteration 0 — repository and deterministic infrastructure

### Implement

- monorepo
- CMake
- Python analysis env
- config schema
- serialization layer
- timestamp types
- SE(3) math
- logging
- run manifests
- deterministic seeds
- test harness
- CI
- **MCAP record/replay (§5.9)** — the determinism substrate everything downstream is tested against
- **the seeded degradation operator `D` (§5.6)**

### Tests

- SE(3) composition/inverse
- quaternion normalization
- Lie exponential/log maps
- covariance propagation sanity
- serialization round trip
- deterministic replay
- `D(image, params, seed, frame_id)` is bit-identical across processes and machines

### Exit condition

A synthetic trajectory can be generated, serialized, replayed and scored exactly.

**Addendum:** the same bag plus the same seed produces **bit-identical metrics**. Record/replay and the degradation
operator are pulled this far forward deliberately — every later iteration's comparison claims rest on them, and
retrofitting determinism after the experiments exist is not possible.

---

## Iteration 1 — end-to-end "thin Ariadne"

Goal: the whole system runs before sophisticated algorithms exist.

### Simulator

- Gazebo world
- 2 agents
- stereo camera
- IMU
- direct kinematic motion

### Perception

- grayscale
- ORB/SIFT baseline
- LK tracker

### SLAM

- pose chain
- simple triangulation
- basic BA

### Collaboration

- exchange poses
- exchange descriptors
- one valid inter-agent loop closure

### Reconstruction

- point cloud
- simple Gaussian initialization
- offline splat render

### Planning

- frontier candidates
- nearest frontier action
- basic A* route

### Evaluation

- trajectory
- map
- bytes
- route

### Exit condition

Two agents independently explore a simple world, produce local maps, merge them, render a basic reconstruction and compute an exit route.

---

## Iteration 2 — proper OpenCV 5 perception

Replace placeholders with:

- XFeat
- stereo calibration
- StereoSGBM
- OpenCV 5 geometry
- ANNIndex
- LightGlue path
- proper quality masks

### Cross-system test

Run the same scene through:

```text
ORB pipeline
XFeat pipeline
```

and verify:

- runtime
- feature count
- relative pose
- bytes
- map quality.

---

## Iteration 3 — real local SLAM

Implement:

- keyframe policy
- landmark lifecycle
- Schur BA
- pose covariance
- local graph
- loop closure

### Oracle testing

For randomly generated pose graphs:

```text
Ariadne solver
vs
GTSAM/Ceres
```

Require agreement within numerical tolerances.

Do not proceed to multi-agent sophistication until this is stable.

---

## Iteration 3.5 — inertial layer

Placed here deliberately: after the BA is oracle-validated (so inertial factors are added to a *known-correct*
optimizer), and before robustness work (so gravity priors exist when the pose graph starts mattering).

### Implement

- IMU preintegration per §10.2, with bias Jacobians and covariance propagation
- stationary initialization (§10.3)
- inertial factors in the windowed BA (§10.2 — camera blocks only; Schur structure unchanged)
- `GRAVITY_PRIOR` unary edges (§10.4)
- frame-rate motion prediction feeding LK (§10.5)
- `imu_mode: attitude_only` fallback path (§10.7)

### Tests

- every inertial Jacobian validated by finite differences (§11.4 harness)
- preintegration against numerical integration of the same samples
- bias-Jacobian first-order correction verified against re-integration at the perturbed bias
- gravity direction recovered to within tolerance from a simulated stationary window

### Exit condition

EuRoC sequences (stereo + IMU — the natural first dataset for this iteration) land within the published
stereo-VIO error class, and gravity priors are active in the pose graph.

**If this iteration slips**, ship §10.7's `attitude_only` mode rather than descoping the inertial layer entirely.
The `GRAVITY_PRIOR` edge keeps a producer under every outcome, and the run manifest records which mode produced
each result.

---

## Iteration 4 — robust single-agent SLAM

Add:

- GNC
- DCS
- switchable constraints
- corridor degeneracy
- gravity prior
- bad measurement injection

### Regression suite

Inject:

```text
10%
25%
50%
75%
90%
```

false constraints into synthetic pose graphs.

Plot:

```text
success probability
ATE
runtime
```

This tests robustness before communication complicates the problem.

---

## Iteration 5 — multi-agent sparse collaboration

Add:

- global descriptor broadcast
- candidate retrieval
- local descriptor requests
- bookkeeping
- heartbeats
- store-and-forward
- inter-agent loop closures
- anchor propagation

### Network fault injection

```text
packet loss
latency
partitions
reordering
agent disconnection
```

Every run must terminate with:

```text
consistent or explicitly incomplete
```

never silent corruption.

---

## Iteration 6 — spectral communication optimization

Add:

- candidate Laplacian
- λ₂
- Fiedler vector
- budgeted selection
- exact small-case oracle
- greedy baseline

### Core plot

```text
ATE
 ^
 |       Ariadne
 |      /
 |     /
 |----/------ greedy
 |
 +--------------------> cumulative bytes
```

This is one of the most important figures in the entire project.

### Byte-table validation — a hard gate

Produce the §15.7 byte table as a **measured artifact**: predicted versus actual wire bytes per message class.

```text
prediction error > 15%  =>  the iteration fails
```

This is not bureaucratic. The spectral policy selects edges by `gain / predicted_cost` (§16.2), so a cost model
that is wrong by more than a modest margin means the flagship algorithm is optimizing a fiction — and the core
plot above would be measuring the wrong thing while looking entirely convincing.

---

## Iteration 7 — perceptual aliasing laboratory

Construct adversarial scenes:

- repeated walls,
- repeated beams,
- repeated windows,
- feature-poor concrete,
- identical-looking intersections.

Add:

- descriptor-space aliasing diagnostics,
- degeneracy detector,
- PCM,
- GNC.

Measure:

```text
candidate count
false closure rate
accepted false closure rate
map deformation
recovery latency
```

---

## Iteration 8 — active perception

Add:

- uncertainty volume
- frontier generation
- candidate view scoring
- travel cost
- information gain
- communication value
- agent bidding
- conflict resolution

Compare:

```text
random
coverage greedy
uncertainty-only
Ariadne
```

Core plot:

```text
reconstruction quality
vs
mission budget
```

---

## Iteration 9 — realistic darkness + co-moving illumination

Add simulator:

- camera-mounted point light
- inverse-square falloff
- saturation
- low-light shot noise
- exposure variation
- cast shadows

Add perception:

- flat-field correction
- exposure state
- variance stabilization
- denoise
- light normalization

Add reconstruction:

- NFL-aware photometric term.

Datasets to use for development/validation should prioritize OIVIO and other co-moving-light sources identified in the algorithm catalogue.

OIVIO:
- https://arpg.colorado.edu/oivio/

The OIVIO dataset contains dark-environment stereo + IMU sequences with calibrated onboard illumination and photometric/light calibration.

### Live-vs-replay exposure comparison

Quantify the approximation stated in §5.6: in replay, the exposure controller acts on the degradation operator's
parameters, which changes *sensing* but not *light transport*. Run the same scenario live and replayed, and report
the divergence in exposure trajectory and downstream tracking metrics.

The approximation is defensible; leaving it unmeasured is not. Report the number whichever way it comes out.

---

## Iteration 10 — dust and confined spaces

Add:

- particulate attenuation
- backscatter
- lens contamination
- narrow-void stereo
- corridor degeneracy.

Validate separately:

```text
clear
low dust
medium dust
high dust
```

and:

```text
wide space
corridor
void
```

---

## Iteration 11 — dense reconstruction becomes first-class

Implement:

- Gaussian initialization
- Gaussian optimization
- stable/unstable lifecycle
- submaps
- overlap fusion
- uncertainty
- held-out render evaluation.

Use a geometry-first rule:

```text
no validated pose -> no authoritative Gaussian
```

---

## Iteration 12 — risk-aware egress

Build:

- traversability field
- clearance field
- uncertainty field
- risk map
- route cost
- dynamic rerouting
- confidence.

Add:

```text
obstacle discovered after route generation
```

and verify that the thread reroutes.

### Required discipline and additions

- **All egress metrics are computed at `ρ_responder`** (§20.5), never at `ρ_agent`.
- Add the **route-confidence calibration plot** (§22.2): predicted confidence against realized validity across
  runs.
- The volumetric layer of §20 (TSDF → occupancy → ESDF → frontiers → traversability) must be in place first; it is
  the producer for every field listed above.

---

## Iteration 13 — agentic orchestration

Expose MCP tools.

Implement an orchestrator that receives:

```text
coverage
uncertainty
agent status
communication
candidate actions
```

and selects:

```text
move
observe
rendezvous
request peer data
return
human approval
```

Every decision is logged.

---

## Iteration 14 — COOL path

Port the per-frame image pipeline to the COOL Graviton node.

**Workload definition (C4).** The benchmarked workload is the **always-on per-frame pipeline** — undistort/rectify
(`remap`), pyramid construction, `calcOpticalFlowPyrLK`, image format conversion, flat-field multiply, VST
transform, DNN-input resize, and descriptor quantization/serialization. Restoration stages are benchmarked as an
*additional*, clearly labeled configuration.

This definition is deliberately independent of the restoration ablation's verdict (§6.2): the COOL claim reads
"COOL executes the per-frame image pipeline that runs on every frame from every agent", which stays true whether
or not restoration survives.

Benchmark:

```text
stock OpenCV 5 on Graviton
vs
COOL OpenCV on same Graviton
```

Measure:

- latency/frame,
- FPS,
- CPU utilization,
- cost/hour,
- cost/frame,
- multi-agent throughput.

The official COOL marketplace documentation recommends m8g.4xlarge or larger and documents custom C++ applications using its optimized OpenCV installation.

Source:
- https://aws.amazon.com/marketplace/pp/prodview-fdvbfiewzuehs

Also measure, once, **stock OpenCV 5 on Arm against OpenCV 4.x** (C3). Universal Intrinsics 2.0 moved the Arm
baseline substantially, and without that context a reader cannot distinguish a COOL optimization from a version
upgrade. Run it under the benchmark discipline of §27.1.

---

## Iteration 15 — full system synchronization

Run:

```text
3–5 agents
dark
dust
communication attenuation
false loop closures
agent failure
active perception
3DGS
egress
```

Collect complete run artifact:

```text
config
logs
trace
metrics
video
map
trajectory
communication accounting
failure events
```

---

## Iteration 16 — adversarial / failure campaign

Mandatory scenarios:

### Failure A

Agent disappears.

Expected:

```text
tasks reassigned
map remains valid
route remains available
```

### Failure B

False loop closure.

Expected:

```text
candidate rejected or GNC downweighted
```

### Failure C

Comms partition.

Expected:

```text
local autonomy continues
data queued
reconnection merges state
```

### Failure D

Tracking collapse.

Expected:

```text
confidence falls
agent chooses recovery action
```

### Failure E

Overexposure.

Expected:

```text
quality detector catches it
exposure/trajectory action changes
```

### Failure F

Dust spike.

Expected:

```text
visual confidence drops
affected observations downweighted
planner changes action
```

### Failure G

Unsafe route.

Expected:

```text
confidence falls
route withdrawn
alternate route or human approval
```

---

# 34. Dataset program

## 34.1 Primary geometry / SLAM validation

Use several independent datasets from the catalogue:

- TUM RGB-D
  > **As built (2.6, D100, 2026-10-22).** `TumSource` feeds the stereo pipeline. Sensor depth goes through a virtual
  > rig (u_R = u_L − fx·B/Z) in place of SGBM and the stereo match, with σ_Z = 1.425e-3·Z² (Khoshelham & Elberink
  > 2012) as the §9.4/§9.6 depth uncertainty. Pinned in `configs/benchmark/tum_regression.yaml`.
- EuRoC
- SubT / subterranean datasets where available
- ScanNet / Matterport-type indoor sets for reconstruction

Purpose:

```text
validate estimator independently from simulator
```

---

## 34.2 Co-moving-light validation

### OIVIO

Particularly valuable because it contains:

- stereo,
- IMU,
- onboard illumination,
- photometric calibration,
- light calibration,
- dark environments.

Source:
- https://arpg.colorado.edu/oivio/

> **As built (5.6, D141, 2026-10-23).** OIVIO could not be obtained (downloads withdrawn; Drive IDs 404). The
> approved substitute is **Hilti 2023 `site2_robot_3`** as the 5.6 sequence: an onboard co-moving light in a dark room
> (algorithms.md §8.1), OAK-D stereo + IMU, public sparse 3-DoF TLS ground truth, CC BY-NC-SA 3.0. There is no
> light-power sweep. OIVIO remains preferred if access is granted.

### VAROS

Use for synthetic moving illumination / scattering with dense ground truth where applicable.

### C3VDv2

Useful for near-field illumination physics and dense pose/depth supervision despite being an endoscopy domain.

The catalogue identifies its v2 form as substantially larger and notes detailed per-frame geometry.

Use it for **physics/algorithm validation**, not application-domain claims.

---

# 35. Benchmark protocol

Every algorithm comparison must use the same:

```text
scene
trajectory seed
agent start poses
image stream
communication trace
compute budget
mission time
```

Do not change data across baselines.

---

## 35.1 Main benchmark

For each strategy:

```text
independent
coverage greedy
similarity greedy
spectral Ariadne
```

sweep:

```text
B = [small, medium, large communication budget]
```

and report:

```text
quality(B)
ATE(B)
route_validity(B)
time_to_route(B)
```

---

# 36. Reproducibility requirements

A benchmark directory must be sufficient to reproduce a figure.

Example:

```text
evaluation/runs/fig_07/
    config.yaml
    seed.txt
    manifest.json
    command.sh
    raw_metrics.parquet
    processed_metrics.parquet
    plot.py
    README.md
```

Never manually edit a final plot.

---

# 37. Numerical testing requirements

## 37.1 Geometry

Test:

- SO(3) exp/log
- SE(3) exp/log
- adjoint
- covariance transform
- Jacobians
- triangulation
- PnP
- reprojection.

## 37.2 Optimization

Test with synthetic ground truth.

Require:

```text
residual decreases
gradient norm decreases
solution stable under small perturbations
```

## 37.3 Robust estimation

Randomly sample outlier rates.

Report:

```text
breakpoint
```

rather than only best-case performance.

## 37.4 Inertial

- every preintegration Jacobian by finite differences (§10.2)
- preintegrated `ΔR, Δv, Δp` against direct numerical integration of the same samples
- first-order bias correction against full re-integration at the perturbed bias
- gravity recovery from a simulated stationary window

## 37.5 Pose-graph numerics

- chordal initialization on graphs with known solutions; **assert `det(R) = +1`** after every SVD projection
  (§13.1 — the reflection case is the one that silently produces a plausible wrong answer)
- PCM consistency statistic against hand-computed values on a 4-node example
- GNC weight formula against the paper's closed form at several `(r, ε, μ)` points, including both saturation
  branches
- optimality certificate on a 5-node graph with a known global optimum, in both the certified and non-certified
  regimes (§14.8)

## 37.6 Volumetric and routing

- ESDF against a brute-force distance transform on small volumes (§20.3)
- frontier extraction on synthetic occupancy with known frontier sets
- D\* Lite replanning against a full A\* re-solve after each change — identical paths, fewer expansions (§22.1)
- the `(f, B, numDisparities, Z_min)` config validator rejects the known-bad combinations of §9.4

## 37.7 Protocol and accounting

- FlatBuffers round-trip for every message class
- `wire_bytes` predicted from §15.7 against measured, per class (the Iteration 6 gate)
- idempotency: duplicate and out-of-order delivery produce identical receiver state
- epoch rejection: a stale-epoch global estimate is discarded (§15.4)

---

# 38. Performance engineering

## 38.1 CPU hot path

Optimize:

- image conversion,
- resize/pyramid,
- LK flow,
- feature tensors,
- descriptor serialization.

### The COOL workload, defined (C4)

The benchmarked COOL workload is the **always-on per-frame pipeline** — the operations that run on every frame
from every agent regardless of any ablation outcome:

```text
undistort / rectify (remap)
pyramid construction
calcOpticalFlowPyrLK
image format conversion
flat-field multiply            (§6.2 Stage 1)
VST transform                  (§6.2 Stage 2)
resize for DNN input           (§7.4)
descriptor quantization / serialization   (§15.7)
```

**Restoration stages are benchmarked separately, as an additional and clearly labeled configuration.**

The reason for drawing the boundary here is strategic, not cosmetic. §6.2 commits to ablating restoration and to
accepting a negative result — the catalogue's low-light findings make that a live possibility. Anchoring the COOL
benchmark on restoration would leave the flagship deployment claim resting on a stage the project might itself
remove. As defined above, the claim reads:

> COOL executes the per-frame image pipeline that runs on every frame from every agent.

which holds under either verdict.

---

## 38.2 GPU hot path

Keep on GPU:

- Gaussian rasterization,
- batch mapping,
- large learned model inference,
- optional rendering.

Do not repeatedly move tensors CPU↔GPU.

---

## 38.3 Thread model

Per agent:

```text
sensor thread
tracking thread
mapping thread
loop-closure thread
planner thread
communication thread
```

Global:

```text
broker optimizer
reconstruction worker
operator telemetry
```

Use bounded queues.

Backpressure is explicit.

A slow Gaussian renderer must never stall camera tracking.

---

# 39. Recommended frequencies

Starting point:

```text
sensor:                20–30 Hz
tracking:              20–30 Hz
keyframe decisions:    5–10 Hz
global retrieval:      1–5 Hz / keyframe
loop verification:     event-driven
PGO:                   event-driven
active planning:       1–2 Hz
task consensus:        event-driven
egress replanning:     event-driven + 1 Hz heartbeat
operator telemetry:    2–10 Hz
Gaussian update:       asynchronous
```

Tune from profiling rather than hardcoding the final values.

---

# 40. Resource budgets

## Per agent conceptual budget

```text
image memory      bounded ring buffer
keyframes         bounded local window + archived compact metadata
landmarks         capped / culled
global descriptors compact
local descriptors on demand
```

## GPU

Target:

```text
< 8 GB/agent for normal reconstruction path
```

when practical.

Avoid architecture that requires tens of GB per agent.

---

# 41. Logging schema

Events:

```text
FRAME
KEYFRAME
TRACK_LOSS
LOOP_CANDIDATE
LOOP_REJECT
LOOP_ACCEPT
GNC_UPDATE
PGO
MESSAGE_SENT
MESSAGE_DROPPED
AGENT_DISCONNECT
AGENT_RECONNECT
TASK_CREATED
TASK_BID
TASK_ASSIGNED
ROUTE_CREATED
ROUTE_INVALIDATED
HUMAN_REQUEST
FAILURE
RECOVERY
```

Every event:

```text
timestamp
agent
run_id
sequence_id
state_hash
```

---

# 42. What should be considered "done"

## Core technical demo

Ariadne is demo-ready when all of the following happen in one deterministic run:

1. 3+ agents enter a dark collapsed-style world.
2. Agents use their own illumination.
3. Each agent builds a local metric visual map.
4. Agents discover cross-agent candidate relationships.
5. Communication is explicitly budgeted.
6. Candidate exchanges are selected by the communication policy.
7. At least one false closure is generated intentionally.
8. PCM/GNC prevents it from corrupting the global map.
9. Agents dynamically choose new observations.
10. A global reconstruction emerges.
11. A traversability/uncertainty-aware route is produced.
12. The route changes after new evidence.
13. At least one agent experiences communication loss.
14. It continues locally and reconciles after reconnection.
15. Operator UI displays the state.
16. All bytes and timing are measured.
17. A complete trace can be replayed offline.

---

# 43. Winning demonstration sequence

The final five-minute demo should be built from a **single coherent mission**.

### 0:00–0:25 — Problem

Show collapsed structure.

Text:

```text
GPS unavailable
radio degraded
light unavailable
humans cannot safely enter
```

### 0:25–1:05 — Fleet enters

Show:

- three agents,
- beams from co-mounted lights,
- distinct exploration branches.

### 1:05–1:45 — Collaboration

Visualize:

```text
descriptors
candidate edges
communication budget
spectral selection
```

### 1:45–2:20 — Failure

Intentionally insert false visual similarity.

Show:

```text
candidate
 -> geometry
 -> consistency
 -> GNC weight -> 0
```

Then display:

```text
map remains consistent
```

### 2:20–3:05 — Agentic active perception

Show:

```text
uncertainty hotspot
 -> agent reasoning
 -> task reassignment
 -> rendezvous
 -> new observation
 -> uncertainty reduction
```

### 3:05–3:45 — Reconstruction

Reveal:

```text
multi-agent Gaussian scene
```

with independent ground-truth comparison.

### 3:45–4:20 — Thread

Show risk-weighted path through reconstructed space.

Invalidate one segment.

Show automatic reroute.

### 4:20–4:45 — Quantitative evidence

Three plots:

```text
ATE vs bytes
coverage vs mission budget
route validity vs mission time
```

### 4:45–5:00 — COOL / AWS

One clean screen:

```text
same workload
same Graviton4
stock OpenCV 5 : X ms/frame
COOL             : Y ms/frame
cost/frame       : ...
```

---

# 44. Research claims Ariadne can responsibly make

Strong claims:

> Ariadne evaluates whether communication-aware active perception can improve collaborative 3D reconstruction under a fixed mission budget.

> Ariadne uses OpenCV 5 as a load-bearing computer-vision substrate across feature extraction, matching, geometry, stereo and point-cloud processing.

> Ariadne explicitly models false inter-agent loop closures and evaluates robust geometric containment.

> Ariadne treats communication as a quantified resource and reports reconstruction quality as a function of exchanged bytes.

> Ariadne produces an uncertainty-aware operational route through the recovered scene.

> Ariadne is a **vision-only** collaborative system — no LiDAR and no depth sensor, using stereo RGB, IMU and a
> carried light — addressing the low-cost degraded-conditions gap named by the SubT survey, with a monocular
> ablation quantifying what scale observability buys. (C5)

> Ariadne's egress routes are computed at an explicit human cross-section radius `ρ_responder`, not at the
> agent's own clearance. (§20.5)

> Ariadne reports whether its pose-graph solution is *certified* globally optimal for the rotation subproblem, and
> reports `not certified` when it is not. (§14.8)

Claims requiring a stated qualifier:

> **Structural risk is a proxy** — an overhang score from TSDF normals plus an unsupported-mass heuristic,
> calibrated against simulated GT zones. Label it a proxy every time it appears (§20.5). Ariadne does not perform
> structural engineering.

> **"Stereo-inertial" requires the inertial layer to be running.** The run manifest records `imu_mode` for every
> result (§10.7); the phrase must not appear in the report while the estimator is vision-only.

> **Replayed exposure control changes sensing, not light transport** (§5.6) — a stated approximation, quantified
> in Iteration 9.

Avoid claiming:

> real disaster deployment readiness

> certified structural safety

> survivor detection accuracy from proxy datasets

> universal robustness to dust

> globally optimal pose graph solution — except where §14.8's certificate actually returns `certified`

> bit-deterministic results from live closed-loop runs — those are reported over seeds (§5.9)

unless the experiments actually establish those properties.

---

# 45. Recommended final algorithm stack

## Perception

```text
OpenCV 5
    |
    +-- radiometric calibration
    +-- Poisson-Gaussian normalization
    +-- XFeat
    +-- ALIKED
    +-- LightGlue
    +-- calcOpticalFlowPyrLK
    +-- StereoSGBM + ximgproc DisparityWLSFilter
    +-- geometry
    +-- calib
    +-- ptcloud (HashTSDF)
    +-- active exposure + lamp co-control   (§6.5)
```

## SLAM

```text
keyframe SLAM
IMU preintegration (Forster) + gravity priors
local landmark map
Schur LM bundle adjustment (visual + inertial factors)
window exit -> ODOMETRY edges, no marginal prior
SE(3) pose graph
chordal rotation synchronization
GNC robust PGO
SE-Sync-style certificate
```

## Collaboration

```text
EigenPlaces
cv::ANNIndex
two-stage exchange
spectral λ₂ selection
vertex-cover exchange
PCM
GNC
elected broker
anchor propagation
store-and-forward
```

## Reconstruction

```text
stereo point clouds
albedo-proxy training images (falloff-normalized)
Gaussian submaps anchored to reference keyframes
stable/unstable Gaussian lifecycle
nearest-chunk pruning (default) / occupancy-aware fusion (measured alternative)
delayed per-keyframe affine exposure transform
near-field photometric correction
gsplat renderer (Python service)
```

## Active perception

```text
HashTSDF -> occupancy -> ESDF -> frontiers -> traversability
frontier extraction
uncertainty hotspots
EIG approximation
connectivity value
TARE-style global/local representation
FAR-style route graph
risk-aware utility
auction/consensus assignment
```

## Operational route

```text
traversability
clearance
structural-risk proxy
uncertainty
dynamic graph
D* Lite/FAR-style replanning
```

## Agentic layer

```text
MCP
tool invocation
task negotiation
exception handling
human approval
audit trace
```

## Deployment

```text
Gazebo Harmonic + ROS2 Jazzy
       |
       +-- GPU AWS node
       |
       +-- Graviton4 COOL OpenCV node
       |
       +-- orchestration
       |
       +-- S3 artifacts
       |
       +-- operator console
```

---

# 46. Priority ranking by technical payoff

### Tier A — absolutely central

1. OpenCV 5 perception stack
2. local metric SLAM
3. windowed BA
4. inter-agent visual registration
5. GNC
6. sparse communication protocol
7. byte accounting
8. active perception
9. global map consistency
10. egress route

### Tier B — major differentiators

11. spectral λ₂ communication prioritization
12. degeneracy-before-loop-closure policy
13. EigenPlaces descriptor/communication co-design
14. co-moving-light model
15. risk-aware task assignment
16. Gaussian submaps
17. failure supervisor
18. agentic MCP loop

### Tier C — depth multipliers

19. SE-Sync certificate
20. DCS / switchable constraints comparison
21. descriptor aliasing diagnostics
22. NFL-aware photometric reconstruction
23. stable/unstable Gaussian lifecycle
24. COOL cost-per-frame benchmark

### Tier D — stretch

25. thermal victim witness
26. event camera experiments
27. GLOW-style advanced photometric effects
28. feed-forward dense SLAM comparator
29. learned NBV
30. learned shape prediction

---

# 47. Final architecture rationale

The entire system should be thought of as **three coupled optimization problems** rather than one giant AI system:

### 1. State estimation

> What world configuration best explains the observations?

Solved by:

```text
stereo geometry
SLAM
BA
pose graph
GNC
```

### 2. Information acquisition

> Given finite resources, what observation should happen next?

Solved by:

```text
uncertainty
information gain
connectivity
risk
communication
multi-agent allocation
```

### 3. World-model delivery

> What information must cross the network to make the shared estimate better?

Solved by:

```text
retrieval
spectral edge selection
sparse feature transfer
pose graph synchronization
submap fusion
```

The egress layer is the operational consumer of all three.

That separation is the architecture's strongest property: **perception estimates; collaboration chooses what information is worth sharing; planning chooses what information is worth acquiring; reconstruction visualizes the validated result; the route consumes the resulting uncertainty.**

---

# 48. Primary sources

1. OpenCV 5 overview  
   https://github.com/opencv/opencv/wiki/OpenCV-5

2. OpenCV feature framework  
   https://docs.opencv.org/5.0/main_modules/features_main.html

3. OpenCV LightGlue  
   https://docs.opencv.org/doc/doxygen/html/d8/dc3/classcv_1_1LightGlueMatcher.html

4. OpenCV ANNIndex  
   https://docs.opencv.org/5.0/main_modules/features_annoy.html

5. OpenCV point-cloud / volumetric reconstruction  
   https://docs.opencv.org/5.0/main_modules/ptcloud.html

6. XFeat  
   https://arxiv.org/abs/2404.19174

7. LightGlue  
   https://arxiv.org/abs/2306.13643

8. EigenPlaces  
   https://arxiv.org/abs/2308.10832

9. Swarm-SLAM  
   https://arxiv.org/abs/2301.06230

10. GNC  
    https://arxiv.org/abs/1909.08605

11. Photo-SLAM  
    https://arxiv.org/abs/2311.16728

12. RTG-SLAM  
    https://arxiv.org/abs/2404.19706

13. MASt3R-SLAM  
    https://arxiv.org/abs/2412.12392

14. VGGT-SLAM 2.0  
    https://arxiv.org/abs/2601.19887

15. EC3R-SLAM  
    https://arxiv.org/abs/2510.02080

16. MAGS-SLAM  
    https://arxiv.org/abs/2605.10760

17. NFL-BA  
    https://arxiv.org/abs/2412.13176

18. GLOW  
    https://arxiv.org/abs/2511.22857

19. Burst Imaging for Light-Constrained SfM  
    https://arxiv.org/abs/2108.09895

20. OIVIO  
    https://arpg.colorado.edu/oivio/

21. TARE  
    https://www.cmu-exploration.com/tare-planner

22. FAR Planner  
    https://arxiv.org/abs/2110.09460

23. CBBA  
    https://pure.kaist.ac.kr/en/publications/consensus-based-decentralized-auctions-for-robust-task-allocation/

24. Multi-Agent Risk-Aware NBV, 2026  
    https://arxiv.org/abs/2606.04158

25. GenNBV  
    https://openaccess.thecvf.com/content/CVPR2024/html/Chen_GenNBV_Generalizable_Next-Best-View_Policy_for_Active_3D_Reconstruction_CVPR_2024_paper.html

26. Active Neural Mapping at Scale  
    https://arxiv.org/abs/2409.20276

27. D* Lite  
    https://aaai.org/papers/00476-aaai02-072-d-lite/

28. ROS 2 Jazzy / Gazebo Harmonic  
    https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Gazebo/Simulation-Gazebo.html

29. AWS Graviton4 M8g  
    https://aws.amazon.com/about-aws/whats-new/2024/09/amazon-ec2-c8g-m8g-instances/

30. AWS COOL for Graviton4  
    https://aws.amazon.com/marketplace/pp/prodview-fdvbfiewzuehs

31. gsplat  
    https://github.com/nerfstudio-project/gsplat

### Verified live during the v2 revision `[VERIFIED web 2026-08-30]`

32. OpenCV 5.0 release facts  
    https://docs.opencv.org/5.0/ ·
    https://github.com/opencv/opencv/wiki/OpenCV-5 ·
    https://github.com/opencv/opencv/wiki/OpenCV-4-to-5-migration ·
    https://opencv.org/opencv-5/

33. ROS 2 Jazzy ↔ Gazebo Harmonic pairing — gazebosim.org ROS/Gazebo vendor-package and installation docs

34. Gazebo headless EGL rendering  
    https://gazebosim.org/api/sim/9/headless_rendering.html

35. Gazebo RFComms (SubT-ported log-distance path loss)  
    https://gazebosim.org/api/sim/9/classgz_1_1sim_1_1systems_1_1RFComms.html

36. gz-sim `light_config` service — gz-sim light_config tutorial

### Papers newly load-bearing in v2

37. Forster, Carlone, Dellaert, Scaramuzza — On-Manifold IMU Preintegration, T-RO 2017  
    https://arxiv.org/abs/1512.02363

38. Foi, Trimeche, Katkovnik, Egiazarian — Poissonian-Gaussian noise modeling and fitting, TIP 2008

39. Mäkitalo & Foi — Exact unbiased inverse of the generalized Anscombe transform, TIP 2013

40. Shim, Lee, Kweon — Auto-adjusting camera exposure using gradient information, IROS 2014

41. Zhang, Forster, Scaramuzza — Active exposure control for robust VO in HDR environments, ICRA 2017

42. Mangelson, Dominic, Eustice, Vasudevan — Pairwise Consistent Measurement Set Maximization (PCM), ICRA 2018

43. Yang, Antonante, Tzoumas, Carlone — Graduated Non-Convexity, RA-L 2020  
    https://arxiv.org/abs/1909.08605

44. Carlone, Tron, Daniilidis, Dellaert — Initialization techniques for 3D pose graph optimization, ICRA 2015  
    (with Martinec & Pajdla, CVPR 2007, for the linear rotation synchronization itself)

45. Rosen, Carlone, Bandeira, Leonard — SE-Sync, IJRR 2019  
    https://arxiv.org/abs/1612.07386

46. Eriksson, Olsson, Kahl, Chin — Rotation Averaging and Strong Duality, CVPR 2018  
    (with Carlone, Rosen et al. — Lagrangian duality in 3D SLAM, IROS 2015)

47. Felzenszwalb & Huttenlocher — Distance Transforms of Sampled Functions, Theory of Computing 2012

48. Koenig & Likhachev — D* Lite, AAAI 2002

49. Choi, Brunet, How — Consensus-Based Decentralized Auctions (CBBA), T-RO 2009

50. Umeyama — Least-squares estimation of transformation parameters, TPAMI 1991

51. Kerbl et al. — Hierarchical 3D Gaussians, SIGGRAPH 2024  
    https://arxiv.org/abs/2406.12080

52. Lajoie & Beltrame — Swarm-SLAM, RA-L 2024  
    https://arxiv.org/abs/2301.06230

---

# 49. Source-of-truth relationship to `algorithms.md`

This implementation document should be treated as the execution layer above `algorithms.md`.

The catalogue remains the detailed algorithm research record.

This document makes the architecture-level decisions:

```text
algorithms.md
    |
    +-- literature evidence
    +-- benchmark evidence
    +-- failure evidence
    +-- implementation difficulty
    |
    v
IMPLEMENTATION.md
    |
    +-- chosen algorithms
    +-- interfaces
    +-- data contracts
    +-- test contracts
    +-- integration order
    +-- deployment topology
```

When an algorithm changes, update:

1. the catalogue entry,
2. this implementation decision,
3. the ablation matrix,
4. the benchmark configuration,
5. the run manifest schema.

Never silently replace an algorithm in runtime code without recording which paper/mechanism was substituted and why.
