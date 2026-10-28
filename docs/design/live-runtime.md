# Live closed-loop runtime (D150–D155)

Status: implemented 2026-10-23; acceptance on the local llvmpipe host, evidence `evaluation/runs/live-agentic/live_loop/`;
the AWS GPU path is built, not run. Amendments from the runs: D152a (estimators take turns), D153b (standoff goals,
the previous outcome is an input), D154 fix (console directory).

Register row L1. Arch §5.4 (bridge, topics, sim clock), §5.8 (agent motion),
§5.9 (record/replay; the live exemption), §5.10 (live topology), §25 (the choose step), §29 (console).

## 1. What it is

Today every Ariadne result comes from a recording: Gazebo renders a scripted trajectory once, and the estimator,
the mission layer and the console consume the bag afterwards. The mission's decisions are logged but never executed
(`trace.jsonl`: `mode replay, executed false`). L1 closes the loop: a decision moves the agent, the agent's cameras
see the consequence, and the estimator's next output feeds the next decision.

```text
gz sim (GPU EGL)                   ── one sim container ──────────────────────────────┐
  cameras / IMU / GT pose ──gz-transport──> RosGzBridge ─┐ intra-process (one         │
  KinematicMover <──gz-transport── Pose_V ──── RosGzBridge <┘ component container)      │
                                               │     ^                                  │
                                               v     │ PoseArray (goto/follow/hold)     │
                                         ariadne_bridge::RuntimeLink (typed subs ->     │
                                         CDR bytes; CDR bytes -> typed pubs)            │
                                               │     ^                                  │
──────────────────────────────── Unix socket: framed (topic, CDR bytes) ───────────────┘
                                               v     │
ariadne_live (core image, ROS-free) ── reorder by stamp ── consumed-stream MCAP (the recording)
   per agent: LiveSimSource -> RunStereoVo (the replay estimator path, unchanged)
   mission: keyframe depth -> occupancy -> candidates -> chooser -> command   ──> trace.jsonl
   telemetry.jsonl + map snapshots ──> operator console (live mode, WebSocket)
```

## 2. Decisions

**D150 — where ROS stops.** ROS 2 exists only in the sim container: `ros_gz_bridge` (the 1.1 bridge YAML, plus
the command and mover-state entries) and one relay component, `ariadne_bridge::RuntimeLink`. The relay subscribes
to the bridged topics with their real types and serializes each message to CDR with `rclcpp::Serialization`. It
writes `(topic, CDR bytes)` frames to a Unix socket. The Ariadne runtime decodes them with its own ROS-free decoder
(`runtime/replay/ros_cdr`, D38): the same decoder and the same frame assembly the bag replay uses. No ROS type
enters `cpp/` (arch §3). The sim image ships no OpenCV and the core image ships no ROS; neither changes.

- *Why not run the estimator inside a ROS node:* that needs OpenCV 5 in the ROS image (a third image, and an
  ambiguous `opencv_build_hash`) and moves ROS types next to the estimator.
- *Why not read gz-transport directly, as the recorder does (D40):* the brief says ros_gz_bridge, and live runs are
  where the full ROS graph belongs (arch §5.10). D40's loss finding applies to the DDS hop, so that hop is removed:
  `ros_gz_bridge::RosGzBridge` is a registered rclcpp component (verified in the installed 1.0.24 package) and
  RuntimeLink is loaded into the same component container with `use_intra_process_comms`. Every input topic still
  counts its stamp-grid gaps, so a loss is measured, not assumed away.
- *Precedent:* domain_bridge and rosbag2 relay serialized messages the same way.
- **ros2/ packages.** `ariadne_bridge` (RuntimeLink, the launch file composing it with RosGzBridge) and
  `ariadne_sim` (the live world launch, render-mode selection). No `ariadne_interfaces`: commands and mover state
  ride on standard types that ros_gz_bridge maps (`geometry_msgs/PoseArray` ↔ `gz.msgs.Pose_V`,
  `std_msgs/String` ↔ `gz.msgs.StringMsg`; header stamp and frame_id verified to pass through, 2026-10-22). No
  `ariadne_agent` / `ariadne_operator`: the agent runtime and the console are not ROS nodes by D150.

**D151 — the command interface (arch §5.8).** One topic per agent, `/ariadne/agent_<k>/cmd`, PoseArray in the world
frame (`frame_id: world`): 0 poses = `hold()`, 1 = `goto(pose)`, N = `follow(path)`. The header stamp is the sim
time of the decision: `(agent, stamp)` is the command id that links a trace record to the motion it caused. The
mover answers on `/ariadne/agent_<k>/mover_state` (String, JSON): accepted or refused (with the reason), and the
segments it scheduled.

- **Motion = appended script segments.** The mover keeps its D38 script and appends turn/move segments from the
  state where the new motion begins: turn to face each waypoint, move straight, then turn to the commanded final
  yaw. Profiles are the D38 family (`plan_profile`: `v_cap`, `a_max`, `omega_max`, `alpha_max`, with the
  image-flow speed cap of `routes.py`), so a commanded path is as trackable as a scripted one. One
  dependency-free planner (`cpp/core/trajectory/command_planner.hpp`) is used by the plugin and tested in the core
  suite.
- **Preemption.** A new command replaces every segment not yet started. The segment in progress finishes, so the
  rig never stops mid-ramp (all segments end at rest; no velocity discontinuity).
- **Safety net.** Before accepting, the mover samples the new segments every 10 ms against the scene ESDF at
  `rho_agent`, as at start-up (D38). A violation refuses the whole command and the agent holds. The planner plans on
  the *estimated* map; the GT check is the bumper, and every refusal is in the trace.
- **The executed script is logged** (every accepted segment, exact doubles) so the canonical IMU/GT of a live run
  can be regenerated offline (D39's tool, same `SampleAt`).

**D152 — the recording is what was consumed.** `ariadne_live` reorders the incoming frames by header stamp behind
a watermark: a message is released when every input topic has delivered a stamp ≥ it, or after a hold-back
timeout, which is logged. Each released message is appended to `inputs.mcap` (rosbag2 MCAP layout, so
`RosbagReader` reads it) with log time = release sequence number, strictly increasing. Replay
(`ariadne_live --replay inputs.mcap`) feeds that order verbatim through the same code. The first replay check is a
digest over the consumed sequence (topic, stamp, sha256 of the payload), before any output is compared.

- **"Reproduces" is defined before the run.** For each agent: sha256-identical `estimate_body.tum`, identical
  keyframe count. Mission: identical `trace.jsonl` once wall-clock fields are stripped. Replay runs twice (two VO
  threads share OpenCV's thread pool; the second run shows the replay itself is deterministic).
- **Live-only tolerance.** A frame whose stereo partner never arrives is dropped and counted (`unpaired_frames`),
  never thrown (SimSource's strict rule stays for bags). A dropped frame enters neither the estimator nor the
  recording.

**D153 — the mission layer runs incrementally, on a sim-time barrier.** The 1.8 logic (OctoMap-model grid, slab,
frontiers, A* at both ρ) moves out of `ariadne_mission.cpp` into `cpp/planning/mission/frontier_mission.hpp`.
The batch tool calls it, with output unchanged, and the live runtime calls it per keyframe and per replan. A replan
at sim time T runs only after BOTH agents' estimators have passed T, so it sees the same keyframes live and in
replay.

- **Anchoring, live.** Each agent is anchored by its OWN briefed entry pose (the `--agents k` rows of 1.8). The
  merged anchor through the 1.6 loop is not used live. Phase 3's map merge will replace this.
- **Candidates + chooser (§25.1, built now).** The deterministic layer enumerates, per agent, the frontier
  clusters reachable at `rho_agent`, each with its path, and the safety envelope (`hold` is always a candidate). A
  chooser picks one: `classical` = the nearest (the 1.8 policy); `llm` (7.1) = the orchestrator's pick, validated
  against the set (out-of-set → classical, logged). 7.2 is the config switch `chooser: classical`. Replay re-applies
  the RECORDED choice from the trace and never calls a model.
- **Phase 4 replaces** `frontier_mission.hpp`'s candidate generator (NBV, allocation). The chooser interface and
  the command path stay.
- **The executed command.** `go` → `follow` along the A* path to the cluster's nearest reachable cell, simplified
  with `cv::approxPolyDP` (ε = 1 cell, each chord re-checked on the traversable grid), truncated to
  `max_command_m`, ending with a yaw that faces the cluster centroid (observe what was approached). Trace records
  carry `mode: live, executed: true` and the command id.

**D154 — console fed live.** `ariadne_live` appends `telemetry.jsonl` (5 Hz in sim time: estimated poses in W, the
latest decision per agent, mover state, byte counters) and writes the map snapshot at each replan. The console
server gains a live mode that tails the run directory and pushes the same frame schema over its WebSocket. It logs
every frame it served (wall time, sim time) as the evidence that it was fed during the run.

**D155 — where live runs execute.** The host has no GPU. Live runs go to one g4dn.xlarge spot instance (the D84
quota: 4 vCPU, T4) through `infra/scripts/spinup.sh` (three brakes, D27). Images are pushed to ECR by digest and
pulled. Binaries built locally in `ariadne/core-dev` ship as a payload with SHA256SUMS (the L5 pattern). The NVIDIA
EGL path of `ariadne/sim` has never run, so it is a `[SMOKE-TEST]` gate first, recorded in validation.md.
CPU is the budget, not GPU: the GT depth cameras are off in the live world (the estimator never sees them), and the
world's real-time factor is set from the smoke's measurement.

**D165 — lockstep (2026-10-23; chosen for the 4.3 gate).** Unpaused, gz keeps running while
the runtime plans and chooses, so a command lands ~1.7 s of sim time after its barrier (measured `applied_ns −
stamp`) and that lag depends on wall latency: two live runs of one seed differ. `lockstep: true` in the live config:
- **gz does the pausing, sim-side.** `[SMOKE-TEST §4w: the bridge forwards run_to_sim_time; /clock stops at T exactly]` The runtime asks for `WorldControl.run_to_sim_time = T` (gz-msgs10 field 7; gz-sim8
  `SimulationRunner::SetRunToSimTime`): gz runs to T and pauses itself, independent of how far behind the runtime's
  processing is. The call is the `/world/<w>/control` service, bridged by `ros_gz_bridge` 1.0.24
  (`ros_gz_interfaces/srv/ControlWorld` ↔ `gz.msgs.WorldControl`/`gz.msgs.Boolean`, one service entry in the bridge
  YAML), called by RuntimeLink when the runtime sends a `#run_to` control frame (int64 ns). No ROS or gz type in the
  core: the frame carries a number. RuntimeLink answers `#run_to_ack` with the sim clock at the moment of the request
  (the paused time; `/clock`, subscribed locally, not forwarded) and the service's verdict.
- **The barrier needs a frame at T.** `[SMOKE-TEST §4w: paused at T, the frame at T is delivered, none after; a world starting paused has no t = 0 frame]` Measured on the first LLM run: the cameras tick every 50 ms exactly, and a frame
  is stamped at every 5 s barrier (1801 frames, all 18 barrier stamps present), so a world paused at T has delivered
  everything the barrier waits for.
- **Zero lag, not "one step".** `[TESTED: test_mover_live.py::test_command_taken_while_paused, applied_ns = the paused time]` The mover now takes queued commands in `PreUpdate` even while paused (gz-sim calls
  systems while paused; motion still advances only when unpaused), so a command sent at barrier T is applied at sim
  time T and answered. The runtime waits for both answers BEFORE asking for `run_to T + replan`: every command's
  `applied_ns` equals its barrier in lockstep. Unpaused runs are unchanged (the next update applies it, as before).
- **Logged per barrier** in `lockstep.jsonl`: T, `paused_sim_ns` (clock when the next run_to was requested),
  `applied_sim_ns` per agent, `resume_to_ns`, the wall seconds the world spent paused. The figure states the lag from
  these, never assumes it.
- **Precondition (else the barrier deadlocks):** every timed topic must publish a message stamped exactly T. The
  reorderer releases the frame at T only when every timed topic has reached T, and a paused world delivers nothing
  more. Holds today: cameras 20 Hz, IMU and GT 200 Hz (the mover publishes both when `ns % period == 0`), and both
  periods divide the 5 s replan (measured: a frame and a GT sample at every barrier of the first LLM run). A sensor
  whose period does not divide `replan_s` breaks lockstep.
- **Result (2026-10-23, `evaluation/runs/live-agentic/live_loop/lockstep_ci_tiny_20s`):** two live lockstep runs of ci_tiny seed 0
  (20 s, classical, llvmpipe) are byte-identical: consumed stream, estimates, trace, mover answers; every command
  applied at its barrier, gz paused at T exactly (0 ns), pauses ≤ 0.1 s wall. Two defects the check found on the
  way: the end-of-run holds raced the last resume (now sent paused, stamp end + 1 ns; `end_s` must be a multiple of
  the replan period in lockstep), and the CDR alignment padding was uninitialised memory (RuntimeLink zeroes its
  buffers). Scope stated there: one renderer, one scene, classical chooser.
- live.sh does not unpause a lockstep run: the runtime's first `run_to` (after `#ready`) starts it. Replay is
  unchanged: it consumes the recording, which is the same in either mode.

## 3. Acceptance (register L1)

1. One live two-agent run: decisions are executed (`executed: true`), each command id appears in the mover's state
   as accepted (or refused, with the reason), and the agent's estimated motion follows it: decide → move → observe.
2. Its `inputs.mcap`, replayed twice, reproduces §D152's outputs exactly.
3. The console served frames during the run (its frame log).
4. Bag replay (`SimSource`, the Phase 1 chain) is unchanged: targeted tests, plus the 1.8 tool's output on the
   committed spool, byte-identical before and after the extraction.

---

<a id="multi-agent-pipeline"></a>

# Multi-agent pipeline

**Scope:** overall design, comms classes, live collab fleet; per-agent planning, coverage summaries, allocation
over messages; the live runtime wiring and the console.
**Status:** design. No new algorithms. This wires what exists so the thesis holds in the live system.

### 0. The gap this closes

Thesis: agents coordinate through limited communication to map the whole space with minimal exchange. Each
subsystem exists and is tested, but the live loop is centralised end to end:
- **One planner sees every agent's keyframes.** One shared TSDF/ESDF; the peers list uses every agent's full track.
- **Allocation is computed in one process.** CBBA runs inside `Phase4Planner::Allocate`. It counts the bytes its
  messages would cost, but never sends them.
- **Alignment is the briefing's constant, with `linked = true`.** No SLAM collaboration runs live.

The architecture already specifies the decentralised design, so this document implements it:
- **§21.7:** each agent scores its own candidates; bids; consensus resolves duplicate ownership; event-triggered;
  consensus messages ride class 2 of §15.3. Broker-connected components may shortcut to a sequential greedy auction
  at the broker, logged identically.
- **§15.3:** a per-peer outbox, strict priorities, and `SUBMAP_SUMMARY` / `SUBMAP_PATCH` (map sharing, class 5).
- **wire.fbs:** classes 7–10 (`SUBMAP_SUMMARY`, `SUBMAP_PATCH`, `TASK_BID`, `TASK_CONSENSUS`) are reserved in the
  enum but have no tables.

### 1. The rule: an agent knows only what it observed or received

Per agent k, onboard (in the sim, k's worker plus a planner instance owned by k):

```text
own sensors ──> VO + CollabFrontend (worker k) ──> own keyframes
                                                     │
              ┌──────────────────────────────────────┼─────────────────────────────┐
              v                                      v                             v
   planner_k.Integrate(own kf, at T_G_W(k))   CollabNode k (SLAM collab,   SUBMAP_SUMMARY out (own
                                              live-collaboration.md, D174)           coverage, 0.5 m, §20.6)
   planner_k.IntegrateSummary(peer j sum,    T_G_W(j) for every j it      TASK_BID out to k's broker
              at T_G_W(j) as k knows it)      has an estimate of
   peers_k = heartbeat poses of the agents k hears
              │
              v
   planner_k.Plan(t, own + received)  ──>  own options  ──>  TASK_BID  ──>  broker of k's component
                                                                             │ sequential greedy (SGA)
   planner_k executes its assignment  <──  TASK_CONSENSUS  <─────────────────┘
   (no assignment within timeout: its own best option, logged "uncoordinated")
```

Nothing reaches planner_k except through CommsNode, the same lossy, budgeted, partitionable link the SLAM
collaboration uses. Every byte is counted at the serializer and tagged by class (invariant 6).

**Peer alignment.** Entry poses are known (2026-10-23: drones inserted through windows), so every agent
holds an entry-pose T_G_W(j) for each peer from t = 0, and peer summaries are placeable before any SLAM collaboration.
Once live-collaboration.md (D174) runs, adopted broker estimates refine those values. The POSE_GRAPH estimate already carries
every member's frame (`agent_frames`); CollabNode is changed to keep the peers' values as well as its own, with no
wire change.

**The operator's map.** The operator station is a comms node at the building entrance. It has no sensors and no vote
in allocation, and receives the agents' coverage summaries over the same link model. The console's top-view map and splat
therefore show what the operator has RECEIVED, and agents out of radio range stop updating it, which is the honest
behaviour. A separate view, labelled "ground truth / evaluation", shows the GT coverage for comparison and is never
presented as the system's map.

### 2. Messages (the tables, codec, ledger classes and CommsNode routing)

| Class | Content | Cadence | Priority (§15.3) |
|---|---|---|---|
| `HEARTBEAT` (exists) | pose summary in G (already in §15.2), plus the keyframe watermark | 1 s | 1 |
| `SUBMAP_SUMMARY` (7) | arch §20.6: the sender's OWN coverage on the 0.5 m global coarse layer (occupancy / uncertainty states), the cells changed since the last summary each receiver acked, run-length coded + zstd, in the sender's frame W_j, tagged with frame and epoch. Never raw maps (§15.1). `SUBMAP_PATCH` (8) stays reserved | event: ≥ N new coarse cells or ≥ 5 s | 5 (the first class cancelled under budget, §20.6) |
| `TASK_BID` (9) | the sender's top-M options: goal (x, y) in G, task key, utility terms on the common scales | per replan (5 s) or on event | 2 |
| `TASK_CONSENSUS` (10) | the broker's assignment for its component: per member, a task key and goal, plus the component member set and epoch | after each SGA round | 2 |

- **Task identity across agents.** Tasks are frontier clusters, and each agent's clusters come from its own map, so
  ids differ between agents. The broker treats two bids whose goals lie within `r_same` as the same task when it
  enforces exactly-once. `r_same` = the planner's `min_goal_m` (1.0 m in live_v1): goals closer than that are already
  "the same place" to the planner. This approximates CBBA's shared task list, and the deviation is stated.
- **Byte budget (revised with the planner-side measurement, 2026-10-23).** Measured on the D168 c7i run (ci_tiny_0 × ariadne,
  200 s, changed blocks only, 2-bit packed plus a 4 B header per block, zlib -9 standing in for zstd):
  - per 5 s patch: median 238 B, p90 463 B, max 508 B;
  - per agent over the run: about 53 B/s; bids about 64 B/s.

  Coordination therefore costs about 0.1–0.2 KB/s per agent, against the SLAM collaboration's tens of KB/s. A
  proportional share (the earlier s_x = 0.3) would be meaningless at that ratio, so instead:
  - classes 8–10 get their own token bucket of 1 KB/s per agent (≥ 5× the measured p90 rate; burst = one full-map
    patch, ~16 KB packed), outside the SLAM budget the spectral policy manages;
  - they are counted and reported per class like everything else;
  - under that bucket SUBMAP_PATCH is cancellable, oldest first; bids and consensus are not.

  Caveats: the ρ_responder bit and maze-sized scenes are not yet measured (expected < 2× and 2–3×). The budget sweep
  (§4) varies only the SLAM share.

### 3. Allocation over messages: broker SGA first, full CBBA as the ablation

Broker SGA per connected component (§21.7's sanctioned shortcut):
- Members send `TASK_BID`.
- The broker runs `JointPick`'s exactly-once greedy over the bids it received, with spatial dedup.
- It returns `TASK_CONSENSUS`.
- Under a partition, each component's broker allocates its own members. Duplicates across components are possible
  and are the honest cost of the partition; they are counted.
- `cbba_distributed` (each agent runs its own CBBA state over delivered TASK_CONSENSUS messages) is the second
  allocation row. It needs the same messages plus per-agent CBBA state; built only if time allows.
- The current centralised Phase4Planner stays as the **oracle row**: full information, zero bytes. It is the upper
  bound the comms-limited system is compared against.

### 4. Experiment rows (the thesis evidence)

Same scene, seeds and starts. Lockstep, so the comparison is deterministic.

**Pre-registered required set** (written before any data):
- oracle, independent and ariadne, at one budget level;
- the level is D171's derived 25 %, applied to the total, with the share split above;
- 2 scenes × 3 seeds = 18 jobs, about 2 h on two lanes;
- the partition row and the budget sweep are extras, run only if time allows.

**Pre-registered pass criterion:** on time-to-90 % coverage (mean over the 6 runs), ariadne closes at least 50 % of
the gap between independent and oracle. Also reported: bytes, redundancy and duplicate tasks. Failing the criterion
is reported, not tuned away.

| Row | Map shared | Allocation | What it shows |
|---|---|---|---|
| oracle | everything (centralised) | centralised CBBA | upper bound |
| independent | nothing | each agent its own best | lower bound: no communication |
| **ariadne** | `SUBMAP_SUMMARY` (§20.6) under budget | broker SGA over messages | the thesis |
| ariadne, partition | as ariadne, `collapsed_partition_v1` | per-component brokers | graceful degradation |
| ariadne, budget sweep | budget at {10, 25, 50, 100} % | as ariadne | coverage vs bytes |

Metrics, all computed by the existing mission eval:
- coverage of the GT free space vs time;
- time to 90 % coverage;
- redundant observation ratio (cells seen by ≥ 2 agents);
- duplicate-task count;
- total bytes by class;
- merged ATE (SLAM collab).

The claim is made only if "ariadne" sits between independent and oracle, closer to the oracle, at a stated byte cost.
Otherwise the result is reported as measured (invariant 7).

### 5. Determinism and replay

- **Sim clock everywhere.** Patch and bid cadences are in sim time, and CommsNode delivery is sim-timed and seeded.
- **Lockstep barrier.** At the 5 s barrier, every agent's planner runs after the comms tick for T, so what each one
  received is a function of the seed. The live-collaboration.md §3 watermark rules apply to keyframe and feature data.
- **Acceptance.** Three replays equal the live run; the `collab: false` digest is unchanged (D168).

### 6. Work items and order (final submission, Oct 26)

| # | Area | What | Depends on |
|---|---|---|---|
| 1 | comms | wire tables and codec for classes 7–10, CommsNode routing to per-agent mailboxes, delivery events, ledger tags; protocol tests | — |
| 2 | comms | live-collaboration.md fleet (D174): CollabFrontend in the worker, CollabFleet in the parent, T_G_W per agent | — |
| 3 | planning | per-agent `Phase4Planner` instances, `ExportSummary` / `IntegrateSummary` on the §20.6 0.5 m coarse layer (planner_k plans on own fine map + peers' coarse coverage), the task key, `TASK_BID` building, the broker SGA over received bids, the uncoordinated fallback; the identity check that the oracle row is unchanged | 1 (message structs) |
| 4 | runtime | ariadne_live: one planner per agent fed only through its mailbox; heartbeat poses as peers; the operator node and its received map; the console shows bytes by class, assignments and "uncoordinated" | 1, 3 (NOT 2: entry poses place the summaries) |
| 5 | evaluation | §4 required rows in lockstep on AWS (CPU), then evidence and plots | 1, 3, 4 |

Item 2 (live-collaboration.md) runs in parallel. It adds drift correction and the SLAM-collaboration panel, and it is not on
the thesis rows' critical path.

**Compute.** N planners at ~3–6 s CPU per 5 s replan is ~0.6–1.2 core per agent, on top of live-collaboration.md §6. With
3 agents, the judge box carries:

| Component | Cores |
|---|---|
| gz | ~2–4 |
| estimators | ~3 |
| collab frontends | ~2.5 |
| planners | ~3 |
| **total** | **~11–13** |

That fits one g4dn.4xlarge (16 vCPU, the quota being requested).

Fallback order if the time runs out: rows oracle / independent / ariadne first, then partition, then the budget sweep.
`cbba_distributed` is last.

### 7. §49

- **catalogue:** no new algorithm (CBBA / SGA per arch §21.7; coarse coverage summaries per §20.6 and §15.3).
- **arch:** an as-built note for §21.7 and §15.3: classes 7–10 implemented, the task key deviation.
- **ablation:** the §4 rows.
- **benchmark config:** `configs/mission/live_multiagent_v1.yaml`.
- **manifest:** row, budget, scenario, per-class bytes, task-key radius.

### 8. Planner-side decisions (2026-10-23)

1. **Received layer.** planner_k's TSDF is built from its OWN keyframes only. Each peer's coverage summary lands in a
   separate per-peer received layer: tri-state, on the §20.6 0.5 m grid. That layer:
   - suppresses frontier clusters and candidates whose target lies in peer-known space;
   - zeroes the IG of voxels whose column a peer already knows.

   Paths and risk use only the agent's own map: no routing through space only a peer saw (the conservative safety
   choice).
2. **Egress (invariant 4).** Each summary cell also carries the sender's ρ_responder traversable/support bit,
   computed by the sender from its own TSDF: 2 bits per cell. The operator node runs D* Lite egress on its received
   layers, so the §22 claim holds at the operator.
3. **Coordination lag (pre-registered as part of what the ariadne row measures).** At barrier T each agent executes
   the TASK_CONSENSUS it RECEIVED by T, which was computed from the T−5 s bids.
   - It takes the assigned goal if that task key is still present among its own options.
   - Otherwise, or with nothing received within `consensus_timeout_s`, it takes its own best option, logged
     "uncoordinated".
   - CommsNode's latency and capacity apply. Zero latency is never assumed.
4. **Broker identity per barrier:** `ComponentOf(agent) -> {broker, members, epoch}` from the comms layer (§15.4: the
   lowest id in the component, with an epoch). TASK_CONSENSUS from a stale epoch is ignored.
5. **Task key:** spatial only. Frontier ids are per-agent hashes. The broker dedups bids greedily by utility; a bid
   within `r_same = min_goal_m` of an already-assigned goal counts as the same task.

**Planning-library interfaces (no comms types; the wire tables mirror these structs):**
- `PlanAgent(t, own inputs, received layers, peer poses)` → options
- `ExportSummary(since_version)` → changed coarse cells
- `IntegrateSummary(sender, cells, T_G_W(sender))`
- `BuildBid(options, M)` → TASK_BID payload
- `BrokerAssign(bids, members, r_same)` → consensus
- `ApplyConsensus(consensus, options, timeout)` → a pick, or "uncoordinated"

**Oracle row:** the current Phase4Planner code path, untouched; proven identical by the replay/eval check.

**Deviation from §20.6, decided 2026-10-23.** Coverage travels as `SUBMAP_PATCH` (8) at the PLANNER grid
(0.05 m, 32 × 32 blocks, 2-bit codes: unknown / occupied / free / free-and-ρ_responder-passable), not as §20.6's
0.5 m coarse summary. Reason: egress at ρ_responder (0.40 m) on 0.5 m cells would erase doorways, breaking invariant 4
at the operator. Sizing:
- **Wire:** 256 B packed per block plus ≤ 64 B FlatBuffers overhead, zstd-compressed (protocol test
  `CoordinationWire.*`).
- **Estimate:** 0.5–3 KB per 5 s patch [ESTIMATE; to be measured on a real run].

`SUBMAP_SUMMARY` (7) stays reserved.

### 9. Broker modes: R1 overlap-aware and R2 discounting (approved 2026-10-23)

Backed by a survey; the report is in a scratch directory and is to be copied into evaluation evidence with the
rows. Its findings:
- Per-view IG stays unknown-voxel counting. That equals MI for a noise-free sensor (Corah & Michael ICRA 2021), and
  per-ray MI models (FSMI, CSQMI) cost more for less.
- The information-per-compute gain is in joint, multi-view accounting at the broker.

`BrokerMode {kPlain, kDiscount, kOverlap}` is one config key, so the three are rows on equal footing:
- **kPlain:** today's exactly-once SGA on bid utilities, with r_same.
- **kDiscount (R2, Burgard et al. T-RO 2005):** after each assignment, every other option's value is multiplied by
  (1 − P(d)), with P(d) = max(0, 1 − d / max_range_m) and d the distance to an assigned goal.
  - **Remembered claims (zero bytes):** an agent that falls back to "uncoordinated" re-ranks its own options by the
    peers' goals in the newest consensus it holds, with P(d) decayed by e^(−age/τ), τ = 30 s [ASSUMPTION].
- **kOverlap (R1, Lauri et al. RA-L 2020, lifted to blocks):**
  - Each bid option carries `footprint`: ≤ 32 `{bx, by, w}`, where w is the count of the option's full-IG unknown
    voxels in that block not known to peers, in units of kFootprintUnit = 16 voxels (rounded up, cap 255; 20de9e8). It is collected inside the existing ray cast and only when the
    per-agent planner asks, so the oracle row is untouched.
  - The broker replaces only the coverage term with its marginal, Δcov(o | S) = Σ_b max(0, w_ob − residual_b). Costs
    are subtracted as in D114 (decision: cost enters as gain − λ·cost).
  - Per round it logs Δf, the pure-gain f(S) and the data-dependent upper bound UB, reported as f/UB on the gain
    proxy only. The ½ partition-matroid bound applies to the pure-gain variant only, never to exploration time
    (invariant 7).
- **Bytes:** a footprint is 5 B per entry, ≤ 160 B per option, ≈ 0.4–1 KB per bid per 5 s, inside the 1 KB/s
  coordination bucket. Measured in the rows.

**Rows** (added to §4's required set): plain, discount and overlap, each at ariadne's budget, 2 scenes × 3 seeds.
Reported per row:
- time-to-90 % coverage;
- redundant-observation ratio;
- bytes by class;
- broker ms, timed by the caller.

---

<a id="operator-console"></a>

# Operator console

Plan 1.10: "Console stub: WebSocket telemetry, 2D top-down map, byte gauges (§29)." Arch §29.3: FastAPI backend,
WebSocket telemetry at 2–10 Hz, REST for approvals; Three.js + a splat renderer frontend; incident timeline (§29.2);
hosted live for judging week (Phase 8). The stub fixes the **interfaces** (telemetry schema, endpoints) and shows
the three things the plan names; the 3D splat view, approvals and live hosting come later.

### Data source in Phase 1: a replayed run chain

The console replays a committed run chain (sim_collab → recon → mission) at a chosen speed. Live runs later publish
the SAME telemetry frames, so the frontend never knows the difference. Two producers gain time-resolved outputs:

- `ariadne_sim_collab` writes `ledger.csv`: every message as the ByteLedger saw it (sequence, class, sender,
  receiver, payload/compressed/wire bytes, the wire stamp, and a **causal send time**). Totals per class (bytes
  and counts) must equal `results.yaml` (asserted). The wire stamp is a keyframe time and is NOT causal (a request
  is stamped with agent 0's keyframe at ~138 s, the response and edge with agent 1's at ~20 s), so the console uses
  the causal time: descriptor = its keyframe time; request = the later of the two keyframes; response = the
  request's time; edge = the response's time — the rule 1.8 uses for t_merge. The wire stamps are unchanged (the
  encoded bytes, and so the 1.6 evidence, stay identical). `[ASSUMPTION]` zero processing and link latency.
- `ariadne_mission` writes `snapshots/slab_<t_ms>.png` at every replan (the 2D tri-state map then).

### Telemetry (WebSocket `/ws/telemetry`, JSON, one frame per tick)

```text
{ "t_s": mission time, "seq": n,
  "agents": [ {"id": 0, "x": m, "y": m, "yaw": rad, "merged": bool} ],   # world frame (briefing anchor + 1.6 merge)
  "bytes":  { "<CLASS>": cumulative wire bytes, ... , "total": B },          # from ledger.csv, by CAUSAL send time
  "rate_Bps": bytes in the last second,
  "map_version": index of the latest snapshot at or before t_s (fetch /api/map/<i>.png),
  "route": {"egress": [[x, y], ...] | null, "agent": [...] | null},          # latest replan at or before t_s
  "events": [ {"t_s", "kind", "text"} ]                                      # new since the last frame
}
```

Agents are drawn from their ESTIMATED poses only, through the same world anchor the mission uses (agent 0's entry
pose; agent 1 through T_O0_O1 and only once merged — before that it is shown as "not merged" with no position, which
is what the broker actually knows). Events come from the chain's own outputs: keyframe counts, each verified edge
(1.6), the merge (t_merge), each replan's frontier decision, the first valid egress route, routes changing exits.

REST: `/api/meta` (grid, exits, target, scene outline, chain ids, speed), `/api/map/<i>.png`, `/healthz`.
Time base: mission time = t_ns − t0 with the mission's t0, for the ledger, trajectories and snapshots alike;
tests assert the console's merge and first-valid-route events land at the mission's 138 s and 140 s. Frontier
decisions appear on the timeline only when the chosen frontier changes, labelled "logged decision · replay · not
executed" (invariant 3). Ticks at 5 Hz wall time; `speed` (default 10× mission time) is a server option. Deterministic: frame n at a given
speed is a pure function of the chain (tested byte for byte).

### Frontend (static, served by the backend)

One page: a canvas top-down map (snapshot image, GT-free outline not shown: GT is not operator knowledge; agents
with trails; egress route at ρ_responder bold, agent route thin), byte gauges per class (cumulative bar + rate), the
incident timeline (§29.2), a status strip (mission time, merged?, route valid?). No CDN libraries needed (Canvas 2D).

### Packaging

`python/operator_console/` (server.py: FastAPI app; replay.py: chain → frames; static/index.html), pinned
`infra/docker/console.Dockerfile` (FastAPI, uvicorn, numpy, pyyaml, opencv-headless), `infra/scripts/console.sh
CHAIN... [--speed]` → http://localhost:8080. Tests: frame builder determinism, ledger totals = results.yaml, the
WebSocket endpoint through FastAPI's TestClient.

### Not in 1.10

3D splat view (the 1.7 viewer exists separately), approvals (§23), raw vs restored feeds (Phase 5), live hosting
(Phase 8), budget gauges against a policy budget (Phase 3 defines budgets; the stub shows cumulative + rate).

---

<a id="web-demo-console"></a>

# Web demo console

This amendment implements the agreed one-screen demo plan. It supersedes the
one-horizon expiry and skipped-map-export portions of agentic.md D169 for demo
mode. Evidence runs retain their lockstep chooser behavior.

### User flow

`infra/scripts/demo.sh` serves the new console at `/`. Legacy chain/live consoles
remain available through their existing environment configuration; `/demo` always
serves the new UI. A controller claims the shared worker; everyone else spectates.

1. Choose a preset and seed (Random chooses and displays a new seed).
2. Inspect the generated mesh and top-down geometry, explicitly labeled ground
   truth. Select existing entrance markers and a reachable target. There is no
   arbitrary spawn placement or cosmetic sensor degradation.
3. Deploy for 60, 120 or 300 simulation seconds. Setup gives way to observed
   reconstruction, estimated occupancy/routes, cameras with tracker features,
   and decision/approval cards. Ground truth remains an explicit separate view.
4. Hold is acknowledged by the runtime at a planning barrier; it is not an
   instantaneous physical stop. End run requests hold and orderly termination.
5. Replay recorded displayed observations/decisions without model calls, or train
   splats from the recorded reconstruction stream. A new live run of the same
   seed recreates the scene, not necessarily the autonomous execution.

The page and its panes never scroll. Decisions/evidence use bounded pages.
Laptop checks cover 1280×600, 1280×720, 1366×768, 1440×900 and 1920×1080.
Map keyboard controls: arrows move the cursor, Shift increases the step, Enter
places a target/region vertex, 1–3 toggle entrances, Escape cancels region drawing.

### Boundaries and contracts

- FastAPI owns one session controller and one telemetry tailer. An advisory
  process lock plus a launcher lock prevents concurrent simulations. The server
  must use one uvicorn worker. Existing mutation endpoints are blocked in hosted
  demo mode, so they cannot bypass controller ownership.
- `POST /api/demo/lease` returns an opaque controller token. Subsequent actions
  use `X-Controller-Token`; the browser heartbeats every five seconds. Thirty
  seconds without a heartbeat queues fleet hold. Spectators cannot mutate state.
- `generate`, `validate`, `start`, `directive`, `hold`, `stop`, `approval`, and
  `train` are POST actions under `/api/demo`. Start/directive require an
  `Idempotency-Key`. Coordinates and entrance indices are validated server-side;
  the worldgen clearance/reachability checker is authoritative for setup.
- `/api/demo/stream` broadcasts current state at 5 Hz; reconnection receives a
  complete snapshot. Payloads include a session ID, sequence and simulation time.
  Slow clients receive the latest state; they cannot backpressure the runtime.
- `/map/{version}`, `/media/{name}`, `/chunk/{name}`, `/table`, `/playback`,
  `/preview.png`, `/world.obj`, `/entrances`, and `/status` supply read-only data.
  Immutable media URLs include the session ID; cross-session requests are refused.
- Command states are queued, effective or rejected. Runtime intake records supply
  the acknowledgment. HTTP success alone never means that a command executed.
- Approvals are accepted only while pending and before their wall deadline. The
  outstanding supervisor decision ID must still match. Verdict files are atomic;
  duplicate/stale submissions fail. C++ independently checks candidate validity,
  original approval ID, exact approved goal and the approved risk envelope.

### Pipeline

ROS/Gazebo and algorithm boundaries remain unchanged. The frontend cannot publish
ROS commands. The hosted launcher uses `live_web_demo_v1.yaml`, Phase 4 planning,
radio coordination and live collaboration. The default exposed agent limit is two;
three requires an explicit deployment setting after hardware qualification.

`DemoCamera` exports at most 10 sensor frames/s per agent. JPEG work runs on a
separate latest-wins thread; overwritten display packets are counted. Each
atomic manifest names an immutable image and that image's actual tracker points.
This is a display channel, not a second feature detector. The worker callback
still obtains the rectified RGB image; its added cost must be measured on target
hardware. Maps are written before telemetry advertises their version.

The existing CPU reconstruction service seeds submaps from observed stereo depth.
The browser renders those records as points, capped at 200,000 displayed points,
with decoding in a Web Worker. Placement corrections update matrices. Geometry
and camera media are separate from telemetry. Historical chunks are retained for
recorded playback. The raw received reconstruction stream is saved as `recon.link`;
this contains received tee frames, not a guarantee that the display tee dropped no
keyframes. The canonical consumed sensor recording remains the full run substrate.

Post-run `--recorded-link` feeds the same ingestion path into the GPU trainer.
The controller uses only an already-installed GPU image, never an implicit cloud
launch or pull. Failure retains the observed point cloud. Trained models up to
500,000 Gaussians use Spark; larger models retain the bounded point representation.
Generated world geometry is never inserted into observed reconstruction.

Demo LLM supervision now retains one outstanding request for up to the configured
wall timeout, allowing a human review to finish. Deterministic planning continues
while it is pending. On arrival, the original goal is matched and validated against
the current candidates; an invalid override falls back. Request timeout or link
failure is visible in the trace. `supervisor.json` tells the console which request
is still actionable. There is no free-text command translator in this release.

### Run locally / provision a worker

Host dependencies: Python 3.12, Node/npm for the one-time browser build, Docker,
and the repository's pinned sim/core/orchestrator/reconstruction images. The host
controller needs Docker access; serve it through an HTTPS reverse proxy for judging.
It binds loopback by default and must not be started with multiple workers.

```sh
python3 -m venv /tmp/ariadne-demo
/tmp/ariadne-demo/bin/pip install -r python/operator_console/requirements-demo.txt
npm --prefix python/operator_console ci
npm --prefix python/operator_console run build
bash infra/scripts/live.sh build
# Build core runtime + ariadne_collab_worker in the pinned core-dev image as usual.
ARIADNE_DEMO_PYTHON=/tmp/ariadne-demo/bin/python bash infra/scripts/demo.sh 8080
```

Deployment settings:

- `ARIADNE_DEMO_MAX_AGENTS=2` (1–3): hardware-qualified fleet size.
- `ARIADNE_DEMO_VIDEO_URL`: the demo video URL; unset by default, which hides the link.
- `ARIADNE_RECON_GPU_IMAGE`: installed CUDA trainer image; default
  `ariadne/recon-gpu:build`. CPU reconstruction uses `ARIADNE_RECON_IMAGE` as before.
- `ARIADNE_LIVE_BINARY`: optional runtime binary path relative to the repository;
  default `build/docker/cpp/runtime/ariadne_live`. Its collab worker is a sibling.
- Existing image and AWS region settings are inherited by the launcher. Bedrock
  uses the existing credential path. Missing credentials fail visibly.

GitHub points to the repository remote. There is no paper link;
the video link is
hidden until configured. Publishing these artifacts and hardware qualification
remain deployment gates, not claims made by a local browser test.

Artifacts live under `build/demo/`: isolated generated scenes and session run
folders containing configuration/scene identity, launcher logs, playback JSONL,
canonical runtime recordings and reconstruction input/output. A controller restart
requests shutdown of unfinished sessions before permitting another launch.

### Validation and release gates

Tests: `python/tests/console/test_demo.py`, opt-in
`RUN_BROWSER_TESTS=1 ... pytest python/tests/console/test_demo_browser.py`,
`DemoCamera.PublishesMatchedDownsampledImageAndFeatures`, and reconstruction
`test_recorded_live_stream_replays_identical_geometry`. The browser integration
uses synthetic runtime telemetry and real world generation; it does not count as
Gazebo or Bedrock acceptance.

Release targets remain measurements to perform on the deployed worker/laptop:
RTF ≥0.95 for every enabled fleet size; ≥30 fps interactive viewport; p95 primary
camera age <500 ms and telemetry age <1 s; warm page usable within two seconds;
real approval/fallback and controller-disconnect exercises. Test both stale media
and reconstruction failure without losing mission control. Do not label the demo
real-time solely because its configured Gazebo real-time factor is 1.

The repository-wide Phase 1 premerge was attempted during this implementation and
failed before tests because the existing FetchContent subbuild cache records
`/data/code/Ariadne` while the container mounts `/work`. An isolated pinned-container
build is used for targeted verification; it does not substitute for that gate.

#### Implementation verification (development tree)

- 49 Python/worldgen/reconstruction/browser checks passed; Chromium exercised
  setup, approval, hold, stop, playback, point rendering and laptop layouts.
- 14 targeted C++ checks passed, including matched image/features and live
  link/planner/lockstep gates. Runtime and collaboration worker compiled in the
  pinned OpenCV 5 container using `build/demo-cpp`.
- 5 mocked Bedrock/MCP orchestration checks passed in `ariadne-orchestrator:1`.
- Real local two-agent smoke: `build/demo-runtime-smoke/live-20261026T061547Z-9965`,
  ten simulation seconds, classical chooser, Mesa software rendering. Both
  cameras exported 640×360 images with 500/495 tracked points and zero recorded
  display overwrites. Two occupancy snapshots and two reconstruction submaps were
  produced; the reconstruction service received 28 keyframes without service-side
  drops. This is a functional smoke, not a real-time or LLM performance result.
- Real controlled-stop smoke: `build/demo-stop-smoke/live-20261026T062336Z-820d`.
  Stop requested after camera time 2 s; runtime reported `operator_stopped: true`,
  `actual_end_s: 5`, exit 0, with MCAP and the reconstruction table preserved.
- Full Phase 1 premerge remains blocked at its existing FetchContent cache-path
  mismatch. GPU training/display performance, full hosted collaborative/Bedrock
  acceptance, final video URL and public HTTPS publication remain open.
