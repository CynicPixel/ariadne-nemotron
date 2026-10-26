# Live pipeline latency, scheduling and overload control

Updated: 2026-10-27. Branch: `fix/live-pipeline-final`.
This is the persistent implementation/research record for the final live pipeline.
Frontend work is outside this change. Main is untouched.

## Objective and evidence

Run the adopted estimator, collaborative mapping/planning and online reconstruction
on a free-running simulated world, with bounded observation and command age. RTF >1
is desirable, but throughput alone does not demonstrate a functioning feedback loop.
Whole stereo pairs may be dropped in demo mode; IMU intervals must be retained or a
gap must cause an explicit estimator response. Lockstep evidence stays lossless.

The completed 30-s cloud smoke (`20261026T165827Z-gpu-ec5893`) processed
106/111 agent frames, integrated 15/16 keyframes and trained 36 reconstruction
keyframes. Runtime wall time was 206.103 s (RTF 0.145558). It exited successfully,
but the decision stamped 5 s was applied at 29.901 s and both recorded ground-truth
translation paths were zero. This establishes transport/trainer liveness, not
exploration, planner convergence or map completeness. Replay was skipped.
Small durable artifacts are in `evaluation/runs/live_pipeline_final/smoke30/`;
full artifacts are under the run ID in S3 bucket `ariadne-836505524290-eun1`.

Confirmed code hazards: blocking socket writes inside image callbacks; shared
single-threaded component executor; reliable history depth 500 for all sensor
subscriptions. At 20 Hz, 500 images can represent 25 simulated seconds of backlog.
This capacity is consistent with the observed delay, but is not a measured
attribution of the entire delay. Stage-level tracing must establish that.

## Primary-source research and application

1. **Callback/executor isolation.** ROS 2's default mutually exclusive callback
   group serializes its callbacks even with a multithreaded executor. Explicit
   groups can be assigned to separate executors. Our component container is
   actually single-threaded, making dedicated execution capacity necessary.
   [Jazzy callback groups](https://github.com/ros2/ros2_documentation/blob/jazzy/source/How-To-Guides/Using-callback-groups.rst)
   and [executors](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Executors.rst).
   Application: demo clock, IMU and supervisory callbacks use a private executor;
   image serialization remains on the container executor. Each serialization
   buffer remains confined to a mutually exclusive callback group.

2. **Freshness at ingress.** ROS SensorDataQoS favors small histories and best
   effort delivery. QoS compatibility must be checked at both endpoints.
   [Jazzy QoS](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst).
   Application: demo image subscriptions use depth 2 / best effort. Non-image
   data retains reliable delivery. The downstream stereo queue accepts complete
   matching pairs, preserves an in-flight partner and discards stale pairs.
   Small DDS histories alone do not provide stereo synchronization or IMU continuity.

3. **Sensor-specific synchronization.** Isaac ROS VSLAM exposes separate image
   synchronizer and IMU sequencer buffers, QoS, timestamp matching and jitter checks.
   [Pinned release 3.1 interface](https://nvidia-isaac-ros.github.io/v/release-3.1/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html).
   Application: retain all IMU samples between selected stereo timestamps; record
   accepted-frame intervals and explicitly detect gaps. Do not copy buffer defaults
   from another release or assume arbitrary frame skipping preserves tracking.

4. **Periodic control with independent scheduling.** Nav2 exposes controller
   frequency, costmap update timeout and optional elevated priority; ros2_control
   attempts FIFO scheduling for its main loop.
   [Nav2 Jazzy](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/controller_server/),
   [ros2_control Jazzy](https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/userdoc.html).
   Application: command supervision and communication heartbeats should advance
   independently of map integration, graph solves, DNNs, LLM calls and splat training.
   Current radio catch-up repairs skipped horizons but is still decision-driven.
   Elevated scheduler priority is deferred until blocking dependencies are removed.

5. **Asynchronous transport with bounded overload.** Jazzy rmw_fastrtps supports
   background publication and documents synchronous default publication.
   [Implementation documentation](https://github.com/ros2/rmw_fastrtps/blob/jazzy/README.md).
   Application: use a dedicated bounded socket sender, with fair image/control
   servicing and explicit overflow failure for indispensable data. Changing DDS
   settings cannot fix the application's own blocking Unix socket. One shared
   socket still permits head-of-line delay behind an in-flight image; separate
   bulk/control channels are a possible later improvement if measured necessary.

6. **Reduce serialization/copy cost.** NITROS avoids unnecessary CPU/GPU copies
   between compatible nodes through type adaptation/negotiation.
   [NVIDIA explanation](https://nvidia-isaac-ros.github.io/v/release-3.1/faq/index.html).
   Application: select owned stereo messages before serialization, then consider
   shared memory. NITROS is not a drop-in acceleration for the current Unix protocol.
   Current sender still serializes and copies before its pair-selection queue.

7. **Trace latency distributions.** ROS tracing exposes callback events/durations.
   [Jazzy tutorial](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Tutorials/Advanced/ROS2-Tracing-Trace-and-Analyze.rst).
   Add application timestamps at capture, ROS receipt, socket enqueue/write,
   estimator start/end, decision and command application. Use monotonic wall clocks
   for processing latency and simulation stamps for world-state age. Report p50,
   p95, p99, maximum, sample counts, drops and queue peaks. Existing bridge latency
   uses clock receipt as a proxy and cannot establish complete capture latency.

## Implementation sequence and acceptance

- [x] Nonblocking sensor demultiplexing; bounded latest stereo queue; explicit IMU
  overflow; worker failures propagated; map ingestion and allocation identity fixes.
- [x] Dedicated fair socket sender for demo, with no socket lock held across I/O
  that also protects statistics. Lossless mode retains existing semantics.
- [x] Radio tick cursor covers intervals skipped by demo planning horizons.
- [x] Implement demo-only private executor for non-image/clock/supervisory callbacks,
  plus depth-2 image ingress. ROS build passed; live validation pending.
- [x] Expose `LIVE_AWS_INSTANCE_TYPE`; smoke defaults to `g4dn.2xlarge` after the
  approved quota increase. Preserve generic launcher defaults for other experiments.
- [ ] Live validation: actual translation, current commands, bounded queues, both
  agents ingesting maps, collaboration events, online optimizer progress, clean exit.
- [ ] Independently scheduled communication/control; command expiry and per-stage
  age diagnostics; bounded DNN concurrency; model/GPU warmup before mission unpause.
- [ ] Select owned stereo messages before serialization if profiling justifies it.
- [ ] Independent map/planner and reconstruction intake/training workers; coherent
  map corrections; hidden visual target; completion-aware allocation and seed sweep.

Overload tests should deliberately slow image consumption and reconstruction.
Clock/IMU/control must keep progressing; image backlog must remain bounded; stale
commands must be superseded or rejected. Track maximum accepted-pair gap and VIO
health. Map/route quality and mission completion require their own GT scoring.
A passing process exit and smooth render are insufficient acceptance criteria.

## Compute approval and experiment interpretation

AWS support approved **8 vCPUs** for Stockholm **All G and VT Spot Instance Requests**,
partially fulfilling the requested 16. User supplied approval on 2026-10-27;
`service-quotas get-service-quota` independently returned 8.0.
Next host: `g4dn.2xlarge` (8 vCPU, 32 GiB, one T4), subject to Spot capacity.
It doubles CPU/RAM versus the preceding g4dn.xlarge; GPU count remains one.
This quota is aggregate concurrent G/VT Spot vCPU capacity, not a per-instance grant.
No 16-vCPU GPU host or two simultaneous 8-vCPU GPU hosts fit it.

The larger host should relieve CPU contention, but no speedup is claimed before
measurement. Since bridge changes and host size both change, the next smoke is an
integration test, not an isolated hardware speedup benchmark. Keep resolved source,
config, host type, startup costs and stage timings in the evidence manifest.
