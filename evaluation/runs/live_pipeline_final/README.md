# Live pipeline repair checkpoint

Branch: `fix/live-pipeline-final`. Isolated worktree: `/data/code/Ariadne-live-final`.
Main and its uncommitted changes were not modified.

## Implemented and locally checked

- Nonblocking, byte-bounded sensor demultiplexing prevents the image/IMU circular wait.
- Demo queues retain two complete pending stereo pairs and at most two incomplete timestamps;
  an in-flight pair is preserved. IMU is never silently dropped. Selection/drop events are journalled.
- Demo decision scheduling can advance with any agent; collective acknowledgement waits are removed.
  Stale agents receive hold commands. A 120-wall-second estimator-progress watchdog fails explicitly.
- Collaborative demo keyframes now reach the per-agent maps. Those maps remain mission-thread-owned;
  asynchronous per-agent map/planner actors are still outstanding.
- Allocation uses broker/member topology identity instead of the geometric solve counter.
  Consensus additionally checks full broker/member equality. Map epochs remain geometric epochs.
- Energy accounting uses estimated odometry instead of ground truth.
- Worker and input-reader failures propagate to the mission exit status.
- Runtime stack manifest records resolved settings and explicitly labels the existing known-coordinate target.
- GPU live defaults use a 50 ms optimization budget instead of 100 fixed iterations. Minimum three
  iterations and unavoidable initialization/lifecycle work may exceed that budget. Batch mode is retained.

## Validation

- C++ runtime and collaboration worker built in `ariadne/core-dev:dev`, OpenCV 5.
- 101 unit tests passed, including six sensor-queue tests.
- 22 targeted scheduling, snapshot and multi-agent tests passed.
- 16 reconstruction tests passed in `ariadne-recon:1` (CPU tests; no GPU speed claim).
- Incremental runtime rebuild passed after stack manifest addition.
- The first 30-s live cloud smoke completed with GPU training, but its first command
  arrived at 29.901 simulated seconds and recorded GT translation was zero. It
  validates liveness, not exploration. Replay was skipped. See `smoke30/`.
- Subsequent sender/tick changes passed 102 unit and 21 targeted tests; the ROS
  bridge built. The further callback-isolation change also built; its live gate is pending.
- Persistent research and remaining performance work: `docs/design/LIVE_PIPELINE_BACKPRESSURE.md`.

`prior_200s_audit.json` describes the earlier successful live run and includes its archive SHA256.
It records coordinated assignments on 20/40 and 18/40 decisions, frequent epoch mismatches,
and no reported egress route. Map counters are not ground-truth coverage.

## Remaining release gates

Reconcile newer backend changes from main; short live transport/mapping verification;
completion-aware planning and paired outcomes; hidden visual target; coherent map correction;
asynchronous trained reconstruction and profiling; fresh-seed live acceptance and resource comparison.
This checkpoint is not completion of the end-to-end implementation plan.
