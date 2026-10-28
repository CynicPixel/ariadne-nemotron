# Local Decisions

Dated decisions taken during implementation. Anything here that changes the
architecture must be mirrored back into `architecture.md` per §49.

---

## 2026-08-31 — D1: Tangent-vector ordering is `[rho; phi]` (translation first)

Arch §11.3 pins the *perturbation convention* (left-multiplicative) but not the
*ordering* of the 6-vector. Chosen: `[translation; rotation]`, matching Sophus.

**Why it matters:** the SE(3) adjoint's block layout depends on this ordering. GTSAM
orders it `[rotation; translation]`, so when GTSAM is used as a test oracle (§11.6) the
ordering must be swapped **at that boundary**, never inside `cpp/core`.
Enforced by `SE3.AdjointMovesPerturbationsBetweenFrames` in `tests/numerical/test_lie.cpp`,
which fails on a wrong ordering while every round-trip test still passes.

## 2026-08-31 — D2: Eigen pinned at 3.4.1, not 5.0.x

Arch §27.1 says "Eigen 3.4". Upstream now also offers 5.0.1. Doc wins: pinned **3.4.1**
(latest 3.4.x). Revisit only if a needed feature is 5.x-only; it would be an arch change.

## 2026-08-31 — D3: MCAP via the standalone C++ SDK, not rosbag2

Arch §5.9 names "rosbag2 MCAP" as the format; arch §3's dependency rule requires replay
to work from files alone, ROS-free. These are compatible because MCAP is a container, not
a ROS API: we read/write it with the standalone Foxglove MCAP C++ SDK, so `cpp/core` never
sees a ROS type and replay needs no ROS install. Bags stay rosbag2-compatible.

## 2026-08-31 — D4: `SE3Log` inverts the left Jacobian numerically

`V^-1` is computed with a 3x3 `Eigen::inverse()` rather than its closed form. `V` is
always invertible for `|phi| < 2*pi`, the cost is negligible, and this removes a class of
transcription bugs from a coefficient formula that is easy to get subtly wrong.
Alternative (closed form) may be added later **with a test against this version**.

## 2026-08-31 — D5: `-ffp-contract=off` project-wide, no fast-math

The Phase 0 gate requires the degradation operator `D` to be bit-identical across
machines. Compilers may fuse `a*b+c` into an FMA, changing the last bit depending on
target ISA — differently on amd64 vs Graviton. Contraction is therefore disabled
globally rather than only for `D`, so the property cannot be lost by a file moving
between targets.

## 2026-08-31 — D6 `[OPEN — needs a decision]`: `cv::XFeat` does not exist

arch §0.2 locks XFeat as the primary feature extractor. `docs/algorithms.md` records
this as low-risk because XFeat is "a native OpenCV 5 class", and at line 63 documents a
concrete signature:
`cv::XFeat::create(modelPath, maxKeypoints, scoreThreshold, inputSize, backendId, targetId)`.

**That class does not exist in OpenCV 5.0.0 or contrib** (evidence:
`validation.md` §2 — exhaustive case-insensitive search of both trees). The
catalogue's claim is a factual error, and it is load-bearing in six places there
(§1.2 shortlist, the OpenCV-5 class table, the recommended per-agent front-end, and the
EC3R-SLAM "strongest cross-domain signal" argument).

**The locked decision itself may still survive.** What is false is "OpenCV ships a class
for it", not "XFeat is usable". Verified upstream facts (2026-08-31):
- `github.com/verlab/accelerated_features`, Apache-2.0 (license confirmed in catalogue L36).
- Ships **PyTorch** weights only: `weights/xfeat.pt` (6.2 MB), `xfeat-lighterglue.pt`.
  **No ONNX export is published** — we would own that step.
- The model is small (~10 conv blocks; `modules/model.py` is 4.5 KB).

**Relevant fact that applies to every option:** OpenCV 5 ships ALIKED/DISK/LightGlue as
classes but **not** their models — all three take a `modelPath` to an ONNX file. So a
learned feature path requires sourcing/exporting ONNX weights no matter which is chosen;
that cost is not unique to XFeat, and §7.4's DNN execution contract governs all of them.

Options, none yet chosen:
- **(a) Keep XFeat primary, via ONNX + `cv::dnn`.** Export `xfeat.pt` ourselves under
  §7.4 (ORT fallback allowed, log which ran). Preserves the locked decision and the
  catalogue's benchmark case. Risk: the export must survive `cv::dnn`'s ONNX coverage —
  a `[SMOKE-TEST]` gate, not an assumption.
- **(b) Promote ALIKED to primary** via the §49 ritual, demoting XFeat to an ablation.
  Native class, but C8 rates ALIKED ~5 FPS-class on CPU, and the catalogue's speed
  argument for XFeat (16.7 ms; the only learned method above 1 FPS on embedded) is
  exactly what motivated the choice.
- **(c) Defer to the Phase 2 opening**, recorded now, decided when 2.1 starts.

Does not block Phase 0. Blocks Phase 2.1.

## 2026-09-06 — D7: manifests pin the PEELED COMMIT, never the tag-object SHA

`5.0.0` is an annotated tag in both OpenCV repos. `git ls-remote --tags --refs` returns the
**tag object** SHA; the commit is the peeled `refs/tags/5.0.0^{}` row. The two differ, and an
earlier revision of `validation.md` recorded tag objects as commits — which would have
looked like a provenance mismatch against the image's own recorded SHAs during a reproducibility
audit (§36).

**Rule:** every §28.2 manifest field naming an upstream revision records the commit, obtained
from `git rev-parse HEAD` inside the build (as `opencv-base.Dockerfile` already does) or from
the `^{}` row of `git ls-remote --tags <url> '<tag>*'`. Never from `--refs`.

Correct commits for OpenCV 5.0.0: core `40738fb16ceddb5fb3fea747585f7ce6abb0605b`,
contrib `755e50675d97db9b7d449d8bd6b09888646f6c6e`.

## 2026-09-06 — D8: contrib build set is exactly C7's four modules

`xfeatures2d` was in the first `BUILD_LIST` speculatively and **removed**: nothing in the
architecture uses SURF/BRIEF/FREAK/DAISY, and it fetches `boostdesc`/`vgg` blobs at *configure*
time from a slow third-party host, which stalled the first build entirely. Built contrib set is
`ximgproc`, `xphoto`, `optflow`, `quality` — exactly the modules C7 names.

## 2026-09-06 — D9 `[OPEN — needs a decision]`: dust is composited after the sensor chain

§5.6 states the degradation chain in this order:

```
radiance scale -> shot noise -> read noise -> PRNU/vignette
  -> exposure/gain -> quantization/clipping -> optional dust compositing
```

`Degrade()` implements that literally, because the doc wins. Recording the physical
objection so the choice is deliberate rather than accidental:

**Dust is a scene-level effect and physically precedes the sensor.** Airborne particulate
attenuates scene radiance and adds airlight *before* photons reach the detector. Applying
`I = J*t + A*(1-t)` after quantization makes dust a pure post-process, which loses two real
couplings: attenuation should REDUCE photon count and therefore reduce shot noise, and
airlight should ADD photons and therefore add its own shot noise. In the implemented order a
heavily dusted pixel carries the shot-noise statistics of the clean radiance it no longer has.

**Why it matters beyond tidiness:** Phase 5 measures each radiometric stage's effect on
repeatability, match precision and ATE (§31.5). A dehazing or contrast-recovery stage
evaluated against post-sensor dust is being tested against a strawman that is easier to
invert than real dust, because the noise field it must fight was generated from
undegraded radiance.

The same objection applies, more weakly, to PRNU and vignette sitting after read noise:
vignetting is optical and precedes photon conversion, and read noise is added by the readout
circuit and is not vignetted. The implemented order multiplies read noise by the vignette
field.

Options: (a) keep §5.6's order and note the approximation in the report; (b) amend §5.6 to
the physical order under the §49 ritual and re-record all golden digests; (c) implement both
behind a flag and measure the difference once, which turns the question into a number.
Recommendation is (c) then (b) if the difference is material. Not blocking: the operator is
deterministic and self-consistent either way, and no published result depends on it yet.

## 2026-09-06 — D10: one keying discipline for all seeded fields

Every seeded value in the degradation chain is keyed as
`(seed, frame_id, site)` where `site` encodes pixel, channel and stage, and
multi-octave noise folds the octave index BENEATH a caller-owned `salt_base`
(`salt = salt_base * 64 + octave`).

Fixed a violation of this in the dust field, which had been keyed on
`frame_id ^ NoiseStage::kDustField`. Mixing a stage id into the frame dimension
permutes frames against one another (frame 0 and frame 3 exchange fields), and
the `salt` argument was already consumed by the octave index, so any second
caller would have collided with octaves. Stage now travels in `salt_base` and
`frame_id` passes through unmodified.

Consequence: the degradation golden digest changed from `9169008833890659974`
to `12016663175328231476`. No published result depended on the old value. The
determinism gate caught the change and refused, which is the intended behaviour.

## 2026-09-06 — D11 `[PROXY — grouped with D9 for decision]`: the dust field is temporally white

The particulate field is keyed on `frame_id`, so it is statistically independent
from frame to frame. Real airborne dust advects and evolves over seconds, so
consecutive frames are strongly correlated.

This is the same class of modelling error as drawing PRNU per frame, which is
explicitly avoided (see `kFixedPatternFrameId`): a temporally white field can be
removed by simple frame averaging, so a dehazing or contrast-recovery stage
evaluated against it is being tested against an unrealistically easy target. The
Phase 5 DVE ablation (§31.5) is where this would distort a result.

Options: (a) static field per sequence, correlated but not advecting; (b)
interpolate between fields keyed on `frame_id / N` for a chosen N, giving a
tunable correlation time; (c) keep it white and state the limitation. (b) is
cheap and closest to the physics. Grouped with D9 because both are questions
about how faithful the dust model needs to be before Phase 5 depends on it.

## 2026-09-06 — D12 `[OPEN — needs a decision]`: §15.7 size table vs measurement

Measured on `cpp/core/serialization/wire.fbs` (envelope overhead 96 B excluded by
differencing against an empty graph; messages fully POPULATED, see the trap below):

| Element | §15.7 predicts | Measured | Delta |
|---|---|---|---|
| Pose-graph node | 28 B | 65.8 B | +135% |
| SE(3) pose edge | 150 B | 193 B | +29% |
| Graph 200n+40e, uncompressed | ~12 KB | 21.0 KB | +75% |
| **Graph 200n+40e, zstd-3** | ~12 KB | **4.3 KB** | **-64%** |

### The finding that reframes this

**Pose graphs compress 4.93x** (21008 -> 4260 B at zstd level 3). Node fields are
near-sequential keyframe ids, closely spaced timestamps and smoothly varying poses, which is
almost ideal for an entropy coder. §15.7 currently compresses classes 4-5 only
(MATCH_RESPONSE, POSE_EDGE) and leaves POSE_GRAPH uncompressed.

So the real gap is not that the schema is bloated. Uncompressed it is 75% over the
prediction; compressed it is 64% under. Either way §15.7's 12 KB is not what the link
carries, and the direction depends entirely on a compression policy the doc sets elsewhere.

**This strengthens rather than weakens the §15.7 thesis** that front-end descriptor traffic
dominates back-end traffic by orders of magnitude: back-end traffic is even cheaper than
predicted once compressed.

### Correction to an earlier version of this entry

An earlier revision claimed 28 B/node was "unreachable" because §15.5 requires frame-tagged
poses. That was too strong. The tag need not be per node: `PoseGraph` already carries a
batch-level `anchor:FrameTag`, and nodes from one agent share a frame, so hoisting the tag to
the batch satisfies §15.5 while removing 8 B/node. Combined with a `struct` node (no vtable,
-10 B) and delta-encoded ids and timestamps, 28 B is plausibly reachable.

### The actual question

Is 28 B a *prediction* of what this schema costs, or a *design target* the schema must meet?

- If **prediction**: amend §15.7 to the measured values under the §49 ritual, and state the
  compression policy per class alongside them. Cheapest, and the numbers become honest.
- If **target**: apply the levers in increasing cost of properties surrendered:
  1. compress POSE_GRAPH (zero schema change, measured 4.93x, already exceeds the target);
  2. hoist `frame` and `agent` to batch level (-12 B/node, no property lost);
  3. `struct` instead of `table` (-10 B/node, forfeits schema evolution that §15.7 wants);
  4. varint ids and quantized rotations (large, costs precision and readability).

Recommendation: 1 and 2, then amend §15.7 to the resulting measured numbers. Lever 1 alone
takes the graph well below the prediction, so 3 and 4 are unnecessary for now.

### Measurement trap, recorded because it nearly published a wrong number

A first measurement put the edge at 148.8 B, apparently confirming the prediction. It used
default-constructed edges, and **FlatBuffers omits any scalar equal to its schema default**,
so unset fields cost nothing. Populating the message realistically gives 193 B. Every Phase 3
byte measurement must use populated messages.

## 2026-09-06 — D13: an uncompressed frame header precedes every payload

`Decode` previously required the caller to supply the message class in order to know whether
to decompress, but the class also lived inside the envelope: a receiver could not read it
without already knowing it. §15.3's store-and-forward triages queued messages by priority
class and TTL and must do so without decompressing, so class and compression flag now travel
in a 6-byte uncompressed header (`class`, `flags`, `payload_length`), readable via
`PeekFrameHeader`. `wire_bytes` now includes it.

Also: a compressed form is kept only when it is actually smaller. High-entropy descriptors can
expand under zstd, and transmitting the larger form would corrupt the byte table.

## 2026-09-06 — D14 `[DECIDED]`: POSE_GRAPH is compressed, extending §15.7's classes 4-5

§15.7 applies zstd-3 to classes 4-5 (MATCH_RESPONSE, POSE_EDGE) and leaves POSE_GRAPH
uncompressed. Measured, that leaves a 200-node/40-edge batch at 21.0 KB against a 12 KB
prediction. Compressed it is **4.3 KB**, a 4.93x ratio, comfortably inside the budget.

Pose graphs compress well because their content is structurally redundant: keyframe ids are
near-sequential, timestamps are evenly spaced, and consecutive poses differ slightly. This is
close to the best case for an entropy coder, and unlike descriptor payloads there is no
high-entropy content to defeat it.

**Decision:** `IsCompressedClass` covers MATCH_RESPONSE, POSE_EDGE and POSE_GRAPH. A
compressed form is kept only when smaller, so a pathological payload cannot inflate traffic.

Consequences:
- The Phase 0 gate's recorded traffic fell from 12632 B to 8626 B for 200 poses.
- D12's headline gap closes: the graph now lands under the §15.7 prediction rather than 75%
  over it. The per-element figures (node 65.8 B, edge 193 B) are unchanged and still differ
  from §15.7's 28 B / 150 B, so **D12 remains open** for the doc amendment.
- `PoseGraphsRemainHighlyCompressible` asserts the ratio stays above 3x, so a schema change
  that destroys compressibility (random uuids for ids, say) fails here rather than as a blown
  byte budget in Phase 3.

This extends §15.7 rather than contradicting it, and needs the §49 ritual to land in the
architecture doc.

## 2026-09-06 — D15 `[DECIDED]`: XFeat is implemented on OpenCV 5 primitives, not run through ONNX

Resolves D6. XFeat has no implementation in OpenCV 5.0.0 despite `algorithms.md` L63 claiming
`cv::XFeat` exists. Two routes were available: export upstream's PyTorch weights to ONNX and
run them through `cv::dnn`, or implement the network directly on OpenCV 5 primitives.

**Chosen: implement on OpenCV 5 primitives.** The competition rewards substantive technical
OpenCV 5 usage, and a hand-built modern learned extractor running on OpenCV 5 is a far
stronger claim than loading someone's ONNX file through a general-purpose runtime. It also
removes the `cv::dnn` ONNX-coverage risk flagged for XFeat's keypoint head entirely.

**What this does and does not mean.** It is a from-scratch INFERENCE implementation. Training
is not attempted: `algorithms.md` L124 records that training needs COCO synthetic warps plus
MegaDepth on multiple GPUs. Weights come from upstream `verlab/accelerated_features`
(Apache-2.0, `weights/xfeat.pt`, 6.2 MB), extracted to a plain tensor format we load. The
report must say exactly this: the architecture and forward pass are ours, the weights are
upstream and credited. Claiming more would breach the §44 claims discipline.

Scope estimate from the catalogue (L124): ~10 conv blocks, "a weekend in C++/Eigen". Work
items when Phase 2.1 opens:
1. Extract weights from `xfeat.pt` to a versioned binary blob plus a manifest of shapes.
2. Implement the forward pass: conv/BN/ReLU blocks, the depth-redistribution trunk, the
   1x1 keypoint branch over an 8x8 tensor-block-transformed image, and the 64-D dense
   descriptor head.
3. Validate against upstream PyTorch outputs on fixed inputs, to a stated tolerance. This is
   the acceptance test: without it, "we implemented XFeat" is unverified.
4. Benchmark against the catalogue's published figures (L103, L115: 16.7 ms class) and report
   the delta honestly, including if ours is slower.

Risk: this is a Phase 2.1 critical-path item with real depth. If step 3 does not converge,
the fallback is the ONNX route (unchanged in cost) or promoting ALIKED, which is native.
Record whichever ships in the run manifest.

## 2026-09-08 — D16: bags store FRAMED messages, and chunk compression is a storage choice

`McapRecordWriter` stores the framed wire message (frame header plus
possibly-compressed FlatBuffer), not the raw payload. Replay therefore exercises the
identical decode path, frame-tag validation and byte accounting as a live run, so a
wire-format bug cannot hide behind a different on-disk representation.

MCAP chunk compression is set to zstd. This is a FILE-SIZE decision with no effect on wire
byte accounting: bytes are counted at the serializer (§4.4), and a bag is storage rather
than a link. Classes 4-6 arrive already compressed, so the additional gain falls mostly on
the uncompressed classes.

Log time comes from the message stamp, never a wall clock. A wall clock would make every
bag unique and destroy the reproducibility §5.9 exists to provide. Asserted by
`LogTimesComeFromMessageStampsNotWallClock`.

Measured: bags written twice from identical inputs are **byte-identical**, which is stronger
than the Phase 0 gate requires (identical metrics) and is what makes a bag checksum in a
§28.2 manifest meaningful. Also measured: replaying through MCAP reproduces the record-file
gate exactly (ATE 0.0633155, RPE 0.0174459, 8626 wire bytes), so adopting the container
costs no determinism.

LZ4 is disabled (`MCAP_COMPRESSION_NO_LZ4`): nothing selects it and vendoring a codec we
would never use adds a dependency for nothing.

## 2026-09-08 — D17: Philox counter is PARTITIONED, and `site` is bounded to 48 bits

`PhiloxStream` mixed its refill block index in with `ctr.v[3] ^= block`, while the
constructor wrote `site >> 32` into that same word. The two therefore aliased:
`(site=0, block=1)` and `(site=2^32, block=0)` produced an identical counter and
byte-for-byte identical output. The code comment claimed a site could draw
"without ever colliding with a neighbouring site's stream", which was false.

Philox4x32's counter is 128 bits and has to hold `frame_id`, `site` and `block`,
so one of the three must be bounded — there is no layout that gives all three
64 bits. The options considered:

| Layout | Cost | Verdict |
|---|---|---|
| Steal `block` from `frame_id`'s high word | `site` keeps 64 bits | **rejected**: `kFixedPatternFrameId` is the all-ones sentinel, so `frame_id` genuinely uses all 64 bits |
| Bound `block` only | would cap draws per site very low | rejected |
| **`site` < 2^48, `block` < 2^16, `frame_id` keeps 64** | 262144 words per site | **chosen** |

`SiteId` for a 1280x720x3 image with 4 noise stages peaks around 11M, four orders
of magnitude inside 2^48. `LatticeNoise` hashes a 2D cell to 64 bits and is now
masked to 48; among the ~10^3-10^4 cells one image touches the birthday collision
probability in 2^48 is ~1e-11, and a collision would mean two lattice cells sharing
a value, not a correlated stream.

The map from `(frame_id, site, block)` to the counter is injective on the declared
domain. **Corrected same-day:** the first version of this entry claimed the bounds
were "asserted, so the guarantee is structural". `assert` is compiled out under
`NDEBUG`, which this project's `Release` build defines -- so in the shipped binary
neither bound was checked, `site = 2^48` aliased onto `site = 0`, and block 65536
wrapped onto block 0 and reused the stream. That is the same defect class the entry
was written to close, reintroduced by its own fix. Now:

- `site` is **masked in the constructor** in every build. It is a documented 48-bit
  identifier; passing more bits is defined truncation, not an aliasing surprise.
  Callers holding a 64-bit id (Phase 1 keyframe/landmark/submap hashes) must use
  `SiteFromHash64()`, which mixes all 64 bits down with a splitmix64 finalizer so
  the high bits still matter.
- Block exhaustion **throws**, in every build. Wrapping would silently repeat a
  site's numbers, and a determinism substrate that quietly repeats itself is worse
  than one that stops. Cost is one comparison per four draws.

Note the asymmetry that hid this: `check_determinism.sh` compiles with bare
`-O0/-O2/-O3` and no `-DNDEBUG`, so asserts are live in the gate and dead in the
release binary. The gate structurally could not have caught it. This matches how Random123-derived APIs partition counter words
(JAX's `fold_in`/`split`, cuRAND's Philox usage) rather than XOR-mixing them.

Consequence: `philox_digest` and `degrade_digest` changed. Nothing published
depended on the old values.

## 2026-09-08 — D18: golden digests must evaluate the boundaries they claim to guard

`DetExp` returned `inf` for x in [709.44, 709.78] and `0` for x in [-745.13, -745.0]
— inside its own guard, where the true results are finite and representable. The
range reduction produces k=1024 (resp. -1075) there and `Exp2Int` saturates. Fixed
by splitting the scaling so neither factor leaves the normal range; now within
1 ulp of libm across both bands.

The decision worth recording is not the fix but what it exposed. After fixing both
`DetExp` and Philox, **two of the three golden digests were unchanged**:

- `det_math_digest` swept x in [-500, 500] only — the bug lived outside it entirely.
- `philox_digest` used sites < 2^32 drawing 2 words each, so `site >> 32` and `block`
  were both always zero — it never varied the fields that aliased.

Both digests were byte-identical across genuinely broken code, and would have stayed
green through a release. **A digest guards only the inputs it evaluates.** The sweeps
in `check_determinism.sh`, `test_det_math.cpp` and `test_philox.cpp` now include the
overflow/underflow boundaries and high-site/multi-block draws respectively.

Standing rule from this: when a function's contract names a domain, the digest
sweep must reach the ends of that domain, not just its comfortable interior.

## 2026-09-08 — D19: CI determinism gate must not be piped

`run: ./infra/scripts/check_determinism.sh | tee determinism-$arch.txt` was green on
failure. GitHub's default shell is `bash -e {0}` **without** `pipefail`, so the step
exited with `tee`'s status; verified locally that even a compilation failure inside
the script yielded exit 0. Replaced with redirect, capture status, `cat`, `exit`.
`if: always()` added to the artifact upload, since the log matters most when the gate
failed.

Related correction: `cross-arch-determinism` was commented as "THE JOB THAT MATTERS".
With per-arch assertion working it cannot fail — both arches are compared against the
same committed constants, so reaching the job implies they agree. It is now described
as a second, independent comparison, runs with `if: always()`, and the script emits a
machine-readable `det=... philox=... degrade=...` line on **both** its pass and fail
paths so two failing runs can still be diffed. Previously the failure path printed no
`=` at all, so the extracting `grep` matched nothing and the job died on an empty
match rather than reporting the mismatch.

## 2026-09-08 — D20: `poisson_exact_max_lambda` is bounded at both the config and the sampler

The field was config-exposed and unvalidated. Knuth's exact branch is hard-capped at
10000 iterations; setting the threshold high enough would make every bright pixel
return exactly 10000 photons — a plausible-looking image built on fabricated
statistics, with no test failing.

Bounded to (0, 50] in `ValidateDegradation`, and clamped again inside `SamplePoisson`
for callers that do not come through a config. The Gaussian approximation is already
accurate by lambda=30, so nothing is lost. With the clamp, `target = e^-lambda > 1e-22`
and the 10000-iteration cap is unreachable short of a broken RNG, which turns it from
a routine truncation path into a genuine guard.

`dust_field_frequency` is now required `> 0` for the same reason: zero maps every pixel
onto lattice cell 0, so the field silently becomes a spatial constant while still
reading like a field in the config.

## 2026-09-08 — D21: manifest required-field set now covers provenance, not just identity

`MissingRequiredFields` returned empty for a manifest whose `seed`,
`degradation_config`, `execution_mode` and all three determinism digests were
`UNRECORDED`. §5.6 requires degradation parameters to be named configs *referenced by
run manifests*, and the digests exist "so a result produced by drifted math is
identifiable after the fact" — both guarantees were empty as written.

All six added to the required set. `seed` needed a separate `seed_recorded` flag
because 0 is a legitimate seed and the value alone cannot distinguish "seeded with 0"
from "nobody recorded it".

## 2026-09-08 — D22: `EventLog` chain hash must not use zero as a sentinel

`ChainHash` began with `state_hash_ != 0 ? state_hash_ : kFnvOffsetBasis`, using
zero to mean "no previous event". Zero is a legitimate hash value, so a chain that
happened to produce it would silently RESTART, and two different histories could
converge onto the same hash from then on. That destroys the property the chain
exists for: localizing WHERE two runs diverged, not merely that they did.

Replaced with a `chain_` member seeded at construction and always carried forward.
`state_hash_` still reports 0 before the first event, so the public surface is
unchanged. Same bug shape as treating `seed == 0` as unset in the run manifest
(D21), which is why both were fixed in the same pass.

## 2026-09-08 — D23: framing is charged per DATAGRAM, not per message

Arch §15.7 (line 2699) specifies "28 B emulated UDP/IP header **per datagram**".
The serializer charged it once per message, so any message exceeding the path MTU
was under-billed. Doc wins per arch §0.2.

Added `kPathMtuBytes = 1500`, `kDatagramPayloadBytes = 1472` and `DatagramCount()`;
`wire_bytes = framed + 28 * DatagramCount(framed)`.

Measured effect — it engages exactly where the byte table is load-bearing:

| Message | framed | datagrams | wire before | wire after |
|---|---|---|---|---|
| pose graph 200n/40e (compressed) | 4266 | 3 | 4294 | **4350** |
| Phase 0 gate, 4 messages total | — | — | 8647 | **8759** |

Small messages are unaffected (one datagram, 28 B, as before). The correction is
~1.3% on the compressed pose graph, so it does not threaten §15.7's <=15%
prediction-error gate — but the Phase 3 spectral policy optimizes
`gain / predicted_cost`, and systematically under-pricing the largest classes is
exactly the bias that would distort which messages it chooses to send.

Checked for a second mismatch, since Phase 3's gate measures against §15.7's table:
**the table is payload-only.** Its rows are encoding arithmetic with no framing term
at all (200 nodes x 28 B + 40 edges x 150 B = 11600 B, the printed "~12 KB"), and the
28 B appears only in the formula line, which already says "per datagram". So the fix
aligns the code with both the formula and the table; it does not trade one mismatch
for another.

Assumption recorded: 1500 B Ethernet MTU with no jumbo frames, and IPv4 (20 B)
rather than IPv6 (40 B). Both are `[ASSUMPTION]` and should be revisited when the
real link model lands in Phase 3.

## 2026-09-08 — D24: `execution_mode` is validated by membership, not presence

`MissingRequiredFields` called `require("execution_mode", ...)`, but the field
defaults to `"replay"` -- non-empty and not `UNRECORDED` -- so the check could never
fire. It read as a guarantee and verified nothing.

§5.9 makes the distinction load-bearing: replayed runs are bit-deterministic, live
closed-loop runs are not and must be reported over seeds. What matters is therefore
that the value is one of the two known modes, which is now what is checked.

**Follow-up for Phase 1:** nothing in the tree populates the six newly-required
manifest fields yet. Whoever writes the first real manifest writer must wire
`seed_recorded`, `degradation_config` and the three determinism digests, or the
first real run will correctly report itself unpublishable.

## 2026-09-29 — D25: sim image installs the ros_gz bridge/sim subset, not the `ros-jazzy-ros-gz` metapackage

Arch §5 says "install `ros-jazzy-ros-gz` for the bridge and sim launch integration". Measured
2026-09-29: the metapackage pulls `ros_gz_image -> image_transport_plugins -> cv_bridge`, which
installs **Ubuntu's OpenCV 4.6 (43 packages)**. That puts a second OpenCV in `ariadne-sim`, against
C7's "one OpenCV, built from the pinned tag", and makes a sim-image manifest's `opencv_build_hash`
ambiguous.

The image installs `ros-jazzy-ros-gz-bridge`, `ros-jazzy-ros-gz-sim` and `ros-jazzy-ros-gz-interfaces`
instead: **zero** OpenCV packages, and still gz-sim8 8.15.0 / gz-rendering / gz-sensors / sdformat14 /
ogre-next. This serves the doc's stated purpose (bridge + sim launch). Camera frames cross as
`sensor_msgs/Image` through `parameter_bridge`, which needs no OpenCV. What is lost is
`ros_gz_image`'s image_transport compression plugins, which Ariadne is not planning to use.

**Open for Phase 1.3 `[DESIGN GAP]`:** rosbag2 records CDR-serialized `sensor_msgs/Image`
(verified in the image), but `McapRecordReader` (D3) replays Ariadne-framed FlatBuffers on
`/ariadne/wire`. Nothing converts between the two yet. 1.3 must decide whether sim bags stay
rosbag2/CDR and a ROS-free CDR reader is added, or a recorder node writes the Ariadne wire form
directly. The Dockerfile fails the build if any OpenCV package
ever enters the image through a transitive dependency.

Also recorded: ROS packages come from the dated snapshot `snapshots.ros.org/jazzy/2026-09-11`,
signed by "ROS Snapshot builder <rosbuild@ros.org>", fingerprint
`4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA`. The fingerprint was verified against keyserver.ubuntu.com
and against the snapshot's own InRelease signature, and the key is vendored at
`infra/docker/keys/ros-snapshots.asc` with a build-time fingerprint check. The live ROS repo keeps
only the newest build of each package, so `pkg=version` pins break on its next sync. The snapshot
does not move.

## 2026-09-29 — D26: AWS runs on the free plan, ~$100-200 of credits, in one standalone account

The grant request did not go through. The account (`836505524290`, `ap-southeast-2`) is on the
AWS **free plan**. Consequences, from AWS's own documentation:

- **Never join AWS Organizations or set up Control Tower.** Either upgrades the account to paid
  and **expires the free credits immediately** (Free Tier FAQ). This rules out org-instance IAM
  Identity Center, and with it SSO permission sets. The account-instance Identity Center that was
  created does not support permission sets (IAM Identity Center docs) and is unused.
- The free plan **closes the account** when credits run out or after 6 months (data kept 90
  days), and excludes "certain AWS Marketplace offers that can incur charges" — which may include
  the COOL AMI. `[UNVERIFIED]`: attempt the COOL subscription to find out.
- Credits: $100 at signup, up to $100 more from activities.

Setup chosen (standard for a standalone account):

| Concern | Choice |
|---|---|
| Human access | IAM user + MFA; `aws login --remote` (CLI 2.37.5) for short-lived credentials, no access keys |
| Terraform credentials | provider cannot read the `aws login` cache (terraform-provider-aws #45316); `infra/scripts/aws.sh` exports current short-lived credentials into a mode-600 env file per run |
| CI | GitHub OIDC -> IAM role scoped to ECR push on the project repos. Zero stored secrets |
| State | S3 bucket `ariadne-tfstate-836505524290` (versioned, SSE, TLS-only, public access blocked), `use_lockfile = true`, Terraform >= 1.11; bootstrapped with local state then migrated |
| Tooling | official `amazon/aws-cli:2.37.5` and `hashicorp/terraform:1.16.0` images, pinned by digest |

**OIDC subject.** GitHub repos created after 2026-07-15 use the immutable subject
`repo:OWNER@OWNER_ID/REPO@REPO_ID:ref:...` (GitHub OIDC reference). The owner id (`95163434`) is
pinned. The repo id is wildcarded because the repository is private and its id was not readable
without credentials. Tighten it once the first CI run shows the actual `sub` in CloudTrail.

**Budget pulled forward from Phase 7.** The account closing on credit exhaustion would take the
bags, ECR images and judging console with it. The budget uses `include_credit = false`: a cost
budget nets credits by default, which would hold it at $0 so it never fires. Thresholds are 25/50/80%
actual plus 100% forecast, and the limit is a variable (100 now, 200 after activities).
`[UNVERIFIED]` whether Budgets sees gross spend on the free plan; check after the first billable run.

**The budget is the backstop, not the brake.** Budget data lags by hours, and on a GPU instance
the credits are gone before the email arrives. Every Phase 7 launch template MUST set
`instance_initiated_shutdown_behavior = "terminate"` and a max-lifetime shutdown in user-data.

**Locked-decision tension, open.** arch §0.2 puts Gazebo on g5/g6. That may not fit
the credit pool. Record-once rendering could move to local Mesa llvmpipe (proven at 320x240 / 5 Hz,
unmeasured at 1280x720 stereo), but gsplat needs CUDA regardless. It is decided once real
prices are pulled.

**D26 addendum (2026-09-29), credit activities done.** All five Free Tier credit activities were
COMPLETED and the balance is **$200.00** (FREE plan). EC2, Budgets, Lambda and RDS were done via the CLI;
completion is tracked by API calls, not console clicks. Bedrock was done in the console playground.
All temporary resources were deleted and the account verified empty of billable resources.
Two follow-ups:
- The budget `ariadne-free-credits` ($100, gross spend, 25/50/80% + 100% forecast) was created via CLI.
  Before the main stack's first apply: `terraform import module.budget.aws_budgets_budget.credits
  836505524290:ariadne-free-credits`, and raise `credit_budget_usd` to 200.
- Bedrock `converse` via the API returned `Operation not allowed` / `authorizationStatus: NOT_AUTHORIZED`
  for Nova Micro on the free plan, although the model is region-available. The Phase 7 orchestrator
  needs API access: re-test after the plan upgrade. `[UNVERIFIED]` cause.

## 2026-09-29 — D27: tests-only AWS usage; cheapest configurations that run each workload

The account is on the PAID plan with $200 of credits (upgraded via the console button, so the credits
were kept). Graviton4 launches were verified after the upgrade. AWS is used ONLY for short, self-terminating
test runs; development, CI and simulator bag rendering stay local.

| Workload | Config | Sydney $/h (AWS price data 2026-09-29) | Quota |
|---|---|---|---|
| COOL / Arm CPU pipeline | **c8g.large** (Graviton4, 2 vCPU, 4 GiB) | 0.1038 on-demand, + $0.01 COOL software fee | Standard 5 vCPU: fits |
| GPU: gsplat, NVIDIA EGL rendering | **g4dn.xlarge spot** (T4 16 GB, 4 vCPU) | 0.2622 spot (0.684 on-demand) | G/VT 0: **request 4** |

Why these and not smaller:
- GPU quotas count vCPUs, and g4dn.xlarge (4 vCPU) is the smallest standard NVIDIA type with a full GPU.
  The only 2-vCPU GPU, g6f.large (1/8 L4), costs the same ($0.2626 on-demand) but has ~2.8 GiB VRAM and
  needs GRID drivers with unverified headless EGL. That's the wrong trade for gsplat.
- c8g.large is the smallest size on COOL's published price list (c8g.medium is not among the visible
  dimensions).

**Locked-decision tension, open.** arch §0.2 / Invariant 2 name **m8g.4xlarge** for the
COOL benchmark, and §27.1's concurrent-pipeline "knee" sweep is not meaningful on 2 vCPUs. c8g.large is
right for development and smoke runs. Sizing the FINAL benchmark is a separate decision. m8g.4xlarge needs
16 vCPU, above the current Standard quota of 5, so a request would be needed.

**Biggest cost unknown:** whether local Mesa llvmpipe renders 1280x720 stereo at a usable rate. If it does,
the GPU is needed for gsplat only. Measuring it is free.

Mechanism: `infra/scripts/spinup.sh run --profile cpu|gpu -- '<cmd>'`. It uses spot by default; on-demand only via
an explicit flag, never as a silent fallback. There are three independent brakes: (1) `shutdown -h +MAX` as the
first user-data line, with shutdown meaning terminate; (2) a host trap that terminates on exit/Ctrl-C; (3) `sweep`,
which terminates stray instances past their deadline and lists unattached volumes. Root volumes are forced to
`DeleteOnTermination=true`. IMDSv2 is required, with hop limit 2 so containers can reach the instance role. The
role can only read/write `runs/` in the artifact bucket and pull from the project's ECR repos. Debugging uses SSM,
never SSH. Every run appends type, market, AZ, duration, the actual spot price and cost to `build/runs/ledger.csv`,
which is also the cost evidence the COOL rubric asks for.

~~`[UNVERIFIED]` until the first real run~~ **Verified 2026-09-29** (run `20260929T145114Z-cpu-762cb1`): AWS accepts
spot + `instance-initiated-shutdown-behavior=terminate`, and an in-instance `shutdown -h` ends in `terminated`
(reason `Client.InstanceInitiatedShutdown`), not `stopped`. Two silent-exit bugs were found on the way and fixed
(a `read` at EOF-without-newline under `set -e`, and a SIGPIPE-prone `sort | while break`). The script now has an
ERR trap that names the failing line.

Open: the Bedrock API (`converse`) still returns `Operation not allowed` after the upgrade and as the IAM user
(so neither the free plan nor root identity is the cause). Needed by the Phase 7 orchestrator; to investigate.

## 2026-09-29 — D28: region is eu-north-1 (Stockholm), the cheapest one that meets every need

Requirements, all in ONE region (cross-region traffic would distort cost-per-frame): c8g.large (Graviton4,
COOL), g4dn.xlarge (T4 16 GB), the COOL Graviton4 AMI, and Bedrock with Anthropic models.

Method: all 18 enabled regions scanned via the live API (offerings, current spot, Bedrock model lists).
Then 7-day spot history (median / p90) for the leaders, plus on-demand from AWS's published price data.

| Region | c8g.large spot 7d median | g4dn.xlarge spot 7d median (p90) | on-demand c8g / g4dn |
|---|---|---|---|
| **eu-north-1** | **0.0206** | **0.0814** (0.0899) | 0.0851 / 0.558 |
| sa-east-1 | 0.0342 | 0.0973 | 0.122 / 0.894 |
| ap-south-1 | 0.0262 | 0.2121 | 0.054 / 0.579 |
| eu-west-3 | 0.0446 | 0.1906 | 0.094 / 0.615 |
| ap-southeast-2 (previous choice) | ~0.046 | ~0.296 | 0.104 / 0.684 |

ap-south-2 (Hyderabad) has the cheapest Graviton spot (0.011) but no g4dn at all, so it's excluded.
ap-south-1 (Mumbai) has the cheapest on-demand Graviton: if the final COOL benchmark runs on-demand on
m8g.4xlarge, Mumbai is $0.51/h against Stockholm's $0.76/h. Worth revisiting for that single run.

- **COOL availability:** the listing page shows no region list, but Marketplace AMIs are public EC2 images.
  Product code `aajkmdd4qo3r7yhqg61a7aah9` (image `Graviton5-COOL-v3-prod-k6u24vxijyzvg`, where
  `prod-k6u24vxijyzvg` is the Graviton4 listing) is published in **all 18 regions**; in eu-north-1 it's
  `ami-0a302ff85af8d8764`. (The earlier "0 images" result came from a case-sensitive name filter guess.)
- **Spot placement scores were discarded.** AWS returns an artificially low score when fewer than three
  instance types are specified, and the query named one type. Real capacity is proven only by a real launch.
- **Bedrock "Operation not allowed" explained:** in eu-north-1 the same call returns "Your account is
  currently being verified ... normally takes less than 2 hours". That's post-upgrade account verification,
  not a configuration problem.
- **Absolute savings are small** (a few dollars against $200 on tests-only use). The region choice matters less
  than spot capacity and single-region co-location.
- **Quotas are per region:** eu-north-1 has Standard 5 vCPU, and G/VT 0 for both on-demand and spot.

## 2026-09-29 — D29: minimal network: the region's default VPC, one egress-only security group

Requirement: the network is to be as minimal as possible to keep costs down. The main stack no longer
creates a VPC. It uses eu-north-1's **default VPC** (172.31.0.0/16, one public subnet per AZ, internet
gateway), which every region ships with, free and already present. The only network object Ariadne
creates is `ariadne-test-runner`, an egress-only security group with no inbound rules; debugging goes
through SSM, not SSH.

Deliberately not used: NAT gateways (~$0.045/h each, the classic silent cost), Elastic IPs, interface
endpoints, and a custom VPC. Runners use their auto-assigned public IPv4 (billed only while an instance runs).
`modules/network` is kept, unused, in case the Phase 7 hosted console justifies its own VPC.

`spinup.sh` launches spot runs into the AZ with the **lowest current spot price** (it reads all three
AZs before launch), since spot prices differ per AZ.

Applied 2026-09-29: bootstrap (6 resources; state migrated to `s3://ariadne-tfstate-836505524290/bootstrap/`)
and main (1 imported, 23 added, 1 changed, 0 destroyed; post-apply plan converged). Verified by that
apply: AWS accepted the GitHub OIDC provider without a `thumbprint_list`. The CLI-created budget was
adopted with a declarative `import` block, because `terraform import` failed on the default-subnet
`for_each`. The block was removed after the apply.

GPU quota requests filed in eu-north-1 (quotas are per region): G/VT spot and on-demand to 4 vCPU,
both PENDING at filing.

## 2026-09-29 — D30: a GPU is needed ONLY for gsplat; the GPU quota is 0 and the spot request was denied

- **Quota, authoritative:** `Running On-Demand G and VT instances` and `All G and VT Spot Instance Requests`
  are both applied **0** in eu-north-1, and the AWS default is also 0. The account has no GPU access today.
  **Both requests were denied** on 2026-09-29 with "ramp up gradually": case 179069316400187 (spot, 4 vCPU)
  and case 179069316600303 (on-demand, 4 vCPU). The quota API still reported both as `CASE_OPENED` after the
  denial emails, so the API status lags the support case; the applied value (0) is what counts. The appeal
  text for both cases is in `docs/architecture.md`.
- **What needs a GPU:** gsplat (CUDA-only). The Phase 1 exit gate includes "render a splat", which puts a GPU
  on Phase 1's critical path, not just Phase 6's.
- **What does not:** bag rendering (validation.md §4f: software rendering records the full rig
  completely at RTF 0.57), SLAM, COOL, Bedrock.
- arch §0.2 places Gazebo on g5/g6. §4f shows that is not necessary for record-once bags, so the GPU
  requirement shrinks to gsplat alone. The locked decision is not changed here; this only records the evidence.
- Fallbacks if both cases fail (e.g. off-AWS GPU time for gsplat) conflict with the locked "gsplat service on
  AWS g5/g6" and need a separate decision.

## 2026-09-29 — D31: §30.2 becomes a code registry; rotational ATE added; "rotation/translation error" interpreted

`cpp/runtime/metrics/metric_registry.hpp` holds the 27 §30.2 metrics (4 localization, 3 registration, 6 mapping,
7 communication, 7 mission) as stable ids with units. `MissingMetrics()` treats absent AND non-finite values as
missing, since "emits a number, however bad" must not be satisfiable by a NaN. `UnknownMetrics()` rejects
unregistered ids, so a typo is an error rather than an orphaned value. The Phase 1 exit gate becomes
`MissingMetrics(report).empty()` on the thin run's report.

`[ASSUMPTION]` §30.2's localization line ("ATE RMSE, RPE, rotation error, translation error") does not say which
quantity each error refers to. It is read the way evo reports trajectories: translation and rotation parts of
both ATE and RPE (`ate_rmse_m`, `ate_rotation_rmse_rad`, `rpe_translation_rmse_m`, `rpe_rotation_rmse_rad`).

`ComputeATERotation` closes the audit carry-over. It uses the same position-based Umeyama alignment as
`ComputeATE` (factored into `AlignTrajectories`, so the two can never describe different registrations), then
takes the angle of R_gt^T·(R_align·R_est) per pose, which is evo's APE rotation part. The test that motivates it:
positions exact, every orientation off by 0.05 rad gives translation ATE = 0 and rotation ATE = 0.05.
`energy_proxy` carries no invented unit; §30.2 gives none, and its definition lands with the planner (4.3).

## 2026-09-29 — D32: determinism digests exist once, and the run manifest measures them

Each digest (det_math, Philox, degradation) previously existed twice, once in `check_determinism.sh`'s inline C++
and once in a test, and the two agreed only because they were kept in sync by hand. They now live once:
`core/determinism/digests.hpp` (`DetMathDigest`, `PhiloxDigest`, `Fnv1a64`) and
`perception/degradation/degradation_digest.hpp` (`DegradeDigest`). The tests, the script, and the run manifest
all call them, and the D18 boundary-coverage rationale moved with them.

`runtime/manifest/determinism.hpp::RecordDeterminismDigests(RunManifest&)` fills the manifest by MEASURING
the digests at run time. Copying the committed constants would make D21's "a result produced by drifted math is
identifiable after the fact" empty again: the manifest would claim the recorded values whatever the host
actually computed. `ariadne_runtime` now links `ariadne::perception`, which is allowed because runtime sits above
the domain libraries in the §3 dependency order.

Verified: 156 tests pass; `check_determinism.sh` PASS on 6/6 configurations using the shared code; committed
constants unchanged.

## 2026-09-29 — D33: the ariadne-core image, and what the OpenCV build hash does and does not identify

`infra/docker/core.Dockerfile` builds FROM `opencv-base` (tag locally; CI passes a reference through the
`OPENCV_BASE` ARG, by digest once ECR push is wired). Two stages: `dev` (toolchain only; the repo is bind-mounted
and built into `build/docker/`, never the host's `build/`) and `ci` (sources bind-mounted read-only for the build step so they never enter an image layer, configured with
`ARIADNE_REQUIRE_OPENCV=ON`, full `ctest` run inside the build, `ariadne_identity` installed). ONNX models from the
§27.1 row are deliberately absent until XFeat (Phase 2) consumes one. `infra/scripts/core.sh image|gate|dev` is
the only entry point; `docker-bake.hcl` builds `opencv-base` then `core` in one CI invocation with separate GHA
cache scopes.

OpenCV stays optional on the host: `find_package(OpenCV 5 CONFIG QUIET PATHS /opt/opencv5 NO_DEFAULT_PATH)`, and
OpenCV-dependent code lives in targets that exist only when it is found (`ariadne_runtime_opencv`,
`ariadne_identity`, `test_opencv`). `core`, the degradation operator, replay and metrics remain OpenCV-free.

**The build hash identifies a build artifact, not a configuration.** `cv::getBuildInformation()` contains
`Timestamp:` and `Host:` lines, so its hash (`fnv1a64:<hex>`, via the existing `core::Fnv1a64`) changes on every
rebuild of the same tag and differs between amd64 and arm64. No golden value of it is ever committed. It is
measured at run time from the LOADED `libopencv_core`, which is what makes it useful for COOL: the one binary run
against `/opt/cool` records a different hash than against stock, so every manifest says which OpenCV produced it.
The Phase 0 gate 4 check is therefore "the hash a manifest records at run time equals the hash baked into the
image at build time by the same binary" (`core.sh gate`).

**Loader linkage protects Invariant 2.** DT_RPATH is searched before `LD_LIBRARY_PATH`, so a binary carrying
`RPATH=/opt/opencv5/lib` would ignore the COOL loader switch and benchmark stock OpenCV against itself. The image
resolves OpenCV through `/etc/ld.so.conf.d` (searched after `LD_LIBRARY_PATH`), builds with `CMAKE_SKIP_RPATH=ON`,
and fails the build if `readelf -d` finds RPATH or RUNPATH on any installed binary or test. (The stock OpenCV
libraries themselves carry RUNPATH, which `LD_LIBRARY_PATH` does override.) Open for 0.3: confirm the COOL
libraries use the same sonames (`libopencv_core.so.500`).

**OpenCV outputs are not cross-machine golden values.** OpenCV compiles several SIMD variants of hot kernels and
selects one per call from the running CPU (the dev box dispatches up to AVX512_SKX). The manifest gains
`opencv_cpu_features` (`cv::getCPUFeaturesLine()`) so a cross-machine difference can be attributed. Consequence for
the thin-run determinism item: OpenCV-stage determinism is defined run-vs-run on the same machine; only the
OpenCV-free digests (D18/D32) are committed as cross-machine constants.

`image_digest` currently holds the LOCAL image ID (the config digest, `docker image inspect .Id`). After an ECR
push the registry manifest digest is a different value; when push is wired, manifests must record the registry
digest, and the two kinds must not be compared.

Also: `.dockerignore` now excludes `*.tfvars`, Terraform state and `.venv`, since the core build context is the repo
root and `terraform.tfvars` holds the alert email. `opencv-base` still uses an unpinned `FROM ubuntu:24.04`; pinning
it forces an OpenCV rebuild, so it is deferred to the 0.2 arm64 multi-arch rebuild that needs one anyway.

Verified (amd64, 2026-09-29): the `ci` stage builds and passes **160/160** tests inside the image (the 156
OpenCV-free tests plus 4 `OpenCvIdentity` tests), so the committed D32 digests also hold on the image's gcc 13.3.
`core.sh gate`: baked and manifest-recorded hash both `fnv1a64:cf8ea084bb984703`, `CORE_IMAGE_GATE=PASS`. The
RPATH detector was checked against binaries deliberately linked with RPATH and with RUNPATH (both detected);
`LD_DEBUG=libs` shows `libopencv_core.so.500` resolved from `/opt/opencv5/lib` via the loader cache. Not yet
demonstrated: a hash CHANGE under a different OpenCV (needs a second build, e.g. COOL at 0.3). The first image
build also exposed a latent bug: `run_manifest.hpp` used `uint64_t` without `<cstdint>` and compiled only through
transitive includes; fixed, and every header under `cpp/` now compiles standalone.

## 2026-09-29 — D34: placeholder perception front-end (1.4), and a near-range stereo hazard

Built from OpenCV 5 primitives with the algorithm logic owned by Ariadne (§8.1's split), following the reference
implementations read for it (VINS-Fusion `feature_tracker.cpp`, OpenVINS `TrackKLT.cpp` / `Grider_GRID`,
ORB-SLAM3 `ORBextractor`) and the papers behind them (Rublee 2011 ORB; Shi & Tomasi 1994; Bouguet's pyramidal LK;
Kalal 2010 forward-backward error; Hirschmueller 2008 SGM). API names verified against the installed 5.0.0
headers: `features.hpp` (ORB), `video/tracking.hpp` (`buildOpticalFlowPyramid`, `calcOpticalFlowPyrLK`),
`stereo.hpp` (`StereoSGBM`), `geometry/3d.hpp` (`findFundamentalMat`, `UsacParams`).

- **`OrbEngine`** (§7.1 interface, with `Detect` split from `DetectAndCompute` so tracking never pays for
  descriptors): ORB detect oversampled 4x; cells left starved re-detected at a lower FAST threshold through ORB's
  mask argument (ORB-SLAM's iniThFAST/minThFAST); our own `GridSelect` keeps top-k per 8x6 cell (§7.3) with a
  total tie-break order; descriptors computed on exactly the selected set. ORB's internal `nfeatures` cap is not
  the selection, because it is a global top-N, which §7.3 forbids.
- **`KltTracker`**: one pyramid per frame reused for the forward, backward and next-frame passes; forward-backward
  check at 0.5 px; border check; epipolar rejection; VINS-style spacing mask with older tracks winning;
  replenishment through the §7.1 engine (so the XFeat swap changes tracking too); `DEGRADED` below a configured
  track count.
- **Determinism:** `findFundamentalMat`'s `FM_RANSAC` path samples from `cv::theRNG()`, per-thread state that any
  earlier OpenCV call advances. The tracker uses the `UsacParams` overload with `isParallel = false` and
  `randomGeneratorState` drawn from Philox at `(seed, frame_id, reserved site)`. Verified: tracks, ORB descriptors
  and SGBM disparity are bit-identical across 1, 2 and 8 OpenCV threads and across arbitrary `cv::theRNG()` state.
- **§9.6 deviation:** `StereoDepth::disparity` is SGBM's native `CV_16S` (1/16 px), not fp16. fp16 spacing is
  0.125 px above 128 px, i.e. coarser than SGBM's output for every depth under 0.5 m at f = 640 px, B = 0.10 m.
  `CV_16S` is exact and also 2 bytes. Z and sigma_Z are computed in float32, then stored as fp16. `confidence` is an
  all-zero `[PLACEHOLDER]` flagged by `confidence_is_placeholder` until the WLS filter (Phase 2).
- **§9.4 validator** implemented (`ValidateStereoAgainstRig`), refusing e.g. B = 0.20 m at 256 disparities.
- Parameters live in `configs/perception/placeholder.yaml`, recorded per run in the new manifest field
  `perception_config` (§7.3: manifest fields, not constants). The OpenCV tests load that shipped file rather than
  struct defaults, so the tests exercise what runs.

**Approved 2026-09-29:** the §9.6 `CV_16S` disparity and the §7.1 `Detect`/`DetectAndCompute` split; both are
mirrored into the architecture doc.

**Hazard found: SGBM aliases surfaces nearer than its search range into phantom FAR depth.** A textured plane at
0.20 m (true disparity 320, beyond the 255 searched) still yields "valid" pixels: 1.38% of the image at uniqueness
10 and speckle window 100. The reported depths spread from 0.3 m to past 3 m rather than clustering at the range
limit, so **they cannot be recognised by their value**. A consumer that carves free space along depth rays would
carve it straight through a real obstacle: an error in the unsafe direction for egress.

What does separate them was measured rather than assumed (probe over planes at 1.5, 0.30, 0.26 m true and 0.20,
0.15, 0.10 m aliased):
- **Structure:** aliases form ~48 blobs of 116-312 px; true surfaces form one connected region.
- **Speckle window:** 100 lets those blobs through; 200 (top of §9.3's 100-200) cuts aliased pixels from 6518 to
  720 / 215 / 0 at 0.20 / 0.15 / 0.10 m, with no loss of true depth on the planes. Set to 200. `uniqueness_ratio` is
  set to 15 (top of §9.3's 10-15), which halves the rate relative to 10.
- **Local density:** the fraction of valid pixels in a 31x31 window is >= 0.84 for true depth (1st percentile;
  median 1.0) and <= 0.42 for the surviving aliases. `DenseSupportMask` (window 31, density >= 0.75, configured in
  `stereo.support_*`) passes > 90% of true depth and none of the aliases (tested).

Hand-off rule, testable: **1.8's occupancy placeholder carves free space only along rays whose depth is in
`DenseSupportMask`.** Unsupported valid depth may mark occupancy but never free space. **Superseded in part by
D35:** on real EuRoC texture the gate catches 87% of aliases, not all of them, so 1.8 must also choose between
"stereo never carves free space" and a near-field pass. `[ASSUMPTION]` These numbers
come from synthetic random-texture planes. Real scenes have genuinely sparse true depth (low texture, occlusion
edges), where the gate discards true depth, which errs in the safe direction. They must be re-measured on EuRoC,
and Phase 2's WLS confidence plus a near-field check (e.g. a reduced-resolution pass whose effective range covers
the blind zone) replaces the gate before any egress claim rests on stereo free space.

Measured (amd64, 8 threads, 1280x720 synthetic texture): 697 initial tracks; 689 survive a (3.3, -1.7) px shift
with median error 0.013 px (p95 0.028 px); dark-half keypoints 33 -> 600 with the low-threshold rescue; SGBM valid
fraction 0.80 = every non-blind column. Timings: pyramid 0.5 ms, tracking 18 ms/frame, ORB 48 ms/keyframe, SGBM
235 ms/keyframe. Known shortfall: the first frame reaches 697 of the 1200 target tracks (20 px spacing with ORB's
clustered corners); later frames reach about 1035.

Not yet done for 1.4 (Invariant 5, datasets before simulator): a EuRoC sequence through rectification
(`stereoRectify` / `initUndistortRectifyMap` / `remap`) and this front-end.

## 2026-09-30 — D35: EuRoC adapter, rectification, and the 1.4 real-sequence run

**Source and licence.** EuRoC MAV (Burri et al., IJRR 2016), DOI 10.3929/ethz-b-000690084. The old ASL page now
redirects to ethz-asl.github.io, whose EuRoC page points to the ETH Research Collection. That collection's HTML
frontend returns HTTP 500 or 429, but its DSpace REST API works (`/server/api/pid/find?id=hdl:20.500.11850/690084`
-> bundles -> bitstreams). It lists one zip per environment: `machine_hall.zip` is 12.7 GB and holds each MH
sequence as a ROS bag plus an ASL-format zip. **Licence: "In Copyright - Non-Commercial Use Permitted"** (dc.rights,
rightsstatements.org InC-NC 1.0). The competition licenses every *submitted* material to OpenCV and AWS "for any
lawful commercial or noncommercial purpose" (`submission/README.md` line 209); data only referenced is
not covered (line 210); third-party-terms violations disqualify (line 222). Therefore **no EuRoC pixels in any
submitted material** (report figures, demo video, S3/ECR artifacts). Derived metrics are treated as facts that may
be reported (confirmed 2026-09-30: report EuRoC numbers, never its pixels). The data lives only in `.cache/` (gitignored and
dockerignored; `spinup.sh` snapshots via `git ls-files --exclude-standard`, so it never uploads).

**Fetch** (`infra/scripts/fetch_euroc.py`, stdlib only). The server honours HTTP Range. The script reads the outer
zip's central directory (2 requests), downloads exactly the member's bytes in one streaming ranged GET (1.57 GB
deflated for MH_01 instead of 12.7 GB), checks the central-directory CRC-32, and writes a provenance file with
the sha256 (`5f4ecbc5...` for MH_01). It retries with backoff on 429/5xx and resumes stored members. That sha256
goes into the manifest's `bag_checksum`.

**Reader** (`runtime/datasets/euroc.hpp`, OpenCV-free): strict `sensor.yaml` parsing (pinhole + radial-tangential
only, rigid `T_BS` checked, OpenCV's `%YAML:1.0` directive tolerated although MH_01's files do not carry it), a
strictly increasing `data.csv` index, and stereo pairing by EXACT timestamp, never by row. **Frame convention
pinned by test**: composing the Kalibr `T_BS` values reproduces ORB-SLAM3's independently published
`Stereo.T_c1_c2` to 1e-6, and the `stereoRectify` input `T_right_left` has t.x = -0.110 m.

**Rectifier** (`perception/stereo/stereo_rectifier.*`), in the idiom of ORB-SLAM3 (`precomputeRectificationMaps`)
and Kimera-VIO (`StereoCamera`): `stereoRectify(..., STEREO_ZERO_DISPARITY, alpha = 0)`, then
`initUndistortRectifyMap` straight to `CV_16SC2` fixed-point maps, then `remap`. `alpha = 0` follows Kimera-VIO (the
default -1 leaves black borders that create invalid keypoints, opencv/opencv#7240). `CALIB_ZERO_DISPARITY` is
deprecated in OpenCV 5 in favour of `STEREO_ZERO_DISPARITY`. The rectifier refuses a non-horizontal result or a
negative baseline (the inverted-extrinsic failure). Synthetic geometric test: 3D points projected through a raw,
distorted, toed-in rig land on one rectified row within 1e-3 px, with depth from disparity within 1e-4 relative.
The negative control, a 1-degree pitch error, produces > 5 px. `remap` output is bit-identical across thread counts.

**`configs/perception/euroc.yaml`** is a DATASET config: 96 disparities (Z_min 0.505 m at the rectified
fB = 436.2 x 0.1101 m; 13% blind columns), 400 target tracks (VINS-Fusion ~150, OpenVINS ~200 at 752x480), and
`required_min_range_m` 0.60. The mission's 0.30 m requirement stays in `placeholder.yaml`.

**MH_01_easy, all 3682 frames** (`ariadne_euroc_frontend`; stereo, alias and row checks on every 10th frame, i.e.
369 keyframes; 8 OpenCV threads, amd64):
- Pairing: 3682 pairs, 0 unmatched on either side.
- Rectification: sub-pixel LK left->right with a 0.5 px reverse check gives **median |dy| 0.135 px** (p90 0.68 px)
  over 96742 points. The controls must fail it and do: raw pair 13.1 px, 1-degree pitch error 8.1 px (f*theta
  ~ 7.6 px). ORB mutual matches give median 0 at their ~1 px resolution (integer FAST positions) and 96% positive
  disparity.
- Tracker: 377 tracks/frame mean (p10 333); 0 DEGRADED frames. Rejections over 1.39 M attempted:
  forward-backward 35.0k, LK lost 9.4k, epipolar 8.8k, crowding 33.7k, border 0.8k.
- Stereo: valid 70.4% of pixels, dense-supported 62.3%.
- **Aliasing on real texture** (pseudo-truth = 288-disparity SGBM, counted only where its own support gate keeps
  it; SGBM judging SGBM, so this is an estimate, not ground truth): 0.70% of pseudo-truth pixels are aliases.
  They are **episodic, not diffuse**: max 26% in one keyframe, and 43 of 369 keyframes above 1%, i.e. genuinely near
  surfaces. **The gate catches 87%**, leaving a residual of 0.09% of pixels (synthetic planes: 100%). Gate cost on
  agreeing depth: 0.14% in the interior, 0.24% within 15 px of edges. That cost is measured only on densely
  supported pixels, so its cost on sparse true depth remains unmeasured.
- Timing: rectify 1.4 ms/pair, tracking 22.9 ms/frame (p95 40 ms), SGBM 31.6 ms/keyframe at 752x480x96.

**Consequence for 1.8 (decision deferred to 1.8, recorded now):** stereo either never carves free space in the
placeholder, or a near-field pass is added (e.g. half-resolution SGBM at the same 96 disparities covers the blind
zone at ~1/4 the cost), with any carving gated by it.

Also: the run manifest now records `compiler`/`architecture` (shared `build_identity.hpp`), `imu_mode: none` (the
default `attitude_only` would claim an inertial estimator ran), `loop_backend: none`, ISO `created_utc`, and the
exact command in `extra.command` (§36). `core.sh euroc` reproduces the run with provenance.

## 2026-09-30 — D36: placeholder stereo VO (1.5), conventions, and the MH_01 trajectory

Reference implementations read for it: ORB-SLAM3 (`StereoInitialization`, `ThDepth` = 60 B close points,
`NeedNewKeyFrame`'s 0.75 reference ratio), VINS-Fusion (`solvePoseByPnP`, stereo DLT `triangulatePoint`), and evo
(`associate_trajectories` max_diff 0.01 s, TUM format). OpenCV 5 signatures were verified in the installed headers:
`solvePnPRansac(..., UsacParams)`, `solvePnPRefineLM`, `triangulatePoints`.

**Conventions (one place each).** The solver state is world->camera `T_C_W` with a LEFT perturbation
`T <- exp([rho; phi]) T` (§11.3). Nodes and keyframes store `T_W_C` (§4.2); their covariance is converted once
with `core::InvertCovariance`. Pinhole Jacobians (`core/geometry/pinhole.hpp`) are finite-difference tested at
typical depth, near-zero depth (1 cm), a wide-FOV ray (X/Z = 5), per tangent direction, and for the right camera
of a rectified pair. **Monte Carlo covariance consistency**: the mean NEES over 2000 noisy PnP solves is 5.93 (400 trials gave 6.37, by chance) (chi^2
with 6 dof, i.e. about 6) for both `T_C_W` and the transported `T_W_C`. The §12.2 edge residual
`Log(Z^-1 X_i^-1 X_j)` is a right-side error on Z, so an edge stores its information in the residual's tangent,
converted through `Adj(Z^-1)`. A Monte Carlo test with a 1-rad relative rotation confirms it (mean r^T Info r ~ 6).

**One optimizer** (`slam/bundle_adjustment/reprojection_lm.hpp`): LM (§11.6 Madsen-Nielsen damping, lambda_0 =
1e-4 max diag H, x1/3 on accept, x2 on reject, a relative-cost OR step-size stop, 15 iterations), Huber (§11.3),
Schur elimination of landmark blocks (§11.5), and pose covariance by solving `S X = E` (§11.6). With landmarks
fixed it is the PnP refinement, and it agrees with `cv::solvePnPRefineLM` to 1e-6. With the older keyframe fixed as
the gauge it is the two-frame BA stub. Dense S for now; sparse Cholesky arrives with the Phase 2 window.

**Stereo VO** (`slam/odometry/stereo_vo.*`, plain inputs per §3; runtime glues perception to it):
constant-velocity prediction; `solvePnPRansac` with the `UsacParams` overload seeded from Philox at a new
reserved site (`kMaxSite - 2`), single-threaded; the shared LM for the pose and its covariance; ORB-SLAM3's keyframe
rule plus min-tracked and max-frames limits; at keyframes, LK left->right correspondences (promoted to
`perception/stereo/stereo_lk_matcher`, the idiom validated at 0.135 px in D35), `triangulatePoints` with the §11.2
rejections (depth range, reprojection in both views, sigma_Z/Z, SGBM consistency where SGBM passes
`DenseSupportMask`), the two-frame BA, and an ODOMETRY edge carrying the BA's relative covariance. When PnP fails it
holds the motion model, flags the frame, and forces a keyframe with an uninformative (1 m / 1 rad) edge; it never
silently resets. Parameters live in `configs/slam/{placeholder,euroc}.yaml` (strict loader), with `slam_config`
added to the run manifest. **Limits:** a landmark lives only as long as its LK track (no re-association, so no
loop closure), two-keyframe BA, no IMU.

**Interfaces:** `slam::PoseGraph` (§12.1 edge types, §12.2 residual, a version counter bumped on every mutation, and
ODOMETRY's GNC weight fixed at 1); the §4.2 `Keyframe` and §18.2 `KeyframeBundle` records live in
`runtime/agent/keyframe_records.hpp`, because they join slam and perception outputs. `KeyframeBundle` is stamped with
the graph version (its `frame_id_of_pose`), and fields without a producer are listed and left unset.

**Synthetic end-to-end test (CI, no dataset):** a textured room projected through a rectified stereo rig along an
S-curve, with 0.3 px noise: 14 keyframes, **translation RMSE 4.6 mm over 3.5 m**, never lost. It stays lost-free
with 10% of tracks jumping 20 px per frame, and is bit-identical across 1, 2 and 8 OpenCV threads and arbitrary
`cv::theRNG()` state. (The test first tripped the D18 Philox exhaustion guard by drawing a whole sequence from one
stream; it now keys noise per frame as (seed, frame_id, site).)

**EuRoC MH_01_easy, all 3682 frames** (`ariadne_euroc_vo`; poses mapped to the body frame via
`T_B_Crect = T_B_C0 [R1^T | 0]`, a round trip tested with a negative control; associated to 3638 ground-truth
samples):
- **ATE (SE(3)): 0.126 m RMSE over an 80.5 m path (0.16%)**, mean 0.110 m, max 0.365 m; rotation 0.077 rad.
- **evo 1.31.1 as the oracle reproduces 0.126268 m and 0.077268 rad exactly** (ATE translation and rotation;
  the evo RPE command was not captured), so the ATE metrics and the frame conversion agree with the
  field-standard tool.
- Sim(3) scale 1.004 (no scale drift). RPE over 1 s: 1.39 cm and 0.0025 rad.
- The rotation error is drift, not a frame bug: anchored at the first pose it grows 0 -> 4.9 deg over 182 s. The
  3.5 deg at t = 0 in the aligned view comes from the position-based alignment spreading the end drift.
- 0 lost frames, 556 keyframes, 555 BA runs (mean final/initial cost 0.28). 22.6k landmarks created from 113.5k
  candidates (rejections: 55.5k no stereo match, 34.0k depth range, 1.45k SGBM-inconsistent).
- Timing: tracking 16.3 ms/frame, VO 4.0 ms/frame, keyframe (stereo + SGBM + triangulation + BA) 34.8 ms.

Review fixes (2026-09-30): node covariance is ABSOLUTE, composed along the odometry chain from the gauge keyframe
(`ComposeCovariance`); the conditional BA/PnP covariance stays on the edge, and `VoTrackResult` names its field
`covariance_given_map`. Tested: the node covariance trace never decreases, and the final keyframe's NEES under the
composed covariance is 1.01 (< 22.46). A blackout frame with no tracks is flagged LOST, holds the constant-velocity
pose, forces a keyframe with an uninformative edge, and tracking resumes within 3 frames. `MatchLeftToRight` is the
single stereo LK implementation (the frontend tool calls it with the row gate off) and has its own unit test. Every
shipped config is load-tested on the host. The EuRoC ground-truth copy is written under the gitignored `raw/`.

## 2026-09-30 — D37: procedural room-maze worlds (1.1–1.3 worldgen) and their deviations from arch §5.3/§5.5

Design: `docs/design/simulation.md` (the source of this entry). Code: `python/dataset_tools/worldgen/`
(numpy + pyyaml only; runs in the pinned ariadne-sim image via `infra/scripts/worldgen.sh`), presets
`configs/worldgen/{maze_short,maze_medium,maze_long,ci_tiny}.yaml`, ChArUco assets
`simulator/assets/charuco/` from `cpp/runtime/tools/ariadne_charuco_assets.cpp`. 100 worldgen tests pass in the
image, including two pinned golden scene-directory digests (GENERATOR_VERSION 1).

**What a scene is.** A seeded maze of 6 m lattice cells (Wilson uniform spanning tree) with 2–4 rooms, dead-end
stubs, two entrances with vestibules, and a target room whose ρ_responder geodesic from the primary entrance lies
in the preset's length band (short 10–20 m, medium 20–30 m, long 30–45 m, set from a 1000-seed census run on a
dirty tree before props and routes existed; to be re-run from the clean commit). Every
entrance is made 2-edge-connected to the target by a fundamental-cycle cover (so loop routes exist by
construction), and agent-only narrow passages (0.55–0.70 m) are placed off those cycles so ρ_agent and
ρ_responder routes genuinely differ (Invariant 4). Identity: (GENERATOR_VERSION, seed, preset digest, attempt).

**Consensus-aware scripting.** Scripted routes use the least-length edge-disjoint route pair per entrance
(min-cost flow); under the default `cross` template each return leg is edge-disjoint from the other agent's
outbound leg, so no head-on corridor meetings. Agents survey the target and both entrance rooms with 360° spins
at per-agent offsets. A scene is accepted only with ≥ 3 inter-agent same-heading, line-of-sight closure
opportunities in ≥ 2 rooms (~400–850 in 3 rooms on the seeds inspected), and only if a start delay keeps the agents ≥ 1.0 m
apart throughout.

**Deviations from the arch text.** Items 1–6 were in the design draft approved for implementation (2026-09-30). Items 7–9 arose during implementation and are **pending confirmation**:
1. IMU is synthesized by the kinematic mover from the trajectory's closed-form derivatives; IMU noise and bias
   random walk are a seeded consumption-time operator, not SDF `<imu><noise>` (§5.3). Forced by SMOKE §4g: gz's
   IMU reports (0, 0, 9.8) and zero rate on a pose-written body. One IMU config file serves the noise operator
   and the estimator (§10.2's "same numbers" holds).
2. 1.1 uses the generator; the plan's hand-authored two-room + corridor scene is the `ci_tiny` preset.
3. Phase 1 textures are rich and unique per surface, not §5.5's low-texture concrete; `appearance.tau` is the
   dial toward it (Phase 5), `appearance.repetition` the I7 aliasing dial (refused until 3.5).
4. Phase 1 GT is 2D (exact for extruded plans): occupancy, exact ESDF, traversability at both ρ, geodesics.
5. GT pose is published by the mover (exact), not gz's pose-publisher.
6. Surfaces are in-house OBJ quads, not trimesh meshes (§5.5 stage 3).
7. Scene worlds inline the scene model (SMOKE §4i: sdformat14 does not resolve relative `<include>`).
8. Phase 1 agent models carry no visual geometry: agents are invisible to each other (a stated simplification;
   a visible peer is a dynamic-object problem for later phases). Separation is enforced in the scripts.
9. Motion primitives are S-curves (C² velocity, continuous jerk) with yaw+pitch, not minimum-jerk; the rig
   pitches during the calibration dance (views rotated about one axis only are a degenerate Zhang set).

Withdrawn: GT depth at 640×360 (SMOKE §4j: saves ~1 % of compressed bytes).

**Measured sizing (SMOKE §4j, one scene):** ~84 MB per sim-second for two agents, RTF ~0.17 on the 8-core
llvmpipe host; missions 2–7 min per agent on the seeds inspected (the accept sweep records the distribution).
**Open:** how the `ci_tiny` bag (~16 GB) reaches nightly CI.

## 2026-10-01 — D38: ROS-free bag reading, the kinematic mover, and verified recording (1.1–1.3)

**ROS-free CDR decoding** (`cpp/runtime/replay/ros_cdr.{hpp,cpp}`, `rosbag_reader.hpp`): XCDR1 decoding of exactly
the recorded types (Image, CameraInfo, Imu, PoseStamped, TFMessage, Clock) into Ariadne's own structs, over the
Foxglove MCAP SDK (zstd chunks). Bounds-checked; every truncation of every fixture throws. Fixtures are bytes
produced by `rclpy.serialize_message` and a `rosbag2_py`-written zstd MCAP. Closes D25's design gap.
`McapRecordReader` now requests LOG-TIME order explicitly (the SDK default is file order); the Phase 0 replay
gate is unchanged (226/226 image, 199/199 host).

**Kinematic mover** (`simulator/plugins/kinematic_mover/`): a gz-sim system on each agent model, no Physics.
The trajectory math is `cpp/core/trajectory/scripted.hpp` — one dependency-free implementation that the core
suite checks against the Python generator's golden samples (808 samples incl. pitched ones, 1e-9) and the
plugin compiles. The script arrives as `<segment>` elements in the mission SDF (exact float repr), is validated
for continuity and against the scene ESDF (collision envelope, 10 ms sampling) at start-up, and a rotation
self-check pins gz's Euler convention; any failure aborts the run. The IMU message's orientation field is
identity and documented as unmeasured (an IMU does not measure attitude; GT stays out of perception).

**Mission composer** (`python/dataset_tools/worldgen/mission.py`) + rig config `configs/sim/rig_stereo_v1.yaml`:
the mission world inlines the scene and adds per-agent body link (IMU frame), stereo cameras, a GT depth camera
co-located with the left camera, an inert spot lamp, and the mover; the manifest carries T_B_C to the optical
frames and nominal intrinsics (the recorded camera_info and the §6 recovery are authoritative).

**Recording** (`infra/scripts/sim_record.{sh,py}`): bridge from the generated YAML, latched `/tf_static` with the
rig extrinsics, `ros2 bag record -s mcap --storage-preset-profile zstd_fast --use-sim-time`, then a STRICT
verification of the bag it wrote (every expected message present, period-exact gaps, stereo pairing 1.0, GT pose
and IMU equal to the script at their stamps) and `recording.json` with sha256, versions and identity. Deep queues
on the 200 Hz streams after SMOKE §4k found a stall-induced drop.

## 2026-10-01 — D39: canonical IMU / GT-pose streams written from the scripts; the live ones become a cross-check

**Accepted 2026-10-01**, on the condition that it is deterministically correct.

**Problem.** Deep queues (D38) did not make the live 200 Hz path lossless: the full `ci_tiny` mission (199 s)
lost one of 39,800 agent-1 IMU samples (t = 174.435 s) between gz-transport and the bag, while every sample that
arrived equalled the analytic value exactly (SMOKE §4k/§4l). The loss rate rises with agents (message rate and
host load both grow), and a missing IMU sample corrupts preintegration.

**Decision.** The recording's IMU and GT pose come from `ariadne_canonical_streams`
(`simulator/plugins/tools/canonical_streams.cc`), which evaluates the mission SDF's scripts offline and writes
them as their own rosbag2 MCAP bag, `RECDIR/canonical/`. The rendered bag moves to `RECDIR/sensors/` and keeps
the live IMU/GT as a cross-check.

- **Same code, same numbers.** The parser and the per-tick sampler (`MoverScript.hh`: `ParseMoverConfig`,
  `SampleAt`) are shared by the plugin and the tool; both targets compile with `-ffp-contract=off` so no FMA
  contraction can differ between inlining contexts (GCC fuses by default where the ISA has FMA, e.g. aarch64).
- **Stamp grid** t_k = k·period for k = 0..⌊T/period⌋. The plugin publishes k ≥ 1 (its first PreUpdate is one
  step in); k = 0 is added so GT brackets the first camera frame (stamped 2 ms). log time = header stamp.
- **Own CDR encoder** (`cpp/runtime/replay/ros_cdr_writer.hpp`), not rclcpp: Fast-CDR skips alignment padding
  without writing it, so rclcpp output carried heap garbage (observed "at" in the 2 pad bytes after
  "agent_0/base_link") and two runs of the tool differed. With zeroed padding every message is decoded back
  by the runtime decoder and compared bit-for-bit (a -0.0 stays -0.0) before writing.
- **What "reproducible" means, scoped.** (a) *Container bytes*: identical run-to-run in the same image and
  arch — checked on every recording (generated twice, compared) and the recording aborts before gz if not.
  (b) *Content*: `canonical_content_sha256` = sha256 over (topic, stamp, payload) in (stamp, topic) order,
  independent of rosbag2/MCAP/zstd versions; pinned for ci_tiny seed 0 / 8 s by
  `python/tests/sim/test_canonical_streams.py` (`worldgen.sh simtest`), so a one-bit change anywhere in the
  trajectory code, encoder, composer or routes fails in ~15 s instead of on the next render. (c) *Across images
  or arches* (libm, codegen) the content may move in the last bits; the guard there is the ≤ 1e-9 agreement
  with the independent Python evaluator, not bit equality. The manifest pins the inputs: mission SDF sha256,
  `end_ns`, tool binary sha256 (built from `git_commit` in `image_id`).
- **Verification** (`infra/scripts/sim_record.py`): canonical — exact grid, log time = stamp, values = the
  independent Python evaluator (≤ 1e-9), regenerated a second time and compared byte-for-byte; live IMU/GT —
  every sample bit-identical to canonical (bytes, else decoded fields as IEEE bit patterns), no off-grid or
  duplicate stamps, losses counted (`live_gaps`, with stamps) and fatal only below 99 % coverage; cameras and
  infos — unchanged STRICT (a frame cannot be regenerated; remedy = re-record). The world's `<max_step_size>`
  is asserted to be the 1 ms the recorder counts in. The C++ `RosbagReader` reads a committed 1 s canonical bag
  ROS-free (`test_ros_cdr`), and the encoder reproduces live bridge bytes including padding.
- **Honesty.** Manifest `streams.canonical.source` = "analytic IMU/GT from script [DESIGN] … not sensor output".
  gz never measured this IMU; the mover already computed it analytically (OpenVINS-simulator pattern); noise,
  bias and jitter stay a seeded consumption-time operator (D37).
- **Replay rule.** IMU + GT from `canonical/`, everything else from `sensors/`, merged by header stamp (ties:
  topic name), not log time (sensor log times are receive times). Implemented with the 1.4/1.5 sim consumer.
- **Cost.** 5.1 MB of canonical MCAP against 12.6 GB of sensors for the 199 s mission (0.04 %).

**Consequences / open.** Live closed-loop and demo runs have no canonical file: the estimator must bridge a
missing IMU sample (integrate across the gap, log it) — Phase 2 requirement. The §4g "frame stamped t shows the
pose at t" measurement that the canonical GT relies on was made at 2 agents on llvmpipe; re-check it whenever
the agent count grows. `live_gaps` per run is the trend line. The bridge and recorder logs are now kept
(`RECDIR/logs/`) and their loss reports parsed into the manifest; transport loss is measured on `/clock`
(SMOKE §4l: 27 on an 8 s run, matching the bag exactly). The first fresh full recording under D39 then lost
one **camera frame** (agent 1 right, 3980/3981) and failed strict — the remaining exposure, since frames
cannot be regenerated. Its layer (DDS transport, recorder cache, gz → bridge) is unknown: that run predates
log capture. Open: attribute with logged runs, then size image queues / recorder cache; until then the
remedy is re-record.

## 2026-10-01 — D40: the sim recorder reads gz-transport directly; no bridge or DDS in the recording path

**Accepted 2026-10-01** (option A, with loud failures). Deviates from design §7 /
arch §5.9's `ros2 bag record` via `ros_gz_bridge`; the bridge stays for live (closed-loop, demo) runs.

**Why.** Three logged full recordings (SMOKE §4l) put every lost message on the DDS link between the bridge
and `ros2 bag record`: per topic, expected − in bag = the recorder's own transport-lost count, residual 0; no
cache drops; isolated single losses, including GT pose with queues 1000 deep on both sides. Depth is not a
guarantee, and frames cannot be regenerated.

**What.** `ariadne_gz_recorder` (`simulator/plugins/tools/gz_recorder.cc`) subscribes to the mission's gz
topics (the bridge YAML's GZ→ROS set), converts with `ros_gz_bridge::convert_gz_to_ros` from the installed
`libros_gz_bridge.so` (the bytes the bridge would publish; ROS CDR serialized into zero-filled buffers), and
writes `RECDIR/sensors/` through `rosbag2_cpp` with `max_cache_size = 0` (synchronous; no cache drop path),
log time = header stamp. gz-transport's ZeroMQ high-water marks are 0 (unbounded) on both ends
(`GZ_TRANSPORT_{RCV,SND}HWM`; default 1000 drops silently). The world starts paused and is unpaused only
after the recorder has discovered every publisher. `/tf_static` moves into `canonical/`, built from the SDF
camera sensor poses (identity rotation asserted) and checked against the mission manifest's extrinsics.

**Loud.** Every way a message can go missing is an error printed when it happens, counted in
`logs/gz_recorder.json`, and a non-zero exit that fails the recording: a stamp interval that is a multiple
of the period (k−1 LOST), any other irregular interval, duplicate, out-of-order, a first stamp other than the
expected one (start missed; 0 for rendered topics and `/clock` with the paused start, one period for the
mover's IMU/pose, as measured), a topic not reaching floor(end/period)·period (incomplete), the writer backlog
exceeding its cap (default 4 GiB: ABORT rather than drop), no publisher discovered (exit 2), an unsupported
type pair (exit 2). Sole exception, from measurement: a paused world republishes `/clock` = 0, so repeats
of stamp 0 before a topic's first non-zero stamp are counted as `paused_repeats` and not written.
`/clock` becomes strict in the verifier (one message per 1 ms step). Each failure mode is exercised against
the real binary by `python/tests/sim/test_gz_recorder.py` (13 tests, fake gz publisher). Log time = header
stamp in the sensor bag too (it was the recorder's receive time), so both bags merge in stamp order.
Not logged by the recorder: running out of memory (the ZeroMQ queues are now unbounded); it still surfaces
as a non-zero exit of gz or the recorder, which fails the run. Recordings made before D40 (no recorder report,
no canonical `/tf_static`) no longer pass `--verify-only`; they are superseded, not migrated.

**Confirmed** (SMOKE §4m): two full `ci_tiny` missions recorded through it both PASSED strict verification
with 0 recorder errors and every message present, where the bridge path lost 3–9 non-clock messages per run.

## 2026-10-01 — D41: lens distortion in the simulated rig is a read-time operator (option A)

**Finding (SMOKE §4n).** Under gz Harmonic's ogre2 renderer the SDF `<distortion>` is ignored (no
`DistortionPass` in ogre2) while `camera_info` still advertises the coefficients; the rendered camera is an
exact pinhole. `camera_info` also states cx = W/2 where the pixel-centre convention needs (W-1)/2. Arch §5.3
asks for a mild plumb-bob so the undistort path is exercised.

**Until decided:** the recorder's gz-warning gate fails every recording with non-zero rig distortion; the
1.4/1.5 sim adapter must not take D (nor cx, cy) from `camera_info` — the truth is pinhole fx = fy = 640,
cx = 639.5, cy = 359.5. The 1.3 evidence recording (`evaluation/runs/gates/phase1_thin_e2e/sim_record/ci_tiny_0-20261001T042105Z`)
predates the gate: it is valid evidence for the lossless recording path; its camera_info D is false.

**Options.** (A, recommended) zero distortion in the SDF so camera_info is honest about the pixels (cx
convention documented and corrected in the adapter), and Brown distortion applied as a seeded consumption-time
operator (`cv::remap` with the forward model), consistent with D37's degradation-at-read; the undistort path is
exercised with exactly known coefficients. (B) pinhole only in sim; EuRoC (real radtan) exercises undistort.
(C) the Ogre1 backend — not verified to render headless on EGL here; not recommended.

**Accepted 2026-10-01 (option a). As built:**
- **Render:** `configs/sim/rig_stereo_v1.yaml` has no `camera.distortion` (the composer refuses one, naming
  this decision); the SDF carries no `<distortion>`, so gz has nothing to ignore and `camera_info` D is zero.
  The render truth is pinhole fx = fy = 640, cx = 639.5, cy = 359.5 (pixel-centre); the mission manifest
  records it as `render_intrinsics_pixel_centre` and states gz's W/2 camera_info convention.
- **Lens:** `rig.lens` (plumb_bob, fx = fy = 680, cx = 639.5, cy = 359.5, k1 -0.05, k2 0.01, p1 0.0005,
  p2 -0.0002, k3 0) is applied at read time by `cpp/perception/degradation/lens.hpp`: per output pixel, invert
  Brown-Conrady (fixed-point, 30 iterations, verified to 1e-12) and sample the render bilinearly; depth is
  resampled nearest-neighbour. OpenCV-free, IEEE double with `-ffp-contract=off`, pinned by a digest. Optics
  precede the sensor stages of `Degrade()`.
- **Why fx 680, not 640:** with barrel distortion the inverted ray lies outside x_d, so a lens with the render's
  focal length would sample beyond the render (black corners). The minimum for these coefficients is 674.05;
  680 leaves >= 3.37 px. Both `BuildLensMap` and the composer refuse a lens that samples outside the render.
- **Verified** (`tests/unit/test_lens.cpp`, SMOKE §4n): the model equals `cv::projectPoints` to < 1e-9 px;
  identity and linear-image exactness; refusal of fx 670. §6 on a full recording with the lens on: GT
  reprojection through the lens model 0.07-0.09 px RMS, no radial trend; OpenCV's free calibration recovers
  k1 -0.0506, k2 0.0104 (truth -0.05, 0.01) and fx within 0.4 %; stereo with the truth intrinsics within
  0.09 mm. §6 now has a verdict (`configs/sim/acceptance_s6.yaml`) and exits 1 on FAIL. That PASS is CIRCULAR:
  the thresholds were derived from the same dev run; the first independent test is the evidence run on a fresh
  D41-rig recording. Demonstrated FAIL: a 5 mm board misplacement (every A check). The other failure modes are
  argued from magnitude, not demonstrated. The free calibration's fx bias (+0.2-0.4 %) persists with k1/k2/k3
  held at truth `[ASSUMPTION: fx-distance ambiguity of near-frontal planar views]`.
- **Consumers** take the camera model from `rig.lens` (via the mission manifest), never from `camera_info`.
  Nothing applies the lens yet outside §6: the 1.4/1.5 sim adapter must run `ApplyLens` then `Degrade`.
- **Evidence:** the committed recording `ci_tiny_0-20261001T042105Z` used the pre-D41 rig (camera_info D false);
  it stays evidence for the lossless recording path only. The upcoming D41-rig recording replaces it as the
  1.3 evidence and is the input of the §6 evidence run.
- **Open, recorded:** (1) the route speed budget uses `focal_px: 640` (worldgen presets); the estimator's camera
  is now 680, ~6 % more pixel motion at the image centre than planned — changing it moves the worldgen goldens
  (GENERATOR_VERSION bump), so it is a deliberate later change; the same 6 % applies to SGBM disparity ranges in
  the 1.4/1.5 sim config. (2) The lens digest is pinned only by an x86 unit-test golden; add it to
  `RecordDeterminismDigests` (D32) before any Graviton/COOL run so manifests and the Arm gate check it.
  DONE 2026-10-01: `perception/degradation/lens_digest.hpp` (map bits + output bytes, all coefficients
  non-zero), manifest field `lens_digest` (required), `check_determinism.sh` EXPECTED_LENS
  1253886791886396038 (g++/clang++, -O0..-O3).
- **Evidence status:** the D41-rig recording `ci_tiny_0-20261001T133914Z` (clean) is the 1.3 evidence. §6 on it
  reproduces the dev run exactly (renders reproducible), so it does not break the threshold circularity; the
  independent §6 test is ci_tiny seed 1 (other rooms, other board pose) — PASSED with margin on every check
  (SMOKE §4n). Thresholds `s6_v1` stand.


## 2026-10-15 — D42: 1.4/1.5 run from a sim recording through one StereoSource interface

**What.** `cpp/runtime/datasets/stereo_source.hpp` defines `StereoSource` (raw calibration, T_B_C0, GT body poses,
time-ordered RAW gray stereo pairs) with two implementations: `EurocSource` and `SimSource`. The 1.4 + 1.5 loop and
its scoring moved verbatim from `ariadne_euroc_vo.cpp` into `cpp/runtime/tools/stereo_vo_runner.hpp`
(`RunStereoVo`); `ariadne_euroc_vo` and the new `ariadne_sim_vo` only bind a source. **Regression:** the EuRoC
MH_01 trajectory reproduces byte-identically (estimate sha256 22d3a40d…; results.yaml identical except timing).

**SimSource** reads one agent of a two-bag recording ROS-free: frames from `sensors/` through the read-time lens
(D41) then RGB→gray; calibration from `rig.lens` (both cameras) and `T_right_left` from the canonical
`/tf_static`; GT from the canonical pose at each image stamp (images sit on the 5 ms grid, so association is
exact). It never reads camera_info. It refuses a pre-D41 mission, k3 != 0 (PinholeRadtan carries four
coefficients) and any stereo pair that completes out of order (a missing frame). Not yet: the sensor stages of
`Degrade()` after the lens (Phase 5; needs GT depth and a named config); manifests say degradation "none".

**Configs `sim_v1`** (perception, slam): euroc.yaml with only resolution/geometry-dependent values changed. The
§9.4 stereo guard refused the EuRoC disparity range on this rig (Z_min 0.71 m at rectified f 671.5, B 0.1);
240 disparities give Z_min 0.281 m, meeting the 0.30 m mission requirement (the sim is the mission geometry,
unlike a dataset), at the cost of 19 % blinded columns and ~0.28 s SGBM per keyframe at 720p.


## 2026-10-15 — D43: 1.6 placeholder collaboration — one broker, verified SE(3) edges, bytes from day one

**What.** `ariadne_sim_collab` (design `docs/design/collaboration.md#first-collaboration-loop`) runs both agents' 1.4/1.5 VO in replay from one
sim recording and then the two-stage exchange of arch §15.1 with placeholder internals: agent 1 sends a
GLOBAL_DESCRIPTOR per keyframe (256-D mean-bit ORB, `collaboration/retrieval/mean_bit_descriptor.hpp`) to the broker
(agent 0, lowest AgentId); the broker takes cosine top-1 per query, keeps up to `max_candidates`, sends
MATCH_REQUEST; agent 1 answers MATCH_RESPONSE (ORB keypoints + descriptors + stereo 3D points + camera); the broker
verifies (`collaboration/broker/edge_verifier.hpp`: ratio test → `findEssentialMat` RANSAC gate → `solvePnPRansac`
for the metric SE(3) edge, RANSAC seeded by rank) and sends POSE_EDGE (`LoopEdge` table, status GEOMETRY_VERIFIED).
Merge: T_O0_O1 from the best-ranked verified edge; both trajectories scored in one frame. Every message goes through
`WireCodec` (three new `wire.fbs` tables appended to `Envelope`, decode validates array lengths) and `ByteLedger`,
and through the idempotent `collaboration/protocol/inbox.hpp` (per-(sender, class) sequence, out-of-order buffer,
duplicates dropped). The VO runner gained `on_keyframe` / `on_body_pose` hooks (called only when set).

**Choices.** The essential matrix is the plan's geometric gate but has no scale, so the edge is PnP of agent 1's
stereo 3D points against agent 0's pixels (SE(3), locked decision; never Sim3). Edge information is a `[PROXY]`
diagonal (σt 2 cm, σr 0.01 rad at 30 PnP inliers, ∝ 1/√inliers). Outputs are built from a rank-keyed outcome map
after the exchange, so they cannot depend on arrival order; any unexpected message class, lost edge, undrained link
or leftover inbox message fails the run. GT is evaluation-only (opportunity labels, edge error), never fed back.

**Fault injection** (`--inject-faults 1`): duplicate every 5th send, hold every 3rd delivery back past the next
flush's messages, drain at the end; the run fails unless both inboxes saw duplicates AND buffered reorders. The
first version (adjacent swaps) mostly swapped messages of different classes, which the per-class inbox never had
to reorder; replaced before any result was taken.

**Evidence** (clean commit 2ed0a49, `git_dirty: false`): `evaluation/runs/gates/phase1_thin_e2e/sim_collab/`
`aadffb8df5790ca0-20261015T083420Z` (no faults) and `aadffb8df5790ca0-faults-20261015T083420Z`;
`infra/scripts/check_collab_fault_identity.sh` → `fault_identity.txt` **VERDICT PASS** (CSVs byte-identical,
results.yaml identical except inject_faults/protocol, both agents' estimates sha-identical, faults exercised:
broker 54 dup / 4 buffered, agent 1 8 / 8). Every pre-diagnostic column and results.yaml are identical to the
dev runs (the diagnostics do not change the verifier).

**Measured (ci_tiny seed 0, `collab_v1`; evidence = dev runs).** 327 + 250 keyframes. 20 candidates (the cap), 20 verified,
**20/20 edges within 5 cm / 1° of GT** (max 3.3 cm / 0.57°); 17 inside the 30° opportunity cone, the other 3 are
correct edges at 33–48° view angle (the cone is worldgen's guarantee, not a correctness test; gates were NOT
tightened to exclude them). Merged two-agent ATE 9.8 mm (one alignment, 7962 poses). Bytes (§15.7 first numbers):
GLOBAL_DESCRIPTOR 250 × 1170 B = 292 KB; MATCH_REQUEST 20 × 137 B; MATCH_RESPONSE 20 × 48.6 KB = 971 KB (76 % of
1.27 MB); POSE_EDGE 20 × 207 B. Core suite 248/248 (4 protocol + 2 verifier-diagnostic tests new); EuRoC MH_01
estimate reproduces byte-identically (22d3a40d…). Fault run: broker 54 duplicates / 4 buffered, agent 1 8 / 8, `candidates.csv`
byte-identical and `results.yaml` identical except `inject_faults`/`protocol`; both runs' VO trajectories
byte-identical to the committed `sim_vo` evidence (the hooks do not perturb VO).

**What the numbers do not show (open items).**
1. Retrieval is weak and the threshold is inert: over all 250 queries the top-1 cosine spans 0.884–0.996
   (median 0.893), so `retrieval_threshold: 0.5` passes everything and the cap selects (effectively top-20 by
   score). 71 queries have a GT opportunity; top-1 is a true opportunity for 51 (72 %). EigenPlaces + λ₂
   prioritization replace this in Phase 3; the threshold is not re-tuned to look meaningful.
2. The proxy information is overconfident: rank 4 (333 inliers, σt ≈ 0.6 cm) is off by 3.3 cm ≈ 5σ, rotation
   ≈ 3σ. Harmless for the 1.6 merge; PCM/GNC (Phase 3) need a real edge covariance.
3. Candidates cluster: agent 1 parks for ~70 % of its run (GT, 200 Hz: stationary at (9.0, 7.9) from ~57.5 s to
   the end at 199 s; camera 20 Hz, hence its constant 159 VO tracks), so many of its keyframes view one place.
   Vertex-cover / dedup is Phase 3.
4. MATCH_RESPONSE dominates bytes; 1000 ORB features with float32 3D points per request. Budgeting is Phase 3.
5. `edge_correct_t_m`/`_r_deg` (5 cm / 1°) were set after the first run, to separate correctness from the cone.
6. **The verifier accepts a wrong edge** (negative probe, below). Not gated in 1.6: the exchanged edges are all
   correct, the degeneracy gate → PCM → GNC is the locked Phase 3 design, and any threshold picked now would be
   fit to one negative (`min_essential_inlier_ratio` 0.57–0.67 would reject it and keep all 25 correct edges;
   `collab_v1` is NOT changed). This exact pair becomes a Phase 3 regression case.

**Negative probe** (evaluation only; no wire, bytes, edges or merge): the verifier is fed the 20 highest-scoring
top-1 pairs that are NOT opportunities (`negative_probe.csv`); a false accept is a verified edge that is not correct
vs GT. Dev run: **14 rejected** — including all 12 pairs at different places (4.9–7.7 m apart; 7 at the essential
gate, 5 for too few matches) plus 2 near pairs (too few matches / 3D points) — **5 accepted, all correct** (out-of-cone overlaps), **1 false
accept**: agent 0 frame 3372 vs agent 1 frame 382, 0.48 m apart at 54.5° view angle, 289 matches / 162 essential /
129 PnP inliers, edge error 1.10 m / 51.6°. So the gate rejects appearance-only matches between different places
and fails on a near pair. A yaw-only bearing check suggests both cameras face ChArUco board 1 (room (1,0), wall S)
from ~1.3–1.4 m (a planar target, where PnP has a flip ambiguity). **Tested and NOT confirmed as a separable
cause:** the non-gating diagnostics `h_over_e` (homography / essential inliers on the same matches) and
`planarity` (λmin/λmax of the PnP inliers' 3D covariance) put the false accept (0.611, 0.00104) INSIDE the range
of the 25 correct edges (h_over_e 0.19–0.71, planarity 0.00041–0.167; ranks 1, 2 and 4 are more planar, rank 6
has a higher h/E). In a room maze most matches lie on one wall, so correct edges are near-planar too: neither
scalar can gate this. Cause `[UNVERIFIED]`; the Phase 3 answer is consistency (PCM against odometry and the other
edges), not a per-edge threshold.


## 2026-10-15 — D44: gsplat runs off-AWS (Kaggle) until the GPU quota lands; no CPU rasterizer

**Facts.** No NVIDIA GPU on the dev box. AWS GPU quota is 0 for EC2 G/VT (on-demand and spot) in eu-north-1,
us-east-1, us-west-2, eu-west-1 and eu-central-1, and for every SageMaker g4dn/g5/g6 training/processing/notebook/
Studio quota checked (eu-north-1, us-east-1). The 2026-09-29 cases auto-closed unanswered; re-filed 2026-10-15
(spot case 179205877700888, on-demand case 179205877900026, 4 vCPU each); AWS escalated both to an internal team the
same day. gsplat 1.4.0 cannot rasterize without CUDA: even `_torch_impl._rasterize_to_pixels` calls a CUDA kernel and
needs `nerfacc` (SMOKE §4o).

**Decision.** Delegate the gsplat render to Kaggle for now. Kaggle rather than Colab because it has a
CLI (`kaggle datasets` / `kaggle kernels push|status|output`), so the run is scripted and its environment
recorded. This deviates from the locked compute placement ("gsplat on AWS g5/g6", arch §18.2) and is **interim**;
arch §18.2 carries the note. A further requirement: **the demo needs realtime rendering**, so a live GPU
(AWS once the quota lands) stays on the critical path and the quota cases stay open.

**Consequences.** No Ariadne-written CPU rasterizer (it would be throwaway once any GPU exists). The C++→Python
boundary is the spool (design reconstruction.md#first-reconstruction-loop); validation, fusion, initialization and geometry metrics run locally on
CPU and deterministically; only a small render package goes to Kaggle. Manifests record
`splat_backend: gsplat_cuda@kaggle` (later `@aws`) with the remote torch/CUDA/gsplat versions and GPU name. Remote
renders are compared by tolerance, not bytes.


## 2026-10-15 — D45: 1.8 placeholder mission layer — OctoMap-model occupancy, nearest frontier, egress at ρ_responder

**What.** `ariadne_mission` (design `docs/design/mission.md`) consumes the 1.7 spool and the mission briefing
and, in replay, integrates keyframe depth into a 0.05 m voxel grid (OctoMap sensor model, Hornung et al. 2013:
log-odds hit 0.7 / miss 0.4, clamp [0.12, 0.97], fixed-point; rays walked with Amanatides & Woo 1987), collapses a
standing-body z band into a 2D tri-state, and every 5 s computes frontiers (Yamauchi 1997), a nearest-frontier
decision per agent (JSONL trace, `mode: replay, executed: false`), the **egress route at ρ_responder** and the
**agent route at ρ_agent** (A*, octile, worldgen's corner rule). `python/analysis/mission_eval.py` scores against
`scene_gt.npz` (evaluation only).

**Choices (each fixed before measuring).**
1. *ρ discipline:* the plan's wording ("A* route to exit at ρ_agent") is superseded by invariant 4 / arch §20.5:
   the egress product is at ρ_responder (0.40 m); the ρ_agent route is reported beside it, labelled.
2. *Clearance convention = GT's* (cell centre to blocked-square edge, ≥ ρ − 1e-9), ported exactly; egress clearance
   counts unknown as blocked ("observed, not merely not contradicted"), agent clearance counts occupied only.
3. *World anchor:* agent 0's briefed entry pose × the rig's body←camera extrinsic (`[ASSUMPTION]` a team knows
   where it entered; R1 ≈ I on the ideal rig). Agent 1's entry pose is NOT used: it enters W only through the
   verified 1.6 loop, at t_merge = the later of the two matched keyframes (`[ASSUMPTION]`: that edge was picked
   with hindsight). The evaluator's oracle: agent 0's first 10 s through this anchor vs GT.
4. *Depth gates from the noise model:* σ_Z = 0.00745 Z² m (σ_d 0.5 px, f 671.5, B 0.1) → σ ≤ 0.06 m ⇒ Z ≤ 2.84 m, so
   wall hits land within ~1 cell and the 1.05 m entrance's 2.5-cell slack at ρ_responder survives; free carving
   stops 2σ short of each surface.

**Measured (dev, ci_tiny seed 0, the 1.7 evidence spool).** Frame anchors (first 10 s vs GT): agent 0 5.0 mm /
0.07°, agent 1 (own entry pose, solo run) 6.2 mm; agent 1 placed through the 1.6 merge: 3.0 cm / 0.47° (the merge's
accuracy). Merged map vs GT: coverage 0.93, free precision 0.97, occupied precision 0.71. Three runs (first-valid
times are replan instants: the route appeared within the preceding 5 s):

| run | first valid egress route | final exit, length (GT geodesic) | coverage |
|---|---|---|---|
| agent 0 alone | 195 s | west, 11.26 m (11.21) | 0.89 |
| agent 1 alone (its own entry anchor) | **55 s** | north, 4.44 m (4.44) | 0.43 |
| merged (agent 1 joins at t_merge 138 s) | 140 s | north, 4.44 m (4.44) | **0.93** |

Every route is valid on every cell against GT `trav_responder`. **Reading:** the target is in agent 1's room, so
agent 1 alone has a valid route at 55 s; the merge brings it into the shared (broker) map only at 140 s, because
the one verified inter-agent loop arrives at 138 s. What collaboration adds here is the joint map (coverage 0.93 vs
0.89 / 0.43) and, for agent 0, the shorter exit 55 s before its own exit leg would have found any. The 85 s of
merge latency is a measured cost of a single late loop: the Phase 3 case for earlier, prioritized exchange.
Deterministic: trace, routes and maps byte-identical across runs. 257/257 C++ tests (7 new planning tests).

**What the numbers do not show (open items).**
1. Occupied precision 0.71 is a convention effect, measured: of the 783 map-occupied, GT-free cells, 726 (93 %)
   are exactly one cell from GT structure, 56 two cells, 1 three, none further. Walls are thin planes on cell
   boundaries, so their hits land in the last GT-free cell. It costs clearance along walls (the north entrance's
   traversable band is 5 cells at ρ_responder) but blocked no route here.
2. Stray "free" cells beyond the walls: rays slipping past the thin wall meshes into space GT counts as structure.
   Disconnected from every route; costs ~3 % free precision.
3. The policy is logged, not executed (replay follows the scripts): there is no exploration-efficiency number yet;
   that needs closed-loop live runs. Nearest frontier is the baseline for the owed Domain 3 survey (FUEL, TARE, FAR).
4. One scene, one seed. The egress timing depends on where the scripts face; seed 1 is the next check.


## 2026-10-15 — D46: 1.9 evaluation harness v1 — every §30.2 metric, gated by the C++ registry, cross-checked by evo

**What.** `infra/scripts/report.sh` over one Phase 1 run chain (sim_collab → recon → mission): the C++ tool
`ariadne_report_check` dumps the D31 registry (single source of truth) and later gates the report with
`MissingMetrics` / `UnknownMetrics`; `python/analysis/report.py compute` (pinned analysis image, evo 1.31.1)
computes the 27 metrics with the definitions in `docs/design/evaluation-harness.md`, checks the gates (all input manifests
clean, the mission's and the spool's T_O0_O1 identical) and three oracles; `render` draws the figures and a
self-contained `report.html` (figures embedded, every number beside its definition, inputs with sha256).

**Choices.** (1) RPE pairing = the C++ `rpe_1s` pairing, (i, i+20) for every i, which is evo with
`all_pairs=True, delta_unit=frames`; evo's default (non-overlapping pairs) would differ. (2) Closure metrics use
denominators the harness cannot inflate: precision over everything ACCEPTED (25/26, the probe's false accept stays
visible), false-closure rate over the edges that reached the map (0/20). (3) Distance travelled from GT (VO jitter while
agent 1 is parked would inflate the estimate; both reported). (4) Calibration and energy are `[PROXY]`, labelled on the
page. (5) evo sees agent 1's timestamps offset by 10⁶ s: both agents share the sim clock and evo associates by time.
(6) Rotation angles via atan2: arccos lost ~1e-6 relative at mrad level and failed the first evo comparison.

**Measured (dev, the committed 1.6–1.8 evidence chain).** All 27 metrics emit a number: **gate PASS**. Oracles:
per-agent ATE/RPE equal the C++ results; merged ATE 0.0097643 m over 7962 poses = 1.6; evo APE translation and
rotation and RPE translation and rotation agree to ≤ 4.5e-14 relative (APE rotation 1.5e-11) over 7962 / 7922
samples. This is the **first
RPE oracle** (1.5's evo run never captured RPE). Headline values: ATE 9.76 mm / 0.26°, RPE (1 s) 0.86 mm / 0.018°,
inter-agent edge RMSE 12.0 mm, closure precision 0.962, false closure rate 0, Chamfer 0.126 m, completeness 0.66,
coverage 0.93, PSNR 17.5 dB (untrained init), 1.27 MB total (2.2 kB/keyframe), first valid egress 140 s, route
validity 1.0. Harness output byte-identical across reruns (except the analysis image id recorded in inputs.yaml).

**Open.** Single run chain, single seed: no intervals, no cross-run calibration plot, no §31 matrix yet; outputs YAML
not parquet. `time_to_first_usable_map_s` is defined as the first agent-ρ route `[ASSUMPTION]`.


## 2026-10-16 — D47: 1.10 operator console stub — replayed telemetry over WebSocket, causal message times

**What.** `python/operator_console/` (FastAPI + a Canvas 2D page, pinned `infra/docker/console.Dockerfile`,
`infra/scripts/console.sh check|serve`): a committed run chain replayed as telemetry frames at 5 Hz over
`/ws/telemetry` (agents from ESTIMATED poses through the mission's world anchor, agent 1 only once merged; cumulative
wire bytes per class + rate; the latest map snapshot and routes; incident events), REST `/api/meta`, `/api/map/<i>.png`,
`/api/frame?t=` (a static frame with all events so far: screenshots and thumbnails). Design `live-runtime.md#operator-console`.

**Producers gained time-resolved outputs, with identity proven.** `ariadne_sim_collab` writes `ledger.csv` (one row per
message the ByteLedger counted) and `ariadne_mission` writes a map snapshot per replan. A full dev rerun of the 1.6/1.7
chain is byte-identical to the committed evidence (candidates, probe, results, both VO estimates, and the render
package npz `9ec2f672…`, so the 1.7 Kaggle render stays valid); the mission rerun is identical to the 1.8 evidence.

**Causal message time (a correction, not a choice).** Wire stamps are keyframe times and are not causal: a match
request carries agent 0's keyframe (~138 s), its response and edge agent 1's (~20 s), so a stamp-keyed console would
show edges before the requests that triggered them. The ledger therefore also records a causal send time —
descriptor = its keyframe; request = the later of the two keyframes; response = request; edge = response — the rule
1.8 uses for t_merge. Wire stamps are unchanged, so encoded bytes and the 1.6 evidence are identical.
`[ASSUMPTION]` zero processing and link latency.

**Checks (`console.sh check`, PASS on the dev chain).** Ledger totals = results.yaml (310 messages, 1,270,717 B, per
class bytes and counts); the console's merge event = the mission's t_merge_s (137.95 s) and its first-valid-route event
= 140 s (one time base); no edge before its request, the first edge not after the merge; frames byte-identical across
builds. Visual check by headless screenshot at 150 s: map, both agents, egress route, gauges, timeline. Frontier
decisions appear only when the choice changes, labelled "logged decision · replay · not executed" (invariant 3).

**Not in 1.10.** 3D splat view in the console (the separate 1.7 viewer page exists), approvals (§23), raw vs restored
feeds, live hosting (Phase 8), budget gauges against a policy budget (Phase 3). A claude.ai artifact cannot reach a
localhost WebSocket, so the console is served locally (`127.0.0.1`, ssh tunnel from elsewhere).

## 2026-10-16 — D48: Phase 1 exit gate — one replay command, the §4 contracts wired and counted, determinism vs evidence

**What.** `infra/scripts/phase1.sh` replays the recorded bag through every Phase 1 stage from one commit, into one
directory (`evaluation/runs/gates/phase1_thin_e2e/exit_gate/<scene>-<UTC>/`; large outputs in `build/phase1_gate/`, sha256
committed): build + full ctest, the protocol tests, 1.6 clean + fault-injected runs concurrently, fault identity,
1.7 prepare → fresh Kaggle render → score, 1.8 mission ×3 + eval, 1.9 report, 1.10 console check. Then
`python/analysis/phase1_gate.py` (config `configs/benchmark/phase1_gate.yaml`) writes `gate.yaml` and one verdict.
Design `evaluation-harness.md#end-to-end-gate`.

**The §4 contracts were not exercised before (a correction).** `runtime::Keyframe` (§4.2) and `KeyframeBundle` (§18.2)
were defined, but no run built them: the VO hook passed loose fields, and `ariadne_mission` declared its own `Keyframe`.
Now `RunStereoVo` builds both at each keyframe. The Keyframe holds the graph node pose, its absolute covariance, SGBM
depth and the observed landmarks (`StereoVo::KeyframeLandmarks`); the Bundle is built by `BuildKeyframeBundle` at the
graph's version. The 1.6 exchange completes the Keyframe with ORB and the mean-bit descriptor, and derives its wire
features from it; the spool writes the Bundle. `StereoFrame` (§4.1) gains `agent` and `id`, asserted in sequence.
Agent 1 now checks every decoded POSE_EDGE field for field against the encoded one, and checks that the merge's edge
was received. The merge itself still comes from the broker's double-precision verifier output: the broker owns the
global estimate. Integrating received edges into agent 1's graph is Phase 3, and merging from the float32 wire copy
would change results. `sim_collab` writes `contracts.yaml` with the per-hop counts, the per-field gaps, and the
declared-unset fields. Every output stays byte-identical: the EuRoC MH_01 estimate and results, and the 1.6 CSVs,
results and both estimates, all match. The gate re-proves this against all committed evidence.

**Determinism means agreement with independent runs, with named exclusions only.** The gate compares its outputs
file by file with the 1.6–1.9 evidence, which came from four other commits and processes. Exclusions are per-file
line regexes, and each must carry a reason, `run_identity` or `timing`; the checker refuses one without a reason.
The dev run's 12 differing files were all run ids, paths, commit, the spool.yaml hash (it embeds the run id), or
wall clock. `report.yaml`'s `inputs_clean` stays compared on purpose, so a dirty run cannot pass.

**Dev result (dirty tree, 2026-10-15).** Every I1, contract, stage and determinism check passes, except
`inputs_clean` (expected on a dirty tree). The **fresh Kaggle render is sha256-identical to the 1.7 render** (`cc3a2498…`):
gsplat rasterization is deterministic across T4 sessions for this package. Gauge note: keyframe 0's absolute covariance
is zero by definition (the gauge), counted as `gauge_zero_cov`, not as missing.

**Not gate items, reported.** §37.7 round trips for message classes with no schema yet, the ≤15% byte prediction
and stale-epoch rejection (Phase 3). Nightly CI of the thin run is open: it needs the 12 GB bag and a GPU.

## 2026-10-16 — D49: no nightly CI — the thin run is a local pre-merge gate

**Decision.** No nightly CI for the end-to-end run: "this is just a hackathon project", and "before we
check off each feature, we do run e2e once". Nightly infrastructure (a self-hosted runner, the 12 GB recording, a GPU)
would cost more than it saves before Oct 24. Unit tests alone are not enough: the 257 tests stayed green while §4.2 and
§18.2 went unused (D48). arch §0.2 is updated to say this.

**What.** `infra/scripts/phase1.sh --premerge` runs the same chain and checks on the working tree, into
`build/premerge/`, in ~20 min with no network. It runs before commit commands are handed over for any change that
touches the pipeline, and before a plan item is checked off. The full run (a clean commit and a fresh Kaggle render)
runs at phase gates and before submission. The GitHub `ci.yml` (build, test, cross-architecture determinism) is
unchanged.

**The render is reused, never faked.** Three Kaggle sessions gave the same `renders.npz` (`cc3a2498…`), so renders are a
deterministic function of the render package. Full runs store them in `build/render_cache/<package sha256>/`.
Pre-merge reuses them only for a byte-identical package. Otherwise it stops and asks for a full run (that branch was
tested in isolation). The splat check and the verdict line say "REUSED … NOT exit-gate evidence". The cache was seeded
from the exit-gate evidence run's render, which predates the cache code.

**One mode-only exclusion.** In pre-merge, `report.yaml`'s `inputs_clean` is excluded, under its own reason
`premerge_tree`. Full runs still require it to be true.

**Verified.** A pre-merge run on the working tree (commit e10dd8e plus these edits) gave VERDICT PASS, 54/54, in
19.5 min. From Phase 2 on, each phase's gate compares against its own committed evidence through a new config.

## 2026-10-16 — D50: XFeat runs as a cv::dnn graph WE assemble from upstream's weights; a GEMM reference checks it

**Context.** D15 chose "XFeat on OpenCV 5 primitives, no ONNX". The Phase 2 smoke test (SMOKE §4p) verified it.

**What.** `cpp/perception/features/xfeat/`: `atb.hpp` (a tensor blob, no dependencies) and `xfeat_net.{hpp,cpp}`. The
weights are upstream's (`verlab/accelerated_features` e92685f, Apache-2.0, credited), exported offline by
`python/model_tools/xfeat_reference.py` in the pinned `ariadne-xfeat-ref:1` image (`infra/scripts/models.sh xfeat`:
pinned commit, sha256-checked `.pt`, `--network none`, `weights_only=True`). BatchNorm (affine=False) folds into the
convolutions at load. Two engines run the same folded tensors:
- **`opencv_dnn_graph` (runtime):** the network is built layer by layer with `Net::addLayer`/`connect` (Convolution,
  ReLU, Pooling, Eltwise, Resize, Sigmoid). The 8×8 unfold is an exact one-hot stride-8 convolution. 12–15 ms per
  640×480/736×480 frame with 8 threads, 27–33 ms with one.
- **`gemm_reference`:** every op written out (im2col + `cv::gemm`). It is the readable reference, ~2.5–2.9 s because
  `cv::gemm` without external BLAS runs at 0.88 GFLOP/s.

**Why the graph API and not ONNX.** It keeps D15's claim intact: the architecture and forward pass are ours, OpenCV
5's dispatched convolution kernels execute it, and no importer coverage risk exists. Why not only the GEMM path: it
is ~200× slower.

**Acceptance (both engines, `tests/unit/test_xfeat_reference.cpp`, against upstream's own PyTorch outputs):** maps
≤ 8e-6 relative, 4096/4096 keypoints at the same pixel on both inputs, min descriptor cosine 1.000000. Upstream's
quirks are reproduced, not fixed, and are listed in SMOKE §4p. The test SKIPS without the export, but FAILS under
`ARIADNE_REQUIRE_MODELS=1`, which the pre-merge gate now sets. The engine that ran becomes a manifest field when the
engine is wired into the pipeline (2.1).

## 2026-10-16 — D51: the Phase 2 solvers run on OpenCV 5's `LevMarqBase`; our residuals, Jacobians and sparse algebra behind it

**Context.** Arch §11.6 specifies LM as `(H + λI)δ = −g` with its own λ schedule (λ₀ = 1e-4·max diag H, ×2 on
reject, ÷3 on accept, 15 iterations). Requirement: Phase 2 uses OpenCV natively as far as it honestly
goes. OpenCV 5 ships `cv::detail::LevMarqBase`, a Ceres-style LM driver over a pluggable `LevMarqBackend`.

**Decision.** The pose-graph solver (and, when 2.3 lands, the windowed BA) implements `LevMarqBackend`; OpenCV's
driver runs the LM loop. Ours: residuals, exact SE(3) Jacobians (`J_l⁻¹` with Barfoot's Q), block assembly, the
sparse `SimplicialLDLT` decomposition with a positive-pivot guard, the gauge (the anchor is excluded from the
parameter vector), and the left `exp` update. Design: `pose-graph.md`.

**This deviates from §11.6, and why that is acceptable.** OpenCV damps with Marquardt's `λ·diag(JᵀJ)` (clamped to
[1e-6, 1e32]) and adapts λ by step quality, not ×2/÷3. The optimum depends on the cost, not on the damping path, so
the GTSAM oracle (the §11.6 reference check) is unaffected. Every `Settings` field is set explicitly from config and
recorded in the manifest. Geodesic acceleration starts OFF and becomes an ablation. The arch doc §11.6 is updated
(§49 ritual).

**Not changed.** The two-frame BA stub keeps its own §11.6 LM (Phase 1 outputs stay byte-identical).
`cv::detail::PoseGraph` is NOT used: our pinned OpenCV has no `HAVE_EIGEN`, so its `optimize()` throws. Rebuilding
OpenCV would change the build hash, breaking the Phase 0 gate and the COOL baseline. Its quaternion-space residual
also differs from §12.2.

**Risk.** `cv::detail` is an internal namespace; the OpenCV tag is pinned (D7), so it cannot drift under us. The
contract (energy = ½Σ‖√Ω r‖², `jtb = +Jᵀr`, `solveDecomposed` solves `(JᵀJ+D)x = −right`) was read from
`levmarq.cpp` and OpenCV's own `PoseGraphLevMarqBackend`. A one-step linear test pins it.

## 2026-10-16 — D52: the Phase 2 solver cores, verified: PGO agrees with GTSAM 100/100; windowed BA matches OpenCV's PnP refine

**PGO** (`pose-graph.md`):
- exact `J_l⁻¹`, verified against its definition at every angle class;
- gauge null space exactly 6, "the same ξ on every node";
- OpenCV `LevMarqBase` driver with the contract pinned by a one-step linear test;
- **GTSAM 4.3 oracle 100/100**, half the graphs with full correlated information: energy ≤ 1.5e-12 relative, poses
  ≤ 1e-6 absolute (99) or ≤ 1e-6 Mahalanobis² (all).

Deviations recorded in the design:
- dead-reckoning shared init, because GTSAM's `InitializePose3` threw on one case [UNVERIFIED, open for 2.5];
- the anisotropic-noise generator bug, which drove both solvers to different local minima (an initialisation test,
  not a solver test);
- the scale-invariant pose criterion.

`pgo_digest` is required in every manifest; it is same-architecture only, because the SE(3) maths uses libm.
After a solve, node covariances are the marginals at the optimum, or NaN by explicit choice, never stale.

**Windowed BA core** (`local-slam.md`):
- `window_ba.hpp` (OpenCV-free): left-perturbation `T_W_C`, Huber/IRLS assembly, Schur elimination after damping;
  the guard refuses non-PD systems;
- `window_ba_levmarq` on OpenCV's driver;
- Jacobians match finite differences, including 0.05 m depth and |x/z| up to 2.5 (exit-gate item);
- gauge exactly 6, with landmark rows `[I, −[X]×]`;
- the gradient is exactly the robust cost's; Schur equals the dense solve;
- **one free pose agrees with `cv::solvePnPRefineLM` to 1.3e-9 m / 2.8e-9 rad**;
- the camera covariance equals the dense-inverse block.

**Not yet:** BA integration into `StereoVo` (window, keyframe score, lifecycle, exit edges) and the EuRoC/sim runs
against the targets fixed in `local-slam.md` §8.

## 2026-10-16 — D53: Phase-1 correctness fixes found by the first full EuRoC sweep (byte-identical against the Phase 1 evidence)

The Phase 1 evidence covered MH_01 only. Running all 11 EuRoC flights exposed four defects in 1.x code. Each is
fixed with a test. The pre-merge gate stayed byte-identical (54/54, ctest 286/286,
`build/premerge/aadffb8df5790ca0-20261016T173330Z`). That was verified on an intermediate working tree, before the
D54 changes. D53 and D54 land in the same commit, so no commit carries D53 alone:

- **OpenCV 5.0.0 USAC defect (SMOKE §4q).** `findFundamentalMat` (`UsacParams`) can throw `!model.empty()`. It
  aborted all six V-room flights; reproduced standalone, 62/200 seeds on V1_01 frame 1009. Only that throw is
  handled: up to 3 redraws from the same Philox stream, then the check is skipped for that frame. Both outcomes are
  counted. The first draw is unchanged. Env-gated replay test: `ARIADNE_USAC_FIXTURE`.
- **Unstable coasting.** While lost, the constant-velocity model re-derived its velocity from its own extrapolations.
  Rounding error in R's orthonormality grew ≈ 2.4× per frame, and MH_04 went non-finite after 48 lost frames.
  The measured velocity is now frozen while coasting.
- **Silent NaN.** A non-finite pose or ATE now stops the run with the frame, the lost-streak length and the last
  finite position. It previously produced a NaN score with exit 0.
- **`PoseGraph::AddEdge`** refuses information that is not finite, symmetric and PSD. The SE-Sync cubicle/rim files
  carry 5021 and 8815 indefinite matrices; our LM had correctly refused to factor them, while GTSAM did not check.
  The SE-Sync test excludes exactly those two files and switches to an energy + stationarity criterion; evidence in
  `pose-graph.md` §5.

## 2026-10-17 — D54: frontend re-baseline — tracked points become map points again (option A)

`docs/design/local-slam.md#window-audit` traced the hard flights' lost frames to how tracked points become 3-D points. Feature
quality and the 2.3 backend were not the cause.

- Fixes 1 and 2 were proven by one-setting interventions before landing.
- Fix 3 (exposure) was proven by an on/off pair after implementation.
- Fix 4 (time-based) is unit-tested, but **no intervention showed it matters on EuRoC**. It is a correctness fix for
  dropped frames.

All are done with the current features. These are Phase-1 frontend fixes, so the v1 numbers and the Phase-1 sim outputs
change **by design**, and the gate is re-pinned from a clean-commit full run.

1. **Online stereo row refinement.** The V2 recordings sit ≈ 1.7 px off their published calibration (rows), and the
   1 px row gate threw away 97% of correct stereo matches.
   - The residual `dy = a + b x + c y` is fitted from round-trip-consistent LK matches with no row gate. Samples
     pool across keyframes, fitting is trimmed least squares, and estimates beyond the search bound are refused
     loudly.
   - The fit is folded into the right rectification map as `A·P2`. P2, Q and `Rig()` are untouched.
   - Acceptance: V2_01's residual drops from 1.46 px to 0.09 px.
   - It is disclosed as online calibration refinement in `results.yaml` and in the manifest field
     `stereo_row_refinement`.
2. **Far points kept.** The 60-baseline discard misread ORB-SLAM, which classifies far points rather than dropping
   them (ORB-SLAM2 §III-A). The relative-σ gate is now the depth bound (≈ 9.6 m on EuRoC). Multi-view triangulation
   beyond it is not implemented.
3. **Exposure match** before stereo LK and SGBM: a global gain/bias match, needed for EuRoC's per-camera
   auto-exposure. It lands on evidence: V2_03 v1 lost 1026 → 192 frames.
4. **Time, not frame counts.** Coasting predicts by elapsed time (velocity per second), the keyframe cap is
   `max_interval_s`, and RPE pairs samples 1 s apart by timestamp. V2_03 drops 414 frames.

σ was re-measured by the same protocol (MH_01 at σ = 1): 0.453 px, within 10% of 0.46, so 0.46 is kept.

**Result** (11 EuRoC flights; v1 = the 1.5 placeholder, v2 = 2.3; ATE in m):

| flight | MH01 | MH02 | MH03 | MH04 | MH05 | V101 | V102 | V103 | V201 | V202 | V203 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| v1 before | 0.126 | 0.081 | 0.140 | 3.93 | 0.82 | 0.152 | 0.218 | 1.18 | 2.36 | 2.47 | 3.72 |
| v1 D54 | 0.112 | 0.128 | 0.151 | 1.60 | 0.257 | 0.162 | 0.226 | 1.74 | 0.105 | 0.213 | 1.65 |
| v2 D54 | **0.075** | **0.075** | **0.093** | 2.78 | 0.560 | **0.078** | **0.113** | 0.439 | **0.083** | **0.124** | 2.12 |

- **2.3 against its target** (`local-slam.md` §8): v2 beats v1 on **8/11** and lies within the worst published
  stereo class on **7/11**.
- **v1 got worse on 5 of the 6 already-clean flights**: MH_02 0.081 → 0.128, V1_03 1.18 → 1.74, and slightly
  MH_03, V1_01 and V1_02. **[UNATTRIBUTED]**; the likely cause is far points feeding the two-frame stub. Stated, not
  hidden (invariant 7).
- After D54, **55.6% of window solves stop at the 10-iteration cap** (`found = false`); it was 23% at σ = 1 before
  D54. §11.6 says 15. Open.
- **Every remaining failure is a flight that still loses more than 100 frames** (MH_04 266, V2_03 181, V1_03 139,
  MH_05 100). There the window is worse than v1: lost stretches poison the map (AUDIT §4), and a 10-keyframe window
  spreads the damage.

**Next:** lost-frame handling, then 2.1 / 2.4.

- **Lost-frame handling (AUDIT §4):** no keyframes and no new points at a coasted pose, a new map segment on loss,
  and a lost-status field on keyframe records.
- **2.1** (XFeat + re-association) and **2.4** (IMU bridging), for the losses that remain.

## 2026-10-17 — D55: the gate's determinism references re-pinned to a clean post-D54 run; the sim change attributed

D53/D54 change the Phase-1 VO **by design**. The full gate on the dev tree had 32 determinism failures, all
VO-derived; the digests (det_math, philox, degradation, lens) were identical. This decision re-pins
`configs/benchmark/phase1_gate.yaml` to a full run from clean commit 150ce28 with a fresh Kaggle render, and records
what changed on the sim and why, so the new references do not bake in an unexplained regression.

**The headline sim number got worse, but it measures one loop edge, not the VO.** `merged_two_agent_ate_rmse_m`
rose from 9.8 to 14.4 mm.
- The 1.6 placeholder merges the two maps with ONE edge: the best-ranked verified candidate
  (`ariadne_sim_collab.cpp`, "merge with the best verified edge").
- The keyframe timing changed, so a different keyframe pair now ranks first. The reference edge was 11.6 mm / 0.19°;
  the new one is 18.8 mm / 0.33°.
- In every variant below, the top three verified edges span 10–29 mm and 0.18–0.53°. The merged number is one draw
  from that spread. The depth-cap variant drew a 10 mm / 0.18° edge and scored 9.1 mm.
- The real merge is the Phase 3 pose graph over all edges. Until then the merged number is reported **next to** the
  per-agent numbers, never alone.

**Per-agent attribution** (the same bag, the same build; ATE translation mm / rotation mrad):

| run | agent 0 | agent 1 | merged mm | merge edge |
|---|---|---|---|---|
| reference (Phase-1 evidence) | 9.9 / 3.2 | 4.4 / 2.6 | 9.8 | 11.6 mm, 0.19° |
| D54 (current) | **8.4** / 4.9 | **4.3** / 4.6 | 14.4 | 18.8 mm, 0.33° |
| row refinement off | 7.6 / 4.3 | 4.6 / 4.2 | 13.3 | 18.1 mm, 0.31° |
| exposure match off | 8.5 / 4.8 | 4.5 / 2.9 | 15.2 | 21.4 mm, 0.37° |
| 60-baseline depth cap restored | 10.5 / 4.5 | 5.6 / 3.2 | 9.1 | 10.0 mm, 0.18° |

- **Translation improved** on both agents.
- **Rotation RMSE rose** by about 50% on both agents: 3.2 → 4.9 and 2.6 → 4.6 mrad.
  - Restoring the depth cap recovers part of it on agent 1 (3.2 mrad), but costs translation on both agents (10.5,
    5.6 mm). Far points are kept.
  - Row refinement and exposure match each move it by less than the agent-to-agent spread.
  - The rest is **[UNATTRIBUTED]** among the remaining sim-relevant D53/D54 changes: time-based motion and keyframe
    timing, the live-track culling exemption, the χ² statistics. sim_v1 runs no window BA.
  - EuRoC does not show it. v2 rotation RMSE held or improved after D54 on every clean flight (e.g. MH_01 59 → 47,
    V1_03 231 → 177 mrad), and so did v1 (MH_01 77 → 54, MH_02 65 → 39 mrad).
- **Row refinement on a perfect rig** fits a −0.24 px offset (agent 0) from one 115-match pool. The post-refinement
  check reads +0.18 px, so that first estimate was mostly scene-dependent LK bias.
  - On EuRoC the same check confirms the real offsets: V2 at 1.3–1.7 px, and V1_01 at −0.30 px with a residual of
    0.01 px.
  - A magnitude gate (e.g. 0.5 px) would wrongly drop V1_01, so none is added.
  - A residual of ≤ 0.3 px is within the 1 px row gate and is harmless here: sim with refinement off is better on
    agent 0 and worse on agent 1.
  - Open: decide from several independent pools instead of the first one to reach `row_min_matches`.

Variant runs: `build/sim_attr/{current,norow,noexp,zcap}`. The depth-cap variant came from a temporary, env-gated
diagnostic build, reverted at once; it is not in any commit.

**The re-pin.** `configs/benchmark/phase1_gate.yaml` now points every reference at
`evaluation/runs/gates/phase1_thin_e2e/exit_gate/aadffb8df5790ca0-20261017T072201Z`. That is one full gate run from clean commit
150ce28 (`git_dirty: false`) with a fresh Kaggle render.
- Its own verdict against the old references was FAIL on exactly the 32 VO-derived comparisons, and PASS on the
  other 22: I1, the §4 contracts, every stage.
- The det_math, philox, degradation and lens digests were unchanged.
- A pre-merge run against the new references passed **54/54**, render reused (`build/premerge/…-20261017T080657Z`).
  It ran on a working tree whose only code change writes a new per-frame `vo_trace.csv` (D56), which no gate
  comparison reads.
- The old per-item evidence stays committed where it is.

## 2026-10-17 — D56: lost-frame handling — the visual rotation bridge, a decaying velocity, tagged segments

Design and evidence: `docs/design/local-slam.md#lost-frames`. It answers `local-slam.md#window-audit` §4.

**Diagnosis** (new per-frame `vo_trace.csv`):
- In a lost stretch the landmarks fail, but the 2-D tracks survive (40–360).
- Coasting at a frozen constant velocity turned MH_04's 10 s, 9 cm dark hover into 7.6 m and 117°.

**Decision (v2 configs; v1 unchanged, it stays the 1.5 baseline):**
- **`lost.motion: bridge`.**
  - Rotation is MEASURED from the 2-D tracks by seeded USAC H and E (OpenCV 5 `geometry`), selected by ORB-SLAM's
    published score rule (T-RO 2015 §IV-B).
  - It is measured from an ANCHOR (the last measured frame, or the last frame 20 tracks reach back to), so there is
    one two-view estimate per frame, not a chain.
  - Translation is the last measured velocity, clamped to 3 m/s and decaying as `exp(−t/0.5 s)`.
- **`lost.reanchor: every_frame`.** The `gated` alternative (ORB-SLAM3's no-keyframes-while-lost) RAISED lost
  frames (V2_01 20 → 120 bridged) and is rejected: it is kept as an ablation.
- **Provenance:** `PoseSource {measured, bridged, coasted}` and a segment counter (loss onsets) on every frame and in
  `runtime::Keyframe` (`pose_source`, `segment`). Also in `results.yaml` `lost:` and manifest `lost_handling`.

**Evidence** (11 EuRoC flights):
- `coast` / `every_frame` reproduces D54 byte for byte, except wall-clock timing.
- The bridge: MH_04 2.78 → **0.36** m (rotation 55° → 1.8°), V2_03 2.12 → **1.27**, MH_05 0.56 → **0.35**.
  - Small costs: V1_03 0.44 → 0.56, attributed to the decay (no decay 0.46, τ 0.2 0.70); V1_02 0.113 → 0.137.
- **Against `local-slam.md` §8:** within the published class **9/11** (was 7/11); better than the placeholder
  **10/11**.
- **Held-out per-pair accuracy** (frame-to-frame bridge on 25,606 tracked pairs vs ground truth): H median 0.21°,
  E median 0.036°, p99 ≈ 1.1°; robust σ ≈ (0.09°, 0.09°, 0.04°) per pair.
- Lost-frame COUNTS barely move (MH_04 278 → 264). This limits the damage of a loss; 2.1 and 2.4 must cut the losses.
- Cost: VO tracking 9.3 → 12.4 ms per frame (the bridge runs on every frame, for the held-out check).

**Rejected on the way, each by a failing test or a measurement (local-slam.md#lost-frames §4):**
- ORB-SLAM's ratio on inlier counts (blind to sub-pixel parallax; 0.1°/frame H bias, chained).
- "E unless it disagrees with H by > 5°" (breaks at anchor baselines).
- ReconstructF's cheirality test: `recoverPose`'s default overload counts only points within 50 baselines, none at
  1–2 cm/frame. This silently dropped E until `distanceThresh` was passed.

**Open:**
- Bridged edges keep the uninformative covariance until a NEES check. The measured σ is the input.
- The worst blur gaps need 2.4's gyro.
- Limit the bridge to lost frames once it is validated.

## 2026-10-17 — D57: two cheap accuracy levers measured and closed; the window keeps 10 iterations

Run from the committed D56 tree; the shipped `euroc_v2.yaml` reproduced the D56 sweep exactly on all 11 flights.

- **ORB-SLAM's trajectory protocol** (frames re-composed on the final keyframe poses,
  `System::SaveTrajectoryEuRoC`) is now reported next to the causal score, as `results.yaml` `keyframe_refined`
  (window backend only, so v1 bytes are unchanged). It differs by ≤ 1 mm on every flight.
  - The protocol caveat in `local-slam.md` §10 is closed: scoring is not part of the gap to ORB-SLAM3.
- **Window max_iterations 15** (§11.6) vs 10:
  - The share of solves that end at the cap drops from 45–69% to 28–47%.
  - ATE: MH_01 0.075 → 0.072, MH_03 0.093 → 0.086, V2_01 0.071 → 0.069; but MH_05 0.349 → 0.360 and **V1_03 0.56 →
    0.96**. Still 9/11 within the published class.
  - **Kept at 10.** At 15, a third of the solves still hit the cap, so the cap is not what limits them. The likely
    cause is the termination tolerances (`rel_energy_tolerance` 1e-6) [UNVERIFIED].
- **What remains of the gap to ORB-SLAM3:** data association (re-association, 2.1), loop closure (Phase 3), the IMU
  (2.4). It is not the protocol, and not the iteration budget.

## 2026-10-18 — D58: 2.4 step 1 — attitude_only ships for EuRoC v2: gyro rotation while lost, gravity on keyframes

Design and evidence: `docs/design/inertial.md`.

- **Validated first.** The gyro integral matches the ground-truth camera rotation to **0.019° per frame pair** with
  the ground-truth bias removed (26,530 pairs, 11 flights); 0.23° raw. Timing is aligned within a sample.
- **Gyro bias is VO-aided** (a robust median of `(Log R_gyro,raw − Log R_vo)/dt` over 200 tracked pairs; seeded by a
  stationary test). Mahony cannot observe the gravity-axis bias.
- **While lost, the gyro replaces the D56 visual bridge's rotation** (`PoseSource::kGyro`).
  - EuRoC v2 average ATE 0.289 → **0.223 m**; V2_03 1.27 → **0.68**, V1_03 0.56 → 0.42, V1_02 0.14 → 0.10.
  - Costs: V1_01 +17 mm, MH_04 +30 mm.
  - Stereo class: **10/11**. Stereo-inertial class: not yet (`full` needed, as planned).
- **Gyro-seeded LK** (Hwangbo 2009) is built as a one-hunk seam in `klt_tracker.*` (approved), tested,
  and **off**. Lost frames −3% only; V1_03 0.42 → 0.82.
- **Gravity** (Mahony 2008 Theorem 5.1, the paper's kP = 1, kI = 0.3) is on every keyframe record. Its error against
  ground truth is a median 1.3–4.2° per flight, 15° worst: MAV dynamics leak through. Recorded so that a
  GRAVITY_PRIOR uses an honest σ. The factor itself is 2.5's (pose graph). [Corrected in D59: the plan puts
  producing gravity-prior edges in 2.4; 2.5 consumes them.]
- `imu.mode` defaults to `none`: v1, `sim_v2` (no sim IMU plumbing yet) and the sim gate are unchanged. Pre-merge
  **PASS**.
- **Decision recorded for `full`:** window exit carries a loose bias prior forward; poses and velocity
  still drop and emit an edge.

## 2026-10-19 — D59 (part 2): 2.4 full — inertial window BA, initialisation + refinement, opt-in `euroc_v2_full`

Design and evidence: `docs/design/inertial.md` §4 (as built). Part 1 (8691fb3): the preintegrator and factor.

- **Window BA.** Velocity and biases are a separate segment of the reduced system (`[poses | v,b_g,b_a | landmarks]`):
  Schur, marginals and the exit edge unchanged; with no inertial blocks every number is bit-identical (MH_01 on
  `euroc_v2` reproduces D58 byte for byte). Energy ½rᵀΩr, no kernel; bias random walk; window[0]: pose fixed,
  v and b free, bias prior = previous marginal × 10 (carried by the random walk when degenerate or unsolved).
  Tests: FD gradient and Hessian of every inertial block, Schur = dense, noise-free VI window → truth.
- **Correction while building (§4.2):** the IMU does not observe velocity (Galilean null space, tested), so an inertial
  chain ties keyframes only out of a group with two pose-determined keyframes. **Decision:** in a window
  whose window[0] has no covisible partner, v₀ is held at the previous solve's value instead of holding lost
  keyframes at predicted poses. On EuRoC this triggers in up to ~25% of solves on the dark flights (MH_04, V2_03),
  ~0–5% on the easy ones: the oldest keyframe often shares fewer than `pnp.min_correspondences` landmarks.
- **Initialisation** (`slam/inertial/initialization.hpp`) after 10 measured keyframes: b_g by Gauss–Newton on the
  keyframe rotations (**changed from §4.3's "from the attitude tracker"**, recorded there), then v, g, b_a as a linear
  MAP (ORB-SLAM3's inertial-only MAP, scale known), g on the 9.81 sphere; refused on |g| ±10 % or χ²/dof > 100.
  The threshold comes from the 11 EuRoC inits checked against GT: the one off by 8.4° (MH_04) had χ²/dof 1296,
  all others ≤ 55. **Refinement** re-solves over all keyframes since init at +5 s and +15 s (ORB-SLAM3 §V-B).
  Synthetic: exact with a flat b_a prior (1e-6); b_a/gravity-tilt trade-off quantified (0.44° ↔ 0.076 m/s²).
- **EuRoC (`euroc_v2_full`, opt-in):** average ATE **0.223 → 0.137 m** (target ≤ 0.138 on the average); **per flight
  7/11** inside the stereo-inertial class (V1_03 0.254, V2_01 0.077, V2_02 0.107, V2_03 0.281 out). Inertial
  χ²/dof at the solutions 0.38–1.23 (validates the ×3 / ×9 inflation, frames and lever arm). Costs vs
  attitude_only: MH_01 +2 mm, V2_01 +7 mm.
- **Open defect (part 3):** the estimated gravity, in each keyframe's own body frame, degrades over a run on the hard
  flights (V2_03 median 3.5°, final 5.6°): window[0]'s full 6-DoF pose is the gauge, so gravity cannot correct the
  trajectory's roll/pitch drift (a VIO gauge is 4-DoF).
- **§4.4 not decided.** Fake outages (0.5–10 s, MH_01/MH_02/V1_01/V2_01): IMU propagation drifts less to 2 s (0.24 vs
  0.45 m), then diverges (10 s: 11 m vs 3.1 m), consistent with the gravity defect (δg T²/2). The metric also scored
  the wrong instant. Re-measured after part 3 with drift to ~2–3 s after re-anchoring; `decay` stays the default.
- Pre-merge **not run** (2026-10-18 rule: only when absolutely needed); the EuRoC sweeps and unit tests are
  the evidence. `euroc_v2` (attitude_only) and `sim_v2` (none) unchanged.

## 2026-10-19 — D59 (part 3): bias prior never looser than the carried prior; gravity-in-window is a negative result

Design and evidence: `docs/design/inertial.md` §4.8–4.9.

- **Gravity's direction in every window + rotate-back (a 4-DoF gauge in effect): built, tested, NEGATIVE, off**
  (`imu.full.estimate_gravity: false`, kept as an ablation). EuRoC average 0.137 → 0.566 m, 0/11: a 10-keyframe
  window cannot separate tilt from b_a, and the ×10 prior bookkeeping loosened the unobserved tilt every window
  (σ 3.6° → 12° on MH_01). ORB-SLAM3's local VI BA also anchors on fixed keyframes (§V-C, §VI-C); long-term tilt
  there comes from map reuse and loop closure (2.1 / Phase 3 here), so part 2's gauge stands.
- **Bias prior fixed by the same lesson.** The ×10 deflation applied to the whole marginal lost the prior's own
  information on unobserved directions (b_a in dark windows; V2_01 propagate 10 s: |b_a error| 0.15 → 2.0 m/s²).
  Now Λ_next = Λ_carried + PSD(Λ_marg − Λ_carried)/κ: only the window's added information is deflated.
  **EuRoC `euroc_v2_full`: average 0.137 → 0.121 m, per flight 7/11 → 9/11** (V1_03 0.357 and V2_02 0.101 out).
  Bias and velocity errors vs GT drop on every hard flight; V1_03's ATE rises 0.254 → 0.357 despite better states
  (reported as measured).
- **§4.4 `lost_motion`: decided by measurement, stays `decay`.** With `reanchor: every_frame` each lost frame is an
  inertial-window keyframe, so the prediction only seeds the solve: decay and propagate are identical to the mm.
  Full vs attitude_only across outages (2.5 s after re-anchoring): better up to 5 s (2 s: 0.18 vs 0.61 m), worse at
  10 s (4.6 vs 3.1 m): long IMU-only stretches diverge; bounding them belongs with map segments (2.1 / Phase 3).
- ctest 328/328. `euroc_v2` (attitude_only) byte-identical to D58 on MH_01; `euroc_v2_full` stays opt-in pending
  a switch decision. Pre-merge not run (by rule: targeted runs suffice).

## 2026-10-19 — D59 (switch): `euroc_v2` is imu.mode full; attitude_only kept as `euroc_v2_attitude`

- **Decision.** EuRoC average ATE 0.121 vs 0.223 m, 9/11 vs 4/11 flights in the stereo-inertial class
  (inertial.md §4.9). Costs: MH_03 +6 mm; V1_03 (0.357) and V2_02 (+1 mm) stay out of class.
- `configs/slam/euroc_v2.yaml` takes the full-mode values of `euroc_v2_full` (removed: identical apart from its name;
  the D59 evidence directories keep that name). `configs/slam/euroc_v2_attitude.yaml` is the D58 configuration,
  kept as the baseline/ablation.
- **Verified by targeted runs, not the pre-merge (by rule):** the new `euroc_v2` reproduces the
  committed full-mode V2_03 trajectory byte for byte, and `euroc_v2_attitude` reproduces D58 on MH_01 byte for byte.
  `sim_v2` (imu none) and the sim gate are untouched.
- **Ownership corrected:** gravity-prior edges (arch §10.4) are a 2.4 deliverable per the plan, produced here and
  consumed by 2.5's pose graph; the D58 entry had assigned them to 2.5. They are the next item.
- **Merge note for 2.1:** 2.1's D60 bars were measured against D58 (attitude_only). After merging, EuRoC needs one
  combined sweep; the two gains (re-association, IMU) both act on the lost stretches and need not add.

## 2026-10-19 — D70: chordal initialisation for the pose graph; pgo_v2 drops OpenCV's absolute damping floor; robust backend placed in `cpp/slam/robust/`

**Context.** Plan 2.5 opens with arch §13.1's chordal initialisation, ahead of the degeneracy gate → PCM → GNC-TLS
order of §14.5. Three open items came with it: (a) GTSAM `InitializePose3` throwing in the oracle, (b) parking-garage
needing 1000 LM iterations where GTSAM needs 8, (c) cubicle and rim refused for indefinite information. Design:
`robust-backend.md` §1–§3, §9.

**Decision.**
1. **Chordal initialisation** (`cpp/slam/pose_graph/chordal_init.{hpp,cpp}`). It solves one sparse linear least-squares
   problem for the rotation rows (κ-weighted, hard anchor, three right-hand sides). Each block is projected to SO(3)
   with `cv::SVDecomp` and the determinant fix, then translations come from a τ-weighted Laplacian. κ and τ follow
   SE-Sync's isotropic conversion (`τ = 3/tr(I_t⁻¹)`, `κ = 3/(2 tr(I_R⁻¹))`), shared with D74.
   - **Baselines (§49 ablation):** dead reckoning and a `cv::buildMST` spanning tree (Prim, highest κ).
   - **Gate (decision 2026-10-18, design §9):** on the SE-Sync graphs where our LM from dead reckoning
     fails, chordal + LM must reach GTSAM's minimum.
     - The failure set is **torus3D and grid3D**, where GTSAM's LM from the same start stops in the same local
       minima (29950.006 and 253372.10).
     - Chordal + LM reaches the reference (12117.637 and 42179.979) in 6 and 8 iterations. On all six graphs it
       ends within 7.0e-9 relative, stationary, in 4–15 iterations. MST rescues both as well.
     - This matches Carlone et al. 2015, Table I, on the four graphs the two studies share. **PASSED.**
2. **pgo_v2 = pgo_v1 with `clamp_diagonal` off**, as the preset `PgoV2()` and `SolvePoseGraph`'s default.
   - **Why:** OpenCV clamps the damping `λ·diag(H)` to an absolute [1e-6, 1e32]. Ceres, which it mirrors, clamps
     `diag(JᵀJ)` before applying λ.
   - **Effect on garage:** with the floor, 95 iterations and a stop 3.7e-5 above the minimum at production
     tolerances; without it, 15 iterations, at the minimum.
   - **Open item (b) is reduced, not at parity:** 15–18 iterations against GTSAM's 7.
   - **No regressions:** the 100-case oracle 100/100 (worst 1.48e-12 relative); the SE-Sync graphs reach the same
     minimum as GTSAM on all six; ctest 300/300 (8 env-gated skips, the oracle results cited separately).
   - **The struct defaults stay pgo_v1** because the windowed BA builds on them. Flipping them would have changed
     odometry silently.
   - **Guard owed to D73:** the floor also regularised a rank-deficient H, so GNC must anchor every connected
     component of the active graph, with its own test.
3. **Oracle corrections** (`python/oracle/pgo_gtsam.py`, oracles only):
   - GTSAM's `Rot3.Quaternion` does not normalise, and the g2o files are off unit length by up to 6.8e-7. The old
     SE-Sync oracle therefore solved a slightly different problem, which was garage's "different minimum". Now
     normalised, and `SeSyncBenchmarkGraphs` asserts equal initial energies.
   - GTSAM's chordal soft-anchors node 0 at unit precision, against edge weights near 15, which costs 6.5e-3 rad.
     The oracle scales the edge information by 2⁻⁵⁴ to make the anchor effectively hard.
   - New cases go in the branch cache only. The shared `.cache/oracle/*` is untouched.
4. **Open item (a), diagnosed.** `InitializePose3` throws on every case without a `PriorFactorPose3`, because
   `NonlinearEqualityPose3` is dropped and the anchor is left disconnected. With a unit prior, 3 of 100 cases throw.
   - The throw is in multifrontal Cholesky only: QR and sequential Cholesky succeed on the same linearisation, and H
     is positive definite. So this is a false positive in GTSAM, not a problem with the graphs.
   - The reference start falls back to GTSAM's chordal rotations. The original "case 2" was never committed and is
     not reproduced [UNVERIFIED].
5. **Placement (arch §3 updated, §49).** The degeneracy gate, PCM, GNC and the certificate go in `cpp/slam/robust/`,
   not `perception/` and `collaboration/`. All of them act on pose-graph edges and must also run intra-agent, with no
   broker. `loop_closure/` holds the intra-agent candidate source (D73).

**Not done here, stated.** No runtime path solves the pose graph yet, so the `pgo_init` config selection, a
`configs/slam` PGO entry and the manifest fields land with the first caller (D73). Open item (c) belongs to D74.
grid3D dominates the 2.6 h gate. The gate is an env-gated evidence run, never part of `--premerge`.

**Evidence.**
- `PgoInit.SeSyncDeadReckoningGate`, `PgoOracle.SeSyncBenchmarkGraphs`, `PgoOracle.DampingAblation`,
  `PgoInit.ChordalRotationsAgreeWithGtsam` (50 isotropic cases within 2.6e-10 rad of GTSAM's chordal).
- `test_chordal_init.cpp`, 4 tests: the determinant fix, κ/τ, exact noise-free recovery, refusal of a graph
  disconnected from the anchor.
- algorithms.md §7.2a.

## 2026-10-19 — D71: loop-closure degeneracy gate on the loop's own covariance (marginal σ_t / σ_r, parallax, collinearity), stereo depth noise propagated

**Context.** Arch §14.4 puts a degeneracy gate before PCM. A degenerate PnP loop carries an honest but huge
covariance, so it is consistent with almost anything and PCM cannot discriminate it. The arch criterion is
`λ_min/λ_max < τ` on the Hessian, which mixes metres and radians and depends on the frame. The design also assumed
that 3D point noise could be ignored. Design: `robust-backend.md` §4.

**Decision.** `cpp/slam/robust/degeneracy.{hpp,cpp}`, `AssessLoop`, a pure library with no wiring.
- **Pose refinement:** the PnP pose is refined with `cv::solvePnPRefineLM`, then Gauss–Newton weighted by per-point
  pixel covariances that carry B's stereo noise in (u, v, disparity) (Vakhitov et al., CVPR 2021).
- **Covariance:** H⁻¹ inflated by max(1, σ̂²), in the left tangent, converted to the pose-graph residual tangent by
  `OdometryEdge`'s formula.
- **Gates:**
  - worst-direction 1σ of the translation marginal [m] and the rotation marginal [rad], absolute bounds (Zhang 2016);
  - too few inliers with a stereo triangulation angle ≥ θ_min;
  - collinearity: the 3D inliers lie on a line (λ_mid/λ_max of their covariance).
- **Reported only:** the within-block ratios, pixel spread, planarity and the corridor score.
- **§49:** the arch §14.4 criterion is replaced, with a dated update recorded there; algorithms.md §7.8a.

**Evidence** (`tests/unit/test_degeneracy.cpp`, 8/8; ctest 308/308, 8 env-gated skips):
- the finite-difference check of the stereo back-projection Jacobian;
- the information equals `OdometryEdge`'s conversion;
- Monte Carlo mean NEES **5.95** over 500 trials (χ²₆ mean 6) at 1.5–12 m with pixel and disparity noise. At
  25–40 m it is **54.7**: the linearised covariance is about 9× overconfident at far range, which is the measured
  reason parallax is its own gate;
- the control passes. A 3D line is named collinear, far-only points are named for parallax, and the small object
  is named for conditioning;
- **Collinearity moved from pixels to 3D after measurement.** A pixel-spread gate refused a well-conditioned planar
  fan (6.4 mm, 1.35 mrad). Conditioning alone missed a 3D rail (8.3 mrad reported), because stereo noise fabricates
  the missing rotation information. The fan is now a must-not-refuse test.
- **Scene correction:** 120 points on a 40 px patch measured 2.7 cm and 0.4°, a usable loop. The scene was corrected
  to 25 points on a 20 px patch; the threshold was not loosened.

**Not done, stated.**
- **Thresholds** (σ_t 0.10 m, σ_r 0.02 rad, 10 points at ≥ 1°, spread 0.01, σ_px 0.46, σ_d = √2 σ_px) are
  [ASSUMPTION]. Calibration follows Zhang's margin-midpoint rule on real candidates with ground truth, protocol fixed
  in design §4, run in D73.
- **Not wired.** `VerifyResult` does not export inlier correspondences, and the broker's `ProxyInformation` stays,
  because replacing it changes the Phase-1 sim_collab outputs. Both need a decision before D73.
- **Honest covariance is shown at 1.5–12 m only.** On the small-object scene the residuals are about 3× the model
  (σ̂² 3.2), so the covariance is honest there only through the inflation.
- **Not timed.** It is O(N) per loop with a 6×6 solve [UNVERIFIED].

## 2026-10-19 — D72: PCM against the odometry-only chain, with a per-loop (Bonferroni) pair threshold

**Context.** Arch §14.5: pairwise cycle consistency (Mangelson et al., ICRA 2018) at χ²₆(0.95), a maximum consistent
set by greedy maximal clique, exact for ≤ 25, after the degeneracy gate (D71) and before GNC (D73).
Design: `robust-backend.md` §5.

**Decision.** `cpp/slam/robust/pcm.{hpp,cpp}`, `Pcm(PoseGraph)`, a pure library with no wiring.
- **Odometry legs:** prefix-composed adjoints on the odometry-only chain (`kOdometry` and `kStereoOdometry`).
  Non-chain components throw.
- **Shared odometry edges:** when both legs run along one chain, each shared edge is counted once with summed,
  signed coefficients.
- **Groups:** per pair of odometry components. One candidate is accepted as "untested". Two or more candidates with
  a maximum clique of 1 accept nothing ("no_consensus").
- **Clique:** greedy CLIQUEHEU (Pattabiraman et al.); exact Bron–Kerbosch behind a flag. Loops are ordered by
  (from, to), so the result does not depend on insertion order.
- **Threshold (§49 deviation, arch §14.5 updated):** per-loop false rejection α_loop/(n − 1) per pair, with
  α_loop = 0.001 derived from the recall requirement. The fixed χ²₆(0.95) is kept as the ablation baseline.

**Why the threshold changed.** A maximum clique keeps a true loop only if all of its pairs pass. At 5 % per pair the
fixed quantile measured recall 0.84–0.86 mean and 0.60–0.70 worst. The acceptance criterion was fixed beforehand:
recall ≥ 0.95 on every seed at ≤ 40 % outliers, because a dropped true loop is unrecoverable and outliers are GNC's
job.

**Evidence** (`tests/unit/test_pcm.cpp`, 12/12; ctest 355/355 on the merged tree):
- **Odometry legs:** Monte Carlo NEES 6.01 and 6.24.
- **Pair statistic** (mean, χ²₆: 6): 5.90, 6.15, 5.95, 5.93 and 5.75 for disjoint, nested, crossing, backward and
  shared-end legs; 6.07 and 5.96 for inter-agent pairs with loop 2 forward and reversed.
- **Cliques:** greedy = exact on 200/200 planted-clique graphs.
- **Random outliers, 10–40 %:** recall **1.000** on every seed, precision 1.000.
- **Near-miss outliers:** precision 0.75 at 0.2 m / 1°, 0.94 at 0.5 m / 5°, 1.00 at 1 m / 10°. Recall stays
  0.99–1.00, against 0.83–0.85 for the fixed quantile.
- **Blind spot:** 5 aliased, mutually consistent wrong loops beat 3 true ones. Asserted and labelled.
- **Timing:** 186 ms for 300 loops on 3000 nodes.

**Decision (2026-10-19), following the review's recommendation: the per-loop threshold is the default.**
- **The two failures are not symmetric.** True loops dropped by the fixed quantile cannot be recovered; near misses
  passed by the default are GNC's job. A 0.5 m / 5° near miss has rᵀΩr ≈ 176, against GNC's c̄² = 16.8.
- **Stated trade:** the bound loosens with group size (27.6 at 10 loops, 30.3 at 30, 33.1 at 100, 35.6 at 300,
  against 12.59).
- **Blind spot:** unchanged under either threshold.
- **Condition for D73:** GNC's acceptance runs on PCM's output with 0.5 m / 5° near misses and requires precision
  ≥ 0.99 on them; 0.2 m / 1° near misses are scored by ATE. If GNC fails, the first remedy is α_loop = 0.01, not the
  fixed quantile.

**Not done, stated.**
- Near misses near the loop noise pass PCM by design; D73's GNC test must include them.
- D56 lost stretches are bridged by identity-covariance edges, so PCM is blunt across them. Not measured on real
  data.
- The greedy clique can return a smaller set than exact. It didn't on the test graphs, and a shuffled-order run
  before the tie fix showed 12 against 13.
- Not wired: D73 builds the loop source → gate → PCM → GNC.

## 2026-10-20 — D73 part 1: GNC-TLS on PCM's output; every connected component anchored

**Context.** Arch §14.6: GNC-TLS (Yang et al., RA-L 2020) on the loops that survive PCM, odometry at w = 1, weight
trajectories exposed. pgo_v2 (D70) removed OpenCV's damping floor, which had been the only regulariser for a graph
split by rejections. Design: `robust-backend.md` §6. D73 is split into three parts (design §6); this is part 1.

**Decision.**
- **`cpp/slam/robust/gnc.{hpp,cpp}`:** whitened r² = rᵀΩr, c̄² = χ²₆(0.99) = 16.81, Prop. 4 weights, μ₀ =
  c̄²/(2r²max − c̄²), μ ×1.4.
- **Stopping:** on weights within 1e-4 of {0, 1}, a relative cost change below 1e-5, or 100 outer iterations.
- **Execution:** runs on a working copy. PCM's accepted set arrives as an edge mask, so indices stay stable. The
  input graph gets the final poses and weights once (new `PoseGraph::SetGncWeight`, loop edges only, w ∈ [0, 1]).
  Inner solves skip marginals.
- **Conventions:** checked against GTSAM 4.3.0's `GncOptimizer.h`, and they match. The oracle run is deferred to
  part 2 [UNVERIFIED].
- **`SolvePoseGraph` anchors every connected component** of the active graph (loops at w ≤ 1e-4 are inactive). The
  common single-component path is unchanged.

**Evidence** (`tests/unit/test_gnc.cpp`, 10/10; existing PGO tests 10/10 numerical and 9/9 opencv, with the GTSAM
oracle at 100/100):
- **Weights:** the Prop. 4 weight equals the dual's argmin.
- **No outliers:** energy equals plain LM.
- **Gate met:** grid and sphere at 10–40 % random outliers, precision 1.000 and recall worst 0.967. The ramp gives
  10/10 up to 90 %.
- **Near misses:** at 0.5 m / 5°, PCM → GNC accepts none (PCM removed them all), and GNC alone accepts 0/200. At
  0.2 m / 1°, 143/200 accepted, +11 % ATE.
- **Self-consistent 8-loop cluster:** 0/8 accepted.
- **Split graphs:** two agents linked by 3 random loops → PCM no_consensus → 2 components, B held at its anchor.
- **Bit-identical twice:** yes.
- **Timing:** 1.6 s at 1,000 nodes and 4.75 s at 3,000 per event.

**Not done, stated.**
- **A lone inter-agent link is unverifiable** by GNC: measured, it is kept. PCM tags it "untested", so the first
  inter-agent loop rests on the degeneracy gate alone.
- **Hard outliers:** the 10–90 % ramp uses random outliers, so no breakpoint was found. A harder protocol (aliasing
  clusters, near misses at scale) belongs to part 2's real candidates.
- **The demo-latency estimate given earlier (0.3–0.8 s at 1,000 nodes) was optimistic.** Measured: 1.6 s.
  Warm-starting the weights across events is the unimplemented lever.
- **Not wired;** parts 2 and 3 follow.

## 2026-10-20 — D73 part 2a: GNC weight reuse across loop events (fast path = fixed point + half-bound innovation test)

**Context.** Weight reuse was requested before the wiring, to cut per-event latency. Design §6 part 2a
holds the protocol, fixed before the first run.

**Decision.**
- **State.** `SolveGnc(..., GncState*)`: the caller carries per-edge decisions. A loop that was PCM-active last event
  and still is starts at its previous decision; other active loops start at 1; the mask always wins.
- **Fast path:** accepted only if all of these hold:
  - every loop held at 1 has r² ≤ c̄²;
  - every active loop held at 0 has r² > c̄²;
  - the n new loops raise the full cost by 2ΔE ≤ ½χ²₆ₙ(0.99). This takes one extra warm solve with them at 0.
- **Fallback.** Otherwise the fast-path solve is discarded and cold GNC runs from the event's input poses.
- **Path dependent** (deterministic under replay).

**Why the half bound (measured, design §6 table).**
- **The fixed point alone diverged from cold at 25 events:** a wrong loop bent the odometry.
- **Adding the innovation test at χ²₆ₙ(0.99) diverged at 45 events** and kept a near miss for good: precision
  0.986, ATE +3.6 %.
- **Half the bound (Remark 5's region)** matched cold at every one of 84 events, with ATE identical.

**Evidence** (`tests/unit/test_gnc.cpp`, GncWarm 3/3):
- **1,000 nodes:** 72 % fast path, −22 % time, and PCM's rejected-then-accepted case covered.
- **3,000 nodes:** 83 % fast path, 227 ms mean per event, 655 ms worst.
- **Determinism:** bit-identical over a fixed sequence.

**Correction, stated.** Part 1's 1.6 s / 4.75 s per event was GNC with outliers present and no PCM in front. In
the wired order PCM removes them first, and a loop event costs about 0.1–0.2 s even cold.

## 2026-10-20 — D59 (gravity priors): GRAVITY_PRIOR edges produced; Mahony is the source, σ 2.3°

Design and evidence: `docs/design/inertial.md` §4.10–4.11.

- **Edge** (`slam/pose_graph/gravity_prior.hpp`): unary on a keyframe's T_W_C, r = B(g_C)ᵀ R_WCᵀ ĝ_W (2-dof, blind to
  yaw), the body-side form of GTSAM's `AttitudeFactor<Pose3>` (same cost for isotropic information). FD-tested
  Jacobian; kept in `PoseGraph::gravity_priors()`, apart from `edges()` and `Cost()`, which are 2.5's.
- **Emission:** one edge per 10 keyframes (non-overlapping windows), both inertial modes; none without an IMU.
- **Source decided by 2-dof NEES vs ground truth, never ATE** (22 EuRoC runs): the VIO-derived candidates are accurate
  (0.4–2.4°) but inherit the reference gravity's constant per-flight error, overconfident 2–70×. **Mahony** is
  independent and consistent: σ = 2.3° calibrated on Machine Hall (NEES 2.00), held-out Vicon flights NEES 3.0;
  heavy tail (p99 ≈ 18).
- `euroc_v2` and `euroc_v2_attitude` emit them; trajectories unchanged (byte-identical), `graph_version` changes.
- **Handoff to 2.5 (consumer):** (1) add the edges' cost to the PGO from `gravity_priors()` (residual and Jacobian in
  `LinearizeGravityPrior`); (2) robustify them (heavy tail); (3) estimate the agent's gravity direction as a 2-DoF
  variable initialised at `g_ref_W` (its error is shared by every edge); (4) wire field: Phase 3 (inter-agent).


## 2026-10-21 — D73 part 2b: intra-agent loop closure wired (off by default); calibrated loop noise model

**Context.** The loop backend (gate → PCM → GNC on a VO mirror) existed but was not wired. The first EuRoC run
showed the PnP loop covariance far too confident for PCM and GNC. Design: `robust-backend.md` §8.

**Decision.**
- **The adapter** (`runtime/agent/intra_loop_closure.hpp`), wired into `stereo_vo_runner.hpp` behind an optional
  `loop_closure:` section (`configs/slam/euroc_v2_loops.yaml`). Per keyframe:
  - ORB features plus SGBM 3D points, and the mean-bit placeholder descriptor;
  - retrieval, with the last 50 keyframes and the last 10 s excluded;
  - verification with `edge_verifier`, whose new inlier export was an approved hunk;
  - each loop queued until its keyframes leave the BA window, then one backend event.
  The VO is never written. Outputs: `loop_trace.jsonl`, `loop_candidates.csv` (ground-truth errors, full error
  vector and covariance), and a summary block with the funnel, recall, latency and the PCM good-pair statistic.
- **Edge noise model** (`DegeneracyParams::noise_*`): var = k σ²_PnP + floor², per block.
  - Fitted jointly on MH_01 + V1_01: k_t 1.33, floor_t 0.014 m, k_r 0.178, floor_r 0.0095 rad.
  - The gate's criteria still read the PnP σ.
  - The labels' "3σ of its own covariance" prong was dropped; labels are 0.2 m / 2° against interpolated ground truth.

**Why.** On V1_01 the PnP covariance gave median NEES ≈ 1e4, about 1000× overconfident. Correlated errors don't
average down; COVINS-G reports the same. With that covariance, PCM did not improve on the gate (0.67 against 0.63).
The MH_01-only fit failed held-out V1_01, which is stated; hence the joint fit.

**Evidence** (keyframe-refined ATE, VO → loops):
- MH_01 0.0676 → 0.0207 m (−69 %)
- V1_01 0.0563 → 0.0382 m (−32 %)
- V1_02, held out: 0.0517 → 0.0254 m (−51 %)
- **V1_02 funnel precision:** 0.53 → 0.80 → 0.88 → 0.97 (GNC: 142 good, 4 bad).
- **Held-out NEES:** mean 5.24 passes; the tail is 4.4 % against ≤ 3 %, which misses. Decision: accept the
  model with the miss stated; GNC absorbs the tail.
- **PCM good-pair statistic:** 2.1–7.8, so no odometry scale is needed.
- **MH_01 without the model:** GNC kept 51 good of 55 (0.93); with it, 118 good of 177 (0.67), at the same ATE
  (0.0205 / 0.0207 m). On V1_01 the model raised recall (21 → 86 good) and precision (0.81 → 0.91).
- **Evidence:** `evaluation/runs/robust-backend/loop_closure/` (five runs, no ground truth, `analyze_loops.py`). The manifests'
  `git_dirty: false` is wrong: the binaries were built from the uncommitted tree on 4b45934.
- **Latency:** 1.3–2.4 s mean, 7.3 s worst, mostly waiting for the BA window.
- **Tests:** `Degeneracy` 9/9, including the noise-model test.

**Not done, stated.**
- **Sim calibration**, which the demo needs.
- **ORB per-octave σ.**
- **MH_01's 59 accepted loops labelled bad:** small translation error, a ~3.8° systematic rotation offset, none
  gross. Cause [UNVERIFIED].
- **The gate's σ thresholds** are unchanged (the Youden fit barely moved them).
- **Retrieval recall** 51–66 % (placeholder descriptor).

## 2026-10-21 — D73 part 2c: GRAVITY_PRIOR edges consumed by the pose graph (per-agent 2-DoF gravity, Huber)

**Context.** The D59 handoff to 2.5: (1) add the gravity priors' cost to the PGO; (2) robustify them (heavy tail);
(3) estimate each agent's gravity direction as a 2-DoF variable starting at g_ref_W; (4) wiring, which is Phase 3.
Design: `robust-backend.md` §8.

**Decision.**
- **`PgoProblem` / `PgoSettings::gravity`** (off by default):
  - one 2-DoF gravity variable per agent with priors, on the sphere;
  - priors cost ½ρ(s) with a Huber kernel at k² = χ²₂(0.99) and information scaled by 0.6;
  - they are not in GNC.
- **`SolvePoseGraph`** reports `gravity_W` per agent and carries the priors through the per-component split.
- **The loop mirror** syncs the VO's priors (queued, never dropped) and re-solves when they land.
- **Config:** `loop_closure.gravity_priors` (`euroc_v2_loops` false, `euroc_v2_loops_gravity` true).

**Why 0.6:** the lag-1 autocorrelation of Mahony's prior errors is 0.12–0.36 on three EuRoC flights, measured
offline from the run traces, so the effective number of independent priors is lower than the count.

**Evidence** (`tests/unit/test_gravity_pgo.cpp`, 4/4):
- the gradient matches finite differences, Huber included;
- the solve is bit-identical when off;
- a 1.5° reference error is estimated to 0.62°, and tilt RMS goes 3.78° → 2.01°;
  **corrected 2026-10-24 (D123–D124):** that is one seed under x86's argument-evaluation order (arm64 gets 2.71°).
  The fixture's own σ for ĝ is ≈ 2.0° per axis, and only 14–17 % of seeds reach < 1°. Recovery to 0.62° is not
  shown. The redesigned check is the 50-seed NEES consistency test;
- a 30° outlier prior moves its keyframe by 1.85°.
`PgoLevMarq` 4/4 and numerical `PgoProblem`/`PoseGraph` 10/10 pass; the oracle was not rerun (the off path is
unchanged).

**Not done, stated.**
- No EuRoC run yet, by priority (sim calibration and part 3 first).
- The 4-DoF-pose-graph comparison is recorded, not built [UNVERIFIED citations].

## 2026-10-21 — D65: gain-up degradation configs for the VST; the 16 shipped configs stay

The 2.2 preflight measured what D produces (`docs/design/radiometric.md` §2). At unity gain the 8-bit output
posterises dark2 (0–15 DN, σ 0.2–0.7 DN, mostly rounding) and dark3 (0–4 DN). The shipped dark configs therefore carry no
measurable sensor noise, and a measured-noise gate correctly refuses the VST on all of them.

- **Added:** `dark2_dust0_ag8` and `dark3_dust0_ag16`: the same sensor and scene radiance, with the analog gain raised to
  8 and 16. That's what a real camera does in the dark. Their noise follows `var = a·m + b` to within 0.5%.
- **Not chosen:**
  - re-tuning the 16 shipped configs (they feed every Phase-1 baseline);
  - a >8-bit D output (the frontend takes 8-bit only; a wide change).

## 2026-10-21 — D66: D applied in SimSource, opt-in; per-camera seeds; byte-identical parallel D

- **Opt-in.** `SimSource::SetDegradation(named config, run seed, threads)` and `ariadne_sim_vo --degradation YAML`.
  Without them nothing changes, and the manifest still says `degradation_config: none`.
- **Order:** lens → D on the RGB frame → BT.601 gray, the order a real camera has.
- **Seeds:** each (agent, camera) gets `DeriveSeed(DeriveSeed(run_seed, "degradation"), "agentK/left|right")`, so left
  and right do not share noise or PRNU fields.
- **D's frame id** is the per-camera arrival index, which equals the pair index (a missing frame aborts the run as
  before).
- **Dust is refused:** it needs GT depth for both cameras (Phase 5).
- **`DegradeParallel`:** row bands over `std::thread`. It is byte-identical to `Degrade` by the determinism contract,
  and that is tested, not assumed. The degradation digest is unchanged (8535765251477677907).
- **Flat field:** each camera's K is calibrated at `SetDegradation` from 16 D frames of a flat target under its own
  sensor and seed. The frame ids are `0xC0FFEE…`, disjoint from run frames. The calibration agrees with D's analytic
  vignette × PRNU to ≤ 0.5% RMS.

## 2026-10-21 — D67: the radiometric front end: flat field + gated GAT/exact-inverse VST, before rectification

Arch §6.2 stages 1–2 (§6.4 gate). Raw 8-bit gray in and out, between the source and the rectifier: the rectifier's
interpolation would mix the noise the GAT models, and K lives in raw coordinates.

- **Config.** An optional `radiometric:` section in the perception config.
  - Absent means `mode: off`: a call-through decorator, so no pixel and no gate-compared byte changes.
  - Modes: `flat_only | auto | on`. Denoisers: `nlm | gaussian | none`. Inverses: `exact | closed_form | algebraic`.
  - Shipped ablation configs: `sim_v1_rad_{flat,auto,gauss,alg}`, `euroc_rad_{auto,on}`.
- **Order: flat field, then VST.** Correct for D, which scales the noise by K. algorithms.md §14.5's opposite order
  (VST, then ÷K) is the right one for real optics. **D's K-after-noise order is therefore unphysical:** recorded as an
  open question for Phase 5, not changed now, because it would move the degradation digest.
- **Noise model fitted online** (single-image, simplified Foi 2008, [PROXY]) on the first 8 pairs, per camera.
  **Deviation from arch §6.2**, which calibrates (α, σ) offline per (exposure, gain): EuRoC has no static captures, and a
  per-(t, g) table needs the §6.5 photometric state (Phase 5).
- **Gate,** decided once per run, jointly for both cameras. ON iff every camera has:
  - a valid fit;
  - σ_z ∈ [0.8, 1.25] after the GAT (the RPG-VST check);
  - SNR at the median level < **30**.

  **The threshold was corrected from the draft's 20 on 2026-10-21, before any VO measurement.** 20 was derived from
  single-channel D output. The sim path is RGB → D → gray, which scales the variance by Σw² = 0.447, so SNR is 1.5×
  higher: the same boundary is 30. Test `GrayPathScalesTheModel` pins the scaling.
- **Stage 4 (exposure/gain normalisation) is inert** and says so in the results. D54's `MatchExposure` keeps the
  statistics-driven left/right match. Stage 4 is the metadata-driven temporal normalisation to (t₀, g₀); no source
  carries (t, g) yet. No duplication, and `stereo_lk_matcher.*` is untouched.
- **Manifest:** a new field `radiometric` (`off`, or mode/decision/denoiser/inverse/flat-field summary). The fits and
  the gate's reasons go in the `radiometric:` block of results.yaml, written only when enabled.
- **Denoiser: `GaussianBlur` 5×5 by default** (0.8 ms per 1280×720 image; the demo must be near-realtime,
  decision 2026-10-21).
  - `cv::fastNlMeansDenoising`, arch §6.2's suggestion, is an ablation row only. It takes 8U/16U only, so the
    stabilised image is scaled into 8U, and the stage costs ≈ 360 ms per image with it.
  - BM3D is unavailable: our build has `OPENCV_ENABLE_NONFREE` off.

## 2026-10-21 — D68: the exact unbiased inverse is ours, implemented from the paper and checked against its figures

The invansc package (Mäkitalo & Foi) is under a non-commercial licence. None of its code or `.mat` tables is used or
shipped; the inverse is implemented from the thesis' equations.

- **The table.** E{f_s(z) | y, s} (eq. 4.10) by quadrature: the Poisson sum truncated at 12 SD; the Gaussian part
  integrated in u, with z = t0 + u², which removes the square-root kink. It is tabulated on a quadratic y-grid over
  [0, 200]. Below the table: the ML zero (eq. 4.14). Above it: the closed form.
- **Evidence** (`tests/numerical/test_gat.cpp`): our tables reproduce the paper's accuracy figures.

  | quantity | ours | paper |
  |---|---|---|
  | closed form, L∞ | 0.0470 | 0.0468 |
  | closed form, weighted L2² | 0.00690 | 0.0069 |
  | I0 − s², L∞ | 0.0470 | 0.0470 |
  | I0 − s², weighted L2² | 0.00295 | 0.0028 |

  The exact inverse passes an independent exact-Poisson Monte Carlo unbiasedness test that the algebraic inverse fails.
- **Correction to algorithms.md §14.5** (recorded there by an insert): the closed form's `D < 2√(3/8) → 0` branch comes
  from the thesis (§3.2.2), not the `.m` file, which omits it.

## 2026-10-21 — D69: 2.2 verdict: the gate works as designed; on the measured segment the VST buys nothing measurable

**What was measured.** Rule: only what cannot be done without, and the demo must be near-realtime.
`ariadne_sim_vo`, `ci_tiny_0` agent 0, 1000 frames, `sim_v2`, seed 1. Full numbers: `radiometric.md` §6.

| row | ATE | lost | keyframes | landmarks alive | stereo valid |
|---|---|---|---|---|---|
| clean recording | 3 mm | 0 | 80 | 770 | 0.776 |
| `dark3_dust0_ag16`, stage off | 5 mm | 0 | 76 | 238 | 0.747 |
| `dark3_dust0_ag16`, flat field + VST, Gaussian (gate on) | 4 mm | 0 | **104** | **197** | **0.722** |

- **The gate decides from measured noise, as designed.**
  - Sim: ON at both gain-up configs (σ_z 1.00, SNR 7–16); OFF at dark0 (SNR) and at posterised dark2 (no valid fit).
  - EuRoC: OFF on all 11 flights (SNR 34–80).
  - The draft's threshold of 20 would have given the same decisions on these scenes. The correction to 30 (D67) was
    physics, not outcome.
- **No measurable benefit on this segment** (negative result, reported).
  - The noise costs little here: 0 lost frames either way, ATE 5 mm vs 3 mm clean.
  - The 1 mm ATE difference is one seed on an easy 50 s segment, and is **not** claimed.
  - Secondary metrics are slightly **worse** with the stage: keyframes +37%, landmarks alive −17%, stereo-valid
    fraction −2.5 points. The smoothing plausibly shortens LK tracks [UNVERIFIED: not isolated].
  - What would show a benefit, if any: a segment where `stage off` loses frames, i.e. deeper darkness or harder motion.
    Not run.
- **Cost.** The stage takes 14 ms per 1280×720 image with the Gaussian denoiser; most of it is the single-threaded
  inverse-LUT loop, which can be parallelised. The default denoiser is Gaussian; NLM (≈ 360 ms per image) is an
  ablation row only.
- **Defaults stay off.** The shipped perception configs carry no `radiometric:` section. 2.2 lands as an opt-in stage
  with measured gating. Making it default-on is not justified by this evidence.
- **Not run (defined in `configs/benchmark/radiometric_2.2.yaml`):** the rest of the sim matrix (dark2_ag8,
  flat-only, NLM, algebraic) and the EuRoC forced-on rows.

## 2026-10-21 — D80: Phase 3 opened before the Phase 2 gate (explicit decision); decision IDs D80–D89 claimed

- **Opened early, explicitly.** arch §0.2 opens a phase only on the previous gate. Phase 2's gate is not passed:
  2.1 and 2.2 are unmerged, 2.5's D74 certificate is open, and the phase-gate premerge has not run.
  Phase 3 was started on 2026-10-21, given the deadline.
- **What the early opening must not do.** Phase 3 must not change the Phase 2 code paths the gate will measure.
  The Phase-1 gate's pinned references (incl. `figures/bytes.json`) keep describing the Phase-1 exchange until a
  recorded re-pin. The new protocol lives beside it (`docs/design/collaboration.md` §9).
- **Ownership.** The critical path runs in order: 3.2, then 3.4, then 3.5. 3.1 and 3.6 (∥A) and 3.3
  (∥B) run in parallel; 3.3 implements the `LinkModel` seam published in collaboration.md §2.
- **IDs.** D80–D89 are Phase 3's critical path. In use elsewhere: D60s (2.1), D70–D74 (2.5).

## 2026-10-21 — D81: the 3.2 protocol design — per-class delivery, fragment ARQ, one-hop election with epochs

Design: `docs/design/collaboration.md`; design-reviewed twice. Preflight: Swarm-SLAM (code-verified), Tian et al. 2018
(vertex cover), CERBERUS (completed-transmission rule), Lajoie 2026 (real link drop-out), COSMO-Bench. No DTN+C-SLAM
paper found. OpenCV has nothing native here.

- **Delivery semantics differ by class.** Phase 1's strict per-stream order would stall forever on a TTL-expired or
  cancelled message.
  - Heartbeat/ack: latest-wins.
  - Descriptors, requests, feature responses, submaps: idempotent sets, best effort, re-requestable.
  - Edges and graphs: reliable and in order.
- **Fragment-level reassembly with selective retransmit.** Whole-message delivery cannot converge under loss
  (0.7^15 ≈ 0.5%).
  - Reliable sequence numbers are assigned at first transmission, so superseding/expiry/cancel never opens a hole.
  - Unacked messages to an old broker are abandoned and counted on election change.
- **Deviation from §15.3, superseding:** a newer queued pose-graph snapshot replaces an unsent older one from the same
  agent. "Never expire" still holds in substance, because the snapshot contains the older one. Edges are never
  superseded.
- **Deviation from §15.4 (§49 note), the election rule:** the broker is the lexicographic min of (anchor_id, agent_id)
  over the agent and its symmetric one-hop neighbours.
  - This is Swarm-SLAM's exact `is_optimizer()` rule, not "lowest AgentId of the component".
  - There is no relaying, so a broker serves only agents it reaches directly.
  - Neighbour links use hysteresis (in after 2 of 3 periods, out after 4 misses).
- **Epochs and staleness:**
  - An epoch is `(counter, members_hash, broker)`, with counter = 1 + max over members' heartbeats. That makes a
    merge strictly newer than both sides.
  - An agent accepts an estimate only if it is a member, the estimate comes from the broker it elected, and the
    counter is newer.
  - The corruption check keys on the full epoch, with digests over the wire-encoded (float32) estimate.
- **Gauge:** the anchor's first pose is fixed in full. A side that lacks its anchor fixes its lowest member's
  `T_G_Wi`. Yaw-only waits behind a switch for 2.5's gravity-prior consumer.
- **The I6 byte gate is fixed now, before any data:**
  - predicted = the §16 cost model before sending, with a compression-ratio model calibrated on disjoint runs;
  - measured = first-transmission wire bytes, per class, classes 1–6, >15% fails;
  - retransmit, ack and heartbeat overhead get their own rows and their own curve.
- **Store-and-forward = hold until the destination is reachable.** Multi-hop relay is out of scope.

## 2026-10-21 — D82: 3.2 protocol, the fixes the fault campaign forced; I5 campaign 54/54 consistent

Design: `docs/design/collaboration.md` §10. Code: `cpp/collaboration/protocol/` (`network`, `transport`, `membership`,
`collab_node`, `outcome`); tests `tests/protocol/test_{network,transport,membership,collab_v2}.cpp`.

- **Descriptors are acked and resent until their TTL.** A broker cannot re-request a descriptor it never heard of.
  An expired descriptor leaves the sent set, so the next broker is sent it again.
- **Acks are repeated 3×, and the retransmit timeout is capped at 8 s.** Measured: 1–4 unacked edge messages per
  30%-loss run before the change; none after.
- **Reliable stream incarnations** (`seq = incarnation << 40 | n`). Abandonment on a broker change would otherwise leave
  a permanent sequence hole (measured: 435 edges held in every partition run).
- **Heartbeat hysteresis is sized from the loss rate:** 0.5 s period, out after 12 missed periods. `k = 4` at 1 Hz
  flapped the broker 40 times in 290 s at 30% loss.
- **The broker re-solves on a new edge or a member-set change.**
- **I5 evidence, unit-level (synthetic 4-agent scenario, ground-truth hooks):** 6 faults × loss {0, 10, 30%} × 3 seeds
  = 54 runs, **54 consistent, 0 incomplete, 0 corrupt**. Split-brain {0,1}|{2,3} merges under a new epoch, and the
  frames match ground truth within 2 mm. An unknown-frame pose is rejected.
- **Not yet:**
  - the production broker solve (multi-agent PGO, then 2.5's robust stage);
  - runtime/sim integration with real XFeat features;
  - 3.3's link model.
  - **Snapshots carry nodes only.** They lack odometry edges and their information. The oracle solve reads node poses
    as exact, so nothing failed, but a multi-agent PGO needs odometry factors, and the I6 POSE_GRAPH sizes need the
    §15.2 payload (nodes, edges, covariances).
    - The next step changes the API first: `AddKeyframe` takes the odometry edge with its information, and the snapshot
      carries the intra-agent edges.
  - The hooks are oracles: this evidence covers the protocol, not estimation accuracy.
- **The classifiers are proven to fire.**
  - A rogue estimate (an unissued epoch, a perturbed T_G_W) that a member adopts is classified `corrupt`.
  - A run that ends while partitioned is classified `explicitly_incomplete`, and its reasons name the missing pieces.
  - Same seed ⇒ identical ledgers and outcome reasons at the node level.

## 2026-10-21 — D83: the production broker solve — one multi-agent PGO; snapshots carry odometry edges

Code: `slam/pose_graph/multi_agent.{hpp,cpp}` (slam_opencv), adapter `collaboration/broker/broker_solve.hpp`
(`NodeHooks::solve` for the runtime); tests `tests/unit/test_multi_agent.cpp`.

- **What it solves.** One joint pose graph over the members' subgraphs, with node ids `(agent << 40) | keyframe`:
  their nodes, their odometry edges with information, and the verified inter-agent edges.
- **Placement and initialisation.** Agents with no edge path to the gauge are reported unplaced. The rest start from a
  spanning-tree alignment, then the chordal initialiser runs (D70), then LM on `LevMarqBase` (D51).
- **Gauge.** The gauge agent's first node is fixed at `T_G_W_gauge · T_W_C` (§15.4). The solve is skipped when the
  joint graph has no edge: a lone broker with one keyframe has zero free variables, and that crashed LevMarq
  (a segfault, found by the protocol-level test).
- **Per-agent T_G_W.** The gauge agent reports the gauge value (G = W_anchor). Every other agent reports a frame fit
  of its W-frame poses onto their optimised G poses: the chordal rotation mean, then the mean translation residual.
  - A positions-only Kabsch fit was rejected: it is ambiguous on a straight trajectory.
  - Per-keyframe corrections are returned as `T_G_C` but are **not yet on the wire**. The estimate carries only the
    rigid T_G_W; keyframe-local consumers (Phase 6) will need the node poses.
- **Snapshots now carry the agent's odometry edges.** `EdgeStatus::ODOMETRY` was appended to the schema, the
  information is packed as the 21-float upper triangle, and `AddKeyframe` takes the odometry edge. This is the §15.2
  payload the I6 byte gate prices.
- **Measured (synthetic, 3 agents, 60 keyframes, odometry noise 1 cm / 0.4° per step):**
  - node ATE: 0.228 m dead reckoning → 0.019 m after the solve;
  - each T_G_W lies within 5 cm / 0.6° of the best rigid fit of that agent's drifted trajectory to ground truth.
- **Protocol with the production solve:** a 4-agent run with a partition {0,1}|{2,3} at 10–30 s and 10% loss ends
  `consistent`, with every T_G_W within 5 mm / 5 mrad of ground truth. The 54-run oracle campaign is still 54/54
  consistent with odometry edges in the snapshots.
- **Not yet:**
  - the robust stage (degeneracy gate → PCM → GNC-TLS, 2.5's `cpp/slam/robust/`) between the edge list and the solve;
    inter-agent edges currently enter as verified;
  - the gravity-prior yaw-only gauge.
  - Both wait on 2.5 merging.

## 2026-10-21 — D84: AWS unblocked: Bedrock works (the 2026-09-29 verification block has cleared); GPU spot quota approved

Re-tested live as `user/pixel` in eu-north-1. Superseded: the D26–D28 "Bedrock: Operation not allowed / account being
verified" open item, and STATUS's "Pending with AWS: GPU quota … Bedrock is blocked".

- **Bedrock runtime works.** `converse` on `eu.amazon.nova-micro-v1:0` answered (9 tokens). No support case is needed.
- **Orchestrator model (requirement: any cheap model that is good enough).** Primary: **Amazon Nova 2 Lite** via the
  EU profile `eu.amazon.nova-2-lite-v1:0`. Fallback: **DeepSeek V3.2** (`deepseek.v3.2`, in-region on-demand).
  - Quotas (cross-region, EU): Nova 2 Lite 200 RPM / 8M TPM. In-region on-demand: DeepSeek V3.2 and Qwen3-32B at
    1000 RPM / 100M TPM.
  - Nova is first-party, needs no use-case form or Marketplace subscription, and keeps data in the EU profile.
  - Tool-use smoke test: both made a correct structured `toolUse` call on a frontier-assignment prompt. They chose the
    adjacent safe frontier over a high-gain one behind a 0.95-risk collapse, with a stated reason.
  - `[UNVERIFIED]` per-token prices: pull them before Phase 7 budgets.
- **Claude on Bedrock is not used.** The Anthropic use-case form is not filed, and the 5.5-family models have 0 TPM on
  this account. Haiku 4.5 has quota (50 RPM / 5M TPM) if it is ever wanted.
- **GPU: EC2 Spot "All G and VT" in eu-north-1 approved at 4 vCPU (AWS, 2026-10-19).** One 4-vCPU GPU instance; spot,
  so reclaimable: checkpoint gsplat and keep a demo fallback. On-demand G/VT (case 179205877900026) is still pending.

## 2026-10-19 — D60: 2.1 — ORB + lost-mode re-association ships for EuRoC v2; XFeat stays a measured ablation (overrides "XFeat primary" for this phase)

> **Approved as scoped, 2026-10-19.** It overrides arch §0.2's "XFeat primary features" for the
> EuRoC v2 VO front-end only. "XFeat primary" stands for the learned paths. arch §0.2 itself is not amended.

Design, every run and every bar: `docs/design/perception.md` (§3 rev 3, §9).

**What ships.** `configs/perception/euroc_v2.yaml` becomes the EuRoC v2 perception config, paired with
`configs/slam/euroc_v2.yaml`. It is `euroc.yaml` plus a `reassociation:` section. **No file in the repo defines the
v2 pair** (D58's v2 sweep ran `perception/euroc.yaml` + `slam/euroc_v2.yaml`). So from D60 on, every EuRoC v2
sweep, the critical path's included, must pass `--perception-config configs/perception/euroc_v2.yaml`, or
it silently runs without re-association.
- Lost-mode re-association, the ORB-SLAM §V-C relocalisation idea restricted to the BA window. When fewer than 30
  live tracks hold a landmark, tracks holding none are described at their pixel and matched against healthy (born at
  MEASURED keyframes) untracked landmarks: mutual NN, Lowe 0.75, Hamming ≤ 50 (ORB-SLAM2 `ORBmatcher`, `TH_LOW`).
- The matches are verified by seeded USAC PnP together with the healthy correspondences.
- Accepted tracks are renamed (`KltTracker::Relabel`), so StereoVo finds the old landmark by id.
- Off when the section is absent.
  - Verified byte-identical on EuRoC v2: section absent, D58 seed 1 equals D58's own run on all 11 flights on the
    merged binary; on the D56 tree, ORB `euroc_v2` re-runs equal A1, and with the code present but the section
    absent, runs equal D56.
  - Identical **by construction, not run** for v1 (`slam/euroc.yaml`) and the sim tools that include the runner:
    every new call returns early when its config section is absent.

**Evidence (merged tree with D58's gyro; §3 rev 3: 3-seed medians on the failing four, seed 1 on the passing seven
by the time-gated amendment):**
- Failing four, against min(D54-equivalent, D58):
  - lost frames pass on all four (medians 264 → 67, 101 → 99, 141 → 92, 171 → 122);
  - ATE passes on MH_04, V1_03 and V2_03 (V2_03 0.595 → 0.535 on every seed).
- Passing seven: six hold at seed 1. V2_02 missed at seed 1 (0.126 against 0.118); a post-hoc 3-seed check gives a
  median of 0.1169 against D58's 0.1165, with lost 16 → 8.
- 11-flight seed 1: average ATE **0.223 → 0.204 m**, lost frames 743 → 631 (MH_04's seed 1 is its bimodal bad draw:
  266 against 66/67).
- **Against the brief's literal D54 table** (seed 1, §3.4):
  - A1 beats it on ATE and lost frames on all four failing flights: MH_04 2.777/278 → 0.365/266, MH_05
    0.560/107 → 0.314/101, V1_03 0.439/146 → 0.391/89, V2_03 2.118/183 → 0.569/130.
  - It equals or beats it on six passing flights.
  - V2_02's ATE is 1.9% worse (0.1238 → 0.1262), with lost frames 17 → 8.

**Accepted with one stated miss: MH_05 ATE** (0.314 against the bar of 0.2274).
- The bar is the D54-equivalent (coasting) median. Coasting crosses MH_05's frame-1242 loss 25× worse than D58's gyro
  (0.5 m / 4.3° against 0.02 m / 0.14°), but on seeds 2 and 3 that error happens to cancel earlier drift. Seed 1
  of the same coasting scores 0.560.
- Each half of the flight scores about 0.08 m in every configuration, and D58 has 20% less RPE than coasting.
- A1 beats D58 on MH_05 on every seed (0.314 against 0.343). Analysis: perception.md §9.15. Nothing is wrong with
  D56/D58's lost handling on MH_05.

**Why not XFeat (arch §0.2 "XFeat primary features", arch §7.2).** It is built and verified, and stays
config-selectable (`features.engine: xfeat`):
- `XFeatEngine` over D50's net: one cached forward per image, mask before §7.3 grid selection, `Describe` at any
  pixel with upstream's sampler, upstream's 752 → 736 preprocessing reproduced to float precision.
- Keyframe cadence (`tracker.replenish: keyframe`, `replenish_min_tracks`) and the §7.2 image-gradient gate
  (`replenish_min_eig_quality`).

Measured, it cuts lost frames hardest but costs accuracy:
- On the D56 tree, XFeat at keyframes (A4) regressed 6 of the 7 passing flights.
- An engine × cadence 2×2 showed both contribute.
- The best fix (A4t-r: resupply below 300 tracks + ratio 0.75 on XFeat's matcher) failed rev 3 on the merged tree:
  V2_01, V2_02, V2_03 and MH_02 ATE.
- With re-association OFF, XFeat alone still regresses V2_03 and MH_02 (§9.13), so the regressor is the engine
  (or its cadence), not the association.
- It also runs at 0.53–0.88 forwards per frame on the V2 flights, near C8's frame-rate limit.

XFeat remains the planned primary for the learned paths it was chosen for. This decision is about the EuRoC v2 VO
front-end in 2.1, revisited if a later phase removes the regression.

**Not done in 2.1 here, stated plainly:**
- **XFeat timing within the §4p budget: unmet (unmeasured).** Forward passes were only timed under load (28–78 ms
  against ≤ 16 ms).
- **The PRIORITY item is half built.** Descriptor re-association with PnP verification exists. Projection-guided
  search into the window (ORB-SLAM's Track Local Map) does not.
- The XFeat input-size decision is not taken; `resize` (upstream) is the default.
- The rest of plan 2.1 is not started: WLS/§9.4/§9.6, ANNIndex, ALIKED+LightGlue.

**Correction to D54 (append-only).** D54's quoted lost frames "MH_04 266, V2_03 181, V1_03 139, MH_05 100" are
v1's, taken from the v1 rows of the same sweep. v2's are 278, 183, 146 and 107 (perception.md §2.1).

**Merge note.** The phase-2 foundations (D57, D58) are merged into the 2.1 perception line (`dbbf8d5`).
`algorithms.md`: D58's inertial section keeps §1.3b; 2.1's re-association catalogue entry is §1.3c.

## 2026-10-19 — D61: §9.3's confidence — the WLS map wired without touching our SGBM; off by default until consumed

Design: `docs/design/perception.md` §10.

- **What:** `stereo.confidence: none | wls` (default `none`, the placeholder). Under `wls`, `SgbmStereo` runs
  `ximgproc::createRightMatcher(sgbm)` (a new SGBM) and `createDisparityWLSFilterGeneric(true)`, with the ROI
  (min+num, 0, W − (min+num) − max(0, −min), H) and discontinuity radius ceil(0.5·blockSize) that the native SGBM
  wiring sets. `getConfidenceMap()` (CV_32F 0–255) → u8, 0 wherever `valid` is 0. The left disparity is
  byte-identical unless `wls_filtered_disparity: true`.
- **Why not `createDisparityWLSFilter(sgbm)`, the documented route:** it sets our matcher's uniqueness to 0,
  disp12MaxDiff to 1e6 and speckle window to 0 (pinned `disparity_filters.cpp`). That would silently change every
  disparity, the SGBM gate and all v1/v2 outputs.
- **Deviation from OpenCV's intent, stated:** the map is computed from OUR left disparity (uniqueness 15,
  disp12MaxDiff 1, speckle 200), which already passed SGBM's 1 px left-right check. On valid pixels WLS's 1.5 px
  check is therefore mostly redundant, and the confidence is dominated by its variance term (depth edges).
- **Found:** OpenCV 5.0.0 SGBM clamps a non-positive `disp12MaxDiff` to 1 (`stereosgbm.cpp`: `> 0 ? v : 1`), so
  "non-positive disables" is false in this build.
- **Evidence:**
  - 9 unit tests: placeholder default; byte-identity of disparity/Z/σ_Z/valid with confidence on; high on smooth
    surfaces, low at a depth step; the left-right check rejects an occlusion band once SGBM's own check is off (1e6);
    filtered disparity opt-in; 1 vs 8 threads byte-identical; config validation; §9.4's width and boundary cases.
  - EuRoC MH_01 and V2_01 with `confidence: wls`: results and trajectories byte-identical to the v2 config.
  - Cost per stereo pass at 752×480: right SGBM 23 ms + filter 14 ms, best of 5 at load 3.8 (pipeline median 40–52 ms
    under load).
- **Not measured: quality.** The AUC against EuRoC's Leica scans is deferred (2026-10-19) and
  recorded as unmeasured. Nothing in 2.1 consumes the map. Turning it on in a shipped config waits for a consumer
  (TSDF §20.1 or §11.2) and that measurement.


## 2026-10-20 — D62 (DRAFT, pending approval): place retrieval — EigenPlaces R50-256 via cv::dnn; exact index default, cv::ANNIndex the ablation

Design: `docs/design/perception.md` §11. Catalogue: `algorithms.md` §1.6 (as-built note).

- **What:**
  - `perception::PlaceRecognizer` runs EigenPlaces R50-256 (Berton et al., ICCV 2023, MIT, `a2969f7`). The export
    comes from `python/model_tools/eigenplaces_reference.py` (opset 17, fixed input size). It runs in `cv::dnn`
    `ENGINE_CLASSIC`, one Net per thread, and outputs a 256-D unit vector (the existing `GLOBAL_DESCRIPTOR`
    contract).
  - `perception::PlaceIndex` provides top-k cosine with an admission filter.
- **Deviation, stated (§49):** the arch says "Use `cv::ANNIndex`". The default here is **`exact`** (one gemm, a few
  µs at thousands of keyframes, deterministic). `annoy` (`cv::ANNIndex`) is kept behind the same interface as the
  persistence/scale ablation. Reason: three findings from the pinned source (ids restart at 0 per `addItems`, no add
  after build, a per-thread seed over `hardware_concurrency()`).
- **Evidence:**
  - SMOKE §4r: `cv::dnn` matches upstream at cosine 1.0000000, both engines, both sizes.
  - Unit tests: `test_place` (exact, filter, annoy agreement 199/200, repeat 200/200, rebuild after add,
    preprocessing ≤ 1e-5, end-to-end cosine ≥ 0.9999) and `test_eigenplaces_reference`.
  - EuRoC, 11 flights, 297 queries: pooled R@1 EP-376 0.626, EP-752 0.677, mean-bit 0.444, random 0.167. Annoy
    top-1 equals exact on 297/297.
- **Acceptance (§11.5, fixed before the run):** bar 1 PASS. Bar 2 MISSED by 15/297 = 0.0505 against 0.05: half a
  query, consistent direction, paired McNemar p = 0.028 (optimistic, the queries are correlated).
- **Size (§11.7): DECIDED 2026-10-20, (a) 376×240 is the default.** The bar-2 miss is accepted
  and stated. 752×480 stays a selectable export. A size in between was not measured.
- **Limits:**
  - EuRoC-scoped: the rig is 1280×720 RGB, and sim needs its own export and an RGB path.
  - No consumer in 2.1 (2.5 owns loop closure).
  - CLAHE not run.
  - Annoy determinism across core counts is unverified.

## 2026-10-21 — D63: the hard-pair path — ALIKED-n16 + LightGlue from our own upstream export, through cv::ALIKED / cv::LightGlueMatcher

**Approved as scoped, 2026-10-21.**

Design: `docs/design/perception.md` §12. Smoke: `validation.md` §4s / §4s-2. Catalogue: `algorithms.md`
(the correction note under the OpenCV-5 inventory).

- **What:** `perception::HardPairMatcher`, with ALIKED-n16 at K = 1024, native 752×480, and LightGlue running all
  9 layers with `filter_threshold` 0.1. Upstream keypoint normalisation goes through `setPairInfo(..., Size(),
  Size())`; one instance per thread. The models are exported by `python/model_tools/aliked_lightglue_reference.py`
  from `cvg/LightGlue@eb42fee` with the weights `Shiaoming/ALIKED@683d7c6` `aliked-n16.pth` and LightGlue
  `v0.1_arxiv` `aliked_lightglue.pth`, configured as glue-factory's official setting.
- **Why our own export** (decision, 2026-10-20): opencv_extra's files are an unlicensed personal
  export, of ALIKED-**n16rot** (LightGlue's ALIKED weights were trained on n16), at a fixed 640×640 square.
- **Deviations from OpenCV's documented use, stated:**
  - pre-normalised keypoints, not OpenCV's per-axis rule;
  - keypoints emitted in cv::ALIKED's own convention, so its (n+1)/2·W mapping returns upstream's (W−1) pixels;
  - export-only rewrites for cv::dnn ENGINE_NEW (DCN as GridSample + MatMul, gathers, explicit reshapes, a Floor
    before a Cast), each checked against unmodified upstream.
- **Evidence:**
  - **Exact against upstream PyTorch through OpenCV's classes:** keypoints 100% within 9.3e-5 px, descriptors
    within 4e-6, identical LightGlue match sets.
  - **Tests:** 2 reference tests, 3 wrapper tests (normalisation, size and parameter checks, upstream match set
    reproduced), 2 geometry convention tests (a synthetic pose recovered within 0.0003°).
  - **EuRoC loop-candidate verification**, 4815 EP-376 top-5 pairs on 11 flights, with the 1.6 verifier's own
    RANSAC essential matrix, against the ORB placeholder:

    | | ORB placeholder (A) | ALIKED + LightGlue (C) |
    |---|---|---|
    | pose AUC@10° | 0.284 | 0.607 (+0.322) |
    | flights where C ≥ A | | 11/11 |
    | AP_geom, fixed denominator (all 676 positives) | 0.303 | 0.807 |
    | AP_geom, per-row denominator (as first scored) | 0.578 | 0.891 |

    LightGlue over mutual NN: AUC@10° +0.042. Both §12.5 bars **PASS**, bar 1 under both AP denominators.
    The per-row one was the scorer as first written; the fixed one is like-for-like (§12.6).
  - **Reported:** OpenCV's per-axis normalisation is a wash, so upstream stays the default. K = 2048 brings no pose
    gain on all positives at ≈ 2.8× the cost, so K = 1024 stays. E's subset AP is biased against C by
    construction and does not decide K.
- **Stated limits:**
  - Zero-false-accept recall as fixed is ≈ 0 for every row: tiny-baseline revisits with correct rotation fail the
    translation term. A rotation-only diagnostic gives C 0.381.
  - **4 confident wrong verifications** (210–264 inliers, rotation error 37–59°) in the Vicon rooms. An
    essential-matrix inlier count alone is not a loop-closure gate; the downstream PnP and degeneracy gates (1.6,
    D71) must stay.
  - The inlier threshold τ does not transfer stably from MH to V.
  - The exports are 752×480 only; sim at 1280×720 needs its own export plus an RGB path.
  - No runtime consumer in 2.1 (2.5 and 3.x call it).
  - Cost ≈ 1.2 s per warm pair and ≈ 3 s per cold pair under load (≈ 0.6 / 1.7 s quiet).
  - The location of cv::dnn's rounding behind the Floor fix is `[UNVERIFIED]`.
- **Ablation baselines kept:** ORB + ratio (1.6 verifier), ALIKED + mutual NN, OpenCV per-axis normalisation,
  K = 2048.
- **§49 ritual for this adoption** (locked by arch §0.2, adopted here):

  | step | status |
  |---|---|
  | catalogue | the algorithms.md correction and as-built note |
  | arch decision | D63, plus the §7.4 blockquote |
  | ablation matrix | perception.md §12.4 rows A–E |
  | benchmark config | **deferred to 2.5**, the first runtime consumer: no `configs/` entry and no `HardPairParams` in `perception_config` yet. The offline tool takes its paths on the command line. |
  | manifest schema | **deferred to 2.5:** §28.2 has no hard-pair field yet. `ariadne_loop_verify`'s manifest records `hard_pair: [<HardPairMatcher::Name()>…]`. |
- **The rotation-only zero-false-accept numbers are a diagnostic chosen after seeing the data, not a bar.**

## 2026-10-22 — 2.1 close-out notes (bookkeeping; no new decision)

- **D62 stays DRAFT.** Its size question was decided (376×240, 2026-10-20), but its deviation from the
  arch's "Use `cv::ANNIndex`" (exact index as the default) has not been explicitly approved. 2.5, the first
  consumer, should take that call.
- **Projection-guided re-association** (the PRIORITY item's second half, ORB-SLAM's track-local-map search into the
  window) is a **stated gap, deferred.** Trigger: revisit if the combined EuRoC run shows lost stretches still
  dominating V1_03/V2_02. Descriptor re-association with PnP verification (D60) is what shipped.
- **Reference exports:** the regeneration commands for `.cache/models/{eigenplaces_ref,aliked_lg_ref}` are in
  perception.md §13.

## 2026-10-21 — D85: the combined EuRoC run (2.4 full VIO + 2.1 re-association), `main` 62441c4: neutral overall; V1_03 still out

Run: `build/d85_tools/combined_euroc.sh`, with `euroc_v2` perception (ORB + lost-mode re-association, D60) and `euroc_v2` slam
(full VIO, D59). 11 flights run sequentially, all `RUN_OK`. Metric: keyframe-refined ATE (D57 protocol), the same
metric as `inertial.md` §4.9.

| flight | MH01 | MH02 | MH03 | MH04 | MH05 | V101 | V102 | V103 | V201 | V202 | V203 | avg |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 2.4 only (D59 part 3) | 0.068 | 0.039 | 0.099 | 0.222 | 0.128 | 0.056 | 0.052 | 0.357 | 0.039 | 0.101 | 0.168 | 0.121 |
| combined | 0.068 | 0.044 | 0.099 | 0.223 | 0.140 | 0.054 | 0.047 | 0.329 | 0.039 | 0.110 | 0.169 | **0.120** |
| lost frames / re-associations accepted | 0/2 | 13/11 | 0/0 | 262/32 | 96/15 | 1/8 | 3/34 | 92/125 | 16/4 | 7/28 | 118/112 | 608 total |

- **Effect: neutral.** The average drops 1 mm. Per flight it is mixed: V1_03 −28 mm, V1_02 −5 mm; MH_05 +12 mm,
  V2_02 +9 mm, MH_02 +5 mm. It is still **9/11 in the stereo-inertial class**, with the same two flights out:
  V1_03 (0.329 vs 0.120) and V2_02 (0.110 vs 0.100). The ±10 mm moves are single runs; V1_03 has flipped on small
  changes before (D58).
- **The Phase 2 EuRoC gate condition stands at 9/11, average 0.120 ≤ 0.138.**
- **A1's trigger for projection-guided re-association, evaluated:**
  - V1_03 still has 92 lost frames and is out of class, so the trigger fires for V1_03.
  - V2_02 has 7 lost frames: its miss is drift, not lost stretches, so the trigger does not fire there.
  - The 2.4-only per-flight lost counts were not recorded in §4.9, so "still dominate" is judged on the combined run
    alone.
  - **Open decision:** whether V1_03 alone justifies building it (register: 2.1 row).

## 2026-10-21 — D62 approved: the exact index is PlaceIndex's default; cv::ANNIndex stays as the scale ablation

**Approved as scoped, 2026-10-21.** This supersedes the "D62 stays DRAFT" line in the 2.1 close-out
notes above. The D62 entry itself is unchanged.

- **What is approved:** `PlaceIndex` defaults to `exact` (one gemm, deterministic, a few µs at thousands of
  keyframes). `annoy` (`cv::ANNIndex`) stays behind the same interface as the persistence/scale ablation. The arch
  line "Use `cv::ANNIndex`" now reads as the ablation, not the default (§6 blockquote).
- **Why still exact:** the three pinned-source findings in D62 (ids restart at 0 per `addItems`, no add after build,
  a per-thread seed over `hardware_concurrency()`), and top-1 agreement 297/297 on EuRoC, so exact loses nothing at
  this scale.
- **§49 ritual:**

  | step | status |
  |---|---|
  | catalogue | `algorithms.md` §1.6 as-built note (D62) |
  | arch decision | D62, plus the §6 and §31.4 blockquotes |
  | ablation matrix | arch §31.4 as-built note: index `exact` (default) vs `annoy`/trees (scale); perception.md §11.5 |
  | benchmark config | `ariadne_place_eval` + `place_recall_eval.py` (perception.md §11.5). No `configs/` entry until the first runtime consumer (2.5 / 3.1) |
  | manifest schema | `ariadne_place_eval`'s manifest records `place_index: [exact, annoy/trees<N>/seed1]`. §28.2's `place_recognition` field exists; a `place_index` field is **deferred to the first runtime consumer**, like D63's hard-pair field |
- **Open:** Annoy determinism across core counts is still unverified. It matters only if the ablation is promoted.

## 2026-10-22 — D90: sim-rig exports — EigenPlaces and ALIKED-n16 + LightGlue at 1280×720 RGB, with an RGB path

**Settled 2026-10-22** by the sim retrieval evaluation (`docs/design/retrieval.md#sim-retrieval`, rule fixed before the
run): EigenPlaces' sim default is **640×360**. Everything else follows D62/D63 as approved. The earlier
approval of the size was replaced by that pre-registered rule, as the Phase 4 brief asked; it is re-openable.

- **What:**
  - Sim-rig exports from the same pinned upstreams and weights as D62/D63, made by `models.sh eigenplaces-sim` /
    `aliked-lg-sim` and checked against sha256s pinned in the script:
    - EigenPlaces R50-256 at 1280×720 and 640×360;
    - ALIKED-n16 K = 1024 at 1280×720;
    - LightGlue (byte-identical to D63's).
  - The reference frames are recorded `left_rgb` from the phase-1 bag (`ariadne_frame_dump`).
  - The wrappers gain an RGB path whose channel order is in the name: `PlaceRecognizer::DescribeRgb`,
    `HardPairMatcher::ExtractRgb`. Both take RGB order, which is what the sim gives (`StereoFrame::left_rgb`).
    `ExtractRgb` converts RGB → BGR because `cv::ALIKED` swaps back.
- **Evidence:** SMOKE §4t.
  - Exact against upstream at the new size, which re-runs D63's 752×480-only validation of the export rewrites.
  - The regeneration is bit-exact.
  - A channel-swap guard shows an R/B swap changes the result instead of passing silently.
- **Default (settled):** EigenPlaces' sim default is **640×360**, the analogue of D62's half-size 376×240. It costs
  a quarter of 1280×720 (L5: 312 vs 1280 ms p50; here 331 vs 1297 ms on the dev host). ALIKED runs at the native
  1280×720: hard pairs only, so cost matters less than localisation.
- **Measured (clean ci_tiny recording, retrieval.md#sim-retrieval §5):** 61 revisit queries after motion thinning, all of them
  agent 0's (33 intra-agent, 28 against agent 1's database). Pooled R@1 is **0.705 at both sizes** (McNemar p = 1.0),
  R@5 0.885 vs 0.869. The rule kept 640×360 unless 1280×720 won with p < 0.05; it did not. With 61 queries from
  one small scene, this is "no evidence that full resolution helps", not "they are equal".
- **Limits:**
  - Four frames from one scene, with weak colour.
  - The size question was settled on one clean scene only; the dark-scene row (§31.5) is carried to 5.1.
  - There is no runtime consumer yet (3.1/3.5).
- **§49:**

  | step | status |
  |---|---|
  | catalogue | unchanged: same models as D62/D63 (`algorithms.md` §1.6 notes) |
  | arch decision | D90, plus the §7.4 blockquote |
  | ablation matrix | arch §31.4 sizes (EigenPlaces 640×360 / 1280×720); ALIKED K as D63 |
  | benchmark config | `models.sh eigenplaces-sim` / `aliked-lg-sim`; no `configs/` entry until a runtime consumer |
  | manifest schema | each export's `manifest.yaml` (`preprocessing_rgb`, `rgb_images`); `Name()` carries the size |

## 2026-10-22 — D91: the DNN execution contract object — `dnn::PerThread<T>` in every cv::dnn wrapper, and the callers own the parallelism

- **What:**
  - `perception/dnn/per_thread.hpp`: `PerThread<T>::Local()` returns the calling thread's instance, built by a
    factory on that thread's first call.
  - XFeatNet (graphs per size), PlaceRecognizer (Net) and HardPairMatcher (the ALIKED + LightGlue pair) hold their
    networks in it. One wrapper object can be shared by a worker pool, as §7.4 requires: "the feature worker pool
    owns N nets loaded from the same file".
  - `ModelRecord {model, file, engine}` (`Record()` / `Records()`) is what the manifest records about which engine
    ran.
- **Second rule, found by the test:** per-thread instances are not enough for byte identity.
  - Every Net shares OpenCV's single process-wide `parallel_for_` pool. Two concurrent ENGINE_NEW ALIKED forwards at
    the default pool (8 threads) **intermittently** differed: 462/1024 keypoints by up to 6.1e-5 px, descriptors by
    up to 1.3e-6, LightGlue scores by up to 4.4e-5, match indices identical.
  - Sequential runs on another thread: identical.
  - At `cv::setNumThreads(1)`: identical in every repeat (5 × 4 tests).
  - So when a wrapper serves several threads, **the parallelism is the callers'**: the runtime sets
    `cv::setNumThreads(dnn::kPoolOpenCvThreads = 1)` once and records it. It is a process-wide setting, so it is the
    runtime's call and the library does not set it. A single caller may keep the pool.
- **Evidence:** `test_dnn_contract`. Two threads are released together on one shared wrapper, and both match a
  single-thread reference byte for byte, for XFeat (1.1 MB of output), PlaceRecognizer and HardPairMatcher.
  `PerThread` has its own unit test. The 2.1 wrapper and reference suites still pass (31/31).
- **Not done:** no runtime pool uses it yet; Phase 7 multithreading and 7.3 COOL are the consumers. The pool rule's
  cost is unmeasured: N single-threaded nets against one N-threaded net is an L5/7.3 question.
- **§49:** the arch decision is the §7.4 blockquote. A manifest field (the pool size, `ModelRecord`s) is **deferred to
  the first runtime consumer**, like D63's.

## 2026-10-22 — D93: global-descriptor compression (3.6) — int8 with a per-vector scale is free on EuRoC; PCA is not worth its shared basis

Design and protocol fixed before the run: `docs/design/retrieval.md` §2. Result: §4.

- **What:** the §17.2 rows (fp32, fp16, int8, PCA→128 fp16, PCA→256 int8, plus PCA→64 fp16 and PCA→128 int8) are
  scored offline on D62's EuRoC dumps by `python/analysis/retrieval_rows_eval.py compression`. The protocol is
  §11.5's; PCA is fitted on the other environment.
- **Result:**
  - fp32 reproduces D62 exactly.
  - fp16 and int8 (258 B) give identical hits on all 297 queries, at both sizes.
  - PCA truncation is admissible at 376×240 but not at 752×480. PCA→256's gain is mean-centring.
  - All differences are n.s. (McNemar p ≥ 0.07).
- **Recommendation for the wire (3.2's call):** int8 with a per-vector fp16 scale, 258 B per keyframe
  against 1024 B. `wire.fbs` `GlobalDescriptor.descriptor` is `[float]`, so this needs a schema change; 3.6 changed
  no comms file. The bytes here are payload `[PROXY]`.
- **Limits:**
  - EuRoC only, with 297 correlated queries.
  - Sim retrieval rows come with 3.1's sim evaluation.
  - **Stated gap:** §31.9's XFeat local-descriptor fp16/int8 row is not run. D60 ships ORB (32 B binary) on the wire,
    so it lands with the first XFeat-on-the-wire configuration (register).
- **§49:**

  | step | status |
  |---|---|
  | catalogue | no new algorithm |
  | arch decision | D93, plus the §17.2 blockquote |
  | ablation matrix | §31.9 global rows: done (§4); XFeat row: stated gap |
  | benchmark config | `retrieval_rows_eval.py compression` with its fixed defaults (bar 0.01) |
  | manifest | the run directory's `manifest.yaml` (input sha256, image, command) |

## 2026-10-22 — D92: retrieval-path CLAHE (3.1, arch §6.3) — built behind `enhance`, default off; no effect on EuRoC; the deciding row is sim

Design fixed before the run: `docs/design/retrieval.md` §3. Result: §5.

- **What:** `PlaceRecognizerParams.enhance = none | clahe` (clip 2.0, 8×8 tiles; gray directly, RGB on Lab L). It
  runs inside the recogniser on a copy, so the geometry path's pixels are untouched (unit test). `Name()` and
  `Record()` carry `+clahe`. `ariadne_place_eval --retrieval-3.1` runs the ablation, with the seeded operator D
  applied for darkened conditions.
- **Result (EuRoC, 297 queries):**
  - CLAHE's ΔR@1 is +0.013, −0.010 and +0.013 on clean / `dark1_dust0` / `dark2_dust0_ag8`. All n.s. (p ≥ 0.45).
  - The clean dump equals D62's byte for byte.
  - The darkened proxy is *easier* than clean (0.667 vs 0.626, p = 0.023), cause `[UNVERIFIED]`. So it is not a test
    of CLAHE in the dark.
- **Decision:** the default stays `none`. CLAHE is a selectable retrieval-path ablation, never applied to the geometry
  path.
- **Remaining part of 3.1:** the §31.5 sim row (dark + co-moving light + retrieval-path enhancement), which needs a sim
  retrieval evaluation with revisit ground truth. That evaluation also settles D90's 640×360 vs 1280×720 question.
- **§49:**

  | step | status |
  |---|---|
  | catalogue | CLAHE is OpenCV-native (`cv::createCLAHE`); no new learned model |
  | arch decision | D92, plus the §6.3 blockquote |
  | ablation matrix | §31.5 retrieval-path row: EuRoC proxy done (§5), sim row open |
  | benchmark config | `ariadne_place_eval --retrieval-3.1 [--degradation]` + `retrieval_rows_eval.py methods` |
  | manifest | `place_recognition` names carry `+clahe`; the tool's manifest records the degradation name, seed and digest |

## 2026-10-22 — D74: the global-optimality certificate, behind a flag (chordal surrogate, grounded Cholesky, relative η)

**Context.** Arch §14.8 asks for an SE-Sync-style certificate as a verification artifact. Design:
`robust-backend.md` §7. Code: `cpp/slam/robust/certificate.{hpp,cpp}`.

**Decision.**
- **What is certified:** SE-Sync's isotropic chordal cost of the active edge set (κ/τ from the information diagonal
  blocks, D70), weighted by GNC weight. It is not our anisotropic Log cost. Reports say "certified for the isotropic
  chordal surrogate", and the certificate is one-directional: `not certified` means unknown, not wrong.
- **The point:** LM on the chordal residuals (OpenCV `LevMarqBase`, anchor fixed), then an exact translation solve. The
  distance moved and the remaining Riemannian gradient are reported.
- **The matrix:**
  - the translation-explicit M, assembled from the residuals rather than transcribed;
  - Λ_i = sym((M Xᵀ)_{R_i} R_i);
  - S = M − Λ̃, SE-Sync's sign. Arch §14.8's Λ − Q̃ is the other sign; the noise-free test fixes it.
- **§49 substitution of the test.** Arch §14.8 says shifted Lanczos. Instead:
  - Cholesky of S with the **anchor's translation grounded** and **η added to the rotation diagonal only**;
  - by Schur, this is exactly SE-Sync's reduced test Q̃ − Λ + ηI ≻ 0;
  - Lanczos (full reorthogonalisation) runs only on failure, to report λ_min;
  - a certified point carries the bound f − f* ≤ |gap| + 3nη.
- **η is relative:** η = 1e-10·‖M‖∞. Also reported: the absolute-η verdict (SE-Sync's 1e-3 [UNVERIFIED default]) and
  the smallest passing decade of ‖M‖.
- **The flag:** `loop_closure.certificate` (optional key, false when absent; no shipped config sets it).
  - When on, it runs after each solved loop event on the mirror, read-only.
  - The trace gains a `certificate` object only when it ran.

**Why grounded, not a full shift.** Shifting the translation blocks too is a weaker test, since (A + ηI)⁻¹ ⪯ A⁺. On
a non-certified planted-outlier graph a full shift would pass at 0.61× the reduced η, so it over-certifies.

**Evidence** (`tests/unit/test_certificate.cpp`, 8/8 plus the env-gated SE-Sync test):
- the objective equals tr(XMXᵀ), and the gradient matches finite differences;
- noise-free 5 nodes: explicit null space 4 (3 rotation + 1 translation shift), reduced null space 3, the other sign
  indefinite, certified;
- grounded test = reduced test at ±5 % of −λ_min, and the Ritz value matches the dense reduced λ_min;
- a stationary non-global point (a wound ring, max|grad| 1.6e-12) and a perturbed point are not certified;
- low noise: 10/10 certified, worst bound 4e-4 of f;
- the loop backend is bit-identical with the flag on, and 12/12 events are certified.
- **SE-Sync, run so far** (chordal init → refine):

  | graph | f | Table 1 | bound |
  |---|---|---|---|
  | parking-garage | 1.26252 | 1.263 | f − f* ≤ 0.024 % |
  | tinyGrid3D | — | — | certified |
  | smallGrid3D | — | — | certified |

- **The large SE-Sync graphs (2026-10-22, approved; `evaluation/runs/robust-backend/certificate_sesync/`).** One
  container each, in order, hard memory cap, cgroup `memory.peak`. grid3D ran only after the projection rule written
  into the manifest beforehand (≈ 12 kB per edge on the first four, ×10 for 3D-lattice fill → < 6 GB) passed.

  | graph | nodes / edges | f | Table 1 | verdict | f − f* ≤ | LM its, max\|grad_R\| | peak | wall |
  |---|---|---|---|---|---|---|---|---|
  | sphere2500 | 2500 / 4949 | 1687.01 | 1.687e3 | **certified** | 1.1e-3 | 72, 7.4e-5 | 79 MB | 20 s |
  | torus3D | 5000 / 9048 | 24227 | 2.423e4 | **certified** | 3.6e-3 | 200 (cap), 2.9e-2 | 152 MB | 180 s |
  | cubicle | 5750 / 16869 | 717.126 | 7.171e2 | **certified** | 2.9e-2 | 30, 1.9e-5 | 215 MB | 16 s |
  | rim | 10195 / 29743 | 5460.9 | 5.461e3 | **not certified** | — | 56, 2.4e-4 | 348 MB | 89 s |
  | grid3D | 8000 / 22236 | 84319 | 8.432e4 | **certified** | 0.13 | 200 (cap), 2.3 | 616 MB | 7176 s |

  - Every objective reproduces Table 1 to its 4 significant figures.
  - **torus3D and grid3D stopped at the LM iteration cap** with a gradient far from zero, and still certify. That is
    valid: the bound comes from weak duality (S ⪰ 0 at the multipliers Λ of the point reached), not from
    stationarity. A non-stationary point simply gets a looser bound (grid3D: 0.13, 1.5e-4 % of f).
  - **rim is not certified, and its test assertion is red** (gtest exit 1). λ_min ≤ −0.233 at η = 1e-10‖M‖∞
    (9.8e-6). The η that would pass is 0.977 ≈ 1e-5‖M‖∞. The objective matches Table 1, so this is most likely an
    under-converged Λ (gradient 2.4e-4 after 56 iterations) rather than a non-global point, but that is not shown.
    Verdict: **unknown, not wrong**. Neither η nor the LM budget was changed after seeing it: that would be tuning
    to pass.
  - Cost: grid3D's two hours are the Lanczos and Cholesky passes of the 200-iteration refinement on 22 k edges. The
    runtime path never certifies graphs this size (the flag is off and the loop backend's graphs are small).

**Planted outliers, reported, not tuned** (local-slam.md#estimator-plan §3):
- 3 uniform SE(3) loops kept in a 60-node cost: **6/10 certified**, from each of the truth, dead-reckoning and chordal
  starts. All starts reached the same stationary point, so the "strictly worse point must fail" assertion had
  nothing to bite on here; the wound ring covers that.
- After GNC: 10/10 outliers rejected and 10/10 certified.
- The phased-plan gate's wording "not certified on planted-outlier graphs" is therefore **not met as worded**. A
  certified outlier-corrupted optimum is correct behaviour: the certificate proves optimality of the given cost, not
  correctness.

## 2026-10-22 — D75: the outlier ramp uses perceptual-aliasing groups; breakpoint reported

**Context.** D73's ramp used random SE(3) outliers and found no breakpoint up to 90 %. The exit gate needs one.

**Decision.** A reusable generator, `cpp/slam/robust/aliasing_generator.hpp` (header-only, test support), plants
groups of false loops Z_k = T_{a_k}⁻¹ Δ T_{b_k} on two runs of consecutive keyframes. These are Sünderhauf &
Protzel's grouped outliers: mutually consistent through the true odometry.
- Its own test asserts that PCM accepts a planted group whole; otherwise it would have rebuilt random outliers.
- The protocol (in the test header, fixed before the run): 60 loops per graph, policies `size10` and `count2`, arms
  GNC-only and PCM → GNC, 10 seeds per rate, success = precision ≥ 0.99 and recall ≥ 0.95, breakpoint = the lowest rate
  with success < 0.5.
- 3.5 reuses the generator with JointId chains, `cross_chain_only` and `kInterAgentLoop`.

**Result** (`evaluation/runs/robust-backend/outlier_ramp/`):
- PCM → GNC breaks at **90 %** (`size10`) and **70 %** (`count2`). That is exactly where the largest false group
  outnumbers the true loops, PCM's maximum-clique limit.
- GNC alone: no breakpoint (worst 7/10 and 5/10, both at 70 %), at ~10× the runtime.
- The shipped order stays PCM → GNC (D72); this is its stated limit.

## 2026-10-22 — D76: an entry point for the robust stage on a joint multi-agent graph; shared gravity option

**Context.** collaboration.md §5 says the broker's solve uses 2.5's robust pipeline (main's row). §11 puts the gate at the
verifying agent, so the broker sees pre-gated edges.

**Decision.**
- **`RobustJointSolve`** (`cpp/slam/robust/joint_robust.hpp`): PCM → chordal re-init over the anchor's PCM-accepted
  component → GNC-TLS. Nodes are JointId(agent, kf).
  - The re-init removes the hazard that `SolveMultiAgent` places agents along unfiltered inter-agent edges.
  - `GatedInterAgentEdge` gates on the broker when it holds the geometry.
  - Preconditions are in the header: noise-model information; GncState only for an append-only joint graph; GNC
    decides every loop; PCM is per agent pair.
- **`GravityOptions::shared`:** one gravity variable for all agents (key `kSharedGravityAgent`). The per-agent default
  decouples the agents' tilt in a joint graph.
  - A fixed 6-DoF anchor plus a free shared gravity *is* the yaw-only gauge, up to a global rotation; do not also fix
    gravity.

**Evidence** (`tests/unit/test_joint_robust.cpp`, 2/2):
- **Aliasing rejected:** agent 1 placed along a false edge of a 6-loop inter-agent aliasing group; on 5 seeds, 10/10
  true inter-agent loops kept, 0 false. The ATE equals the true-loops-only solve's.
  - Re-init changed nothing in these seeds: PCM had already removed the false edges, and LM recovered from the bad
    placement.
- **Tilt bias:** a 2° tilt bias on all inter-agent edges.
  - Per-agent gravity leaves agent 1 at 2.59°/2.77° tilt.
  - Shared gravity: 2.34° (−10 %) with 0.01 rad edges, 0.90° (−68 %) with 0.05 rad edges. The correction follows the
    priors' confidence against the edges'.

**Not done, stated.** `AgentSubgraph` carries no gravity priors, and `SolveMultiAgent` does not call the entry point.
Both are main's (3.2 / 3.5).

## 2026-10-22 — D86: Phase 2 acceptances (EuRoC 9/11, TUM fr1, D74 wording, D91 pool rule, D90 size)

**Criterion.** Decided with an independent review, against this criterion: "we SHOULD NOT
COMPROMISE ON QUALITY AND ROBUSTNESS to a reasonable degree, if we are at par with sota lit mostly im good to go".
Each verdict below is held to that criterion against the local-slam.md#estimator-plan §1 reference table (ORB-SLAM3 T-RO 2021
Table II). Nothing here is from memory.

1. **EuRoC 9/11 (D85): accepted as the VIO gate.** The gate bar (average ≤ 0.138) is met at 0.120.
   - **The parity statement, exact:** VIO only, 0.120 sits beside Kimera (0.119) and below VINS-Fusion (0.138). It is
     not at BASALT (0.051) or ORB-SLAM3 (0.035, full SLAM). So it is inside the published stereo-inertial range, at
     the weak end, not "state of the art".
   - V2_02 misses by 10 mm, which is inside D85's own ±10 mm single-run spread. V1_03 (0.329, 92 lost frames) is the
     one real miss.
   - **V1_03 is not closed as won't-fix.** It is fast motion with lost tracking, which is this project's core
     robustness case. It becomes the regression case for Phase 5's degraded-vision work, with projection-guided
     re-association (D85's trigger) as the candidate fix. Register: 2.1 and 5.x rows.
   - **The full-system run is owed.** With loops on (D73 2b), three flights already sit at ORB-SLAM3's level:
     MH_01 0.021 (ORB-SLAM3 0.036), V1_01 0.038 (0.038), V1_02 0.025 (0.014). No 11-flight loops-on run exists.
     The SOTA-parity claim may only be made from that run, which is the like-for-like comparison with ORB-SLAM3.
     It is a gate item for Phase 2 (heavy, so it is approved before it runs).
2. **TUM fr1 (D100 after renumbering): accepted as "regression pinned"**, which is the gate's wording.
   - The fr1 rows are a cross-class comparison: ORB-SLAM2's RGB-D rows are full SLAM; ours is VO with loops off.
     The deployed rig is stereo, not RGB-D, so this does not block.
   - **Condition:** the fr1/xyz target (0.017) is tagged [UNVERIFIED]. It is sourced from the cited table, or its
     in-class verdict is dropped, before the number is cited anywhere.
3. **D74's gate wording: accepted as "certified fraction reported".** A certificate proves optimality of the given
   cost, not that the cost is outlier-free. The wound-ring test covers the intent of the original wording: a
   strictly worse stationary point is not certified.
   - **Blocks the Phase 2 checkmark:** parity with SE-Sync is shown only on parking-garage (Table 1) and two small
     grids. sphere, torus, grid3D, cubicle and rim must land (about 2 h) before D74 or Phase 2 is marked done.
4. **D91: per-thread instances accepted; the process-wide pool rule is accepted only for shared-wrapper DNN use.**
   `cv::setNumThreads(1)` also throttles the always-on per-frame pipeline (remap, pyramids, LK), which is what the
   COOL benchmark measures (INV2).
   - **Carry to 7.3:** the benchmark's thread setting is decided explicitly and recorded in its manifest, and pool=1
     is never set process-wide in the runtime that the benchmark times.
5. **D90's sim size: accepted as provisional (640×360), to be settled by data.** The sim retrieval evaluation reports
   a 640×360 vs 1280×720 recall row, and the default follows that row. Owner: 3.1.

**What the Phase 2 checkmark now waits on, in order:**
1. the merged-main build and push;
2. the 2.6 (TUM) merge;
3. the integrated premerge;
4. the SE-Sync large graphs;
5. the 11-flight EuRoC run with loops on;
6. the phase-gate full run.

Items 3, 5 and 6 are heavy: each is approved before it runs.

## 2026-10-23 — D88: 5.6's real-data low-light check uses EuRoC's dark flights; OIVIO dropped

**Context.** Plan 5.6 validates the degraded-vision path on OIVIO. On 2026-10-22 the official page said "downloads
temporarily unavailable", and every Drive ID returned 404. The candidate substitute, CAVERS, is
RGB-D, not stereo, so it does not match the rig.

**Decision (2026-10-23).** "its not that deep, darkening is already a stretch goal ... go with Euroc's
dark flights."
- 5.6's real-data check runs on EuRoC's dark, fast flights: MH_04_difficult, MH_05_difficult and V1_03_difficult.
- They are real stereo + IMU with ground truth, at our rig's modality. V1_03 is already the D86 regression case.
- OIVIO is not pursued.

**Claims, stated.** EuRoC's dark flights are low-light handheld/MAV footage under ambient light, **not** a
co-mounted lamp in total darkness. They validate the low-light robustness of the pipeline on real sensors. The lamp
falloff and active-exposure claims (5.2/5.3) rest on the sim (5.1 calibration) alone, and are written that way.

**§49:**
- catalogue: `algorithms.md` dataset note;
- arch: the arch decision is this entry;
- ablation matrix: the 5.6 row names the three flights;
- benchmark config: the 5.6 config lists the flights;
- manifest: unchanged, since the dataset is already recorded by name.

## 2026-10-23 — D89: supersedes D88 — 5.6 runs on UMA-VI's illumination-change sequences (open, real stereo + IMU); EuRoC's dark flights stay as the full-GT companion

**Decision, 2026-10-23:** "if there is a dataset available openly without chasing maintainers go for it, revert
the decision". So D88's choice of EuRoC as the primary is reverted.

**Checked 2026-10-23:**

| dataset | access | rig match | verdict |
|---|---|---|---|
| OIVIO (ARPG) | page now has no download links at all | stereo + IMU + onboard lamp, the ideal | gone |
| ViViD++ (KAIST) | request form (`forms.gle`), auto-reply mail | RGB/thermal/event/depth + IMU, not stereo RGB | out: a request, and the wrong rig |
| SubT-MRS (CMU) | no download location confirmed | multi-camera + LiDAR + thermal | out: unconfirmed |
| **UMA-VI** (Málaga, IJRR 2020, doi 10.1177/0278364920938439) | **Zenodo record 7809282, open**: API lists every file; a ranged GET returns 206 | **hardware-synced stereo (Bumblebee2 + a custom stereo rig) + IMU, handheld** | **chosen** |

**Decision.**
- **The 5.6 primary:** UMA-VI's five `IllChange` sequences, about 20.7 GB:
  - third-floor-csc1 / csc2;
  - conference-csc1 / csc2 / csc3.
- **Metric:** the dataset's own protocol. Ground truth exists only at the start and end of each sequence, so the
  number is the accumulated drift at the end (the authors' tools), not ATE. Report it that way.
- **The full-GT companion:** EuRoC MH_04/MH_05/V1_03 stay as D88 set them, because they carry full-trajectory ATE.
- **Claim scope, as in D88:** abrupt illumination changes and low light on real sensors. Not a co-mounted lamp in
  total darkness; the lamp claims rest on the sim.

**Open, to settle from the data before writing code:**
- What `IllChange` actually shows (lights switched off and on is the expected content) [UNVERIFIED]: inspect
  `sample.zip` first.
- Image size and rate, the IMU model and rate, and the on-disk format: verify against the calibration files.
- **The licence differs by source:** Zenodo's metadata says CC BY 4.0; the university repository (RIUMA) says
  CC BY-NC-ND 4.0. Apply the stricter one. Evaluation use only, and no UMA-VI pixels in any submitted material
  (as D35 for EuRoC).

**§49:**
- catalogue: an `algorithms.md` dataset entry;
- ablation matrix: the 5.6 row names the 5 sequences and the 3 EuRoC flights;
- benchmark config: a 5.6 config;
- manifest: the dataset name plus the Zenodo file md5 (the record ships `checksum.md5`).

## 2026-10-23 — D87: the full-system EuRoC run (11 flights, loops on) on AWS; the loop FIFO deadlock found and fixed

**Run.** `infra/eval/euroc_full.sh` via `spinup.sh --profile cpux86`: a fresh c7i.4xlarge spot instance with the
dev image from ECR (`sha256:04d8a7c5…`).
- EuRoC is fetched from ETH; all 11 sha256 match the local provenance.
- A clean build, then two flights at a time on disjoint 8-CPU sets (8 OpenCV threads each, as locally).
- Configs `euroc_v2` (perception) and `euroc_v2_loops` (slam).
- Metric: keyframe-refined ATE (D57).
- Evidence: `evaluation/runs/vio/euroc_full_aws/` (run 1 at d4a8a9c, run 2 at 36709eb).
- Cost: $0.015 (failed attempt) + $0.049 + $0.044.

**Determinism.** The VIO column reproduces D85 to 4 decimals on all 11 flights (average 0.1202) on a fresh x86 host
with a clean build. Between runs 1 and 2, the 9 flights the fix does not touch are identical in every results.yaml
field except timings.

**Bug (run 1).** MH_04 and MH_05 closed 0 loops: about 1000 verified candidates each, `events: 0`,
`unprocessed_at_end` about 350.
- **Cause:**
  - Before inertial init, the VO skips the window solve when no keyframe is free or landmarks are too few. Each
    skip loses that exit edge.
  - So the loop mirror (seeded by the first final odometry edge) starts at keyframe 33 on MH_04 and 77 on MH_05.
  - A gap after the anchor would throw in `Sync`, so these are startup prefixes only.
  - Both flights start and end in the same place, so the first candidates point into the prefix. They never become
    `Ready`, and `Drain`'s FIFO waited on them for the rest of the run.
- **Fix (36709eb):**
  - `LoopBackend::Unreachable(k)` = seeded ∧ k < anchor ∧ not mirrored, which is exact for the reason above.
  - `Drain` drops those candidates only (the query's others still run) and counts them (`unmirrorable`,
    `mirror_anchor_kf` in results.yaml).
  - Unit test `LoopBackend.KeyframesBeforeTheAnchorAreUnreachable`.
- **Not done, registered (2.4 row):** the VO-side prefix itself. An uninformative bridge edge would let PCM
  pass almost any pair spanning it. The same chain feeds the 3.2 broker.

**Result (run 2, keyframe-refined ATE in m):**

| flight | MH01 | MH02 | MH03 | MH04 | MH05 | V101 | V102 | V103 | V201 | V202 | V203 | avg |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| VIO (loops off) | 0.068 | 0.044 | 0.099 | 0.223 | 0.140 | 0.054 | 0.047 | 0.329 | 0.039 | 0.110 | 0.169 | 0.120 |
| **full system (loops on)** | **0.021** | **0.028** | **0.033** | **0.046** | **0.044** | 0.038 | 0.027 | 0.288 | **0.030** | 0.071 | 0.097 | **0.066** |
| ORB-SLAM3 (T-RO 2021 Table II) | 0.036 | 0.033 | 0.035 | 0.051 | 0.082 | 0.038 | 0.014 | 0.024 | 0.032 | 0.014 | 0.024 | 0.035 |
| class worst (local-slam.md#estimator-plan) | 0.166 | 0.152 | 0.125 | 0.280 | 0.284 | 0.076 | 0.110 | 0.120 | 0.070 | 0.100 | 0.190 | 0.138 |

- **MH_04 and MH_05 after the fix:**
  - MH_04: 272 events, 408 candidates dropped as unmirrorable, 0 bad loops among the GNC inliers.
  - MH_05: 155 events, 536 dropped, 0 bad inliers.
- **The statement, exact:**
  - The full system is at or better than ORB-SLAM3 on 7/11 flights (all five MH, V1_01 and V2_01).
  - The average is 0.066: between BASALT (0.051, VIO, 10 flights) and Kimera (0.119); not at ORB-SLAM3 (0.035).
  - It is in the stereo-inertial class on 10/11 flights.
  - V1_03 is the outlier: 0.288, 12× ORB-SLAM3. Without it the average is 0.043.
  - V1_02, V2_02 and V2_03 are 2–5× ORB-SLAM3.
  - All are single runs, seed 1.
- **What this changes:**
  - D86 item 1's "owed" run is done. The SOTA comparison is stated as above, not as parity.
  - V1_03 stays the 5.x regression case (D86, D89).

## 2026-10-23 — D150–D155: the live closed-loop runtime (L1)

**Context.** Register row L1: every Ariadne result came from a recording, and the mission's decisions were logged,
never executed. Design and alternatives: `docs/design/live-runtime.md` (arch §5.4, §5.8–§5.10, §25.1, §29).
Each decision below is summarised there with its reasons; the evidence is filled in from the runs.

- **D150 — ROS stops at one relay.** gz → `ros_gz_bridge::RosGzBridge` (the 1.1 bridge YAML plus the command and
  mover-state entries) → `ariadne_bridge::RuntimeLink`, both loaded into ONE component container with intra-process
  comms (the bridge's own launch action composes it this way; no DDS hop, where D40's losses were). RuntimeLink
  serializes each typed message to CDR and writes `(topic, bytes)` frames to a Unix socket; `ariadne_live` (core
  image, ROS-free) decodes them with `runtime/replay/ros_cdr`, the bag decoder. No ROS type enters `cpp/`; the sim
  image still has no OpenCV and the core image no ROS. Packages: `ros2/ariadne_bridge`, `ros2/ariadne_sim`.
  No custom ROS interfaces: commands are `geometry_msgs/PoseArray` ↔ `gz.msgs.Pose_V`, state `std_msgs/String` ↔
  `gz.msgs.StringMsg` (header stamp and frame_id verified to pass through the bridge, 2026-10-22).
- **D151 — goto / follow / hold on the kinematic mover.** PoseArray in `world`: 0 poses = hold, 1 = goto,
  N = follow; header stamp = the command id; a zero quaternion = no final yaw. The mover appends segments planned by
  `core/trajectory/command_planner.hpp` (turn to face, move, optional final turn; motion.py's `plan_profile` and
  routes.py's clearance-scaled speed: the `PlanProfile` test reproduces every turn and move of the Python golden
  scripts exactly). A new command replaces every segment not yet started; the one in progress finishes (all end at
  rest). The scene ESDF at ρ_agent is checked first; a violation refuses the command and the agent holds. Every
  command is answered on `mover_state`; the executed script is logged with exact doubles (`mover_agent_<k>.jsonl`)
  so canonical IMU/GT can be regenerated later (D39). Test: `python/tests/sim/test_mover_live.py` (real plugin in
  gz: goto accepted and reached, a goto through a wall refused with the envelope reason, hold accepted).
- **D152 — the recording is what was consumed.** Arrivals are reordered by header stamp behind a watermark over the
  timed topics (hold-back 3 s, timeouts counted) and every released message is written to `inputs.mcap` with log
  time = release sequence, so a replay sees the consumed order even where stamps tie. `ariadne_live --replay` feeds
  it through the same code; the first check is an FNV-1a digest over (topic, stamp, payload). A stereo frame whose
  partner never arrives is dropped and counted in live mode (`SimFrameAssembler`, lenient); bags keep the strict
  rule. "Reproduces" was defined before the run: `python/analysis/live_replay_check.py`.
- **D153 — the mission runs incrementally on a sim-time barrier.** The 1.8 layer moved into
  `planning/mission/frontier_mission.hpp`; `ariadne_mission` calls it with **byte-identical outputs** (trace, routes,
  slab, support, snapshots, results; merged and both single-agent modes, committed spool, old vs new binary). Live,
  a replan at sim time T waits until both estimators have passed T. The deterministic layer lists every frontier
  cluster reachable at ρ_agent (the candidates, nearest first: element 0 is 1.8's choice) and a chooser picks one
  (`classical` now: the nearest one at least `min_goal_m` = 1.0 m away, because the first live smoke showed the
  nearest cluster is always the band under the agent, path 0 m, so 1.8's rule executed never moves; 7.1 adds the
  orchestrator, validated against the set). The pick becomes a follow along the A*
  path (approxPolyDP, every chord re-checked, cut at 3 m), ending facing the cluster. Each agent is anchored by its
  own briefed entry pose; the map merge is Phase 3's.
- **D154 — console fed live.** `ariadne_live` appends `telemetry.jsonl`; the console server's live mode
  (`ARIADNE_LIVE=run:scene`) tails it and serves the replay frame schema, logging every frame it sends
  (`console_frames.jsonl`). Test: `python/tests/console/test_live_console.py` (telemetry appended while the socket
  is open reaches the client; the stream ends when the run does — this test found an end-of-stream bug, fixed).
- **D155 — where live runs execute.** Built: `live_aws.sh` (one g4dn.xlarge spot, the whole D84 quota, through
  `spinup.sh` and its three brakes; images in ECR tagged by image id, pushed 2026-10-23; locally built binaries as a
  checksummed S3 payload). **Not run:** the launch needs a go-ahead for the spend, and the NVIDIA EGL
  path of the sim image is still an open `[SMOKE-TEST]`. **The acceptance runs below are local, Mesa llvmpipe, RTF
  ≈ 0.14** (no GPU on the dev host). The register's acceptance does not depend on the GPU; a real-time-factor-1 demo
  does.
- **D152a — the estimators take turns, one frame at a time** (a process-wide lock in `LiveSimSource`). Found by the
  replay check of a 25 s run: replay 1 equalled the live run, replay 2 differed on agent 0 in ONE pose of 501 (frame
  158, 0.1 mm; same tracks and correspondences, PnP inliers 323 vs 322). `[UNVERIFIED]` cause: two estimators
  calling into OpenCV's one `parallel_for_` pool at once; a busy pool changes stripe partitioning and so the order of
  floating-point reductions. With turns, three replays of the same recording all equal the live run. Sequential tools
  (sim_collab) never had two pipelines at once.
- **D153b — standoff goals; the previous outcome is an input.** The 30 s acceptance run on `943a558` closed the loop
  at t = 5 s (both gotos accepted; each agent's ESTIMATED distance to its goal 2.50 → 0.86 m within 5 s), then the
  mover refused 10/10 follows: the goal was the frontier cell itself, and a frontier borders unknown space that is
  often an unseen wall, invisible to a planner that only avoids OBSERVED occupancy. The mission re-chose the refused
  target at every replan. Now: goals lie in `trav_safe` (free with ρ_agent clearance to anything not observed free),
  as near the cluster as that allows (≤ 2 m), reached over `trav_safe` plus an escape radius of 0.5 m around the
  agent; each replan first waits for the mover's answer to the previous command (in the recording, so deterministic
  in replay; a live wait is bounded at 30 s and counted) and never re-chooses a goal within 0.5 m of a refused one.
  Checked before any new live run: on the SAME recorded observations, all 9 new command paths clear the GT ESDF at
  ρ_agent (minimum 0.275 m), where the old ones were refused. `ariadne_mission` is untouched (byte-identical again).
- **D154 fix.** The first acceptance run's console served 1012 frames over 204 s of wall time, every one at sim time
  0: `live.sh` pointed it at the outer run directory, the runtime writes one level down. The console now resolves
  the runtime's directory; the test uses the real layout.

**Unchanged, checked.** `SimSource` after the frame-assembly extraction: `ariadne_sim_vo` on the d41 bag gives
sha-identical `estimate_body.tum` for both agents, old binary vs new.


## 2026-10-23 — D110: 4.1 volumetric stack — from-scratch voxel-hash TSDF at 5 cm (HashTSDF the oracle), incremental FH ESDF, 3D frontiers, coarse RLE layer

- **What** (`docs/design/volumetric.md`, `cpp/planning/volumetric/{tsdf_map,esdf,frontiers3d}`):
  - 8³-voxel blocks in a hash; along-ray SDF, clamped to τ = 3 voxels; per-keyframe averaging, then one fusion per
    voxel, with weights `(σ_ref/σ_Z)²`; a silhouette guard.
  - Tri-state: unknown = weight 0; occupied = SDF < half a voxel.
  - ESDF: distance to the nearest point of a blocked cube (the GT's and D45's convention), by three exact windowed
    min-convolution passes, capped at ρ_responder + 2 voxels. Two fields: egress (unknown blocks) and agent.
    Recomputed per dirty box (± reach), never globally.
  - **Correction (2026-10-23):** as first committed (`f3eb6bb`), clearance was `edt·v − v/2` on an FH
    (centre-to-centre) transform, mislabelled as the GT convention. It is optimistic on diagonals, gave 4 false
    egress claims at GT clearance 0.395 m, and the brute-force test had copied it. The map metrics are unaffected
    (they read the TSDF slab). The 5 cm decision was re-run with worldgen's `_edt_to` and survives. The preflight
    was incomplete: `docs/design/next-best-view.md`.
  - Frontiers in the motion band, 26-connected, with centroid, normal and a 64-ray visible-unknown fan; updated
    incrementally (FUEL's rule).
  - Coarse layer: 0.5 m cells, RLE codec.
- **Why from scratch:** SMOKE §4u. `cv::Volume` has no voxel or weight read and no per-pixel weight. It stays the
  oracle (a test).
- **Supersedes the arch's 8–10 cm voxel:** measured on both scenes' GT, a one-cell wall bleed closes agent 1's
  entrance at ρ_responder for 8 and 10 cm, not for 5 cm.
- **Evidence:** 8 tests (§7). On the evidence spool vs 1.8: coverage 0.942 / 0.933, free precision 0.979 / 0.961,
  occupied precision 0.865 / 0.707; about 105 ms per keyframe on the dev host. Corrections made after measuring,
  each reported: unknown = weight 0 (the first run used `w_min = 0.5`, coverage 0.861); the oracle tolerance is two
  voxels (the oracle's own p95 error is 1.35 voxels).
- **Not here:** the live keyframe path (L1's `frontier_mission.hpp` seam, once committed) and the wire field for the
  coarse summary (3.2's).
- **§49:**

  | step | status |
  |---|---|
  | catalogue | Voxblox (block hash, weighting, projective error), nvblox, FIESTA, FH 2012, FUEL: `volumetric.md` §1, `docs/design/next-best-view.md` §2; **`algorithms.md` entry owed** |
  | arch decision | D110, plus the §20.1 blockquote |
  | ablation matrix | 1.8 OctoMap placeholder (D45) vs D110, same spool and scorer |
  | benchmark config | `configs/mission/mission_v1.yaml` (stride, gates, band); D110 defaults in `TsdfParams` |
  | manifest schema | `ariadne_volumetric_eval` manifest (`layer: phase4_volumetric`), spool sha256 |


## 2026-10-23 — D111: 4.2 traversability at ρ_agent / ρ_responder — egress = observed column ∧ 3D occupied capsule; structural risk [PROXY] built, uncalibrated

**Partial: traversability is done in replay; the risk proxy is not calibrated.**

- **What** (`docs/design/traversability-risk.md`, `cpp/planning/volumetric/traversability.{hpp,cpp}`):
  - **Egress at ρ_responder** (the product, invariant 4) is the intersection of two tests:
    - D45's observed-column rule on 4.1's TSDF slab, with 1.8's `EdgeClearance` ≥ ρ;
    - a standing-body capsule (axis 0.45–1.40 m) on the occupied-only 3D field.
  - **Agent at ρ_agent:** a capsule over the flying rig's 0.8–1.2 m altitude band on the agent field.
  - **Risk `[PROXY]`:** saturating counts of down-facing undersides and of mass over open columns, in a 1 m disc,
    in the 1.8–2.4 m band.
  - Outputs on the briefing grid: traversability, per-column clearance, risk.
- **Supersedes arch §22.1's 26-connected 3D lattice:** 2.5D, because a responder walks on a flat floor. The 3D test
  survives as the capsule.
- **Why not strict 3D:** the rig cannot observe a standing body's volume. On the evidence spool the strict rule has
  recall 0.354 and closes the target. Reported, not the product.
- **Evidence:** 5 tests, with independent brute force. On the evidence spool: precision 1.000, recall 0.900,
  starts → target both, against 1.8's 1.000 / 0.898 / both (`evaluation/runs/mapping/traversability/`).
- **Changed after measuring:**
  - the strict → hybrid rule (design review);
  - the clearance convention (D110's correction);
  - the support test (before any calibration data existed).
- **Open:**
  - risk calibration against GT zones (worldgen hazards + renders; GPU spend to be asked);
  - Voxblox's half-voxel inflation (P2), decided separately.
- **§49:**

  | step | status |
  |---|---|
  | catalogue | STEP (risk map as cost), GBPlanner2 (ground beneath edges): `traversability-risk.md` §1; **`algorithms.md` entry owed** |
  | arch decision | D111, plus the §20.5 / §22.1 blockquotes |
  | ablation matrix | strict 3D vs hybrid vs 1.8, same spool and scorer |
  | benchmark config | `TravParams` defaults; ρ from `configs/mission/mission_v1.yaml` |
  | manifest schema | `ariadne_volumetric_eval` results (`traversability`, `risk_terms`) |


## 2026-10-23 — D112: worldgen structural hazards (cantilever slabs, hung masses) with GT risk zones, for calibrating D111's risk proxy

- **What** (`docs/design/traversability-risk.md#risk-zones`, `python/dataset_tools/worldgen/hazards.py`):
  - An opt-in preset section `hazards`. Its own Philox stream (`S_HAZARDS = 40`) places cantilever slabs (held by a
    room wall) and hung masses, with undersides ≥ 1.95 m: above a standing responder, so they threaten a route
    without blocking it.
  - Six-face meshes, bottom included (`K_HAZARD`).
  - `scene_gt.npz` gains `hazard_footprint` and `risk_zone` (footprint ⊕ 0.5 m, `[ASSUMPTION]` a debris-fall
    margin); `scene.json` gains `hazards`.
  - New preset `configs/worldgen/ci_tiny_hazard.yaml` (ci_tiny + 2–3 cantilevers + 1–2 hung masses).
- **Why these two:** the US&R collapse patterns (secondary sources, §1 of the design). Cantilever collapse is "the
  most dangerous type of collapse to operate in". Lean-to and V slabs reach the floor and would need a 3D GT
  traversability: out of v1, stated.
- **Existing scenes unchanged, proven:**
  - at its default the section is left out of `Preset.to_canonical()`, so all four preset digests equal their
    pre-D112 values (pinned in `test_hazards.py`);
  - every golden scene-directory digest still matches (`test_golden.py`);
  - a no-hazard scene's `scene_gt.npz` gains no layers.
- **Tests:** 112/112 worldgen tests. The new ones:
  - digests;
  - a twin check: `ci_tiny_hazard` seed 0 has the same attempt, `free`, `esdf`, `trav_*`, starts, target and
    scripts as `ci_tiny` seed 0, since the streams are keyed by (seed, attempt, site);
  - placement constraints;
  - outward faces, with the bottom normal at −z;
  - the GT margin by hand.
- **Scenes:** seeds 0 / 1 / 2 have 5 / 4 / 5 hazards (scene ids `f232a40b77b3538f`, `d0ab877b3a06fc04`,
  `3406f0792c57a975`).
- **Next:** render on AWS g4dn (decision, 2026-10-23), run the Phase 1 estimator chain to a spool, then the 4.1
  map, then calibration by the protocol fixed in the design §3: the threshold on seed 0, the report on seeds 1–2,
  and AUC ≥ 0.75 to enter planning.

## 2026-10-23 — D113: 4.5 egress = D* Lite on D111's ρ_responder lattice, many-exit backward search, 5th-percentile body-support confidence

- **What** (`docs/design/egress.md`, `cpp/planning/route/`):
  - **Search:** Koenig & Likhachev Fig. 3 (transcribed). The search runs from the exits, each seeded rhs = 0, to
    the fixed target. k_m = 0, and h = octile(target, ·), which is consistent because every edge costs ≥ its length.
  - **Lattice:** 1.8's, 8-connected with the corner rule. A non-traversable column has no edge (invariant 4 as a
    constraint).
  - **Edge cost:** |x−y|·(1 + mean endpoint penalty), with penalty = w_u(1 − support) + w_r·risk +
    w_c·max(0, ρ_safe − clearance)/ρ_safe. The weights are 1 / 0 / 1, fixed a priori; w_r stays 0 until D112's
    calibration passes.
  - **Support** = the observed fraction of the ρ-capsule body envelope.
  - **Replan:** every changed cell re-runs UpdateVertex on its 3×3 block.
- **Why:**
  - arch §22.1/§22.3 name D* Lite;
  - the map changes under a fixed plan, and each replan touches only the vertices whose costs changed;
  - the 1.8 A* stays as the ablation baseline (unchanged).
- **Measured:**
  - tests: D* Lite = fresh Dijkstra exactly on 180 random flip rounds (7.6× fewer expansions);
  - evidence spool: 12/12 routes GT-valid and the final route GT-optimal, the same as 1.8 (both first route at 145 s);
  - incremental = fresh at the final replan.
- **Pre-registered range check fired** (support spread 0.0999 < 0.1). The geometric observable band
  ([−0.31, 2.31] m, from altitude 0.8–1.2 m, half-FOV 28° and the 2.84 m depth gate) contains the whole envelope,
  so the switch is a no-op (byte-identical support).
- **Calibration:** degenerate (all valid, confidence 0.900–0.902), reported as degenerate.
- **Exit-gate reroute:** open; no run-level instance yet.

## 2026-10-23 — D114: 4.3 NBV design (generators A–E, a shared FUEL viewpoint sampler, 256-ray IG, p90 self-normalisation, one path cost, the energy-reserve rule); gate on lockstep L1

- **What** (`docs/design/next-best-view.md`):
  - **Generators A–E** feed one cylindrical viewpoint sampler (FUEL): radii 1.0 / 1.5 / 2.0 m, 16 azimuths,
    z = 1.0 m; kept if in `trav_a` with line of sight; best 2 per target.
  - **IG:** a 256-ray frustum bundle to 2.84 m.
    - Δcoverage: unknown voxels;
    - Δentropy: with p_occ = ½e^(−w) for free voxels and 1 − ½e^(−w) for occupied, the weight's unit being one
      σ_ref observation;
    - Δegress: unknown voxels in the route envelope;
    - Δpose_connectivity: a planning-side `ConnectivityOracle`, the Δλ₂ of 3.4's selector through a runtime
      adapter;
    - Δloop_disambiguation: `AliasSource`.
  - **Normalisation:** each term by its candidate-set p90, and G = the sum.
  - **Costs:** the path-proportional costs (motion, time, energy) are counted once, as
    λ_path(t) = (1 + λ_e(t) + λ_t(t))/3 ∈ [1, 5/3], with the arch's cubic schedules. Bandwidth and risk are
    p90-normalised with λ = 1.
  - **Hard rule:** if E_remaining < 1.3·E(return), only {return, hold}, as a supervisor event.
- **Why:**
  - arch §21.2–§21.4 as written, except cost normalisation, an extension flagged in the design: without it every
    λ⁰ is unit-bearing;
  - counting the costs once avoids the hidden 3× motion weight that per-term normalisation of ∝-path costs would
    create.
- **Measured before adoption:** Δentropy is not saturated on the evidence-spool map (known-voxel w p50 0.74–0.83;
  mean H 0.61–0.66 bit; a third over 0.9 bit).
- **Conflict surfaced:** two §21.3 producers are absent on this branch. 3.4's spectral gain is unmerged
  (3.4), and §17.3's alias flags don't exist anywhere. Those terms are 0, with the reason in the trace,
  until they land. They're never a stand-in value.
- **Gate:** the coverage-vs-budget ablation has pre-registered axes (§5). It runs on **lockstep L1** (decision):
  the live runtime builds a gz pause at each barrier, so runs are reproducible per seed. The replay decision log tests the
  machinery only.

## 2026-10-23 — D115: 4.4 allocation = CBBA (L_t ≤ 2, event-triggered) and broker SGA on a time-discounted DMG score with common auction scales

- **What** (`docs/design/allocation.md`; the paper transcribed in algorithms.md §22.3.1):
  - **Algorithms:** CBBA (Algorithm 3, Table I, eq. 6 release), run only on a new task, a completed task, or a
    topology change. Broker sequential greedy (SGA) on the same inputs. Both logged identically.
  - **Score:** S_i(p) = Σ_j c̄_ij · exp(−λ_path · d_ij(p) / P_path).
    - c̄_ij = the D114 value net of bandwidth and risk;
    - d_ij(p) = the geodesic distance along the path;
    - P = common scales: the max over agents of each term's p90, frozen per allocation event.
- **Why:**
  - **DMG holds** (paper eqs. 11–12, triangle inequality on geodesics). D114's linear U is not DMG for two-task
    bundles, because an insertion shrinks a neighbour's detour.
  - **Under DMG, Theorem 1 makes CBBA's assignment equal SGA's.** So the arch's "auction vs broker-greedy" row
    measures communication (rounds, messages, bytes), not quality, and "logged identically" becomes an exact test.
  - **Common scales make bids comparable across agents.** Per-agent p90s would not.
- **Oracle:** exact every round. L_t = 1 by `cv::solveLP` (total unimodularity). L_t = 2 with N_u = 2 by a sorted
  disjoint scan.
- **Wire:** `TaskConsensus` (MessageClass TASK_CONSENSUS = 10 already exists) is to be added with the pending
  3.4 edits to `wire.fbs`, still open. Until then the bytes come from a test codec, labelled, and are
  never claimed.
- **Replay** exercises only the connected two-agent case. The disconnection and staleness regimes come from the
  synthetic tests and live L1.

- **D115 correction (2026-10-23):** "CBBA = SGA under DMG" is exact at **L_t = 1** (2000 / 2000) but not at L_t = 2
  (5 / 1000 diverge through a stale second-slot bid, each with an eq. 6 tail release; a hand-built instance pins
  it). **Default L_t = 1.** L_t = 2 is a reported row, and its round and 50 % bounds are empirical there.
- **D115 measured (2026-10-23, replay; `evaluation/runs/planning/allocation_replay/`):**
  - CBBA = SGA on 40 / 40 replans. CBBA/optimum on linked replans: mean 0.994, worst 0.983 (greedy pick: 0.985 /
    0.900).
  - 2.4 KB per TASK_CONSENSUS message, 13.9 KB per linked event.
  - Event triggering saves only 26 % of bytes, because 4.3's task set churns. That's reported, not tuned; top-K
    announcement and deltas are listed as possible design changes.
- **D114 gate amendment (2026-10-23, before any gate run; reason: the pilot's flight speed, not a result).**
  - **Axes:** primary x = mission time (10 fractions of 200 s, on lockstep barriers); secondary = summed metres
    flown at the barrier points.
  - **Map:** every row is scored from one shadow Phase 4 map.
  - `e_total_m` stays 120 m; the reserve rule and λ_e do not engage at the pilot's ≈ 0.15 m/s (stated).

- **D112 result (2026-10-23):** the risk proxy passes its pre-registered bar (evaluation AUC 0.831 ≥ 0.75; control
  FPR 0) and **enters planning**. It saturates, though: the threshold falls at 1.0, with FPR 0.34. Overhang alone is
  better (AUC 0.92, FPR 0.16) and is recorded as the next pre-registered candidate, not switched to post hoc.
  - Evidence: `evaluation/runs/mapping/risk_calibration/`, one g4dn spot run, ~$0.21.
  - Provenance: payload built at df38a12, run snapshot fc0c0a9 (diff checked: no estimator or map code differs).
- **D115a (2026-10-23): sparse TASK_CONSENSUS.**
  - Only tasks with a winner travel, and bids go as doubles. The encoding is exact: assignment, bids and rounds are
    identical on 300 random instances through encode → decode → expand.
  - Replay: 226 B per message (was 2.4 KB), 11.8 KB per spool (was 123.4 KB), CBBA = SGA 40 / 40.
  - The L_t = 2 row: CBBA = SGA 40 / 40, the same optimum ratio (0.994), +1.4 s per replan. **L_t = 1 stays the
    default.**
- **Phase 4 literature review of every post-design fix (2026-10-23, before any gate run).**
  Primary-text reports: the literature notes `phase4_fixes_nbv` and `phase4_fixes_route_cbba` (archive/dev-history branch).
  - **D114, plan space: supported.** Known-free viewpoints and paths match FUEL (code), NBVP 2018 §3 and GBPlanner2
    §IV-A. Optimistic planning through unknown space appears only for goal-directed navigation.
  - **D114 / D158, goal risk: the max over the escape disc has no precedent.** It is replaced by the bottleneck at
    the commanded cell, which is known-free since the plan-space change. That is the FUEL / NBVP / GBPlanner2 practice.
  - **§21.4 return: E(return) is now computed over the known-free plan plus the agent's swept body along its flown
    track (`FlownReturnSpace`).** This follows GBPlanner's homing over robot odometry and CERBERUS's Recovery Homing.
    - It fixes the live-run finding that E(return) = inf put every replan on {hold}.
    - A route lost even over the track is its own supervisor event and does not restrict the options `[DESIGN]`.
    - The swept band is agent-width only: it never enters a ρ_responder claim (invariant 4).
    - ×1.3 is the architecture's `[DESIGN]`; GBPlanner2's margin is additive.
  - **D115: the earlier reason for L_t = 1 was wrong.** Theorem 1 holds for any L_t. Its DMG precondition fails for
    our insertion marginal: 7.8 % of triples in a Python port, and pinned in C++.
    - BuildBundle now warps bids (Johnson et al. CDC 2012, eq. 6). It is the identity at L_t = 1.
    - At L_t = 2 the bids are monotone, conflict-free, and converge within N_min·D (asserted). 5 / 1000 instances
      still differ from raw-score SGA, which is not the reference.
    - L_t = 1 stays the default because it is trivially DMG, the planner flies one task per replan, and L_t = 2
      costs 1.4 s per replan.
  - **D116: "as FAR joins start/goal through its local planner" was wrong.** FAR uses visibility edges and otherwise
    attempts the goal through unknown space. The lattice join is `[DESIGN]`.
    - The both-directions line-of-sight check is `[DESIGN]`: it misses no crossed cell but over-blocks on 272 of 624
      segments.
    - The D\* Lite trigger split is supported (AAAI 2002).
  - **D116 amended 2026-10-24 (D123): "FAR ≤ string-pulled lattice" is FALSE.** After the test's map draws were
    sequenced, `VisibilityGraph.ConvexGraphMatchesAllVertexReferenceAndBeatsLattice` trial #9 routes 37.40 cells
    against the lattice's 33.24 (+12.5 %).
    - The old pass came from x86's argument order, which drew a different map.
    - Same defect class as hazard seed 0 (exploration.md §10).
    - The test is quarantined (`DISABLED_`, unchanged) and OPEN, parked until after the freeze.
    - FAR stays `[DESIGN]`. No report or video may claim FAR ≤ lattice. The live `Route` uses A*.
## 2026-10-22 — D100: 2.6 TUM RGB-D — sensor depth into StereoVo through a virtual rig; regression pinned (fr1 not in class)

- **Loader:** `runtime::TumSource` (`cpp/runtime/datasets/tum.hpp`), with `infra/scripts/fetch_tum.py` (sha256
  pinned). Facts are from the TUM file-format page, read 2026-10-21:
  - depth is stored at a factor of 5000, pre-scaled, and registered to the distorted RGB;
  - ground truth is the colour camera's pose, written `qx qy qz qw`;
  - RGB and depth frames are associated within 20 ms.

  Timestamps are parsed to nanoseconds from the decimal string.
- **Undistortion:** we use the calibrated fr1/fr2 intrinsics with 5 coefficients. The SAME map is applied to RGB
  (bilinear) and to depth (nearest), which is exact for depth registered to the distorted pixels.
  - **Deviation:** the TUM page recommends its uncalibrated ROS default set. We follow ORB-SLAM2's published
    setup instead. `--intrinsics ros_default` is the ablation (not run).
- **Depth path (§9.6 contract):** an RGB-D source declares a `DepthSensorModel`
  (`perception/rgbd/depth_stereo.hpp`). The runner then takes explicit RGB-D branches; stereo sources take the
  unchanged path.
  - The right point is synthesised: u_R = u_L − fx·B/Z, with virtual baseline B = 0.08 m (ORB-SLAM2's RGB-D virtual
    stereo).
  - The sensor Z fills the §9.6 slot.
  - Keyframe records get a `StereoDepth` built from the sensor.
  - Depth is masked to [0.4, 4] m and to a 3×3 max/min ratio of at most 1.05 (flying pixels).
  - Depth is rectified nearest-neighbour (`StereoRectifier::RectifyLeftNearest`).
  - The radiometric stage is refused on RGB-D sources.
- **σ_Z model, stated:** σ_Z = k·Z² with k = 1.425e-3 m⁻¹. Source: Khoshelham & Elberink, Sensors 2012, eq. 7,
  with |m/fb| = 2.85e-5 cm⁻¹ and σ_d′ = ½ px, transcribed from the PMC full text. It enters the triangulation gate as
  the virtual rig's disparity noise, σ_d = k·fx·B ≈ 0.059 px, set in code and written to `results.yaml`.
  - **Stated consequences:**
    - the §9.6 cross-check compares Z with itself and cannot reject;
    - BA weights the virtual right point at `ba.sigma_px`, about 15× looser than the model (ORB-SLAM2 does the same).
- **Configs:** `tum_v1` (perception and slam) is euroc_v2's values with `imu.mode none` (TUM has no gyro) and
  `min_depth_m` 0.4. `ba.sigma_px` stays EuRoC's calibrated 0.46 [ASSUMPTION].
- **Regression (`configs/benchmark/tum_regression.yaml`, checked by `infra/scripts/tum_regression.py`):**
  - It is a separate pinned check, not part of `--premerge` (decision).
  - Targets were fixed before measuring: the worst of ORB-SLAM2 T-RO 2017 Table III's RGB-D class.
  - Seed 1, full sequences, ATE with SE(3) alignment. evo_ape (`tum -a`) on TUM's raw `groundtruth.txt` agrees to
    1e-6 m on all three.
  - Tolerance: pin × 1.1 + 2 mm.

  | sequence | frames | ATE m | target m | in class | keyframes | lost |
  |---|---|---|---|---|---|---|
  | fr1/xyz | 792 | **0.0496** | 0.014 (Kerl et al. IROS 2013 Table III; was 0.017 [UNVERIFIED], corrected 2026-10-23 per D86) | **no** | 305 | 0 |
  | fr1/desk | 573 | **0.0581** | 0.037 | **no** | 343 | 0 |
  | fr2/xyz | 3615 | **0.0139** | 0.029 | yes | 370 | 0 |

- **The negative result, reported:** both fr1 sequences are 1.6–2.9× above the class bar. That class is full SLAM
  (loops, global BA); ours is VO with loops off. Causes not yet measured: [UNATTRIBUTED].
  - Candidates:
    - fr1's fast handheld motion with a rolling shutter;
    - RGB/depth time offset of up to 20 ms (unsynchronised on fr1);
    - a keyframe every ~2 frames (305/792) under euroc_v2's `ref_ratio`;
    - `ba.sigma_px` not re-calibrated;
    - the calibrated-vs-ROS-default intrinsics choice.
- **EuRoC unchanged:** MH_01, 300 frames, `euroc_v2`. `estimate_body.tum` and `vo_trace.csv` are byte-identical
  between `main` (aa7dce2) and this change.
- **Cost:** track ≈ 16–18 ms per frame, keyframe ≈ 24–30 ms (640×480).

## 2026-10-22 — D101: L2 — the seeded sim IMU noise operator; sim_v2 runs the full inertial layer

(Decision IDs D100–D109 are claimed for 2.6 in STATUS.)

- **Operator:** `core/inertial/imu_noise_operator.hpp`. It applies the Kalibr/EuRoC continuous model, discretised at
  the nominal rate, to the sim's clean canonical IMU stream at consumption time (D37), like D:
  - white noise σ_nd/√Δt;
  - a bias random walk, step σ_rw·√Δt, from the config's initial bias (zero).
- **Determinism:** each sample's draws are Philox keyed by (seed, sample index): `PhiloxStream(seed, k, 0)`, with a
  fixed draw order. The bias is the running sum of steps 0..k−1, so any replay prefix is bit-identical (tested).
  - The seed is `DeriveSeed(DeriveSeed(run_seed, "imu_noise"), "agentK")`, the same plumbing as D's.
- **Config:** `configs/sim/imu_noise_v1.yaml` is rig-scoped (`rig: rig_stereo_v1`). It holds the ADIS16448 densities
  exactly as EuRoC's `imu0/sensor.yaml` gives them, so sim and EuRoC share one noise regime.
  - It is a separate file, not keys in `rig_stereo_v1.yaml`, because worldgen snapshots the rig into
    `mission_*.json`. New keys there would change future scene outputs for an operator that acts at consumption
    time.
- **Wiring:** `SimSource::SetImuNoise`, via `ariadne_sim_vo --imu-noise YAML`. The source exposes **no IMU unless
  the operator is set**, so default sim runs, including the Phase-1 gate's `sim_v1`, see what they saw before.
  - `ImuNoiseModel()` reports exactly the injected model.
  - `results.yaml` records `imu_noise_config`.
- **`sim_v2` → `imu.mode full`** (decision). It uses euroc_v2's imu section with noise inflation 1,
  because the injected model is the estimator's model.
  - The pre-switch `sim_v2` is kept verbatim as `sim_v2_vo`, as euroc_v2 → euroc_v2_attitude was. D69 and the
    synthetic `StereoVo` tests now name it.
  - `sim_v2` without `--imu-noise` is refused by the runner's existing "no IMU" check.
- **Evidence:**
  - Unit tests:
    - prefix replay is bit-identical at n = 1/17/1000/3999 of 4000;
    - zero densities return clean + b₀;
    - white-noise σ matches within 2 % (60k samples × 3 axes);
    - the bias-walk variance equals σ_rw²·T within 20 % (300 seeds × 3 axes, a ~4σ bound);
    - the shipped config loads.
  - The `StereoVo` lost-frame and inertial tests pass on `sim_v2_vo` (4/4).
- **Sim run** (ci_tiny_0 agent 0, 1000 frames, seed 1, `sim_v2` + `imu_noise_v1`):
  - initialised on the first attempt at 5.35 s, with unconstrained |g| 9.79 m/s² and χ²/dof 4.1;
  - gravity error vs GT: median 0.13°, p90 1.3°;
  - final gyro bias 1–4e-4 rad/s per axis. The injected walk's 1σ after 50 s is 1.4e-4;
  - inertial χ²/dof 0.30;
  - **ATE 3.06 mm** (keyframe-refined 2.79 mm) vs **2.76 mm** for the visual-only path on the same frames (D69's
    clean row), 0 lost frames. The clean sim is easy, so this shows the layer works on the sim IMU; it does not
    show a benefit.
  - Keyframes cost ≈ 200 ms with the inertial solve (at ≤ 1 keyframe per second).
- **Not built:** `ariadne_sim_collab --imu-noise` (multi-agent), and ground-truth inertial states for the sim (so
  `init_vs_gt` reports −1).

## 2026-10-22 — D102: 3.3 comms broker node — RFComms-derived link model with wall attenuation, scripted partitions, one ledger; L8 keeps the 1500 B MTU

(Decision IDs D100–D109 are claimed for 2.6 in STATUS.) Design and numbers:
`docs/design/comms.md`.

- **`RfLinkModel` implements the collaboration.md §2 `LinkModel` seam.**
  - RFComms' equations are transcribed from gz-sim8, with its quirks reproduced and tagged.
  - It adds a per-wall term: solid blocks crossed, from a ray-cast against the scene's GT wall faces, counting face
    entries once per surface. Doorways are free.
  - `loss_p` = E[PER] over the shadowing, integrated on a fine grid. It matches a Monte Carlo of RFComms'
    per-packet procedure.
- **`CommsNode`** owns the link stack (base + `FaultPlan`), the `Network` and the one `ByteLedger`. Only Endpoints
  write the ledger; forwarding adds no rows (tested).
- **Scenarios:** `configs/comms/` holds ideal, `rf_gz_default`, `collapsed_v1` and `collapsed_partition_v1`.
  - Parameters that depart from gz defaults are tagged [ASSUMPTION].
  - The COST-231 heavy wall, 6.9 dB, is [UNVERIFIED-primary].
- **Measured on ci_tiny** (`ariadne_comms_probe`):
  - The gz-default radio is loss-free at these ranges, the stated null.
  - `collapsed_v1` loses 1.6 / 17 / 49 % of 1500-byte datagrams at 0 / 1 / 2 walls.
  - The partition scenario is down for exactly the scripted 25 s.
- **Seam limit, stated:** `Sample()` has no datagram size, so `loss_p` is for 1500 B. 40-byte heartbeats would see
  1.3–2.3× less loss near the edge. A size on the seam is 3.2's call; nothing in
  `collaboration/protocol/` was changed.
- **L8, conditional: keep `kPathMtuBytes` = 1500 under RFComms as modelled.**
  - At `collapsed_v1`'s median power, goodput rises with size: 0.61 at 64 B vs 0.80 at 1472 B.
  - At the edge (the 2-wall mean, −81.7 dBm) the best size is 512 B, and 1472 B costs ≤ 3 %.
  - Under the physical case instead (PER in bits, slow per-link fading), 1472 B gives 0.06 against 0.59 at 64 B: a
    3–10× loss at the edge.
  - Recommendation for 3.2 (comms.md §4): if the link model is made physical, cap or adapt edge-link
    fragment size. The serializer is not changed here.
- **3.3 is partial.** No protocol traffic runs over the real link until 3.2's sim integration (`protocol: v2`)
  exists.

## 2026-10-22 — D130: the sim render is sRGB-encoded; D gains an sRGB decode (opt-in, default unchanged)

SMOKE §4u: gz-sim8/ogre2 R8G8B8 frames are sRGB-encoded (lamp ×2 gives pixel ×1.38; the L16 camera is the same render
decoded, linear, ×2.00). D (arch §5.6) reads `input/255` as linear radiance, so every D run so far scaled, Poissoned and
quantized **encoded** values. A dark config's `radiance_scale` 0.06 on encoded values is a 0.06^2.2 ≈ 0.002 change in
linear light at the top and a different one at each level: not the darkness the config name claims.

- `DegradationParams.input_transfer: linear | srgb`. `srgb` decodes (IEC 61966-2-1 piecewise) before stage 1. D's output
  stays linear raw: the sensor model is a linear machine-vision camera, which is what the §6.2 Poisson–Gaussian fit and
  the GAT assume.
- Default `linear`: the golden digest, the gate and every pinned result are byte-identical. Phase 5's frozen configs
  (5.4) set `srgb`.
- **Caveat, not re-run:** D runs on SIM frames (the 2.2 radiometric rows, D67–D69) used encoded values.
  - **What stands:** the gate decisions. D's noise is affine in D's own output whatever the input encoding, so the
    GAT fits and the on/off calls are unaffected.
  - **What changes:** the meaning of the darkness labels. `dark3` on encoded values is not the linear-light darkness
    its name says.
  - **EuRoC is untouched by this probe.** The 3.1 darkened-proxy rows, and radiometric.md's `[UNVERIFIED]`
    companded-response note, concern EuRoC's camera. A sim render measurement cannot close them, so the note stays
    open.

## 2026-10-22 — D131: lamp photometry as rendered; the rig lamp becomes inverse-square, wide and shadow-casting

SMOKE §4u measured what the renderer does, so Stage 3 (§6.2) and the lamp calibration (5.1, §5.3) fit THAT law:

  I(x) = A · RID(θ) · albedo · cos(incidence) / (c′ + r²),   r, θ from the lamp centre and axis (T_C_L)

- **Attenuation.** The rig shipped (constant 0.2, quadratic 0.05): c′ ≈ 10 m², nearly flat to 3 m, nothing like a real
  lamp. Set (0, 0, 1), which renders c′ ≈ 0.5 m²: inverse-square beyond ~1.5 m. c′ is a renderer artefact; the
  calibration estimates it and Stage 3 uses the calibrated value, so the code does not assume 0.
- **Cone.** SDF spot angles are FULL cone angles. §5.3's "inner/outer cone 25°/40°" is read as half-angles (a headlamp
  that lights the ~86° camera view), so the SDF gets 50°/80°. As shipped the lamp lit only the central ~40°.
- **Lobe ("cosine lobe decision", plan 5.1).** Within the inner cone the RID is flat (±0.6 %); Lambertian incidence
  contributes exactly one cos. The calibration stores the RID as a table over θ (flat top + roll-off), not cos^μ.
  Separating RID from incidence needs tilted planes (Park et al., CVPR 2014), so the calibration renders several plane
  poses.
- **Shadows.** World `<shadows>true</shadows>`, lamp `cast_shadows true`; dark worlds drop the zone point lights and set
  ambient 0. Verified to render on llvmpipe.

## 2026-10-22 — D132: 5.3 exposure + lamp co-controller; Stage 4 normalisation; the replay lamp actuator

`perception/exposure_control/` (arch §6.5), CPU-only so far, not yet wired into a runner:
- **Objective:** Shim, Lee & Kweon (IROS 2014; arXiv 1708.07338 eq. 1, transcribed): mean over pixels of
  log(λ(m − δ) + 1)/N for m ≥ δ, δ = 0.06, λ = 10³ (the paper's values). Candidates at ±0.5 stop (0.125 grid) are
  synthesised by scaling the linear frame and clipping (Zhang, Forster & Scaramuzza, ICRA 2017, with a known response),
  not Shim's γ-mapping, which stands in for an unknown response. Ours is known: linear, D's model (D130).
- **One brightness decision, spent by cost:** brighter = lamp (to its energy budget) → exposure (to the blur ceiling
  t ≤ blur_px / (f |ω|), §10.5) → gain; darker = the reverse. Slew ≤ 0.5 stop per decision (§6.5).
- **Fallbacks, stated:** > 2 % clipped → step down at the slew limit (clipped pixels hide how far over they are);
  no gradient above δ in any candidate (many stops under or over) → step toward mid-grey. The second was found by the
  closed-loop test: from 8 stops under, Shim's metric is 0 everywhere and a pure-metric controller never moves.
  **Not a new finding:** algorithms.md §9.4, read from Zhang et al. ICRA 2017, already says M_shim is "better, still
  not robust", and that a gradient-percentile metric wins 13 of 18 datasets. The stall and the limit cycle below are
  that documented weakness. **Comparator row (§6.5):** `metric: percentile` (the p-th percentile of gradient
  magnitude, Zhang's M_perc; our grid search needs no smooth approximation).
- **Deviation under §49, flagged for decision:** arch §6.5 has the lamp "optimizing the same objective plus an
  energy cost term". D132 spends one brightness decision in a FIXED cost order (lamp → t → gain), which is the
  limiting case of an energy cost below the noise cost of gain. A continuous energy-weighted split is not built.
  The same test found a limit cycle: the metric barely penalises clipping, so it stepped up into 20 % clipped and the
  guard stepped back, ±0.5 stop forever. Candidates whose synthesised frame would trip the guard are now excluded.
  Closed loop from 8 stops under and 5 over: settles within 5 % of the best clip-limited metric, |step| ≤ 0.25.
- **Stage 4:** `NormalizeExposure` scales a linear image by (t₀g₀)/(tg). Lamp power is not divided there; Stage 3
  owns P.
- **Replay:** the decision drives D's `exposure`/`analog_gain` and the new `lamp_power` (a radiance scale). With
  ambient 0 and a decoded render, lamp power is exactly a radiance scale (SMOKE §4u). The approximations that remain:
  pre-render clipping and quantisation, the lamp's shadows (unchanged by power), and exposure time adding no motion
  blur in D. The live-vs-replay comparison (exit gate) measures them.

## 2026-10-22 — D133: 5.1 lamp calibration procedure, executed for rig_stereo_v2

`python -m dataset_tools.lamp_calib render|fit`: a matte grey plane (SDF diffuse 0.5) under the rig's lamp at 15 known
poses: 0.75–4.5 m × {fronto-parallel, yaw 30°, pitch −25°}. Tilts make incidence differ from the lamp angle, which is
what separates the lobe from cos(incidence) (Park et al., CVPR 2014 — read at abstract level only). c′ is not only a
renderer artefact: algorithms.md §14.2 (DarkGS/NeLiS, IROS 2024) measured real lamps departing from 1/r² at close range
and fitted a Lorentzian term, which is this model's form. So c′ stays a fitted parameter on real data. Fit: for a fixed c′, I(c′ + r²)/[cos inc] is a
function of θ alone, whose per-θ median is A·RID. c′ minimises the across-pose spread (golden-section search). There
is no initialisation to get wrong. The lamp pose is the rig's (sim truth); the procedure calibrates what the renderer
adds.

Result, `configs/sim/lamp_calib_v2.yaml` (568 k samples):
- gain_A 1.952 at intensity 6. That is 0.325 per unit intensity; SMOKE §4u measured 0.327.
- c′ = 0.543 m² (§4u: ≈ 0.5).
- RID flat to ±0.5 % out to 25°, rolling off to 0.04 at 39.5°.
- **Cosine decision (plan 5.1): Lambertian incidence is IN.** Held-out log-RMS is 0.048 with it and 0.167 without.
- All-pose log-RMS 0.052, dominated by 8-bit quantisation in the dim tail.

## 2026-10-22 — D134: 5.2 Stage 3 falloff normalisation → albedo proxy

`perception/radiometric/falloff.{hpp,cpp}`: albedo = I · albedo_ref / max(E, E_max/max_gain), with
E = gain_A · p · RID(θ) · cos(inc) / (c′ + r²).
- r and θ are taken from the lamp centre (T_C_L, rotated into the rectified frame), not the camera.
- Normals come from depth central differences.
- Depth is the estimator's (the §9.6 contract: SGBM or RGB-D); never GT in production. A GT-depth row is an
  upper-bound ablation only.
- Pixels outside the lobe (RID < 0.05), without depth, or at grazing incidence (cos < 0.15) are invalid. Unlit pixels
  have no albedo to recover.
- Placement: after rectification and stereo, so it cannot feed SGBM.
- The geometry path may consume it (§6.3: calibrated physics); whether features do is a §31.5 row.
- Synthetic tests: exact albedo recovery on a tilted textured plane; distance and power invariance; invalidity; the
  gain cap.
- **Normals from stereo depth are the weak link.** The calibration's "cos is in" (D133) describes the light. Dividing
  by an ESTIMATED cos is a separate estimator choice, so `incidence: calibrated | off` is a parameter.
- Test: a tilted plane, depth from 10 cm-baseline disparity quantised to 1/16 px plus 0.1 px noise. Median albedo
  error by normal baseline:

  | normal baseline | median albedo error |
  |---|---|
  | 1 px | 88 % |
  | 3 px | 14 % |
  | 8 px | 4.8 % |
  | incidence off (the shading bias) | 11.6 % |

- Default 8 px, calibrated. A plane flatters long baselines (they smear normals across depth edges), so the §31.5 row
  on renders decides between calibrated and off.

## 2026-10-22 — D135 `[OPEN — measurement pending dark renders]`: L9 dust fidelity (D9, D11) — options built, decision after one measured pair

All opt-in. Defaults are the §5.6 operator, so the golden digest is unchanged. Options:
- `dust_order: pre_sensor` (D9 option c): the medium acts on radiance before shot noise. Test: over a black scene in
  dense dust, post-sensor airlight has zero variance, while pre-sensor airlight has shot-noise variance at the same
  mean.
- `dust_field_tau_frames: N` (D11 option b): the field blends the epoch fields at frame/N and frame/N + 1, normalised so
  that its variance does not pulse. `dust_drift_px_per_frame` adds advection. Test: frame-to-frame change is under ¼ of
  the white field's.
- `dust_airlight_mode: lamp`: the veil follows the lamp lobe and scales with `lamp_power`. **Superseded as physics
  (preflight, 2026-10-23):** it keeps the daylight shape A·(1 − e^{−βd}), so the veil grows with scene depth. algorithms.md
  §10.2 (after Akkaynak & Treibitz, CVPR 2018/2019) gives the headlamp model:
  - **direct term:** out-and-back attenuation, e^{−2β_D d};
  - **backscatter:** an integral along the ray, ∫ I_L(θ)/r² β_B e^{−2β_B r} dr. It is dominated by the near field,
    saturates within tens of cm, and is nearly depth-independent beyond. β_D and β_B are two separate coefficients.
  - The lamp offset sets where the cone enters the view, and §10.2 calls it the highest-value variable in the corpus.
  - **Replacement:** `dust_model: headlamp` (D139).

**Retracted 2026-10-23:** "dark worlds use `dust_airlight_mode: lamp`, decided on physics". That model is the wrong
physics (above). Dark worlds will use the D139 headlamp model once it is built and tested.

**To measure, one factor per row, at MATCHED mean veil.**
- In pre_sensor mode airlight is scaled by `radiance_scale`; post-sensor it is not. Comparing them at equal
  `dust_airlight` would therefore measure "more fog" (about 16× at dark2), not fidelity. Each row sets the airlight so
  the mean veil matches.
- Rows: order (post vs pre) at a fixed field, then tau (0 vs ≈ 1 s of frames) at a fixed order.
- Metrics: match precision, track length, ATE.
- Replay is CPU-only once one dark recording exists.

**Before the 5.4 freeze:**
- The dark0..3 levels are re-specified against lamp-lit renders with `input_transfer: srgb`.
- With the decode, even `radiance_scale` 1 puts midtones about 2 stops lower than the encoded reading did.
- The "ambient normal" row uses the same decode, or the matrix would compare two sensor models.

## 2026-10-23 — D136: TUM fr1 follow-up (D100) — scoring-side diagnosis; the RGB/depth-offset rerun is next

Analysis of the pinned D100 outputs only (`estimate_body.tum` against `groundtruth.txt`; no rerun).

| Check | fr1_xyz | fr1_desk | fr2_xyz | Reading |
|---|---|---|---|---|
| ATE, SE(3), as pinned | 49.6 mm | 58.1 mm | 13.9 mm | — |
| best estimate↔GT time shift (±60 ms) | 49.5 mm at +10 ms | 55.3 mm at +35 ms | 13.9 mm at 0 | a GT clock offset is not the cause |
| Sim(3) ATE (scale) | 46.9 mm, s 0.919 | 58.0 mm, s 1.002 | 9.8 mm, s 0.974 | no global scale bias explains fr1 |
| median 1 s ‖Δest‖/‖Δgt‖ | 0.994 | 0.992 | 1.026 | local scale is right; a depth-scale error is ruled out |
| orientation error, first-frame aligned | 1.1 → 2.6° | 1.6–2.8° | 0.4–1.7° | no large rotation drift |
| 1 s translation RPE | 10.7 mm | 16.2 mm | 2.7 mm | fr1 drifts 4–6× faster per second |
| RGB↔depth stamp gap (median / p90) | 7.3 / 13.0 ms | 11.9 / 16.4 ms | 8.3 / 14.6 ms | the gaps are similar; the motion is not |
| GT speed, median (100 ms window) | 0.33 m/s, 14°/s | 0.48 m/s, 28°/s | 0.06 m/s, 4°/s | fr1 moves 5–8× faster |

- The residual over time is U-shaped (fr1_desk: 100 → 22 → 67 mm by eighths) and does not rise with instantaneous
  speed. So it is accumulated drift, not per-frame noise.
- **The RGB/depth-offset hypothesis is still open.**
  - Size: a depth frame 7–12 ms stale at fr1's speeds misplaces points by about v·Δt ≈ 2–6 mm, plus about ω·Δt ≈
    0.1–0.3° of rotation (~2–6 mm at 1 m). That is a biased per-frame depth, which could integrate into the measured
    1 s RPE.
  - Test: rerun fr1 with the depth association tightened (`max_dt` 20 → 5 ms; frames without a close depth are
    dropped) and with depth warped to the RGB stamp by the previous frame's velocity.
  - Where the flag goes: `--depth-max-dt-ms` on the TUM tool. It lands after the 2.6 TUM work merges, not as a commit
    to that branch mid-merge.

## 2026-10-23 — D137: the first GPU render session — ci_tiny_dark + its fixed-light twin on g4dn spot

Approved 2026-10-23.
- **Script:** `infra/scripts/remote_dark_render.sh`, through `spinup.sh --profile gpu` (spot, `--max-minutes 150`).
- **Image:** the exact local sim image `sha256:8b780f52…`, uploaded as zstd (735 MB) to `runs/images/`, the only
  prefix the instance role may read. The instance checks the image id before rendering.
- **GPU smoke first:** the NVIDIA EGL path has never run (SMOKE §4); the session stops if the smoke fails.
- **Recordings:**
  - `ci_tiny_dark` seed 0 + `rig_stereo_v2` (lamp);
  - `ci_tiny_dark_fixed` seed 0 + `rig_stereo_v1`: ambient 0 and the zone point lights, no lamp (§31.5 "dark +
    fixed light").
  - Both share ci_tiny seed 0's geometry and trajectories (only the lighting fields differ), so the existing
    ci_tiny_0 recording is their "ambient normal" row.
- **Upload:** each recording goes to `runs/ph5-dark/<commit>/` as soon as it finishes, because a spot reclaim skips
  spinup's end-of-run upload.

## 2026-10-23 — D138: 5.5 confined worlds and the one-pass baseline sweep

**Confined worlds.** `configs/worldgen/confined_v1.yaml`: maze_short's grammar squeezed.
- Responder corridors at the grid minimum (0.90 m).
- Three agent-only narrow passages at 0.45–0.60 m, inside the Z_min ≤ 0.3 m regime of §9.4.
- 2.2–2.4 m ceilings and more braids.
- Acceptance: 20/20 seeds (`worldgen accept`, seeds 0–19). Missions 237–340 s. Where narrow passages are used, agent
  routes are shorter than responder routes.

**Baseline sweep (§9.1, §31.8).**
- Rig option `stereo.sweep_baselines_m`: extra right cameras `right_b04/08/12/20`, rendered in the SAME pass as the
  main pair. Every baseline sees identical frames, poses and lighting, and the sweep costs one render, not four.
- They are bridged and recorded through the mission's bridge YAML, and `sim_record` verifies each one's count, period
  and pairing with the left camera.
- `configs/sim/rig_stereo_sweep_v1.yaml` = v1 + the sweep.
- Rigs without the key are unchanged: v1 worldgen output stays byte-identical to main (0 differing files).
- Tests: worldgen 106/106, including sweep poses, topics, the manifest and distinct baselines.
- **Consumer side, not built yet:** SimSource picks the right camera by name and takes the baseline from the manifest's
  `t_B_camera_m`. That change is in `stereo_source.hpp`, so it lands after the 2.6/L2 merge.
- §9.4's validator already exists (`perception_config.hpp`, `MinObservableRange`). The B = 20 cm row needs
  `num_disparities` 512, or it must declare its blind zone.
- **Confound to report with the sweep:** every sweep camera sits to the right of the left camera, while the lamp
  stays at the body centre. The lamp is centred between the cameras only at the 10 cm main pair. algorithms.md §10.7
  flags asymmetric lamp placement as a stereo photometric-consistency problem. In the lit `confined_v1` render the
  effect is absent (the zone lights are fixed); in a dark sweep it confounds baseline with lamp asymmetry.

## 2026-10-23 — D140: Stage 3 uses OpenCV's normals and enforces a feature mask (preflight corrections to D134)

The algorithms.md survey and the design review (2026-10-23) changed D134 in two ways.

**1. Normals: `cv::depthTo3d` + `cv::RgbdNormals`** (ptcloud, OpenCV 5.0.0, `opencv_ptcloud` linked into
`perception_opencv`). The OpenCV-native check had missed this; D134's hand-rolled cross product is removed because it
loses. Median albedo error on the noisy-stereo tilted plane of `test_falloff` (10 cm baseline, 1/16 px quantisation
+ 0.1 px noise):

| normals | median albedo error |
|---|---|
| **RgbdNormals FALS, window 7 (default)** | **3.9 %** |
| RgbdNormals LINEMOD, window 5 or 7 (diff threshold 0.05 m) | 4.4 % |
| D134 hand-rolled cross product, 8 px (removed) | 4.8 % |
| RgbdNormals SRI, window 7 | 5.9 % |
| RgbdNormals FALS, window 5 | 7.9 % |
| RgbdNormals CROSS_PRODUCT (1 px neighbours) | 189 % |
| no incidence correction | 11.6 % |

- **Pinned-API trap:** OpenCV 5.0.0's `RgbdNormals::create(rows, cols, depth, K, window, diff_threshold, method)`
  takes `diff_threshold` BEFORE `method`. Passing the method in that slot silently ran FALS for every method; it was
  found because all methods gave identical numbers.
- LINEMOD's threshold is in depth units: its default 50 assumes millimetres, so it is set to 0.05 m for metric depth.
- A plane flatters every method. The §31.5 render row decides between FALS and LINEMOD.

**2. No camera-fixed edges (algorithms.md §17.4).**
- Invalid pixels (outside the lobe, no depth, grazing incidence) now hold the median valid albedo, not 0.
- `AlbedoProxy::feature_mask` (valid, eroded by `feature_margin_px` 8, with the image border counted as invalid) is
  the mask every detector or descriptor on the proxy must use.
- The lobe edge and depth holes move WITH the camera. A detector firing on them makes keypoints that track perfectly
  and describe nothing in the world.

**Also (D132):** `metric: percentile` is built: Zhang's M_perc, the §6.5 comparator row. Its p is `[ASSUMPTION]` 0.5,
because Zhang's value was not transcribed. A closed-loop test settles it without clipping.

Tests: test_opencv ExposureControl/Falloff/DegradationSrgb 14/14, test_numerical Degradation 25/25, test_unit
Config/Degrad 20/20.

- **Correction (2026-10-23, D148 run).** "Recorded and verified" overstated what had been done: the sweep had never
  been recorded end to end. The first full render (run 20261023T105356Z-gpu-155842, g4dn, ~$0.088) failed twice:
  - `/tf_static`: the canonical tool listed {left, right} only. It is fixed now: every `camera` sensor on base_link,
    left and right still required, 12 transforms on confined_v1_0 checked in the sim image. Stereo rigs keep their
    order, so their canonical bags should be unchanged; to verify in the pre-merge determinism run.
  - Writer backlog: 12 RGB 1280×720 streams plus depth at 20 Hz exceeded the recorder's 4 GiB cap at t ≈ 70 s, and
    it aborted rather than drop messages (D40). The one-pass sweep is therefore not recordable on a g4dn as
    designed. Splitting it (passes or agents) is a separate decision (D148/D149).

## 2026-10-23 — D139: the headlamp dust model in D (replaces D135's lamp airlight as the physics; algorithms.md §10.2)

`dust_model: headlamp` (opt-in; it requires `dust_order: pre_sensor` explicitly, because it is light in the medium).
- **Direct:** the rendered, lamp-lit scene is attenuated out and back, J·e^{−2β_D·range}. Range is computed from GT
  z-depth along the pixel ray [ASSUMPTION: gz `depth_camera` stores z].
- **Backscatter:** single scattering of the carried lamp along each pixel ray,
  S·∫_{near}^{range} RID(θ)/|s·u − L|²·e^{−β_B(s + |s·u − L|)} ds, normalised by the coaxial on-axis infinite-range
  value.
  - β_D (`dust_beta`) and β_B (`dust_beta_backscatter`) are separate coefficients (Akkaynak & Treibitz, CVPR 2018).
  - L, the lamp centre in the camera frame, is a parameter: §10.2's lamp-offset lever becomes a replay sweep.
  - Limit, stated: a replay sweep moves only the veil, not the recorded direct light or shadows.
  - The local particulate field scales both terms.
- **Deterministic:** the integral depends on pixel and range only, so it is tabulated once per call on an 8 px grid
  × 96 geometric range nodes (0.02–60 m), with DetExp/DetSqrt and a literal node ratio. Lookups are a binary search
  plus bilinear interpolation; there is no libm on the per-pixel path.
- **Tests** (test_numerical Degradation 31/31; the golden digest is unchanged, so the default is byte-identical):
  - the veil is near-field dominated (wall at 0.5 m vs 5 m: within 10 %, where the daylight model grows > 7×);
  - a 0.30 m lamp offset cuts the on-axis veil to < 20 % of coaxial;
  - the 96-node table matches a 200k-node libm integral to < 3 % at 0.3, 1 and 3 m;
  - the direct term is exactly e^{−2βd};
  - output is byte-identical across row bands;
  - the config is strict.
- **D135 now:**
  - `dust_airlight_mode: lamp` remains only as the daylight-shape negative-control row.
  - pre_sensor and the correlated field stay.
  - The 5.4 freeze measures the headlamp model at matched mean veil against Koschmieder, on the dark recording that
    has now been rendered (D137).

## 2026-10-23 — D141: 5.6 validates on Hilti 2023 `site2_robot_3`, not OIVIO (approved substitute, §49)

**Why the substitute.** OIVIO (arch §34.2, plan 5.6) cannot be obtained.
- Its page says trajectory downloads are "temporarily unavailable" and asks people to email the authors.
- Every sequence's Google Drive ID returns 404 (checked 2026-10-22).

**Which substitute.** algorithms.md §8.1 names Hilti 2023 `site2_robot_3` as the only public sequence whose own paper
confirms an onboard, co-moving light in a dark room (RA-L, arXiv 2404.09765).
- Data: OAK-D stereo + IMU; open Hugging Face `Hilti-Research/hilti-slam-challenge-2023`, ungated, CC BY-NC-SA 3.0.
- Ground truth is public (`ground_truth/site2_robot_3.txt`): TLS-surveyed control points, 3-DoF positions only, sparse.
- The considered alternative, CAVERS (arXiv 2604.15052), is RGB-D only, so it was not taken.
- Approved 2026-10-23.

**What changes against §34.2 / plan 5.6.**
- **Not available here:** the 100/50/15 % light-power sweep (OIVIO only).
- **Metric:** sparse-position error at the control points, Hilti's protocol, not dense ATE.
- **Report:** names the substitute and the reason.
- **Unchanged:** the honesty requirement. Report where sim-tuned settings transfer and where they do not.
- **Still open:** OIVIO access may be requested from ARPG. If it arrives, OIVIO runs as well.
- **Supersedes D88/D89 (main, 2026-10-23).** D88 chose EuRoC's dark flights as the 5.6 primary; D89 then chose
  UMA-VI's `IllChange` sequences (ambient switching, end-drift metric). This entry came after both, and merge
  planning confirmed that it stands: Hilti `site2_robot_3` has the co-moving onboard light in a dark
  room, which neither of those has (algorithms.md §8.1). EuRoC MH_04/MH_05/V1_03 remain the full-GT low-light companion,
  cited from D87's full-system run (keyframe-refined ATE 0.046 / 0.044 / 0.288 m), not re-run here.

**§49 ritual:**
- catalogue: algorithms.md §8.1, already present;
- arch decision: this entry plus the as-built note in §34.2;
- ablation and benchmark config: `configs/benchmark/hilti_5.6.yaml`, written with the loader;
- manifest: dataset, sequence, sha256 and licence recorded per run, as for EuRoC and TUM.

## 2026-10-23 — D142: Phase 5 CPU work runs next to the data on AWS; the no-dust lighting rows first

**Why AWS.** This host's link measured about 1.9 MB/s total, shared with a concurrent 12 GB upload. At that
rate the two dark recordings (25 GB) plus Hilti `site2_robot_3` (35 GB) would take about 9 h. Hugging Face → EC2
and S3 → EC2 are fast and in-region. Approved 2026-10-23.

**Session** (`infra/eval/ph5_aws.sh`, spinup `--profile cpux86 --type c7i.2xlarge`; the profile is main's D86
`cpux86`, carried over).

**Image.** ECR `ariadne-core:rc-ariadne-core-dev-91643838031ab6cbda6a`, the same image as the local
`ariadne/core-dev:dev` the unit tests ran on (pushed 2026-10-22 21:01). The first launch asked for
`d86-coredev-04d8a7c58de1`, a tag that exists only locally: pull failed after 70 s, $0.0013.

**Hilti.**
- Fetched from the pinned HF revision, each file sha256-checked against its LFS oid.
- `python -m dataset_tools.hilti.to_asl` exports one stereo pair (auto-picked as the first `left`/`right` image
  topic twin; the full topic list goes to results) plus every IMU topic, in EuRoC layout, with `calibration.zip`.
- Only that extract comes back.
- The converter was tested on a synthetic ROS1 bag. Its topic names were invented: the real bag's topics are unknown
  until the session lists them.

**Replays.** Four rows, agent 0, 1000 frames, sim_v1 perception + sim_v2 slam. These are the configs of D69's lit
rows (ATE 3 mm clean) on the same ci_tiny seed-0 trajectories.

| row | recording | D |
|---|---|---|
| dark + co-moving lamp | `ci_tiny_dark_0` | none / `lampdark_sensor_v0` |
| dark + fixed light | `ci_tiny_dark_fixed_0` | none / `lampdark_sensor_v0` |

`lampdark_sensor_v0`: the sRGB decode plus the 2.2 camera (shot + read noise, PRNU, vignette), radiance_scale 1. The
darkness is the render's. It is not a frozen 5.4 level.

**Dust rows: unblocked 2026-10-23.**
- `runtime/datasets/sim_depth.hpp::WarpDepthToRight` gives the right camera's GT depth: a forward z-buffered splat on
  the render grid (u − f·B/z, nearest surface wins, disocclusions unknown, so they get no dust). Test 1/1.
- `SimSource` now buffers each stamp's left, right and (with dust) depth, then lens-maps each depth like its image
  before D. D's frame id stays the pair index.
- **Byte-identity checked:** a 40-frame `lampdark_sensor_v0` run before and after the change gives an identical
  `estimate_body.tum` and `vo_trace.csv`.
- A functional headlamp run works. At S = 0.3 on the dim 4 s probe recording (lamp intensity 2), VO is lost from
  frame 1: the veil buries the scene, the §10.2 "glowing wall". Dust strengths are set against the intensity-6
  recordings before the D135/D139 rows run.
- `configs/degradation/lampdark_headlamp_v0.yaml` holds the rig v2 lamp geometry; D works on the lens image, so
  focal 680.

**Sessions and cost** (spinup ledger, compute only; EBS/IPv4 add cents):

| session | outcome | cost |
|---|---|---|
| 1 (`d86-coredev` tag) | image tag exists only locally: pull failed after 70 s | $0.0013 |
| 2 (1b) | spot reclaim during the Hilti fetch | ~$0.007 |
| 3 (`e3bc7f6`) | Hilti export done (3.58 GB tar, sha256 `78d2c3bf…`); build failed: the worktree's `.cache` symlink ships dangling, so FetchContent could not write | $0.0198 |
| 4 (`d6da28b`) | build passed (121 s); recordings fetch failed: the instance role has GetObject, not ListBucket, so `--recursive` cannot list | $0.0068 |
| 5 (`380c900`) | Hilti side files (exposure + control points) + build + 4 replays, all OK | $0.0352 |

Fixes: FetchContent cache under `build/`; `HILTI=full|extras|skip`; recordings fetched by key from the committed
`infra/eval/ph5_dark_manifest.txt` (213 keys, sizes checked). CPU total ≈ $0.07.

**Results: the no-dust lighting rows — PRELIMINARY** (not §31.5 exit-gate rows: they ran before the
preflight rule of 2026-10-23, and the fixed-light / lamp definitions still need their preflight decision; session 5; manifests `git_commit 380c900`, `git_dirty false`, image
`sha256:91643838…`). `ariadne_sim_vo`, agent 0, 1000 frames, seed 1, `sim_v1` + `sim_v2`, 0 lost frames in every
row. All four dark rows share one trajectory (GT path 7.355 m). D69's lit row is `ci_tiny_0` with the same seed and
agent; that its trajectory is identical (only lighting differs between the presets) is [ASSUMPTION], not re-checked.

| row | ATE rmse | keyframes | landmarks alive | stereo valid |
|---|---|---|---|---|
| lit (D69 reference) | 3 mm | 80 | 770 | 0.776 |
| dark + fixed light, no D | 2.5 mm | 78 | 674 | 0.782 |
| dark + fixed light, `lampdark_sensor_v0` | 5.2 mm | 72 | 237 | 0.784 |
| dark + co-moving lamp, no D | **15.2 mm** | **219** | 471 | 0.782 |
| dark + co-moving lamp, `lampdark_sensor_v0` | **17.0 mm** | **186** | 364 | 0.777 |

- **Dark + lamp is ×6 worse in ATE than dark + fixed light on the same path, with ×2.8 keyframes.** This is a
  comparison of two lighting setups, NOT a measurement of what the lamp's motion costs: the rows also differ in
  overall brightness (not matched) and coverage (the zone lights fill the rooms; the lamp lights a cone). Attributing
  the gap to the co-moving falloff needs the brightness-matched fixed-light row of D145. The falloff is the
  plausible mechanism and Stage 3 (D134/D140) targets it; not yet isolated.
- **Sensor noise is the second-order term:** +2.7 mm fixed, +1.8 mm lamp, and roughly ⅓–½ of the landmarks lost,
  as in D69.
- One seed on one 50 s segment. Differences of ~1 mm are not claimed; the ×6 between these two setups is, its cause
  is not.

## 2026-10-23 — D143: equidistant (Kannala–Brandt) cameras through `cv::fisheye`; sequences without dense GT

**Why.** Hilti 2023's robot OAK-D Pro W (5.6, D141) is calibrated by Kalibr as `pinhole` + `equidistant`, fx 567
at 1280 px: about 127° horizontal field of view. Read as radial-tangential, its coefficients are silently wrong (the
negative control in the test below fails by far more than the tolerance). Re-rectifying offline in the converter
was the alternative. It was not taken, so that the real raw images go through the pipeline's own rectifier.

**OpenCV-native check** (in the image, `/opt/opencv5`): `cv::fisheye::stereoRectify` (stereo.hpp),
`cv::fisheye::initUndistortRectifyMap` (imgproc.hpp), `cv::fisheye::undistortPoints` / `projectPoints`
(geometry/3d.hpp). algorithms.md §camera models: KB4 = `cv::fisheye`. Its fisheye warning ("rectifying to a plane
wastes the FOV") applies, and is accepted for dense stereo.

**What changed.**
- `PinholeRadtan` gains `model` (radtan | equidistant). It is the last field, so positional init still means radtan.
  `dist` holds k1..k4 for equidistant (cv::fisheye's D order, the same as Kalibr's).
- `StereoRectifier`: fisheye path through the three functions above. `alpha` is passed as fisheye `balance`: the
  endpoints share intent, the interior does not. The flag is `STEREO_ZERO_DISPARITY` (OpenCV 5's name for
  the `CALIB_ZERO_DISPARITY` the fisheye doc still mentions). `RefineRightRows` rebuilds with the same model.
- `loop_verify_geometry`: `fisheye::undistortPoints` for equidistant cameras. The sim source states radtan.
- EuRoC loader: accepts Kalibr `equidistant`; any other name still throws.
- EuRoC-layout sequences may omit `state_groundtruth_estimate0`. The runner then writes `dense_gt: false` and
  `null` for every ATE/RPE field: the zero-default metric structs must not read as a 0 m error.

**Tests.** Hilti-shaped rig (published coefficients, B 7.5 cm, points out to 55° off-axis): rows 1e-3 px, depth
1e-4 relative. The same lens read as radtan fails. Rectified fx 468.7 px, 107.6° HFOV, 99.9 % lens-fed pixels at
balance 0. That is the crop, measured. EuRoC 12/12 (incl. accept/refuse); rectification + loop-verify 6/6.

**Hilti data preparation** (`dataset_tools.hilti`, tests 6 + 2 + 5 scorer):
- `sensor_yaml`: T_BS = T_cam_imu⁻¹. The stereo chain matches Kalibr's T_cn_cnm1 to 2e-15, B 0.07514 m, and the
  T_BS translation equals Hilti's own `T_imu0_cam0`.
- **Stereo pairing by trigger.** 13 of 3592 pairs carry centre-of-exposure stamps 9.984 µs apart: same
  `start_of_frame`, exposure 29988 vs 30008 µs. They are re-stamped to the left stamp only under those two
  conditions; anything else is refused. Exact pairing would have dropped them (200 ms holes).
- **Control points** are LiDAR measurements (`frame_id rslidar`, ~0.83 m from the sensor), stamped exactly at the GT
  rows. The GT file holds the surveyed marker positions. So the scorer places each GCP as T_W_I(t)·T_I_L·p_L
  (RA-L 2024 §V-B, Kalibr T_lidar_imu), not at the IMU.
- **Real auto-exposure** (`exposure_info`): exposure sits at its ~30 ms cap for the whole sequence; only ISO moves
  (930–1549). On 253 frames the right camera's ISO lags the left by one step. That is real-data evidence for 5.3's
  actuator order (exposure to its ceiling, then gain) and for the stereo exposure match.

## 2026-10-23 — D144: 5.6 first result: neither the sim-tuned nor the EuRoC-tuned settings transfer to Hilti `site2_robot_3` (0/100); losses track a dark far field while the robot turns; cause open

**Runs** (`configs/benchmark/hilti_5.6.yaml`, fixed before measuring; commit `1468344`, clean; all 3592 frames;
evidence `evaluation/runs/perception/radiometric/hilti_5.6/`). Hilti score, 5 GCPs, SE(3) fit / leave-one-out:

| row | score fit / loo | GCP error rms, fit | lost frames | keyframes | stereo valid |
|---|---|---|---|---|---|
| sim-tuned (`sim_v1` + `sim_v2`) | 0 / 0 | 10.1 m | 602 (17 %) | 2167 | 0.27 |
| EuRoC-tuned (`euroc_v2` ×2) | 0 / 0 | 229 m | 779 (22 %) | 3006 | 0.31 |
| EuRoC-tuned + radiometric gate (`euroc_rad_auto`) | 0 / 0 | 17.0 m | 485 (14 %) | 2031 | 0.31 |

- **Not a scoring artefact.** Distances between the predicted GCPs do not depend on any rigid transform, so a frame
  or extrinsic error in the chain would leave them intact. They match the survey early (ratio 0.88–0.95 between the
  first two GCPs, 15.5 m apart) and then fall apart: 0.27 for the sim-tuned row; 0.5–50 for the EuRoC rows. The
  estimated height spans 14 m (sim-tuned), 32 m and 527 m on one floor.
- **No phase tracks well.** Status stays "tracking" through the first 600 frames, but the sim-tuned row makes 190
  and then 300 keyframes per 300 frames; the lit sim made ~80 in 1000. Landmarks do not survive from frame to frame,
  so the map is rebuilt almost continuously from frame 0. The first GCP pair (frames ~196 → ~688) is already 5–12 %
  short.
- **The 2.2 radiometric gate** refuses on real data: the noise fit gives a < 0 on both cameras, so the VST stays off.
  The row differs from EuRoC-tuned only through that path, and its spread (17 vs 229 m) is one seed and is not
  claimed.

**Diagnosis (measured correlation; mechanism not isolated).** Per 300-frame window, IMU gyro rate (`imu0`, z up),
the mean brightness of the far field (the darkest 30 % of a 200-frame mean image, where the lamp never reaches),
and the two rows' lost frames / keyframes:

| frames | yaw p90 °/s | far-field mean | lost sim / rad | keyframes sim / rad |
|---|---|---|---|---|
| 0–299 | 1.5 | 85 | 0 / 0 | 190 / 173 |
| 300–599 | 3.2 | 80 | 0 / 0 | 300 / 290 |
| 600–899 | 2.8 | 94 | 39 / 38 | 212 / 192 |
| 900–1199 | 3.1 | **44** | **174 / 182** | 243 / 256 |
| 1200–1499 | **0.2** | 52 | 5 / 2 | 104 / 72 |
| 1500–1799 | 3.1 | **50** | **203 / 207** | 241 / 235 |
| 1800–2099 | 3.5 | 71 | 97 / 37 | 299 / 298 |
| 2100–2399 | 3.0 | 81 | 76 / 19 | 208 / 198 |
| 2400–2699 | 2.2 | 57 | 8 / 0 | 220 / 206 |
| 2700–3591 | **0.1–0.2** | 52–54 | 0 / 0 | 34–60 per 300 |

- **The quiet windows are the robot driving without turning** (0.2 °/s). It is not standing still: 62–65 % of LK
  tracks still move there (table below).
- **While it moves (2–7 °/s), losses are heavy only where the far field is dark** (44–50: 174–207 of 300 lost) and
  light where it is lit (70–94: 0–97). Keyframes run at 200–300 per 300 whenever it moves, dark or lit.
- **The camera-locked lamp pattern exists but is NOT the mechanism (measured).** The 200-frame mean keeps the lamp's
  ring pattern on the floor sharp, with its lobe and a dark smudge (likely lens dirt): for a ground robot the floor
  is fixed relative to the camera. The hypothesis that tracks on it vote for zero motion was tested with
  `cv::goodFeaturesToTrack` (500) + pyramidal LK on 30 consecutive raw-frame pairs per window. A track is static if
  it moves < 0.5 px; the pattern mask is the strongest 10 % of the mean image's edges, dilated (31 % of the image):

  | frames | gyro °/s | static % | static on pattern % |
  |---|---|---|---|
  | 300–599 (0 lost) | 6.3 | 4.9 | 2.4 |
  | 900–1199 (174–182 lost) | 5.1 | 9.9 | 7.4 |
  | 1500–1799 (203–207 lost) | 6.8 | 10.0 | 7.2 |
  | 2700–3599 (0 lost, driving straight) | 0.2 | 36–38 | 32–34 |

  In the failing windows ~90 % of tracks move with the scene. Camera-locked tracks are a minority, and they are
  most common where nothing fails. The rejection rests on the static-track share, which does not depend on the
  mask (a 200-frame mean inside one room also keeps lasting room structure). **What this rejects is one mechanism
  only: static ring edges voting for zero motion.** It says nothing about the lamp's camera-locked *falloff*
  (brightness changing with range), which D142 measured in sim as keyframe churn, the same symptom seen here.
- **Stereo, measured and not separating.** The sim lamp rows kept stereo-valid at 0.78, as the fixed light did; on
  Hilti it is 0.27–0.31 from frame 0. Keyframe left→right LK outcomes per image third, from the new `results.yaml`
  field `stereo_match_by_third` (output only; estimate and trace byte-identical):

  | run (sim-tuned config) | valid | forward-backward | off row | LK lost | max disparity, bottom |
  |---|---|---|---|---|---|
  | Hilti, 1200 frames | 29 % | 50 % | 12 % | 2.5 % | 110 px |
  | EuRoC MH_04, 300 frames | 17 % | 40 % | 3 % | 40 % | 270 px |

  Forward-backward rejection dominates on Hilti in all three thirds. But MH_04 (D87's full system reaches 0.046 m
  on it) is lower still with the same config. So the valid fraction alone does not separate a real sequence the
  pipeline handles from one it does not. **The cause of the Hilti failure is open.** Diagnosis stopped here
  (timebox): what remains is a preflighted investigation, not more ad-hoc probes.
- **Smaller, not isolated:** rotation blur ≤ 7 °/s × 30 ms ≈ 1.6 px; 10 Hz (EuRoC 20 Hz); rectified FOV 107.6° of
  ~127° (D143); ISO up to 1549.

**Not done, deliberately.** No parameter was changed after the failed run (design-preflight rule). Candidate
fixes, each to go through preflight (literature, OpenCV-native check, design review) before any code:
- isolate first: what Hilti does to the pipeline that MH_04 does not (frame rate 10 vs 20 Hz, the co-moving lamp's
  falloff under rotation, fisheye-rectified periphery, left/right ISO lag); the candidates are listed, not ranked;
- Stage 3 falloff normalisation with its `feature_mask` (D134/D140): needs depth, i.e. SGBM on this rig;
- a motion-consistency check that drops tracks with zero image motion while the gyro reports rotation.

**What transfers, stated:** extraction, calibration, fisheye rectification (row residual 0.42 px after refinement),
stereo pairing and the Hilti scorer. **What does not:** pose accuracy on a real dark
sequence with a co-moving lamp. The honest 5.6 line is 0/100 until a preflighted fix moves it.

## 2026-10-23 — D145: §31.5 illumination rows defined (preflight: arch, SOTA pass, OpenCV check, design review)

**Preflight.**
- **Arch read:** §5.3 (lamp), §5.6 (D: "scene radiance scale (darkness level)", replay exposure acts on sensing only),
  §6.2–6.5, §19.2, §31.5, and the plan's Phase 5 gate. That gate requires each radiometric stage's effect on
  repeatability, match precision, ATE and loop-closure precision, with negatives kept.
- **SOTA pass:** primary sources opened, list below.
- **OpenCV-native:** not applicable to scene lighting (the renderer's job, §5.6). The radiometric stages were checked
  in D134/D140.
- **Design review:** reviewed twice.

**Literature facts that shape the rows.**
- **No benchmark compares world-fixed lights with a co-moving lamp in the same dark scene.**
  - OIVIO varies only the onboard lamp's flux: ~100 / 50 / 15 % of 9000 lm on repeated trajectories (project page;
    paper body not opened).
  - Hilti 2023 and M2DGR describe their darkness qualitatively, with no lux values.
  - Crocetti et al. ICRA'25 (arXiv 2502.13708) compares a steered robot lamp against a fixed-aim robot lamp, both
    co-moving. Their "up to 75 %" ATE gain comes with their own caveat that the fixed-light ATEs are from partial
    trajectories.
  - The fixed-light row is therefore **[DESIGN]**.
- **No community lux standard for "dark" exists.** OSHA 29 CFR 1926.56 gives working-light minima (tunnels and
  underground ≈ 54 lx, headings ≈ 108 lx). That is real-world context only: Gazebo intensity is not calibrated to lux,
  so no row is set from it.
- **Photometric hazard.** NFL-BA (NeurIPS'25, arXiv 2412.13176) is the evidence that dynamic near-field lighting
  degrades SLAM, but it is a neural photometric-BA method, not sparse VIO. DSO's (arXiv 1607.02565) affine brightness
  term is global per frame and cannot absorb per-pixel 1/r²·cos falloff, so radiometric correction and falloff
  normalisation stay separate rows.
- **Exposure evaluation.** Tomasi et al. RA-L'21 (arXiv 2102.04341): a controller "cannot be evaluated with
  previously-captured data". Published fixes use parallel cameras (Zhang ICRA'17; Shim TCSVT'18, arXiv 1708.07338),
  repeated traversals, or synthetic re-exposure.
  - Verified in our sim image: the sdformat14 camera spec has no exposure and no blur element, and gz-sensors /
    gz-rendering headers have no motion blur.
  - So in sim, exposure acts only through D, live or replay alike. Live vs replay differs only in the lamp path: a true
    re-render (`light_config`) against D's linear `lamp_power` rescale, which is exact at ambient 0 below clipping
    (D132).
  - The comparison measures that difference, not exposure-dependent blur, which neither mode has.
  - No published joint controller for lamp power and exposure was found, so D132's co-controller stays **[DESIGN]**,
    and its §49 deviation already covers the fixed cost order.

**Rows** (VO/SLAM rows for the Phase 5 gate; the two reconstruction rows are plan 6.1, defined here, run in Phase 6):

| §31.5 row | definition | held fixed vs the lamp row |
|---|---|---|
| ambient normal | lit render (`ci_tiny`, D69), no D | geometry, trajectory, seed |
| dark | lit render × D darkness level, `input_transfer: srgb` (§5.6); levels = 5.4's frozen dark0..3 | the same |
| dark + fixed light **[DESIGN]** | `ci_tiny_dark_fixed`: ambient 0, the world's zone lights, no lamp. **Brightness-matched** to the lamp row through D's `radiance_scale`: trajectory-median linear brightness equal, exact only if neither render clips | geometry, trajectory, seed, median brightness |
| dark + co-moving light | `ci_tiny_dark` (rig v2 lamp, D131/D133); lamp sweep 100 / 50 / 15 % through replay `lamp_power` (OIVIO's protocol), valid where the render does not clip | — |
| + radiometric correction | 2.2 stages 1–2 (+ Stage 4 when exposure varies) | pixels |
| + falloff normalisation | Stage 3 albedo proxy + `feature_mask` (D134/D140) | pixels |
| + retrieval-path enhancement only | §6.3 split: enhancement on the retrieval path, not geometry | pixels |
| fixed vs active exposure | D exposure fixed vs D132 controller; live-vs-replay quantifies the lamp path only (above) | trajectory |
| NFL-aware / albedo-proxy reconstruction | plan 6.1 | — |

**Residual confounds, stated, not removed:**
- **Coverage.** Zone lights fill the rooms and the lamp lights a cone. Reported as the fraction of pixels above the
  noise floor in each row.
- **Clipping.** A hotspot clipped in the render cannot be rescaled. Reported per row as the clipped fraction.

**Consequence.** D142's rows stay preliminary. The matched-brightness `radiance_scale` comes from a measurement of
both recordings: sRGB-decoded median brightness, clipped fraction and above-noise fraction, over ~20 chunks of each
`sensors_0.mcap`, read by ranged GET. That needs one approved S3 read of a few hundred MB.

**Citations:** OIVIO project page (arpg.colorado.edu/oivio); Nair et al. RA-L 2024 (arXiv 2404.09765); Yin et al.
M2DGR (arXiv 2112.13659); Ebadi et al. (arXiv 2208.01787); Crocetti et al. ICRA 2025 (arXiv 2502.13708); Dunn Beltran
et al. NFL-BA NeurIPS 2025 (arXiv 2412.13176); Engel et al. DSO (arXiv 1607.02565); Zhang, Forster, Scaramuzza ICRA
2017; Shim et al. TCSVT 2018 (arXiv 1708.07338); Tomasi et al. RA-L 2021 (arXiv 2102.04341); 29 CFR 1926.56 (eCFR).

**D145 measurement (2026-10-23, approved S3 read).** ~55 MB of ranged reads in total: each `sensors_0.mcap`'s summary
plus 30 chunks evenly spaced over the first 50 s of agent 0 (the replay span), left camera. sRGB-decoded (D130),
BT.601 luminance. "Lit" = at least one 8-bit output step: the noise floor of `lampdark_sensor_v0` (10⁴ e⁻ full well,
2.5 e⁻ read) lies well below it.

| render | median linear Y (p10–p90) | clipped pixels mean / max | lit pixels |
|---|---|---|---|
| `ci_tiny_dark_0` (lamp) | 0.0364 (0.0117–0.0808) | 3.4 % / 16.2 % | 69 % |
| `ci_tiny_dark_fixed_0` (zone lights) | 0.0450 (0.0209–0.0743) | 0 % / 0 % | 95 % |

- **Matched row:** `configs/degradation/lampdark_fixed_matched_v0.yaml` = `lampdark_sensor_v0` with
  `radiance_scale` 0.81. Exact, because the fixed render has ambient 0 and never clips.
- **Brightness was not the big gap.** D142's unmatched fixed-light row was only 1.24× brighter.
- **Coverage is the residual confound: 69 % vs 95 % lit.** The matched row cannot remove it.
- **The lamp render clips** on 3.4 % of pixels on average, 16 % at worst. The 100/50/15 % lamp sweep through
  `lamp_power` cannot recover clipped hotspot pixels. Those rows carry the clipped fraction, and a live re-render
  (`light_config`) is the reference for them.

## 2026-10-23 — D146: 5.2 wiring: Stage 3 runs at keyframes; per-frame is an ablation row (choice (c))

**Where depth comes from (the open question).** Stage 3 divides by the predicted lamp irradiance,
which needs a depth map. The runner computes SGBM only at keyframes. Option (c) was chosen, 2026-10-23: Stage 3
runs at keyframes now, and an every-frame SGBM row comes later as an ablation. Grounds:
- **Arch §19.1 / §18.2:** the load-bearing consumer is the keyframe bundle (`rgb // falloff-normalized albedo-proxy`).
  §6.2 lets features use the proxy only "when the ablation says it helps".
- **DarkGS** (Zhang et al., arXiv 2403.10814) builds its calibrated-light model into training-view rendering, which
  is per view used for training, not per tracked frame.
- **The per-frame cost is not measured.** The "20–40 ms" estimate given was `[ASSUMPTION]`. The keyframe
  path now times Stage 3 (`falloff_ms`), and SGBM is already timed inside `t_kf`.

**Inputs, and what Stage 3 refuses (no silent wrong division):**
- **Linear pixels.** The sim render is sRGB-encoded (D130). Only D with `input_transfer: srgb` makes the frame linear.
  The sim source attaches a per-frame photometric state (`StereoFrame::photo`, the §4.1 slot) only then. EuRoC,
  Hilti and an undegraded sim run have none, and Stage 3 refuses them.
- **Flat field first** (arch §6.2 order: flat field → VST → falloff). D's vignette would otherwise be read as albedo.
  Stage 3 refuses `radiometric: off`.
- **Stage 4:** gray / 255 / (exposure · analog_gain · digital_gain), D's values for that frame.
- **Lamp power:** lamp_rel = (rig lamp intensity / calibration `intensity_ref`) · D `radiance_scale` · D `lamp_power`.
- **A lamp-only scene:** scene `ambient` 0 and no world lights. The fixed-light and matched rows (D145) are refused.
- **The calibration's rig:** the calibration's `rig` must equal the mission's rig, and the lamp position must agree
  within 1 mm (the rig's lamp in body, through T_B_C0, against `lamp_in_camera`).
- **Geometry:** depth is SGBM Z where `DenseSupportMask` holds, NaN elsewhere (no unsupported disparities). K is the
  rectified K. `R_cam_from_raw` is the rectifier's R1.

**Output, first commit:** `KeyframeBundle` gains `albedo`, `albedo_valid` and `albedo_feature_mask`. The spool (6.1)
does not write them yet. `exposure_state` and `lamp_state` stay unset: D's exposure is a relative factor, not the
seconds `ExposureState` holds, and 5.3's controller is what will own both. The manifest gets `falloff` under
Additional, and the tool takes `--lamp-calib` (the calibration describes the recording's rig, like `--degradation`).
Tracking is untouched, so a run with Stage 3 on must give the same trajectory as with it off; a test checks this.
results.yaml gets a `falloff:` block only when the stage is on: keyframes normalised, median valid fraction, median
feature-mask fraction, `falloff_ms` p50/p95.

**Second commit, the features row.** It needs the `replenish: keyframe` cadence: detection then happens only at
keyframes, where the proxy exists. The row's baseline is the same cadence with Stage 3 off, so the row measures
Stage 3 and not the cadence change. The tracker gains `Replenish(gray, detect, detect_mask)`: LK keeps the raw
pyramid, and detection uses the proxy with `feature_mask`. The proxy maps to the engines' 8 bits as reflectance:
`255 · albedo`, clipped at 1. `FalloffNormalize` already returns reflectance in the calibration's units (the plane
reads its known 0.5), so no scale is chosen. The clipped fraction goes into results. (A first draft of this entry
read the proxy as "relative, 1 = the plane", following a stale comment in falloff.hpp; the glue test's exact 0.5
error exposed it, and the comment is corrected.)

**D146 checks after the first commit (8fab27d), before any AWS run:**
- **No double exposure division.** The radiometric stage's own exposure normalisation (its step [4]) is inert: "no
  source carries photometric state" (`radiometric_stage.hpp`). Stage 4 therefore happens once, in the falloff glue.
  When 5.3 gives sources a photometric state, step [4] and the glue must not both divide. 5.3 decides where it lives.
- **The 1 mm lamp check against the real rig, by hand.** `rig_stereo_v2`: lamp in body [0.05, 0, 0.04], left camera
  in body [0.03, 0.05, 0.02], body x forward / y left / z up, no camera tilt. Lamp minus camera = [0.02, −0.05, 0.02].
  In the optical axes (x = −y, y = −z, z = x) that is [0.05, −0.02, 0.02], exactly `lamp_calib_v2`'s
  `lamp_in_camera_optical_m`. The unit test builds its lamp from the calibration, so it could not show this. A
  recording's own `/tf_static` is checked on the first AWS run (the refusal fires before frame 0).
- **`ariadne_sim_collab`** has neither `--degradation` nor `--lamp-calib`. Its frames carry no photometric state, so a
  falloff config there would refuse. Wiring it is part of 6.1 (the spool writing the proxy).
- **§6.3, plan 5.2's second clause.** Enhancement exists only on the retrieval path: CLAHE is called in
  `perception/place/place_recognizer.cpp` (`enhance: none|clahe`) and nowhere else. The geometry path (LK, stereo,
  SGBM, descriptors) receives only flat field, VST denoise and, now, the falloff proxy. It is enforced by structure,
  not by a test. The retrieval-enhancement ablation row is `enhance` (§31.5).

**Second commit, as built.**
- `falloff: features: raw | proxy` (default raw). `proxy` is refused unless `mode` makes a proxy and the tracker runs
  `replenish: keyframe`.
- `KltTracker::Replenish(gray, detect, detect_mask)`: detection reads `detect`, inside the live-track discs AND the
  proxy's `feature_mask`. LK keeps `gray`'s pyramid, and the §7.2 eigenvalue quality check still reads `gray`
  (it is about LK's conditioning on the image it tracks). The one-argument form delegates with `detect = gray` and
  no mask, so it is unchanged.
- results gain `features: proxy, clipped_fraction_p50/max` (reflectance > 1 inside the feature mask) on that row only.
- Re-association descriptors (2.1) stay on the raw gray. Describing on the proxy would be a further row, not built.
- Tests: TrackerCadence.ReplenishDetectsOnTheDetectionImageInsideItsMask. A flat detection image adds nothing; a
  top-half mask keeps every new track in the top half; size and type mismatches throw. FalloffGlue: the detection
  image of the 0.2/0.6 checker has mean 255·0.4, and the cadence and mode guards hold.

## 2026-10-23 — D147: 5.3 wiring (replay): the controller at keyframes, the applied state on every frame, known-gain LK

Preflight: arch §6.5 and §6.2 Stage 4, D132, the runner and sources as built, a literature check, and design review
(2026-10-23). Replay only. The live lamp path (gz `light_config`) waits on an ownership decision.

**The loop.** At each keyframe the runner calls D132's `Decide` on the frame it just tracked (arch §6.5: keyframe
rate; slew ≤ 0.5 stop per decision). The command goes to the source: `StereoSource::SetPhotometric` (default no-op,
`RadiometricSource` forwards it, `SimSource` applies it to D's exposure, analog gain and lamp_power from the next
frame on).
- **Downstream code reads only the applied state** (`frame.photo`, what D used on that frame), never the command:
  Stage 3, the LK ratio, the traces. Latency then cannot desynchronise anything. A test checks that the command reaches
  `SimSource` through the wrapper (the `Lamp()` lesson, D146).
- **Determinism:** D's noise is keyed by (seed, frame, pixel, stage), and the decisions are a function of the pixels,
  so a replay reproduces the whole sequence.

**`radiometric: flat_only` with the controller; `auto` and `on` are refused.**
- The VST noise model is fitted once, and analog gain changes it (Poisson term ∝ g, read term ∝ g²).
- `flat_only` has no 8-frame gate hold. In `auto`/`on`, frames are degraded before any decision could reach them.
- Stage 3 needs only the flat field, and D69 measured the VST neutral.
- Rescaling the fitted model by the known gain is a possible later row, not built.

**Stage 4 has one owner per consumer.**
- **LK (the cross-frame photometric comparison, arch §6.2):** when the applied brightness changes between frames,
  the tracker rebuilds its previous pyramid from its level-0 image × r, where r = (exposure·gain·lamp_scale)_now /
  (…)_prev. In replay D's lamp_power scales all radiance, so r is exact up to 8-bit rounding and clipping. The
  pyramid holds Scharr derivatives (`withDerivatives`), so the image is rebuilt, not scaled in place. This is the
  known-gain case of gain-robust KLT: Kim, Frahm & Pollefeys (ICCV 2007) and Bergmann, Wang & Cremers (RA-L 2018,
  arXiv 1710.02081) estimate the inter-frame gain ratio inside KLT, because plain KLT fails under exposure changes.
  We know the ratio and apply it. OpenCV's `calcOpticalFlowPyrLK` has no gain term. The ratio is at most √2 per
  decision.
- **Stage 3:** the falloff glue divides by `frame.photo` (D146), unchanged.
- **The radiometric stage's step [4]** stays inert and is marked superseded. Nothing divides twice.
- **Stereo and SGBM:** both cameras get the same D parameters, so the pair needs nothing.
- **ORB re-association descriptors:** intensity comparisons, invariant to a gain. Nothing to do.

**Units: what matters is headroom.**
- D's exposure is relative; the controller works in seconds. D adds no motion blur (stated approximation), so the
  blur ceiling only clamps. ω = 0 unless the run has an IMU (`--imu-noise`).
- `t_max` = the frame period (`rig_stereo_v2`: 20 Hz → 50 ms).
- `t_ref`, the seconds that D exposure 1 stands for, is `[ASSUMPTION]` 12.5 ms: 2 stops of headroom.
- **Derivation, from D145:**
  - The lamp render's median is linear Y 0.0364, and frames near walls already clip (3.4 % mean, 16 % max).
  - The clean render's peak, about 150/255 sRGB (linear 0.30), has about 1.7 stops before clipping.
  - So the controller's 2 % clip guard binds before a 2-stop ceiling on every measured frame type. A larger headroom
    would change nothing measured; a smaller one would be the ceiling, not the scene, deciding.
- `t_min` stays D132's 0.5 ms.

**Overexposure event (Failure E), sensor side.**
- **The scripted event:** a SCENE radiance step before D's sensor, e.g. +2 stops at a set time:
  `ariadne_sim_vo --radiance-event AT_S:STOPS`, recorded in the manifest as `scenario`. It is not reported in
  `FramePhoto`: the system does not know the scene changed, so the controller has to see it in the pixels.
  (Corrected while building it: this entry first said "a D exposure step", but exposure is the controller's own
  actuator, and its next absolute command would overwrite the step.)
- **What the response must show:** the controller pulls back at ≤ 0.5 stop per keyframe, and the trace shows it.
- **Why not the lamp:** in replay `lamp_power` cannot recover pixels clipped before the render (up to 16 %, D145), so a
  lamp-side event would look like a controller failure. That needs live `light_config`.

**Traces.**
- `exposure_trace.csv`, one row per keyframe: metric, saturated and shadow fractions, reason, step, command, and the
  applied state one frame later.
- The bundle's `exposure_state` (the applied t in seconds via `t_ref`, and gain) and `lamp_state` are now set when the
  controller runs. 5.3 owns them.
- This is perception evidence (Failure E), not the agentic runtime trace (arch §0.2 invariant 3).

**D132 deviation approved (2026-10-23).** The fixed order stays: lamp → exposure → gain, which is the
limiting case of an energy cost below gain's noise cost. Arch §6.5 gives the lamp an energy-cost term; that text now
records an approved deviation (§49). The trade-off is energy, which replay cannot measure anyway.

**The actuator seam: the contract a live sink implements** (split set on 2026-10-23: the photometric side owns the
controller and this interface; the live runtime owns the live sink: gz `light_config` through the ros_gz/gz
service wrapper). No ROS or gz type goes below runtime (arch §3). The seam is
`StereoSource::SetPhotometric(const PhotometricCommand&)`; the report comes back on the frames.
- **Command** (`runtime::PhotometricCommand`): `decision_id` (increasing), `decided_ns` (the stamp of the frame it
  was decided on), and three targets:
  - `exposure_rel`: relative to the rig's nominal exposure (1 = nominal; seconds = `exposure_rel · t_ref_s`);
  - `gain`: linear analog gain (1 = unity);
  - `lamp_rel`: relative to the lamp calibration's `intensity_ref`. Live, gz `<intensity>` = `lamp_rel ·
    intensity_ref`; in replay it is D's `lamp_power`.
- **Report, on every frame** (`StereoFrame::photo`, `runtime::FramePhoto`): the state that frame was actually exposed
  with, i.e. `exposure_rel`, `gain`, `lamp_rel` as applied. Plus:
  - `applied_ns`: the sim-time stamp at which the newest command reached the actuator (the live runtime's requirement; mover answers
    already carry one);
  - `decision_id`: the last command the sink has applied, in whole or in part;
  - `clamped` and `clamp_reason`: true when the sink applied something other than the command (actuator limits,
    a refused service call).
  - **Latency** = `applied_ns − decided_ns`, as logged in the trace.
- **The controller reads only the report.** A late command, a clamped one or a dropped one changes nothing
  downstream except through the frames' applied values. That is how replay works too.
- **Per actuator, live:** the sim camera has no exposure or gain element (sdformat14, D145). So live exposure and gain
  are applied by D at consumption, same as replay, immediately. Only the lamp goes through `light_config`, so only
  the lamp has real latency and can be refused.
- **Ordering:** a sink applies commands in `decision_id` order. A newer command supersedes an older one that is
  still pending: the older is dropped, not applied late.

**As built (5.3 replay code).**
- **Structs** in `runtime/datasets/stereo_source.hpp`:
  - `PhotometricCommand` (the command);
  - `FramePhoto` (the report: `exposure_rel`, `gain`, `digital_gain`, `lamp_rel`, `radiance_scale`, `decision_id`,
    `applied_ns`, `clamped`, `clamp_reason`; `Brightness()` is the LK ratio's input);
  - `StereoSource::CanActuatePhotometric` / `SetPhotometric`;
  - replay helpers `PhotoOf` / `ApplyPhotometric`.
- **Glue:** `runtime/tools/exposure_glue.hpp`.
- **Config:** `exposure_control:` in `perception/exposure_control/exposure_config.hpp`. The params moved to
  `exposure_params.hpp`, OpenCV-free, so test_unit can parse them.
- **Known-gain LK measured** (TrackerCadence.RescalePreviousAppliesTheKnownGain). The scene is shifted (2.3, 1.6) px
  and made √2 brighter, half a stop, the controller's largest step:

  | previous pyramid | tracks | median error |
  |---|---|---|
  | rebuilt × √2 | 327 | 0.015 px |
  | not rescaled | 290 | 0.263 px |

- **Live-side notes (ack of the contract, 2026-10-23):**
  - In lockstep the world pauses at 5 s barriers. A `light_config` call near a barrier is applied at the first step
    after resume, and `applied_ns` is that sim time. Latency then quantises to the step.
  - A refused call reports `clamped = true`, `clamp_reason = "service refused"`, and the PREVIOUS `lamp_rel` as
    applied.
  - Both are consistent with "the report says what the frame was actually lit with".
- **`t_max_s` is required** (design review of fe6194c). D132's 30 ms default would silently give 1.3 stops of
  headroom instead of 2. The first decision refuses a `t_max_s` longer than the measured frame interval.
- **What the replay rows can and cannot show.**
  - **The lamp does not move in replay.** Frames start at `lamp_rel` 1 = `lamp_max`, so brightening goes to exposure.
    Darkening spends gain, then exposure, and reaches the lamp only below `t_min`, about 4.6 stops under `t_ref`.
  - So the replay row is exposure (and gain) control, not lamp co-control, and a flat `lamp_state` is expected. Evidence
    for the lamp half comes from the live sink.
- **Known bias, `[UNVERIFIED]` size.** The controller reads the keyframe AFTER Stage 1's flat field. Where K < 1
  (D's vignette 0.28 toward the edges), a raw value above 255·K becomes 255 and counts as clipped, although the sensor
  did not clip.
  - **Expected size:** small under the lamp (D145: median linear Y 0.036; the hotspots are central, where K ≈ 1),
    larger in bright, evenly lit scenes.
  - **Not measured yet.** The AWS rows log `saturated_frac` per decision. If it diverges from the D-output clip count,
    the guard moves before Stage 1.
- **Checks for the first AWS run** (the SimSource pending → next-pair path is covered locally only through
  `ApplyPhotometric` and the wrapper): in `exposure_trace.csv`, every row's `applied_decision` equals the previous row's
  `decision`, and `applied_ns` is the stamp of the frame right after `decided_ns`.

### D145–D147 Results (AWS, 2026-10-23, commit `649f156`, run 20261023T113916Z-cpux86-7b3f1b, ~$0.06)

Evidence: `evaluation/runs/perception/radiometric/d145_rows_20261023T113916Z/`. Agent 0, 1000 frames, seed 1, `sim_v2_vo`, all 10 rows OK.

| Row | What | ATE SE(3) | KF | lost |
|---|---|---|---|---|
| r01 | flat-field only (sim_v1_rad_flat), sensor D | 14.8 mm | 182 | 0 |
| r02 | + Stage 3 at keyframes (D146) | 14.8 mm | 182 | 0 |
| r03 | keyframe-cadence baseline | 14.6 mm | 179 | 0 |
| r04 | + features on the proxy | 17.5 mm | 152 | 0 |
| r05 | active exposure (replay controller) | 14.9 mm | 172 | 0 |
| r06 | Failure E (+2 stops at 20 s), controller on | 16.1 mm | 169 | 0 |
| r07 | Failure E, fixed exposure | 19.1 mm | 249 | 0 |
| r08 | lamp at 50 % | 150.6 mm | 243 | 22 |
| r09 | lamp at 15 % | 1317 mm | 749 | 223 |
| r10 | fixed light, brightness-matched | 4.2 mm | 71 | 0 |

- **D146 equality holds.** r02's `estimate_body.tum` is byte-identical to r01's (sha256 `4e4bf88c…`). The proxy runs at
  every keyframe (182) and never touches tracking. Median proxy valid fraction 0.47; feature-mask fraction 0.22.
- **Features on the proxy (r04 vs r03): negative.** ATE +2.9 mm and 27 fewer keyframes. Detecting on the
  reflectance image does not help this sequence. Kept as a negative row; the default stays `features: raw`.
- **Active exposure (r05 vs r01): neutral** without an event (14.9 vs 14.8 mm). 172 decisions: 148 metric, 24
  overexposed, saturation ≤ 4.7 %, steps ≤ 0.5 stop. Known-gain LK rescaled the previous pyramid 69 times.
- **D147's pre-stated trace checks pass exactly.** `applied_decision` equals the previous decision in 171/171 (r05)
  and 168/168 (r06). `applied_ns` is the stamp of the frame after `decided_ns` in all of them. Latency is one frame
  (50 ms at 20 Hz).
- **Failure E (the overexposure failure-and-response scenario, plan 5.3).**
  - With the controller, it reacts at the first keyframe after the event (t = 20.0 s, saturation 7.4 %) and steps
    −0.5 stop per keyframe: exposure 1 → 0.71 → 0.5 → 0.35 → 0.25 → 0.18. Saturation is 0 by t = 22.35 s, and it
    then backs off +0.375 stop.
  - ATE 16.1 mm with the controller against 19.1 mm with fixed exposure, and 169 vs 249 keyframes (the fixed run
    keyframes through the washed-out stretch).
- **Lamp power (r08/r09): steep, and reported as a finding.** At 50 % the lamp costs 10× the ATE and 22 lost frames;
  at 15 % tracking collapses (1.3 m, 223 lost, 749 keyframes). This is the OIVIO-style lamp sweep of D145.
- **Fixed light, brightness-matched (r10): 4.2 mm, 71 KF.** That is 3.5× better than the carried lamp at matched
  brightness, so a co-moving light costs accuracy on its own, independent of mean brightness. D145 asked exactly
  this; the coverage difference (the fixed lights reach the far field) is the likely cause [UNVERIFIED: not
  isolated].
- **Not in these rows** (gate bullet 1 asks for them): repeatability, match precision and loop-closure precision per
  radiometric stage, and the dust rows. They are recorded as open in D149.

## 2026-10-23 — D148: 5.5 measurement plan, fixed before the confined_v1 render is read — corridor gate ordering and the baseline sweep

Written while the render was still recording (run 20261023T105356Z-gpu-155842, commit `8af6c71`); nothing below has
seen its pixels.

**Recording.** `confined_v1` seed 0 + `rig_stereo_sweep_v1`, lit (ambient 0.35 + zone lights: the sweep isolates
geometry, D138), both agents, one g4dn spot pass, uploaded to `runs/ph5-confined/<commit>/confined_v1_0`.

**A. Corridor degeneracy gate ordering** (Phase 5 gate bullet 4; arch §14.4: degeneracy gate → PCM → GNC).
- **Replay.** `ariadne_sim_vo`, agents 0 and 1, full mission, seed 1, `sim_v1` perception, `sim_v2_loops` slam
  (sim_v2 + euroc_v2_loops' loop block verbatim; `sim_v2` itself has loop closure off, so it makes no attempts to
  suppress), `--imu-noise imu_noise_v1`, main pair (B = 10 cm), no D. Gate thresholds as shipped (D71/D73) [ASSUMPTION
  on the sim rig]; none is changed after looking.
- **Labels, from GT only.** Clearance d = the scene ESDF (`esdf_f32.bin`, 5 cm cells, metres to the nearest
  obstacle) at each keyframe's GT xy. Narrow: d ≤ 0.30 m (the 0.45–0.60 m passages). Corridor: 0.30 < d ≤ 0.50 m (the
  0.90–0.95 m passages). Open: d > 0.50 m. A candidate is *confined* if its query or its match keyframe is narrow or
  corridor.
- **Hard criterion (ordering).** Every candidate with `degenerate = true` has `pcm = false` and `gnc_inlier = false`:
  it never reaches PCM. 0 violations, counted over all candidates of both agents.
- **Firing.** The gate fires on a candidate when `degenerate = true` for any reason code (`too_few_points`,
  `conditioning_translation`, `conditioning_rotation`, `parallax`, `collinear`). Reported per label: candidates,
  refusals, the reason histogram, the D71 `corridor` score (reported, not gated), and the GT label of refused and
  accepted candidates (good/bad at 0.2 m / 2°, D73).
- **The bullet is met** when the ordering holds, at least one confined candidate is refused, and no GT-bad confined
  candidate ends as a GNC inlier. If GT-bad confined candidates pass the gate and PCM/GNC remove them, the report says
  so: the later stages did that work, not the detector. If retrieval yields no confined candidate, the bullet is
  recorded "not exercised". Neither thresholds nor the scene change after looking.

**B. Baseline sweep** (5.5, §9.1 "evaluate 4/8/12/20 cm against range/depth error", §31.8 "subject to the §9.4
validator").
- **Rows.** B = 4, 8, 12, 20 cm (`--right-camera right_b04|b08|b12|b20`, 0bfc95b) and the main pair 10 cm. Agents 0
  and 1, full mission, seed 1, `sim_v1` perception except `stereo.num_disparities`, `sim_v2` slam with
  `imu_noise_v1`, no D. Same frames, poses and lighting in every row (one render).
- **Disparity range per baseline.** The smallest multiple of 16 that passes the §9.4 validator at
  `required_min_range_m` 0.30, with rectified f = 671.54 px. That gives B04 96 (Z_min 0.283 m), B08 192 (0.281 m),
  B12 272 (0.297 m) and B20 464 (0.290 m). The main pair keeps its shipped 240 (0.281 m; the rule would give 224), so its
  perception config equals every earlier sim row. Its slam config does not: the sweep runs `sim_v2` (full VIO), while
  the D142/D145 rows run `sim_v2_vo` (visual-only). Columns without a full search range: 7.5 / 15 / 18.75 / 21 / 36 %. SGBM time scales
  with the range and is reported from the frontend timings.
- **Metrics.**
  - ATE (SE(3) alignment), keyframes, lost frames, tracks.
  - Depth: each keyframe's SGBM Z against the recording's GT depth camera (co-located with the left camera) in GT
    range bins 0.3–1 / 1–2 / 2–4 / 4–8 m. Reported as the median and p90 |Z − Z_gt| / Z_gt over SGBM-valid pixels,
    plus the valid fraction per bin, every 2nd pixel.
  - **Amended 2026-10-23, before the render was read** (the render was still recording). The first wording was
    point-to-GT-mesh distance of GT-pose-placed points. The per-pixel GT depth measures the same along-ray range error
    exactly, needs no mesh sampling, and needs no multi-GB spool upload per row. GT depth goes on the raw grid
    (ApplyLensDepth), is rectified nearest-neighbour, and Z_rect = Z_raw / (R1ᵀ K_rect⁻¹ [u v 1]ᵀ).z. Built as the
    runner's evaluation-only `depth_eval` block (`ariadne_sim_vo --depth-eval 1`, `StereoFrame::gt_depth`, never
    read by perception), with a unit test, before the sweep is read.
- **Reported as measured.** One seed chooses no baseline. The lamp-asymmetry confound of D138 does not arise here: the
  render is lit.

**Cost.** Render g4dn.xlarge spot $0.087/h, capped at 150 min. The replays are CPU, and their placement (local or
c7i spot) is asked separately.

## 2026-10-23 — D149: Phase 5 exit gate — NOT closed; each bullet against its evidence

The format follows D172. Evidence is as of tip `93a21a9`. A 2026-10-23 ruling made
dark, lamp-lit and dusty runs STRETCH goals, taken up after the live demo runs in real time (D199). Bullets that
need them are marked so: they are deferred by that ruling, not dropped.

| Gate bullet | Status | Evidence |
|---|---|---|
| DVE ablation rows (§31.5): each radiometric stage's effect on repeatability, match precision, ATE, loop-closure precision; negatives kept | **partial** | ATE / keyframes / lost frames for the 10 §31.5 rows (D145–D147 Results, AWS `649f156`), with negatives kept (features-on-proxy +2.9 mm; lamp 15 % 1.32 m). D142's preliminary lighting rows. **Not measured:** repeatability, match precision and loop-closure precision per stage; the dust rows (5.4: stretch) |
| Live-vs-replay exposure-control comparison (§5.6 approximation) | **blocked: stretch** | Replay side done: D147 contract, trace checks 171/171 and 168/168, Failure E recovered in 2.35 s. The live side needs the 5.3 live lamp sink, and D145 limits the comparison to the lamp path (sdformat14 has no exposure element). The lamp path is a dark-world item, so stretch under D199 |
| OIVIO (→ Hilti 2023 site2_robot_3, D141): tracking survives lamp-lit sequences with reported ATE; failure segments documented | **measured, failing** | D144: all 3 rows score 0/100 (GCP rms 10–229 m; 14–22 % of frames lost). The failure segments are documented as failure case FC-5.6-1: losses while turning with a dark far field. The sim-tuned settings do not transfer. Kept as the negative result §34.2 asks for |
| Corridor degeneracy detector fires on degenerate scenes and suppresses loop attempts (gate ordering) | **blocked: recording** | Pre-registered in D148 (criteria fixed before data). The sweep render failed twice over: the TF bug (fixed `82ffa00`) and the writer backlog (one-pass not recordable). The user chose a 2-camera confined_v1 render (approved; g4dn spot had no capacity in any eu-north-1 AZ at 14:47Z, retrying), then the approved CPU replay (`ph5_aws.sh ROWS=d148`) |

**Plan items.**
- 5.1: dark worlds, lamp, calibration. Done (D131, D133, D137).
- 5.2: Stage 3 at keyframes. Done (D146; trajectory equality measured).
- 5.3: replay co-control and the Failure E scenario. Done (D147 Results). The live `light_config` sink belongs to the live runtime.
- 5.4: dust configs frozen. Open, stretch (D199).
- 5.5: confined worlds done (D138). The baseline sweep is not recordable one-pass on g4dn (D138 correction), and its
  re-design is open. The SimSource right-camera selection, the sweep configs and depth_eval are built and tested.
- 5.6: measured, failing (D144).

**Spend for Phase 5 on 2026-10-23.**
- CPU $0.1156: four row launches, one of them successful.
- GPU $0.0881: the failed sweep render.
- S3: 15.8 GB of the failed render deleted on approval.
- Earlier sessions: GPU $0.1241, CPU ~$0.07.

**Closes when:** the corridor bullet is measured against D148, and the per-stage repeatability, match-precision and
loop-closure rows are run (a CPU row set, still to be defined after preflight). The stretch bullets are re-opened by
decision once the live demo runs in real time.

## 2026-10-23 — D170: 3.4 spectral prioritisation — full-graph Fiedler space, greedy + 1-swap, a valid shared-cost bound, broker selection behind a hook

Design: `docs/design/spectral-selection.md`. Code:
- `cpp/collaboration/candidate_selection/{selection,spectral_selector,response_cost}.hpp`;
- `CollabNode::MaybeSelect`.

Plan item 3.4.

**Decision.**
- **Objective:** λ₂ of the broker's candidate graph, built Swarm-SLAM's way (verified from the `cslam` source). Nodes
  are keyframe ids 0..max per agent, joined by odometry at κ = 1. Accepted inter-agent edges are fixed at κ = 1;
  candidates take κ = retrieval similarity. No intra-agent loops, because snapshots carry none to the solve.
- **Solver: §49 substitution.** Arch §16 says LOBPCG or shifted Lanczos. This uses block inverse iteration on L⁺
  (grounded SimplicialLDLT), with Rayleigh–Ritz and warm starts. A near-repeated λ₂ is treated as one cluster (for
  determinism).
- **Selection:**
  - a connectivity bootstrap, re-checked after every pick;
  - greedy by κ‖Vᵀ(e_a − e_b)‖² per marginal byte;
  - then a bounded 1-swap refinement (arch §16.2's "local refinement").
- **Bound:** MAC's post-hoc dual bound over an LP relaxation of the **shared-cost** feasible set (`cv::solveLP`). It
  is reported per run and absent if the LP fails.
- **Cost:**
  - an exact payload via the codec, from the appended `GlobalDescriptor.n_features` / `feature_bytes`;
  - the calibrated per-encoding compression ratio of collaboration.md §7, from disjoint runs (uncalibrated runs are
    labelled and excluded from I6);
  - the codec's own framing.
- **Budget:** a token bucket of predicted bytes over a 5 s selection period (Swarm-SLAM's period; its budget is a
  count, ours bytes per arch §16.3).
- **Policy:** `NodeHooks::select`. Unset gives today's request-everything behaviour (the "similarity" baseline);
  `SelectBySimilarity` is "greedy"; `SelectSpectral` is "spectral". A policy requires `NodeHooks::similarity`.

**Evidence (unit, synthetic):**
- selector 6/6: the solver matches dense eigen cold and warm, and a repeated λ₂ is detected;
- exact oracle under the shared cost, 60 instances:
  - bound valid 60/60, but loose, 1.3–6.4× the optimum;
  - selected/optimum mean 0.970, worst 0.650 (greedy alone: 0.936 / 0.533);
- protocol 60/60, including:
  - **v1 `GlobalDescriptor` bytes identical to `5bc95a9`**: a golden from the pre-change tree, 1142 B;
  - on a scene disjoint from the calibration: response prediction error 7.2%, request 0.29% (gate ≤ 15%);
  - the calibrated bucket is spent, not idle: 269 KB sent of 380 KB allowed.
- The synthetic descriptors compress to 0.21; real ORB is nearly incompressible, so **the I6 evidence is the sim
  runs**, not this.

**Stated gaps:**
- The vertex-cover direction choice (broker-side verification fetches every remote endpoint).
- The 0..max chain assumes odometry across D87's prefix.
- `candidate_ttl_s` 120 is an [ASSUMPTION] and `refine_passes` 2 is unmeasured at scale. Both are decided by the sim
  runs, with stats `max_pending` and `select_ms_*`.
- The D-optimal ablation is deferred (design §8).
- The real gate harness must sum predictions over every node that acted as broker.

**§49:**

| step | status |
|---|---|
| catalogue | `algorithms.md` §1.4 M2/M3 (Swarm-SLAM, MAC, Giamou/Tian); D170 adds the as-built note |
| arch decision | this entry; arch §16 blockquote pending with the merge (the solver substitution, the refinement, the bound) |
| ablation matrix | policy {similarity, greedy, spectral}; `refine_passes`; D-optimal (deferred) |
| benchmark config | the budget sweep (`budget_bytes_per_s`) for the ATE-vs-bytes plot: with the 3.2 sim integration |
| manifest | policy, `budget_bytes_per_s` / `budget_burst_bytes`, `selection_interval_s`, `refine_passes`, response ratio + calibration run; emitted by the runtime once wired |

## 2026-10-23 — D171: 3.2 sim integration — the v2 protocol over CommsNode on sim recordings (design collaboration.md#simulated-collaboration)

**Preflight:**
- **Literature.** COSMO-Bench (arXiv 2508.16731) for the method: the front-end runs over recorded data, and a
  distance-driven comms model fitted to real Wi-Fi gates the exchange. Swarm-SLAM for the front-end/back-end split
  and the arm comparison. Tian et al. RSS 2018 for budgeted exchange and the greedy-coverage row. RFComms (D102) for
  per-size PER.
- **OpenCV-native check:** cv::dnn EigenPlaces; cv::ALIKED + cv::LightGlueMatcher; findEssentialMat; solvePnPRansac.
- **Design review:** review 1, folded into the design.

**Decisions:**
1. **Two passes.**
   - Pass A: each agent's shipped front-end (the D69 configuration), cached per (scene, seed).
   - Pass B: a discrete-event run on the recording clock, CollabNode × CommsNode.
   - A keyframe is released whole at `t_final`, when it leaves the BA window, so nothing reaches pass B before the
     agent would know it. A test checks there is no lookahead.
   - Broker compute takes zero sim time [ASSUMPTION]; the measured time is reported.
2. **The VO startup prefix (D87 carry) is never released.** 3.4's chain spans min..max ids seen, not 0..max.
3. **Inter-agent features are ALIKED-n16 + LightGlue** (the hard-pair path, D63: AUC@10° 0.607 against ORB's 0.284).
   - fp16 on the wire; int8 is a 3.6 row.
   - The verify chain is collaboration.md §11.
   - Thresholds are calibrated on the other scene.
   - ORB/mean-bit becomes the `orb_meanbit` ablation. Intra-agent loops stay ORB (a registered carry).
4. **Datagram size goes on the link seam.** Loss is evaluated per datagram at its size with RFComms `ExpectedPer`. The
   MTU stays 1500 (D102 L8).
5. **Arms** map to D170 and §31.1 (design §4). Budget levels are {10, 25, 50, 100} % of the similarity arm's
   measured fetch rate on the calibration scene. D170's 20 kB/s and 200 kB defaults are unsourced placeholders until
   that measurement replaces them.
6. **The byte gate is completed:**
   - per-message pairing of predictions with first transmissions;
   - a predictor per class;
   - retransmit, ack and resync (after an election) as overhead rows;
   - predictions summed over every broker;
   - cross-scene calibration.
7. **"3 seeds"** = one seed driving both passes: IMU noise and VO RANSAC in pass A, link loss and verifier RANSAC in
   pass B. D is `none`.
8. **Scene 1's inter-agent opportunities are measured first.** If there are fewer than 10, a third scene (maze_short)
   is rendered before the gate.

**As built (same day, before any gate run):**
- **Global descriptor on the wire: fp16**, arch §15.7's encoding.
  - An appended `descriptor_f16` field in `GlobalDescriptor`. v1 bytes are unchanged; the golden test still pins
    1142 B and FNV `0x653ebb54ae4fd40b`.
  - The codec had shipped float32 only. D93's int8 stays a 3.6 row.
- **Retrieval threshold: cosine 0.8.**
  - Source: Swarm-SLAM's stereo `similarity_threshold`, from `cslam_experiments` `kitti_stereo.yaml`, used there on
    CosPlace, EigenPlaces' predecessor from the same group.
  - `top_k` 3 is [ASSUMPTION].
  - `edges.csv` records every verify call with its ground-truth edge error, so the scene-1 check can revise both.
- **Predictions per message.**
  - Every descriptor, pose edge and graph batch is priced when planned: the codec's exact payload, times the class's
    calibrated ratio for the compressed classes (4, 5, 6, D14), plus framing.
  - The prediction reaches the ledger with the first transmission (`ByteRecord::predicted_wire_bytes`, ledger only).
  - Requests and responses keep D170's selection-time prices.
- **`resync`.** Sends to a new broker carry a ledger-only tag: descriptors not yet sent, every edge, and the forced
  snapshot. They form an overhead row, outside the gate.
- **Robust stage.** `RobustMultiAgentSolve` runs D76's `RobustJointSolve` through a new `MultiAgentInput::solver`
  hook.
  - An agent tied to the gauge only through rejected edges is reported **unplaced**, never placed on those edges.
  - The plain-PGO `MultiAgentSolve` is the `robust: false` ablation row.
- **Edge noise model:** the EuRoC fit (`euroc_v2_loops.yaml`), carried to the sim [ASSUMPTION]. A sim fit from
  `edges.csv` follows the first sweep.
- **Local descriptors: int8 with a per-keyframe scale.** This is arch §15.7's default (fp16 is its ablation), and it
  corrects this entry's earlier fp16 choice.
  - Measured (`LocalDescriptorCodec.LightGlueMatchSetUnderFp16AndInt8OnSimPairs`, D90 sim pairs f200/f205 and
    f700/f710): LightGlue's match set is **identical** to fp32's, precision 1 and recall 1, for both int8 and fp16.
  - Caveat: these are near-temporal pairs. Inter-agent pairs are harder, and `edges.csv` keeps watching.
  - The feature response carries `feature_bytes` and `header_bytes` (appended wire fields; v1 bytes unchanged).
    The decoder had hard-coded ORB's 32 B per feature and rejected every other layout; the smoke found this, and
    `Serialization.LargeFeatureResponseRoundTripsThroughFragmentation` now pins it.
- **Token-bucket depth:** at least the largest single fetch (RFC 2697: CBS ≥ the largest packet). With a depth of one
  period's level alone, a level below one fetch per period starved the arm instead of rationing it (smoke: 0 of 246
  candidates ever selected).
- **Verifier calibration, pre-registered (2026-10-23, before any scene-1 data).**
  - **Label:** a candidate is *good* iff its pose error is at most 10°. That is D63's protocol for this matcher, the
    LightGlue/SuperGlue relative-pose AUC@10°: the larger of the rotation error and the translation-direction
    error, with rotation only when the true baseline is under 0.1 m.
  - D73's 0.2 m / 2° rule is not used here. It would label a true 13 cm / 2.6° inter-agent edge (180 inliers) bad.
  - **Data:** the similarity arm on the **ideal** link of the OTHER scene, all seeds. Verification is geometry; the
    `collapsed_v1` runs deliver too few responses to calibrate on.
  - **Candidates:** rows that reached PnP and that the D71 gate did not reject.
  - **Floor:** the smallest PnP-inlier count ≥ 25 (the 1.6 verifier's) that accepts no bad candidate. Only this floor
    is varied, and only upward: B1 records PnP counts only for candidates that passed the E gate.
  - **Reported:** the recall of good candidates, and the margin between the worst bad and the weakest good.
  - **Stop condition:** if either scene has fewer than 10 good candidates, the sweep stops before B2 (this entry's
    precondition).
- **Smoke finding (scene 0, 30 s, NOT gate data): perceptual aliasing reaches the merged map.**
  - One query keyframe of agent 1 verified against five of agent 0's keyframes, each about 10 m / 80° off, with
    25–27 PnP inliers. The true edges have 143–180.
  - The five are mutually consistent, so PCM accepts them as a clique: D75's aliasing-group failure.
  - On the `ideal` link the true edges also arrived, and the robust stage kept them: merged ATE 2.6 cm.
  - On `collapsed_v1` only the false group arrived: merged ATE 3.55 m.
  - No back-end setting can reject a self-consistent group with no competitor. The group-level defence (five edges
    from one query are one piece of evidence) is 3.5's, with its own preflight (arch §17.3). The sweep reports the
    accepted-false-edge rate and the map deformation per arm, and those runs stay in the plot.
- **First AWS sweep (`20261023T073806Z-cpux86-7f912a`, 74 min, about $0.15): failed at calibration on a script bug.**
  - Stage A (12 stores, 43 min) and B1 (12 runs, 25 min) completed. The calibration then globbed `cal_s<k>_seed*`,
    which also matches the jobs' `.log` files, and crashed. Fixed: directories only.
  - The script exited before collecting, so the instance took B1 and the stores with it. Fixed: any failure now
    collects first, and the pass-A stores are kept under the run's results (`STORES_FROM` reuses them, checked
    against `stores.SHA256SUMS`).
  - **Trimmed to the gate's rows** (2026-10-23 rule: no runs beyond what the gate needs). B2 runs two budget
    levels (25 %, 50 %) instead of four, and C runs one seed per variant and scene. The plot still has three arms on
    2 scenes × 3 seeds; the 10 % and 100 % levels are not run.

**§49:**
- catalogue: algorithms.md (COSMO-Bench, Tian et al.);
- arch: §5.7/§15.6 as-built note at merge;
- ablation matrix: §31.1 rows mapped;
- benchmark config: the sweep settings, recorded in each run's manifest (no separate benchmark file);
- manifest: protocol, arm, budget level and its derivation run, comms scenario, seed, pass-A cache sha.

## 2026-10-23 — D172: Phase 2 full gate on AWS — 53/54; the 54th was the D55 reference's own dirty flag; `report.yaml` re-pinned

**The run.** `phase1.sh` in full mode on clean commit b59a77d, on AWS (cpux86 spot, 32 min), with a fresh Kaggle
render.
- The render is relayed: the instance hands the package to S3, and the workstation renders it with its own token.
  The Kaggle token never leaves the workstation; staging it to S3 was rejected.
- Evidence: `evaluation/runs/gates/phase2_exit/exit_gate/aadffb8df5790ca0-20261023T050501Z/`.

**Result: 53/54.**
- ctest 539/539, protocol 18/18, every I1 condition and every §4 contract.
- Fault identity matches the committed reference, and so does `bytes.json`.
- The fused cloud, render package and `renders.npz` sha256 all match. The fresh Kaggle render is byte-identical
  again, D48 reproduced on a third render session.

**The FAIL.** `report/report.yaml` differs from the D55 reference on one line: `inputs_clean: true` here against
`false` there.
- The D55 run, despite its clean commit, recorded `git_dirty: 'true'` in `recon/.../manifest_score.yaml`.
- The comparator compares `inputs_clean` deliberately (D48: "so a dirty run cannot pass"). Against that reference
  it did the opposite: a clean run failed, and a dirty run would have passed.

**Decision.**
- Re-pin only `reference.report` to this run. Every other reference stays D55's, and all of them matched.
- The comparison rule itself is unchanged.
- The next AWS pre-merge (3.4's) confirms 54/54 against the new reference.

**Phase 2 exit gate, item by item (plan "Phase 2 testing / exit gate"):**

| bullet | evidence | state |
|---|---|---|
| Jacobian finite-difference suite incl. near-zero depth and wide-FOV rays | `tests/numerical/test_pinhole.cpp` and the analytic-Jacobian tests, in ctest 539/539 | green |
| Oracle agreement on 100 random synthetic graphs | `PgoOracle.AgreesWithGtsamOnEveryCase` over `.cache/oracle/pgo` (100 graphs), in ctest | green |
| Certificate certifies known-optimal graphs, not planted-outlier ones | `Certificate.*` (D74); SE-Sync large graphs (c2648e6): sphere/torus/cubicle/grid3D certified | green, **one stated deviation:** rim "not certified, unknown not wrong" (D74; no re-tuning) |
| Outlier ramp 10→90 %, breakpoint | `evaluation/runs/robust-backend/outlier_ramp/` (D75): PCM → GNC breakpoint at 90 % (groups of 10), 70 % (two growing groups) | done |
| EuRoC stereo-VIO in the reference error class | D85 / D86 (9/11 accepted), D87 full system with loops (10/11 in class, ≥ ORB-SLAM3 on 7/11) | accepted |
| TUM regression pinned | D100 / D86 (fr1 accepted as pinned) | done |
| The full run at the phase gate (arch §0.2) | this run | 53/54, the 54th resolved above |

**Phase 2 is closed.** Carries:
- V1_03 is the 5.5 regression case;
- the VO-side startup prefix (D87);
- D91's pool rule goes to 7.3.


## 2026-10-23 — D190: Phase 6 architecture — the GPU trains, the browser renders; only the active submap trains; budgets derived from a T4 measurement

Design `docs/design/reconstruction.md` (preflight: arch §18–19, §29, D44, plan 6.1–6.4 and the exit gate; algorithms.md §2–4;
SOTA survey reconstruction.md Appendix A; OpenCV-native check Appendix B; design review). Reviewed on 2026-10-23: the
design is sound. Benchmark approved; the rest is built once D191–D195 are written.
- **Split.** The python service (PyTorch + gsplat v1.5.3 pinned, Apache-2.0) trains. A browser renderer draws
  the splats on the client GPU (every web renderer surveyed rasterises client-side, A.5). Two budgets, kept apart:
  - T4 ms per iteration × iterations per keyframe ≤ the keyframe interval;
  - browser splat count, sort rate and bytes per update.
- **Measured inputs.**
  - Keyframe rate: dark + lamp 4.4 KF/s per agent, lit 1.6 (D142 rows). That gives ~110 ms of GPU per keyframe for
    two agents in the dark demo.
  - No published T4 number exists for any candidate or for gsplat (A.3/A.4), so the iteration count comes from
    `kaggle.sh bench`: train and render ms/iter against N, resolution and packed, with and without D-SSIM, plus the
    append path.
  - The demo T4 is shared with gz EGL and the live estimators on one g4dn.xlarge (2026-10-23). The idle number is an
    upper bound; the budget is measured with gz rendering.
- **Bounded work.** Only the active submap trains (Gaussian-SLAM / GLC-SLAM). Closed submaps are frozen and sent once.
  Submaps are anchored to their first keyframe in the agent's odometry frame O and close at 30 KF (arch §18.6: 20–40).
  A PGO or O→W change is one 4×4 per submap, applied as a per-object transform in the renderer: no retraining and no
  re-download. Exit-gate bullet 2 becomes a CPU unit test of that algebra.
- **Seeding and loss.**
  - Seeding: half-res Z back-projected where the current render's α < 0.5, with a radius check (Gaussian-SLAM); the
    initial scale is the pixel footprint Z/f.
  - Iterations: at least 40 % on the new keyframe (Gaussian-SLAM).
  - Loss: 0.8 L1 + 0.2 D-SSIM (Kerbl 2023) plus a σ-weighted depth L1 (§19.3 L_photo + L_depth). L_light and the
    §19.4 specular weight belong to the batch path (6.3).
- **Interfaces** (agreed with the live runtime): `ariadne_live` tees each D168 `#kf` record (LinkFrame framing) to a recon socket, and
  sends T_W_O[k] once per agent. The replay spool (reconstruction.md#first-reconstruction-loop, schema bumped) feeds the same reader. The splat panel is
  its own JS module on its own `/splat` websocket inside the operator console.
- **Open:** judging-week GPU hosting (one T4 shared with gz, or a second GPU; on-demand
  $0.558/h, quota 0; spot reclaimed 3× on 2026-10-23), and Spark instead of GaussianSplats3D (arch §29.3's example
  is unmaintained per its own README).

## 2026-10-23 — D191: the RTG-SLAM stability threshold, converted to keyframes rather than copied — δη_KF = 15 [ASSUMPTION]

RTG-SLAM (arXiv 2404.19706) freezes a Gaussian once its confidence count reaches δη. δη = 100 gives 35.43 dB at
17.31 FPS against 33.63 dB at δη = 50 (Table 15). That count is per **frame**: Replica processes every frame at
30 Hz, so δη = 100 is about 3.3 s of consistent observation. Ariadne counts **keyframe updates**. Converted at equal
observation time with the measured dark keyframe rate: 3.3 s × 4.4 KF/s ≈ **15**.
- [ASSUMPTION] Equal time stands in for equal evidence. Keyframes are more distinct views than consecutive frames, so
  15 errs towards freezing late (more optimisation, never a premature freeze).
- In the lit (ablation) rate of 1.6 KF/s the same rule gives ~5. The lifecycle reads the run's measured rate, not a
  constant.
- Reported from the first spool run: the stable fraction over time, and the PSNR of frozen against unfrozen
  Gaussians. Changed only through a measured ablation, never by eye.

## 2026-10-23 — D192: the per-keyframe 3×4 exposure transform likely never switches on live; the batch path carries the evidence

Arch §18.7 enables the affine only after ~5000 optimisation steps (Hierarchical 3DGS; skipping the delay is the
"geometry mush" failure). Live, ~110 ms of GPU per keyframe at the dark rate leaves on the order of 10 iterations per
keyframe (to be derived from the D190 benchmark). A 30-KF submap then gets ~300 steps, far below 5000.
- **Decision.** The rule is kept exactly: a per-submap step counter, transform identity until 5000. It will therefore
  most likely stay off live, and the live trace says so (`exposure_affine: off (steps N < 5000)` per submap).
- **Evidence path.** The exit-gate exposure-delay ablation (on-time vs early) runs on the batch path (Kaggle or g4dn
  on a spool), where submaps reach 5000 steps. Live never claims the transform.
- With §19.1's normalisation in place the transform's job is small (arch §18.7), so its absence live costs little.
  That claim is [UNVERIFIED] until the batch ablation measures it.

## 2026-10-23 — D193: no clone/split densification on the live path — depth seeding is the only growth

On the live path, Gaussians are added only by depth seeding at keyframes (α < 0.5 plus a radius check, D190) and
removed only by pruning opacity < 0.005 (gsplat DefaultStrategy's threshold). No gradient-driven clone or split.
- **Backing.** Gaussian-SLAM runs without clone or prune (reconstruction.md A.2). LoopSplat and GLC-SLAM inherit that, and
  RTG-SLAM grows from depth-sampled pixels only. gsplat's DefaultStrategy refines from step 500 to 15,000 with an
  opacity reset every 3000 steps. A live submap never reaches those schedules (D192's step count), so running it would
  be untested behaviour.
- **Growth bound.** The live Gaussian count follows the observed surface area, which is what keeps the browser budget
  (D190) predictable.
- **Batch path.** The batch trainer may run DefaultStrategy or MCMC as an ablation row with its own entry. The default
  stays depth-seeded on both paths, so live and batch reconstructions are the same model.

## 2026-10-23 — D194: without a GPU the console still grows a map, labelled "initialisation only, no GPU"

With no CUDA device, the service runs the reconstruction.md#first-reconstruction-loop CPU path (fuse + init, D44) on each keyframe. It streams those
depth-seeded, **unoptimised** Gaussians to the same `/splat` panel: geometry from stereo depth, colour from the
training image, no photometric optimisation.
- The panel and the manifest say `splat_backend: cpu_init` and "initialisation only, no GPU". No PSNR/SSIM/LPIPS is
  computed or quoted on this path; Chamfer against the GT mesh may be.
- The realtime demo still needs a GPU (D44). This path exists so a reclaimed or missing GPU degrades the
  demo instead of blanking the panel.

## 2026-10-23 — D195: training images are the albedo proxy by default; linear RGB on lit worlds is an ablation row

Locked (arch §0.2, arch §19.1): the training images are the falloff-normalised albedo proxy. A design review
(2026-10-23) adds that the demo world is dark, lit by the agents' own lamps (arch §42–43).
- **Default.** The trainer refuses a record without the proxy: a `#kf` or spool record tagged `training_image: none`.
  That refusal applies unless the run is configured as the RGB ablation. There is no silent fallback.
- **Ablation row** (`training_image: rgb`, lit worlds only). Linear RGB, where the radiance is static (fixed lights),
  so §19.1's coherence problem does not arise. It is labelled in the manifest and on the panel, and compared with the
  proxy only on its own scene.
- **The live chain to the proxy** (stated so nobody expects it sooner):
  1. the 5.3 live lamp sink, because `KeyframeFalloff::Normalize` needs the frame's applied photometric state and live has
     no D;
  2. a dark demo world (Phase 8, unowned; raised for decision);
  3. a lamp calibration for that world's rig.
- **Until then.** The live `#kf` carries the fields, tagged `none`. The trainer is built and gated on replay spools of
  `ci_tiny_dark_0` (lamp-only, calibrated, proxy in `KeyframeBundle`, D146). The spool writer emitting the proxy is
  6.1's first item.
- **Display.** The panel shows the albedo proxy directly and is labelled so. §19.1's relit view needs a custom
  renderer shader and is out of scope.

## 2026-10-23 — D196: T4 measured — gsplat 1.5.3 train/render cost; iterations per keyframe derived from it

`infra/scripts/kaggle.sh bench` ran on a Kaggle Tesla T4 (idle) with torch 2.11.0+cu128 and gsplat 1.5.3. Evidence:
`evaluation/runs/reconstruction/bench_t4_20261023T1113Z` (L1 + depth) and `bench_t4_dssim_*` (the reconstruction.md §2 loss). Each point is
the median of 40 timed iterations after 8 warm-up. Times are ms per iteration (forward + backward + Adam), unpacked:

| N Gaussians | 320×180 full loss | 640×360 full loss | 640×360 without D-SSIM | render 640×360 |
|---|---|---|---|---|
| 50k | 5.9 | 9.0 | 3.8 | 1.4 |
| 100k | 6.3 | 10.9 | 5.3 | 1.7 |
| 250k | 9.1 | 15.3 | 10.2 | 3.0 |
| 500k | 13.5 | 20.7 | 15.9 | 4.9 |

- **Packed** is slower at every point here (by 10–25 %), as gsplat's docs predict for small scenes. The live path runs
  unpacked.
- **D-SSIM** costs ~5 ms per iteration at 640×360: a plain-torch 11×11 convolution SSIM. A fused SSIM kernel is the
  obvious lever, measured before it is adopted.
- **The append path works.** 100k + 10k Gaussians took 1.4–2.0 ms, the Adam moments of the old rows stayed bitwise
  unchanged, and 10 steps after the append were finite. That is the mechanism the live seeding relies on (D193).
- **SH degree 3** costs 15.3 ms at 250k / 640×360, against 10.2 ms at SH0 without D-SSIM. Live stays SH0, which is
  what the .splat chunk carries anyway.
- **Derived budget.**
  - Dark demo: ~110 ms per keyframe for two agents (D190). At a 100k–250k active submap that allows **~7–10
    iterations per keyframe at 640×360, or ~12–17 at 320×180**, on an idle T4.
  - The trainer's `budget_ms` mode computes the count from its own measured median, never from this table. The table
    is the planning number.
  - Whether live trains at 320×180 is decided by the first spool run's held-out PSNR at each resolution, not here.
- **Caveats.**
  - [ASSUMPTION] The synthetic scene is representative: ~2 px footprints filling the view. Real submaps with larger
    Gaussians touch more pixels each.
  - The demo T4 also renders gz, so these are upper bounds until measured under gz load (D190).

## 2026-10-23 — D197: the console splat renderer is Spark, not GaussianSplats3D (approved)

Arch §29.3 names GaussianSplats3D "e.g." as the in-browser Gaussian renderer. Its README now says it is no longer
actively developed and points to Spark (reconstruction.md A.5). Spark was approved on 2026-10-23.
- **Spark (sparkjs, MIT, v2.4.0 at 2026-10-20).** Many `SplatMesh` objects are sorted together, each with its own
  transform, and they resize incrementally. That is exactly the shape D190 needs: one mesh per submap, re-anchoring
  = a new transform, and the active submap replaced in place. Formats include .splat (32 B/Gaussian), .spz and .sog.
- **The version is pinned exactly** in the page's import (an exact version at least two weeks old), never a floating
  tag.
- **As-shipped version check:** the submitted viewer imports Spark 2.3.1 (see `python/operator_console/static/splat.html`); the 2.4.0 value above records the version current when this decision was written, not the version in the delivered page. Reproduction follows the import map: Spark 2.3.1 and Three.js 0.180.0.
- **§49 ritual.** The arch §29.3 sentence gets a dated update naming Spark and this entry. GaussianSplats3D stays
  allowed as a fallback only if Spark fails a measured smoke.

## 2026-10-23 — D198: the live trainer follows Gaussian-SLAM's own recipe; three T4 smokes, measured honestly

The first trainer (D190–D193) mixed three systems: RTG's 5 % sample, a 1-px footprint scale, a σ-normalised depth
term of my own and RTG's opaque discs. Two Kaggle T4 smokes on a synthetic corridor showed it. The corridor has exact
ray-cast depth and an analytic albedo with std 0.226, so predicting the mean scores **12.9 dB**.
- **Smoke 1** (`fbbe641`): held-out PSNR flat at ~17 dB. The 1-px scale at a 5 % sample left ~4.5 px holes.
- **Smoke 2** (`da9d9d4`, kNN scale): 13–16 dB. It seeded nothing after the first keyframe, and the uncited
  σ-normalised depth term dominated the loss (~5 per pixel against ~0.15 for the photometric term).

**Decision.** Transcribe Gaussian-SLAM (arXiv 2312.10070; TUM-RGBD values, the real-sensor setting), the system D193
already rests on, without mixing:
- **Seeding.** A submap's first keyframe takes a uniform M_u plus colour-gradient M_c, as fractions of 307,200 pixels
  (0.326 / 0.163). Later keyframes sample uniformly where α < 0.6. ρ = 0.01 m; scale = the nearest-neighbour
  distance; opacity 0.5.
- **Loss.** L = (0.8 L1 + 0.2 D-SSIM) + metric depth L1 + an isotropy term |s − mean(s)|, all weights 1. MonoGS's
  code uses the same metric-L1 form: α·L1_rgb + (1−α)·L1_depth, with α = 0.9 on TUM.
- **Pruning and iterations.** Prune opacity < 0.5 at the middle and the end of a keyframe's iterations. Batch path:
  100 iterations for a submap's first keyframe and 100 for the rest.
- **Unspecified in the main text:** M_k (the supplement's value; the uniform fraction is used), and what "high
  colour gradient" means (above the median Sobel magnitude here). Both are tagged [UNVERIFIED] / [ASSUMPTION] in the
  code.
- **Freezing fix.** STABLE rows (D191) are restored after each Adam step and their moments zeroed. Zeroing the
  gradient alone let `exp_avg` keep moving them.

**Smoke 3** (`c09d202`, evidence `evaluation/runs/reconstruction/train_smoke_20261023T1340Z`, 320×180, 60 KF, 30-KF submaps):

| Variant | Iterations/KF | ms/KF | Train-view PSNR, init → after (median) | Held-out PSNR | α coverage |
|---|---|---|---|---|---|
| Gaussian-SLAM batch | 100 | 715 | 22.2 → 25.1 (early KFs 19.5 → 26.7–29.6) | 17.8–20.1 | 0.93–0.96 |
| live budget | 15 | 123 | 20.8 → 21.3 | 17.7–18.5 | 0.91–0.96 |
| batch, no depth term (diagnostic) | 100 | 694 | 22.8 → 26.5 (early 32.3) | 18.6–19.8 | 0.94–0.97 |

- **No pipeline bug.** The model fits the views it trains on, so the earlier 17 dB was the recipe. Freezing and
  re-anchoring are bitwise exact on the GPU.
- **The live budget barely moves the seeds.** At 15 iterations per keyframe the training view gains ~0.5 dB.
  Pruning at the middle of so few iterations removes fresh seeds before they are optimised: the first keyframe goes
  19.5 → 16 dB. Live quality is therefore essentially the seed quality. That is a measured property of the live path
  at this budget, not a defect to tune away by eye.
- **The held-out gap** (train 25–30 dB, held-out 18–20 dB) is a generalisation gap on a hard 6-rad/m checker with a
  fine sinusoid, seen from a forward-walking camera.
- **Both are open.** Real-scene quality is the next measurement: a lit sim spool (D199, the demo world) at the batch
  and live budgets, with the PSNR, SSIM and LPIPS of the exit gate. Any change to the live recipe (iteration
  placement, pruning at low iteration counts) goes through its own entry with backing.

## 2026-10-23 — D199: the live demo trains on linear RGB on the lit procedural maze; the albedo proxy is the dark-world stretch (directive)

The directive of 2026-10-23: dark, lamp-lit and dusty runs are stretch goals, taken up only
after the core demo runs in real time. The demo world is a lit, random, seeded procedural maze. The reconstruction demo
parts are the incrementally updating splat in the console and a 3D model of the maze shown next to it.
- **Default for the live demo:** `training_image: rgb`. The service trains on the keyframe's linear RGB, labelled
  "RGB (lit world)" on the panel and in the manifest. Under fixed lights the radiance is static, so §19.1's
  coherence problem does not arise. The live `#kf` record carries the RGB as is.
- **D195 is narrowed, not reversed.** The albedo proxy stays the locked training image when the light source is the
  agents' own lamp: dark and lamp-lit worlds, the stretch. D195's refusal rule still applies there. A dark-world
  record without the proxy is refused, never silently trained on RGB.
- **The Oct 24 offline gsplat** runs over the lit 3-agent live mission, not maze_dark_0. The glue is unchanged: per-agent
  spools plus pass B's T_G_W per agent.
- **Priority:** real-time incremental splat updates first.
- **Open:** what "a 3D model of the maze" means here. One reading is the generator's own scene mesh (ground
  truth, labelled as such). The other is a reconstructed model (the planner's TSDF/ESDF). They make different claims and
  must not be confused on screen.

## 2026-10-23 — D173: §42 item 7 false-closure injection in pass B; merge evidence; the N-agent mission spool (schema 3)

**Context.** Phase 8's demo mission (D213) gets the collaboration items (§42 items 4–8, 13–14, 16) from one
mission recording replayed through 3.2's pass A/B. Item 7 needs "at least one false closure generated
intentionally", and the N-agent mission stage needs per-agent merges, which the 2-agent 1.7 spool cannot carry.

**Decisions.**
- **`CollabNode::InjectVerifiedEdge`** (evaluation only) plants an edge at the broker exactly as if its verify hook had
  returned it. It travels and is solved like any verified edge, so only the robust stage (PCM → GNC, D76) can reject
  it. It counts as `edges_injected`.
- **Pass B `inject_false_edges:`** (default none, so outputs without it are unchanged): `{t_s, agent_a, kf_a, agent_b,
  kf_b, offset_m, yaw_deg}`. Z = the true T_Ca_Cb · [R_z(yaw) | offset]. The information is copied from the last edge
  verified on that agent pair, and planting waits until both keyframes are released and the pair has a verified edge.
  Logged as reason `injected` in edges.csv. results.yaml reports, per injection: planted time, ground-truth error,
  solves seen, solves that kept it, final weight, and rejected.
- **What is and is not rejected (correcting an earlier claim).** PCM accepts a lone candidate in a pair group as
  "untested" (pcm.hpp), so a false closure with no competitor is kept. It is rejected once true loops for that pair
  exist. Smoke (scene 0, ideal link, 30 s, `build/smoke/b_inject`, NOT gate data):
  - The injection was planted at 25.0 s, 4 m / 45° off, when the pair's only other edges were the false aliasing group.
  - It was **kept for 1 solve**, then rejected (final weight 0) after the true loops arrived at 27.8 s.
  - Merged ATE was 2.6 cm, unchanged.
  - The demo's injection must follow the pair's true loops, and its evidence must report `kept_solves`.
- **Merge evidence** (`results.yaml`):
  - `estimates`: per agent, the adopted frame anchor and T_G_W.
  - `kept_inter_edges`: every inter-agent loop still kept by the last solve that contained it, with when it was first
    kept. Injected loops are never evidence: the smoke's injection was the first loop ever kept.
- **Mission spool schema 3** (agreed across planning, reconstruction and photometrics): the schema-2 bundle fields, plus a `merges:` list in place
  of the single `merge:` block. Composer: `python/analysis/compose_mission_spool.py`.
  - Inputs: per-agent `ariadne_sim_vo --spool` runs and the pass-B run.
  - For each agent k: T_O0_Ok = T_G_W0⁻¹ · T_G_Wk, only when agents 0 and k share an anchor.
  - The edge is the earliest kept loop joining k to an agent already in agent 0's frame, with both keyframes spooled.
  - Otherwise `ok: false`.
  - Files are symlinked. [ASSUMPTION] pass A and sim_vo give identical keyframe ids on the same recording; the
    composer checks that the edge keyframes exist, and the mission run checks that the ids match.

**Tests.** protocol 62/62. Composer 2/2: a 3-agent chain merge, an unmerged agent, and the split-brain case (different
anchors never merge).

## D218 (2026-10-24): D175 item 4 - the per-agent pipeline in ariadne_live, POC proven

multiagent.enabled: per-agent Phase 4 planners fed only through the radio (coordination-only CollabNodes over the
seeded CommsNode, scenario yaml), the D175 SS5 barrier order with a 0.5 s radio tick ladder + delivery pump,
share-nothing fork-join between barriers (no locks), auction picks replacing the centralised joint[k], the
operator's received map + OperatorEgress driving the console. Evidence (20 s, 3 agents, lit maze_demo_0,
lockstep, plain broker, ideal link): one component of 4 every barrier; bytes patch 6100 / bid 6880 / consensus
1744; picks 6 coordinated / 3 no-consensus (barrier-1 lag, pre-registered) / 3 no-assignment (exactly-once
dedup); double replay PASS; oracle A/B on the SAME binary reproduces digest 3d10d011018c42d9 byte-identical
(multiagent:false = D175's oracle row, untouched). Runs: build/live/ma_smoke3, build/live/oracle_ab.
## 2026-10-24 — D230: splat placement — every agent trains in its own odometry frame; T_W_O per agent from the briefing or the merge

**Context.** The multi-agent POC (item 10, ordered 2026-10-24) needs one splat view of all agents in the mission
world W, side by side with the generated world's GT mesh, which lives in W. The replay used W = agent 0's odometry
frame and baked agent 1's merge into the poses (`T_O0_C`), so the reconstruction and the GT viewport did not share a
frame, a third agent was not handled, and a live T_W_O change would have meant re-ingesting.

**Decision** (`python/reconstruction/ariadne_recon/live/placement.py`, `service.py`).
- Keyframes enter in their own agent's odometry frame O_k. A submap is drawn at T_W_O[k] T_O_anchor (arch §18.6:
  a frame change is one matrix per submap, no retraining). `Service.set_placement(k, T_W_O)` is the one entry point,
  used by the replay now and by the runtime's `t_w_o` tee record later (2026-10-24: the tee is a strict superset of the
  briefing constant).
- **briefing** (default with `--scene`): T_W_Ok = T_W_Bk T_B_C for every agent: entrance k's pose (scene.json
  `entrances[k]`, z from `routes/agent_k.json`) times the rig extrinsic (mission `R_B_optical_rows`,
  `t_B_camera_m.left`). This is the live demo's known-starts rule (D216), transcribed from `ariadne_mission.cpp`'s
  world anchor and generalised from agent 0 to agent k.
  A live scene's `mission_live.json` is read first: `live.demo.entrances[k]` is the WORLD entrance mission agent k
  ran (worldgen `mission.py`; 2026-10-24); without the block, agent k = entrance k.
- **merges**: agent 0 from the briefing, agent k as T_W_O0 T_O0_Ok from the spool (schema 3 `merges:`, D213; or
  schema < 3's `merge:`). Agent k's own entrance is not used, as in `ariadne_mission.cpp`.
- An agent with no placement is skipped and listed under `unplaced` in the manifest. It is never drawn in someone
  else's frame (honesty rule: an unmerged map is not shown as merged).

**Not decided here.** Cross-agent overlap pruning (6.3, double walls) is unchanged: each agent's submaps are separate.

**Tests.** `python/reconstruction/tests/test_live.py` 12/12: the briefing transform on a yawed entrance, the live.demo entrance choice, both modes on a 2-agent schema-3
spool, an unmerged agent never drawn, and set_placement moving the table without new chunks. All reconstruction
python tests 23/23 (ariadne-recon:1).

## 2026-10-24 — D231: the live keyframe tee contract (runtime ↔ service) and the service's `--listen` mode

**Context.** The POC's last splat gap: the live runtime feeds the service directly, not through a spool. The runtime side is
`ariadne_live` `deliver_kf` and `live.sh`; the other side is the service. The format was proposed by the service side and
confirmed with amendments by the runtime side (2026-10-24).

**Contract** (`python/reconstruction/ariadne_recon/live/link.py`).
- **Transport.** The runtime's LinkFrame framing (D150: `"ARL1" | topic | payload`, little-endian) on a unix stream socket. The
  service listens on `ARIADNE_RECON_SOCK`; `ariadne_live` connects. The runtime side keeps a bounded queue of 8 per
  agent, drops the oldest, counts drops in its results, and is a silent no-op with no listener. The splat never
  back-pressures VO or the mission (memory: pipeline share-nothing).
- **`#kf`** = u32 json_len | JSON {agent, id, t_ns, pose_version (0 until PGO re-anchoring), T_O_C (12 row-major),
  K (full res, the RGB's), Kz (spool scale, Z/S's), w, h, zw, zh, training_image} | Z | S (float16, zh×zw, **0 =
  invalid**, the runtime's HalfAsSpool(MaskedDepth32)) | RGB (rgb8, h×w×3, rectified left).
- **`#t_w_o`** = JSON {agent, t_s, xyzquat [x, y, z, qx, qy, qz, qw], **qw last**}: once per agent at start (the
  briefing constant), then on every change. `#pose_version` is the agreed upgrade path for re-anchoring.
- **Live trains on the replay's pixels.** The spool stores RGB at scale 0.5 by INTER_AREA. `half_rgb()` is the
  integer 2×2 area average (`(a + b + c + d + 2) >> 2`), tested bit-exact against `cv2.resize(INTER_AREA)`, and the
  trainer gets Kz. Proved end to end: the 3 bring-up spools through the mock tee and the socket give byte-identical
  chunks to the replay (4 submaps; placements equal to 4e-16 after the quaternion round trip).
- **Robustness.** A malformed `#kf` or `#t_w_o`, a keyframe of an agent with no placement, a non-RGB training image,
  or an unknown topic is dropped and counted in the manifest's `link` block, never guessed and never fatal. With
  `--scene`, every mission agent starts at the briefing constant (D230) until its `#t_w_o` arrives.
- **Image.** `ariadne-recon:1` for `cpu_init`; the console image lacks scipy and cv2, which seeding and the spool
  reader need. The GPU backend is the gsplat image (Kaggle kernel image today; a judge-box image is open).

**Tests.** `python/reconstruction/tests/test_link.py` 13/13 (bit-exact half_rgb, #kf round trip and short payload, qw-last quaternion,
Shepperd round trip at every entrance heading incl. 180°, live == replay chunks, placement by #t_w_o and the drop
counters, briefing until #t_w_o, a bad #t_w_o not fatal); all reconstruction python tests 38/38. The mock caught two
bugs before the runtime tee existed: a naive quaternion at 180° (norm 5e-7), and a service crash on a bad header.

## 2026-10-24 — D232: the estimator is thread-count-invariant; the pipeline's critical path is the per-agent lane (measured)

**Context.** The pipeline profile (measurement only). Job 44b (`20261024T024346Z-cpux86-29bfe2`, c7i.4xlarge) replayed
the ma200 live run (`70a514`: 200 s, 3 agents, pre-D179 lockstep) from its recorded `inputs.mcap`. It used that run's own
snapshot, payload binaries and core image (by id), with the flags of `live.sh cmd_replay`. The only variable was the
OpenCV pool of the 3 D168 estimator workers. Evidence: `evaluation/runs/performance/pipeline_profile_20261024/aws_replay/`.

**Facts.**
- **Thread-count invariance.** `estimate_body.tum` is bit-identical at pools of 16 (default), 5 and 2: all 3 agents,
  4001 poses each, max position difference 0. It is also identical to the run's own earlier replay1. So, unlike `cv::ALIKED`
  (live-collaboration.md §7: output depends on the thread count), the estimator's pool size is not part of the determinism
  contract.
- **Pool sizing barely helps.** Wall time 402.8 s (16), 383.4 s (5, −4.8 %), 417.9 s (2: keyframes 117 → 155 ms). PSI CPU
  pressure is 9.4 % at 16 and 0.1 % at 5. Recommendation: default the worker pool to 5 threads (3 × 5 = 15 of 16
  vCPU). Sizing does not close the RTF 0.52 → 1 gap.
- **Critical path.** The 3 per-agent lane threads run at 93-97 % of one core each, while the box is 51 % busy and the
  pool threads sit near 20 %. Per frame the lane spends ~94 ms (mean): ~36 ms in the worker's own track and keyframe
  timers, and ~58 ms of ingest on the same thread. The ingest (`SimFrameAssembler::Feed`, per image) is a memcpy,
  then the scalar single-threaded double-precision `ApplyLens`, then cvtColor, then a clone. Per-function attribution
  comes from job 45 (perf).

**Decision (2026-10-24): the fix order.**
- (a) Parallelise `ApplyLens` rows with `cv::parallel_for_`. Bit-identical, so no decision is needed.
- (a2) Pipeline the lane: ingest of frame n+1 on a second thread while frame n tracks. A 2-stage in-order SPSC
  hand-off on the same data, also bit-identical.
- (a3) Remove the redundant copies where ownership allows.
- (b) `cv::remap` with `CV_16SC2` maps is NOT taken. It changes pixels by ≤ 1 LSB, and every banked result consumed
  the current operator (the record/replay invariant). Revisit only if (a) + (a2) miss RTF 1, as a pre-registered D41
  amendment.

## 2026-10-24 — D233: Phase 3.6 local-feature wire codings — pre-registered adoption rule (before any run)

**Context.** 3.6 was opened on 2026-10-24. The MATCH_RESPONSE dominates the collaboration bytes: today's
int8 / K1024 / float XYZ is ~136 kB per keyframe (3.2 sweep). The survey (`collab_fast_survey.md` §1-2) finds no
published test of the stock LightGlue-ALIKED weights on quantised or PCA'd inputs [UNVERIFIED], so this is measured
on our data, as arch §31.9 requires (accuracy against bytes). Design: `docs/design/wire-coding.md` (design-reviewed).

**Method (fixed here).**
- Inputs: the 120 real verify pairs of run `0345a5` (`pairs.py`: per scene 30 verified + 6 per rejection reason),
  from the pass-A stores, which are production's exact features. ALIKED never runs.
- Settings: 6 descriptor codings (fp32, int8 today, int4 uniform step 0.3352σ [Max 1960 via SECONDARY], bin1 sign,
  PCA-64/32 with cv::PCA fitted on scene 1's stores only and an int8 per-component scale) × 4 geometry codings (xyz
  today, z32, disparity u16 at 1/16 px, + u16 keypoints at 1/32 px), at K 1024.
- K 512 cannot come from the stores: no scores are stored, and cv::ALIKED emits unsorted (order check: 515
  inversions). It is judged by the same bars on job 70's knobs rows.
- Bytes: `WireCodec(kFragmented).EncodeFeatureResponse` records (payload, zstd, +34 B per 1,466 B fragment).
  Codings without a schema field are packed into the descriptors vector ([PROXY], ±16 B/KF).
- Tool: `ariadne_perf_bench bytes`. Verdicts: `evaluation/runs/collaboration/wire_coding_20261024/scripts/analyze.py`, which
  is mechanical.

**Rule.** Reference R = int8 / K1024 / xyz, run in the same process.
- Controls:
  - R re-verified at 4 other RANSAC seeds;
  - fp32.
- Precondition: R reproduces the production verdict on all 120 pairs. If it doesn't, the platform difference is
  reported and flips stay measured against the in-run R.
- A codec flip is an accept↔reject change vs R on a pair whose verdict is stable across R's 5 seeds.
- Per coding (its own row, the other factor at today's value), on each scene (scene 0 is the held-out claim for PCA):
  1. 0 codec flips in either direction;
  2. median pnp_inliers(S)/pnp_inliers(R) over R-accepted pairs ≥ 0.90 (ProxyInformation σ ∝ √(30/n): ≤ 5.4 %
     wider);
  3. wire bytes/KF ≥ 15 kB below R (≥ 1 s of airtime at 15 kB/s).
- **Adopted** only if bars 1, 2 and 3 all hold. **A separate decision is made** if bars 2 and 3 hold with 0 reject→accept and
  ≤ 1 accept→reject codec flip per scene. Otherwise **not adopted**.
- Pose error (D63 pose_err, GT t/r) is reported. It is flagged if S's median exceeds the largest median of R's
  5 seeds.
- If fp32 itself fails bar 1 or bar 2, nothing is adopted without a separate decision.
- The combination row must itself clear bars 1-3. It reports the absolute bytes/KF next to the reduction, and
  predicted vs measured wire bytes: the zstd ratio is calibrated on scene 1 and predicts scene 0, and today's
  layout-blind predictor is shown alongside (I6, ≤ 15 %).
- Reach: 0 flips bounds the stratified flip rate at ≤ 10 % per scene and direction, ≤ 5 % over both (95 %, rule of
  three). It is a screen.

**Approved:** 2026-10-24, as written, plus the two reporting additions (absolute bytes; predicted vs measured).
Runs on test-compute's Kaggle CPU harness (4 vCPU AVX2; the platform difference is covered by the in-run R).

## 2026-10-24 — D232 correction: the pool-sizing gain is −1.6 %, not −4.8 % (cold start in the first variant's wall)

D232 quoted job 44b's walls from `wall_s.txt`, the container wall: 402.8 s (16), 383.4 s (5), 417.9 s (2). The 16-thread
variant ran first and carries ~14 s of cold start. The runtime's own `results.yaml` wall_s is 389.1 s (16), 383.0 s
(5) and 417.5 s (2). Job 45's 16-thread rerun of the same inputs gives 389.0 s. So 5 threads gain **−1.6 %**, not
−4.8 %, over 16 threads.

The conclusion is unchanged and stronger: pool sizing does not close the RTF gap. RTF is 0.514 (16) vs 0.522 (5).
The 5-thread recommendation stands on PSI (9.4 % → 0.1 %), not on wall time. Both walls are tabulated by
`evaluation/runs/performance/pipeline_profile_20261024/scripts/replay.py` → `processed/replay_runs.csv` (container_wall_s,
runtime_wall_s).

## 2026-10-24 — D235: D182 verdict — the l2conv ALIKED graph is NOT adopted (criteria i and ii missed); K 512 not adopted (D233)

Job 70 (`20261024T051236Z-cpux86-d80b45`, c7i.4xlarge spot, $0.1168) measured D182's pre-registered criteria for the
l2conv rewrite (ALIKED's dense ReduceL2 as a 1×1 ones-Conv). The 120 real verify pairs were re-extracted with
ariadne_perf_kfdump. That re-extraction reproduces the pass-A stores: every keypoint is within 6.1e-5 px on all 178
pair keyframes, and the store replay reproduces production on 120/120 pairs. Evidence:
`evaluation/runs/performance/pipeline_profile_20261024/processed/{variants,fidelity,knobs}_summary.csv`.

| Criterion | Bar | Measured | |
|---|---|---|---|
| (i) accept/reject flips vs 1280x720 K1024 | 0 | **1** (scene 0, pair (0,13)-(1,6): ref verified 68/30/25 → l2 67/29/0 essential_gate; production had rejected it, pnp_gate) | **miss** |
| (ii) keypoints + scores bitwise; descriptor max \|diff\| | all; ≤ 4.9e-6 | 178/178 bitwise; **7.08e-6** (s0), 6.05e-6 (s1) | **miss** |
| (iii) p50 at 8 threads | ≥ 1.5× | 1112.7 → 469.9 ms = **2.37×** (1 thread: 2303.6 → 1848.6 ms) | pass |

**Decision: not adopted.** That is D182's rule as pre-registered: if (i) or (ii) misses, it is not adopted. It is not
relaxed after the fact, although the flip is a borderline pair, one match from the essential gate, which production
itself rejected. The local 2-keyframe check (≤ 2.07e-6) did not predict the 178-keyframe maximum.
- The ready adoption commit (`fb05b8b`) stays unmerged. Production keeps
  `aliked_n16_k1024_1280x720.onnx`.
- Revisiting it would need a new pre-registration with a RANSAC-seed envelope for the flip criterion (D233's
  control). That is a new open, so it is outside the 2026-10-24 feature freeze.

**K 512 (D233's job-70 row): not adopted.** `1280x720:512` against `1280x720:1024` loses 4 accepted pairs (bar: 0)
at a median PnP-inlier ratio of 0.526 (bar ≥ 0.90).

## 2026-10-24 — D234: Phase 3.6 result (D233's rule) — no wire coding adopted; the fp32 control fails, so it goes to a separate decision; post-hoc: int4 and 1-bit reproduce fp32's verdicts

Run `20261024T053045Z-kaggle-a2-bytes36`: Kaggle CPU (4 vCPU AMD EPYC 7B12, AVX2), commit `13cad72`, test-compute's
off-AWS harness, $0. The run was 120 pairs × 24 settings plus 4 reseeds of R; the PCA was fitted on scene 1
(623 KF, 637,952 rows, explained variance 0.703 at 32 components and 0.933 at 64). Evidence:
`evaluation/runs/collaboration/wire_coding_20261024/` (`raw_kaggle/`, `processed/`, `scripts/analyze.py`, `figures/`).

**Pre-registered checks.**
- **Precondition: pass.** R (int8/1024/xyz) reproduces the production verdict on 120/120 pairs, even on AVX2.
- **Seed envelope: zero.** All 120 pairs are seed-stable and R's poses are identical across the 5 RANSAC seeds.
  Every flip below is therefore a codec flip.
- **Control: fp32 fails bar 1.** It has 1 reject→accept per scene against int8: scene 0 pair 0:299-1:53 is GT-true,
  scene 1 pair 0:21-1:63 is GT-false. Today's int8 is already one verdict per scene from the uncompressed features.
  By D233, nothing is adopted without a separate decision.

**Per coding (D233 bars; R wire = 135.70 kB/KF; each row at the other factor's today value):**

| Coding | wire kB/KF (vs R) | Codec flips s0 / s1 (r→a, a→r) | Median inlier ratio | D233 |
|---|---|---|---|---|
| fp32 (control) | 515.0 (+379) | 1,0 / 1,0 | 1.00 / 1.00 | not adopted (control) |
| int4 | 79.4 (−56.3) | 1,0 / 1,0 | 0.996 / 1.00 | not adopted |
| bin1 | 33.6 (−102.2) | 1,0 / 1,0 | 0.995 / 0.996 | not adopted |
| pca64 | 77.6 (−58.1) | 1,0 / 0,0 | 1.00 / 1.00 | not adopted |
| pca32 | 47.9 (−87.8) | 2,0 / 0,0 | 1.00 / 0.997 | not adopted |
| z32 | 127.2 (−8.5) | 0,0 / 0,1 | 1.00 / 1.00 | not adopted (bars 1 and 3) |
| disp16 | 126.2 (−9.5) | 0,0 / 0,0 | 1.00 / 1.00 | not adopted (bar 3) |
| kp16 + disp16 | 122.8 (−12.9) | 0,0 / 1,0 | 1.00 / 1.00 | not adopted (bars 1 and 3) |

- There is no combination row, since no coding was adopted. Pose error is reported in `coding_verdicts.csv`. Scene
  0's medians are ~88-90°: most of its accepted edges are D176's aliased false edges (R: 14 GT-true of 30 accepted).
- K 512 is not adopted (D235: job 70's knobs row).

**Post-hoc, report-only** (`processed/posthoc_vs_fp32.csv`; written after the control failed and NOT part of the
rule):
- **int4 and bin1 give exactly fp32's verdict on all 120 pairs.** Their only differences from int8 are the two pairs
  where int8 itself differs from fp32. On this sample, 1-bit descriptors (33.6 kB/KF, −75 %) are as faithful to the
  uncompressed features as int8 (135.7 kB) is, or more faithful.
- PCA-64 differs from fp32 on one pair: it rejects scene 1's GT-false edge. PCA-32 also accepts a GT-false edge
  (0:13-1:6, the dangerous direction).
- The lossless z32 flips a scene 1 pair, though its points differ from xyz only in the last float bit. A 0-flip bar
  on this sample sits below its numeric resolution: borderline pairs flip at 1e-7 perturbations, independently of
  RANSAC's seed.

**Decision (per the rule).** No coding is adopted under D233. Anything else needs a new pre-registration with
fp32 as the reference and a flip tolerance justified by the 1-ulp floor above. That is a new open, so it waits
until after the 2026-10-24 feature freeze.
## 2026-10-24 — D123–D124 (applied 2026-10-24, signed off; 2.x SLAM's tests): the arm64 GravityPgo divergence is a test-fixture defect

**Finding** (`evaluation/runs/arm64/20261023T233147Z-cpuarm-1e25b3/README.md`, commit 1647d6b). Graviton's 2.71 deg
against x86's 0.62 deg is not a numerical difference.
- `Make()` builds each prior's noise as `Vec3(nz(gen), nz(gen), 0)`. The order in which C++ evaluates function
  arguments is unspecified, so GCC x86-64 draws y first and aarch64 draws x first. The two architectures solve
  different problems.
- x86 forced to aarch64's order reproduces arm64 exactly (2.7104 deg, tilt 3.1702, seeds 14/100).
- arm64's path is clean: converged, perturbation-stable, HAL ops bit-exact, and the same answer from a scalar LM and
  with Eigen vectorisation off. **The arm64 path is not wrong; the COOL Arm-vs-Arm claim is unaffected.**

**D123: brace every multi-draw RNG initialiser in the tests.**
- 44 sites in 13 files (`sequencing_proposal.txt`, a dry run); none in `cpp/`.
- `{a(g), b(g)}` is evaluated left to right by rule ([dcl.init.list]), even when it resolves to a constructor. Every
  architecture then draws the same noise (aarch64's present order).
- x86 test values change wherever the old order mattered: those tests' expectations were x86-order artefacts and
  must be re-checked on both architectures in one run.
- Add a CI grep that fails on the pattern (`Type(f(g), f(g)…)` with ≥ 2 draws).
- Not applied at first: the files are 2.x's, and a scripted rewrite of them was rightly rejected.
- **Applied 2026-10-24** (`tests/**` edits approved, including the outcome below).
  - **The 44 sites, plus 19 more** found by the CI check, which the dry run had missed:
    - generators named `rng`;
    - two braced vectors as separate arguments of one call, e.g. `p.Integrate(Vec3{…}, Vec3{…})`;
    - operands of one operator, e.g. `RandomAxis(g) * ang(g)` (an overloaded operator is a call) and
      `u(rng) < Per(… g(rng) …)`.
    These are fixed by braces, one draw per declarator, or `RandomRotation()` in `test_lie`.
  - **The check:** `infra/scripts/check_rng_sequencing.py`, registered as the ctest
    `RngSequencing.NoUnsequencedDrawsInTests`; 0 sites in `tests/`. It treats `?:` and control headers as
    sequenced. It was validated by flagging all 44 original sites on the pre-D123 tree.
  - **Local x86 ctest: 695/696, with 1 disabled.** `VisibilityGraph.ConvexGraphMatchesAllVertexReferenceAndBeatsLattice`
    is quarantined because the resequenced map shows "FAR ≤ lattice" is FALSE on trial #9 (37.40 vs 33.24 cells;
    D116 amended above; OPEN).
  - **Values that moved, old → new.** Every printed value of the touched suites, before and after, is in
    `evaluation/runs/arm64/d123_local_x86/values_diff.txt`. Two are evidence of the 2.5 back end and must be cited
    with the new numbers:
    - **planted-outlier certificate (D74):** the stationary points strictly worse than truth go **0 → 2**, all not
      certified. "Certified out of 10" stays 6/6/6, and after GNC 10/10. The reported seed changes 0 → 3
      (λ_min −1.1865e4 → −1.3414e4);
    - **GNC alone on 3 random inter-agent loops:** accepted **1 → 0**, against the stated "a lone link is unverifiable
      without redundancy".
  - **Other moves, all still passing:**
    - the outlier prior moves its keyframe 1.85 → 1.82°;
    - shared-gravity improvement 68 % → 63 % (2.77/0.90 → 3.04/1.13°) and 10 % → 9 %;
    - degeneracy NEES 5.95 → 6.09 (54.69 → 58.15 far);
    - near-miss PCM/GNC precision and recall by ≤ 0.006;
    - GNC near-miss acceptance 143 → 149 of 199;
    - approxPolyDP clipping 53 → 49 of 200;
    - two-lap ATE 0.086 → 0.082 m;
    - the rest are timings or the last digits.
  - **Re-baselines** on x86 (ci_aws) and arm64 are queued at this commit.

**D124: GravityPgo's recovery test asserts something its fixture cannot deliver; redesign it, don't loosen it.**
- Measured over 200 seeds (`conditioning.txt`), the fixture (200 keyframes, odometry rotation σ 0.01 rad per step,
  priors every 10 keyframes at σ 2.3 deg, a 1.5 deg reference error) passes "gerr < 1 deg and tilt reduced" on
  14–17 %. Median gerr is 2.0 deg.
- Denser priors barely help: every keyframe at σ 1.0 deg gives median 0.80 deg and passes 61–63 %.
- Reason [ASSUMPTION, a rough random-walk smoothing estimate, not derived in a test]:
  - with only the anchor fixed and the tilt random-walking at 0.57 deg per step, gravity is observable mainly through
    the priors near the anchor;
  - that gives σ ≈ 0.7 deg per axis, a median of about 0.85 deg, matching the 0.80 measured;
  - so the estimator looks about as efficient as the fixture allows. The bound, not the solver, is wrong.
- **Proposed (recommended): a consistency test.**
  - Over N = 50 seeds, compare each seed's gerr with the solver's own 2 × 2 marginal covariance of ĝ, taken from H
    at the optimum.
  - Assert that the mean NEES lies inside the 99 % χ²(2N)/N band. The priors are drawn with the noise the solver
    assumes (generation σ = s/√info_scale).
  - This tests what the feature claims ("the 2-DoF variable recovers the reference error as well as the information
    allows") and holds on any architecture.
- **Alternative:** a better-conditioned fixture plus a quantile bound. It needs odometry rotation noise small enough
  that P(gerr < 1 deg) is at least 0.95, checked over 50 seeds before it becomes the bound.
- Either way, the "tilt RMS reduced" check stays per seed.

**D124 validated before it is written** (2026-10-24, x86, `gpgo_diag nees 50`, `nees_x86.txt`). The NEES
form was approved; the Bar-Shalom test, as in D178.
- The solver's ĝ marginal is the gravity block of H⁻¹ at the optimum, by an LDLT solve on the two gravity columns.
  `MarginalCovariances` returns pose blocks only. σ ≈ 2.0° per axis for this fixture.
- Mean NEES over seeds 0–49 lands inside the 99 % band [1.347, 2.803] under both draw orders:

  | prior noise drawn at | aarch64 order (D123's) | x86 order |
  |---|---|---|
  | σ/√0.6, the noise the solver assumes (recommended) | 2.53 | 2.30 |
  | σ, today's draw | 1.84 | 1.67 |

- **Amendment:** "tilt reduced" fails on 1–4 of 50 seeds per configuration (it holds on 46–49 of 50), so it cannot
  stay a per-seed assertion. It becomes the tilt RMS over the 50 seeds below the odometry's, plus a reduction on
  ≥ 90 % of seeds.
- After D123 the seeds are fixed, so pass/fail is deterministic and identical on both architectures.
- **Applied 2026-10-24:** `GravityPgo.ReferenceGravityErrorIsRecoveredAsWellAsTheInformationAllowsAndTiltShrinks`
  replaces the single-seed bound.
  - Priors are drawn at σ/√info_scale; `Make(…, gen_scale)` defaults to 1 for the other tests.
  - Local x86: mean NEES **2.526** in [1.347, 2.803] (validated 2.53); tilt RMS 8.47 → 3.78°, reduced on 46/50.
## 2026-10-24 — D117–D122 (proposed; §49 preflight, docs only): the 4.3 time-axis loss is turning — charge the mover's executed time

Full record: `docs/design/next-best-view.md` §15; literature the literature notes `early_coverage` (archive/dev-history branch); evidence
`evaluation/runs/planning/nbv_gate/{time_budget,turn_back,turn_episodes}.{py,txt}` (the 36 gate runs' GT; no new run).

**Cause, measured.**
- Ariadne turns in place for 45–46 % of the mission and moves for 31–32 % (coverage-greedy: 31–42 % and 44–48 %), at
  1.6–1.7 rad of yaw per metre against 0.7–1.1.
- About half of that turning (47–50 %; 67 % in maze_short's first 60 s) is "face the waypoint, drive, turn back to the
  view's yaw", because every view faces its target and views are sampled around it.
- U(a) charges metres only, while one radian at ω_max 0.44 rad/s costs the time of 1.4 m.
- No single turn exceeds 180° (the wrap is correct).

**Decisions (proposed).**
- **D117:** C_time = the duration of `PlanCommand` over the polyline `Route` will send, from the current heading.
  This is the mover's own profile, on the planner's clearance [PROXY]. Predicted against realised time is measured
  per row.
  - §4's "time ∝ path" collapse is dropped for this rig, and the arch's separate C_motion/C_time terms return, with
    weights summing to 1 at t = 0.
  - [PAPER] Schmid 2020 Eq. 6 (execution time), FUEL Eq. 1 (time with yaw; max() there because a quadrotor yaws
    while it flies), TARE Eq. 4 (a per-stop penalty for an in-place turner).
- **D118:** every kept view also gets an arrival-heading twin (own IG, no final turn), and U chooses. [DESIGN]: no
  surveyed planner keeps the arrival heading. FUEL's alternative (≤ 15 views at ≥ 0.8× coverage) is cited.
- **D119 (defect, every Phase 4 row):** a command cut at `max_length_m` carries no final yaw. Today the rig turns to
  the view's yaw mid-route.
- **D120:** the trace logs each candidate's normalised IG terms, C_path, C_time and turn time.
- **D121 (reserved):** efficiency over the local tour, Schmid's v_GN. Our linear `g − αc` was the worst form in their
  Table V. Not part of this change.
- **D122 (reserved):** one per-voxel gain scale (Schmid Eq. 5) instead of per-term p90. Decided on D120's log.
- **D125 (defect, every Phase 4 row; a prerequisite of D117):** a view at the agent's own cell runs as an in-place
  turn, not a hold.
  - In ci_tiny seed 2 the same at-cell view was chosen for six barriers. `Route` returned an empty path, so the
    agent held for 30 s.
  - That one livelock carries most of ci_tiny's 40–60 s loss: on seeds 0 and 1 Ariadne is best at 60 s
    (0.909 / 0.900).
- **D126 (reserved):** TARE's commitment rule plus waypoint extension. Its L has to be re-derived for this rig first.

**Pre-registered ablation (§15.5).** 7 rows × 2 scenes × 3 seeds = 42 lockstep runs:
- the four Phase 4 baselines re-run with D119 and D125;
- ariadne_v0, ariadne_t (D117) and ariadne_ta (D117 + D118);
- the 1.8 row reused under a replay identity check.

The claim is unchanged ("≥ every baseline at every T"), with a non-regression bound at 200 s and no tuning after
the runs. Spend needs approval.

## 2026-10-24 — D125/D119 implemented; D151 amended (NaN = turn here); §15.5 pre-registration amended before any run

**D151 amendment (approved on design and for the mover and the live loop).** A waypoint whose x or y
is NaN means "the rig's current position": `PlanCommand` adds no face-turn and no move, only the final turn when the
pose has a yaw.
- Why a sentinel and not a finite goto: the mover plans from its GT pose, and the planner's estimate is centimetres
  away. A finite waypoint there makes the rig first turn to face that offset, in any direction up to 180°, then
  creep, then turn to the yaw. A tolerance on short moves would be a new constant. The sentinel needs none.
- Every finite command is unchanged. A waypoint 1e-12 m away still takes the coincident rule (the final turn only),
  and one 1 cm away still faces, moves and turns (`test_nbv_time`).
- NaN survives the wire: `ros_cdr::PoseArray` doubles end to end, through the link, the bridge and gz.
  `ClearanceAlong` is never reached, since there is no move.
- The trace never prints the NaN (the live loop's condition): a turn-here command is `"kind":"turn"`, `"waypoints_xy_m":[]`,
  `"turn_yaw":<rad>`.

**D125 as built.**
- `Phase4Planner::Route` returns one (NaN, NaN) waypoint with the view's yaw when A* reaches the goal at the start
  cell.
- An unreachable goal still routes empty, and the loop holds.
- With `face_frontier: false`, an at-cell view has nothing to execute and holds, stated in the trace.
- Tests (`tests/unit/test_nbv_time.cpp`):
  - the livelock geometry (eye 2 cm from the agent, inside its cell) routes as one turn-here waypoint;
  - from 8 poses 3 cm around, it is exactly one `kTurn` of `PlanProfile(|Δyaw|)`;
  - closed loop in miniature: integrating the frame the turn sees collapses the view's coverage (> 5×), so the view
    stops winning.

**D119 as built.**
- `Route` returns a NaN yaw when it cuts the polyline at `max_command_m`, and the loop then sends no final yaw.
- An uncut command keeps the view's yaw (test).

**§15.5 amendments, written before any run:**
- (a) **Replay identity check.** The "byte-identical to d59ee3d with every key off" wording contradicts the
  always-on D119 and D125, so it was false by construction. The check is now:
  - with every new key off, decisions and commands are identical to the base, except the enumerated records where a
    cut command or an at-cell view occurs;
  - the fixes fire exactly there.
  The base is now the N-agent planning line after the main merge (`9efbdba`: main 4a12b50; the D227 lens-off is NOT in it), not
  d59ee3d.
- (b) **Rig.** 8 rows × 6 = **48 development runs** (`configs/mission/ablation_4.3/runs_48.jobs`).
  - The 1.8 row is re-run rather than reused, because the base commit changed (main merged into the tree; one commit is pinned for all 48 runs).
  - A run counts only if every lockstep barrier is answered (`answer_timeouts` = 0) and the D152 reorder has no
    timeouts.
  - The runs were first routed off AWS (the Kaggle CPU queue). That was declined: no docker or gz on its kernels today, and
    llvmpipe on 4 vCPU risks answer timeouts. So the runs go on the gate's own rig (AWS c7i, llvmpipe, slotq band
    30+), with spend approval.
  - Measured gate wall time: median 13.1 min per 200 s run (34 logged jobs), so ≈ 11–12 c7i.4xlarge spot-hours.
- **Replay identity check, done before any run** (`evaluation/runs/planning/nbv_ablation/replay_identity/`). The local
  20 s ci_tiny recording (a1p `build/gate/local/repro_a`) was replayed with the new binary.
  - **Keys off:** all 8 decisions have identical choices and commands. The only differences are new trace fields
    from main and CBBA wire bytes 920 vs 904 (main's appended D175/D171 wire fields, not this change).
  - That recording has no cut or at-cell command, so it does not exercise the D119/D125 fixes themselves; the unit
    tests do.
  - **Keys on (ariadne_ta):** the run completes and every trace record parses. Arrival twins win 7/8 decisions;
    0 unroutable.
- (c) The gated `ariadne` row (36-run gate) is compared to ariadne_v0 only as context: the base commit and the
  fixes all differ.
- (d)–(h), also before any run (next-best-view.md §15.5):
  - predicted-vs-realised C_time only on rows with `time_model: exact_command`;
  - plan cost via `lockstep.jsonl` `wall_paused_s`, which includes the command round-trip;
  - the first job, ci_tiny seed 2 ariadne_v0, is an abort check: turn commands accepted with one segment, no answer
    timeouts, no hold at a won at-cell view;
  - D119 evidence comes from the mover logs;
  - one pinned commit for all 48 runs.
## 2026-10-24 — D176: 3.2 sweep (fixed) shows perceptual aliasing wins PCM under the byte budget; place-grouped PCM pre-registered

**Measured** (sweep rerun `20261023T191248Z-cpux86-eee456`, with the D171 config-writer and calibration fixes; gate
`build/collab_gate_eee456`). Outcomes PASS (31 incomplete, 5 consistent, 0 corrupt), but the geometry fails:
- **Scene 0, every budgeted row (greedy/spectral x 25/50 %, 3 seeds) and the campaign: merged ATE 3.77-3.81 m.** The
  unbudgeted similarity arm on the same scene: 0.017-0.037 m. Scene 1: 0.007-0.010 m everywhere.
- Cause, from edges.csv + the pass-A stores' GT: a perceptual-aliasing group. On agent 1's side it is 1-6 query
  keyframes within 0.5 m (one place); it verifies against 40-60 of agent 0's keyframes (1.6-1.8 m span), every edge
  ~5 m / ~100 deg off (the 6 m lattice period: worldgen `cell_m: 6.0`). 40-154 such edges per run, mutually consistent.
  The true loops (15-25 edges, ~3 m span on both sides) were also verified. **PCM's maximum clique chose the larger
  false clique and rejected the true one in every scene-0 budgeted run** (D171's smoke called this "no competitor";
  here the competitor existed and lost on edge count).
- The budgeted selection admits more of the group (high retrieval score: median 0.905-0.919 bad vs 0.92-0.96 good,
  overlapping), so the spectral-vs-greedy dominance rows on scene 0 compare two corrupted maps: not evidence either way.
- The cross-scene PnP floor did not transfer: scene 1's floor (72) applied to scene 0, whose own floor is 169. Reported,
  not switched (D171 pre-registration stands).
- Byte table FAIL (worst 0.86): match_request predicted ~6x low (e.g. 2992 vs 18760 B); match_response ~15 % high in
  some runs. Unexplained yet; the accounting is checked against the ledger before any change (see below).

**Decision (pre-registered before any rerun): place-grouped PCM, "one place pair, one vote".** Edges are not independent
evidence when many come from one place; ORB-SLAM3's place recognition likewise checks a candidate against several
keyframes before merging maps [PAPER: Campos et al. 2021, §VI; verify the exact rule when citing], and DBoW2 uses temporal
consistency. Before PCM, the broker groups inter-agent edges by (agent_a, place_a, agent_b, place_b), where a place is a
greedy cluster of an agent's keyframes in its OWN odometry frame (the snapshot it sent; id order; a new place when the
keyframe is farther than `radius_m` from the current place's first keyframe). PCM then runs over one representative per
group (the most PnP inliers; ties: lowest ids); other members do not enter the solve.
- `radius_m: 1.0`: one corridor width (worldgen corridors 1.0-1.4 m: consecutive keyframes in one corridor segment see
  the same structure) and well under half the 6 m lattice period (aliases sit a period apart, so they never share a
  place). Not tuned on scene 0.
- Config `pcm_place_grouping: {enabled: false, radius_m: 1.0}` in collab_v2.yaml; default off, so the 3.2 smoke and
  every earlier run stay byte-identical.
- **Pass criteria (fixed now):** rerun B2 + C with it on (stores reused). Scene 0 budgeted rows merged ATE < 0.10 m in
  >= 5 of 6 cells (arm x level, seed-mean); scene 1 rows not worse than now by more than 10 % (or 2 mm); no run
  corrupt. Failing either is reported as a negative result and the alias group stays a documented limitation.
- Live (D174): the known entry poses give a second, independent guard: an inter-agent edge whose implied alignment
  disagrees with the entry-pose alignment by more than the VO drift bound is rejected before PCM. Designed separately
  (D177) with its bound from measured VO drift, not from this failure.

**Not 3.5 done:** 3.5 still needs repetition-operator worlds (§5.5), the §17.3 diagnostics (ROC/PR of retrieval score
against GT, false-closure probability) and the degeneracy-gate-before-PCM ordering test.

**D176 result (2026-10-24, NEGATIVE, reported as measured).** Pass B, scene 0, greedy 25 %, seed 1, stores of
`0345a5`, grouping on vs off: merged ATE 3.7837 m vs 3.7835 m. The rule took effect (inter-agent edges entering the
robust stage over the run: 89 vs 1441) but the false group still won. Why: agent 0 passed the aliased spot several
times (its keyframes 32, 50, 69, 144, 148, 155 are > 1 m apart in its own frame, and its VO drift is only 0.02-0.03 m,
so they really are separate places); each pass pairs with agent 1's one place and counts as its own vote. "One place
pair, one vote" undercounts nothing on the true side and still overcounts revisits on the false side. Not retuned on
this data. `pcm_place_grouping` stays in the code, default off, as a measured ablation row.

## 2026-10-24 — D177: known-entry gate on inter-agent edges (pre-registered; known starts, decision 2026-10-23)

**Basis.** Agents enter through known openings (drones inserted through windows), so each agent's odometry
frame W_k has a known placement T_World_Wk at entry. An inter-agent loop edge (a, ka) -> (b, kb) with measurement
Z = T_Ca_Cb implies T_Wa_Wb = T_Wa_Ca Z T_Wb_Cb^-1 (the snapshot poses). Against the prior T_Wa_Wb = T_World_Wa^-1
T_World_Wb, the only legitimate disagreement is VO drift accumulated since entry by both agents.
**Measured drift** (pass-A stores of `0345a5`, 2 scenes x 3 seeds x 2 agents, camera pose vs GT aligned at the first
released keyframe): worst 0.11 m and 1.43 deg; worst drift / path length 1.0 % (11-33 m paths).
**Rule.** Before PCM the broker drops an inter-agent edge whose deviation D = prior^-1 implied exceeds
  |D.t| > 0.5 m + 0.05 (L_a(ka) + L_b(kb))   or   angle(D.R) > 10 deg,
with L_k(kf) the odometry path length of agent k from its first keyframe to kf (its snapshot). 5 % is 5x the worst
measured drift ratio, the 0.5 m floor covers entry placement error, 10 deg is 7x the worst measured rotation drift. The
aliasing group (about 5 m / 100 deg, the 6 m lattice) is far outside; drift-sized true loops are far inside.
- Config `entry_gate: {enabled: false, floor_m: 0.5, per_m: 0.05, rot_deg: 10}`, default off; the entry poses come from
  the briefing (live: the runtime's T_W_O[k]; pass B: the scene's agent start poses, i.e. GT of the first released keyframe
  composed with its odometry pose, the same placement an operator records at insertion).
- **Pass criteria (fixed now, same as D176):** B2 + C rerun with it on (grouping off): scene 0 budgeted rows merged ATE
  < 0.10 m in >= 5 of 6 cells; scene 1 not worse by > 10 % (or 2 mm); no run corrupt. Also reported: true edges the
  gate drops (must be 0 on both scenes, else a negative result).
- **Scope, honestly:** this guards the known-starts deployment. It does not solve aliasing between agents whose starts
  are unknown, nor aliases smaller than the drift bound; those remain 3.5 (repetition worlds, §17.3 diagnostics).

**D176/D177 status correction (2026-10-24).** Both were designed without the required preflight (SOTA
literature pass + design review before any design; no ad-hoc incremental fixes after a failed run). D177 is WITHDRAWN as a
pre-registration: not built, not run. The D176 grouping code is parked uncommitted (git stash).
The measurements above stand. The design restarts from a literature survey of perceptual aliasing in multi-robot loop
closure (PCM's large-false-clique failure and its successors, systems in repetitive/subterranean environments, gating
against known initial poses, ambiguity-aware back ends), then an design review, then a pre-registered decision.

**D176 record correction (2026-10-24).** The negative result covers the IMPLEMENTATION tested, not spatial place
grouping in general: places were clustered sequentially in keyframe-id order (a new place whenever a keyframe left the
current place's seed), so a revisit always opened a new place id even when it returned to the same spot. Spatial
grouping in each agent's own frame (revisits merged), with one-to-one support per place pair (CLIPPER-style) and an
ambiguity test (Olson 2009), is the literature's direction for the unknown-start case and belongs to 3.5. Surveys:
scratchpad alias_survey.md / collab_fast_survey.md, summarized in the next decision.

## 2026-10-24 — Preflight for the collaboration redesign (surveys + design review)

Two literature surveys from primary sources (scratchpad `alias_survey.md`, `collab_fast_survey.md`; summarized here),
then design review. Findings that drive D178/D179:
- **PCM states its own failure mode** (Mangelson et al., ICRA 2018, Assumption 2: the measurement system must not be
  "biased toward selecting incorrect measurements over correct ones"); revisits + top-k retrieval are that bias. Its
  successors (group-k consistency, Forsgren et al. IROS 2022 / IJRR 2024; CLIPPER, Lusk et al. ICRA 2021; ROBIN, Shi et
  al. ICRA 2021) still pick the set with most support: none fixes "a large false group beats a small true group".
- **Systems with a known common frame gate candidates by proximity before consistency:** LAMP (Ebadi et al. ICRA 2020:
  fiducials at the entrance, search radius around the estimate) and LAMP 2.0 (Chang et al. RA-L 2022: d_max = α·n_curr
  growing with traversal in the gate-calibrated frame, α = 0.2 m/node). Olson (RAS 2009) gives the principled form:
  Mahalanobis gating with covariance projected along the odometry path ("Dijkstra projection", conservative).
- **GNC needs an outlier-free initial guess** (Kimera-Multi, Tian et al. T-RO 2022: "In multi-robot PGO, there is no
  longer an outlier-free initial guess (i.e., odometry), which is crucial for the strong performance of GNC"). Ours is
  chordal over PCM's output, which seeded the false basin.
- **Bytes:** one side's features travel to the other side, which verifies (vertex cover: Giamou et al. ICRA 2018; Tian
  et al. IJRR 2021; DOOR-SLAM; Kimera-Multi) - our broker fetching both endpoints deviates from the locked
  "vertex-cover sharing". Our 60-130 kB per fetched keyframe is 3-6x D2SLAM's 20.1 kB/keyframe. No survey found an
  information-theoretic bound: we report measured bytes against the vertex-cover minimum, never "the limit".
- **Async + reproducible:** asynchronous multi-robot back ends (ASAPP, D2SLAM/ARock) do not address replay
  determinism; the workable form is asynchronous compute with commits logged on the sim clock (D179).

## 2026-10-24 — D178: known-entry Mahalanobis gate on inter-agent loops + GNC initialised from the entry prior (pre-registered)

**Basis:** LAMP / LAMP 2.0 (known entry frame, traversal-growing proximity gate), Olson 2009 (Mahalanobis gate, covariance
projected along the odometry path), Kimera-Multi (GNC's initial guess). Known starts: decision 2026-10-23.
**Prior.** T_World_Wk for each agent from the mission briefing's start poses (live: the runtime's T_W_O[k]; pass B: the scene's
agent spawn poses, NOT ground truth of a keyframe), labelled in manifests as a known-start prior.
**Residual.** For an inter-agent edge (a, ka) -> (b, kb) with Z = T_Ca_Cb: implied T_Wa_Wb = T_Wa_Ca Z T_Wb_Cb^-1,
prior P = T_World_Wa^-1 T_World_Wb, r = Log(P^-1 implied) in R^6 (the left-multiplicative convention).
**Covariance.** Σ = Σ_a(ka) ⊕ Σ_b(kb) ⊕ Σ_Z ⊕ 2Σ_entry, each transported to the residual's frame by the SE(3) adjoint
(first order). Σ_k(kf): the odometry edges' covariances (their information^-1, in the snapshot) composed along agent k's
chain from its first keyframe (Olson's projection; the chain IS the minimum path without intra-agent loops). Σ_entry:
operator placement, σ = 0.10 m and 2 deg per axis [ASSUMPTION, physical: a hand-placed drone at a window]. 
**Calibration (pre-registered, scene 1 only - never gate data).** NEES of the chain-composed odometry pose against GT
aligned at entry, over every released keyframe of scene 1's stores (3 seeds x 2 agents): κ = mean(NEES)/6; Σ_k ← κ Σ_k
if κ > 1 (the odometry information is overconfident), unchanged otherwise. κ and the NEES histogram are recorded.
**Gate.** Keep the edge iff r^T Σ^-1 r ≤ χ²_6(0.999) = 22.46. Rotation alone of the observed aliases (≈100 deg) gives
(100/σ_r)^2 ≥ 400 for any σ_r ≤ 5 deg, above every χ² cutoff from 0.95 to 0.9999: the decision does not hinge on the
threshold.
**GNC initial guess.** With the gate on, the joint solve initialises every agent's T_G_W from the entry prior instead of
chordal over PCM's accepted set.
**Config.** `entry_gate: {enabled: false, chi2_conf: 0.999, entry_sigma_m: 0.10, entry_sigma_deg: 2.0, kappa: <from
calibration>}`; default off; the 3.2 smoke byte-identical with it off.
**Pass criteria (fixed now).** B2 + C rerun with the gate on: scene 0 budgeted rows merged ATE < 0.10 m in >= 5 of 6
cells (arm x level, seed-mean); scene 1 not worse by > 10 % (or 2 mm); no run corrupt; true verified edges (pose error
<= 10 deg) rejected by the gate <= 1 % on both scenes. Failing any is a reported negative result.
**Scope.** Known starts only. [INFERENCE] decisive against pure one-period translation aliases only up to ~100 m of
travel per agent at 1 % drift; the demo's paths are checked against that. Unknown-start aliasing stays 3.5 (spatial
place grouping per agent frame with one-to-one support, Olson's eigenvalue-ratio ambiguity test, repetition worlds,
§17.3 diagnostics).

## 2026-10-24 — D179: asynchronous collaboration, committed on the sim clock (requirement: no barrier syncs)

**Requirement** (2026-10-24): collaboration "blazingly fast", no barrier syncs, asynchronous. **Semantics:**
- All collaboration compute is asynchronous and never waited on: DNN in the per-agent collab workers; retrieval,
  verification and broker solves on the fleet's own thread.
- The radio stays a single owner on SIM time (D175's coordination and its D218 replay proof depend on it). Every
  asynchronous result (a #ckf record, a #fresp, a verification, a solve) is committed at the sim tick at which the fleet
  thread observed it, and the (tick, event) pair is logged. Live never waits for a result; replay applies the logged
  events at their logged ticks, so replays equal the live run they log (record/replay is the substrate, arch §0.2).
- The runtime's `Step(t)` enqueues tick t and returns. With collab on, the fleet thread owns the radio nodes; D175 sends
  are sim-timestamped entries in a queue into the fleet thread, and coordination deliveries come back through a queue
  the runtime drains at its barrier step 2. **Invariant 1:** the drain order is per tick, FIFO per sender, ticks in
  order, and replay reproduces exactly that order from the log. **Invariant 2:** the published snapshot {T_O0_O,
  Linked, Telemetry} is swapped atomically as one value; the planner reads it lock-free.
- Replaces D174's lockstep (watermark waits, feature replies at ask + feature_latency_s).
- **Amendment (2026-10-24, the runtime's request):** the snapshot also carries, per fleet member k, what k's radio node knew at
  that tick: `Component()` (broker, members, accepted epoch), `LastHeartbeat(j)` for every peer heard, and
  `AckedPatchVersion(j)` for every other member (`CollabSnapshot::agent[k]`). The D175 fork-join reads these from its
  one per-barrier snapshot instead of from nodes the fleet thread owns (a data race under D179). The values are frozen
  at publish, and the published version is logged, so replay serves the same values. Reads within a barrier all see
  one tick's state. A read never reflects the runtime's own same-barrier enqueues, which apply on the fleet thread
  after it.

## 2026-10-24 — D180: protocol re-sends are overhead in the byte table (pre-registered before the next byte run)

**Finding** (3.2 rerun `20261023T191248Z-cpux86-eee456`): match_request was measured about 6× above prediction.
`RetryRequests` re-sends an unanswered request every `request_retry_s` (10 s) under the same request id, and the
transport logged each re-send as a new first transmission. The owner answers each retry that is not still deferred,
so a duplicate MATCH_RESPONSE (about 136 KB) is also logged as first transmission; this is the likely cause of
match_response running about 15 % over. The predictor prices one exchange per request, which is what the spectral
policy's `gain/predicted_cost` needs.
**Rule:** a MATCH_REQUEST or MATCH_RESPONSE whose (receiver, class, request id) the sender has already first-transmitted
is a protocol re-send. Its ledger row is flagged `resend`. It is counted in its own overhead column
(`wire_resend`), next to the ARQ retransmit and resync columns, and it is excluded from first-transmission bytes and
from the ≤ 15 % prediction-error gate (I6). It stays in the "all bytes" totals and the hollow points of
ate_vs_bytes, and the table reports it per class, so the cost is never hidden. Only these two classes are covered:
their key is a unique request id, and a repeat is the same content by construction.
**Not changed by this rule:** the predictor, the retry and give-up timers, and how the owner answers. Whether duplicate
responses should be suppressed while the first response is still in flight is a protocol question for the one-way
verification redesign (next bytes item), and is not settled here.
**Pass/fail for the next byte-table run:** the existing I6 gate (each gated class ≤ 15 % error on first transmission),
unchanged.

## D225 addendum (2026-10-24): the deterministic gate guard + the N-generic audit

- The barrier predicates are factored into runtime/live/lockstep_gate.hpp (AllReached / AllDoneShort /
  MinProgress, templates over any lane range) and unit-tested (LockstepGate, 4 tests): ANY lagging lane holds
  the barrier at N = 3 and 4, and PreD225ShapeTripsHere keeps the old lanes[0]&&[1] shape as a documented trip
  target - it fails deterministically, no timing involved (decision: the race was not reproducible in replay,
  whose unbounded all-N answer wait hard-syncs the lanes; the on-instance replay x2 remains the e2e tripwire,
  test_barrier_n3.sh the forward lane-skew invariant).
- N-generic audit of the live lane (ariadne_live.cpp, live/*.hpp): the Reorderer's watermark is min over ALL
  timed topics (built from every mission agent); the answer wait loops all NA; remaining "both"/"= 2" hits are
  constructor DEFAULTS always overridden with NA, prose that is genuinely two-valued, or stale comments (two
  LLM-chooser comment lines still say "both agents" over N-generic code; cosmetic).
## 2026-10-24 — D178 amendment: how "true edges rejected ≤ 1 %" is scored (fixed before any gated sweep result is read)

The sweep staged at 4459d57 (`20261023T213707Z-cpux86-e31421`) logs only the gate's totals (tested, rejected), so the
pre-registered true-edge criterion cannot be scored from it. **Fix:** `ariadne_collab_sim` now writes
`entry_gate.csv`: one row per inter-agent edge the gate ever tested, with times tested and rejected, and the last
squared Mahalanobis distance and decision. Logging only; the gate's inputs and outputs are unchanged.
**Definitions:**
- A true edge is one in `edges.csv` with `reason == verified` and `pose_err_deg ≤ 10` (D171 §5's label, same as the
  precision row). Edges are matched as an unordered (agent, keyframe) pair.
- "Rejected" means rejected at its LAST test (the latest solve that saw the edge, with the longest odometry chain).
  "Ever rejected" is also reported but is not the criterion.
- A true edge the gate never tested (ungated) does not count as rejected, and the ungated count is reported.
- The criterion is pooled per scene over every gated run (arms × budget levels × seeds): rejected true edges / tested
  true edges ≤ 1 %, on each scene.
**Procedure:**
- The ATE criteria are scored on the e31421 sweep as staged.
- The true-edge criterion is scored on a logged rerun of the same gated cells (same stores, seeds and configs, with the
  logging build).
- Every rerun cell must reproduce e31421's merged ATE and first-transmission bytes exactly. Bytes are compared after
  folding D180's `wire_resend` back in, since the rerun's build includes D180. If any cell differs, the rerun is not
  the same experiment: the true-edge criterion is then reported UNSCORED, with the reason.

## 2026-10-24 — D181: endpoint (vertex-cover) verification measured before design: no 2-agent payoff on budgeted rows; deferred

**Question:** would moving verification to one endpoint, with the vertex-cover direction choice of Giamou 2018 and Tian
2021, cut the bytes the gate rows move? It was measured from existing data before any design was written (preflight;
design review).
**Method:** sweep `20261023T213707Z-cpux86-e31421`, every run's attempted inter-agent pairs (`edges.csv`). With N = 2
and broker 0, today's scheme already moves features one way: it fetches each distinct agent-1 endpoint. Per agent
pair the exact optimum is a minimum bipartite vertex cover (König: = maximum matching). Its weighted form is an LP
with integral optima, solvable by `cv::solveLP` (OpenCV-native check). Keyframes were compared at unit weight.
**Result:**
- **Budgeted gate rows (greedy and spectral, 24/24):** 0.0 % saving. Today's fetch set already is the minimum cover,
  because the policy's shared-keyframe pricing selects candidates that share fetched keyframes.
- **Elsewhere:** the unbudgeted calibration rows save 2.6–4.3 % on scene 1 and 55–56 % on scene 0, campaign
  `ideal_s0` saves 59.1 %, and the other campaign rows and the similarity arm save 0.
- **Pooled over all runs:** 1450 keyframes moved against a minimum of 997, a 31.2 % saving, almost all of it in rows
  that do not count toward I6.
**Decision:**
- The protocol change is NOT built now. Its only gate-relevant benefit is the third-party saving at N ≥ 3 (a non-broker
  pair fetched to the broker on both sides), which no Phase 3 scene exercises.
- It is re-measured when 3-agent stores exist (the 3-agent live maze runs), with the same script and pre-registered:
  "build if the selected rows of the 3-agent demo move ≥ 20 % more keyframes than their minimum cover".
- The speed side (LightGlue serialized on the fleet thread) is decided by the AWS pipeline profile. If it is the bottleneck,
  the remedy is a verify pool with D179-logged commits, which needs no protocol change.
- Bytes work moves to the 3.6 compression rows and to diagnosing the match_response prediction miss.

## 2026-10-24 — D182: ALIKED dense-L2 graph rewrite (l2conv): adoption rule, pre-registered before the measuring run

**Finding (c7i run `20261024T001208Z-cpux86-6ba75f`):**
- ALIKED-1280 stops scaling at about 8 OpenCV threads: 843 ms at 8, 829 ms at 16, cpu/wall 3.8.
- Its "Normalize" (F.normalize on the dense 128×736×1280 map: ReduceL2 → Clip → Expand → Div) runs as one serial
  cv::dnn layer, about 535 ms at 8–16 threads (64 % of the forward).
**Change:** an exact ONNX graph rewrite, not an exporter change (`scripts/l2conv_rewrite.py`, 282e1da). The sum of
squares becomes Mul plus a 1×1 conv with a ones kernel, then Sqrt, then Div by broadcasting; the 470 MB Expand copy is
dropped.
- Local fidelity: keypoints and scores bit-identical, descriptors within 1.3e-6.
- Speed at 8 threads, about 300–350 ms: [PROJECTION].
**Adopt only if all hold on the rerun (`70-A2-perf-x86b`):**
- (i) 0 accept/reject flips on the 120 real verify pairs, against the 1280×720 K1024 reference;
- (ii) keypoints bit-identical and descriptor max abs diff ≤ 4.9e-6 (the measured thread-count effect), on every
  keyframe of those pairs;
- (iii) measured p50 at 8 threads improves ≥ 1.5×.
If (iii) misses but (i) and (ii) hold, it is reported and decided separately. If (i) or (ii) misses, it is not adopted.
On adoption: the model sha changes, every manifest names the rewritten graph, and the rewrite script plus the
input and output ONNX hashes are committed together.

## 2026-10-24 — D227 (D41 amendment): the synthetic lens is OFF in the live runtime

**Order (2026-10-24):** turn the D41 read-time lens distortion off for live/demo/POC runs. The
per-function profile (job 44b/45 line) measured it as most of the ~58 ms/frame ingest cost, about 50 % of the lane,
and it is optional sim realism, not a claim-bearing operator. No baseline reruns: every run from the next one on is
the new baseline.

**Mechanism (this commit):**
- `LiveConfig.lens` (bool, **default false**), yaml key `lens:` in the live config. All seven
  `configs/mission/live_*.yaml` now carry an explicit `lens: false`. `lens: true` restores the D41 operator exactly.
- The toggle reaches both estimator arrangements: in-process `LiveSimSource` directly, worker processes via a new
  internal `--lens on|off` flag (parent passes it from `LiveConfig.lens`; absent = off).
- Off: `SimFrameAssembler` gets a null lens map and skips `ApplyLens` (the resampling never runs — the frame stays
  the clean pinhole render); `LiveSimSource::Calibration()` reports the RENDER intrinsics with **D = 0** (the camera
  the estimator then actually sees); `CameraConfig()` says `+lens(off)` so every manifest and digest names the state.
- Replay honours it: replay consumes `inputs.mcap` (clean pixels) through the same `LiveSimSource`/worker path with
  the same live config, so a lens-off replay of a lens-on recording is valid for perf (the RTF measurement) but its
  trace is a new baseline, not comparable to the live trace.
- Untouched: `SimSource` (dataset/eval bag path, D37/D41 regression rows), `ariadne_sim_vo`/spool tools, and
  `ariadne_sim_calib` (which already had `--lens auto|on|off`; its lens-off truth is the raw render). The D41
  operator, its tests and its digest stay — this is a default flip scoped to the live runtime, not a removal.

**Evidence (local, ma_smoke3 replay of the 20 s 3-agent lit maze_demo_0 recording):**
- `lens: true` replay trace byte-identical to the pre-change reference (sha256 f732c08056b1f937…) — the toggle-on
  path is the old pipeline, bit for bit.
- default (off) replay ×2: identical to each other (deterministic), trace sha256 21a51e0d5d3660ba… — the new baseline.
- Manifests: `camera_config: rig_stereo_v2+lens(D41)` (restored) vs `rig_stereo_v2+lens(off)` (default).
- (artifacts: `build/live/ma_smoke3/.../replay_lenson`, `replay_lensoff{1,2}`.)

## 2026-10-24 — D210–D229 registry backfill: the Phase-8 block, recorded (an audit found STATUS claimed them, DECISIONS lacked them)

D210, D228, D229 are UNUSED (free numbers in the block). D230/D231 are recorded above. Each entry: decision,
why, evidence.

### D211: maze_dark — the arch §42 demo world (commit 663e4ce)
Dark variant of the demo maze, lit only by the agents' own spot lamps. Built so the dark demo is a scene swap, not a
code path. Evidence: `build/live/scenes/maze_dark_0/`. NOTE: all dark/lighting/dust work is PAUSED
(see D217); the world exists, nothing consumes it yet.

### D212: three agents in worldgen (commit 9659abe)
Entrances to 3, greedy-maximin start assignment, N-agent scheduling, a third mover board. Why: the multi-agent demo
needs N=3; worldgen was the first hard-coded-2 to fall. Evidence: mission_live.json of any 3-agent scene;
`live.sh demo-mission` presets.

### D213: the live loop is N-agent (commit 71df9aa)
Every lane/choice/command path in ariadne_live sized by the scene's agent count; placeholder planner seam kept.
Why: D175. Evidence: 2-agent oracle A/B digest 3d10d011018c42d9 byte-identical pre/post (multiagent:false untouched).
CAVEAT recorded honestly: two "<2" leftovers survived this change and were found later — the barrier gate (fixed,
D225) and the operator-directive filter (fixed, commit 780f122).

### D214: a live recording is a sim_record recording (commit 0b02fef)
SimSource tools (sim_vo, spools, frame dump) read live run dirs directly (live_to_rec). Why: one toolchain for
datasets and live runs; the splat training spools come from exactly what the estimator consumed. Evidence: the
ma200-family spool builds; `cpp/runtime/datasets/` readers unchanged since.

### D215: the demo console (commits 48146ec + 5fff283)
GT ghost arrows, per-agent camera panes (/api/cam/{k}.jpg + .json), reroute events, N-agent UI; then the feature
overlay (StereoVo::KeyframeObservationsLeft -> #kf CV_32FC2 row -> snapshots/cam_<k>.json -> canvas `featovl`).
Why: the judges see what the estimator sees. Evidence: overlay validated 491/371/298 pts on 3 agents; trace digest
3d10d011018c42d9 unchanged (display-only).

### D216: demo-mission knobs (commit 55fae85)
`live.sh demo-mission PRESET SEED [AGENTS] [TARGET] [RTF]` + `python -m dataset_tools.worldgen demo-mission`:
user-chosen entrance subset and target, world untouched; check_demo_target floods the ESDF at ρ_responder (invariant
4). The live.demo block in mission_live.json is authoritative. Evidence: default recompose byte-identical; knob runs
in `build/live/`.

### D217: DEFERRED — the dark/lamp demo toggle
Decision (2026-10-24): pause ALL dark/lighting/dust work until the multi-agent POC runs. The number is
reserved for the toggle; no code exists. Revisit after the POC demo is green at real time.

### D219: live_aws.sh knobs (commit 149f01c)
LIVE_AWS_PROFILE (cpux86 = c7i CPU slots), LIVE_AWS_MAX_MIN, and cmd_run bakes the caller's
ARIADNE_LOCKSTEP/LIVE_CONFIG/PLANNER_CONFIG/RT_MEM/PROFILE/SAMPLER into the remote script. Why: slotq jobs must be
one self-contained line. Evidence: every 5x/6x job file since.

### D220: the live splat tee (commit 2a123e8)
SplatTee in ariadne_live: #kf = u32 len | JSON (agent, id, t_ns, pose_version, T_O_C, K full-res, Kz spool) | Z fp16 |
S fp16 | RGB; #t_w_o xyzquat (qw last); bounded 8/agent, drop-oldest, sender thread per agent; ARIADNE_SPLAT=1 or a
pre-set ARIADNE_RECON_SOCK. Why: D230/D231 contract with the gsplat service. Evidence: no-tee digest
cd6a07aa6fdd000a reproduced 3x with tee variants (determinism-neutral); e2e with the real service: placement
<= 2.2e-16, 92.9 % of wall points within 10 cm, 71 kf -> 4 submaps, 259,994 Gaussians.

### D221: the live collaboration wiring (commits 78f1595, d530c42, 030813b)
`collab: true` (requires multiagent): CollabTee in the estimator worker (serialization only, NO DNN in the lane),
one ariadne_collab_worker per agent on /tmp sockets, collab_inbox operator actions (cut_comms/restore_comms/
inject_false), configs collab_live_v1.yaml (collab_v2 over ideal) + live_collab_v1.yaml; the payload carries the
worker binary and .cache/models. Why: D174. Evidence: collab60c live PASS (e9e975); acceptance pending (see D179
notes + collab60d diagnosis).

### D222: the comms panel (commit 67a8a4f)
Console ingests the planner's multiagent block and CollabLive telemetry ("kept_inter_edges"); /api/frame carries
comms/collab real byte counts; POST /api/collab/{action}?groups= drives cut/restore/inject from the UI. Why: the
operator story (and invariant 6's bytes visible live). Evidence: D222 commit; panel exercised in the collab60x runs.

### D223: collab rewired onto D179's async CollabLive (commits 948a540, 0b14df6 b, 60ed7b8 c)
Fleet thread owns compute+radio; Send(k,fn,t) enqueues in call order; Drain(k,T) at barrier step 2 (FIFO-per-sender,
logged + replayed); ONE Snapshot(T) per barrier, all reads off the snapshot (b). A missing agent view = "nothing
heard yet" (fresh-node semantics: singleton component, broker=self, acked 0) — never .at() (c; the collab60b exit-139
lesson, run 056f14). Why: the no-barrier-sync requirement for collab (D179). Evidence: collab60c live PASS at this
shape; replay logs replayable from collab_events.jsonl.

### D224: pipeline profiling hooks + the host sampler (commits 805ae69, 7e082c1, 95544d6)
Default-off instrumentation (digest-neutral) + perf_sampler.sh vendored verbatim (sha c93b1497ce8484a3) so
ARIADNE_SAMPLER works on instances. Why: D232 perf work needs per-function attribution, not guesses. Evidence: the
job 45 attribution ran on these hooks.

### D225 (root entry; addendum above): the lockstep barrier waits on EVERY lane (commit 0105f5e)
The barrier wait and end condition were hardcoded lanes[0]&&lanes[1] (D213 leftover): agent 2 was never waited on and
its keyframes slid past barrier cuts under load. Found as ma200's replay1 != replay2 trace divergence (STOP-THE-LINE,
run 70a514; first diff t=70, keyframes_integrated agent2 115 vs 114). Fix: lockstep_gate.hpp predicates over N lanes.
Evidence: replay x2 of ma200's own recording byte-identical post-fix (82166ed26dc6094b); unit guard
PreD225ShapeTripsHere; ma200 rerun 626abe live+replay PASS. Also: cmd_replay honors ARIADNE_RT_MEM (41fb2d3).

### D226: ApplyLens rows under cv::parallel_for_ (commit 5cb9e71)
The D41 lens resample parallelized bit-identically (rows are independent; fixed arithmetic per pixel). Evidence:
ma_smoke3 replay trace sha f732c08056b1f937 unchanged. NOTE: in the live runtime this is moot since D227 (lens off
by default); it still serves SimSource/dataset consumers with the lens on.
## 2026-10-24 — D178 RESULT: PASS on all four pre-registered criteria

Evidence: `evaluation/runs/collaboration/known_entry_gate/` (verdict.yaml, cells.csv, true_edges.csv, per-run results/edges/entry_gate,
command.txt). Gated sweep `20261023T213707Z-cpux86-e31421` (staged 4459d57). Logged rerun
`20261024T042708Z-cpux86-4ec3f6` (staged c0e498b); it reproduced every one of the 30 gated cells exactly (merged ATE,
and first-transmission bytes with D180's re-sends folded back in). Baseline without the gate: `eee456`.

| Criterion (pre-registered) | Result |
|---|---|
| Scene 0 budgeted cells, merged ATE < 0.10 m | 4/4 cells (greedy/spectral × p25/p50), worst 0.0147 m. Without the gate: 3.78–3.79 m. Campaign rows 0.013–0.017 m, down from 3.78 m |
| Scene 1 not worse by > 10 % (or 2 mm) | 0/7 cells worse; the largest change is +0.17 % |
| No corrupt run | 0 |
| True edges (verified, pose_err ≤ 10°) rejected at their last test, ≤ 1 % per scene | Scene 0: 0 of 269 tested. Scene 1: 0 of 291 tested. Also 0 ever rejected |

**Stated with it:**
- The pre-registration said "≥ 5 of 6 cells", but the sweep has 4 budgeted cells. It was scored as at most one
  failing cell, and passes under either reading.
- 10 scene-0 true edges were never tested by the gate (ungated: verified after the last solve). They are reported,
  not counted.
- The similarity arm is unbudgeted and ungated by design. Its `sim_s1_seed2` aliases at 3.09 m, unchanged from the
  baseline sweep. The gate is NOT applied there, and that arm keeps the failure.
- The gate is decisive only with known starts. Unknown-start aliasing remains 3.5.
**Cost:** the gated sweep $0.0875 and the logged rerun $0.0882. Two rerun attempts were lost: 4fc41e was killed by the
host on a transient AuthFailure (spinup since fixed), and 080b8f didn't collect entry_gate.csv (fixed in c0e498b).
**Next:** the entry gate is ENABLED in the live config `collab_live_v1`. D178 merges to main with D179/D180 after
the D179 acceptance.

## 2026-10-24 — D180 RESULT: request accounting fixed; byte table still FAILS on match_response (diagnosed)

Run: the D178 logged rerun `20261024T042708Z-cpux86-4ec3f6`, a D180 build. This was the pre-registered next
byte-table run. Scored with `collab_gate.py`; evidence in `evaluation/runs/collaboration/byte_table/`.
- **match_request:** all 24 budgeted rows ≤ 15 %, worst 10.65 %, median 0.77 %. Before D180 it was about 6× under.
  The re-sends moved to wire_resend, 401,792 B over the sweep.
- **The duplicate-response hypothesis is REFUTED.** match_response has wire_resend = 0 in every row. D180's "likely
  cause" sentence was wrong.
- **match_response FAILS:** 8/24 rows over 15 %, worst 27.6 %, median 12.8 %. Measured is ALWAYS below predicted.
  - Diagnosis: per message, the response price is accurate. Where the policy priced at the observed 135.8–135.9 kB,
    the per-message error is −0.4 % to +0.4 %.
  - The class total misses because some selected requests are never answered: 0–5 of 10–44 per run, so the policy
    priced a response that was never sent. Which of those were "gone", given up, or still outstanding at run end is
    the open question.
  - A few rows also carry a higher planned price (142–150 kB). That's the D171 shared-keyframe pricing.
- **Verdict:** I6 byte table FAIL, reported as such. The fix (an expected answer rate in the predictor, calibrated
  cross-scene; or fewer unanswered requests) needs its own preflight and pre-registration before the next byte run.

## 2026-10-24 — D228: estimator workers pin their OpenCV pool to 5 threads (approved, D232 line)
**Decision:** RunEstimatorWorker (D168 process-per-agent mode) calls cv::setNumThreads on entry: default 5,
env ARIADNE_WORKER_CV_THREADS overrides (experiments only). The in-process arrangement (D152a turn lock) is
untouched — its semantics assume an idle pool per call, and it is not the production path.
**Why:** job 46 on c7i.4xlarge (3 workers, lens off): trajectories bit-identical 16 vs 5 threads (the
estimator is thread-count-invariant), wall −1.6 %, and cpu PSI 26 % → 1.2 % — the fleet stops thrashing the box.
**Evidence:** evaluation/runs/performance/pipeline_profile_20261024/processed/replay_perf_threads.csv;
run 20261024T051058Z-cpux86-f94953.

## 2026-10-24 — D229: the lane ingest pipeline (D232 item a2) — pre-registered before code
**Decision:** in LiveSimSource (worker mode only), frame n+1's ingest — CDR decode → lens (D41, when on) →
D (when set) → gray — runs on ONE dedicated ingest thread, handing results to the VO thread through a DEPTH-1
in-order SPSC. SimFrameAssembler::Feed splits into Ingest (stateless per message except the per-side D arrival
counter, which stays in delivery order on the single ingest thread) and Apply (pairing state + emission, on the
VO thread). Depth 1 preserves the D166 pacing backpressure: at most one frame is pre-ingested, so no queue
between gz and the estimator holds more than one sub-step. The D167 wait timer becomes the SPSC pop wait.
**Why:** post-D227 the lane sits at 48–52 ms ≈ the 50 ms frame period (job 46); overlapping ingest with
tracking is the remaining lever to RTF ≥ 1 for the demo. The ingest output is bit-identical regardless of which
thread computes it, so content is unchanged by construction; only timing moves.
**Acceptance before merge (2026-10-24):**
1. lockstep barrier semantics unchanged — a frame must not cross a barrier cut (the gate still reads estimator
   progress, and ingest-ahead is bounded at 1);
2. live + replay×2 byte-identical on a 3-agent run;
3. pre/post trajectory digest on the same recorded inputs digest-neutral (ma_smoke3 lens-off reference
   21a51e0d5d3660ba…), or the divergence explained;
4. RTF measured on c7i via slotq, lens off, 5 threads;
5. a unit guard: Ingest+Apply across threads ≡ the old Feed, byte-identical, including unpaired/drop handling.

## 2026-10-24 — D226 amendment: the parallel lens moved out of the determinism header (stop-the-line fix)
D226 as first landed (5cb9e71) put `#include <opencv2/core.hpp>` + cv::parallel_for_ into the header-only
lens.hpp. That header is the determinism substrate and sits on the manifest include path
(lens_digest.hpp → determinism.hpp), which OpenCV-free targets build by design (arch §3 dependency rule) —
test_unit stopped compiling (Kaggle ct-llm-788a810: 239/306 at 943 s; same cmake as ci_aws, so docker too).
**Fix (e0a55b1):** lens.hpp is OpenCV-free again — serial ApplyLens wraps a factored pure ApplyLensRowSpan.
The cv::parallel_for_ wrapper lives in a new lens_parallel.hpp (ApplyLensParallel), included only by
OpenCV-linked consumers (stereo_source's Ingest, SimSource). NOT a .cpp in the opencv lib: LensDigest()
executes ApplyLens from OpenCV-free targets, so moving the definition there would have broken their link.
**Evidence:** test_unit builds and its observability/determinism/lens tests pass on e0a55b1; ma_smoke3 lens-ON
replay trace sha f732c08056b1f937… = the pre-D226 reference on the restructured binary — D226's bit-identity
claim survives; the row partition is provably value-neutral (pure per-pixel function).

## 2026-10-24 — D229 outcome: acceptance met, RTF gain ZERO (negative result, recorded not buried)
All five pre-registered criteria ran:
1. barrier semantics unchanged (construction + the lockstep runs below);
2. live + replay×2 PASS (ma_smoke4, 3 agents, replay_check 17/17);
3. digest-neutral (pipelined AND kill-switch replays = lens-off reference 21a51e0d…);
4. **RTF on c7i (job 15, run 4d412c, one instance, both arms, $0.027): pipelined 0.833 vs kill-switch 0.839 vs
   job-46 reference 0.836 — a −0.7 % change, within noise of zero.** Both arms' trajectories byte-identical to the
   reference on all 3 agents.
5. unit guard in (PipelinedIngestMatchesFeedExactly).
**Why no gain:** D229 overlaps the ingest half of Feed. Post-D227 (lens off) that half is a few ms of decode+gray;
job 45's ~50 ms Feed share WAS the lens. The lens-off lane is tracking + keyframe compute (FAST ~11 ms, SGBM, LK;
keyframe ~126 ms on 30 % of steps one agent runs >80 ms).
**Decision (on the measurement):** D229 is recorded NEGATIVE and the default is OFF — the single-threaded
shape is the production path (an extra thread with no measured gain does not belong on the demo path; the
pipeline-hyperoptimized rule). The code stays behind ARIADNE_LANE_PIPELINE=1, unit-guarded and digest-proven;
its payoff case is a live path whose ingest is heavy again (lens-on rows, the paused dark/degradation demo —
DegradeParallel runs inside Ingest and would overlap tracking).
**RTF ≥ 1 (inference, agreed):** needs less tracking/keyframe compute per frame or keyframe work off the
lockstep path - a new decision, NOT started (feature freeze).

## 2026-10-27 — D178 demorec250 RESULT: the entry gate holds over 250 s and the fleet keeps 291 TRUE inter-agent edges
The demo recording mission (run 20261024T123707Z-cpux86-99ee9d live leg, 3 agents, live_demo_rec_v1.yaml =
live_collab_v1 + llm chooser + end_s 250, gate ON; gate report from the in-region pack job 285fd8):
- entry gate: tested 11,251 candidates, rejected 2,686; verify 3,610 calls / 350 gated; **kept_inter_edges 291**
  by t=245 s, all three agents merged into agent 0's anchor frame.
- shared-frame pose vs GT (max / final, m): agent0 0.221 / 0.181, agent1 0.219 / 0.130, agent2 0.194 / 0.188
  (5,000 samples each). The collab60d gate-off failure mode (aliased merge, 15.263 m corruption) is absent.
- A/B provenance for the film: collab60d and collab60e are same scene (sim-live/8410a254a94cc16b) AND same run
  seed (1) — both manifests read (60e's extracted in-region from the debba5 recording in the same pack job).

## 2026-10-27 — demorec250 replay RESULT: replay1 died to cgroup OOM (diagnosed, not fixed); D179 replay×2 NOT met for this recording
replay1 (run 788896) failed exit 4 ("worker closed its socket after 0.993 s; 405 records, 0 replies"). The
diagnostic rerun (c442c1, same pinned payload live-payload-20261024T120124Z, same ARIADNE_RT_MEM=20g, host-side
1 s memory sampler + post-exit dmesg per the agreed spec, live_aws.sh c365126) names the mechanism:
- dmesg: "Memory cgroup out of memory: Killed process (ariadne_collab_)" — two workers at ~6.0/5.9 GB anon-RSS,
  oom_kill count 3 on the RT container scope, memory.peak exactly 20 GiB (the limit). Three free-running collab
  workers in replay inflate to ~6-7 GB each (an unpaced replay floods the worker inbox; live stays modest).
- Verdict (rule agreed before the run): **worker inbox backpressure is the fix — a post-demo design item
  under preflight. No cap tweak** ("never raised just to make it pass" would prove nothing on a 32 GiB c7i).
- Consequence, stated not buried: the demorec250 recording has a PASSING live leg and NO passing replay pair, so
  the D179 replay×2 acceptance is NOT met for this recording. The gate/edge results above come from the live
  leg's telemetry (its own digest-checked recording), not from a replay.
- The reaper now names worker deaths ("collab worker pid N KILLED by signal S", 90c41fe); replay-s3 ships
  docker-stats/ps-RSS/cgroup-peak samplers + dmesg on every future replay (c365126).

## 2026-10-28 — D300: ariadne-nemotron fork — shared history, tree-only scrub, local upstream

**What.** ariadne-nemotron is a fork of Ariadne at `0ca523c`. It adds an evidence-seeking agentic layer on NVIDIA Nemotron
served by Nebius Token Factory (Physical AI track).
- **History:** the fork keeps Ariadne's full git history.
- **Scrub:** mentions of the earlier competition are removed from the *current tree* only, by forward commits (`tools/scrub/`), and
  re-applied after every upstream merge.
- **Remotes:** `upstream` is the local `/data/code/Ariadne` with push disabled; `origin` is `CynicPixel/ariadne-nemotron`.
- **Identifiers:** Ariadne identifiers (`ariadne_live`, `ARIADNE_*` env, image names) are not renamed.

**Why.**
- The real-time live-path work keeps landing upstream (`perf/live-decision-path`, 94 files). A shared ancestor keeps those merges ordinary.
- An orphan or rewritten history would turn every sync into a manual ~94-file patch, and would drop the in-window
  provenance (first commit 2026-09-06).
- Renaming identifiers would turn every merge into a conflict storm for no functional gain.

**Verified.** Clone at `0ca523c`. `git remote -v` shows the new origin, with upstream push `DISABLED`. `git ls-remote origin` reaches the empty repo.
Decision IDs D300–D349 are claimed by the fork: FM D300–D309, FP D310–D329, FV D330–D339, D340–D349 reserved.

## 2026-10-29 — D301: Token Factory endpoint, Nemotron model IDs and prices (G0)

**What.** The fork uses base URL `https://api.tokenfactory.nebius.com/v1` (OpenAI-compatible; `api.studio.nebius.com` also answers).
Model IDs are taken from `GET /v1/models?verbose=true` on 2026-10-29 02:29 IST, exactly as served:

| model ID | modality | region | $/1M in | $/1M out | features |
|---|---|---|---|---|---|
| `nvidia/Nemotron-3.5-Super-VL` | text+image→text | us-north1 | 0.30 | 0.90 | tools, reasoning |
| `nvidia/nemotron-3-super-120b-a12b` | text→text | us-central1 | 0.30 | 0.90 | tools, reasoning |
| `nvidia/Nemotron-3_5-Lightning` | text→text | eu-north1 | 0.06 | 0.24 | tools, reasoning |
| `nvidia/NVIDIA-Nemotron-3-Nano-30B-A3B` | text→text | eu-north1 | 0.06 | 0.24 | tools, reasoning |
| `nvidia/Nemotron-3-Ultra-550b-a55b` | text→text | us-central1 | 1.00 | 3.00 | tools, reasoning |

Image tokens are billed as prompt tokens (`pricing.image` is 0). A 262-token map image was observed.

**Why.** The ID casing differs between sources (`Nemotron-3_5-Lightning`, lower-case `nemotron-3-super`), so config takes IDs from this
listing, never from docs. The user's credit is USD 59 and any overrun is billed to their bank. The provider therefore enforces a ledger cap
(hard USD 40, warn USD 25) using these prices.

**Verified [VERIFIED].** Artifacts are in `evaluation/runs/nemotron/g0_20261028T205925Z/` (`models_verbose.json`, `g0_calls.json`, `g0_call.py`); the key appears in none of them.
- **Super-VL**, with a PNG occupancy slab (observed map, no ground truth), an `assign` tool, `tool_choice:auto` and
  `reasoning_effort:"none"`: HTTP 200 in 1.06 s, 637 in / 66 out tokens, reasoning_tokens 0, one well-formed `assign` call.
- **Super 120B**, with `chat_template_kwargs.enable_thinking:false`: HTTP 200 in 1.24 s, 456 / 80, reasoning_tokens 0, one well-formed `assign` call.
- **Caveat:** VL's free-text reason cited "a0's current position", which the prompt never gave. Model rationales are
  untrusted text and are never evidence; this is consistent with the invariant that the model never certifies geometry.
