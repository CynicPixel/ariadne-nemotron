# Third-Party Licenses

Every dependency Ariadne builds against, with the licence **verified from the fetched
source tree** rather than from memory or a package index. Versions match
`cmake/AriadneDependencies.cmake` and `infra/docker/opencv-base.Dockerfile`; a version bump
requires re-checking the row.

Last verified: 2026-09-06.

## Compiled dependencies

| Component | Version | Licence | Verified from | Notes |
|---|---|---|---|---|
| OpenCV | 5.0.0 (`40738fb1`) | Apache-2.0 | upstream `LICENSE` | core of the perception and geometry stack |
| OpenCV contrib | 5.0.0 (`755e5067`) | Apache-2.0 | upstream `LICENSE` | `ximgproc`, `xphoto`, `optflow`, `quality` (C7) |
| Eigen | 3.4.1 | **MPL-2.0** | `COPYING.README` | see the MPL2-only note below |
| FlatBuffers | v25.12.19 | Apache-2.0 | `LICENSE` | wire format (§15.7) |
| zstd | v1.5.7 | **BSD-3-Clause** | `LICENSE` | dual-licensed BSD OR GPLv2; **we take BSD** |
| yaml-cpp | 0.9.0 | MIT | `LICENSE` | config and manifest parsing |
| GoogleTest | v1.18.0 | BSD-3-Clause | `LICENSE` | tests only, not shipped in any runtime image |

### Eigen: MPL-2.0 enforced by the build, not asserted

Eigen is primarily MPL-2.0 but ships some LGPL-2.1 code, which its own `COPYING.README`
documents. The build defines **`EIGEN_MPL2_ONLY`**, which makes including any LGPL-licensed
Eigen header a **compile error**. The project therefore demonstrates that it uses no LGPL
Eigen code by compiling at all, rather than by claiming so. Removing that define would
silently widen the licence surface, so it should not be removed without a decision.

### zstd: dual-licensed

zstd offers BSD-3-Clause (`LICENSE`) **or** GPLv2 (`COPYING`). Ariadne takes the BSD grant.
The presence of `COPYING` in the source tree does not make the project GPL.

## Planned dependencies, not yet integrated

Recorded so licence review happens before adoption rather than after. Each row is checked
again at integration.

| Component | Expected licence | Purpose |
|---|---|---|
| XFeat (`verlab/accelerated_features`) | Apache-2.0 | weights only; inference implemented on OpenCV 5 primitives (D15) |
| gsplat | Apache-2.0 | reconstruction service (§18.2) |
| PyTorch | BSD-3-Clause | reconstruction training path |
| MCAP C++ SDK | Apache-2.0 | recording container (§5.9) |
| EigenPlaces weights | to verify | retrieval (§17.1) |
| ROS 2 Jazzy, Gazebo Harmonic | Apache-2.0 | simulation only, never linked into the estimator |

## Model weights

Weights are **not** source code and carry their own terms. §44 claims discipline applies:
where Ariadne implements a network but uses published weights, the report says exactly that.
For XFeat (D15) the architecture and forward pass are ours; the weights are upstream,
Apache-2.0, and credited.

Only permissively licensed weights and reference implementations are used, per the
`algorithms.md` licence audit.

## Ariadne's own licence

MIT: see [`LICENSE`](../LICENSE) (chosen by the maintainer on 2026-10-27). The third-party components above keep their own
licences.
