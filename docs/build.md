# Build, test, run, deploy

Every command below goes through [`./ariadne`](../ariadne), a thin wrapper over the scripts in `infra/`.

**Host requirements:**
- **Local commands:** Docker with BuildKit. No host OpenCV, ROS or Gazebo is needed, because everything runs in the pinned images.
- **Cloud commands:** AWS credentials. Run `infra/scripts/aws.sh login` once; it uses a short-lived browser sign-in and no access keys.
- **Optional GPU:** an NVIDIA GPU gives EGL rendering for the live simulator. Without one it falls back to Mesa llvmpipe, which is slow.

## Quick start

```bash
./ariadne build      # OpenCV 5 base → core image (compiles everything, runs the full C++ suite) → simulator → orchestrator
./ariadne test       # C++ suite (ctest), reconstruction, world-generation and determinism tests
./ariadne gate       # end-to-end replay gate over the pinned recording: one verdict
./ariadne run maze_short 6 200                     # live closed loop: seeded world, 200 s
./ariadne replay build/live/runs/<run> build/live/scenes/maze_short_6   # replay twice, compare digests
./ariadne demo 8080  # web operator console (the hosted endpoint runs this)
```

## Cloud (AWS, eu-north-1)

| command | what | instance |
|---|---|---|
| `./ariadne cloud e2e` | the final integrated stack: full VIO, collaboration, Bedrock chooser, GPU splat. Writes the evidence bundle to `evaluation/runs/live-agentic/final_stack_e2e/` | g4dn.xlarge spot |
| `./ariadne cloud premerge` | the replay gate on a fresh instance (stage, run, teardown) | c7i.4xlarge spot |
| `./ariadne cloud arm64` | native Graviton4 build and full test suite | c8g.4xlarge spot |
| `./ariadne cloud audit` | list every billable resource | none |
| `./ariadne infra plan` / `apply` | Terraform: state bucket, artifact bucket, ECR, IAM runner role (S3 runs/*, ECR pull, Bedrock), GitHub OIDC, budget | none |

How cloud runs work:
- Each run is one spot instance (`infra/scripts/spinup.sh`). The instance pulls the repository snapshot and images, runs one command, uploads `s3://<artifact bucket>/runs/<id>/output/`, and terminates. There is no SSH; debugging goes through SSM.
- `./ariadne runs` indexes every run with its cost.

## Pinned dependencies

| component | pin | where |
|---|---|---|
| OpenCV + contrib | 5.0.0: opencv `40738fb16ceddb5fb3fea747585f7ce6abb0605b`, contrib `755e50675d97db9b7d449d8bd6b09888646f6c6e`. The build-information hash is recorded in every run manifest | `infra/docker/opencv-base.Dockerfile` |
| Base images | `ubuntu@sha256:68ee22…`; ROS 2 Jazzy `ros@sha256:c3706e…` plus a pinned ROS apt snapshot (2026-09-11, signed); `python@sha256:f77ac9…`; `nvidia/cuda@sha256:520292…` | `infra/docker/*.Dockerfile` |
| C++ (FetchContent) | Eigen 3.4.1, FlatBuffers v25.12.19, zstd v1.5.7, yaml-cpp 0.9.0, MCAP cpp v2.1.3, GoogleTest v1.18.0; C++17, gcc 13.3 | `cmake/AriadneDependencies.cmake` |
| Python (exact) | torch 2.11.0, gsplat 1.5.3, numpy 2.5.4, scipy 1.18.1, evo 1.31.1, fastapi 0.143.2, uvicorn 0.54.0, mcp 2.3.0, boto3 1.43.118 | `infra/docker/{recon-gpu,recon,analysis,console,orchestrator}.Dockerfile` |
| Test oracles (never runtime) | GTSAM 4.3.0 | `infra/docker/oracle.Dockerfile` |
| Web console | npm lockfile | `python/operator_console/package-lock.json` |
| Terraform | provider lock files | `infra/aws/terraform/.terraform.lock.hcl`, `bootstrap/.terraform.lock.hcl` |
| Models | XFeat, EigenPlaces, ALIKED, LightGlue: exported to ONNX at pinned upstream commits and checked against PyTorch | `infra/scripts/models.sh` |

## Determinism and evidence

- **Recorded inputs and seeded replay.** Gazebo renders a mission once into MCAP. Degradation, IMU noise and radio loss are applied at consumption time from a seed, so baselines and ablations all consume identical pixels.
- **Replays are checked byte for byte.** `./ariadne replay` runs a recording twice and compares the input, estimator, directive, motion and trace digests.
- **Every run writes a manifest:** commit, dirty flag, image digests, OpenCV build hash, configs, seeds and selections.
- **Every number has a home.** Each figure and result in [`docs/results/index.md`](results/index.md) points to its run directory under `evaluation/runs/`.

## Layout

| path | contents |
|---|---|
| `cpp/` | C++17 core: `core/` (geometry, serialization, MCAP), `perception/`, `slam/`, `collaboration/`, `planning/`, `reconstruction/`, `runtime/` (live runtime, tools) |
| `python/` | reconstruction service (gsplat), orchestration (Bedrock + MCP), operator console, world generation, analysis, demo film |
| `ros2/`, `simulator/` | Gazebo Harmonic world, mover plugin, ROS 2 bridge |
| `configs/` | every pipeline configuration (perception, SLAM, collaboration, mission, simulation) |
| `infra/` | Dockerfiles, run scripts, AWS launchers, Terraform |
| `tests/` | unit, numerical, integration and protocol tests (C++ and Python) |
| `evaluation/` | committed run evidence ([`evaluation/README.md`](../evaluation/README.md)) |
| `docs/` | architecture, algorithms, design per subsystem, decisions, results, report sources ([`docs/README.md`](README.md)) |
