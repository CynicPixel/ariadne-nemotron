# syntax=docker/dockerfile:1.7
# ---------------------------------------------------------------------------
# ariadne core-gpu: the live runtime image for GPU DNN (collab.dnn_backend: ort_cuda). It is core-dev (the same
# pinned CPU OpenCV 5 at /opt/opencv5, the same toolchain, the same uid) plus the CUDA 12.8 runtime and cuDNN 9
# libraries that ONNX Runtime's CUDA execution provider loads. ariadne_live and ariadne_collab_worker are the
# ARIADNE_WITH_ORT build; the ONNX Runtime libraries ship beside them (build/ort/lib, LD_LIBRARY_PATH in live.sh).
# Both stages are Ubuntu 24.04, so the copied libraries match core-dev's glibc.
#
#   docker build -f infra/docker/core-gpu.Dockerfile -t ariadne/core-gpu:dev .     (needs ariadne/core-dev:dev)
# ---------------------------------------------------------------------------
ARG CORE_DEV=ariadne/core-dev:dev
FROM nvidia/cuda@sha256:ac55d124da4882b497f732d8dfd9a702d5447a5f29d08d56da6f64f0a1eb34bc AS cuda
# = nvidia/cuda:12.8.1-cudnn-runtime-ubuntu24.04 (manifest list digest)

FROM ${CORE_DEV}
USER root
# cudart, cuBLAS/cuBLASLt, cuFFT, cuRAND (ONNX Runtime's CUDA provider) and cuDNN 9.
COPY --from=cuda /usr/local/cuda/lib64/ /opt/cuda/lib64/
COPY --from=cuda /usr/lib/x86_64-linux-gnu/libcudnn* /opt/cuda/cudnn/
RUN echo /opt/cuda/lib64 > /etc/ld.so.conf.d/cuda.conf && echo /opt/cuda/cudnn >> /etc/ld.so.conf.d/cuda.conf && ldconfig
# The NVIDIA container runtime injects the driver (libcuda) when the container runs with --gpus.
ENV NVIDIA_VISIBLE_DEVICES=all NVIDIA_DRIVER_CAPABILITIES=compute,utility
USER ariadne
WORKDIR /work
