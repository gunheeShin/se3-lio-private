# Python-only image — run the se3_lio Python package + CLI (offline pipeline,
# benchmark, Polyscope/Rerun viz). No ROS.
#
# Builds se3_lio from the LOCAL source WITH OpenCV, so the colorizer/BEV module
# (se3_lio::visual, has_visual=True) is compiled in — the released PyPI wheel is
# OpenCV-less and would leave colouring off. OpenMP/Eigen/Sophus are handled by
# the core build (libgomp + FetchContent).
#
# Build (context = MODULE ROOT, needs cpp/ + python/):  bash docker/python/build_docker.sh
# For a CPU-only / headless image, swap the base for ubuntu:22.04 (software GL).
FROM nvidia/cuda:11.8.0-base-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC
ENV NVIDIA_DRIVER_CAPABILITIES=all

# GL/X11 runtime for Polyscope + a compile toolchain kept so a runtime editable
# install (pip install -e ./python) can still build the core for local dev.
# build-essential/cmake/git/ninja: compile the core. libopencv-dev/libeigen3-dev:
# the colorizer needs OpenCV (Eigen is also FetchContent'd, kept for system reuse).
# GL/X11: Polyscope viewer runtime.
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3 python3-pip python3-dev \
      build-essential cmake git ninja-build libgomp1 \
      libopencv-dev libeigen3-dev \
      libgl1 libglu1-mesa libxext6 libxrender1 \
      libx11-6 libxrandr2 libxinerama1 libxcursor1 libxi6 \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir scikit-build-core pybind11 ninja

# Source build with OpenCV -> se3_lio::visual compiled in (has_visual=True).
# --no-build-isolation uses the apt cmake (3.22, <4) + the pip toolchain above.
# python/CMakeLists finds ../cpp and builds the core locally.
WORKDIR /src
COPY cpp ./cpp
COPY python ./python
COPY README.md ./README.md
RUN pip install --no-cache-dir --no-build-isolation "./python[viz,rerun]"

WORKDIR /ws
CMD ["bash"]
