# Third-party pins. Fetched in-tree rather than apt-installed so the build is
# hermetic and needs no sudo: the same versions resolve on the dev box, in CI,
# and on Graviton. Every version here is mirrored into the run manifest (§28.2).
include(FetchContent)
set(FETCHCONTENT_QUIET OFF)

# Eigen 3.4.1 — latest 3.4.x, which is what arch §27.1's "Eigen 3.4" pins.
# (Eigen 5.0.1 exists upstream; deliberately NOT taken — see docs/decisions.md.)
# SOURCE_SUBDIR names a directory with no CMakeLists.txt on purpose: that makes
# FetchContent populate the sources WITHOUT running Eigen's own build system,
# which we do not need (Eigen is header-only) and do not want polluting our cache.
FetchContent_Declare(eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG        3.4.1
  GIT_SHALLOW    TRUE
  SOURCE_SUBDIR  do-not-configure-eigen)
FetchContent_MakeAvailable(eigen)
add_library(ariadne_eigen INTERFACE)
target_include_directories(ariadne_eigen SYSTEM INTERFACE ${eigen_SOURCE_DIR})
# Eigen is primarily MPL2 but ships some LGPL 2.1 code (see its COPYING.README).
# EIGEN_MPL2_ONLY turns including any LGPL-licensed Eigen header into a COMPILE
# ERROR, so permissive licensing becomes something the build proves rather than
# something we assert. The competition checks component licences individually.
target_compile_definitions(ariadne_eigen INTERFACE EIGEN_MPL2_ONLY)

# FlatBuffers is the locked serializer (arch §0.2): schema-versioned,
# zero-copy read, and deterministic size accounting at the single serialization
# boundary §4.4 requires. Building it here also builds `flatc`, which generates
# headers from the .fbs schemas at build time, so no generated code is committed
# and the schema is the single source of truth.
set(FLATBUFFERS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(FLATBUFFERS_INSTALL OFF CACHE BOOL "" FORCE)
set(FLATBUFFERS_BUILD_FLATLIB ON CACHE BOOL "" FORCE)
set(FLATBUFFERS_BUILD_FLATC ON CACHE BOOL "" FORCE)
set(FLATBUFFERS_BUILD_FLATHASH OFF CACHE BOOL "" FORCE)
FetchContent_Declare(flatbuffers
  GIT_REPOSITORY https://github.com/google/flatbuffers.git
  GIT_TAG        v25.12.19
  GIT_SHALLOW    TRUE)
FetchContent_MakeAvailable(flatbuffers)

# zstd: applied to message classes 4-5 (MATCH_RESPONSE, POSE_EDGE) per §15.7.
# Descriptors are high-entropy, so the achieved ratio is MEASURED and reported,
# never assumed. Its CMake lives in build/cmake, not at the repo root.
set(ZSTD_BUILD_PROGRAMS OFF CACHE BOOL "" FORCE)
set(ZSTD_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(ZSTD_BUILD_SHARED OFF CACHE BOOL "" FORCE)
set(ZSTD_BUILD_STATIC ON CACHE BOOL "" FORCE)
set(ZSTD_LEGACY_SUPPORT OFF CACHE BOOL "" FORCE)
FetchContent_Declare(zstd
  GIT_REPOSITORY https://github.com/facebook/zstd.git
  GIT_TAG        v1.5.7
  GIT_SHALLOW    TRUE
  SOURCE_SUBDIR  build/cmake)
FetchContent_MakeAvailable(zstd)

# yaml-cpp: configs and run manifests are YAML (§28.2, §5.6). A hand-rolled
# subset parser was considered and rejected: config files grow nested sections
# (camera, comms, planner), and each extension of a bespoke parser is another
# chance to mis-parse silently, which is the one failure mode a config loader
# must not have.
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_FORMAT_SOURCE OFF CACHE BOOL "" FORCE)
FetchContent_Declare(yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG        yaml-cpp-0.9.0
  GIT_SHALLOW    TRUE)
FetchContent_MakeAvailable(yaml-cpp)

# MCAP: the recording container of §5.9. The STANDALONE Foxglove C++ SDK, not
# rosbag2 (D3): §3's dependency rule requires replay to work from files alone,
# ROS-free, and MCAP is a container rather than a ROS API, so bags stay
# rosbag2-compatible either way. Header-only; one TU defines MCAP_IMPLEMENTATION.
FetchContent_Declare(mcap
  GIT_REPOSITORY https://github.com/foxglove/mcap.git
  GIT_TAG        releases/cpp/v2.1.3
  GIT_SHALLOW    TRUE
  SOURCE_SUBDIR  do-not-configure-mcap)
FetchContent_MakeAvailable(mcap)
add_library(ariadne_mcap INTERFACE)
target_include_directories(ariadne_mcap SYSTEM INTERFACE ${mcap_SOURCE_DIR}/cpp/mcap/include)
# LZ4 is not vendored and nothing needs it: chunk compression uses zstd, which
# is already a dependency. Disabling it here avoids adding a library for a
# codec we would never select.
target_compile_definitions(ariadne_mcap INTERFACE MCAP_COMPRESSION_NO_LZ4)
target_link_libraries(ariadne_mcap INTERFACE libzstd_static)

# ONNX Runtime (opt-in, ARIADNE_WITH_ORT): the GPU path for ALIKED + LightGlue + EigenPlaces in live GPU runs.
# OpenCV 5.0.0's cv::dnn cannot take them there: ENGINE_NEW is CPU-only (dnn.hpp) and ENGINE_CLASSIC cannot import
# the ALIKED/LightGlue graphs (SMOKE §4s). cv::dnn stays the default and the evidence path; the runtime records
# which backend ran. The official prebuilt release (CUDA 12 + cuDNN 9 build; its CPU provider also runs without a
# GPU, which the equality tests use). Pinned by version and sha256 (the release asset digest).
option(ARIADNE_WITH_ORT "ONNX Runtime backend for the hard-pair matcher and place recognizer" OFF)
if(ARIADNE_WITH_ORT)
  FetchContent_Declare(onnxruntime
    URL      https://github.com/microsoft/onnxruntime/releases/download/v1.31.0/onnxruntime-linux-x64-gpu_cuda12-1.31.0.tgz
    URL_HASH SHA256=62d3af5269af2143b5d37494337573f1ffb8885f3ad7477ae0cb1cc79c8dedfd)
  FetchContent_MakeAvailable(onnxruntime)
  add_library(ariadne_onnxruntime SHARED IMPORTED GLOBAL)
  set_target_properties(ariadne_onnxruntime PROPERTIES
    IMPORTED_LOCATION ${onnxruntime_SOURCE_DIR}/lib/libonnxruntime.so
    INTERFACE_INCLUDE_DIRECTORIES ${onnxruntime_SOURCE_DIR}/include
    INTERFACE_COMPILE_DEFINITIONS ARIADNE_HAVE_ORT=1)
  set(ARIADNE_ORT_LIB_DIR ${onnxruntime_SOURCE_DIR}/lib CACHE INTERNAL "ONNX Runtime shared libraries (ship with the binary)")
endif()

if(ARIADNE_BUILD_TESTS)
  FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.18.0
    GIT_SHALLOW    TRUE)
  FetchContent_MakeAvailable(googletest)
endif()

# OpenCV 5: the pinned build installed at /opt/opencv5 by opencv-base
# (C7: one OpenCV, built from the pinned tag, never apt's 4.x). CONFIG mode
# reads OpenCVConfig.cmake from that install rather than a Find-module guess.
# The version constraint `5` makes an accidentally-found OpenCV 4 a miss, not a
# silent downgrade.
if(ARIADNE_REQUIRE_OPENCV)
  find_package(OpenCV 5 CONFIG REQUIRED PATHS /opt/opencv5 NO_DEFAULT_PATH)
else()
  find_package(OpenCV 5 CONFIG QUIET PATHS /opt/opencv5 NO_DEFAULT_PATH)
endif()
if(OpenCV_FOUND)
  message(STATUS "Ariadne: OpenCV ${OpenCV_VERSION} from ${OpenCV_DIR}")
else()
  message(STATUS "Ariadne: OpenCV 5 not found; OpenCV-dependent targets are skipped")
endif()
