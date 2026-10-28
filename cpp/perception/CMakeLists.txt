# ariadne_perception: image-domain code. Header-only for now. Depends on core
# and, deliberately, NOT on OpenCV -- the degradation operator is the
# determinism substrate and must be testable without it (arch §3).
add_library(ariadne_perception INTERFACE)
target_include_directories(ariadne_perception INTERFACE ${CMAKE_SOURCE_DIR}/cpp)
target_link_libraries(ariadne_perception INTERFACE ariadne::core)
add_library(ariadne::perception ALIAS ariadne_perception)

# ariadne_perception_opencv: the OpenCV-backed front-end (arch §7-§9). Exists
# only when OpenCV was found, so the OpenCV-free layer above keeps building and
# testing on a host without it. OpenCV 5 module names: features (ORB), video
# (pyramidal LK), stereo (SGBM), geometry (findFundamentalMat; ex-calib3d).
if(OpenCV_FOUND)
  add_library(ariadne_perception_opencv STATIC
    features/orb_engine.cpp
    tracking/klt_tracker.cpp
    stereo/sgbm_stereo.cpp
    stereo/stereo_rectifier.cpp
    stereo/stereo_lk_matcher.cpp
    features/xfeat/xfeat_net.cpp
    radiometric/radiometric_stage.cpp
    radiometric/falloff.cpp
    exposure_control/exposure_controller.cpp
    features/xfeat_engine.cpp
    place/place_recognizer.cpp
    association/landmark_reassociation.cpp
    association/hard_pair_matcher.cpp)
  target_link_libraries(ariadne_perception_opencv PUBLIC
    ariadne::perception opencv_core opencv_imgproc opencv_features opencv_video opencv_dnn opencv_ximgproc
    opencv_stereo opencv_geometry opencv_photo opencv_ptcloud)
  add_library(ariadne::perception_opencv ALIAS ariadne_perception_opencv)
  if(ARIADNE_WITH_ORT)
    target_sources(ariadne_perception_opencv PRIVATE dnn/ort_session.cpp)
    target_link_libraries(ariadne_perception_opencv PUBLIC ariadne_onnxruntime)
  endif()
endif()
