include(GoogleTest)

add_executable(test_numerical
  numerical/test_lie.cpp
  numerical/test_covariance.cpp
  numerical/test_time.cpp
  numerical/test_philox.cpp
  numerical/test_det_math.cpp
  numerical/test_distributions.cpp
  numerical/test_degradation.cpp
  numerical/test_gat.cpp
  numerical/test_noise_estimate.cpp
  numerical/test_flat_field.cpp
  numerical/test_pinhole.cpp
  numerical/test_ba.cpp
  numerical/test_pose_graph.cpp
  numerical/test_pgo.cpp
  numerical/test_window_ba.cpp
  numerical/test_preintegration.cpp
  numerical/test_window_ba_inertial.cpp
  numerical/test_inertial_init.cpp
  numerical/test_gravity_prior.cpp
  numerical/test_scripted.cpp
  numerical/test_command_planner.cpp)
target_link_libraries(test_numerical PRIVATE ariadne::core ariadne::perception ariadne::slam
  yaml-cpp::yaml-cpp GTest::gtest_main)
target_compile_definitions(test_numerical PRIVATE ARIADNE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
gtest_discover_tests(test_numerical)

# D123: no unsequenced random draws in a test fixture (the argument order differs between x86-64 and aarch64 GCC).
find_package(Python3 COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
  add_test(NAME RngSequencing.NoUnsequencedDrawsInTests
           COMMAND ${Python3_EXECUTABLE} -I ${CMAKE_SOURCE_DIR}/infra/scripts/check_rng_sequencing.py ${CMAKE_SOURCE_DIR}/tests)
endif()

add_executable(test_protocol
  protocol/test_serialization.cpp
  protocol/test_collab_protocol.cpp
  protocol/test_network.cpp
  protocol/test_transport.cpp
  protocol/test_membership.cpp
  protocol/test_collab_v2.cpp
  protocol/test_collab_selection.cpp
  protocol/test_coordination_wire.cpp
  protocol/test_coordination_node.cpp
  protocol/test_deferred_features.cpp)
target_link_libraries(test_protocol PRIVATE ariadne::core GTest::gtest_main)
gtest_discover_tests(test_protocol)

# The Phase 0 exit gate: generate -> serialize -> replay -> score, twice.
add_executable(test_phase0_gate
  end_to_end/test_phase0_gate.cpp
  end_to_end/test_mcap_replay.cpp)
target_link_libraries(test_phase0_gate PRIVATE ariadne::runtime GTest::gtest_main)
gtest_discover_tests(test_phase0_gate)

add_executable(test_unit
  unit/test_observability.cpp
  unit/test_config.cpp
  unit/test_sim_depth.cpp
  unit/test_metrics.cpp
  unit/test_perception_config.cpp
  unit/test_euroc.cpp
  unit/test_imu_noise.cpp
  unit/test_comms.cpp
  unit/test_ros_cdr.cpp)
target_link_libraries(test_unit PRIVATE ariadne::runtime ariadne::perception ariadne::slam GTest::gtest_main)
# Tests read the shipped configs from the source tree.
target_compile_definitions(test_unit PRIVATE ARIADNE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
gtest_discover_tests(test_unit)

# Tests that need OpenCV exist only where OpenCV does (the ariadne-core image).
if(OpenCV_FOUND)
  add_executable(test_opencv
    unit/test_opencv_identity.cpp
    unit/test_perception_cv.cpp
    unit/test_tum.cpp
    unit/test_stereo_vo.cpp
    unit/test_lens.cpp
    unit/test_radiometric_stage.cpp
    unit/test_exposure_control.cpp
    unit/test_falloff.cpp
    unit/test_falloff_glue.cpp
    unit/test_exposure_glue.cpp
    unit/test_edge_verifier.cpp
    unit/test_multi_agent.cpp
    unit/test_entry_gate.cpp
    unit/test_keyframe_spool.cpp
    unit/test_live_link.cpp
    unit/test_live_planner.cpp
    unit/test_lockstep_gate.cpp
    unit/test_depth_eval.cpp
    unit/test_planning.cpp
    unit/test_xfeat_reference.cpp
    unit/test_pgo_levmarq.cpp
    unit/test_pgo_oracle.cpp
    unit/test_chordal_init.cpp
    unit/test_degeneracy.cpp
    unit/test_pcm.cpp
    unit/test_gnc.cpp
    unit/test_loop_backend.cpp
    unit/test_gravity_pgo.cpp
    unit/test_certificate.cpp
    unit/test_aliasing_ramp.cpp
    unit/test_joint_robust.cpp
    unit/test_spectral_selector.cpp
    unit/test_local_descriptor_codec.cpp
    unit/test_window_ba_levmarq.cpp
    unit/test_reassociation.cpp
    unit/test_xfeat_engine.cpp
    unit/test_stereo_confidence.cpp
    unit/test_eigenplaces_reference.cpp
    unit/test_place.cpp
    unit/test_aliked_lightglue_reference.cpp
    unit/test_hard_pair.cpp
    unit/test_loop_verify_geometry.cpp
    unit/test_dnn_contract.cpp
    unit/test_volumetric.cpp
    unit/test_traversability.cpp
    unit/test_egress.cpp
    unit/test_nbv.cpp
    unit/test_phase4_planner.cpp
    unit/test_nbv_time.cpp
    unit/test_multiagent.cpp
    unit/test_allocation.cpp
    unit/test_visibility_graph.cpp
    unit/test_collab_link.cpp
    unit/test_collab_snapshot.cpp)
  target_link_libraries(test_opencv PRIVATE ariadne::runtime_opencv ariadne::perception_opencv
    ariadne::slam_opencv ariadne::planning opencv_features opencv_imgcodecs opencv_ptcloud opencv_geometry GTest::gtest_main)
  target_compile_definitions(test_opencv PRIVATE ARIADNE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
  gtest_discover_tests(test_opencv)
endif()
