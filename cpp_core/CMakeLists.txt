cmake_minimum_required(VERSION 3.14)
project(AmiOgma CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Required for static libraries that get linked into a shared .so (GDExtension).
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Allow FetchContent'd dependencies that still declare a pre-3.5 minimum
# (Eigen 3.4.0, nlohmann_json 3.11.2) to configure under CMake 4.x.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "Min policy for fetched deps")

# When included as a subdirectory (e.g. by godot_host), default tests to OFF.
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    set(_OGMA_TESTS_DEFAULT ON)
else()
    set(_OGMA_TESTS_DEFAULT OFF)
endif()
option(OGMA_BUILD_TESTS "Build ogma_core test binaries" ${_OGMA_TESTS_DEFAULT})

include(FetchContent)

# ---------------------------------------------------------------------------
# Dependencies (header-only or fetched)
# ---------------------------------------------------------------------------

# 1. Eigen3 (header-only, required)
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG        3.4.0
)
FetchContent_MakeAvailable(eigen)

# 2. nlohmann/json (header-only, required)
FetchContent_Declare(
  json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG        v3.11.2
)
FetchContent_MakeAvailable(json)

# 3. GoogleTest (tests only — skip entirely when building as a subdirectory)
if(OGMA_BUILD_TESTS)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()

# ---------------------------------------------------------------------------
# Optional system dependencies
# ---------------------------------------------------------------------------

# OpenCV — optional, enables video capture and optical flow
find_package(OpenCV QUIET)
if(OpenCV_FOUND)
    message(STATUS "OpenCV found — video capture and optical flow enabled")
else()
    message(STATUS "OpenCV not found — video capture disabled")
endif()

# ZeroMQ — required for multi-EPM message bus (Phase 3)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)
message(STATUS "ZeroMQ found: ${ZMQ_VERSION}")

# X11 — optional, enables window capture (Linux)
find_package(X11 QUIET)
if(X11_FOUND)
    message(STATUS "X11 found — window capture enabled")
endif()

# ---------------------------------------------------------------------------
# Include paths
# ---------------------------------------------------------------------------

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${eigen_SOURCE_DIR}
    ${json_SOURCE_DIR}/include
)

# ---------------------------------------------------------------------------
# Core infrastructure library (sensors + control server)
# Unchanged from v1; no ONNX dependency.
# ---------------------------------------------------------------------------

add_library(ogma_infra
    src/video_capture.cpp
    src/control_server.cpp
)
# NOTE(xaq): src/audio_capture.cpp (miniaudio mic capture) is AMI-Awen-only
# and intentionally omitted from the open xaq tree.

target_include_directories(ogma_infra PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${eigen_SOURCE_DIR}
    ${json_SOURCE_DIR}/include
)

if(OpenCV_FOUND)
    target_compile_definitions(ogma_infra PRIVATE USE_OPENCV)
    target_include_directories(ogma_infra PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(ogma_infra PUBLIC ${OpenCV_LIBRARIES})
endif()

if(X11_FOUND)
    target_compile_definitions(ogma_infra PRIVATE USE_X11)
    target_link_libraries(ogma_infra PUBLIC X11::X11)
else()
    message(STATUS "X11 not found — window capture disabled (headless build)")
endif()

# ---------------------------------------------------------------------------
# v3 geometric core library
# Implements: FrozenJLEncoder, FrozenSTFTEncoder, GNG
# No ONNX — fully geometric, tabula rasa.
# ---------------------------------------------------------------------------

add_library(ogma_v3
    src/v3/encoder_jl.cpp
    src/v3/encoder_stft.cpp
    src/v3/encoder_rbf.cpp
    src/v3/gng.cpp
    src/v3/epm.cpp
)
# NOTE(xaq): src/v3/encoder_hopf.cpp (bio-mimetic cochlear encoder) is
# AMI-Awen-only; encoder_stft.cpp is the generic IP-free audio front-end.
# TODO(xaq-audio): windowed STFT + mel + JL projection (see encoder_stft.hpp).

target_include_directories(ogma_v3 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${eigen_SOURCE_DIR}
    ${json_SOURCE_DIR}/include
)

if(OpenCV_FOUND)
    target_compile_definitions(ogma_v3 PRIVATE USE_OPENCV)
    target_include_directories(ogma_v3 PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(ogma_v3 PUBLIC ${OpenCV_LIBRARIES})
endif()

# ---------------------------------------------------------------------------
# v4 Ogma Core library
#
# The new namespace where every v4 cognitive module lives: Bus, Module,
# Scheduler, OgmaInstance, Topics, GraphConfig, plus the nine primitives
# (NeurochemState, EPM, LateralVoter, HomeostaticDrive, ActionDecoder,
# DescendingPredictor, SequenceGNG, GNGRollout, MotorRepertoire). Phase 0
# froze the header surface; Phase 1 adds the implementations in dependency
# order.  v3 code under include/v3 and src/v3 remains as port reference
# until its modules are absorbed.
# ---------------------------------------------------------------------------

add_library(ogma_core
    src/ogma/Rng.cpp
    src/ogma/InProcessBus.cpp
    src/ogma/Module.cpp
    src/ogma/ModuleRegistry.cpp
    src/ogma/Scheduler.cpp
    src/ogma/OgmaInstance.cpp
    src/ogma/GraphConfig.cpp
    src/ogma/LiveGraph.cpp
    src/ogma/PayloadTypeName.cpp
    src/ogma/DiagPublisher.cpp
    src/ogma/modules/NeurochemState.cpp
    src/ogma/modules/EPM.cpp
    src/ogma/modules/LateralVoter.cpp
    src/ogma/modules/HomeostaticDrive.cpp
    src/ogma/modules/ActionDecoder.cpp
    src/ogma/modules/DescendingPredictor.cpp
    src/ogma/modules/SequenceGNG.cpp
    src/ogma/modules/GNGRollout.cpp
    src/ogma/modules/MotorRepertoire.cpp
    src/ogma/modules/HomeokineticExploration.cpp
    src/ogma/modules/Premotor.cpp
    src/ogma/modules/PremotorAI.cpp
    src/ogma/modules/ActionGate.cpp
    src/ogma/modules/WhiskerAversionReflex.cpp
    src/ogma/modules/DualEMADetector.cpp
    src/ogma/modules/AdaptiveThresholdTracker.cpp
    src/ogma/modules/StuckEscapeReflex.cpp
    src/ogma/modules/WhiskerSteerReflex.cpp
    src/ogma/modules/ScentGateReflex.cpp
    src/ogma/modules/ForwardDriveReflex.cpp
    src/ogma/modules/EmbeddingRegistry.cpp
    src/ogma/modules/MotorFader.cpp
    src/ogma/modules/MotorBus.cpp
    src/ogma/modules/FaderController.cpp
    src/ogma/modules/CellReflex.cpp
    src/ogma/modules/ChemotaxisAI.cpp
    src/ogma/modules/HeadingController.cpp
    src/ogma/modules/GoalBelief.cpp
    src/ogma/modules/HeadingPlanner.cpp
    src/ogma/modules/HeadingProbe.cpp
    src/ogma/modules/MotivationGate.cpp
    src/ogma/modules/BearingEstimator.cpp
    src/ogma/modules/VisualBearing.cpp
    src/ogma/modules/BearingFusion.cpp
    src/ogma/modules/ScentHomingLearner.cpp
    src/ogma/modules/SaccadeReflex.cpp
    src/ogma/modules/CylinderBuilder.cpp
    src/ogma/modules/ColumnBuilder.cpp
    src/ogma/modules/PlaceGraphPlanner.cpp
    src/ogma/modules/PlayLoop.cpp
    src/ogma/modules/RunTumbleNav.cpp
    src/ogma/modules/RunTumbleNavV2.cpp
    src/ogma/modules/VisualHomingNav.cpp
    src/ogma/modules/PlaceNav.cpp
    src/ogma/modules/EFEArbiter.cpp
    src/ogma/modules/GradientEPM.cpp
    src/ogma/modules/Klinotaxis.cpp
    src/ogma/modules/ScentCompass.cpp
    src/ogma/modules/SensorBundle.cpp
    src/ogma/modules/KeyframeAverager.cpp
    src/ogma/modules/KeyframePeakDetector.cpp
    src/ogma/modules/EventConjunction.cpp
    src/ogma/modules/ChunkAbortGate.cpp
    src/ogma/modules/ChunkOutcomeGate.cpp
    src/ogma/modules/EpisodicCapture.cpp
    src/ogma/modules/StaleConfidenceDecay.cpp
    src/ogma/modules/DistressDrive.cpp
    src/ogma/modules/JointSensorimotorBridge.cpp
    src/ogma/modules/PolicyChannelAggregator.cpp
    src/ogma/modules/CPGOscillator.cpp
    src/ogma/modules/MotorEPM.cpp
    src/ogma/modules/GainEvolver.cpp
    src/ogma/modules/MotorEPMv2.cpp
    src/ogma/modules/MotorPlanner.cpp
    src/ogma/modules/PosturalPrior.cpp
    src/ogma/modules/KeyframeGait.cpp
    src/ogma/modules/BodyRhythmTracker.cpp
    src/ogma/modules/SynergyTimer.cpp
    src/ogma/modules/CruseCoordinator.cpp
    src/ogma/modules/GaitSelector.cpp
    src/ogma/golden/Replay.cpp
)

target_include_directories(ogma_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${eigen_SOURCE_DIR}
    ${json_SOURCE_DIR}/include
)

# EPM wraps the v3 encoders + GNG.
target_link_libraries(ogma_core PUBLIC ogma_v3 ${ZMQ_LIBRARIES})
target_include_directories(ogma_core PUBLIC ${ZMQ_INCLUDE_DIRS})
target_link_directories(ogma_core PUBLIC ${ZMQ_LIBRARY_DIRS})

# ---------------------------------------------------------------------------
# Unit tests  (only built when OGMA_BUILD_TESTS=ON, the default when
# this file is the top-level CMakeLists)
# ---------------------------------------------------------------------------

if(OGMA_BUILD_TESTS)

enable_testing()

add_executable(test_encoder_jl tests/v3/test_encoder_jl.cpp)
target_link_libraries(test_encoder_jl PRIVATE ogma_v3 GTest::gtest_main pthread)

add_executable(test_gng tests/v3/test_gng.cpp)
target_link_libraries(test_gng PRIVATE ogma_v3 GTest::gtest_main pthread)

add_executable(test_epm tests/v3/test_epm.cpp)
target_link_libraries(test_epm PRIVATE ogma_v3 GTest::gtest_main pthread)

add_executable(test_mitosis tests/v3/test_mitosis.cpp)
target_link_libraries(test_mitosis PRIVATE ogma_v3 GTest::gtest_main pthread)

# v4 Phase 0 header-compatibility smoke test
add_executable(test_ogma_headers tests/ogma/test_headers_compile.cpp)
target_link_libraries(test_ogma_headers PRIVATE ogma_core GTest::gtest_main pthread)

# v4 Phase 1.0 — Rng cross-language parity test
add_executable(test_rng_parity tests/ogma/test_rng_parity.cpp)
target_link_libraries(test_rng_parity PRIVATE ogma_core GTest::gtest_main pthread)

# v4 Phase 1.0 — InProcessBus unit tests
add_executable(test_inprocess_bus tests/ogma/test_inprocess_bus.cpp)
target_link_libraries(test_inprocess_bus PRIVATE ogma_core GTest::gtest_main pthread)

# v4 Phase 1.1 — NeurochemState unit tests
add_executable(test_neurochem_state tests/ogma/test_neurochem_state.cpp)
target_link_libraries(test_neurochem_state PRIVATE ogma_core GTest::gtest_main pthread)

# v4 Phase 1.1 — pair_neurochem_epm (Phase 1.2: real EPM)
add_executable(pair_neurochem_epm tests/ogma/pair_neurochem_epm.cpp)
target_link_libraries(pair_neurochem_epm PRIVATE ogma_core GTest::gtest_main pthread)

# v4 Phase 1.2 — EPM unit tests
add_executable(test_epm_module tests/ogma/test_epm_module.cpp)
target_link_libraries(test_epm_module PRIVATE ogma_core GTest::gtest_main pthread)

# v4 Phase 1.2 — pair_epm_lateralvoter (Phase 1.3: real LateralVoter)
add_executable(pair_epm_lateralvoter tests/ogma/pair_epm_lateralvoter.cpp)
target_link_libraries(pair_epm_lateralvoter PRIVATE ogma_core GTest::gtest_main pthread)

# v4 Phase 1.3 — LateralVoter unit tests
add_executable(test_lateral_voter tests/ogma/test_lateral_voter.cpp)
target_link_libraries(test_lateral_voter PRIVATE ogma_core GTest::gtest_main pthread)

# Phase 6.5.24 — LateralVoter behavioral / stream tests.  Drives synthetic
# RealityToken streams (multi-tick patterns) through the voter and asserts
# convergent trust + geometric fusion + disagreement resolution.  Covers
# the gap between per-tick correctness (test_lateral_voter.cpp) and the
# end-to-end behavioral guarantee the consensus layer claims to provide.
add_executable(test_lateral_voter_streams tests/ogma/test_lateral_voter_streams.cpp)
target_link_libraries(test_lateral_voter_streams PRIVATE ogma_core GTest::gtest_main pthread)

# Phase 6.5.25 — Premotor module behavioral / stream tests.
add_executable(test_premotor_streams tests/ogma/test_premotor_streams.cpp)
target_link_libraries(test_premotor_streams PRIVATE ogma_core GTest::gtest_main pthread)

# Phase 6.5.26 — ActionGate behavioral / stream tests.
add_executable(test_action_gate_streams tests/ogma/test_action_gate_streams.cpp)
target_link_libraries(test_action_gate_streams PRIVATE ogma_core GTest::gtest_main pthread)

# Phase 6.6.D — reflex-as-module primitives stream tests.
add_executable(test_reflex_modules_streams tests/ogma/test_reflex_modules_streams.cpp)
target_link_libraries(test_reflex_modules_streams PRIVATE ogma_core GTest::gtest_main pthread)

# v4 Phase 1.4 — HomeostaticDrive
add_executable(test_homeostatic_drive tests/ogma/test_homeostatic_drive.cpp)
target_link_libraries(test_homeostatic_drive PRIVATE ogma_core GTest::gtest_main pthread)

add_executable(pair_lateralvoter_homeostatic tests/ogma/pair_lateralvoter_homeostatic.cpp)
target_link_libraries(pair_lateralvoter_homeostatic PRIVATE ogma_core GTest::gtest_main pthread)

add_executable(pair_homeostatic_actiondecoder tests/ogma/pair_homeostatic_actiondecoder.cpp)
target_link_libraries(pair_homeostatic_actiondecoder PRIVATE ogma_core GTest::gtest_main pthread)

# v4 Phase 1.5 — ActionDecoder
add_executable(test_action_decoder tests/ogma/test_action_decoder.cpp)
target_link_libraries(test_action_decoder PRIVATE ogma_core GTest::gtest_main pthread)

add_executable(pair_lateralvoter_actiondecoder tests/ogma/pair_lateralvoter_actiondecoder.cpp)
target_link_libraries(pair_lateralvoter_actiondecoder PRIVATE ogma_core GTest::gtest_main pthread)

# v4 Phase 2 — thin-slice replay (5-module integration on OGFS golden capture)
add_executable(test_thin_slice_replay tests/ogma/test_thin_slice_replay.cpp)
target_link_libraries(test_thin_slice_replay PRIVATE ogma_core GTest::gtest_main pthread)

# v4 Phase 1.6 — DescendingPredictor
add_executable(test_descending_predictor tests/ogma/test_descending_predictor.cpp)
target_link_libraries(test_descending_predictor PRIVATE ogma_core GTest::gtest_main pthread)

add_executable(pair_descendingpred_epm tests/ogma/pair_descendingpred_epm.cpp)
target_link_libraries(pair_descendingpred_epm PRIVATE ogma_core GTest::gtest_main pthread)

# v4 Phase 1.7 — SequenceGNG
add_executable(test_sequence_gng tests/ogma/test_sequence_gng.cpp)
target_link_libraries(test_sequence_gng PRIVATE ogma_core GTest::gtest_main pthread)

add_executable(pair_seqgng_epm tests/ogma/pair_seqgng_epm.cpp)
target_link_libraries(pair_seqgng_epm PRIVATE ogma_core GTest::gtest_main pthread)

# v4 Phase 1.8 — GNGRollout
add_executable(test_gng_rollout tests/ogma/test_gng_rollout.cpp)
target_link_libraries(test_gng_rollout PRIVATE ogma_core GTest::gtest_main pthread)

# v4 Phase 1.9 — MotorRepertoire
add_executable(test_motor_repertoire tests/ogma/test_motor_repertoire.cpp)
target_link_libraries(test_motor_repertoire PRIVATE ogma_core GTest::gtest_main pthread)

add_executable(pair_motorrepertoire_seqgng tests/ogma/pair_motorrepertoire_seqgng.cpp)
target_link_libraries(pair_motorrepertoire_seqgng PRIVATE ogma_core GTest::gtest_main pthread)

# v4 Phase 5b — HomeokineticExploration (sixth doc-primitive, dead-zone gate).
add_executable(test_homeokinetic_exploration tests/ogma/test_homeokinetic_exploration.cpp)
target_link_libraries(test_homeokinetic_exploration PRIVATE ogma_core GTest::gtest_main pthread)

add_executable(pair_homeokinetic_actiondecoder tests/ogma/pair_homeokinetic_actiondecoder.cpp)
target_link_libraries(pair_homeokinetic_actiondecoder PRIVATE ogma_core GTest::gtest_main pthread)

# v4 Phase 3 — deferred Phase-1 pair tests now wired to real ActionDecoder.
add_executable(pair_gngrollout_actiondecoder tests/ogma/pair_gngrollout_actiondecoder.cpp)
target_link_libraries(pair_gngrollout_actiondecoder PRIVATE ogma_core GTest::gtest_main pthread)

add_executable(pair_motorrepertoire_actiondecoder tests/ogma/pair_motorrepertoire_actiondecoder.cpp)
target_link_libraries(pair_motorrepertoire_actiondecoder PRIVATE ogma_core GTest::gtest_main pthread)

# v4 Phase 3 — hot-patch API exercised against a running OgmaInstance.
add_executable(test_hot_patch tests/ogma/test_hot_patch.cpp)
target_link_libraries(test_hot_patch PRIVATE ogma_core GTest::gtest_main pthread)

# v4 Phase 3 — full cellular-bath E2E + JSON roundtrip + ConnectOp/DisconnectOp.
add_executable(test_phase3_integration tests/ogma/test_phase3_integration.cpp)
target_link_libraries(test_phase3_integration PRIVATE ogma_core GTest::gtest_main pthread)

# v4 Phase 6.5.4 — OgmaInstance::clone() determinism contract.
# Phase 6.6.F — MotorFader brain↔reflex crossfade primitive.
add_executable(test_motor_fader_streams tests/ogma/test_motor_fader_streams.cpp)
target_link_libraries(test_motor_fader_streams PRIVATE ogma_core GTest::gtest_main pthread)

add_executable(test_motor_epm tests/ogma/test_motor_epm.cpp)
target_link_libraries(test_motor_epm PRIVATE ogma_core GTest::gtest_main pthread)

add_executable(test_postural_prior tests/ogma/test_postural_prior.cpp)
target_link_libraries(test_postural_prior PRIVATE ogma_core GTest::gtest_main pthread)

add_executable(test_state_prior tests/ogma/test_state_prior.cpp)
target_link_libraries(test_state_prior PRIVATE ogma_core GTest::gtest_main pthread)

add_executable(test_keyframe_gait tests/ogma/test_keyframe_gait.cpp)
target_link_libraries(test_keyframe_gait PRIVATE ogma_core GTest::gtest_main pthread)

add_executable(test_body_rhythm_tracker tests/ogma/test_body_rhythm_tracker.cpp)
target_link_libraries(test_body_rhythm_tracker PRIVATE ogma_core GTest::gtest_main pthread)

add_executable(test_gain_evolver tests/ogma/test_gain_evolver.cpp)
target_link_libraries(test_gain_evolver PRIVATE ogma_core GTest::gtest_main pthread)

add_executable(test_diag_lite tests/ogma/test_diag_lite.cpp)
target_link_libraries(test_diag_lite PRIVATE ogma_core GTest::gtest_main pthread)

add_executable(test_phase6_6f_verification tests/ogma/test_phase6_6f_verification.cpp)
target_link_libraries(test_phase6_6f_verification PRIVATE ogma_core GTest::gtest_main pthread)

# Phase 6.6.G — FaderController (α-computation extracted from MotorFader).
add_executable(test_fader_controller_streams tests/ogma/test_fader_controller_streams.cpp)
target_link_libraries(test_fader_controller_streams PRIVATE ogma_core GTest::gtest_main pthread)

# Phase 6.6.G — Premotor bilateral output mode.
add_executable(test_premotor_bilateral tests/ogma/test_premotor_bilateral.cpp)
target_link_libraries(test_premotor_bilateral PRIVATE ogma_core GTest::gtest_main pthread)

# Phase 6.6.G — bilateral fader chain end-to-end.
add_executable(test_phase6_6g_verification tests/ogma/test_phase6_6g_verification.cpp)
target_link_libraries(test_phase6_6g_verification PRIVATE ogma_core GTest::gtest_main pthread)

# Phase 6.6.G — consolidated Cell-environment reflex state machine.
add_executable(test_cell_reflex_streams tests/ogma/test_cell_reflex_streams.cpp)
target_link_libraries(test_cell_reflex_streams PRIVATE ogma_core GTest::gtest_main pthread)

# 2026-06-23 — vision+scent sensor-fusion proof-of-concept modules.
add_executable(test_visual_bearing tests/ogma/test_visual_bearing.cpp)
target_link_libraries(test_visual_bearing PRIVATE ogma_core GTest::gtest_main pthread)
add_executable(test_bearing_fusion tests/ogma/test_bearing_fusion.cpp)
target_link_libraries(test_bearing_fusion PRIVATE ogma_core GTest::gtest_main pthread)
add_executable(test_scent_homing_learner tests/ogma/test_scent_homing_learner.cpp)
target_link_libraries(test_scent_homing_learner PRIVATE ogma_core GTest::gtest_main pthread)
add_executable(test_saccade_reflex tests/ogma/test_saccade_reflex.cpp)
target_link_libraries(test_saccade_reflex PRIVATE ogma_core GTest::gtest_main pthread)
add_executable(test_cylinder_builder tests/ogma/test_cylinder_builder.cpp)
target_link_libraries(test_cylinder_builder PRIVATE ogma_core GTest::gtest_main pthread)
add_executable(test_place_graph_planner tests/ogma/test_place_graph_planner.cpp)
target_link_libraries(test_place_graph_planner PRIVATE ogma_core GTest::gtest_main pthread)
add_executable(test_play_loop tests/ogma/test_play_loop.cpp)
target_link_libraries(test_play_loop PRIVATE ogma_core GTest::gtest_main pthread)

add_executable(test_visual_homing_nav tests/ogma/test_visual_homing_nav.cpp)
target_link_libraries(test_visual_homing_nav PRIVATE ogma_core GTest::gtest_main pthread)
add_executable(test_run_tumble_nav tests/ogma/test_run_tumble_nav.cpp)
target_link_libraries(test_run_tumble_nav PRIVATE ogma_core GTest::gtest_main pthread)
add_executable(test_run_tumble_nav_v2 tests/ogma/test_run_tumble_nav_v2.cpp)
target_link_libraries(test_run_tumble_nav_v2 PRIVATE ogma_core GTest::gtest_main pthread)
add_executable(test_place_nav tests/ogma/test_place_nav.cpp)
target_link_libraries(test_place_nav PRIVATE ogma_core GTest::gtest_main pthread)
add_executable(test_efe_arbiter tests/ogma/test_efe_arbiter.cpp)
target_link_libraries(test_efe_arbiter PRIVATE ogma_core GTest::gtest_main pthread)
add_executable(test_gradient_epm tests/ogma/test_gradient_epm.cpp)
target_link_libraries(test_gradient_epm PRIVATE ogma_core GTest::gtest_main pthread)
add_executable(test_klinotaxis tests/ogma/test_klinotaxis.cpp)
target_link_libraries(test_klinotaxis PRIVATE ogma_core GTest::gtest_main pthread)
add_executable(test_column_builder tests/ogma/test_column_builder.cpp)
target_link_libraries(test_column_builder PRIVATE ogma_core GTest::gtest_main pthread)
add_executable(test_bearing_estimator tests/ogma/test_bearing_estimator.cpp)
target_link_libraries(test_bearing_estimator PRIVATE ogma_core GTest::gtest_main pthread)

# Phase v5.2 — sensor / motor downsampler for multi-rate brain.
add_executable(test_keyframe_averager_streams tests/ogma/test_keyframe_averager_streams.cpp)
target_link_libraries(test_keyframe_averager_streams PRIVATE ogma_core GTest::gtest_main pthread)

# Phase 6.6.E — predicted_pathway + EmbeddingRegistry + voter surprise modulation.
add_executable(test_embedding_registry tests/ogma/test_embedding_registry.cpp)
target_link_libraries(test_embedding_registry PRIVATE ogma_core GTest::gtest_main pthread)

add_executable(test_epm_predicted_pathway tests/ogma/test_epm_predicted_pathway.cpp)
target_link_libraries(test_epm_predicted_pathway PRIVATE ogma_core GTest::gtest_main pthread)

add_executable(test_voter_surprise_modulation tests/ogma/test_voter_surprise_modulation.cpp)
target_link_libraries(test_voter_surprise_modulation PRIVATE ogma_core GTest::gtest_main pthread)

# Phase 6.6.E — end-to-end verification (real EPMs + real LateralVoter,
# bit-identical-numbers safeguard borrowed from 6.5.37).
add_executable(test_phase6_6e_verification tests/ogma/test_phase6_6e_verification.cpp)
target_link_libraries(test_phase6_6e_verification PRIVATE ogma_core GTest::gtest_main pthread)

add_executable(test_clone tests/ogma/test_clone.cpp)
target_link_libraries(test_clone PRIVATE ogma_core GTest::gtest_main pthread)

# W3.1 — determinism contract on shipping JSON configs.  Skips configs whose
# modules still fall through to base no-op snapshot_state; flips to PASS as
# entries leave kModulesMissingSnapshot in test_clone_shipping_configs.cpp.
add_executable(test_clone_shipping_configs
    tests/ogma/test_clone_shipping_configs.cpp)
target_link_libraries(test_clone_shipping_configs
    PRIVATE ogma_core GTest::gtest_main pthread)

# W3.6 — disk-round-trip smoke test.  Serialises snapshot via dump()/parse()
# through a temp file before restore, catching float-precision losses that
# the in-memory clone() path can't surface.
add_executable(test_snapshot_disk_round_trip
    tests/ogma/test_snapshot_disk_round_trip.cpp)
target_link_libraries(test_snapshot_disk_round_trip
    PRIVATE ogma_core GTest::gtest_main pthread)

# Manual-routing aux-send filter contract — exercises auto/manual switch,
# explicit-edge gating, runtime toggle non-destructiveness.
add_executable(test_manual_routing
    tests/ogma/test_manual_routing.cpp)
target_link_libraries(test_manual_routing
    PRIVATE ogma_core GTest::gtest_main pthread)

# Patch-mode workflow harness — end-to-end scenarios mimicking human-paced
# patch-mode interactions.  Covers sequential-remove, blank-canvas-build,
# auto_subscribe toggling, manual-mode edge ablation, snapshot interleaving,
# inspector-reader race proxies, and remove-readd cycles.
add_executable(test_patch_mode_harness
    tests/ogma/test_patch_mode_harness.cpp)
target_link_libraries(test_patch_mode_harness
    PRIVATE ogma_core GTest::gtest_main pthread)

include(GoogleTest)
gtest_discover_tests(test_encoder_jl)
gtest_discover_tests(test_gng)
gtest_discover_tests(test_epm)
gtest_discover_tests(test_mitosis)
gtest_discover_tests(test_ogma_headers)
gtest_discover_tests(test_rng_parity)
gtest_discover_tests(test_inprocess_bus)
gtest_discover_tests(test_neurochem_state)
gtest_discover_tests(pair_neurochem_epm)
gtest_discover_tests(test_epm_module)
gtest_discover_tests(pair_epm_lateralvoter)
gtest_discover_tests(test_lateral_voter)
gtest_discover_tests(test_lateral_voter_streams)
gtest_discover_tests(test_premotor_streams)
gtest_discover_tests(test_action_gate_streams)
gtest_discover_tests(test_reflex_modules_streams)
gtest_discover_tests(test_homeostatic_drive)
gtest_discover_tests(pair_lateralvoter_homeostatic)
gtest_discover_tests(pair_homeostatic_actiondecoder)
gtest_discover_tests(test_action_decoder)
gtest_discover_tests(pair_lateralvoter_actiondecoder)
gtest_discover_tests(test_thin_slice_replay)
gtest_discover_tests(test_descending_predictor)
gtest_discover_tests(pair_descendingpred_epm)
gtest_discover_tests(test_sequence_gng)
gtest_discover_tests(pair_seqgng_epm)
gtest_discover_tests(test_gng_rollout)
gtest_discover_tests(test_motor_repertoire)
gtest_discover_tests(pair_motorrepertoire_seqgng)
gtest_discover_tests(pair_gngrollout_actiondecoder)
gtest_discover_tests(pair_motorrepertoire_actiondecoder)
gtest_discover_tests(test_hot_patch)
gtest_discover_tests(test_phase3_integration)
gtest_discover_tests(test_clone)
gtest_discover_tests(test_clone_shipping_configs)
gtest_discover_tests(test_snapshot_disk_round_trip)
gtest_discover_tests(test_manual_routing)
gtest_discover_tests(test_patch_mode_harness)
gtest_discover_tests(test_embedding_registry)
gtest_discover_tests(test_epm_predicted_pathway)
gtest_discover_tests(test_voter_surprise_modulation)
gtest_discover_tests(test_phase6_6e_verification)
gtest_discover_tests(test_motor_fader_streams)
gtest_discover_tests(test_motor_epm)
gtest_discover_tests(test_postural_prior)
gtest_discover_tests(test_state_prior)
gtest_discover_tests(test_keyframe_gait)
gtest_discover_tests(test_body_rhythm_tracker)
gtest_discover_tests(test_gain_evolver)
gtest_discover_tests(test_diag_lite)
gtest_discover_tests(test_phase6_6f_verification)
gtest_discover_tests(test_fader_controller_streams)
gtest_discover_tests(test_premotor_bilateral)
gtest_discover_tests(test_phase6_6g_verification)
gtest_discover_tests(test_cell_reflex_streams)
gtest_discover_tests(test_visual_bearing)
gtest_discover_tests(test_bearing_fusion)
gtest_discover_tests(test_scent_homing_learner)
gtest_discover_tests(test_saccade_reflex)
gtest_discover_tests(test_cylinder_builder)
gtest_discover_tests(test_place_graph_planner)
gtest_discover_tests(test_play_loop)
gtest_discover_tests(test_visual_homing_nav)
gtest_discover_tests(test_run_tumble_nav)
gtest_discover_tests(test_run_tumble_nav_v2)
gtest_discover_tests(test_place_nav)
gtest_discover_tests(test_efe_arbiter)
gtest_discover_tests(test_gradient_epm)
gtest_discover_tests(test_klinotaxis)
gtest_discover_tests(test_column_builder)
gtest_discover_tests(test_bearing_estimator)
gtest_discover_tests(test_keyframe_averager_streams)

# ---------------------------------------------------------------------------
# Synthetic bench for the Kalman-lessons campaign
# (docs/plans-and-designs/epm_kalman_lessons_plan.md, Stage 0.1).  NOT a test:
# a JSONL emitter that drives the real EPM / DescendingPredictor / LateralVoter
# on an InProcessBus against streams whose optimal estimator is known, so
# tools/epm_bench/analyze.py can compare the substrate to the Kalman reference.
# ---------------------------------------------------------------------------
add_executable(epm_kalman_bench bench/epm_kalman_bench.cpp)
target_link_libraries(epm_kalman_bench PRIVATE ogma_core pthread)

endif() # OGMA_BUILD_TESTS
