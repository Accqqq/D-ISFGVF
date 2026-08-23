cmake_minimum_required(VERSION 3.0.2)
project(bspline_race)
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -Wall")
find_package(NLopt REQUIRED)
set(NLopt_INCLUDE_DIRS ${NLOPT_INCLUDE_DIR})
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  visualization_msgs
  message_generation
  message_filters
  cv_bridge
  common_msgs
  plan_env
  path_searching
  phase_offset_core
  phase_offset_navigation
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bspline_race
  CATKIN_DEPENDS roscpp rospy tf std_msgs visualization_msgs message_runtime common_msgs plan_env path_searching phase_offset_core phase_offset_navigation
  DEPENDS message_generation
)
include_directories(
  SYSTEM
  include 
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${NLOPT_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/devel/include
)

add_library(continuous_phase_path
  src/UniformBspline_3d.cpp
  src/continuous_phase_path.cpp
  src/continuous_phase_normal_frame.cpp
)
target_link_libraries(continuous_phase_path
  ${catkin_LIBRARIES}
)

add_library(bspline_gvf
   src/bspline_opt_3d.cpp 
   src/gvf_manager.cpp
   src/gvf.cpp
)

add_library(isf_reference_kernel
  src/guidance/isf_reference_kernel.cpp
)

add_library(phase_offset_active_adapter
  src/integration/phase_offset_active_adapter.cpp
)
target_link_libraries(phase_offset_active_adapter
  phase_offset_core
  isf_reference_kernel
)

add_library(phase_offset_executed_reference_query
  src/integration/phase_offset_executed_reference_query.cpp
)
target_link_libraries(phase_offset_executed_reference_query
  continuous_phase_path
  phase_offset_core
  phase_offset_navigation
)

add_library(phase_offset_environment_evidence_query
  src/integration/phase_offset_environment_evidence_query.cpp
)
target_link_libraries(phase_offset_environment_evidence_query
  phase_offset_navigation
  plan_env
  ${catkin_LIBRARIES}
)

# Standalone, immutable path/snapshot margin-accounting seam.  It deliberately
# has no Runtime or publisher dependency and leaves existing diagnostics sizes
# unchanged.
add_library(phase_offset_clearance_audit
  src/integration/phase_offset_clearance_audit.cpp
)
target_link_libraries(phase_offset_clearance_audit
  continuous_phase_path
  phase_offset_core
  phase_offset_navigation
  ${catkin_LIBRARIES}
)

add_library(phase_offset_matched_adapter
  src/integration/phase_offset_cloud_occupancy_query.cpp
  src/integration/phase_offset_matched_adapter.cpp
  src/integration/phase_offset_raw_candidate_diagnostics.cpp
  src/integration/phase_offset_tube_epoch_diagnostics.cpp
  src/integration/phase_offset_tube_markers.cpp
)
target_link_libraries(phase_offset_matched_adapter
  continuous_phase_path
  phase_offset_executed_reference_query
  phase_offset_core
  phase_offset_navigation
  plan_env
  isf_reference_kernel
  phase_offset_active_adapter
  ${catkin_LIBRARIES}
)

add_library(phase_offset_shadow_adapter
  src/integration/phase_offset_shadow_adapter.cpp
)
target_link_libraries(phase_offset_shadow_adapter ${catkin_LIBRARIES})

target_link_libraries(bspline_gvf
  ${catkin_LIBRARIES}
  continuous_phase_path
  phase_offset_shadow_adapter
  isf_reference_kernel
  phase_offset_active_adapter
  phase_offset_matched_adapter
)


add_executable( formation_planning 
    src/formation_planning.cpp
 )
target_link_libraries(formation_planning 
bspline_gvf
 ${catkin_LIBRARIES}
 ${NLOPT_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(reference_join_test test/reference_join_test.cpp)
  catkin_add_gtest(gvf_switch_policy_test test/gvf_switch_policy_test.cpp)
  catkin_add_gtest(gvf_lifted_visualization_test test/gvf_lifted_visualization_test.cpp)
  catkin_add_gtest(continuous_phase_path_test test/continuous_phase_path_test.cpp)
  catkin_add_gtest(continuous_phase_normal_frame_test
    test/continuous_phase_normal_frame_test.cpp)
  # Test-only legacy planar geometry coverage.  Keep this target distinct from
  # phase_offset_core's phase_offset_geometry_test; phase_offset_geometry.cpp
  # is intentionally linked here only and is not a production library source.
  catkin_add_gtest(bspline_race_phase_offset_geometry_test
    test/phase_offset_geometry_test.cpp
    src/phase_offset_geometry.cpp)
  catkin_add_gtest(fastplanner_parameterization_test
    test/fastplanner_parameterization_test.cpp)
  catkin_add_gtest(phase_offset_shadow_adapter_test
    test/phase_offset_shadow_adapter_test.cpp)
  catkin_add_gtest(isf_reference_kernel_test
    test/isf_reference_kernel_test.cpp)
  catkin_add_gtest(phase_offset_active_adapter_test
    test/phase_offset_active_adapter_test.cpp)
  catkin_add_gtest(phase_offset_matched_adapter_test
    test/phase_offset_matched_adapter_test.cpp)
  catkin_add_gtest(phase_offset_raw_occupancy_query_test
    test/phase_offset_raw_occupancy_query_test.cpp)
  catkin_add_gtest(phase_offset_cloud_occupancy_query_test
    test/phase_offset_cloud_occupancy_query_test.cpp)
  catkin_add_gtest(phase_offset_environment_evidence_query_test
    test/phase_offset_environment_evidence_query_test.cpp)
  catkin_add_gtest(phase_offset_clearance_audit_test
    test/phase_offset_clearance_audit_test.cpp)
  catkin_add_gtest(phase_offset_raw_candidate_diagnostics_test
    test/phase_offset_raw_candidate_diagnostics_test.cpp)
  catkin_add_gtest(phase_offset_tube_markers_test
    test/phase_offset_tube_markers_test.cpp)
  catkin_add_gtest(phase_offset_tube_epoch_diagnostics_test
    test/phase_offset_tube_epoch_diagnostics_test.cpp)
  catkin_add_gtest(phase_offset_tube_epoch_integration_test
    test/phase_offset_tube_epoch_integration_test.cpp)
  # G2d removes this raw-log-odds bridge from all production targets.  Keep a
  # test-only target so the deprecated reference remains independently
  # exercisable without a path from formation_planning to raw occupancy.
  add_library(phase_offset_raw_occupancy_query_legacy
    src/integration/phase_offset_raw_occupancy_query.cpp)
  target_link_libraries(phase_offset_raw_occupancy_query_legacy
    phase_offset_navigation
    plan_env
    ${catkin_LIBRARIES}
  )
  catkin_add_gtest(phase_offset_executed_reference_query_test
    test/phase_offset_executed_reference_query_test.cpp)
  if(TARGET phase_offset_shadow_adapter_test)
    target_link_libraries(phase_offset_shadow_adapter_test
      phase_offset_shadow_adapter
      ${catkin_LIBRARIES}
    )
  endif()
  if(TARGET isf_reference_kernel_test)
    target_link_libraries(isf_reference_kernel_test
      isf_reference_kernel
    )
  endif()
  if(TARGET phase_offset_active_adapter_test)
    target_link_libraries(phase_offset_active_adapter_test
      phase_offset_active_adapter
      isf_reference_kernel
      phase_offset_core
      ${catkin_LIBRARIES}
    )
  endif()
  if(TARGET phase_offset_executed_reference_query_test)
    target_link_libraries(phase_offset_executed_reference_query_test
      phase_offset_executed_reference_query
      continuous_phase_path
      phase_offset_core
      phase_offset_navigation
    )
  endif()
  if(TARGET phase_offset_matched_adapter_test)
    target_link_libraries(phase_offset_matched_adapter_test
      phase_offset_matched_adapter
      phase_offset_active_adapter
      isf_reference_kernel
      phase_offset_core
      phase_offset_navigation
      ${catkin_LIBRARIES}
    )
  endif()
  if(TARGET phase_offset_raw_occupancy_query_test)
    target_link_libraries(phase_offset_raw_occupancy_query_test
      phase_offset_raw_occupancy_query_legacy
      phase_offset_navigation
      plan_env
      ${catkin_LIBRARIES}
    )
  endif()
  if(TARGET phase_offset_cloud_occupancy_query_test)
    target_link_libraries(phase_offset_cloud_occupancy_query_test
      phase_offset_matched_adapter
      phase_offset_navigation
      plan_env
      ${catkin_LIBRARIES}
    )
  endif()
  if(TARGET phase_offset_environment_evidence_query_test)
    target_link_libraries(phase_offset_environment_evidence_query_test
      phase_offset_environment_evidence_query
      phase_offset_navigation
      plan_env
      ${catkin_LIBRARIES}
    )
  endif()
  if(TARGET phase_offset_clearance_audit_test)
    target_link_libraries(phase_offset_clearance_audit_test
      phase_offset_clearance_audit
      continuous_phase_path
      phase_offset_core
      phase_offset_navigation
      ${catkin_LIBRARIES}
    )
  endif()
  if(TARGET phase_offset_raw_candidate_diagnostics_test)
    target_link_libraries(phase_offset_raw_candidate_diagnostics_test
      phase_offset_matched_adapter
      phase_offset_navigation
      ${catkin_LIBRARIES}
    )
  endif()
  if(TARGET phase_offset_tube_markers_test)
    target_link_libraries(phase_offset_tube_markers_test
      phase_offset_matched_adapter
      continuous_phase_path
      phase_offset_navigation
      ${catkin_LIBRARIES}
    )
  endif()
  if(TARGET phase_offset_tube_epoch_diagnostics_test)
    target_link_libraries(phase_offset_tube_epoch_diagnostics_test
      phase_offset_matched_adapter
      phase_offset_navigation
      ${catkin_LIBRARIES}
    )
  endif()
  if(TARGET phase_offset_tube_epoch_integration_test)
    target_link_libraries(phase_offset_tube_epoch_integration_test
      phase_offset_matched_adapter
      phase_offset_active_adapter
      isf_reference_kernel
      continuous_phase_path
      phase_offset_core
      phase_offset_navigation
      ${catkin_LIBRARIES}
    )
  endif()
  if(TARGET phase_offset_matched_adapter_test)
    target_link_libraries(phase_offset_matched_adapter_test
      continuous_phase_path
    )
  endif()
  if(TARGET fastplanner_parameterization_test)
    target_link_libraries(fastplanner_parameterization_test
      bspline_gvf
      ${catkin_LIBRARIES}
      ${NLOPT_LIBRARIES}
    )
  endif()
  if(TARGET gvf_lifted_visualization_test)
    target_link_libraries(gvf_lifted_visualization_test
      bspline_gvf
      ${catkin_LIBRARIES}
      ${NLOPT_LIBRARIES}
    )
  endif()
  if(TARGET continuous_phase_path_test)
    target_link_libraries(continuous_phase_path_test
      bspline_gvf
      ${catkin_LIBRARIES}
      ${NLOPT_LIBRARIES}
    )
  endif()
  if(TARGET continuous_phase_normal_frame_test)
    target_link_libraries(continuous_phase_normal_frame_test
      continuous_phase_path
      phase_offset_core
      ${catkin_LIBRARIES}
    )
  endif()
  if(TARGET gvf_switch_policy_test)
    target_link_libraries(gvf_switch_policy_test
      bspline_gvf
      ${catkin_LIBRARIES}
      ${NLOPT_LIBRARIES}
    )
  endif()
endif()
