cmake_minimum_required(VERSION 3.0.2)
project(phase_offset_navigation)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS phase_offset_core)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES phase_offset_navigation
  CATKIN_DEPENDS phase_offset_core
  DEPENDS Eigen3
)

add_library(phase_offset_navigation
  src/environment_evidence.cpp
  src/tube_cross_section.cpp
  src/phase_offset_runtime.cpp
  src/active_reference_authority.cpp
  src/phase_offset_recovery_owner.cpp
  src/tube_execution_v2.cpp
  src/tube_viability.cpp
  src/phase_offset_allocator.cpp
  src/tube_certificate_v2.cpp
  src/section_tube.cpp
)
target_compile_definitions(phase_offset_navigation
  PRIVATE PHASE_OFFSET_RUNTIME_V2_PRODUCTION=1)
target_include_directories(phase_offset_navigation
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(phase_offset_navigation PUBLIC phase_offset_core)
if(TARGET Eigen3::Eigen)
  target_link_libraries(phase_offset_navigation PUBLIC Eigen3::Eigen)
else()
  target_include_directories(phase_offset_navigation SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
  )
endif()

if(CATKIN_ENABLE_TESTING)
  # Historical Tube construction remains available only to explicit offline
  # regression executables.  This target is not exported and no production
  # target may link it.
  add_library(phase_offset_navigation_legacy_tube
    src/environment_evidence.cpp
    src/tube_cross_section.cpp
    src/tube_builder.cpp
    src/tube_filter.cpp
    src/tube_surface_validator.cpp
    src/certified_tube_builder.cpp
    src/phase_offset_runtime.cpp
    src/active_reference_authority.cpp
    src/preview_feasibility.cpp
    src/handoff_state_machine.cpp
    src/phase_offset_recovery_owner.cpp
    src/tube_execution_v2.cpp
    src/tube_epoch_manager.cpp
    src/tube_viability.cpp
    src/phase_offset_allocator.cpp
    src/tube_certificate_v2.cpp
  )
  target_include_directories(phase_offset_navigation_legacy_tube
    PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(phase_offset_navigation_legacy_tube
    PUBLIC phase_offset_core)
  if(TARGET Eigen3::Eigen)
    target_link_libraries(phase_offset_navigation_legacy_tube
      PUBLIC Eigen3::Eigen)
  else()
    target_include_directories(phase_offset_navigation_legacy_tube
      SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
  endif()

  catkin_add_gtest(phase_offset_environment_evidence_test
    test/environment_evidence_test.cpp)
  catkin_add_gtest(phase_offset_tube_cross_section_test
    test/tube_cross_section_test.cpp)
  catkin_add_gtest(phase_offset_tube_builder_test test/tube_builder_test.cpp)
  catkin_add_gtest(phase_offset_tube_filter_test test/tube_filter_test.cpp)
  catkin_add_gtest(phase_offset_tube_surface_validator_test
    test/tube_surface_validator_test.cpp)
  catkin_add_gtest(phase_offset_certified_tube_builder_test
    test/certified_tube_builder_test.cpp)
  catkin_add_gtest(phase_offset_runtime_test test/runtime_test.cpp)
  target_compile_definitions(phase_offset_runtime_test
    PRIVATE PHASE_OFFSET_RUNTIME_V2_TEST_ONLY=1)
  catkin_add_gtest(phase_offset_runtime_legacy_test test/runtime_test.cpp)
  target_compile_definitions(phase_offset_runtime_legacy_test
    PRIVATE PHASE_OFFSET_RUNTIME_LEGACY_TEST_ONLY=1)
  catkin_add_gtest(phase_offset_active_reference_authority_test
    test/active_reference_authority_test.cpp)
  catkin_add_gtest(phase_offset_preview_feasibility_test
    test/preview_feasibility_test.cpp)
  catkin_add_gtest(phase_offset_handoff_state_machine_test
    test/handoff_state_machine_test.cpp)
  catkin_add_gtest(phase_offset_recovery_owner_test
    test/phase_offset_recovery_owner_test.cpp)
  catkin_add_gtest(phase_offset_tube_epoch_manager_test
    test/tube_epoch_manager_test.cpp)
  catkin_add_gtest(phase_offset_tube_viability_test
    test/tube_viability_test.cpp)
  catkin_add_gtest(phase_offset_allocator_test
    test/phase_offset_allocator_test.cpp)
  catkin_add_gtest(phase_offset_tube_certificate_v2_test
    test/tube_certificate_v2_test.cpp)
  catkin_add_gtest(phase_offset_tube_execution_v2_test
    test/tube_execution_v2_test.cpp)
  catkin_add_gtest(phase_offset_section_tube_test
    test/section_tube_test.cpp)
  foreach(target
      phase_offset_environment_evidence_test
      phase_offset_tube_cross_section_test
      phase_offset_recovery_owner_test
      phase_offset_tube_viability_test
      phase_offset_allocator_test)
    if(TARGET ${target})
      target_link_libraries(${target}
        phase_offset_navigation
        phase_offset_core
      )
    endif()
  endforeach()
  if(TARGET phase_offset_runtime_test)
    target_link_libraries(phase_offset_runtime_test
      phase_offset_navigation
      phase_offset_core
    )
  endif()
  foreach(target
      phase_offset_tube_builder_test
      phase_offset_tube_filter_test
      phase_offset_tube_surface_validator_test
      phase_offset_certified_tube_builder_test
      phase_offset_runtime_legacy_test
      phase_offset_active_reference_authority_test
      phase_offset_preview_feasibility_test
      phase_offset_handoff_state_machine_test
      phase_offset_tube_epoch_manager_test)
    if(TARGET ${target})
      target_link_libraries(${target}
        phase_offset_navigation_legacy_tube
        phase_offset_core
      )
    endif()
  endforeach()
  if(TARGET phase_offset_tube_certificate_v2_test)
    target_link_libraries(phase_offset_tube_certificate_v2_test
      phase_offset_navigation
      phase_offset_core
    )
  endif()
  if(TARGET phase_offset_tube_execution_v2_test)
    target_link_libraries(phase_offset_tube_execution_v2_test
      phase_offset_navigation
      phase_offset_core
    )
  endif()
  if(TARGET phase_offset_section_tube_test)
    target_link_libraries(phase_offset_section_tube_test
      phase_offset_navigation
      phase_offset_core
    )
  endif()
endif()
