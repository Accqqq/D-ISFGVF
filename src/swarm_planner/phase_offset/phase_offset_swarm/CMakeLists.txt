cmake_minimum_required(VERSION 3.0.2)
project(phase_offset_swarm)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  phase_offset_msgs
  roscpp
  std_msgs
  std_srvs
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES phase_offset_swarm_core
  CATKIN_DEPENDS geometry_msgs nav_msgs phase_offset_msgs roscpp std_msgs std_srvs
  DEPENDS Eigen3
)

add_library(phase_offset_swarm_core
  src/neighbor_manager.cpp
  src/elastic_interaction.cpp
  src/conflict_friction.cpp
  src/swarm_intent.cpp
  src/sph_intent.cpp
  src/swarm_backend.cpp
  src/sph_provider_runtime.cpp
)

target_include_directories(phase_offset_swarm_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(TARGET Eigen3::Eigen)
  target_link_libraries(phase_offset_swarm_core PUBLIC Eigen3::Eigen)
else()
  target_include_directories(phase_offset_swarm_core SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
  )
endif()

add_executable(swarm_intent_shadow_node
  src/swarm_intent_shadow_node.cpp
)
target_include_directories(swarm_intent_shadow_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(swarm_intent_shadow_node
  phase_offset_swarm_core
  ${catkin_LIBRARIES}
)
add_dependencies(swarm_intent_shadow_node
  ${catkin_EXPORTED_TARGETS}
  ${phase_offset_msgs_EXPORTED_TARGETS}
)

add_executable(agent_state_neighbor_runtime_node
  src/agent_state_neighbor_runtime_node.cpp
)
target_include_directories(agent_state_neighbor_runtime_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(agent_state_neighbor_runtime_node
  phase_offset_swarm_core
  ${catkin_LIBRARIES}
)
add_dependencies(agent_state_neighbor_runtime_node
  ${catkin_EXPORTED_TARGETS}
  ${phase_offset_msgs_EXPORTED_TARGETS}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  catkin_add_gtest(phase_offset_swarm_neighbor_manager_test
    test/neighbor_manager_test.cpp
  )
  if(TARGET phase_offset_swarm_neighbor_manager_test)
    target_link_libraries(phase_offset_swarm_neighbor_manager_test
      phase_offset_swarm_core
    )
  endif()

  catkin_add_gtest(phase_offset_swarm_elastic_interaction_test
    test/elastic_interaction_test.cpp
  )
  if(TARGET phase_offset_swarm_elastic_interaction_test)
    target_link_libraries(phase_offset_swarm_elastic_interaction_test
      phase_offset_swarm_core
    )
  endif()

  catkin_add_gtest(phase_offset_swarm_conflict_friction_test
    test/conflict_friction_test.cpp
  )
  if(TARGET phase_offset_swarm_conflict_friction_test)
    target_link_libraries(phase_offset_swarm_conflict_friction_test
      phase_offset_swarm_core
    )
  endif()

  catkin_add_gtest(phase_offset_swarm_intent_test
    test/swarm_intent_test.cpp
  )
  if(TARGET phase_offset_swarm_intent_test)
    target_link_libraries(phase_offset_swarm_intent_test
      phase_offset_swarm_core
    )
  endif()

  add_rostest(test/swarm_intent_shadow.test)
  add_rostest(test/agent_state_neighbor_transport.test)
endif()

install(TARGETS phase_offset_swarm_core swarm_intent_shadow_node
                agent_state_neighbor_runtime_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(PROGRAMS
  test/swarm_intent_shadow_test.py
  test/agent_state_neighbor_transport_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
