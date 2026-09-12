cmake_minimum_required(VERSION 3.0.2)
project(phase_offset_sim_bringup)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  phase_offset_swarm
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp geometry_msgs nav_msgs phase_offset_swarm visualization_msgs
)

# Launch-only/runtime dependencies are declared in package.xml.  Keeping them
# out of this visualizer target's CMake link interface avoids manufacturing
# links to protected planner/simulator libraries; this package remains a
# top-level assembly node whose manifest edges point downward.

include_directories(${catkin_INCLUDE_DIRS})

add_executable(swarm_visualizer src/swarm_visualizer.cpp)
add_dependencies(swarm_visualizer ${catkin_EXPORTED_TARGETS})
target_link_libraries(swarm_visualizer ${catkin_LIBRARIES})

catkin_install_python(
  PROGRAMS
    scripts/swarm_orchestrator.py
    scripts/scenario_goal_publisher.py
    scripts/clicked_goal_translator.py
    test/sim_c_bringup_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS swarm_visualizer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/sim_b_bringup.test)
  add_rostest(test/sim_c_bringup.test)
endif()
