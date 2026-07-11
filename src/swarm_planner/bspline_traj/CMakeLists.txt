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
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bspline_race
  CATKIN_DEPENDS roscpp rospy tf std_msgs visualization_msgs message_runtime common_msgs plan_env path_searching
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

add_library(bspline_gvf
   src/UniformBspline_3d.cpp
   src/bspline_opt_3d.cpp 
   src/gvf_manager.cpp
   src/gvf.cpp
)
target_link_libraries(bspline_gvf ${catkin_LIBRARIES})


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
  catkin_add_gtest(fastplanner_parameterization_test
    test/fastplanner_parameterization_test.cpp)
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
endif()
