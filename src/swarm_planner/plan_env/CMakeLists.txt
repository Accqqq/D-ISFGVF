cmake_minimum_required(VERSION 2.8.3)
project(plan_env)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -Wall")

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  cv_bridge
  message_filters
  message_generation
  common_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_env
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs common_msgs
 DEPENDS message_generation
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library(plan_env 
    src/local_obstacle_view.cpp
    src/sdf_map.cpp 
    src/obj_predictor.cpp 
    src/edt_environment.cpp 
    src/raycast.cpp
)
target_link_libraries(plan_env
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)

add_executable(obj_generator
    src/obj_generator.cpp 
)
target_link_libraries(obj_generator 
    ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
    # The immutable cloud-snapshot implementation is retained solely for the
    # historical regression target.  It is deliberately not part of the
    # production plan_env library (or any planner executable); the M3C map
    # path uses LocalObstacleView instead.
    add_library(plan_env_cloud_occupancy_snapshot_legacy
        src/cloud_occupancy_snapshot.cpp)
    target_link_libraries(plan_env_cloud_occupancy_snapshot_legacy
        ${catkin_LIBRARIES})

    catkin_add_gtest(manual_map_layer_test test/manual_map_layer_test.cpp)
    catkin_add_gtest(local_obstacle_view_test
        test/local_obstacle_view_test.cpp)
    catkin_add_gtest(local_obstacle_capture_test
        test/local_obstacle_capture_test.cpp)
    catkin_add_gtest(cloud_occupancy_snapshot_test
        test/cloud_occupancy_snapshot_test.cpp)
    if(TARGET manual_map_layer_test)
        target_link_libraries(manual_map_layer_test
            plan_env
            ${catkin_LIBRARIES}
            ${PCL_LIBRARIES}
            ${OpenCV_LIBS}
        )
    endif()
    if(TARGET local_obstacle_view_test)
        target_link_libraries(local_obstacle_view_test
            plan_env
            ${catkin_LIBRARIES}
            ${PCL_LIBRARIES}
            ${OpenCV_LIBS}
        )
    endif()
    if(TARGET local_obstacle_capture_test)
        target_link_libraries(local_obstacle_capture_test
            plan_env
            ${catkin_LIBRARIES}
            ${PCL_LIBRARIES}
            ${OpenCV_LIBS}
        )
    endif()
    if(TARGET cloud_occupancy_snapshot_test)
        target_link_libraries(cloud_occupancy_snapshot_test
            plan_env_cloud_occupancy_snapshot_legacy
            plan_env
            ${catkin_LIBRARIES}
            ${PCL_LIBRARIES}
            ${OpenCV_LIBS}
        )
    endif()
endif()
