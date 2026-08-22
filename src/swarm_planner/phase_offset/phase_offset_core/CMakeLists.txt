cmake_minimum_required(VERSION 3.0.2)
project(phase_offset_core)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES phase_offset_core
  DEPENDS Eigen3
)

add_library(phase_offset_core
  src/geometry.cpp
  src/matched_port.cpp
  src/port_projector.cpp
)

target_include_directories(phase_offset_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(TARGET Eigen3::Eigen)
  target_link_libraries(phase_offset_core PUBLIC Eigen3::Eigen)
else()
  target_include_directories(phase_offset_core SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
  )
endif()

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(phase_offset_geometry_test
    test/geometry_test.cpp
  )
  if(TARGET phase_offset_geometry_test)
    target_link_libraries(phase_offset_geometry_test
      phase_offset_core
    )
  endif()
  catkin_add_gtest(phase_offset_matched_port_test
    test/matched_port_test.cpp
  )
  if(TARGET phase_offset_matched_port_test)
    target_link_libraries(phase_offset_matched_port_test
      phase_offset_core
    )
  endif()
  catkin_add_gtest(phase_offset_port_projector_test
    test/port_projector_test.cpp
  )
  if(TARGET phase_offset_port_projector_test)
    target_link_libraries(phase_offset_port_projector_test
      phase_offset_core
    )
  endif()
endif()
