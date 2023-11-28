cmake_minimum_required(VERSION 3.0.0)
project(demo_ros_controllers VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
add_executable(demo_quintic_spline_segment demo_quintic_spline_segment.cpp)
add_executable(demo_cartesian_trajectory_segment demo_cartesian_trajectory_segment.cpp cartesian_state.cpp cartesian_trajectory_segment.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
