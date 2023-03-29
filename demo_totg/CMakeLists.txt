cmake_minimum_required(VERSION 3.0.0)
project(demo_totg VERSION 0.1.0)

include(CTest)
enable_testing()

# set(CMAKE_CXX_FLAGS "-std=c++14")

include_directories("/usr/include/eigen3")

set(SRC_LIST demo_totg.cpp Path.cpp Trajectory.cpp)
add_executable(demo_totg ${SRC_LIST})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
