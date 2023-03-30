cmake_minimum_required(VERSION 3.0.0)
project(demo_eigen VERSION 0.1.0)

include(CTest)
enable_testing()

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message(${EIGEN3_INCLUDE_DIR})

add_executable(demo_eigen main.cpp test_eigen.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
