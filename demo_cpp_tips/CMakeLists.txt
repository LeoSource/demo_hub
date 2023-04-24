cmake_minimum_required(VERSION 3.0.0)
project(demo_cpp_tips VERSION 0.1.0)

include(CTest)
enable_testing()


if(NOT CMAKE_CROSSCOMPILING)
    add_definitions(-DSIMULATION -Dhello="hello cmake")
endif()

set(CMAKE_CXX_FLAGS "-std=c++11")
add_executable(demo_cpp_tips main.cpp test_other.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
