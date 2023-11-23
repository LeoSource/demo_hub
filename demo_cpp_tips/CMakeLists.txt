cmake_minimum_required(VERSION 3.0.0)
project(demo_cpp_tips VERSION 0.1.0)

include(CTest)
enable_testing()

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

if(NOT CMAKE_CROSSCOMPILING)
    add_definitions(-DSIMULATION -Dhello="hello cmake")
endif()

set(CMAKE_CXX_FLAGS "-std=c++11 -pthread")
aux_source_directory(. SRC_LIST)
message(${SRC_LIST})
list(REMOVE_ITEM SRC_LIST ./remove.cpp)
message(${SRC_LIST})
add_executable(demo_cpp_tips ${SRC_LIST})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
