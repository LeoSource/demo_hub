cmake_minimum_required(VERSION 3.0.0)
project(demo_cpp_tips VERSION 0.1.0)

include(CTest)
enable_testing()

add_executable(demo_cpp_tips main.cpp test_other.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
