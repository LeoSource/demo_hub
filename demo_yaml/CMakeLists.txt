cmake_minimum_required(VERSION 3.0.0)
project(yaml_demo VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 11)
include(CTest)
enable_testing()

set(INC_DIR "${PROJECT_SOURCE_DIR}/include/yaml-cpp")
set(LIB_DIR "${PROJECT_SOURCE_DIR}")
include_directories(${INC_DIR})
# link_directories("/usr/lib/x86_64-linux-gnu/")
add_executable(yaml_demo main.cpp)
target_link_libraries(yaml_demo yaml-cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
