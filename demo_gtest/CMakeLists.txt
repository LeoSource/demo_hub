cmake_minimum_required(VERSION 3.0.0)
project(demo_gtest VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()

find_package(gtest CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
message(STATUS ${GTest_INCLUDE_DIRS})

include_directories("D:/vcpkg/installed/x64-windows/include")
add_executable(demo_gtest main.cpp)
target_link_libraries(demo_gtest PRIVATE GTest::gtest GTest::gtest_main)

add_subdirectory(./test)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
