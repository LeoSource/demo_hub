cmake_minimum_required(VERSION 3.0.0)
project(factorial_test VERSION 0.1.0 LANGUAGES C CXX)

find_package(gtest CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
message(STATUS ${GTest_INCLUDE_DIRS})

include_directories("D:/vcpkg/installed/x64-windows/include")
add_executable(factorial_test factorial_test.cpp)
target_link_libraries(factorial_test PRIVATE GTest::gtest GTest::gtest_main)

add_test(NAME simple_test COMMAND factorial_test)
