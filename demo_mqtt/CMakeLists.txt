cmake_minimum_required(VERSION 3.0.0)
project(mqtt_test VERSION 0.1.0)

include(CTest)
enable_testing()

add_definitions(-w)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_CXX_FLAGS "-std=c++11 -pthread")
set(CMAKE_BUILD_TYPE "Release")
message(${CMAKE_CROSSCOMPILING})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

if(${CMAKE_CROSSCOMPILING})
    include_directories(${PROJECT_SOURCE_DIR}/include)
    include_directories(${PROJECT_SOURCE_DIR}/include/mqtt)
    link_directories(${PROJECT_SOURCE_DIR}/lib)
else()
    include_directories("/usr/local/include")
    link_directories("/usr/local/lib")
endif()

add_executable(mqtt_test main.cpp MQTTCommunication.cpp robot_state.cpp)
target_link_libraries(mqtt_test paho-mqttpp3
                                paho-mqtt3a 
                                paho-mqtt3c)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
