cmake_minimum_required(VERSION 3.0.0)
project(spdlog_demo VERSION 0.1.0)

include(CTest)
enable_testing()

execute_process(COMMAND ${PROJECT_SOURCE_DIR}/xenomai/bin/xeno-config --posix --cflags OUTPUT_VARIABLE XENO_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND ${PROJECT_SOURCE_DIR}/xenomai/bin/xeno-config --auto-init-solib --posix --ldflags OUTPUT_VARIABLE XENO_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)

set(CMAKE_CXX_FLAGS "-pthread -std=c++11")

include_directories("${PROJECT_SOURCE_DIR}/xenomai/include/trank"
                    "${PROJECT_SOURCE_DIR}/xenomai/include/cobalt"
                    "${PROJECT_SOURCE_DIR}/xenomai/include")
include_directories("/usr/local/include/")
link_directories("${PROJECT_SOURCE_DIR}/xenomai/lib")
add_executable(spdlog_demo main.cpp)
target_link_libraries(spdlog_demo ${XENO_CFLAGS} 
                            ${XENO_LDFLAGS})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
