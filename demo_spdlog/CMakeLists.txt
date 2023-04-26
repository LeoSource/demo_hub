cmake_minimum_required(VERSION 3.0.0)
project(spdlog_demo VERSION 0.1.0)

include(CTest)
enable_testing()

set(CMAKE_CXX_FLAGS "-pthread -std=c++11")
if(WIN32)
    include_directories("F:/0_project/spdlog_demo/include")
elseif(UNIX)
    include_directories("/usr/local/include")
endif()
# find_package(spdlog CONFIG REQUIRED)
# find_package(fmt)
# include_directories(${spdlog_INCLUDE_DIRS})

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

# message(${fmt_DIR})
# if(${spdlog_FOUND})
#   # message(${spdlog_INCLUDE_DIRS})
# else((${spdlog_FOUND}))
#   message(FATAL_ERROR "spdlog library not found!!")
# endif()
add_executable(spdlog_demo main.cpp)
# target_link_libraries(spdlog_demo D:/vcpkg/installed/x64-windows/lib/fmt.lib
#     D:/vcpkg/installed/x64-windows/lib/spdlog.lib
#     )

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
