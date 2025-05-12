# # Get the current folder name
# string(REGEX REPLACE ".*/\(.*\)" "\\1" CUR_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# # Get namespace
# get_namespace(CUR_SUPERIOR_NAMESPACE)
# string(REPLACE "::" "_" CUR_SUPERIOR_NAMESPACE_UNDERLINE ${CUR_SUPERIOR_NAMESPACE})

# # Set target name
# set(CUR_TARGET_NAME ${CUR_SUPERIOR_NAMESPACE_UNDERLINE}_${CUR_DIR})
# set(CUR_TARGET_ALIAS_NAME ${CUR_SUPERIOR_NAMESPACE}::${CUR_DIR})

# # Set file collection
# file(GLOB_RECURSE head_files ${CMAKE_CURRENT_SOURCE_DIR}/*.h ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp)
# file(GLOB_RECURSE src ${CMAKE_CURRENT_SOURCE_DIR}/*.cc ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
# file(GLOB_RECURSE test_files ${CMAKE_CURRENT_SOURCE_DIR}/*_test.cc)
# list(REMOVE_ITEM src ${test_files})

# Add target
# add_library(${CUR_TARGET_NAME} SHARED)
# add_library(${CUR_TARGET_ALIAS_NAME} ALIAS ${CUR_TARGET_NAME})

# Set source file of target
# target_sources(${CUR_TARGET_NAME} PRIVATE ${src})

# target_compile_options(${CUR_TARGET_NAME} PRIVATE -ggdb)

# find_package(Eigen3 REQUIRED)
# if(${Eigen3_FOUND})
#   message(STATUS "gsgdrhhfhfhf-===")
# endif()
set(CUR_TARGET_NAME collision_check)
add_executable(${CUR_TARGET_NAME} collision_checker.cpp)

# Set include path of target
target_include_directories(
  ${CUR_TARGET_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         ${PCL_INCLUDE_DIRS}
         ${hpp-fcl_INCLUDE_DIRS}
         ${pinocchio_INCLUDE_DIRS}
        #  ${Eigen3_INCLUDE_DIRS}
         )

# Set head files of target
# target_sources(${CUR_TARGET_NAME} INTERFACE FILE_SET HEADERS BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} FILES ${head_files})

# Set link libraries of target
target_link_libraries(
  ${CUR_TARGET_NAME}
  PUBLIC
#   PUBLIC pinocchio::pinocchio
        #  motion_control::common::data_board
        #  motion_control::core::math
        #  ocs2::ocs2_core
        #  ocs2::ocs2_pinocchio::ocs2_pinocchio_interface
        #  ocs2::ocs2_pinocchio::ocs2_self_collision
        #  motion_control::base
        #  motion_control::api
        #  motion_control::data
        #  motion_control::common::db
        #  motion_control::manager::aimrt
        #  motion_control::module::main::converts
        #  motion_control::manager::publisher
        #  aimdk::protocol::hal
        #  ${PCL_LIBRARIES}
        ${hpp-fcl_LIBRARIES}
        ${pinocchio_LIBRARIES}
         )

# target_compile_definitions(${CUR_TARGET_NAME} PRIVATE ${PINOCCHIO_CFLAGS_OTHER})
# # Set installation of target
# set_property(TARGET ${CUR_TARGET_NAME} PROPERTY EXPORT_NAME ${CUR_TARGET_ALIAS_NAME})
# install(
#   TARGETS ${CUR_TARGET_NAME}
#   EXPORT ${INSTALL_CONFIG_NAME}
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#           FILE_SET HEADERS
#           DESTINATION include/${CUR_DIR})


