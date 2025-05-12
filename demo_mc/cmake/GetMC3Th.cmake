include(FetchContent)

message(STATUS "get motion_control_third_party ...")

FetchContent_Declare(
  motion_control_third_party #
  GIT_REPOSITORY https://${CODE_AGIBOT_COM_HTTP_TOKEN}code.agibot.com/agibot_playground/motion_control/motion_control_third_party.git # main
  GIT_TAG a74e1ce0dd40a06e19c7c7b194683a82a4f6e9f0)

FetchContent_GetProperties(motion_control_third_party)
if(NOT motion_control_third_party_POPULATED)
  FetchContent_MakeAvailable(motion_control_third_party)
  include(${motion_control_third_party_SOURCE_DIR}/cmake/FindPackage.cmake)
endif()
