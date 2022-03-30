cmake_minimum_required(VERSION 3.0.2)
project(dataspeed_can_msg_filters)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  can_msgs
)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  message(STATUS "Enabling coverage testing")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
endif()

add_compile_options(${TARGET_NAME} -Wall -Wextra -Werror) #-pedantic fails in rosconsole https://github.com/ros/rosconsole/issues/9

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp can_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
