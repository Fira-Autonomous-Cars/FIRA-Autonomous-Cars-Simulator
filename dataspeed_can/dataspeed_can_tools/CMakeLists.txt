cmake_minimum_required(VERSION 3.0.2)
project(dataspeed_can_tools)

find_package(catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  roslib
  std_msgs
  can_msgs
)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  message(STATUS "Enabling coverage testing")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
endif()

if (NOT "${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++11")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif ("${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}" STREQUAL "98")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++11")
  set(CMAKE_CXX_STANDARD 11)
endif()

add_compile_options(${TARGET_NAME} -Wall -Wextra -Werror) #-pedantic fails in rosconsole https://github.com/ros/rosconsole/issues/9

catkin_package(
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/DbcIterator.cpp src/DbcMessage.cpp src/DbcSignal.cpp
  src/CanExtractor.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(dbc_node src/dbc_node.cpp)
target_link_libraries(dbc_node
  ${PROJECT_NAME}
)

add_executable(dbc_bag src/dbc_bag.cpp)
target_link_libraries(dbc_bag
  ${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME} dbc_node dbc_bag
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
