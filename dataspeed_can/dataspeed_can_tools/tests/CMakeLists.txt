### Unit tests
#
#   Only configured when CATKIN_ENABLE_TESTING is true.

find_package(roslib REQUIRED)
find_package(rostest REQUIRED)

catkin_add_gtest(test_signal test_signal.cpp)
target_link_libraries(test_signal ${PROJECT_NAME})

catkin_add_gtest(test_message test_message.cpp)
target_link_libraries(test_message ${PROJECT_NAME})

catkin_add_gtest(test_iterator test_iterator.cpp)
target_link_libraries(test_iterator ${PROJECT_NAME})

add_rostest_gtest(test_full test_full.test test_full.cpp)
target_link_libraries(test_full ${PROJECT_NAME})
