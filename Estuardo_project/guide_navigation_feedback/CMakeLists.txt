cmake_minimum_required(VERSION 2.8.3)
project(guide_navigation_feedback)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sound_play nav_msgs)

set( CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(guide_navigation_feedback guide_navigation_feedback.cpp)
target_link_libraries(guide_navigation_feedback ${catkin_LIBRARIES})
