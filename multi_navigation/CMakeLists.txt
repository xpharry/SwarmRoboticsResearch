cmake_minimum_required(VERSION 2.8.3)
project(multi_navigation)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
  rospy
  tf
)

catkin_package()

## Declare a C++ executable
add_executable(multi_navigation_send_goal src/multi_navigation_send_goal.cpp)
add_executable(multi_navigation_send_goal2 src/multi_navigation_send_goal2.cpp)
add_executable(multi_navigation_send_goal3 src/multi_navigation_send_goal3.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(multi_navigation_send_goal ${catkin_LIBRARIES})
target_link_libraries(multi_navigation_send_goal2 ${catkin_LIBRARIES})
target_link_libraries(multi_navigation_send_goal3 ${catkin_LIBRARIES})



