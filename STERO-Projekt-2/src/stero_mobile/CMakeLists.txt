cmake_minimum_required(VERSION 3.5)
project(stero_mobile_init)
set(CMAKE_CXX_STANDARD 11)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  roscpp
  move_base
  tf2
  tf2_ros
  costmap_2d
  dwa_local_planner
  global_planner
  rotate_recovery
  clear_costmap_recovery
)
catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES stero_mobile
#   CATKIN_DEPENDS roscpp nodelet
#   DEPENDS elektron
)

install (DIRECTORY worlds/ DESTINATION share/${CMAKE_PROJECT_NAME}/worlds 
         FILES_MATCHING PATTERN "*.world")
install (DIRECTORY launch/ DESTINATION share/${CMAKE_PROJECT_NAME}/launch 
         FILES_MATCHING PATTERN "*.launch*")
install (DIRECTORY config/ DESTINATION share/${CMAKE_PROJECT_NAME}/config 
         FILES_MATCHING PATTERN "*.yaml")
install (DIRECTORY param/ DESTINATION share/${CMAKE_PROJECT_NAME}/param 
         FILES_MATCHING PATTERN "*.yaml")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  std_msgs
  rospy
  roscpp
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
 
add_executable(package src/package.cpp)
add_executable(error src/error.cpp)
add_executable(listener src/listener.cpp)

target_link_libraries(package
   ${catkin_LIBRARIES}
)

target_link_libraries(error ${catkin_LIBRARIES})
