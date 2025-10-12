cmake_minimum_required(VERSION 3.0.2)
project(gust)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gust
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs tf tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

add_executable(gust_manual_node src/gust/gust_manual_node.cpp)
add_dependencies(gust_manual_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(gust_manual_node
  ${catkin_LIBRARIES}
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libmotor_init_single.so
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libcontrolcan.so

)

add_library(cmd SHARED src/gust/cmd.cpp)

add_executable(gust_robot_node src/gust/gust_robot_node.cpp)
add_dependencies(gust_robot_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(gust_robot_node
  ${catkin_LIBRARIES}
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libmotor_init_single.so
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libcontrolcan.so
  cmd
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/librobot.so
  
)


