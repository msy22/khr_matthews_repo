cmake_minimum_required(VERSION 2.8.3)
project(knee_high_robotics)

## Compile as C++11, supported in ROS Kinetic and newer (also works w/ Indigo)
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  roslaunch
)

## Declare a catkin package - necessary otherwise .cpp file won't be callable
## The INCLUDE_DIRS and LIBRARIES stuff is mostly for exporting purposes
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES knee_high_robotics
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


# Executables (.cpp files won't be compiled properly without these lines)
add_executable(robot_navigator         src/robot_navigator.cpp)
add_executable(trimble_access_listener src/trimble_access_listener.cpp)


# Linking
target_link_libraries(robot_navigator         ${catkin_LIBRARIES})
target_link_libraries(trimble_access_listener ${catkin_LIBRARIES})
