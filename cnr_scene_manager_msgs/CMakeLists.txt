cmake_minimum_required(VERSION 3.0.2)
project(cnr_scene_manager_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++2a -funroll-loops -Wall -Ofast)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  std_srvs
  roscpp
  message_generation
)

add_message_files( DIRECTORY msg
                   FILES Object.msg)

add_service_files( DIRECTORY srv
                   FILES
                   AddObjects.srv
                   RemoveObjects.srv
                   MoveObjects.srv
                 )

generate_messages(DEPENDENCIES
                  geometry_msgs
                  std_msgs
                  std_srvs
)

## System dependencies are found with CMake's conventions
catkin_package( CATKIN_DEPENDS
    geometry_msgs
    std_msgs
    std_srvs
    message_generation
)
