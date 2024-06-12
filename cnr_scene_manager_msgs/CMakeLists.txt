cmake_minimum_required(VERSION 3.5)
project(cnr_scene_manager_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
set(CMAKE_CXX_STANDARD 1 4)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        srv/RemoveObjects.srv
        srv/AddObjects.srv
        msg/Object.msg
    DEPENDENCIES
        geometry_msgs
)

## System dependencies are found with CMake's conventions

ament_export_dependencies(geometry_msgs std_msgs std_srvs message_generation)
ament_package()

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++2a -funroll-loops -Wall -Ofast)
