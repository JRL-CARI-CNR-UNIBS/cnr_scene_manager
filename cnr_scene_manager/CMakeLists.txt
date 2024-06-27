cmake_minimum_required(VERSION 3.8)
project(cnr_scene_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-std=c++2a -funroll-loops -Wall -Ofast)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED On)

set(CMAKE_BUILD_TYPE Debug)

find_package(cnr_param REQUIRED)
find_package(cnr_logger REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(cnr_tf_named_object_loader REQUIRED)
find_package(cnr_scene_manager_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometric_shapes REQUIRED)

add_executable(${PROJECT_NAME} src/cnr_scene_manager_node.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC cnr_param::cnr_param cnr_logger::cnr_logger)
ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp tf2_eigen geometric_shapes moveit_msgs tf2_ros cnr_scene_manager_msgs cnr_tf_named_object_loader)

add_executable(test_scene_manager src/test.cpp)
target_link_libraries(test_scene_manager PUBLIC cnr_param::cnr_param cnr_logger::cnr_logger)
ament_target_dependencies(test_scene_manager PUBLIC rclcpp tf2_eigen geometric_shapes moveit_msgs tf2_ros cnr_scene_manager_msgs cnr_tf_named_object_loader)

install(TARGETS
    ${PROJECT_NAME}
    test_scene_manager
    DESTINATION lib/${PROJECT_NAME}
    )

install(DIRECTORY config example_meshes launch script
    DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rclcpp moveit_msgs moveit_ros_planning_interface tf2_ros Eigen3 yaml-cpp) 
ament_package()


