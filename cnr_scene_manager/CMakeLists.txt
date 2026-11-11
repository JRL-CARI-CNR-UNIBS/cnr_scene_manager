cmake_minimum_required(VERSION 3.8)
project(cnr_scene_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-std=c++2a -funroll-loops -Wall -Ofast)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED On)

set(CMAKE_BUILD_TYPE Debug)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(cnr_tf_named_object_loader REQUIRED)
find_package(cnr_scene_manager_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometric_shapes REQUIRED)

add_executable(${PROJECT_NAME} src/cnr_scene_manager_node.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp::yaml-cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${rclcpp_TARGETS} ${ament_index_cpp_TARGETS} ${tf2_eigen_TARGETS} ${geometric_shapes_TARGETS} ${moveit_msgs_TARGETS} ${tf2_ros_TARGETS} ${cnr_scene_manager_msgs_TARGETS} ${cnr_tf_named_object_loader_TARGETS})

add_executable(test_scene_manager src/test.cpp)
target_link_libraries(test_scene_manager PUBLIC ${rclcpp_TARGETS} ${ament_index_cpp_TARGETS} ${tf2_eigen_TARGETS} ${geometric_shapes_TARGETS} ${moveit_msgs_TARGETS} ${tf2_ros_TARGETS} ${cnr_scene_manager_msgs_TARGETS} ${cnr_tf_named_object_loader_TARGETS})

add_executable(test_move src/test_move.cpp)
target_link_libraries(test_move PUBLIC ${rclcpp_TARGETS} ${ament_index_cpp_TARGETS} ${tf2_eigen_TARGETS} ${geometric_shapes_TARGETS} ${moveit_msgs_TARGETS} ${tf2_ros_TARGETS} ${cnr_scene_manager_msgs_TARGETS} ${cnr_tf_named_object_loader_TARGETS})


install(TARGETS
    ${PROJECT_NAME}
    test_move
    test_scene_manager
    DESTINATION lib/${PROJECT_NAME}
    )

install(DIRECTORY config example_meshes launch
    DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rclcpp moveit_msgs moveit_ros_planning_interface tf2_ros Eigen3 yaml-cpp) 
ament_package()


