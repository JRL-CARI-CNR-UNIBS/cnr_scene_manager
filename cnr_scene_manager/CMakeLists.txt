cmake_minimum_required(VERSION 3.5)
project(cnr_scene_manager)
find_package(ament_cmake REQUIRED)
find_package(cnr_tf_named_object_loader REQUIRED)
find_package(tf_conversions REQUIRED)
find_package(eigen_conversions REQUIRED)
find_package(cnr_scene_manager_msgs REQUIRED)

### 
find_package(cnr_param REQUIRED)
find_package(cnr_logger REQUIRED)

# #################################
# Preamble
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED On)

set(CMAKE_BUILD_TYPE Debug)

### 

###################################
## catkin specific configuration ##
###################################

###########
## Build ##
###########
include_directories(
  
)

add_executable(${PROJECT_NAME} src/cnr_scene_manager_node.cpp)

## Specify libraries to link a library or executable target against
ament_target_dependencies(${PROJECT_NAME}
  cnr_param::cnr_param
  cnr_logger::cnr_logger
  "eigen_conversions"
  "tf2_ros"
  "moveit_msgs"
  "cnr_scene_manager_msgs"
  "cnr_tf_named_object_loader"
  "rclcpp"
  "geometric_shapes"
)

add_executable(test_scene_manager src/test.cpp)

## Specify libraries to link a library or executable target against
ament_target_dependencies(test_scene_manager
  cnr_param::cnr_param
  cnr_logger::cnr_logger
  "eigen_conversions"
  "tf2_ros"
  "moveit_msgs"
  "cnr_scene_manager_msgs"
  "cnr_tf_named_object_loader"
  "rclcpp"
  "geometric_shapes"
)
ament_export_dependencies(cnr_tf_named_object_loader tf_conversions eigen_conversions cnr_scene_manager_msgs)
ament_package()

#############
## Install ##
#############

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS ${PROJECT_NAME}
   RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Mark libraries for installation
# See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE
#)

