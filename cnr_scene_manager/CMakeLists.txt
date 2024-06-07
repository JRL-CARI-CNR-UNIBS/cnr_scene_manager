cmake_minimum_required(VERSION 3.0.2)
project(cnr_scene_manager)

# #################################
# Preamble
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED On)

### 
find_package(cnr_param REQUIRED)
find_package(cnr_logger REQUIRED)

### 
find_package(catkin REQUIRED COMPONENTS
  cnr_tf_named_object_loader
  tf_conversions
  eigen_conversions
  cnr_scene_manager_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES cnr_scene_manager
  CATKIN_DEPENDS cnr_tf_named_object_loader tf_conversions eigen_conversions cnr_scene_manager_msgs
  DEPENDS cnr_param cnr_logger
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/cnr_scene_manager_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  cnr_param::cnr_param
  cnr_logger::cnr_logger
)

#############
## Install ##
#############

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS ${PROJECT_NAME}_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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


