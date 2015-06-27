cmake_minimum_required(VERSION 2.8.3)
project(vesc_driver)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  diagnostic_updater
  hardware_interface
  roscpp
  std_srvs
)

find_package(Boost REQUIRED COMPONENTS thread)

################################################
## Declare ROS messages, services and actions ##
################################################

generate_dynamic_reconfigure_options(
  cfg/VESC.py
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vesc_ros
  CATKIN_DEPENDS diagnostic_updater dynamic_reconfigure hardware_interface roscpp std_srvs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(vesc src/vesc.c)
add_library(vesc_ros src/vesc_driver.cpp src/vesc_handle.cpp)

add_executable(vesc_node src/vesc_node.cpp)

add_dependencies(vesc_ros ${PROJECT_NAME}_gencfg)

target_link_libraries(vesc_node vesc_ros ${catkin_LIBRARIES})
target_link_libraries(vesc_ros vesc ${catkin_LIBRARIES} ${Boost_LIBRARIES})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

#############
## Install ##
#############

install(TARGETS vesc vesc_node vesc_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME} include/vesc
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
