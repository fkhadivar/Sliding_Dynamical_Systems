cmake_minimum_required(VERSION 2.8.3)
project(sliding_ds)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  iiwa_tools
)
# find_package(Boost REQUIRED)
## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED eigen3) 
# if(NOT Eigen_FOUND)
#   include(cmake/FindEigen3.cmake)
# endif()

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  iiwa_tools
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_generation iiwa_tools
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( include ${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(sliding_ds src/iiwaSlidingDs_node.cpp src/iiwaSlidingDs.cpp ${MODEL_SOURCES})
target_link_libraries(sliding_ds ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
add_dependencies(sliding_ds ${PROJECT_NAME}_gencpp)
