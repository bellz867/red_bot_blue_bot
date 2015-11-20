cmake_minimum_required(VERSION 2.8.3)
project(trying_again)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  joy
  rospy
  std_msgs
  cv_bridge
  image_geometry
  image_transport
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
#  /home/zack/opencv-2.4.11/cmake/

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(trying_again_node trying_again_generate_messages_cpp)

## Specify libraries to link a library or executable target against
#target_link_libraries(trying_again_node
#  ${catkin_LIBRARIES}
#)
#target_link_libraries(trying_again_node
#${OpenCV_LIBRARIES}
#)
