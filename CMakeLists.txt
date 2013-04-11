cmake_minimum_required(VERSION 2.8.3)
project(smd_ardrone2)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS camera_info_manager geometry_msgs image_transport nodelet pluginlib roscpp sensor_msgs std_srvs tf message_generation)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
pkg_check_modules(AVFORMAT REQUIRED libavformat)
pkg_check_modules(AVCODEC REQUIRED libavcodec)
pkg_check_modules(SWSCALE REQUIRED libswscale)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and scripts declared therein get installed
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  DroneAnimate.srv
  DroneConfig.srv
  DroneLEDAnimate.srv
  DroneSetCam.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ARDrone2_Control ARDrone2_H264
   CATKIN_DEPENDS camera_info_manager geometry_msgs image_transport nodelet pluginlib roscpp sensor_msgs std_srvs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${AVFORMAT_INCLUDE_DIRS}
  ${AVCODEC_INCLUDE_DIRS}
  ${SWSCALE_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(ARDrone2_Control
  src/ARDrone2_Control.cpp
)
add_library(ARDrone2_H264
  src/ARDrone2_H264.cpp
)
add_library(ARDrone2_Helper
  src/ARDrone2_Helper.cpp
)

## Declare a cpp executable
# add_executable(smd_ardrone2_node src/smd_ardrone2_node.cpp)

## Add dependencies to the executable
add_dependencies(ARDrone2_Control smd_ardrone2_gensrv)
add_dependencies(ARDrone2_Helper smd_ardrone2_gensrv)

## Specify libraries to link a library or executable target against
target_link_libraries(ARDrone2_Control
  ${catkin_LIBRARIES}
)
target_link_libraries(ARDrone2_H264
  ${catkin_LIBRARIES}
  ${AVCODEC_LIBRARIES}
  ${AVFORMAT_LIBRARIES}
  ${SWSCALE_LIBRARIES}
)
target_link_libraries(ARDrone2_Helper
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## not required for python when using catkin_python_setup()
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ARDrone2_Control ARDrone2_H264 ARDrone2_Helper
# ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  nodelet_plugins.xml
  launch/ardrone2.launch
  launch/ardrone2_with_helper.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_smd_ardrone2.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
