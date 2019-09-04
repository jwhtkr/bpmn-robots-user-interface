cmake_minimum_required(VERSION 2.8.3)
project(interface_process_manager)

add_compile_options(-std=c++11 -g -O0)

find_package(catkin REQUIRED
  COMPONENTS
  # ROS Packages
    roscpp
  # Architecture Packages
    behavior_manager
  # BPMN Interface Packages
    bpmn_interface
  # Camunda API Packages
    camunda_c_api
  # Message Packages
    architecture_msgs
    interface_msgs
)

find_package(cpprestsdk REQUIRED COMPONENTS cpprest)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
  # ROS Packages
    roscpp
  # Architecture Packages
    behavior_manager
  # BPMN Interface Packages
    bpmn_interface
  # Camunda API Packages
    camunda_c_api
  # Message Packages
    architecture_msgs
    interface_msgs
  DEPENDS
    cpprestsdk::cpprest
)

###########
## Build ##
###########

include_directories(
  include
  ${cpprestsdk_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

