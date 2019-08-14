cmake_minimum_required(VERSION 2.8.3)
project(interface_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    message_runtime
    message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  Service.msg
)

add_service_files(
  FILES
  Complete.srv
  GetVariable.srv
  SendMessage.srv
  SendSignal.srv
  ServiceList.srv
  SetVariable.srv
  ThrowError.srv
)

generate_messages()

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    message_generation
)

