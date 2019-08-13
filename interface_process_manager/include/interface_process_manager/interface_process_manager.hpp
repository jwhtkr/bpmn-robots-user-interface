/**
 * @File: interface_process_manager.hpp
 * @Date: 13 August 2019
 * @Author: James Swedeen
 *
 * @brief
 * A bass class that defines that interface between Behavior Managers and the UI.
 **/

#ifndef INTERFACE_PROCESS_MANAGER_INTERFACE_PROCESS_MANAGER_HPP
#define INTERFACE_PROCESS_MANAGER_INTERFACE_PROCESS_MANAGER_HPP

/* Architecture Messages */
#include<architecture_msgs/BehaviorStatusResponse.h>

/* Interface Messages */
#include"inter

/* Behavior Manager Headers */
#include<behavior_manager/behavior_manager.hpp>

/* BPMN Interface Headers */
#include<task_lock/task_lock.hpp>
#include<error_handler/error_handler.hpp>
#include<message_handler/message_handler.hpp>

/* ROS Headers */
#include<ros/ros.h>

/* C++ Headers */
#include<string>

namespace behavior_ui
{
  template<typename TASK_LOCK = bpmn::TaskLock<>, typename ERROR = bpmn::ErrorHandler<>>
  class InterfaceProcessManager : public behavior_manager::BehaviorManager<TASK_LOCK, ERROR>
  {
  public:
    /**
     * @Default Constructor
     **/
    InterfaceProcessManager() = delete;
    /**
     * @Copy Constructor
     **/
    InterfaceProcessManager(const InterfaceProcessManager&) = delete;
    /**
     * @Move Constructor
     **/
    InterfaceProcessManager(InterfaceProcessManager&&) = delete;
    /**
     * @Constructor
     **/

    /**
     * @Deconstructor
     **/
    ~InterfaceProcessManager() override;
    /**
     * @Assignment Operators
     **/
    InterfaceProcessManager& operator=(const InterfaceProcessManager&) = delete;
    InterfaceProcessManager& operator=(InterfaceProcessManager&&)      = delete;
    /**
     * @get
     **/
    architecture_msgs::BehaviorStatus::Response::Ptr getStatus() const noexcept override;
    protected:
    /* Handles sending and receiving messages and sending signals */
    bpmn::MessageHandler message_handler;
    /* Communicates with user interface node */
    ros::ServiceServer service_list_srv;
    ros::ServiceServer complete_srv;
    ros::ServiceServer throw_error_srv;
    ros::ServiceServer get_message;
    ros::ServiceServer send_message_srv;
    ros::ServiceServer send_signal_srv;
    ros::ServiceServer get_variable_srv;
    ros::ServiceServer set_variable_srv;
    /**
     * @Service Callbacks
     **/

    /**
     * @runBehavior
     **/
    void runBehavior() override;
  };
}// behavior_ui

#endif
/* interface_process_manager.hpp */
