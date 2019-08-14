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
#include"interface_msgs/Complete.h"
#include"interface_msgs/GetVariable.h"
#include"interface_msgs/SendMessage.h"
#include"interface_msgs/SendSignal.h"
#include"interface_msgs/ServiceList.h"
#include"interface_msgs/SetVariable.h"
#include"interface_msgs/ThrowError.h"

#include"interface_msgs/Service.h"

/* Behavior Manager Headers */
#include<behavior_manager/behavior_manager.hpp>

/* Camunda C++ API Headers */
#include<camunda_objects/topics.hpp>
#include<camunda_objects/throw_signal.hpp>
#include<camunda_objects/variables.hpp>

/* BPMN Interface Headers */
#include<task_lock/task_lock.hpp>
#include<error_handler/error_handler.hpp>
#include<message_handler/message_handler.hpp>

/* Rest Headers */
#include<cpprest/json.h>

/* ROS Headers */
#include<ros/ros.h>

/* C++ Headers */
#include<string>
#include<stdexcept>
#include<utility>
#include<system_error>

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
    InterfaceProcessManager(const std::string&              base_uri,
                            const std::string&              name,
                            const uint8_t                   priority,
                            const std::string&              camunda_topic,
                            const std::string&              status_topic,
                            const std::string&              get_resources_topic,
                            const std::string&              give_resources_topic,
                            const std::string&              give_up_resources_topic,
                            const std::string&              modify_robots_topic,
                            const std::string&              config_file_path,
                            const std::vector<std::string>& variables     = std::vector<std::string>(),
                            const uint32_t                  managing_rate = 30);
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
    ros::ServiceServer send_message_srv;
    ros::ServiceServer send_signal_srv;
    ros::ServiceServer get_variable_srv;
    ros::ServiceServer set_variable_srv;
    /**
     * @Service Callbacks
     **/
    virtual bool service_list_callback(interface_msgs::ServiceList::Request&,
                                       interface_msgs::ServiceList::Response& res);

    virtual bool complete_callback(interface_msgs::Complete::Request&  req,
                                   interface_msgs::Complete::Response& res);

    virtual bool throw_error_callback(interface_msgs::ThrowError::Request&  req,
                                      interface_msgs::ThrowError::Response& res);

    virtual bool send_message_callback(interface_msgs::SendMessage::Request&  req,
                                       interface_msgs::SendMessage::Response& res);

    virtual bool send_signal_callback(interface_msgs::SendSignal::Request&  req,
                                      interface_msgs::SendSignal::Response& res);

    virtual bool get_variable_callback(interface_msgs::GetVariable::Request&  req,
                                       interface_msgs::GetVariable::Response& res);

    virtual bool set_variable_callback(interface_msgs::SetVariable::Request&  req,
                                       interface_msgs::SetVariable::Response& res);
    /**
     * @runBehavior
     **/
    void runBehavior() override;
  };

  template<typename TASK_LOCK, typename ERROR>
  InterfaceProcessManager<TASK_LOCK, ERROR>::InterfaceProcessManager(const std::string&              base_uri,
                                                                     const std::string&              name,
                                                                     const uint8_t                   priority,
                                                                     const std::string&              camunda_topic,
                                                                     const std::string&              status_topic,
                                                                     const std::string&              get_resources_topic,
                                                                     const std::string&              give_resources_topic,
                                                                     const std::string&              give_up_resources_topic,
                                                                     const std::string&              modify_robots_topic,
                                                                     const std::string&              config_file_path,
                                                                     const std::vector<std::string>& variables,
                                                                     const uint32_t                  managing_rate)
   : behavior_manager::BehaviorManager<TASK_LOCK, ERROR>(base_uri,
                                                         name,
                                                         priority,
                                                         camunda_topic,
                                                         status_topic,
                                                         get_resources_topic,
                                                         give_resources_topic,
                                                         give_up_resources_topic,
                                                         modify_robots_topic,
                                                         config_file_path,
                                                         variables,
                                                         managing_rate),
   message_handler(web::json::value(base_uri), behavior_manager::BehaviorManager<TASK_LOCK, ERROR>::task_lock.getWorkerId())
  {}

  template<typename TASK_LOCK, typename ERROR>
  InterfaceProcessManager<TASK_LOCK, ERROR>::~InterfaceProcessManager()
  {

  }

  template<typename TASK_LOCK, typename ERROR>
  bool InterfaceProcessManager<TASK_LOCK, ERROR>::service_list_callback(interface_msgs::ServiceList::Request&,
                                                                        interface_msgs::ServiceList::Response& res)
  {
    res.services.reserve(6);

    if(ros::ServiceServer() != this->complete_srv)
    {
      res.services.emplace_back();

      res.services.back().name = this->complete_srv.getService();
      res.services.back().type = interface_msgs::Service::COMPLETE;
    }
    if(ros::ServiceServer() != this->throw_error_srv)
    {
      res.services.emplace_back();

      res.services.back().name = this->throw_error_srv.getService();
      res.services.back().type = interface_msgs::Service::THROW_ERROR;
    }
    if(ros::ServiceServer() != this->send_message_srv)
    {
      res.services.emplace_back();

      res.services.back().name = this->send_message_srv.getService();
      res.services.back().type = interface_msgs::Service::SEND_MESSAGE;
    }
    if(ros::ServiceServer() != this->send_signal_srv)
    {
      res.services.emplace_back();

      res.services.back().name = this->send_signal_srv.getService();
      res.services.back().type = interface_msgs::Service::SEND_SIGNAL;
    }
    if(ros::ServiceServer() != this->get_variable_srv)
    {
      res.services.emplace_back();

      res.services.back().name          = this->get_variable_srv.getService();
      res.services.back().type          = interface_msgs::Service::GET_VARIABLE;
      res.services.back().json_template = this->task_lock.getResponsVars().serialize();
    }
    if(ros::ServiceServer() != this->set_variable_srv)
    {
      res.services.emplace_back();

      res.services.back().name          = this->set_variable_srv.getService();
      res.services.back().type          = interface_msgs::Service::SET_VARIABLE;
      res.services.back().json_template = this->task_lock.getResponsVars().serialize();
    }

    return true;
  }

  template<typename TASK_LOCK, typename ERROR>
  bool InterfaceProcessManager<TASK_LOCK, ERROR>::complete_callback(interface_msgs::Complete::Request&  req,
                                                                    interface_msgs::Complete::Response& res)
  {
    if(std::string() != req.variables)
    {
      std::error_code error_code(0, std::generic_category());

      this->task_lock.addVariables(web::json::value::parse(req.variables, error_code));
      if(0 != error_code.value())
      {
        ROS_ERROR("Complete Json for " + this->getName() + " is incorrectly formatted.");
      }
    }

    try
    {
      this->task_lock.complete();
    }
    catch(const std::exception& ex)
    {
      ROS_ERROR("Competition of " + this->getName() + " failed with this exception " + ex.what());
      res.success = false;
      return true;
    }

    res.success = true;
    return true;
  }

  template<typename TASK_LOCK, typename ERROR>
  bool InterfaceProcessManager<TASK_LOCK, ERROR>::throw_error_callback(interface_msgs::ThrowError::Request&  req,
                                                                       interface_msgs::ThrowError::Response& res)
  {
    try
    {
      std::error_code error_code(0, std::generic_category());

      this->error_handler.ThrowError(req.message,
                                     std::string(),
                                     web::json::value::parse(req.variables, error_code));

      if(0 != error_code.value())
      {
        ROS_ERROR("Error Json for " + this->getName() + " is incorrectly formatted.");
      }
    }
    catch(const std::exception& ex)
    {
      ROS_ERROR("Behavior " + this->getName() + " failed to throw an error with this exception " + ex.what());
      res.success = false;
      return true;
    }

    res.success = true;
    return true;
  }

  template<typename TASK_LOCK, typename ERROR>
  bool InterfaceProcessManager<TASK_LOCK, ERROR>::send_message_callback(interface_msgs::SendMessage::Request&  req,
                                                                        interface_msgs::SendMessage::Response& res)
  {
    try
    {
      std::error_code error_code(0, std::generic_category());

      this->message_handler.template SendMessage<camunda::Variables, camunda::Topics>(camunda::Variables(web::json::value::parse(req.variables, error_code)),
                                                                                      camunda::Topics(req.message_name, 9999));

      if(0 != error_code.value())
      {
        ROS_ERROR("Message Json for " + this->getName() + " is incorrectly formatted.");
      }
    }
    catch(const std::exception& ex)
    {
      ROS_ERROR("Behavior " + this->getName() + " failed to send a message with this exception " + ex.what());
      res.success = false;
      return true;
    }

    res.success = true;
    return true;
  }

  template<typename TASK_LOCK, typename ERROR>
  bool InterfaceProcessManager<TASK_LOCK, ERROR>::send_signal_callback(interface_msgs::SendSignal::Request&  req,
                                                                       interface_msgs::SendSignal::Response& res)
  {
    try
    {
      std::error_code error_code(0, std::generic_category());

      this->message_handler.template throwSignal<camunda::ThrowSignal>(camunda::ThrowSignal(req.name, camunda::Variables(web::json::value::parse(req.variables, error_code))));

      if(0 != error_code.value())
      {
        ROS_ERROR("Signal Json for " + this->getName() + " is incorrectly formatted.");
      }
    }
    catch(const std::exception& ex)
    {
      ROS_ERROR("Behavior " + this->getName() + " failed to send a signal with this exception " + ex.what());
      res.success = false;
      return true;
    }

    res.success = true;
    return true;
  }



}// behavior_ui

#endif
/* interface_process_manager.hpp */
