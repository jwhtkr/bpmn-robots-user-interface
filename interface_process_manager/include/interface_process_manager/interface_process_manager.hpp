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
#include<process_variables/process_variable.hpp>

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
     *
     * @brief
     * This constructor calls the bass class BehaviorManager's constructor which fetches and
     * locks a task, loads the config file of resources, asks the Resource Manager for those
     * resources, as well as setup all base ROS connections and configure the ErrorHandler it
     * holds. The only other thing that this constructor does is initialize it's local
     * MessageHandler object.
     * @base_uri: The URI of the camunda server
     * @name: The string that will used to denote this behavior
     * @camunda_topic: The camunda topic that this object will attempt to fetch and lock from
     * @status_topic: ROS topic of a service that provides status updates from this object
     * @get_resources_topic: ROS service topic that this object will use to ask the Resource
     *                       Manager for resources
     * @give_resources_topic: ROS service topic that this object will use to give the Resource
     *                        Manager resources it's not using anymore
     * @give_up_resources_topic: ROS service topic that the Resource Manager will use to take
     *                           resources back from this object
     * @modify_robots_topic: ROS service topic that this object will use to tell the Thread
     *                       Pool Manager what resources to spin-up
     * @config_file_path: The absolute path of this behavior's config file
     * @variables: The variables you want to get in the fetch and lock. If left blank
     *             Camunda will return all of the global variables
     * @managing_rate: The frequency that this object will attempt to run all of its
     *                 duties at least once
     **/
    InterfaceProcessManager(const std::string&              base_uri,
                            const std::string&              name,
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
    ~InterfaceProcessManager() override = default;
    /**
     * @Assignment Operators
     **/
    InterfaceProcessManager& operator=(const InterfaceProcessManager&) = delete;
    InterfaceProcessManager& operator=(InterfaceProcessManager&&)      = delete;
    /**
     * @getStatus
     *
     * @brief
     * Returns some basic information about this Behavior Manager. It is recommended that
     * base classes use this function to help fill out the name and managerStatus fields.
     **/
    architecture_msgs::BehaviorStatus::Response::Ptr getStatus() const noexcept override = 0;
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
     *
     * @brief
     * Each callback defines the most basic version of what the services are.
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
     *
     * @brief
     * This is where any behavior specific logic should go.
     **/
    void runBehavior() override = 0;
  };

  template<typename TASK_LOCK, typename ERROR>
  InterfaceProcessManager<TASK_LOCK, ERROR>::InterfaceProcessManager(const std::string&              base_uri,
                                                                     const std::string&              name,
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
  architecture_msgs::BehaviorStatus::Response::Ptr InterfaceProcessManager<TASK_LOCK, ERROR>::getStatus() const noexcept
  {
    return this->behavior_manager::BehaviorManager<TASK_LOCK, ERROR>::getStatus();
  }

  template<typename TASK_LOCK, typename ERROR>
  bool InterfaceProcessManager<TASK_LOCK, ERROR>::service_list_callback(interface_msgs::ServiceList::Request&,
                                                                        interface_msgs::ServiceList::Response& res)
  {
    res.behavior_name = this->getName();
    res.services.reserve(6);

    if(ros::ServiceServer() != this->complete_srv)
    {
      res.services.emplace_back();

      res.services.back().service_name = this->complete_srv.getService();
      res.services.back().name         = "Complete";
      res.services.back().type         = interface_msgs::Service::COMPLETE;
    }
    if(ros::ServiceServer() != this->throw_error_srv)
    {
      res.services.emplace_back();

      res.services.back().service_name = this->throw_error_srv.getService();
      res.services.back().name         = "Throw Error";
      res.services.back().type         = interface_msgs::Service::THROW_ERROR;
    }
    if(ros::ServiceServer() != this->send_message_srv)
    {
      res.services.emplace_back();

      res.services.back().service_name = this->send_message_srv.getService();
      res.services.back().name         = "Send Message";
      res.services.back().type         = interface_msgs::Service::SEND_MESSAGE;
    }
    if(ros::ServiceServer() != this->send_signal_srv)
    {
      res.services.emplace_back();

      res.services.back().service_name = this->send_signal_srv.getService();
      res.services.back().name         = "Send Signal";
      res.services.back().type         = interface_msgs::Service::SEND_SIGNAL;
    }
    if(ros::ServiceServer() != this->get_variable_srv)
    {
      res.services.emplace_back();

      res.services.back().service_name  = this->get_variable_srv.getService();
      res.services.back().name          = "Get Variable";
      res.services.back().type          = interface_msgs::Service::GET_VARIABLE;
      res.services.back().json_template = this->task_lock.getResponsVars().serialize();
    }
    if(ros::ServiceServer() != this->set_variable_srv)
    {
      res.services.emplace_back();

      res.services.back().service_name  = this->set_variable_srv.getService();
      res.services.back().name          = "Set Variable";
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

      this->task_lock.addVariables(camunda::Variables(web::json::value::parse(req.variables, error_code)));
      if(0 != error_code.value())
      {
        ROS_ERROR("Complete Json for %s is incorrectly formatted.", this->getName().c_str());
      }
    }

    try
    {
      this->task_lock.complete();
    }
    catch(const std::exception& ex)
    {
      ROS_ERROR("Competition of %s failed with this exception  %s", this->getName().c_str(), ex.what());
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
      if(std::string() != req.variables)
      {
        std::error_code error_code(0, std::generic_category());

        this->error_handler.throwBpmnError(req.message,
                                           std::string(),
                                           camunda::Variables(web::json::value::parse(req.variables, error_code)));

        if(0 != error_code.value())
        {
          ROS_ERROR("Error Json for %s is incorrectly formatted.", this->getName().c_str());
        }
      }
      else
      {
        this->error_handler.throwBpmnError(req.message, std::string());
      }
    }
    catch(const std::exception& ex)
    {
      ROS_ERROR("Behavior %s failed to throw an error with this exception %s", this->getName().c_str(), ex.what());
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
      if(std::string != req.variables)
      {
        std::error_code error_code(0, std::generic_category());

        this->message_handler.template sendMessage<camunda::Variables>(req.message_name,
                                                                       camunda::Variables(web::json::value::parse(req.variables, error_code)),
                                                                       this->getInstanceId());
        if(0 != error_code.value())
        {
          ROS_ERROR("Message Json for %s is incorrectly formatted.", this->getName().c_str());
        }
      }
      else
      {
         this->message_handler.template sendMessage<camunda::Variables>(req.message_name,
                                                                       camunda::Variables(),
                                                                       this->getInstanceId());
      }
    }
    catch(const std::exception& ex)
    {
      ROS_ERROR("Behavior %s failed to send a message with this exception %s", this->getName().c_str(), ex.what());
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
      if(std::string() != req.variables)
      {
        std::error_code error_code(0, std::generic_category());

        this->message_handler.template throwSignal<camunda::ThrowSignal>(camunda::ThrowSignal(req.name, camunda::Variables(web::json::value::parse(req.variables, error_code))));

        if(0 != error_code.value())
        {
          ROS_ERROR("Signal Json for %s is incorrectly formatted.", this->getName().c_str());
        }
      }
      else
      {
        this->message_handler.template throwSignal<camunda::ThrowSignal>(camunda::ThrowSignal(req.name, camunda::Variables()));
      }
    }
    catch(const std::exception& ex)
    {
      ROS_ERROR("Behavior %s failed to send a signal with this exception %s", this->getName().c_str(), ex.what());
      res.success = false;
      return true;
    }

    res.success = true;
    return true;
  }

  template<typename TASK_LOCK, typename ERROR>
  bool InterfaceProcessManager<TASK_LOCK, ERROR>::get_variable_callback(interface_msgs::GetVariable::Request&  req,
                                                                        interface_msgs::GetVariable::Response& res)
  {
    try
    {
      res.variable = this->task_lock.getResponsVars().at(req.name).serialize();
    }
    catch(const std::exception& ex)
    {
      ROS_ERROR("Behavior %s failed to get the %s variable because of this exception, %s", this->getName().c_str(), req.name.c_str(), ex.what());
    }
    return true;
  }

  template<typename TASK_LOCK, typename ERROR>
  bool InterfaceProcessManager<TASK_LOCK, ERROR>::set_variable_callback(interface_msgs::SetVariable::Request&  req,
                                                                        interface_msgs::SetVariable::Response& res)
  {
    try
    {
      if(interface_msgs::SetVariable::Request::STRING == req.type)
      {
        camunda::ProcessVariable<std::string> p_var(this->getBaseUri(), this->getInstanceId(), req.name);

        p_var.update(req.value);
      }
      else if(interface_msgs::SetVariable::Request::INTEGER == req.type)
      {
        camunda::ProcessVariable<int64_t> p_var(this->getBaseUri(), this->getInstanceId(), req.name);

        p_var.update(std::stol(req.value));
      }
      else if(interface_msgs::SetVariable::Request::DOUBLE == req.type)
      {
        camunda::ProcessVariable<double> p_var(this->getBaseUri(), this->getInstanceId(), req.name);

        p_var.update(std::stod(req.value));
      }
      else if(interface_msgs::SetVariable::Request::BOOLEAN == req.type)
      {
        camunda::ProcessVariable<bool> p_var(this->getBaseUri(), this->getInstanceId(), req.name);

        if("true" == req.value)
        {
          p_var.update(true);
        }
        else if("false" == req.value)
        {
          p_var.update(false);
        }
        else
        {
          throw std::runtime_error(req.value + " is not a boolean type.");
        }
      }
    }
    catch(const std::exception& ex)
    {
      res.success = false;
      ROS_ERROR("Behavior %s failed to set the %s variable because of this exception %s", this->getName().c_str(), req.name.c_str(), ex.what());
      return true;
    }
    res.success = true;
    return true;
  }

  template<typename TASK_LOCK, typename ERROR>
  void InterfaceProcessManager<TASK_LOCK, ERROR>::runBehavior()
  {}

}// behavior_ui

#endif
/* interface_process_manager.hpp */
