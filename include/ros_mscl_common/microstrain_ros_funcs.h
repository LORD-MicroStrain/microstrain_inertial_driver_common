/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord GX5-Series Driver Definition File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c)  2020, Parker Hannifin Corp
//
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef ROS_MSCL_COMMON_MICROSTRAIN_ROS_FUNCS_H
#define ROS_MSCL_COMMON_MICROSTRAIN_ROS_FUNCS_H

#include <string>
#include <memory>
#include "ros_mscl_common/microstrain_defs.h"

namespace microstrain
{
/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// ROS1 Functions
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#if MICROSTRAIN_ROS_VERSION == 1

inline ::ros::Time ros_time_now(RosNodeType* node)
{
  return ros::Time::now();
}

inline ::ros::Time to_ros_time(int64_t time)
{
  return ros::Time().fromNSec(time);
}

inline int64_t get_time_ref_sec(const ros::Time& time_ref)
{
  return time_ref.toSec();
}

inline void set_seq(RosHeaderType* header, const uint32_t seq)
{
  header->seq = seq;
}

template <class ConfigType>
void get_param(RosNodeType* node, const std::string& param_name, ConfigType& param_val, const ConfigType& default_val)
{
  node->param<ConfigType>(param_name, param_val, default_val);
}

template <class MessageType>
std::shared_ptr<::ros::Publisher> create_publisher(RosNodeType* node, const std::string& topic,
                                                   const uint32_t queue_size)
{
  return std::make_shared<::ros::Publisher>(node->template advertise<MessageType>(topic, queue_size));
}

template <class MessageType, class ClassType>
std::shared_ptr<::ros::Subscriber> create_subscriber(RosNodeType* node, const std::string& topic,
                                                     const uint32_t queue_size,
                                                     void (ClassType::*fp)(const MessageType&), ClassType* obj)
{
  return std::make_shared<::ros::Subscriber>(node->template subscribe(topic.c_str(), queue_size, fp, obj));
}

template <class MessageType, class ClassType, class RequestType, class ResponseType>
std::shared_ptr<::ros::ServiceServer> create_service(RosNodeType* node, const std::string& service,
                                                     bool (ClassType::*srv_func)(RequestType&, ResponseType&),
                                                     ClassType* obj)
{
  return std::make_shared<::ros::ServiceServer>(
      node->template advertiseService<ClassType, RequestType, ResponseType>(service, srv_func, obj));
}

template <class ClassType>
RosTimerType create_timer(RosNodeType* node, double hz, void (ClassType::*fp)(), ClassType* obj)
{
  return std::make_shared<::ros::Timer>(
      node->template createTimer(ros::Duration(1.0 / hz), [=](const ros::TimerEvent& event) { (obj->*fp)(); }));
}

inline void stop_timer(RosTimerType timer)
{
  timer->stop();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// ROS2 Functions
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#elif MICROSTRAIN_ROS_VERSION == 2

inline ::rclcpp::Time ros_time_now(RosNodeType* node)
{
  return node->get_clock()->now();
}

inline ::rclcpp::Time to_ros_time(int64_t time)
{
  return ::rclcpp::Time(time);
}

inline int64_t get_time_ref_sec(const builtin_interfaces::msg::Time& time_ref)
{
  return time_ref.sec;
}

inline void set_seq(RosHeaderType* header, const uint32_t seq)
{
  // NOOP because seq was removed in ROS2
}

template <class ConfigType>
void get_param(RosNodeType* node, const std::string& param_name, ConfigType& param_val, const ConfigType& default_val)
{
  node->get_parameter_or<ConfigType>(param_name, param_val, default_val);
}

template <class MessageType>
typename ::rclcpp_lifecycle::LifecyclePublisher<MessageType>::SharedPtr create_publisher(RosNodeType* node,
                                                                                         const std::string& topic,
                                                                                         const uint32_t qos)
{
  return node->template create_publisher<MessageType>(topic, qos);
}

template <class MessageType, class ClassType>
typename ::rclcpp::Subscription<MessageType>::SharedPtr create_subscriber(RosNodeType* node, const std::string& topic,
                                                                          const uint32_t qos,
                                                                          void (ClassType::*fp)(const MessageType&),
                                                                          ClassType* obj)
{
  return node->template create_subscription<MessageType>(
      topic, qos, [obj, fp](const std::shared_ptr<MessageType> req) { (obj->*fp)(*req); });
}

template <class MessageType, class ClassType, class RequestType, class ResponseType>
typename ::rclcpp::Service<MessageType>::SharedPtr
create_service(RosNodeType* node, const std::string& service, bool (ClassType::*srv_func)(RequestType&, ResponseType&),
               ClassType* obj)
{
  return node->template create_service<MessageType>(
      service, [obj, srv_func](const std::shared_ptr<RequestType> req, std::shared_ptr<ResponseType> res)
      {
        (obj->*srv_func)(*req, *res);
      }
    );  // NOLINT(whitespace/parens)  No way to avoid this
}

template <class ClassType>
RosTimerType create_timer(RosNodeType* node, double hz, void (ClassType::*fp)(), ClassType* obj)
{
  std::chrono::milliseconds timer_interval_ms(static_cast<int>(1.0 / hz * 1000.0));
  return node->template create_wall_timer(timer_interval_ms, [=]() { (obj->*fp)(); });
}

inline void stop_timer(RosTimerType timer)
{
  timer->cancel();
}

#else
#error "Unsupported ROS version. -DMICROSTRAIN_ROS_VERSION must be set to 1 or 2"
#endif

}  // namespace microstrain

#endif  // ROS_MSCL_COMMON_MICROSTRAIN_ROS_FUNCS_H
