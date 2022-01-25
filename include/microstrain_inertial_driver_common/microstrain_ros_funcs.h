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
#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_MICROSTRAIN_ROS_FUNCS_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_MICROSTRAIN_ROS_FUNCS_H

#include <string>
#include <memory>
#include "microstrain_inertial_driver_common/microstrain_defs.h"

namespace microstrain
{

/**
 * ROS1 Functions
 */
#if MICROSTRAIN_ROS_VERSION == 1

/**
 * \brief Gets the current ROS time
 * \param node  Unused in this function as the ros time function is static
 * \return Current ROS time
 */
inline RosTimeType ros_time_now(RosNodeType* node)
{
  return ros::Time::now();
}

/**
 * \brief Converts nanoseconds to ROS time
 * \param time  The current time in nanoseconds
 * \return ROS time at the specified nanoseconds
 */
inline RosTimeType to_ros_time(int64_t time)
{
  return ros::Time().fromNSec(time);
}

/**
 * \brief Gets the seconds from a ROS time object because the interface changed between ROS1 and ROS2
 * \param time_ref  The ros Time object to extract the seconds from
 * \return seconds from the ros time object
 */
inline int64_t get_time_ref_sec(const ros::Time& time_ref)
{
  return time_ref.toSec();
}

/**
 * \brief Sets the sequence number on a ROS header. This is only useful in ROS1 as ROS2 removed the seq member
 * \param header  The header to set the sequence number on
 * \param seq  The sequence number to set on the header
 */
inline void set_seq(RosHeaderType* header, const uint32_t seq)
{
  header->seq = seq;
}

/**
 * \brief Extracts config from the ROS node.
 * \tparam ConfigType  The type to extract the config to e.g. int, std::string, double, etc.
 * \param node  The ROS node to extract the config from
 * \param param_name  The name of the config value to extract
 * \param param_val  Variable to store the extracted config value in
 * \param default_val  The default value to set param_val to if the config can't be found
 */
template <class ConfigType>
void get_param(RosNodeType* node, const std::string& param_name, ConfigType& param_val, const ConfigType& default_val)
{
  node->param<ConfigType>(param_name, param_val, default_val);
}

/**
 * \brief Creates a transform broadcaster
 * \param node The ROS node that the broadcaster will be associated with
 * \return Initialized shared pointer containing a transdorm broadcaster
 */
inline TransformBroadcasterType create_transform_broadcaster(RosNodeType* node)
{
  return std::make_shared<tf2_ros::TransformBroadcaster>();
}

/**
 * \brief Creates a ROS publisher
 * \tparam MessageType  The type of message that this publisher will publish
 * \param node  The ROS node to create the publisher on
 * \param topic  The topic that this publisher will publish on
 * \param queue_size  The size of the queue to enable in ROS
 * \return Shared Pointer containing the initialized publisher
 */
template <class MessageType>
std::shared_ptr<::ros::Publisher> create_publisher(RosNodeType* node, const std::string& topic,
                                                   const uint32_t queue_size)
{
  return std::make_shared<::ros::Publisher>(node->template advertise<MessageType>(topic, queue_size));
}

/**
 * \brief Creates a ROS subscriber
 * \tparam MessageType  The type of message that this subscriber will listen to
 * \tparam ClassType  The type of class that the member function passed to this function will be on
 * \param node  The ROS node to create the publisher on
 * \param topic  The topic that this publisher will subscribe on
 * \param queue_size  The size of the queue to enable on ROS
 * \param fp  Function pointer to call whenever a message is received on the topic
 * \param obj  Reference to an object of type ClassType that will be passed as the this pointer to fp
 * \return Shared Pointer containing the initialized subscriber
 */
template <class MessageType, class ClassType>
std::shared_ptr<::ros::Subscriber> create_subscriber(RosNodeType* node, const std::string& topic,
                                                     const uint32_t queue_size,
                                                     void (ClassType::*fp)(const MessageType&), ClassType* obj)
{
  return std::make_shared<::ros::Subscriber>(node->template subscribe(topic.c_str(), queue_size, fp, obj));
}

/**
 * \brief Creates a ROS Service
 * \tparam MessageType  The type of message that this service will use
 * \tparam ClassType The type of class that the callback will be registered to
 * \tparam RequestType  The type of request that this function will receive
 * \tparam ResponseType  The type of response that this function will respond with
 * \param node  The ROS node to create the service on
 * \param service  The name to give the created service
 * \param srv_func  Function pointer to a function that will be called when the service receives a message
 * \param obj  Reference to an object of type ClassType that will be used to call the callback
 * \return Shared Pointer containing the initialized service
 */
template <class MessageType, class ClassType, class RequestType, class ResponseType>
std::shared_ptr<::ros::ServiceServer> create_service(RosNodeType* node, const std::string& service,
                                                     bool (ClassType::*srv_func)(RequestType&, ResponseType&),
                                                     ClassType* obj)
{
  return std::make_shared<::ros::ServiceServer>(
      node->template advertiseService<ClassType, RequestType, ResponseType>(service, srv_func, obj));
}

/**
 * \brief Creates and starts a ROS timer
 * \tparam ClassType  The type of class that the callback will be registered to
 * \param node  The ROS node to create the timer on
 * \param hz  Rate in hertz to execute the callback on
 * \param fp  Function pointer to execute at the specified rate
 * \param obj  Reference to an object of type Class Type that will be used to call the callback
 * \return Shard pointer containing the initialized and started timer
 */
template <class ClassType>
RosTimerType create_timer(RosNodeType* node, double hz, void (ClassType::*fp)(), ClassType* obj)
{
  return std::make_shared<::ros::Timer>(
      node->template createTimer(ros::Duration(1.0 / hz), [=](const ros::TimerEvent& event) { (obj->*fp)(); }));
}

/**
 * \brief Stops a ROS timer
 * \param timer  The timer to stop
 */
inline void stop_timer(RosTimerType timer)
{
  timer->stop();
}

/**
 * ROS2 Functions
 */
#elif MICROSTRAIN_ROS_VERSION == 2

/**
 * \brief Gets the current ROS time
 * \param node  Unused in this function as the ros time function is static
 * \return Current ROS time
 */
inline RosTimeType ros_time_now(RosNodeType* node)
{
  return node->get_clock()->now();
}

/**
 * \brief Converts nanoseconds to ROS time
 * \param time  The current time in nanoseconds
 * \return ROS time at the specified nanoseconds
 */
inline RosTimeType to_ros_time(int64_t time)
{
  return ::rclcpp::Time(time);
}

/**
 * \brief Gets the seconds from a ROS time object because the interface changed between ROS1 and ROS2
 * \param time_ref  The ros Time object to extract the seconds from
 * \return seconds from the ros time object
 */
inline int64_t get_time_ref_sec(const builtin_interfaces::msg::Time& time_ref)
{
  return time_ref.sec;
}

/**
 * \brief Sets the sequence number on a ROS header. This is only useful in ROS1 as ROS2 removed the seq member
 * \param header  The header to set the sequence number on
 * \param seq  The sequence number to set on the header
 */
inline void set_seq(RosHeaderType* header, const uint32_t seq)
{
  // NOOP because seq was removed in ROS2
}

/**
 * \brief Extracts config from the ROS node.
 * \tparam ConfigType  The type to extract the config to e.g. int, std::string, double, etc.
 * \param node  The ROS node to extract the config from
 * \param param_name  The name of the config value to extract
 * \param param_val  Variable to store the extracted config value in
 * \param default_val  The default value to set param_val to if the config can't be found
 */
template <class ConfigType>
void get_param(RosNodeType* node, const std::string& param_name, ConfigType& param_val, const ConfigType& default_val)
{
  if (node->has_parameter(param_name))
  {
    node->get_parameter_or<ConfigType>(param_name, param_val, default_val);
  }
  else
  {
    param_val = node->declare_parameter<ConfigType>(param_name, default_val);
  }
}

/**
 * \brief Creates a transform broadcaster
 * \param node The ROS node that the broadcaster will be associated with
 * \return Initialized shared pointer containing a transdorm broadcaster
 */
inline TransformBroadcasterType create_transform_broadcaster(RosNodeType* node)
{
  return std::make_shared<tf2_ros::TransformBroadcaster>(node);
}

/**
 * \brief Creates a ROS publisher
 * \tparam MessageType  The type of message that this publisher will publish
 * \param node  The ROS node to create the publisher on
 * \param topic  The topic that this publisher will publish on
 * \param queue_size  The size of the queue to enable in ROS
 * \return Shared Pointer containing the initialized publisher
 */
template <class MessageType>
typename ::rclcpp_lifecycle::LifecyclePublisher<MessageType>::SharedPtr create_publisher(RosNodeType* node,
                                                                                         const std::string& topic,
                                                                                         const uint32_t qos)
{
  return node->template create_publisher<MessageType>(topic, qos);
}

/**
 * \brief Creates a ROS subscriber
 * \tparam MessageType  The type of message that this subscriber will listen to
 * \tparam ClassType  The type of class that the member function passed to this function will be on
 * \param node  The ROS node to create the publisher on
 * \param topic  The topic that this publisher will subscribe on
 * \param queue_size  The size of the queue to enable on ROS
 * \param fp  Function pointer to call whenever a message is received on the topic
 * \param obj  Reference to an object of type ClassType that will be passed as the this pointer to fp
 * \return Shared Pointer containing the initialized subscriber
 */
template <class MessageType, class ClassType>
typename ::rclcpp::Subscription<MessageType>::SharedPtr create_subscriber(RosNodeType* node, const std::string& topic,
                                                                          const uint32_t qos,
                                                                          void (ClassType::*fp)(const MessageType&),
                                                                          ClassType* obj)
{
  return node->template create_subscription<MessageType>(
      topic, qos, [obj, fp](const std::shared_ptr<MessageType> req) { (obj->*fp)(*req); });
}

/**
 * \brief Creates a ROS Service
 * \tparam MessageType  The type of message that this service will use
 * \tparam ClassType The type of class that the callback will be registered to
 * \tparam RequestType  The type of request that this function will receive
 * \tparam ResponseType  The type of response that this function will respond with
 * \param node  The ROS node to create the service on
 * \param service  The name to give the created service
 * \param srv_func  Function pointer to a function that will be called when the service receives a message
 * \param obj  Reference to an object of type ClassType that will be used to call the callback
 * \return Shared Pointer containing the initialized service
 */
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

/**
 * \brief Creates and starts a ROS timer
 * \tparam ClassType  The type of class that the callback will be registered to
 * \param node  The ROS node to create the timer on
 * \param hz  Rate in hertz to execute the callback on
 * \param fp  Function pointer to execute at the specified rate
 * \param obj  Reference to an object of type Class Type that will be used to call the callback
 * \return Shard pointer containing the initialized and started timer
 */
template <class ClassType>
RosTimerType create_timer(RosNodeType* node, double hz, void (ClassType::*fp)(), ClassType* obj)
{
  std::chrono::milliseconds timer_interval_ms(static_cast<int>(1.0 / hz * 1000.0));
  return node->template create_wall_timer(timer_interval_ms, [=]() { (obj->*fp)(); });
}

/**
 * \brief Stops a ROS timer
 * \param timer  The timer to stop
 */
inline void stop_timer(RosTimerType timer)
{
  timer->cancel();
}

#else
#error "Unsupported ROS version. -DMICROSTRAIN_ROS_VERSION must be set to 1 or 2"
#endif

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_MICROSTRAIN_ROS_FUNCS_H
