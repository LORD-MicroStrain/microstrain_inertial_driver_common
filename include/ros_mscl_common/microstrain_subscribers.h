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
#ifndef _MICROSTRAIN_SUBSCRIBERS_H
#define _MICROSTRAIN_SUBSCRIBERS_H

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Include Files
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#include "microstrain_defs.h"
#include "microstrain_config.h"


/////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief Contains functions for micostrain driver
///
/////////////////////////////////////////////////////////////////////////////////////////////////////
namespace Microstrain
{

///
/// \brief Contains publishers for microstrain node
///
class MicrostrainSubscribers
{
 public:
  MicrostrainSubscribers() = default;
  MicrostrainSubscribers(RosNodeType* node, MicrostrainConfig* config);

  bool configure_subscribers();

  void velocity_zupt_callback(const BoolMsg& state);
  void vel_zupt_start();
  void vel_zupt();
  
  void ang_zupt_callback(const BoolMsg& state);
  void ang_zupt_start();
  void ang_zupt();

  void external_gps_time_callback(const TimeReferenceMsg& time);
  
  //ZUPT subscribers
  BoolSubType m_filter_vel_state_sub;
  BoolSubType m_filter_ang_state_sub;

  //External GNSS subscriber
  TimeReferenceSubType m_external_gps_time_sub;
 private:
  // Generic function to create a publisher
#if MICROSTRAIN_ROS_VERSION==1
  template<class M, class T>
  std::shared_ptr<::ros::Subscriber> create_subscriber(RosNodeType* node, const std::string& topic, const uint32_t queue_size, void(T::*fp)(const M&), T* obj)
  {
    return std::make_shared<::ros::Subscriber>(node->template subscribe(topic.c_str(), queue_size, fp, obj));
  }

  template<class T>
  RosTimerType create_timer(RosNodeType* node, double hz, void(T::*fp)(), T* obj)
  {
    return std::make_shared<::ros::Timer>(node->template createTimer(ros::Duration(1.0 / hz),
      [=](const ros::TimerEvent& event)
      {
        (obj->*fp)();
      }));
  }

  void stop_timer(RosTimerType timer)
  {
    timer->stop();
  }

  long get_time_ref_sec(const ros::Time& time_ref)
  {
    return time_ref.toSec();
  }
#elif MICROSTRAIN_ROS_VERSION==2
  template<class M, class T>
  typename ::rclcpp::Subscription<M>::SharedPtr create_subscriber(RosNodeType* node, const std::string& topic, const uint32_t qos, void(T::*fp)(const M&), T* obj)
  {
    return node->template create_subscription<M>(topic, qos,
      [obj, fp](const std::shared_ptr<M> req)
      {
        (obj->*fp)(*req);
      }
    );
  }

  template<class T>
  RosTimerType create_timer(RosNodeType* node, double hz, void(T::*fp)(), T* obj)
  {
    std::chrono::milliseconds timer_interval_ms(static_cast<int>(1.0 / hz * 1000.0));
    return node->template create_wall_timer(timer_interval_ms,
      [=]()
      {
        (obj->*fp)();
      });
  }

  void stop_timer(RosTimerType timer)
  {
    timer->cancel();
  }

  long get_time_ref_sec(const builtin_interfaces::msg::Time& time_ref)
  {
    return time_ref.sec;
  }
#else
#error "Unsupported ROS version. -DMICROSTRAIN_ROS_VERSION must be set to 1 or 2"
#endif

  //Node Information
  RosNodeType* m_node;
  MicrostrainConfig* m_config;

  bool m_vel_still;
  bool m_ang_still;

  RosTimerType m_vel_zupt_timer;
  RosTimerType m_ang_zupt_timer;
};  // struct MicrostrainPublishers

} // namespace Microstrain

#endif  // _MICROSTRAIN_3DM_GX5_45_H
