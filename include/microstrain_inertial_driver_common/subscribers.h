/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Driver Definition File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2020, Parker Hannifin Corp
//
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_SUBSCRIBERS_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_SUBSCRIBERS_H

#include "microstrain_inertial_driver_common/utils/ros_compat.h"
#include "microstrain_inertial_driver_common/utils/clock_bias_monitor.h"
#include "microstrain_inertial_driver_common/config.h"

#include "mip/definitions/commands_aiding.hpp"

namespace microstrain
{

constexpr auto EXT_TIME_GPS_TOPIC = "ext/time/gps";
constexpr auto EXT_TIME_TOPIC     = "ext/time";
constexpr auto EXT_FIX_TOPIC      = "ext/fix";
constexpr auto EXT_VEL_NED_TOPIC  = "ext/vel/ned";
constexpr auto EXT_VEL_ENU_TOPIC  = "ext/vel/enu";
constexpr auto EXT_VEL_ECEF_TOPIC = "ext/vel/ecef";
constexpr auto EXT_VEL_BODY_TOPIC = "ext/vel/body";
constexpr auto EXT_PRESSURE_TOPIC = "ext/pressure";
constexpr auto EXT_POSE_TOPIC     = "ext/pose";
constexpr auto EXT_HEADING_TOPIC  = "ext/heading";

/**
 * Contains subscribers and the functions they call
 */
class Subscribers
{
public:
  /**
   * \brief Default Constructor
   */
  Subscribers() = default;

  /**
   * \brief Constructs this class with a reference to the node, and a config object
   * \param node  Reference to a node that will be saved to this class and used to log and interact with ROS
   * \param config Reference to the config object that will be saved to this class and used to determine whether or not to enable the subscriptions
   */
  Subscribers(RosNodeType* node, Config* config);

  /**
   * \brief Activates the subscribers. After this function is called, the subscriptions will be ready to receive messages
   * \return true if activation was successful and false if activation failed
   */
  bool activate();

  /**
   * \brief Accepts external speed measurement to set speed on the device
   * \param speed  Message containing the external speed measurement
   */
  void externalSpeedCallback(const InputSpeedMeasurementMsg& speed);

  void externalTimeGpsCallback(const TimeReferenceMsg& time);
  void externalTimeCallback(const TimeReferenceMsg& time);

  void externalGnssPositionCallback(const NavSatFixMsg& fix);
  void externalGnssVelNedCallback(const TwistWithCovarianceStampedMsg& gnss_vel);
  void externalGnssVelEnuCallback(const TwistWithCovarianceStampedMsg& gnss_vel);
  void externalGnssVelEcefCallback(const TwistWithCovarianceStampedMsg& gnss_ecef_vel);
  void externalWheelSpeedCallback(const TwistWithCovarianceStampedMsg& wheel_speed);
  void externalBodyVelCallback(const TwistWithCovarianceStampedMsg& body_vel);
  void externalPressureCallback(const FluidPressureMsg& pressure);
  void externalPoseCallback(const PoseWithCovarianceStampedMsg& pose);
  void externalHeadingCallback(const DualAntennaHeadingMsg& heading);

  /**
   * \brief Accepts RTCM corrections from a ROS topic
   * \param rtcm Message containing 
   */
  void rtcmCallback(const RTCMMsg& rtcm);

  // External GNSS subscriber
  RosSubType<TimeReferenceMsg>::SharedPtr external_gps_time_sub_;

  // External speed subscriber
  RosSubType<InputSpeedMeasurementMsg>::SharedPtr external_speed_sub_;

  // External aiding measurement subscribers
  RosSubType<TimeReferenceMsg>::SharedPtr                   external_time_gps_sub_;
  RosSubType<TimeReferenceMsg>::SharedPtr                   external_time_sub_;
  RosSubType<NavSatFixMsg>::SharedPtr                       external_gnss_position_sub_;
  RosSubType<TwistWithCovarianceStampedMsg>::SharedPtr      external_gnss_vel_ned_sub_;
  RosSubType<TwistWithCovarianceStampedMsg>::SharedPtr      external_gnss_vel_enu_sub_;
  RosSubType<TwistWithCovarianceStampedMsg>::SharedPtr      external_gnss_vel_ecef_sub_;
  RosSubType<TwistWithCovarianceStampedMsg>::SharedPtr      external_body_vel_sub_;
  RosSubType<FluidPressureMsg>::SharedPtr                   external_pressure_sub_;
  RosSubType<PoseWithCovarianceStampedMsg>::SharedPtr       external_pose_sub_;
  RosSubType<DualAntennaHeadingMsg>::SharedPtr              external_heading_sub_;

  // RTCM subscriber
  RosSubType<RTCMMsg>::SharedPtr rtcm_sub_;

private:
  bool populateAidingTime(const RosHeaderType& header, const std::string& topic, mip::commands_aiding::Time* time);

  // Node Information
  RosNodeType* node_;
  Config* config_;

  // Clock bias monitor from ROS time to GPS time
  std::string gps_time_frame_id_ = "";
  ClockBiasMonitor ros_time_to_gps_time_clock_bias_monitor_ = ClockBiasMonitor(0.9, 1);

  // Mapping between different sensor times and ROS time
  std::map<std::string, ClockBiasMonitor> device_time_to_ros_time_clock_bias_monitors_;
};

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_SUBSCRIBERS_H
