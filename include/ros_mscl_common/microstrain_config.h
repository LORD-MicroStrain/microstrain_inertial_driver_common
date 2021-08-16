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


#ifndef _MICROSTRAIN_CONFIG_H
#define _MICROSTRAIN_CONFIG_H

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Include Files
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#include <stddef.h>
#include <string>
#include <vector>
#include <fstream>

#if MICROSTRAIN_ROS_VERSION==1
#include <tf2/LinearMath/Matrix3x3.h>
#endif

#include "mscl/mscl.h"

#include "microstrain_defs.h"
#include "microstrain_ros_funcs.h"

constexpr auto default_matrix = {9.0, 0.0};
constexpr auto default_vector = {3.0, 0.0};
constexpr auto default_quaternion = {4.0, 0.0};

/////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief Contains functions for micostrain driver
///
/////////////////////////////////////////////////////////////////////////////////////////////////////

namespace Microstrain
{

///
/// \brief Contains configuration information
///
class MicrostrainConfig
{
 public:
  MicrostrainConfig() = default;
  MicrostrainConfig(RosNodeType* node);

  bool configure(RosNodeType* node);
  bool connect_device(RosNodeType* node);
  bool setup_device(RosNodeType* node);
  bool setup_raw_file(RosNodeType* node);
  bool configure_gpio(RosNodeType* node);
  bool configure_imu(RosNodeType* node);
  bool configure_gnss(RosNodeType* node, uint8_t gnss_id);
  bool configure_rtk(RosNodeType* node);
  bool configure_filter(RosNodeType* node);
  bool configure_sensor2vehicle(RosNodeType* node);

  //Device pointer used to interact with the device
  std::unique_ptr<mscl::InertialNode> m_inertial_device;

  //Config read from the device
  bool m_supports_gnss1;
  bool m_supports_gnss2;
  bool m_supports_rtk;
  bool m_supports_filter;
  bool m_supports_imu;

  //Info for converting to the ENU frame
  bool m_use_enu_frame;
  tf2::Matrix3x3 m_t_ned2enu;

  //Flag for using device timestamp instead of PC received time
  bool m_use_device_timestamp;

  //FILTER
  double m_gps_leap_seconds;
  bool m_filter_enable_gnss_heading_aiding;
  bool m_filter_enable_gnss_pos_vel_aiding;
  bool m_filter_enable_altimeter_aiding;
  bool m_filter_enable_odometer_aiding;
  bool m_filter_enable_magnetometer_aiding;
  bool m_filter_enable_external_heading_aiding;
  bool m_filter_enable_external_gps_time_update;
  bool m_filter_enable_wheeled_vehicle_constraint;
  bool m_filter_enable_vertical_gyro_constraint;
  bool m_filter_enable_gnss_antenna_cal;
 
  //Frame ids
  std::string m_imu_frame_id;
  std::string m_gnss_frame_id[NUM_GNSS];
  std::string m_filter_frame_id;
  std::string m_filter_child_frame_id;
 
  //Topic strings
  std::string m_velocity_zupt_topic;
  std::string m_angular_zupt_topic;
  std::string m_external_gps_time_topic;
  
  //Publish data flags
  bool m_publish_imu;
  bool m_publish_gps_corr;
  bool m_publish_gnss[NUM_GNSS];
  bool m_publish_gnss_aiding_status[NUM_GNSS];
  bool m_publish_gnss_dual_antenna_status;
  bool m_publish_filter;
  bool m_publish_filter_relative_pos;
  bool m_publish_rtk;

  //ZUPT, angular ZUPT topic listener variables
  bool m_angular_zupt;
  bool m_velocity_zupt;
  
  //Static covariance vectors
  std::vector<double> m_imu_linear_cov;
  std::vector<double> m_imu_angular_cov;
  std::vector<double> m_imu_orientation_cov;

  // Update rates
  int m_imu_data_rate;
  int m_gnss_data_rate[NUM_GNSS];
  int m_filter_data_rate;

  //Gnss antenna offsets
  std::vector<double> m_gnss_antenna_offset[NUM_GNSS];

  //Various settings variables
  clock_t m_start;
  uint8_t m_com_mode;
  float   m_field_data[3];
  float   m_soft_iron[9];
  float   m_soft_iron_readback[9];
  float   m_angles[3];
  float   m_heading_angle;
  float   m_readback_angles[3];
  float   m_noise[3];
  float   m_beta[3];
  float   m_readback_beta[3];
  float   m_readback_noise[3];
  float   m_offset[3];
  float   m_readback_offset[3];
  double  m_reference_position_command[3];
  double  m_reference_position_readback[3];
  uint8_t m_dynamics_mode;

  //Raw data file parameters
  bool          m_raw_file_enable;
  bool          m_raw_file_include_support_data;
  std::ofstream m_raw_file;

 private:
  RosNodeType* m_node;
};  // MicrostrainConfig class

} // namespace Microstrain

#endif  // _MICROSTRAIN_CONFIG_H
