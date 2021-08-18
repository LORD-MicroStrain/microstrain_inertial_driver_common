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

#ifndef MICROSTRAIN_COMMON_MICROSTRAIN_CONFIG_H
#define MICROSTRAIN_COMMON_MICROSTRAIN_CONFIG_H

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Include Files
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#include <stddef.h>
#include <string>
#include <vector>
#include <memory>
#include <fstream>

#include "mscl/mscl.h"

#include "microstrain_common/microstrain_defs.h"
#include "microstrain_common/microstrain_ros_funcs.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief Contains functions for micostrain driver
///
/////////////////////////////////////////////////////////////////////////////////////////////////////

namespace microstrain
{

constexpr auto DEFAULT_MATRIX = { 9.0, 0.0 };
constexpr auto DEFAULT_VECTOR = { 3.0, 0.0 };
constexpr auto DEFAULT_QUATERNION = { 4.0, 0.0 };

///
/// \brief Contains configuration information
///
class MicrostrainConfig
{
public:
  MicrostrainConfig() = default;
  explicit MicrostrainConfig(RosNodeType* node);

  bool configure(RosNodeType* node);
  bool connectDevice(RosNodeType* node);
  bool setupDevice(RosNodeType* node);
  bool setupRawFile(RosNodeType* node);
  bool configureGPIO(RosNodeType* node);
  bool configureIMU(RosNodeType* node);
  bool configureGNSS(RosNodeType* node, uint8_t gnss_id);
  bool configureRTK(RosNodeType* node);
  bool configureFilter(RosNodeType* node);
  bool configureSensor2vehicle(RosNodeType* node);

  // Device pointer used to interact with the device
  std::unique_ptr<mscl::InertialNode> inertial_device_;

  // Config read from the device
  bool supports_gnss1_;
  bool supports_gnss2_;
  bool supports_rtk_;
  bool supports_filter_;
  bool supports_imu_;

  // Info for converting to the ENU frame
  bool use_enu_frame_;
  tf2::Matrix3x3 t_ned2enu_;

  // Flag for using device timestamp instead of PC received time
  bool use_device_timestamp_;

  // FILTER
  double gps_leap_seconds_;
  bool filter_enable_gnss_heading_aiding_;
  bool filter_enable_gnss_pos_vel_aiding_;
  bool filter_enable_altimeter_aiding_;
  bool filter_enable_odometer_aiding_;
  bool filter_enable_magnetometer_aiding_;
  bool filter_enable_external_heading_aiding_;
  bool filter_enable_external_gps_time_update_;
  bool filter_enable_wheeled_vehicle_constraint_;
  bool filter_enable_vertical_gyro_constraint_;
  bool filter_enable_gnss_antenna_cal_;

  // Frame ids
  std::string imu_frame_id_;
  std::string gnss_frame_id_[NUM_GNSS];
  std::string filter_frame_id_;
  std::string filter_child_frame_id_;

  // Topic strings
  std::string velocity_zupt_topic_;
  std::string angular_zupt_topic_;
  std::string external_gps_time_topic_;

  // Publish data flags
  bool publish_imu_;
  bool publish_gps_corr_;
  bool publish_gnss_[NUM_GNSS];
  bool publish_gnss_aiding_status_[NUM_GNSS];
  bool publish_gnss_dual_antenna_status_;
  bool publish_filter_;
  bool publish_filter_relative_pos_;
  bool publish_rtk_;

  // ZUPT, angular ZUPT topic listener variables
  bool angular_zupt_;
  bool velocity_zupt_;

  // Static covariance vectors
  std::vector<double> imu_linear_cov_;
  std::vector<double> imu_angular_cov_;
  std::vector<double> imu_orientation_cov_;

  // Update rates
  int imu_data_rate_;
  int gnss_data_rate_[NUM_GNSS];
  int filter_data_rate_;

  // Gnss antenna offsets
  std::vector<double> gnss_antenna_offset_[NUM_GNSS];

  // Various settings variables
  clock_t start_;
  uint8_t com_mode_;
  float field_data_[3];
  float soft_iron_[9];
  float soft_iron_readback_[9];
  float angles_[3];
  float heading_angle_;
  float readback_angles_[3];
  float noise_[3];
  float beta_[3];
  float readback_beta_[3];
  float readback_noise_[3];
  float offset_[3];
  float readback_offset_[3];
  double reference_position_command_[3];
  double reference_position_readback_[3];
  uint8_t dynamics_mode_;

  // Raw data file parameters
  bool raw_file_enable_;
  bool raw_file_include_support_data_;
  std::ofstream raw_file_;

private:
  RosNodeType* node_;
};  // MicrostrainConfig class

}  // namespace microstrain

#endif  // MICROSTRAIN_COMMON_MICROSTRAIN_CONFIG_H
