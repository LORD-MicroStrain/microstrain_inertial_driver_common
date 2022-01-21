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

#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_MICROSTRAIN_CONFIG_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_MICROSTRAIN_CONFIG_H

#include <stddef.h>
#include <string>
#include <vector>
#include <memory>
#include <fstream>

#include "mscl/mscl.h"

#include "microstrain_inertial_driver_common/microstrain_defs.h"
#include "microstrain_inertial_driver_common/microstrain_ros_funcs.h"

namespace microstrain
{

const std::vector<double> DEFAULT_MATRIX = { 9.0, 0.0 };
const std::vector<double> DEFAULT_VECTOR = { 3.0, 0.0 };
const std::vector<double> DEFAULT_QUATERNION = { 4.0, 0.0 };

/**
 * Contains configuration information for the node, configures the device on startup
 *  This class holds the pointer to the MSCL device, so any communication to the device should be done through this class
 */
class MicrostrainConfig
{
public:
  /**
   * \brief Default Constructor
   */
  MicrostrainConfig() = default;

  /**
   * \brief Constructs the config object with a reference to the ROS node. The reference will be saved as a member variable for later usage
   * \param node  The ROS node that is constructing this object.
   */
  explicit MicrostrainConfig(RosNodeType* node);

  /**
   * \brief Reads configuration, and configures the device
   * \param node  The ROS node that contains configuration information. For ROS1 this is the private node handle ("~")
   * \return true if configuration was successful and false if configuration failed
   */
  bool configure(RosNodeType* node);

  /**
   * \brief Connects to the inertial device and sets up communication
   * \param node  The ROS node that contains configuration information. For ROS1 this is the private node handle ("~")
   * \return true if connection was successful and false if connection failed
   */
  bool connectDevice(RosNodeType* node);

  /**
   * \brief Configures the device by reading options from the ROS config and sending them to the device
   * \param node  The ROS node that contains configuration information. For ROS1 this is the private node handle ("~")
   * \return true if configuration was successful and false if configuration failed
   */
  bool setupDevice(RosNodeType* node);

  /**
   * \brief Creates the raw file and enables debug mode on the device to save data to a raw file
   * \param node  The ROS node that contains configuration information. For ROS1 this is the private node handle ("~")
   * \return true if configuration was successful and false if configuration failed
   */
  bool setupRawFile(RosNodeType* node);

  /**
   * \brief Configures GPIO settings on the inertial device
   * \param node  The ROS node that contains configuration information. For ROS1 this is the private node handle ("~")
   * \return true if configuration was successful and false if configuration failed
   */
  bool configureGPIO(RosNodeType* node);

  /**
   * \brief Configures IMU settings on the inertial device
   * \param node  The ROS node that contains configuration information. For ROS1 this is the private node handle ("~")
   * \return true if configuration was successful and false if configuration failed
   */
  bool configureIMU(RosNodeType* node);

  /**
   * \brief Configures GNSS settings on the inertial device
   * \param node  The ROS node that contains configuration information. For ROS1 this is the private node handle ("~")
   * \return true if configuration was successful and false if configuration failed
   */
  bool configureGNSS(RosNodeType* node, uint8_t gnss_id);

  /**
   * \brief Configures RTK settings on the inertial device
   * \param node  The ROS node that contains configuration information. For ROS1 this is the private node handle ("~")
   * \return true if configuration was successful and false if configuration failed
   */
  bool configureRTK(RosNodeType* node);

  /**
   * \brief Configures Filter settings on the inertial device
   * \param node  The ROS node that contains configuration information. For ROS1 this is the private node handle ("~")
   * \return true if configuration was successful and false if configuration failed
   */
  bool configureFilter(RosNodeType* node);

  /**
   * \brief Configures Sensor 2 Vehicle settings on the inertial device
   * \param node  The ROS node that contains configuration information. For ROS1 this is the private node handle ("~")
   * \return true if configuration was successful and false if configuration failed
   */
  bool configureSensor2vehicle(RosNodeType* node);

  // Device pointer used to interact with the device
  std::unique_ptr<mscl::InertialNode> inertial_device_;
  std::unique_ptr<mscl::Connection> aux_connection_;

  // Config read from the device
  bool supports_gnss1_;
  bool supports_gnss2_;
  bool supports_rtk_;
  bool supports_filter_;
  bool supports_imu_;

  // Info for converting to the ENU frame
  bool use_enu_frame_;
  tf2::Matrix3x3 t_ned2enu_;
  tf2::Matrix3x3 t_vehiclebody2sensorbody_;

  // Flag for using device timestamp instead of PC received time
  bool use_device_timestamp_;

  // Flag for using ROS time instead of PC received time. If this is and use_device_timestamp_ is set, this will be preferred when setting message timestamps
  bool use_ros_time_;

  // Whether to enable the hardware odometer through the GPIO pins
  bool enable_hardware_odometer_;

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
  std::string nmea_frame_id_;

  // Topic strings
  std::string velocity_zupt_topic_;
  std::string angular_zupt_topic_;
  std::string external_gps_time_topic_;
  std::string external_speed_topic_;

  // Publish data flags
  bool publish_imu_;
  bool publish_gps_corr_;
  bool publish_gnss_[NUM_GNSS];
  bool publish_gnss_aiding_status_[NUM_GNSS];
  bool publish_gnss_dual_antenna_status_;
  bool publish_filter_;
  bool publish_filter_relative_pos_;
  bool publish_filter_aiding_measurement_summary_;
  bool publish_rtk_;
  bool publish_nmea_;

  // RTCM subscriber
  bool subscribe_rtcm_;
  std::string rtcm_topic_;

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

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_MICROSTRAIN_CONFIG_H
