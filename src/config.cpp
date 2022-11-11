/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Inertial Device Driver Implementation File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2020, Parker Hannifin Corp
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <errno.h>
#include <vector>
#include <string>
#include <memory>
#include <algorithm>

#include <yaml-cpp/yaml.h>

#include "mip/mip_version.h"
#include "mip/definitions/commands_base.hpp"
#include "mip/definitions/commands_3dm.hpp"
#include "mip/definitions/commands_gnss.hpp"
#include "mip/definitions/data_sensor.hpp"
#include "mip/definitions/data_gnss.hpp"
#include "mip/definitions/data_filter.hpp"
#include "mip/platform/serial_connection.hpp"
#include "mip/extras/recording_connection.hpp"

#include "microstrain_inertial_driver_common/utils/mappings/mip_mapping.h"
#include "microstrain_inertial_driver_common/config.h"

namespace microstrain
{

Config::Config(RosNodeType* node) : node_(node)
{
  nmea_max_rate_hz_ = 0;
}

bool Config::configure(RosNodeType* node)
{
  // Initialize some default and static config
  imu_frame_id_ = "sensor";
  gnss_frame_id_[GNSS1_ID] = "gnss1_antenna_wgs84_ned";
  gnss_frame_id_[GNSS2_ID] = "gnss2_antenna_wgs84_ned";
  filter_frame_id_ = "sensor_wgs84_ned";
  filter_child_frame_id_ = "sensor";
  nmea_frame_id_ = "nmea";
  t_ned2enu_ = tf2::Matrix3x3(0, 1, 0, 1, 0, 0, 0, 0, -1);
  t_vehiclebody2sensorbody_ = tf2::Matrix3x3(1, 0, 0, 0, -1, 0, 0, 0, -1);

  ///
  /// Generic configuration used by the rest of the driver
  ///

  // General
  getParam<bool>(node, "debug", debug_, false);

  // Device
  getParam<bool>(node, "use_device_timestamp", use_device_timestamp_, false);
  getParam<bool>(node, "use_ros_time", use_ros_time_, false);
  getParam<bool>(node, "use_enu_frame", use_enu_frame_, false);

  // If using ENU frame, reflect in the device frame id
  if (use_enu_frame_)
  {
    gnss_frame_id_[GNSS1_ID] = "gnss1_antenna_wgs84_enu";
    gnss_frame_id_[GNSS2_ID] = "gnss2_antenna_wgs84_enu";
    filter_frame_id_ = "sensor_wgs84_enu";
  }

  // IMU
  getParam<std::vector<double>>(node, "imu_orientation_cov", imu_orientation_cov_, DEFAULT_MATRIX);
  getParam<std::vector<double>>(node, "imu_linear_cov", imu_linear_cov_, DEFAULT_MATRIX);
  getParam<std::vector<double>>(node, "imu_angular_cov", imu_angular_cov_, DEFAULT_MATRIX);
  getParam<std::string>(node, "imu_frame_id", imu_frame_id_, imu_frame_id_);

  // GNSS 1/2
  std::vector<double> gnss_antenna_offset_double[NUM_GNSS];
  getParam<std::vector<double>>(node, "gnss1_antenna_offset", gnss_antenna_offset_double[GNSS1_ID], DEFAULT_VECTOR);
  getParam<std::vector<double>>(node, "gnss2_antenna_offset", gnss_antenna_offset_double[GNSS2_ID], DEFAULT_VECTOR);
  getParam<std::string>(node, "gnss1_frame_id", gnss_frame_id_[GNSS1_ID], gnss_frame_id_[GNSS1_ID]);
  getParam<std::string>(node, "gnss2_frame_id", gnss_frame_id_[GNSS2_ID], gnss_frame_id_[GNSS2_ID]);

  // HARDWARE ODOM
  getParam<bool>(node, "enable_hardware_odometer", enable_hardware_odometer_, false);

  // ROS TF control
  getParam<bool>(node, "filter_vel_in_vehicle_frame", filter_vel_in_vehicle_frame_, false);

  // RTK/GQ7 specific
  getParam<bool>(node, "rtk_dongle_enable", rtk_dongle_enable_, false);
  getParam<bool>(node, "subscribe_rtcm", subscribe_rtcm_, false);
  getParam<std::string>(node, "rtcm_topic", rtcm_topic_, std::string("/rtcm"));
  getParam<bool>(node, "publish_nmea", publish_nmea_, false);
  getParam<std::string>(node, "nmea_frame_id", nmea_frame_id_, nmea_frame_id_);

  // FILTER
  getParam<std::string>(node, "filter_frame_id", filter_frame_id_, filter_frame_id_);
  getParam<std::string>(node, "filter_child_frame_id", filter_child_frame_id_, filter_child_frame_id_);
  getParam<bool>(node, "filter_relative_position_config", filter_relative_pos_config_, false);
  getParam<double>(node, "gps_leap_seconds", gps_leap_seconds_, 18.0);
  getParam<bool>(node, "filter_angular_zupt", angular_zupt_, false);
  getParam<bool>(node, "filter_velocity_zupt", velocity_zupt_, false);
  getParam<bool>(node, "filter_enable_gnss_heading_aiding", filter_enable_gnss_heading_aiding_, true);
  getParam<bool>(node, "filter_enable_gnss_pos_vel_aiding", filter_enable_gnss_pos_vel_aiding_, true);
  getParam<bool>(node, "filter_enable_altimeter_aiding", filter_enable_altimeter_aiding_, false);
  getParam<bool>(node, "filter_enable_odometer_aiding", filter_enable_odometer_aiding_, false);
  getParam<bool>(node, "filter_enable_magnetometer_aiding", filter_enable_magnetometer_aiding_, false);
  getParam<bool>(node, "filter_enable_external_heading_aiding", filter_enable_external_heading_aiding_, false);
  getParam<bool>(node, "filter_enable_external_gps_time_update", filter_enable_external_gps_time_update_, false);
  getParam<bool>(node, "filter_enable_wheeled_vehicle_constraint", filter_enable_wheeled_vehicle_constraint_, false);
  getParam<bool>(node, "filter_enable_vertical_gyro_constraint", filter_enable_vertical_gyro_constraint_, false);
  getParam<bool>(node, "filter_enable_gnss_antenna_cal", filter_enable_gnss_antenna_cal_, false);
  getParam<std::string>(node, "filter_velocity_zupt_topic", velocity_zupt_topic_, std::string("/moving_vel"));
  getParam<std::string>(node, "filter_angular_zupt_topic", angular_zupt_topic_, std::string("/moving_ang"));
  getParam<std::string>(node, "filter_external_gps_time_topic", external_gps_time_topic_,
                         std::string("/external_gps_time"));
  getParam<std::string>(node, "filter_external_speed_topic", external_speed_topic_, "/external_speed");
  getParam<bool>(node, "filter_use_compensated_accel", filter_use_compensated_accel_, true);

  // Raw data file save
  getParam<bool>(node, "raw_file_enable", raw_file_enable_, false);
  getParam<bool>(node, "raw_file_include_support_data", raw_file_include_support_data_, false);

  // ROS2 can only fetch double vectors from config, so convert the doubles to floats for the MIP SDK
  for (int i = 0; i < NUM_GNSS; i++)
    gnss_antenna_offset_[i] = std::vector<float>(gnss_antenna_offset_double[i].begin(), gnss_antenna_offset_double[i].end());

  // Log the driver version if it was built properly
  MICROSTRAIN_INFO(node_, "Running microstrain_inertial_driver version: %s", MICROSTRAIN_DRIVER_VERSION);

  // Log the MIP SDK version
  MICROSTRAIN_INFO(node_, "Using MIP SDK version: %s", MIP_SDK_VERSION_FULL);

  // Connect to the device and set it up if we were asked to
  if (!connectDevice(node))
    return false;

  if (!setupDevice(node))
    return false;

  return true;
}

bool Config::connectDevice(RosNodeType* node)
{
  // Open the device interface
  mip_device_ = std::make_shared<RosMipDeviceMain>(node_);
  if (!mip_device_->configure(node))
    return false;

  // Connect the aux port
  if (rtk_dongle_enable_)
  {
    if (subscribe_rtcm_ || publish_nmea_)
    {
      aux_device_ = std::make_shared<RosMipDeviceAux>(node_);
      if (!aux_device_->configure(node))
      {
        // Only return an error if we were expected to subscribe to RTCM.
        if (subscribe_rtcm_)
        {
          return false;
        }
        else
        {
          MICROSTRAIN_WARN(node_, "Failed to open aux port, but we were not asked to subscribe to RTCM corrections, so this is not a fatal error");
          MICROSTRAIN_WARN(node_, "  Note: We will not publish any NMEA sentences from the aux port.");
          aux_device_ = nullptr;
        }
      }
    }
    else
    {
      MICROSTRAIN_INFO(node_, "Note: Not opening aux port since publish_nmea and subscribe_rtcm are both false");
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: Not opening aux port because RTK dongle enable was not set to true.");
    if (subscribe_rtcm_)
    {
      MICROSTRAIN_ERROR(node_, "Invalid configuration. In order to subscribe to RTCM, 'rtk_dongle_enable' must be set to true");
      return false;
    }
    else if (publish_nmea_)
    {
      MICROSTRAIN_INFO(node_, "Note: Not publishing NMEA from aux port despite 'publish_nmea' being set to true since 'rtk_donble_enable' is false");
    }
  }
  return true;
}

bool Config::setupDevice(RosNodeType* node)
{
  // Read the config used by this section
  bool device_setup;
  bool save_settings;
  bool filter_reset_after_config;
  getParam<bool>(node, "device_setup", device_setup, false);
  getParam<bool>(node, "save_settings", save_settings, true);
  getParam<bool>(node, "filter_reset_after_config", filter_reset_after_config, true);

  mip::CmdResult mip_cmd_result;

  // Configure the device to stream data using the topic mapping
  MICROSTRAIN_DEBUG(node_, "Setting up data streams");
  mip_publisher_mapping_ = std::make_shared<MipPublisherMapping>(node_, mip_device_);
  if (!mip_publisher_mapping_->configure(node))
    return false;

  // Send commands to the device to configure it
  if (device_setup)
  {
    MICROSTRAIN_DEBUG(node_, "Configuring device");
    if (!configureBase(node) ||
        !configure3DM(node) ||
        !configureGNSS(node) ||
        !configureFilter(node))
      return false;

    // Save the settings to the device, if enabled
    if (save_settings)
    {
      if (mip_device_->supportsDescriptor(mip::commands_3dm::DESCRIPTOR_SET, mip::commands_3dm::CMD_DEVICE_SETTINGS))
      {
        MICROSTRAIN_INFO(node_, "Saving the launch file configuration settings to the device");
        if (!(mip_cmd_result = mip::commands_3dm::saveDeviceSettings(*mip_device_)))
        {
          MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to save device settings");
          return false;
        }
      }
      else
      {
        MICROSTRAIN_WARN(node_, "Device does not support the device settings command");
      }
    }
    else
    {
      MICROSTRAIN_INFO(node_, "Note: The settings were not saved as startup settings. Power cycling will remove changes from device");
    }

    // Reset the filter, if enabled
    if (filter_reset_after_config)
    {
      if (mip_device_->supportsDescriptor(mip::commands_filter::DESCRIPTOR_SET, mip::commands_filter::CMD_RESET_FILTER))
      {
        MICROSTRAIN_INFO(node_, "Resetting the filter after the configuration is complete.");
        if (!(mip_cmd_result = mip::commands_filter::reset(*mip_device_)))
        {
          MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to reset filter");
          return false;
        }
      }
      else
      {
        MICROSTRAIN_WARN(node_, "Device does not support the filter reset command");
      }
    }
    else
    {
      MICROSTRAIN_INFO(node_, "Note: The filter was not reset after configuration.");
    }
  }
  return true;
}

bool Config::configureBase(RosNodeType* node)
{
  // No base configuration commands in the driver yet
  return true;
}

bool Config::configure3DM(RosNodeType* node)
{
  // Read local config
  bool gpio_config;
  bool nmea_message_config;
  int filter_pps_source;
  float hardware_odometer_scaling;
  float hardware_odometer_uncertainty;
  getParam<bool>(node, "gpio_config", gpio_config, false);
  getParam<int32_t>(node, "filter_pps_source", filter_pps_source, 1);
  getParam<float>(node, "odometer_scaling", hardware_odometer_scaling, 0.0);
  getParam<float>(node, "odometer_uncertainty", hardware_odometer_uncertainty, 0.0);

  mip::CmdResult mip_cmd_result;
  const uint8_t descriptor_set = mip::commands_3dm::DESCRIPTOR_SET;

  // Configure all available pins
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_3dm::CMD_GPIO_CONFIG))
  {
    if (gpio_config)
    {
      const int max_num_gpio = 4;
      for (int gpio_pin = 1; gpio_pin < max_num_gpio + 1; gpio_pin++)
      {
        int32_t gpio_feature;
        int32_t gpio_behavior;
        int32_t gpio_pin_mode;
        getParam<int32_t>(node, "gpio" + std::to_string(gpio_pin) + "_feature", gpio_feature, 0);
        getParam<int32_t>(node, "gpio" + std::to_string(gpio_pin) + "_behavior", gpio_behavior, 0);
        getParam<int32_t>(node, "gpio" + std::to_string(gpio_pin) + "_pin_mode", gpio_pin_mode, 0);

        MICROSTRAIN_INFO(node_, "Configuring GPIO%i to: feature = %i, behavior = %i, pinMode = %i", gpio_pin, gpio_feature, gpio_behavior, gpio_pin_mode);

        mip::commands_3dm::GpioConfig::PinMode gpio_pin_mode_bitfield;
        gpio_pin_mode_bitfield.value = gpio_pin_mode;
        if (!(mip_cmd_result = mip::commands_3dm::writeGpioConfig(*mip_device_, gpio_pin,
            static_cast<mip::commands_3dm::GpioConfig::Feature>(gpio_feature),
            static_cast<mip::commands_3dm::GpioConfig::Behavior>(gpio_behavior),
            gpio_pin_mode_bitfield)))
        {
          MICROSTRAIN_ERROR(node_, "Failed to configure GPIO%i", gpio_pin);
          MICROSTRAIN_ERROR(node_, "  Error(%d): %s", mip_cmd_result.value, mip_cmd_result.name());
          return false;
        }
      }
    }
    else
    {
      MICROSTRAIN_INFO(node_, "Note: Not configuring GPIO");
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the GPIO config command");
  }

  // Set PPS source
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_3dm::CMD_PPS_SOURCE))
  {
    MICROSTRAIN_INFO(node_, "Setting PPS source to 0x%04x", filter_pps_source);
    if (!(mip_cmd_result = mip::commands_3dm::writePpsSource(*mip_device_, static_cast<mip::commands_3dm::PpsSource::Source>(filter_pps_source))))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure PPS source");
      return false;
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the PPS source command");
  }

  // Hardware odometer configuration
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_3dm::CMD_ODOMETER_CONFIG))
  {
    const auto hardware_odometer_mode = enable_hardware_odometer_ ? mip::commands_3dm::Odometer::Mode::QUADRATURE : mip::commands_3dm::Odometer::Mode::DISABLED;
    MICROSTRAIN_INFO(node_, "Setting hardware odometer to: mode = %d, scaling = %f, uncertainty = %f", static_cast<int32_t>(hardware_odometer_mode), hardware_odometer_scaling, hardware_odometer_uncertainty);
    if (!(mip::commands_3dm::writeOdometer(*mip_device_, hardware_odometer_mode, hardware_odometer_scaling, hardware_odometer_uncertainty)))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure hardware odometer");
      return false;
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the odometer settings command");
  }

  // Support channel setup
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_3dm::CMD_CONFIGURE_FACTORY_STREAMING))
  {
    if (raw_file_include_support_data_)
    {
      if (!(mip_cmd_result = mip::commands_3dm::factoryStreaming(*mip_device_, mip::commands_3dm::FactoryStreaming::Action::MERGE, 0)))
      {
        MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure factory streaming channels");
        return false;
      }
    }
    else
    {
      MICROSTRAIN_INFO(node_, "Not configuring factory streaming channels");
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the factory streaming channels setup command");
    if (raw_file_include_support_data_)
    {
      MICROSTRAIN_ERROR(node_, "Could not configure support data even though it was requested. Exiting...");
      return false;
    }
  }
  
  // NMEA Message format
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_3dm::CMD_NMEA_MESSAGE_FORMAT))
  {
    if (nmea_message_config)
    {
      // Get the config for each descriptor set
      std::string sensor_nmea_formats, gnss1_nmea_formats, gnss2_nmea_formats, filter_nmea_formats;
      getParam<std::string>(node, "imu_nmea_messages", sensor_nmea_formats, "");
      getParam<std::string>(node, "gnss1_nmea_messages", gnss1_nmea_formats, "");
      getParam<std::string>(node, "gnss2_nmea_messages", gnss2_nmea_formats, "");
      getParam<std::string>(node, "filter_nmea_messages", filter_nmea_formats, "");

      // Populate the NMEA message config options
      std::vector<mip::commands_3dm::NmeaMessage> formats;
      if (!populateNmeaMessageFormats(sensor_nmea_formats, mip::data_sensor::DESCRIPTOR_SET, &formats) ||
          !populateNmeaMessageFormats(gnss1_nmea_formats, mip::data_gnss::MIP_GNSS1_DATA_DESC_SET, &formats) ||
          !populateNmeaMessageFormats(gnss2_nmea_formats, mip::data_gnss::MIP_GNSS2_DATA_DESC_SET, &formats) ||
          !populateNmeaMessageFormats(filter_nmea_formats, mip::data_filter::DESCRIPTOR_SET, &formats))
        return false;

      // Send them to the device
      MICROSTRAIN_INFO(node_, "Sending %lu NMEA message formats to device", formats.size());
      if (!(mip_cmd_result = mip::commands_3dm::writeNmeaMessageFormat(*mip_device_, formats.size(), formats.data())))
      {
        MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure NMEA message format");
        return false;
      }
    }
    else
    {
      MICROSTRAIN_INFO(node_, "Not configuring NMEA message format because 'nmea_message_config' is false");
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the nmea message format command");
  }

  return true;
}

bool Config::configureGNSS(RosNodeType* node)
{
  mip::CmdResult mip_cmd_result;
  const uint8_t descriptor_set = mip::commands_gnss::DESCRIPTOR_SET;

  // RTK configuration
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_gnss::CMD_RTK_DONGLE_CONFIGURATION))
  {
    uint8_t reserved[3];
    MICROSTRAIN_INFO(node_, "Setting RTK dongle enable to %d", rtk_dongle_enable_);
    if (!(mip_cmd_result = mip::commands_gnss::writeRtkDongleConfiguration(*mip_device_, rtk_dongle_enable_, reserved)))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to write RTK dongle configuration");
      return false;
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: Device does not support the RTK dongle config command");
  }

  return true;
}

bool Config::configureFilter(RosNodeType* node)
{
  // Read some generic filter info
  int heading_source;
  float initial_heading;
  bool filter_auto_init = true;
  int dynamics_mode;
  int32_t declination_source;
  double declination;
  getParam<int32_t>(node, "filter_declination_source", declination_source, 2);
  getParam<double>(node, "filter_declination", declination, 0.23);
  getParam<int32_t>(node, "filter_heading_source", heading_source, 0x1);
  getParam<float>(node, "filter_initial_heading", initial_heading, 0.0);
  getParam<bool>(node, "filter_auto_init", filter_auto_init, true);
  getParam<int32_t>(node, "filter_dynamics_mode", dynamics_mode, 1);

  // Read some QG7 specific filter options
  int filter_adaptive_level;
  int filter_adaptive_time_limit_ms;
  int filter_init_condition_src;
  int filter_auto_heading_alignment_selector;
  int filter_init_reference_frame;
  std::vector<double> filter_init_position_double(3, 0.0);
  std::vector<double> filter_init_velocity_double(3, 0.0);
  std::vector<double> filter_init_attitude_double(3, 0.0);
  int filter_relative_position_frame;
  std::vector<double> filter_relative_position_ref_double(3, 0.0);
  std::vector<double> filter_speed_lever_arm_double(3, 0.0);
  double filter_gnss_antenna_cal_max_offset;
  getParam<int32_t>(node, "filter_adaptive_level", filter_adaptive_level, 2);
  getParam<int32_t>(node, "filter_adaptive_time_limit_ms", filter_adaptive_time_limit_ms, 15000);
  getParam<int32_t>(node, "filter_init_condition_src", filter_init_condition_src, 0);
  getParam<int32_t>(node, "filter_auto_heading_alignment_selector", filter_auto_heading_alignment_selector, 0);
  getParam<int32_t>(node, "filter_init_reference_frame", filter_init_reference_frame, 2);
  getParam<std::vector<double>>(node, "filter_init_position", filter_init_position_double, DEFAULT_VECTOR);
  getParam<std::vector<double>>(node, "filter_init_velocity", filter_init_velocity_double, DEFAULT_VECTOR);
  getParam<std::vector<double>>(node, "filter_init_attitude", filter_init_attitude_double, DEFAULT_VECTOR);
  getParam<int32_t>(node, "filter_relative_position_frame", filter_relative_position_frame, 2);
  getParam<std::vector<double>>(node, "filter_relative_position_ref", filter_relative_position_ref_double, DEFAULT_VECTOR);
  getParam<std::vector<double>>(node, "filter_speed_lever_arm", filter_speed_lever_arm_double, DEFAULT_VECTOR);
  getParam<double>(node, "filter_gnss_antenna_cal_max_offset", filter_gnss_antenna_cal_max_offset, 0.1);

  // Sensor2vehicle config
  int filter_sensor2vehicle_frame_selector;
  std::vector<double> filter_sensor2vehicle_frame_transformation_euler_double(3, 0.0);
  std::vector<double> filter_sensor2vehicle_frame_transformation_matrix_double(9, 0.0);
  std::vector<double> filter_sensor2vehicle_frame_transformation_quaternion_double(4, 0.0);
  getParam<int32_t>(node, "filter_sensor2vehicle_frame_selector", filter_sensor2vehicle_frame_selector, 0);
  getParam<std::vector<double>>(node, "filter_sensor2vehicle_frame_transformation_euler", filter_sensor2vehicle_frame_transformation_euler_double, DEFAULT_VECTOR);
  getParam<std::vector<double>>(node, "filter_sensor2vehicle_frame_transformation_matrix", filter_sensor2vehicle_frame_transformation_matrix_double, DEFAULT_VECTOR);
  getParam<std::vector<double>>(node, "filter_sensor2vehicle_frame_transformation_quaternion", filter_sensor2vehicle_frame_transformation_quaternion_double, DEFAULT_VECTOR);

  // ROS2 can only fetch double vectors from config, so convert the doubles to floats for the MIP SDK
  std::vector<float> filter_init_position(filter_init_position_double.begin(), filter_init_position_double.end());
  std::vector<float> filter_init_velocity(filter_init_velocity_double.begin(), filter_init_velocity_double.end());
  std::vector<float> filter_init_attitude(filter_init_attitude_double.begin(), filter_init_attitude_double.end());

  std::vector<double> filter_relative_position_ref(filter_relative_position_ref_double.begin(), filter_relative_position_ref_double.end());
  std::vector<float> filter_speed_lever_arm(filter_speed_lever_arm_double.begin(), filter_speed_lever_arm_double.end());

  std::vector<float> filter_sensor2vehicle_frame_transformation_euler(filter_sensor2vehicle_frame_transformation_euler_double.begin(), filter_sensor2vehicle_frame_transformation_euler_double.end());
  std::vector<float> filter_sensor2vehicle_frame_transformation_matrix(filter_sensor2vehicle_frame_transformation_matrix_double.begin(), filter_sensor2vehicle_frame_transformation_matrix_double.end());
  std::vector<float> filter_sensor2vehicle_frame_transformation_quaternion(filter_sensor2vehicle_frame_transformation_quaternion_double.begin(), filter_sensor2vehicle_frame_transformation_quaternion_double.end());

  mip::CmdResult mip_cmd_result;
  const uint8_t descriptor_set = mip::commands_filter::DESCRIPTOR_SET;

  // Set Declination Source
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_DECLINATION_SOURCE))
  {
    // If the declination source is none, set the declination to 0
    const auto declination_source_enum = static_cast<mip::commands_filter::FilterMagDeclinationSource>(declination_source);
    if (declination_source_enum == mip::commands_filter::FilterMagDeclinationSource::NONE)
      declination = 0;

    MICROSTRAIN_INFO(node_, "Setting Declination Source to %d %f", declination_source, declination);
    if (!(mip_cmd_result = mip::commands_filter::writeMagneticDeclinationSource(*mip_device_, declination_source_enum, declination)))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to set declination source");
      return false;
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: Device does not support the declination source command.");
  }

  // GNSS 1/2 antenna offsets
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_ANTENNA_OFFSET))
  {
    MICROSTRAIN_INFO(node_, "Setting single antenna offset to [%f, %f, %f]",
        gnss_antenna_offset_[GNSS1_ID][0], gnss_antenna_offset_[GNSS1_ID][1], gnss_antenna_offset_[GNSS1_ID][2]);
    if (!(mip_cmd_result = mip::commands_filter::writeAntennaOffset(*mip_device_, gnss_antenna_offset_[GNSS1_ID].data())))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Cound not set single antenna offset");
      return false;
    }
  }
  else if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_MULTI_ANTENNA_OFFSET))
  {
    MICROSTRAIN_INFO(node_, "Setting GNSS1 antenna offset to [%f, %f, %f]",
        gnss_antenna_offset_[GNSS1_ID][0], gnss_antenna_offset_[GNSS1_ID][1], gnss_antenna_offset_[GNSS1_ID][2]);
    if (!(mip_cmd_result = mip::commands_filter::writeMultiAntennaOffset(*mip_device_, GNSS1_ID + 1, gnss_antenna_offset_[GNSS1_ID].data())))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Could not set multi antenna offset for GNSS1");
      return false;
    }

    MICROSTRAIN_INFO(node_, "Setting GNSS2 antenna offset to [%f, %f, %f]",
        gnss_antenna_offset_[GNSS2_ID][0], gnss_antenna_offset_[GNSS2_ID][1], gnss_antenna_offset_[GNSS2_ID][2]);
    if (!(mip_cmd_result = mip::commands_filter::writeMultiAntennaOffset(*mip_device_, GNSS2_ID + 1, gnss_antenna_offset_[GNSS2_ID].data())))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Could not set multi antenna offset for GNSS2");
      return false;
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: Device does not support GNSS antenna offsets");
  }

  // Set dynamics mode
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_VEHICLE_DYNAMICS_MODE))
  {
    MICROSTRAIN_INFO(node_, "Setting vehicle dynamics mode to 0x%02x", dynamics_mode);
    if (!(mip_cmd_result = mip::commands_filter::writeVehicleDynamicsMode(*mip_device_, static_cast<mip::commands_filter::VehicleDynamicsMode::DynamicsMode>(dynamics_mode))))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Could not set vehicle dynamics mode");
      return false;
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the vehicle dynamics mode command.");
  }

  // Set heading Source
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_HEADING_UPDATE_CONTROL))
  {
    MICROSTRAIN_INFO(node_, "Setting heading source to %d", heading_source);
    const auto heading_source_enum = static_cast<mip::commands_filter::HeadingSource::Source>(heading_source);
    if (!configureHeadingSource(heading_source_enum))
      return false;

    if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_SET_INITIAL_HEADING))
    {
      if (heading_source_enum == mip::commands_filter::HeadingSource::Source::NONE)
      {
        MICROSTRAIN_INFO(node_, "Setting initial heading to %f", initial_heading);
        if (!(mip_cmd_result = mip::commands_filter::setInitialHeading(*mip_device_, initial_heading)))
        {
          MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Could not set initial heading");
          return false;
        }
      }
      else
      {
        MICROSTRAIN_INFO(node_, "Note: Not setting initial heading because heading source is not 0");
      }
    }
    else
    {
      MICROSTRAIN_INFO(node_, "Note: Device does not support the set initial heading command");
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the heading source command.");
  }

  // Set the filter autoinitialization, if suppored
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_AUTOINIT_CONTROL))
  {
    MICROSTRAIN_INFO(node_, "Setting autoinitialization to %d", filter_auto_init);
    if (!(mip::commands_filter::writeAutoInitControl(*mip_device_, filter_auto_init)))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure filter auto initialization");
      return false;
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the filter autoinitialization command.");
  }

  // Set the filter adaptive settings
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_ADAPTIVE_FILTER_OPTIONS))
  {
    MICROSTRAIN_INFO(node_, "Setting autoadaptive options to: level = %d, time_limit = %d", filter_adaptive_level, filter_adaptive_time_limit_ms);
    if (!(mip_cmd_result = mip::commands_filter::writeAdaptiveFilterOptions(*mip_device_, filter_adaptive_level, filter_adaptive_time_limit_ms)))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure auto adaptive filter settings");
      return false;
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the filter adaptive settings command.");
  }

  // Set the filter aiding settings
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_AIDING_MEASUREMENT_ENABLE))
  {
    if (!configureFilterAidingMeasurement(mip::commands_filter::AidingMeasurementEnable::AidingSource::GNSS_POS_VEL, filter_enable_gnss_pos_vel_aiding_) ||
        !configureFilterAidingMeasurement(mip::commands_filter::AidingMeasurementEnable::AidingSource::GNSS_HEADING, filter_enable_gnss_heading_aiding_) ||
        !configureFilterAidingMeasurement(mip::commands_filter::AidingMeasurementEnable::AidingSource::ALTIMETER, filter_enable_altimeter_aiding_) ||
        !configureFilterAidingMeasurement(mip::commands_filter::AidingMeasurementEnable::AidingSource::SPEED, filter_enable_odometer_aiding_) ||
        !configureFilterAidingMeasurement(mip::commands_filter::AidingMeasurementEnable::AidingSource::MAGNETOMETER, filter_enable_magnetometer_aiding_) ||
        !configureFilterAidingMeasurement(mip::commands_filter::AidingMeasurementEnable::AidingSource::EXTERNAL_HEADING, filter_enable_external_heading_aiding_))
      return false;
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the filter aiding command.");
  }

  // Set the filter relative position frame settings
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_REL_POS_CONFIGURATION))
  {
    if (filter_relative_pos_config_)
    {
      MICROSTRAIN_INFO(node_, "Setting relative position to: [%f, %f, %f], ref frame = %d",
          filter_relative_position_ref[0], filter_relative_position_ref[1], filter_relative_position_ref[2], filter_relative_position_frame);
      if (!(mip_cmd_result = mip::commands_filter::writeRelPosConfiguration(*mip_device_, 1, static_cast<mip::commands_filter::FilterReferenceFrame>(filter_relative_position_frame), filter_relative_position_ref.data())))
      {
        MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure relative position settings");
        return false;
      }
    }
    else
    {
      MICROSTRAIN_INFO(node_, "Note: Not configuring filter relative position");
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the relative position configuration command");
  }

  // Set the filter speed lever arm
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_SPEED_LEVER_ARM))
  {
    MICROSTRAIN_INFO(node_, "Setting speed lever arm to: [%f, %f, %f]", filter_speed_lever_arm[0], filter_speed_lever_arm[1], filter_speed_lever_arm[2]);
    if (!(mip_cmd_result = mip::commands_filter::writeSpeedLeverArm(*mip_device_, 1, filter_speed_lever_arm.data())))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure speed lever arm");
      return false;
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the filter speed lever arm command.");
  }

  // Set the wheeled vehicle constraint
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_VEHICLE_CONSTRAINT_CONTROL))
  {
    MICROSTRAIN_INFO(node_, "Setting wheeled vehicle contraint enable to %d", filter_enable_wheeled_vehicle_constraint_);
    if (!(mip_cmd_result = mip::commands_filter::writeWheeledVehicleConstraintControl(*mip_device_, filter_enable_wheeled_vehicle_constraint_)))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure wheeled vehicle constraint");
      return false;
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the wheeled vehicle constraint command.");
  }

  // Set the vertical gyro constraint
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_GYRO_CONSTRAINT_CONTROL))
  {
    MICROSTRAIN_INFO(node_, "Setting vertical gyro contraint enable to %d", filter_enable_vertical_gyro_constraint_);
    if (!(mip::commands_filter::writeVerticalGyroConstraintControl(*mip_device_, filter_enable_vertical_gyro_constraint_)))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure vertical gyro constraint");
      return false;
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the vertical gyro constraint command.");
  }

  // Set the GNSS antenna calibration settings
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_ANTENNA_CALIBRATION_CONTROL))
  {
    MICROSTRAIN_INFO(node_, "Setting GNSS antenna calibration control to: enable = %d, offset = %f", filter_enable_gnss_antenna_cal_, filter_gnss_antenna_cal_max_offset);
    if (!(mip_cmd_result = mip::commands_filter::writeGnssAntennaCalControl(*mip_device_, filter_enable_gnss_antenna_cal_, filter_gnss_antenna_cal_max_offset)))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure antenna calibration");
      return false;
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the GNSS antenna calibration command.");
  }

  // Set the filter initialization settings
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_INITIALIZATION_CONFIGURATION))
  {
    MICROSTRAIN_INFO(node_, "Setting filter initialization configuration to:");
    MICROSTRAIN_INFO(node_, "  auto init = %d", filter_auto_init);
    MICROSTRAIN_INFO(node_, "  initial condition source = %d", filter_init_condition_src);
    MICROSTRAIN_INFO(node_, "  auto heading alignment selector = %d", filter_auto_heading_alignment_selector);
    MICROSTRAIN_INFO(node_, "  initial attitude = [%f, %f, %f]", filter_init_attitude[0], filter_init_attitude[1], filter_init_attitude[2]);
    MICROSTRAIN_INFO(node_, "  initial position = [%f, %f, %f]", filter_init_position[0], filter_init_position[1], filter_init_position[2]);
    MICROSTRAIN_INFO(node_, "  initial velocity = [%f, %f, %f]", filter_init_velocity[0], filter_init_velocity[1], filter_init_velocity[2]);
    MICROSTRAIN_INFO(node_, "  reference frame selector = %d", filter_init_reference_frame);
    if (!(mip_cmd_result = mip::commands_filter::writeInitializationConfiguration(*mip_device_, !filter_auto_init,
        static_cast<mip::commands_filter::InitializationConfiguration::InitialConditionSource>(filter_init_condition_src),
        static_cast<mip::commands_filter::InitializationConfiguration::AlignmentSelector>(filter_auto_heading_alignment_selector),
        filter_init_attitude[2], filter_init_attitude[1], filter_init_attitude[0],
        filter_init_position.data(), filter_init_velocity.data(),
        static_cast<mip::commands_filter::FilterReferenceFrame>(filter_init_reference_frame))))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure filter initialization");
      return false;
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the next-gen filter initialization command.");
  }

  // Sensor to vehicle configuration
  // This includes some 3DM commands because some devices do this through the 3DM descriptor set, and some do it through the filter descriptor set
  if (filter_sensor2vehicle_frame_selector == 0)
  {
    MICROSTRAIN_INFO(node_, "Note: Not configuring sensor2vehicle transformation or rotation");
  }
  else if (filter_sensor2vehicle_frame_selector == 1)
  {
    if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_SENSOR2VEHICLE_ROTATION_EULER))
    {
      MICROSTRAIN_INFO(node_, "Setting sensor to vehicle rotation euler to [%f, %f, %f]", -filter_sensor2vehicle_frame_transformation_euler[0],
          -filter_sensor2vehicle_frame_transformation_euler[1], -filter_sensor2vehicle_frame_transformation_euler[2]);
      if (!(mip_cmd_result = mip::commands_filter::writeSensorToVehicleRotationEuler(*mip_device_, -filter_sensor2vehicle_frame_transformation_euler[0],
          -filter_sensor2vehicle_frame_transformation_euler[1], -filter_sensor2vehicle_frame_transformation_euler[2])))
      {
        MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure sensor to vehicle rotation euler");
        return false;
      }
    }
    else if (mip_device_->supportsDescriptor(mip::commands_3dm::DESCRIPTOR_SET, mip::commands_3dm::CMD_SENSOR2VEHICLE_TRANSFORM_EUL))
    {
      MICROSTRAIN_INFO(node_, "Setting sensor to vehicle transformation euler to [%f, %f, %f]", filter_sensor2vehicle_frame_transformation_euler[0],
          filter_sensor2vehicle_frame_transformation_euler[1], filter_sensor2vehicle_frame_transformation_euler[2]);
      if (!(mip_cmd_result = mip::commands_3dm::writeSensor2VehicleTransformEuler(*mip_device_, -filter_sensor2vehicle_frame_transformation_euler[0],
          -filter_sensor2vehicle_frame_transformation_euler[1], -filter_sensor2vehicle_frame_transformation_euler[2])))
      {
        MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure sensor to vehicle transformation euler");
        return false;
      }
    }
    else
    {
      MICROSTRAIN_WARN(node_, "Note: The device does not support the sensor to vehicle transformation or rotation euler command");
    }
  }
  else if (filter_sensor2vehicle_frame_selector == 2)
  {
    if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_SENSOR2VEHICLE_ROTATION_DCM))
    {
      // Transpose the matrix for the rotation
      float dcm[9]
      {
          filter_sensor2vehicle_frame_transformation_matrix[0], filter_sensor2vehicle_frame_transformation_matrix[3], filter_sensor2vehicle_frame_transformation_matrix[6],
          filter_sensor2vehicle_frame_transformation_matrix[1], filter_sensor2vehicle_frame_transformation_matrix[4], filter_sensor2vehicle_frame_transformation_matrix[7],
          filter_sensor2vehicle_frame_transformation_matrix[2], filter_sensor2vehicle_frame_transformation_matrix[5], filter_sensor2vehicle_frame_transformation_matrix[8]
      };
      MICROSTRAIN_INFO(node_, "Setting sensor to vehicle rotation matrix to [ [%f, %f, %f], [%f, %f, %f], [%f, %f, %f] ]", dcm[0], dcm[1], dcm[2], dcm[3], dcm[4], dcm[5], dcm[6], dcm[7], dcm[8]);
      if (!(mip_cmd_result = mip::commands_filter::writeSensorToVehicleRotationDcm(*mip_device_, dcm)))
      {
        MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure sensor to vehicle rotation matrix");
        return false;
      }
    }
    else if (mip_device_->supportsDescriptor(mip::commands_3dm::DESCRIPTOR_SET, mip::commands_3dm::CMD_SENSOR2VEHICLE_TRANSFORM_DCM))
    {
      float dcm[9]
      {
        filter_sensor2vehicle_frame_transformation_matrix[0], filter_sensor2vehicle_frame_transformation_matrix[1], filter_sensor2vehicle_frame_transformation_matrix[2],
        filter_sensor2vehicle_frame_transformation_matrix[3], filter_sensor2vehicle_frame_transformation_matrix[4], filter_sensor2vehicle_frame_transformation_matrix[5],
        filter_sensor2vehicle_frame_transformation_matrix[6], filter_sensor2vehicle_frame_transformation_matrix[7], filter_sensor2vehicle_frame_transformation_matrix[8]
      };
      MICROSTRAIN_INFO(node_, "Setting sensor to vehicle rotation matrix to [ [%f, %f, %f], [%f, %f, %f], [%f, %f, %f] ]", dcm[0], dcm[1], dcm[2], dcm[3], dcm[4], dcm[5], dcm[6], dcm[7], dcm[8]);
      if (!(mip_cmd_result = mip::commands_3dm::writeSensor2VehicleTransformDcm(*mip_device_, dcm)))
      {
        MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure sensor to vehicle transformation matrix");
        return false;
      }
    }
    else
    {
      MICROSTRAIN_WARN(node_, "Note: The device does not support the sensor to vehicle transformation or rotation matrix command");
    }
  }
  else if (filter_sensor2vehicle_frame_selector == 3)
  {
    if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_SENSOR2VEHICLE_ROTATION_QUATERNION))
    {
      float quaternion[4]
      {
        filter_sensor2vehicle_frame_transformation_quaternion[3],
        -filter_sensor2vehicle_frame_transformation_quaternion[0],
        -filter_sensor2vehicle_frame_transformation_quaternion[1],
        -filter_sensor2vehicle_frame_transformation_quaternion[2]
      };
      MICROSTRAIN_INFO(node_, "Setting sensor to vehicle rotation quaternion to [%f, %f, %f, %f]", quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
      if (!(mip_cmd_result = mip::commands_filter::writeSensorToVehicleRotationQuaternion(*mip_device_, quaternion)))
      {
        MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure sensor to vehicle rotation quaternion");
        return false;
      }
    }
    else if (mip_device_->supportsDescriptor(mip::commands_3dm::DESCRIPTOR_SET, mip::commands_3dm::CMD_SENSOR2VEHICLE_TRANSFORM_EUL))
    {
      float quaternion[4]
      {
        filter_sensor2vehicle_frame_transformation_quaternion[0],
        filter_sensor2vehicle_frame_transformation_quaternion[1],
        filter_sensor2vehicle_frame_transformation_quaternion[2],
        filter_sensor2vehicle_frame_transformation_quaternion[3]
      };
      MICROSTRAIN_INFO(node_, "Setting sensor to vehicle transformation quaternion to [%f, %f, %f, %f]", quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
      if (!(mip_cmd_result = mip::commands_3dm::writeSensor2VehicleTransformQuaternion(*mip_device_, quaternion)))
      {
        MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure sensor to vehicle transformation quaternion");
        return false;
      }
    }
    else
    {
      MICROSTRAIN_WARN(node_, "Note: The device does not support the sensor to vehicle transformation or rotation quaternion command");
    }
  }
  else
  {
    MICROSTRAIN_ERROR(node_, "Unsupported sensor 2 vechicle frame selector: %d", filter_sensor2vehicle_frame_selector);
    return false;
  }

  return true;
}

bool Config::configureFilterAidingMeasurement(const mip::commands_filter::AidingMeasurementEnable::AidingSource aiding_source, const bool enable)
{
  // Find the name of the aiding measurement so we can log some info about it
  std::string aiding_measurement_name;
  switch (aiding_source)
  {
    case mip::commands_filter::AidingMeasurementEnable::AidingSource::GNSS_POS_VEL:
      aiding_measurement_name = "gnss pos/vel";
      break;
    case mip::commands_filter::AidingMeasurementEnable::AidingSource::GNSS_HEADING:
      aiding_measurement_name = "gnss heading";
      break;
    case mip::commands_filter::AidingMeasurementEnable::AidingSource::ALTIMETER:
      aiding_measurement_name = "altimeter";
      break;
    case mip::commands_filter::AidingMeasurementEnable::AidingSource::SPEED:
      aiding_measurement_name = "odometer";
      break;
    case mip::commands_filter::AidingMeasurementEnable::AidingSource::MAGNETOMETER:
      aiding_measurement_name = "magnetometer";
      break;
    case mip::commands_filter::AidingMeasurementEnable::AidingSource::EXTERNAL_HEADING:
      aiding_measurement_name = "external heading";
      break;
    default:
      aiding_measurement_name = std::to_string(static_cast<uint8_t>(aiding_source));
      break;
  }

  const mip::CmdResult mip_cmd_result = mip::commands_filter::writeAidingMeasurementEnable(*mip_device_, aiding_source, enable);
  if (mip_cmd_result == mip::CmdResult::NACK_INVALID_PARAM)
  {
    if (enable)
      MICROSTRAIN_WARN(node_, "Note: Filter aiding %s not supported, but it was requested. Disable in params file to remove this warning", aiding_measurement_name.c_str());
    else
      MICROSTRAIN_INFO(node_, "Note: Filter aiding %s not supported", aiding_measurement_name.c_str());
  }
  else if (!mip_cmd_result)
  {
    MICROSTRAIN_ERROR(node_, "Failed to set %s aiding measurement", aiding_measurement_name.c_str());
    MICROSTRAIN_ERROR(node_, "  Error(%d): %s", mip_cmd_result.value, mip_cmd_result.name());
    return false;
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Filter aiding %s = %d", aiding_measurement_name.c_str(), enable);
  }
  return true;
}

bool Config::configureHeadingSource(const mip::commands_filter::HeadingSource::Source heading_source)
{
  const mip::CmdResult mip_cmd_result = mip::commands_filter::writeHeadingSource(*mip_device_, heading_source);
  if (mip_cmd_result == mip::CmdResult::NACK_INVALID_PARAM)
  {
    MICROSTRAIN_WARN(node_, "Heading source 0x%02x is not valid for this device. Please refer to the device manual for more information", static_cast<uint32_t>(heading_source));
  }
  else if (!mip_cmd_result)
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to set heading source");
    return false;
  }
  return true;
}

bool Config::populateNmeaMessageFormats(const std::string& nmea_messages_config, uint8_t descriptor_set, std::vector<mip::commands_3dm::NmeaMessage>* formats)
{
  // Attempt to parse the config as yml
  YAML::Node nmea_config_yml;
  try
  {
    nmea_config_yml = YAML::Load(nmea_messages_config);
  }
  catch (const YAML::Exception& e)
  {
    MICROSTRAIN_ERROR(node_, "Encountered error while parsing NMEA messages config YML");
    MICROSTRAIN_ERROR(node_, "  Error %s", e.what());
    return false;
  }

  // Empty config is valid, we just won't do anything
  if (nmea_config_yml.IsNull())
    return true;

  // Do some validation on the overall object
  if (!nmea_config_yml.IsSequence())
  {
    MICROSTRAIN_ERROR(node_, "nmea_messages must contain an array of objects");
    return false;
  }

  static constexpr uint8_t MAX_NMEA_MESSAGE_FORMATS = 100;
  if (nmea_config_yml.size() > MAX_NMEA_MESSAGE_FORMATS)
  {
    MICROSTRAIN_ERROR(node_, "Nmea message config array contains %lu objects, but can only contain a maximum of %d", nmea_config_yml.size(), MAX_NMEA_MESSAGE_FORMATS);
    return false;
  }

  // Loop through each of the config entries
  for (size_t i = 0; i < nmea_config_yml.size(); i++)
  {
    // Get the element in the array
    const auto& nmea_config_entry = nmea_config_yml[i];

    // Find the possible entries in the object
    const auto& message_id_yml = nmea_config_entry["message_id"];
    const auto& talker_id_yml = nmea_config_entry["talker_id"];
    const auto& data_rate_yml = nmea_config_entry["data_rate"];

    // Do some validation on types and presence of fields
    bool valid_entry = true;
    if (!message_id_yml || !data_rate_yml.IsScalar())
    {
      MICROSTRAIN_ERROR(node_, "'message_id' must be present and of type 'string' or 'number'");
      valid_entry = false;
    }
    if (!data_rate_yml.IsScalar())
    {
      MICROSTRAIN_ERROR(node_, "'talker_id' must be of type 'string' or 'number'");
      valid_entry = false;
    }
    if (!data_rate_yml || !data_rate_yml.IsScalar())
    {
      MICROSTRAIN_ERROR(node_, "'data_rate' must be present and of type 'number'");
      valid_entry = false;
    }
    if (!valid_entry)
    {
      YAML::Emitter nmea_config_entry_emitter;
      nmea_config_entry_emitter << nmea_config_entry;
      MICROSTRAIN_ERROR(node_, "Invalid NMEA message entry:\n%s", nmea_config_entry_emitter.c_str());
      return false;
    }

    // Looks like this is a valid entry yml object, so let's extract the data
    mip::commands_3dm::NmeaMessage format;
    format.source_desc_set = descriptor_set;

    // Just in case the type is invalid
    int32_t data_rate;
    try
    {
      data_rate = data_rate_yml.as<int32_t>();
      format.decimation = mip_device_->getDecimationFromHertz(descriptor_set, data_rate);
    }
    catch (const YAML::TypedBadConversion<int32_t>& t)
    {
      MICROSTRAIN_ERROR(node_, "Type exception parsing 'data_rate'");
      MICROSTRAIN_ERROR(node_, "  Error: %s", t.what());
      return false;
    }

    // We accept string or number types for the message and talker IDs, so parse those out
    // Really annoying how yaml-cpp does this, so we need to try catch to determine the types
    try
    {
      format.message_id = static_cast<mip::commands_3dm::NmeaMessage::MessageID>(message_id_yml.as<uint8_t>());
    }
    catch (const YAML::TypedBadConversion<uint8_t>&)  // Bad conversion should mean the type is a string
    {
      try
      {
        // Lookup what the message id string represents
        std::string message_id_string = message_id_yml.as<std::string>();
        std::transform(message_id_string.begin(), message_id_string.end(), message_id_string.begin(), ::toupper);
        if (MipMapping::nmea_message_string_message_id_mapping_.find(message_id_string) == MipMapping::nmea_message_string_message_id_mapping_.end())
        {
          MICROSTRAIN_ERROR(node_, "Invalid 'message_id': %s", message_id_string.c_str());
          return false;
        }
        format.message_id = MipMapping::nmea_message_string_message_id_mapping_.at(message_id_string);
      }
      catch (const YAML::TypedBadConversion<std::string>& t)
      {
        MICROSTRAIN_ERROR(node_, "Type exception parsing 'message_id'");
        MICROSTRAIN_ERROR(node_, "  Error: %s", t.what());
        return false;
      }
    }

    // Talker ID is optional
    if (talker_id_yml)
    {
      try
      {
        format.talker_id = static_cast<mip::commands_3dm::NmeaMessage::TalkerID>(talker_id_yml.as<uint8_t>());
      }
      catch (const YAML::TypedBadConversion<uint8_t>&)  // Bad conversion should mean the type is a string
      {
        try
        {
          // Lookup what the talker ID string represents
          std::string talker_id_string = talker_id_yml.as<std::string>();
          std::transform(talker_id_string.begin(), talker_id_string.end(), talker_id_string.begin(), ::toupper);
          if (MipMapping::nmea_message_string_talker_id_mapping_.find(talker_id_string) == MipMapping::nmea_message_string_talker_id_mapping_.end())
          {
            MICROSTRAIN_ERROR(node_, "Invalid 'talker_id': %s", talker_id_string.c_str());
            return false;
          }
          format.talker_id = MipMapping::nmea_message_string_talker_id_mapping_.at(talker_id_string);
        }
        catch (const YAML::TypedBadConversion<std::string>& t)
        {
          MICROSTRAIN_ERROR(node_, "Type exception parsing 'message_id'");
          MICROSTRAIN_ERROR(node_, "  Error: %s", t.what());
          return false;
        }
      }
    }

    if (talker_id_yml)
    {
      MICROSTRAIN_DEBUG(node_, "Streaming NMEA sentence '%s' with talker ID: '%s' from the '%s' descriptor set to stream at %d hz",
        MipMapping::nmeaFormatMessageIdString(format.message_id).c_str(),
        MipMapping::nmeaFormatTalkerIdString(format.talker_id).c_str(),
        MipMapping::descriptorSetString(descriptor_set).c_str(),
        data_rate);
    }
    else
    {
      MICROSTRAIN_DEBUG(node_, "Streaming NMEA sentence '%s' with no talker ID from the '%s' descriptor set to stream at %d hz",
        MipMapping::nmeaFormatMessageIdString(format.message_id).c_str(),
        MipMapping::descriptorSetString(descriptor_set).c_str(),
        data_rate);
    }

    // If the data rate is 0, we can just not add the structure to the vector
    if (data_rate != 0)
    {
      // Save the data rate if it is the highest one
      if (data_rate > nmea_max_rate_hz_)
        nmea_max_rate_hz_ = data_rate;

      // Should finally have the fully formed struct, so add it to the vector
      formats->push_back(format);
    }
  }
  return true;
}

}  // namespace microstrain
