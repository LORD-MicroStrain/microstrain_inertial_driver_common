/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Inertial Device Driver Implementation File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2020, Parker Hannifin Corp
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Include Files
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#include <errno.h>
#include <sys/stat.h>
#include <vector>
#include <string>
#include <memory>
#include "microstrain_inertial_driver_common/microstrain_config.h"

namespace microstrain
{
MicrostrainConfig::MicrostrainConfig(RosNodeType* node) : node_(node)
{
}

bool MicrostrainConfig::configure(RosNodeType* node)
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

  // Device
  get_param<bool>(node, "use_device_timestamp", use_device_timestamp_, false);
  get_param<bool>(node, "use_ros_time", use_ros_time_, false);
  get_param<bool>(node, "use_enu_frame", use_enu_frame_, false);

  // If using ENU frame, reflect in the device frame id
  if (use_enu_frame_)
  {
    gnss_frame_id_[GNSS1_ID] = "gnss1_antenna_wgs84_enu";
    gnss_frame_id_[GNSS2_ID] = "gnss2_antenna_wgs84_enu";
    filter_frame_id_ = "sensor_wgs84_enu";
  }

  // IMU
  get_param<bool>(node, "publish_imu", publish_imu_, true);
  get_param<bool>(node, "publish_gps_corr", publish_gps_corr_, false);
  get_param<int32_t>(node, "imu_data_rate", imu_data_rate_, 10);
  get_param<std::vector<double>>(node, "imu_orientation_cov", imu_orientation_cov_, DEFAULT_MATRIX);
  get_param<std::vector<double>>(node, "imu_linear_cov", imu_linear_cov_, DEFAULT_MATRIX);
  get_param<std::vector<double>>(node, "imu_angular_cov", imu_angular_cov_, DEFAULT_MATRIX);
  get_param<std::string>(node, "imu_frame_id", imu_frame_id_, imu_frame_id_);

  // GNSS 1/2
  get_param<bool>(node, "publish_gnss1", publish_gnss_[GNSS1_ID], false);
  get_param<bool>(node, "publish_gnss2", publish_gnss_[GNSS2_ID], false);
  get_param<int32_t>(node, "gnss1_data_rate", gnss_data_rate_[GNSS1_ID], 1);
  get_param<int32_t>(node, "gnss2_data_rate", gnss_data_rate_[GNSS2_ID], 1);
  get_param<std::vector<double>>(node, "gnss1_antenna_offset", gnss_antenna_offset_[GNSS1_ID], DEFAULT_VECTOR);
  get_param<std::vector<double>>(node, "gnss2_antenna_offset", gnss_antenna_offset_[GNSS2_ID], DEFAULT_VECTOR);
  get_param<std::string>(node, "gnss1_frame_id", gnss_frame_id_[GNSS1_ID], gnss_frame_id_[GNSS1_ID]);
  get_param<std::string>(node, "gnss2_frame_id", gnss_frame_id_[GNSS2_ID], gnss_frame_id_[GNSS2_ID]);

  // HARDWARE ODOM
  get_param<bool>(node, "enable_hardware_odometer", enable_hardware_odometer_, false);

  // RTK/GQ7 specific
  get_param<bool>(node, "rtk_dongle_enable", publish_rtk_, false);
  get_param<bool>(node, "subscribe_rtcm", subscribe_rtcm_, false);
  get_param<std::string>(node, "rtcm_topic", rtcm_topic_, std::string("/rtcm"));
  get_param<bool>(node, "publish_nmea", publish_nmea_, false);
  get_param<std::string>(node, "nmea_frame_id", nmea_frame_id_, nmea_frame_id_);

  // FILTER
  get_param<bool>(node, "publish_filter", publish_filter_, false);
  get_param<int32_t>(node, "filter_data_rate", filter_data_rate_, 10);
  get_param<std::string>(node, "filter_frame_id", filter_frame_id_, filter_frame_id_);
  get_param<std::string>(node, "filter_child_frame_id", filter_child_frame_id_, filter_child_frame_id_);
  get_param<bool>(node, "publish_relative_position", publish_filter_relative_pos_, false);
  get_param<bool>(node, "publish_aiding_measurement_summary", publish_filter_aiding_measurement_summary_, false);
  get_param<double>(node, "gps_leap_seconds", gps_leap_seconds_, 18.0);
  get_param<bool>(node, "filter_angular_zupt", angular_zupt_, false);
  get_param<bool>(node, "filter_velocity_zupt", velocity_zupt_, false);
  get_param<bool>(node, "filter_enable_gnss_heading_aiding", filter_enable_gnss_heading_aiding_, true);
  get_param<bool>(node, "filter_enable_gnss_pos_vel_aiding", filter_enable_gnss_pos_vel_aiding_, true);
  get_param<bool>(node, "filter_enable_altimeter_aiding", filter_enable_altimeter_aiding_, false);
  get_param<bool>(node, "filter_enable_odometer_aiding", filter_enable_odometer_aiding_, false);
  get_param<bool>(node, "filter_enable_magnetometer_aiding", filter_enable_magnetometer_aiding_, false);
  get_param<bool>(node, "filter_enable_external_heading_aiding", filter_enable_external_heading_aiding_, false);
  get_param<bool>(node, "filter_enable_external_gps_time_update", filter_enable_external_gps_time_update_, false);
  get_param<bool>(node, "filter_enable_wheeled_vehicle_constraint", filter_enable_wheeled_vehicle_constraint_, false);
  get_param<bool>(node, "filter_enable_vertical_gyro_constraint", filter_enable_vertical_gyro_constraint_, false);
  get_param<bool>(node, "filter_enable_gnss_antenna_cal", filter_enable_gnss_antenna_cal_, false);
  get_param<std::string>(node, "filter_velocity_zupt_topic", velocity_zupt_topic_, std::string("/moving_vel"));
  get_param<std::string>(node, "filter_angular_zupt_topic", angular_zupt_topic_, std::string("/moving_ang"));
  get_param<std::string>(node, "filter_external_gps_time_topic", external_gps_time_topic_,
                         std::string("/external_gps_time"));
  get_param<std::string>(node, "filter_external_speed_topic", external_speed_topic_, "/external_speed");

  // Enable dual antenna messages
  publish_gnss_dual_antenna_status_ = filter_enable_gnss_heading_aiding_;

  // Raw data file save
  get_param<bool>(node, "raw_file_enable", raw_file_enable_, false);
  get_param<bool>(node, "raw_file_include_support_data", raw_file_include_support_data_, false);

  MICROSTRAIN_INFO(node_, "Using MSCL Version: %s", mscl::MSCL_VERSION.str().c_str());

  // Connect to the device and set it up if we were asked to
  bool device_setup;
  get_param<bool>(node, "device_setup", device_setup, false);

  if (!connectDevice(node))
    return false;

  if (device_setup)
  {
    if (!setupDevice(node))
      return false;
  }

  if (!setupRawFile(node))
    return false;

  return true;
}

bool MicrostrainConfig::connectDevice(RosNodeType* node)
{
  // Read the config required for only this section
  std::string port;
  std::string aux_port;
  int32_t baudrate;
  bool poll_port;
  double poll_rate_hz;
  int32_t poll_max_tries;
  get_param<std::string>(node, "port", port, "/dev/ttyACM0");
  get_param<std::string>(node, "aux_port", aux_port, "/dev/ttyACM1");
  get_param<int32_t>(node, "baudrate", baudrate, 115200);
  get_param<bool>(node, "poll_port", poll_port, false);
  get_param<double>(node, "poll_rate_hz", poll_rate_hz, 1.0);
  get_param<int32_t>(node, "poll_max_tries", poll_max_tries, 60);

  // If we were asked to, poll the port until it exists
  if (poll_port)
  {
    int32_t poll_tries = 0;
    RosRateType poll_rate(poll_rate_hz);
    struct stat port_stat;
    while (stat(port.c_str(), &port_stat) != 0 && (poll_tries++ < poll_max_tries || poll_max_tries == -1))
    {
      // If the error isn't that the file does not exist, polling won't help, so we can fail here
      if (errno != ENOENT)
      {
        MICROSTRAIN_ERROR(node_,
            "Error while polling for file %s. File appears to exist, but stat returned error: %s",
            port.c_str(), strerror(errno));
        return false;
      }

      // Wait for the specified amount of time
      MICROSTRAIN_WARN(node_, "%s doesn't exist yet. Waiting for file to appear...", port.c_str());
      poll_rate.sleep();
    }

    // If the file still doesn't exist we can safely fail here.
    if (stat(port.c_str(), &port_stat) != 0)
    {
      MICROSTRAIN_ERROR(node_, "Unable to open requested port, error: %s", strerror(errno));
      return false;
    }
  }

  try
  {
    //
    // Initialize the serial interface to the device and create the inertial device object
    //
    MICROSTRAIN_INFO(node_, "Attempting to open serial port <%s> at <%d>", port.c_str(), baudrate);

    mscl::Connection connection = mscl::Connection::Serial(realpath(port.c_str(), 0), (uint32_t)baudrate);
    inertial_device_ = std::unique_ptr<mscl::InertialNode>(new mscl::InertialNode(connection));

    // At this point, we have connected to the device but if it is streaming.
    // Reading information may fail. Retry a few times to accomodate
    int32_t set_to_idle_tries = 0;
    while (set_to_idle_tries++ < 3)
    {
      try
      {
        // Put into idle mode
        MICROSTRAIN_INFO(node_, "Setting to Idle: Stopping data streams and/or waking from sleep");
        inertial_device_->setToIdle();
        break;  // If setting to idle succeeded, we don't need to loop anymore as the device can now communicate
      }
      catch (const mscl::Error_Communication& e)
      {
        MICROSTRAIN_WARN(node_,
            "It looks like the device is streaming. Waiting a second before setting to idle again");
        RosRateType rate(1.0);
        rate.sleep();
      }
    }

    // Print the device info
    MICROSTRAIN_INFO(node_, R"(
      #######################
      Model Name:    %s
      Serial Number: %s
      #######################
    )", inertial_device_->modelName().c_str(), inertial_device_->serialNumber().c_str());

    // Get supported features
    supports_gnss1_ = inertial_device_->features().supportsCategory(mscl::MipTypes::DataClass::CLASS_GNSS) |
                       inertial_device_->features().supportsCategory(mscl::MipTypes::DataClass::CLASS_GNSS1);
    supports_gnss2_ = inertial_device_->features().supportsCategory(mscl::MipTypes::DataClass::CLASS_GNSS2);
    supports_rtk_ = inertial_device_->features().supportsCategory(mscl::MipTypes::DataClass::CLASS_GNSS3);
    supports_filter_ = inertial_device_->features().supportsCategory(mscl::MipTypes::DataClass::CLASS_ESTFILTER);
    supports_imu_ = inertial_device_->features().supportsCategory(mscl::MipTypes::DataClass::CLASS_AHRS_IMU);

    // Connect the aux port if we were asked to stream RTCM corrections
    if (supports_rtk_ && (subscribe_rtcm_ || publish_nmea_))
    {
      MICROSTRAIN_INFO(node_, "Attempting to open aux serial port <%s> at <%d>", aux_port.c_str(), baudrate);
      aux_connection_ = std::unique_ptr<mscl::Connection>(new mscl::Connection(mscl::Connection::Serial(realpath(aux_port.c_str(), 0), (uint32_t)baudrate)));
      aux_connection_->rawByteMode(true);
    }

  }
  catch (mscl::Error_Connection& e)
  {
    MICROSTRAIN_ERROR(node_, "Device Disconnected");
    return false;
  }
  catch (mscl::Error& e)
  {
    MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    return false;
  }
  return true;
}

bool MicrostrainConfig::setupDevice(RosNodeType* node)
{
  // Read the config used by this section
  bool save_settings;
  bool gpio_config;
  bool filter_reset_after_config;
  get_param<bool>(node, "save_settings", save_settings, true);
  get_param<bool>(node, "gpio_config", gpio_config, false);
  get_param<bool>(node, "filter_reset_after_config", filter_reset_after_config, true);

  // GPIO config
  if (inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_GPIO_CONFIGURATION) && gpio_config)
  {
    if (!configureGPIO(node))
      return false;
  }

  // IMU Setup
  if (publish_imu_ && supports_imu_)
  {
    if (!configureIMU(node))
      return false;
  }

  // GNSS1 setup
  if (supports_gnss1_)
  {
    if (!configureGNSS(node, GNSS1_ID))
      return false;
  }

  // GNSS2 setup
  if (supports_gnss2_)
  {
    if (!configureGNSS(node, GNSS2_ID))
      return false;
  }

  // RTK Dongle
  if (supports_rtk_)
  {
    if (!configureRTK(node))
      return false;
  }

  // Filter setup
  if (publish_filter_ && supports_filter_)
  {
    if (!configureFilter(node))
      return false;
  }

  // Sensor2Vehicle setup
  if (!configureSensor2vehicle(node))
    return false;

  // Support channel setup
  if (raw_file_enable_ && raw_file_include_support_data_)
  {
    if (inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_FACTORY_STREAMING))
    {
      MICROSTRAIN_INFO(node_, "Enabling factory support channels");
      inertial_device_->setFactoryStreamingChannels(mscl::InertialTypes::FactoryStreamingOption::FACTORY_STREAMING_MERGE);
    }
    else
    {
      MICROSTRAIN_ERROR(node_, "**The device does not support the factory streaming channels setup command!");
    }
  }

  // Save the settings to the device, if enabled
  if (save_settings)
  {
    MICROSTRAIN_INFO(node_, "Saving the launch file configuration settings to the device");
    inertial_device_->saveSettingsAsStartup();
  }

  // Reset the filter, if enabled
  if (filter_reset_after_config &&
      inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_RESET_FILTER))
  {
    MICROSTRAIN_INFO(node_, "Resetting the filter after the configuration is complete.");
    inertial_device_->resetFilter();
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The filter was not reset after configuration.");
  }

  // Resume the device
  inertial_device_->resume();
  return true;
}

bool MicrostrainConfig::setupRawFile(RosNodeType* node)
{
  // Open raw data file, if enabled, and configure the device for raw data output
  std::string raw_file_directory;
  get_param<std::string>(node, "raw_file_directory", raw_file_directory, std::string("."));

  if (raw_file_enable_)
  {
    time_t raw_time;
    struct tm curr_time;
    char curr_time_buffer[100];

    // Get the current time
    time(&raw_time);
    localtime_r(&raw_time, &curr_time);
    strftime(curr_time_buffer, sizeof(curr_time_buffer), "%y_%m_%d_%H_%M_%S", &curr_time);

    std::string time_string(curr_time_buffer);

    std::string filename = raw_file_directory + std::string("/") + inertial_device_->modelName() + std::string("_") +
                           inertial_device_->serialNumber() + std::string("_") + time_string + std::string(".bin");

    raw_file_.open(filename, std::ios::out | std::ios::binary | std::ios::trunc);

    if (!raw_file_.is_open())
    {
      MICROSTRAIN_ERROR(node_, "ERROR opening raw binary datafile at %s", filename.c_str());
      return false;
    }
    else
    {
      MICROSTRAIN_INFO(node_, "Raw binary datafile opened at %s", filename.c_str());
    }

    inertial_device_->connection().debugMode(true);
  }
  return true;
}

bool MicrostrainConfig::configureGPIO(RosNodeType* node)
{
  // Read the config required only by this section
  int32_t gpio1_feature;
  int32_t gpio1_behavior;
  int32_t gpio1_pin_mode;
  int32_t gpio2_feature;
  int32_t gpio2_behavior;
  int32_t gpio2_pin_mode;
  int32_t gpio3_feature;
  int32_t gpio3_behavior;
  int32_t gpio3_pin_mode;
  int32_t gpio4_feature;
  int32_t gpio4_behavior;
  int32_t gpio4_pin_mode;
  get_param<int32_t>(node, "gpio1_feature", gpio1_feature, 0);
  get_param<int32_t>(node, "gpio1_behavior", gpio1_behavior, 0);
  get_param<int32_t>(node, "gpio1_pin_mode", gpio1_pin_mode, 0);
  get_param<int32_t>(node, "gpio2_feature", gpio2_feature, 0);
  get_param<int32_t>(node, "gpio2_behavior", gpio2_behavior, 0);
  get_param<int32_t>(node, "gpio2_pin_mode", gpio2_pin_mode, 0);
  get_param<int32_t>(node, "gpio3_feature", gpio3_feature, 0);
  get_param<int32_t>(node, "gpio3_behavior", gpio3_behavior, 0);
  get_param<int32_t>(node, "gpio3_pin_mode", gpio3_pin_mode, 0);
  get_param<int32_t>(node, "gpio4_feature", gpio4_feature, 0);
  get_param<int32_t>(node, "gpio4_behavior", gpio4_behavior, 0);
  get_param<int32_t>(node, "gpio4_pin_mode", gpio4_pin_mode, 0);

  try
  {
    mscl::GpioConfiguration gpioConfig;

    gpioConfig.pin = 1;
    gpioConfig.feature = static_cast<mscl::GpioConfiguration::Feature>(gpio1_feature);
    gpioConfig.behavior = gpio1_behavior;
    gpioConfig.pinMode.value(gpio1_pin_mode);
    inertial_device_->setGpioConfig(gpioConfig);

    MICROSTRAIN_INFO(node_, "Configuring GPIO1 to feature: %i, behavior: %i, pinMode: %i", gpio1_feature,
                     gpio1_behavior, gpio1_pin_mode);

    gpioConfig.pin = 2;
    gpioConfig.feature = static_cast<mscl::GpioConfiguration::Feature>(gpio2_feature);
    gpioConfig.behavior = gpio2_behavior;
    gpioConfig.pinMode.value(gpio4_pin_mode);
    inertial_device_->setGpioConfig(gpioConfig);

    MICROSTRAIN_INFO(node_, "Configuring GPIO2 to feature: %i, behavior: %i, pinMode: %i", gpio2_feature,
                     gpio2_behavior, gpio2_pin_mode);

    gpioConfig.pin = 3;
    gpioConfig.feature = static_cast<mscl::GpioConfiguration::Feature>(gpio3_feature);
    gpioConfig.behavior = gpio3_behavior;
    gpioConfig.pinMode.value(gpio4_pin_mode);
    inertial_device_->setGpioConfig(gpioConfig);

    MICROSTRAIN_INFO(node_, "Configuring GPIO3 to feature: %i, behavior: %i, pinMode: %i", gpio3_feature,
                     gpio3_behavior, gpio3_pin_mode);

    gpioConfig.pin = 4;
    gpioConfig.feature = static_cast<mscl::GpioConfiguration::Feature>(gpio4_feature);
    gpioConfig.behavior = gpio4_behavior;
    gpioConfig.pinMode.value(gpio4_pin_mode);
    inertial_device_->setGpioConfig(gpioConfig);

    MICROSTRAIN_INFO(node_, "Configuring GPIO4 to feature: %i, behavior: %i, pinMode: %i", gpio4_feature,
                     gpio4_behavior, gpio4_pin_mode);
  }
  catch (mscl::Error& e)
  {
    MICROSTRAIN_ERROR(node_, "GPIO Config Error: %s", e.what());
    return false;
  }

  return true;
}

bool MicrostrainConfig::configureIMU(RosNodeType* node)
{
  // Read the config required only by this section
  int32_t declination_source;
  double declination;
  get_param<int32_t>(node, "filter_declination_source", declination_source, 2);
  get_param<double>(node, "filter_declination", declination, 0.23);

  mscl::SampleRate imu_rate = mscl::SampleRate::Hertz(imu_data_rate_);

  MICROSTRAIN_INFO(node_, "Setting IMU data to stream at %d hz", imu_data_rate_);

  mscl::MipTypes::MipChannelFields ahrsChannels
  {
    mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_SCALED_ACCEL_VEC,
    mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_SCALED_GYRO_VEC,
    mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_ORIENTATION_QUATERNION,
    mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_SCALED_MAG_VEC,
    mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_GPS_CORRELATION_TIMESTAMP
  };

  mscl::MipChannels supportedChannels;
  for (mscl::MipTypes::ChannelField channel :
       inertial_device_->features().supportedChannelFields(mscl::MipTypes::DataClass::CLASS_AHRS_IMU))
  {
    if (std::find(ahrsChannels.begin(), ahrsChannels.end(), channel) != ahrsChannels.end())
    {
      supportedChannels.push_back(mscl::MipChannel(channel, imu_rate));
    }
  }

  inertial_device_->setActiveChannelFields(mscl::MipTypes::DataClass::CLASS_AHRS_IMU, supportedChannels);

  if (inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_DECLINATION_SRC))
  {
    MICROSTRAIN_INFO(node_, "Setting Declination Source");
    inertial_device_->setDeclinationSource(mscl::GeographicSourceOptions(
        static_cast<mscl::InertialTypes::GeographicSourceOption>((uint8_t)declination_source), declination));
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: Device does not support the declination source command.");
  }

  inertial_device_->enableDataStream(mscl::MipTypes::DataClass::CLASS_AHRS_IMU);
  return true;
}

bool MicrostrainConfig::configureGNSS(RosNodeType* node, uint8_t gnss_id)
{
  // Set the antenna offset, if supported (needs to process 2 different ways for old devices vs. new for GNSS1)
  mscl::PositionOffset antenna_offset(gnss_antenna_offset_[gnss_id][0], gnss_antenna_offset_[gnss_id][1],
                                      gnss_antenna_offset_[gnss_id][2]);

  if (inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_ANTENNA_OFFSET))
  {
    MICROSTRAIN_INFO(node_, "Setting GNSS%d antenna offset to [%f, %f, %f]", gnss_id + 1, antenna_offset.x(),
                     antenna_offset.y(), antenna_offset.z());
    inertial_device_->setAntennaOffset(antenna_offset);
  }
  else if (inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_MULTI_ANTENNA_OFFSET))
  {
    MICROSTRAIN_INFO(node_, "Setting GNSS%d antenna offset to [%f, %f, %f]", gnss_id + 1, antenna_offset.x(),
                     antenna_offset.y(), antenna_offset.z());
    inertial_device_->setMultiAntennaOffset(gnss_id + 1, antenna_offset);
  }
  else
  {
    MICROSTRAIN_ERROR(node_, "Could not set GNSS%d antenna offset!", gnss_id + 1);
    return false;
  }

  // If we were requested to publish GNSS data, set up streaming at the requested rate
  if (publish_gnss_[gnss_id])
  {
    mscl::SampleRate gnss1_rate = mscl::SampleRate::Hertz(gnss_data_rate_[gnss_id]);

    MICROSTRAIN_INFO(node_, "Setting GNSS%d data to stream at %d hz", gnss_id, gnss_data_rate_[gnss_id]);

    mscl::MipTypes::MipChannelFields gnssChannels
    {
      mscl::MipTypes::ChannelField::CH_FIELD_GNSS_LLH_POSITION,
      mscl::MipTypes::ChannelField::CH_FIELD_GNSS_NED_VELOCITY,
      mscl::MipTypes::ChannelField::CH_FIELD_GNSS_GPS_TIME
    };

    mscl::MipTypes::DataClass gnss_data_class = mscl::MipTypes::DataClass::CLASS_GNSS;

    if (inertial_device_->features().supportsCategory(mscl::MipTypes::DataClass::CLASS_GNSS1) && gnss_id == GNSS1_ID)
    {
      gnss_data_class = mscl::MipTypes::DataClass::CLASS_GNSS1;

      gnssChannels.clear();
      gnssChannels.push_back(mscl::MipTypes::ChannelField::CH_FIELD_GNSS_1_LLH_POSITION);
      gnssChannels.push_back(mscl::MipTypes::ChannelField::CH_FIELD_GNSS_1_NED_VELOCITY);
      gnssChannels.push_back(mscl::MipTypes::ChannelField::CH_FIELD_GNSS_1_GPS_TIME);
      gnssChannels.push_back(mscl::MipTypes::ChannelField::CH_FIELD_GNSS_1_FIX_INFO);
    }
    else if (inertial_device_->features().supportsCategory(mscl::MipTypes::DataClass::CLASS_GNSS2) &&
            gnss_id == GNSS2_ID)
    {
      gnss_data_class = mscl::MipTypes::DataClass::CLASS_GNSS2;

      gnssChannels.clear();
      gnssChannels.push_back(mscl::MipTypes::ChannelField::CH_FIELD_GNSS_2_LLH_POSITION);
      gnssChannels.push_back(mscl::MipTypes::ChannelField::CH_FIELD_GNSS_2_NED_VELOCITY);
      gnssChannels.push_back(mscl::MipTypes::ChannelField::CH_FIELD_GNSS_2_GPS_TIME);
      gnssChannels.push_back(mscl::MipTypes::ChannelField::CH_FIELD_GNSS_2_FIX_INFO);
    }

    mscl::MipChannels supportedChannels;
    for (mscl::MipTypes::ChannelField channel : inertial_device_->features().supportedChannelFields(gnss_data_class))
    {
      if (std::find(gnssChannels.begin(), gnssChannels.end(), channel) != gnssChannels.end())
      {
        supportedChannels.push_back(mscl::MipChannel(channel, gnss1_rate));
      }
    }

    // set the GNSS channel fields
    inertial_device_->setActiveChannelFields(gnss_data_class, supportedChannels);

    // Enable publishing aiding status messages
    publish_gnss_aiding_status_[gnss_id] = inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_AIDING_MEASUREMENT_ENABLE);
    if (!publish_gnss_aiding_status_[gnss_id])
    {
      MICROSTRAIN_INFO(node_, "Note: Device not support publishing GNSS Aiding measurements.");
    }

    inertial_device_->enableDataStream(gnss_data_class);
  }

  return true;
}

bool MicrostrainConfig::configureRTK(RosNodeType* node)
{
  if (publish_rtk_)
  {
    mscl::SampleRate gnss3_rate = mscl::SampleRate::Hertz(1);

    MICROSTRAIN_INFO(node_, "Setting RTK data to stream at 1 hz");

    mscl::MipTypes::MipChannelFields gnssChannels{ mscl::MipTypes::ChannelField::CH_FIELD_GNSS_3_RTK_CORRECTIONS_STATUS };

    mscl::MipChannels supportedChannels;
    for (mscl::MipTypes::ChannelField channel :
        inertial_device_->features().supportedChannelFields(mscl::MipTypes::DataClass::CLASS_GNSS3))
    {
      if (std::find(gnssChannels.begin(), gnssChannels.end(), channel) != gnssChannels.end())
      {
        supportedChannels.push_back(mscl::MipChannel(channel, gnss3_rate));
      }
    }

    // set the GNSS channel fields
    inertial_device_->setActiveChannelFields(mscl::MipTypes::DataClass::CLASS_GNSS3, supportedChannels);

    inertial_device_->enableDataStream(mscl::MipTypes::DataClass::CLASS_GNSS3);
  }

  // Check if the device even supports the RTK command, and enable/disable accordingly
  if (inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_GNSS_RTK_CONFIG))
  {
    MICROSTRAIN_INFO(node_, "Setting RTK dongle enable to %d", publish_rtk_);
    inertial_device_->enableRtk(publish_rtk_);
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: Device does not support the RTK dongle config command");
  }

  return true;
}

bool MicrostrainConfig::configureFilter(RosNodeType* node)
{
  // Read some generic filter info
  int heading_source;
  float initial_heading;
  bool filter_auto_init = true;
  int dynamics_mode;
  float hardware_odometer_scaling;
  float hardware_odometer_uncertainty;
  get_param<int32_t>(node, "filter_heading_source", heading_source, 0x1);
  get_param<float>(node, "filter_initial_heading", initial_heading, 0.0);
  get_param<bool>(node, "filter_auto_init", filter_auto_init, true);
  get_param<int32_t>(node, "filter_dynamics_mode", dynamics_mode, 1);
  get_param<float>(node, "odometer_scaling", hardware_odometer_scaling, 0.0);
  get_param<float>(node, "odometer_uncertainty", hardware_odometer_uncertainty, 0.0);

  // Read some QG7 specific filter options
  int filter_adaptive_level;
  int filter_adaptive_time_limit_ms;
  int filter_init_condition_src;
  int filter_auto_heading_alignment_selector;
  int filter_init_reference_frame;
  std::vector<double> filter_init_position(3, 0.0);
  std::vector<double> filter_init_velocity(3, 0.0);
  std::vector<double> filter_init_attitude(3, 0.0);
  int filter_relative_position_frame;
  std::vector<double> filter_relative_position_ref(3, 0.0);
  std::vector<double> filter_speed_lever_arm(3, 0.0);
  double filter_gnss_antenna_cal_max_offset;
  int filter_pps_source;
  get_param<int32_t>(node, "filter_adaptive_level", filter_adaptive_level, 2);
  get_param<int32_t>(node, "filter_adaptive_time_limit_ms", filter_adaptive_time_limit_ms, 15000);
  get_param<int32_t>(node, "filter_init_condition_src", filter_init_condition_src, 0);
  get_param<int32_t>(node, "filter_auto_heading_alignment_selector", filter_auto_heading_alignment_selector, 0);
  get_param<int32_t>(node, "filter_init_reference_frame", filter_init_reference_frame, 2);
  get_param<std::vector<double>>(node, "filter_init_position", filter_init_position, DEFAULT_VECTOR);
  get_param<std::vector<double>>(node, "filter_init_velocity", filter_init_velocity, DEFAULT_VECTOR);
  get_param<std::vector<double>>(node, "filter_init_attitude", filter_init_attitude, DEFAULT_VECTOR);
  get_param<int32_t>(node, "filter_relative_position_frame", filter_relative_position_frame, 2);
  get_param<std::vector<double>>(node, "filter_relative_position_ref", filter_relative_position_ref, DEFAULT_VECTOR);
  get_param<std::vector<double>>(node, "filter_speed_lever_arm", filter_speed_lever_arm, DEFAULT_VECTOR);
  get_param<double>(node, "filter_gnss_antenna_cal_max_offset", filter_gnss_antenna_cal_max_offset, 0.1);
  get_param<int32_t>(node, "filter_pps_source", filter_pps_source, 1);

  mscl::SampleRate filter_rate = mscl::SampleRate::Hertz(filter_data_rate_);

  MICROSTRAIN_INFO(node_, "Setting Filter data to stream at %d hz", filter_data_rate_);

  mscl::MipTypes::MipChannelFields navChannels
  {
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_GPS_TIMESTAMP,
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_FILTER_STATUS,
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_LLH_POS,
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_NED_VELOCITY,
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_QUATERNION,
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_LLH_UNCERT,
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_NED_UNCERT,
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ATT_UNCERT_QUAT,
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ANGULAR_RATE,
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ATT_UNCERT_EULER,
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_COMPENSATED_ACCEL,
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_EULER,
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_HEADING_UPDATE_SOURCE,
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_NED_RELATIVE_POS
  };

  if (filter_enable_gnss_pos_vel_aiding_)
    navChannels.push_back(mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_POSITION_AIDING_STATUS);
  if (filter_enable_gnss_heading_aiding_)
    navChannels.push_back(mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_GNSS_DUAL_ANTENNA_STATUS);
  if (publish_filter_aiding_measurement_summary_)
    navChannels.push_back(mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_AIDING_MEASURE_SUMMARY);

  mscl::MipChannels supportedChannels;
  for (mscl::MipTypes::ChannelField channel :
       inertial_device_->features().supportedChannelFields(mscl::MipTypes::DataClass::CLASS_ESTFILTER))
  {
    if (std::find(navChannels.begin(), navChannels.end(), channel) != navChannels.end())
    {
      supportedChannels.push_back(mscl::MipChannel(channel, filter_rate));
    }
  }

  inertial_device_->setActiveChannelFields(mscl::MipTypes::DataClass::CLASS_ESTFILTER, supportedChannels);

  // set dynamics mode
  if (inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_VEHIC_DYNAMICS_MODE))
  {
    mscl::VehicleModeTypes modes = inertial_device_->features().supportedVehicleModeTypes();
    if (std::find(modes.begin(), modes.end(), static_cast<mscl::InertialTypes::VehicleModeType>(dynamics_mode)) !=
        modes.end())
    {
      MICROSTRAIN_INFO(node_, "Setting dynamics mode to %#04X",
                       static_cast<mscl::InertialTypes::VehicleModeType>(dynamics_mode));
      inertial_device_->setVehicleDynamicsMode(static_cast<mscl::InertialTypes::VehicleModeType>(dynamics_mode));
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the vehicle dynamics mode command.");
  }

  // Set PPS source
  if (inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_PPS_SOURCE))
  {
    mscl::PpsSourceOptions sources = inertial_device_->features().supportedPpsSourceOptions();
    if (std::find(sources.begin(), sources.end(), static_cast<mscl::InertialTypes::PpsSource>(filter_pps_source)) !=
        sources.end())
    {
      MICROSTRAIN_INFO(node_, "Setting PPS source to %#04X",
                       static_cast<mscl::InertialTypes::PpsSource>(filter_pps_source));
      inertial_device_->setPpsSource(static_cast<mscl::InertialTypes::PpsSource>(filter_pps_source));
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the PPS source command.");
  }

  // Set heading Source
  if (inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_HEADING_UPDATE_CTRL))
  {
    for (mscl::HeadingUpdateOptions headingSources : inertial_device_->features().supportedHeadingUpdateOptions())
    {
      if (headingSources.AsOptionId() == static_cast<mscl::InertialTypes::HeadingUpdateEnableOption>(heading_source))
      {
        MICROSTRAIN_INFO(node_, "Setting heading source to %#04X", heading_source);
        inertial_device_->setHeadingUpdateControl(
            mscl::HeadingUpdateOptions(static_cast<mscl::InertialTypes::HeadingUpdateEnableOption>(heading_source)));
        break;
      }
    }

    // Set the initial heading
    if ((heading_source == 0) &&
        (inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_INIT_HEADING)))
    {
      MICROSTRAIN_INFO(node_, "Setting initial heading to %f", initial_heading);
      inertial_device_->setInitialHeading(initial_heading);
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the heading source command.");
  }

  // Set the filter autoinitialization, if suppored
  if (inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_AUTO_INIT_CTRL))
  {
    MICROSTRAIN_INFO(node_, "Setting autoinitialization to %d", filter_auto_init);
    inertial_device_->setAutoInitialization(filter_auto_init);
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the filter autoinitialization command.");
  }

  // (GQ7 and GX5-45 only) Set the filter adaptive settings
  if (inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_ADAPTIVE_FILTER_OPTIONS))
  {
    MICROSTRAIN_INFO(node_, "Setting autoadaptive options to: level = %d, time_limit = %d", filter_adaptive_level,
                     filter_adaptive_time_limit_ms);
    mscl::AutoAdaptiveFilterOptions options(
        static_cast<mscl::InertialTypes::AutoAdaptiveFilteringLevel>(filter_adaptive_level),
        (uint16_t)filter_adaptive_time_limit_ms);

    inertial_device_->setAdaptiveFilterOptions(options);
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the filte adaptive settings command.");
  }

  // (GQ7 only) Set the filter aiding settings
  if (inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_AIDING_MEASUREMENT_ENABLE))
  {
    MICROSTRAIN_INFO(node_,
                     "Filter aiding set to: pos/vel = %d, gnss heading = %d, altimeter = %d, odometer = %d, "
                     "magnetometer = %d, external heading = %d",
                     filter_enable_gnss_heading_aiding_, filter_enable_gnss_heading_aiding_,
                     filter_enable_altimeter_aiding_, filter_enable_odometer_aiding_,
                     filter_enable_magnetometer_aiding_, filter_enable_external_heading_aiding_);

    inertial_device_->enableDisableAidingMeasurement(mscl::InertialTypes::AidingMeasurementSource::GNSS_POS_VEL_AIDING,
                                                      filter_enable_gnss_pos_vel_aiding_);
    inertial_device_->enableDisableAidingMeasurement(mscl::InertialTypes::AidingMeasurementSource::GNSS_HEADING_AIDING,
                                                      filter_enable_gnss_heading_aiding_);
    inertial_device_->enableDisableAidingMeasurement(mscl::InertialTypes::AidingMeasurementSource::ALTIMETER_AIDING,
                                                      filter_enable_altimeter_aiding_);
    inertial_device_->enableDisableAidingMeasurement(mscl::InertialTypes::AidingMeasurementSource::ODOMETER_AIDING,
                                                      filter_enable_odometer_aiding_);
    inertial_device_->enableDisableAidingMeasurement(mscl::InertialTypes::AidingMeasurementSource::MAGNETOMETER_AIDING,
                                                      filter_enable_magnetometer_aiding_);
    inertial_device_->enableDisableAidingMeasurement(
        mscl::InertialTypes::AidingMeasurementSource::EXTERNAL_HEADING_AIDING, filter_enable_external_heading_aiding_);
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the filter aiding command.");
  }

  // (GQ7 only) Set the filter relative position frame settings
  if (publish_filter_relative_pos_ &&
      inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_RELATIVE_POSITION_REF))
  {
    mscl::PositionReferenceConfiguration ref;
    ref.position = mscl::Position(filter_relative_position_ref[0], filter_relative_position_ref[1],
                                  filter_relative_position_ref[2],
                                  static_cast<mscl::PositionVelocityReferenceFrame>(filter_relative_position_frame));

    MICROSTRAIN_INFO(node_, "Setting reference position to: [%f, %f, %f], ref frame = %d",
                     filter_relative_position_ref[0], filter_relative_position_ref[1], filter_relative_position_ref[2],
                     filter_relative_position_frame);
    inertial_device_->setRelativePositionReference(ref);
  }
  else if (publish_filter_relative_pos_)
  {
    MICROSTRAIN_ERROR(node_, "The device does not support the relative position command, but it was requested with \"publish_relative_position\"");
    return false;
  }

  // (GQ7 only) Set the filter speed lever arm
  if (inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SPEED_MEASUREMENT_OFFSET))
  {
    mscl::PositionOffset offset(filter_speed_lever_arm[0], filter_speed_lever_arm[1], filter_speed_lever_arm[2]);

    inertial_device_->setSpeedMeasurementOffset(offset);
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the filter speed lever arm command.");
  }

  // (GQ7 only) Set the wheeled vehicle constraint
  if (inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_WHEELED_VEHICLE_CONSTRAINT))
  {
    MICROSTRAIN_INFO(node_, "Setting wheeled vehicle contraint enable to %d",
                     filter_enable_wheeled_vehicle_constraint_);
    inertial_device_->enableWheeledVehicleConstraint(filter_enable_wheeled_vehicle_constraint_);
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the wheeled vehicle constraint command.");
  }

  // (GQ7 only) Set the vertical gyro constraint
  if (inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_VERTICAL_GYRO_CONSTRAINT))
  {
    MICROSTRAIN_INFO(node_, "Setting vertical gyro contraint enable to %d", filter_enable_vertical_gyro_constraint_);
    inertial_device_->enableVerticalGyroConstraint(filter_enable_vertical_gyro_constraint_);
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the vertical gyro constraint command.");
  }

  // (GQ7 only) Set the GNSS antenna calibration settings
  if (inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_GNSS_ANTENNA_LEVER_ARM_CAL))
  {
    mscl::AntennaLeverArmCalConfiguration config;
    config.enabled = filter_enable_gnss_antenna_cal_;
    config.maxOffsetError = filter_gnss_antenna_cal_max_offset;

    MICROSTRAIN_INFO(node_, "Setting GNSS antenna calibration to: enable = %d, max_offset = %f",
                     filter_enable_gnss_antenna_cal_, filter_gnss_antenna_cal_max_offset);
    inertial_device_->setAntennaLeverArmCal(config);
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the GNSS antenna calibration command.");
  }

  // (GQ7 only) Set the filter initialization settings
  if (inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_INITIALIZATION_CONFIG))
  {
    mscl::FilterInitializationValues filter_config;

    // API Variable: autoInitialize
    filter_config.autoInitialize = filter_auto_init;

    // API Variable: initialValuesSource
    filter_config.initialValuesSource = static_cast<mscl::FilterInitialValuesSource>(filter_init_condition_src);

    // API Variable: autoHeadingAlignmentMethod
    filter_config.autoHeadingAlignmentMethod =
        static_cast<mscl::HeadingAlignmentMethod>(filter_auto_heading_alignment_selector);

    // API Variable: initialAttitude
    //  Note: Only heading value will be used if initialValueSource indicates pitch/roll will be determined
    //  automatically.
    filter_config.initialAttitude =
        mscl::EulerAngles(filter_init_attitude[0], filter_init_attitude[1], filter_init_attitude[2]);

    // API Variable: initialPosition
    filter_config.initialPosition =
        mscl::Position(filter_init_position[0], filter_init_position[1], filter_init_position[2],
                       static_cast<mscl::PositionVelocityReferenceFrame>(filter_init_reference_frame));

    // API Variable: initialVelocity
    filter_config.initialVelocity =
        mscl::GeometricVector(filter_init_velocity[0], filter_init_velocity[1], filter_init_velocity[2],
                              static_cast<mscl::PositionVelocityReferenceFrame>(filter_init_reference_frame));

    // API Variable: referenceFrame
    filter_config.referenceFrame = static_cast<mscl::PositionVelocityReferenceFrame>(filter_init_reference_frame);

    inertial_device_->setInitialFilterConfiguration(filter_config);
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the next-gen filter initialization command.");
  }

  // Configure the hardware odometer settings  
  if (inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_ODOMETER_SETTINGS))
  {
    mscl::OdometerConfiguration odom_config;
    odom_config.mode(enable_hardware_odometer_ ? odom_config.QUADRATURE : odom_config.DISABLED);
    odom_config.scaling(hardware_odometer_scaling);
    odom_config.uncertainty(hardware_odometer_uncertainty);
    inertial_device_->setOdometerConfig(odom_config);
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the odometer settings command");
  }

  // Enable the filter datastream
  inertial_device_->enableDataStream(mscl::MipTypes::DataClass::CLASS_ESTFILTER);
  return true;
}

bool MicrostrainConfig::configureSensor2vehicle(RosNodeType* node)
{
  int filter_sensor2vehicle_frame_selector;
  std::vector<double> filter_sensor2vehicle_frame_transformation_euler(3, 0.0);
  std::vector<double> filter_sensor2vehicle_frame_transformation_matrix(9, 0.0);
  std::vector<double> filter_sensor2vehicle_frame_transformation_quaternion(4, 0.0);
  get_param<int32_t>(node, "filter_sensor2vehicle_frame_selector", filter_sensor2vehicle_frame_selector, 0);
  get_param<std::vector<double>>(node, "filter_sensor2vehicle_frame_transformation_euler",
                                 filter_sensor2vehicle_frame_transformation_euler, DEFAULT_VECTOR);
  get_param<std::vector<double>>(node, "filter_sensor2vehicle_frame_transformation_matrix",
                                 filter_sensor2vehicle_frame_transformation_matrix, DEFAULT_MATRIX);
  get_param<std::vector<double>>(node, "filter_sensor2vehicle_frame_transformation_quaternion",
                                 filter_sensor2vehicle_frame_transformation_quaternion, DEFAULT_QUATERNION);

  // Euler Angles
  if (filter_sensor2vehicle_frame_selector == 1)
  {
    // Old style - set rotation (inverse of transformation)
    if (inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SENS_VEHIC_FRAME_ROTATION_EULER))
    {
      // Invert the angles for "rotation"
      mscl::EulerAngles angles(-filter_sensor2vehicle_frame_transformation_euler[0],
                               -filter_sensor2vehicle_frame_transformation_euler[1],
                               -filter_sensor2vehicle_frame_transformation_euler[2]);

      MICROSTRAIN_INFO(node_, "Setting sensor2vehicle frame rotation with euler angles [%f, %f, %f]",
                       -filter_sensor2vehicle_frame_transformation_euler[0],
                       -filter_sensor2vehicle_frame_transformation_euler[1],
                       -filter_sensor2vehicle_frame_transformation_euler[2]);
      inertial_device_->setSensorToVehicleRotation_eulerAngles(angles);
    }
    else if (inertial_device_->features().supportsCommand(
                 mscl::MipTypes::Command::CMD_EF_SENS_VEHIC_FRAME_TRANSFORM_EULER))
    {
      mscl::EulerAngles angles(filter_sensor2vehicle_frame_transformation_euler[0],
                               filter_sensor2vehicle_frame_transformation_euler[1],
                               filter_sensor2vehicle_frame_transformation_euler[2]);

      MICROSTRAIN_INFO(node_, "Setting sensor2vehicle frame transformation with euler angles [%f, %f, %f]",
                       filter_sensor2vehicle_frame_transformation_euler[0],
                       filter_sensor2vehicle_frame_transformation_euler[1],
                       filter_sensor2vehicle_frame_transformation_euler[2]);
      inertial_device_->setSensorToVehicleTransform_eulerAngles(angles);
    }
    else
    {
      MICROSTRAIN_ERROR(node_, "**Failed to set sensor2vehicle frame transformation with euler angles!");
      return false;
    }
  }
  // Matrix
  else if (filter_sensor2vehicle_frame_selector == 2)
  {
    // Old style - set rotation (inverse of transformation)
    if (inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SENS_VEHIC_FRAME_ROTATION_DCM))
    {
      // Transpose the matrix for "rotation"
      mscl::Matrix_3x3 dcm(
          filter_sensor2vehicle_frame_transformation_matrix[0], filter_sensor2vehicle_frame_transformation_matrix[3],
          filter_sensor2vehicle_frame_transformation_matrix[6], filter_sensor2vehicle_frame_transformation_matrix[1],
          filter_sensor2vehicle_frame_transformation_matrix[4], filter_sensor2vehicle_frame_transformation_matrix[7],
          filter_sensor2vehicle_frame_transformation_matrix[2], filter_sensor2vehicle_frame_transformation_matrix[5],
          filter_sensor2vehicle_frame_transformation_matrix[8]);

      MICROSTRAIN_INFO(node_, "Setting sensor2vehicle frame rotation with a matrix");
      inertial_device_->setSensorToVehicleRotation_matrix(dcm);
    }
    else if (inertial_device_->features().supportsCommand(
                 mscl::MipTypes::Command::CMD_EF_SENS_VEHIC_FRAME_TRANSFORM_DCM))
    {
      mscl::Matrix_3x3 dcm(
          filter_sensor2vehicle_frame_transformation_matrix[0], filter_sensor2vehicle_frame_transformation_matrix[1],
          filter_sensor2vehicle_frame_transformation_matrix[2], filter_sensor2vehicle_frame_transformation_matrix[3],
          filter_sensor2vehicle_frame_transformation_matrix[4], filter_sensor2vehicle_frame_transformation_matrix[5],
          filter_sensor2vehicle_frame_transformation_matrix[6], filter_sensor2vehicle_frame_transformation_matrix[7],
          filter_sensor2vehicle_frame_transformation_matrix[8]);

      MICROSTRAIN_INFO(node_, "Setting sensor2vehicle frame transformation with a matrix");
      inertial_device_->setSensorToVehicleTransform_matrix(dcm);
    }
    else
    {
      MICROSTRAIN_ERROR(node_, "**Failed to set sensor2vehicle frame transformation with a matrix!");
      return false;
    }
  }
  // Quaternion
  else if (filter_sensor2vehicle_frame_selector == 3)
  {
    // Old style - set rotation (inverse of transformation)
    if (inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SENS_VEHIC_FRAME_ROTATION_QUAT))
    {
      // Invert the quaternion for "rotation" (note: device uses aerospace quaternion definition [w, -i, -j, -k])
      mscl::Quaternion quat(filter_sensor2vehicle_frame_transformation_quaternion[3],
                            -filter_sensor2vehicle_frame_transformation_quaternion[0],
                            -filter_sensor2vehicle_frame_transformation_quaternion[1],
                            -filter_sensor2vehicle_frame_transformation_quaternion[2]);

      MICROSTRAIN_INFO(node_, "Setting sensor2vehicle frame rotation with quaternion [%f %f %f %f]",
                       -filter_sensor2vehicle_frame_transformation_quaternion[0],
                       -filter_sensor2vehicle_frame_transformation_quaternion[1],
                       -filter_sensor2vehicle_frame_transformation_quaternion[2],
                       filter_sensor2vehicle_frame_transformation_quaternion[3]);
      inertial_device_->setSensorToVehicleRotation_quaternion(quat);
    }
    else if (inertial_device_->features().supportsCommand(
                 mscl::MipTypes::Command::CMD_EF_SENS_VEHIC_FRAME_TRANSFORM_QUAT))
    {
      // No inversion for transformation (note: device uses aerospace quaternion definition [w, i, j, k])
      mscl::Quaternion quat(filter_sensor2vehicle_frame_transformation_quaternion[3],
                            filter_sensor2vehicle_frame_transformation_quaternion[0],
                            filter_sensor2vehicle_frame_transformation_quaternion[1],
                            filter_sensor2vehicle_frame_transformation_quaternion[2]);

      MICROSTRAIN_INFO(node_, "Setting sensor2vehicle frame transformation with quaternion [%f %f %f %f]",
                       filter_sensor2vehicle_frame_transformation_quaternion[0],
                       filter_sensor2vehicle_frame_transformation_quaternion[1],
                       filter_sensor2vehicle_frame_transformation_quaternion[2],
                       filter_sensor2vehicle_frame_transformation_quaternion[3]);
      inertial_device_->setSensorToVehicleTransform_quaternion(quat);
    }
    else
    {
      MICROSTRAIN_ERROR(node_, "**Failed to set sensor2vehicle frame transformation with quaternion!");
      return false;
    }
  }
  return true;
}

}  // namespace microstrain
