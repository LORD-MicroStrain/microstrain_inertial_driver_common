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
#include "microstrain_config.h"


namespace Microstrain
{

MicrostrainConfig::MicrostrainConfig(RosNodeType* node) : m_node(node)
{}

bool MicrostrainConfig::configure(RosNodeType* node)
{
  //Initialize some default and static config
  m_imu_frame_id = "sensor";
  m_gnss_frame_id[GNSS1_ID] = "gnss1_antenna_wgs84_ned";
  m_gnss_frame_id[GNSS2_ID] = "gnss2_antenna_wgs84_ned";
  m_filter_frame_id = "sensor_wgs84_ned";
  m_filter_child_frame_id = "sensor";
  m_t_ned2enu = tf2::Matrix3x3(0,1,0,1,0,0,0,0,-1);

  ///
  /// Generic configuration used by the rest of the driver
  ///

  //Device
  get_param<bool>(node, "use_device_timestamp",  m_use_device_timestamp, false);
  get_param<bool>(node, "use_enu_frame",         m_use_enu_frame, false);

  //If using ENU frame, reflect in the device frame id
  if (m_use_enu_frame)
  {
    m_gnss_frame_id[GNSS1_ID] = "gnss1_antenna_wgs84_enu";
    m_gnss_frame_id[GNSS2_ID] = "gnss2_antenna_wgs84_enu";
    m_filter_frame_id         = "sensor_wgs84_enu";
  }

  //IMU
  get_param<bool>(node, "publish_imu",           m_publish_imu, true);
  get_param<bool>(node, "publish_gps_corr",      m_publish_gps_corr, false);
  get_param<int32_t>(node, "imu_data_rate",         m_imu_data_rate, 10);
  get_param<std::vector<double>>(node, "imu_orientation_cov",   m_imu_orientation_cov, default_matrix);
  get_param<std::vector<double>>(node, "imu_linear_cov",        m_imu_linear_cov,      default_matrix);
  get_param<std::vector<double>>(node, "imu_angular_cov",       m_imu_angular_cov,     default_matrix);
  get_param<std::string>(node, "imu_frame_id",          m_imu_frame_id, m_imu_frame_id);

  //GNSS 1/2
  get_param<bool>(node, "publish_gnss1",         m_publish_gnss[GNSS1_ID], false);
  get_param<bool>(node, "publish_gnss2",         m_publish_gnss[GNSS2_ID], false);
  get_param<int32_t>(node, "gnss1_data_rate",       m_gnss_data_rate[GNSS1_ID], 1);
  get_param<int32_t>(node, "gnss2_data_rate",       m_gnss_data_rate[GNSS2_ID], 1);
  get_param<std::vector<double>>(node, "gnss1_antenna_offset",  m_gnss_antenna_offset[GNSS1_ID],  default_vector);
  get_param<std::vector<double>>(node, "gnss2_antenna_offset",  m_gnss_antenna_offset[GNSS2_ID],  default_vector);
  get_param<std::string>(node, "gnss1_frame_id",        m_gnss_frame_id[GNSS1_ID], m_gnss_frame_id[GNSS1_ID]);
  get_param<std::string>(node, "gnss2_frame_id",        m_gnss_frame_id[GNSS2_ID], m_gnss_frame_id[GNSS2_ID]);

  //FILTER
  get_param<bool>(node, "publish_filter",             m_publish_filter, false);
  get_param<int32_t>(node, "filter_data_rate",           m_filter_data_rate, 10);
  get_param<std::string>(node, "filter_frame_id",            m_filter_frame_id, m_filter_frame_id);
  get_param<bool>(node, "publish_relative_position",  m_publish_filter_relative_pos, false);
  get_param<double>(node, "gps_leap_seconds",                m_gps_leap_seconds, 18.0);
  get_param<bool>(node, "filter_angular_zupt",             m_angular_zupt, false);
  get_param<bool>(node, "filter_velocity_zupt",            m_velocity_zupt, false);
  get_param<bool>(node, "filter_enable_gnss_heading_aiding",        m_filter_enable_gnss_heading_aiding, true);
  get_param<bool>(node, "filter_enable_gnss_pos_vel_aiding",        m_filter_enable_gnss_pos_vel_aiding, true);
  get_param<bool>(node, "filter_enable_altimeter_aiding",           m_filter_enable_altimeter_aiding, false);
  get_param<bool>(node, "filter_enable_odometer_aiding",            m_filter_enable_odometer_aiding, false);
  get_param<bool>(node, "filter_enable_magnetometer_aiding",        m_filter_enable_magnetometer_aiding, false);
  get_param<bool>(node, "filter_enable_external_heading_aiding",    m_filter_enable_external_heading_aiding, false);
  get_param<bool>(node, "filter_enable_external_gps_time_update",   m_filter_enable_external_gps_time_update, false);
  get_param<bool>(node, "filter_enable_wheeled_vehicle_constraint", m_filter_enable_wheeled_vehicle_constraint, false);
  get_param<bool>(node, "filter_enable_vertical_gyro_constraint",   m_filter_enable_vertical_gyro_constraint, false);
  get_param<bool>(node, "filter_enable_gnss_antenna_cal",           m_filter_enable_gnss_antenna_cal, false);
  get_param<std::string>(node, "filter_velocity_zupt_topic",      m_velocity_zupt_topic, std::string("/moving_vel"));
  get_param<std::string>(node, "filter_angular_zupt_topic",       m_angular_zupt_topic, std::string("/moving_ang"));
  get_param<std::string>(node, "filter_external_gps_time_topic",  m_external_gps_time_topic, std::string("/external_gps_time"));
  
  //Raw data file save
  get_param<bool>(node, "raw_file_enable",               m_raw_file_enable, false);
  get_param<bool>(node, "raw_file_include_support_data", m_raw_file_include_support_data, false);

  MICROSTRAIN_INFO(m_node, "Using MSCL Version: %s", mscl::MSCL_VERSION.str().c_str());

  //Connect to the device and set it up if we were asked to
  bool device_setup;
  get_param<bool>(node, "device_setup",          device_setup, false);
  
  if (!connect_device(node))
    return false;

  if (device_setup)
  {
    if (!setup_device(node))
      return false;
  }

  if (!setup_raw_file(node))
    return false;

  return true;
}

bool MicrostrainConfig::connect_device(RosNodeType* node)
{
  //Read the config required for only this section
  std::string port;
  int32_t baudrate;
  get_param<std::string>(node, "port",     port,      "/dev/ttyACM1");
  get_param<int32_t>(node,    "baudrate", baudrate,  115200);

  try
  {
    //
    //Initialize the serial interface to the device and create the inertial device object
    //
    MICROSTRAIN_INFO(m_node, "Attempting to open serial port <%s> at <%d> \n", port.c_str(), baudrate);

    mscl::Connection connection = mscl::Connection::Serial(realpath(port.c_str(), 0), (uint32_t)baudrate);
    m_inertial_device           = std::unique_ptr<mscl::InertialNode>(new mscl::InertialNode(connection));

    //Print the device info
    MICROSTRAIN_INFO(m_node, "Model Name:    %s\n", m_inertial_device->modelName().c_str());
    MICROSTRAIN_INFO(m_node, "Serial Number: %s\n", m_inertial_device->serialNumber().c_str());

    //Get supported features
    m_supports_gnss1  = m_inertial_device->features().supportsCategory(mscl::MipTypes::DataClass::CLASS_GNSS) | m_inertial_device->features().supportsCategory(mscl::MipTypes::DataClass::CLASS_GNSS1);
    m_supports_gnss2  = m_inertial_device->features().supportsCategory(mscl::MipTypes::DataClass::CLASS_GNSS2);
    m_supports_rtk    = m_inertial_device->features().supportsCategory(mscl::MipTypes::DataClass::CLASS_GNSS3);
    m_supports_filter = m_inertial_device->features().supportsCategory(mscl::MipTypes::DataClass::CLASS_ESTFILTER);
    m_supports_imu    = m_inertial_device->features().supportsCategory(mscl::MipTypes::DataClass::CLASS_AHRS_IMU);
  }
  catch (mscl::Error_Connection& e)
  {
    // TODO: Log more information here
    MICROSTRAIN_ERROR(m_node, "Device Disconnected");
    return false;
  }
  catch (mscl::Error& e)
  {
    MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    return false;
  }
  return true;
}

bool MicrostrainConfig::setup_device(RosNodeType* node)
{
  //Read the config used by this section
  bool save_settings;
  bool gpio_config;
  bool rtk_dongle_enable;
  bool filter_reset_after_config;
  get_param<bool>(node, "save_settings",         save_settings, true);
  get_param<bool>(node, "gpio_config",     gpio_config, false);
  get_param<bool>(node, "rtk_dongle_enable",  rtk_dongle_enable,  false);
  get_param<bool>(node, "filter_reset_after_config",  filter_reset_after_config, true);

  //Put into idle mode
  MICROSTRAIN_INFO(m_node, "Setting to Idle: Stopping data streams and/or waking from sleep");
  m_inertial_device->setToIdle();

  //GPIO config
  if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_GPIO_CONFIGURATION) && gpio_config)
  {
    if (!configure_gpio(node))
      return false;
  }

  //IMU Setup
  if(m_publish_imu && m_supports_imu)
  {
    if (!configure_imu(node))
      return false;
  }


  //GNSS1 setup
  if(m_publish_gnss[GNSS1_ID] && m_supports_gnss1)
  {
    if (!configure_gnss(node, GNSS1_ID))
      return false;
  }

  
  //GNSS2 setup
  if(m_publish_gnss[GNSS2_ID] && m_supports_gnss2)
  {
    if (!configure_gnss(node, GNSS2_ID))
      return false;
  }

  // RTK Dongle
  if(rtk_dongle_enable && m_supports_rtk)
  {
    if (!configure_rtk(node))
      return false;
  }

  //Filter setup
  if(m_publish_filter && m_supports_filter)
  {
    if (!configure_filter(node))
      return false;
  }

  //Sensor2Vehicle setup
  if (!configure_sensor2vehicle(node))
    return false;

  //Support channel setup
  if(m_raw_file_enable && m_raw_file_include_support_data)
  {
    if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_FACTORY_STREAMING))
    {
      MICROSTRAIN_INFO(m_node, "Enabling factory support channels");
      m_inertial_device->setFactoryStreamingChannels(mscl::InertialTypes::FACTORY_STREAMING_ADDITIVE);
    }
    else 
    {
      MICROSTRAIN_ERROR(m_node, "**The device does not support the factory streaming channels setup command!");          
    }
  }


  //Save the settings to the device, if enabled
  if(save_settings)
  {
    MICROSTRAIN_INFO(m_node, "Saving the launch file configuration settings to the device");
    m_inertial_device->saveSettingsAsStartup();
  }


  //Reset the filter, if enabled
  if(filter_reset_after_config && m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_RESET_FILTER))
  {
    MICROSTRAIN_INFO(m_node, "Resetting the filter after the configuration is complete.");
    m_inertial_device->resetFilter();
  }
  else
  {
    MICROSTRAIN_INFO(m_node, "Note: The filter was not reset after configuration.");          
  }

  //Resume the device
  m_inertial_device->resume();
  return true;
}

bool MicrostrainConfig::setup_raw_file(RosNodeType* node)
{
  //Open raw data file, if enabled, and configure the device for raw data output
  std::string raw_file_directory;
  get_param<std::string>(node, "raw_file_directory",            raw_file_directory, std::string("."));

  if(m_raw_file_enable)
  {
    time_t raw_time;
    struct tm * curr_time;
    char curr_time_buffer[100];

    //Get the current time
    time(&raw_time);
    curr_time = localtime(&raw_time);
    strftime(curr_time_buffer, sizeof(curr_time_buffer),"%y_%m_%d_%H_%M_%S", curr_time);
    
    std::string time_string(curr_time_buffer);
    
    std::string filename = raw_file_directory + std::string("/") + 
                            m_inertial_device->modelName() + std::string("_") + m_inertial_device->serialNumber() + 
                            std::string("_") + time_string + std::string(".bin");

    m_raw_file.open(filename, std::ios::out | std::ios::binary | std::ios::trunc);
  
    if(!m_raw_file.is_open())
    {
      MICROSTRAIN_ERROR(m_node, "ERROR opening raw binary datafile at %s", filename.c_str());
      return false;
    }
    else
    {
      MICROSTRAIN_INFO(m_node, "Raw binary datafile opened at %s", filename.c_str()); 
    }

    m_inertial_device->connection().debugMode(true);
  }
  return true;
}

bool MicrostrainConfig::configure_gpio(RosNodeType* node)
{
  //Read the config required only by this section
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
  get_param<int32_t>(node, "gpio1_feature",   gpio1_feature,  0);
  get_param<int32_t>(node, "gpio1_behavior",  gpio1_behavior, 0);
  get_param<int32_t>(node, "gpio1_pin_mode",  gpio1_pin_mode, 0);
  get_param<int32_t>(node, "gpio2_feature",   gpio2_feature,  0);
  get_param<int32_t>(node, "gpio2_behavior",  gpio2_behavior, 0);
  get_param<int32_t>(node, "gpio2_pin_mode",  gpio2_pin_mode, 0);
  get_param<int32_t>(node, "gpio3_feature",   gpio3_feature,  0);
  get_param<int32_t>(node, "gpio3_behavior",  gpio3_behavior, 0);
  get_param<int32_t>(node, "gpio3_pin_mode",  gpio3_pin_mode, 0);
  get_param<int32_t>(node, "gpio4_feature",   gpio4_feature,  0);
  get_param<int32_t>(node, "gpio4_behavior",  gpio4_behavior, 0);
  get_param<int32_t>(node, "gpio4_pin_mode",  gpio4_pin_mode, 0);

  try
  {
    mscl::GpioConfiguration gpioConfig;
    
    gpioConfig.pin = 1;
    gpioConfig.feature = static_cast<mscl::GpioConfiguration::Feature>(gpio1_feature);
    gpioConfig.behavior = gpio1_behavior;
    gpioConfig.pinMode.value(gpio1_pin_mode);
    m_inertial_device->setGpioConfig(gpioConfig);

    MICROSTRAIN_INFO(m_node, "Configuring GPIO1 to feature: %i, behavior: %i, pinMode: %i", gpio1_feature, gpio1_behavior, gpio1_pin_mode);

    gpioConfig.pin = 2;
    gpioConfig.feature = static_cast<mscl::GpioConfiguration::Feature>(gpio2_feature);
    gpioConfig.behavior = gpio2_behavior;
    gpioConfig.pinMode.value(gpio4_pin_mode);
    m_inertial_device->setGpioConfig(gpioConfig);

    MICROSTRAIN_INFO(m_node, "Configuring GPIO2 to feature: %i, behavior: %i, pinMode: %i", gpio2_feature, gpio2_behavior, gpio2_pin_mode);

    gpioConfig.pin = 3;
    gpioConfig.feature = static_cast<mscl::GpioConfiguration::Feature>(gpio3_feature);
    gpioConfig.behavior = gpio3_behavior;
    gpioConfig.pinMode.value(gpio4_pin_mode);
    m_inertial_device->setGpioConfig(gpioConfig);

    MICROSTRAIN_INFO(m_node, "Configuring GPIO3 to feature: %i, behavior: %i, pinMode: %i", gpio3_feature, gpio3_behavior, gpio3_pin_mode);

    gpioConfig.pin = 4;
    gpioConfig.feature = static_cast<mscl::GpioConfiguration::Feature>(gpio4_feature);
    gpioConfig.behavior = gpio4_behavior;
    gpioConfig.pinMode.value(gpio4_pin_mode);
    m_inertial_device->setGpioConfig(gpioConfig);

    MICROSTRAIN_INFO(m_node, "Configuring GPIO4 to feature: %i, behavior: %i, pinMode: %i", gpio4_feature, gpio4_behavior, gpio4_pin_mode);
  }
  catch(mscl::Error& e)
  {
    MICROSTRAIN_ERROR(m_node, "GPIO Config Error: %s", e.what());
    return false;
  }

  return true;
}

bool MicrostrainConfig::configure_imu(RosNodeType* node)
{
  //Read the config required only by this section
  int32_t declination_source;
  double declination;
  get_param<int32_t>(node, "filter_declination_source",       declination_source, 2);
  get_param<double>(node,  "filter_declination",              declination,        0.23);

  mscl::SampleRate imu_rate = mscl::SampleRate::Hertz(m_imu_data_rate);

  MICROSTRAIN_INFO(m_node, "Setting IMU data to stream at %d hz", m_imu_data_rate);

  mscl::MipTypes::MipChannelFields ahrsChannels{
      mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_SCALED_ACCEL_VEC,
      mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_SCALED_GYRO_VEC,
      mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_ORIENTATION_QUATERNION,
      mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_SCALED_MAG_VEC,
      mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_GPS_CORRELATION_TIMESTAMP
      };

  mscl::MipChannels supportedChannels;
  for(mscl::MipTypes::ChannelField channel : m_inertial_device->features().supportedChannelFields(mscl::MipTypes::DataClass::CLASS_AHRS_IMU))
  {
    if(std::find(ahrsChannels.begin(), ahrsChannels.end(), channel) != ahrsChannels.end())
    {
      supportedChannels.push_back(mscl::MipChannel(channel, imu_rate));
    }
  }

  m_inertial_device->setActiveChannelFields(mscl::MipTypes::DataClass::CLASS_AHRS_IMU, supportedChannels);

  if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_DECLINATION_SRC))
  {
    MICROSTRAIN_INFO(m_node, "Setting Declination Source");
    m_inertial_device->setDeclinationSource(mscl::GeographicSourceOptions(static_cast<mscl::InertialTypes::GeographicSourceOption>((uint8_t)declination_source), declination));
  }
  else
  {
    MICROSTRAIN_INFO(m_node, "Note: Device does not support the declination source command.");
  }

  m_inertial_device->enableDataStream(mscl::MipTypes::DataClass::CLASS_AHRS_IMU);
  return true;
}

bool MicrostrainConfig::configure_gnss(RosNodeType* node, uint8_t gnss_id)
{
  mscl::SampleRate gnss1_rate = mscl::SampleRate::Hertz(m_gnss_data_rate[gnss_id]);

  MICROSTRAIN_INFO(m_node, "Setting GNSS%d data to stream at %d hz", gnss_id, m_gnss_data_rate[gnss_id]);

  mscl::MipTypes::MipChannelFields gnssChannels{
      mscl::MipTypes::ChannelField::CH_FIELD_GNSS_LLH_POSITION,
      mscl::MipTypes::ChannelField::CH_FIELD_GNSS_NED_VELOCITY,
      mscl::MipTypes::ChannelField::CH_FIELD_GNSS_GPS_TIME};

  mscl::MipTypes::DataClass gnss_data_class = mscl::MipTypes::DataClass::CLASS_GNSS;

  if(m_inertial_device->features().supportsCategory(mscl::MipTypes::DataClass::CLASS_GNSS1) && gnss_id == GNSS1_ID)
  {
    gnss_data_class = mscl::MipTypes::DataClass::CLASS_GNSS1;
    
    gnssChannels.clear();
    gnssChannels.push_back(mscl::MipTypes::ChannelField::CH_FIELD_GNSS_1_LLH_POSITION);
    gnssChannels.push_back(mscl::MipTypes::ChannelField::CH_FIELD_GNSS_1_NED_VELOCITY);
    gnssChannels.push_back(mscl::MipTypes::ChannelField::CH_FIELD_GNSS_1_GPS_TIME);
  }
  else if (m_inertial_device->features().supportsCategory(mscl::MipTypes::DataClass::CLASS_GNSS2) && gnss_id == GNSS2_ID)
  {
    gnss_data_class = mscl::MipTypes::DataClass::CLASS_GNSS2;
    
    gnssChannels.clear();
    gnssChannels.push_back(mscl::MipTypes::ChannelField::CH_FIELD_GNSS_2_LLH_POSITION);
    gnssChannels.push_back(mscl::MipTypes::ChannelField::CH_FIELD_GNSS_2_NED_VELOCITY);
    gnssChannels.push_back(mscl::MipTypes::ChannelField::CH_FIELD_GNSS_2_GPS_TIME);
  }

  mscl::MipChannels supportedChannels;
  for (mscl::MipTypes::ChannelField channel : m_inertial_device->features().supportedChannelFields(gnss_data_class))
  {
    if (std::find(gnssChannels.begin(), gnssChannels.end(), channel) != gnssChannels.end())
    {
      supportedChannels.push_back(mscl::MipChannel(channel, gnss1_rate));
    }
  }

  //set the GNSS channel fields
  m_inertial_device->setActiveChannelFields(gnss_data_class, supportedChannels);

  //Set the antenna offset, if supported (needs to process 2 different ways for old devices vs. new for GNSS1)
  mscl::PositionOffset antenna_offset(m_gnss_antenna_offset[gnss_id][0], m_gnss_antenna_offset[gnss_id][1], m_gnss_antenna_offset[gnss_id][2]);

  if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_ANTENNA_OFFSET))
  {
    MICROSTRAIN_INFO(m_node, "Setting GNSS%d antenna offset to [%f, %f, %f]", gnss_id, antenna_offset.x(), antenna_offset.y(), antenna_offset.z());
    m_inertial_device->setAntennaOffset(antenna_offset);
  }
  else if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_MULTI_ANTENNA_OFFSET))
  {
    MICROSTRAIN_INFO(m_node, "Setting GNSS%d antenna offset to [%f, %f, %f]", gnss_id, antenna_offset.x(), antenna_offset.y(), antenna_offset.z());
    m_inertial_device->setMultiAntennaOffset(1, antenna_offset);
  }
  else
  {
    MICROSTRAIN_ERROR(m_node, "Could not set GNSS%d antenna offset!", gnss_id);
    return false;
  }

  //Enable publishing aiding status messages
  if (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_AIDING_MEASUREMENT_ENABLE))
  {
    m_publish_gnss_aiding_status[gnss_id] = true;
  }

  m_inertial_device->enableDataStream(gnss_data_class);
  return true;
}

bool MicrostrainConfig::configure_rtk(RosNodeType* node)
{
  mscl::SampleRate gnss3_rate = mscl::SampleRate::Hertz(1);

  MICROSTRAIN_INFO(m_node, "Setting RTK data to stream at 1 hz");

  mscl::MipTypes::MipChannelFields gnssChannels{
      mscl::MipTypes::ChannelField::CH_FIELD_GNSS_3_RTK_CORRECTIONS_STATUS};

  mscl::MipChannels supportedChannels;
  for (mscl::MipTypes::ChannelField channel : m_inertial_device->features().supportedChannelFields(mscl::MipTypes::DataClass::CLASS_GNSS3))
  {
    if (std::find(gnssChannels.begin(), gnssChannels.end(), channel) != gnssChannels.end())
    {
      supportedChannels.push_back(mscl::MipChannel(channel, gnss3_rate));
    }
  }

  //set the GNSS channel fields
  m_inertial_device->setActiveChannelFields(mscl::MipTypes::DataClass::CLASS_GNSS3, supportedChannels);

  if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_GNSS_RTK_CONFIG))
  {
    MICROSTRAIN_INFO(m_node, "Setting RTK dongle enable to 1");
    m_inertial_device->enableRtk(true);
    m_publish_rtk = true;
  }
  else
  {
    MICROSTRAIN_INFO(m_node, "Note: Device does not support the RTK dongle config command");          
  }

  m_inertial_device->enableDataStream(mscl::MipTypes::DataClass::CLASS_GNSS3);
  return true;
}

bool MicrostrainConfig::configure_filter(RosNodeType* node)
{
  //Read some generic filter info
  int   heading_source;
  float initial_heading;
  bool  filter_auto_init  = true;
  int   dynamics_mode;
  get_param<int32_t>(node, "filter_heading_source",           heading_source, 0x1);
  get_param<float>(node, "filter_initial_heading",          initial_heading, (float)0.0);
  get_param<bool>(node, "filter_auto_init",           filter_auto_init, true);
  get_param<int32_t>(node, "filter_dynamics_mode",            dynamics_mode, 1);

  //Read some QG7 specific filter options
  int filter_adaptive_level;
  int filter_adaptive_time_limit_ms;
  bool filter_enable_gnss_pos_vel_aiding;
  bool filter_enable_altimeter_aiding;
  bool filter_enable_odometer_aiding;
  bool filter_enable_magnetometer_aiding;
  bool filter_enable_external_heading_aiding;
  bool filter_enable_external_gps_time_update;
  int filter_enable_acceleration_constraint; 
  int filter_enable_velocity_constraint;
  int filter_enable_angular_constraint;
  int filter_init_condition_src;
  int filter_auto_heading_alignment_selector;
  int filter_init_reference_frame;
  std::vector<double> filter_init_position(3, 0.0);
  std::vector<double> filter_init_velocity(3, 0.0);
  std::vector<double> filter_init_attitude(3, 0.0);
  int filter_relative_position_frame;
  std::vector<double> filter_relative_position_ref(3, 0.0);
  std::vector<double> filter_speed_lever_arm(3, 0.0);
  bool filter_enable_wheeled_vehicle_constraint;
  bool filter_enable_vertical_gyro_constraint;
  bool filter_enable_gnss_antenna_cal;
  double filter_gnss_antenna_cal_max_offset;
  int filter_pps_source;
  get_param<int32_t>(node, "filter_adaptive_level" ,                   filter_adaptive_level, 2);
  get_param<int32_t>(node, "filter_adaptive_time_limit_ms" ,           filter_adaptive_time_limit_ms, 15000);
  get_param<int32_t>(node, "filter_enable_acceleration_constraint",    filter_enable_acceleration_constraint, 0);
  get_param<int32_t>(node, "filter_enable_velocity_constraint",        filter_enable_velocity_constraint, 0);
  get_param<int32_t>(node, "filter_enable_angular_constraint",         filter_enable_angular_constraint, 0);
  get_param<int32_t>(node, "filter_init_condition_src",                filter_init_condition_src, 0);
  get_param<int32_t>(node, "filter_auto_heading_alignment_selector",   filter_auto_heading_alignment_selector, 0);
  get_param<int32_t>(node, "filter_init_reference_frame",              filter_init_reference_frame, 2);
  get_param<std::vector<double>>(node, "filter_init_position",                     filter_init_position, default_vector);   
  get_param<std::vector<double>>(node, "filter_init_velocity",                     filter_init_velocity, default_vector);
  get_param<std::vector<double>>(node, "filter_init_attitude",                     filter_init_attitude, default_vector);
  get_param<int32_t>(node, "filter_relative_position_frame",           filter_relative_position_frame, 2);
  get_param<std::vector<double>>(node, "filter_relative_position_ref",             filter_relative_position_ref, default_vector);   
  get_param<std::vector<double>>(node, "filter_speed_lever_arm",                   filter_speed_lever_arm, default_vector);   
  get_param<double>(node, "filter_gnss_antenna_cal_max_offset",       filter_gnss_antenna_cal_max_offset, 0.1);   
  get_param<int32_t>(node, "filter_pps_source",               filter_pps_source, 1);
  

  mscl::SampleRate filter_rate = mscl::SampleRate::Hertz(m_filter_data_rate);

  MICROSTRAIN_INFO(m_node, "Setting Filter data to stream at %d hz", m_filter_data_rate);

  mscl::MipTypes::MipChannelFields navChannels{
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
      mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_NED_RELATIVE_POS};

      if(filter_enable_gnss_pos_vel_aiding)
        navChannels.push_back(mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_POSITION_AIDING_STATUS);
      if(m_filter_enable_gnss_heading_aiding)
        navChannels.push_back(mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_GNSS_DUAL_ANTENNA_STATUS);
  
  mscl::MipChannels supportedChannels;
  for(mscl::MipTypes::ChannelField channel : m_inertial_device->features().supportedChannelFields(mscl::MipTypes::DataClass::CLASS_ESTFILTER))
  {
    if(std::find(navChannels.begin(), navChannels.end(), channel) != navChannels.end())
    {
      supportedChannels.push_back(mscl::MipChannel(channel, filter_rate));
    }
  }

  m_inertial_device->setActiveChannelFields(mscl::MipTypes::DataClass::CLASS_ESTFILTER, supportedChannels);

  //set dynamics mode
  if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_VEHIC_DYNAMICS_MODE))
  {
    mscl::VehicleModeTypes modes = m_inertial_device->features().supportedVehicleModeTypes();
    if (std::find(modes.begin(), modes.end(), static_cast<mscl::InertialTypes::VehicleModeType>(dynamics_mode)) != modes.end())
    {
      MICROSTRAIN_INFO(m_node, "Setting dynamics mode to %#04X", static_cast<mscl::InertialTypes::VehicleModeType>(dynamics_mode));
      m_inertial_device->setVehicleDynamicsMode(static_cast<mscl::InertialTypes::VehicleModeType>(dynamics_mode));
    }
  }
  else
  {
    MICROSTRAIN_INFO(m_node, "Note: The device does not support the vehicle dynamics mode command.");          
  }
  

  //Set PPS source
  if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_PPS_SOURCE))
  {
    mscl::PpsSourceOptions sources = m_inertial_device->features().supportedPpsSourceOptions();
    if (std::find(sources.begin(), sources.end(), static_cast<mscl::InertialTypes::PpsSource>(filter_pps_source)) != sources.end())
    {
      MICROSTRAIN_INFO(m_node, "Setting PPS source to %#04X", static_cast<mscl::InertialTypes::PpsSource>(filter_pps_source));
      m_inertial_device->setPpsSource(static_cast<mscl::InertialTypes::PpsSource>(filter_pps_source));
    }
  }
  else
  {
    MICROSTRAIN_INFO(m_node, "Note: The device does not support the PPS source command.");          
  }

  //Set heading Source
  if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_HEADING_UPDATE_CTRL))
  {
    for(mscl::HeadingUpdateOptions headingSources : m_inertial_device->features().supportedHeadingUpdateOptions())
    {
      if(headingSources.AsOptionId() == static_cast<mscl::InertialTypes::HeadingUpdateEnableOption>(heading_source))
      {
        MICROSTRAIN_INFO(m_node, "Setting heading source to %#04X", heading_source);
        m_inertial_device->setHeadingUpdateControl(mscl::HeadingUpdateOptions(static_cast<mscl::InertialTypes::HeadingUpdateEnableOption>(heading_source)));
        break;
      }
    }

    //Set the initial heading
    if((heading_source == 0) && (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_INIT_HEADING)))
    {
      MICROSTRAIN_INFO(m_node, "Setting initial heading to %f", initial_heading);
      m_inertial_device->setInitialHeading(initial_heading);
    }
  }
  else
  {
    MICROSTRAIN_INFO(m_node, "Note: The device does not support the heading source command.");          
  }


  //Set the filter autoinitialization, if suppored
  if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_AUTO_INIT_CTRL))
  {
    MICROSTRAIN_INFO(m_node, "Setting autoinitialization to %d", filter_auto_init);
    m_inertial_device->setAutoInitialization(filter_auto_init);
  }
  else
  {
    MICROSTRAIN_INFO(m_node, "Note: The device does not support the filter autoinitialization command.");          
  }


  //(GQ7 and GX5-45 only) Set the filter adaptive settings
  if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_ADAPTIVE_FILTER_OPTIONS))
  {
    MICROSTRAIN_INFO(m_node, "Setting autoadaptive options to: level = %d, time_limit = %d", filter_adaptive_level, filter_adaptive_time_limit_ms);
    mscl::AutoAdaptiveFilterOptions options(static_cast<mscl::InertialTypes::AutoAdaptiveFilteringLevel>(filter_adaptive_level), (uint16_t)filter_adaptive_time_limit_ms);

    m_inertial_device->setAdaptiveFilterOptions(options);
  }
  else
  {
    MICROSTRAIN_INFO(m_node, "Note: The device does not support the filte adaptive settings command.");          
  }


  //(GQ7 only) Set the filter aiding settings
  if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_AIDING_MEASUREMENT_ENABLE))
  {
    MICROSTRAIN_INFO(m_node, "Filter aiding set to: pos/vel = %d, gnss heading = %d, altimeter = %d, odometer = %d, magnetometer = %d, external heading = %d", 
              m_filter_enable_gnss_heading_aiding, m_filter_enable_gnss_heading_aiding, filter_enable_altimeter_aiding, filter_enable_odometer_aiding,
              filter_enable_magnetometer_aiding, filter_enable_external_heading_aiding);

    m_inertial_device->enableDisableAidingMeasurement(mscl::InertialTypes::AidingMeasurementSource::GNSS_POS_VEL_AIDING,     filter_enable_gnss_pos_vel_aiding);
    m_inertial_device->enableDisableAidingMeasurement(mscl::InertialTypes::AidingMeasurementSource::GNSS_HEADING_AIDING,     m_filter_enable_gnss_heading_aiding);
    m_inertial_device->enableDisableAidingMeasurement(mscl::InertialTypes::AidingMeasurementSource::ALTIMETER_AIDING,        filter_enable_altimeter_aiding);
    m_inertial_device->enableDisableAidingMeasurement(mscl::InertialTypes::AidingMeasurementSource::ODOMETER_AIDING,         filter_enable_odometer_aiding);
    m_inertial_device->enableDisableAidingMeasurement(mscl::InertialTypes::AidingMeasurementSource::MAGNETOMETER_AIDING,     filter_enable_magnetometer_aiding);
    m_inertial_device->enableDisableAidingMeasurement(mscl::InertialTypes::AidingMeasurementSource::EXTERNAL_HEADING_AIDING, filter_enable_external_heading_aiding);
  }
  else
  {
    MICROSTRAIN_INFO(m_node, "Note: The device does not support the filter aiding command.");          
  }


  //(GQ7 only) Set the filter relative position frame settings
  if(m_publish_filter_relative_pos && m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_RELATIVE_POSITION_REF))
  {
    mscl::PositionReferenceConfiguration ref;
    ref.position = mscl::Position(filter_relative_position_ref[0], filter_relative_position_ref[1], filter_relative_position_ref[2], static_cast<mscl::PositionVelocityReferenceFrame>(filter_relative_position_frame));

    MICROSTRAIN_INFO(m_node, "Setting reference position to: [%f, %f, %f], ref frame = %d", filter_relative_position_ref[0], filter_relative_position_ref[1], filter_relative_position_ref[2], filter_relative_position_frame);
    m_inertial_device->setRelativePositionReference(ref);
  }
  else
  {
    MICROSTRAIN_INFO(m_node, "Note: The device does not support the relative position command.");          
  }


  //(GQ7 only) Set the filter speed lever arm
  if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SPEED_MEASUREMENT_OFFSET))
  {
    mscl::PositionOffset offset(filter_speed_lever_arm[0], filter_speed_lever_arm[1], filter_speed_lever_arm[2]);

    m_inertial_device->setSpeedMeasurementOffset(offset);
  }
  else
  {
    MICROSTRAIN_INFO(m_node, "Note: The device does not support the filter speed lever arm command.");          
  }


  //(GQ7 only) Set the wheeled vehicle constraint
  if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_WHEELED_VEHICLE_CONSTRAINT))
  {
    MICROSTRAIN_INFO(m_node, "Setting wheeled vehicle contraint enable to %d", filter_enable_wheeled_vehicle_constraint);
    m_inertial_device->enableWheeledVehicleConstraint(filter_enable_wheeled_vehicle_constraint);
  }
  else
  {
    MICROSTRAIN_INFO(m_node, "Note: The device does not support the wheeled vehicle constraint command.");          
  }


  //(GQ7 only) Set the vertical gyro constraint
  if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_VERTICAL_GYRO_CONSTRAINT))
  {
    MICROSTRAIN_INFO(m_node, "Setting vertical gyro contraint enable to %d", filter_enable_vertical_gyro_constraint);
    m_inertial_device->enableVerticalGyroConstraint(filter_enable_vertical_gyro_constraint);
  }
  else
  {
    MICROSTRAIN_INFO(m_node, "Note: The device does not support the vertical gyro constraint command.");          
  }


  //(GQ7 only) Set the GNSS antenna calibration settings
  if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_GNSS_ANTENNA_LEVER_ARM_CAL))
  {
    mscl::AntennaLeverArmCalConfiguration config;
    config.enabled        = filter_enable_gnss_antenna_cal;
    config.maxOffsetError = filter_gnss_antenna_cal_max_offset;

    MICROSTRAIN_INFO(m_node, "Setting GNSS antenna calibration to: enable = %d, max_offset = %f", filter_enable_gnss_antenna_cal, filter_gnss_antenna_cal_max_offset);
    m_inertial_device->setAntennaLeverArmCal(config);
  }
  else
  {
    MICROSTRAIN_INFO(m_node, "Note: The device does not support the GNSS antenna calibration command.");          
  }


  //(GQ7 only) Set the filter initialization settings 
  if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_INITIALIZATION_CONFIG))
  {
    mscl::FilterInitializationValues filter_config;

    //API Variable: autoInitialize
    filter_config.autoInitialize = filter_auto_init;

    //API Variable: initialValuesSource
    filter_config.initialValuesSource = static_cast<mscl::FilterInitialValuesSource>(filter_init_condition_src);

    //API Variable: autoHeadingAlignmentMethod
    filter_config.autoHeadingAlignmentMethod = static_cast<mscl::HeadingAlignmentMethod>(filter_auto_heading_alignment_selector);

    //API Variable: initialAttitude
    //  Note: Only heading value will be used if initialValueSource indicates pitch/roll will be determined automatically.
    filter_config.initialAttitude = mscl::EulerAngles(filter_init_attitude[0], filter_init_attitude[1], filter_init_attitude[2]);

    //API Variable: initialPosition
    filter_config.initialPosition = mscl::Position(filter_init_position[0], filter_init_position[1], filter_init_position[2], static_cast<mscl::PositionVelocityReferenceFrame>(filter_init_reference_frame));

    //API Variable: initialVelocity
    filter_config.initialVelocity = mscl::GeometricVector(filter_init_velocity[0], filter_init_velocity[1], filter_init_velocity[2], static_cast<mscl::PositionVelocityReferenceFrame>(filter_init_reference_frame));

    //API Variable: referenceFrame
    filter_config.referenceFrame = static_cast<mscl::PositionVelocityReferenceFrame>(filter_init_reference_frame);    

    m_inertial_device->setInitialFilterConfiguration(filter_config);
  }
  else
  {
    MICROSTRAIN_INFO(m_node, "Note: The device does not support the next-gen filter initialization command.");          
  }

  //Enable dual antenna messages
  m_publish_gnss_dual_antenna_status = m_filter_enable_gnss_heading_aiding;

  //Enable the filter datastream
  m_inertial_device->enableDataStream(mscl::MipTypes::DataClass::CLASS_ESTFILTER);
  return true;
}

bool MicrostrainConfig::configure_sensor2vehicle(RosNodeType* node)
{
  int  filter_sensor2vehicle_frame_selector;
  std::vector<double> filter_sensor2vehicle_frame_transformation_euler(3, 0.0);
  std::vector<double> filter_sensor2vehicle_frame_transformation_matrix(9, 0.0);
  std::vector<double> filter_sensor2vehicle_frame_transformation_quaternion(4, 0.0);
  get_param<int32_t>(node, "filter_sensor2vehicle_frame_selector",                  filter_sensor2vehicle_frame_selector, 0);
  get_param<std::vector<double>>(node, "filter_sensor2vehicle_frame_transformation_euler",      filter_sensor2vehicle_frame_transformation_euler,      default_vector);
  get_param<std::vector<double>>(node, "filter_sensor2vehicle_frame_transformation_matrix",     filter_sensor2vehicle_frame_transformation_matrix,     default_matrix);
  get_param<std::vector<double>>(node, "filter_sensor2vehicle_frame_transformation_quaternion", filter_sensor2vehicle_frame_transformation_quaternion, default_quaternion);

  //Euler Angles
  if(filter_sensor2vehicle_frame_selector == 1)
  {
    //Old style - set rotation (inverse of transformation)
    if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SENS_VEHIC_FRAME_ROTATION_EULER))
    {
      //Invert the angles for "rotation"
      mscl::EulerAngles angles(-filter_sensor2vehicle_frame_transformation_euler[0], -filter_sensor2vehicle_frame_transformation_euler[1], -filter_sensor2vehicle_frame_transformation_euler[2]);

      MICROSTRAIN_INFO(m_node, "Setting sensor2vehicle frame rotation with euler angles [%f, %f, %f]", -filter_sensor2vehicle_frame_transformation_euler[0], -filter_sensor2vehicle_frame_transformation_euler[1], -filter_sensor2vehicle_frame_transformation_euler[2]);
      m_inertial_device->setSensorToVehicleRotation_eulerAngles(angles);
    }
    else if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SENS_VEHIC_FRAME_TRANSFORM_EULER))
    {
      mscl::EulerAngles angles(filter_sensor2vehicle_frame_transformation_euler[0], filter_sensor2vehicle_frame_transformation_euler[1], filter_sensor2vehicle_frame_transformation_euler[2]);
        
      MICROSTRAIN_INFO(m_node, "Setting sensor2vehicle frame transformation with euler angles [%f, %f, %f]", filter_sensor2vehicle_frame_transformation_euler[0], filter_sensor2vehicle_frame_transformation_euler[1], filter_sensor2vehicle_frame_transformation_euler[2]);
      m_inertial_device->setSensorToVehicleTransform_eulerAngles(angles);
    }
    else
    {
      MICROSTRAIN_ERROR(m_node, "**Failed to set sensor2vehicle frame transformation with euler angles!");
      return false;
    }
  }
  //Matrix
  else if(filter_sensor2vehicle_frame_selector == 2)
  { 
    //Old style - set rotation (inverse of transformation)
    if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SENS_VEHIC_FRAME_ROTATION_DCM))
    {
      //Transpose the matrix for "rotation"
      mscl::Matrix_3x3 dcm(filter_sensor2vehicle_frame_transformation_matrix[0], filter_sensor2vehicle_frame_transformation_matrix[3], filter_sensor2vehicle_frame_transformation_matrix[6], 
                            filter_sensor2vehicle_frame_transformation_matrix[1], filter_sensor2vehicle_frame_transformation_matrix[4], filter_sensor2vehicle_frame_transformation_matrix[7],
                            filter_sensor2vehicle_frame_transformation_matrix[2], filter_sensor2vehicle_frame_transformation_matrix[5], filter_sensor2vehicle_frame_transformation_matrix[8]);

      MICROSTRAIN_INFO(m_node, "Setting sensor2vehicle frame rotation with a matrix");
      m_inertial_device->setSensorToVehicleRotation_matrix(dcm);
    }
    else if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SENS_VEHIC_FRAME_TRANSFORM_DCM))
    {
      mscl::Matrix_3x3 dcm(filter_sensor2vehicle_frame_transformation_matrix[0], filter_sensor2vehicle_frame_transformation_matrix[1], filter_sensor2vehicle_frame_transformation_matrix[2], 
                            filter_sensor2vehicle_frame_transformation_matrix[3], filter_sensor2vehicle_frame_transformation_matrix[4], filter_sensor2vehicle_frame_transformation_matrix[5],
                            filter_sensor2vehicle_frame_transformation_matrix[6], filter_sensor2vehicle_frame_transformation_matrix[7], filter_sensor2vehicle_frame_transformation_matrix[8]);
          
      MICROSTRAIN_INFO(m_node, "Setting sensor2vehicle frame transformation with a matrix");
      m_inertial_device->setSensorToVehicleTransform_matrix(dcm);
    }
    else
    {
      MICROSTRAIN_ERROR(m_node, "**Failed to set sensor2vehicle frame transformation with a matrix!");
      return false;
    }
    
  }
  //Quaternion
  else if(filter_sensor2vehicle_frame_selector == 3)
  {
    //Old style - set rotation (inverse of transformation)
    if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SENS_VEHIC_FRAME_ROTATION_QUAT))
    {
      //Invert the quaternion for "rotation" (note: device uses aerospace quaternion definition [w, -i, -j, -k])
      mscl::Quaternion quat(filter_sensor2vehicle_frame_transformation_quaternion[3], -filter_sensor2vehicle_frame_transformation_quaternion[0],
                            -filter_sensor2vehicle_frame_transformation_quaternion[1], -filter_sensor2vehicle_frame_transformation_quaternion[2]);
        
      MICROSTRAIN_INFO(m_node, "Setting sensor2vehicle frame rotation with quaternion [%f %f %f %f]", -filter_sensor2vehicle_frame_transformation_quaternion[0], -filter_sensor2vehicle_frame_transformation_quaternion[1],
                            -filter_sensor2vehicle_frame_transformation_quaternion[2], filter_sensor2vehicle_frame_transformation_quaternion[3]);
      m_inertial_device->setSensorToVehicleRotation_quaternion(quat);
    }
    else if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SENS_VEHIC_FRAME_TRANSFORM_QUAT))
    {
      //No inversion for transformation (note: device uses aerospace quaternion definition [w, i, j, k])
      mscl::Quaternion quat(filter_sensor2vehicle_frame_transformation_quaternion[3], filter_sensor2vehicle_frame_transformation_quaternion[0],
                            filter_sensor2vehicle_frame_transformation_quaternion[1], filter_sensor2vehicle_frame_transformation_quaternion[2]);

      MICROSTRAIN_INFO(m_node, "Setting sensor2vehicle frame transformation with quaternion [%f %f %f %f]", filter_sensor2vehicle_frame_transformation_quaternion[0], filter_sensor2vehicle_frame_transformation_quaternion[1],
                            filter_sensor2vehicle_frame_transformation_quaternion[2], filter_sensor2vehicle_frame_transformation_quaternion[3]);
      m_inertial_device->setSensorToVehicleTransform_quaternion(quat);
    }
    else
    {
      MICROSTRAIN_ERROR(m_node, "**Failed to set sensor2vehicle frame transformation with quaternion!");
      return false;
    }
  }
  return true;
}

} // namespace Microstrain
