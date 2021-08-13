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
#include "microstrain_services.h"


namespace Microstrain
{

MicrostrainServices::MicrostrainServices(RosNodeType* node, MicrostrainConfig* config) : m_node(node), m_config(config)
{}

bool MicrostrainServices::configure_services()
{
  if(m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_DEVICE_STATUS))
  {
    m_get_basic_status_service      = create_service<TriggerServiceMsg>(m_node, "get_basic_status",      &MicrostrainServices::get_basic_status, this);
    m_get_diagnostic_report_service = create_service<TriggerServiceMsg>(m_node, "get_diagnostic_report", &MicrostrainServices::get_diagnostic_report, this);
  }

  if(m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_GET_DEVICE_INFO))
  {
    m_device_report_service = create_service<TriggerServiceMsg>(m_node, "device_report", &MicrostrainServices::device_report, this);
  }

  //IMU tare orientation service
  if (m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_TARE_ORIENT))
  {
    m_set_tare_orientation_service = create_service<SetTareOrientationServiceMsg>(m_node, "set_tare_orientation", &MicrostrainServices::set_tare_orientation, this);
  }

  //IMU Complementary filter service
  if (m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_COMPLEMENTARY_FILTER_SETTINGS))
  {
    m_set_complementary_filter_service = create_service<SetComplementaryFilterServiceMsg>(m_node, "set_complementary_filter", &MicrostrainServices::set_complementary_filter, this);
    m_get_complementary_filter_service = create_service<GetComplementaryFilterServiceMsg>(m_node, "get_complementary_filter", &MicrostrainServices::get_complementary_filter, this);
  }
  
  //IMU sensor2vehicle frame rotation service
  if (m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SENS_VEHIC_FRAME_ROTATION_EULER))
  {
    m_set_sensor2vehicle_rotation_service = create_service<SetSensor2VehicleRotationServiceMsg>(m_node, "set_sensor2vehicle_rotation", &MicrostrainServices::set_sensor2vehicle_rotation, this);
    m_get_sensor2vehicle_rotation_service = create_service<GetSensor2VehicleRotationServiceMsg>(m_node, "get_sensor2vehicle_rotation", &MicrostrainServices::get_sensor2vehicle_rotation, this);
  }

  //IMU sensor2vehicle frame offset service
  if (m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SENS_VEHIC_FRAME_OFFSET))
  {
    m_set_sensor2vehicle_offset_service = create_service<SetSensor2VehicleOffsetServiceMsg>(m_node, "set_sensor2vehicle_offset", &MicrostrainServices::set_sensor2vehicle_offset, this);
    m_get_sensor2vehicle_offset_service = create_service<GetSensor2VehicleOffsetServiceMsg>(m_node, "get_sensor2vehicle_offset", &MicrostrainServices::get_sensor2vehicle_offset, this);
  }

  //IMU sensor2vehicle transformation service
  if (m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SENS_VEHIC_FRAME_OFFSET) &&
      m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SENS_VEHIC_FRAME_ROTATION_EULER))
  {
    m_get_sensor2vehicle_transformation_service = create_service<GetSensor2VehicleTransformationServiceMsg>(m_node, "get_sensor2vehicle_transformation", &MicrostrainServices::get_sensor2vehicle_transformation, this);
  }

  //IMU Accel bias service
  if (m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_ACCEL_BIAS))
  {
    m_set_accel_bias_service = create_service<SetAccelBiasServiceMsg>(m_node, "set_accel_bias", &MicrostrainServices::set_accel_bias, this);
    m_get_accel_bias_service = create_service<GetAccelBiasServiceMsg>(m_node, "get_accel_bias", &MicrostrainServices::get_accel_bias, this);
  }

  //IMU gyro bias service
  if (m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_GYRO_BIAS))
  {
    m_set_gyro_bias_service = create_service<SetGyroBiasServiceMsg>(m_node, "set_gyro_bias", &MicrostrainServices::set_gyro_bias, this);
    m_get_gyro_bias_service = create_service<GetGyroBiasServiceMsg>(m_node, "get_gyro_bias", &MicrostrainServices::get_gyro_bias, this);
  }

  //IMU Gyro bias capture service
  if (m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_CAP_GYRO_BIAS))
  {
    m_gyro_bias_capture_service = create_service<TriggerServiceMsg>(m_node, "gyro_bias_capture", &MicrostrainServices::gyro_bias_capture, this);
  }
  
  //IMU Mag Hard iron offset service
  if (m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_MAG_HARD_IRON_OFFSET))
  {
    m_set_hard_iron_values_service = create_service<SetHardIronValuesServiceMsg>(m_node, "set_hard_iron_values", &MicrostrainServices::set_hard_iron_values, this);
    m_get_hard_iron_values_service = create_service<GetHardIronValuesServiceMsg>(m_node, "get_hard_iron_values", &MicrostrainServices::get_hard_iron_values, this);
  }

  //IMU Mag Soft iron matrix service
  if (m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_MAG_SOFT_IRON_MATRIX))
  {
    m_set_soft_iron_matrix_service = create_service<SetSoftIronMatrixServiceMsg>(m_node, "set_soft_iron_matrix", &MicrostrainServices::set_soft_iron_matrix, this);
    m_get_soft_iron_matrix_service = create_service<GetSoftIronMatrixServiceMsg>(m_node, "get_soft_iron_matrix", &MicrostrainServices::get_soft_iron_matrix, this);
  }

  //IMU Coning and sculling enable service
  if (m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_CONING_SCULLING))
  {
    m_set_coning_sculling_comp_service = create_service<SetConingScullingCompServiceMsg>(m_node, "set_coning_sculling_comp", &MicrostrainServices::set_coning_sculling_comp, this);
    m_get_coning_sculling_comp_service = create_service<GetConingScullingCompServiceMsg>(m_node, "get_coning_sculling_comp", &MicrostrainServices::get_coning_sculling_comp, this);
  }
  
  //Kalman filter reset
  if (m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_RESET_FILTER))
  {
    m_reset_filter_service = create_service<EmptyServiceMsg>(m_node, "reset_kf", &MicrostrainServices::reset_filter, this);
  }

  //Kalman filter estimation control service
  if (m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_BIAS_EST_CTRL))
  {
    m_set_estimation_control_flags_service = create_service<SetEstimationControlFlagsServiceMsg>(m_node, "set_estimation_control_flags", &MicrostrainServices::set_estimation_control_flags, this);
    m_get_estimation_control_flags_service = create_service<GetEstimationControlFlagsServiceMsg>(m_node, "get_estimation_control_flags", &MicrostrainServices::get_estimation_control_flags, this);
  }

  //Kalman filter initialization with full Euler angles service
  if (m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_INIT_ATTITUDE))
  {
    m_init_filter_euler_service = create_service<InitFilterEulerServiceMsg>(m_node, "init_filter_euler", &MicrostrainServices::init_filter_euler, this);
  }

  //Kalman filter initialization with heading only service
  if (m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_INIT_HEADING))
  {
    m_init_filter_heading_service = create_service<InitFilterHeadingServiceMsg>(m_node, "init_filter_heading", &MicrostrainServices::init_filter_heading, this);
  }

  //Kalman filter heading source service
  if(m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_HEADING_UPDATE_CTRL))
  {
    m_set_heading_source_service = create_service<SetHeadingSourceServiceMsg>(m_node, "set_heading_source", &MicrostrainServices::set_heading_source, this);
    m_get_heading_source_service = create_service<GetHeadingSourceServiceMsg>(m_node, "get_heading_source", &MicrostrainServices::get_heading_source, this);
  }

  //Kalman filter commanded ZUPT service
  if (m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_CMDED_ZERO_VEL_UPDATE))
  {
    m_commanded_vel_zupt_service = create_service<TriggerServiceMsg>(m_node, "commanded_vel_zupt", &MicrostrainServices::commanded_vel_zupt, this);
  }
  
  //Kalman filter commanded angular ZUPT service
  if (m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_CMDED_ZERO_ANG_RATE_UPDATE))
  {
    m_commanded_ang_rate_zupt_service = create_service<TriggerServiceMsg>(m_node, "commanded_ang_rate_zupt", &MicrostrainServices::commanded_ang_rate_zupt, this);
  }

  //Kalman filter Accel white noise 1-sigma service
  if (m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_ACCEL_WHT_NSE_STD_DEV))
  {
    m_set_accel_noise_service = create_service<SetAccelNoiseServiceMsg>(m_node, "set_accel_noise", &MicrostrainServices::set_accel_noise, this);
    m_get_accel_noise_service = create_service<GetAccelNoiseServiceMsg>(m_node, "get_accel_noise", &MicrostrainServices::get_accel_noise, this);
  }

  //Kalman filter Gyro white noise 1-sigma service
  if (m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_GYRO_WHT_NSE_STD_DEV))
  {
    m_set_gyro_noise_service = create_service<SetGyroNoiseServiceMsg>(m_node, "set_gyro_noise", &MicrostrainServices::set_gyro_noise, this);
    m_get_gyro_noise_service = create_service<GetGyroNoiseServiceMsg>(m_node, "get_gyro_noise", &MicrostrainServices::get_gyro_noise, this);
  }

  //Kalman filter Mag noise 1-sigma service
  if (m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_HARD_IRON_OFFSET_PROCESS_NOISE))
  {
    m_set_mag_noise_service = create_service<SetMagNoiseServiceMsg>(m_node, "set_mag_noise", &MicrostrainServices::set_mag_noise, this);
    m_get_mag_noise_service = create_service<GetMagNoiseServiceMsg>(m_node, "get_mag_noise", &MicrostrainServices::get_mag_noise, this);
  }

  //Kalman filter accel bias model service
  if (m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_ACCEL_BIAS_MODEL_PARAMS))
  {
    m_set_accel_bias_model_service = create_service<SetAccelBiasModelServiceMsg>(m_node, "set_accel_bias_model", &MicrostrainServices::set_accel_bias_model, this);
    m_get_accel_bias_model_service = create_service<GetAccelBiasModelServiceMsg>(m_node, "get_accel_bias_model", &MicrostrainServices::get_accel_bias_model, this);
  }

  //Kalman filter gyro bias model service
  if (m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_GYRO_BIAS_MODEL_PARAMS))
  {
    m_set_gyro_bias_model_service = create_service<SetGyroBiasModelServiceMsg>(m_node, "set_gyro_bias_model", &MicrostrainServices::set_gyro_bias_model, this);
    m_get_gyro_bias_model_service = create_service<GetGyroBiasModelServiceMsg>(m_node, "get_gyro_bias_model", &MicrostrainServices::get_gyro_bias_model, this);
  }

  //Kalman filter magnetometer magnitude adaptive filter service
  if (m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_MAG_MAGNITUDE_ERR_ADAPT_MEASURE))
  {
    m_set_mag_adaptive_vals_service = create_service<SetMagAdaptiveValsServiceMsg>(m_node, "set_mag_adaptive_vals", &MicrostrainServices::set_mag_adaptive_vals, this);
    m_get_mag_adaptive_vals_service = create_service<GetMagAdaptiveValsServiceMsg>(m_node, "get_mag_adaptive_vals", &MicrostrainServices::get_mag_adaptive_vals, this);
  }

  //Kalman filter magnetometer dip angle adaptive filter service
  if (m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_MAG_DIP_ANGLE_ERR_ADAPT_MEASURE))
  {
    m_set_mag_dip_adaptive_vals_service = create_service<SetMagDipAdaptiveValsServiceMsg>(m_node, "set_mag_dip_adaptive_vals", &MicrostrainServices::set_mag_dip_adaptive_vals, this);
    m_get_mag_dip_adaptive_vals_service = create_service<GetMagDipAdaptiveValsServiceMsg>(m_node, "get_mag_dip_adaptive_vals", &MicrostrainServices::get_mag_dip_adaptive_vals, this);
  }
  
  //Kalman filter gravity adaptive filtering settings service
  if (m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_GRAV_MAGNITUDE_ERR_ADAPT_MEASURE))
  {
    m_set_gravity_adaptive_vals_service = create_service<SetGravityAdaptiveValsServiceMsg>(m_node, "set_gravity_adaptive_vals", &MicrostrainServices::set_gravity_adaptive_vals, this);
    m_get_gravity_adaptive_vals_service = create_service<GetGravityAdaptiveValsServiceMsg>(m_node, "get_gravity_adaptive_vals", &MicrostrainServices::get_gravity_adaptive_vals, this);
  }
  
  //Kalman filter automatic angular ZUPT configuration service
  if (m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_ZERO_ANG_RATE_UPDATE_CTRL))
  {
    m_set_zero_angle_update_threshold_service = create_service<SetZeroAngleUpdateThresholdServiceMsg>(m_node, "set_zero_angle_update_threshold", &MicrostrainServices::set_zero_angle_update_threshold, this);
    m_get_zero_angle_update_threshold_service = create_service<GetZeroAngleUpdateThresholdServiceMsg>(m_node, "get_zero_angle_update_threshold", &MicrostrainServices::get_zero_angle_update_threshold, this);
  }
  
  //Kalman Filter automatic ZUPT configuration service
  if (m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_ZERO_VEL_UPDATE_CTRL))
  {
    m_set_zero_velocity_update_threshold_service = create_service<SetZeroVelocityUpdateThresholdServiceMsg>(m_node, "set_zero_velocity_update_threshold", &MicrostrainServices::set_zero_velocity_update_threshold, this);
    m_get_zero_velocity_update_threshold_service = create_service<GetZeroVelocityUpdateThresholdServiceMsg>(m_node, "get_zero_velocity_update_threshold", &MicrostrainServices::get_zero_velocity_update_threshold, this);
  }

  //Reference position service
  if (m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SET_REF_POSITION))
  {
    m_set_reference_position_service = create_service<SetReferencePositionServiceMsg>(m_node, "set_reference_position", &MicrostrainServices::set_reference_position, this);
    m_get_reference_position_service = create_service<GetReferencePositionServiceMsg>(m_node, "get_reference_position", &MicrostrainServices::get_reference_position, this);
  }

  //Dynamics mode service
  if (m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_VEHIC_DYNAMICS_MODE))
  {
    m_set_dynamics_mode_service = create_service<SetDynamicsModeServiceMsg>(m_node, "set_dynamics_mode", &MicrostrainServices::set_dynamics_mode, this);
    m_get_dynamics_mode_service = create_service<GetDynamicsModeServiceMsg>(m_node, "get_dynamics_mode", &MicrostrainServices::get_dynamics_mode, this);
  } 


  //Device Settings Service
  if (m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_SAVE_STARTUP_SETTINGS))
  {
    m_device_settings_service = create_service<DeviceSettingsServiceMsg>(m_node, "device_settings", &MicrostrainServices::device_settings, this);
  }

  //External Heading Service
  if ((m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_EXTERN_HEADING_UPDATE)) ||
      (m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_EXT_HEADING_UPDATE_TS)))
  {
    m_external_heading_service = create_service<ExternalHeadingUpdateServiceMsg>(m_node, "external_heading", &MicrostrainServices::external_heading_update, this);
  }

  //Relative Position Reference Service
  if (m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_EXT_HEADING_UPDATE_TS))
  {
    m_set_relative_position_reference_service = create_service<SetRelativePositionReferenceServiceMsg>(m_node, "set_relative_position_reference", &MicrostrainServices::set_relative_position_reference, this);
    m_get_relative_position_reference_service = create_service<GetRelativePositionReferenceServiceMsg>(m_node, "get_relative_position_reference", &MicrostrainServices::get_relative_position_reference, this);
  }
  return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Device Report 
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::device_report(TriggerServiceMsg::Request &req, TriggerServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device)
  {
    try
    {
      MICROSTRAIN_INFO(m_node, "Model Name       => %s\n",   m_config->m_inertial_device->modelName().c_str());
      MICROSTRAIN_INFO(m_node, "Model Number     => %s\n",   m_config->m_inertial_device->modelNumber().c_str());
      MICROSTRAIN_INFO(m_node, "Serial Number    => %s\n",   m_config->m_inertial_device->serialNumber().c_str());
      MICROSTRAIN_INFO(m_node, "Options          => %s\n",   m_config->m_inertial_device->deviceOptions().c_str());
      MICROSTRAIN_INFO(m_node, "Firmware Version => %s\n\n", m_config->m_inertial_device->firmwareVersion().str().c_str());
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Reset Filter Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::reset_filter(EmptyServiceMsg::Request &req,
                               EmptyServiceMsg::Response &res)
{
  MICROSTRAIN_INFO(m_node, "Resetting filter\n");

  if(m_config->m_inertial_device)
  {
    try
    {
      m_config->m_inertial_device->resetFilter();
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return true;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialize Filter (Euler Angles) Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::init_filter_euler(InitFilterEulerServiceMsg::Request &req,
                                    InitFilterEulerServiceMsg::Response &res)
{
  res.success = false;
  MICROSTRAIN_INFO(m_node, "Initializing the Filter with Euler angles\n");

  if(m_config->m_inertial_device)
  {
    try
    {
      mscl::EulerAngles attitude(req.angle.x,
                                 req.angle.y,
                                 req.angle.z);

      m_config->m_inertial_device->setInitialAttitude(attitude);
      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialize Filter (Heading Angle) Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::init_filter_heading(InitFilterHeadingServiceMsg::Request &req,
                                      InitFilterHeadingServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device)
  {
    try
    {
      MICROSTRAIN_INFO(m_node, "Initializing the Filter with a heading angle\n");
      m_config->m_inertial_device->setInitialHeading(req.angle);

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Accel Bias Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::set_accel_bias(SetAccelBiasServiceMsg::Request &req,
                                 SetAccelBiasServiceMsg::Response &res)
{
  res.success = false;
  MICROSTRAIN_INFO(m_node, "Setting accel bias values");

  if(m_config->m_inertial_device)
  {
    try
    {
      mscl::GeometricVector biasVector = m_config->m_inertial_device->getAccelerometerBias();

      MICROSTRAIN_INFO(m_node, "Accel bias vector values are: %f %f %f",
               biasVector.x(), biasVector.y(), biasVector.z());
      MICROSTRAIN_INFO(m_node, "Client request values are: %.2f %.2f %.2f",
               req.bias.x, req.bias.y, req.bias.z);

      biasVector.x(req.bias.x);
      biasVector.y(req.bias.y);
      biasVector.z(req.bias.z);

      m_config->m_inertial_device->setAccelerometerBias(biasVector);

      MICROSTRAIN_INFO(m_node, "New accel bias vector values are: %.2f %.2f %.2f",
               biasVector.x(), biasVector.y(), biasVector.z());

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Accel Bias Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::get_accel_bias(GetAccelBiasServiceMsg::Request &req,
                                 GetAccelBiasServiceMsg::Response &res)
{
  res.success = false;
  MICROSTRAIN_INFO(m_node, "Getting accel bias values\n");

  if(m_config->m_inertial_device)
  {
    try
    {
      mscl::GeometricVector biasVector = m_config->m_inertial_device->getAccelerometerBias();

      MICROSTRAIN_INFO(m_node, "Accel bias vector values are: %f %f %f.\n",
               biasVector.x(), biasVector.y(), biasVector.z());

      res.bias.x = biasVector.x();
      res.bias.y = biasVector.y();
      res.bias.z = biasVector.z();

     res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }
  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Gyro Bias Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::set_gyro_bias(SetGyroBiasServiceMsg::Request &req,
                                SetGyroBiasServiceMsg::Response &res)
{
  res.success = false;
  MICROSTRAIN_INFO(m_node, "Setting gyro bias values");

  if(m_config->m_inertial_device)
  {
    try
    {
      mscl::GeometricVector biasVector = m_config->m_inertial_device->getGyroBias();

      MICROSTRAIN_INFO(m_node, "Gyro bias vector values are: %f %f %f",
               biasVector.x(), biasVector.y(), biasVector.z());

      MICROSTRAIN_INFO(m_node, "Client request values are: %.2f %.2f %.2f",
               req.bias.x, req.bias.y, req.bias.z);

      biasVector.x(req.bias.x);
      biasVector.y(req.bias.y);
      biasVector.z(req.bias.z);

      m_config->m_inertial_device->setGyroBias(biasVector);

      MICROSTRAIN_INFO(m_node, "New gyro bias vector values are: %.2f %.2f %.2f",
               biasVector.x(), biasVector.y(), biasVector.z());

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Gyro Bias Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::get_gyro_bias(GetGyroBiasServiceMsg::Request &req,
                                GetGyroBiasServiceMsg::Response &res)
{
  res.success = false;
  MICROSTRAIN_INFO(m_node, "Getting gyro bias values");
  if(m_config->m_inertial_device)
  {
    try
    {
      mscl::GeometricVector biasVector = m_config->m_inertial_device->getGyroBias();

      MICROSTRAIN_INFO(m_node, "Gyro bias vector values are: %f %f %f",
               biasVector.x(), biasVector.y(), biasVector.z());
               
      res.bias.x = biasVector.x();
      res.bias.y = biasVector.y();
      res.bias.z = biasVector.z();

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Gyro Bias Capture Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::gyro_bias_capture(TriggerServiceMsg::Request &req, TriggerServiceMsg::Response &res)
{
  res.success = false;
  MICROSTRAIN_INFO(m_node, "Performing Gyro Bias capture.\nPlease keep device stationary during the 10 second gyro bias capture interval\n");

  if(m_config->m_inertial_device)
  {
    try
    {
      mscl::GeometricVector biasVector = m_config->m_inertial_device->captureGyroBias(10000);

      MICROSTRAIN_INFO(m_node, "Gyro Bias Captured:\nbias_vector[0] = %f\nbias_vector[1] = %f\nbias_vector[2] = %f\n\n",
               biasVector.x(), biasVector.y(), biasVector.z());

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Hard Iron Offset Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::set_hard_iron_values(SetHardIronValuesServiceMsg::Request &req,
                                       SetHardIronValuesServiceMsg::Response &res)
{
  res.success = false;
  MICROSTRAIN_INFO(m_node, "Setting hard iron values");

  if(m_config->m_inertial_device)
  {
    try
    {
      mscl::GeometricVector biasVector = m_config->m_inertial_device->getMagnetometerHardIronOffset();

      MICROSTRAIN_INFO(m_node, "Hard Iron vector values are: %f %f %f",
               biasVector.x(), biasVector.y(), biasVector.z());
      MICROSTRAIN_INFO(m_node, "Client request values are: %.2f %.2f %.2f",
               req.bias.x, req.bias.y, req.bias.z);

      biasVector.x(req.bias.x);
      biasVector.y(req.bias.y);
      biasVector.z(req.bias.z);

      m_config->m_inertial_device->setMagnetometerHardIronOffset(biasVector);

      MICROSTRAIN_INFO(m_node, "New hard iron values are: %.2f %.2f %.2f",
               biasVector.x(), biasVector.y(), biasVector.z());

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Hard Iron Offset Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::get_hard_iron_values(GetHardIronValuesServiceMsg::Request &req,
                                       GetHardIronValuesServiceMsg::Response &res)
{
  res.success = false;
  MICROSTRAIN_INFO(m_node, "Getting gyro bias values");

  if(m_config->m_inertial_device)
  {
    try
    {
      mscl::GeometricVector biasVector = m_config->m_inertial_device->getMagnetometerHardIronOffset();

      MICROSTRAIN_INFO(m_node, "Hard iron values are: %f %f %f",
               biasVector.x(), biasVector.y(), biasVector.z());

      res.bias.x = biasVector.x();
      res.bias.y = biasVector.y();
      res.bias.z = biasVector.z();

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Soft Iron Matrix Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::set_soft_iron_matrix(SetSoftIronMatrixServiceMsg::Request &req,
                                       SetSoftIronMatrixServiceMsg::Response &res)
{
  res.success = false;
  MICROSTRAIN_INFO(m_node, "Setting the soft iron matrix values\n");

  if(m_config->m_inertial_device)
  {
    try
    {
      mscl::Matrix_3x3 data;
      data.set(0, 0, req.soft_iron_1.x);
      data.set(0, 1, req.soft_iron_1.y);
      data.set(0, 2, req.soft_iron_1.z);
      data.set(1, 0, req.soft_iron_2.x);
      data.set(1, 1, req.soft_iron_2.y);
      data.set(1, 2, req.soft_iron_2.z);
      data.set(2, 0, req.soft_iron_3.x);
      data.set(2, 1, req.soft_iron_3.y);
      data.set(2, 2, req.soft_iron_3.z);

      m_config->m_inertial_device->setMagnetometerSoftIronMatrix(data);
      MICROSTRAIN_INFO(m_node, "Sent values:     [%f  %f  %f][%f  %f  %f][%f  %f  %f]\n",
               data(0, 0), data(0, 1), data(0, 2),
               data(1, 0), data(1, 1), data(1, 2),
               data(2, 0), data(2, 1), data(2, 2));

      data = m_config->m_inertial_device->getMagnetometerSoftIronMatrix();

      MICROSTRAIN_INFO(m_node, "Returned values:     [%f  %f  %f][%f  %f  %f][%f  %f  %f]\n",
               data(0, 0), data(0, 1), data(0, 2),
               data(1, 0), data(1, 1), data(1, 2),
               data(2, 0), data(2, 1), data(2, 2));

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Soft Iron Matrix Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::get_soft_iron_matrix(GetSoftIronMatrixServiceMsg::Request &req,
                                       GetSoftIronMatrixServiceMsg::Response &res)
{
  res.success = false;
  MICROSTRAIN_INFO(m_node, "Getting the soft iron matrix values\n");

  if(m_config->m_inertial_device)
  {
    try
    {
      mscl::Matrix_3x3 data = m_config->m_inertial_device->getMagnetometerSoftIronMatrix();

      MICROSTRAIN_INFO(m_node, "Soft iron matrix values: [%f  %f  %f][%f  %f  %f][%f  %f  %f]\n",
               data(0, 0), data(0, 1), data(0, 2),
               data(1, 0), data(1, 1), data(1, 2),
               data(2, 0), data(2, 1), data(2, 2));

      res.soft_iron_1.x = data(0, 0);
      res.soft_iron_1.y = data(0, 1);
      res.soft_iron_1.z = data(0, 2);
      res.soft_iron_2.x = data(1, 0);
      res.soft_iron_2.y = data(1, 1);
      res.soft_iron_2.z = data(1, 2);
      res.soft_iron_3.x = data(2, 0);
      res.soft_iron_3.y = data(2, 1);
      res.soft_iron_3.z = data(2, 2);

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Complementary Filter Settings Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::set_complementary_filter(SetComplementaryFilterServiceMsg::Request &req,
                                           SetComplementaryFilterServiceMsg::Response &res)
{
  MICROSTRAIN_INFO(m_node, "Setting the complementary filter values\n");
  res.success = false;

  if(m_config->m_inertial_device)
  {
    try
    {
      mscl::ComplementaryFilterData comp_filter_command;
      comp_filter_command.upCompensationEnabled          = req.up_comp_enable;
      comp_filter_command.upCompensationTimeInSeconds    = req.up_comp_time_const;
      comp_filter_command.northCompensationEnabled       = req.north_comp_enable;
      comp_filter_command.northCompensationTimeInSeconds = req.north_comp_time_const;

      m_config->m_inertial_device->setComplementaryFilterSettings(comp_filter_command);

      MICROSTRAIN_INFO(m_node, "Sent values:     Up Enable: %d North Enable: %d Up Time Constant: %f North Time Constant: %f \n",
               comp_filter_command.upCompensationEnabled, comp_filter_command.northCompensationEnabled,
               comp_filter_command.upCompensationTimeInSeconds, comp_filter_command.northCompensationTimeInSeconds);

      comp_filter_command = m_config->m_inertial_device->getComplementaryFilterSettings();

      MICROSTRAIN_INFO(m_node, "Returned values: Up Enable: %d North Enable: %d Up Time Constant: %f North Time Constant: %f \n",
               comp_filter_command.upCompensationEnabled, comp_filter_command.northCompensationEnabled,
               comp_filter_command.upCompensationTimeInSeconds, comp_filter_command.northCompensationTimeInSeconds);

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Complementary Filter Settings Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::get_complementary_filter(GetComplementaryFilterServiceMsg::Request &req,
                                           GetComplementaryFilterServiceMsg::Response &res)
{
  res.success = false;
  MICROSTRAIN_INFO(m_node, "Getting the complementary filter values\n");

  if(m_config->m_inertial_device)
  {
    try
    {
      mscl::ComplementaryFilterData comp_filter_command = m_config->m_inertial_device->getComplementaryFilterSettings();

      MICROSTRAIN_INFO(m_node, "Returned values: Up Enable: %d North Enable: %d Up Time Constant: %f North Time Constant: %f \n",
               comp_filter_command.upCompensationEnabled, comp_filter_command.northCompensationEnabled,
               comp_filter_command.upCompensationTimeInSeconds, comp_filter_command.northCompensationTimeInSeconds);

      res.up_comp_enable        = comp_filter_command.upCompensationEnabled;
      res.up_comp_time_const    = comp_filter_command.upCompensationTimeInSeconds;
      res.north_comp_enable     = comp_filter_command.northCompensationEnabled;
      res.north_comp_time_const = comp_filter_command.northCompensationTimeInSeconds ;

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Heading Source Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::set_heading_source(SetHeadingSourceServiceMsg::Request &req,
                                     SetHeadingSourceServiceMsg::Response &res)
{
  res.success = false;
  MICROSTRAIN_INFO(m_node, "Set Heading Source\n");

  if(m_config->m_inertial_device)
  {
    try
    {
      mscl::InertialTypes::HeadingUpdateEnableOption source = static_cast<mscl::InertialTypes::HeadingUpdateEnableOption>(req.heading_source);
      
      for(mscl::HeadingUpdateOptions headingSources : m_config->m_inertial_device->features().supportedHeadingUpdateOptions())
      {
        if(headingSources.AsOptionId() == source)
        {
          MICROSTRAIN_INFO(m_node, "Setting heading source to %#04X", source);
          m_config->m_inertial_device->setHeadingUpdateControl(mscl::HeadingUpdateOptions(source));
          res.success = true;
          break;
        }
      }
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Heading Source Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::get_heading_source(GetHeadingSourceServiceMsg::Request &req,
                                     GetHeadingSourceServiceMsg::Response &res)
{
  res.success = false;
  MICROSTRAIN_INFO(m_node, "Getting the heading source\n");

  if(m_config->m_inertial_device)
  {
    try
    {
      mscl::HeadingUpdateOptions source = m_config->m_inertial_device->getHeadingUpdateControl();

      MICROSTRAIN_INFO(m_node, "Current heading source is %#04X", source.AsOptionId());

      res.heading_source = static_cast<uint8_t>(source.AsOptionId());

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Sensor2Vehicle Frame Rotation Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::set_sensor2vehicle_rotation(SetSensor2VehicleRotationServiceMsg::Request &req,
                                              SetSensor2VehicleRotationServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device)
  {
    try
    {
      MICROSTRAIN_INFO(m_node, "Setting the sensor to vehicle frame rotation\n");
      mscl::EulerAngles angles(req.angle.x, req.angle.y, req.angle.z);
      m_config->m_inertial_device->setSensorToVehicleRotation_eulerAngles(angles);

      angles = m_config->m_inertial_device->getSensorToVehicleRotation_eulerAngles();

      MICROSTRAIN_INFO(m_node, "Rotation successfully set.\n");
      MICROSTRAIN_INFO(m_node, "New angles: %f roll %f pitch %f yaw\n",
               angles.roll(), angles.pitch(), angles.yaw());

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Sensor2Vehicle Frame Rotation Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::get_sensor2vehicle_rotation(GetSensor2VehicleRotationServiceMsg::Request &req,
                                              GetSensor2VehicleRotationServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device)
  {
    try
    {
      mscl::EulerAngles angles = m_config->m_inertial_device->getSensorToVehicleRotation_eulerAngles();
      MICROSTRAIN_INFO(m_node, "Sensor Vehicle Frame Rotation Angles: %f roll %f pitch %f yaw\n",
               angles.roll(), angles.pitch(), angles.yaw());

      res.angle.x = angles.roll();
      res.angle.y = angles.pitch();
      res.angle.z = angles.yaw();

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Sensor2Vehicle Frame Offset Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

// Set sensor to vehicle frame offset. Only in 45
bool MicrostrainServices::set_sensor2vehicle_offset(SetSensor2VehicleOffsetServiceMsg::Request &req,
                                            SetSensor2VehicleOffsetServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device)
  {
    try
    {
      MICROSTRAIN_INFO(m_node, "Setting the sensor to vehicle frame offset\n");
      mscl::PositionOffset offset(req.offset.x, req.offset.y, req.offset.z);
      m_config->m_inertial_device->setSensorToVehicleOffset(offset);

      offset = m_config->m_inertial_device->getSensorToVehicleOffset();
      MICROSTRAIN_INFO(m_node, "Offset successfully set.\n");
      MICROSTRAIN_INFO(m_node, "Returned offset: %f X %f Y %f Z\n",
               offset.x(), offset.y(), offset.z());

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Sensor2Vehicle Frame Offset Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::get_sensor2vehicle_offset(GetSensor2VehicleOffsetServiceMsg::Request &req,
                                            GetSensor2VehicleOffsetServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device)
  {
    try
    {
      MICROSTRAIN_INFO(m_node, "Getting the sensor to vehicle frame offset\n");

      mscl::PositionOffset offset = m_config->m_inertial_device->getSensorToVehicleOffset();
      MICROSTRAIN_INFO(m_node, "Returned offset: %f X %f Y %f Z\n",
               offset.x(), offset.y(), offset.z());

      res.offset.x = offset.x();
      res.offset.y = offset.y();
      res.offset.z = offset.z();

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Sensor2Vehicle Frame Transformation (Combination of Offset and Rotation) Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::get_sensor2vehicle_transformation(GetSensor2VehicleTransformationServiceMsg::Request &req, 
                                                    GetSensor2VehicleTransformationServiceMsg::Response &res)
{
  res.success = false;

  if(!m_config->m_inertial_device)
  {
    return res.success;
  }

  try
  {
    MICROSTRAIN_INFO(m_node, "Getting transform from sensor frame to vehicle frame");
    const mscl::PositionOffset offset = m_config->m_inertial_device->getSensorToVehicleOffset();
    const mscl::EulerAngles rotation  = m_config->m_inertial_device->getSensorToVehicleRotation_eulerAngles();

    //set offset components from the device-stored values 
    res.offset.x = offset.x();
    res.offset.y = offset.y();
    res.offset.z = offset.z();

    //set rotational components from the device-stored values 
    tf2::Quaternion quat;
    quat.setRPY(rotation.roll(), rotation.pitch(), rotation.yaw());
    tf2::convert(quat, res.rotation);

    res.success = true;
  }
  catch(mscl::Error &e)
  {
    MICROSTRAIN_ERROR(m_node, "Error getting sensor to vehicle transform: '%s'", e.what());
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Reference Position Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::set_reference_position(SetReferencePositionServiceMsg::Request &req,
                                         SetReferencePositionServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device)
  {
    try
    {
      MICROSTRAIN_INFO(m_node, "Setting reference Position\n");

      mscl::Position referencePosition(req.position.x, req.position.y, req.position.z);
      mscl::FixedReferencePositionData referencePositionData(true, referencePosition);

      m_config->m_inertial_device->setFixedReferencePosition(referencePositionData);

      MICROSTRAIN_INFO(m_node, "Reference position successfully set\n");
      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Reference Position Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::get_reference_position(GetReferencePositionServiceMsg::Request &req,
                                         GetReferencePositionServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device)
  {
    try
    {
      MICROSTRAIN_INFO(m_node, "Getting reference position");

      mscl::Position referencePosition = m_config->m_inertial_device->getFixedReferencePosition().referencePosition;
      MICROSTRAIN_INFO(m_node, "Reference position: Lat %f , Long %f, Alt %f", referencePosition.latitude(),
               referencePosition.longitude(), referencePosition.altitude());

      res.position.x = referencePosition.latitude();
      res.position.y = referencePosition.longitude();
      res.position.z = referencePosition.altitude();

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Enable/Disable Coning and Sculling Compensation Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::set_coning_sculling_comp(SetConingScullingCompServiceMsg::Request &req,
                                           SetConingScullingCompServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device)
  {
    try
    {
      MICROSTRAIN_INFO(m_node, "%s Coning and Sculling compensation", req.enable ? "DISABLED" : "ENABLED\n");
      m_config->m_inertial_device->setConingAndScullingEnable(req.enable);

      MICROSTRAIN_INFO(m_node, "Reading Coning and Sculling compensation enabled state:\n");

      bool enabled = m_config->m_inertial_device->getConingAndScullingEnable();
      MICROSTRAIN_INFO(m_node, "%s Coning and Sculling compensation", enabled ? "DISABLED" : "ENABLED\n");

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Enable/Disable Coning and Sculling Compensation Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::get_coning_sculling_comp(GetConingScullingCompServiceMsg::Request &req,
                                           GetConingScullingCompServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device)
  {
    try
    {
      MICROSTRAIN_INFO(m_node, "Reading Coning and Sculling compensation enabled state:\n");

      bool enabled = m_config->m_inertial_device->getConingAndScullingEnable();
      MICROSTRAIN_INFO(m_node, "%s Coning and Sculling compensation", enabled ? "DISABLED" : "ENABLED\n");
      
      res.enable  = enabled;
      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Estimation Control Flags Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::set_estimation_control_flags(SetEstimationControlFlagsServiceMsg::Request &req,
                                               SetEstimationControlFlagsServiceMsg::Response &res)
{
  if(m_config->m_inertial_device)
  {
    try
    {
      mscl::EstimationControlOptions flags(req.flags);
      m_config->m_inertial_device->setEstimationControlFlags(flags);
      flags = m_config->m_inertial_device->getEstimationControlFlags();
      MICROSTRAIN_INFO(m_node, "Estimation control set to: %d", flags.AsUint16());

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Estimation Control Flags Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::get_estimation_control_flags(GetEstimationControlFlagsServiceMsg::Request &req,
                                               GetEstimationControlFlagsServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device)
  {
    try
    {
      uint16_t flags = m_config->m_inertial_device->getEstimationControlFlags().AsUint16();

      MICROSTRAIN_INFO(m_node, "Estimation control set to: %x", flags);

      res.flags   = flags;
      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Device Basic Status Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

void MicrostrainServices::get_basic_status_ptr(const std::shared_ptr<TriggerServiceMsg::Request> req, std::shared_ptr<TriggerServiceMsg::Response> res)
{}

bool MicrostrainServices::get_basic_status(TriggerServiceMsg::Request &req, TriggerServiceMsg::Response &res)
{
  if(!m_config->m_inertial_device)
  {
    return false;
  }
  
  if(m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_DEVICE_STATUS))
  {
    mscl::DeviceStatusMap status;

    if(m_config->m_inertial_device->features().supportedStatusSelectors().size() > 0)
    {
      mscl::DeviceStatusData statusData = m_config->m_inertial_device->getBasicDeviceStatus();
      status = statusData.asMap();
    }
    else
    {
      MICROSTRAIN_INFO(m_node, "Model Number: \t\t\t\t\t%s\n", m_config->m_inertial_device->modelNumber().c_str());
      return true;
    }
    
    mscl::DeviceStatusMap::iterator it;

    for(it = status.begin(); it != status.end(); it++ )
    {
      switch (it->first)
      {
      case mscl::DeviceStatusValues::ModelNumber:
        MICROSTRAIN_INFO(m_node, "Model Number: \t\t\t\t\t%s\n", (it->second).c_str());
        break;

      case mscl::DeviceStatusValues::StatusStructure_Value:
        MICROSTRAIN_INFO(m_node, "Status Selector: \t\t\t\t%s\n", (it->second).c_str());
        break;

      case mscl::DeviceStatusValues::SystemState_Value:
        MICROSTRAIN_INFO(m_node, "System state: \t\t\t\t\t%s\n", (it->second).c_str());
        break;

      default:
        break;
      }
    }
  }
  return true;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Diagnostic Status Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::get_diagnostic_report(TriggerServiceMsg::Request &req,
                                        TriggerServiceMsg::Response &res)
{
  res.success = false;

  if(!m_config->m_inertial_device)
  {    
    return false;
  }
  
  if(m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_DEVICE_STATUS))
  {
    mscl::DeviceStatusMap status;

    if(m_config->m_inertial_device->features().supportedStatusSelectors().size() > 1) 
    {
      mscl::DeviceStatusData statusData = m_config->m_inertial_device->getDiagnosticDeviceStatus();
      status = statusData.asMap();
      res.success = true;
    }
      
    else if(m_config->m_inertial_device->features().supportedStatusSelectors().size() > 0)
    {
      mscl::DeviceStatusData statusData = m_config->m_inertial_device->getBasicDeviceStatus();
      status = statusData.asMap();
      res.success = true;
    }
    else
    {
      MICROSTRAIN_INFO(m_node, "Model Number: \t\t\t\t\t%s\n", m_config->m_inertial_device->modelNumber().c_str());
      return true;
    }
    
    mscl::DeviceStatusMap::iterator it;

    for( it = status.begin(); it != status.end(); it++ )
    {
      switch(it->first)
      {
      case mscl::DeviceStatusValues::ModelNumber:
        MICROSTRAIN_INFO(m_node, "Model Number: \t\t\t\t\t%s\n", (it->second).c_str());
        break;

      case mscl::DeviceStatusValues::StatusStructure_Value:
        MICROSTRAIN_INFO(m_node, "Status Selector: \t\t\t\t%s\n", (it->second).c_str());
        break;

      case mscl::DeviceStatusValues::SystemState_Value:
        MICROSTRAIN_INFO(m_node, "System state: \t\t\t\t\t%s\n", (it->second).c_str());
        break;

      case mscl::DeviceStatusValues::ImuStreamInfo_Enabled:
        MICROSTRAIN_INFO(m_node, "IMU Streaming Enabled: \t\t\t\t%s\n", strcmp((it->second).c_str(),"1") == 0 ? "TRUE" : "FALSE");
        break;

      case mscl::DeviceStatusValues::ImuStreamInfo_PacketsDropped:
        MICROSTRAIN_INFO(m_node, "Number of Dropped IMU Packets: \t\t\t%s Packets\n", (it->second).c_str());
        break;

      case mscl::DeviceStatusValues::EstimationFilterStreamInfo_Enabled:
        MICROSTRAIN_INFO(m_node, "FILTER Streaming Enabled: \t\t\t%s\n", strcmp((it->second).c_str(),"1") == 0 ? "TRUE" : "FALSE");
        break;

      case mscl::DeviceStatusValues::EstimationFilterStreamInfo_PacketsDropped:
        MICROSTRAIN_INFO(m_node, "Number of Dropped FILTER Packets: \t\t%s Packets\n", (it->second).c_str());
        break;

      case mscl::DeviceStatusValues::ComPortInfo_BytesWritten:
        MICROSTRAIN_INFO(m_node, "Communications Port Bytes Written: \t\t%s Bytes\n", (it->second).c_str());
        break;

      case mscl::DeviceStatusValues::ComPortInfo_BytesRead:
        MICROSTRAIN_INFO(m_node, "Communications Port Bytes Read: \t\t%s Bytes\n", (it->second).c_str());
        break;

      case mscl::DeviceStatusValues::ComPortInfo_OverrunsOnWrite:
        MICROSTRAIN_INFO(m_node, "Communications Port Write Overruns: \t\t%s\n", (it->second).c_str());
        break;

      case mscl::DeviceStatusValues::ComPortInfo_OverrunsOnRead:
        MICROSTRAIN_INFO(m_node, "Communications Port Read Overruns: \t\t%s\n", (it->second).c_str());
        break;

      case mscl::DeviceStatusValues::ImuMessageInfo_MessageParsingErrors:
        MICROSTRAIN_INFO(m_node, "IMU Parser Errors: \t\t\t\t%s Errors\n", (it->second).c_str());
        break;

      case mscl::DeviceStatusValues::ImuMessageInfo_MessagesRead:
        MICROSTRAIN_INFO(m_node, "IMU Message Count: \t\t\t\t%s Messages\n", (it->second).c_str());
        break;

      case mscl::DeviceStatusValues::ImuMessageInfo_LastMessageReadinMS:
        MICROSTRAIN_INFO(m_node, "IMU Last Message Received: \t\t\t%s ms\n", (it->second).c_str());
        break;

      default:
        break;
      }
    }
  }
  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Auto Angular Zupt Enable/Threshold Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

// Set zero angular-rate update threshold
bool MicrostrainServices::set_zero_angle_update_threshold(SetZeroAngleUpdateThresholdServiceMsg::Request &req,
                                                  SetZeroAngleUpdateThresholdServiceMsg::Response &res)
{
  res.success = false;

  MICROSTRAIN_INFO(m_node, "Setting Zero Angular-Rate-Update threshold\n");

  if(m_config->m_inertial_device)
  {
    try
    {
      mscl::ZUPTSettingsData ZUPTSettings(req.enable, req.threshold);
      m_config->m_inertial_device->setAngularRateZUPT(ZUPTSettings);

      ZUPTSettings = m_config->m_inertial_device->getAngularRateZUPT();
      MICROSTRAIN_INFO(m_node, "Enable value set to: %d, Threshold is: %f rad/s",
               ZUPTSettings.enabled, ZUPTSettings.threshold);

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Auto Angular Zupt Enable/Threshold Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::get_zero_angle_update_threshold(GetZeroAngleUpdateThresholdServiceMsg::Request &req,
                                                  GetZeroAngleUpdateThresholdServiceMsg::Response &res)
{
  res.success = false;

  MICROSTRAIN_INFO(m_node, "Getting Zero Angular-Rate-Update threshold\n");

  if(m_config->m_inertial_device)
  {
    try
    {
      mscl::ZUPTSettingsData ZUPTSettings = m_config->m_inertial_device->getAngularRateZUPT();
      MICROSTRAIN_INFO(m_node, "Enable value set to: %d, Threshold is: %f rad/s",
               ZUPTSettings.enabled, ZUPTSettings.threshold);

      res.enable    = ZUPTSettings.enabled;
      res.threshold = ZUPTSettings.threshold;
      res.success   = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Auto Velocity Zupt Enable/Threshold Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::set_zero_velocity_update_threshold(SetZeroVelocityUpdateThresholdServiceMsg::Request &req,
                                                     SetZeroVelocityUpdateThresholdServiceMsg::Response &res)
{
  res.success = false;

  MICROSTRAIN_INFO(m_node, "Setting Zero Velocity-Update threshold\n");

  if(m_config->m_inertial_device)
  {
    try
    {
      mscl::ZUPTSettingsData ZUPTSettings(req.enable, req.threshold);
      m_config->m_inertial_device->setVelocityZUPT(ZUPTSettings);

      ZUPTSettings = m_config->m_inertial_device->getVelocityZUPT();
      MICROSTRAIN_INFO(m_node, "Enable value set to: %d, Threshold is: %f m/s",
               ZUPTSettings.enabled, ZUPTSettings.threshold);

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Auto Velocity Zupt Enable/Threshold Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::get_zero_velocity_update_threshold(GetZeroVelocityUpdateThresholdServiceMsg::Request &req,
                                                     GetZeroVelocityUpdateThresholdServiceMsg::Response &res)
{
  res.success = false;

  MICROSTRAIN_INFO(m_node, "Getting Zero Velocity-Update threshold\n");

  if(m_config->m_inertial_device)
  {
    try
    {
      mscl::ZUPTSettingsData ZUPTSettings = m_config->m_inertial_device->getVelocityZUPT();
      MICROSTRAIN_INFO(m_node, "Enable value set to: %d, Threshold is: %f rad/s",
               ZUPTSettings.enabled, ZUPTSettings.threshold);

      res.enable    = ZUPTSettings.enabled;
      res.threshold = ZUPTSettings.threshold;
      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Tare Orientation Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::set_tare_orientation(SetTareOrientationServiceMsg::Request &req,
                                       SetTareOrientationServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device)
  {
    try
    {
      mscl::TareAxisValues axisValue(req.axis & mscl::InertialTypes::TARE_PITCH_AXIS,
                                     req.axis & mscl::InertialTypes::TARE_ROLL_AXIS,
                                     req.axis & mscl::InertialTypes::TARE_YAW_AXIS);
      m_config->m_inertial_device->tareOrientation(axisValue);

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Accel Noise 1-Sigma Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::set_accel_noise(SetAccelNoiseServiceMsg::Request &req,
                                  SetAccelNoiseServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device)
  {
    try
    {
      MICROSTRAIN_INFO(m_node, "Setting the accel noise values\n");
      mscl::GeometricVector noise(req.noise.x, req.noise.y, req.noise.z);
      m_config->m_inertial_device->setAccelNoiseStandardDeviation(noise);

      noise = m_config->m_inertial_device->getAccelNoiseStandardDeviation();
      MICROSTRAIN_INFO(m_node, "Accel noise values successfully set.\n");
      MICROSTRAIN_INFO(m_node, "Returned values: %f X %f Y %f Z\n",
               noise.x(), noise.y(), noise.z());

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Accel Noise 1-Sigma Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::get_accel_noise(GetAccelNoiseServiceMsg::Request &req,
                                  GetAccelNoiseServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device)
  {
    try
    {
      MICROSTRAIN_INFO(m_node, "Getting the accel noise values\n");

      mscl::GeometricVector noise = m_config->m_inertial_device->getAccelNoiseStandardDeviation();
      MICROSTRAIN_INFO(m_node, "Returned values: %f X %f Y %f Z\n",
               noise.x(), noise.y(), noise.z());
      
      res.noise.x = noise.x();
      res.noise.y = noise.y();
      res.noise.z = noise.z();
      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Gyro Noise 1-Sigma Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::set_gyro_noise(SetGyroNoiseServiceMsg::Request &req,
                                 SetGyroNoiseServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device)
  {
    try
    {
      MICROSTRAIN_INFO(m_node, "Setting the gyro noise values\n");
      mscl::GeometricVector noise(req.noise.x, req.noise.y, req.noise.z);
      m_config->m_inertial_device->setGyroNoiseStandardDeviation(noise);

      noise = m_config->m_inertial_device->getGyroNoiseStandardDeviation();
      MICROSTRAIN_INFO(m_node, "Gyro noise values successfully set.\n");
      MICROSTRAIN_INFO(m_node, "Returned values: %f X %f Y %f Z\n",
               noise.x(), noise.y(), noise.z());

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Gyro Noise 1-Sigma Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::get_gyro_noise(GetGyroNoiseServiceMsg::Request &req,
                                 GetGyroNoiseServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device)
  {
    try
    {
      MICROSTRAIN_INFO(m_node, "Getting the gyro noise values\n");

      mscl::GeometricVector noise = m_config->m_inertial_device->getGyroNoiseStandardDeviation();
      MICROSTRAIN_INFO(m_node, "Gyro noise values: %f X %f Y %f Z\n",
               noise.x(), noise.y(), noise.z());

      res.noise.x = noise.x();
      res.noise.y = noise.y();
      res.noise.z = noise.z();
      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Magnetometer Noise 1-Sigma Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::set_mag_noise(SetMagNoiseServiceMsg::Request &req,
                                SetMagNoiseServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device)
  {
    try
    {
      MICROSTRAIN_INFO(m_node, "Setting the mag noise values\n");
      mscl::GeometricVector noise(req.noise.x, req.noise.y, req.noise.z);
      m_config->m_inertial_device->setHardIronOffsetProcessNoise(noise);

      noise = m_config->m_inertial_device->getHardIronOffsetProcessNoise();
      MICROSTRAIN_INFO(m_node, "Mag noise values successfully set.\n");
      MICROSTRAIN_INFO(m_node, "Returned values: %f X %f Y %f Z\n",
               noise.x(), noise.y(), noise.z());

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Magnetometer Noise 1-Sigma Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::get_mag_noise(GetMagNoiseServiceMsg::Request &req,
                                GetMagNoiseServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device)
  {
    try
    {
      MICROSTRAIN_INFO(m_node, "Getting the mag noise values\n");
      mscl::GeometricVector noise = m_config->m_inertial_device->getHardIronOffsetProcessNoise();
      MICROSTRAIN_INFO(m_node, "Returned values: %f X %f Y %f Z\n",
               noise.x(), noise.y(), noise.z());

      res.noise.x = noise.x();
      res.noise.y = noise.y();
      res.noise.z = noise.z();
      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Gyro Bias Model Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::set_gyro_bias_model(SetGyroBiasModelServiceMsg::Request &req,
                                      SetGyroBiasModelServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device)
  {
    try
    {
      MICROSTRAIN_INFO(m_node, "Setting the gyro bias model values\n");

      mscl::GeometricVectors collection;
      mscl::GeometricVector noise(req.noise_vector.x, req.noise_vector.y, req.noise_vector.z);
      collection.push_back(noise);

      mscl::GeometricVector beta_vector(req.beta_vector.x, req.beta_vector.y, req.beta_vector.z);
      collection.push_back(beta_vector);

      m_config->m_inertial_device->setGyroBiasModelParams(collection);

      collection = m_config->m_inertial_device->getGyroBiasModelParams();
      MICROSTRAIN_INFO(m_node, "Gyro bias model values successfully set.\n");
      MICROSTRAIN_INFO(m_node, "Returned values:  Beta: %f X %f Y %f Z, White Noise: %f X %f Y %f Z\n",
               collection[0].x(), collection[0].y(), collection[0].z(),
               collection[1].x(), collection[1].y(), collection[1].z());

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Gyro Bias Model Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::get_gyro_bias_model(GetGyroBiasModelServiceMsg::Request &req,
                                      GetGyroBiasModelServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device)
  {
    try
    {
      MICROSTRAIN_INFO(m_node, "Getting the gyro bias model values\n");
      mscl::GeometricVectors collection = m_config->m_inertial_device->getGyroBiasModelParams();
      MICROSTRAIN_INFO(m_node, "Gyro bias model values:  Beta: %f X %f Y %f Z, White Noise: %f X %f Y %f Z\n",
               collection[0].x(), collection[0].y(), collection[0].z(),
               collection[1].x(), collection[1].y(), collection[1].z());

      res.noise_vector.x = collection[0].x();
      res.noise_vector.y = collection[0].y();
      res.noise_vector.z = collection[0].z();
      res.beta_vector.x  = collection[1].x();
      res.beta_vector.y  = collection[1].y();
      res.beta_vector.z  = collection[1].z();

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Accel Bias Model Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::set_accel_bias_model(SetAccelBiasModelServiceMsg::Request &req,
                                       SetAccelBiasModelServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device)
  {
    try
    {
      MICROSTRAIN_INFO(m_node, "Setting the accel bias model values\n");
      mscl::GeometricVectors collection;
      mscl::GeometricVector noise(req.noise_vector.x, req.noise_vector.y, req.noise_vector.z);
      collection.push_back(noise);

      mscl::GeometricVector beta_vector(req.beta_vector.x, req.beta_vector.y, req.beta_vector.z);
      collection.push_back(beta_vector);
      m_config->m_inertial_device->setAccelBiasModelParams(collection);

      collection = m_config->m_inertial_device->getAccelBiasModelParams();
      MICROSTRAIN_INFO(m_node, "Accel bias model values successfully set.\n");
      MICROSTRAIN_INFO(m_node, "Returned values:  Beta: %f X %f Y %f Z, White Noise: %f X %f Y %f Z\n",
               collection[0].x(), collection[0].y(), collection[0].z(),
               collection[1].x(), collection[1].y(), collection[1].z());

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Accel Bias Model Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::get_accel_bias_model(GetAccelBiasModelServiceMsg::Request &req,
                                       GetAccelBiasModelServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device)
  {
    try
    {
      MICROSTRAIN_INFO(m_node, "Getting the accel bias model values\n");
      mscl::GeometricVectors collection = m_config->m_inertial_device->getAccelBiasModelParams();

      MICROSTRAIN_INFO(m_node, "Accel bias model values:  Beta: %f X %f Y %f Z, White Noise: %f X %f Y %f Z\n",
               collection[0].x(), collection[0].y(), collection[0].z(),
               collection[1].x(), collection[1].y(), collection[1].z());

      res.noise_vector.x = collection[0].x();
      res.noise_vector.y = collection[0].y();
      res.noise_vector.z = collection[0].z();
      res.beta_vector.x  = collection[1].x();
      res.beta_vector.y  = collection[1].y();
      res.beta_vector.z  = collection[1].z();

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Accel Magnitude Error Adaptive Measurement Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::set_gravity_adaptive_vals(SetGravityAdaptiveValsServiceMsg::Request &req,
                                            SetGravityAdaptiveValsServiceMsg::Response &res)
{
  res.success = false;
  
  if(m_config->m_inertial_device)
  {
    try
    {
      MICROSTRAIN_INFO(m_node, "Setting the accel magnitude error adaptive measurement values\n");

      mscl::AdaptiveMeasurementData adaptiveData;
      adaptiveData.mode                 = static_cast<mscl::InertialTypes::AdaptiveMeasurementMode>(req.enable);
      adaptiveData.lowPassFilterCutoff  = req.low_pass_cutoff;
      adaptiveData.lowLimit             = req.low_limit;
      adaptiveData.highLimit            = req.high_limit;
      adaptiveData.lowLimitUncertainty  = req.low_limit_1sigma;
      adaptiveData.highLimitUncertainty = req.high_limit_1sigma;
      adaptiveData.minUncertainty       = req.min_1sigma;

      m_config->m_inertial_device->setGravityErrorAdaptiveMeasurement(adaptiveData);

      adaptiveData = m_config->m_inertial_device->getGravityErrorAdaptiveMeasurement();
      MICROSTRAIN_INFO(m_node, "accel magnitude error adaptive measurement values successfully set.\n");
      MICROSTRAIN_INFO(m_node, "Returned values: Enable: %i, Parameters: %f %f %f %f %f %f",
               adaptiveData.mode, adaptiveData.lowPassFilterCutoff,
               adaptiveData.minUncertainty, adaptiveData.lowLimit,
               adaptiveData.highLimit, adaptiveData.lowLimitUncertainty,
               adaptiveData.highLimitUncertainty);

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Accel Magnitude Error Adaptive Measurement Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::get_gravity_adaptive_vals(GetGravityAdaptiveValsServiceMsg::Request &req,
                                            GetGravityAdaptiveValsServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device)
  {
    try
    {
      MICROSTRAIN_INFO(m_node, "Getting the accel magnitude error adaptive measurement values\n");

      mscl::AdaptiveMeasurementData adaptiveData = m_config->m_inertial_device->getGravityErrorAdaptiveMeasurement();
      MICROSTRAIN_INFO(m_node, "Accel magnitude error adaptive measurement values are: Enable: %i, Parameters: %f %f %f %f %f %f",
               adaptiveData.mode, adaptiveData.lowPassFilterCutoff,
               adaptiveData.minUncertainty, adaptiveData.lowLimit,
               adaptiveData.highLimit, adaptiveData.lowLimitUncertainty,
               adaptiveData.highLimitUncertainty);

      res.enable            = adaptiveData.mode;
      res.low_pass_cutoff   = adaptiveData.lowPassFilterCutoff;
      res.low_limit         = adaptiveData.lowLimit;
      res.high_limit        = adaptiveData.highLimit;
      res.low_limit_1sigma  = adaptiveData.lowLimitUncertainty;
      res.high_limit_1sigma = adaptiveData.highLimitUncertainty; 
      res.min_1sigma        = adaptiveData.minUncertainty;

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Magnetometer Magnitude Error Adaptive Measurement Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::set_mag_adaptive_vals(SetMagAdaptiveValsServiceMsg::Request &req,
                                        SetMagAdaptiveValsServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device)
  {
    try
    {
      MICROSTRAIN_INFO(m_node, "Setting the mag magnitude error adaptive measurement values\n");

      mscl::AdaptiveMeasurementData adaptiveData;
      adaptiveData.mode                 = static_cast<mscl::InertialTypes::AdaptiveMeasurementMode>(req.enable);
      adaptiveData.lowPassFilterCutoff  = req.low_pass_cutoff;
      adaptiveData.lowLimit             = req.low_limit;
      adaptiveData.highLimit            = req.high_limit;
      adaptiveData.lowLimitUncertainty  = req.low_limit_1sigma;
      adaptiveData.highLimitUncertainty = req.high_limit_1sigma;
      adaptiveData.minUncertainty       = req.min_1sigma;

      m_config->m_inertial_device->setMagnetometerErrorAdaptiveMeasurement(adaptiveData);

      adaptiveData = m_config->m_inertial_device->getMagnetometerErrorAdaptiveMeasurement();
      MICROSTRAIN_INFO(m_node, "mag magnitude error adaptive measurement values successfully set.\n");

      MICROSTRAIN_INFO(m_node, "Returned values: Enable: %i, Parameters: %f %f %f %f %f %f",
               adaptiveData.mode, adaptiveData.lowPassFilterCutoff,
               adaptiveData.minUncertainty, adaptiveData.lowLimit,
               adaptiveData.highLimit, adaptiveData.lowLimitUncertainty,
               adaptiveData.highLimitUncertainty);

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Magnetometer Magnitude Error Adaptive Measurement Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::get_mag_adaptive_vals(GetMagAdaptiveValsServiceMsg::Request &req,
                                        GetMagAdaptiveValsServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device)
  {
    try
    {
      MICROSTRAIN_INFO(m_node, "Getting the mag magnitude error adaptive measurement values\n");

      mscl::AdaptiveMeasurementData adaptiveData = m_config->m_inertial_device->getMagnetometerErrorAdaptiveMeasurement();
      MICROSTRAIN_INFO(m_node, "Mag magnitude error adaptive measurement values are: Enable: %i, Parameters: %f %f %f %f %f %f",
               adaptiveData.mode, adaptiveData.lowPassFilterCutoff, adaptiveData.minUncertainty, adaptiveData.lowLimit,
               adaptiveData.highLimit, adaptiveData.lowLimitUncertainty, adaptiveData.highLimitUncertainty);

      res.enable            = adaptiveData.mode;
      res.low_pass_cutoff   = adaptiveData.lowPassFilterCutoff;
      res.low_limit         = adaptiveData.lowLimit;
      res.high_limit        = adaptiveData.highLimit;
      res.low_limit_1sigma  = adaptiveData.lowLimitUncertainty;
      res.high_limit_1sigma = adaptiveData.highLimitUncertainty; 
      res.min_1sigma        = adaptiveData.minUncertainty;

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Magnetometer Dip Angle Error Adaptive Measurement Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::set_mag_dip_adaptive_vals(SetMagDipAdaptiveValsServiceMsg::Request &req,
                                            SetMagDipAdaptiveValsServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device)
  {
    try
    {
      MICROSTRAIN_INFO(m_node, "Setting the mag dip angle error adaptive measurement values\n");

      mscl::AdaptiveMeasurementData adaptiveData;
      adaptiveData.mode                 = static_cast<mscl::InertialTypes::AdaptiveMeasurementMode>(req.enable);
      adaptiveData.lowPassFilterCutoff  = req.low_pass_cutoff;
      adaptiveData.highLimit            = req.high_limit;
      adaptiveData.highLimitUncertainty = req.high_limit_1sigma;
      adaptiveData.minUncertainty       = req.min_1sigma;

      m_config->m_inertial_device->setMagDipAngleErrorAdaptiveMeasurement(adaptiveData);

      adaptiveData = m_config->m_inertial_device->getMagDipAngleErrorAdaptiveMeasurement();
      MICROSTRAIN_INFO(m_node, "mag dip angle error adaptive measurement values successfully set.\n");
      MICROSTRAIN_INFO(m_node, "Returned values: Enable: %i, Parameters: %f %f %f %f\n",
               adaptiveData.mode, adaptiveData.lowPassFilterCutoff,
               adaptiveData.minUncertainty, adaptiveData.highLimit, 
               adaptiveData.highLimitUncertainty);

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Magnetometer Dip Angle Error Adaptive Measurement Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::get_mag_dip_adaptive_vals(GetMagDipAdaptiveValsServiceMsg::Request &req,
                                            GetMagDipAdaptiveValsServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device)
  {
    try
    {
      MICROSTRAIN_INFO(m_node, "Getting the mag dip angle error adaptive measurement values\n");

      mscl::AdaptiveMeasurementData adaptiveData = m_config->m_inertial_device->getMagDipAngleErrorAdaptiveMeasurement();
      MICROSTRAIN_INFO(m_node, "Returned values: Enable: %i, Parameters: %f %f %f %f %f %f",
               adaptiveData.mode, adaptiveData.lowPassFilterCutoff,
               adaptiveData.minUncertainty, adaptiveData.lowLimit,
               adaptiveData.highLimit, adaptiveData.lowLimitUncertainty,
               adaptiveData.highLimitUncertainty);

      res.enable            = adaptiveData.mode;
      res.low_pass_cutoff   = adaptiveData.lowPassFilterCutoff;
      res.high_limit        = adaptiveData.highLimit;
      res.high_limit_1sigma = adaptiveData.highLimitUncertainty; 
      res.min_1sigma        = adaptiveData.minUncertainty;

      res.success = true;
    }
    catch (mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Vehicle Dynamics Mode Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::set_dynamics_mode(SetDynamicsModeServiceMsg::Request &req,
                                    SetDynamicsModeServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device)
  {
    try
    {
      MICROSTRAIN_INFO(m_node, "Setting the vehicle dynamics mode\n");

      mscl::InertialTypes::VehicleModeType mode = static_cast<mscl::InertialTypes::VehicleModeType>(req.mode);
      m_config->m_inertial_device->setVehicleDynamicsMode(mode);

      mode = m_config->m_inertial_device->getVehicleDynamicsMode();

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Vehicle Dynamics Mode Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::get_dynamics_mode(GetDynamicsModeServiceMsg::Request &req,
                                    GetDynamicsModeServiceMsg::Response &res)
{
  res.success = false;

  MICROSTRAIN_INFO(m_node, "Getting the vehicle dynamics mode\n");

  if(m_config->m_inertial_device)
  {
    try
    {
      mscl::InertialTypes::VehicleModeType mode = m_config->m_inertial_device->getVehicleDynamicsMode();
      MICROSTRAIN_INFO(m_node, "Vehicle dynamics mode is: %d\n", mode);

      res.mode    = mode;
      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}
  

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Change Device Settings
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::device_settings(DeviceSettingsServiceMsg::Request &req, DeviceSettingsServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device)
  {
    try
    {
      switch(req.function_selector)
      {
        //Save
        case 3:
        {
          MICROSTRAIN_INFO(m_node, "Processing device settings command with function selector = 3 (Save)\n");
          m_config->m_inertial_device->saveSettingsAsStartup();
        }break;
        
        //Load Saved Settings
        case 4:
        {
          MICROSTRAIN_INFO(m_node, "Processing device settings command with function selector = 4 (Load Saved Settings)\n");
          m_config->m_inertial_device->loadStartupSettings();
        }break;

        //Load Default Settings
        case 5:
        {
          MICROSTRAIN_INFO(m_node, "Processing device settings command with function selector = 5 (Load Defailt Settings)\n");
          m_config->m_inertial_device->loadFactoryDefaultSettings();
        }break;

        //Unsupported function selector
        default: 
        {
          MICROSTRAIN_INFO(m_node, "Error: Unsupported function selector for device settings command\n");
          return res.success;
        }break;
      }
 
      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Commanded Velocity Zupt Service
/////////////////////////////////////////////////////////////////////////////////////////////////////
 
bool MicrostrainServices::commanded_vel_zupt(TriggerServiceMsg::Request &req, TriggerServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device && m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_CMDED_ZERO_VEL_UPDATE))
  {
    try
    {
      m_config->m_inertial_device->cmdedVelZUPT();
      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}
  

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Commanded Angular Rate Zupt Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::commanded_ang_rate_zupt(TriggerServiceMsg::Request &req, TriggerServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device && m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_CMDED_ZERO_ANG_RATE_UPDATE))
  {
    try
    {
      m_config->m_inertial_device->cmdedAngRateZUPT();
      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// External Heading Update Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::external_heading_update(ExternalHeadingUpdateServiceMsg::Request &req,
                                          ExternalHeadingUpdateServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device)
  {
    try
    {
      mscl::HeadingData heading_data;

      heading_data.headingAngle            = req.heading_rad;
      heading_data.headingAngleUncertainty = req.heading_1sigma_rad;
      heading_data.heading                 = (mscl::HeadingData::HeadingType)req.heading_type;

      mscl::TimeUpdate timestamp(req.gps_tow, req.gps_week_number);

      if(req.use_time)
      {
        m_config->m_inertial_device->sendExternalHeadingUpdate(heading_data, timestamp);
        MICROSTRAIN_INFO(m_node, "Sent External Heading update with timestamp.\n");
      }
      else
      {
        m_config->m_inertial_device->sendExternalHeadingUpdate(heading_data);
        MICROSTRAIN_INFO(m_node, "Sent External Heading update.\n");
      }      
       
      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Relative Position Reference Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::set_relative_position_reference(SetRelativePositionReferenceServiceMsg::Request &req, 
                                                  SetRelativePositionReferenceServiceMsg::Response &res)
{
  res.success = false;

  if(m_config->m_inertial_device)
  {
    try
    {
      mscl::PositionReferenceConfiguration ref;

      ref.position   = mscl::Position(req.position.x, req.position.y, req.position.z, static_cast<mscl::PositionVelocityReferenceFrame>(req.frame));
      ref.autoConfig = !((bool)req.source);

      m_config->m_inertial_device->setRelativePositionReference(ref);
 
      if(req.source == 0)
        MICROSTRAIN_INFO(m_node, "Setting reference position to RTK base station (automatic)");
      else
        MICROSTRAIN_INFO(m_node, "Setting reference position to: [%f, %f, %f], ref frame = %d", req.position.x, req.position.y, req.position.z, req.frame);
      
      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Relative Position Reference Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::get_relative_position_reference(GetRelativePositionReferenceServiceMsg::Request &req, 
                                                  GetRelativePositionReferenceServiceMsg::Response &res)
{
  res.success = false; 

  if(m_config->m_inertial_device)
  {
    try
    {
      mscl::PositionReferenceConfiguration ref = m_config->m_inertial_device->getRelativePositionReference();

      if(ref.autoConfig)
        MICROSTRAIN_INFO(m_node, "Reference position is set to RTK base station (automatic)");
      else
        MICROSTRAIN_INFO(m_node, "Reference position is: [%f, %f, %f], ref frame = %d", ref.position.x(), ref.position.y(), ref.position.z(), (int)ref.position.referenceFrame);
      
      res.source     = !ref.autoConfig;
      res.frame      = (int)ref.position.referenceFrame;
      res.position.x = ref.position.x();
      res.position.y = ref.position.y();
      res.position.z = ref.position.z();
       
      res.success = true;
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }

  return res.success;
}

} // namespace Microstrain
