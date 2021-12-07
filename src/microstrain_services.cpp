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
#include <memory>
#include "microstrain_inertial_driver_common/microstrain_services.h"

namespace microstrain
{
MicrostrainServices::MicrostrainServices(RosNodeType* node, MicrostrainConfig* config) : node_(node), config_(config)
{
}

bool MicrostrainServices::configure()
{
  if (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_DEVICE_STATUS))
  {
    get_basic_status_service_ =
        create_service<TriggerServiceMsg>(node_, "get_basic_status", &MicrostrainServices::getBasicStatus, this);
    get_diagnostic_report_service_ = create_service<TriggerServiceMsg>(
        node_, "get_diagnostic_report", &MicrostrainServices::getDiagnosticReport, this);
  }

  if (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_GET_DEVICE_INFO))
  {
    device_report_service_ =
        create_service<DeviceReportServiceMsg>(node_, "device_report", &MicrostrainServices::deviceReport, this);
  }

  // IMU tare orientation service
  if (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_TARE_ORIENT))
  {
    set_tare_orientation_service_ = create_service<SetTareOrientationServiceMsg>(
        node_, "set_tare_orientation", &MicrostrainServices::setTareOrientation, this);
  }

  // IMU Complementary filter service
  if (config_->inertial_device_->features().supportsCommand(
          mscl::MipTypes::Command::CMD_COMPLEMENTARY_FILTER_SETTINGS))
  {
    set_complementary_filter_service_ = create_service<SetComplementaryFilterServiceMsg>(
        node_, "set_complementary_filter", &MicrostrainServices::setComplementaryFilter, this);
    get_complementary_filter_service_ = create_service<GetComplementaryFilterServiceMsg>(
        node_, "get_complementary_filter", &MicrostrainServices::getComplementaryFilter, this);
  }

  // IMU sensor2vehicle frame rotation service
  if (config_->inertial_device_->features().supportsCommand(
          mscl::MipTypes::Command::CMD_EF_SENS_VEHIC_FRAME_ROTATION_EULER))
  {
    set_sensor2vehicle_rotation_service_ = create_service<SetSensor2VehicleRotationServiceMsg>(
        node_, "set_sensor2vehicle_rotation", &MicrostrainServices::setSensor2vehicleRotation, this);
    get_sensor2vehicle_rotation_service_ = create_service<GetSensor2VehicleRotationServiceMsg>(
        node_, "get_sensor2vehicle_rotation", &MicrostrainServices::getSensor2vehicleRotation, this);
  }

  // IMU sensor2vehicle frame offset service
  if (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SENS_VEHIC_FRAME_OFFSET))
  {
    set_sensor2vehicle_offset_service_ = create_service<SetSensor2VehicleOffsetServiceMsg>(
        node_, "set_sensor2vehicle_offset", &MicrostrainServices::setSensor2vehicleOffset, this);
    get_sensor2vehicle_offset_service_ = create_service<GetSensor2VehicleOffsetServiceMsg>(
        node_, "get_sensor2vehicle_offset", &MicrostrainServices::getSensor2vehicleOffset, this);
  }

  // IMU sensor2vehicle transformation service
  if (config_->inertial_device_->features().supportsCommand(
          mscl::MipTypes::Command::CMD_EF_SENS_VEHIC_FRAME_OFFSET) &&
      config_->inertial_device_->features().supportsCommand(
          mscl::MipTypes::Command::CMD_EF_SENS_VEHIC_FRAME_ROTATION_EULER))
  {
    get_sensor2vehicle_transformation_service_ = create_service<GetSensor2VehicleTransformationServiceMsg>(
        node_, "get_sensor2vehicle_transformation", &MicrostrainServices::getSensor2vehicleTransformation, this);
  }

  // IMU Accel bias service
  if (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_ACCEL_BIAS))
  {
    set_accel_bias_service_ =
        create_service<SetAccelBiasServiceMsg>(node_, "set_accel_bias", &MicrostrainServices::setAccelBias, this);
    get_accel_bias_service_ =
        create_service<GetAccelBiasServiceMsg>(node_, "get_accel_bias", &MicrostrainServices::getAccelBias, this);
  }

  // IMU gyro bias service
  if (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_GYRO_BIAS))
  {
    set_gyro_bias_service_ =
        create_service<SetGyroBiasServiceMsg>(node_, "set_gyro_bias", &MicrostrainServices::setGyroBias, this);
    get_gyro_bias_service_ =
        create_service<GetGyroBiasServiceMsg>(node_, "get_gyro_bias", &MicrostrainServices::getGyroBias, this);
  }

  // IMU Gyro bias capture service
  if (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_CAP_GYRO_BIAS))
  {
    gyro_bias_capture_service_ =
        create_service<TriggerServiceMsg>(node_, "gyro_bias_capture", &MicrostrainServices::gyroBiasCapture, this);
  }

  // IMU Mag Hard iron offset service
  if (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_MAG_HARD_IRON_OFFSET))
  {
    set_hard_iron_values_service_ = create_service<SetHardIronValuesServiceMsg>(
        node_, "set_hard_iron_values", &MicrostrainServices::setHardIronValues, this);
    get_hard_iron_values_service_ = create_service<GetHardIronValuesServiceMsg>(
        node_, "get_hard_iron_values", &MicrostrainServices::getHardIronValues, this);
  }

  // IMU Mag Soft iron matrix service
  if (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_MAG_SOFT_IRON_MATRIX))
  {
    set_soft_iron_matrix_service_ = create_service<SetSoftIronMatrixServiceMsg>(
        node_, "set_soft_iron_matrix", &MicrostrainServices::setSoftIronMatrix, this);
    get_soft_iron_matrix_service_ = create_service<GetSoftIronMatrixServiceMsg>(
        node_, "get_soft_iron_matrix", &MicrostrainServices::getSoftIronMatrix, this);
  }

  // IMU Coning and sculling enable service
  if (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_CONING_SCULLING))
  {
    set_coning_sculling_comp_service_ = create_service<SetConingScullingCompServiceMsg>(
        node_, "set_coning_sculling_comp", &MicrostrainServices::setConingScullingComp, this);
    get_coning_sculling_comp_service_ = create_service<GetConingScullingCompServiceMsg>(
        node_, "get_coning_sculling_comp", &MicrostrainServices::getConingScullingComp, this);
  }

  // Kalman filter reset
  if (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_RESET_FILTER))
  {
    reset_filter_service_ =
        create_service<EmptyServiceMsg>(node_, "reset_kf", &MicrostrainServices::resetFilter, this);
  }

  // Kalman filter estimation control service
  if (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_BIAS_EST_CTRL))
  {
    set_estimation_control_flags_service_ = create_service<SetEstimationControlFlagsServiceMsg>(
        node_, "set_estimation_control_flags", &MicrostrainServices::setEstimationControlFlags, this);
    get_estimation_control_flags_service_ = create_service<GetEstimationControlFlagsServiceMsg>(
        node_, "get_estimation_control_flags", &MicrostrainServices::getEstimationControlFlags, this);
  }

  // Kalman filter initialization with full Euler angles service
  if (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_INIT_ATTITUDE))
  {
    init_filter_euler_service_ = create_service<InitFilterEulerServiceMsg>(
        node_, "init_filter_euler", &MicrostrainServices::initFilterEuler, this);
  }

  // Kalman filter initialization with heading only service
  if (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_INIT_HEADING))
  {
    init_filter_heading_service_ = create_service<InitFilterHeadingServiceMsg>(
        node_, "init_filter_heading", &MicrostrainServices::initFilterHeading, this);
  }

  // Kalman filter heading source service
  if (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_HEADING_UPDATE_CTRL))
  {
    set_heading_source_service_ = create_service<SetHeadingSourceServiceMsg>(
        node_, "set_heading_source", &MicrostrainServices::setHeadingSource, this);
    get_heading_source_service_ = create_service<GetHeadingSourceServiceMsg>(
        node_, "get_heading_source", &MicrostrainServices::getHeadingSource, this);
  }

  // Kalman filter commanded ZUPT service
  if (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_CMDED_ZERO_VEL_UPDATE))
  {
    commanded_vel_zupt_service_ =
        create_service<TriggerServiceMsg>(node_, "commanded_vel_zupt", &MicrostrainServices::commandedVelZupt, this);
  }

  // Kalman filter commanded angular ZUPT service
  if (config_->inertial_device_->features().supportsCommand(
          mscl::MipTypes::Command::CMD_EF_CMDED_ZERO_ANG_RATE_UPDATE))
  {
    commanded_ang_rate_zupt_service_ = create_service<TriggerServiceMsg>(
        node_, "commanded_ang_rate_zupt", &MicrostrainServices::commandedAngRateZupt, this);
  }

  // Kalman filter Accel white noise 1-sigma service
  if (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_ACCEL_WHT_NSE_STD_DEV))
  {
    set_accel_noise_service_ =
        create_service<SetAccelNoiseServiceMsg>(node_, "set_accel_noise", &MicrostrainServices::setAccelNoise, this);
    get_accel_noise_service_ =
        create_service<GetAccelNoiseServiceMsg>(node_, "get_accel_noise", &MicrostrainServices::getAccelNoise, this);
  }

  // Kalman filter Gyro white noise 1-sigma service
  if (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_GYRO_WHT_NSE_STD_DEV))
  {
    set_gyro_noise_service_ =
        create_service<SetGyroNoiseServiceMsg>(node_, "set_gyro_noise", &MicrostrainServices::setGyroNoise, this);
    get_gyro_noise_service_ =
        create_service<GetGyroNoiseServiceMsg>(node_, "get_gyro_noise", &MicrostrainServices::getGyroNoise, this);
  }

  // Kalman filter Mag noise 1-sigma service
  if (config_->inertial_device_->features().supportsCommand(
          mscl::MipTypes::Command::CMD_EF_HARD_IRON_OFFSET_PROCESS_NOISE))
  {
    set_mag_noise_service_ =
        create_service<SetMagNoiseServiceMsg>(node_, "set_mag_noise", &MicrostrainServices::setMagNoise, this);
    get_mag_noise_service_ =
        create_service<GetMagNoiseServiceMsg>(node_, "get_mag_noise", &MicrostrainServices::getMagNoise, this);
  }

  // Kalman filter accel bias model service
  if (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_ACCEL_BIAS_MODEL_PARAMS))
  {
    set_accel_bias_model_service_ = create_service<SetAccelBiasModelServiceMsg>(
        node_, "set_accel_bias_model", &MicrostrainServices::setAccelBiasModel, this);
    get_accel_bias_model_service_ = create_service<GetAccelBiasModelServiceMsg>(
        node_, "get_accel_bias_model", &MicrostrainServices::getAccelBiasModel, this);
  }

  // Kalman filter gyro bias model service
  if (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_GYRO_BIAS_MODEL_PARAMS))
  {
    set_gyro_bias_model_service_ = create_service<SetGyroBiasModelServiceMsg>(
        node_, "set_gyro_bias_model", &MicrostrainServices::setGyroBiasModel, this);
    get_gyro_bias_model_service_ = create_service<GetGyroBiasModelServiceMsg>(
        node_, "get_gyro_bias_model", &MicrostrainServices::getGyroBiasModel, this);
  }

  // Kalman filter magnetometer magnitude adaptive filter service
  if (config_->inertial_device_->features().supportsCommand(
          mscl::MipTypes::Command::CMD_EF_MAG_MAGNITUDE_ERR_ADAPT_MEASURE))
  {
    set_mag_adaptive_vals_service_ = create_service<SetMagAdaptiveValsServiceMsg>(
        node_, "set_mag_adaptive_vals", &MicrostrainServices::setMagAdaptiveVals, this);
    get_mag_adaptive_vals_service_ = create_service<GetMagAdaptiveValsServiceMsg>(
        node_, "get_mag_adaptive_vals", &MicrostrainServices::getMagAdaptiveVals, this);
  }

  // Kalman filter magnetometer dip angle adaptive filter service
  if (config_->inertial_device_->features().supportsCommand(
          mscl::MipTypes::Command::CMD_EF_MAG_DIP_ANGLE_ERR_ADAPT_MEASURE))
  {
    set_mag_dip_adaptive_vals_service_ = create_service<SetMagDipAdaptiveValsServiceMsg>(
        node_, "set_mag_dip_adaptive_vals", &MicrostrainServices::setMagDipAdaptiveVals, this);
    get_mag_dip_adaptive_vals_service_ = create_service<GetMagDipAdaptiveValsServiceMsg>(
        node_, "get_mag_dip_adaptive_vals", &MicrostrainServices::getMagDipAdaptiveVals, this);
  }

  // Kalman filter gravity adaptive filtering settings service
  if (config_->inertial_device_->features().supportsCommand(
          mscl::MipTypes::Command::CMD_EF_GRAV_MAGNITUDE_ERR_ADAPT_MEASURE))
  {
    set_gravity_adaptive_vals_service_ = create_service<SetGravityAdaptiveValsServiceMsg>(
        node_, "set_gravity_adaptive_vals", &MicrostrainServices::setGravityAdaptiveVals, this);
    get_gravity_adaptive_vals_service_ = create_service<GetGravityAdaptiveValsServiceMsg>(
        node_, "get_gravity_adaptive_vals", &MicrostrainServices::getGravityAdaptiveVals, this);
  }

  // Kalman filter automatic angular ZUPT configuration service
  if (config_->inertial_device_->features().supportsCommand(
          mscl::MipTypes::Command::CMD_EF_ZERO_ANG_RATE_UPDATE_CTRL))
  {
    set_zero_angle_update_threshold_service_ = create_service<SetZeroAngleUpdateThresholdServiceMsg>(
        node_, "set_zero_angle_update_threshold", &MicrostrainServices::setZeroAngleUpdateThreshold, this);
    get_zero_angle_update_threshold_service_ = create_service<GetZeroAngleUpdateThresholdServiceMsg>(
        node_, "get_zero_angle_update_threshold", &MicrostrainServices::getZeroAngleUpdateThreshold, this);
  }

  // Kalman Filter automatic ZUPT configuration service
  if (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_ZERO_VEL_UPDATE_CTRL))
  {
    set_zero_velocity_update_threshold_service_ = create_service<SetZeroVelocityUpdateThresholdServiceMsg>(
        node_, "set_zero_velocity_update_threshold", &MicrostrainServices::setZeroVelocityUpdateThreshold, this);
    get_zero_velocity_update_threshold_service_ = create_service<GetZeroVelocityUpdateThresholdServiceMsg>(
        node_, "get_zero_velocity_update_threshold", &MicrostrainServices::getZeroVelocityUpdateThreshold, this);
  }

  // Reference position service
  if (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SET_REF_POSITION))
  {
    set_reference_position_service_ = create_service<SetReferencePositionServiceMsg>(
        node_, "set_reference_position", &MicrostrainServices::setReferencePosition, this);
    get_reference_position_service_ = create_service<GetReferencePositionServiceMsg>(
        node_, "get_reference_position", &MicrostrainServices::getReferencePosition, this);
  }

  // Dynamics mode service
  if (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_VEHIC_DYNAMICS_MODE))
  {
    set_dynamics_mode_service_ = create_service<SetDynamicsModeServiceMsg>(
        node_, "set_dynamics_mode", &MicrostrainServices::setDynamicsMode, this);
    get_dynamics_mode_service_ = create_service<GetDynamicsModeServiceMsg>(
        node_, "get_dynamics_mode", &MicrostrainServices::getDynamicsMode, this);
  }

  // Device Settings Service
  if (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_SAVE_STARTUP_SETTINGS))
  {
    device_settings_service_ = create_service<DeviceSettingsServiceMsg>(node_, "device_settings",
                                                                         &MicrostrainServices::deviceSettings, this);
  }

  // External Heading Service
  if ((config_->inertial_device_->features().supportsCommand(
          mscl::MipTypes::Command::CMD_EF_EXTERN_HEADING_UPDATE)) ||
      (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_EXT_HEADING_UPDATE_TS)))
  {
    external_heading_service_ = create_service<ExternalHeadingUpdateServiceMsg>(
        node_, "external_heading", &MicrostrainServices::externalHeadingUpdate, this);
  }

  // Relative Position Reference Service
  if (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_EXT_HEADING_UPDATE_TS))
  {
    set_relative_position_reference_service_ = create_service<SetRelativePositionReferenceServiceMsg>(
        node_, "set_relative_position_reference", &MicrostrainServices::setRelativePositionReference, this);
    get_relative_position_reference_service_ = create_service<GetRelativePositionReferenceServiceMsg>(
        node_, "get_relative_position_reference", &MicrostrainServices::getRelativePositionReference, this);
  }

  // Filter Speed Lever Arm Service
  if (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SPEED_MEASUREMENT_OFFSET))
  {
    set_filter_speed_lever_arm_service_ = create_service<SetFilterSpeedLeverArmServiceMsg>(
        node_, "set_filter_speed_lever_arm", &MicrostrainServices::setFilterSpeedLeverArm, this);
  }

  return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Device Report
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::deviceReport(DeviceReportServiceMsg::Request& req, DeviceReportServiceMsg::Response& res)
{
  res.success = false;
  if (config_->inertial_device_)
  {
    try
    {
      res.model_name = config_->inertial_device_->modelName();
      res.model_number = config_->inertial_device_->modelNumber();
      res.serial_number = config_->inertial_device_->serialNumber();
      res.options = config_->inertial_device_->deviceOptions();
      res.firmware_version = config_->inertial_device_->firmwareVersion().str();
      MICROSTRAIN_DEBUG(node_, "Model Name       => %s\n", res.model_name.c_str());
      MICROSTRAIN_DEBUG(node_, "Model Number     => %s\n", res.model_number.c_str());
      MICROSTRAIN_DEBUG(node_, "Serial Number    => %s\n", res.serial_number.c_str());
      MICROSTRAIN_DEBUG(node_, "Options          => %s\n", res.options.c_str());
      MICROSTRAIN_DEBUG(node_, "Firmware Version => %s\n\n",
                       res.firmware_version.c_str());
      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
      res.success = false;
    }
  }
  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Reset Filter Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::resetFilter(EmptyServiceMsg::Request& req, EmptyServiceMsg::Response& res)
{
  MICROSTRAIN_INFO(node_, "Resetting filter\n");

  if (config_->inertial_device_)
  {
    try
    {
      config_->inertial_device_->resetFilter();
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialize Filter (Euler Angles) Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::initFilterEuler(InitFilterEulerServiceMsg::Request& req,
                                            InitFilterEulerServiceMsg::Response& res)
{
  res.success = false;
  MICROSTRAIN_INFO(node_, "Initializing the Filter with Euler angles\n");

  if (config_->inertial_device_)
  {
    try
    {
      mscl::EulerAngles attitude(req.angle.x, req.angle.y, req.angle.z);

      config_->inertial_device_->setInitialAttitude(attitude);
      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialize Filter (Heading Angle) Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::initFilterHeading(InitFilterHeadingServiceMsg::Request& req,
                                              InitFilterHeadingServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      MICROSTRAIN_INFO(node_, "Initializing the Filter with a heading angle\n");
      config_->inertial_device_->setInitialHeading(req.angle);

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Accel Bias Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::setAccelBias(SetAccelBiasServiceMsg::Request& req, SetAccelBiasServiceMsg::Response& res)
{
  res.success = false;
  MICROSTRAIN_INFO(node_, "Setting accel bias values");

  if (config_->inertial_device_)
  {
    try
    {
      mscl::GeometricVector biasVector = config_->inertial_device_->getAccelerometerBias();

      MICROSTRAIN_INFO(node_, "Accel bias vector values are: %f %f %f", biasVector.x(), biasVector.y(),
                       biasVector.z());
      MICROSTRAIN_INFO(node_, "Client request values are: %.2f %.2f %.2f", req.bias.x, req.bias.y, req.bias.z);

      biasVector.x(req.bias.x);
      biasVector.y(req.bias.y);
      biasVector.z(req.bias.z);

      config_->inertial_device_->setAccelerometerBias(biasVector);

      MICROSTRAIN_INFO(node_, "New accel bias vector values are: %.2f %.2f %.2f", biasVector.x(), biasVector.y(),
                       biasVector.z());

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Accel Bias Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::getAccelBias(GetAccelBiasServiceMsg::Request& req, GetAccelBiasServiceMsg::Response& res)
{
  res.success = false;
  MICROSTRAIN_INFO(node_, "Getting accel bias values\n");

  if (config_->inertial_device_)
  {
    try
    {
      mscl::GeometricVector biasVector = config_->inertial_device_->getAccelerometerBias();

      MICROSTRAIN_INFO(node_, "Accel bias vector values are: %f %f %f.\n", biasVector.x(), biasVector.y(),
                       biasVector.z());

      res.bias.x = biasVector.x();
      res.bias.y = biasVector.y();
      res.bias.z = biasVector.z();

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }
  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Gyro Bias Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::setGyroBias(SetGyroBiasServiceMsg::Request& req, SetGyroBiasServiceMsg::Response& res)
{
  res.success = false;
  MICROSTRAIN_INFO(node_, "Setting gyro bias values");

  if (config_->inertial_device_)
  {
    try
    {
      mscl::GeometricVector biasVector = config_->inertial_device_->getGyroBias();

      MICROSTRAIN_INFO(node_, "Gyro bias vector values are: %f %f %f", biasVector.x(), biasVector.y(), biasVector.z());

      MICROSTRAIN_INFO(node_, "Client request values are: %.2f %.2f %.2f", req.bias.x, req.bias.y, req.bias.z);

      biasVector.x(req.bias.x);
      biasVector.y(req.bias.y);
      biasVector.z(req.bias.z);

      config_->inertial_device_->setGyroBias(biasVector);

      MICROSTRAIN_INFO(node_, "New gyro bias vector values are: %.2f %.2f %.2f", biasVector.x(), biasVector.y(),
                       biasVector.z());

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Gyro Bias Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::getGyroBias(GetGyroBiasServiceMsg::Request& req, GetGyroBiasServiceMsg::Response& res)
{
  res.success = false;
  MICROSTRAIN_INFO(node_, "Getting gyro bias values");
  if (config_->inertial_device_)
  {
    try
    {
      mscl::GeometricVector biasVector = config_->inertial_device_->getGyroBias();

      MICROSTRAIN_INFO(node_, "Gyro bias vector values are: %f %f %f", biasVector.x(), biasVector.y(), biasVector.z());

      res.bias.x = biasVector.x();
      res.bias.y = biasVector.y();
      res.bias.z = biasVector.z();

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Gyro Bias Capture Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::gyroBiasCapture(TriggerServiceMsg::Request& req, TriggerServiceMsg::Response& res)
{
  res.success = false;
  MICROSTRAIN_INFO(node_,
                   "Performing Gyro Bias capture.\nPlease keep device stationary during the 10 second gyro bias "
                   "capture interval\n");

  if (config_->inertial_device_)
  {
    try
    {
      mscl::GeometricVector biasVector = config_->inertial_device_->captureGyroBias(10000);

      MICROSTRAIN_INFO(node_, "Gyro Bias Captured:\nbias_vector[0] = %f\nbias_vector[1] = %f\nbias_vector[2] = %f\n\n",
                       biasVector.x(), biasVector.y(), biasVector.z());

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Hard Iron Offset Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::setHardIronValues(SetHardIronValuesServiceMsg::Request& req,
                                               SetHardIronValuesServiceMsg::Response& res)
{
  res.success = false;
  MICROSTRAIN_INFO(node_, "Setting hard iron values");

  if (config_->inertial_device_)
  {
    try
    {
      mscl::GeometricVector biasVector = config_->inertial_device_->getMagnetometerHardIronOffset();

      MICROSTRAIN_INFO(node_, "Hard Iron vector values are: %f %f %f", biasVector.x(), biasVector.y(), biasVector.z());
      MICROSTRAIN_INFO(node_, "Client request values are: %.2f %.2f %.2f", req.bias.x, req.bias.y, req.bias.z);

      biasVector.x(req.bias.x);
      biasVector.y(req.bias.y);
      biasVector.z(req.bias.z);

      config_->inertial_device_->setMagnetometerHardIronOffset(biasVector);

      MICROSTRAIN_INFO(node_, "New hard iron values are: %.2f %.2f %.2f", biasVector.x(), biasVector.y(),
                       biasVector.z());

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Hard Iron Offset Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::getHardIronValues(GetHardIronValuesServiceMsg::Request& req,
                                               GetHardIronValuesServiceMsg::Response& res)
{
  res.success = false;
  MICROSTRAIN_INFO(node_, "Getting gyro bias values");

  if (config_->inertial_device_)
  {
    try
    {
      mscl::GeometricVector biasVector = config_->inertial_device_->getMagnetometerHardIronOffset();

      MICROSTRAIN_INFO(node_, "Hard iron values are: %f %f %f", biasVector.x(), biasVector.y(), biasVector.z());

      res.bias.x = biasVector.x();
      res.bias.y = biasVector.y();
      res.bias.z = biasVector.z();

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Soft Iron Matrix Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::setSoftIronMatrix(SetSoftIronMatrixServiceMsg::Request& req,
                                               SetSoftIronMatrixServiceMsg::Response& res)
{
  res.success = false;
  MICROSTRAIN_INFO(node_, "Setting the soft iron matrix values\n");

  if (config_->inertial_device_)
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

      config_->inertial_device_->setMagnetometerSoftIronMatrix(data);
      MICROSTRAIN_INFO(node_, "Sent values:     [%f  %f  %f][%f  %f  %f][%f  %f  %f]\n", data(0, 0), data(0, 1),
                       data(0, 2), data(1, 0), data(1, 1), data(1, 2), data(2, 0), data(2, 1), data(2, 2));

      data = config_->inertial_device_->getMagnetometerSoftIronMatrix();

      MICROSTRAIN_INFO(node_, "Returned values:     [%f  %f  %f][%f  %f  %f][%f  %f  %f]\n", data(0, 0), data(0, 1),
                       data(0, 2), data(1, 0), data(1, 1), data(1, 2), data(2, 0), data(2, 1), data(2, 2));

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Soft Iron Matrix Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::getSoftIronMatrix(GetSoftIronMatrixServiceMsg::Request& req,
                                               GetSoftIronMatrixServiceMsg::Response& res)
{
  res.success = false;
  MICROSTRAIN_INFO(node_, "Getting the soft iron matrix values\n");

  if (config_->inertial_device_)
  {
    try
    {
      mscl::Matrix_3x3 data = config_->inertial_device_->getMagnetometerSoftIronMatrix();

      MICROSTRAIN_INFO(node_, "Soft iron matrix values: [%f  %f  %f][%f  %f  %f][%f  %f  %f]\n", data(0, 0),
                       data(0, 1), data(0, 2), data(1, 0), data(1, 1), data(1, 2), data(2, 0), data(2, 1), data(2, 2));

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
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Complementary Filter Settings Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::setComplementaryFilter(SetComplementaryFilterServiceMsg::Request& req,
                                                   SetComplementaryFilterServiceMsg::Response& res)
{
  MICROSTRAIN_INFO(node_, "Setting the complementary filter values\n");
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      mscl::ComplementaryFilterData comp_filter_command;
      comp_filter_command.upCompensationEnabled = req.up_comp_enable;
      comp_filter_command.upCompensationTimeInSeconds = req.up_comp_time_const;
      comp_filter_command.northCompensationEnabled = req.north_comp_enable;
      comp_filter_command.northCompensationTimeInSeconds = req.north_comp_time_const;

      config_->inertial_device_->setComplementaryFilterSettings(comp_filter_command);

      MICROSTRAIN_INFO(
          node_, "Sent values:     Up Enable: %d North Enable: %d Up Time Constant: %f North Time Constant: %f \n",
          comp_filter_command.upCompensationEnabled, comp_filter_command.northCompensationEnabled,
          comp_filter_command.upCompensationTimeInSeconds, comp_filter_command.northCompensationTimeInSeconds);

      comp_filter_command = config_->inertial_device_->getComplementaryFilterSettings();

      MICROSTRAIN_INFO(
          node_, "Returned values: Up Enable: %d North Enable: %d Up Time Constant: %f North Time Constant: %f \n",
          comp_filter_command.upCompensationEnabled, comp_filter_command.northCompensationEnabled,
          comp_filter_command.upCompensationTimeInSeconds, comp_filter_command.northCompensationTimeInSeconds);

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Complementary Filter Settings Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::getComplementaryFilter(GetComplementaryFilterServiceMsg::Request& req,
                                                   GetComplementaryFilterServiceMsg::Response& res)
{
  res.success = false;
  MICROSTRAIN_INFO(node_, "Getting the complementary filter values\n");

  if (config_->inertial_device_)
  {
    try
    {
      mscl::ComplementaryFilterData comp_filter_command = config_->inertial_device_->getComplementaryFilterSettings();

      MICROSTRAIN_INFO(
          node_, "Returned values: Up Enable: %d North Enable: %d Up Time Constant: %f North Time Constant: %f \n",
          comp_filter_command.upCompensationEnabled, comp_filter_command.northCompensationEnabled,
          comp_filter_command.upCompensationTimeInSeconds, comp_filter_command.northCompensationTimeInSeconds);

      res.up_comp_enable = comp_filter_command.upCompensationEnabled;
      res.up_comp_time_const = comp_filter_command.upCompensationTimeInSeconds;
      res.north_comp_enable = comp_filter_command.northCompensationEnabled;
      res.north_comp_time_const = comp_filter_command.northCompensationTimeInSeconds;

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Heading Source Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::setHeadingSource(SetHeadingSourceServiceMsg::Request& req,
                                             SetHeadingSourceServiceMsg::Response& res)
{
  res.success = false;
  MICROSTRAIN_INFO(node_, "Set Heading Source\n");

  if (config_->inertial_device_)
  {
    try
    {
      mscl::InertialTypes::HeadingUpdateEnableOption source =
          static_cast<mscl::InertialTypes::HeadingUpdateEnableOption>(req.heading_source);

      for (mscl::HeadingUpdateOptions headingSources :
           config_->inertial_device_->features().supportedHeadingUpdateOptions())
      {
        if (headingSources.AsOptionId() == source)
        {
          MICROSTRAIN_INFO(node_, "Setting heading source to %#04X", source);
          config_->inertial_device_->setHeadingUpdateControl(mscl::HeadingUpdateOptions(source));
          res.success = true;
          break;
        }
      }
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Heading Source Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::getHeadingSource(GetHeadingSourceServiceMsg::Request& req,
                                             GetHeadingSourceServiceMsg::Response& res)
{
  res.success = false;
  MICROSTRAIN_INFO(node_, "Getting the heading source\n");

  if (config_->inertial_device_)
  {
    try
    {
      mscl::HeadingUpdateOptions source = config_->inertial_device_->getHeadingUpdateControl();

      MICROSTRAIN_INFO(node_, "Current heading source is %#04X", source.AsOptionId());

      res.heading_source = static_cast<uint8_t>(source.AsOptionId());

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Sensor2Vehicle Frame Rotation Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::setSensor2vehicleRotation(SetSensor2VehicleRotationServiceMsg::Request& req,
                                                      SetSensor2VehicleRotationServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      MICROSTRAIN_INFO(node_, "Setting the sensor to vehicle frame rotation\n");
      mscl::EulerAngles angles(req.angle.x, req.angle.y, req.angle.z);
      config_->inertial_device_->setSensorToVehicleRotation_eulerAngles(angles);

      angles = config_->inertial_device_->getSensorToVehicleRotation_eulerAngles();

      MICROSTRAIN_INFO(node_, "Rotation successfully set.\n");
      MICROSTRAIN_INFO(node_, "New angles: %f roll %f pitch %f yaw\n", angles.roll(), angles.pitch(), angles.yaw());

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Sensor2Vehicle Frame Rotation Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::getSensor2vehicleRotation(GetSensor2VehicleRotationServiceMsg::Request& req,
                                                      GetSensor2VehicleRotationServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      mscl::EulerAngles angles = config_->inertial_device_->getSensorToVehicleRotation_eulerAngles();
      MICROSTRAIN_INFO(node_, "Sensor Vehicle Frame Rotation Angles: %f roll %f pitch %f yaw\n", angles.roll(),
                       angles.pitch(), angles.yaw());

      res.angle.x = angles.roll();
      res.angle.y = angles.pitch();
      res.angle.z = angles.yaw();

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Sensor2Vehicle Frame Offset Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

// Set sensor to vehicle frame offset. Only in 45
bool MicrostrainServices::setSensor2vehicleOffset(SetSensor2VehicleOffsetServiceMsg::Request& req,
                                                    SetSensor2VehicleOffsetServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      MICROSTRAIN_INFO(node_, "Setting the sensor to vehicle frame offset\n");
      mscl::PositionOffset offset(req.offset.x, req.offset.y, req.offset.z);
      config_->inertial_device_->setSensorToVehicleOffset(offset);

      offset = config_->inertial_device_->getSensorToVehicleOffset();
      MICROSTRAIN_INFO(node_, "Offset successfully set.\n");
      MICROSTRAIN_INFO(node_, "Returned offset: %f X %f Y %f Z\n", offset.x(), offset.y(), offset.z());

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Sensor2Vehicle Frame Offset Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::getSensor2vehicleOffset(GetSensor2VehicleOffsetServiceMsg::Request& req,
                                                    GetSensor2VehicleOffsetServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      MICROSTRAIN_INFO(node_, "Getting the sensor to vehicle frame offset\n");

      mscl::PositionOffset offset = config_->inertial_device_->getSensorToVehicleOffset();
      MICROSTRAIN_INFO(node_, "Returned offset: %f X %f Y %f Z\n", offset.x(), offset.y(), offset.z());

      res.offset.x = offset.x();
      res.offset.y = offset.y();
      res.offset.z = offset.z();

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Sensor2Vehicle Frame Transformation (Combination of Offset and Rotation) Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::getSensor2vehicleTransformation(GetSensor2VehicleTransformationServiceMsg::Request& req,
                                                            GetSensor2VehicleTransformationServiceMsg::Response& res)
{
  res.success = false;

  if (!config_->inertial_device_)
  {
    return res.success;
  }

  try
  {
    MICROSTRAIN_INFO(node_, "Getting transform from sensor frame to vehicle frame");
    const mscl::PositionOffset offset = config_->inertial_device_->getSensorToVehicleOffset();
    const mscl::EulerAngles rotation = config_->inertial_device_->getSensorToVehicleRotation_eulerAngles();

    // set offset components from the device-stored values
    res.offset.x = offset.x();
    res.offset.y = offset.y();
    res.offset.z = offset.z();

    // set rotational components from the device-stored values
    tf2::Quaternion quat;
    quat.setRPY(rotation.roll(), rotation.pitch(), rotation.yaw());
    tf2::convert(quat, res.rotation);

    res.success = true;
  }
  catch (mscl::Error& e)
  {
    MICROSTRAIN_ERROR(node_, "Error getting sensor to vehicle transform: '%s'", e.what());
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Reference Position Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::setReferencePosition(SetReferencePositionServiceMsg::Request& req,
                                                 SetReferencePositionServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      MICROSTRAIN_INFO(node_, "Setting reference Position\n");

      mscl::Position referencePosition(req.position.x, req.position.y, req.position.z);
      mscl::FixedReferencePositionData referencePositionData(true, referencePosition);

      config_->inertial_device_->setFixedReferencePosition(referencePositionData);

      MICROSTRAIN_INFO(node_, "Reference position successfully set\n");
      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Reference Position Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::getReferencePosition(GetReferencePositionServiceMsg::Request& req,
                                                 GetReferencePositionServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      MICROSTRAIN_INFO(node_, "Getting reference position");

      mscl::Position referencePosition = config_->inertial_device_->getFixedReferencePosition().referencePosition;
      MICROSTRAIN_INFO(node_, "Reference position: Lat %f , Long %f, Alt %f", referencePosition.latitude(),
                       referencePosition.longitude(), referencePosition.altitude());

      res.position.x = referencePosition.latitude();
      res.position.y = referencePosition.longitude();
      res.position.z = referencePosition.altitude();

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Enable/Disable Coning and Sculling Compensation Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::setConingScullingComp(SetConingScullingCompServiceMsg::Request& req,
                                                   SetConingScullingCompServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      MICROSTRAIN_INFO(node_, "%s Coning and Sculling compensation", req.enable ? "DISABLED" : "ENABLED\n");
      config_->inertial_device_->setConingAndScullingEnable(req.enable);

      MICROSTRAIN_INFO(node_, "Reading Coning and Sculling compensation enabled state:\n");

      bool enabled = config_->inertial_device_->getConingAndScullingEnable();
      MICROSTRAIN_INFO(node_, "%s Coning and Sculling compensation", enabled ? "DISABLED" : "ENABLED\n");

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Enable/Disable Coning and Sculling Compensation Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::getConingScullingComp(GetConingScullingCompServiceMsg::Request& req,
                                                   GetConingScullingCompServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      MICROSTRAIN_INFO(node_, "Reading Coning and Sculling compensation enabled state:\n");

      bool enabled = config_->inertial_device_->getConingAndScullingEnable();
      MICROSTRAIN_INFO(node_, "%s Coning and Sculling compensation", enabled ? "DISABLED" : "ENABLED\n");

      res.enable = enabled;
      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Estimation Control Flags Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::setEstimationControlFlags(SetEstimationControlFlagsServiceMsg::Request& req,
                                                       SetEstimationControlFlagsServiceMsg::Response& res)
{
  if (config_->inertial_device_)
  {
    try
    {
      mscl::EstimationControlOptions flags(req.flags);
      config_->inertial_device_->setEstimationControlFlags(flags);
      flags = config_->inertial_device_->getEstimationControlFlags();
      MICROSTRAIN_INFO(node_, "Estimation control set to: %d", flags.AsUint16());

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Estimation Control Flags Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::getEstimationControlFlags(GetEstimationControlFlagsServiceMsg::Request& req,
                                                       GetEstimationControlFlagsServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      uint16_t flags = config_->inertial_device_->getEstimationControlFlags().AsUint16();

      MICROSTRAIN_INFO(node_, "Estimation control set to: %x", flags);

      res.flags = flags;
      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Device Basic Status Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::getBasicStatus(TriggerServiceMsg::Request& req, TriggerServiceMsg::Response& res)
{
  if (!config_->inertial_device_)
  {
    return false;
  }

  if (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_DEVICE_STATUS))
  {
    mscl::DeviceStatusMap status;

    if (config_->inertial_device_->features().supportedStatusSelectors().size() > 0)
    {
      mscl::DeviceStatusData statusData = config_->inertial_device_->getBasicDeviceStatus();
      status = statusData.asMap();
    }
    else
    {
      MICROSTRAIN_INFO(node_, "Model Number: \t\t\t\t\t%s\n", config_->inertial_device_->modelNumber().c_str());
      return true;
    }

    mscl::DeviceStatusMap::iterator it;

    for (it = status.begin(); it != status.end(); it++)
    {
      switch (it->first)
      {
        case mscl::DeviceStatusValues::ModelNumber:
          MICROSTRAIN_INFO(node_, "Model Number: \t\t\t\t\t%s\n", (it->second).c_str());
          break;

        case mscl::DeviceStatusValues::StatusStructure_Value:
          MICROSTRAIN_INFO(node_, "Status Selector: \t\t\t\t%s\n", (it->second).c_str());
          break;

        case mscl::DeviceStatusValues::SystemState_Value:
          MICROSTRAIN_INFO(node_, "System state: \t\t\t\t\t%s\n", (it->second).c_str());
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

bool MicrostrainServices::getDiagnosticReport(TriggerServiceMsg::Request& req, TriggerServiceMsg::Response& res)
{
  res.success = false;

  if (!config_->inertial_device_)
  {
    return false;
  }

  if (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_DEVICE_STATUS))
  {
    mscl::DeviceStatusMap status;

    if (config_->inertial_device_->features().supportedStatusSelectors().size() > 1)
    {
      mscl::DeviceStatusData statusData = config_->inertial_device_->getDiagnosticDeviceStatus();
      status = statusData.asMap();
      res.success = true;
    }

    else if (config_->inertial_device_->features().supportedStatusSelectors().size() > 0)
    {
      mscl::DeviceStatusData statusData = config_->inertial_device_->getBasicDeviceStatus();
      status = statusData.asMap();
      res.success = true;
    }
    else
    {
      MICROSTRAIN_INFO(node_, "Model Number: \t\t\t\t\t%s\n", config_->inertial_device_->modelNumber().c_str());
      return true;
    }

    mscl::DeviceStatusMap::iterator it;

    for (it = status.begin(); it != status.end(); it++)
    {
      switch (it->first)
      {
        case mscl::DeviceStatusValues::ModelNumber:
          MICROSTRAIN_INFO(node_, "Model Number: \t\t\t\t\t%s\n", (it->second).c_str());
          break;

        case mscl::DeviceStatusValues::StatusStructure_Value:
          MICROSTRAIN_INFO(node_, "Status Selector: \t\t\t\t%s\n", (it->second).c_str());
          break;

        case mscl::DeviceStatusValues::SystemState_Value:
          MICROSTRAIN_INFO(node_, "System state: \t\t\t\t\t%s\n", (it->second).c_str());
          break;

        case mscl::DeviceStatusValues::ImuStreamInfo_Enabled:
          MICROSTRAIN_INFO(node_, "IMU Streaming Enabled: \t\t\t\t%s\n",
                           strcmp((it->second).c_str(), "1") == 0 ? "TRUE" : "FALSE");
          break;

        case mscl::DeviceStatusValues::ImuStreamInfo_PacketsDropped:
          MICROSTRAIN_INFO(node_, "Number of Dropped IMU Packets: \t\t\t%s Packets\n", (it->second).c_str());
          break;

        case mscl::DeviceStatusValues::EstimationFilterStreamInfo_Enabled:
          MICROSTRAIN_INFO(node_, "FILTER Streaming Enabled: \t\t\t%s\n",
                           strcmp((it->second).c_str(), "1") == 0 ? "TRUE" : "FALSE");
          break;

        case mscl::DeviceStatusValues::EstimationFilterStreamInfo_PacketsDropped:
          MICROSTRAIN_INFO(node_, "Number of Dropped FILTER Packets: \t\t%s Packets\n", (it->second).c_str());
          break;

        case mscl::DeviceStatusValues::ComPortInfo_BytesWritten:
          MICROSTRAIN_INFO(node_, "Communications Port Bytes Written: \t\t%s Bytes\n", (it->second).c_str());
          break;

        case mscl::DeviceStatusValues::ComPortInfo_BytesRead:
          MICROSTRAIN_INFO(node_, "Communications Port Bytes Read: \t\t%s Bytes\n", (it->second).c_str());
          break;

        case mscl::DeviceStatusValues::ComPortInfo_OverrunsOnWrite:
          MICROSTRAIN_INFO(node_, "Communications Port Write Overruns: \t\t%s\n", (it->second).c_str());
          break;

        case mscl::DeviceStatusValues::ComPortInfo_OverrunsOnRead:
          MICROSTRAIN_INFO(node_, "Communications Port Read Overruns: \t\t%s\n", (it->second).c_str());
          break;

        case mscl::DeviceStatusValues::ImuMessageInfo_MessageParsingErrors:
          MICROSTRAIN_INFO(node_, "IMU Parser Errors: \t\t\t\t%s Errors\n", (it->second).c_str());
          break;

        case mscl::DeviceStatusValues::ImuMessageInfo_MessagesRead:
          MICROSTRAIN_INFO(node_, "IMU Message Count: \t\t\t\t%s Messages\n", (it->second).c_str());
          break;

        case mscl::DeviceStatusValues::ImuMessageInfo_LastMessageReadinMS:
          MICROSTRAIN_INFO(node_, "IMU Last Message Received: \t\t\t%s ms\n", (it->second).c_str());
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
bool MicrostrainServices::setZeroAngleUpdateThreshold(SetZeroAngleUpdateThresholdServiceMsg::Request& req,
                                                          SetZeroAngleUpdateThresholdServiceMsg::Response& res)
{
  res.success = false;

  MICROSTRAIN_INFO(node_, "Setting Zero Angular-Rate-Update threshold\n");

  if (config_->inertial_device_)
  {
    try
    {
      mscl::ZUPTSettingsData ZUPTSettings(req.enable, req.threshold);
      config_->inertial_device_->setAngularRateZUPT(ZUPTSettings);

      ZUPTSettings = config_->inertial_device_->getAngularRateZUPT();
      MICROSTRAIN_INFO(node_, "Enable value set to: %d, Threshold is: %f rad/s", ZUPTSettings.enabled,
                       ZUPTSettings.threshold);

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Auto Angular Zupt Enable/Threshold Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::getZeroAngleUpdateThreshold(GetZeroAngleUpdateThresholdServiceMsg::Request& req,
                                                          GetZeroAngleUpdateThresholdServiceMsg::Response& res)
{
  res.success = false;

  MICROSTRAIN_INFO(node_, "Getting Zero Angular-Rate-Update threshold\n");

  if (config_->inertial_device_)
  {
    try
    {
      mscl::ZUPTSettingsData ZUPTSettings = config_->inertial_device_->getAngularRateZUPT();
      MICROSTRAIN_INFO(node_, "Enable value set to: %d, Threshold is: %f rad/s", ZUPTSettings.enabled,
                       ZUPTSettings.threshold);

      res.enable = ZUPTSettings.enabled;
      res.threshold = ZUPTSettings.threshold;
      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Auto Velocity Zupt Enable/Threshold Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::setZeroVelocityUpdateThreshold(SetZeroVelocityUpdateThresholdServiceMsg::Request& req,
                                                             SetZeroVelocityUpdateThresholdServiceMsg::Response& res)
{
  res.success = false;

  MICROSTRAIN_INFO(node_, "Setting Zero Velocity-Update threshold\n");

  if (config_->inertial_device_)
  {
    try
    {
      mscl::ZUPTSettingsData ZUPTSettings(req.enable, req.threshold);
      config_->inertial_device_->setVelocityZUPT(ZUPTSettings);

      ZUPTSettings = config_->inertial_device_->getVelocityZUPT();
      MICROSTRAIN_INFO(node_, "Enable value set to: %d, Threshold is: %f m/s", ZUPTSettings.enabled,
                       ZUPTSettings.threshold);

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Auto Velocity Zupt Enable/Threshold Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::getZeroVelocityUpdateThreshold(GetZeroVelocityUpdateThresholdServiceMsg::Request& req,
                                                             GetZeroVelocityUpdateThresholdServiceMsg::Response& res)
{
  res.success = false;

  MICROSTRAIN_INFO(node_, "Getting Zero Velocity-Update threshold\n");

  if (config_->inertial_device_)
  {
    try
    {
      mscl::ZUPTSettingsData ZUPTSettings = config_->inertial_device_->getVelocityZUPT();
      MICROSTRAIN_INFO(node_, "Enable value set to: %d, Threshold is: %f rad/s", ZUPTSettings.enabled,
                       ZUPTSettings.threshold);

      res.enable = ZUPTSettings.enabled;
      res.threshold = ZUPTSettings.threshold;
      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Tare Orientation Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::setTareOrientation(SetTareOrientationServiceMsg::Request& req,
                                               SetTareOrientationServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      mscl::TareAxisValues axisValue(req.axis & mscl::InertialTypes::TARE_PITCH_AXIS,
                                     req.axis & mscl::InertialTypes::TARE_ROLL_AXIS,
                                     req.axis & mscl::InertialTypes::TARE_YAW_AXIS);
      config_->inertial_device_->tareOrientation(axisValue);

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Accel Noise 1-Sigma Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::setAccelNoise(SetAccelNoiseServiceMsg::Request& req, SetAccelNoiseServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      MICROSTRAIN_INFO(node_, "Setting the accel noise values\n");
      mscl::GeometricVector noise(req.noise.x, req.noise.y, req.noise.z);
      config_->inertial_device_->setAccelNoiseStandardDeviation(noise);

      noise = config_->inertial_device_->getAccelNoiseStandardDeviation();
      MICROSTRAIN_INFO(node_, "Accel noise values successfully set.\n");
      MICROSTRAIN_INFO(node_, "Returned values: %f X %f Y %f Z\n", noise.x(), noise.y(), noise.z());

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Accel Noise 1-Sigma Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::getAccelNoise(GetAccelNoiseServiceMsg::Request& req, GetAccelNoiseServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      MICROSTRAIN_INFO(node_, "Getting the accel noise values\n");

      mscl::GeometricVector noise = config_->inertial_device_->getAccelNoiseStandardDeviation();
      MICROSTRAIN_INFO(node_, "Returned values: %f X %f Y %f Z\n", noise.x(), noise.y(), noise.z());

      res.noise.x = noise.x();
      res.noise.y = noise.y();
      res.noise.z = noise.z();
      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Gyro Noise 1-Sigma Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::setGyroNoise(SetGyroNoiseServiceMsg::Request& req, SetGyroNoiseServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      MICROSTRAIN_INFO(node_, "Setting the gyro noise values\n");
      mscl::GeometricVector noise(req.noise.x, req.noise.y, req.noise.z);
      config_->inertial_device_->setGyroNoiseStandardDeviation(noise);

      noise = config_->inertial_device_->getGyroNoiseStandardDeviation();
      MICROSTRAIN_INFO(node_, "Gyro noise values successfully set.\n");
      MICROSTRAIN_INFO(node_, "Returned values: %f X %f Y %f Z\n", noise.x(), noise.y(), noise.z());

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Gyro Noise 1-Sigma Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::getGyroNoise(GetGyroNoiseServiceMsg::Request& req, GetGyroNoiseServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      MICROSTRAIN_INFO(node_, "Getting the gyro noise values\n");

      mscl::GeometricVector noise = config_->inertial_device_->getGyroNoiseStandardDeviation();
      MICROSTRAIN_INFO(node_, "Gyro noise values: %f X %f Y %f Z\n", noise.x(), noise.y(), noise.z());

      res.noise.x = noise.x();
      res.noise.y = noise.y();
      res.noise.z = noise.z();
      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Magnetometer Noise 1-Sigma Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::setMagNoise(SetMagNoiseServiceMsg::Request& req, SetMagNoiseServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      MICROSTRAIN_INFO(node_, "Setting the mag noise values\n");
      mscl::GeometricVector noise(req.noise.x, req.noise.y, req.noise.z);
      config_->inertial_device_->setHardIronOffsetProcessNoise(noise);

      noise = config_->inertial_device_->getHardIronOffsetProcessNoise();
      MICROSTRAIN_INFO(node_, "Mag noise values successfully set.\n");
      MICROSTRAIN_INFO(node_, "Returned values: %f X %f Y %f Z\n", noise.x(), noise.y(), noise.z());

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Magnetometer Noise 1-Sigma Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::getMagNoise(GetMagNoiseServiceMsg::Request& req, GetMagNoiseServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      MICROSTRAIN_INFO(node_, "Getting the mag noise values\n");
      mscl::GeometricVector noise = config_->inertial_device_->getHardIronOffsetProcessNoise();
      MICROSTRAIN_INFO(node_, "Returned values: %f X %f Y %f Z\n", noise.x(), noise.y(), noise.z());

      res.noise.x = noise.x();
      res.noise.y = noise.y();
      res.noise.z = noise.z();
      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Gyro Bias Model Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::setGyroBiasModel(SetGyroBiasModelServiceMsg::Request& req,
                                              SetGyroBiasModelServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      MICROSTRAIN_INFO(node_, "Setting the gyro bias model values\n");

      mscl::GeometricVectors collection;
      mscl::GeometricVector noise(req.noise_vector.x, req.noise_vector.y, req.noise_vector.z);
      collection.push_back(noise);

      mscl::GeometricVector beta_vector(req.beta_vector.x, req.beta_vector.y, req.beta_vector.z);
      collection.push_back(beta_vector);

      config_->inertial_device_->setGyroBiasModelParams(collection);

      collection = config_->inertial_device_->getGyroBiasModelParams();
      MICROSTRAIN_INFO(node_, "Gyro bias model values successfully set.\n");
      MICROSTRAIN_INFO(node_, "Returned values:  Beta: %f X %f Y %f Z, White Noise: %f X %f Y %f Z\n",
                       collection[0].x(), collection[0].y(), collection[0].z(), collection[1].x(), collection[1].y(),
                       collection[1].z());

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Gyro Bias Model Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::getGyroBiasModel(GetGyroBiasModelServiceMsg::Request& req,
                                              GetGyroBiasModelServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      MICROSTRAIN_INFO(node_, "Getting the gyro bias model values\n");
      mscl::GeometricVectors collection = config_->inertial_device_->getGyroBiasModelParams();
      MICROSTRAIN_INFO(node_, "Gyro bias model values:  Beta: %f X %f Y %f Z, White Noise: %f X %f Y %f Z\n",
                       collection[0].x(), collection[0].y(), collection[0].z(), collection[1].x(), collection[1].y(),
                       collection[1].z());

      res.noise_vector.x = collection[0].x();
      res.noise_vector.y = collection[0].y();
      res.noise_vector.z = collection[0].z();
      res.beta_vector.x = collection[1].x();
      res.beta_vector.y = collection[1].y();
      res.beta_vector.z = collection[1].z();

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Accel Bias Model Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::setAccelBiasModel(SetAccelBiasModelServiceMsg::Request& req,
                                               SetAccelBiasModelServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      MICROSTRAIN_INFO(node_, "Setting the accel bias model values\n");
      mscl::GeometricVectors collection;
      mscl::GeometricVector noise(req.noise_vector.x, req.noise_vector.y, req.noise_vector.z);
      collection.push_back(noise);

      mscl::GeometricVector beta_vector(req.beta_vector.x, req.beta_vector.y, req.beta_vector.z);
      collection.push_back(beta_vector);
      config_->inertial_device_->setAccelBiasModelParams(collection);

      collection = config_->inertial_device_->getAccelBiasModelParams();
      MICROSTRAIN_INFO(node_, "Accel bias model values successfully set.\n");
      MICROSTRAIN_INFO(node_, "Returned values:  Beta: %f X %f Y %f Z, White Noise: %f X %f Y %f Z\n",
                       collection[0].x(), collection[0].y(), collection[0].z(), collection[1].x(), collection[1].y(),
                       collection[1].z());

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Accel Bias Model Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::getAccelBiasModel(GetAccelBiasModelServiceMsg::Request& req,
                                               GetAccelBiasModelServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      MICROSTRAIN_INFO(node_, "Getting the accel bias model values\n");
      mscl::GeometricVectors collection = config_->inertial_device_->getAccelBiasModelParams();

      MICROSTRAIN_INFO(node_, "Accel bias model values:  Beta: %f X %f Y %f Z, White Noise: %f X %f Y %f Z\n",
                       collection[0].x(), collection[0].y(), collection[0].z(), collection[1].x(), collection[1].y(),
                       collection[1].z());

      res.noise_vector.x = collection[0].x();
      res.noise_vector.y = collection[0].y();
      res.noise_vector.z = collection[0].z();
      res.beta_vector.x = collection[1].x();
      res.beta_vector.y = collection[1].y();
      res.beta_vector.z = collection[1].z();

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Accel Magnitude Error Adaptive Measurement Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::setGravityAdaptiveVals(SetGravityAdaptiveValsServiceMsg::Request& req,
                                                    SetGravityAdaptiveValsServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      MICROSTRAIN_INFO(node_, "Setting the accel magnitude error adaptive measurement values\n");

      mscl::AdaptiveMeasurementData adaptiveData;
      adaptiveData.mode = static_cast<mscl::InertialTypes::AdaptiveMeasurementMode>(req.enable);
      adaptiveData.lowPassFilterCutoff = req.low_pass_cutoff;
      adaptiveData.lowLimit = req.low_limit;
      adaptiveData.highLimit = req.high_limit;
      adaptiveData.lowLimitUncertainty = req.low_limit_1sigma;
      adaptiveData.highLimitUncertainty = req.high_limit_1sigma;
      adaptiveData.minUncertainty = req.min_1sigma;

      config_->inertial_device_->setGravityErrorAdaptiveMeasurement(adaptiveData);

      adaptiveData = config_->inertial_device_->getGravityErrorAdaptiveMeasurement();
      MICROSTRAIN_INFO(node_, "accel magnitude error adaptive measurement values successfully set.\n");
      MICROSTRAIN_INFO(node_, "Returned values: Enable: %i, Parameters: %f %f %f %f %f %f", adaptiveData.mode,
                       adaptiveData.lowPassFilterCutoff, adaptiveData.minUncertainty, adaptiveData.lowLimit,
                       adaptiveData.highLimit, adaptiveData.lowLimitUncertainty, adaptiveData.highLimitUncertainty);

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Accel Magnitude Error Adaptive Measurement Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::getGravityAdaptiveVals(GetGravityAdaptiveValsServiceMsg::Request& req,
                                                    GetGravityAdaptiveValsServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      MICROSTRAIN_INFO(node_, "Getting the accel magnitude error adaptive measurement values\n");

      mscl::AdaptiveMeasurementData adaptiveData = config_->inertial_device_->getGravityErrorAdaptiveMeasurement();
      MICROSTRAIN_INFO(
          node_, "Accel magnitude error adaptive measurement values are: Enable: %i, Parameters: %f %f %f %f %f %f",
          adaptiveData.mode, adaptiveData.lowPassFilterCutoff, adaptiveData.minUncertainty, adaptiveData.lowLimit,
          adaptiveData.highLimit, adaptiveData.lowLimitUncertainty, adaptiveData.highLimitUncertainty);

      res.enable = adaptiveData.mode;
      res.low_pass_cutoff = adaptiveData.lowPassFilterCutoff;
      res.low_limit = adaptiveData.lowLimit;
      res.high_limit = adaptiveData.highLimit;
      res.low_limit_1sigma = adaptiveData.lowLimitUncertainty;
      res.high_limit_1sigma = adaptiveData.highLimitUncertainty;
      res.min_1sigma = adaptiveData.minUncertainty;

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Magnetometer Magnitude Error Adaptive Measurement Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::setMagAdaptiveVals(SetMagAdaptiveValsServiceMsg::Request& req,
                                                SetMagAdaptiveValsServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      MICROSTRAIN_INFO(node_, "Setting the mag magnitude error adaptive measurement values\n");

      mscl::AdaptiveMeasurementData adaptiveData;
      adaptiveData.mode = static_cast<mscl::InertialTypes::AdaptiveMeasurementMode>(req.enable);
      adaptiveData.lowPassFilterCutoff = req.low_pass_cutoff;
      adaptiveData.lowLimit = req.low_limit;
      adaptiveData.highLimit = req.high_limit;
      adaptiveData.lowLimitUncertainty = req.low_limit_1sigma;
      adaptiveData.highLimitUncertainty = req.high_limit_1sigma;
      adaptiveData.minUncertainty = req.min_1sigma;

      config_->inertial_device_->setMagnetometerErrorAdaptiveMeasurement(adaptiveData);

      adaptiveData = config_->inertial_device_->getMagnetometerErrorAdaptiveMeasurement();
      MICROSTRAIN_INFO(node_, "mag magnitude error adaptive measurement values successfully set.\n");

      MICROSTRAIN_INFO(node_, "Returned values: Enable: %i, Parameters: %f %f %f %f %f %f", adaptiveData.mode,
                       adaptiveData.lowPassFilterCutoff, adaptiveData.minUncertainty, adaptiveData.lowLimit,
                       adaptiveData.highLimit, adaptiveData.lowLimitUncertainty, adaptiveData.highLimitUncertainty);

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Magnetometer Magnitude Error Adaptive Measurement Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::getMagAdaptiveVals(GetMagAdaptiveValsServiceMsg::Request& req,
                                                GetMagAdaptiveValsServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      MICROSTRAIN_INFO(node_, "Getting the mag magnitude error adaptive measurement values\n");

      mscl::AdaptiveMeasurementData adaptiveData =
          config_->inertial_device_->getMagnetometerErrorAdaptiveMeasurement();
      MICROSTRAIN_INFO(
          node_, "Mag magnitude error adaptive measurement values are: Enable: %i, Parameters: %f %f %f %f %f %f",
          adaptiveData.mode, adaptiveData.lowPassFilterCutoff, adaptiveData.minUncertainty, adaptiveData.lowLimit,
          adaptiveData.highLimit, adaptiveData.lowLimitUncertainty, adaptiveData.highLimitUncertainty);

      res.enable = adaptiveData.mode;
      res.low_pass_cutoff = adaptiveData.lowPassFilterCutoff;
      res.low_limit = adaptiveData.lowLimit;
      res.high_limit = adaptiveData.highLimit;
      res.low_limit_1sigma = adaptiveData.lowLimitUncertainty;
      res.high_limit_1sigma = adaptiveData.highLimitUncertainty;
      res.min_1sigma = adaptiveData.minUncertainty;

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Magnetometer Dip Angle Error Adaptive Measurement Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::setMagDipAdaptiveVals(SetMagDipAdaptiveValsServiceMsg::Request& req,
                                                    SetMagDipAdaptiveValsServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      MICROSTRAIN_INFO(node_, "Setting the mag dip angle error adaptive measurement values\n");

      mscl::AdaptiveMeasurementData adaptiveData;
      adaptiveData.mode = static_cast<mscl::InertialTypes::AdaptiveMeasurementMode>(req.enable);
      adaptiveData.lowPassFilterCutoff = req.low_pass_cutoff;
      adaptiveData.highLimit = req.high_limit;
      adaptiveData.highLimitUncertainty = req.high_limit_1sigma;
      adaptiveData.minUncertainty = req.min_1sigma;

      config_->inertial_device_->setMagDipAngleErrorAdaptiveMeasurement(adaptiveData);

      adaptiveData = config_->inertial_device_->getMagDipAngleErrorAdaptiveMeasurement();
      MICROSTRAIN_INFO(node_, "mag dip angle error adaptive measurement values successfully set.\n");
      MICROSTRAIN_INFO(node_, "Returned values: Enable: %i, Parameters: %f %f %f %f\n", adaptiveData.mode,
                       adaptiveData.lowPassFilterCutoff, adaptiveData.minUncertainty, adaptiveData.highLimit,
                       adaptiveData.highLimitUncertainty);

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Magnetometer Dip Angle Error Adaptive Measurement Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::getMagDipAdaptiveVals(GetMagDipAdaptiveValsServiceMsg::Request& req,
                                                    GetMagDipAdaptiveValsServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      MICROSTRAIN_INFO(node_, "Getting the mag dip angle error adaptive measurement values\n");

      mscl::AdaptiveMeasurementData adaptiveData =
          config_->inertial_device_->getMagDipAngleErrorAdaptiveMeasurement();
      MICROSTRAIN_INFO(node_, "Returned values: Enable: %i, Parameters: %f %f %f %f %f %f", adaptiveData.mode,
                       adaptiveData.lowPassFilterCutoff, adaptiveData.minUncertainty, adaptiveData.lowLimit,
                       adaptiveData.highLimit, adaptiveData.lowLimitUncertainty, adaptiveData.highLimitUncertainty);

      res.enable = adaptiveData.mode;
      res.low_pass_cutoff = adaptiveData.lowPassFilterCutoff;
      res.high_limit = adaptiveData.highLimit;
      res.high_limit_1sigma = adaptiveData.highLimitUncertainty;
      res.min_1sigma = adaptiveData.minUncertainty;

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Vehicle Dynamics Mode Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::setDynamicsMode(SetDynamicsModeServiceMsg::Request& req,
                                            SetDynamicsModeServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      MICROSTRAIN_INFO(node_, "Setting the vehicle dynamics mode\n");

      mscl::InertialTypes::VehicleModeType mode = static_cast<mscl::InertialTypes::VehicleModeType>(req.mode);
      config_->inertial_device_->setVehicleDynamicsMode(mode);

      mode = config_->inertial_device_->getVehicleDynamicsMode();

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Vehicle Dynamics Mode Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::getDynamicsMode(GetDynamicsModeServiceMsg::Request& req,
                                            GetDynamicsModeServiceMsg::Response& res)
{
  res.success = false;

  MICROSTRAIN_INFO(node_, "Getting the vehicle dynamics mode\n");

  if (config_->inertial_device_)
  {
    try
    {
      mscl::InertialTypes::VehicleModeType mode = config_->inertial_device_->getVehicleDynamicsMode();
      MICROSTRAIN_INFO(node_, "Vehicle dynamics mode is: %d\n", mode);

      res.mode = mode;
      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Change Device Settings
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::deviceSettings(DeviceSettingsServiceMsg::Request& req,
                                          DeviceSettingsServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      switch (req.function_selector)
      {
        // Save
        case 3:
        {
          MICROSTRAIN_INFO(node_, "Processing device settings command with function selector = 3 (Save)\n");
          config_->inertial_device_->saveSettingsAsStartup();
        }
        break;

        // Load Saved Settings
        case 4:
        {
          MICROSTRAIN_INFO(node_,
                           "Processing device settings command with function selector = 4 (Load Saved Settings)\n");
          config_->inertial_device_->loadStartupSettings();
        }
        break;

        // Load Default Settings
        case 5:
        {
          MICROSTRAIN_INFO(node_,
                           "Processing device settings command with function selector = 5 (Load Defailt Settings)\n");
          config_->inertial_device_->loadFactoryDefaultSettings();
        }
        break;

        // Unsupported function selector
        default:
        {
          MICROSTRAIN_INFO(node_, "Error: Unsupported function selector for device settings command\n");
          return res.success;
        }
        break;
      }

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Commanded Velocity Zupt Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::commandedVelZupt(TriggerServiceMsg::Request& req, TriggerServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_ &&
      config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_CMDED_ZERO_VEL_UPDATE))
  {
    try
    {
      config_->inertial_device_->cmdedVelZUPT();
      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Commanded Angular Rate Zupt Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::commandedAngRateZupt(TriggerServiceMsg::Request& req, TriggerServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_ && config_->inertial_device_->features().supportsCommand(
                                         mscl::MipTypes::Command::CMD_EF_CMDED_ZERO_ANG_RATE_UPDATE))
  {
    try
    {
      config_->inertial_device_->cmdedAngRateZUPT();
      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// External Heading Update Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::externalHeadingUpdate(ExternalHeadingUpdateServiceMsg::Request& req,
                                                  ExternalHeadingUpdateServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      mscl::HeadingData heading_data;

      heading_data.headingAngle = req.heading_rad;
      heading_data.headingAngleUncertainty = req.heading_1sigma_rad;
      heading_data.heading = (mscl::HeadingData::HeadingType)req.heading_type;

      mscl::TimeUpdate timestamp(req.gps_tow, req.gps_week_number);

      if (req.use_time)
      {
        config_->inertial_device_->sendExternalHeadingUpdate(heading_data, timestamp);
        MICROSTRAIN_INFO(node_, "Sent External Heading update with timestamp.\n");
      }
      else
      {
        config_->inertial_device_->sendExternalHeadingUpdate(heading_data);
        MICROSTRAIN_INFO(node_, "Sent External Heading update.\n");
      }

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Relative Position Reference Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::setRelativePositionReference(SetRelativePositionReferenceServiceMsg::Request& req,
                                                          SetRelativePositionReferenceServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      mscl::PositionReferenceConfiguration ref;

      ref.position = mscl::Position(req.position.x, req.position.y, req.position.z,
                                    static_cast<mscl::PositionVelocityReferenceFrame>(req.frame));
      ref.autoConfig = !(static_cast<bool>(req.source));

      config_->inertial_device_->setRelativePositionReference(ref);

      if (req.source == 0)
        MICROSTRAIN_INFO(node_, "Setting reference position to RTK base station (automatic)");
      else
        MICROSTRAIN_INFO(node_, "Setting reference position to: [%f, %f, %f], ref frame = %d", req.position.x,
                         req.position.y, req.position.z, req.frame);

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Relative Position Reference Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainServices::getRelativePositionReference(GetRelativePositionReferenceServiceMsg::Request& req,
                                                          GetRelativePositionReferenceServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      mscl::PositionReferenceConfiguration ref = config_->inertial_device_->getRelativePositionReference();

      if (ref.autoConfig)
        MICROSTRAIN_INFO(node_, "Reference position is set to RTK base station (automatic)");
      else
        MICROSTRAIN_INFO(node_, "Reference position is: [%f, %f, %f], ref frame = %d", ref.position.x(),
                         ref.position.y(), ref.position.z(), (int)ref.position.referenceFrame);

      res.source = !ref.autoConfig;
      res.frame = static_cast<int32_t>(ref.position.referenceFrame);
      res.position.x = ref.position.x();
      res.position.y = ref.position.y();
      res.position.z = ref.position.z();

      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

bool MicrostrainServices::setFilterSpeedLeverArm(SetFilterSpeedLeverArmServiceMsg::Request& req,
                              SetFilterSpeedLeverArmServiceMsg::Response& res)
{
  res.success = false;

  if (config_->inertial_device_)
  {
    try
    {
      mscl::PositionOffset offset(req.offset.x, req.offset.y, req.offset.z);

      config_->inertial_device_->setSpeedMeasurementOffset(offset);
      res.success = true;
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }

  return res.success;
}

}  // namespace microstrain
