/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Inertial Device Driver Implementation File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2020, Parker Hannifin Corp
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <string>
#include <memory>
#include <iomanip>

#include "microstrain_inertial_driver_common/services.h"

namespace microstrain
{
Services::Services(RosNodeType* node, Config* config) : node_(node), config_(config)
{
}

bool Services::configure()
{
  {
    using namespace mip::commands_base;  // NOLINT(build/namespaces)

    device_report_service_ = configureService<DeviceReportServiceMsg, GetDeviceInfo>("device_report", &Services::deviceReport);
  }

  {
    using namespace mip::commands_3dm;  // NOLINT(build/namespaces)

    get_basic_status_service_ = configureService<TriggerServiceMsg>("get_basic_status", &Services::getBasicStatus);
    get_diagnostic_report_service_ = configureService<TriggerServiceMsg>("get_diagnostic_report", &Services::getDiagnosticReport);

    set_complementary_filter_service_ = configureService<SetComplementaryFilterServiceMsg, ComplementaryFilter>("set_complementary_filter", &Services::setComplementaryFilter);
    get_complementary_filter_service_ = configureService<GetComplementaryFilterServiceMsg, ComplementaryFilter>("get_complementary_filter", &Services::getComplementaryFilter);

    set_accel_bias_service_ = configureService<SetAccelBiasServiceMsg, AccelBias>("set_accel_bias", &Services::setAccelBias);
    get_accel_bias_service_ = configureService<GetAccelBiasServiceMsg, AccelBias>("get_accel_bias", &Services::getAccelBias);

    set_gyro_bias_service_ = configureService<SetGyroBiasServiceMsg, GyroBias>("set_gyro_bias", &Services::setGyroBias);
    get_gyro_bias_service_ = configureService<GetGyroBiasServiceMsg, GyroBias>("get_gyro_bias", &Services::getGyroBias);

    gyro_bias_capture_service_ = configureService<TriggerServiceMsg, CaptureGyroBias>("gyro_bias_capture", &Services::gyroBiasCapture);

    set_hard_iron_values_service_ = configureService<SetHardIronValuesServiceMsg, MagHardIronOffset>("set_hard_iron_values", &Services::setHardIronValues);
    get_hard_iron_values_service_ = configureService<GetHardIronValuesServiceMsg, MagHardIronOffset>("get_hard_iron_values", &Services::getHardIronValues);

    set_soft_iron_matrix_service_ = configureService<SetSoftIronMatrixServiceMsg, MagSoftIronMatrix>("set_soft_iron_matrix", &Services::setSoftIronMatrix);
    get_soft_iron_matrix_service_ = configureService<GetSoftIronMatrixServiceMsg, MagSoftIronMatrix>("get_soft_iron_matrix", &Services::getSoftIronMatrix);

    set_coning_sculling_comp_service_ = configureService<SetConingScullingCompServiceMsg, ConingScullingEnable>("set_coning_sculling_comp", &Services::setConingScullingComp);
    get_coning_sculling_comp_service_ = configureService<GetConingScullingCompServiceMsg, ConingScullingEnable>("get_coning_sculling_comp", &Services::getConingScullingComp);

    device_settings_service_ = configureService<DeviceSettingsServiceMsg, DeviceSettings>("device_settings", &Services::deviceSettings);
  }

  {
    using namespace mip::commands_filter;  // NOLINT(build/namespaces)

    set_tare_orientation_service_ = configureService<SetTareOrientationServiceMsg, TareOrientation>("set_tare_orientation", &Services::setTareOrientation);

    set_sensor2vehicle_rotation_service_ = configureService<SetSensor2VehicleRotationServiceMsg, SensorToVehicleRotationEuler>("set_sensor2vehicle_rotation", &Services::setSensor2vehicleRotation);
    get_sensor2vehicle_rotation_service_ = configureService<GetSensor2VehicleRotationServiceMsg, SensorToVehicleRotationEuler>("get_sensor2vehicle_rotation", &Services::getSensor2vehicleRotation);

    set_sensor2vehicle_offset_service_ = configureService<SetSensor2VehicleOffsetServiceMsg, SensorToVehicleOffset>("set_sensor2vehicle_offset", &Services::setSensor2vehicleOffset);
    get_sensor2vehicle_offset_service_ = configureService<GetSensor2VehicleOffsetServiceMsg, SensorToVehicleOffset>("get_sensor2vehicle_offset", &Services::getSensor2vehicleOffset);

    if (set_sensor2vehicle_offset_service_ && get_sensor2vehicle_offset_service_)
      get_sensor2vehicle_transformation_service_ = configureService<GetSensor2VehicleTransformationServiceMsg>("get_sensor2vehicle_transformation", &Services::getSensor2vehicleTransformation);

    reset_filter_service_ = configureService<EmptyServiceMsg, Reset>("reset_kf", &Services::resetFilter);

    set_estimation_control_flags_service_ = configureService<SetEstimationControlFlagsServiceMsg, EstimationControl>("set_estimation_control_flags", &Services::setEstimationControlFlags);
    get_estimation_control_flags_service_ = configureService<GetEstimationControlFlagsServiceMsg, EstimationControl>("get_estimation_control_flags", &Services::getEstimationControlFlags);

    init_filter_euler_service_ = configureService<InitFilterEulerServiceMsg, SetInitialAttitude>("init_filter_euler", &Services::initFilterEuler);

    init_filter_heading_service_ = configureService<InitFilterHeadingServiceMsg, SetInitialHeading>("init_filter_heading", &Services::initFilterHeading);

    set_heading_source_service_ = configureService<SetHeadingSourceServiceMsg, HeadingSource>("set_heading_source", &Services::setHeadingSource);
    get_heading_source_service_ = configureService<GetHeadingSourceServiceMsg, HeadingSource>("get_heading_source", &Services::getHeadingSource);

    commanded_vel_zupt_service_ = configureService<TriggerServiceMsg, CommandedZupt>("commanded_vel_zupt", &Services::commandedVelZupt);

    commanded_ang_rate_zupt_service_ = configureService<TriggerServiceMsg, CommandedAngularZupt>("commanded_ang_rate_zupt", &Services::commandedAngRateZupt);

    set_accel_noise_service_ = configureService<SetAccelNoiseServiceMsg, AccelNoise>("set_accel_noise", &Services::setAccelNoise);
    get_accel_noise_service_ = configureService<GetAccelNoiseServiceMsg, AccelNoise>("get_accel_noise", &Services::getAccelNoise);

    set_gyro_noise_service_ = configureService<SetGyroNoiseServiceMsg, GyroNoise>("set_gyro_noise", &Services::setGyroNoise);
    get_gyro_noise_service_ = configureService<GetGyroNoiseServiceMsg, GyroNoise>("get_gyro_noise", &Services::getGyroNoise);

    set_mag_noise_service_ = configureService<SetMagNoiseServiceMsg, HardIronOffsetNoise>("set_mag_noise", &Services::setMagNoise);
    get_mag_noise_service_ = configureService<GetMagNoiseServiceMsg, HardIronOffsetNoise>("get_mag_noise", &Services::getMagNoise);

    set_accel_bias_model_service_ = configureService<SetAccelBiasModelServiceMsg, AccelBiasModel>("set_accel_bias_model", &Services::setAccelBiasModel);
    get_accel_bias_model_service_ = configureService<GetAccelBiasModelServiceMsg, AccelBiasModel>("get_accel_bias_model", &Services::getAccelBiasModel);

    set_gyro_bias_model_service_ = configureService<SetGyroBiasModelServiceMsg, GyroBiasModel>("set_gyro_bias_model", &Services::setGyroBiasModel);
    get_gyro_bias_model_service_ = configureService<GetGyroBiasModelServiceMsg, GyroBiasModel>("get_gyro_bias_model", &Services::getGyroBiasModel);

    set_mag_adaptive_vals_service_ = configureService<SetMagAdaptiveValsServiceMsg, MagMagnitudeErrorAdaptiveMeasurement>("set_mag_adaptive_vals", &Services::setMagAdaptiveVals);
    get_mag_adaptive_vals_service_ = configureService<GetMagAdaptiveValsServiceMsg, MagMagnitudeErrorAdaptiveMeasurement>("get_mag_adaptive_vals", &Services::getMagAdaptiveVals);

    set_mag_dip_adaptive_vals_service_ = configureService<SetMagDipAdaptiveValsServiceMsg, MagDipAngleErrorAdaptiveMeasurement>("set_mag_dip_adaptive_vals", &Services::setMagDipAdaptiveVals);
    get_mag_dip_adaptive_vals_service_ = configureService<GetMagDipAdaptiveValsServiceMsg, MagDipAngleErrorAdaptiveMeasurement>("get_mag_dip_adaptive_vals", &Services::getMagDipAdaptiveVals);

    set_gravity_adaptive_vals_service_ = configureService<SetGravityAdaptiveValsServiceMsg, AccelMagnitudeErrorAdaptiveMeasurement>("set_gravity_adaptive_vals", &Services::setGravityAdaptiveVals);
    get_gravity_adaptive_vals_service_ = configureService<GetGravityAdaptiveValsServiceMsg, AccelMagnitudeErrorAdaptiveMeasurement>("get_gravity_adaptive_vals", &Services::getGravityAdaptiveVals);

    set_zero_angle_update_threshold_service_ = configureService<SetZeroAngleUpdateThresholdServiceMsg, AutoAngularZupt>("set_zero_angle_update_threshold", &Services::setZeroAngleUpdateThreshold);
    get_zero_angle_update_threshold_service_ = configureService<GetZeroAngleUpdateThresholdServiceMsg, AutoAngularZupt>("get_zero_angle_update_threshold", &Services::getZeroAngleUpdateThreshold);

    set_zero_velocity_update_threshold_service_ = configureService<SetZeroVelocityUpdateThresholdServiceMsg, AutoZupt>("set_zero_velocity_update_threshold", &Services::setZeroVelocityUpdateThreshold);
    get_zero_velocity_update_threshold_service_ = configureService<GetZeroVelocityUpdateThresholdServiceMsg, AutoZupt>("get_zero_velocity_update_threshold", &Services::getZeroVelocityUpdateThreshold);

    set_reference_position_service_ = configureService<SetReferencePositionServiceMsg, ReferencePosition>("set_reference_position", &Services::setReferencePosition);
    get_reference_position_service_ = configureService<GetReferencePositionServiceMsg, ReferencePosition>("get_reference_position", &Services::getReferencePosition);

    set_dynamics_mode_service_ = configureService<SetDynamicsModeServiceMsg, VehicleDynamicsMode>("set_dynamics_mode", &Services::setDynamicsMode);
    get_dynamics_mode_service_ = configureService<GetDynamicsModeServiceMsg, VehicleDynamicsMode>("get_dynamics_mode", &Services::getDynamicsMode);

    external_heading_service_ = configureService<ExternalHeadingUpdateServiceMsg, ExternalHeadingUpdate>("external_heading", &Services::externalHeadingUpdate);

    set_relative_position_reference_service_ = configureService<SetRelativePositionReferenceServiceMsg, RelPosConfiguration>("set_relative_position_reference", &Services::setRelativePositionReference);
    get_relative_position_reference_service_ = configureService<GetRelativePositionReferenceServiceMsg, RelPosConfiguration>("get_relative_position_reference", &Services::getRelativePositionReference);

    set_filter_speed_lever_arm_service_ = configureService<SetFilterSpeedLeverArmServiceMsg, SpeedLeverArm>("set_filter_speed_lever_arm", &Services::setFilterSpeedLeverArm);
  }

  return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Device Report
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::deviceReport(DeviceReportServiceMsg::Request& req, DeviceReportServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Fetching device info");

  mip::CmdResult mip_cmd_result;
  mip::commands_base::BaseDeviceInfo device_info;
  res.success = !!(mip_cmd_result = config_->mip_device_->getDeviceInfo(&device_info));
  if (res.success)
  {
    MICROSTRAIN_DEBUG(node_, "Fetched device info");
    MICROSTRAIN_DEBUG(node_, "  model name = %s", device_info.model_name);
    MICROSTRAIN_DEBUG(node_, "  model number = %s", device_info.model_number);
    MICROSTRAIN_DEBUG(node_, "  serial number = %s", device_info.serial_number);
    MICROSTRAIN_DEBUG(node_, "  device options = %s", device_info.device_options);
    MICROSTRAIN_DEBUG(node_, "  firmware version (raw) = %u", device_info.firmware_version);

    res.model_name = device_info.model_name;
    res.model_number = device_info.model_number;
    res.serial_number = device_info.serial_number;
    res.options = device_info.device_options;
    res.firmware_version = RosMipDevice::firmwareVersionString(device_info.firmware_version);
  }
  else
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to fetch device info");
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Reset Filter Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::resetFilter(EmptyServiceMsg::Request& req, EmptyServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Resetting filter");

  const mip::CmdResult mip_cmd_result = mip::commands_filter::reset(*(config_->mip_device_));
  if (!mip_cmd_result)
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to reset filter");
  else
    MICROSTRAIN_DEBUG(node_, "Reset filter");

  return !!mip_cmd_result;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialize Filter (Euler Angles) Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::initFilterEuler(InitFilterEulerServiceMsg::Request& req,
                                            InitFilterEulerServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Initializing filter euler to [%f, %f, %f]", req.angle.x, req.angle.y, req.angle.z);

  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::setInitialAttitude(*(config_->mip_device_), req.angle.x, req.angle.y, req.angle.z));
  if (!res.success)
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to initialize filter euler");
  else
    MICROSTRAIN_DEBUG(node_, "Initialized filter euler");

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialize Filter (Heading Angle) Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::initFilterHeading(InitFilterHeadingServiceMsg::Request& req,
                                              InitFilterHeadingServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Initializing filter heading to %f", req.angle);

  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::setInitialHeading(*(config_->mip_device_), req.angle));
  if (!res.success)
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to initialize filter heading");
  else
    MICROSTRAIN_DEBUG(node_, "Initialized filter heading");

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Accel Bias Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::setAccelBias(SetAccelBiasServiceMsg::Request& req, SetAccelBiasServiceMsg::Response& res)
{
  float accel_bias[3] = {req.bias.x, req.bias.y, req.bias.z};
  MICROSTRAIN_DEBUG(node_, "Setting accel bias to [%f, %f, %f]", accel_bias[0], accel_bias[1], accel_bias[2]);

  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_3dm::writeAccelBias(*(config_->mip_device_), accel_bias));
  if (!res.success)
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to set accel bias");
  else
    MICROSTRAIN_DEBUG(node_, "Set accel bias");

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Accel Bias Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::getAccelBias(GetAccelBiasServiceMsg::Request& req, GetAccelBiasServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Getting accel bias");

  float accel_bias[3];
  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_3dm::readAccelBias(*(config_->mip_device_), accel_bias));
  if (res.success)
  {
    MICROSTRAIN_DEBUG(node_, "Got accel bias [%f, %f, %f]", accel_bias[0], accel_bias[1], accel_bias[2]);
    res.bias.x = accel_bias[0];
    res.bias.y = accel_bias[1];
    res.bias.z = accel_bias[2];
  }
  else
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to get accel bias");
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Gyro Bias Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::setGyroBias(SetGyroBiasServiceMsg::Request& req, SetGyroBiasServiceMsg::Response& res)
{
  float gyro_bias[3] = {req.bias.x, req.bias.y, req.bias.z};
  MICROSTRAIN_DEBUG(node_, "Setting gyro bias to [%f, %f, %f]", gyro_bias[0], gyro_bias[1], gyro_bias[2]);

  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_3dm::writeGyroBias(*(config_->mip_device_), gyro_bias));
  if (!res.success)
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to set gyro bias");
  else
    MICROSTRAIN_DEBUG(node_, "Set gyro bias");

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Gyro Bias Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::getGyroBias(GetGyroBiasServiceMsg::Request& req, GetGyroBiasServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Getting gyro bias");

  float gyro_bias[3];
  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_3dm::readGyroBias(*(config_->mip_device_), gyro_bias));
  if (res.success)
  {
    MICROSTRAIN_DEBUG(node_, "Got gyro bias [%f, %f, %f]", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
    res.bias.x = gyro_bias[0];
    res.bias.y = gyro_bias[1];
    res.bias.z = gyro_bias[2];
  }
  else
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to get gyro bias");
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Gyro Bias Capture Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::gyroBiasCapture(TriggerServiceMsg::Request& req, TriggerServiceMsg::Response& res)
{
  const int32_t capture_timeout = 10000;
  MICROSTRAIN_DEBUG(node_, "Capturing gyro bias");
  MICROSTRAIN_WARN(node_, "Performing Gyro Bias capture.");
  MICROSTRAIN_WARN(node_, "Please keep device stationary during the %f second gyro bias capture interval", static_cast<float>(capture_timeout) / 1000);

  // We need to change the timeout to allow for this longer command
  const int32_t old_mip_sdk_timeout = config_->mip_device_->device().baseReplyTimeout();
  config_->mip_device_->device().setBaseReplyTimeout(capture_timeout * 2);

  float gyro_bias[3];
  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_3dm::captureGyroBias(*(config_->mip_device_), capture_timeout, gyro_bias));
  if (res.success)
  {
    MICROSTRAIN_DEBUG(node_, "Captured gyro bias: [%f, %f, %f]", gyro_bias[0], gyro_bias[1], gyro_bias[2]);

    // TODO(robbiefish): This should really return the captured gyro bias instead of just printing it
    MICROSTRAIN_INFO(node_, "Captured gyro bias: [%f, %f, %f]", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
  }
  else
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to capture gyro bias");
  }

  // Reset the timeout
  config_->mip_device_->device().setBaseReplyTimeout(old_mip_sdk_timeout);

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Hard Iron Offset Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::setHardIronValues(SetHardIronValuesServiceMsg::Request& req,
                                               SetHardIronValuesServiceMsg::Response& res)
{
  float mag_offset[3] = {req.bias.x, req.bias.y, req.bias.z};
  MICROSTRAIN_DEBUG(node_, "Setting hard iron offset to [%f, %f, %f]", mag_offset[0], mag_offset[1], mag_offset[2]);

  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_3dm::writeMagHardIronOffset(*(config_->mip_device_), mag_offset));
  if (!res.success)
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to set hard iron offset");
  else
    MICROSTRAIN_DEBUG(node_, "Set hard iron offset");

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Hard Iron Offset Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::getHardIronValues(GetHardIronValuesServiceMsg::Request& req,
                                               GetHardIronValuesServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Getting hard iron offsets");

  float mag_offsets[3];
  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_3dm::readMagHardIronOffset(*(config_->mip_device_), mag_offsets));
  if (res.success)
  {
    MICROSTRAIN_DEBUG(node_, "Got hard iron offsets [%f, %f, %f]", mag_offsets[0], mag_offsets[1], mag_offsets[2]);
    res.bias.x = mag_offsets[0];
    res.bias.y = mag_offsets[1];
    res.bias.z = mag_offsets[2];
  }
  else
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to get hard iron offsets");
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Soft Iron Matrix Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::setSoftIronMatrix(SetSoftIronMatrixServiceMsg::Request& req,
                                               SetSoftIronMatrixServiceMsg::Response& res)
{
  float soft_iron_matrix[9] =
  {
    req.soft_iron_1.x, req.soft_iron_1.y, req.soft_iron_1.z,
    req.soft_iron_2.x, req.soft_iron_2.y, req.soft_iron_2.z,
    req.soft_iron_3.x, req.soft_iron_3.y, req.soft_iron_3.z,
  };
  MICROSTRAIN_DEBUG(node_, "Setting soft iron matrix to:");
  MICROSTRAIN_DEBUG(node_, "  [%f, %f, %f]", soft_iron_matrix[0], soft_iron_matrix[1], soft_iron_matrix[2]);
  MICROSTRAIN_DEBUG(node_, "  [%f, %f, %f]", soft_iron_matrix[3], soft_iron_matrix[4], soft_iron_matrix[5]);
  MICROSTRAIN_DEBUG(node_, "  [%f, %f, %f]", soft_iron_matrix[6], soft_iron_matrix[7], soft_iron_matrix[8]);

  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_3dm::writeMagSoftIronMatrix(*(config_->mip_device_), soft_iron_matrix));
  if (!res.success)
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to set soft iron matrix");
  else
    MICROSTRAIN_DEBUG(node_, "Set soft iron matrix");

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Soft Iron Matrix Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::getSoftIronMatrix(GetSoftIronMatrixServiceMsg::Request& req,
                                               GetSoftIronMatrixServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Getting soft iron matrix");

  float soft_iron_matrix[9];
  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_3dm::readMagSoftIronMatrix(*(config_->mip_device_), soft_iron_matrix));
  if (res.success)
  {
    MICROSTRAIN_DEBUG(node_, "Got soft iron matrix:");
    MICROSTRAIN_DEBUG(node_, "  [%f, %f, %f]", soft_iron_matrix[0], soft_iron_matrix[1], soft_iron_matrix[2]);
    MICROSTRAIN_DEBUG(node_, "  [%f, %f, %f]", soft_iron_matrix[3], soft_iron_matrix[4], soft_iron_matrix[5]);
    MICROSTRAIN_DEBUG(node_, "  [%f, %f, %f]", soft_iron_matrix[6], soft_iron_matrix[7], soft_iron_matrix[8]);
    res.soft_iron_1.x = soft_iron_matrix[0];
    res.soft_iron_1.y = soft_iron_matrix[1];
    res.soft_iron_1.z = soft_iron_matrix[2];
    res.soft_iron_2.x = soft_iron_matrix[3];
    res.soft_iron_2.y = soft_iron_matrix[4];
    res.soft_iron_2.z = soft_iron_matrix[5];
    res.soft_iron_3.x = soft_iron_matrix[6];
    res.soft_iron_3.y = soft_iron_matrix[7];
    res.soft_iron_3.z = soft_iron_matrix[8];
  }
  else
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to get soft iron matrix");
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Complementary Filter Settings Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::setComplementaryFilter(SetComplementaryFilterServiceMsg::Request& req,
                                                   SetComplementaryFilterServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Setting complementary filter:");
  MICROSTRAIN_DEBUG(node_, "  pitch roll enable = %d", req.up_comp_enable);
  MICROSTRAIN_DEBUG(node_, "  heading enable = %d", req.north_comp_enable);
  MICROSTRAIN_DEBUG(node_, "  pitch roll time constant = %f", req.up_comp_time_const);
  MICROSTRAIN_DEBUG(node_, "  heading time constant = %f", req.north_comp_time_const);

  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_3dm::writeComplementaryFilter(*(config_->mip_device_), req.up_comp_enable, req.north_comp_enable, req.up_comp_time_const, req.north_comp_time_const));
  if (!res.success)
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to set complementary filter");
  else
    MICROSTRAIN_DEBUG(node_, "Set complementary filter");

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Complementary Filter Settings Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::getComplementaryFilter(GetComplementaryFilterServiceMsg::Request& req,
                                                   GetComplementaryFilterServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Getting complementary filter");

  mip::CmdResult mip_cmd_result;
  bool pitch_roll_comp_enable, heading_comp_enable;
  float pitch_roll_time_constant, heading_time_constant;
  res.success = !!(mip_cmd_result = mip::commands_3dm::readComplementaryFilter(*(config_->mip_device_), &pitch_roll_comp_enable, &heading_comp_enable, &pitch_roll_time_constant, &heading_time_constant));
  if (res.success)
  {
    MICROSTRAIN_DEBUG(node_, "Got complementary filter:");
    MICROSTRAIN_DEBUG(node_, "  pitch roll enable = %d", pitch_roll_comp_enable);
    MICROSTRAIN_DEBUG(node_, "  heading enable = %d", heading_comp_enable);
    MICROSTRAIN_DEBUG(node_, "  pitch roll time constant = %f", pitch_roll_time_constant);
    MICROSTRAIN_DEBUG(node_, "  heading time constant = %f", heading_time_constant);
    res.up_comp_enable = pitch_roll_comp_enable;
    res.north_comp_enable = heading_comp_enable;
    res.up_comp_time_const = pitch_roll_time_constant;
    res.north_comp_time_const = heading_time_constant;
  }
  else
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to get hard iron offsets");
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Heading Source Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::setHeadingSource(SetHeadingSourceServiceMsg::Request& req,
                                             SetHeadingSourceServiceMsg::Response& res)
{
  const auto source = static_cast<mip::commands_filter::HeadingSource::Source>(req.heading_source);
  MICROSTRAIN_DEBUG(node_, "Setting heading source to %d", req.heading_source);

  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::writeHeadingSource(*(config_->mip_device_), source));
  if (!res.success)
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to set heading source");
  else
    MICROSTRAIN_DEBUG(node_, "Set heading source");

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Heading Source Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::getHeadingSource(GetHeadingSourceServiceMsg::Request& req,
                                             GetHeadingSourceServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Getting heading source");

  mip::commands_filter::HeadingSource::Source source;
  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::readHeadingSource(*(config_->mip_device_), &source));
  if (res.success)
  {
    MICROSTRAIN_DEBUG(node_, "Got heading source %d", static_cast<int8_t>(source));
    res.heading_source = static_cast<int8_t>(source);
  }
  else
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to get heading source");
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Sensor2Vehicle Frame Rotation Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::setSensor2vehicleRotation(SetSensor2VehicleRotationServiceMsg::Request& req,
                                                      SetSensor2VehicleRotationServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Setting sensor to vehicle rotation to [%f, %f, %f]", req.angle.x, req.angle.y, req.angle.z);

  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::writeSensorToVehicleRotationEuler(*(config_->mip_device_), req.angle.x, req.angle.y, req.angle.z));
  if (!res.success)
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to set sensor to vehicle rotation");
  else
    MICROSTRAIN_DEBUG(node_, "Set sensor to vehicle rotation");

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Sensor2Vehicle Frame Rotation Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::getSensor2vehicleRotation(GetSensor2VehicleRotationServiceMsg::Request& req,
                                                      GetSensor2VehicleRotationServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Getting sensor to vehicle rotation");

  float roll, pitch, yaw;
  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::readSensorToVehicleRotationEuler(*(config_->mip_device_), &roll, &pitch, &yaw));
  if (res.success)
  {
    MICROSTRAIN_DEBUG(node_, "Got sensor to vehicle rotation [%f, %f, %f]", roll, pitch, yaw);
    res.angle.x = roll;
    res.angle.y = pitch;
    res.angle.z = yaw;
  }
  else
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to get sensor to vehicle rotation");
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Sensor2Vehicle Frame Offset Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

// Set sensor to vehicle frame offset. Only in 45
bool Services::setSensor2vehicleOffset(SetSensor2VehicleOffsetServiceMsg::Request& req,
                                                    SetSensor2VehicleOffsetServiceMsg::Response& res)
{
  float offset[3] = {req.offset.x, req.offset.y, req.offset.z};
  MICROSTRAIN_DEBUG(node_, "Setting sensor to vehicle offset to [%f, %f, %f]", offset[0], offset[1], offset[2]);

  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::writeSensorToVehicleOffset(*(config_->mip_device_), offset));
  if (!res.success)
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to set sensor to vehicle offset");
  else
    MICROSTRAIN_DEBUG(node_, "Set sensor to vehicle offset");

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Sensor2Vehicle Frame Offset Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::getSensor2vehicleOffset(GetSensor2VehicleOffsetServiceMsg::Request& req,
                                                    GetSensor2VehicleOffsetServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Getting sensor to vehicle offsets");

  float offsets[3];
  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::readSensorToVehicleOffset(*(config_->mip_device_), offsets));
  if (res.success)
  {
    MICROSTRAIN_DEBUG(node_, "Got sensor to vehicle offsets [%f, %f, %f]", offsets[0], offsets[1], offsets[2]);
    res.offset.x = offsets[0];
    res.offset.y = offsets[1];
    res.offset.z = offsets[2];
  }
  else
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to get sensor to vehicle offsets");
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Sensor2Vehicle Frame Transformation (Combination of Offset and Rotation) Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::getSensor2vehicleTransformation(GetSensor2VehicleTransformationServiceMsg::Request& req,
                                                            GetSensor2VehicleTransformationServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Getting sensor to vehicle transformation");

  // Just call the two existing services
  GetSensor2VehicleOffsetServiceMsg::Request offset_req;
  GetSensor2VehicleOffsetServiceMsg::Response offset_res;
  GetSensor2VehicleRotationServiceMsg::Request rotation_req;
  GetSensor2VehicleRotationServiceMsg::Response rotation_res;
  res.success = getSensor2vehicleOffset(offset_req, offset_res) && getSensor2vehicleRotation(rotation_req, rotation_res);

  if (res.success)
  {
    MICROSTRAIN_DEBUG(node_, "Got sensor to vehicle transformation");
    MICROSTRAIN_DEBUG(node_, "  offset = [%f, %f, %f]", offset_res.offset.x, offset_res.offset.y, offset_res.offset.z);
    MICROSTRAIN_DEBUG(node_, "  rotation = [%f, %f, %f]", rotation_res.angle.x, rotation_res.angle.y, rotation_res.angle.z);
    res.offset.x = offset_res.offset.x;
    res.offset.y = offset_res.offset.y;
    res.offset.z = offset_res.offset.z;

    tf2::Quaternion quat;
    quat.setRPY(rotation_res.angle.x, rotation_res.angle.y, rotation_res.angle.z);
    tf2::convert(quat, res.rotation);
  }
  else
  {
    MICROSTRAIN_ERROR(node_, "Failed to get sensor to vehicle transformation");
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Reference Position Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::setReferencePosition(SetReferencePositionServiceMsg::Request& req,
                                                 SetReferencePositionServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Setting reference position to [%f, %f, %f]", req.position.x, req.position.y, req.position.z);

  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::writeReferencePosition(*(config_->mip_device_), true, req.position.x, req.position.y, req.position.z));
  if (!res.success)
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to set reference position");
  else
    MICROSTRAIN_DEBUG(node_, "Set reference position");

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Reference Position Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::getReferencePosition(GetReferencePositionServiceMsg::Request& req,
                                                 GetReferencePositionServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Getting reference position");

  bool enable;
  double latitude, longitude, altitude;
  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::readReferencePosition(*(config_->mip_device_), &enable, &latitude, &longitude, &altitude));
  if (res.success)
  {
    MICROSTRAIN_DEBUG(node_, "Got reference position:");
    MICROSTRAIN_DEBUG(node_, "  enabled = %d", enable);
    MICROSTRAIN_DEBUG(node_, "  position = [%f, %f, %f]", latitude, longitude, altitude);
    res.position.x = latitude;
    res.position.y = longitude;
    res.position.z = altitude;
  }
  else
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to get reference position");
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Enable/Disable Coning and Sculling Compensation Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::setConingScullingComp(SetConingScullingCompServiceMsg::Request& req,
                                                   SetConingScullingCompServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Setting coning sculling enable to %d", req.enable);

  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_3dm::writeConingScullingEnable(*(config_->mip_device_), req.enable));
  if (!res.success)
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to set coning sculling enable");
  else
    MICROSTRAIN_DEBUG(node_, "Set coning sculling enable");

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Enable/Disable Coning and Sculling Compensation Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::getConingScullingComp(GetConingScullingCompServiceMsg::Request& req,
                                                   GetConingScullingCompServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Getting coning sculling enable");

  bool enable;
  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_3dm::readConingScullingEnable(*(config_->mip_device_), &enable));
  if (res.success)
  {
    MICROSTRAIN_DEBUG(node_, "Got coning sculling enable %d", enable);
    res.enable = enable;
  }
  else
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to get coning sculling enable");
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Estimation Control Flags Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::setEstimationControlFlags(SetEstimationControlFlagsServiceMsg::Request& req,
                                                       SetEstimationControlFlagsServiceMsg::Response& res)
{
  const auto flags = static_cast<mip::commands_filter::EstimationControl::EnableFlags>(req.flags);
  MICROSTRAIN_DEBUG(node_, "Setting estimation control flags to %d", req.flags);

  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::writeEstimationControl(*(config_->mip_device_), flags));
  if (!res.success)
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to set estimation control flags");
  else
    MICROSTRAIN_DEBUG(node_, "Set estimation control flags");

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Estimation Control Flags Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::getEstimationControlFlags(GetEstimationControlFlagsServiceMsg::Request& req,
                                                       GetEstimationControlFlagsServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Getting estimation control flags");

  mip::commands_filter::EstimationControl::EnableFlags flags;
  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::readEstimationControl(*(config_->mip_device_), &flags));
  if (res.success)
  {
    MICROSTRAIN_DEBUG(node_, "Got estimation control flags %d", static_cast<int8_t>(flags));
    res.flags = static_cast<int8_t>(flags);
  }
  else
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to get estimation control flags");
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Device Basic Status Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::getBasicStatus(TriggerServiceMsg::Request& req, TriggerServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Getting basic status");

  // The DEVICE_STATUS command is actually internal, so we don't have it exposed easily. Just run the command manually
  // Hopefully this is a prospect product and we don't need model number, so just try with all 0s
  mip::CmdResult mip_cmd_result;
  uint8_t mip_buffer[mip::C::MIP_FIELD_PAYLOAD_LENGTH_MAX];
  uint8_t mip_buffer_out_size = sizeof(mip_buffer);
  memset(mip_buffer, 0, sizeof(mip_buffer));
  res.success = !!(mip_cmd_result = mip::C::mip_interface_run_command_with_response(&(config_->mip_device_->device()), mip::commands_3dm::DESCRIPTOR_SET, 0x64, mip_buffer, 3, 0x90, mip_buffer, &mip_buffer_out_size));

  // Just print the raw bytes on success
  if (res.success)
  {
    std::stringstream basic_status_ss;
    for (uint8_t i = 0; i < mip_buffer_out_size; i++)
      basic_status_ss << std::setfill('0') << std::setw(2) << std::hex << static_cast<uint16_t>(mip_buffer[i]);
    MICROSTRAIN_INFO(node_, "Device Basic Status: 0x%s", basic_status_ss.str().c_str());
    return res.success;
  }

  // Get the device info to determine the model number if we need it
  MICROSTRAIN_DEBUG(node_, "Looks like this is a philo device, fetching device info");
  mip::commands_base::BaseDeviceInfo device_info;
  res.success = !!(mip_cmd_result = config_->mip_device_->getDeviceInfo(&device_info));
  if (!res.success)
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to fetch device info");
    return res.success;
  }

  // Parse out the high digits in the model number
  MICROSTRAIN_DEBUG(node_, "Parsing model number %s", device_info.model_number);
  const std::string& model_number_str = device_info.model_number;
  const size_t dash_index = model_number_str.find('-');
  uint16_t model_number = 0;
  if (dash_index != std::string::npos)
    model_number = std::stoi(model_number_str.substr(0, dash_index));
  else
    MICROSTRAIN_WARN(node_, "Model number not found in string, if this is a philo product, the call to DEVICE_STATUS will fail, otherwise this is fine");
  MICROSTRAIN_DEBUG(node_, "Model number u16 is %u", model_number);

  // Try the command again with the model number
  mip_buffer[0] = model_number & 0xFF;
  mip_buffer[1] = (model_number >> 8) & 0xFF;
  mip_buffer[2] = 1;
  res.success = !!(mip_cmd_result = mip::C::mip_interface_run_command_with_response(&(config_->mip_device_->device()), mip::commands_3dm::DESCRIPTOR_SET, 0x64, mip_buffer, 3, 0x90, mip_buffer, &mip_buffer_out_size));

  // Just print the raw bytes on success
  if (res.success)
  {
    std::stringstream basic_status_ss;
    for (uint8_t i = 0; i < mip_buffer_out_size; i++)
      basic_status_ss << std::setfill('0') << std::setw(2) << std::hex << static_cast<uint16_t>(mip_buffer[i]);
    MICROSTRAIN_INFO(node_, "Device Basic Status: 0x%s", basic_status_ss.str().c_str());
  }
  else
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to fetch basic device status");
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Diagnostic Status Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::getDiagnosticReport(TriggerServiceMsg::Request& req, TriggerServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Getting diagnostic status");

  // The DEVICE_STATUS command is actually internal, so we don't have it exposed easily. Just run the command manually
  // Hopefully this is a prospect product and we don't need model number, so just try with all 0s
  mip::CmdResult mip_cmd_result;
  uint8_t mip_buffer[mip::C::MIP_FIELD_PAYLOAD_LENGTH_MAX];
  uint8_t mip_buffer_out_size = sizeof(mip_buffer);
  memset(mip_buffer, 0, sizeof(mip_buffer));
  res.success = !!(mip_cmd_result = mip::C::mip_interface_run_command_with_response(&(config_->mip_device_->device()), mip::commands_3dm::DESCRIPTOR_SET, 0x64, mip_buffer, 3, 0x90, mip_buffer, &mip_buffer_out_size));

  // Just print the raw bytes on success
  if (res.success)
  {
    std::stringstream basic_status_ss;
    for (uint8_t i = 0; i < mip_buffer_out_size; i++)
      basic_status_ss << std::setfill('0') << std::setw(2) << std::hex << static_cast<uint16_t>(mip_buffer[i]);
    MICROSTRAIN_INFO(node_, "Device Diagnostic Info:");
    MICROSTRAIN_INFO(node_, "  0x%s", basic_status_ss.str().c_str());
    return res.success;
  }

  // Get the device info to determine the model number if we need it
  MICROSTRAIN_DEBUG(node_, "Looks like this is a philo device, fetching device info");
  mip::commands_base::BaseDeviceInfo device_info;
  res.success = !!(mip_cmd_result = config_->mip_device_->getDeviceInfo(&device_info));
  if (!res.success)
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to fetch device info");
    return res.success;
  }

  // Parse out the high digits in the model number
  MICROSTRAIN_DEBUG(node_, "Parsing model number %s", device_info.model_number);
  const std::string& model_number_str = device_info.model_number;
  const size_t dash_index = model_number_str.find('-');
  uint16_t model_number = 0;
  if (dash_index != std::string::npos)
    model_number = std::stoi(model_number_str.substr(0, dash_index));
  else
    MICROSTRAIN_WARN(node_, "Model number not found in string, if this is a philo product, the call to DEVICE_STATUS will fail, otherwise this is fine");
  MICROSTRAIN_DEBUG(node_, "Model number u16 is %u", model_number);

  // Try the command again with the model number
  mip_buffer[0] = model_number & 0xFF;
  mip_buffer[1] = (model_number >> 8) & 0xFF;
  mip_buffer[2] = 2;
  res.success = !!(mip_cmd_result = mip::C::mip_interface_run_command_with_response(&(config_->mip_device_->device()), mip::commands_3dm::DESCRIPTOR_SET, 0x64, mip_buffer, 3, 0x90, mip_buffer, &mip_buffer_out_size));

  // Just print the raw bytes on success
  if (res.success)
  {
    std::stringstream basic_status_ss;
    for (uint8_t i = 0; i < mip_buffer_out_size; i++)
      basic_status_ss << std::setfill('0') << std::setw(2) << std::hex << static_cast<uint16_t>(mip_buffer[i]);
    MICROSTRAIN_INFO(node_, "Device Diagnostic Info:");
    MICROSTRAIN_INFO(node_, "  0x%s", basic_status_ss.str().c_str());
  }
  else
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to fetch device diagnostic info");
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Auto Angular Zupt Enable/Threshold Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

// Set zero angular-rate update threshold
bool Services::setZeroAngleUpdateThreshold(SetZeroAngleUpdateThresholdServiceMsg::Request& req,
                                                          SetZeroAngleUpdateThresholdServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Setting auto angular ZUPT");
  MICROSTRAIN_DEBUG(node_, "  enable = %d", req.enable);
  MICROSTRAIN_DEBUG(node_, "  threshold = %f", req.threshold);

  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::writeAutoAngularZupt(*(config_->mip_device_), req.enable, req.threshold));
  if (!res.success)
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to set auto angular ZUPT");
  else
    MICROSTRAIN_DEBUG(node_, "Set auto angular ZUPT");

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Auto Angular Zupt Enable/Threshold Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::getZeroAngleUpdateThreshold(GetZeroAngleUpdateThresholdServiceMsg::Request& req,
                                                          GetZeroAngleUpdateThresholdServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Getting auto angular ZUPT");

  uint8_t enable;
  float threshold;
  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::readAutoAngularZupt(*(config_->mip_device_), &enable, &threshold));
  if (res.success)
  {
    MICROSTRAIN_DEBUG(node_, "Got auto angular ZUPT");
    MICROSTRAIN_DEBUG(node_, "  enable = %d", enable);
    MICROSTRAIN_DEBUG(node_, "  threshold = %f", threshold);
    res.enable = enable;
    res.threshold = threshold;
  }
  else
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to get auto angular ZUPT");
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Auto Velocity Zupt Enable/Threshold Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::setZeroVelocityUpdateThreshold(SetZeroVelocityUpdateThresholdServiceMsg::Request& req,
                                                             SetZeroVelocityUpdateThresholdServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Setting auto vel ZUPT");
  MICROSTRAIN_DEBUG(node_, "  enable = %d", req.enable);
  MICROSTRAIN_DEBUG(node_, "  threshold = %f", req.threshold);

  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::writeAutoZupt(*(config_->mip_device_), req.enable, req.threshold));
  if (!res.success)
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to set auto vel ZUPT");
  else
    MICROSTRAIN_DEBUG(node_, "Set auto vel ZUPT");

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Auto Velocity Zupt Enable/Threshold Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::getZeroVelocityUpdateThreshold(GetZeroVelocityUpdateThresholdServiceMsg::Request& req,
                                                             GetZeroVelocityUpdateThresholdServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Getting auto vel ZUPT");

  uint8_t enable;
  float threshold;
  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::readAutoZupt(*(config_->mip_device_), &enable, &threshold));
  if (res.success)
  {
    MICROSTRAIN_DEBUG(node_, "Got auto vel ZUPT");
    MICROSTRAIN_DEBUG(node_, "  enable = %d", enable);
    MICROSTRAIN_DEBUG(node_, "  threshold = %f", threshold);
    res.enable = enable;
    res.threshold = threshold;
  }
  else
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to get auto vel ZUPT");
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Tare Orientation Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::setTareOrientation(SetTareOrientationServiceMsg::Request& req,
                                               SetTareOrientationServiceMsg::Response& res)
{
  const auto axis = static_cast<mip::commands_filter::TareOrientation::MipTareAxes>(req.axis);
  MICROSTRAIN_DEBUG(node_, "Setting tare orientation %d", req.axis);

  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::writeTareOrientation(*(config_->mip_device_), axis));
  if (!res.success)
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to set tare orientation");
  else
    MICROSTRAIN_DEBUG(node_, "Set tare orientation");

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Accel Noise 1-Sigma Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::setAccelNoise(SetAccelNoiseServiceMsg::Request& req, SetAccelNoiseServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Setting accel noise to [%f, %f, %f]", req.noise.x, req.noise.y, req.noise.z);

  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::writeAccelNoise(*(config_->mip_device_), req.noise.x, req.noise.y, req.noise.z));
  if (!res.success)
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to set accel noise");
  else
    MICROSTRAIN_DEBUG(node_, "Set accel noise");

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Accel Noise 1-Sigma Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::getAccelNoise(GetAccelNoiseServiceMsg::Request& req, GetAccelNoiseServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Getting accel noise");

  float x, y, z;
  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::readAccelNoise(*(config_->mip_device_), &x, &y, &z));
  if (res.success)
  {
    MICROSTRAIN_DEBUG(node_, "Got accel noise [%f, %f, %f]", x, y, z);
    res.noise.x = x;
    res.noise.y = y;
    res.noise.z = z;
  }
  else
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to get accel noise");
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Gyro Noise 1-Sigma Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::setGyroNoise(SetGyroNoiseServiceMsg::Request& req, SetGyroNoiseServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Setting gyro noise to [%f, %f, %f]", req.noise.x, req.noise.y, req.noise.z);

  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::writeGyroNoise(*(config_->mip_device_), req.noise.x, req.noise.y, req.noise.z));
  if (!res.success)
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to set gyro noise");
  else
    MICROSTRAIN_DEBUG(node_, "Set gyro noise");

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Gyro Noise 1-Sigma Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::getGyroNoise(GetGyroNoiseServiceMsg::Request& req, GetGyroNoiseServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Getting gyro noise");

  float x, y, z;
  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::readGyroNoise(*(config_->mip_device_), &x, &y, &z));
  if (res.success)
  {
    MICROSTRAIN_DEBUG(node_, "Got gyro noise [%f, %f, %f]", x, y, z);
    res.noise.x = x;
    res.noise.y = y;
    res.noise.z = z;
  }
  else
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to get gyro noise");
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Magnetometer Noise 1-Sigma Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::setMagNoise(SetMagNoiseServiceMsg::Request& req, SetMagNoiseServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Setting mag noise to [%f, %f, %f]", req.noise.x, req.noise.y, req.noise.z);

  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::writeHardIronOffsetNoise(*(config_->mip_device_), req.noise.x, req.noise.y, req.noise.z));
  if (!res.success)
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to set mag noise");
  else
    MICROSTRAIN_DEBUG(node_, "Set mag noise");

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Magnetometer Noise 1-Sigma Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::getMagNoise(GetMagNoiseServiceMsg::Request& req, GetMagNoiseServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Getting mag noise");

  float x, y, z;
  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::readHardIronOffsetNoise(*(config_->mip_device_), &x, &y, &z));
  if (res.success)
  {
    MICROSTRAIN_DEBUG(node_, "Got mag noise [%f, %f, %f]", x, y, z);
    res.noise.x = x;
    res.noise.y = y;
    res.noise.z = z;
  }
  else
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to get mag noise");
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Gyro Bias Model Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::setGyroBiasModel(SetGyroBiasModelServiceMsg::Request& req,
                                              SetGyroBiasModelServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Setting gyro bias model:");
  MICROSTRAIN_DEBUG(node_, "  beta = [%f, %f, %f]", req.beta_vector.x, req.beta_vector.y, req.beta_vector.z);
  MICROSTRAIN_DEBUG(node_, "  noise = [%f, %f, %f]", req.noise_vector.x, req.noise_vector.y, req.noise_vector.z);

  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::writeGyroBiasModel(*(config_->mip_device_), req.beta_vector.x, req.beta_vector.y, req.beta_vector.z, req.noise_vector.x, req.noise_vector.y, req.noise_vector.z));
  if (!res.success)
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to set gyro bias model");
  else
    MICROSTRAIN_DEBUG(node_, "Set gyro bias model");

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Gyro Bias Model Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::getGyroBiasModel(GetGyroBiasModelServiceMsg::Request& req,
                                              GetGyroBiasModelServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Getting gyro bias model");

  float x_beta, y_beta, z_beta, x, y, z;
  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::readGyroBiasModel(*(config_->mip_device_), &x_beta, &y_beta, &z_beta, &x, &y, &z));
  if (res.success)
  {
    MICROSTRAIN_DEBUG(node_, "Got gyro bias model:");
    MICROSTRAIN_DEBUG(node_, "  beta = [%f, %f, %f]", x_beta, y_beta, z_beta);
    MICROSTRAIN_DEBUG(node_, "  noise = [%f, %f, %f]", x, y, z);
    res.beta_vector.x = x_beta;
    res.beta_vector.y = y_beta;
    res.beta_vector.z = z_beta;
    res.noise_vector.x = x;
    res.noise_vector.y = y;
    res.noise_vector.z = z;
  }
  else
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to get gyro bias model");
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Accel Bias Model Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::setAccelBiasModel(SetAccelBiasModelServiceMsg::Request& req,
                                               SetAccelBiasModelServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Setting accel bias model:");
  MICROSTRAIN_DEBUG(node_, "  beta = [%f, %f, %f]", req.beta_vector.x, req.beta_vector.y, req.beta_vector.z);
  MICROSTRAIN_DEBUG(node_, "  noise = [%f, %f, %f]", req.noise_vector.x, req.noise_vector.y, req.noise_vector.z);

  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::writeAccelBiasModel(*(config_->mip_device_), req.beta_vector.x, req.beta_vector.y, req.beta_vector.z, req.noise_vector.x, req.noise_vector.y, req.noise_vector.z));
  if (!res.success)
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to set accel bias model");
  else
    MICROSTRAIN_DEBUG(node_, "Set accel bias model");

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Accel Bias Model Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::getAccelBiasModel(GetAccelBiasModelServiceMsg::Request& req,
                                               GetAccelBiasModelServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Getting accel bias model");

  float x_beta, y_beta, z_beta, x, y, z;
  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::readAccelBiasModel(*(config_->mip_device_), &x_beta, &y_beta, &z_beta, &x, &y, &z));
  if (res.success)
  {
    MICROSTRAIN_DEBUG(node_, "Got accel bias model:");
    MICROSTRAIN_DEBUG(node_, "  beta = [%f, %f, %f]", x_beta, y_beta, z_beta);
    MICROSTRAIN_DEBUG(node_, "  noise = [%f, %f, %f]", x, y, z);
    res.beta_vector.x = x_beta;
    res.beta_vector.y = y_beta;
    res.beta_vector.z = z_beta;
    res.noise_vector.x = x;
    res.noise_vector.y = y;
    res.noise_vector.z = z;
  }
  else
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to get accel bias model");
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Accel Magnitude Error Adaptive Measurement Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::setGravityAdaptiveVals(SetGravityAdaptiveValsServiceMsg::Request& req,
                                                    SetGravityAdaptiveValsServiceMsg::Response& res)
{
  const auto mode = static_cast<mip::commands_filter::FilterAdaptiveMeasurement>(req.enable);
  MICROSTRAIN_DEBUG(node_, "Setting accel magnitude error adaptive measurement:");
  MICROSTRAIN_DEBUG(node_, "  mode = %d", static_cast<uint8_t>(mode));
  MICROSTRAIN_DEBUG(node_, "  low pass cutoff frequency = %f", req.low_pass_cutoff);
  MICROSTRAIN_DEBUG(node_, "  low limit = %f", req.low_limit);
  MICROSTRAIN_DEBUG(node_, "  high limit = %f", req.high_limit);
  MICROSTRAIN_DEBUG(node_, "  low limit uncertainty = %f", req.low_limit_1sigma);
  MICROSTRAIN_DEBUG(node_, "  high limit uncertainty = %f", req.high_limit_1sigma);
  MICROSTRAIN_DEBUG(node_, "  minimum uncertainty = %f", req.min_1sigma);

  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::writeAccelMagnitudeErrorAdaptiveMeasurement(*(config_->mip_device_), mode, req.low_pass_cutoff, req.low_limit, req.high_limit, req.low_limit_1sigma, req.high_limit_1sigma, req.min_1sigma));
  if (!res.success)
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to set accel magnitude error adaptive measurement");
  else
    MICROSTRAIN_DEBUG(node_, "Set accel magnitude error adaptive measurement");

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Accel Magnitude Error Adaptive Measurement Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::getGravityAdaptiveVals(GetGravityAdaptiveValsServiceMsg::Request& req,
                                                    GetGravityAdaptiveValsServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Getting accel magnitude error adaptive measurements");

  mip::commands_filter::FilterAdaptiveMeasurement mode;
  float frequency, low_limit, high_limit, low_limit_uncertainty, high_limit_uncertainty, minimum_uncertainty;
  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::readAccelMagnitudeErrorAdaptiveMeasurement(*(config_->mip_device_), &mode, &frequency, &low_limit, &high_limit, &low_limit_uncertainty, &high_limit_uncertainty, &minimum_uncertainty));
  if (res.success)
  {
    MICROSTRAIN_DEBUG(node_, "Got accel magnitude error adaptive measurement:");
    MICROSTRAIN_DEBUG(node_, "  mode = %d", static_cast<uint8_t>(mode));
    MICROSTRAIN_DEBUG(node_, "  low pass cutoff frequency = %f", frequency);
    MICROSTRAIN_DEBUG(node_, "  low limit = %f", low_limit);
    MICROSTRAIN_DEBUG(node_, "  high limit = %f", high_limit);
    MICROSTRAIN_DEBUG(node_, "  low limit uncertainty = %f", low_limit_uncertainty);
    MICROSTRAIN_DEBUG(node_, "  high limit uncertainty = %f", high_limit_uncertainty);
    MICROSTRAIN_DEBUG(node_, "  minimum uncertainty = %f", minimum_uncertainty);
    res.enable = static_cast<float>(mode);
    res.low_pass_cutoff = frequency;
    res.low_limit = low_limit;
    res.high_limit = high_limit;
    res.low_limit_1sigma = low_limit_uncertainty;
    res.high_limit_1sigma = high_limit_uncertainty;
    res.min_1sigma = minimum_uncertainty;
  }
  else
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to get accel magnitude error adaptive measurements");
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Magnetometer Magnitude Error Adaptive Measurement Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::setMagAdaptiveVals(SetMagAdaptiveValsServiceMsg::Request& req,
                                                SetMagAdaptiveValsServiceMsg::Response& res)
{
  const auto mode = static_cast<mip::commands_filter::FilterAdaptiveMeasurement>(req.enable);
  MICROSTRAIN_DEBUG(node_, "Setting mag magnitude error adaptive measurement:");
  MICROSTRAIN_DEBUG(node_, "  mode = %d", static_cast<uint8_t>(mode));
  MICROSTRAIN_DEBUG(node_, "  low pass cutoff frequency = %f", req.low_pass_cutoff);
  MICROSTRAIN_DEBUG(node_, "  low limit = %f", req.low_limit);
  MICROSTRAIN_DEBUG(node_, "  high limit = %f", req.high_limit);
  MICROSTRAIN_DEBUG(node_, "  low limit uncertainty = %f", req.low_limit_1sigma);
  MICROSTRAIN_DEBUG(node_, "  high limit uncertainty = %f", req.high_limit_1sigma);
  MICROSTRAIN_DEBUG(node_, "  minimum uncertainty = %f", req.min_1sigma);

  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::writeMagMagnitudeErrorAdaptiveMeasurement(*(config_->mip_device_), mode, req.low_pass_cutoff, req.low_limit, req.high_limit, req.low_limit_1sigma, req.high_limit_1sigma, req.min_1sigma));
  if (!res.success)
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to set mag magnitude error adaptive measurement");
  else
    MICROSTRAIN_DEBUG(node_, "Set mag magnitude error adaptive measurement");

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Magnetometer Magnitude Error Adaptive Measurement Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::getMagAdaptiveVals(GetMagAdaptiveValsServiceMsg::Request& req,
                                                GetMagAdaptiveValsServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Getting mag magnitude error adaptive measurements");

  mip::commands_filter::FilterAdaptiveMeasurement mode;
  float frequency, low_limit, high_limit, low_limit_uncertainty, high_limit_uncertainty, minimum_uncertainty;
  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::readMagMagnitudeErrorAdaptiveMeasurement(*(config_->mip_device_), &mode, &frequency, &low_limit, &high_limit, &low_limit_uncertainty, &high_limit_uncertainty, &minimum_uncertainty));
  if (res.success)
  {
    MICROSTRAIN_DEBUG(node_, "Got mag magnitude error adaptive measurement:");
    MICROSTRAIN_DEBUG(node_, "  mode = %d", static_cast<uint8_t>(mode));
    MICROSTRAIN_DEBUG(node_, "  low pass cutoff frequency = %f", frequency);
    MICROSTRAIN_DEBUG(node_, "  low limit = %f", low_limit);
    MICROSTRAIN_DEBUG(node_, "  high limit = %f", high_limit);
    MICROSTRAIN_DEBUG(node_, "  low limit uncertainty = %f", low_limit_uncertainty);
    MICROSTRAIN_DEBUG(node_, "  high limit uncertainty = %f", high_limit_uncertainty);
    MICROSTRAIN_DEBUG(node_, "  minimum uncertainty = %f", minimum_uncertainty);
    res.enable = static_cast<float>(mode);
    res.low_pass_cutoff = frequency;
    res.low_limit = low_limit;
    res.high_limit = high_limit;
    res.low_limit_1sigma = low_limit_uncertainty;
    res.high_limit_1sigma = high_limit_uncertainty;
    res.min_1sigma = minimum_uncertainty;
  }
  else
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to get mag magnitude error adaptive measurements");
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Magnetometer Dip Angle Error Adaptive Measurement Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::setMagDipAdaptiveVals(SetMagDipAdaptiveValsServiceMsg::Request& req,
                                                    SetMagDipAdaptiveValsServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Setting mag dip angle error adaptive measurement:");
  MICROSTRAIN_DEBUG(node_, "  enable = %f", req.enable);
  MICROSTRAIN_DEBUG(node_, "  low pass cutoff frequency = %f", req.low_pass_cutoff);
  MICROSTRAIN_DEBUG(node_, "  high limit = %f", req.high_limit);
  MICROSTRAIN_DEBUG(node_, "  high limit uncertainty = %f", req.high_limit_1sigma);
  MICROSTRAIN_DEBUG(node_, "  minimum uncertainty = %f", req.min_1sigma);

  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::writeMagDipAngleErrorAdaptiveMeasurement(*(config_->mip_device_), static_cast<bool>(req.enable), req.low_pass_cutoff, req.high_limit, req.high_limit_1sigma, req.min_1sigma));
  if (!res.success)
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to set mag dip angle error adaptive measurement");
  else
    MICROSTRAIN_DEBUG(node_, "Set mag dip angle error adaptive measurement");

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Magnetometer Dip Angle Error Adaptive Measurement Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::getMagDipAdaptiveVals(GetMagDipAdaptiveValsServiceMsg::Request& req,
                                                    GetMagDipAdaptiveValsServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Getting mag dip angle error adaptive measurements");

  bool enable;
  float frequency, high_limit, high_limit_uncertainty, minimum_uncertainty;
  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::readMagDipAngleErrorAdaptiveMeasurement(*(config_->mip_device_), &enable, &frequency, &high_limit, &high_limit_uncertainty, &minimum_uncertainty));
  if (res.success)
  {
    MICROSTRAIN_DEBUG(node_, "Got mag dip angle error adaptive measurement:");
    MICROSTRAIN_DEBUG(node_, "  mode = %d", enable);
    MICROSTRAIN_DEBUG(node_, "  low pass cutoff frequency = %f", frequency);
    MICROSTRAIN_DEBUG(node_, "  high limit = %f", high_limit);
    MICROSTRAIN_DEBUG(node_, "  high limit uncertainty = %f", high_limit_uncertainty);
    MICROSTRAIN_DEBUG(node_, "  minimum uncertainty = %f", minimum_uncertainty);
    res.enable = static_cast<float>(enable);
    res.low_pass_cutoff = frequency;
    res.high_limit = high_limit;
    res.high_limit_1sigma = high_limit_uncertainty;
    res.min_1sigma = minimum_uncertainty;
  }
  else
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to get mag dip angle error adaptive measurements");
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Vehicle Dynamics Mode Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::setDynamicsMode(SetDynamicsModeServiceMsg::Request& req,
                                            SetDynamicsModeServiceMsg::Response& res)
{
  const auto mode = static_cast<mip::commands_filter::VehicleDynamicsMode::DynamicsMode>(req.mode);
  MICROSTRAIN_DEBUG(node_, "Setting vehicle dynamics mode to %d", req.mode);

  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::writeVehicleDynamicsMode(*(config_->mip_device_), mode));
  if (!res.success)
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to set vehicle dynamics mode");
  else
    MICROSTRAIN_DEBUG(node_, "Set vehicle dynamics mode");

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Vehicle Dynamics Mode Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::getDynamicsMode(GetDynamicsModeServiceMsg::Request& req,
                                            GetDynamicsModeServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Getting vehicle dynamics mode");

  mip::commands_filter::VehicleDynamicsMode::DynamicsMode mode;
  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::readVehicleDynamicsMode(*(config_->mip_device_), &mode));
  if (res.success)
  {
    MICROSTRAIN_DEBUG(node_, "Got vehicle dynamics mode %d", static_cast<int8_t>(mode));
    res.mode = static_cast<int8_t>(mode);
  }
  else
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to get vehicle dynamics mode");
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Change Device Settings
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::deviceSettings(DeviceSettingsServiceMsg::Request& req,
                                          DeviceSettingsServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Processing device settings command with function selector %d", req.function_selector);

  // We need to change the timeout to allow for this longer command
  const int32_t old_mip_sdk_timeout = config_->mip_device_->device().baseReplyTimeout();
  config_->mip_device_->device().setBaseReplyTimeout(10000);  // 10 seconds should be enough

  mip::CmdResult mip_cmd_result;
  switch (req.function_selector)
  {
    // Save
    case 3:
      MICROSTRAIN_DEBUG(node_, "Saving settings as startup");
      res.success = !!(mip_cmd_result = mip::commands_3dm::saveDeviceSettings(*(config_->mip_device_)));
      break;

    // Load Saved Settings
    case 4:
      MICROSTRAIN_DEBUG(node_, "Loading saved settings");
      res.success = !!(mip_cmd_result = mip::commands_3dm::loadDeviceSettings(*(config_->mip_device_)));
      break;

    // Load Default Settings
    case 5:
      MICROSTRAIN_DEBUG(node_, "Loading default settings");
      res.success = !!(mip_cmd_result = mip::commands_3dm::defaultDeviceSettings(*(config_->mip_device_)));
      break;

    // Unsupported function selector
    default:
      MICROSTRAIN_ERROR(node_, "Unsupported function selector for device_settins service: %d", req.function_selector);
      res.success = false;
      break;
  }

  if (!mip_cmd_result)
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to run device settings command");
  else if (res.success)
    MICROSTRAIN_DEBUG(node_, "Ran device settings command");

  // Reset the timeout
  config_->mip_device_->device().setBaseReplyTimeout(old_mip_sdk_timeout);

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Commanded Velocity Zupt Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::commandedVelZupt(TriggerServiceMsg::Request& req, TriggerServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Commanding vel ZUPT");

  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::commandedZupt(*(config_->mip_device_)));
  if (!res.success)
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to send commanded vel ZUPT");
  else
    MICROSTRAIN_DEBUG(node_, "Sent commanded vel ZUPT");

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Commanded Angular Rate Zupt Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::commandedAngRateZupt(TriggerServiceMsg::Request& req, TriggerServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Commanding angular ZUPT");

  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::commandedAngularZupt(*(config_->mip_device_)));
  if (!res.success)
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to send commanded angular ZUPT");
  else
    MICROSTRAIN_DEBUG(node_, "Sent commanded angular ZUPT");

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// External Heading Update Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::externalHeadingUpdate(ExternalHeadingUpdateServiceMsg::Request& req,
                                                  ExternalHeadingUpdateServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Updating with external heading");
  MICROSTRAIN_DEBUG(node_, "  gps time of week = %f", req.gps_tow);
  MICROSTRAIN_DEBUG(node_, "  gps week number = %u", req.gps_week_number);
  MICROSTRAIN_DEBUG(node_, "  heading (radians) = %f", req.heading_rad);
  MICROSTRAIN_DEBUG(node_, "  heading uncertainty = %f", req.heading_1sigma_rad);
  MICROSTRAIN_DEBUG(node_, "  heading type = %d", req.heading_type);

  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::externalHeadingUpdateWithTime(*(config_->mip_device_), req.gps_tow, req.gps_week_number, req.heading_rad, req.heading_1sigma_rad, req.heading_type));
  if (!res.success)
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to update with external heading");
  else
    MICROSTRAIN_DEBUG(node_, "Sent external heading update command");

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Relative Position Reference Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::setRelativePositionReference(SetRelativePositionReferenceServiceMsg::Request& req,
                                                          SetRelativePositionReferenceServiceMsg::Response& res)
{
  const auto frame = static_cast<mip::commands_filter::FilterReferenceFrame>(req.frame);
  double coordinates[3] = {req.position.x, req.position.y, req.position.z};
  MICROSTRAIN_DEBUG(node_, "Setting relative position reference");
  MICROSTRAIN_DEBUG(node_, "  source = %d", req.source);
  MICROSTRAIN_DEBUG(node_, "  frame = %d", req.frame);
  MICROSTRAIN_DEBUG(node_, "  position = [%f, %f, %f]", coordinates[0], coordinates[1], coordinates[2]);

  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::writeRelPosConfiguration(*(config_->mip_device_), req.source, frame, coordinates));
  if (!res.success)
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to set relative position reference");
  else
    MICROSTRAIN_DEBUG(node_, "Set relative position reference");

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Relative Position Reference Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Services::getRelativePositionReference(GetRelativePositionReferenceServiceMsg::Request& req,
                                                          GetRelativePositionReferenceServiceMsg::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Getting relative position reference");

  uint8_t source;
  mip::commands_filter::FilterReferenceFrame frame;
  double coordinates[3];
  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::readRelPosConfiguration(*(config_->mip_device_), &source, &frame, coordinates));
  if (res.success)
  {
    MICROSTRAIN_DEBUG(node_, "Got relative position reference");
    MICROSTRAIN_DEBUG(node_, "  source = %d", source);
    MICROSTRAIN_DEBUG(node_, "  frame = %d", static_cast<uint8_t>(frame));
    MICROSTRAIN_DEBUG(node_, "  position = [%f, %f, %f]", coordinates[0], coordinates[1], coordinates[2]);
    res.source = source;
    res.frame = static_cast<uint8_t>(frame);
    res.position.x = coordinates[0];
    res.position.y = coordinates[1];
    res.position.z = coordinates[2];
  }
  else
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to get relative position reference");
  }

  return res.success;
}

bool Services::setFilterSpeedLeverArm(SetFilterSpeedLeverArmServiceMsg::Request& req,
                              SetFilterSpeedLeverArmServiceMsg::Response& res)
{
  float offset[3] = {req.offset.x, req.offset.y, req.offset.z};
  MICROSTRAIN_DEBUG(node_, "Setting filter speed lever arm [%f, %f, %f]", offset[0], offset[1], offset[2]);

  mip::CmdResult mip_cmd_result;
  res.success = !!(mip_cmd_result = mip::commands_filter::writeSpeedLeverArm(*(config_->mip_device_), 1, offset));
  if (!res.success)
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to set filter speed lever arm");
  else
    MICROSTRAIN_DEBUG(node_, "Set filter speed lever arm");

  return res.success;
}

}  // namespace microstrain
