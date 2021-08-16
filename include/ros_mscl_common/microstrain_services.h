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
#ifndef _MICROSTRAIN_SERVICES_H
#define _MICROSTRAIN_SERVICES_H

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Include Files
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#include "microstrain_defs.h"
#include "microstrain_ros_funcs.h"
#include "microstrain_config.h"


/////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief Contains functions for micostrain driver
///
/////////////////////////////////////////////////////////////////////////////////////////////////////
namespace Microstrain
{

///
/// \brief Contains publishers for microstrain node
///
class MicrostrainServices
{
 public:
  MicrostrainServices() = default;
  MicrostrainServices(RosNodeType* node, MicrostrainConfig* config);

  bool configure_services();

  bool device_report(TriggerServiceMsg::Request &req, TriggerServiceMsg::Response &res);
  bool get_basic_status(TriggerServiceMsg::Request &req, TriggerServiceMsg::Response &res);
  bool get_diagnostic_report(TriggerServiceMsg::Request &req, TriggerServiceMsg::Response &res);

  bool set_accel_bias(SetAccelBiasServiceMsg::Request &req, SetAccelBiasServiceMsg::Response &res);
  bool get_accel_bias(GetAccelBiasServiceMsg::Request &req, GetAccelBiasServiceMsg::Response &res);

  bool set_gyro_bias(SetGyroBiasServiceMsg::Request &req, SetGyroBiasServiceMsg::Response &res);
  bool get_gyro_bias(GetGyroBiasServiceMsg::Request &req, GetGyroBiasServiceMsg::Response &res);

  bool gyro_bias_capture(TriggerServiceMsg::Request &req, TriggerServiceMsg::Response &res);

  bool set_hard_iron_values(SetHardIronValuesServiceMsg::Request &req, SetHardIronValuesServiceMsg::Response &res);
  bool get_hard_iron_values(GetHardIronValuesServiceMsg::Request &req, GetHardIronValuesServiceMsg::Response &res);

  bool set_soft_iron_matrix(SetSoftIronMatrixServiceMsg::Request &req, SetSoftIronMatrixServiceMsg::Response &res);
  bool get_soft_iron_matrix(GetSoftIronMatrixServiceMsg::Request &req, GetSoftIronMatrixServiceMsg::Response &res);

  bool set_complementary_filter(SetComplementaryFilterServiceMsg::Request &req, SetComplementaryFilterServiceMsg::Response &res);
  bool get_complementary_filter(GetComplementaryFilterServiceMsg::Request &req, GetComplementaryFilterServiceMsg::Response &res);

  bool set_coning_sculling_comp(SetConingScullingCompServiceMsg::Request &req, SetConingScullingCompServiceMsg::Response &res);
  bool get_coning_sculling_comp(GetConingScullingCompServiceMsg::Request &req, GetConingScullingCompServiceMsg::Response &res);

  bool set_sensor2vehicle_rotation(SetSensor2VehicleRotationServiceMsg::Request &req, SetSensor2VehicleRotationServiceMsg::Response &res);
  bool get_sensor2vehicle_rotation(GetSensor2VehicleRotationServiceMsg::Request &req, GetSensor2VehicleRotationServiceMsg::Response &res);

  bool set_sensor2vehicle_offset(SetSensor2VehicleOffsetServiceMsg::Request &req, SetSensor2VehicleOffsetServiceMsg::Response &res);
  bool get_sensor2vehicle_offset(GetSensor2VehicleOffsetServiceMsg::Request &req, GetSensor2VehicleOffsetServiceMsg::Response &res);

  bool get_sensor2vehicle_transformation(GetSensor2VehicleTransformationServiceMsg::Request &req, GetSensor2VehicleTransformationServiceMsg::Response &res);
    
  bool reset_filter(EmptyServiceMsg::Request &req, EmptyServiceMsg::Response &resp);

  bool init_filter_euler(InitFilterEulerServiceMsg::Request &req, InitFilterEulerServiceMsg::Response &res);
  bool init_filter_heading(InitFilterHeadingServiceMsg::Request &req, InitFilterHeadingServiceMsg::Response &res);

  bool set_heading_source(SetHeadingSourceServiceMsg::Request &req, SetHeadingSourceServiceMsg::Response &res);
  bool get_heading_source(GetHeadingSourceServiceMsg::Request &req, GetHeadingSourceServiceMsg::Response &res);

  bool set_reference_position(SetReferencePositionServiceMsg::Request &req, SetReferencePositionServiceMsg::Response &res);
  bool get_reference_position(GetReferencePositionServiceMsg::Request &req, GetReferencePositionServiceMsg::Response &res);

  bool set_estimation_control_flags(SetEstimationControlFlagsServiceMsg::Request &req, SetEstimationControlFlagsServiceMsg::Response &res);
  bool get_estimation_control_flags(GetEstimationControlFlagsServiceMsg::Request &req, GetEstimationControlFlagsServiceMsg::Response &res);

  bool set_dynamics_mode(SetDynamicsModeServiceMsg::Request &req, SetDynamicsModeServiceMsg::Response &res);
  bool get_dynamics_mode(GetDynamicsModeServiceMsg::Request &req, GetDynamicsModeServiceMsg::Response &res);

  bool set_zero_angle_update_threshold(SetZeroAngleUpdateThresholdServiceMsg::Request &req, SetZeroAngleUpdateThresholdServiceMsg::Response &res);
  bool get_zero_angle_update_threshold(GetZeroAngleUpdateThresholdServiceMsg::Request &req, GetZeroAngleUpdateThresholdServiceMsg::Response &res);
  
  bool set_zero_velocity_update_threshold(SetZeroVelocityUpdateThresholdServiceMsg::Request &req, SetZeroVelocityUpdateThresholdServiceMsg::Response &res);
  bool get_zero_velocity_update_threshold(GetZeroVelocityUpdateThresholdServiceMsg::Request &req, GetZeroVelocityUpdateThresholdServiceMsg::Response &res);

  bool set_tare_orientation(SetTareOrientationServiceMsg::Request &req, SetTareOrientationServiceMsg::Response &res);
  
  bool commanded_vel_zupt(TriggerServiceMsg::Request &req, TriggerServiceMsg::Response &res);
  bool commanded_ang_rate_zupt(TriggerServiceMsg::Request &req, TriggerServiceMsg::Response &res);

  bool set_accel_noise(SetAccelNoiseServiceMsg::Request &req, SetAccelNoiseServiceMsg::Response &res);
  bool get_accel_noise(GetAccelNoiseServiceMsg::Request &req, GetAccelNoiseServiceMsg::Response &res);

  bool set_gyro_noise(SetGyroNoiseServiceMsg::Request &req, SetGyroNoiseServiceMsg::Response &res);
  bool get_gyro_noise(GetGyroNoiseServiceMsg::Request &req, GetGyroNoiseServiceMsg::Response &res);

  bool set_mag_noise(SetMagNoiseServiceMsg::Request &req, SetMagNoiseServiceMsg::Response &res);
  bool get_mag_noise(GetMagNoiseServiceMsg::Request &req, GetMagNoiseServiceMsg::Response &res);

  bool set_gyro_bias_model(SetGyroBiasModelServiceMsg::Request &req, SetGyroBiasModelServiceMsg::Response &res);
  bool get_gyro_bias_model(GetGyroBiasModelServiceMsg::Request &req, GetGyroBiasModelServiceMsg::Response &res);

  bool set_accel_bias_model(SetAccelBiasModelServiceMsg::Request &req, SetAccelBiasModelServiceMsg::Response &res);
  bool get_accel_bias_model(GetAccelBiasModelServiceMsg::Request &req, GetAccelBiasModelServiceMsg::Response &res);

  bool set_gravity_adaptive_vals(SetGravityAdaptiveValsServiceMsg::Request &req, SetGravityAdaptiveValsServiceMsg::Response &res);
  bool get_gravity_adaptive_vals(GetGravityAdaptiveValsServiceMsg::Request &req, GetGravityAdaptiveValsServiceMsg::Response &res);

  bool set_mag_adaptive_vals(SetMagAdaptiveValsServiceMsg::Request &req, SetMagAdaptiveValsServiceMsg::Response &res);
  bool get_mag_adaptive_vals(GetMagAdaptiveValsServiceMsg::Request &req, GetMagAdaptiveValsServiceMsg::Response &res);

  bool set_mag_dip_adaptive_vals(SetMagDipAdaptiveValsServiceMsg::Request &req, SetMagDipAdaptiveValsServiceMsg::Response &res);
  bool get_mag_dip_adaptive_vals(GetMagDipAdaptiveValsServiceMsg::Request &req, GetMagDipAdaptiveValsServiceMsg::Response &res);
  
  bool external_heading_update(ExternalHeadingUpdateServiceMsg::Request &req, ExternalHeadingUpdateServiceMsg::Response &res);

  bool set_relative_position_reference(SetRelativePositionReferenceServiceMsg::Request &req, SetRelativePositionReferenceServiceMsg::Response &res);
  bool get_relative_position_reference(GetRelativePositionReferenceServiceMsg::Request &req, GetRelativePositionReferenceServiceMsg::Response &res);

  bool device_settings(DeviceSettingsServiceMsg::Request &req, DeviceSettingsServiceMsg::Response &res);

  void get_basic_status_ptr(const std::shared_ptr<TriggerServiceMsg::Request> req, std::shared_ptr<TriggerServiceMsg::Response> res);

 private:
  RosNodeType* m_node;
  MicrostrainConfig* m_config;

  TriggerServiceType m_get_basic_status_service;
  TriggerServiceType m_get_diagnostic_report_service;
  TriggerServiceType m_device_report_service;
  SetTareOrientationServiceType m_set_tare_orientation_service;
  SetComplementaryFilterServiceType m_set_complementary_filter_service;
  GetComplementaryFilterServiceType m_get_complementary_filter_service;
  SetSensor2VehicleRotationServiceType m_set_sensor2vehicle_rotation_service;
  GetSensor2VehicleRotationServiceType m_get_sensor2vehicle_rotation_service;
  SetSensor2VehicleOffsetServiceType m_set_sensor2vehicle_offset_service;
  GetSensor2VehicleOffsetServiceType m_get_sensor2vehicle_offset_service;
  GetSensor2VehicleTransformationServiceType m_get_sensor2vehicle_transformation_service;
  SetAccelBiasServiceType m_set_accel_bias_service;
  GetAccelBiasServiceType m_get_accel_bias_service;
  SetGyroBiasServiceType m_set_gyro_bias_service;
  GetGyroBiasServiceType m_get_gyro_bias_service;
  TriggerServiceType m_gyro_bias_capture_service;
  SetHardIronValuesServiceType m_set_hard_iron_values_service;
  GetHardIronValuesServiceType m_get_hard_iron_values_service;
  SetSoftIronMatrixServiceType m_set_soft_iron_matrix_service;
  GetSoftIronMatrixServiceType m_get_soft_iron_matrix_service;
  SetConingScullingCompServiceType m_set_coning_sculling_comp_service;
  GetConingScullingCompServiceType m_get_coning_sculling_comp_service;
  EmptyServiceType m_reset_filter_service;
  SetEstimationControlFlagsServiceType m_set_estimation_control_flags_service;
  GetEstimationControlFlagsServiceType m_get_estimation_control_flags_service;
  InitFilterEulerServiceType m_init_filter_euler_service;
  InitFilterHeadingServiceType m_init_filter_heading_service;
  SetHeadingSourceServiceType m_set_heading_source_service;
  GetHeadingSourceServiceType m_get_heading_source_service;
  TriggerServiceType m_commanded_vel_zupt_service;
  TriggerServiceType m_commanded_ang_rate_zupt_service;
  SetAccelNoiseServiceType m_set_accel_noise_service;
  GetAccelNoiseServiceType m_get_accel_noise_service;
  SetGyroNoiseServiceType m_set_gyro_noise_service;
  GetGyroNoiseServiceType m_get_gyro_noise_service;
  SetMagNoiseServiceType m_set_mag_noise_service;
  GetMagNoiseServiceType m_get_mag_noise_service;
  SetAccelBiasModelServiceType m_set_accel_bias_model_service;
  GetAccelBiasModelServiceType m_get_accel_bias_model_service;
  SetGyroBiasModelServiceType m_set_gyro_bias_model_service;
  GetGyroBiasModelServiceType m_get_gyro_bias_model_service;
  SetMagAdaptiveValsServiceType m_set_mag_adaptive_vals_service;
  GetMagAdaptiveValsServiceType m_get_mag_adaptive_vals_service;
  SetMagDipAdaptiveValsServiceType m_set_mag_dip_adaptive_vals_service;
  GetMagDipAdaptiveValsServiceType m_get_mag_dip_adaptive_vals_service;
  SetGravityAdaptiveValsServiceType m_set_gravity_adaptive_vals_service;
  GetGravityAdaptiveValsServiceType m_get_gravity_adaptive_vals_service;
  SetZeroAngleUpdateThresholdServiceType m_set_zero_angle_update_threshold_service;
  GetZeroAngleUpdateThresholdServiceType m_get_zero_angle_update_threshold_service;
  SetZeroVelocityUpdateThresholdServiceType m_set_zero_velocity_update_threshold_service;
  GetZeroVelocityUpdateThresholdServiceType m_get_zero_velocity_update_threshold_service;
  SetReferencePositionServiceType m_set_reference_position_service;
  GetReferencePositionServiceType m_get_reference_position_service;
  SetDynamicsModeServiceType m_set_dynamics_mode_service;
  GetDynamicsModeServiceType m_get_dynamics_mode_service;
  DeviceSettingsServiceType m_device_settings_service;
  ExternalHeadingUpdateServiceType m_external_heading_service;
  SetRelativePositionReferenceServiceType m_set_relative_position_reference_service;
  GetRelativePositionReferenceServiceType m_get_relative_position_reference_service;
};  // struct MicrostrainServices

} // namespace Microstrain

#endif  // _MICROSTRAIN_SERVICES_H
