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
#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_MICROSTRAIN_SERVICES_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_MICROSTRAIN_SERVICES_H

#include <memory>
#include "microstrain_inertial_driver_common/microstrain_defs.h"
#include "microstrain_inertial_driver_common/microstrain_ros_funcs.h"
#include "microstrain_inertial_driver_common/microstrain_config.h"

namespace microstrain
{

/**
 * Contains service functions and service handles
 */
class MicrostrainServices
{
public:
  /**
   * Default Constructor
   */
  MicrostrainServices() = default;

  /**
   * \brief Constructs this class with a reference to the node, and a config object
   * \param node  Reference to a node that will be saved to this class and used to log and interact with ROS
   * \param config Reference to the config object that will be saved to this class and used to determine whether or not to enable the services
   */
  MicrostrainServices(RosNodeType* node, MicrostrainConfig* config);

  /**
   * \brief Configures the services. After this function is called, the services will be created, but (ROS2 only) will not be activated
   * \return true if configuration was successful and false if configuration failed
   */
  bool configure();

  // TODO(robbiefish): Document all of these service functions
  bool deviceReport(DeviceReportServiceMsg::Request& req, DeviceReportServiceMsg::Response& res);
  bool getBasicStatus(TriggerServiceMsg::Request& req, TriggerServiceMsg::Response& res);
  bool getDiagnosticReport(TriggerServiceMsg::Request& req, TriggerServiceMsg::Response& res);

  bool setAccelBias(SetAccelBiasServiceMsg::Request& req, SetAccelBiasServiceMsg::Response& res);
  bool getAccelBias(GetAccelBiasServiceMsg::Request& req, GetAccelBiasServiceMsg::Response& res);

  bool setGyroBias(SetGyroBiasServiceMsg::Request& req, SetGyroBiasServiceMsg::Response& res);
  bool getGyroBias(GetGyroBiasServiceMsg::Request& req, GetGyroBiasServiceMsg::Response& res);

  bool gyroBiasCapture(TriggerServiceMsg::Request& req, TriggerServiceMsg::Response& res);

  bool setHardIronValues(SetHardIronValuesServiceMsg::Request& req, SetHardIronValuesServiceMsg::Response& res);
  bool getHardIronValues(GetHardIronValuesServiceMsg::Request& req, GetHardIronValuesServiceMsg::Response& res);

  bool setSoftIronMatrix(SetSoftIronMatrixServiceMsg::Request& req, SetSoftIronMatrixServiceMsg::Response& res);
  bool getSoftIronMatrix(GetSoftIronMatrixServiceMsg::Request& req, GetSoftIronMatrixServiceMsg::Response& res);

  bool setComplementaryFilter(SetComplementaryFilterServiceMsg::Request& req,
                                SetComplementaryFilterServiceMsg::Response& res);
  bool getComplementaryFilter(GetComplementaryFilterServiceMsg::Request& req,
                                GetComplementaryFilterServiceMsg::Response& res);

  bool setConingScullingComp(SetConingScullingCompServiceMsg::Request& req,
                                SetConingScullingCompServiceMsg::Response& res);
  bool getConingScullingComp(GetConingScullingCompServiceMsg::Request& req,
                                GetConingScullingCompServiceMsg::Response& res);

  bool setSensor2vehicleRotation(SetSensor2VehicleRotationServiceMsg::Request& req,
                                   SetSensor2VehicleRotationServiceMsg::Response& res);
  bool getSensor2vehicleRotation(GetSensor2VehicleRotationServiceMsg::Request& req,
                                   GetSensor2VehicleRotationServiceMsg::Response& res);

  bool setSensor2vehicleOffset(SetSensor2VehicleOffsetServiceMsg::Request& req,
                                 SetSensor2VehicleOffsetServiceMsg::Response& res);
  bool getSensor2vehicleOffset(GetSensor2VehicleOffsetServiceMsg::Request& req,
                                 GetSensor2VehicleOffsetServiceMsg::Response& res);

  bool getSensor2vehicleTransformation(GetSensor2VehicleTransformationServiceMsg::Request& req,
                                         GetSensor2VehicleTransformationServiceMsg::Response& res);

  bool resetFilter(EmptyServiceMsg::Request& req, EmptyServiceMsg::Response& resp);

  bool initFilterEuler(InitFilterEulerServiceMsg::Request& req, InitFilterEulerServiceMsg::Response& res);
  bool initFilterHeading(InitFilterHeadingServiceMsg::Request& req, InitFilterHeadingServiceMsg::Response& res);

  bool setHeadingSource(SetHeadingSourceServiceMsg::Request& req, SetHeadingSourceServiceMsg::Response& res);
  bool getHeadingSource(GetHeadingSourceServiceMsg::Request& req, GetHeadingSourceServiceMsg::Response& res);

  bool setReferencePosition(SetReferencePositionServiceMsg::Request& req,
                              SetReferencePositionServiceMsg::Response& res);
  bool getReferencePosition(GetReferencePositionServiceMsg::Request& req,
                              GetReferencePositionServiceMsg::Response& res);

  bool setEstimationControlFlags(SetEstimationControlFlagsServiceMsg::Request& req,
                                    SetEstimationControlFlagsServiceMsg::Response& res);
  bool getEstimationControlFlags(GetEstimationControlFlagsServiceMsg::Request& req,
                                    GetEstimationControlFlagsServiceMsg::Response& res);

  bool setDynamicsMode(SetDynamicsModeServiceMsg::Request& req, SetDynamicsModeServiceMsg::Response& res);
  bool getDynamicsMode(GetDynamicsModeServiceMsg::Request& req, GetDynamicsModeServiceMsg::Response& res);

  bool setZeroAngleUpdateThreshold(SetZeroAngleUpdateThresholdServiceMsg::Request& req,
                                       SetZeroAngleUpdateThresholdServiceMsg::Response& res);
  bool getZeroAngleUpdateThreshold(GetZeroAngleUpdateThresholdServiceMsg::Request& req,
                                       GetZeroAngleUpdateThresholdServiceMsg::Response& res);

  bool setZeroVelocityUpdateThreshold(SetZeroVelocityUpdateThresholdServiceMsg::Request& req,
                                          SetZeroVelocityUpdateThresholdServiceMsg::Response& res);
  bool getZeroVelocityUpdateThreshold(GetZeroVelocityUpdateThresholdServiceMsg::Request& req,
                                          GetZeroVelocityUpdateThresholdServiceMsg::Response& res);

  bool setTareOrientation(SetTareOrientationServiceMsg::Request& req, SetTareOrientationServiceMsg::Response& res);

  bool commandedVelZupt(TriggerServiceMsg::Request& req, TriggerServiceMsg::Response& res);
  bool commandedAngRateZupt(TriggerServiceMsg::Request& req, TriggerServiceMsg::Response& res);

  bool setAccelNoise(SetAccelNoiseServiceMsg::Request& req, SetAccelNoiseServiceMsg::Response& res);
  bool getAccelNoise(GetAccelNoiseServiceMsg::Request& req, GetAccelNoiseServiceMsg::Response& res);

  bool setGyroNoise(SetGyroNoiseServiceMsg::Request& req, SetGyroNoiseServiceMsg::Response& res);
  bool getGyroNoise(GetGyroNoiseServiceMsg::Request& req, GetGyroNoiseServiceMsg::Response& res);

  bool setMagNoise(SetMagNoiseServiceMsg::Request& req, SetMagNoiseServiceMsg::Response& res);
  bool getMagNoise(GetMagNoiseServiceMsg::Request& req, GetMagNoiseServiceMsg::Response& res);

  bool setGyroBiasModel(SetGyroBiasModelServiceMsg::Request& req, SetGyroBiasModelServiceMsg::Response& res);
  bool getGyroBiasModel(GetGyroBiasModelServiceMsg::Request& req, GetGyroBiasModelServiceMsg::Response& res);

  bool setAccelBiasModel(SetAccelBiasModelServiceMsg::Request& req, SetAccelBiasModelServiceMsg::Response& res);
  bool getAccelBiasModel(GetAccelBiasModelServiceMsg::Request& req, GetAccelBiasModelServiceMsg::Response& res);

  bool setGravityAdaptiveVals(SetGravityAdaptiveValsServiceMsg::Request& req,
                                 SetGravityAdaptiveValsServiceMsg::Response& res);
  bool getGravityAdaptiveVals(GetGravityAdaptiveValsServiceMsg::Request& req,
                                 GetGravityAdaptiveValsServiceMsg::Response& res);

  bool setMagAdaptiveVals(SetMagAdaptiveValsServiceMsg::Request& req, SetMagAdaptiveValsServiceMsg::Response& res);
  bool getMagAdaptiveVals(GetMagAdaptiveValsServiceMsg::Request& req, GetMagAdaptiveValsServiceMsg::Response& res);

  bool setMagDipAdaptiveVals(SetMagDipAdaptiveValsServiceMsg::Request& req,
                                 SetMagDipAdaptiveValsServiceMsg::Response& res);
  bool getMagDipAdaptiveVals(GetMagDipAdaptiveValsServiceMsg::Request& req,
                                 GetMagDipAdaptiveValsServiceMsg::Response& res);

  bool externalHeadingUpdate(ExternalHeadingUpdateServiceMsg::Request& req,
                               ExternalHeadingUpdateServiceMsg::Response& res);

  bool setRelativePositionReference(SetRelativePositionReferenceServiceMsg::Request& req,
                                       SetRelativePositionReferenceServiceMsg::Response& res);
  bool getRelativePositionReference(GetRelativePositionReferenceServiceMsg::Request& req,
                                       GetRelativePositionReferenceServiceMsg::Response& res);

  bool deviceSettings(DeviceSettingsServiceMsg::Request& req, DeviceSettingsServiceMsg::Response& res);

  bool setFilterSpeedLeverArm(SetFilterSpeedLeverArmServiceMsg::Request& req,
                                SetFilterSpeedLeverArmServiceMsg::Response& res);

private:
  RosNodeType* node_;
  MicrostrainConfig* config_;

  TriggerServiceType get_basic_status_service_;
  TriggerServiceType get_diagnostic_report_service_;
  DeviceReportServiceType device_report_service_;
  SetTareOrientationServiceType set_tare_orientation_service_;
  SetComplementaryFilterServiceType set_complementary_filter_service_;
  GetComplementaryFilterServiceType get_complementary_filter_service_;
  SetSensor2VehicleRotationServiceType set_sensor2vehicle_rotation_service_;
  GetSensor2VehicleRotationServiceType get_sensor2vehicle_rotation_service_;
  SetSensor2VehicleOffsetServiceType set_sensor2vehicle_offset_service_;
  GetSensor2VehicleOffsetServiceType get_sensor2vehicle_offset_service_;
  GetSensor2VehicleTransformationServiceType get_sensor2vehicle_transformation_service_;
  SetAccelBiasServiceType set_accel_bias_service_;
  GetAccelBiasServiceType get_accel_bias_service_;
  SetGyroBiasServiceType set_gyro_bias_service_;
  GetGyroBiasServiceType get_gyro_bias_service_;
  TriggerServiceType gyro_bias_capture_service_;
  SetHardIronValuesServiceType set_hard_iron_values_service_;
  GetHardIronValuesServiceType get_hard_iron_values_service_;
  SetSoftIronMatrixServiceType set_soft_iron_matrix_service_;
  GetSoftIronMatrixServiceType get_soft_iron_matrix_service_;
  SetConingScullingCompServiceType set_coning_sculling_comp_service_;
  GetConingScullingCompServiceType get_coning_sculling_comp_service_;
  EmptyServiceType reset_filter_service_;
  SetEstimationControlFlagsServiceType set_estimation_control_flags_service_;
  GetEstimationControlFlagsServiceType get_estimation_control_flags_service_;
  InitFilterEulerServiceType init_filter_euler_service_;
  InitFilterHeadingServiceType init_filter_heading_service_;
  SetHeadingSourceServiceType set_heading_source_service_;
  GetHeadingSourceServiceType get_heading_source_service_;
  TriggerServiceType commanded_vel_zupt_service_;
  TriggerServiceType commanded_ang_rate_zupt_service_;
  SetAccelNoiseServiceType set_accel_noise_service_;
  GetAccelNoiseServiceType get_accel_noise_service_;
  SetGyroNoiseServiceType set_gyro_noise_service_;
  GetGyroNoiseServiceType get_gyro_noise_service_;
  SetMagNoiseServiceType set_mag_noise_service_;
  GetMagNoiseServiceType get_mag_noise_service_;
  SetAccelBiasModelServiceType set_accel_bias_model_service_;
  GetAccelBiasModelServiceType get_accel_bias_model_service_;
  SetGyroBiasModelServiceType set_gyro_bias_model_service_;
  GetGyroBiasModelServiceType get_gyro_bias_model_service_;
  SetMagAdaptiveValsServiceType set_mag_adaptive_vals_service_;
  GetMagAdaptiveValsServiceType get_mag_adaptive_vals_service_;
  SetMagDipAdaptiveValsServiceType set_mag_dip_adaptive_vals_service_;
  GetMagDipAdaptiveValsServiceType get_mag_dip_adaptive_vals_service_;
  SetGravityAdaptiveValsServiceType set_gravity_adaptive_vals_service_;
  GetGravityAdaptiveValsServiceType get_gravity_adaptive_vals_service_;
  SetZeroAngleUpdateThresholdServiceType set_zero_angle_update_threshold_service_;
  GetZeroAngleUpdateThresholdServiceType get_zero_angle_update_threshold_service_;
  SetZeroVelocityUpdateThresholdServiceType set_zero_velocity_update_threshold_service_;
  GetZeroVelocityUpdateThresholdServiceType get_zero_velocity_update_threshold_service_;
  SetReferencePositionServiceType set_reference_position_service_;
  GetReferencePositionServiceType get_reference_position_service_;
  SetDynamicsModeServiceType set_dynamics_mode_service_;
  GetDynamicsModeServiceType get_dynamics_mode_service_;
  DeviceSettingsServiceType device_settings_service_;
  ExternalHeadingUpdateServiceType external_heading_service_;
  SetRelativePositionReferenceServiceType set_relative_position_reference_service_;
  GetRelativePositionReferenceServiceType get_relative_position_reference_service_;
  SetFilterSpeedLeverArmServiceType set_filter_speed_lever_arm_service_;
};  // struct MicrostrainServices

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_MICROSTRAIN_SERVICES_H
