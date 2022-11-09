/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Driver Definition File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2020, Parker Hannifin Corp
//
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_SERVICES_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_SERVICES_H

#include <memory>
#include <string>

#include "microstrain_inertial_driver_common/utils/ros_compat.h"
#include "microstrain_inertial_driver_common/config.h"

namespace microstrain
{

/**
 * Contains service functions and service handles
 */
class Services
{
public:
  /**
   * Default Constructor
   */
  Services() = default;

  /**
   * \brief Constructs this class with a reference to the node, and a config object
   * \param node  Reference to a node that will be saved to this class and used to log and interact with ROS
   * \param config Reference to the config object that will be saved to this class and used to determine whether or not to enable the services
   */
  Services(RosNodeType* node, Config* config);

  /**
   * \brief Configures the services. After this function is called, the services will be created, but (ROS2 only) will not be activated
   * \return true if configuration was successful and false if configuration failed
   */
  bool configure();

  // Service functions. Too many to document
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
  /**
   * \brief Configures a non MIP command dependent service. This service will always be configured if this functions is called.
   * \tparam ServiceType. The ROS service type that the service will use
   * \param name The name to give the service
   * \param callback The callback to trigger when the service is called
   * \return Pointer to an initialized service
   */
  template<typename ServiceType>
  typename RosServiceType<ServiceType>::SharedPtr configureService(const std::string& name, bool (Services::*callback)(typename ServiceType::Request&, typename ServiceType::Response&));

  /**
   * \brief Configures a MIP command dependent service. This service will only be configured if the device supports the command
   * \tparam ServiceType The ROS service type that the service will use
   * \tparam MipType The MIP type that the device must support
   * \tparam DescriptorSet The descriptor set to use with the MipType if the MipType's descriptor set should not be used
   * \param name The name to give the service
   * \param callback The callback to trigger when the service is called
   * \return Pointer to an initialized service, or nullptr if the device does not support the MipType
   */
  template<typename ServiceType, typename MipType, uint8_t DescriptorSet = MipType::DESCRIPTOR_SET>
  typename RosServiceType<ServiceType>::SharedPtr configureService(const std::string& name, bool (Services::*callback)(typename ServiceType::Request&, typename ServiceType::Response&));

  // Handles to the ROS node and the config
  RosNodeType* node_;
  Config* config_;

  // Pointers to the services that this class will hold onto
  RosServiceType<TriggerServiceMsg>::SharedPtr get_basic_status_service_;
  RosServiceType<TriggerServiceMsg>::SharedPtr get_diagnostic_report_service_;
  RosServiceType<DeviceReportServiceMsg>::SharedPtr device_report_service_;
  RosServiceType<SetTareOrientationServiceMsg>::SharedPtr set_tare_orientation_service_;
  RosServiceType<SetComplementaryFilterServiceMsg>::SharedPtr set_complementary_filter_service_;
  RosServiceType<GetComplementaryFilterServiceMsg>::SharedPtr get_complementary_filter_service_;
  RosServiceType<SetSensor2VehicleRotationServiceMsg>::SharedPtr set_sensor2vehicle_rotation_service_;
  RosServiceType<GetSensor2VehicleRotationServiceMsg>::SharedPtr get_sensor2vehicle_rotation_service_;
  RosServiceType<SetSensor2VehicleOffsetServiceMsg>::SharedPtr set_sensor2vehicle_offset_service_;
  RosServiceType<GetSensor2VehicleOffsetServiceMsg>::SharedPtr get_sensor2vehicle_offset_service_;
  RosServiceType<GetSensor2VehicleTransformationServiceMsg>::SharedPtr get_sensor2vehicle_transformation_service_;
  RosServiceType<SetAccelBiasServiceMsg>::SharedPtr set_accel_bias_service_;
  RosServiceType<GetAccelBiasServiceMsg>::SharedPtr get_accel_bias_service_;
  RosServiceType<SetGyroBiasServiceMsg>::SharedPtr set_gyro_bias_service_;
  RosServiceType<GetGyroBiasServiceMsg>::SharedPtr get_gyro_bias_service_;
  RosServiceType<TriggerServiceMsg>::SharedPtr gyro_bias_capture_service_;
  RosServiceType<SetHardIronValuesServiceMsg>::SharedPtr set_hard_iron_values_service_;
  RosServiceType<GetHardIronValuesServiceMsg>::SharedPtr get_hard_iron_values_service_;
  RosServiceType<SetSoftIronMatrixServiceMsg>::SharedPtr set_soft_iron_matrix_service_;
  RosServiceType<GetSoftIronMatrixServiceMsg>::SharedPtr get_soft_iron_matrix_service_;
  RosServiceType<SetConingScullingCompServiceMsg>::SharedPtr set_coning_sculling_comp_service_;
  RosServiceType<GetConingScullingCompServiceMsg>::SharedPtr get_coning_sculling_comp_service_;
  RosServiceType<EmptyServiceMsg>::SharedPtr reset_filter_service_;
  RosServiceType<SetEstimationControlFlagsServiceMsg>::SharedPtr set_estimation_control_flags_service_;
  RosServiceType<GetEstimationControlFlagsServiceMsg>::SharedPtr get_estimation_control_flags_service_;
  RosServiceType<InitFilterEulerServiceMsg>::SharedPtr init_filter_euler_service_;
  RosServiceType<InitFilterHeadingServiceMsg>::SharedPtr init_filter_heading_service_;
  RosServiceType<SetHeadingSourceServiceMsg>::SharedPtr set_heading_source_service_;
  RosServiceType<GetHeadingSourceServiceMsg>::SharedPtr get_heading_source_service_;
  RosServiceType<TriggerServiceMsg>::SharedPtr commanded_vel_zupt_service_;
  RosServiceType<TriggerServiceMsg>::SharedPtr commanded_ang_rate_zupt_service_;
  RosServiceType<SetAccelNoiseServiceMsg>::SharedPtr set_accel_noise_service_;
  RosServiceType<GetAccelNoiseServiceMsg>::SharedPtr get_accel_noise_service_;
  RosServiceType<SetGyroNoiseServiceMsg>::SharedPtr set_gyro_noise_service_;
  RosServiceType<GetGyroNoiseServiceMsg>::SharedPtr get_gyro_noise_service_;
  RosServiceType<SetMagNoiseServiceMsg>::SharedPtr set_mag_noise_service_;
  RosServiceType<GetMagNoiseServiceMsg>::SharedPtr get_mag_noise_service_;
  RosServiceType<SetAccelBiasModelServiceMsg>::SharedPtr set_accel_bias_model_service_;
  RosServiceType<GetAccelBiasModelServiceMsg>::SharedPtr get_accel_bias_model_service_;
  RosServiceType<SetGyroBiasModelServiceMsg>::SharedPtr set_gyro_bias_model_service_;
  RosServiceType<GetGyroBiasModelServiceMsg>::SharedPtr get_gyro_bias_model_service_;
  RosServiceType<SetMagAdaptiveValsServiceMsg>::SharedPtr set_mag_adaptive_vals_service_;
  RosServiceType<GetMagAdaptiveValsServiceMsg>::SharedPtr get_mag_adaptive_vals_service_;
  RosServiceType<SetMagDipAdaptiveValsServiceMsg>::SharedPtr set_mag_dip_adaptive_vals_service_;
  RosServiceType<GetMagDipAdaptiveValsServiceMsg>::SharedPtr get_mag_dip_adaptive_vals_service_;
  RosServiceType<SetGravityAdaptiveValsServiceMsg>::SharedPtr set_gravity_adaptive_vals_service_;
  RosServiceType<GetGravityAdaptiveValsServiceMsg>::SharedPtr get_gravity_adaptive_vals_service_;
  RosServiceType<SetZeroAngleUpdateThresholdServiceMsg>::SharedPtr set_zero_angle_update_threshold_service_;
  RosServiceType<GetZeroAngleUpdateThresholdServiceMsg>::SharedPtr get_zero_angle_update_threshold_service_;
  RosServiceType<SetZeroVelocityUpdateThresholdServiceMsg>::SharedPtr set_zero_velocity_update_threshold_service_;
  RosServiceType<GetZeroVelocityUpdateThresholdServiceMsg>::SharedPtr get_zero_velocity_update_threshold_service_;
  RosServiceType<SetReferencePositionServiceMsg>::SharedPtr set_reference_position_service_;
  RosServiceType<GetReferencePositionServiceMsg>::SharedPtr get_reference_position_service_;
  RosServiceType<SetDynamicsModeServiceMsg>::SharedPtr set_dynamics_mode_service_;
  RosServiceType<GetDynamicsModeServiceMsg>::SharedPtr get_dynamics_mode_service_;
  RosServiceType<DeviceSettingsServiceMsg>::SharedPtr device_settings_service_;
  RosServiceType<ExternalHeadingUpdateServiceMsg>::SharedPtr external_heading_service_;
  RosServiceType<SetRelativePositionReferenceServiceMsg>::SharedPtr set_relative_position_reference_service_;
  RosServiceType<GetRelativePositionReferenceServiceMsg>::SharedPtr get_relative_position_reference_service_;
  RosServiceType<SetFilterSpeedLeverArmServiceMsg>::SharedPtr set_filter_speed_lever_arm_service_;
};

template<typename ServiceType>
typename RosServiceType<ServiceType>::SharedPtr Services::configureService(const std::string& name, bool (Services::*callback)(typename ServiceType::Request&, typename ServiceType::Response&))
{
  MICROSTRAIN_DEBUG(node_, "Configuring service %s", name.c_str());
  return createService<ServiceType>(node_, name, callback, this);
}

template<typename ServiceType, typename MipType, uint8_t DescriptorSet = MipType::DESCRIPTOR_SET>
typename RosServiceType<ServiceType>::SharedPtr Services::configureService(const std::string& name, bool (Services::*callback)(typename ServiceType::Request&, typename ServiceType::Response&))
{
  if (config_->mip_device_->supportsDescriptor(DescriptorSet, MipType::FIELD_DESCRIPTOR))
  {
    MICROSTRAIN_DEBUG(node_, "Configuring service %s to execute MIP command 0x%02x%02x", name.c_str(), DescriptorSet, MipType::FIELD_DESCRIPTOR);
    return createService<ServiceType>(node_, name, callback, this);
  }
  else
  {
    MICROSTRAIN_DEBUG(node_, "Device does not support the %s service because the device does not support descriptor 0x%02x%02x", name.c_str(), DescriptorSet, MipType::FIELD_DESCRIPTOR);
    return nullptr;
  }
}

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_SERVICES_H
