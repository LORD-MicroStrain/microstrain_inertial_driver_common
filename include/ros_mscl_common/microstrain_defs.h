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
#ifndef ROS_MSCL_COMMON_MICROSTRAIN_DEFS_H
#define ROS_MSCL_COMMON_MICROSTRAIN_DEFS_H

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Common Includes
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#include <memory>

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// ROS1 Includes
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#if MICROSTRAIN_ROS_VERSION == 1
#include "ros/ros.h"

#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/TimeReference.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/MagneticField.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "mscl_msgs/Status.h"
#include "mscl_msgs/RTKStatus.h"
#include "mscl_msgs/FilterStatus.h"
#include "mscl_msgs/FilterHeading.h"
#include "mscl_msgs/FilterHeadingState.h"
#include "mscl_msgs/GPSCorrelationTimestampStamped.h"
#include "mscl_msgs/GNSSAidingStatus.h"
#include "mscl_msgs/GNSSDualAntennaStatus.h"

#include "std_srvs/Empty.h"
#include "std_srvs/Trigger.h"

#include "mscl_msgs/SetAccelBias.h"
#include "mscl_msgs/GetAccelBias.h"
#include "mscl_msgs/SetGyroBias.h"
#include "mscl_msgs/GetGyroBias.h"
#include "mscl_msgs/SetHardIronValues.h"
#include "mscl_msgs/GetHardIronValues.h"
#include "mscl_msgs/SetSoftIronMatrix.h"
#include "mscl_msgs/GetSoftIronMatrix.h"
#include "mscl_msgs/SetComplementaryFilter.h"
#include "mscl_msgs/GetComplementaryFilter.h"
#include "mscl_msgs/InitFilterEuler.h"
#include "mscl_msgs/InitFilterHeading.h"
#include "mscl_msgs/DeviceSettings.h"
#include "mscl_msgs/SetAccelBiasModel.h"
#include "mscl_msgs/GetAccelBiasModel.h"
#include "mscl_msgs/SetGravityAdaptiveVals.h"
#include "mscl_msgs/GetGravityAdaptiveVals.h"
#include "mscl_msgs/SetSensor2VehicleRotation.h"
#include "mscl_msgs/GetSensor2VehicleRotation.h"
#include "mscl_msgs/SetSensor2VehicleOffset.h"
#include "mscl_msgs/GetSensor2VehicleOffset.h"
#include "mscl_msgs/SetReferencePosition.h"
#include "mscl_msgs/GetReferencePosition.h"
#include "mscl_msgs/SetConingScullingComp.h"
#include "mscl_msgs/GetConingScullingComp.h"
#include "mscl_msgs/SetEstimationControlFlags.h"
#include "mscl_msgs/GetEstimationControlFlags.h"
#include "mscl_msgs/SetDynamicsMode.h"
#include "mscl_msgs/GetDynamicsMode.h"
#include "mscl_msgs/SetZeroAngleUpdateThreshold.h"
#include "mscl_msgs/GetZeroAngleUpdateThreshold.h"
#include "mscl_msgs/SetZeroVelocityUpdateThreshold.h"
#include "mscl_msgs/GetZeroVelocityUpdateThreshold.h"
#include "mscl_msgs/SetTareOrientation.h"
#include "mscl_msgs/SetAccelNoise.h"
#include "mscl_msgs/GetAccelNoise.h"
#include "mscl_msgs/SetGyroNoise.h"
#include "mscl_msgs/GetGyroNoise.h"
#include "mscl_msgs/SetMagNoise.h"
#include "mscl_msgs/GetMagNoise.h"
#include "mscl_msgs/SetGyroBiasModel.h"
#include "mscl_msgs/GetGyroBiasModel.h"
#include "mscl_msgs/SetMagAdaptiveVals.h"
#include "mscl_msgs/GetMagAdaptiveVals.h"
#include "mscl_msgs/SetMagDipAdaptiveVals.h"
#include "mscl_msgs/GetMagDipAdaptiveVals.h"
#include "mscl_msgs/SetHeadingSource.h"
#include "mscl_msgs/GetHeadingSource.h"
#include "mscl_msgs/GetSensor2VehicleTransformation.h"
#include "mscl_msgs/ExternalHeadingUpdate.h"
#include "mscl_msgs/SetRelativePositionReference.h"
#include "mscl_msgs/GetRelativePositionReference.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// ROS2 Includes
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#elif MICROSTRAIN_ROS_VERSION == 2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "mscl_msgs/msg/status.hpp"
#include "mscl_msgs/msg/rtk_status.hpp"
#include "mscl_msgs/msg/filter_status.hpp"
#include "mscl_msgs/msg/filter_heading.hpp"
#include "mscl_msgs/msg/filter_heading_state.hpp"
#include "mscl_msgs/msg/gps_correlation_timestamp_stamped.hpp"
#include "mscl_msgs/msg/gnss_aiding_status.hpp"
#include "mscl_msgs/msg/gnss_dual_antenna_status.hpp"

#include "mscl_msgs/srv/set_accel_bias.hpp"
#include "mscl_msgs/srv/get_accel_bias.hpp"
#include "mscl_msgs/srv/set_gyro_bias.hpp"
#include "mscl_msgs/srv/get_gyro_bias.hpp"
#include "mscl_msgs/srv/set_hard_iron_values.hpp"
#include "mscl_msgs/srv/get_hard_iron_values.hpp"
#include "mscl_msgs/srv/set_soft_iron_matrix.hpp"
#include "mscl_msgs/srv/get_soft_iron_matrix.hpp"
#include "mscl_msgs/srv/set_complementary_filter.hpp"
#include "mscl_msgs/srv/get_complementary_filter.hpp"
#include "mscl_msgs/srv/init_filter_euler.hpp"
#include "mscl_msgs/srv/init_filter_heading.hpp"
#include "mscl_msgs/srv/device_settings.hpp"
#include "mscl_msgs/srv/set_accel_bias_model.hpp"
#include "mscl_msgs/srv/get_accel_bias_model.hpp"
#include "mscl_msgs/srv/set_gravity_adaptive_vals.hpp"
#include "mscl_msgs/srv/get_gravity_adaptive_vals.hpp"
#include "mscl_msgs/srv/set_sensor2_vehicle_rotation.hpp"
#include "mscl_msgs/srv/get_sensor2_vehicle_rotation.hpp"
#include "mscl_msgs/srv/set_sensor2_vehicle_offset.hpp"
#include "mscl_msgs/srv/get_sensor2_vehicle_offset.hpp"
#include "mscl_msgs/srv/set_reference_position.hpp"
#include "mscl_msgs/srv/get_reference_position.hpp"
#include "mscl_msgs/srv/set_coning_sculling_comp.hpp"
#include "mscl_msgs/srv/get_coning_sculling_comp.hpp"
#include "mscl_msgs/srv/set_estimation_control_flags.hpp"
#include "mscl_msgs/srv/get_estimation_control_flags.hpp"
#include "mscl_msgs/srv/set_dynamics_mode.hpp"
#include "mscl_msgs/srv/get_dynamics_mode.hpp"
#include "mscl_msgs/srv/set_zero_angle_update_threshold.hpp"
#include "mscl_msgs/srv/get_zero_angle_update_threshold.hpp"
#include "mscl_msgs/srv/set_zero_velocity_update_threshold.hpp"
#include "mscl_msgs/srv/get_zero_velocity_update_threshold.hpp"
#include "mscl_msgs/srv/set_tare_orientation.hpp"
#include "mscl_msgs/srv/set_accel_noise.hpp"
#include "mscl_msgs/srv/get_accel_noise.hpp"
#include "mscl_msgs/srv/set_gyro_noise.hpp"
#include "mscl_msgs/srv/get_gyro_noise.hpp"
#include "mscl_msgs/srv/set_mag_noise.hpp"
#include "mscl_msgs/srv/get_mag_noise.hpp"
#include "mscl_msgs/srv/set_gyro_bias_model.hpp"
#include "mscl_msgs/srv/get_gyro_bias_model.hpp"
#include "mscl_msgs/srv/set_mag_adaptive_vals.hpp"
#include "mscl_msgs/srv/get_mag_adaptive_vals.hpp"
#include "mscl_msgs/srv/set_mag_dip_adaptive_vals.hpp"
#include "mscl_msgs/srv/get_mag_dip_adaptive_vals.hpp"
#include "mscl_msgs/srv/set_heading_source.hpp"
#include "mscl_msgs/srv/get_heading_source.hpp"
#include "mscl_msgs/srv/get_sensor2_vehicle_transformation.hpp"
#include "mscl_msgs/srv/external_heading_update.hpp"
#include "mscl_msgs/srv/set_relative_position_reference.hpp"
#include "mscl_msgs/srv/get_relative_position_reference.hpp"
#else
#error "Unsupported ROS version. -DMICROSTRAIN_ROS_VERSION must be set to 1 or 2"
#endif

namespace microstrain
{
/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// ROS1 Defines
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#if MICROSTRAIN_ROS_VERSION == 1
///
/// ROS1 Types
///
using RosNodeType = ::ros::NodeHandle;
using RosTimerType = std::shared_ptr<::ros::Timer>;
using RosHeaderType = ::std_msgs::Header;

// ROS1 Publisher Messgae Types
using OdometryMsg = ::nav_msgs::Odometry;
using ImuMsg = ::sensor_msgs::Imu;
using NavSatFixMsg = ::sensor_msgs::NavSatFix;
using MagneticFieldMsg = ::sensor_msgs::MagneticField;
using TimeReferenceMsg = ::sensor_msgs::TimeReference;
using StatusMsg = ::mscl_msgs::Status;
using RTKStatusMsg = ::mscl_msgs::RTKStatus;
using FilterStatusMsg = ::mscl_msgs::FilterStatus;
using FilterHeadingMsg = ::mscl_msgs::FilterHeading;
using GNSSAidingStatusMsg = ::mscl_msgs::GNSSAidingStatus;
using GNSSDualAntennaStatusMsg = ::mscl_msgs::GNSSDualAntennaStatus;
using FilterHeadingStateMsg = ::mscl_msgs::FilterHeadingState;
using GPSCorrelationTimestampStampedMsg = ::mscl_msgs::GPSCorrelationTimestampStamped;

// ROS1 Publisher Types
using OdometryPubType = std::shared_ptr<::ros::Publisher>;
using ImuPubType = std::shared_ptr<::ros::Publisher>;
using NavSatFixPubType = std::shared_ptr<::ros::Publisher>;
using MagneticFieldPubType = std::shared_ptr<::ros::Publisher>;
using TimeReferencePubType = std::shared_ptr<::ros::Publisher>;
using StatusPubType = std::shared_ptr<::ros::Publisher>;
using RTKStatusPubType = std::shared_ptr<::ros::Publisher>;
using FilterStatusPubType = std::shared_ptr<::ros::Publisher>;
using FilterHeadingPubType = std::shared_ptr<::ros::Publisher>;
using GNSSAidingStatusPubType = std::shared_ptr<::ros::Publisher>;
using GNSSDualAntennaStatusPubType = std::shared_ptr<::ros::Publisher>;
using FilterHeadingStatePubType = std::shared_ptr<::ros::Publisher>;
using GPSCorrelationTimestampStampedPubType = std::shared_ptr<::ros::Publisher>;

// ROS1 Subscriber Message Types
using BoolMsg = ::std_msgs::Bool;
using TimeReferenceMsg = ::sensor_msgs::TimeReference;

// ROS1 Subscriber Types
using BoolSubType = std::shared_ptr<::ros::Subscriber>;
using TimeReferenceSubType = std::shared_ptr<::ros::Subscriber>;

// ROS1 Service Message Types
using TriggerServiceMsg = std_srvs::Trigger;
using EmptyServiceMsg = std_srvs::Empty;

using SetAccelBiasServiceMsg = ::mscl_msgs::SetAccelBias;
using GetAccelBiasServiceMsg = ::mscl_msgs::GetAccelBias;

using SetGyroBiasServiceMsg = ::mscl_msgs::SetGyroBias;
using GetGyroBiasServiceMsg = ::mscl_msgs::GetGyroBias;

using SetHardIronValuesServiceMsg = ::mscl_msgs::SetHardIronValues;
using GetHardIronValuesServiceMsg = ::mscl_msgs::GetHardIronValues;

using SetSoftIronMatrixServiceMsg = ::mscl_msgs::SetSoftIronMatrix;
using GetSoftIronMatrixServiceMsg = ::mscl_msgs::GetSoftIronMatrix;

using SetComplementaryFilterServiceMsg = ::mscl_msgs::SetComplementaryFilter;
using GetComplementaryFilterServiceMsg = ::mscl_msgs::GetComplementaryFilter;

using SetConingScullingCompServiceMsg = ::mscl_msgs::SetConingScullingComp;
using GetConingScullingCompServiceMsg = ::mscl_msgs::GetConingScullingComp;

using SetSensor2VehicleRotationServiceMsg = ::mscl_msgs::SetSensor2VehicleRotation;
using GetSensor2VehicleRotationServiceMsg = ::mscl_msgs::GetSensor2VehicleRotation;

using SetSensor2VehicleOffsetServiceMsg = ::mscl_msgs::SetSensor2VehicleOffset;
using GetSensor2VehicleOffsetServiceMsg = ::mscl_msgs::GetSensor2VehicleOffset;

using GetSensor2VehicleTransformationServiceMsg = ::mscl_msgs::GetSensor2VehicleTransformation;

using InitFilterEulerServiceMsg = ::mscl_msgs::InitFilterEuler;
using InitFilterHeadingServiceMsg = ::mscl_msgs::InitFilterHeading;

using SetHeadingSourceServiceMsg = ::mscl_msgs::SetHeadingSource;
using GetHeadingSourceServiceMsg = ::mscl_msgs::GetHeadingSource;

using SetReferencePositionServiceMsg = ::mscl_msgs::SetReferencePosition;
using GetReferencePositionServiceMsg = ::mscl_msgs::GetReferencePosition;

using SetEstimationControlFlagsServiceMsg = ::mscl_msgs::SetEstimationControlFlags;
using GetEstimationControlFlagsServiceMsg = ::mscl_msgs::GetEstimationControlFlags;

using SetDynamicsModeServiceMsg = ::mscl_msgs::SetDynamicsMode;
using GetDynamicsModeServiceMsg = ::mscl_msgs::GetDynamicsMode;

using SetZeroAngleUpdateThresholdServiceMsg = ::mscl_msgs::SetZeroAngleUpdateThreshold;
using GetZeroAngleUpdateThresholdServiceMsg = ::mscl_msgs::GetZeroAngleUpdateThreshold;

using SetZeroVelocityUpdateThresholdServiceMsg = ::mscl_msgs::SetZeroVelocityUpdateThreshold;
using GetZeroVelocityUpdateThresholdServiceMsg = ::mscl_msgs::GetZeroVelocityUpdateThreshold;

using SetTareOrientationServiceMsg = ::mscl_msgs::SetTareOrientation;

using SetAccelNoiseServiceMsg = ::mscl_msgs::SetAccelNoise;
using GetAccelNoiseServiceMsg = ::mscl_msgs::GetAccelNoise;

using SetGyroNoiseServiceMsg = ::mscl_msgs::SetGyroNoise;
using GetGyroNoiseServiceMsg = ::mscl_msgs::GetGyroNoise;

using SetMagNoiseServiceMsg = ::mscl_msgs::SetMagNoise;
using GetMagNoiseServiceMsg = ::mscl_msgs::GetMagNoise;

using SetGyroBiasModelServiceMsg = ::mscl_msgs::SetGyroBiasModel;
using GetGyroBiasModelServiceMsg = ::mscl_msgs::GetGyroBiasModel;

using SetAccelBiasModelServiceMsg = ::mscl_msgs::SetAccelBiasModel;
using GetAccelBiasModelServiceMsg = ::mscl_msgs::GetAccelBiasModel;

using SetGravityAdaptiveValsServiceMsg = ::mscl_msgs::SetGravityAdaptiveVals;
using GetGravityAdaptiveValsServiceMsg = ::mscl_msgs::GetGravityAdaptiveVals;

using SetMagAdaptiveValsServiceMsg = ::mscl_msgs::SetMagAdaptiveVals;
using GetMagAdaptiveValsServiceMsg = ::mscl_msgs::GetMagAdaptiveVals;

using SetMagDipAdaptiveValsServiceMsg = ::mscl_msgs::SetMagDipAdaptiveVals;
using GetMagDipAdaptiveValsServiceMsg = ::mscl_msgs::GetMagDipAdaptiveVals;

using ExternalHeadingUpdateServiceMsg = ::mscl_msgs::ExternalHeadingUpdate;

using SetRelativePositionReferenceServiceMsg = ::mscl_msgs::SetRelativePositionReference;
using GetRelativePositionReferenceServiceMsg = ::mscl_msgs::GetRelativePositionReference;

using DeviceSettingsServiceMsg = ::mscl_msgs::DeviceSettings;

// ROS1 Service Types
using TriggerServiceType = std::shared_ptr<::ros::ServiceServer>;
using EmptyServiceType = std::shared_ptr<::ros::ServiceServer>;

using SetAccelBiasServiceType = std::shared_ptr<::ros::ServiceServer>;
using GetAccelBiasServiceType = std::shared_ptr<::ros::ServiceServer>;

using SetGyroBiasServiceType = std::shared_ptr<::ros::ServiceServer>;
using GetGyroBiasServiceType = std::shared_ptr<::ros::ServiceServer>;

using SetHardIronValuesServiceType = std::shared_ptr<::ros::ServiceServer>;
using GetHardIronValuesServiceType = std::shared_ptr<::ros::ServiceServer>;

using SetSoftIronMatrixServiceType = std::shared_ptr<::ros::ServiceServer>;
using GetSoftIronMatrixServiceType = std::shared_ptr<::ros::ServiceServer>;

using SetComplementaryFilterServiceType = std::shared_ptr<::ros::ServiceServer>;
using GetComplementaryFilterServiceType = std::shared_ptr<::ros::ServiceServer>;

using SetConingScullingCompServiceType = std::shared_ptr<::ros::ServiceServer>;
using GetConingScullingCompServiceType = std::shared_ptr<::ros::ServiceServer>;

using SetSensor2VehicleRotationServiceType = std::shared_ptr<::ros::ServiceServer>;
using GetSensor2VehicleRotationServiceType = std::shared_ptr<::ros::ServiceServer>;

using SetSensor2VehicleOffsetServiceType = std::shared_ptr<::ros::ServiceServer>;
using GetSensor2VehicleOffsetServiceType = std::shared_ptr<::ros::ServiceServer>;

using GetSensor2VehicleTransformationServiceType = std::shared_ptr<::ros::ServiceServer>;

using InitFilterEulerServiceType = std::shared_ptr<::ros::ServiceServer>;
using InitFilterHeadingServiceType = std::shared_ptr<::ros::ServiceServer>;

using SetHeadingSourceServiceType = std::shared_ptr<::ros::ServiceServer>;
using GetHeadingSourceServiceType = std::shared_ptr<::ros::ServiceServer>;

using SetReferencePositionServiceType = std::shared_ptr<::ros::ServiceServer>;
using GetReferencePositionServiceType = std::shared_ptr<::ros::ServiceServer>;

using SetEstimationControlFlagsServiceType = std::shared_ptr<::ros::ServiceServer>;
using GetEstimationControlFlagsServiceType = std::shared_ptr<::ros::ServiceServer>;

using SetDynamicsModeServiceType = std::shared_ptr<::ros::ServiceServer>;
using GetDynamicsModeServiceType = std::shared_ptr<::ros::ServiceServer>;

using SetZeroAngleUpdateThresholdServiceType = std::shared_ptr<::ros::ServiceServer>;
using GetZeroAngleUpdateThresholdServiceType = std::shared_ptr<::ros::ServiceServer>;

using SetZeroVelocityUpdateThresholdServiceType = std::shared_ptr<::ros::ServiceServer>;
using GetZeroVelocityUpdateThresholdServiceType = std::shared_ptr<::ros::ServiceServer>;

using SetTareOrientationServiceType = std::shared_ptr<::ros::ServiceServer>;

using SetAccelNoiseServiceType = std::shared_ptr<::ros::ServiceServer>;
using GetAccelNoiseServiceType = std::shared_ptr<::ros::ServiceServer>;

using SetGyroNoiseServiceType = std::shared_ptr<::ros::ServiceServer>;
using GetGyroNoiseServiceType = std::shared_ptr<::ros::ServiceServer>;

using SetMagNoiseServiceType = std::shared_ptr<::ros::ServiceServer>;
using GetMagNoiseServiceType = std::shared_ptr<::ros::ServiceServer>;

using SetGyroBiasModelServiceType = std::shared_ptr<::ros::ServiceServer>;
using GetGyroBiasModelServiceType = std::shared_ptr<::ros::ServiceServer>;

using SetAccelBiasModelServiceType = std::shared_ptr<::ros::ServiceServer>;
using GetAccelBiasModelServiceType = std::shared_ptr<::ros::ServiceServer>;

using SetGravityAdaptiveValsServiceType = std::shared_ptr<::ros::ServiceServer>;
using GetGravityAdaptiveValsServiceType = std::shared_ptr<::ros::ServiceServer>;

using SetMagAdaptiveValsServiceType = std::shared_ptr<::ros::ServiceServer>;
using GetMagAdaptiveValsServiceType = std::shared_ptr<::ros::ServiceServer>;

using SetMagDipAdaptiveValsServiceType = std::shared_ptr<::ros::ServiceServer>;
using GetMagDipAdaptiveValsServiceType = std::shared_ptr<::ros::ServiceServer>;

using ExternalHeadingUpdateServiceType = std::shared_ptr<::ros::ServiceServer>;

using SetRelativePositionReferenceServiceType = std::shared_ptr<::ros::ServiceServer>;
using GetRelativePositionReferenceServiceType = std::shared_ptr<::ros::ServiceServer>;

using DeviceSettingsServiceType = std::shared_ptr<::ros::ServiceServer>;

///
/// ROS1 Logging
///
#define MICROSTRAIN_DEBUG(NODE, ...) ROS_DEBUG(__VA_ARGS__)
#define MICROSTRAIN_INFO(NODE, ...) ROS_INFO(__VA_ARGS__)
#define MICROSTRAIN_WARN(NODE, ...) ROS_WARN(__VA_ARGS__)
#define MICROSTRAIN_ERROR(NODE, ...) ROS_ERROR(__VA_ARGS__)
#define MICROSTRAIN_FATAL(NOE, ...) ROS_FATAL(__VA_ARGS__)

#define MICROSTRAIN_DEBUG_THROTTLE(NODE, PERIOD, ...) ROS_DEBUG_THROTTLE(PERIOD, __VA_ARGS__)

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// ROS2 Defines
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#elif MICROSTRAIN_ROS_VERSION == 2
///
/// ROS2 Types
///
using RosNodeType = ::rclcpp_lifecycle::LifecycleNode;
using RosTimerType = ::rclcpp::TimerBase::SharedPtr;
using RosHeaderType = ::std_msgs::msg::Header;

// ROS2 Publisher Message Types
using OdometryMsg = ::nav_msgs::msg::Odometry;
using ImuMsg = ::sensor_msgs::msg::Imu;
using NavSatFixMsg = ::sensor_msgs::msg::NavSatFix;
using MagneticFieldMsg = ::sensor_msgs::msg::MagneticField;
using TimeReferenceMsg = ::sensor_msgs::msg::TimeReference;
using StatusMsg = ::mscl_msgs::msg::Status;
using RTKStatusMsg = ::mscl_msgs::msg::RTKStatus;
using FilterStatusMsg = ::mscl_msgs::msg::FilterStatus;
using FilterHeadingMsg = ::mscl_msgs::msg::FilterHeading;
using GNSSAidingStatusMsg = ::mscl_msgs::msg::GNSSAidingStatus;
using GNSSDualAntennaStatusMsg = ::mscl_msgs::msg::GNSSDualAntennaStatus;
using FilterHeadingStateMsg = ::mscl_msgs::msg::FilterHeadingState;
using GPSCorrelationTimestampStampedMsg = ::mscl_msgs::msg::GPSCorrelationTimestampStamped;

// ROS2 Publisher Types
using OdometryPubType = ::rclcpp_lifecycle::LifecyclePublisher<OdometryMsg>::SharedPtr;
using ImuPubType = ::rclcpp_lifecycle::LifecyclePublisher<ImuMsg>::SharedPtr;
using NavSatFixPubType = ::rclcpp_lifecycle::LifecyclePublisher<NavSatFixMsg>::SharedPtr;
using MagneticFieldPubType = ::rclcpp_lifecycle::LifecyclePublisher<MagneticFieldMsg>::SharedPtr;
using TimeReferencePubType = ::rclcpp_lifecycle::LifecyclePublisher<TimeReferenceMsg>::SharedPtr;
using StatusPubType = ::rclcpp_lifecycle::LifecyclePublisher<StatusMsg>::SharedPtr;
using RTKStatusPubType = ::rclcpp_lifecycle::LifecyclePublisher<RTKStatusMsg>::SharedPtr;
using FilterStatusPubType = ::rclcpp_lifecycle::LifecyclePublisher<FilterStatusMsg>::SharedPtr;
using FilterHeadingPubType = ::rclcpp_lifecycle::LifecyclePublisher<FilterHeadingMsg>::SharedPtr;
using GNSSAidingStatusPubType = ::rclcpp_lifecycle::LifecyclePublisher<GNSSAidingStatusMsg>::SharedPtr;
using GNSSDualAntennaStatusPubType = ::rclcpp_lifecycle::LifecyclePublisher<GNSSDualAntennaStatusMsg>::SharedPtr;
using FilterHeadingStatePubType = ::rclcpp_lifecycle::LifecyclePublisher<FilterHeadingStateMsg>::SharedPtr;
using GPSCorrelationTimestampStampedPubType =
    ::rclcpp_lifecycle::LifecyclePublisher<GPSCorrelationTimestampStampedMsg>::SharedPtr;

// ROS2 Subscriber Message Types
using BoolMsg = ::std_msgs::msg::Bool;
using TimeReferenceMsg = ::sensor_msgs::msg::TimeReference;

// ROS2 Subscriber Types
using BoolSubType = ::rclcpp::Subscription<BoolMsg>::SharedPtr;
using TimeReferenceSubType = ::rclcpp::Subscription<TimeReferenceMsg>::SharedPtr;

// ROS2 Service Message Types
using TriggerServiceMsg = std_srvs::srv::Trigger;
using EmptyServiceMsg = std_srvs::srv::Empty;

using SetAccelBiasServiceMsg = mscl_msgs::srv::SetAccelBias;
using GetAccelBiasServiceMsg = mscl_msgs::srv::GetAccelBias;

using SetGyroBiasServiceMsg = mscl_msgs::srv::SetGyroBias;
using GetGyroBiasServiceMsg = mscl_msgs::srv::GetGyroBias;

using SetHardIronValuesServiceMsg = mscl_msgs::srv::SetHardIronValues;
using GetHardIronValuesServiceMsg = mscl_msgs::srv::GetHardIronValues;

using SetSoftIronMatrixServiceMsg = mscl_msgs::srv::SetSoftIronMatrix;
using GetSoftIronMatrixServiceMsg = mscl_msgs::srv::GetSoftIronMatrix;

using SetComplementaryFilterServiceMsg = mscl_msgs::srv::SetComplementaryFilter;
using GetComplementaryFilterServiceMsg = mscl_msgs::srv::GetComplementaryFilter;

using SetConingScullingCompServiceMsg = mscl_msgs::srv::SetConingScullingComp;
using GetConingScullingCompServiceMsg = mscl_msgs::srv::GetConingScullingComp;

using SetSensor2VehicleRotationServiceMsg = mscl_msgs::srv::SetSensor2VehicleRotation;
using GetSensor2VehicleRotationServiceMsg = mscl_msgs::srv::GetSensor2VehicleRotation;

using SetSensor2VehicleOffsetServiceMsg = mscl_msgs::srv::SetSensor2VehicleOffset;
using GetSensor2VehicleOffsetServiceMsg = mscl_msgs::srv::GetSensor2VehicleOffset;

using GetSensor2VehicleTransformationServiceMsg = mscl_msgs::srv::GetSensor2VehicleTransformation;

using InitFilterEulerServiceMsg = mscl_msgs::srv::InitFilterEuler;
using InitFilterHeadingServiceMsg = mscl_msgs::srv::InitFilterHeading;

using SetHeadingSourceServiceMsg = mscl_msgs::srv::SetHeadingSource;
using GetHeadingSourceServiceMsg = mscl_msgs::srv::GetHeadingSource;

using SetReferencePositionServiceMsg = mscl_msgs::srv::SetReferencePosition;
using GetReferencePositionServiceMsg = mscl_msgs::srv::GetReferencePosition;

using SetEstimationControlFlagsServiceMsg = mscl_msgs::srv::SetEstimationControlFlags;
using GetEstimationControlFlagsServiceMsg = mscl_msgs::srv::GetEstimationControlFlags;

using SetDynamicsModeServiceMsg = mscl_msgs::srv::SetDynamicsMode;
using GetDynamicsModeServiceMsg = mscl_msgs::srv::GetDynamicsMode;

using SetZeroAngleUpdateThresholdServiceMsg = mscl_msgs::srv::SetZeroAngleUpdateThreshold;
using GetZeroAngleUpdateThresholdServiceMsg = mscl_msgs::srv::GetZeroAngleUpdateThreshold;

using SetZeroVelocityUpdateThresholdServiceMsg = mscl_msgs::srv::SetZeroVelocityUpdateThreshold;
using GetZeroVelocityUpdateThresholdServiceMsg = mscl_msgs::srv::GetZeroVelocityUpdateThreshold;

using SetTareOrientationServiceMsg = mscl_msgs::srv::SetTareOrientation;

using SetAccelNoiseServiceMsg = mscl_msgs::srv::SetAccelNoise;
using GetAccelNoiseServiceMsg = mscl_msgs::srv::GetAccelNoise;

using SetGyroNoiseServiceMsg = mscl_msgs::srv::SetGyroNoise;
using GetGyroNoiseServiceMsg = mscl_msgs::srv::GetGyroNoise;

using SetMagNoiseServiceMsg = mscl_msgs::srv::SetMagNoise;
using GetMagNoiseServiceMsg = mscl_msgs::srv::GetMagNoise;

using SetGyroBiasModelServiceMsg = mscl_msgs::srv::SetGyroBiasModel;
using GetGyroBiasModelServiceMsg = mscl_msgs::srv::GetGyroBiasModel;

using SetAccelBiasModelServiceMsg = mscl_msgs::srv::SetAccelBiasModel;
using GetAccelBiasModelServiceMsg = mscl_msgs::srv::GetAccelBiasModel;

using SetGravityAdaptiveValsServiceMsg = mscl_msgs::srv::SetGravityAdaptiveVals;
using GetGravityAdaptiveValsServiceMsg = mscl_msgs::srv::GetGravityAdaptiveVals;

using SetMagAdaptiveValsServiceMsg = mscl_msgs::srv::SetMagAdaptiveVals;
using GetMagAdaptiveValsServiceMsg = mscl_msgs::srv::GetMagAdaptiveVals;

using SetMagDipAdaptiveValsServiceMsg = mscl_msgs::srv::SetMagDipAdaptiveVals;
using GetMagDipAdaptiveValsServiceMsg = mscl_msgs::srv::GetMagDipAdaptiveVals;

using ExternalHeadingUpdateServiceMsg = mscl_msgs::srv::ExternalHeadingUpdate;

using SetRelativePositionReferenceServiceMsg = mscl_msgs::srv::SetRelativePositionReference;
using GetRelativePositionReferenceServiceMsg = mscl_msgs::srv::GetRelativePositionReference;

using DeviceSettingsServiceMsg = mscl_msgs::srv::DeviceSettings;

// ROS2 Service Types
using TriggerServiceType = ::rclcpp::Service<TriggerServiceMsg>::SharedPtr;
using EmptyServiceType = ::rclcpp::Service<EmptyServiceMsg>::SharedPtr;

using SetAccelBiasServiceType = ::rclcpp::Service<SetAccelBiasServiceMsg>::SharedPtr;
using GetAccelBiasServiceType = ::rclcpp::Service<GetAccelBiasServiceMsg>::SharedPtr;

using SetGyroBiasServiceType = ::rclcpp::Service<SetGyroBiasServiceMsg>::SharedPtr;
using GetGyroBiasServiceType = ::rclcpp::Service<GetGyroBiasServiceMsg>::SharedPtr;

using SetHardIronValuesServiceType = ::rclcpp::Service<SetHardIronValuesServiceMsg>::SharedPtr;
using GetHardIronValuesServiceType = ::rclcpp::Service<GetHardIronValuesServiceMsg>::SharedPtr;

using SetSoftIronMatrixServiceType = ::rclcpp::Service<SetSoftIronMatrixServiceMsg>::SharedPtr;
using GetSoftIronMatrixServiceType = ::rclcpp::Service<GetSoftIronMatrixServiceMsg>::SharedPtr;

using SetComplementaryFilterServiceType = ::rclcpp::Service<SetComplementaryFilterServiceMsg>::SharedPtr;
using GetComplementaryFilterServiceType = ::rclcpp::Service<GetComplementaryFilterServiceMsg>::SharedPtr;

using SetConingScullingCompServiceType = ::rclcpp::Service<SetConingScullingCompServiceMsg>::SharedPtr;
using GetConingScullingCompServiceType = ::rclcpp::Service<GetConingScullingCompServiceMsg>::SharedPtr;

using SetSensor2VehicleRotationServiceType = ::rclcpp::Service<SetSensor2VehicleRotationServiceMsg>::SharedPtr;
using GetSensor2VehicleRotationServiceType = ::rclcpp::Service<GetSensor2VehicleRotationServiceMsg>::SharedPtr;

using SetSensor2VehicleOffsetServiceType = ::rclcpp::Service<SetSensor2VehicleOffsetServiceMsg>::SharedPtr;
using GetSensor2VehicleOffsetServiceType = ::rclcpp::Service<GetSensor2VehicleOffsetServiceMsg>::SharedPtr;

using GetSensor2VehicleTransformationServiceType =
    ::rclcpp::Service<GetSensor2VehicleTransformationServiceMsg>::SharedPtr;

using InitFilterEulerServiceType = ::rclcpp::Service<InitFilterEulerServiceMsg>::SharedPtr;
using InitFilterHeadingServiceType = ::rclcpp::Service<InitFilterHeadingServiceMsg>::SharedPtr;

using SetHeadingSourceServiceType = ::rclcpp::Service<SetHeadingSourceServiceMsg>::SharedPtr;
using GetHeadingSourceServiceType = ::rclcpp::Service<GetHeadingSourceServiceMsg>::SharedPtr;

using SetReferencePositionServiceType = ::rclcpp::Service<SetReferencePositionServiceMsg>::SharedPtr;
using GetReferencePositionServiceType = ::rclcpp::Service<GetReferencePositionServiceMsg>::SharedPtr;

using SetEstimationControlFlagsServiceType = ::rclcpp::Service<SetEstimationControlFlagsServiceMsg>::SharedPtr;
using GetEstimationControlFlagsServiceType = ::rclcpp::Service<GetEstimationControlFlagsServiceMsg>::SharedPtr;

using SetDynamicsModeServiceType = ::rclcpp::Service<SetDynamicsModeServiceMsg>::SharedPtr;
using GetDynamicsModeServiceType = ::rclcpp::Service<GetDynamicsModeServiceMsg>::SharedPtr;

using SetZeroAngleUpdateThresholdServiceType = ::rclcpp::Service<SetZeroAngleUpdateThresholdServiceMsg>::SharedPtr;
using GetZeroAngleUpdateThresholdServiceType = ::rclcpp::Service<GetZeroAngleUpdateThresholdServiceMsg>::SharedPtr;

using SetZeroVelocityUpdateThresholdServiceType =
    ::rclcpp::Service<SetZeroVelocityUpdateThresholdServiceMsg>::SharedPtr;
using GetZeroVelocityUpdateThresholdServiceType =
    ::rclcpp::Service<GetZeroVelocityUpdateThresholdServiceMsg>::SharedPtr;

using SetTareOrientationServiceType = ::rclcpp::Service<SetTareOrientationServiceMsg>::SharedPtr;

using SetAccelNoiseServiceType = ::rclcpp::Service<SetAccelNoiseServiceMsg>::SharedPtr;
using GetAccelNoiseServiceType = ::rclcpp::Service<GetAccelNoiseServiceMsg>::SharedPtr;

using SetGyroNoiseServiceType = ::rclcpp::Service<SetGyroNoiseServiceMsg>::SharedPtr;
using GetGyroNoiseServiceType = ::rclcpp::Service<GetGyroNoiseServiceMsg>::SharedPtr;

using SetMagNoiseServiceType = ::rclcpp::Service<SetMagNoiseServiceMsg>::SharedPtr;
using GetMagNoiseServiceType = ::rclcpp::Service<GetMagNoiseServiceMsg>::SharedPtr;

using SetGyroBiasModelServiceType = ::rclcpp::Service<SetGyroBiasModelServiceMsg>::SharedPtr;
using GetGyroBiasModelServiceType = ::rclcpp::Service<GetGyroBiasModelServiceMsg>::SharedPtr;

using SetAccelBiasModelServiceType = ::rclcpp::Service<SetAccelBiasModelServiceMsg>::SharedPtr;
using GetAccelBiasModelServiceType = ::rclcpp::Service<GetAccelBiasModelServiceMsg>::SharedPtr;

using SetGravityAdaptiveValsServiceType = ::rclcpp::Service<SetGravityAdaptiveValsServiceMsg>::SharedPtr;
using GetGravityAdaptiveValsServiceType = ::rclcpp::Service<GetGravityAdaptiveValsServiceMsg>::SharedPtr;

using SetMagAdaptiveValsServiceType = ::rclcpp::Service<SetMagAdaptiveValsServiceMsg>::SharedPtr;
using GetMagAdaptiveValsServiceType = ::rclcpp::Service<GetMagAdaptiveValsServiceMsg>::SharedPtr;

using SetMagDipAdaptiveValsServiceType = ::rclcpp::Service<SetMagDipAdaptiveValsServiceMsg>::SharedPtr;
using GetMagDipAdaptiveValsServiceType = ::rclcpp::Service<GetMagDipAdaptiveValsServiceMsg>::SharedPtr;

using ExternalHeadingUpdateServiceType = ::rclcpp::Service<ExternalHeadingUpdateServiceMsg>::SharedPtr;

using SetRelativePositionReferenceServiceType = ::rclcpp::Service<SetRelativePositionReferenceServiceMsg>::SharedPtr;
using GetRelativePositionReferenceServiceType = ::rclcpp::Service<GetRelativePositionReferenceServiceMsg>::SharedPtr;

using DeviceSettingsServiceType = ::rclcpp::Service<DeviceSettingsServiceMsg>::SharedPtr;

///
/// ROS2 Logging
///
#define MICROSTRAIN_DEBUG(NODE, ...) RCLCPP_DEBUG(NODE->get_logger(), __VA_ARGS__)
#define MICROSTRAIN_INFO(NODE, ...) RCLCPP_INFO(NODE->get_logger(), __VA_ARGS__)
#define MICROSTRAIN_WARN(NODE, ...) RCLCPP_WARN(NODE->get_logger(), __VA_ARGS__)
#define MICROSTRAIN_ERROR(NODE, ...) RCLCPP_ERROR(NODE->get_logger(), __VA_ARGS__)
#define MICROSTRAIN_FATAL(NODE, ...) RCLCPP_FATAL(NODE->get_logger(), __VA_ARGS__)

#define MICROSTRAIN_DEBUG_THROTTLE(NODE, PERIOD, ...)                                                                  \
  RCLCPP_DEBUG_THROTTLE(NODE->get_logger(), *NODE->get_clock(), PERIOD, __VA_ARGS__)
#else
#error "Unsupported ROS version. -DMICROSTRAIN_ROS_VERSION must be set to 1 or 2"
#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Common Defines
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
constexpr auto NUM_COMMAND_LINE_ARGUMENTS = 3;

constexpr auto DEFAULT_PACKET_TIMEOUT_MS = 1000;  // milliseconds

constexpr auto SECS_PER_WEEK = (60L * 60 * 24 * 7);
constexpr auto UTC_GPS_EPOCH_DUR = (315964800);

constexpr auto USTRAIN_G =
    9.80665;  // from section 5.1.1 in
              // https://www.microstrain.com/sites/default/files/3dm-gx5-25_dcp_manual_8500-0065_reference_document.pdf

constexpr auto GNSS1_ID = 0;
constexpr auto GNSS2_ID = 1;
constexpr auto NUM_GNSS = 2;
}  // namespace microstrain

#endif  // ROS_MSCL_COMMON_MICROSTRAIN_DEFS_H
