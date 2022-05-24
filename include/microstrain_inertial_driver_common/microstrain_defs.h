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
#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_MICROSTRAIN_DEFS_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_MICROSTRAIN_DEFS_H

/**
 * Common Includes
 */
#include <memory>

/**
 * Common Defines
 */


#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif


namespace microstrain
{
constexpr auto GNSS1_ID = 0;
constexpr auto GNSS2_ID = 1;
constexpr auto NUM_GNSS = 2;
};  // namespace microstrain

/**
 * ROS1 Includes
 */
#if MICROSTRAIN_ROS_VERSION == 1
#include "ros/ros.h"
#include "tf2_ros/transform_broadcaster.h"

#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/TimeReference.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/MagneticField.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "mavros_msgs/RTCM.h"
#include "nmea_msgs/Sentence.h"

#include "microstrain_inertial_msgs/Status.h"
#include "microstrain_inertial_msgs/RTKStatus.h"
#include "microstrain_inertial_msgs/RTKStatusV1.h"
#include "microstrain_inertial_msgs/FilterStatus.h"
#include "microstrain_inertial_msgs/FilterHeading.h"
#include "microstrain_inertial_msgs/FilterHeadingState.h"
#include "microstrain_inertial_msgs/FilterAidingMeasurementSummary.h"
#include "microstrain_inertial_msgs/GPSCorrelationTimestampStamped.h"
#include "microstrain_inertial_msgs/GNSSAidingStatus.h"
#include "microstrain_inertial_msgs/GNSSDualAntennaStatus.h"
#include "microstrain_inertial_msgs/GNSSFixInfo.h"

#include "microstrain_inertial_msgs/InputSpeedMeasurement.h"

#include "std_srvs/Empty.h"
#include "std_srvs/Trigger.h"

#include "microstrain_inertial_msgs/SetAccelBias.h"
#include "microstrain_inertial_msgs/GetAccelBias.h"
#include "microstrain_inertial_msgs/SetGyroBias.h"
#include "microstrain_inertial_msgs/GetGyroBias.h"
#include "microstrain_inertial_msgs/SetHardIronValues.h"
#include "microstrain_inertial_msgs/GetHardIronValues.h"
#include "microstrain_inertial_msgs/SetSoftIronMatrix.h"
#include "microstrain_inertial_msgs/GetSoftIronMatrix.h"
#include "microstrain_inertial_msgs/SetComplementaryFilter.h"
#include "microstrain_inertial_msgs/GetComplementaryFilter.h"
#include "microstrain_inertial_msgs/InitFilterEuler.h"
#include "microstrain_inertial_msgs/InitFilterHeading.h"
#include "microstrain_inertial_msgs/DeviceReport.h"
#include "microstrain_inertial_msgs/DeviceSettings.h"
#include "microstrain_inertial_msgs/SetAccelBiasModel.h"
#include "microstrain_inertial_msgs/GetAccelBiasModel.h"
#include "microstrain_inertial_msgs/SetGravityAdaptiveVals.h"
#include "microstrain_inertial_msgs/GetGravityAdaptiveVals.h"
#include "microstrain_inertial_msgs/SetSensor2VehicleRotation.h"
#include "microstrain_inertial_msgs/GetSensor2VehicleRotation.h"
#include "microstrain_inertial_msgs/SetSensor2VehicleOffset.h"
#include "microstrain_inertial_msgs/GetSensor2VehicleOffset.h"
#include "microstrain_inertial_msgs/SetReferencePosition.h"
#include "microstrain_inertial_msgs/GetReferencePosition.h"
#include "microstrain_inertial_msgs/SetConingScullingComp.h"
#include "microstrain_inertial_msgs/GetConingScullingComp.h"
#include "microstrain_inertial_msgs/SetEstimationControlFlags.h"
#include "microstrain_inertial_msgs/GetEstimationControlFlags.h"
#include "microstrain_inertial_msgs/SetDynamicsMode.h"
#include "microstrain_inertial_msgs/GetDynamicsMode.h"
#include "microstrain_inertial_msgs/SetZeroAngleUpdateThreshold.h"
#include "microstrain_inertial_msgs/GetZeroAngleUpdateThreshold.h"
#include "microstrain_inertial_msgs/SetZeroVelocityUpdateThreshold.h"
#include "microstrain_inertial_msgs/GetZeroVelocityUpdateThreshold.h"
#include "microstrain_inertial_msgs/SetTareOrientation.h"
#include "microstrain_inertial_msgs/SetAccelNoise.h"
#include "microstrain_inertial_msgs/GetAccelNoise.h"
#include "microstrain_inertial_msgs/SetGyroNoise.h"
#include "microstrain_inertial_msgs/GetGyroNoise.h"
#include "microstrain_inertial_msgs/SetMagNoise.h"
#include "microstrain_inertial_msgs/GetMagNoise.h"
#include "microstrain_inertial_msgs/SetGyroBiasModel.h"
#include "microstrain_inertial_msgs/GetGyroBiasModel.h"
#include "microstrain_inertial_msgs/SetMagAdaptiveVals.h"
#include "microstrain_inertial_msgs/GetMagAdaptiveVals.h"
#include "microstrain_inertial_msgs/SetMagDipAdaptiveVals.h"
#include "microstrain_inertial_msgs/GetMagDipAdaptiveVals.h"
#include "microstrain_inertial_msgs/SetHeadingSource.h"
#include "microstrain_inertial_msgs/GetHeadingSource.h"
#include "microstrain_inertial_msgs/GetSensor2VehicleTransformation.h"
#include "microstrain_inertial_msgs/ExternalHeadingUpdate.h"
#include "microstrain_inertial_msgs/SetRelativePositionReference.h"
#include "microstrain_inertial_msgs/GetRelativePositionReference.h"
#include "microstrain_inertial_msgs/SetFilterSpeedLeverArm.h"

/**
 * ROS2 Includes
 */
#elif MICROSTRAIN_ROS_VERSION == 2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "lifecycle_msgs/msg/transition.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/transform_stamped.h"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "mavros_msgs/msg/rtcm.hpp"
#include "nmea_msgs/msg/sentence.hpp"

// .h header was deprecated in rolling and will likely be removed in future releases.
#if MICROSTRAIN_ROLLING == 1
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#endif

#include "microstrain_inertial_msgs/msg/status.hpp"
#include "microstrain_inertial_msgs/msg/rtk_status.hpp"
#include "microstrain_inertial_msgs/msg/rtk_status_v1.hpp"
#include "microstrain_inertial_msgs/msg/filter_status.hpp"
#include "microstrain_inertial_msgs/msg/filter_heading.hpp"
#include "microstrain_inertial_msgs/msg/filter_heading_state.hpp"
#include "microstrain_inertial_msgs/msg/filter_aiding_measurement_summary.hpp"
#include "microstrain_inertial_msgs/msg/gps_correlation_timestamp_stamped.hpp"
#include "microstrain_inertial_msgs/msg/gnss_aiding_status.hpp"
#include "microstrain_inertial_msgs/msg/gnss_dual_antenna_status.hpp"
#include "microstrain_inertial_msgs/msg/gnss_fix_info.hpp"

#include "microstrain_inertial_msgs/msg/input_speed_measurement.hpp"

#include "microstrain_inertial_msgs/srv/set_accel_bias.hpp"
#include "microstrain_inertial_msgs/srv/get_accel_bias.hpp"
#include "microstrain_inertial_msgs/srv/set_gyro_bias.hpp"
#include "microstrain_inertial_msgs/srv/get_gyro_bias.hpp"
#include "microstrain_inertial_msgs/srv/set_hard_iron_values.hpp"
#include "microstrain_inertial_msgs/srv/get_hard_iron_values.hpp"
#include "microstrain_inertial_msgs/srv/set_soft_iron_matrix.hpp"
#include "microstrain_inertial_msgs/srv/get_soft_iron_matrix.hpp"
#include "microstrain_inertial_msgs/srv/set_complementary_filter.hpp"
#include "microstrain_inertial_msgs/srv/get_complementary_filter.hpp"
#include "microstrain_inertial_msgs/srv/init_filter_euler.hpp"
#include "microstrain_inertial_msgs/srv/init_filter_heading.hpp"
#include "microstrain_inertial_msgs/srv/device_report.hpp"
#include "microstrain_inertial_msgs/srv/device_settings.hpp"
#include "microstrain_inertial_msgs/srv/set_accel_bias_model.hpp"
#include "microstrain_inertial_msgs/srv/get_accel_bias_model.hpp"
#include "microstrain_inertial_msgs/srv/set_gravity_adaptive_vals.hpp"
#include "microstrain_inertial_msgs/srv/get_gravity_adaptive_vals.hpp"
#include "microstrain_inertial_msgs/srv/set_sensor2_vehicle_rotation.hpp"
#include "microstrain_inertial_msgs/srv/get_sensor2_vehicle_rotation.hpp"
#include "microstrain_inertial_msgs/srv/set_sensor2_vehicle_offset.hpp"
#include "microstrain_inertial_msgs/srv/get_sensor2_vehicle_offset.hpp"
#include "microstrain_inertial_msgs/srv/set_reference_position.hpp"
#include "microstrain_inertial_msgs/srv/get_reference_position.hpp"
#include "microstrain_inertial_msgs/srv/set_coning_sculling_comp.hpp"
#include "microstrain_inertial_msgs/srv/get_coning_sculling_comp.hpp"
#include "microstrain_inertial_msgs/srv/set_estimation_control_flags.hpp"
#include "microstrain_inertial_msgs/srv/get_estimation_control_flags.hpp"
#include "microstrain_inertial_msgs/srv/set_dynamics_mode.hpp"
#include "microstrain_inertial_msgs/srv/get_dynamics_mode.hpp"
#include "microstrain_inertial_msgs/srv/set_zero_angle_update_threshold.hpp"
#include "microstrain_inertial_msgs/srv/get_zero_angle_update_threshold.hpp"
#include "microstrain_inertial_msgs/srv/set_zero_velocity_update_threshold.hpp"
#include "microstrain_inertial_msgs/srv/get_zero_velocity_update_threshold.hpp"
#include "microstrain_inertial_msgs/srv/set_tare_orientation.hpp"
#include "microstrain_inertial_msgs/srv/set_accel_noise.hpp"
#include "microstrain_inertial_msgs/srv/get_accel_noise.hpp"
#include "microstrain_inertial_msgs/srv/set_gyro_noise.hpp"
#include "microstrain_inertial_msgs/srv/get_gyro_noise.hpp"
#include "microstrain_inertial_msgs/srv/set_mag_noise.hpp"
#include "microstrain_inertial_msgs/srv/get_mag_noise.hpp"
#include "microstrain_inertial_msgs/srv/set_gyro_bias_model.hpp"
#include "microstrain_inertial_msgs/srv/get_gyro_bias_model.hpp"
#include "microstrain_inertial_msgs/srv/set_mag_adaptive_vals.hpp"
#include "microstrain_inertial_msgs/srv/get_mag_adaptive_vals.hpp"
#include "microstrain_inertial_msgs/srv/set_mag_dip_adaptive_vals.hpp"
#include "microstrain_inertial_msgs/srv/get_mag_dip_adaptive_vals.hpp"
#include "microstrain_inertial_msgs/srv/set_heading_source.hpp"
#include "microstrain_inertial_msgs/srv/get_heading_source.hpp"
#include "microstrain_inertial_msgs/srv/get_sensor2_vehicle_transformation.hpp"
#include "microstrain_inertial_msgs/srv/external_heading_update.hpp"
#include "microstrain_inertial_msgs/srv/set_relative_position_reference.hpp"
#include "microstrain_inertial_msgs/srv/get_relative_position_reference.hpp"
#include "microstrain_inertial_msgs/srv/set_filter_speed_lever_arm.hpp"
#else
#error "Unsupported ROS version. -DMICROSTRAIN_ROS_VERSION must be set to 1 or 2"
#endif

namespace microstrain
{
/**
 * ROS1 Defines
 */
#if MICROSTRAIN_ROS_VERSION == 1
// ROS1 General Types
using RosNodeType = ::ros::NodeHandle;
using RosTimeType = ::ros::Time;
using RosTimerType = std::shared_ptr<::ros::Timer>;
using RosRateType = ::ros::Rate;
using RosHeaderType = ::std_msgs::Header;
using RosPubType = std::shared_ptr<::ros::Publisher>;
using RosSubType = std::shared_ptr<::ros::Subscriber>;
using RosServiceType = std::shared_ptr<::ros::ServiceServer>;

// ROS1 Publisher Message Types
using OdometryMsg = ::nav_msgs::Odometry;
using ImuMsg = ::sensor_msgs::Imu;
using NavSatFixMsg = ::sensor_msgs::NavSatFix;
using MagneticFieldMsg = ::sensor_msgs::MagneticField;
using TimeReferenceMsg = ::sensor_msgs::TimeReference;
using NMEASentenceMsg = ::nmea_msgs::Sentence;
using StatusMsg = ::microstrain_inertial_msgs::Status;
using RTKStatusMsg = ::microstrain_inertial_msgs::RTKStatus;
using RTKStatusMsgV1 = ::microstrain_inertial_msgs::RTKStatusV1;
using FilterStatusMsg = ::microstrain_inertial_msgs::FilterStatus;
using FilterHeadingMsg = ::microstrain_inertial_msgs::FilterHeading;
using FilterAidingMeasurementSummaryMsg = ::microstrain_inertial_msgs::FilterAidingMeasurementSummary;
using FilterAidingMeasurementSummaryIndicatorMsg = ::microstrain_inertial_msgs::FilterAidingMeasurementSummaryIndicator;
using GNSSAidingStatusMsg = ::microstrain_inertial_msgs::GNSSAidingStatus;
using GNSSDualAntennaStatusMsg = ::microstrain_inertial_msgs::GNSSDualAntennaStatus;
using GNSSFixInfoMsg = ::microstrain_inertial_msgs::GNSSFixInfo;
using FilterHeadingStateMsg = ::microstrain_inertial_msgs::FilterHeadingState;
using GPSCorrelationTimestampStampedMsg = ::microstrain_inertial_msgs::GPSCorrelationTimestampStamped;
using TransformStampedMsg = ::geometry_msgs::TransformStamped;

// ROS1 Publisher Types
using OdometryPubType = RosPubType;
using ImuPubType = RosPubType;
using NavSatFixPubType = RosPubType;
using MagneticFieldPubType = RosPubType;
using TimeReferencePubType = RosPubType;
using NMEASentencePubType = RosPubType;
using StatusPubType = RosPubType;
using RTKStatusPubType = RosPubType;
using RTKStatusPubTypeV1 = RosPubType;
using FilterStatusPubType = RosPubType;
using FilterHeadingPubType = RosPubType;
using FilterAidingMeasurementSummaryPubType = RosPubType;
using GNSSAidingStatusPubType = RosPubType;
using GNSSDualAntennaStatusPubType = RosPubType;
using GNSSFixInfoPubType = RosPubType;
using FilterHeadingStatePubType = RosPubType;
using GPSCorrelationTimestampStampedPubType = RosPubType;

// ROS1 Transform Broadcaster
using TransformBroadcasterType = std::shared_ptr<::tf2_ros::TransformBroadcaster>;

// ROS1 Subscriber Message Types
using BoolMsg = ::std_msgs::Bool;
using TimeReferenceMsg = ::sensor_msgs::TimeReference;
using InputSpeedMeasurementMsg = ::microstrain_inertial_msgs::InputSpeedMeasurement;
using RTCMMsg = ::mavros_msgs::RTCM;

// ROS1 Subscriber Types
using BoolSubType = RosSubType;
using TimeReferenceSubType = RosSubType;
using InputSpeedMeasurementSubType = RosSubType;
using RTCMSubType = RosSubType;

// ROS1 Service Message Types
using TriggerServiceMsg = std_srvs::Trigger;
using EmptyServiceMsg = std_srvs::Empty;

using SetAccelBiasServiceMsg = ::microstrain_inertial_msgs::SetAccelBias;
using GetAccelBiasServiceMsg = ::microstrain_inertial_msgs::GetAccelBias;

using SetGyroBiasServiceMsg = ::microstrain_inertial_msgs::SetGyroBias;
using GetGyroBiasServiceMsg = ::microstrain_inertial_msgs::GetGyroBias;

using SetHardIronValuesServiceMsg = ::microstrain_inertial_msgs::SetHardIronValues;
using GetHardIronValuesServiceMsg = ::microstrain_inertial_msgs::GetHardIronValues;

using SetSoftIronMatrixServiceMsg = ::microstrain_inertial_msgs::SetSoftIronMatrix;
using GetSoftIronMatrixServiceMsg = ::microstrain_inertial_msgs::GetSoftIronMatrix;

using SetComplementaryFilterServiceMsg = ::microstrain_inertial_msgs::SetComplementaryFilter;
using GetComplementaryFilterServiceMsg = ::microstrain_inertial_msgs::GetComplementaryFilter;

using SetConingScullingCompServiceMsg = ::microstrain_inertial_msgs::SetConingScullingComp;
using GetConingScullingCompServiceMsg = ::microstrain_inertial_msgs::GetConingScullingComp;

using SetSensor2VehicleRotationServiceMsg = ::microstrain_inertial_msgs::SetSensor2VehicleRotation;
using GetSensor2VehicleRotationServiceMsg = ::microstrain_inertial_msgs::GetSensor2VehicleRotation;

using SetSensor2VehicleOffsetServiceMsg = ::microstrain_inertial_msgs::SetSensor2VehicleOffset;
using GetSensor2VehicleOffsetServiceMsg = ::microstrain_inertial_msgs::GetSensor2VehicleOffset;

using GetSensor2VehicleTransformationServiceMsg = ::microstrain_inertial_msgs::GetSensor2VehicleTransformation;

using InitFilterEulerServiceMsg = ::microstrain_inertial_msgs::InitFilterEuler;
using InitFilterHeadingServiceMsg = ::microstrain_inertial_msgs::InitFilterHeading;

using SetHeadingSourceServiceMsg = ::microstrain_inertial_msgs::SetHeadingSource;
using GetHeadingSourceServiceMsg = ::microstrain_inertial_msgs::GetHeadingSource;

using SetReferencePositionServiceMsg = ::microstrain_inertial_msgs::SetReferencePosition;
using GetReferencePositionServiceMsg = ::microstrain_inertial_msgs::GetReferencePosition;

using SetEstimationControlFlagsServiceMsg = ::microstrain_inertial_msgs::SetEstimationControlFlags;
using GetEstimationControlFlagsServiceMsg = ::microstrain_inertial_msgs::GetEstimationControlFlags;

using SetDynamicsModeServiceMsg = ::microstrain_inertial_msgs::SetDynamicsMode;
using GetDynamicsModeServiceMsg = ::microstrain_inertial_msgs::GetDynamicsMode;

using SetZeroAngleUpdateThresholdServiceMsg = ::microstrain_inertial_msgs::SetZeroAngleUpdateThreshold;
using GetZeroAngleUpdateThresholdServiceMsg = ::microstrain_inertial_msgs::GetZeroAngleUpdateThreshold;

using SetZeroVelocityUpdateThresholdServiceMsg = ::microstrain_inertial_msgs::SetZeroVelocityUpdateThreshold;
using GetZeroVelocityUpdateThresholdServiceMsg = ::microstrain_inertial_msgs::GetZeroVelocityUpdateThreshold;

using SetTareOrientationServiceMsg = ::microstrain_inertial_msgs::SetTareOrientation;

using SetAccelNoiseServiceMsg = ::microstrain_inertial_msgs::SetAccelNoise;
using GetAccelNoiseServiceMsg = ::microstrain_inertial_msgs::GetAccelNoise;

using SetGyroNoiseServiceMsg = ::microstrain_inertial_msgs::SetGyroNoise;
using GetGyroNoiseServiceMsg = ::microstrain_inertial_msgs::GetGyroNoise;

using SetMagNoiseServiceMsg = ::microstrain_inertial_msgs::SetMagNoise;
using GetMagNoiseServiceMsg = ::microstrain_inertial_msgs::GetMagNoise;

using SetGyroBiasModelServiceMsg = ::microstrain_inertial_msgs::SetGyroBiasModel;
using GetGyroBiasModelServiceMsg = ::microstrain_inertial_msgs::GetGyroBiasModel;

using SetAccelBiasModelServiceMsg = ::microstrain_inertial_msgs::SetAccelBiasModel;
using GetAccelBiasModelServiceMsg = ::microstrain_inertial_msgs::GetAccelBiasModel;

using SetGravityAdaptiveValsServiceMsg = ::microstrain_inertial_msgs::SetGravityAdaptiveVals;
using GetGravityAdaptiveValsServiceMsg = ::microstrain_inertial_msgs::GetGravityAdaptiveVals;

using SetMagAdaptiveValsServiceMsg = ::microstrain_inertial_msgs::SetMagAdaptiveVals;
using GetMagAdaptiveValsServiceMsg = ::microstrain_inertial_msgs::GetMagAdaptiveVals;

using SetMagDipAdaptiveValsServiceMsg = ::microstrain_inertial_msgs::SetMagDipAdaptiveVals;
using GetMagDipAdaptiveValsServiceMsg = ::microstrain_inertial_msgs::GetMagDipAdaptiveVals;

using ExternalHeadingUpdateServiceMsg = ::microstrain_inertial_msgs::ExternalHeadingUpdate;

using SetRelativePositionReferenceServiceMsg = ::microstrain_inertial_msgs::SetRelativePositionReference;
using GetRelativePositionReferenceServiceMsg = ::microstrain_inertial_msgs::GetRelativePositionReference;

using DeviceReportServiceMsg = ::microstrain_inertial_msgs::DeviceReport;
using DeviceSettingsServiceMsg = ::microstrain_inertial_msgs::DeviceSettings;

using SetFilterSpeedLeverArmServiceMsg = ::microstrain_inertial_msgs::SetFilterSpeedLeverArm;

// ROS1 Service Types
using TriggerServiceType = RosServiceType;
using EmptyServiceType = RosServiceType;

using SetAccelBiasServiceType = RosServiceType;
using GetAccelBiasServiceType = RosServiceType;

using SetGyroBiasServiceType = RosServiceType;
using GetGyroBiasServiceType = RosServiceType;

using SetHardIronValuesServiceType = RosServiceType;
using GetHardIronValuesServiceType = RosServiceType;

using SetSoftIronMatrixServiceType = RosServiceType;
using GetSoftIronMatrixServiceType = RosServiceType;

using SetComplementaryFilterServiceType = RosServiceType;
using GetComplementaryFilterServiceType = RosServiceType;

using SetConingScullingCompServiceType = RosServiceType;
using GetConingScullingCompServiceType = RosServiceType;

using SetSensor2VehicleRotationServiceType = RosServiceType;
using GetSensor2VehicleRotationServiceType = RosServiceType;

using SetSensor2VehicleOffsetServiceType = RosServiceType;
using GetSensor2VehicleOffsetServiceType = RosServiceType;

using GetSensor2VehicleTransformationServiceType = RosServiceType;

using InitFilterEulerServiceType = RosServiceType;
using InitFilterHeadingServiceType = RosServiceType;

using SetHeadingSourceServiceType = RosServiceType;
using GetHeadingSourceServiceType = RosServiceType;

using SetReferencePositionServiceType = RosServiceType;
using GetReferencePositionServiceType = RosServiceType;

using SetEstimationControlFlagsServiceType = RosServiceType;
using GetEstimationControlFlagsServiceType = RosServiceType;

using SetDynamicsModeServiceType = RosServiceType;
using GetDynamicsModeServiceType = RosServiceType;

using SetZeroAngleUpdateThresholdServiceType = RosServiceType;
using GetZeroAngleUpdateThresholdServiceType = RosServiceType;

using SetZeroVelocityUpdateThresholdServiceType = RosServiceType;
using GetZeroVelocityUpdateThresholdServiceType = RosServiceType;

using SetTareOrientationServiceType = RosServiceType;

using SetAccelNoiseServiceType = RosServiceType;
using GetAccelNoiseServiceType = RosServiceType;

using SetGyroNoiseServiceType = RosServiceType;
using GetGyroNoiseServiceType = RosServiceType;

using SetMagNoiseServiceType = RosServiceType;
using GetMagNoiseServiceType = RosServiceType;

using SetGyroBiasModelServiceType = RosServiceType;
using GetGyroBiasModelServiceType = RosServiceType;

using SetAccelBiasModelServiceType = RosServiceType;
using GetAccelBiasModelServiceType = RosServiceType;

using SetGravityAdaptiveValsServiceType = RosServiceType;
using GetGravityAdaptiveValsServiceType = RosServiceType;

using SetMagAdaptiveValsServiceType = RosServiceType;
using GetMagAdaptiveValsServiceType = RosServiceType;

using SetMagDipAdaptiveValsServiceType = RosServiceType;
using GetMagDipAdaptiveValsServiceType = RosServiceType;

using ExternalHeadingUpdateServiceType = RosServiceType;

using SetRelativePositionReferenceServiceType = RosServiceType;
using GetRelativePositionReferenceServiceType = RosServiceType;

using DeviceReportServiceType = RosServiceType;
using DeviceSettingsServiceType = RosServiceType;

using SetFilterSpeedLeverArmServiceType = RosServiceType;

// ROS1 Logging
#define MICROSTRAIN_DEBUG(NODE, ...) ROS_DEBUG(__VA_ARGS__)
#define MICROSTRAIN_INFO(NODE, ...) ROS_INFO(__VA_ARGS__)
#define MICROSTRAIN_WARN(NODE, ...) ROS_WARN(__VA_ARGS__)
#define MICROSTRAIN_ERROR(NODE, ...) ROS_ERROR(__VA_ARGS__)
#define MICROSTRAIN_FATAL(NOE, ...) ROS_FATAL(__VA_ARGS__)

#define MICROSTRAIN_DEBUG_THROTTLE(NODE, PERIOD, ...) ROS_DEBUG_THROTTLE(PERIOD, __VA_ARGS__)
/**
 * ROS2 Defines
 */
#elif MICROSTRAIN_ROS_VERSION == 2
// ROS2 Generic Types
using RosNodeType = ::rclcpp_lifecycle::LifecycleNode;
using RosTimeType = ::rclcpp::Time;
using RosTimerType = ::rclcpp::TimerBase::SharedPtr;
using RosRateType = ::rclcpp::Rate;
using RosHeaderType = ::std_msgs::msg::Header;

// ROS2 Publisher Message Types
using OdometryMsg = ::nav_msgs::msg::Odometry;
using ImuMsg = ::sensor_msgs::msg::Imu;
using NavSatFixMsg = ::sensor_msgs::msg::NavSatFix;
using MagneticFieldMsg = ::sensor_msgs::msg::MagneticField;
using TimeReferenceMsg = ::sensor_msgs::msg::TimeReference;
using NMEASentenceMsg = ::nmea_msgs::msg::Sentence;
using StatusMsg = ::microstrain_inertial_msgs::msg::Status;
using RTKStatusMsg = ::microstrain_inertial_msgs::msg::RTKStatus;
using RTKStatusMsgV1 = ::microstrain_inertial_msgs::msg::RTKStatusV1;
using FilterStatusMsg = ::microstrain_inertial_msgs::msg::FilterStatus;
using FilterHeadingMsg = ::microstrain_inertial_msgs::msg::FilterHeading;
using FilterAidingMeasurementSummaryMsg = ::microstrain_inertial_msgs::msg::FilterAidingMeasurementSummary;
using FilterAidingMeasurementSummaryIndicatorMsg = ::microstrain_inertial_msgs::msg::FilterAidingMeasurementSummaryIndicator;
using GNSSAidingStatusMsg = ::microstrain_inertial_msgs::msg::GNSSAidingStatus;
using GNSSDualAntennaStatusMsg = ::microstrain_inertial_msgs::msg::GNSSDualAntennaStatus;
using GNSSFixInfoMsg = ::microstrain_inertial_msgs::msg::GNSSFixInfo;
using FilterHeadingStateMsg = ::microstrain_inertial_msgs::msg::FilterHeadingState;
using GPSCorrelationTimestampStampedMsg = ::microstrain_inertial_msgs::msg::GPSCorrelationTimestampStamped;
using TransformStampedMsg = ::geometry_msgs::msg::TransformStamped;

// ROS2 Publisher Types
using OdometryPubType = ::rclcpp_lifecycle::LifecyclePublisher<OdometryMsg>::SharedPtr;
using ImuPubType = ::rclcpp_lifecycle::LifecyclePublisher<ImuMsg>::SharedPtr;
using NavSatFixPubType = ::rclcpp_lifecycle::LifecyclePublisher<NavSatFixMsg>::SharedPtr;
using MagneticFieldPubType = ::rclcpp_lifecycle::LifecyclePublisher<MagneticFieldMsg>::SharedPtr;
using TimeReferencePubType = ::rclcpp_lifecycle::LifecyclePublisher<TimeReferenceMsg>::SharedPtr;
using NMEASentencePubType = ::rclcpp_lifecycle::LifecyclePublisher<NMEASentenceMsg>::SharedPtr;
using StatusPubType = ::rclcpp_lifecycle::LifecyclePublisher<StatusMsg>::SharedPtr;
using RTKStatusPubType = ::rclcpp_lifecycle::LifecyclePublisher<RTKStatusMsg>::SharedPtr;
using RTKStatusPubTypeV1 = ::rclcpp_lifecycle::LifecyclePublisher<RTKStatusMsgV1>::SharedPtr;
using FilterStatusPubType = ::rclcpp_lifecycle::LifecyclePublisher<FilterStatusMsg>::SharedPtr;
using FilterHeadingPubType = ::rclcpp_lifecycle::LifecyclePublisher<FilterHeadingMsg>::SharedPtr;
using FilterAidingMeasurementSummaryPubType = ::rclcpp_lifecycle::LifecyclePublisher<FilterAidingMeasurementSummaryMsg>::SharedPtr;
using GNSSAidingStatusPubType = ::rclcpp_lifecycle::LifecyclePublisher<GNSSAidingStatusMsg>::SharedPtr;
using GNSSDualAntennaStatusPubType = ::rclcpp_lifecycle::LifecyclePublisher<GNSSDualAntennaStatusMsg>::SharedPtr;
using GNSSFixInfoPubType = ::rclcpp_lifecycle::LifecyclePublisher<GNSSFixInfoMsg>::SharedPtr;
using FilterHeadingStatePubType = ::rclcpp_lifecycle::LifecyclePublisher<FilterHeadingStateMsg>::SharedPtr;
using GPSCorrelationTimestampStampedPubType =
    ::rclcpp_lifecycle::LifecyclePublisher<GPSCorrelationTimestampStampedMsg>::SharedPtr;

// ROS2 Transform Broadcaster
using TransformBroadcasterType = std::shared_ptr<::tf2_ros::TransformBroadcaster>;

// ROS2 Subscriber Message Types
using BoolMsg = ::std_msgs::msg::Bool;
using TimeReferenceMsg = ::sensor_msgs::msg::TimeReference;
using InputSpeedMeasurementMsg = ::microstrain_inertial_msgs::msg::InputSpeedMeasurement;
using RTCMMsg = ::mavros_msgs::msg::RTCM;

// ROS2 Subscriber Types
using BoolSubType = ::rclcpp::Subscription<BoolMsg>::SharedPtr;
using TimeReferenceSubType = ::rclcpp::Subscription<TimeReferenceMsg>::SharedPtr;
using InputSpeedMeasurementSubType = ::rclcpp::Subscription<InputSpeedMeasurementMsg>::SharedPtr;
using RTCMSubType = rclcpp::Subscription<RTCMMsg>::SharedPtr;

// ROS2 Service Message Types
using TriggerServiceMsg = std_srvs::srv::Trigger;
using EmptyServiceMsg = std_srvs::srv::Empty;

using SetAccelBiasServiceMsg = microstrain_inertial_msgs::srv::SetAccelBias;
using GetAccelBiasServiceMsg = microstrain_inertial_msgs::srv::GetAccelBias;

using SetGyroBiasServiceMsg = microstrain_inertial_msgs::srv::SetGyroBias;
using GetGyroBiasServiceMsg = microstrain_inertial_msgs::srv::GetGyroBias;

using SetHardIronValuesServiceMsg = microstrain_inertial_msgs::srv::SetHardIronValues;
using GetHardIronValuesServiceMsg = microstrain_inertial_msgs::srv::GetHardIronValues;

using SetSoftIronMatrixServiceMsg = microstrain_inertial_msgs::srv::SetSoftIronMatrix;
using GetSoftIronMatrixServiceMsg = microstrain_inertial_msgs::srv::GetSoftIronMatrix;

using SetComplementaryFilterServiceMsg = microstrain_inertial_msgs::srv::SetComplementaryFilter;
using GetComplementaryFilterServiceMsg = microstrain_inertial_msgs::srv::GetComplementaryFilter;

using SetConingScullingCompServiceMsg = microstrain_inertial_msgs::srv::SetConingScullingComp;
using GetConingScullingCompServiceMsg = microstrain_inertial_msgs::srv::GetConingScullingComp;

using SetSensor2VehicleRotationServiceMsg = microstrain_inertial_msgs::srv::SetSensor2VehicleRotation;
using GetSensor2VehicleRotationServiceMsg = microstrain_inertial_msgs::srv::GetSensor2VehicleRotation;

using SetSensor2VehicleOffsetServiceMsg = microstrain_inertial_msgs::srv::SetSensor2VehicleOffset;
using GetSensor2VehicleOffsetServiceMsg = microstrain_inertial_msgs::srv::GetSensor2VehicleOffset;

using GetSensor2VehicleTransformationServiceMsg = microstrain_inertial_msgs::srv::GetSensor2VehicleTransformation;

using InitFilterEulerServiceMsg = microstrain_inertial_msgs::srv::InitFilterEuler;
using InitFilterHeadingServiceMsg = microstrain_inertial_msgs::srv::InitFilterHeading;

using SetHeadingSourceServiceMsg = microstrain_inertial_msgs::srv::SetHeadingSource;
using GetHeadingSourceServiceMsg = microstrain_inertial_msgs::srv::GetHeadingSource;

using SetReferencePositionServiceMsg = microstrain_inertial_msgs::srv::SetReferencePosition;
using GetReferencePositionServiceMsg = microstrain_inertial_msgs::srv::GetReferencePosition;

using SetEstimationControlFlagsServiceMsg = microstrain_inertial_msgs::srv::SetEstimationControlFlags;
using GetEstimationControlFlagsServiceMsg = microstrain_inertial_msgs::srv::GetEstimationControlFlags;

using SetDynamicsModeServiceMsg = microstrain_inertial_msgs::srv::SetDynamicsMode;
using GetDynamicsModeServiceMsg = microstrain_inertial_msgs::srv::GetDynamicsMode;

using SetZeroAngleUpdateThresholdServiceMsg = microstrain_inertial_msgs::srv::SetZeroAngleUpdateThreshold;
using GetZeroAngleUpdateThresholdServiceMsg = microstrain_inertial_msgs::srv::GetZeroAngleUpdateThreshold;

using SetZeroVelocityUpdateThresholdServiceMsg = microstrain_inertial_msgs::srv::SetZeroVelocityUpdateThreshold;
using GetZeroVelocityUpdateThresholdServiceMsg = microstrain_inertial_msgs::srv::GetZeroVelocityUpdateThreshold;

using SetTareOrientationServiceMsg = microstrain_inertial_msgs::srv::SetTareOrientation;

using SetAccelNoiseServiceMsg = microstrain_inertial_msgs::srv::SetAccelNoise;
using GetAccelNoiseServiceMsg = microstrain_inertial_msgs::srv::GetAccelNoise;

using SetGyroNoiseServiceMsg = microstrain_inertial_msgs::srv::SetGyroNoise;
using GetGyroNoiseServiceMsg = microstrain_inertial_msgs::srv::GetGyroNoise;

using SetMagNoiseServiceMsg = microstrain_inertial_msgs::srv::SetMagNoise;
using GetMagNoiseServiceMsg = microstrain_inertial_msgs::srv::GetMagNoise;

using SetGyroBiasModelServiceMsg = microstrain_inertial_msgs::srv::SetGyroBiasModel;
using GetGyroBiasModelServiceMsg = microstrain_inertial_msgs::srv::GetGyroBiasModel;

using SetAccelBiasModelServiceMsg = microstrain_inertial_msgs::srv::SetAccelBiasModel;
using GetAccelBiasModelServiceMsg = microstrain_inertial_msgs::srv::GetAccelBiasModel;

using SetGravityAdaptiveValsServiceMsg = microstrain_inertial_msgs::srv::SetGravityAdaptiveVals;
using GetGravityAdaptiveValsServiceMsg = microstrain_inertial_msgs::srv::GetGravityAdaptiveVals;

using SetMagAdaptiveValsServiceMsg = microstrain_inertial_msgs::srv::SetMagAdaptiveVals;
using GetMagAdaptiveValsServiceMsg = microstrain_inertial_msgs::srv::GetMagAdaptiveVals;

using SetMagDipAdaptiveValsServiceMsg = microstrain_inertial_msgs::srv::SetMagDipAdaptiveVals;
using GetMagDipAdaptiveValsServiceMsg = microstrain_inertial_msgs::srv::GetMagDipAdaptiveVals;

using ExternalHeadingUpdateServiceMsg = microstrain_inertial_msgs::srv::ExternalHeadingUpdate;

using SetRelativePositionReferenceServiceMsg = microstrain_inertial_msgs::srv::SetRelativePositionReference;
using GetRelativePositionReferenceServiceMsg = microstrain_inertial_msgs::srv::GetRelativePositionReference;

using DeviceReportServiceMsg = microstrain_inertial_msgs::srv::DeviceReport;
using DeviceSettingsServiceMsg = microstrain_inertial_msgs::srv::DeviceSettings;

using SetFilterSpeedLeverArmServiceMsg = microstrain_inertial_msgs::srv::SetFilterSpeedLeverArm;

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

using DeviceReportServiceType = ::rclcpp::Service<DeviceReportServiceMsg>::SharedPtr;
using DeviceSettingsServiceType = ::rclcpp::Service<DeviceSettingsServiceMsg>::SharedPtr;

using SetFilterSpeedLeverArmServiceType = ::rclcpp::Service<SetFilterSpeedLeverArmServiceMsg>::SharedPtr;

// ROS2 Logging
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

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_MICROSTRAIN_DEFS_H
