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

#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_ROS_COMPAT_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_ROS_COMPAT_H

/**
 * Common Includes
 */
#include <string>
#include <memory>
#include <vector>

/**
 * Common Defines
 */
#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

// Version of the driver
#ifndef MICROSTRAIN_DRIVER_VERSION
#define MICROSTRAIN_DRIVER_VERSION "unknown"
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
#include "microstrain_inertial_msgs/ImuOverrangeStatus.h"
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
#include "microstrain_inertial_msgs/GNSSSbasInfo.h"
#include "microstrain_inertial_msgs/GNSSRfErrorDetection.h"

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
#if MICROSTRAIN_ROLLING == 1 || MICROSTRAIN_HUMBLE == 1
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#endif

#include "microstrain_inertial_msgs/msg/status.hpp"
#include "microstrain_inertial_msgs/msg/imu_overrange_status.hpp"
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
#include "microstrain_inertial_msgs/msg/gnss_sbas_info.hpp"
#include "microstrain_inertial_msgs/msg/gnss_rf_error_detection.hpp"

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

/**
 * \brief Wrapper for a ROS1 publisher to make it look similar to a ROS2 publisher
 */
template<typename MessageType>
class RosPubType : public ::ros::Publisher
{
 public:
  using MessageSharedPtr = std::shared_ptr<MessageType>;
  using SharedPtr = std::shared_ptr<RosPubType<MessageType>>;

  explicit RosPubType(const ::ros::Publisher& rhs) : ::ros::Publisher(rhs) {}

  void on_activate() { (void)0; }
  void on_deactivate() { (void)0; }
};

/**
 * \brief Wrapper for a ROS1 service to make it look similar to a ROS2 service
 */
template<typename ServiceType>
class RosServiceType : public ::ros::ServiceServer
{
 public:
  using SharedPtr = std::shared_ptr<RosServiceType<ServiceType>>;

  explicit RosServiceType(const ::ros::ServiceServer& rhs) : ::ros::ServiceServer(rhs) {}
};

// ROS1 General Types
using RosNodeType = ::ros::NodeHandle;
using RosTimeType = ::ros::Time;
using RosTimerType = std::shared_ptr<::ros::Timer>;
using RosRateType = ::ros::Rate;
using RosHeaderType = ::std_msgs::Header;
using RosSubType = std::shared_ptr<::ros::Subscriber>;

// ROS1 Publisher Message Types
using OdometryMsg = ::nav_msgs::Odometry;
using ImuMsg = ::sensor_msgs::Imu;
using NavSatFixMsg = ::sensor_msgs::NavSatFix;
using MagneticFieldMsg = ::sensor_msgs::MagneticField;
using TimeReferenceMsg = ::sensor_msgs::TimeReference;
using NMEASentenceMsg = ::nmea_msgs::Sentence;
using StatusMsg = ::microstrain_inertial_msgs::Status;
using ImuOverrangeStatusMsg = ::microstrain_inertial_msgs::ImuOverrangeStatus;
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
using GNSSSbasInfoMsg = ::microstrain_inertial_msgs::GNSSSbasInfo;
using GNSSRfErrorDetectionMsg = ::microstrain_inertial_msgs::GNSSRfErrorDetection;

using TransformStampedMsg = ::geometry_msgs::TransformStamped;

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

// ROS1 aliases not intended to be used outside this file
using ParamIntVector = std::vector<int32_t>;

// ROS1 Logging
#define MICROSTRAIN_DEBUG(NODE, ...) ROS_DEBUG(__VA_ARGS__)
#define MICROSTRAIN_INFO(NODE, ...) ROS_INFO(__VA_ARGS__)
#define MICROSTRAIN_WARN(NODE, ...) ROS_WARN(__VA_ARGS__)
#define MICROSTRAIN_ERROR(NODE, ...) ROS_ERROR(__VA_ARGS__)
#define MICROSTRAIN_FATAL(NOE, ...) ROS_FATAL(__VA_ARGS__)

#define MICROSTRAIN_DEBUG_THROTTLE(NODE, PERIOD, ...) ROS_DEBUG_THROTTLE(PERIOD, __VA_ARGS__)

// ROS1 functions

/**
 * \brief Gets the current ROS time
 * \param node  Unused in this function as the ros time function is static
 * \return Current ROS time
 */
inline RosTimeType rosTimeNow(RosNodeType* node)
{
  return ros::Time::now();
}

/**
 * \brief Sets the time in seconds and nanoseconds to a ROS time object
 * \param time The time object to set the time on
 * \param sec Number of seconds to set on the object
 * \param nsec Number of nanoseconds to set on the object
 */
inline void setRosTime(RosTimeType* time, int32_t sec, int32_t nsec)
{
  time->sec = sec;
  time->nsec = nsec;
}

/**
 * \brief Gets the seconds from a ROS time object because the interface changed between ROS1 and ROS2
 * \param time_ref  The ros Time object to extract the seconds from
 * \return seconds from the ros time object
 */
inline int64_t getTimeRefSec(const ros::Time& time_ref)
{
  return time_ref.toSec();
}

/**
 * \brief Sets the sequence number on a ROS header. This is only useful in ROS1 as ROS2 removed the seq member
 * \param header  The header to set the sequence number on
 * \param seq  The sequence number to set on the header
 */
inline void setSeq(RosHeaderType* header, const uint32_t seq)
{
  header->seq = seq;
}

/**
 * \brief Extracts config from the ROS node.
 * \tparam ConfigType  The type to extract the config to e.g. int, std::string, double, etc.
 * \param node  The ROS node to extract the config from
 * \param param_name  The name of the config value to extract
 * \param param_val  Variable to store the extracted config value in
 * \param default_val  The default value to set param_val to if the config can't be found
 */
template <class ConfigType>
void getParam(RosNodeType* node, const std::string& param_name, ConfigType& param_val, const ConfigType& default_val)
{
  node->param<ConfigType>(param_name, param_val, default_val);
}

/**
 * \brief Creates a transform broadcaster
 * \param node The ROS node that the broadcaster will be associated with
 * \return Initialized shared pointer containing a transdorm broadcaster
 */
inline TransformBroadcasterType createTransformBroadcaster(RosNodeType* node)
{
  return std::make_shared<tf2_ros::TransformBroadcaster>();
}

/**
 * \brief Creates a ROS publisher
 * \tparam MessageType  The type of message that this publisher will publish
 * \param node  The ROS node to create the publisher on
 * \param topic  The topic that this publisher will publish on
 * \param queue_size  The size of the queue to enable in ROS
 * \return Shared Pointer containing the initialized publisher
 */
template <class MessageType>
typename RosPubType<MessageType>::SharedPtr createPublisher(RosNodeType* node, const std::string& topic,
                                                   const uint32_t queue_size)
{
  return std::make_shared<RosPubType<MessageType>>(node->template advertise<MessageType>(topic, queue_size));
}

/**
 * \brief Creates a ROS subscriber
 * \tparam MessageType  The type of message that this subscriber will listen to
 * \tparam ClassType  The type of class that the member function passed to this function will be on
 * \param node  The ROS node to create the publisher on
 * \param topic  The topic that this publisher will subscribe on
 * \param queue_size  The size of the queue to enable on ROS
 * \param fp  Function pointer to call whenever a message is received on the topic
 * \param obj  Reference to an object of type ClassType that will be passed as the this pointer to fp
 * \return Shared Pointer containing the initialized subscriber
 */
template <class MessageType, class ClassType>
std::shared_ptr<::ros::Subscriber> createSubscriber(RosNodeType* node, const std::string& topic,
                                                     const uint32_t queue_size,
                                                     void (ClassType::*fp)(const MessageType&), ClassType* obj)
{
  return std::make_shared<::ros::Subscriber>(node->template subscribe(topic.c_str(), queue_size, fp, obj));
}

/**
 * \brief Creates a ROS Service
 * \tparam MessageType  The type of message that this service will use
 * \tparam ClassType The type of class that the callback will be registered to
 * \tparam RequestType  The type of request that this function will receive
 * \tparam ResponseType  The type of response that this function will respond with
 * \param node  The ROS node to create the service on
 * \param service  The name to give the created service
 * \param srv_func  Function pointer to a function that will be called when the service receives a message
 * \param obj  Reference to an object of type ClassType that will be used to call the callback
 * \return Shared Pointer containing the initialized service
 */
template <class MessageType, class ClassType, class RequestType, class ResponseType>
typename RosServiceType<MessageType>::SharedPtr createService(RosNodeType* node, const std::string& service,
                                                     bool (ClassType::*srv_func)(RequestType&, ResponseType&),
                                                     ClassType* obj)
{
  return std::make_shared<RosServiceType<MessageType>>(
      node->template advertiseService<ClassType, RequestType, ResponseType>(service, srv_func, obj));
}

/**
 * \brief Creates and starts a ROS timer
 * \tparam ClassType  The type of class that the callback will be registered to
 * \param node  The ROS node to create the timer on
 * \param hz  Rate in hertz to execute the callback on
 * \param fp  Function pointer to execute at the specified rate
 * \param obj  Reference to an object of type Class Type that will be used to call the callback
 * \return Shard pointer containing the initialized and started timer
 */
template <class ClassType>
RosTimerType createTimer(RosNodeType* node, double hz, void (ClassType::*fp)(), ClassType* obj)
{
  return std::make_shared<::ros::Timer>(
      node->template createTimer(ros::Duration(1.0 / hz), [=](const ros::TimerEvent& event) { (obj->*fp)(); }));
}

/**
 * \brief Stops a ROS timer
 * \param timer  The timer to stop
 */
inline void stopTimer(RosTimerType timer)
{
  timer->stop();
}

/**
 * ROS2 Defines
 */
#elif MICROSTRAIN_ROS_VERSION == 2
// ROS2 Generic Types
using RosNodeType = ::rclcpp_lifecycle::LifecycleNode;
using RosTimeType = ::builtin_interfaces::msg::Time;
using RosTimerType = ::rclcpp::TimerBase::SharedPtr;
using RosRateType = ::rclcpp::Rate;
using RosHeaderType = ::std_msgs::msg::Header;

/**
 * \brief Wrapper to allow the publisher from ROS2 be compatible with ROS1
 *        This could almost be just "using", but the "MessageSharedPtr" of the base class is constant, and we need it to not be constant
 */
template<typename MessageType>
class RosPubType : public ::rclcpp_lifecycle::LifecyclePublisher<MessageType>
{
 public:
  using MessageSharedPtr = std::shared_ptr<MessageType>;

  explicit RosPubType(const ::rclcpp_lifecycle::LifecyclePublisher<MessageType>& rhs) : ::rclcpp_lifecycle::LifecyclePublisher<MessageType>(rhs) {}
};

// Alias for the service type so it can be compatible with ROS1
template<typename ServiceType>
using RosServiceType = ::rclcpp::Service<ServiceType>;

// ROS2 Publisher Message Types
using OdometryMsg = ::nav_msgs::msg::Odometry;
using ImuMsg = ::sensor_msgs::msg::Imu;
using NavSatFixMsg = ::sensor_msgs::msg::NavSatFix;
using MagneticFieldMsg = ::sensor_msgs::msg::MagneticField;
using TimeReferenceMsg = ::sensor_msgs::msg::TimeReference;
using NMEASentenceMsg = ::nmea_msgs::msg::Sentence;
using StatusMsg = ::microstrain_inertial_msgs::msg::Status;
using ImuOverrangeStatusMsg = ::microstrain_inertial_msgs::msg::ImuOverrangeStatus;
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
using GNSSSbasInfoMsg = ::microstrain_inertial_msgs::msg::GNSSSbasInfo;
using GNSSRfErrorDetectionMsg = ::microstrain_inertial_msgs::msg::GNSSRfErrorDetection;

using TransformStampedMsg = ::geometry_msgs::msg::TransformStamped;

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

// ROS2 aliases not intended to be used outside this file
using ParamIntVector = std::vector<int64_t>;

// ROS2 Logging
#define MICROSTRAIN_DEBUG(NODE, ...) RCLCPP_DEBUG(NODE->get_logger(), __VA_ARGS__)
#define MICROSTRAIN_INFO(NODE, ...) RCLCPP_INFO(NODE->get_logger(), __VA_ARGS__)
#define MICROSTRAIN_WARN(NODE, ...) RCLCPP_WARN(NODE->get_logger(), __VA_ARGS__)
#define MICROSTRAIN_ERROR(NODE, ...) RCLCPP_ERROR(NODE->get_logger(), __VA_ARGS__)
#define MICROSTRAIN_FATAL(NODE, ...) RCLCPP_FATAL(NODE->get_logger(), __VA_ARGS__)

#define MICROSTRAIN_DEBUG_THROTTLE(NODE, PERIOD, ...)                                                                  \
  RCLCPP_DEBUG_THROTTLE(NODE->get_logger(), *NODE->get_clock(), PERIOD, __VA_ARGS__)

// ROS2 functions

/**
 * \brief Gets the current ROS time
 * \param node  Unused in this function as the ros time function is static
 * \return Current ROS time
 */
inline RosTimeType rosTimeNow(RosNodeType* node)
{
  return node->get_clock()->now();
}

/**
 * \brief Sets the time in seconds and nanoseconds to a ROS time object
 * \param time The time object to set the time on
 * \param sec Number of seconds to set on the object
 * \param nsec Number of nanoseconds to set on the object
 */
inline void setRosTime(builtin_interfaces::msg::Time* time, int32_t sec, int32_t nsec)
{
  time->sec = sec;
  time->nanosec = nsec;
}

/**
 * \brief Gets the seconds from a ROS time object because the interface changed between ROS1 and ROS2
 * \param time_ref  The ros Time object to extract the seconds from
 * \return seconds from the ros time object
 */
inline int64_t getTimeRefSec(const builtin_interfaces::msg::Time& time_ref)
{
  return time_ref.sec;
}

/**
 * \brief Sets the sequence number on a ROS header. This is only useful in ROS1 as ROS2 removed the seq member
 * \param header  The header to set the sequence number on
 * \param seq  The sequence number to set on the header
 */
inline void setSeq(RosHeaderType* header, const uint32_t seq)
{
  // NOOP because seq was removed in ROS2
}

/**
 * \brief Extracts config from the ROS node.
 * \tparam ConfigType  The type to extract the config to e.g. int, std::string, double, etc.
 * \param node  The ROS node to extract the config from
 * \param param_name  The name of the config value to extract
 * \param param_val  Variable to store the extracted config value in
 * \param default_val  The default value to set param_val to if the config can't be found
 */
template <class ConfigType>
void getParam(RosNodeType* node, const std::string& param_name, ConfigType& param_val, const ConfigType& default_val)
{
  if (node->has_parameter(param_name))
  {
    node->get_parameter_or<ConfigType>(param_name, param_val, default_val);
  }
  else
  {
    param_val = node->declare_parameter<ConfigType>(param_name, default_val);
  }
}

/**
 * \brief Creates a transform broadcaster
 * \param node The ROS node that the broadcaster will be associated with
 * \return Initialized shared pointer containing a transdorm broadcaster
 */
inline TransformBroadcasterType createTransformBroadcaster(RosNodeType* node)
{
  return std::make_shared<tf2_ros::TransformBroadcaster>(node);
}

/**
 * \brief Creates a ROS publisher
 * \tparam MessageType  The type of message that this publisher will publish
 * \param node  The ROS node to create the publisher on
 * \param topic  The topic that this publisher will publish on
 * \param queue_size  The size of the queue to enable in ROS
 * \return Shared Pointer containing the initialized publisher
 */
template <class MessageType>
typename ::rclcpp_lifecycle::LifecyclePublisher<MessageType>::SharedPtr createPublisher(RosNodeType* node,
                                                                                         const std::string& topic,
                                                                                         const uint32_t qos)
{
  return node->template create_publisher<MessageType>(topic, qos);
}

/**
 * \brief Creates a ROS subscriber
 * \tparam MessageType  The type of message that this subscriber will listen to
 * \tparam ClassType  The type of class that the member function passed to this function will be on
 * \param node  The ROS node to create the publisher on
 * \param topic  The topic that this publisher will subscribe on
 * \param queue_size  The size of the queue to enable on ROS
 * \param fp  Function pointer to call whenever a message is received on the topic
 * \param obj  Reference to an object of type ClassType that will be passed as the this pointer to fp
 * \return Shared Pointer containing the initialized subscriber
 */
template <class MessageType, class ClassType>
typename ::rclcpp::Subscription<MessageType>::SharedPtr createSubscriber(RosNodeType* node, const std::string& topic,
                                                                          const uint32_t qos,
                                                                          void (ClassType::*fp)(const MessageType&),
                                                                          ClassType* obj)
{
  return node->template create_subscription<MessageType>(
      topic, qos, [obj, fp](const std::shared_ptr<MessageType> req) { (obj->*fp)(*req); });
}

/**
 * \brief Creates a ROS Service
 * \tparam MessageType  The type of message that this service will use
 * \tparam ClassType The type of class that the callback will be registered to
 * \tparam RequestType  The type of request that this function will receive
 * \tparam ResponseType  The type of response that this function will respond with
 * \param node  The ROS node to create the service on
 * \param service  The name to give the created service
 * \param srv_func  Function pointer to a function that will be called when the service receives a message
 * \param obj  Reference to an object of type ClassType that will be used to call the callback
 * \return Shared Pointer containing the initialized service
 */
template <class MessageType, class ClassType, class RequestType, class ResponseType>
typename ::rclcpp::Service<MessageType>::SharedPtr
createService(RosNodeType* node, const std::string& service, bool (ClassType::*srv_func)(RequestType&, ResponseType&),
               ClassType* obj)
{
  return node->template create_service<MessageType>(
      service, [obj, srv_func](const std::shared_ptr<RequestType> req, std::shared_ptr<ResponseType> res)
      {
        (obj->*srv_func)(*req, *res);
      }
    );  // NOLINT(whitespace/parens)  No way to avoid this
}

/**
 * \brief Creates and starts a ROS timer
 * \tparam ClassType  The type of class that the callback will be registered to
 * \param node  The ROS node to create the timer on
 * \param hz  Rate in hertz to execute the callback on
 * \param fp  Function pointer to execute at the specified rate
 * \param obj  Reference to an object of type Class Type that will be used to call the callback
 * \return Shard pointer containing the initialized and started timer
 */
template <class ClassType>
RosTimerType createTimer(RosNodeType* node, double hz, void (ClassType::*fp)(), ClassType* obj)
{
  std::chrono::milliseconds timer_interval_ms(static_cast<int>(1.0 / hz * 1000.0));
  return node->template create_wall_timer(timer_interval_ms, [=]() { (obj->*fp)(); });
}

/**
 * \brief Stops a ROS timer
 * \param timer  The timer to stop
 */
inline void stopTimer(RosTimerType timer)
{
  timer->cancel();
}

#else
#error "Unsupported ROS version. -DMICROSTRAIN_ROS_VERSION must be set to 1 or 2"
#endif

/**
 * \brief Extention of getParam. Explicitly gets float parameter, even if it was specified as an int
 * \param node  The ROS node to extract the config from
 * \param param_name  The name of the config value to extract
 * \param param_val  Variable to store the extracted config value in
 * \param default_val  The default value to set param_val to if the config can't be found
 */
inline void getParamFloat(RosNodeType* node, const std::string& param_name, float& param_val, const float default_val)
{
  // Seems like ROS should be able to figure this out, but for ROS2 at least, we need to cast ints to floats so people don't have to put decimal points
  try
  {
    getParam<float>(node, param_name, param_val, default_val);
  }
  catch (const std::exception& e)
  {
    int32_t param_val_int;
    getParam<int32_t>(node, param_name, param_val_int, static_cast<int32_t>(default_val));
    param_val = static_cast<float>(param_val_int);
  }
}

inline void getUint16ArrayParam(RosNodeType* node, const std::string& param_name, std::vector<uint16_t>& param_val, const std::vector<uint16_t>& default_val)
{
  // Get the parameter as ints since that is all ROS supports
  ParamIntVector param_val_int;
  ParamIntVector default_val_int(default_val.begin(), default_val.end());
  getParam<ParamIntVector>(node, param_name, param_val_int, default_val_int);

  // Convert the type
  param_val = std::vector<uint16_t>(param_val_int.begin(), param_val_int.end());
}


}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_ROS_COMPAT_H
