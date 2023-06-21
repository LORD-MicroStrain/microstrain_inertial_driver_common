/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Inertial Device Driver Implementation File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2020, Parker Hannifin Corp
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <algorithm>

#include "microstrain_inertial_driver_common/publishers.h"

namespace microstrain
{

QuaternionMsg nedToEcefRotation(double lat_deg, double lon_deg, double i, double j, double k, double w)
{
  double lat = lat_deg * (M_PI/180);
  double lon = lon_deg * (M_PI/180);
  const tf2::Matrix3x3 ecef_to_ned(-sin(lat) * cos(lon), -sin(lat) * sin(lon), cos(lat),
                          -sin(lon), cos(lon), 0,
                          -cos(lat) * cos(lon), -cos(lat) * sin(lon), -sin(lat));
  const tf2::Matrix3x3 ned_to_ecef = ecef_to_ned.inverse();

  const tf2::Quaternion ned_attitude(i, j, k, w);
  const tf2::Matrix3x3 ned_attitude_matrix(ned_attitude);

  const tf2::Matrix3x3 ecef_frame_attitude = ned_to_ecef * ned_attitude_matrix;
  tf2::Quaternion ecef_frame_attitude_quat;
  ecef_frame_attitude.getRotation(ecef_frame_attitude_quat);

  return tf2::toMsg(ecef_frame_attitude_quat);
}

constexpr auto USTRAIN_G =
    9.80665;  // from section 5.1.1 in
              // https://www.microstrain.com/sites/default/files/3dm-gx5-25_dcp_manual_8500-0065_reference_document.pdf


Publishers::Publishers(RosNodeType* node, Config* config)
  : node_(node), config_(config)
{
  // Initialize the transform buffer and listener ahead of time
  transform_buffer_ = createTransformBuffer(node_);
  transform_listener_ = createTransformListener(transform_buffer_);
}

bool Publishers::configure()
{
  raw_imu_pub_->configure(node_, config_);
  mag_pub_->configure(node_, config_);
  pressure_pub_->configure(node_, config_);
  imu_pub_->configure(node_, config_);

  for (const auto& pub : gnss_fix_pub_) pub->configure(node_, config_);
  for (const auto& pub : gnss_vel_pub_) pub->configure(node_, config_);
  for (const auto& pub : gnss_vel_ecef_pub_) pub->configure(node_, config_);
  for (const auto& pub : gnss_odom_pub_) pub->configure(node_, config_);
  for (const auto& pub : gnss_time_pub_) pub->configure(node_, config_);

  filter_fix_pub_->configure(node_, config_);
  filter_vel_pub_->configure(node_, config_);
  filter_vel_ecef_pub_->configure(node_, config_);
  filter_odom_pub_->configure(node_, config_);

  // Only publish relative odom if we support the relative position descriptor set
  if (config_->mip_device_->supportsDescriptor(mip::data_filter::DESCRIPTOR_SET, mip::data_filter::RelPosNed::FIELD_DESCRIPTOR))
    filter_relative_odom_pub_->configure(node_, config_);

  mip_sensor_overrange_status_pub_->configure(node_, config_);

  for (const auto& pub : mip_gnss_fix_info_pub_) pub->configure(node_, config_);
  for (const auto& pub : mip_gnss_sbas_info_pub_) pub->configure(node_, config_);
  for (const auto& pub : mip_gnss_rf_error_detection_pub_) pub->configure(node_, config_);

  mip_gnss_corrections_rtk_corrections_status_pub_->configure(node_, config_);

  mip_filter_status_pub_->configure(node_, config_);
  mip_filter_gnss_position_aiding_status_pub_->configure(node_, config_);
  mip_filter_multi_antenna_offset_correction_pub_->configure(node_, config_);
  mip_filter_aiding_measurement_summary_pub_->configure(node_, config_);
  mip_filter_gnss_dual_antenna_status_pub_->configure(node_, config_);

  if (config_->publish_nmea_)
    nmea_sentence_pub_->configure(node_);

  // Frame ID configuration
  raw_imu_pub_->getMessage()->header.frame_id = config_->frame_id_;
  mag_pub_->getMessage()->header.frame_id = config_->frame_id_;
  pressure_pub_->getMessage()->header.frame_id = config_->frame_id_;
  imu_pub_->getMessage()->header.frame_id = config_->frame_id_;

  for (int i = 0; i < gnss_fix_pub_.size(); i++) gnss_fix_pub_[i]->getMessage()->header.frame_id = config_->gnss_frame_id_[i];
  for (int i = 0; i < gnss_vel_pub_.size(); i++) gnss_vel_pub_[i]->getMessage()->header.frame_id = config_->gnss_frame_id_[i];
  for (int i = 0; i < gnss_vel_ecef_pub_.size(); i++) gnss_vel_ecef_pub_[i]->getMessage()->header.frame_id = config_->gnss_frame_id_[i];
  for (int i = 0; i < gnss_odom_pub_.size(); i++) gnss_odom_pub_[i]->getMessage()->header.frame_id = config_->earth_frame_id_;
  for (int i = 0; i < gnss_odom_pub_.size(); i++) gnss_odom_pub_[i]->getMessage()->child_frame_id = config_->gnss_frame_id_[i];
  for (int i = 0; i < gnss_time_pub_.size(); i++) gnss_time_pub_[i]->getMessage()->header.frame_id = config_->gnss_frame_id_[i];

  filter_fix_pub_->getMessage()->header.frame_id = config_->frame_id_;
  filter_vel_pub_->getMessage()->header.frame_id = config_->frame_id_;
  filter_vel_ecef_pub_->getMessage()->header.frame_id = config_->frame_id_;
  filter_odom_pub_->getMessage()->header.frame_id = config_->earth_frame_id_;
  filter_odom_pub_->getMessage()->child_frame_id = config_->frame_id_;
  filter_relative_odom_pub_->getMessage()->header.frame_id = config_->map_frame_id_;
  filter_relative_odom_pub_->getMessage()->child_frame_id = config_->frame_id_;

  // Other assorted static configuration
  auto raw_imu_msg = raw_imu_pub_->getMessage();
  std::copy(config_->imu_linear_cov_.begin(), config_->imu_linear_cov_.end(), raw_imu_msg->linear_acceleration_covariance.begin());
  std::copy(config_->imu_angular_cov_.begin(), config_->imu_angular_cov_.end(), raw_imu_msg->angular_velocity_covariance.begin());
  std::copy(config_->imu_orientation_cov_.begin(), config_->imu_orientation_cov_.end(), raw_imu_msg->orientation_covariance.begin());
  pressure_pub_->getMessage()->variance = config_->imu_pressure_vairance_;

  supports_filter_ecef_ = config_->mip_device_->supportsDescriptor(mip::data_filter::DESCRIPTOR_SET, mip::data_filter::DATA_ECEF_POS);

  // Transform broadcaster setup
  static_transform_broadcaster_ = createStaticTransformBroadcaster(node_);
  transform_broadcaster_ = createTransformBroadcaster(node_);

  // Get the relative position configuration so we can publish the transform between earth and map if we need to
  if (config_->tf_mode_ == TF_MODE_RELATIVE)
  {
    if (config_->mip_device_->supportsDescriptor(mip::commands_filter::DESCRIPTOR_SET, mip::commands_filter::CMD_REL_POS_CONFIGURATION))
    {
      mip::CmdResult mip_cmd_result;
      uint8_t rel_pos_source;
      mip::commands_filter::FilterReferenceFrame rel_pos_frame;
      double rel_pos_coordinates[3];
      if (!(mip_cmd_result = mip::commands_filter::readRelPosConfiguration(*(config_->mip_device_), &rel_pos_source, &rel_pos_frame, rel_pos_coordinates)))
      {
        MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to read relative position configuration");
        return false;
      }

      if (rel_pos_source == 0)  // Relative position will be reported relative to the base station
      {
        // Stream the base station info at 2 hertz so we can publish the earth to map transform
        MICROSTRAIN_DEBUG(node_, "Streaming RTK base station info data at 2 hertz");
        if (!(mip_cmd_result = config_->mip_device_->streamDescriptor(mip::data_gnss::MIP_GNSS3_DATA_DESC_SET, mip::data_gnss::DATA_BASE_STATION_INFO, 2)))
        {
          MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to stream RTK base station info required for earth to map transform");
          return false;
        }
        static_earth_map_transform_ = false;
      }
      else if (rel_pos_source == 1)  // Relative position will be reported relative to the returned coordinates
      {
        if (rel_pos_frame == mip::commands_filter::FilterReferenceFrame::ECEF)
        {
          earth_map_transform_msg_.transform.translation.x = rel_pos_coordinates[0];
          earth_map_transform_msg_.transform.translation.y = rel_pos_coordinates[1];
          earth_map_transform_msg_.transform.translation.z = rel_pos_coordinates[2];
        }
        else if (rel_pos_frame == mip::commands_filter::FilterReferenceFrame::LLH)
        {
          geocentric_converter_.Forward(rel_pos_coordinates[0], rel_pos_coordinates[1], rel_pos_coordinates[2],
            earth_map_transform_msg_.transform.translation.x, earth_map_transform_msg_.transform.translation.y, earth_map_transform_msg_.transform.translation.z);
        }

        // Get the lat lon and alt since that's the only way I know how to get the rotation
        double lat, lon, alt;
        geocentric_converter_.Reverse(earth_map_transform_msg_.transform.translation.x, earth_map_transform_msg_.transform.translation.y, earth_map_transform_msg_.transform.translation.z, lat, lon, alt);
        earth_map_transform_msg_.transform.rotation = nedToEcefRotation(lat, lon, 0, 0, 0, 1);

        earth_map_transform_msg_.header.stamp = rosTimeNow(node_);

        static_earth_map_transform_ = true;
      }
      else
      {
        MICROSTRAIN_ERROR(node_, "Unsupported relative position configuration %d. Unable to publish relative transform", rel_pos_source);
        return false;
      }

      // Fill in the remaining info
      earth_map_transform_msg_.header.frame_id = config_->earth_frame_id_;
      earth_map_transform_msg_.child_frame_id = config_->map_frame_id_;
    }
    else
    {
      MICROSTRAIN_ERROR(node_, "This device does not support the relative position configuration command. Unable to publish relative transforms");
      return false;
    }
  }

  // Configure the GNSS transforms
  if (config_->tf_mode_ != TF_MODE_OFF)
  {
    // Static antenna offsets
    // TODO: If streaming the antenna offset correction topic, correct the offsets with them
    mip::CmdResult mip_cmd_result;
    float gnss_antenna_offsets[3];
    if (config_->mip_device_->supportsDescriptor(mip::commands_filter::DESCRIPTOR_SET, mip::commands_filter::CMD_ANTENNA_OFFSET))
    {
      if (!(mip_cmd_result = mip::commands_filter::readAntennaOffset(*(config_->mip_device_), gnss_antenna_offsets)))
      {
        MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to read GNSS antenna offsets required for GNSS antenna transforms");
        return false;
      }
      TransformStampedMsg& gnss_antenna_transform = gnss_antenna_transform_msg_[GNSS1_ID];
      gnss_antenna_transform.header.stamp = rosTimeNow(node_);
      gnss_antenna_transform.header.frame_id = config_->frame_id_;
      gnss_antenna_transform.child_frame_id = config_->gnss_frame_id_[GNSS1_ID];
      gnss_antenna_transform.transform.translation.x = gnss_antenna_offsets[0];
      gnss_antenna_transform.transform.translation.y = gnss_antenna_offsets[1];
      gnss_antenna_transform.transform.translation.z = gnss_antenna_offsets[2];
      gnss_antenna_transform.transform.rotation.x = 0;
      gnss_antenna_transform.transform.rotation.y = 0;
      gnss_antenna_transform.transform.rotation.z = 0;
      gnss_antenna_transform.transform.rotation.w = 1;
    }
    else if (config_->mip_device_->supportsDescriptor(mip::commands_filter::DESCRIPTOR_SET, mip::commands_filter::CMD_MULTI_ANTENNA_OFFSET))
    {
      for (const uint8_t gnss_id : std::initializer_list<uint8_t>{GNSS1_ID, GNSS2_ID})
      {
        if (!(mip_cmd_result = mip::commands_filter::readMultiAntennaOffset(*(config_->mip_device_), gnss_id + 1, gnss_antenna_offsets)))
        {
          MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to read GNSS antenna offsets required for GNSS antenna transforms");
          return false;
        }
        TransformStampedMsg& gnss_antenna_transform = gnss_antenna_transform_msg_[gnss_id];
        gnss_antenna_transform.header.stamp = rosTimeNow(node_);
        gnss_antenna_transform.header.frame_id = config_->frame_id_;
        gnss_antenna_transform.child_frame_id = config_->gnss_frame_id_[gnss_id];
        gnss_antenna_transform.transform.translation.x = gnss_antenna_offsets[0];
        gnss_antenna_transform.transform.translation.y = gnss_antenna_offsets[1];
        gnss_antenna_transform.transform.translation.z = gnss_antenna_offsets[2];
        gnss_antenna_transform.transform.rotation.x = 0;
        gnss_antenna_transform.transform.rotation.y = 0;
        gnss_antenna_transform.transform.rotation.z = 0;
        gnss_antenna_transform.transform.rotation.w = 1;
      }
    }
  }

  // Register callbacks for each data field we care about. Note that order is preserved here, so if a data field needs to be parsed before another, change it here.
  // Prospect shared field callbacks
  for (const uint8_t descriptor_set : std::initializer_list<uint8_t>{mip::data_sensor::DESCRIPTOR_SET, mip::data_gnss::DESCRIPTOR_SET, mip::data_gnss::MIP_GNSS1_DATA_DESC_SET, mip::data_gnss::MIP_GNSS2_DATA_DESC_SET, mip::data_gnss::MIP_GNSS3_DATA_DESC_SET, mip::data_filter::DESCRIPTOR_SET})
  {
    registerDataCallback<mip::data_shared::EventSource, &Publishers::handleSharedEventSource>(descriptor_set);
    registerDataCallback<mip::data_shared::Ticks, &Publishers::handleSharedTicks>(descriptor_set);
    registerDataCallback<mip::data_shared::DeltaTicks, &Publishers::handleSharedDeltaTicks>(descriptor_set);
    registerDataCallback<mip::data_shared::GpsTimestamp, &Publishers::handleSharedGpsTimestamp>(descriptor_set);
    registerDataCallback<mip::data_shared::DeltaTime, &Publishers::handleSharedDeltaTime>(descriptor_set);
    registerDataCallback<mip::data_shared::ReferenceTimestamp, &Publishers::handleSharedReferenceTimestamp>(descriptor_set);
    registerDataCallback<mip::data_shared::ReferenceTimeDelta, &Publishers::handleSharedReferenceTimeDelta>(descriptor_set);
  }

  // Philo shared field callbacks
  registerDataCallback<mip::data_sensor::GpsTimestamp, &Publishers::handleSensorGpsTimestamp>();
  registerDataCallback<mip::data_gnss::GpsTime, &Publishers::handleGnssGpsTime>();
  registerDataCallback<mip::data_filter::Timestamp, &Publishers::handleFilterTimestamp>();

  // IMU callbacks
  registerDataCallback<mip::data_sensor::ScaledAccel, &Publishers::handleSensorScaledAccel>();
  registerDataCallback<mip::data_sensor::ScaledGyro, &Publishers::handleSensorScaledGyro>();
  registerDataCallback<mip::data_sensor::CompQuaternion, &Publishers::handleSensorCompQuaternion>();
  registerDataCallback<mip::data_sensor::ScaledMag, &Publishers::handleSensorScaledMag>();
  registerDataCallback<mip::data_sensor::ScaledPressure, &Publishers::handleSensorScaledPressure>();
  registerDataCallback<mip::data_sensor::OverrangeStatus, &Publishers::handleSensorOverrangeStatus>();

  // GNSS1/2 callbacks
  for (const uint8_t gnss_descriptor_set : std::initializer_list<uint8_t>{mip::data_gnss::DESCRIPTOR_SET, mip::data_gnss::MIP_GNSS1_DATA_DESC_SET, mip::data_gnss::MIP_GNSS2_DATA_DESC_SET})
  {
    registerDataCallback<mip::data_gnss::PosLlh, &Publishers::handleGnssPosLlh>(gnss_descriptor_set);
    registerDataCallback<mip::data_gnss::VelNed, &Publishers::handleGnssVelNed>(gnss_descriptor_set);
    registerDataCallback<mip::data_gnss::PosEcef, &Publishers::handleGnssPosEcef>(gnss_descriptor_set);
    registerDataCallback<mip::data_gnss::VelEcef, &Publishers::handleGnssVelEcef>(gnss_descriptor_set);
    registerDataCallback<mip::data_gnss::FixInfo, &Publishers::handleGnssFixInfo>(gnss_descriptor_set);
    registerDataCallback<mip::data_gnss::SbasInfo, &Publishers::handleGnssSbasInfo>(gnss_descriptor_set);
    registerDataCallback<mip::data_gnss::RfErrorDetection, &Publishers::handleGnssRfErrorDetection>(gnss_descriptor_set);
  }

  // Note: It is important to make sure this is after the GNSS1/2 callbacks
  for (const uint8_t gnss_descriptor_set : std::initializer_list<uint8_t>{mip::data_gnss::MIP_GNSS1_DATA_DESC_SET, mip::data_gnss::MIP_GNSS2_DATA_DESC_SET})
  {
    registerDataCallback<mip::data_gnss::GpsTime, &Publishers::handleGnssGpsTime>(gnss_descriptor_set);
  }

  // RTK callbacks
  registerDataCallback<mip::data_gnss::RtkCorrectionsStatus, &Publishers::handleRtkCorrectionsStatus>(mip::data_gnss::MIP_GNSS3_DATA_DESC_SET);
  registerDataCallback<mip::data_gnss::BaseStationInfo, &Publishers::handleRtkBaseStationInfo>(mip::data_gnss::MIP_GNSS3_DATA_DESC_SET);

  // Filter callbacks
  registerDataCallback<mip::data_filter::Status, &Publishers::handleFilterStatus>();
  registerDataCallback<mip::data_filter::EcefPos, &Publishers::handleFilterEcefPos>();
  registerDataCallback<mip::data_filter::EcefPosUncertainty, &Publishers::handleFilterEcefPosUncertainty>();
  registerDataCallback<mip::data_filter::PositionLlh, &Publishers::handleFilterPositionLlh>();
  registerDataCallback<mip::data_filter::PositionLlhUncertainty, &Publishers::handleFilterPositionLlhUncertainty>();
  registerDataCallback<mip::data_filter::AttitudeQuaternion, &Publishers::handleFilterAttitudeQuaternion>();
  registerDataCallback<mip::data_filter::EulerAnglesUncertainty, &Publishers::handleFilterEulerAnglesUncertainty>();
  registerDataCallback<mip::data_filter::VelocityNed, &Publishers::handleFilterVelocityNed>();
  registerDataCallback<mip::data_filter::VelocityNedUncertainty, &Publishers::handleFilterVelocityNedUncertainty>();
  registerDataCallback<mip::data_filter::EcefVel, &Publishers::handleFilterEcefVelocity>();
  registerDataCallback<mip::data_filter::EcefVelUncertainty, &Publishers::handleFilterEcefVelocityUncertainty>();
  registerDataCallback<mip::data_filter::CompAngularRate, &Publishers::handleFilterCompAngularRate>();
  registerDataCallback<mip::data_filter::CompAccel, &Publishers::handleFilterCompAccel>();
  registerDataCallback<mip::data_filter::LinearAccel, &Publishers::handleFilterLinearAccel>();
  registerDataCallback<mip::data_filter::RelPosNed, &Publishers::handleFilterRelPosNed>();
  registerDataCallback<mip::data_filter::GnssPosAidStatus, &Publishers::handleFilterGnssPosAidStatus>();
  registerDataCallback<mip::data_filter::MultiAntennaOffsetCorrection, &Publishers::handleFilterMultiAntennaOffsetCorrection>();
  registerDataCallback<mip::data_filter::GnssDualAntennaStatus, &Publishers::handleFilterGnssDualAntennaStatus>();
  registerDataCallback<mip::data_filter::AidingMeasurementSummary, &Publishers::handleFilterAidingMeasurementSummary>();

  // After packet callback
  registerPacketCallback<&Publishers::handleAfterPacket>();
  return true;
}

bool Publishers::activate()
{
  raw_imu_pub_->activate();
  mag_pub_->activate();
  pressure_pub_->activate();
  imu_pub_->activate();

  for (const auto& pub : gnss_fix_pub_) pub->activate();
  for (const auto& pub : gnss_vel_pub_) pub->activate();
  for (const auto& pub : gnss_vel_ecef_pub_) pub->activate();
  for (const auto& pub : gnss_odom_pub_) pub->activate();
  for (const auto& pub : gnss_time_pub_) pub->activate();

  filter_fix_pub_->activate();
  filter_vel_pub_->activate();
  filter_vel_ecef_pub_->activate();
  filter_odom_pub_->activate();
  filter_relative_odom_pub_->activate();

  mip_sensor_overrange_status_pub_->activate();

  for (const auto& pub : mip_gnss_fix_info_pub_) pub->activate();
  for (const auto& pub : mip_gnss_sbas_info_pub_) pub->activate();
  for (const auto& pub : mip_gnss_rf_error_detection_pub_) pub->activate();

  mip_gnss_corrections_rtk_corrections_status_pub_->activate();

  mip_filter_status_pub_->activate();
  mip_filter_gnss_position_aiding_status_pub_->activate();
  mip_filter_multi_antenna_offset_correction_pub_->activate();
  mip_filter_aiding_measurement_summary_pub_->activate();
  mip_filter_gnss_dual_antenna_status_pub_->activate();

  nmea_sentence_pub_->activate();

  // Publish the static transforms
  if (config_->tf_mode_ != TF_MODE_OFF)
  {
    if (config_->publish_base_link_to_frame_id_transform_)
      static_transform_broadcaster_->sendTransform(config_->base_link_to_frame_id_transform_);
    if (config_->tf_mode_ == TF_MODE_RELATIVE && static_earth_map_transform_)
      static_transform_broadcaster_->sendTransform(earth_map_transform_msg_);
    
    // Static antenna offsets
    // Note: If streaming the antenna offset correction topic, correct the offsets with them
    if (config_->mip_device_->supportsDescriptorSet(mip::data_gnss::DESCRIPTOR_SET) || config_->mip_device_->supportsDescriptorSet(mip::data_gnss::MIP_GNSS1_DATA_DESC_SET))
      static_transform_broadcaster_->sendTransform(gnss_antenna_transform_msg_[GNSS1_ID]);
    if (config_->mip_device_->supportsDescriptorSet(mip::data_gnss::MIP_GNSS2_DATA_DESC_SET))
      static_transform_broadcaster_->sendTransform(gnss_antenna_transform_msg_[GNSS2_ID]);
  }

  return true;
}

bool Publishers::deactivate()
{
  raw_imu_pub_->deactivate();
  mag_pub_->deactivate();
  pressure_pub_->deactivate();
  imu_pub_->deactivate();

  for (const auto& pub : gnss_fix_pub_) pub->deactivate();
  for (const auto& pub : gnss_vel_pub_) pub->deactivate();
  for (const auto& pub : gnss_vel_ecef_pub_) pub->deactivate();
  for (const auto& pub : gnss_odom_pub_) pub->deactivate();
  for (const auto& pub : gnss_time_pub_) pub->deactivate();

  filter_fix_pub_->deactivate();
  filter_odom_pub_->deactivate();
  filter_relative_odom_pub_->deactivate();

  mip_sensor_overrange_status_pub_->deactivate();

  for (const auto& pub : mip_gnss_fix_info_pub_) pub->deactivate();
  for (const auto& pub : mip_gnss_sbas_info_pub_) pub->deactivate();
  for (const auto& pub : mip_gnss_rf_error_detection_pub_) pub->deactivate();

  mip_gnss_corrections_rtk_corrections_status_pub_->deactivate();

  mip_filter_status_pub_->deactivate();
  mip_filter_gnss_position_aiding_status_pub_->deactivate();
  mip_filter_multi_antenna_offset_correction_pub_->deactivate();
  mip_filter_aiding_measurement_summary_pub_->deactivate();
  mip_filter_gnss_dual_antenna_status_pub_->deactivate();

  nmea_sentence_pub_->deactivate();
  return true;
}

void Publishers::publish()
{
  // This publish function will get called after each packet is processed.
  // For standard ROS messages this allows us to combine multiple MIP fields and then publish them
  // For custom ROS messages, the messages are published directly in the callbacks
  raw_imu_pub_->publish();
  mag_pub_->publish();
  pressure_pub_->publish();
  imu_pub_->publish();

  for (const auto& pub : gnss_fix_pub_) pub->publish();
  for (const auto& pub : gnss_vel_pub_) pub->publish();
  for (const auto& pub : gnss_vel_ecef_pub_) pub->publish();
  for (const auto& pub : gnss_odom_pub_) pub->publish();
  for (const auto& pub : gnss_time_pub_) pub->publish();

  filter_fix_pub_->publish();
  filter_vel_pub_->publish();
  filter_vel_ecef_pub_->publish();
  filter_odom_pub_->publish();
  filter_relative_odom_pub_->publish();

  // Publish the dynamic transforms after the messages have been filled out
  std::string tf_error_string;
  RosTimeType frame_time; setRosTime(&frame_time, 0, 0);
  if (config_->tf_mode_ == TF_MODE_GLOBAL && ecef_transform_position_updated_ && ecef_transform_attitude_updated_)
  {
    if (transform_buffer_->canTransform(config_->frame_id_, config_->target_frame_id_, frame_time, RosDurationType(0, 0), &tf_error_string))
    {
      TransformStampedMsg earth_target_transform, earth_imu_link_transform;
      const auto& target_imu_link_transform = transform_buffer_->lookupTransform(config_->frame_id_, config_->target_frame_id_, frame_time, RosDurationType(0, 0));

      const auto& filter_odom_msg = filter_odom_pub_->getMessage();
      earth_imu_link_transform.transform.translation.x = filter_odom_msg->pose.pose.position.x;
      earth_imu_link_transform.transform.translation.y = filter_odom_msg->pose.pose.position.y;
      earth_imu_link_transform.transform.translation.z = filter_odom_msg->pose.pose.position.z;
      earth_imu_link_transform.transform.rotation = filter_odom_msg->pose.pose.orientation;

      tf2::doTransform(earth_imu_link_transform, earth_target_transform, target_imu_link_transform);

      // Handle the rotational transform between ECEF and NED
      const auto filter_fix_msg = filter_fix_pub_->getMessage();
      earth_target_transform.transform.rotation = nedToEcefRotation(
        filter_fix_msg->latitude, filter_fix_msg->longitude,
        earth_target_transform.transform.rotation.x,
        earth_target_transform.transform.rotation.y,
        earth_target_transform.transform.rotation.z,
        earth_target_transform.transform.rotation.w
      );

      earth_target_transform.header.stamp = filter_odom_msg->header.stamp;      
      earth_target_transform.header.frame_id = config_->earth_frame_id_;
      earth_target_transform.child_frame_id = config_->target_frame_id_;

      // Publish and reset the booleans
      transform_broadcaster_->sendTransform(earth_target_transform);
      ecef_transform_position_updated_ = false;
      ecef_transform_attitude_updated_ = false;
    }
    else
    {
      MICROSTRAIN_WARN_THROTTLE(node_, 2, "Unable to lookup transform from %s to %s: %s", config_->target_frame_id_.c_str(), config_->frame_id_.c_str(), tf_error_string.c_str());
    }
  }
  else if (config_->tf_mode_ == TF_MODE_RELATIVE && relative_transform_position_updated_ && relative_transform_attitude_updated_)
  {
    if (transform_buffer_->canTransform(config_->frame_id_, config_->target_frame_id_, frame_time, RosDurationType(0, 0), &tf_error_string))
    {
      TransformStampedMsg map_target_transform, map_imu_link_transform;
      const auto& target_imu_link_transform = transform_buffer_->lookupTransform(config_->frame_id_, config_->target_frame_id_, frame_time, RosDurationType(0, 0));

      const auto& filter_relative_odom_msg = filter_relative_odom_pub_->getMessage();
      map_imu_link_transform.transform.translation.x = filter_relative_odom_msg->pose.pose.position.x;
      map_imu_link_transform.transform.translation.y = filter_relative_odom_msg->pose.pose.position.y;
      map_imu_link_transform.transform.translation.z = filter_relative_odom_msg->pose.pose.position.z;
      map_imu_link_transform.transform.rotation = filter_relative_odom_msg->pose.pose.orientation;

      tf2::doTransform(map_imu_link_transform, map_target_transform, target_imu_link_transform);

      map_target_transform.header.stamp = filter_relative_odom_msg->header.stamp;      
      map_target_transform.header.frame_id = config_->map_frame_id_;
      map_target_transform.child_frame_id = config_->target_frame_id_;

      transform_broadcaster_->sendTransform(map_target_transform);
      relative_transform_position_updated_ = false;
      relative_transform_attitude_updated_ = false;
    }
    else
    {
      MICROSTRAIN_WARN_THROTTLE(node_, 2, "Unable to lookup transform from %s to %s: %s", config_->target_frame_id_.c_str(), config_->frame_id_.c_str(), tf_error_string.c_str());
    }
  }

  if (config_->tf_mode_ != TF_MODE_OFF)
  {
    // Only publish the earth to map transform now if it is not static
    if (!static_earth_map_transform_ && earth_map_transform_updated_)
      transform_broadcaster_->sendTransform(earth_map_transform_msg_);
  }
}

void Publishers::handleSharedEventSource(const mip::data_shared::EventSource& event_source, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  event_source_mapping_[descriptor_set] = event_source;
}

void Publishers::handleSharedTicks(const mip::data_shared::Ticks& ticks, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  ticks_mapping_[descriptor_set] = ticks;
}

void Publishers::handleSharedDeltaTicks(const mip::data_shared::DeltaTicks& delta_ticks, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  delta_ticks_mapping_[descriptor_set] = delta_ticks;
}

void Publishers::handleSharedGpsTimestamp(const mip::data_shared::GpsTimestamp& gps_timestamp, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Save the GPS timestamp
  gps_timestamp_mapping_[descriptor_set] = gps_timestamp;

  // Update the GPS time message for this descriptor
  uint8_t gnss_index;
  switch (descriptor_set)
  {
    case mip::data_gnss::MIP_GNSS1_DATA_DESC_SET:
      gnss_index = 0;
      break;
    case mip::data_gnss::MIP_GNSS2_DATA_DESC_SET:
      gnss_index = 1;
      break;
    default:
      return;
  }
  auto gps_time_msg = gnss_time_pub_[gnss_index]->getMessageToUpdate();
  gps_time_msg->header.stamp = rosTimeNow(node_);
  setGpsTime(&gps_time_msg->time_ref, gps_timestamp);
}

void Publishers::handleSharedDeltaTime(const mip::data_shared::DeltaTime& delta_time, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  delta_time_mapping_[descriptor_set] = delta_time;
}

void Publishers::handleSharedReferenceTimestamp(const mip::data_shared::ReferenceTimestamp& reference_timestamp, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  reference_timestamp_mapping_[descriptor_set] = reference_timestamp;
}

void Publishers::handleSharedReferenceTimeDelta(const mip::data_shared::ReferenceTimeDelta& reference_time_delta, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  reference_time_delta_mapping_[descriptor_set] = reference_time_delta;
}

void Publishers::handleSensorGpsTimestamp(const mip::data_sensor::GpsTimestamp& gps_timestamp, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Convert the old philo timestamp into the new format and store it in the map
  mip::data_shared::GpsTimestamp stored_timestamp;
  stored_timestamp.tow = gps_timestamp.tow;
  stored_timestamp.week_number = gps_timestamp.week_number;
  stored_timestamp.valid_flags = gps_timestamp.valid_flags;
  gps_timestamp_mapping_[descriptor_set] = stored_timestamp;
}

void Publishers::handleSensorScaledAccel(const mip::data_sensor::ScaledAccel& scaled_accel, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto raw_imu_msg = raw_imu_pub_->getMessageToUpdate();
  updateHeaderTime(&(raw_imu_msg->header), descriptor_set, timestamp);
  raw_imu_msg->linear_acceleration.x = USTRAIN_G * scaled_accel.scaled_accel[0];
  raw_imu_msg->linear_acceleration.y = USTRAIN_G * scaled_accel.scaled_accel[1];
  raw_imu_msg->linear_acceleration.z = USTRAIN_G * scaled_accel.scaled_accel[2];
  if (config_->use_enu_frame_)
  {
    raw_imu_msg->linear_acceleration.y *= -1.0;
    raw_imu_msg->linear_acceleration.z *= -1.0;
  }
}

void Publishers::handleSensorScaledGyro(const mip::data_sensor::ScaledGyro& scaled_gyro, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto raw_imu_msg = raw_imu_pub_->getMessageToUpdate();
  updateHeaderTime(&(raw_imu_msg->header), descriptor_set, timestamp);
  raw_imu_msg->angular_velocity.x = scaled_gyro.scaled_gyro[0];
  raw_imu_msg->angular_velocity.y = scaled_gyro.scaled_gyro[1];
  raw_imu_msg->angular_velocity.z = scaled_gyro.scaled_gyro[2];
  if (config_->use_enu_frame_)
  {
    raw_imu_msg->angular_velocity.y *= -1.0;
    raw_imu_msg->angular_velocity.z *= -1.0;
  }
}

void Publishers::handleSensorCompQuaternion(const mip::data_sensor::CompQuaternion& comp_quaternion, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto raw_imu_msg = raw_imu_pub_->getMessageToUpdate();
  updateHeaderTime(&(raw_imu_msg->header), descriptor_set, timestamp);
  if (config_->use_enu_frame_)
  {
    tf2::Quaternion q_ned2enu, q_body2enu, q_vehiclebody2sensorbody, q_body2ned(comp_quaternion.q[1], comp_quaternion.q[2], comp_quaternion.q[3], comp_quaternion.q[0]);

    config_->t_ned2enu_.getRotation(q_ned2enu);
    config_->t_vehiclebody2sensorbody_.getRotation(q_vehiclebody2sensorbody);

    q_body2enu = q_ned2enu * q_body2ned * q_vehiclebody2sensorbody;

    raw_imu_msg->orientation = tf2::toMsg(q_body2enu);
  }
  else
  {
    raw_imu_msg->orientation.x = comp_quaternion.q[1];
    raw_imu_msg->orientation.y = comp_quaternion.q[2];
    raw_imu_msg->orientation.z = comp_quaternion.q[3];
    raw_imu_msg->orientation.w = comp_quaternion.q[0];
  }
}

void Publishers::handleSensorScaledMag(const mip::data_sensor::ScaledMag& scaled_mag, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto mag_msg = mag_pub_->getMessageToUpdate();
  updateHeaderTime(&(mag_msg->header), descriptor_set, timestamp);
  mag_msg->magnetic_field.x = scaled_mag.scaled_mag[0];
  mag_msg->magnetic_field.y = scaled_mag.scaled_mag[1];
  mag_msg->magnetic_field.z = scaled_mag.scaled_mag[2];
  if (config_->use_enu_frame_)
  {
    mag_msg->magnetic_field.y *= -1.0;
    mag_msg->magnetic_field.z *= -1.0;
  }
}

void Publishers::handleSensorScaledPressure(const mip::data_sensor::ScaledPressure& scaled_pressure, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto pressure_msg = pressure_pub_->getMessageToUpdate();
  updateHeaderTime(&(pressure_msg->header), descriptor_set, timestamp);
  pressure_msg->fluid_pressure = scaled_pressure.scaled_pressure * 100;
}

void Publishers::handleSensorOverrangeStatus(const mip::data_sensor::OverrangeStatus& overrange_status, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto mip_sensor_overrange_status_msg = mip_sensor_overrange_status_pub_->getMessage();
  updateMicrostrainHeader(&(mip_sensor_overrange_status_msg->header), descriptor_set);
  mip_sensor_overrange_status_msg->status_accel_x = overrange_status.status.accelX();
  mip_sensor_overrange_status_msg->status_accel_y = overrange_status.status.accelY();
  mip_sensor_overrange_status_msg->status_accel_z = overrange_status.status.accelZ();
  mip_sensor_overrange_status_msg->status_gyro_x = overrange_status.status.gyroX();
  mip_sensor_overrange_status_msg->status_gyro_y = overrange_status.status.gyroY();
  mip_sensor_overrange_status_msg->status_gyro_z = overrange_status.status.gyroZ();
  mip_sensor_overrange_status_msg->status_mag_x = overrange_status.status.magX();
  mip_sensor_overrange_status_msg->status_mag_y = overrange_status.status.magY();
  mip_sensor_overrange_status_msg->status_mag_z = overrange_status.status.magZ();
  mip_sensor_overrange_status_msg->status_press = overrange_status.status.press();
  mip_sensor_overrange_status_pub_->publish(*mip_sensor_overrange_status_msg);
}

void Publishers::handleGnssGpsTime(const mip::data_gnss::GpsTime& gps_time, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Convert the old philo timestamp into the new format and store it in the map
  mip::data_shared::GpsTimestamp stored_timestamp;
  stored_timestamp.tow = gps_time.tow;
  stored_timestamp.week_number = gps_time.week_number;
  stored_timestamp.valid_flags = gps_time.valid_flags;
  gps_timestamp_mapping_[descriptor_set] = stored_timestamp;

  // Also update the time ref messages
  uint8_t gnss_index;
  switch (descriptor_set)
  {
    case mip::data_gnss::DESCRIPTOR_SET:
    case mip::data_gnss::MIP_GNSS1_DATA_DESC_SET:
      gnss_index = 0;
      break;
    case mip::data_gnss::MIP_GNSS2_DATA_DESC_SET:
      gnss_index = 1;
      break;
    default:
      return;
  }
  auto gps_time_msg = gnss_time_pub_[gnss_index]->getMessageToUpdate();
  gps_time_msg->header.stamp = rosTimeNow(node_);
  setGpsTime(&gps_time_msg->time_ref, stored_timestamp);
}

void Publishers::handleGnssPosLlh(const mip::data_gnss::PosLlh& pos_llh, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Different message depending on the descriptor set
  const uint8_t gnss_index = (descriptor_set == mip::data_gnss::DESCRIPTOR_SET || descriptor_set == mip::data_gnss::MIP_GNSS1_DATA_DESC_SET) ? GNSS1_ID : GNSS2_ID;

  // GNSS navsatfix message
  auto gnss_fix_msg = gnss_fix_pub_[gnss_index]->getMessageToUpdate();
  updateHeaderTime(&(gnss_fix_msg->header), descriptor_set, timestamp);
  gnss_fix_msg->status.service = 1;
  gnss_fix_msg->position_covariance_type = 2;
  gnss_fix_msg->latitude = pos_llh.latitude;
  gnss_fix_msg->longitude = pos_llh.longitude;
  gnss_fix_msg->altitude = pos_llh.ellipsoid_height;
  gnss_fix_msg->position_covariance[0] = pos_llh.horizontal_accuracy;
  gnss_fix_msg->position_covariance[4] = pos_llh.horizontal_accuracy;
  gnss_fix_msg->position_covariance[8] = pos_llh.vertical_accuracy;
}

void Publishers::handleGnssVelNed(const mip::data_gnss::VelNed& vel_ned, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Different message depending on the descriptor set
  const uint8_t gnss_index = (descriptor_set == mip::data_gnss::DESCRIPTOR_SET || descriptor_set == mip::data_gnss::MIP_GNSS1_DATA_DESC_SET) ? GNSS1_ID : GNSS2_ID;

  // GNSS velocity message
  auto gnss_vel_msg = gnss_vel_pub_[gnss_index]->getMessageToUpdate();
  updateHeaderTime(&(gnss_vel_msg->header), descriptor_set, timestamp);
  if (config_->use_enu_frame_)
  {
    gnss_vel_msg->twist.twist.linear.x = vel_ned.v[1];
    gnss_vel_msg->twist.twist.linear.y = vel_ned.v[0];
    gnss_vel_msg->twist.twist.linear.z = -vel_ned.v[2];
  }
  else
  {
    gnss_vel_msg->twist.twist.linear.x = vel_ned.v[0];
    gnss_vel_msg->twist.twist.linear.y = vel_ned.v[1];
    gnss_vel_msg->twist.twist.linear.z = vel_ned.v[2];
  }
  gnss_vel_msg->twist.covariance[0] = vel_ned.speed_accuracy;
  gnss_vel_msg->twist.covariance[7] = vel_ned.speed_accuracy;
  gnss_vel_msg->twist.covariance[14] = vel_ned.speed_accuracy;

  // GNSS odometry message (not counted as updating)
  auto gnss_odom_msg = gnss_odom_pub_[gnss_index]->getMessage();
  tf2::Quaternion gnss_course_over_ground;
  gnss_course_over_ground.setRPY(0, 0, vel_ned.heading);
  gnss_odom_msg->pose.pose.orientation = tf2::toMsg(gnss_course_over_ground);
  gnss_odom_msg->pose.covariance[35] = vel_ned.heading_accuracy;

  // Use the orientation to rotate the velocity into the antenna's frame
  const tf2::Vector3 gnss_current_vel(gnss_vel_msg->twist.twist.linear.x, gnss_vel_msg->twist.twist.linear.y, gnss_vel_msg->twist.twist.linear.z);
  const tf2::Vector3 gnss_rotated_vel = tf2::quatRotate(gnss_course_over_ground.inverse(), gnss_current_vel);
  gnss_odom_msg->twist.twist.linear.x = gnss_rotated_vel.getX();
  gnss_odom_msg->twist.twist.linear.y = gnss_rotated_vel.getY();
  gnss_odom_msg->twist.twist.linear.z = gnss_rotated_vel.getZ();
  gnss_odom_msg->twist.covariance[0] = vel_ned.speed_accuracy;
  gnss_odom_msg->twist.covariance[7] = vel_ned.speed_accuracy;
  gnss_odom_msg->twist.covariance[14] = vel_ned.speed_accuracy;
}

void Publishers::handleGnssPosEcef(const mip::data_gnss::PosEcef& pos_ecef, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Different message depending on the descriptor set
  const uint8_t gnss_index = (descriptor_set == mip::data_gnss::DESCRIPTOR_SET || descriptor_set == mip::data_gnss::MIP_GNSS1_DATA_DESC_SET) ? GNSS1_ID : GNSS2_ID;
  auto gnss_odom_msg = gnss_odom_pub_[gnss_index]->getMessageToUpdate();
  updateHeaderTime(&(gnss_odom_msg->header), descriptor_set, timestamp);
  gnss_odom_msg->pose.pose.position.x = pos_ecef.x[0];
  gnss_odom_msg->pose.pose.position.y = pos_ecef.x[1];
  gnss_odom_msg->pose.pose.position.z = pos_ecef.x[2];
  gnss_odom_msg->pose.covariance[0] = pos_ecef.x_accuracy;
  gnss_odom_msg->pose.covariance[7] = pos_ecef.x_accuracy;
  gnss_odom_msg->pose.covariance[14] = pos_ecef.x_accuracy;
}

void Publishers::handleGnssVelEcef(const mip::data_gnss::VelEcef& vel_ecef, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Different message depending on the descriptor set
  const uint8_t gnss_index = (descriptor_set == mip::data_gnss::DESCRIPTOR_SET || descriptor_set == mip::data_gnss::MIP_GNSS1_DATA_DESC_SET) ? GNSS1_ID : GNSS2_ID;
  auto gnss_vel_ecef_msg = gnss_vel_ecef_pub_[gnss_index]->getMessageToUpdate();
  updateHeaderTime(&(gnss_vel_ecef_msg->header), descriptor_set, timestamp);
  gnss_vel_ecef_msg->twist.twist.linear.x = vel_ecef.v[0];
  gnss_vel_ecef_msg->twist.twist.linear.y = vel_ecef.v[1];
  gnss_vel_ecef_msg->twist.twist.linear.z = vel_ecef.v[2];
  gnss_vel_ecef_msg->twist.covariance[0] = vel_ecef.v_accuracy;
  gnss_vel_ecef_msg->twist.covariance[7] = vel_ecef.v_accuracy;
  gnss_vel_ecef_msg->twist.covariance[14] = vel_ecef.v_accuracy;
}

void Publishers::handleGnssFixInfo(const mip::data_gnss::FixInfo& fix_info, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Different message depending on the descriptor set
  const uint8_t gnss_index = (descriptor_set == mip::data_gnss::DESCRIPTOR_SET || descriptor_set == mip::data_gnss::MIP_GNSS1_DATA_DESC_SET) ? GNSS1_ID : GNSS2_ID;

  // GNSS Fix info message
  auto mip_gnss_fix_info_msg = mip_gnss_fix_info_pub_[gnss_index]->getMessage();
  updateMicrostrainHeader(&(mip_gnss_fix_info_msg->header), descriptor_set);
  mip_gnss_fix_info_msg->fix_type = static_cast<uint8_t>(fix_info.fix_type);
  mip_gnss_fix_info_msg->num_sv = fix_info.num_sv;
  mip_gnss_fix_info_msg->sbas_used = fix_info.fix_flags & mip::data_gnss::FixInfo::FixFlags::SBAS_USED;
  mip_gnss_fix_info_msg->dngss_used = fix_info.fix_flags & mip::data_gnss::FixInfo::FixFlags::DGNSS_USED;
  mip_gnss_fix_info_pub_[gnss_index]->publish(*mip_gnss_fix_info_msg);

  // GNSS fix message (not counted as updating)
  auto gnss_fix_msg = gnss_fix_pub_[gnss_index]->getMessage();
  if (fix_info.fix_type == mip::data_gnss::FixInfo::FixType::FIX_RTK_FIXED || fix_info.fix_type == mip::data_gnss::FixInfo::FixType::FIX_RTK_FLOAT)
    gnss_fix_msg->status.status = NavSatFixMsg::_status_type::STATUS_GBAS_FIX;
  else if (fix_info.fix_flags.sbasUsed())
    gnss_fix_msg->status.status = NavSatFixMsg::_status_type::STATUS_SBAS_FIX;
  else if (fix_info.fix_type == mip::data_gnss::FixInfo::FixType::FIX_3D)
    gnss_fix_msg->status.status = NavSatFixMsg::_status_type::STATUS_FIX;
  else
    gnss_fix_msg->status.status = NavSatFixMsg::_status_type::STATUS_NO_FIX;
}

void Publishers::handleGnssRfErrorDetection(const mip::data_gnss::RfErrorDetection& rf_error_detection, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Find the right index for the message
  uint8_t gnss_index;
  switch (descriptor_set)
  {
    case mip::data_gnss::MIP_GNSS1_DATA_DESC_SET:
      gnss_index = 0;
      break;
    case mip::data_gnss::MIP_GNSS2_DATA_DESC_SET:
      gnss_index = 1;
      break;
    default:
      return;  // Nothing to do if the descriptor set is not something we recognize
  }

  // Different message depending on the descriptor set
  auto mip_gnss_rf_error_detection_msg = mip_gnss_rf_error_detection_pub_[gnss_index]->getMessage();
  updateMicrostrainHeader(&(mip_gnss_rf_error_detection_msg->header), descriptor_set);
  mip_gnss_rf_error_detection_msg->rf_band = static_cast<uint8_t>(rf_error_detection.rf_band);
  mip_gnss_rf_error_detection_msg->jamming_state = static_cast<uint8_t>(rf_error_detection.jamming_state);
  mip_gnss_rf_error_detection_msg->spoofing_state = static_cast<uint8_t>(rf_error_detection.spoofing_state);
  mip_gnss_rf_error_detection_pub_[gnss_index]->publish(*mip_gnss_rf_error_detection_msg);
}

void Publishers::handleGnssSbasInfo(const mip::data_gnss::SbasInfo& sbas_info, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Find the right index for the message
  uint8_t gnss_index;
  switch (descriptor_set)
  {
    case mip::data_gnss::MIP_GNSS1_DATA_DESC_SET:
      gnss_index = 0;
      break;
    case mip::data_gnss::MIP_GNSS2_DATA_DESC_SET:
      gnss_index = 1;
      break;
    default:
      return;  // Nothing to do if the descriptor set is not something we recognize
  }

  // Different message depending on descriptor
  auto mip_gnss_sbas_info_msg = mip_gnss_sbas_info_pub_[gnss_index]->getMessage();
  updateMicrostrainHeader(&(mip_gnss_sbas_info_msg->header), descriptor_set);
  mip_gnss_sbas_info_msg->time_of_week = sbas_info.time_of_week;
  mip_gnss_sbas_info_msg->week_number = sbas_info.week_number;
  mip_gnss_sbas_info_msg->sbas_system = static_cast<uint8_t>(sbas_info.sbas_system);
  mip_gnss_sbas_info_msg->sbas_id = sbas_info.sbas_id;
  mip_gnss_sbas_info_msg->count = sbas_info.count;
  mip_gnss_sbas_info_msg->status_range_available = sbas_info.sbas_status.rangeAvailable();
  mip_gnss_sbas_info_msg->status_corrections_available = sbas_info.sbas_status.correctionsAvailable();
  mip_gnss_sbas_info_msg->status_integrity_available = sbas_info.sbas_status.integrityAvailable();
  mip_gnss_sbas_info_msg->status_test_mode = sbas_info.sbas_status.testMode();
  mip_gnss_sbas_info_pub_[gnss_index]->publish(*mip_gnss_sbas_info_msg);
}

void Publishers::handleRtkCorrectionsStatus(const mip::data_gnss::RtkCorrectionsStatus& rtk_corrections_status, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  const mip::commands_rtk::GetStatusFlags::StatusFlags dongle_status(rtk_corrections_status.dongle_status);
  switch (dongle_status.version())
  {
    // V1 dongle
    case 0:
    {
      // RTK Dongle v1 not supported
      return;
    }
    // V2 dongle
    default:
    {
      auto mip_gnss_corrections_rtk_corrections_status_msg = mip_gnss_corrections_rtk_corrections_status_pub_->getMessage();
      updateMicrostrainHeader(&(mip_gnss_corrections_rtk_corrections_status_msg->header), descriptor_set);
      mip_gnss_corrections_rtk_corrections_status_msg->time_of_week = rtk_corrections_status.time_of_week;
      mip_gnss_corrections_rtk_corrections_status_msg->week_number = rtk_corrections_status.week_number;

      mip_gnss_corrections_rtk_corrections_status_msg->epoch_status_antenna_location_received = rtk_corrections_status.epoch_status.antennaLocationReceived();
      mip_gnss_corrections_rtk_corrections_status_msg->epoch_status_antenna_description_received = rtk_corrections_status.epoch_status.antennaDescriptionReceived();
      mip_gnss_corrections_rtk_corrections_status_msg->epoch_status_gps_received = rtk_corrections_status.epoch_status.gpsReceived();
      mip_gnss_corrections_rtk_corrections_status_msg->epoch_status_glonass_received = rtk_corrections_status.epoch_status.glonassReceived();
      mip_gnss_corrections_rtk_corrections_status_msg->epoch_status_dongle_status_read_failed = rtk_corrections_status.epoch_status.dongleStatusReadFailed();

      mip_gnss_corrections_rtk_corrections_status_msg->dongle_status_modem_state = dongle_status.modemState();
      mip_gnss_corrections_rtk_corrections_status_msg->dongle_status_connection_type = dongle_status.connectionType();
      mip_gnss_corrections_rtk_corrections_status_msg->dongle_status_rssi = -1 * dongle_status.rssi();
      mip_gnss_corrections_rtk_corrections_status_msg->dongle_status_signal_quality = dongle_status.signalQuality();
      mip_gnss_corrections_rtk_corrections_status_msg->dongle_status_tower_change_indicator = dongle_status.towerChangeIndicator();
      mip_gnss_corrections_rtk_corrections_status_msg->dongle_status_nmea_timeout_flag = dongle_status.nmeaTimeout();
      mip_gnss_corrections_rtk_corrections_status_msg->dongle_status_server_timeout_flag = dongle_status.serverTimeout();
      mip_gnss_corrections_rtk_corrections_status_msg->dongle_status_rtcm_timeout_flag = dongle_status.correctionsTimeout();
      mip_gnss_corrections_rtk_corrections_status_msg->dongle_status_device_out_of_range_flag = dongle_status.deviceOutOfRange();
      mip_gnss_corrections_rtk_corrections_status_msg->dongle_status_corrections_unavailable_flag = dongle_status.correctionsUnavailable();

      mip_gnss_corrections_rtk_corrections_status_msg->gps_correction_latency = rtk_corrections_status.gps_correction_latency;
      mip_gnss_corrections_rtk_corrections_status_msg->glonass_correction_latency = rtk_corrections_status.glonass_correction_latency;
      mip_gnss_corrections_rtk_corrections_status_msg->galileo_correction_latency = rtk_corrections_status.galileo_correction_latency;
      mip_gnss_corrections_rtk_corrections_status_msg->beidou_correction_latency = rtk_corrections_status.beidou_correction_latency;
      mip_gnss_corrections_rtk_corrections_status_pub_->publish(*mip_gnss_corrections_rtk_corrections_status_msg);
      break;
    }
  }
}

void Publishers::handleRtkBaseStationInfo(const mip::data_gnss::BaseStationInfo& base_station_info, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Update the earth to map transform
  updateHeaderTime(&(earth_map_transform_msg_.header), descriptor_set, timestamp);
  earth_map_transform_msg_.transform.translation.x = base_station_info.ecef_pos[0];
  earth_map_transform_msg_.transform.translation.y = base_station_info.ecef_pos[1];
  earth_map_transform_msg_.transform.translation.z = base_station_info.ecef_pos[2];

  // Get the lat lon and alt since that's the only way I know how to get the rotation
  double lat, lon, alt;
  geocentric_converter_.Reverse(earth_map_transform_msg_.transform.translation.x, earth_map_transform_msg_.transform.translation.y, earth_map_transform_msg_.transform.translation.z, lat, lon, alt);
  earth_map_transform_msg_.transform.rotation = nedToEcefRotation(lat, lon, 0, 0, 0, 1);

  earth_map_transform_updated_ = true;
}

void Publishers::handleFilterTimestamp(const mip::data_filter::Timestamp& filter_timestamp, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Convert the old philo timestamp into the new format and store it in the map
  mip::data_shared::GpsTimestamp stored_timestamp;
  stored_timestamp.tow = filter_timestamp.tow;
  stored_timestamp.week_number = filter_timestamp.week_number;
  stored_timestamp.valid_flags = filter_timestamp.valid_flags;
  gps_timestamp_mapping_[descriptor_set] = stored_timestamp;
}

void Publishers::handleFilterStatus(const mip::data_filter::Status& status, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto mip_filter_status_msg = mip_filter_status_pub_->getMessage();
  updateMicrostrainHeader(&(mip_filter_status_msg->header), descriptor_set);
  mip_filter_status_msg->filter_state = static_cast<uint16_t>(status.filter_state);
  mip_filter_status_msg->dynamics_mode = static_cast<uint16_t>(status.dynamics_mode);

  // Populate both the philo and prospect flags, it is up to the customer to determine which device they have
  mip_filter_status_msg->status_flags_gx5_init_no_attitude = status.status_flags.gx5InitNoAttitude();
  mip_filter_status_msg->status_flags_gx5_init_no_position_velocity = status.status_flags.gx5InitNoPositionVelocity();
  mip_filter_status_msg->status_flags_gx5_run_imu_unavailable = status.status_flags.gx5RunImuUnavailable();
  mip_filter_status_msg->status_flags_gx5_run_gps_unavailable = status.status_flags.gx5RunGpsUnavailable();
  mip_filter_status_msg->status_flags_gx5_run_matrix_singularity = status.status_flags.gx5RunMatrixSingularity();
  mip_filter_status_msg->status_flags_gx5_run_position_covariance_warning = status.status_flags.gx5RunPositionCovarianceWarning();
  mip_filter_status_msg->status_flags_gx5_run_velocity_covariance_warning = status.status_flags.gx5RunVelocityCovarianceWarning();
  mip_filter_status_msg->status_flags_gx5_run_attitude_covariance_warning = status.status_flags.gx5RunAttitudeCovarianceWarning();
  mip_filter_status_msg->status_flags_gx5_run_nan_in_solution_warning = status.status_flags.gx5RunNanInSolutionWarning();
  mip_filter_status_msg->status_flags_gx5_run_gyro_bias_est_high_warning = status.status_flags.gx5RunGyroBiasEstHighWarning();
  mip_filter_status_msg->status_flags_gx5_run_accel_bias_est_high_warning = status.status_flags.gx5RunAccelBiasEstHighWarning();
  mip_filter_status_msg->status_flags_gx5_run_gyro_scale_factor_est_high_warning = status.status_flags.gx5RunGyroScaleFactorEstHighWarning();
  mip_filter_status_msg->status_flags_gx5_run_accel_scale_factor_est_high_warning = status.status_flags.gx5RunAccelScaleFactorEstHighWarning();
  mip_filter_status_msg->status_flags_gx5_run_mag_bias_est_high_warning = status.status_flags.gx5RunMagBiasEstHighWarning();
  mip_filter_status_msg->status_flags_gx5_run_ant_offset_correction_est_high_warning = status.status_flags.gx5RunAntOffsetCorrectionEstHighWarning();
  mip_filter_status_msg->status_flags_gx5_run_mag_hard_iron_est_high_warning = status.status_flags.gx5RunMagHardIronEstHighWarning();
  mip_filter_status_msg->status_flags_gx5_run_mag_soft_iron_est_high_warning = status.status_flags.gx5RunMagSoftIronEstHighWarning();

  mip_filter_status_msg->status_flags_gq7_filter_condition = status.status_flags.gq7FilterCondition();
  mip_filter_status_msg->status_flags_gq7_roll_pitch_warning = status.status_flags.gq7RollPitchWarning();
  mip_filter_status_msg->status_flags_gq7_heading_warning = status.status_flags.gq7HeadingWarning();
  mip_filter_status_msg->status_flags_gq7_position_warning = status.status_flags.gq7PositionWarning();
  mip_filter_status_msg->status_flags_gq7_velocity_warning = status.status_flags.gq7VelocityWarning();
  mip_filter_status_msg->status_flags_gq7_imu_bias_warning = status.status_flags.gq7ImuBiasWarning();
  mip_filter_status_msg->status_flags_gq7_gnss_clk_warning = status.status_flags.gq7GnssClkWarning();
  mip_filter_status_msg->status_flags_gq7_antenna_lever_arm_warning = status.status_flags.gq7AntennaLeverArmWarning();
  mip_filter_status_msg->status_flags_gq7_mounting_transform_warning = status.status_flags.gq7MountingTransformWarning();
  mip_filter_status_msg->status_flags_gq7_time_sync_warning = status.status_flags.gq7TimeSyncWarning();
  mip_filter_status_msg->status_flags_gq7_solution_error = status.status_flags.gq7SolutionError();
  mip_filter_status_pub_->publish(*mip_filter_status_msg);
}

void Publishers::handleFilterEcefPos(const mip::data_filter::EcefPos& ecef_pos, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto filter_odom_msg = filter_odom_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_odom_msg->header), descriptor_set, timestamp);
  filter_odom_msg->pose.pose.position.x = ecef_pos.position_ecef[0];
  filter_odom_msg->pose.pose.position.y = ecef_pos.position_ecef[1];
  filter_odom_msg->pose.pose.position.z = ecef_pos.position_ecef[2];

  // Note that we are ready to publish the ECEF transform
  ecef_transform_position_updated_ = true;
}

void Publishers::handleFilterEcefPosUncertainty(const mip::data_filter::EcefPosUncertainty& ecef_pos_uncertainty, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto filter_odom_msg = filter_odom_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_odom_msg->header), descriptor_set, timestamp);
  filter_odom_msg->pose.covariance[0] = pow(ecef_pos_uncertainty.pos_uncertainty[0], 2);
  filter_odom_msg->pose.covariance[7] = pow(ecef_pos_uncertainty.pos_uncertainty[1], 2);
  filter_odom_msg->pose.covariance[14] = pow(ecef_pos_uncertainty.pos_uncertainty[2], 2);
}

void Publishers::handleFilterPositionLlh(const mip::data_filter::PositionLlh& position_llh, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto filter_fix_msg = filter_fix_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_fix_msg->header), descriptor_set, timestamp);
  filter_fix_msg->latitude = position_llh.latitude;
  filter_fix_msg->longitude = position_llh.longitude;
  filter_fix_msg->altitude = position_llh.ellipsoid_height;

  // If the device does not support ECEF, fill it out here, and call the callback ourselves
  if (!supports_filter_ecef_)
  {
    mip::data_filter::EcefPos ecef_pos;
    geocentric_converter_.Forward(position_llh.latitude, position_llh.longitude, position_llh.ellipsoid_height,
      ecef_pos.position_ecef[0], ecef_pos.position_ecef[1], ecef_pos.position_ecef[2]);
    handleFilterEcefPos(ecef_pos, descriptor_set, timestamp);
  }
}

void Publishers::handleFilterPositionLlhUncertainty(const mip::data_filter::PositionLlhUncertainty& position_llh_uncertainty, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Filter fix message
  auto filter_fix_msg = filter_fix_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_fix_msg->header), descriptor_set, timestamp);
  filter_fix_msg->position_covariance_type = NavSatFixMsg::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  if (config_->use_enu_frame_)
  {
    filter_fix_msg->position_covariance[0] = pow(position_llh_uncertainty.north, 2);
    filter_fix_msg->position_covariance[4] = pow(position_llh_uncertainty.east, 2);
  }
  else
  {
    filter_fix_msg->position_covariance[0] = pow(position_llh_uncertainty.east, 2);
    filter_fix_msg->position_covariance[4] = pow(position_llh_uncertainty.north, 2);
  }
  filter_fix_msg->position_covariance[8] = pow(position_llh_uncertainty.down, 2);

  // Filter relative odometry message (not counted as updating)
  auto filter_relative_odom_msg = filter_relative_odom_pub_->getMessage();
  if (config_->use_enu_frame_)
  {
    filter_relative_odom_msg->pose.covariance[0] = pow(position_llh_uncertainty.east, 2);
    filter_relative_odom_msg->pose.covariance[7] = pow(position_llh_uncertainty.north, 2);
  }
  else
  {
    filter_relative_odom_msg->pose.covariance[0] = pow(position_llh_uncertainty.north, 2);
    filter_relative_odom_msg->pose.covariance[7] = pow(position_llh_uncertainty.east, 2);
  }
  filter_relative_odom_msg->pose.covariance[14] = pow(position_llh_uncertainty.down, 2);
}

void Publishers::handleFilterAttitudeQuaternion(const mip::data_filter::AttitudeQuaternion& attitude_quaternion, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Filtered IMU message
  auto imu_msg = imu_pub_->getMessageToUpdate();
  updateHeaderTime(&(imu_msg->header), descriptor_set, timestamp);
  if (config_->use_enu_frame_)
  {
    tf2::Quaternion q_ned2enu, q_body2enu, q_vehiclebody2sensorbody, q_body2ned(attitude_quaternion.q[1], attitude_quaternion.q[2], attitude_quaternion.q[3], attitude_quaternion.q[0]);

    config_->t_ned2enu_.getRotation(q_ned2enu);
    config_->t_vehiclebody2sensorbody_.getRotation(q_vehiclebody2sensorbody);

    q_body2enu = q_ned2enu * q_body2ned * q_vehiclebody2sensorbody;

    imu_msg->orientation = tf2::toMsg(q_body2enu);
  }
  else
  {
    imu_msg->orientation.x = attitude_quaternion.q[1];
    imu_msg->orientation.y = attitude_quaternion.q[2];
    imu_msg->orientation.z = attitude_quaternion.q[3];
    imu_msg->orientation.w = attitude_quaternion.q[0];
  }

  // Filter odometry message (not counted as updating)
  auto filter_odom_msg = filter_odom_pub_->getMessage();
  filter_odom_msg->pose.pose.orientation = imu_msg->orientation;

  // Filter relative odometry message (not counted as updating)
  auto filter_relative_odom_msg = filter_relative_odom_pub_->getMessage();
  filter_relative_odom_msg->pose.pose.orientation = imu_msg->orientation;

  // Relative transform
  updateHeaderTime(&(filter_relative_transform_msg_.header), descriptor_set, timestamp);
  filter_relative_transform_msg_.transform.rotation = filter_odom_msg->pose.pose.orientation;

  // ECEF transform updated
  ecef_transform_attitude_updated_ = true;

  // Relative transform updated
  relative_transform_attitude_updated_ = true;

  // Save the quaternion for later
  filter_attitude_quaternion_ = tf2::Quaternion(attitude_quaternion.q[1], attitude_quaternion.q[2], attitude_quaternion.q[3], attitude_quaternion.q[0]);
}

void Publishers::handleFilterEulerAnglesUncertainty(const mip::data_filter::EulerAnglesUncertainty& euler_angles_uncertainty, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Filtered IMU message
  auto imu_msg = imu_pub_->getMessageToUpdate();
  updateHeaderTime(&(imu_msg->header), descriptor_set, timestamp);
  if (config_->use_enu_frame_)
  {
    imu_msg->orientation_covariance[0] = pow(euler_angles_uncertainty.pitch, 2);
    imu_msg->orientation_covariance[4] = pow(euler_angles_uncertainty.roll, 2);
  }
  else
  {
    imu_msg->orientation_covariance[0] = pow(euler_angles_uncertainty.roll, 2);
    imu_msg->orientation_covariance[4] = pow(euler_angles_uncertainty.pitch, 2);
  }
  imu_msg->orientation_covariance[8] = pow(euler_angles_uncertainty.yaw, 2);

  // Filter odometry message (not counted as updating)
  auto filter_odom_msg = filter_odom_pub_->getMessage();
  filter_odom_msg->pose.covariance[21] = imu_msg->orientation_covariance[0];
  filter_odom_msg->pose.covariance[28] = imu_msg->orientation_covariance[4];
  filter_odom_msg->pose.covariance[35] = imu_msg->orientation_covariance[8];

  // Filter relative odometry message (not counted as updating)
  auto filter_relative_odom_msg = filter_relative_odom_pub_->getMessage();
  filter_relative_odom_msg->pose.covariance[21] = imu_msg->orientation_covariance[0];
  filter_relative_odom_msg->pose.covariance[28] = imu_msg->orientation_covariance[4];
  filter_relative_odom_msg->pose.covariance[35] = imu_msg->orientation_covariance[8];
}

void Publishers::handleFilterVelocityNed(const mip::data_filter::VelocityNed& velocity_ned, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Filter ENU velocity message
  auto filter_vel_msg = filter_vel_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_vel_msg->header), descriptor_set, timestamp);
  if (config_->use_enu_frame_)
  {
    filter_vel_msg->twist.twist.linear.x = velocity_ned.east;
    filter_vel_msg->twist.twist.linear.y = velocity_ned.north;
    filter_vel_msg->twist.twist.linear.z = -velocity_ned.down;
  }
  else
  {
    filter_vel_msg->twist.twist.linear.x = velocity_ned.north;
    filter_vel_msg->twist.twist.linear.x = velocity_ned.east;
    filter_vel_msg->twist.twist.linear.x = velocity_ned.down;
  }

  // Filter odometry message (not counted as updating)
  auto filter_odom_msg = filter_odom_pub_->getMessage();

  // Rotate the velocity to the sensor frame
  const tf2::Vector3 filter_current_vel(filter_vel_msg->twist.twist.linear.x, filter_vel_msg->twist.twist.linear.y, filter_vel_msg->twist.twist.linear.z);
  const tf2::Vector3 filter_rotated_vel = tf2::quatRotate(filter_attitude_quaternion_.inverse(), filter_current_vel);
  filter_odom_msg->twist.twist.linear.x = filter_rotated_vel.getX();
  filter_odom_msg->twist.twist.linear.y = filter_rotated_vel.getY();
  filter_odom_msg->twist.twist.linear.z = filter_rotated_vel.getZ();

  // Filter relative odometry message (not counted as updating)
  auto filter_relative_odom_msg = filter_relative_odom_pub_->getMessage();
  filter_relative_odom_msg->twist.twist.linear = filter_odom_msg->twist.twist.linear;
}

void Publishers::handleFilterVelocityNedUncertainty(const mip::data_filter::VelocityNedUncertainty& velocity_ned_uncertainty, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Filter ENU velocity message
  auto filter_vel_msg = filter_vel_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_vel_msg->header), descriptor_set, timestamp);
  if (config_->use_enu_frame_)
  {
    filter_vel_msg->twist.covariance[0] = pow(velocity_ned_uncertainty.east, 2);
    filter_vel_msg->twist.covariance[7] = pow(velocity_ned_uncertainty.north, 2);
  }
  else
  {
    filter_vel_msg->twist.covariance[0] = pow(velocity_ned_uncertainty.north, 2);
    filter_vel_msg->twist.covariance[7] = pow(velocity_ned_uncertainty.east, 2);
  }
  filter_vel_msg->twist.covariance[14] = pow(velocity_ned_uncertainty.down, 2);

  // Filter odometry message (not counted as updating)
  auto filter_odom_msg = filter_odom_pub_->getMessageToUpdate();
  filter_odom_msg->twist.covariance = filter_vel_msg->twist.covariance;

  // Filter relative odometry message (not counted as opening)
  auto filter_relative_odom_msg = filter_relative_odom_pub_->getMessageToUpdate();
  filter_relative_odom_msg->twist.covariance = filter_vel_msg->twist.covariance;
}

void Publishers::handleFilterEcefVelocity(const mip::data_filter::EcefVel& ecef_vel, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto filter_vel_ecef_msg = filter_vel_ecef_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_vel_ecef_msg->header), descriptor_set, timestamp);
  filter_vel_ecef_msg->twist.twist.linear.x = ecef_vel.velocity_ecef[0];
  filter_vel_ecef_msg->twist.twist.linear.y = ecef_vel.velocity_ecef[1];
  filter_vel_ecef_msg->twist.twist.linear.z = ecef_vel.velocity_ecef[2];
}

void Publishers::handleFilterEcefVelocityUncertainty(const mip::data_filter::EcefVelUncertainty& ecef_vel_uncertainty, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto filter_vel_ecef_msg = filter_vel_ecef_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_vel_ecef_msg->header), descriptor_set, timestamp);
  filter_vel_ecef_msg->twist.covariance[0] = pow(ecef_vel_uncertainty.vel_uncertainty[0], 2);
  filter_vel_ecef_msg->twist.covariance[7] = pow(ecef_vel_uncertainty.vel_uncertainty[1], 2);
  filter_vel_ecef_msg->twist.covariance[14] = pow(ecef_vel_uncertainty.vel_uncertainty[2], 2);
}

void Publishers::handleFilterCompAngularRate(const mip::data_filter::CompAngularRate& comp_angular_rate, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Filtered IMU message
  auto imu_msg = imu_pub_->getMessageToUpdate();
  updateHeaderTime(&(imu_msg->header), descriptor_set, timestamp);
  imu_msg->angular_velocity.x = comp_angular_rate.gyro[0];
  imu_msg->angular_velocity.y = comp_angular_rate.gyro[1];
  imu_msg->angular_velocity.z = comp_angular_rate.gyro[2];
  if (config_->use_enu_frame_)
  {
    imu_msg->angular_velocity.y *= -1;
    imu_msg->angular_velocity.z *= -1;
  }

  // Filter odometry message (not counted as updating)
  auto filter_odom_msg = filter_odom_pub_->getMessage();
  filter_odom_msg->twist.twist.angular = imu_msg->angular_velocity;

  // Filter relative odometry message (not counted as updating)
  auto filter_relative_odom_msg = filter_relative_odom_pub_->getMessage();
  filter_relative_odom_msg->twist.twist.angular = imu_msg->angular_velocity;
}

void Publishers::handleFilterCompAccel(const mip::data_filter::CompAccel& comp_accel, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  if (config_->filter_use_compensated_accel_)
  {
    auto imu_msg = imu_pub_->getMessageToUpdate();
    updateHeaderTime(&(imu_msg->header), descriptor_set, timestamp);
    imu_msg->linear_acceleration.x = comp_accel.accel[0];
    imu_msg->linear_acceleration.y = comp_accel.accel[1];
    imu_msg->linear_acceleration.z = comp_accel.accel[2];
    if (config_->use_enu_frame_)
    {
      imu_msg->linear_acceleration.y *= -1.0;
      imu_msg->linear_acceleration.z *= -1.0;
    }
  }
}

void Publishers::handleFilterLinearAccel(const mip::data_filter::LinearAccel& linear_accel, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  if (!config_->filter_use_compensated_accel_)
  {
    auto imu_msg = imu_pub_->getMessageToUpdate();
    updateHeaderTime(&(imu_msg->header), descriptor_set, timestamp);
    imu_msg->linear_acceleration.x = linear_accel.accel[0];
    imu_msg->linear_acceleration.y = linear_accel.accel[1];
    imu_msg->linear_acceleration.z = linear_accel.accel[2];
    if (config_->use_enu_frame_)
    {
      imu_msg->linear_acceleration.y *= -1.0;
      imu_msg->linear_acceleration.z *= -1.0;
    }
  }
}

void Publishers::handleFilterRelPosNed(const mip::data_filter::RelPosNed& rel_pos_ned, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Filter relative odometry message
  auto filter_relative_odom_msg = filter_relative_odom_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_relative_transform_msg_.header), descriptor_set, timestamp);
  if (config_->use_enu_frame_)
  {
    filter_relative_odom_msg->pose.pose.position.x = rel_pos_ned.relative_position[1];
    filter_relative_odom_msg->pose.pose.position.y = rel_pos_ned.relative_position[0];
    filter_relative_odom_msg->pose.pose.position.z = -rel_pos_ned.relative_position[2];
  }
  else
  {
    filter_relative_odom_msg->pose.pose.position.x = rel_pos_ned.relative_position[0];
    filter_relative_odom_msg->pose.pose.position.y = rel_pos_ned.relative_position[1];
    filter_relative_odom_msg->pose.pose.position.z = rel_pos_ned.relative_position[2];
  }

  // Relative transform
  filter_relative_transform_msg_.transform.translation.x = filter_relative_odom_msg->pose.pose.position.x;
  filter_relative_transform_msg_.transform.translation.y = filter_relative_odom_msg->pose.pose.position.y;
  filter_relative_transform_msg_.transform.translation.z = filter_relative_odom_msg->pose.pose.position.z;

  // Relative transform updated
  relative_transform_position_updated_ = true;
}

void Publishers::handleFilterGnssPosAidStatus(const mip::data_filter::GnssPosAidStatus& gnss_pos_aid_status, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Filter GNSS position aiding status
  auto mip_filter_gnss_position_aiding_status_msg = mip_filter_gnss_position_aiding_status_pub_->getMessage();
  updateMicrostrainHeader(&(mip_filter_gnss_position_aiding_status_msg->header), descriptor_set);
  mip_filter_gnss_position_aiding_status_msg->receiver_id = gnss_pos_aid_status.receiver_id;
  mip_filter_gnss_position_aiding_status_msg->time_of_week = gnss_pos_aid_status.time_of_week;
  mip_filter_gnss_position_aiding_status_msg->tight_coupling = gnss_pos_aid_status.status.tightCoupling();
  mip_filter_gnss_position_aiding_status_msg->differential = gnss_pos_aid_status.status.differential();
  mip_filter_gnss_position_aiding_status_msg->integer_fix = gnss_pos_aid_status.status.integerFix();
  mip_filter_gnss_position_aiding_status_msg->gps_l1 = gnss_pos_aid_status.status.gpsL1();
  mip_filter_gnss_position_aiding_status_msg->gps_l2 = gnss_pos_aid_status.status.gpsL2();
  mip_filter_gnss_position_aiding_status_msg->gps_l5 = gnss_pos_aid_status.status.gpsL5();
  mip_filter_gnss_position_aiding_status_msg->glo_l1 = gnss_pos_aid_status.status.gloL1();
  mip_filter_gnss_position_aiding_status_msg->glo_l2 = gnss_pos_aid_status.status.gloL2();
  mip_filter_gnss_position_aiding_status_msg->gal_e1 = gnss_pos_aid_status.status.galE1();
  mip_filter_gnss_position_aiding_status_msg->gal_e5 = gnss_pos_aid_status.status.galE5();
  mip_filter_gnss_position_aiding_status_msg->gal_e6 = gnss_pos_aid_status.status.galE6();
  mip_filter_gnss_position_aiding_status_msg->bei_b1 = gnss_pos_aid_status.status.beiB1();
  mip_filter_gnss_position_aiding_status_msg->bei_b2 = gnss_pos_aid_status.status.beiB2();
  mip_filter_gnss_position_aiding_status_msg->bei_b3 = gnss_pos_aid_status.status.beiB3();
  mip_filter_gnss_position_aiding_status_msg->no_fix = gnss_pos_aid_status.status.noFix();
  mip_filter_gnss_position_aiding_status_msg->config_error = gnss_pos_aid_status.status.configError();
  mip_filter_gnss_position_aiding_status_pub_->publish(*mip_filter_gnss_position_aiding_status_msg);

  // Filter fix message (not counted as updating)
  auto filter_fix_msg = filter_fix_pub_->getMessage();

  // Take the best out of the two receivers
  bool status_set = false;
  const uint8_t gnss_index = gnss_pos_aid_status.receiver_id - 1;
  if (filter_fix_msg->status.status <= NavSatFixMsg::_status_type::STATUS_GBAS_FIX && mip_filter_gnss_position_aiding_status_msg->differential)
  {
    status_set = true;
    filter_fix_msg->status.status = NavSatFixMsg::_status_type::STATUS_GBAS_FIX;
  }
  else if (filter_fix_msg->status.status <= NavSatFixMsg::_status_type::STATUS_SBAS_FIX && mip_gnss_fix_info_pub_[gnss_index]->getMessage()->sbas_used)
  {
    status_set = true;
    filter_fix_msg->status.status = NavSatFixMsg::_status_type::STATUS_SBAS_FIX;
  }
  else if (filter_fix_msg->status.status <= NavSatFixMsg::_status_type::STATUS_FIX && !mip_filter_gnss_position_aiding_status_msg->no_fix)
  {
    status_set = true;
    filter_fix_msg->status.status = NavSatFixMsg::_status_type::STATUS_FIX;
  }
  if (!status_set)
    filter_fix_msg->status.status = NavSatFixMsg::_status_type::STATUS_NO_FIX;
}

void Publishers::handleFilterMultiAntennaOffsetCorrection(const mip::data_filter::MultiAntennaOffsetCorrection& multi_antenna_offset_correction, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto mip_filter_multi_antenna_offset_correction_msg = mip_filter_multi_antenna_offset_correction_pub_->getMessage();
  updateMicrostrainHeader(&(mip_filter_multi_antenna_offset_correction_msg->header), descriptor_set);
  mip_filter_multi_antenna_offset_correction_msg->receiver_id = multi_antenna_offset_correction.receiver_id;
  mip_filter_multi_antenna_offset_correction_msg->offset[0] = multi_antenna_offset_correction.offset[0];
  mip_filter_multi_antenna_offset_correction_msg->offset[1] = multi_antenna_offset_correction.offset[1];
  mip_filter_multi_antenna_offset_correction_msg->offset[2] = multi_antenna_offset_correction.offset[2];
  mip_filter_multi_antenna_offset_correction_pub_->publish(*mip_filter_multi_antenna_offset_correction_msg);

  // Update the GNSS transform
  TransformStampedMsg gnss_antenna_transform = gnss_antenna_transform_msg_[multi_antenna_offset_correction.receiver_id - 1];
  gnss_antenna_transform.header.stamp = rosTimeNow(node_);
  gnss_antenna_transform.transform.translation.x += multi_antenna_offset_correction.offset[0];
  gnss_antenna_transform.transform.translation.y += multi_antenna_offset_correction.offset[1];
  gnss_antenna_transform.transform.translation.z += multi_antenna_offset_correction.offset[2];
  transform_broadcaster_->sendTransform(gnss_antenna_transform);
}

void Publishers::handleFilterGnssDualAntennaStatus(const mip::data_filter::GnssDualAntennaStatus& mip_filter_gnss_dual_antenna_status, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto mip_filter_gnss_dual_antenna_status_msg = mip_filter_gnss_dual_antenna_status_pub_->getMessage();
  updateMicrostrainHeader(&(mip_filter_gnss_dual_antenna_status_msg->header), descriptor_set);
  mip_filter_gnss_dual_antenna_status_msg->time_of_week = mip_filter_gnss_dual_antenna_status.time_of_week;
  mip_filter_gnss_dual_antenna_status_msg->heading = mip_filter_gnss_dual_antenna_status.heading;
  mip_filter_gnss_dual_antenna_status_msg->heading_unc = mip_filter_gnss_dual_antenna_status.heading_unc;
  mip_filter_gnss_dual_antenna_status_msg->fix_type = static_cast<uint8_t>(mip_filter_gnss_dual_antenna_status.fix_type);
  mip_filter_gnss_dual_antenna_status_msg->status_flags_rcv_1_data_valid = mip_filter_gnss_dual_antenna_status.status_flags & mip::data_filter::GnssDualAntennaStatus::DualAntennaStatusFlags::RCV_1_DATA_VALID;
  mip_filter_gnss_dual_antenna_status_msg->status_flags_rcv_2_data_valid = mip_filter_gnss_dual_antenna_status.status_flags & mip::data_filter::GnssDualAntennaStatus::DualAntennaStatusFlags::RCV_2_DATA_VALID;
  mip_filter_gnss_dual_antenna_status_msg->status_flags_antenna_offsets_valid = mip_filter_gnss_dual_antenna_status.status_flags & mip::data_filter::GnssDualAntennaStatus::DualAntennaStatusFlags::ANTENNA_OFFSETS_VALID;
  mip_filter_gnss_dual_antenna_status_pub_->publish(*mip_filter_gnss_dual_antenna_status_msg);
}

void Publishers::handleFilterAidingMeasurementSummary(const mip::data_filter::AidingMeasurementSummary& aiding_measurement_summary, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto mip_filter_aiding_measurement_summary_msg = mip_filter_aiding_measurement_summary_pub_->getMessage();
  updateMicrostrainHeader(&(mip_filter_aiding_measurement_summary_msg->header), descriptor_set);
  mip_filter_aiding_measurement_summary_msg->time_of_week = aiding_measurement_summary.time_of_week;
  mip_filter_aiding_measurement_summary_msg->source = aiding_measurement_summary.source;
  mip_filter_aiding_measurement_summary_msg->type = static_cast<uint8_t>(aiding_measurement_summary.type);

  mip_filter_aiding_measurement_summary_msg->indicator_enabled = aiding_measurement_summary.indicator.enabled();
  mip_filter_aiding_measurement_summary_msg->indicator_used = aiding_measurement_summary.indicator.used();
  mip_filter_aiding_measurement_summary_msg->indicator_residual_high_warning = aiding_measurement_summary.indicator.residualHighWarning();
  mip_filter_aiding_measurement_summary_msg->indicator_sample_time_warning = aiding_measurement_summary.indicator.sampleTimeWarning();
  mip_filter_aiding_measurement_summary_msg->indicator_configuration_error = aiding_measurement_summary.indicator.configurationError();
  mip_filter_aiding_measurement_summary_msg->indicator_max_num_meas_exceeded = aiding_measurement_summary.indicator.maxNumMeasExceeded();
  mip_filter_aiding_measurement_summary_pub_->publish(*mip_filter_aiding_measurement_summary_msg);
}

void Publishers::handleAfterPacket(const mip::Packet& packet, mip::Timestamp timestamp)
{
  // Right now, we don't have to do much, just publish everything
  publish();

  // Reset some shared descriptors. These are unique to the packets, and we do not want to cache them past this packet
  if (event_source_mapping_.find(packet.descriptorSet()) != event_source_mapping_.end())
    event_source_mapping_[packet.descriptorSet()].trigger_id = 0;
}

void Publishers::updateMicrostrainHeader(MicrostrainHeaderMsg* microstrain_header, uint8_t descriptor_set) const
{
  // Update the ROS header with the ROS timestamp
  microstrain_header->header.stamp = rosTimeNow(node_);

  // Set the event source if it was set for this packet
  if (event_source_mapping_.find(descriptor_set) != event_source_mapping_.end())
    microstrain_header->event_source = event_source_mapping_.at(descriptor_set).trigger_id;
  else
    microstrain_header->event_source = 0;
  
  // Set the most recent reference timestamp if we have one
  if (reference_timestamp_mapping_.find(descriptor_set) != reference_timestamp_mapping_.end())
    microstrain_header->reference_timestamp = reference_timestamp_mapping_.at(descriptor_set).nanoseconds;
  
  // Set the GPS timestamp if we have one (should always have one)
  if (gps_timestamp_mapping_.find(descriptor_set) != gps_timestamp_mapping_.end())
  {
    microstrain_header->gps_timestamp.week_number = gps_timestamp_mapping_.at(descriptor_set).week_number;
    microstrain_header->gps_timestamp.tow = gps_timestamp_mapping_.at(descriptor_set).tow;
  }
}

void Publishers::updateHeaderTime(RosHeaderType* header, uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Set the header time to the current ROS time
  header->stamp = rosTimeNow(node_);
}

void Publishers::setGpsTime(RosTimeType* time, const mip::data_shared::GpsTimestamp& timestamp)
{
  // Split the seconds and subseconds out to get around the double resolution issue
  double seconds;
  double subseconds = modf(timestamp.tow, &seconds);

  // Seconds since start of Unix time = seconds between 1970 and 1980 + number of weeks since 1980 * number of seconds in a week + number of complete seconds past in current week - leap seconds since start of GPS time
  const uint64_t utc_milliseconds = static_cast<uint64_t>((315964800 + timestamp.week_number * 604800 + static_cast<uint64_t>(seconds) - GPS_LEAP_SECONDS) * 1000L) + static_cast<uint64_t>(std::round(subseconds * 1000.0));
  setRosTime(time, utc_milliseconds / 1000, (utc_milliseconds % 1000) * 1000000);
}

}  // namespace microstrain
