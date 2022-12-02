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

constexpr auto USTRAIN_G =
    9.80665;  // from section 5.1.1 in
              // https://www.microstrain.com/sites/default/files/3dm-gx5-25_dcp_manual_8500-0065_reference_document.pdf

Publishers::Publishers(RosNodeType* node, Config* config)
  : node_(node), config_(config)
{
}

bool Publishers::configure()
{
  imu_pub_->configure(node_, config_);
  mag_pub_->configure(node_, config_);
  gps_corr_pub_->configure(node_, config_);
  imu_overrange_status_pub_->configure(node_, config_);

  for (const auto& pub : gnss_pub_) pub->configure(node_, config_);
  for (const auto& pub : gnss_odom_pub_) pub->configure(node_, config_);
  for (const auto& pub : gnss_time_pub_) pub->configure(node_, config_);
  for (const auto& pub : gnss_aiding_status_pub_) pub->configure(node_, config_);
  for (const auto& pub : gnss_fix_info_pub_) pub->configure(node_, config_);
  for (const auto& pub : gnss_sbas_info_pub_) pub->configure(node_, config_);
  for (const auto& pub : gnss_rf_error_detection_pub_) pub->configure(node_, config_);

  rtk_pub_->configure(node_, config_);
  rtk_pub_v1_->configure(node_, config_);

  filter_status_pub_->configure(node_, config_);
  filter_heading_pub_->configure(node_, config_);
  filter_heading_state_pub_->configure(node_, config_);
  filter_aiding_mesaurement_summary_pub_->configure(node_, config_);
  filter_odom_pub_->configure(node_, config_);
  filter_imu_pub_->configure(node_, config_);
  gnss_dual_antenna_status_pub_->configure(node_, config_);

  // Only publish relative odom if we support the relative position descriptor set
  if (config_->mip_device_->supportsDescriptor(mip::data_filter::DESCRIPTOR_SET, mip::data_filter::RelPosNed::DESCRIPTOR_SET))
    filter_relative_odom_pub_->configure(node_, config_);

  if (config_->publish_nmea_)
    nmea_sentence_pub_->configure(node_);

  // Frame ID configuration
  imu_pub_->getMessage()->header.frame_id = config_->imu_frame_id_;
  mag_pub_->getMessage()->header.frame_id = config_->imu_frame_id_;
  gps_corr_pub_->getMessage()->header.frame_id = config_->imu_frame_id_;

  for (int i = 0; i < gnss_pub_.size(); i++) gnss_pub_[i]->getMessage()->header.frame_id = config_->gnss_frame_id_[i];
  for (int i = 0; i < gnss_odom_pub_.size(); i++) gnss_odom_pub_[i]->getMessage()->header.frame_id = config_->gnss_frame_id_[i];
  for (int i = 0; i < gnss_time_pub_.size(); i++) gnss_time_pub_[i]->getMessage()->header.frame_id = config_->gnss_frame_id_[i];

  filter_odom_pub_->getMessage()->header.frame_id = config_->filter_frame_id_;
  filter_odom_pub_->getMessage()->child_frame_id = config_->filter_child_frame_id_;
  filter_imu_pub_->getMessage()->header.frame_id = config_->filter_frame_id_;
  filter_relative_odom_pub_->getMessage()->header.frame_id = config_->filter_frame_id_;
  filter_relative_odom_pub_->getMessage()->child_frame_id = config_->filter_child_frame_id_;
  filter_relative_transform_msg_.header.frame_id = config_->filter_frame_id_;
  filter_relative_transform_msg_.child_frame_id = config_->filter_child_frame_id_;

  // Other assorted static configuration
  auto imu_msg = imu_pub_->getMessage();
  std::copy(config_->imu_linear_cov_.begin(), config_->imu_linear_cov_.end(), imu_msg->linear_acceleration_covariance.begin());
  std::copy(config_->imu_angular_cov_.begin(), config_->imu_angular_cov_.end(), imu_msg->angular_velocity_covariance.begin());
  std::copy(config_->imu_orientation_cov_.begin(), config_->imu_orientation_cov_.end(), imu_msg->orientation_covariance.begin());

  // Transform broadcaster setup
  transform_broadcaster_ = createTransformBroadcaster(node_);

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
  registerDataCallback<mip::data_sensor::OverrangeStatus, &Publishers::handleSensorOverrangeStatus>();

  // GNSS1/2 callbacks
  for (const uint8_t gnss_descriptor_set : std::initializer_list<uint8_t>{mip::data_gnss::DESCRIPTOR_SET, mip::data_gnss::MIP_GNSS1_DATA_DESC_SET, mip::data_gnss::MIP_GNSS2_DATA_DESC_SET})
  {
    registerDataCallback<mip::data_gnss::PosLlh, &Publishers::handleGnssPosLlh>(gnss_descriptor_set);
    registerDataCallback<mip::data_gnss::VelNed, &Publishers::handleGnssVelNed>(gnss_descriptor_set);
    registerDataCallback<mip::data_gnss::FixInfo, &Publishers::handleGnssFixInfo>(gnss_descriptor_set);
    registerDataCallback<mip::data_gnss::SbasInfo, &Publishers::handleGnssSbasInfo>(gnss_descriptor_set);
    registerDataCallback<mip::data_gnss::RfErrorDetection, &Publishers::handleGnssRfErrorDetection>(gnss_descriptor_set);
  }

  // Also register callbacks for GNSS1/2 for the GPS time message
  // Note: It is important to make sure this is after the GNSS1/2 callbacks
  for (const uint8_t gnss_descriptor_set : std::initializer_list<uint8_t>{mip::data_gnss::MIP_GNSS1_DATA_DESC_SET, mip::data_gnss::MIP_GNSS2_DATA_DESC_SET})
  {
    registerDataCallback<mip::data_gnss::GpsTime, &Publishers::handleGnssGpsTime>(gnss_descriptor_set);
  }

  // Filter callbacks
  registerDataCallback<mip::data_filter::Status, &Publishers::handleFilterStatus>();
  registerDataCallback<mip::data_filter::EulerAngles, &Publishers::handleFilterEulerAngles>();
  registerDataCallback<mip::data_filter::HeadingUpdateState, &Publishers::handleFilterHeadingUpdateState>();
  registerDataCallback<mip::data_filter::PositionLlh, &Publishers::handleFilterPositionLlh>();
  registerDataCallback<mip::data_filter::PositionLlhUncertainty, &Publishers::handleFilterPositionLlhUncertainty>();
  registerDataCallback<mip::data_filter::AttitudeQuaternion, &Publishers::handleFilterAttitudeQuaternion>();
  registerDataCallback<mip::data_filter::EulerAnglesUncertainty, &Publishers::handleFilterEulerAnglesUncertainty>();
  registerDataCallback<mip::data_filter::VelocityNed, &Publishers::handleFilterVelocityNed>();
  registerDataCallback<mip::data_filter::VelocityNedUncertainty, &Publishers::handleFilterVelocityNedUncertainty>();
  registerDataCallback<mip::data_filter::CompAngularRate, &Publishers::handleFilterCompAngularRate>();
  registerDataCallback<mip::data_filter::CompAccel, &Publishers::handleFilterCompAccel>();
  registerDataCallback<mip::data_filter::LinearAccel, &Publishers::handleFilterLinearAccel>();
  registerDataCallback<mip::data_filter::RelPosNed, &Publishers::handleFilterRelPosNed>();
  registerDataCallback<mip::data_filter::GnssPosAidStatus, &Publishers::handleFilterGnssPosAidStatus>();
  registerDataCallback<mip::data_filter::GnssDualAntennaStatus, &Publishers::handleFilterGnssDualAntennaStatus>();
  registerDataCallback<mip::data_filter::AidingMeasurementSummary, &Publishers::handleFilterAidingMeasurementSummary>();
  return true;
}

bool Publishers::activate()
{
  imu_pub_->activate();
  mag_pub_->activate();
  gps_corr_pub_->activate();
  imu_overrange_status_pub_->activate();

  for (const auto& pub : gnss_pub_) pub->activate();
  for (const auto& pub : gnss_odom_pub_) pub->activate();
  for (const auto& pub : gnss_time_pub_) pub->activate();
  for (const auto& pub : gnss_aiding_status_pub_) pub->activate();
  for (const auto& pub : gnss_fix_info_pub_) pub->activate();
  for (const auto& pub : gnss_sbas_info_pub_) pub->activate();
  for (const auto& pub : gnss_rf_error_detection_pub_) pub->activate();

  rtk_pub_->activate();
  rtk_pub_v1_->activate();

  filter_status_pub_->activate();
  filter_heading_pub_->activate();
  filter_heading_state_pub_->activate();
  filter_aiding_mesaurement_summary_pub_->activate();
  filter_odom_pub_->activate();
  filter_relative_odom_pub_->activate();
  filter_imu_pub_->activate();
  gnss_dual_antenna_status_pub_->activate();

  nmea_sentence_pub_->activate();
  return true;
}

bool Publishers::deactivate()
{
  imu_pub_->deactivate();
  mag_pub_->deactivate();
  gps_corr_pub_->deactivate();
  imu_overrange_status_pub_->deactivate();

  for (const auto& pub : gnss_pub_) pub->deactivate();
  for (const auto& pub : gnss_odom_pub_) pub->deactivate();
  for (const auto& pub : gnss_time_pub_) pub->deactivate();
  for (const auto& pub : gnss_aiding_status_pub_) pub->deactivate();
  for (const auto& pub : gnss_fix_info_pub_) pub->deactivate();
  for (const auto& pub : gnss_sbas_info_pub_) pub->deactivate();
  for (const auto& pub : gnss_rf_error_detection_pub_) pub->deactivate();

  rtk_pub_->deactivate();
  rtk_pub_v1_->deactivate();

  filter_status_pub_->deactivate();
  filter_heading_pub_->deactivate();
  filter_heading_state_pub_->deactivate();
  filter_aiding_mesaurement_summary_pub_->deactivate();
  filter_odom_pub_->deactivate();
  filter_relative_odom_pub_->deactivate();
  filter_imu_pub_->deactivate();
  gnss_dual_antenna_status_pub_->deactivate();
  return true;
}

void Publishers::publish()
{
  // If the corresponding messages were updated, publish the transforms
  if (filter_relative_odom_pub_ && filter_relative_odom_pub_->updated())
    transform_broadcaster_->sendTransform(filter_relative_transform_msg_);

  imu_pub_->publish();
  mag_pub_->publish();
  gps_corr_pub_->publish();
  imu_overrange_status_pub_->publish();

  for (const auto& pub : gnss_pub_) pub->publish();
  for (const auto& pub : gnss_odom_pub_) pub->publish();
  for (const auto& pub : gnss_time_pub_) pub->publish();
  for (const auto& pub : gnss_aiding_status_pub_) pub->publish();
  for (const auto& pub : gnss_fix_info_pub_) pub->publish();
  for (const auto& pub : gnss_sbas_info_pub_) pub->publish();
  for (const auto& pub : gnss_rf_error_detection_pub_) pub->publish();

  rtk_pub_->publish();
  rtk_pub_v1_->publish();

  filter_status_pub_->publish();
  filter_heading_pub_->publish();
  filter_heading_state_pub_->publish();
  filter_aiding_mesaurement_summary_pub_->publish();
  filter_odom_pub_->publish();
  filter_relative_odom_pub_->publish();
  filter_imu_pub_->publish();
  gnss_dual_antenna_status_pub_->publish();
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
  gps_timestamp_mapping_[descriptor_set] = gps_timestamp;
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

  // Populate the ROS message
  auto gps_corr_msg = gps_corr_pub_->getMessageToUpdate();
  updateHeaderTime(&(gps_corr_msg->header), descriptor_set, timestamp);
  gps_corr_msg->gps_cor.gps_tow = gps_timestamp.tow;
  gps_corr_msg->gps_cor.gps_week_number = gps_timestamp.week_number;
  gps_corr_msg->gps_cor.timestamp_flags = gps_timestamp.valid_flags;
}

void Publishers::handleSensorScaledAccel(const mip::data_sensor::ScaledAccel& scaled_accel, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto imu_msg = imu_pub_->getMessageToUpdate();
  updateHeaderTime(&(imu_msg->header), descriptor_set, timestamp);
  imu_msg->linear_acceleration.x = USTRAIN_G * scaled_accel.scaled_accel[0];
  imu_msg->linear_acceleration.y = USTRAIN_G * scaled_accel.scaled_accel[1];
  imu_msg->linear_acceleration.z = USTRAIN_G * scaled_accel.scaled_accel[2];
  if (config_->use_enu_frame_)
  {
    imu_msg->linear_acceleration.y *= -1.0;
    imu_msg->linear_acceleration.z *= -1.0;
  }
}

void Publishers::handleSensorScaledGyro(const mip::data_sensor::ScaledGyro& scaled_gyro, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto imu_msg = imu_pub_->getMessageToUpdate();
  updateHeaderTime(&(imu_msg->header), descriptor_set, timestamp);
  imu_msg->angular_velocity.x = scaled_gyro.scaled_gyro[0];
  imu_msg->angular_velocity.y = scaled_gyro.scaled_gyro[1];
  imu_msg->angular_velocity.z = scaled_gyro.scaled_gyro[2];
  if (config_->use_enu_frame_)
  {
    imu_msg->angular_velocity.y *= -1.0;
    imu_msg->angular_velocity.z *= -1.0;
  }
}

void Publishers::handleSensorCompQuaternion(const mip::data_sensor::CompQuaternion& comp_quaternion, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto imu_msg = imu_pub_->getMessageToUpdate();
  updateHeaderTime(&(imu_msg->header), descriptor_set, timestamp);
  if (config_->use_enu_frame_)
  {
    tf2::Quaternion q_ned2enu, q_body2enu, q_vehiclebody2sensorbody, q_body2ned(comp_quaternion.q[1], comp_quaternion.q[2], comp_quaternion.q[3], comp_quaternion.q[0]);

    config_->t_ned2enu_.getRotation(q_ned2enu);
    config_->t_vehiclebody2sensorbody_.getRotation(q_vehiclebody2sensorbody);

    q_body2enu = q_ned2enu * q_body2ned * q_vehiclebody2sensorbody;

    imu_msg->orientation = tf2::toMsg(q_body2enu);
  }
  else
  {
    imu_msg->orientation.x = comp_quaternion.q[1];
    imu_msg->orientation.y = comp_quaternion.q[2];
    imu_msg->orientation.z = comp_quaternion.q[3];
    imu_msg->orientation.w = comp_quaternion.q[0];
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

void Publishers::handleSensorOverrangeStatus(const mip::data_sensor::OverrangeStatus& overrange_status, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto imu_overrange_status_msg = imu_overrange_status_pub_->getMessageToUpdate();
  imu_overrange_status_msg->status_accel_x = overrange_status.status.accelX();
  imu_overrange_status_msg->status_accel_y = overrange_status.status.accelY();
  imu_overrange_status_msg->status_accel_z = overrange_status.status.accelZ();
  imu_overrange_status_msg->status_gyro_x = overrange_status.status.gyroX();
  imu_overrange_status_msg->status_gyro_y = overrange_status.status.gyroY();
  imu_overrange_status_msg->status_gyro_z = overrange_status.status.gyroZ();
  imu_overrange_status_msg->status_mag_x = overrange_status.status.magX();
  imu_overrange_status_msg->status_mag_y = overrange_status.status.magY();
  imu_overrange_status_msg->status_mag_z = overrange_status.status.magZ();
  imu_overrange_status_msg->status_press = overrange_status.status.press();
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
  auto gnss_msg = gnss_pub_[gnss_index]->getMessageToUpdate();
  updateHeaderTime(&(gnss_msg->header), descriptor_set, timestamp);
  gnss_msg->status.service = 1;
  gnss_msg->position_covariance_type = 2;
  gnss_msg->latitude = pos_llh.latitude;
  gnss_msg->longitude = pos_llh.longitude;
  gnss_msg->altitude = pos_llh.ellipsoid_height;
  gnss_msg->position_covariance[0] = pos_llh.horizontal_accuracy;
  gnss_msg->position_covariance[4] = pos_llh.horizontal_accuracy;
  gnss_msg->position_covariance[8] = pos_llh.vertical_accuracy;

  // GNSS odometry message
  auto gnss_odom_msg = gnss_odom_pub_[gnss_index]->getMessageToUpdate();
  updateHeaderTime(&(gnss_odom_msg->header), descriptor_set, timestamp);
  if (config_->use_enu_frame_)
  {
    gnss_odom_msg->pose.pose.position.x = pos_llh.longitude;
    gnss_odom_msg->pose.pose.position.y = pos_llh.latitude;
  }
  else
  {
    gnss_odom_msg->pose.pose.position.x = pos_llh.latitude;
    gnss_odom_msg->pose.pose.position.y = pos_llh.longitude;
  }
  gnss_odom_msg->pose.pose.position.z = pos_llh.ellipsoid_height;
  gnss_odom_msg->pose.covariance[0] = pos_llh.horizontal_accuracy;
  gnss_odom_msg->pose.covariance[7] = pos_llh.horizontal_accuracy;
  gnss_odom_msg->pose.covariance[14] = pos_llh.vertical_accuracy;
}

void Publishers::handleGnssVelNed(const mip::data_gnss::VelNed& vel_ned, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Different message depending on the descriptor set
  const uint8_t gnss_index = (descriptor_set == mip::data_gnss::DESCRIPTOR_SET || descriptor_set == mip::data_gnss::MIP_GNSS1_DATA_DESC_SET) ? GNSS1_ID : GNSS2_ID;
  auto gnss_odom_msg = gnss_odom_pub_[gnss_index]->getMessageToUpdate();
  updateHeaderTime(&(gnss_odom_msg->header), descriptor_set, timestamp);
  if (config_->use_enu_frame_)
  {
    gnss_odom_msg->twist.twist.linear.x = vel_ned.v[1];
    gnss_odom_msg->twist.twist.linear.y = vel_ned.v[0];
    gnss_odom_msg->twist.twist.linear.z = -vel_ned.v[2];
  }
  else
  {
    gnss_odom_msg->twist.twist.linear.x = vel_ned.v[0];
    gnss_odom_msg->twist.twist.linear.y = vel_ned.v[1];
    gnss_odom_msg->twist.twist.linear.z = vel_ned.v[2];
  }
  gnss_odom_msg->twist.covariance[0] = vel_ned.speed_accuracy;
  gnss_odom_msg->twist.covariance[7] = vel_ned.speed_accuracy;
  gnss_odom_msg->twist.covariance[14] = vel_ned.speed_accuracy;
}

void Publishers::handleGnssFixInfo(const mip::data_gnss::FixInfo& fix_info, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Different message depending on the descriptor set
  const uint8_t gnss_index = (descriptor_set == mip::data_gnss::DESCRIPTOR_SET || descriptor_set == mip::data_gnss::MIP_GNSS1_DATA_DESC_SET) ? GNSS1_ID : GNSS2_ID;
  auto gnss_fix_info_msg = gnss_fix_info_pub_[gnss_index]->getMessageToUpdate();
  gnss_fix_info_msg->fix_type = static_cast<uint8_t>(fix_info.fix_type);
  gnss_fix_info_msg->num_sv = fix_info.num_sv;
  gnss_fix_info_msg->sbas_used = fix_info.fix_flags & mip::data_gnss::FixInfo::FixFlags::SBAS_USED;
  gnss_fix_info_msg->dngss_used = fix_info.fix_flags & mip::data_gnss::FixInfo::FixFlags::DGNSS_USED;
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
  auto rf_error_detection_msg = gnss_rf_error_detection_pub_[gnss_index]->getMessageToUpdate();
  rf_error_detection_msg->rf_band = static_cast<uint8_t>(rf_error_detection.rf_band);
  rf_error_detection_msg->jamming_state = static_cast<uint8_t>(rf_error_detection.jamming_state);
  rf_error_detection_msg->spoofing_state = static_cast<uint8_t>(rf_error_detection.spoofing_state);
  rf_error_detection_msg->valid_flags = rf_error_detection.valid_flags;
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
  auto sbas_info_msg = gnss_sbas_info_pub_[gnss_index]->getMessageToUpdate();
  sbas_info_msg->gps_tow = sbas_info.time_of_week;
  sbas_info_msg->gps_week_number = sbas_info.week_number;
  sbas_info_msg->sbas_system = static_cast<uint8_t>(sbas_info.sbas_system);
  sbas_info_msg->sbas_id = sbas_info.sbas_id;
  sbas_info_msg->count = sbas_info.count;

  sbas_info_msg->status_range_available = sbas_info.sbas_status.rangeAvailable();
  sbas_info_msg->status_corrections_available = sbas_info.sbas_status.correctionsAvailable();
  sbas_info_msg->status_integrity_available = sbas_info.sbas_status.integrityAvailable();
  sbas_info_msg->status_test_mode = sbas_info.sbas_status.testMode();

  sbas_info_msg->valid_flags = sbas_info.valid_flags;
}

void Publishers::handleRtkCorrectionsStatus(const mip::data_gnss::RtkCorrectionsStatus& rtk_corrections_status, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  const uint8_t dongle_version = rtk_corrections_status.dongle_status & mip::commands_rtk::GetStatusFlags::StatusFlags::VERSION;
  switch (dongle_version)
  {
    // V1 dongle
    case 0:
    {
      auto rtk_msg_v1 = rtk_pub_v1_->getMessageToUpdate();
      rtk_msg_v1->raw_status_flags = rtk_corrections_status.dongle_status;
      rtk_msg_v1->dongle_version = dongle_version;
      rtk_msg_v1->dongle_controller_state = rtk_corrections_status.dongle_status & mip::commands_rtk::GetStatusFlags::StatusFlagsLegacy::CONTROLLERSTATE;
      rtk_msg_v1->dongle_platform_state = rtk_corrections_status.dongle_status & mip::commands_rtk::GetStatusFlags::StatusFlagsLegacy::PLATFORMSTATE;
      rtk_msg_v1->dongle_controller_status = rtk_corrections_status.dongle_status & mip::commands_rtk::GetStatusFlags::StatusFlagsLegacy::CONTROLLERSTATUSCODE;
      rtk_msg_v1->dongle_platform_status = rtk_corrections_status.dongle_status & mip::commands_rtk::GetStatusFlags::StatusFlagsLegacy::PLATFORMSTATUSCODE;
      rtk_msg_v1->dongle_reset_reason = rtk_corrections_status.dongle_status & mip::commands_rtk::GetStatusFlags::StatusFlagsLegacy::RESETCODE;
      rtk_msg_v1->dongle_signal_quality = rtk_corrections_status.dongle_status & mip::commands_rtk::GetStatusFlags::StatusFlagsLegacy::SIGNALQUALITY;
      break;
    }
    // V2 dongle
    default:
    {
      auto rtk_msg = rtk_pub_->getMessageToUpdate();
      rtk_msg->raw_status_flags = rtk_corrections_status.dongle_status;
      rtk_msg->dongle_version = dongle_version;
      rtk_msg->dongle_modem_state = rtk_corrections_status.dongle_status & mip::commands_rtk::GetStatusFlags::StatusFlags::MODEM_STATE;
      rtk_msg->dongle_connection_type = rtk_corrections_status.dongle_status & mip::commands_rtk::GetStatusFlags::StatusFlags::CONNECTION_TYPE;
      rtk_msg->dongle_rssi = -rtk_corrections_status.dongle_status & mip::commands_rtk::GetStatusFlags::StatusFlags::RSSI;
      rtk_msg->dongle_signal_quality = rtk_corrections_status.dongle_status & mip::commands_rtk::GetStatusFlags::StatusFlags::SIGNAL_QUALITY;
      rtk_msg->dongle_tower_change_indicator = rtk_corrections_status.dongle_status & mip::commands_rtk::GetStatusFlags::StatusFlags::TOWER_CHANGE_INDICATOR;
      rtk_msg->dongle_nmea_timeout = rtk_corrections_status.dongle_status & mip::commands_rtk::GetStatusFlags::StatusFlags::NMEA_TIMEOUT;
      rtk_msg->dongle_out_of_range = rtk_corrections_status.dongle_status & mip::commands_rtk::GetStatusFlags::StatusFlags::DEVICE_OUT_OF_RANGE;
      rtk_msg->dongle_corrections_unavailable = rtk_corrections_status.dongle_status & mip::commands_rtk::GetStatusFlags::StatusFlags::CORRECTIONS_UNAVAILABLE;

      rtk_msg->gps_tow = rtk_corrections_status.time_of_week;
      rtk_msg->gps_week = rtk_corrections_status.week_number;
      rtk_msg->epoch_status = rtk_corrections_status.epoch_status;
      rtk_msg->gps_correction_latency = rtk_corrections_status.gps_correction_latency;
      rtk_msg->glonass_correction_latency = rtk_corrections_status.glonass_correction_latency;
      rtk_msg->galileo_correction_latency = rtk_corrections_status.galileo_correction_latency;
      rtk_msg->beidou_correction_latency = rtk_corrections_status.beidou_correction_latency;
      break;
    }
  }
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
  auto filter_status_msg = filter_status_pub_->getMessageToUpdate();
  filter_status_msg->filter_state = static_cast<uint16_t>(status.filter_state);
  filter_status_msg->dynamics_mode = static_cast<uint16_t>(status.dynamics_mode);
  filter_status_msg->status_flags = status.status_flags;
}

void Publishers::handleFilterEulerAngles(const mip::data_filter::EulerAngles& euler_angles, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Save the data after converting it to the proper frame
  float filter_yaw;
  if (config_->use_enu_frame_)
  {
    filter_yaw = M_PI / 2.0 - euler_angles.yaw;
    if (filter_yaw > M_PI)
      filter_yaw -= 2.0 * M_PI;
    else if (filter_yaw < -M_PI)
      filter_yaw += 2.0 * M_PI;
  }
  else
  {
    filter_yaw = euler_angles.yaw;
  }

  // Filter heading message
  auto filter_heading_msg = filter_heading_pub_->getMessageToUpdate();
  filter_heading_msg->heading_deg = filter_yaw * 180.0 / M_PI;
  filter_heading_msg->heading_rad = filter_yaw;
  filter_heading_msg->status_flags = euler_angles.valid_flags;
}

void Publishers::handleFilterHeadingUpdateState(const mip::data_filter::HeadingUpdateState& heading_update_state, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto filter_heading_state_msg = filter_heading_state_pub_->getMessageToUpdate();
  filter_heading_state_msg->heading_rad = heading_update_state.heading;
  filter_heading_state_msg->heading_uncertainty = heading_update_state.heading_1sigma;
  filter_heading_state_msg->source = static_cast<uint8_t>(heading_update_state.source);
  filter_heading_state_msg->status_flags = heading_update_state.valid_flags;
}

void Publishers::handleFilterPositionLlh(const mip::data_filter::PositionLlh& position_llh, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto filter_odom_msg = filter_odom_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_odom_msg->header), descriptor_set, timestamp);
  if (config_->use_enu_frame_)
  {
    filter_odom_msg->pose.pose.position.x = position_llh.longitude;
    filter_odom_msg->pose.pose.position.y = position_llh.latitude;
  }
  else
  {
    filter_odom_msg->pose.pose.position.x = position_llh.latitude;
    filter_odom_msg->pose.pose.position.y = position_llh.longitude;
  }
  filter_odom_msg->pose.pose.position.z = position_llh.ellipsoid_height;
}

void Publishers::handleFilterPositionLlhUncertainty(const mip::data_filter::PositionLlhUncertainty& position_llh_uncertainty, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Filter odometry message
  auto filter_odom_msg = filter_odom_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_odom_msg->header), descriptor_set, timestamp);
  if (config_->use_enu_frame_)
  {
    filter_odom_msg->pose.covariance[0] = pow(position_llh_uncertainty.east, 2);
    filter_odom_msg->pose.covariance[7] = pow(position_llh_uncertainty.north, 2);
  }
  else
  {
    filter_odom_msg->pose.covariance[0] = pow(position_llh_uncertainty.north, 2);
    filter_odom_msg->pose.covariance[7] = pow(position_llh_uncertainty.east, 2);
  }
  filter_odom_msg->pose.covariance[14] = pow(position_llh_uncertainty.down, 2);

  // Filter relative odometry message
  auto filter_relative_odom_msg = filter_relative_odom_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_relative_odom_msg->header), descriptor_set, timestamp);
  filter_relative_odom_msg->pose.covariance = filter_odom_msg->pose.covariance;
}

void Publishers::handleFilterAttitudeQuaternion(const mip::data_filter::AttitudeQuaternion& attitude_quaternion, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Filter odometry message
  auto filter_odom_msg = filter_odom_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_odom_msg->header), descriptor_set, timestamp);
  if (config_->use_enu_frame_)
  {
    tf2::Quaternion q_ned2enu, q_body2enu, q_vehiclebody2sensorbody, q_body2ned(attitude_quaternion.q[1], attitude_quaternion.q[2], attitude_quaternion.q[3], attitude_quaternion.q[0]);

    config_->t_ned2enu_.getRotation(q_ned2enu);
    config_->t_vehiclebody2sensorbody_.getRotation(q_vehiclebody2sensorbody);

    q_body2enu = q_ned2enu * q_body2ned * q_vehiclebody2sensorbody;

    filter_odom_msg->pose.pose.orientation = tf2::toMsg(q_body2enu);
  }
  else
  {
    filter_odom_msg->pose.pose.orientation.x = attitude_quaternion.q[1];
    filter_odom_msg->pose.pose.orientation.y = attitude_quaternion.q[2];
    filter_odom_msg->pose.pose.orientation.z = attitude_quaternion.q[3];
    filter_odom_msg->pose.pose.orientation.w = attitude_quaternion.q[0];
  }

  // Filtered IMU message
  auto filter_imu_msg = filter_imu_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_imu_msg->header), descriptor_set, timestamp);
  filter_imu_msg->orientation = filter_odom_msg->pose.pose.orientation;

  // Filter relative odometry message
  auto filter_relative_odom_msg = filter_relative_odom_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_relative_odom_msg->header), descriptor_set, timestamp);
  filter_relative_odom_msg->pose.pose.orientation = filter_odom_msg->pose.pose.orientation;

  // Relative transform
  updateHeaderTime(&(filter_relative_transform_msg_.header), descriptor_set, timestamp);
  filter_relative_transform_msg_.transform.rotation = filter_odom_msg->pose.pose.orientation;

  // Save the quaternion for later
  filter_attitude_quaternion_ = tf2::Quaternion(attitude_quaternion.q[1], attitude_quaternion.q[2], attitude_quaternion.q[3], attitude_quaternion.q[0]);
}

void Publishers::handleFilterEulerAnglesUncertainty(const mip::data_filter::EulerAnglesUncertainty& euler_angles_uncertainty, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Filter odometry message
  auto filter_odom_msg = filter_odom_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_odom_msg->header), descriptor_set, timestamp);
  filter_odom_msg->pose.covariance[21] = pow(euler_angles_uncertainty.roll, 2);
  filter_odom_msg->pose.covariance[28] = pow(euler_angles_uncertainty.pitch, 2);
  filter_odom_msg->pose.covariance[35] = pow(euler_angles_uncertainty.yaw, 2);

  // Filtered IMU message
  auto filter_imu_msg = filter_imu_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_imu_msg->header), descriptor_set, timestamp);
  filter_imu_msg->orientation_covariance[0] = filter_odom_msg->pose.covariance[21];
  filter_imu_msg->orientation_covariance[4] = filter_odom_msg->pose.covariance[28];
  filter_imu_msg->orientation_covariance[8] = filter_odom_msg->pose.covariance[35];

  // Filter relative odometry message
  auto filter_relative_odom_msg = filter_relative_odom_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_relative_transform_msg_.header), descriptor_set, timestamp);
  filter_relative_odom_msg->pose.covariance = filter_odom_msg->pose.covariance;
}

void Publishers::handleFilterVelocityNed(const mip::data_filter::VelocityNed& velocity_ned, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Filter odometry message
  auto filter_odom_msg = filter_odom_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_odom_msg->header), descriptor_set, timestamp);
  if (config_->use_enu_frame_)
  {
    filter_odom_msg->twist.twist.linear.x = velocity_ned.east;
    filter_odom_msg->twist.twist.linear.y = velocity_ned.north;
    filter_odom_msg->twist.twist.linear.z = -velocity_ned.down;
  }
  else
  {
    filter_odom_msg->twist.twist.linear.x = velocity_ned.north;
    filter_odom_msg->twist.twist.linear.y = velocity_ned.east;
    filter_odom_msg->twist.twist.linear.z = velocity_ned.down;
  }

  // If we are publishing velocity in the vehicle frame, rotate the velocity using the attitude
  if (config_->filter_vel_in_vehicle_frame_)
  {
    const tf2::Vector3 filter_current_vel(filter_odom_msg->twist.twist.linear.x, filter_odom_msg->twist.twist.linear.y, filter_odom_msg->twist.twist.linear.z);
    const tf2::Vector3 filter_rotated_vel = tf2::quatRotate(filter_attitude_quaternion_.inverse(), filter_current_vel);
    filter_odom_msg->twist.twist.linear.x = filter_rotated_vel.getX();
    filter_odom_msg->twist.twist.linear.y = filter_rotated_vel.getY();
    filter_odom_msg->twist.twist.linear.z = filter_rotated_vel.getZ();
  }

  // Filter relative odometry message
  auto filter_relative_odom_msg = filter_relative_odom_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_relative_transform_msg_.header), descriptor_set, timestamp);
  filter_relative_odom_msg->twist.twist.linear = filter_odom_msg->twist.twist.linear;
}

void Publishers::handleFilterVelocityNedUncertainty(const mip::data_filter::VelocityNedUncertainty& velocity_ned_uncertainty, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Filter odometry message
  auto filter_odom_msg = filter_odom_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_odom_msg->header), descriptor_set, timestamp);
  if (config_->use_enu_frame_)
  {
    filter_odom_msg->twist.covariance[0] = pow(velocity_ned_uncertainty.east, 2);
    filter_odom_msg->twist.covariance[7] = pow(velocity_ned_uncertainty.north, 2);
  }
  else
  {
    filter_odom_msg->twist.covariance[0] = pow(velocity_ned_uncertainty.north, 2);
    filter_odom_msg->twist.covariance[7] = pow(velocity_ned_uncertainty.east, 2);
  }
  filter_odom_msg->twist.covariance[14] = pow(velocity_ned_uncertainty.down, 2);

  // Filter relative odometry message
  auto filter_relative_odom_msg = filter_relative_odom_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_relative_transform_msg_.header), descriptor_set, timestamp);
  filter_relative_odom_msg->twist.covariance = filter_odom_msg->twist.covariance;
}

void Publishers::handleFilterCompAngularRate(const mip::data_filter::CompAngularRate& comp_angular_rate, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Filter odometry message
  auto filter_odom_msg = filter_odom_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_odom_msg->header), descriptor_set, timestamp);
  filter_odom_msg->twist.twist.angular.x = comp_angular_rate.gyro[0];
  filter_odom_msg->twist.twist.angular.y = comp_angular_rate.gyro[1];
  filter_odom_msg->twist.twist.angular.z = comp_angular_rate.gyro[2];
  if (config_->use_enu_frame_)
  {
    filter_odom_msg->twist.twist.angular.y *= -1.0;
    filter_odom_msg->twist.twist.angular.z *= -1.0;
  }

  // Filtered IMU message
  auto filter_imu_msg = filter_imu_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_imu_msg->header), descriptor_set, timestamp);
  filter_imu_msg->angular_velocity = filter_odom_msg->twist.twist.angular;

  // Filter relative odometry message
  auto filter_relative_odom_msg = filter_relative_odom_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_relative_transform_msg_.header), descriptor_set, timestamp);
  filter_relative_odom_msg->twist.twist.angular = filter_odom_msg->twist.twist.angular;
}

void Publishers::handleFilterCompAccel(const mip::data_filter::CompAccel& comp_accel, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  if (config_->filter_use_compensated_accel_)
  {
    auto filter_imu_msg = filter_imu_pub_->getMessageToUpdate();
    updateHeaderTime(&(filter_imu_msg->header), descriptor_set, timestamp);
    filter_imu_msg->linear_acceleration.x = comp_accel.accel[0];
    filter_imu_msg->linear_acceleration.y = comp_accel.accel[1];
    filter_imu_msg->linear_acceleration.z = comp_accel.accel[2];
    if (config_->use_enu_frame_)
    {
      filter_imu_msg->linear_acceleration.y *= -1.0;
      filter_imu_msg->linear_acceleration.z *= -1.0;
    }
  }
}

void Publishers::handleFilterLinearAccel(const mip::data_filter::LinearAccel& linear_accel, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  if (!config_->filter_use_compensated_accel_)
  {
    auto filter_imu_msg = filter_imu_pub_->getMessageToUpdate();
    updateHeaderTime(&(filter_imu_msg->header), descriptor_set, timestamp);
    filter_imu_msg->linear_acceleration.x = linear_accel.accel[0];
    filter_imu_msg->linear_acceleration.y = linear_accel.accel[1];
    filter_imu_msg->linear_acceleration.z = linear_accel.accel[2];
    if (config_->use_enu_frame_)
    {
      filter_imu_msg->linear_acceleration.y *= -1.0;
      filter_imu_msg->linear_acceleration.z *= -1.0;
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
}

void Publishers::handleFilterGnssPosAidStatus(const mip::data_filter::GnssPosAidStatus& gnss_pos_aid_status, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  const uint8_t gnss_index = gnss_pos_aid_status.receiver_id - 1;
  auto gnss_aiding_status_msg = gnss_aiding_status_pub_[gnss_index]->getMessageToUpdate();
  gnss_aiding_status_msg->gps_tow = gnss_pos_aid_status.time_of_week;
  gnss_aiding_status_msg->has_position_fix = (gnss_pos_aid_status.status & mip::data_filter::GnssAidStatusFlags::NO_FIX) == 0;
  gnss_aiding_status_msg->tight_coupling = gnss_pos_aid_status.status & mip::data_filter::GnssAidStatusFlags::TIGHT_COUPLING;
  gnss_aiding_status_msg->differential_corrections = gnss_pos_aid_status.status & mip::data_filter::GnssAidStatusFlags::DIFFERENTIAL;
  gnss_aiding_status_msg->integer_fix = gnss_pos_aid_status.status & mip::data_filter::GnssAidStatusFlags::INTEGER_FIX;
  gnss_aiding_status_msg->using_gps = gnss_pos_aid_status.status & mip::data_filter::GnssAidStatusFlags::GPS_L1
                                   || gnss_pos_aid_status.status & mip::data_filter::GnssAidStatusFlags::GPS_L2
                                   || gnss_pos_aid_status.status & mip::data_filter::GnssAidStatusFlags::GPS_L5;
  gnss_aiding_status_msg->using_glonass = gnss_pos_aid_status.status & mip::data_filter::GnssAidStatusFlags::GLO_L1
                                       || gnss_pos_aid_status.status & mip::data_filter::GnssAidStatusFlags::GLO_L2;
  gnss_aiding_status_msg->using_galileo = gnss_pos_aid_status.status & mip::data_filter::GnssAidStatusFlags::GAL_E1
                                       || gnss_pos_aid_status.status & mip::data_filter::GnssAidStatusFlags::GAL_E5
                                       || gnss_pos_aid_status.status & mip::data_filter::GnssAidStatusFlags::GAL_E6;
  gnss_aiding_status_msg->using_beidou = gnss_pos_aid_status.status & mip::data_filter::GnssAidStatusFlags::BEI_B1
                                      || gnss_pos_aid_status.status & mip::data_filter::GnssAidStatusFlags::BEI_B2
                                      || gnss_pos_aid_status.status & mip::data_filter::GnssAidStatusFlags::BEI_B3;
}

void Publishers::handleFilterGnssDualAntennaStatus(const mip::data_filter::GnssDualAntennaStatus& gnss_dual_antenna_status, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto gnss_dual_antenna_status_msg = gnss_dual_antenna_status_pub_->getMessageToUpdate();
  gnss_dual_antenna_status_msg->gps_tow = gnss_dual_antenna_status.time_of_week;
  gnss_dual_antenna_status_msg->heading = gnss_dual_antenna_status.heading;
  gnss_dual_antenna_status_msg->heading_uncertainty = gnss_dual_antenna_status.heading_unc;
  gnss_dual_antenna_status_msg->fix_type = static_cast<uint8_t>(gnss_dual_antenna_status.fix_type);
  gnss_dual_antenna_status_msg->rcv_1_valid = gnss_dual_antenna_status.status_flags & mip::data_filter::GnssDualAntennaStatus::DualAntennaStatusFlags::RCV_1_DATA_VALID;
  gnss_dual_antenna_status_msg->rcv_2_valid = gnss_dual_antenna_status.status_flags & mip::data_filter::GnssDualAntennaStatus::DualAntennaStatusFlags::RCV_2_DATA_VALID;
  gnss_dual_antenna_status_msg->antenna_offsets_valid = gnss_dual_antenna_status.status_flags & mip::data_filter::GnssDualAntennaStatus::DualAntennaStatusFlags::ANTENNA_OFFSETS_VALID;
}

void Publishers::handleFilterAidingMeasurementSummary(const mip::data_filter::AidingMeasurementSummary& aiding_measurement_summary, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  FilterAidingMeasurementSummaryIndicatorMsg* indicator = nullptr;
  auto filter_aiding_measurement_summary_msg = filter_aiding_mesaurement_summary_pub_->getMessageToUpdate();
  switch (aiding_measurement_summary.type)
  {
    case mip::data_filter::FilterAidingMeasurementType::GNSS:
      if (aiding_measurement_summary.source == GNSS1_ID)
        indicator = &(filter_aiding_measurement_summary_msg->gnss1);
      else if (aiding_measurement_summary.source == GNSS2_ID)
        indicator = &(filter_aiding_measurement_summary_msg->gnss2);
      break;
    case mip::data_filter::FilterAidingMeasurementType::DUAL_ANTENNA:
      indicator = &(filter_aiding_measurement_summary_msg->dual_antenna);
      break;
    case mip::data_filter::FilterAidingMeasurementType::HEADING:
      indicator = &(filter_aiding_measurement_summary_msg->heading);
      break;
    case mip::data_filter::FilterAidingMeasurementType::PRESSURE:
      indicator = &(filter_aiding_measurement_summary_msg->pressure);
      break;
    case mip::data_filter::FilterAidingMeasurementType::MAGNETOMETER:
      indicator = &(filter_aiding_measurement_summary_msg->magnetometer);
      break;
    case mip::data_filter::FilterAidingMeasurementType::SPEED:
      indicator = &(filter_aiding_measurement_summary_msg->speed);
      break;
  }

  if (indicator != nullptr)
  {
    indicator->enabled = aiding_measurement_summary.indicator & mip::data_filter::FilterMeasurementIndicator::ENABLED;
    indicator->used = aiding_measurement_summary.indicator & mip::data_filter::FilterMeasurementIndicator::USED;
    indicator->residual_high_warning = aiding_measurement_summary.indicator & mip::data_filter::FilterMeasurementIndicator::RESIDUAL_HIGH_WARNING;
    indicator->sample_time_warning = aiding_measurement_summary.indicator & mip::data_filter::FilterMeasurementIndicator::SAMPLE_TIME_WARNING;
    indicator->configuration_error = aiding_measurement_summary.indicator & mip::data_filter::FilterMeasurementIndicator::CONFIGURATION_ERROR;
    indicator->max_num_meas_exceeded = aiding_measurement_summary.indicator & mip::data_filter::FilterMeasurementIndicator::MAX_NUM_MEAS_EXCEEDED;
  }
}

void Publishers::updateHeaderTime(RosHeaderType* header, uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // If we are using device timestamp, we should only assign a value if we have actually received a timestamp
  if (config_->use_device_timestamp_)
  {
    if (gps_timestamp_mapping_.find(descriptor_set) != gps_timestamp_mapping_.end())
    {
      // Convert the GPS time to UTC
      setGpsTime(&header->stamp, gps_timestamp_mapping_[descriptor_set]);
    }
  }
  else if (config_->use_ros_time_)
  {
    header->stamp = rosTimeNow(node_);
  }
  else
  {
    setRosTime(&header->stamp, timestamp / 1000, (timestamp % 1000) * 1000);
  }
}

void Publishers::setGpsTime(RosTimeType* time, const mip::data_shared::GpsTimestamp& timestamp)
{
  // Split the seconds and subseconds out to get around the double resolution issue
  double seconds;
  double subseconds = modf(timestamp.tow, &seconds);

  // Seconds since start of Unix time = seconds between 1970 and 1980 + number of weeks since 1980 * number of seconds in a week + number of complete seconds past in current week - leap seconds since start of GPS time
  const uint64_t utc_milliseconds = static_cast<uint64_t>((315964800 + timestamp.week_number * 604800 + static_cast<uint64_t>(seconds) - 18) * 1000L) + static_cast<uint64_t>(std::round(subseconds * 1000.0));
  setRosTime(time, utc_milliseconds / 1000, (utc_milliseconds % 1000) * 1000);
}

}  // namespace microstrain
