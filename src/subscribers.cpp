/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Inertial Device Driver Implementation File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2020, Parker Hannifin Corp
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include "microstrain_inertial_driver_common/subscribers.h"

namespace microstrain
{

constexpr auto UTC_GPS_EPOCH_DUR = (315964800);
constexpr auto SECS_PER_WEEK = (60L * 60 * 24 * 7);

Subscribers::Subscribers(RosNodeType* node, Config* config)
  : node_(node), config_(config)
{
  // Initialize the transform buffer and listener ahead of time
  transform_buffer_ = createTransformBuffer(node_);
  transform_listener_ = createTransformListener(transform_buffer_);
}

bool Subscribers::activate()
{
  // Create a topic listener for external RTCM updates
  if (config_->ntrip_interface_enable_)
  {
    rtcm_sub_ = createSubscriber<>(node_, RTCM_TOPIC, 1000, &Subscribers::rtcmCallback, this);
  }

  // Create a topic listener for external speed updates
  if (config_->filter_enable_odometer_aiding_ && config_->mip_device_->supportsDescriptor(mip::commands_filter::DESCRIPTOR_SET, mip::commands_filter::CMD_SPEED_MEASUREMENT))
  {
    if (!config_->enable_hardware_odometer_)
    {
      external_speed_sub_ = createSubscriber<>(node_, EXT_WHEEL_SPEED_TOPIC, 1000,
                                                &Subscribers::externalSpeedCallback, this);
    }
    else
    {
      MICROSTRAIN_WARN(node_, "Note: Not enabling external speed subscriber as hardware odometer is enabled");
    }
  }

  // Setup the external measurement subscribers
  if (config_->subscribe_ext_fix_ && config_->mip_device_->supportsDescriptor(mip::commands_aiding::DESCRIPTOR_SET, mip::commands_aiding::CMD_POS_LLH))
  {
    MICROSTRAIN_INFO(node_, "Subscribing to %s for external GPS position", EXT_FIX_TOPIC);
    external_gnss_position_sub_ = createSubscriber<>(node_, EXT_FIX_TOPIC, 1000, &Subscribers::externalGnssPositionCallback, this);
  }
  if (config_->subscribe_ext_vel_ned_ && config_->mip_device_->supportsDescriptor(mip::commands_aiding::DESCRIPTOR_SET, mip::commands_aiding::CMD_VEL_NED))
  {
    MICROSTRAIN_INFO(node_, "Subscribing to %s for external GPS Velocity in the NED frame", EXT_VEL_NED_TOPIC);
    external_gnss_vel_ned_sub_ = createSubscriber<>(node_, EXT_VEL_NED_TOPIC, 1000, &Subscribers::externalGnssVelNedCallback, this);
  }
  if (config_->subscribe_ext_vel_enu_ && config_->mip_device_->supportsDescriptor(mip::commands_aiding::DESCRIPTOR_SET, mip::commands_aiding::CMD_VEL_NED))
  {
    MICROSTRAIN_INFO(node_, "Subscribing to %s for external GPS Velocity in the ENU frame", EXT_VEL_ENU_TOPIC);
    external_gnss_vel_enu_sub_ = createSubscriber<>(node_, EXT_VEL_ENU_TOPIC, 1000, &Subscribers::externalGnssVelEnuCallback, this);
  }
  if (config_->subscribe_ext_vel_ecef_ && config_->mip_device_->supportsDescriptor(mip::commands_aiding::DESCRIPTOR_SET, mip::commands_aiding::CMD_VEL_ECEF))
  {
    MICROSTRAIN_INFO(node_, "Subscribing to %s for external GPS Velocity in the ECEF frame", EXT_VEL_ECEF_TOPIC);
    external_gnss_vel_ecef_sub_ = createSubscriber<>(node_, EXT_VEL_ECEF_TOPIC, 1000, &Subscribers::externalGnssVelEcefCallback, this);
  }
  if (config_->subscribe_ext_vel_body_ && config_->mip_device_->supportsDescriptor(mip::commands_aiding::DESCRIPTOR_SET, mip::commands_aiding::CMD_VEL_ODOM))
  {
    MICROSTRAIN_INFO(node_, "Subscribing to %s for external Velocity in the Body frame", EXT_VEL_BODY_TOPIC);
    external_body_vel_sub_ = createSubscriber<>(node_, EXT_VEL_BODY_TOPIC, 1000, &Subscribers::externalBodyVelCallback, this);
  }
  if (config_->subscribe_ext_pressure_ && false)  // TODO: Check pressure descriptor
  {
    MICROSTRAIN_INFO(node_, "Subscribing to %s for external pressure", EXT_PRESSURE_TOPIC);
    external_pressure_sub_ = createSubscriber<>(node_, EXT_PRESSURE_TOPIC, 1000, &Subscribers::externalPressureCallback, this);
  }
  if (config_->subscribe_ext_pose_ && config_->mip_device_->supportsDescriptor(mip::commands_aiding::DESCRIPTOR_SET, mip::commands_aiding::CMD_DELTA_POSITION))
  {
    MICROSTRAIN_INFO(node_, "Subscribing to %s for external pose", EXT_VEL_BODY_TOPIC);
    external_pose_sub_ = createSubscriber<>(node_, EXT_POSE_TOPIC, 1000, &Subscribers::externalPoseCallback, this);
  }
  if (config_->subscribe_ext_heading_ && config_->mip_device_->supportsDescriptor(mip::commands_aiding::DESCRIPTOR_SET, mip::commands_aiding::CMD_HEADING_TRUE))
  {
    MICROSTRAIN_INFO(node_, "Subscribing to %s for external heading", EXT_HEADING_TOPIC);
    external_heading_sub_ = createSubscriber<>(node_, EXT_HEADING_TOPIC, 1000, &Subscribers::externalHeadingCallback, this);
  }

  // If any of the external subscribers were set up, configure the time callbacks
  if (config_->subscribe_ext_fix_ || config_->subscribe_ext_vel_ned_ || config_->subscribe_ext_vel_enu_ || config_->subscribe_ext_vel_ecef_ ||
      config_->subscribe_ext_vel_body_ || config_->subscribe_ext_pressure_ || config_->subscribe_ext_pose_ || config_->subscribe_ext_heading_)
  {
    if (config_->mip_device_->supportsDescriptor(mip::commands_base::DESCRIPTOR_SET, mip::commands_base::CMD_GPS_TIME_BROADCAST_NEW))
    {
      MICROSTRAIN_INFO(node_, "Subscribing to %s and %s for external timing information", EXT_TIME_GPS_TOPIC, EXT_TIME_TOPIC);
      external_time_gps_sub_ = createSubscriber<>(node_, EXT_TIME_GPS_TOPIC, 1000, &Subscribers::externalTimeGpsCallback, this);
      external_time_sub_ = createSubscriber<>(node_, EXT_TIME_TOPIC, 1000, &Subscribers::externalTimeCallback, this);
    }
  }

  return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// External Speed Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////
void Subscribers::externalSpeedCallback(const InputSpeedMeasurementMsg& speed)
{
  mip::CmdResult mip_cmd_result;
  if (!(mip_cmd_result = mip::commands_filter::speedMeasurement(*(config_->mip_device_), 1, speed.gps_tow, speed.speed, speed.speed_uncertainty)))
  {
    MICROSTRAIN_ERROR(node_, "Failed to write external speed");
    MICROSTRAIN_ERROR(node_, "Error(%d): %s", mip_cmd_result.value, mip_cmd_result.name());
  }
}

void Subscribers::externalTimeGpsCallback(const TimeReferenceMsg& time)
{
  // If this is the first message we received, set the frame ID
  if (time.header.frame_id.empty())
  {
    MICROSTRAIN_WARN_THROTTLE(node_, 2, "Invalid header.frame_id for external GPS time. frame_id must not be empty");
    return;
  }
  if (gps_time_frame_id_.empty())
  {
    gps_time_frame_id_ = time.header.frame_id;

    // Add this as a zero offset clock bias to the device list
    const RosTimeType& now = rosTimeNow(node_);
    device_time_to_ros_time_clock_bias_monitors_[time.header.frame_id] = ClockBiasMonitor(1, 1);
    device_time_to_ros_time_clock_bias_monitors_[time.header.frame_id].addTime(now, now);
  }
  
  // Add the time to the clock bias
  if (gps_time_frame_id_ == time.header.frame_id)
  {
    ros_time_to_gps_time_clock_bias_monitor_.addTime(time.header.stamp, time.time_ref);
  }
  else
  {
    // If we received a different frame_id than the first message, ignore it
    MICROSTRAIN_WARN_THROTTLE(node_, 2, "Invalid header.frame_id %s for external GPS time, we already received a GPS time with frame id %s, all messages must use that frame_id",
      time.header.frame_id.c_str(), gps_time_frame_id_.c_str());
  }

  // Set the time on the device
  const int64_t gps_time = getTimeRefSec(time.time_ref) + config_->gps_leap_seconds_ - UTC_GPS_EPOCH_DUR;
  const uint32_t gps_tow = gps_time % static_cast<int32_t>(SECS_PER_WEEK);
  const uint32_t gps_week_number = (gps_time - gps_tow) / SECS_PER_WEEK;

  mip::CmdResult mip_cmd_result;
  MICROSTRAIN_DEBUG(node_, "Updating GPS week number to %u", gps_week_number);
  if (!(mip_cmd_result = mip::commands_base::writeGpsTimeUpdate(*(config_->mip_device_), mip::commands_base::GpsTimeUpdate::FieldId::WEEK_NUMBER, gps_week_number)))
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to send GPS time update for week number");
  MICROSTRAIN_DEBUG(node_, "Updating GPS time of week to %u", gps_tow);
  if (!(mip_cmd_result = mip::commands_base::writeGpsTimeUpdate(*(config_->mip_device_), mip::commands_base::GpsTimeUpdate::FieldId::TIME_OF_WEEK, gps_tow)))
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to send GPS time update for time of week");
}

void Subscribers::externalTimeCallback(const TimeReferenceMsg& time)
{
  // If this frame ID is the same as the GPS frame ID, do nothing
  if (time.header.frame_id.empty() || time.header.frame_id == gps_time_frame_id_)
    return;

  // If we don't have an entry for this frame_id, make one
  if (device_time_to_ros_time_clock_bias_monitors_.find(time.header.frame_id) == device_time_to_ros_time_clock_bias_monitors_.end())
    device_time_to_ros_time_clock_bias_monitors_[time.header.frame_id] = ClockBiasMonitor(0.99, 2);
  
  // Add the time
  device_time_to_ros_time_clock_bias_monitors_[time.header.frame_id].addTime(time.time_ref, time.header.stamp);
}

void Subscribers::externalGnssPositionCallback(const NavSatFixMsg& fix)
{
  // Throw away messages that do not have a valid fix
  if (fix.status.status == NavSatFixMsg::_status_type::STATUS_NO_FIX)
  {
    MICROSTRAIN_WARN_THROTTLE(node_, 10, "Received LLH position with invalid fix. Ignoring");
    return;
  }

  // Fill out the time for the message
  mip::commands_aiding::LlhPos llh_pos;
  llh_pos.time.timebase = mip::commands_aiding::Time::Timebase::TIME_OF_ARRIVAL;  // TODO: This should use populateAidingTime instead
  /*
  if (!populateAidingTime(fix.header, EXT_FIX_TOPIC, &llh_pos.time))
    return;
  */

  // Get the sensor ID from the frame ID
  if ((llh_pos.sensor_id = getSensorIdFromFrameId(fix.header.frame_id)) == 0)
    return;
  
  // TODO: Don't process if status is not FIX or above

  // If the message has no uncertainty, we can't process it
  if (fix.position_covariance_type == NavSatFixMsg::COVARIANCE_TYPE_UNKNOWN)
  {
    MICROSTRAIN_WARN_THROTTLE(node_, 10, "Invalid NavSatFix message, must have uncertainty to process measurements");
    return;
  }

  // Fill out the rest of the message and send it
  llh_pos.valid_flags.setAll();
  llh_pos.latitude = fix.latitude;
  llh_pos.longitude = fix.longitude;
  llh_pos.height = fix.altitude;
  if (isNed(fix.header))
  {
    llh_pos.uncertainty[0] = sqrt(fix.position_covariance[0]);
    llh_pos.uncertainty[1] = sqrt(fix.position_covariance[4]);
  }
  else
  {
    llh_pos.uncertainty[0] = sqrt(fix.position_covariance[4]);
    llh_pos.uncertainty[1] = sqrt(fix.position_covariance[0]);
  }
  llh_pos.uncertainty[2] = sqrt(fix.position_covariance[8]);

  mip::CmdResult mip_cmd_result;
  if (!(mip_cmd_result = config_->mip_device_->device().runCommand<mip::commands_aiding::LlhPos>(llh_pos)))
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to send aiding LLH position aiding command");
}

void Subscribers::externalGnssVelNedCallback(const TwistWithCovarianceStampedMsg& gnss_vel)
{
  // Make sure the frame_id is NED
  if (!isNed(gnss_vel.header))
  {
    MICROSTRAIN_WARN_THROTTLE(node_, 10, "Invalid velocity message received on %s, the frame_id must end with _ned, but is %s", EXT_VEL_NED_TOPIC, gnss_vel.header.frame_id.c_str());
    return;
  }

  // Fill out the time for the message
  mip::commands_aiding::NedVel ned_vel;
  ned_vel.time.timebase = mip::commands_aiding::Time::Timebase::TIME_OF_ARRIVAL;  // TODO: This should use populateAidingTime instead
  /*
  if (!populateAidingTime(gnss_vel.header, EXT_VEL_NED_TOPIC, &ned_vel.time))
    return;
  */

  // Get the sensor ID from the frame ID
  if ((ned_vel.sensor_id = getSensorIdFromFrameId(gnss_vel.header.frame_id)) == 0)
    return;

  // Fill out the rest of the message and send it
  if (gnss_vel.twist.covariance[0] != 0)
  {
    ned_vel.velocity[0] = gnss_vel.twist.twist.linear.x;
    ned_vel.uncertainty[0] = sqrt(gnss_vel.twist.covariance[0]);
    ned_vel.valid_flags.x(true);
  }
  if (gnss_vel.twist.covariance[7] != 0)
  {
    ned_vel.velocity[1] = gnss_vel.twist.twist.linear.y;
    ned_vel.uncertainty[1] = sqrt(gnss_vel.twist.covariance[7]);
    ned_vel.valid_flags.y(true);
  }
  if (gnss_vel.twist.covariance[14] != 0)
  {
    ned_vel.velocity[2] = gnss_vel.twist.twist.linear.z;
    ned_vel.uncertainty[2] = sqrt(gnss_vel.twist.covariance[14]);
    ned_vel.valid_flags.z(true);
  }

  mip::CmdResult mip_cmd_result;
  if (!(mip_cmd_result = config_->mip_device_->device().runCommand<mip::commands_aiding::NedVel>(ned_vel)))
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to send NED velocity aiding command");
}

void Subscribers::externalGnssVelEnuCallback(const TwistWithCovarianceStampedMsg& gnss_vel)
{
  // Make sure the frame_id is ENU
  if (isNed(gnss_vel.header))
  {
    MICROSTRAIN_WARN_THROTTLE(node_, 10, "Invalid velocity message received on %s, the frame_id must not end with _ned, but is %s", EXT_VEL_NED_TOPIC, gnss_vel.header.frame_id.c_str());
    return;
  }

  // Fill out the time for the message
  mip::commands_aiding::NedVel ned_vel;
  ned_vel.time.timebase = mip::commands_aiding::Time::Timebase::TIME_OF_ARRIVAL;  // TODO: This should use populateAidingTime instead
  /*
  if (!populateAidingTime(gnss_vel.header, EXT_VEL_NED_TOPIC, &ned_vel.time))
    return;
  */

  // Get the sensor ID from the frame ID
  if ((ned_vel.sensor_id = getSensorIdFromFrameId(gnss_vel.header.frame_id)) == 0)
    return;

  // Fill out the rest of the message and send it
  if (gnss_vel.twist.covariance[7] != 0)
  {
    ned_vel.velocity[0] = gnss_vel.twist.twist.linear.y;
    ned_vel.uncertainty[0] = sqrt(gnss_vel.twist.covariance[7]);
    ned_vel.valid_flags.x(true);
  }
  if (gnss_vel.twist.covariance[0] != 0)
  {
    ned_vel.velocity[1] = gnss_vel.twist.twist.linear.x;
    ned_vel.uncertainty[1] = sqrt(gnss_vel.twist.covariance[0]);
    ned_vel.valid_flags.y(true);
  }
  if (gnss_vel.twist.covariance[14] != 0)
  {
    ned_vel.velocity[2] = -gnss_vel.twist.twist.linear.z;
    ned_vel.uncertainty[2] = sqrt(gnss_vel.twist.covariance[14]);
    ned_vel.valid_flags.z(true);
  }

  mip::CmdResult mip_cmd_result;
  if (!(mip_cmd_result = config_->mip_device_->device().runCommand<mip::commands_aiding::NedVel>(ned_vel)))
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to send ENU velocity aiding command");
}

void Subscribers::externalGnssVelEcefCallback(const TwistWithCovarianceStampedMsg& gnss_ecef_vel)
{
  // Fill out the time for the message
  mip::commands_aiding::EcefVel ecef_vel;
  if (!populateAidingTime(gnss_ecef_vel.header, EXT_VEL_ECEF_TOPIC, &ecef_vel.time))
    return;
  
  // TODO: Do sensor_id stuff

  // Fill out the rest of the message and send it
  if (gnss_ecef_vel.twist.covariance[0] != 0)
  {
    ecef_vel.velocity[0] = gnss_ecef_vel.twist.twist.linear.x;
    ecef_vel.uncertainty[0] = sqrt(gnss_ecef_vel.twist.covariance[0]);
    ecef_vel.valid_flags.x(true);
  }
  if (gnss_ecef_vel.twist.covariance[7] != 0)
  {
    ecef_vel.velocity[1] = gnss_ecef_vel.twist.twist.linear.y;
    ecef_vel.uncertainty[1] = sqrt(gnss_ecef_vel.twist.covariance[7]);
    ecef_vel.valid_flags.y(true);
  }
  if (gnss_ecef_vel.twist.covariance[14] != 0)
  {
    ecef_vel.velocity[2] = gnss_ecef_vel.twist.twist.linear.z;
    ecef_vel.uncertainty[2] = sqrt(gnss_ecef_vel.twist.covariance[14]);
    ecef_vel.valid_flags.z(true);
  }

  mip::CmdResult mip_cmd_result;
  if (!(mip_cmd_result = config_->mip_device_->device().runCommand<mip::commands_aiding::EcefVel>(ecef_vel)))
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to send ECEF velocity aiding command");
}

void Subscribers::externalWheelSpeedCallback(const TwistWithCovarianceStampedMsg& wheel_speed)
{
  // TODO: No MIP command yet
}

void Subscribers::externalBodyVelCallback(const TwistWithCovarianceStampedMsg& body_vel)
{
  // Fill out the time of the message
  mip::commands_aiding::VehicleFixedFrameVelocity vehicle_fixed_frame_velocity;
  if (!populateAidingTime(body_vel.header, EXT_VEL_BODY_TOPIC, &vehicle_fixed_frame_velocity.time))
    return;
  
  // TODO: Do sensor_id stuff

  // Fill out the rest of the message and send it
  if (body_vel.twist.covariance[0] != 0)
  {
    vehicle_fixed_frame_velocity.velocity[0] = body_vel.twist.twist.linear.x;
    vehicle_fixed_frame_velocity.uncertainty[0] = sqrt(body_vel.twist.covariance[0]);
    vehicle_fixed_frame_velocity.valid_flags.x(true);
  }
  if (body_vel.twist.covariance[7] != 0)
  {
    vehicle_fixed_frame_velocity.velocity[1] = body_vel.twist.twist.linear.y;
    vehicle_fixed_frame_velocity.uncertainty[1] = sqrt(body_vel.twist.covariance[7]);
    vehicle_fixed_frame_velocity.valid_flags.y(true);
  }
  if (body_vel.twist.covariance[14] != 0)
  {
    vehicle_fixed_frame_velocity.velocity[2] = body_vel.twist.twist.linear.z;
    vehicle_fixed_frame_velocity.uncertainty[2] = sqrt(body_vel.twist.covariance[14]);
    vehicle_fixed_frame_velocity.valid_flags.z(true);
  }

  mip::CmdResult mip_cmd_result;
  if (!(mip_cmd_result = config_->mip_device_->device().runCommand<mip::commands_aiding::VehicleFixedFrameVelocity>(vehicle_fixed_frame_velocity)))
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to send body frame velocity command");
}

void Subscribers::externalPressureCallback(const FluidPressureMsg& pressure)
{
  // TODO: No MIP command yet
}

void Subscribers::externalPoseCallback(const PoseWithCovarianceStampedMsg& pose)
{
  // TODO: No MIP command yet
}

void Subscribers::externalHeadingCallback(const DualAntennaHeadingMsg& heading)
{
  // Fill out the time of the message
  mip::commands_aiding::TrueHeading true_heading;
  if (!populateAidingTime(heading.header, EXT_HEADING_TOPIC, &true_heading.time))
    return;
  
  // TODO: Do sensor_id stuff

  // Fill out the rest of the message and send it
  // TODO: Do I need to rotate this if it is ENU?
  true_heading.valid_flags = 0xFFFF;
  true_heading.heading = heading.heading;
  true_heading.uncertainty = heading.uncertainty;

  mip::CmdResult mip_cmd_result;
  if (!(mip_cmd_result = config_->mip_device_->device().runCommand<mip::commands_aiding::TrueHeading>(true_heading)))
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to send external heading command");
}

void Subscribers::rtcmCallback(const RTCMMsg& rtcm)
{
  MICROSTRAIN_DEBUG(node_, "Received RTCM message of size %lu", rtcm.data.size());
  if (config_->aux_device_)
  {
    if (!config_->aux_device_->send(rtcm.data.data(), rtcm.data.size()))
      MICROSTRAIN_ERROR(node_, "Failed to write RTCM to device");
    else
      MICROSTRAIN_DEBUG(node_, "Successfully wrote RTCM message of size %lu to aux port", rtcm.data.size());
  }
  else
  {
    MICROSTRAIN_WARN(node_, "Note: Device did not configure the aux port. See startup logs for why it was not configured");
  }
}

bool Subscribers::populateAidingTime(const RosHeaderType& header, const std::string& topic, mip::commands_aiding::Time* time)
{
  // Make sure we can do all the required lookups
  if (header.frame_id.empty())
  {
    MICROSTRAIN_WARN_THROTTLE(node_, 2, "Invalid message received on %s, must have header.frame_id set", topic.c_str());
    return false;
  }
  else if (device_time_to_ros_time_clock_bias_monitors_.find(header.frame_id) == device_time_to_ros_time_clock_bias_monitors_.end() ||
           !device_time_to_ros_time_clock_bias_monitors_[header.frame_id].hasBiasEstimate())
  {
    MICROSTRAIN_WARN_THROTTLE(node_, 10, "Invalid message received on %s, we do not have a clock bias for frame_id %s", topic.c_str(), header.frame_id.c_str());
    MICROSTRAIN_WARN_THROTTLE(node_, 10, "  Please ensure that you are publishing a time reference to %s for this frame_id", EXT_TIME_TOPIC);
    MICROSTRAIN_WARN_THROTTLE(node_, 10, "  If this frame_id is perfectly synced with ROS time, add it to the ext_frame_ids_no_clock_bias config parameter");
    return false;
  }
  else if (!ros_time_to_gps_time_clock_bias_monitor_.hasBiasEstimate())
  {
    MICROSTRAIN_WARN_THROTTLE(node_, 10, "Invalid message received on %s, we do not have a clock bias for ROS time to GPS time", topic.c_str());
    MICROSTRAIN_WARN_THROTTLE(node_, 10, "  Ensure that you are publishing a time reference to %s", EXT_TIME_GPS_TOPIC);
    return false;
  }

  // Make the conversion - device_time -> ros_time -> gps_time
  const double time_secs = getTimeRefSecs(header.stamp) + device_time_to_ros_time_clock_bias_monitors_.at(header.frame_id).getBiasEstimate() + ros_time_to_gps_time_clock_bias_monitor_.getBiasEstimate();
  time->timebase = mip::commands_aiding::Time::Timebase::INTERNAL_REFERENCE;
  time->nanoseconds = static_cast<uint64_t>(time_secs * 1000000000);
  return true;
}

uint8_t Subscribers::getSensorIdFromFrameId(const std::string& frame_id)
{
  const auto& frame_id_iter = std::find(external_frame_ids_.begin(), external_frame_ids_.end(), frame_id);
  if (frame_id_iter != external_frame_ids_.end())
  {
    // Looks like we already configured this frame_id, so just grab the index
    return frame_id_iter - external_frame_ids_.begin();
  }
  else
  {
    // If we have reached 256 IDs, we can't do anything
    if (external_frame_ids_size_ >= config_->mip_device_->max_external_frame_ids_)
    {
      MICROSTRAIN_WARN_THROTTLE(node_, 10, "%u Sensor IDs have already been configured, we will not be able to configure %s", config_->mip_device_->max_external_frame_ids_, frame_id.c_str());
      return 0;
    }
    else
    {
      // Attempt to find the transform from ROS
      std::string tf_error_string;
      RosTimeType frame_time; setRosTime(&frame_time, 0, 0);
      if (transform_buffer_->canTransform(frame_id, config_->frame_id_, frame_time, RosDurationType(0, 0), &tf_error_string))
      {
        // Populate the reference frame command from the transform
        const auto& transform = transform_buffer_->lookupTransform(frame_id, config_->frame_id_, frame_time);
        mip::commands_aiding::ReferenceFrame reference_frame;
        reference_frame.format = mip::commands_aiding::ReferenceFrame::Format::QUATERNION;
        reference_frame.frame_id = external_frame_ids_size_ + 1;
        reference_frame.function = mip::FunctionSelector::WRITE;
        reference_frame.translation = {
          static_cast<float>(transform.transform.translation.x),
          static_cast<float>(transform.transform.translation.y),
          static_cast<float>(transform.transform.translation.z)
        };
        reference_frame.rotation = {
          static_cast<float>(transform.transform.rotation.w),
          static_cast<float>(transform.transform.rotation.x),
          static_cast<float>(transform.transform.rotation.y),
          static_cast<float>(transform.transform.rotation.z)
        };

        // Attempt to send the command
        mip::CmdResult mip_cmd_result;
        if (!(mip_cmd_result = config_->mip_device_->device().runCommand<mip::commands_aiding::ReferenceFrame>(reference_frame)))
        {
          MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to send aiding LLH position aiding command");
          return 0;
        }

        // Looks like everything is set up, append the frame ID and return it's index
        external_frame_ids_[++external_frame_ids_size_] = frame_id;
        return external_frame_ids_size_;
      }
      else
      {
        MICROSTRAIN_WARN_THROTTLE(node_, 10, "Unable to determine transform from %s to %s: %s", config_->frame_id_.c_str(), frame_id.c_str(), tf_error_string.c_str());
        return 0;
      }
    }
  }
}

}  // namespace microstrain
