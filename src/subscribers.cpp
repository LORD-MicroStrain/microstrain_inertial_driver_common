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
}

bool Subscribers::activate()
{
  // Clear the ZUPT listener flags
  vel_still_ = false;
  ang_still_ = false;

  // Create a topic listener for ZUPTs
  if (config_->velocity_zupt_ && config_->mip_device_->supportsDescriptor(mip::commands_filter::DESCRIPTOR_SET, mip::commands_filter::CMD_COMMANDED_ZUPT))
    filter_vel_state_sub_ = createSubscriber<>(node_, config_->velocity_zupt_topic_, 1000, &Subscribers::velZuptCallback, this);

  // Create a topic listener for angular ZUPTs
  if (config_->angular_zupt_ && config_->mip_device_->supportsDescriptor(mip::commands_filter::DESCRIPTOR_SET, mip::commands_filter::CMD_COMMANDED_ANGULAR_ZUPT))
    filter_ang_state_sub_ = createSubscriber<>(node_, config_->angular_zupt_topic_.c_str(), 1000, &Subscribers::angZuptCallback, this);

  // Create a topic listener for external GNSS updates
  if (config_->filter_enable_external_gps_time_update_ && config_->mip_device_->supportsDescriptor(mip::commands_base::DESCRIPTOR_SET, mip::commands_base::CMD_GPS_TIME_BROADCAST_NEW))
  {
    MICROSTRAIN_INFO(node_, "Subscribed to %s for external GPS time", config_->external_gps_time_topic_.c_str());
    external_gps_time_sub_ = createSubscriber<>(node_, config_->external_gps_time_topic_.c_str(), 1000, &Subscribers::externalGpsTimeCallback, this);
  }
  if (config_->mip_device_->supportsDescriptor(mip::commands_filter::DESCRIPTOR_SET, mip::commands_filter::CMD_EXTERNAL_GNSS_UPDATE))
  {
    if (config_->filter_enable_external_gps_position_update_)
    {
      MICROSTRAIN_INFO(node_, "Subscribed to %s for external GPS position", config_->external_gps_position_topic_.c_str());
      external_gps_position_sub_ = createSubscriber<>(node_, config_->external_gps_position_topic_.c_str(), 1000, &Subscribers::externalGpsPositionCallback, this);
    }
    if (config_->filter_enable_external_gps_speed_update_)
    {
      MICROSTRAIN_INFO(node_, "Subscribed to %s for external GPS velocity", config_->external_gps_speed_topic_.c_str());
      external_gps_speed_sub_ = createSubscriber<>(node_, config_->external_gps_speed_topic_.c_str(), 1000, &Subscribers::externalGpsSpeedCallback, this);
    }
  }

  // Create a topic listener for external RTCM updates
  if (config_->subscribe_rtcm_)
  {
    MICROSTRAIN_INFO(node_, "Subscribed to %s for RTCM corrections", config_->rtcm_topic_.c_str());
    rtcm_sub_ = createSubscriber<>(node_, config_->rtcm_topic_.c_str(), 1000, &Subscribers::rtcmCallback, this);
  }

  // Create a topic listener for external speed updates
  if (config_->filter_enable_odometer_aiding_ && config_->mip_device_->supportsDescriptor(mip::commands_filter::DESCRIPTOR_SET, mip::commands_filter::CMD_SPEED_MEASUREMENT))
  {
    if (!config_->enable_hardware_odometer_)
    {
      external_speed_sub_ = createSubscriber<>(node_, config_->external_speed_topic_.c_str(), 1000,
                                                &Subscribers::externalSpeedCallback, this);
    }
    else
    {
      MICROSTRAIN_WARN(node_, "Note: Not enabling external speed subscriber as hardware odometer is enabled");
    }
  }

  return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Velocity ZUPT Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////
void Subscribers::velZuptCallback(const BoolMsg& state)
{
  if (vel_still_ != state.data)
  {
    vel_still_ = state.data;

    if (vel_still_)
    {
      vel_zupt_timer_ = createTimer<Subscribers>(node_, 5, &Subscribers::velZupt, this);
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Velocity ZUPT Subtask
//
// Note: Handles sending the ZUPT command regularly while stationary
/////////////////////////////////////////////////////////////////////////////////////////////////////
void Subscribers::velZupt()
{
  if (!vel_still_)
  {
    stopTimer(vel_zupt_timer_);
    return;
  }

  mip::CmdResult mip_cmd_result;
  MICROSTRAIN_DEBUG(node_, "Sending Vel ZUPT");
  if (!(mip_cmd_result = mip::commands_filter::commandedZupt(*(config_->mip_device_))))
  {
    MICROSTRAIN_ERROR(node_, "Failed to send angular ZUPT");
    MICROSTRAIN_ERROR(node_, "Error(%d): %s", mip_cmd_result.value, mip_cmd_result.name());
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Angular Rate ZUPT Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////
void Subscribers::angZuptCallback(const BoolMsg& state)
{
  if (ang_still_ != state.data)
  {
    ang_still_ = state.data;

    if (ang_still_)
    {
      ang_zupt_timer_ = createTimer<Subscribers>(node_, 5, &Subscribers::angZupt, this);
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Angular Rate ZUPT Subtask
//
// Note: Handles sending the ZUPT command regularly while not rotating
/////////////////////////////////////////////////////////////////////////////////////////////////////
void Subscribers::angZupt()
{
  if (!ang_still_)
  {
    stopTimer(ang_zupt_timer_);
    return;
  }

  mip::CmdResult mip_cmd_result;
  MICROSTRAIN_DEBUG(node_, "Sending Angular ZUPT");
  if (!(mip_cmd_result = mip::commands_filter::commandedAngularZupt(*(config_->mip_device_))))
  {
    MICROSTRAIN_ERROR(node_, "Failed to send angular ZUPT");
    MICROSTRAIN_ERROR(node_, "Error(%d): %s", mip_cmd_result.value, mip_cmd_result.name());
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// External GPS Time Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////
void Subscribers::externalGpsTimeCallback(const TimeReferenceMsg& time)
{
  int64_t utcTime = getTimeRefSec(time.time_ref) + config_->gps_leap_seconds_ - UTC_GPS_EPOCH_DUR;

  int64_t secs = utcTime % static_cast<int32_t>(SECS_PER_WEEK);

  int weeks = (utcTime - secs) / SECS_PER_WEEK;

  mip::CmdResult mip_cmd_result;
  MICROSTRAIN_INFO(node_, "GPS Update: w%i, s%ld", weeks, secs);
  if (!(mip_cmd_result = mip::commands_base::writeGpsTimeUpdate(*(config_->mip_device_), mip::commands_base::GpsTimeUpdate::FieldId::WEEK_NUMBER, weeks)))
  {
    MICROSTRAIN_ERROR(node_, "Failed to send GPS time update for week number");
    MICROSTRAIN_ERROR(node_, "Error(%d): %s", mip_cmd_result.value, mip_cmd_result.name());
  }
  if (!(mip_cmd_result = mip::commands_base::writeGpsTimeUpdate(*(config_->mip_device_), mip::commands_base::GpsTimeUpdate::FieldId::TIME_OF_WEEK, secs)))
  {
    MICROSTRAIN_ERROR(node_, "Failed to send GPS time update for time of week");
    MICROSTRAIN_ERROR(node_, "Error(%d): %s", mip_cmd_result.value, mip_cmd_result.name());
  }
}

void Subscribers::externalGpsPositionCallback(const NavSatFixMsg& position)
{
  const double utc_time = position.header.stamp.sec + (position.header.stamp.nanosec / 1000000000);
  const double gps_time = utc_time + GPS_LEAP_SECONDS - 315964800;

  // Fill in the information we know
  mip::commands_filter::ExternalGnssUpdate external_gnss_update;
  external_gnss_update.gps_week = floor(gps_time / 604800);
  external_gnss_update.gps_time = std::fmod(gps_time, 604800);
  external_gnss_update.latitude = position.latitude;
  external_gnss_update.longitude = position.longitude;
  external_gnss_update.height = position.altitude;
  external_gnss_update.pos_uncertainty[0] = std::sqrt(position.position_covariance[0]);
  external_gnss_update.pos_uncertainty[1] = std::sqrt(position.position_covariance[4]);
  external_gnss_update.pos_uncertainty[2] = std::sqrt(position.position_covariance[8]);

  // Set the uncertainty for the values we don't know very high so they will be ignored
  external_gnss_update.vel_uncertainty[0] = std::numeric_limits<float>::max();
  external_gnss_update.vel_uncertainty[1] = std::numeric_limits<float>::max();
  external_gnss_update.vel_uncertainty[2] = std::numeric_limits<float>::max();
}

void Subscribers::externalGpsSpeedCallback(const TwistWithCovarianceStampedMsg& speed)
{
  const double utc_time = speed.header.stamp.sec + (speed.header.stamp.nanosec / 1000000000);
  const double gps_time = utc_time + GPS_LEAP_SECONDS - 315964800;

  // Fill in the information we know
  mip::commands_filter::ExternalGnssUpdate external_gnss_update;
  external_gnss_update.gps_week = floor(gps_time / 604800);
  external_gnss_update.gps_time = std::fmod(gps_time, 604800);
  external_gnss_update.velocity[0] = speed.twist.twist.linear.x;
  external_gnss_update.velocity[1] = speed.twist.twist.linear.y;
  external_gnss_update.velocity[2] = speed.twist.twist.linear.z;
  external_gnss_update.vel_uncertainty[0] = std::sqrt(speed.twist.covariance[0]);
  external_gnss_update.vel_uncertainty[1] = std::sqrt(speed.twist.covariance[7]);
  external_gnss_update.vel_uncertainty[2] = std::sqrt(speed.twist.covariance[14]);

  // Set the uncertainty for the values we don't know very high so they will be ignored
  external_gnss_update.pos_uncertainty[0] = std::numeric_limits<float>::max();
  external_gnss_update.pos_uncertainty[1] = std::numeric_limits<float>::max();
  external_gnss_update.pos_uncertainty[2] = std::numeric_limits<float>::max();
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

/////////////////////////////////////////////////////////////////////////////////////////////////////
// RTCM Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////
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

}  // namespace microstrain
