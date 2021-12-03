/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Inertial Device Driver Implementation File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2020, Parker Hannifin Corp
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Include Files
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#include "microstrain_inertial_driver_common/microstrain_subscribers.h"

namespace microstrain
{

constexpr auto UTC_GPS_EPOCH_DUR = (315964800);
constexpr auto SECS_PER_WEEK = (60L * 60 * 24 * 7);

MicrostrainSubscribers::MicrostrainSubscribers(RosNodeType* node, MicrostrainConfig* config)
  : node_(node), config_(config)
{
}

bool MicrostrainSubscribers::activate()
{
  // Clear the ZUPT listener flags
  vel_still_ = false;
  ang_still_ = false;

  // Create a topic listener for ZUPTs
  if (config_->velocity_zupt_ == 1 &&
      config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_CMDED_ZERO_VEL_UPDATE))
  {
    filter_vel_state_sub_ = create_subscriber<BoolMsg>(node_, config_->velocity_zupt_topic_, 1000,
                                                        &MicrostrainSubscribers::velZuptCallback, this);
  }

  // Create a topic listener for angular ZUPTs
  if (config_->angular_zupt_ == 1 && config_->inertial_device_->features().supportsCommand(
                                           mscl::MipTypes::Command::CMD_EF_CMDED_ZERO_ANG_RATE_UPDATE))
  {
    filter_ang_state_sub_ = create_subscriber<>(node_, config_->angular_zupt_topic_.c_str(), 1000,
                                                 &MicrostrainSubscribers::angZuptCallback, this);
  }

  // Create a topic listener for external GNSS updates
  if (config_->filter_enable_external_gps_time_update_ &&
      config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_GPS_TIME_UPDATE))
  {
    MICROSTRAIN_INFO(node_, "Subscribed to %s for external GPS time", config_->external_gps_time_topic_.c_str());
    external_gps_time_sub_ = create_subscriber<>(node_, config_->external_gps_time_topic_.c_str(), 1000,
                                                  &MicrostrainSubscribers::externalGpsTimeCallback, this);
  }

  // Create a topic listener for external RTCM updates
  if (config_->subscribe_rtcm_ && config_->supports_rtk_)
  {
    MICROSTRAIN_INFO(node_, "Subscribed to %s for RTCM corrections", config_->rtcm_topic_.c_str());
    rtcm_sub_ = create_subscriber<>(node_, config_->rtcm_topic_.c_str(), 1000,
                                    &MicrostrainSubscribers::rtcmCallback, this);
  }

  // Create a topic listener for external speed updates
  if (config_->filter_enable_odometer_aiding_ &&
      config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_EXTERN_SPEED_UPDATE))
  {
    if (!config_->enable_hardware_odometer_)
    {
      external_speed_sub_ = create_subscriber<>(node_, config_->external_speed_topic_.c_str(), 1000,
                                                &MicrostrainSubscribers::externalSpeedCallback, this);
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
void MicrostrainSubscribers::velZuptCallback(const BoolMsg& state)
{
  if (vel_still_ != state.data)
  {
    vel_still_ = state.data;

    if (vel_still_)
    {
      vel_zupt_timer_ = create_timer<MicrostrainSubscribers>(node_, 5, &MicrostrainSubscribers::velZupt, this);
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Velocity ZUPT Subtask
//
// Note: Handles sending the ZUPT command regularly while stationary
/////////////////////////////////////////////////////////////////////////////////////////////////////
void MicrostrainSubscribers::velZupt()
{
  if (!vel_still_)
  {
    stop_timer(vel_zupt_timer_);
    return;
  }

  if (config_->inertial_device_ &&
      config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_CMDED_ZERO_VEL_UPDATE))
  {
    try
    {
      MICROSTRAIN_INFO(node_, "Sending velzupt");
      config_->inertial_device_->cmdedVelZUPT();
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Angular Rate ZUPT Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////
void MicrostrainSubscribers::angZuptCallback(const BoolMsg& state)
{
  if (ang_still_ != state.data)
  {
    ang_still_ = state.data;

    if (ang_still_)
    {
      ang_zupt_timer_ = create_timer<MicrostrainSubscribers>(node_, 5, &MicrostrainSubscribers::angZupt, this);
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Angular Rate ZUPT Subtask
//
// Note: Handles sending the ZUPT command regularly while not rotating
/////////////////////////////////////////////////////////////////////////////////////////////////////
void MicrostrainSubscribers::angZupt()
{
  if (!ang_still_)
  {
    stop_timer(ang_zupt_timer_);
    return;
  }

  if (config_->inertial_device_ &&
      config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_CMDED_ZERO_VEL_UPDATE))
  {
    try
    {
      config_->inertial_device_->cmdedAngRateZUPT();
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// External GPS Time Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////
void MicrostrainSubscribers::externalGpsTimeCallback(const TimeReferenceMsg& time)
{
  if (config_->inertial_device_)
  {
    try
    {
      int64_t utcTime = get_time_ref_sec(time.time_ref) + config_->gps_leap_seconds_ - UTC_GPS_EPOCH_DUR;

      int64_t secs = utcTime % static_cast<int32_t>(SECS_PER_WEEK);

      int weeks = (utcTime - secs) / SECS_PER_WEEK;

      config_->inertial_device_->setGPSTimeUpdate(mscl::MipTypes::TimeFrame::TIME_FRAME_WEEKS, weeks);
      config_->inertial_device_->setGPSTimeUpdate(mscl::MipTypes::TimeFrame::TIME_FRAME_SECONDS, secs);

      MICROSTRAIN_INFO(node_, "GPS Update: w%i, s%ld", weeks, secs);
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// External Speed Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////
void MicrostrainSubscribers::externalSpeedCallback(const InputSpeedMeasurementMsg& speed)
{
  if (config_->inertial_device_)
  {
    try
    {
      config_->inertial_device_->sendExternalSpeedMeasurementUpdate(speed.gps_tow, speed.speed, speed.speed_uncertainty);
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Error: %s", e.what());
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// RTCM Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////
void MicrostrainSubscribers::rtcmCallback(const RTCMMsg& rtcm)
{
  try
  {
    config_->aux_connection_->write(rtcm.data);
  }
  catch (mscl::Error& e)
  {
    MICROSTRAIN_ERROR(node_, "Error sending RTCM corrections: %s", e.what());
  }
}

}  // namespace microstrain
