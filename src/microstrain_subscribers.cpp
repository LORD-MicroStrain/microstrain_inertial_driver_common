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
#include "microstrain_subscribers.h"


namespace Microstrain
{

MicrostrainSubscribers::MicrostrainSubscribers(RosNodeType* node, MicrostrainConfig* config) : m_node(node), m_config(config)
{}

bool MicrostrainSubscribers::configure_subscribers()
{
  //Clear the ZUPT listener flags
  // TODO: Should these be moved to this class?
  m_vel_still = false;
  m_ang_still = false;
  
  //Create a topic listener for ZUPTs
  if(m_config->m_velocity_zupt == 1 && m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_CMDED_ZERO_VEL_UPDATE))
  {
    m_filter_vel_state_sub = create_subscriber<BoolMsg>(m_node, m_config->m_velocity_zupt_topic, 1000, &MicrostrainSubscribers::velocity_zupt_callback, this);
  }
  
  //Create a topic listener for angular ZUPTs
  if(m_config->m_angular_zupt == 1 && m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_CMDED_ZERO_ANG_RATE_UPDATE))
  {
    m_filter_ang_state_sub = create_subscriber<>(m_node, m_config->m_angular_zupt_topic.c_str(), 1000, &MicrostrainSubscribers::ang_zupt_callback, this);
  }
  
  //Create a topic listener for external GNSS updates
  if(m_config->m_filter_enable_external_gps_time_update && m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_GPS_TIME_UPDATE))
  {
    m_external_gps_time_sub = create_subscriber<>(m_node, m_config->m_external_gps_time_topic.c_str(), 1000, &MicrostrainSubscribers::external_gps_time_callback, this);
  }
  return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Velocity ZUPT Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////
void MicrostrainSubscribers::velocity_zupt_callback(const BoolMsg& state)
{
  if(m_vel_still != state.data) 
  {
    m_vel_still = state.data;

    if(m_vel_still) 
    {
      m_vel_zupt_timer = create_timer<MicrostrainSubscribers>(m_node, 5, &MicrostrainSubscribers::vel_zupt, this);
    }
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Velocity ZUPT Subtask
//
// Note: Handles sending the ZUPT command regularly while stationary
/////////////////////////////////////////////////////////////////////////////////////////////////////
void MicrostrainSubscribers::vel_zupt()
{
  if (!m_vel_still)
  {
    stop_timer(m_vel_zupt_timer);
    return;
  }

  if(m_config->m_inertial_device && m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_CMDED_ZERO_VEL_UPDATE))
  {
    try
    {
      MICROSTRAIN_INFO(m_node, "Sending velzupt");
      m_config->m_inertial_device->cmdedVelZUPT();
    }
    catch (mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Angular Rate ZUPT Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////
void MicrostrainSubscribers::ang_zupt_callback(const BoolMsg& state)
{
  if(m_ang_still != state.data) 
  {
    m_ang_still = state.data;

    if(m_ang_still) 
    {
      m_ang_zupt_timer = create_timer<MicrostrainSubscribers>(m_node, 5, &MicrostrainSubscribers::ang_zupt, this);
    }
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Angular Rate ZUPT Subtask
//
// Note: Handles sending the ZUPT command regularly while not rotating
/////////////////////////////////////////////////////////////////////////////////////////////////////
void MicrostrainSubscribers::ang_zupt()
{
  if (!m_ang_still)
  {
    stop_timer(m_ang_zupt_timer);
    return;
  }

  if(m_config->m_inertial_device && m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_CMDED_ZERO_VEL_UPDATE))
  {
    try
    {
      m_config->m_inertial_device->cmdedAngRateZUPT();
    }
    catch (mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// External GPS Time Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////
void MicrostrainSubscribers::external_gps_time_callback(const TimeReferenceMsg& time)
{
  // TODO: Figure out the toSec function in ROS2
  if(m_config->m_inertial_device)
  {
    try
    {
      long utcTime = get_time_ref_sec(time.time_ref) + m_config->m_gps_leap_seconds - UTC_GPS_EPOCH_DUR;

      long secs = utcTime % (int)SECS_PER_WEEK;

      int weeks = (utcTime - secs)/SECS_PER_WEEK;


      m_config->m_inertial_device->setGPSTimeUpdate(mscl::MipTypes::TimeFrame::TIME_FRAME_WEEKS, weeks);
      m_config->m_inertial_device->setGPSTimeUpdate(mscl::MipTypes::TimeFrame::TIME_FRAME_SECONDS, secs);

      MICROSTRAIN_INFO(m_node, "GPS Update: w%i, s%ld", weeks, secs);
    }
    catch(mscl::Error &e)
    {
      MICROSTRAIN_ERROR(m_node, "Error: %s", e.what());
    }
  }
}

} // namespace Microstrain
