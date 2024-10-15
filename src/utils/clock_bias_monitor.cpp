/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Inertial Device Driver Implementation File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2020, Parker Hannifin Corp
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include "microstrain_inertial_driver_common/utils/clock_bias_monitor.h"

namespace microstrain
{

ClockBiasMonitor::ClockBiasMonitor(const double weight, const double max_bias_estimate, const size_t source_time_average_size)
  : weight_(weight), max_bias_estimate_(max_bias_estimate), source_time_average_size_(source_time_average_size)
{
}

void ClockBiasMonitor::addTime(const double source_time, const double target_time)
{
  const double delta_time = source_time - target_time;

  // Check if initialization is required
  if (!have_bias_estimate_)
  {
    bias_estimate_ = delta_time;
    have_bias_estimate_ = true;
    return;
  }

  // Check for outliers or time jumps
  const double delta_from_previous_bias = std::fabs(delta_time - bias_estimate_);
  if (delta_from_previous_bias > max_bias_estimate_)
  {
    bias_estimate_ = delta_time;
    return;
  }

  // Apply a low pass filter
  bias_estimate_ = bias_estimate_ * weight_ + delta_time * (1 - weight_);
}

bool ClockBiasMonitor::hasBiasEstimate() const
{
  return have_bias_estimate_;
}

double ClockBiasMonitor::getBiasEstimate() const
{
  return bias_estimate_;
}

}  // namespace microstrain
