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

#ifndef ROS_MSCL_COMMON_MICROSTRAIN_NODE_BASE_H
#define ROS_MSCL_COMMON_MICROSTRAIN_NODE_BASE_H

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Include Files
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#include <stddef.h>
#include <string>
#include <vector>
#include <fstream>

#include "ros_mscl_common/microstrain_config.h"
#include "ros_mscl_common/microstrain_publishers.h"
#include "ros_mscl_common/microstrain_subscribers.h"
#include "ros_mscl_common/microstrain_services.h"
#include "ros_mscl_common/microstrain_parser.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief Contains functions for micostrain driver
///
/////////////////////////////////////////////////////////////////////////////////////////////////////

namespace microstrain
{
///
/// \brief Contains configuration information
///
class MicrostrainNodeBase
{
public:
  void parse_and_publish();

protected:
  MicrostrainNodeBase() = default;

  bool initialize(RosNodeType* init_node);
  bool configure(RosNodeType* config_node);

  RosNodeType* node_;
  MicrostrainConfig config_;
  MicrostrainPublishers publishers_;
  MicrostrainSubscribers subscribers_;
  MicrostrainServices services_;
  MicrostrainParser parser_;

  double timer_update_rate_hz_;
  int32_t status_counter_;
};  // MicrostrainNodeBase class

}  // namespace microstrain

#endif  // ROS_MSCL_COMMON_MICROSTRAIN_NODE_BASE_H
