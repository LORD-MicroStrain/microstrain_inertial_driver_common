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


#ifndef _MICROSTRAIN_NODE_BASE_H
#define _MICROSTRAIN_NODE_BASE_H

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Include Files
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#include <stddef.h>
#include <string>
#include <vector>
#include <fstream>

#include "microstrain_config.h"
#include "microstrain_publishers.h"
#include "microstrain_subscribers.h"
#include "microstrain_services.h"
#include "microstrain_parser.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief Contains functions for micostrain driver
///
/////////////////////////////////////////////////////////////////////////////////////////////////////

namespace Microstrain
{

///
/// \brief Contains configuration information
///
class MicrostrainNodeBase
{
 // TODO: Figure out if we can make this protected
 public:
  MicrostrainNodeBase() = default;

  bool initialize(RosNodeType* init_node);
  bool configure(RosNodeType* config_node);
  void parse_and_publish();

  RosNodeType* m_node;
  MicrostrainConfig m_config;
  MicrostrainPublishers m_publishers;
  MicrostrainSubscribers m_subscribers;
  MicrostrainServices m_services;
  MicrostrainParser m_parser;

  double m_timer_update_rate_hz;
  int32_t m_status_counter;
};  // MicrostrainNodeBase class

} // namespace Microstrain

#endif  // _MICROSTRAIN_NODE_BASE_H
