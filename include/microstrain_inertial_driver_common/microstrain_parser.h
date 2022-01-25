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
#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_MICROSTRAIN_PARSER_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_MICROSTRAIN_PARSER_H

#include "microstrain_inertial_driver_common/microstrain_defs.h"
#include "microstrain_inertial_driver_common/microstrain_ros_funcs.h"
#include "microstrain_inertial_driver_common/microstrain_config.h"
#include "microstrain_inertial_driver_common/microstrain_publishers.h"

namespace microstrain
{

/**
 * Contains parsing code that will parse messages read from the device and publish them
 */
class MicrostrainParser
{
public:
  /**
   * \brief Default Constructor
   */
  MicrostrainParser() = default;

  /**
   * \brief Constructs the object with a reference to the node, config and publishers
   * \param node  Reference to a node that will be saved to this class and used to log and interact with ROS
   * \param config Reference to the config object that will be saved to this class and used to interact with the device
   * \param publishers Reference to the publishers object that will be saved to this class and used to publish parsed messages to the network
   */
  MicrostrainParser(RosNodeType* node, MicrostrainConfig* config, MicrostrainPublishers* publishers);

  /**
   * \brief Highest level parsing function that will be called on each packet received from the device.
   * \param packet The mip packet to parse
   */
  void parseMIPPacket(const mscl::MipDataPacket& packet);

  /**
   * \brief Parsing function that will parse the strings returned by the aux port of a GQ7
   * \param aux_string The string read from the aux port 
   */
  void parseAuxString(const std::string& aux_string);

private:
  /**
   * \brief Gets the packet timestamp from a MIP packet depending on whether we want to use the collected timestamp or device timestamp
   * \param packet The MIP packet containing a collected timestamp and possibly a device timestamp
   * \return ROS time type populated with the packet timestamp
   */
  RosTimeType getPacketTimestamp(const mscl::MipDataPacket& packet) const;

  /**
   * \brief Parses IMU packets if the MIP packet was an IMU packet and saves the data into a ROS message that will be published when it is filled out
   * \param packet the IMU packet to parse
   */
  void parseIMUPacket(const mscl::MipDataPacket& packet);

  /**
   * \brief Parses Filter packets if the MIP packet was a Filter packet and saves the data into a ROS message that will be published when it is filled out
   * \param packet the IMU packet to parse
   */
  void parseFilterPacket(const mscl::MipDataPacket& packet);

  /**
   * \brief Parses GNSS packets if the MIP packet was a GNSS packet and saves the data into a ROS message that will be published when it is filled out
   * \param packet the IMU packet to parse
   */
  void parseGNSSPacket(const mscl::MipDataPacket& packet, int gnss_id);

  /**
   * \brief Parses RTK packets if the MIP packet was a RTK packet and saves the data into a ROS message that will be published when it is filled out
   * \param packet the IMU packet to parse
   */
  void parseRTKPacket(const mscl::MipDataPacket& packet);

  /**
   * \brief Prints packet stats
   */
  void printPacketStats();

  RosNodeType* node_;
  MicrostrainConfig* config_;
  MicrostrainPublishers* publishers_;

  uint32_t imu_valid_packet_count_;
  uint32_t gnss_valid_packet_count_[NUM_GNSS];
  uint32_t filter_valid_packet_count_;
  uint32_t rtk_valid_packet_count_;

  uint32_t imu_timeout_packet_count_;
  uint32_t gnss_timeout_packet_count_[NUM_GNSS];
  uint32_t filter_timeout_packet_count_;

  uint32_t imu_checksum_error_packet_count_;
  uint32_t gnss_checksum_error_packet_count_[NUM_GNSS];
  uint32_t filter_checksum_error_packet_count_;

  // Data field storage  // D
  // IMU
  float curr_imu_mag_x_;
  float curr_imu_mag_y_;
  float curr_imu_mag_z_;

  mscl::Vector curr_ahrs_quaternion_;

  double curr_filter_pos_lat_;
  double curr_filter_pos_long_;
  double curr_filter_pos_height_;

  float curr_filter_vel_north_;
  float curr_filter_vel_east_;
  float curr_filter_vel_down_;

  mscl::Vector curr_filter_quaternion_;

  float curr_filter_roll_;
  float curr_filter_pitch_;
  float curr_filter_yaw_;

  float curr_filter_angular_rate_x_;
  float curr_filter_angular_rate_y_;
  float curr_filter_angular_rate_z_;

  float curr_filter_pos_uncert_north_;
  float curr_filter_pos_uncert_east_;
  float curr_filter_pos_uncert_down_;

  float curr_filter_vel_uncert_north_;
  float curr_filter_vel_uncert_east_;
  float curr_filter_vel_uncert_down_;

  float curr_filter_att_uncert_roll_;
  float curr_filter_att_uncert_pitch_;
  float curr_filter_att_uncert_yaw_;
};  // struct MicrostrainParser

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_MICROSTRAIN_PARSER_H
