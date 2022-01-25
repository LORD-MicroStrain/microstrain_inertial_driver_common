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
#include <algorithm>
#include "microstrain_inertial_driver_common/microstrain_parser.h"

namespace microstrain
{

constexpr auto USTRAIN_G =
    9.80665;  // from section 5.1.1 in
              // https://www.microstrain.com/sites/default/files/3dm-gx5-25_dcp_manual_8500-0065_reference_document.pdf

MicrostrainParser::MicrostrainParser(RosNodeType* node, MicrostrainConfig* config, MicrostrainPublishers* publishers)
  : node_(node), config_(config), publishers_(publishers)
{
}

void MicrostrainParser::parseMIPPacket(const mscl::MipDataPacket& packet)
{
  switch (packet.descriptorSet())
  {
    case mscl::MipTypes::DataClass::CLASS_AHRS_IMU:
      parseIMUPacket(packet);
      printPacketStats();
      break;

    case mscl::MipTypes::DataClass::CLASS_ESTFILTER:
      parseFilterPacket(packet);
      printPacketStats();
      break;

    case mscl::MipTypes::DataClass::CLASS_GNSS:
    case mscl::MipTypes::DataClass::CLASS_GNSS1:
      parseGNSSPacket(packet, GNSS1_ID);
      printPacketStats();
      break;

    case mscl::MipTypes::DataClass::CLASS_GNSS2:
      parseGNSSPacket(packet, GNSS2_ID);
      printPacketStats();
      break;

    case mscl::MipTypes::DataClass::CLASS_GNSS3:
      parseRTKPacket(packet);
      printPacketStats();
      break;

    default:
      break;
  }
}

void MicrostrainParser::parseAuxString(const std::string& aux_string)
{
  // Each string may have more than one NMEA message
  size_t search_index = 0;
  while (search_index < aux_string.size())
  {
    // If we can't find a $, there are no more NMEA sentences, so exit early
    const size_t nmea_start_index = aux_string.find('$', search_index);
    if (nmea_start_index == std::string::npos)
    {
      break;
    }

    // Make sure that what follows the dollar sign is a string that is 5 characters long and a comma
    const size_t first_comma_index = aux_string.find(',', nmea_start_index + 1);
    if (first_comma_index == std::string::npos || first_comma_index - nmea_start_index > 6)
    {
      // This is either an invalid NMEA message, or a MIP packet, either way skip it
      search_index++;
      continue;
    }

    // Search for the end of the NMEA string
    const size_t nmea_end_index = aux_string.find("\r\n", nmea_start_index + 1) + 1;
    if (nmea_end_index == std::string::npos)
    {
      MICROSTRAIN_WARN(node_, "Malformed NMEA sentence received. Ignoring sentence");
      break;
    }

    // If there is another $ between the first $ and the end string, the first $ might have been part of a MIP message, so start over at the second $
    const size_t possible_mid_index = aux_string.find('$', nmea_start_index + 1);
    if (possible_mid_index != std::string::npos && possible_mid_index < nmea_end_index)
    {
      search_index = possible_mid_index;
      continue;
    }

    // Get the NMEA substring, and update the index for the next iteration
    const std::string& nmea_sentence = aux_string.substr(nmea_start_index, (nmea_end_index - nmea_start_index) + 1);
    search_index = nmea_end_index + 1;

    // Publish the NMEA sentence to ROS
    publishers_->nmea_sentence_msg_.header.stamp = ros_time_now(node_);
    publishers_->nmea_sentence_msg_.header.frame_id = config_->nmea_frame_id_;
    publishers_->nmea_sentence_msg_.sentence = nmea_sentence;
    publishers_->nmea_sentence_pub_->publish(publishers_->nmea_sentence_msg_);
  }
}

RosTimeType MicrostrainParser::getPacketTimestamp(const mscl::MipDataPacket& packet) const
{
  if (packet.hasDeviceTime() && config_->use_device_timestamp_)
  {
    return to_ros_time(packet.deviceTimestamp().nanoseconds());
  }
  else
  {
    return to_ros_time(packet.collectedTimestamp().nanoseconds());
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// MIP IMU Packet Parsing Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

void MicrostrainParser::parseIMUPacket(const mscl::MipDataPacket& packet)
{
  // Update the diagnostics
  imu_valid_packet_count_++;

  // Handle time
  const RosTimeType packet_time = getPacketTimestamp(packet);

  // IMU timestamp
  set_seq(&publishers_->imu_msg_.header, imu_valid_packet_count_);
  publishers_->imu_msg_.header.stamp = config_->use_ros_time_ ? ros_time_now(node_) : packet_time;
  publishers_->imu_msg_.header.frame_id = config_->imu_frame_id_;

  // Magnetometer timestamp
  publishers_->mag_msg_.header = publishers_->imu_msg_.header;

  // GPS correlation timestamp headder
  publishers_->gps_corr_msg_.header = publishers_->imu_msg_.header;

  // Data present flags
  bool has_accel = false;
  bool has_gyro = false;
  bool has_quat = false;
  bool has_mag = false;

  // Get the list of data elements
  const mscl::MipDataPoints& points = packet.data();

  // Loop over the data elements and map them
  for (auto point_iter = points.begin(); point_iter != points.end(); point_iter++)
  {
    auto point = *point_iter;
    switch (point.field())
    {
      // Scaled Accel
      case mscl::MipTypes::CH_FIELD_SENSOR_SCALED_ACCEL_VEC:
      {
        has_accel = true;

        // Stuff into ROS message - acceleration in m/s^2
        if (point.qualifier() == mscl::MipTypes::CH_X)
        {
          publishers_->imu_msg_.linear_acceleration.x = USTRAIN_G * point.as_float();
        }
        else if (point.qualifier() == mscl::MipTypes::CH_Y)
        {
          publishers_->imu_msg_.linear_acceleration.y = USTRAIN_G * point.as_float();

          if (config_->use_enu_frame_)
            publishers_->imu_msg_.linear_acceleration.y *= -1.0;
        }
        else if (point.qualifier() == mscl::MipTypes::CH_Z)
        {
          publishers_->imu_msg_.linear_acceleration.z = USTRAIN_G * point.as_float();

          if (config_->use_enu_frame_)
            publishers_->imu_msg_.linear_acceleration.z *= -1.0;
        }
      }
      break;

      // Scaled Gyro
      case mscl::MipTypes::CH_FIELD_SENSOR_SCALED_GYRO_VEC:
      {
        has_gyro = true;

        if (point.qualifier() == mscl::MipTypes::CH_X)
        {
          publishers_->imu_msg_.angular_velocity.x = point.as_float();
        }
        else if (point.qualifier() == mscl::MipTypes::CH_Y)
        {
          publishers_->imu_msg_.angular_velocity.y = point.as_float();

          if (config_->use_enu_frame_)
            publishers_->imu_msg_.angular_velocity.y *= -1.0;
        }
        else if (point.qualifier() == mscl::MipTypes::CH_Z)
        {
          publishers_->imu_msg_.angular_velocity.z = point.as_float();

          if (config_->use_enu_frame_)
            publishers_->imu_msg_.angular_velocity.z *= -1.0;
        }
      }
      break;

      // Scaled Mag
      case mscl::MipTypes::CH_FIELD_SENSOR_SCALED_MAG_VEC:
      {
        has_mag = true;

        if (point.qualifier() == mscl::MipTypes::CH_X)
        {
          curr_imu_mag_x_ = point.as_float();
          publishers_->mag_msg_.magnetic_field.x = curr_imu_mag_x_;
        }
        else if (point.qualifier() == mscl::MipTypes::CH_Y)
        {
          curr_imu_mag_y_ = point.as_float();

          if (config_->use_enu_frame_)
            curr_imu_mag_y_ *= -1.0;

          publishers_->mag_msg_.magnetic_field.y = curr_imu_mag_y_;
        }
        else if (point.qualifier() == mscl::MipTypes::CH_Z)
        {
          curr_imu_mag_z_ = point.as_float();

          if (config_->use_enu_frame_)
            curr_imu_mag_z_ *= -1.0;

          publishers_->mag_msg_.magnetic_field.z = curr_imu_mag_z_;
        }
      }
      break;

      // Orientation Quaternion
      case mscl::MipTypes::CH_FIELD_SENSOR_ORIENTATION_QUATERNION:
      {
        has_quat = true;

        if (point.qualifier() == mscl::MipTypes::CH_QUATERNION)
        {
          mscl::Vector quaternion = point.as_Vector();
          curr_filter_quaternion_ = quaternion;

          if (config_->use_enu_frame_)
          {
            tf2::Quaternion q_ned2enu, q_body2enu, q_vehiclebody2sensorbody,
                            qbody2ned(quaternion.as_floatAt(1), quaternion.as_floatAt(2),
                                      quaternion.as_floatAt(3), quaternion.as_floatAt(0));

            config_->t_ned2enu_.getRotation(q_ned2enu);
            config_->t_vehiclebody2sensorbody_.getRotation(q_vehiclebody2sensorbody);

            q_body2enu = q_ned2enu*qbody2ned*q_vehiclebody2sensorbody;

            publishers_->imu_msg_.orientation = tf2::toMsg(q_body2enu);
          }
          else
          {
            publishers_->imu_msg_.orientation.x = quaternion.as_floatAt(1);
            publishers_->imu_msg_.orientation.y = quaternion.as_floatAt(2);
            publishers_->imu_msg_.orientation.z = quaternion.as_floatAt(3);
            publishers_->imu_msg_.orientation.w = quaternion.as_floatAt(0);
          }
        }
      }
      break;

      // GPS Corr
      case mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_GPS_CORRELATION_TIMESTAMP:
      {
        // for some reason point.qualifier() == mscl::MipTypes::CH_WEEK_NUMBER and
        // point.qualifier() == mscl::MipTypes::CH_FLAGS always returned false so I used
        // an iterator and manually incremented it to access all the elements
        publishers_->gps_corr_msg_.gps_cor.gps_tow = point_iter->as_double();
        point_iter++;
        publishers_->gps_corr_msg_.gps_cor.gps_week_number = point_iter->as_uint16();
        point_iter++;
        publishers_->gps_corr_msg_.gps_cor.timestamp_flags = point_iter->as_uint16();
      }
      break;
    }
  }

  if (has_accel)
  {
    // Since the sensor does not produce a covariance for linear acceleration, set it based on our pulled in parameters.
    std::copy(config_->imu_linear_cov_.begin(), config_->imu_linear_cov_.end(),
              publishers_->imu_msg_.linear_acceleration_covariance.begin());
  }

  if (has_gyro)
  {
    // Since the sensor does not produce a covariance for angular velocity, set it based on our pulled in parameters.
    std::copy(config_->imu_angular_cov_.begin(), config_->imu_angular_cov_.end(),
              publishers_->imu_msg_.angular_velocity_covariance.begin());
  }

  if (has_quat)
  {
    // Since the MIP_AHRS data does not contain uncertainty values we have to set them based on the parameter values.
    std::copy(config_->imu_orientation_cov_.begin(), config_->imu_orientation_cov_.end(),
              publishers_->imu_msg_.orientation_covariance.begin());
  }

  // Publish
  if (config_->publish_gps_corr_)
    publishers_->gps_corr_pub_->publish(publishers_->gps_corr_msg_);

  if (config_->publish_imu_)
  {
    publishers_->imu_pub_->publish(publishers_->imu_msg_);

    if (has_mag)
      publishers_->mag_pub_->publish(publishers_->mag_msg_);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// MIP Filter Packet Parsing Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

void MicrostrainParser::parseFilterPacket(const mscl::MipDataPacket& packet)
{
  bool gnss_aiding_status_received[NUM_GNSS] = { false };
  bool gnss_dual_antenna_status_received = false;
  bool filter_aiding_measurement_summary_received = false;
  int i;

  // Update diagnostics
  filter_valid_packet_count_++;

  // Handle time
  const RosTimeType packet_time = getPacketTimestamp(packet);

  // Filtered IMU timestamp and frame
  set_seq(&publishers_->filtered_imu_msg_.header, filter_valid_packet_count_);
  publishers_->filtered_imu_msg_.header.stamp = config_->use_ros_time_ ? ros_time_now(node_) : packet_time;
  publishers_->filtered_imu_msg_.header.frame_id = config_->filter_frame_id_;

  // Nav odom timestamp and frame
  set_seq(&publishers_->filter_msg_.header, filter_valid_packet_count_);
  publishers_->filter_msg_.header.stamp = config_->use_ros_time_ ? ros_time_now(node_) : packet_time;
  publishers_->filter_msg_.header.frame_id = config_->filter_frame_id_;

  // Nav relative position odom timestamp and frame (note: Relative position frame is NED for both pos and vel)
  set_seq(&publishers_->filter_relative_pos_msg_.header, filter_valid_packet_count_);
  publishers_->filter_relative_pos_msg_.header.stamp = config_->use_ros_time_ ? ros_time_now(node_) : packet_time;
  publishers_->filter_relative_pos_msg_.header.frame_id = config_->filter_frame_id_;
  publishers_->filter_relative_pos_msg_.child_frame_id = config_->filter_child_frame_id_;
  publishers_->filter_transform_msg_.header = publishers_->filter_relative_pos_msg_.header;  // Same header for the transform
  publishers_->filter_transform_msg_.child_frame_id = publishers_->filter_relative_pos_msg_.child_frame_id;

  // Get the list of data elements
  const mscl::MipDataPoints& points = packet.data();

  // Loop over data elements and map them
  for (mscl::MipDataPoint point : points)
  {
    switch (point.field())
    {
      case mscl::MipTypes::CH_FIELD_ESTFILTER_FILTER_STATUS:
      {
        if (point.qualifier() == mscl::MipTypes::CH_FILTER_STATE)
        {
          publishers_->filter_status_msg_.filter_state = point.as_uint16();
        }
        else if (point.qualifier() == mscl::MipTypes::CH_DYNAMICS_MODE)
        {
          publishers_->filter_status_msg_.dynamics_mode = point.as_uint16();
        }
        else if (point.qualifier() == mscl::MipTypes::CH_FLAGS)
        {
          publishers_->filter_status_msg_.status_flags = point.as_uint16();
        }
      }
      break;

      // Estimated LLH Position
      case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_LLH_POS:
      {
        publishers_->filter_msg_.child_frame_id = config_->filter_child_frame_id_;

        if (point.qualifier() == mscl::MipTypes::CH_LATITUDE)
        {
          curr_filter_pos_lat_ = point.as_double();

          if (config_->use_enu_frame_)
          {
            publishers_->filter_msg_.pose.pose.position.y = curr_filter_pos_lat_;
          }
          else
          {
            publishers_->filter_msg_.pose.pose.position.x = curr_filter_pos_lat_;
          }
        }
        else if (point.qualifier() == mscl::MipTypes::CH_LONGITUDE)
        {
          curr_filter_pos_long_ = point.as_double();

          if (config_->use_enu_frame_)
          {
            publishers_->filter_msg_.pose.pose.position.x = curr_filter_pos_long_;
          }
          else
          {
            publishers_->filter_msg_.pose.pose.position.y = curr_filter_pos_long_;
          }
        }
        else if (point.qualifier() == mscl::MipTypes::CH_HEIGHT_ABOVE_ELLIPSOID)
        {
          curr_filter_pos_height_ = point.as_double();
          publishers_->filter_msg_.pose.pose.position.z = curr_filter_pos_height_;
        }
      }
      break;

      // Estimated NED Velocity
      case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_NED_VELOCITY:
      {
        if (point.qualifier() == mscl::MipTypes::CH_NORTH)
        {
          curr_filter_vel_north_ = point.as_float();

          if (config_->use_enu_frame_)
          {
            publishers_->filter_msg_.twist.twist.linear.y = curr_filter_vel_north_;
            publishers_->filter_relative_pos_msg_.twist.twist.linear.y = curr_filter_vel_north_;
          }
          else
          {
            publishers_->filter_msg_.twist.twist.linear.x = curr_filter_vel_north_;
            publishers_->filter_relative_pos_msg_.twist.twist.linear.x = curr_filter_vel_north_;
          }
        }
        else if (point.qualifier() == mscl::MipTypes::CH_EAST)
        {
          curr_filter_vel_east_ = point.as_float();

          if (config_->use_enu_frame_)
          {
            publishers_->filter_msg_.twist.twist.linear.x = curr_filter_vel_east_;
            publishers_->filter_relative_pos_msg_.twist.twist.linear.x = curr_filter_vel_east_;
          }
          else
          {
            publishers_->filter_msg_.twist.twist.linear.y = curr_filter_vel_east_;
            publishers_->filter_relative_pos_msg_.twist.twist.linear.y = curr_filter_vel_east_;
          }
        }
        else if (point.qualifier() == mscl::MipTypes::CH_DOWN)
        {
          curr_filter_vel_down_ = point.as_float();

          if (config_->use_enu_frame_)
          {
            publishers_->filter_msg_.twist.twist.linear.z = -curr_filter_vel_down_;
            publishers_->filter_relative_pos_msg_.twist.twist.linear.z = -curr_filter_vel_down_;
          }
          else
          {
            publishers_->filter_msg_.twist.twist.linear.z = curr_filter_vel_down_;
            publishers_->filter_relative_pos_msg_.twist.twist.linear.z = curr_filter_vel_down_;
          }
        }
      }
      break;

      case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_EULER:
      {
        if (point.qualifier() == mscl::MipTypes::CH_ROLL)
        {
          curr_filter_roll_ = point.as_float();
        }
        else if (point.qualifier() == mscl::MipTypes::CH_PITCH)
        {
          curr_filter_pitch_ = point.as_float();
        }
        else if (point.qualifier() == mscl::MipTypes::CH_YAW)
        {
          curr_filter_yaw_ = point.as_float();

          if (config_->use_enu_frame_)
          {
          }
          else
          {
            publishers_->filter_heading_msg_.heading_deg = curr_filter_yaw_ * 180.0 / 3.14;
            publishers_->filter_heading_msg_.heading_rad = curr_filter_yaw_;
          }
        }
        else if (point.qualifier() == mscl::MipTypes::CH_FLAGS)
        {
          publishers_->filter_heading_msg_.status_flags = point.as_uint16();
        }
      }
      break;

      case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_QUATERNION:
      {
        mscl::Vector quaternion = point.as_Vector();
        curr_filter_quaternion_ = quaternion;

        if (config_->use_enu_frame_)
        {
          tf2::Quaternion q_body2enu, q_ned2enu, q_vehiclebody2sensorbody, 
                          qbody2ned(quaternion.as_floatAt(1), quaternion.as_floatAt(2),
                                    quaternion.as_floatAt(3), quaternion.as_floatAt(0));

          config_->t_ned2enu_.getRotation(q_ned2enu);
          config_->t_vehiclebody2sensorbody_.getRotation(q_vehiclebody2sensorbody);

          q_body2enu = q_ned2enu*qbody2ned*q_vehiclebody2sensorbody;

          publishers_->filter_msg_.pose.pose.orientation = tf2::toMsg(q_body2enu);
        }
        else
        {
          publishers_->filter_msg_.pose.pose.orientation.x = quaternion.as_floatAt(1);
          publishers_->filter_msg_.pose.pose.orientation.y = quaternion.as_floatAt(2);
          publishers_->filter_msg_.pose.pose.orientation.z = quaternion.as_floatAt(3);
          publishers_->filter_msg_.pose.pose.orientation.w = quaternion.as_floatAt(0);
        }

        publishers_->filtered_imu_msg_.orientation = publishers_->filter_msg_.pose.pose.orientation;
        publishers_->filter_relative_pos_msg_.pose.pose.orientation =
            publishers_->filter_msg_.pose.pose.orientation;
        publishers_->filter_transform_msg_.transform.rotation =
            publishers_->filter_relative_pos_msg_.pose.pose.orientation;
      }
      break;

      case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ANGULAR_RATE:
      {
        if (point.qualifier() == mscl::MipTypes::CH_X)
        {
          curr_filter_angular_rate_x_ = point.as_float();

          publishers_->filter_msg_.twist.twist.angular.x = curr_filter_angular_rate_x_;
          publishers_->filtered_imu_msg_.angular_velocity.x = curr_filter_angular_rate_x_;
          publishers_->filter_relative_pos_msg_.twist.twist.angular.x = curr_filter_angular_rate_x_;
        }
        else if (point.qualifier() == mscl::MipTypes::CH_Y)
        {
          curr_filter_angular_rate_y_ = point.as_float();

          if(config_->use_enu_frame_)
            curr_filter_angular_rate_y_ *= -1.0;

          publishers_->filter_msg_.twist.twist.angular.y = curr_filter_angular_rate_y_;
          publishers_->filtered_imu_msg_.angular_velocity.y = curr_filter_angular_rate_y_;
          publishers_->filter_relative_pos_msg_.twist.twist.angular.y = curr_filter_angular_rate_y_;
        }
        else if (point.qualifier() == mscl::MipTypes::CH_Z)
        {
          curr_filter_angular_rate_z_ = point.as_float();

          if(config_->use_enu_frame_)
            curr_filter_angular_rate_z_ *= -1.0;

          publishers_->filter_msg_.twist.twist.angular.z = curr_filter_angular_rate_z_;
          publishers_->filtered_imu_msg_.angular_velocity.z = curr_filter_angular_rate_z_;
          publishers_->filter_relative_pos_msg_.twist.twist.angular.z = curr_filter_angular_rate_z_;
        }
      }
      break;

      case mscl::MipTypes::CH_FIELD_ESTFILTER_COMPENSATED_ACCEL:
      {
        if (point.qualifier() == mscl::MipTypes::CH_X)
        {
          float accel_x = point.as_float();

          publishers_->filtered_imu_msg_.linear_acceleration.x = accel_x;
        }
        else if (point.qualifier() == mscl::MipTypes::CH_Y)
        {
          float accel_y = point.as_float();

          if(config_->use_enu_frame_)
            accel_y *= -1.0;

          publishers_->filtered_imu_msg_.linear_acceleration.y = accel_y;
        }
        else if (point.qualifier() == mscl::MipTypes::CH_Z)
        {
          float accel_z = point.as_float();

          if(config_->use_enu_frame_)
            accel_z *= -1.0;

          publishers_->filtered_imu_msg_.linear_acceleration.z = accel_z;
        }
      }
      break;

      case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_LLH_UNCERT:
      {
        if (point.qualifier() == mscl::MipTypes::CH_NORTH)
        {
          curr_filter_pos_uncert_north_ = point.as_float();

          if (config_->use_enu_frame_)
          {
            publishers_->filter_msg_.pose.covariance[7] = pow(curr_filter_pos_uncert_north_, 2);
            publishers_->filter_relative_pos_msg_.pose.covariance[7] = publishers_->filter_msg_.pose.covariance[7];
          }
          else
          {
            publishers_->filter_msg_.pose.covariance[0] = pow(curr_filter_pos_uncert_north_, 2);
            publishers_->filter_relative_pos_msg_.pose.covariance[0] = publishers_->filter_msg_.pose.covariance[0];
          }
        }
        else if (point.qualifier() == mscl::MipTypes::CH_EAST)
        {
          curr_filter_pos_uncert_east_ = point.as_float();

          if (config_->use_enu_frame_)
          {
            publishers_->filter_msg_.pose.covariance[0] = pow(curr_filter_pos_uncert_east_, 2);
            publishers_->filter_relative_pos_msg_.pose.covariance[0] = publishers_->filter_msg_.pose.covariance[0];
          }
          else
          {
            publishers_->filter_msg_.pose.covariance[7] = pow(curr_filter_pos_uncert_east_, 2);
            publishers_->filter_relative_pos_msg_.pose.covariance[7] = publishers_->filter_msg_.pose.covariance[7];
          }
        }
        else if (point.qualifier() == mscl::MipTypes::CH_DOWN)
        {
          curr_filter_pos_uncert_down_ = point.as_float();
          publishers_->filter_msg_.pose.covariance[14] = pow(curr_filter_pos_uncert_down_, 2);
          publishers_->filter_relative_pos_msg_.pose.covariance[14] = publishers_->filter_msg_.pose.covariance[14];
        }
      }
      break;

      case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_NED_UNCERT:
      {
        if (point.qualifier() == mscl::MipTypes::CH_NORTH)
        {
          curr_filter_vel_uncert_north_ = point.as_float();

          if (config_->use_enu_frame_)
          {
            publishers_->filter_msg_.twist.covariance[0] = pow(curr_filter_vel_uncert_north_, 2);
            publishers_->filter_relative_pos_msg_.twist.covariance[0] =
                publishers_->filter_msg_.twist.covariance[0];
          }
          else
          {
            publishers_->filter_msg_.twist.covariance[7] = pow(curr_filter_vel_uncert_north_, 2);
            publishers_->filter_relative_pos_msg_.twist.covariance[7] =
                publishers_->filter_msg_.twist.covariance[7];
          }
        }
        else if (point.qualifier() == mscl::MipTypes::CH_EAST)
        {
          curr_filter_vel_uncert_east_ = point.as_float();

          if (config_->use_enu_frame_)
          {
            publishers_->filter_msg_.twist.covariance[7] = pow(curr_filter_vel_uncert_east_, 2);
            publishers_->filter_relative_pos_msg_.twist.covariance[7] =
                publishers_->filter_msg_.twist.covariance[7];
          }
          else
          {
            publishers_->filter_msg_.twist.covariance[0] = pow(curr_filter_vel_uncert_east_, 2);
            publishers_->filter_relative_pos_msg_.twist.covariance[0] =
                publishers_->filter_msg_.twist.covariance[0];
          }
        }
        else if (point.qualifier() == mscl::MipTypes::CH_DOWN)
        {
          curr_filter_vel_uncert_down_ = point.as_float();
          publishers_->filter_msg_.twist.covariance[14] = pow(curr_filter_vel_uncert_down_, 2);
          publishers_->filter_relative_pos_msg_.twist.covariance[14] =
              publishers_->filter_msg_.twist.covariance[14];
        }
      }
      break;

      case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ATT_UNCERT_EULER:
      {
        if (point.qualifier() == mscl::MipTypes::CH_ROLL)
        {
          curr_filter_att_uncert_roll_ = point.as_float();
          publishers_->filter_msg_.pose.covariance[21] = pow(curr_filter_att_uncert_roll_, 2);
          publishers_->filtered_imu_msg_.orientation_covariance[0] = publishers_->filter_msg_.pose.covariance[21];
          publishers_->filter_relative_pos_msg_.pose.covariance[21] = publishers_->filter_msg_.pose.covariance[21];
        }
        else if (point.qualifier() == mscl::MipTypes::CH_PITCH)
        {
          curr_filter_att_uncert_pitch_ = point.as_float();
          publishers_->filter_msg_.pose.covariance[28] = pow(curr_filter_att_uncert_pitch_, 2);
          publishers_->filtered_imu_msg_.orientation_covariance[4] = publishers_->filter_msg_.pose.covariance[28];
          publishers_->filter_relative_pos_msg_.pose.covariance[28] = publishers_->filter_msg_.pose.covariance[28];
        }
        else if (point.qualifier() == mscl::MipTypes::CH_YAW)
        {
          curr_filter_att_uncert_yaw_ = point.as_float();
          publishers_->filter_msg_.pose.covariance[35] = pow(curr_filter_att_uncert_yaw_, 2);
          publishers_->filtered_imu_msg_.orientation_covariance[8] = publishers_->filter_msg_.pose.covariance[35];
          publishers_->filter_relative_pos_msg_.pose.covariance[35] = publishers_->filter_msg_.pose.covariance[35];
        }
      }
      break;

      case mscl::MipTypes::CH_FIELD_ESTFILTER_HEADING_UPDATE_SOURCE:
      {
        if (point.qualifier() == mscl::MipTypes::CH_HEADING)
        {
          publishers_->filter_heading_state_msg_.heading_rad = point.as_float();
        }
        else if (point.qualifier() == mscl::MipTypes::CH_HEADING_UNCERTAINTY)
        {
          publishers_->filter_heading_state_msg_.heading_uncertainty = point.as_float();
        }
        else if (point.qualifier() == mscl::MipTypes::CH_SOURCE)
        {
          publishers_->filter_heading_state_msg_.source = point.as_uint16();
        }
        else if (point.qualifier() == mscl::MipTypes::CH_FLAGS)
        {
          publishers_->filter_heading_state_msg_.status_flags = point.as_uint16();
        }
      }
      break;

      case mscl::MipTypes::CH_FIELD_ESTFILTER_NED_RELATIVE_POS:
      {
        if (point.qualifier() == mscl::MipTypes::CH_X)
        {
          double rel_pos_north = point.as_double();

          if (config_->use_enu_frame_)
          {
            publishers_->filter_relative_pos_msg_.pose.pose.position.y = rel_pos_north;
            publishers_->filter_transform_msg_.transform.translation.y = rel_pos_north;
          }
          else
          {
            publishers_->filter_relative_pos_msg_.pose.pose.position.x = rel_pos_north;
            publishers_->filter_transform_msg_.transform.translation.x = rel_pos_north;
          }
        }
        else if (point.qualifier() == mscl::MipTypes::CH_Y)
        {
          double rel_pos_east = point.as_double();

          if (config_->use_enu_frame_)
          {
            publishers_->filter_relative_pos_msg_.pose.pose.position.x = rel_pos_east;
            publishers_->filter_transform_msg_.transform.translation.x = rel_pos_east;
          }
          else
          {
            publishers_->filter_relative_pos_msg_.pose.pose.position.y = rel_pos_east;
            publishers_->filter_transform_msg_.transform.translation.y = rel_pos_east;
          }
        }
        else if (point.qualifier() == mscl::MipTypes::CH_Z)
        {
          double rel_pos_down = point.as_double();

          if (config_->use_enu_frame_)
          {
            publishers_->filter_relative_pos_msg_.pose.pose.position.z = -rel_pos_down;
            publishers_->filter_transform_msg_.transform.translation.z = -rel_pos_down;
          }
          else
          {
            publishers_->filter_relative_pos_msg_.pose.pose.position.z = rel_pos_down;
            publishers_->filter_transform_msg_.transform.translation.z = rel_pos_down;
          }
        }
      }
      break;

      case mscl::MipTypes::CH_FIELD_ESTFILTER_POSITION_AIDING_STATUS:
      {
        if (point.hasAddlIdentifiers())
        {
          int gnss_id = static_cast<int>(point.addlIdentifiers()[0].id()) - 1;

          if (gnss_id < 0 || gnss_id >= NUM_GNSS)
          {
          }
          else if (point.qualifier() == mscl::MipTypes::CH_TIME_OF_WEEK)
          {
            publishers_->gnss_aiding_status_msg_[gnss_id].gps_tow = point.as_double();
            gnss_aiding_status_received[gnss_id] = true;
          }
          else if (point.qualifier() == mscl::MipTypes::CH_STATUS)
          {
            uint16_t status_flags = point.as_uint16();

            publishers_->gnss_aiding_status_msg_[gnss_id].has_position_fix =
                (status_flags & mscl::InertialTypes::GnssAidingStatus::GNSS_AIDING_NO_FIX) == 0;
            publishers_->gnss_aiding_status_msg_[gnss_id].tight_coupling =
                status_flags & mscl::InertialTypes::GnssAidingStatus::GNSS_AIDING_TIGHT_COUPLING;
            publishers_->gnss_aiding_status_msg_[gnss_id].differential_corrections =
                status_flags & mscl::InertialTypes::GnssAidingStatus::GNSS_AIDING_DIFFERENTIAL;
            publishers_->gnss_aiding_status_msg_[gnss_id].integer_fix =
                status_flags & mscl::InertialTypes::GnssAidingStatus::GNSS_AIDING_INTEGER_FIX;
            publishers_->gnss_aiding_status_msg_[gnss_id].using_gps =
                status_flags & mscl::InertialTypes::GnssAidingStatus::GNSS_AIDING_GPS;
            publishers_->gnss_aiding_status_msg_[gnss_id].using_glonass =
                status_flags & mscl::InertialTypes::GnssAidingStatus::GNSS_AIDING_GLONASS;
            publishers_->gnss_aiding_status_msg_[gnss_id].using_galileo =
                status_flags & mscl::InertialTypes::GnssAidingStatus::GNSS_AIDING_GALILEO;
            publishers_->gnss_aiding_status_msg_[gnss_id].using_beidou =
                status_flags & mscl::InertialTypes::GnssAidingStatus::GNSS_AIDING_BEIDOU;
          }
          else
          {
            MICROSTRAIN_INFO(node_, "Point Qualifier %d", (int)point.qualifier());
          }
        }
      }
      break;

      case mscl::MipTypes::CH_FIELD_ESTFILTER_GNSS_DUAL_ANTENNA_STATUS:
      {
        if (point.qualifier() == mscl::MipTypes::CH_TIME_OF_WEEK)
        {
          gnss_dual_antenna_status_received = true;
          publishers_->gnss_dual_antenna_status_msg_.gps_tow = point.as_float();
        }
        else if (point.qualifier() == mscl::MipTypes::CH_HEADING)
        {
          publishers_->gnss_dual_antenna_status_msg_.heading = point.as_float();
        }
        else if (point.qualifier() == mscl::MipTypes::CH_HEADING_UNCERTAINTY)
        {
          publishers_->gnss_dual_antenna_status_msg_.heading_uncertainty = point.as_float();
        }
        else if (point.qualifier() == mscl::MipTypes::CH_FIX_TYPE)
        {
          publishers_->gnss_dual_antenna_status_msg_.fix_type = point.as_uint8();
        }
        else if (point.qualifier() == mscl::MipTypes::CH_STATUS)
        {
          uint16_t status_flags = point.as_uint16();

          publishers_->gnss_dual_antenna_status_msg_.rcv_1_valid =
              status_flags & mscl::InertialTypes::DualAntennaStatusFlags::DATA_VALID_REC_1;
          publishers_->gnss_dual_antenna_status_msg_.rcv_2_valid =
              status_flags & mscl::InertialTypes::DualAntennaStatusFlags::DATA_VALID_REC_2;
          publishers_->gnss_dual_antenna_status_msg_.antenna_offsets_valid =
              status_flags & mscl::InertialTypes::DualAntennaStatusFlags::ANTENNA_OFFSETS_VALID;
        }
      }
      break;

      case mscl::MipTypes::CH_FIELD_ESTFILTER_AIDING_MEASURE_SUMMARY:
      {
        if (point.qualifier() == mscl::MipTypes::CH_TIME_OF_WEEK)
        {
          filter_aiding_measurement_summary_received = true;
          publishers_->filter_aiding_measurement_summary_msg_.gps_tow = point.as_float();
        }
        if (point.hasAddlIdentifiers())
        {
          uint8_t source = 0;
          mscl::MipChannelIdentifier::AidingMeasurementTypes type;
          FilterAidingMeasurementSummaryIndicatorMsg* indicator = nullptr;
          for (const auto& identifier : point.addlIdentifiers())
          {
            if (identifier.identifierType() == mscl::MipChannelIdentifier::AIDING_MEASUREMENT_TYPE)
            {
              type = static_cast<mscl::MipChannelIdentifier::AidingMeasurementTypes>(identifier.id());
            }
            else if (identifier.identifierType() == mscl::MipChannelIdentifier::GNSS_RECEIVER_ID)
            {
              source = identifier.id() - 1;
            }
          }

          switch (type)
          {
            case mscl::MipChannelIdentifier::AidingMeasurementTypes::GNSS:
              if (source == GNSS1_ID)
                indicator = &(publishers_->filter_aiding_measurement_summary_msg_.gnss1);
              else if (source == GNSS2_ID)
                indicator = &(publishers_->filter_aiding_measurement_summary_msg_.gnss2);
              break;
            case mscl::MipChannelIdentifier::AidingMeasurementTypes::DUAL_ANTENNA:
              indicator = &(publishers_->filter_aiding_measurement_summary_msg_.dual_antenna);
              break;
            case mscl::MipChannelIdentifier::AidingMeasurementTypes::HEADING:
              indicator = &(publishers_->filter_aiding_measurement_summary_msg_.heading);
              break;
            case mscl::MipChannelIdentifier::AidingMeasurementTypes::PRESSURE:
              indicator = &(publishers_->filter_aiding_measurement_summary_msg_.pressure);
              break;
            case mscl::MipChannelIdentifier::AidingMeasurementTypes::MAGNETOMETER:
              indicator = &(publishers_->filter_aiding_measurement_summary_msg_.magnetometer);
              break;
            case mscl::MipChannelIdentifier::AidingMeasurementTypes::SPEED:
              indicator = &(publishers_->filter_aiding_measurement_summary_msg_.speed);
              break;
            default:
              continue;
          }

          if (indicator != nullptr)
          {
            const uint8_t indicator_bits = point.as_uint8();
            indicator->enabled = indicator_bits & mscl::InertialTypes::AidingMeasurementStatus::AIDING_MEASUREMENT_ENABLED;
            indicator->used = indicator_bits & mscl::InertialTypes::AidingMeasurementStatus::AIDING_MEASUREMENT_USED;
            indicator->residual_high_warning = indicator_bits & mscl::InertialTypes::AidingMeasurementStatus::AIDING_MEASUREMENT_WARNING_RESIDUAL_HIGH;
            indicator->sample_time_warning = indicator_bits & mscl::InertialTypes::AidingMeasurementStatus::AIDING_MEASUREMENT_WARNING_SAMPLE_TIME;
            indicator->configuration_error = indicator_bits & mscl::InertialTypes::AidingMeasurementStatus::AIDING_MEASUREMENT_CONFIG_ERROR;
            indicator->max_num_meas_exceeded = indicator_bits & mscl::InertialTypes::AidingMeasurementStatus::AIDING_MEASUREMENT_MAX_COUNT_EXCEEDED;
          }
        }
      }

      default:
        break;
    }
  }

  // Copy fixed covariances to the filtered IMU message
  std::copy(config_->imu_linear_cov_.begin(), config_->imu_linear_cov_.end(),
            publishers_->filtered_imu_msg_.linear_acceleration_covariance.begin());
  std::copy(config_->imu_angular_cov_.begin(), config_->imu_angular_cov_.end(),
            publishers_->filtered_imu_msg_.angular_velocity_covariance.begin());

  // Publish
  if (config_->publish_filter_)
  {
    publishers_->filtered_imu_pub_->publish(publishers_->filtered_imu_msg_);
    publishers_->filter_pub_->publish(publishers_->filter_msg_);
    publishers_->filter_status_pub_->publish(publishers_->filter_status_msg_);
    publishers_->filter_heading_pub_->publish(publishers_->filter_heading_msg_);
    publishers_->filter_heading_state_pub_->publish(publishers_->filter_heading_state_msg_);

    if (config_->publish_gnss_dual_antenna_status_ && gnss_dual_antenna_status_received)
      publishers_->gnss_dual_antenna_status_pub_->publish(publishers_->gnss_dual_antenna_status_msg_);
  }

  if (config_->publish_filter_relative_pos_)
  {
    publishers_->filter_relative_pos_pub_->publish(publishers_->filter_relative_pos_msg_);
    publishers_->transform_broadcaster_->sendTransform(publishers_->filter_transform_msg_);
  }

  for (i = 0; i < NUM_GNSS; i++)
  {
    if (config_->publish_gnss_[i] && config_->publish_gnss_aiding_status_[i] && gnss_aiding_status_received[i])
      publishers_->gnss_aiding_status_pub_[i]->publish(publishers_->gnss_aiding_status_msg_[i]);
  }

  if (config_->publish_filter_aiding_measurement_summary_ && filter_aiding_measurement_summary_received)
    publishers_->filter_aiding_measurement_summary_pub_->publish(publishers_->filter_aiding_measurement_summary_msg_);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// MIP GNSS Packet Parsing Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

void MicrostrainParser::parseGNSSPacket(const mscl::MipDataPacket& packet, int gnss_id)
{
  // Rnage-check id
  if (gnss_id >= NUM_GNSS)
    return;

  // Update diagnostics
  gnss_valid_packet_count_[gnss_id]++;

  // Handle time
  const bool time_valid = packet.hasDeviceTime() && config_->use_device_timestamp_;
  const RosTimeType packet_time = getPacketTimestamp(packet);

  // GPS Fix time
  set_seq(&publishers_->gnss_msg_[gnss_id].header, gnss_valid_packet_count_[gnss_id]);
  publishers_->gnss_msg_[gnss_id].header.stamp = config_->use_ros_time_ ? ros_time_now(node_) : packet_time;
  publishers_->gnss_msg_[gnss_id].header.frame_id = config_->gnss_frame_id_[gnss_id];

  // GPS Odom time
  set_seq(&publishers_->gnss_odom_msg_[gnss_id].header, gnss_valid_packet_count_[gnss_id]);
  publishers_->gnss_odom_msg_[gnss_id].header.stamp = config_->use_ros_time_ ? ros_time_now(node_) : packet_time;
  publishers_->gnss_odom_msg_[gnss_id].header.frame_id = config_->gnss_frame_id_[gnss_id];
  // publishers_->gnss_odom_msg_[gnss_id].child_frame_id  = config_->gnss_odom_child_frame_id_[gnss_id];

  // GPS Time reference
  set_seq(&publishers_->gnss_time_msg_[gnss_id].header, gnss_valid_packet_count_[gnss_id]);
  publishers_->gnss_time_msg_[gnss_id].header.stamp = ros_time_now(node_);
  publishers_->gnss_time_msg_[gnss_id].header.frame_id = config_->gnss_frame_id_[gnss_id];
  publishers_->gnss_time_msg_[gnss_id].time_ref = packet_time;

  // Get the list of data elements
  const mscl::MipDataPoints& points = packet.data();

  // Loop over data elements and map them
  for (mscl::MipDataPoint point : points)
  {
    switch (point.field())
    {
      // LLH Position
      case mscl::MipTypes::CH_FIELD_GNSS_LLH_POSITION:
      case mscl::MipTypes::CH_FIELD_GNSS_1_LLH_POSITION:
      case mscl::MipTypes::CH_FIELD_GNSS_2_LLH_POSITION:
      {
        if (point.qualifier() == mscl::MipTypes::CH_LATITUDE)
        {
          publishers_->gnss_msg_[gnss_id].latitude = point.as_double();

          if (config_->use_enu_frame_)
            publishers_->gnss_odom_msg_[gnss_id].pose.pose.position.y = publishers_->gnss_msg_[gnss_id].latitude;
          else
            publishers_->gnss_odom_msg_[gnss_id].pose.pose.position.x = publishers_->gnss_msg_[gnss_id].latitude;
        }
        else if (point.qualifier() == mscl::MipTypes::CH_LONGITUDE)
        {
          publishers_->gnss_msg_[gnss_id].longitude = point.as_double();

          if (config_->use_enu_frame_)
            publishers_->gnss_odom_msg_[gnss_id].pose.pose.position.x = publishers_->gnss_msg_[gnss_id].longitude;
          else
            publishers_->gnss_odom_msg_[gnss_id].pose.pose.position.y = publishers_->gnss_msg_[gnss_id].longitude;
        }
        else if (point.qualifier() == mscl::MipTypes::CH_HEIGHT_ABOVE_ELLIPSOID)
        {
          publishers_->gnss_msg_[gnss_id].altitude = point.as_double();
          publishers_->gnss_odom_msg_[gnss_id].pose.pose.position.z = publishers_->gnss_msg_[gnss_id].altitude;
        }
        else if (point.qualifier() == mscl::MipTypes::CH_HORIZONTAL_ACCURACY)
        {
          // Horizontal covariance maps to lat and lon
          publishers_->gnss_msg_[gnss_id].position_covariance[0] = pow(point.as_float(), 2);
          publishers_->gnss_msg_[gnss_id].position_covariance[4] =
              publishers_->gnss_msg_[gnss_id].position_covariance[0];
          publishers_->gnss_odom_msg_[gnss_id].pose.covariance[0] =
              publishers_->gnss_msg_[gnss_id].position_covariance[0];
          publishers_->gnss_odom_msg_[gnss_id].pose.covariance[7] =
              publishers_->gnss_msg_[gnss_id].position_covariance[0];
        }
        else if (point.qualifier() == mscl::MipTypes::CH_VERTICAL_ACCURACY)
        {
          publishers_->gnss_msg_[gnss_id].position_covariance[8] = pow(point.as_float(), 2);
          publishers_->gnss_odom_msg_[gnss_id].pose.covariance[14] =
              publishers_->gnss_msg_[gnss_id].position_covariance[8];
        }

        publishers_->gnss_msg_[gnss_id].status.service = 1;
        publishers_->gnss_msg_[gnss_id].position_covariance_type = 2;
      }
      break;

      case mscl::MipTypes::ChannelField::CH_FIELD_GNSS_NED_VELOCITY:
      case mscl::MipTypes::ChannelField::CH_FIELD_GNSS_1_NED_VELOCITY:
      case mscl::MipTypes::ChannelField::CH_FIELD_GNSS_2_NED_VELOCITY:
      {
        if (point.qualifier() == mscl::MipTypes::CH_NORTH)
        {
          float north_velocity = point.as_float();

          if (config_->use_enu_frame_)
            publishers_->gnss_odom_msg_[gnss_id].twist.twist.linear.y = north_velocity;
          else
            publishers_->gnss_odom_msg_[gnss_id].twist.twist.linear.x = north_velocity;
        }
        else if (point.qualifier() == mscl::MipTypes::CH_EAST)
        {
          float east_velocity = point.as_float();

          if (config_->use_enu_frame_)
            publishers_->gnss_odom_msg_[gnss_id].twist.twist.linear.x = east_velocity;
          else
            publishers_->gnss_odom_msg_[gnss_id].twist.twist.linear.y = east_velocity;
        }
        else if (point.qualifier() == mscl::MipTypes::CH_DOWN)
        {
          float down_velocity = point.as_float();

          if (config_->use_enu_frame_)
            publishers_->gnss_odom_msg_[gnss_id].twist.twist.linear.z = -down_velocity;
          else
            publishers_->gnss_odom_msg_[gnss_id].twist.twist.linear.z = down_velocity;
        }
      }
      break;

      case mscl::MipTypes::ChannelField::CH_FIELD_GNSS_1_FIX_INFO:
      case mscl::MipTypes::ChannelField::CH_FIELD_GNSS_2_FIX_INFO:
      {
        if (point.qualifier() == mscl::MipTypes::CH_FIX_TYPE)
        {
          publishers_->gnss_fix_info_msg_[gnss_id].fix_type = point.as_uint8();
        }
        else if (point.qualifier() == mscl::MipTypes::CH_SV_COUNT)
        {
          publishers_->gnss_fix_info_msg_[gnss_id].num_sv = point.as_uint8();
        }
        else if (point.qualifier() == mscl::MipTypes::CH_FLAGS)
        {
          publishers_->gnss_fix_info_msg_[gnss_id].sbas_used = point.as_uint16() & mscl::InertialTypes::GnssFixFlags::FIX_SBAS_CORRECTIONS;
          publishers_->gnss_fix_info_msg_[gnss_id].dngss_used = point.as_uint16() & mscl::InertialTypes::GnssFixFlags::FIX_DGNSS_CORRECTIONS;
        }
      }
      break;
    }
  }

  // Publish
  if (config_->publish_gnss_[gnss_id])
  {
    publishers_->gnss_pub_[gnss_id]->publish(publishers_->gnss_msg_[gnss_id]);
    publishers_->gnss_odom_pub_[gnss_id]->publish(publishers_->gnss_odom_msg_[gnss_id]);
    publishers_->gnss_fix_info_pub_[gnss_id]->publish(publishers_->gnss_fix_info_msg_[gnss_id]);

    if (time_valid)
      publishers_->gnss_time_pub_[gnss_id]->publish(publishers_->gnss_time_msg_[gnss_id]);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// MIP RTK Packet Parsing Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

void MicrostrainParser::parseRTKPacket(const mscl::MipDataPacket& packet)
{
  // Update diagnostics
  rtk_valid_packet_count_++;

  // Get the list of data elements
  const mscl::MipDataPoints& points = packet.data();

  // RTK version from status flags. 1 == v2
  uint8_t version = 1;

  // Loop over data elements and map them
  for (mscl::MipDataPoint point : points)
  {
    switch (point.field())
    {
      // RTK Correction Status
      case mscl::MipTypes::CH_FIELD_GNSS_3_RTK_CORRECTIONS_STATUS:
      {
        if (point.qualifier() == mscl::MipTypes::CH_TIME_OF_WEEK)
        {
          publishers_->rtk_msg_.gps_tow = point.as_double();
        }
        else if (point.qualifier() == mscl::MipTypes::CH_WEEK_NUMBER)
        {
          publishers_->rtk_msg_.gps_week = point.as_uint16();
        }
        else if (point.qualifier() == mscl::MipTypes::CH_STATUS)
        {
          publishers_->rtk_msg_.epoch_status = point.as_uint16();
        }
        else if (point.qualifier() == mscl::MipTypes::CH_FLAGS)
        {
          // Raw status flags value
          mscl::uint32 raw_value = point.as_uint32();

          // Decode dongle status
          mscl::RTKDeviceStatusFlags dongle_status(raw_value);

          // Get the RTK version from the status flags
          version = dongle_status.version();

          switch (version)
          {
            // v1
            case 0:
            {
              // Cast to v1 dongle
              mscl::RTKDeviceStatusFlags_v1 dongle_status_v1 = dongle_status;

              publishers_->rtk_msg_v1_.raw_status_flags = raw_value;
              publishers_->rtk_msg_v1_.dongle_version = version;
              publishers_->rtk_msg_v1_.dongle_controller_state = dongle_status_v1.controllerState();
              publishers_->rtk_msg_v1_.dongle_platform_state = dongle_status_v1.platformState();
              publishers_->rtk_msg_v1_.dongle_controller_status = dongle_status_v1.controllerStatusCode();
              publishers_->rtk_msg_v1_.dongle_platform_status = dongle_status_v1.platformStatusCode();
              publishers_->rtk_msg_v1_.dongle_reset_reason = dongle_status_v1.resetReason();
              publishers_->rtk_msg_v1_.dongle_signal_quality = dongle_status_v1.signalQuality();
              break;
            }
            // v2
            default:
            {
              publishers_->rtk_msg_.raw_status_flags = raw_value;
              publishers_->rtk_msg_.dongle_version = version;
              publishers_->rtk_msg_.dongle_modem_state = dongle_status.modemState();
              publishers_->rtk_msg_.dongle_connection_type = dongle_status.connectionType();
              publishers_->rtk_msg_.dongle_rssi = -dongle_status.rssi();
              publishers_->rtk_msg_.dongle_signal_quality = dongle_status.signalQuality();
              publishers_->rtk_msg_.dongle_tower_change_indicator = dongle_status.towerChangeIndicator();
              publishers_->rtk_msg_.dongle_nmea_timeout = dongle_status.nmeaTimeout();
              publishers_->rtk_msg_.dongle_server_timeout = dongle_status.serverTimeout();
              publishers_->rtk_msg_.dongle_rtcm_timeout = dongle_status.rtcmTimeout();
              publishers_->rtk_msg_.dongle_out_of_range = dongle_status.deviceOutOfRange();
              publishers_->rtk_msg_.dongle_corrections_unavailable = dongle_status.correctionsUnavailable();
              break;
            }
          }
        }
        else if (point.qualifier() == mscl::MipTypes::CH_GPS_CORRECTION_LATENCY)
        {
          publishers_->rtk_msg_.gps_correction_latency = point.as_float();
        }
        else if (point.qualifier() == mscl::MipTypes::CH_GLONASS_CORRECTION_LATENCY)
        {
          publishers_->rtk_msg_.glonass_correction_latency = point.as_float();
        }
        else if (point.qualifier() == mscl::MipTypes::CH_GALILEO_CORRECTION_LATENCY)
        {
          publishers_->rtk_msg_.galileo_correction_latency = point.as_float();
        }
        else if (point.qualifier() == mscl::MipTypes::CH_BEIDOU_CORRECTION_LATENCY)
        {
          publishers_->rtk_msg_.beidou_correction_latency = point.as_float();
        }
      }
      break;
    }
  }

  // Publish
  if (config_->publish_rtk_)
  {
    switch (version)
    {
      // v1
      case 0:
      {
        publishers_->rtk_msg_v1_.gps_tow = publishers_->rtk_msg_.gps_tow;
        publishers_->rtk_msg_v1_.gps_week = publishers_->rtk_msg_.gps_week;
        publishers_->rtk_msg_v1_.epoch_status = publishers_->rtk_msg_.epoch_status;
        publishers_->rtk_msg_v1_.gps_correction_latency = publishers_->rtk_msg_.gps_correction_latency;
        publishers_->rtk_msg_v1_.glonass_correction_latency = publishers_->rtk_msg_.glonass_correction_latency;
        publishers_->rtk_msg_v1_.galileo_correction_latency = publishers_->rtk_msg_.galileo_correction_latency;
        publishers_->rtk_msg_v1_.beidou_correction_latency = publishers_->rtk_msg_.beidou_correction_latency;

        publishers_->rtk_pub_v1_->publish(publishers_->rtk_msg_v1_);
        break;
      }
      // v2
      default:
      {
        publishers_->rtk_pub_->publish(publishers_->rtk_msg_);
        break;
      }
    }
  }
}

void MicrostrainParser::printPacketStats()
{
  if (config_->inertial_device_)
  {
    return;
  }

  if (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_DEVICE_STATUS))
  {
    if (config_->inertial_device_->features().supportedStatusSelectors().size() > 1)
    {
      mscl::DeviceStatusData status = config_->inertial_device_->getDiagnosticDeviceStatus();

      imu_valid_packet_count_ = status.imuMessageInfo().messagesRead;
      imu_checksum_error_packet_count_ = status.imuMessageInfo().messageParsingErrors;
      imu_timeout_packet_count_ = status.imuStreamInfo().outgoingPacketsDropped;
      filter_timeout_packet_count_ = status.estimationFilterStreamInfo().outgoingPacketsDropped;

      MICROSTRAIN_DEBUG_THROTTLE(node_, 1.0, "%u IMU (%u errors) Packets", imu_valid_packet_count_,
                                 imu_timeout_packet_count_ + imu_checksum_error_packet_count_);

      gnss_checksum_error_packet_count_[GNSS1_ID] = status.gnssMessageInfo().messageParsingErrors;
      gnss_valid_packet_count_[GNSS1_ID] = status.gnssMessageInfo().messagesRead;
      gnss_timeout_packet_count_[GNSS1_ID] = status.gnssStreamInfo().outgoingPacketsDropped;

      MICROSTRAIN_DEBUG_THROTTLE(
          node_, 1.0, "%u FILTER (%u errors)    %u IMU (%u errors)    %u GPS (%u errors) Packets",
          filter_valid_packet_count_, filter_timeout_packet_count_, imu_valid_packet_count_,
          imu_timeout_packet_count_ + imu_checksum_error_packet_count_, gnss_valid_packet_count_[GNSS1_ID],
          gnss_timeout_packet_count_[GNSS1_ID] + gnss_checksum_error_packet_count_[GNSS1_ID]);

      MICROSTRAIN_DEBUG_THROTTLE(node_, 1.0, "%u FILTER (%u errors)    %u IMU (%u errors) Packets",
                                 filter_valid_packet_count_, filter_timeout_packet_count_, imu_valid_packet_count_,
                                 imu_timeout_packet_count_ + imu_checksum_error_packet_count_);
    }
  }
}

}  // namespace microstrain
