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
#include "microstrain_parser.h"


namespace Microstrain
{

MicrostrainParser::MicrostrainParser(RosNodeType* node, MicrostrainConfig* config, MicrostrainPublishers* publishers)
  : m_node(node), m_config(config), m_publishers(publishers)
{}

void MicrostrainParser::parse_mip_packet(const mscl::MipDataPacket &packet)
{
  switch (packet.descriptorSet())
  {
  case mscl::MipTypes::DataClass::CLASS_AHRS_IMU:
    parse_imu_packet(packet);
    print_packet_stats();
    break;

  case mscl::MipTypes::DataClass::CLASS_ESTFILTER:
    parse_filter_packet(packet);
    print_packet_stats();
    break;

  case mscl::MipTypes::DataClass::CLASS_GNSS:
  case mscl::MipTypes::DataClass::CLASS_GNSS1:
    parse_gnss_packet(packet, GNSS1_ID);
    print_packet_stats();
    break;

  case mscl::MipTypes::DataClass::CLASS_GNSS2:
    parse_gnss_packet(packet, GNSS2_ID);
    print_packet_stats();
    break;

  case mscl::MipTypes::DataClass::CLASS_GNSS3:
    parse_rtk_packet(packet);
    print_packet_stats();
    break;

  default:
    break;
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// MIP IMU Packet Parsing Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

void MicrostrainParser::parse_imu_packet(const mscl::MipDataPacket &packet)
{
  //Update the diagnostics
  m_imu_valid_packet_count++;
  
  //Handle time
  uint64_t time = packet.collectedTimestamp().nanoseconds();
  
  //Check if the user wants to use the device timestamp instead of PC collected time
  if(packet.hasDeviceTime() && m_config->m_use_device_timestamp) 
  {
     time = packet.deviceTimestamp().nanoseconds();
  }

  //IMU timestamp
  set_seq(&m_publishers->m_imu_msg.header, m_imu_valid_packet_count);
  m_publishers->m_imu_msg.header.stamp    = to_ros_time(time);
  m_publishers->m_imu_msg.header.frame_id = m_config->m_imu_frame_id;

  //Magnetometer timestamp
  m_publishers->m_mag_msg.header      = m_publishers->m_imu_msg.header;

  //GPS correlation timestamp headder
  m_publishers->m_gps_corr_msg.header = m_publishers->m_imu_msg.header;

  //Data present flags
  bool has_accel = false;
  bool has_gyro  = false;
  bool has_quat  = false;
  bool has_mag   = false;

  //Get the list of data elements
  const mscl::MipDataPoints &points = packet.data();

  //Loop over the data elements and map them
  for(auto point_iter = points.begin(); point_iter != points.end(); point_iter++)
  {
    auto point = *point_iter;
    switch(point.field())
    {
    //Scaled Accel
    case mscl::MipTypes::CH_FIELD_SENSOR_SCALED_ACCEL_VEC:
    {
      has_accel = true;

      // Stuff into ROS message - acceleration in m/s^2
      if(point.qualifier() == mscl::MipTypes::CH_X)
      {
        m_publishers->m_imu_msg.linear_acceleration.x = USTRAIN_G * point.as_float();
      }
      else if(point.qualifier() == mscl::MipTypes::CH_Y)
      {
        m_publishers->m_imu_msg.linear_acceleration.y = USTRAIN_G * point.as_float();
      }
      else if(point.qualifier() == mscl::MipTypes::CH_Z)
      {
        m_publishers->m_imu_msg.linear_acceleration.z = USTRAIN_G * point.as_float();
      }
    }break;

    //Scaled Gyro
    case mscl::MipTypes::CH_FIELD_SENSOR_SCALED_GYRO_VEC:
    {
      has_gyro = true;

      if(point.qualifier() == mscl::MipTypes::CH_X)
      {
        m_publishers->m_imu_msg.angular_velocity.x = point.as_float();
      }
      else if(point.qualifier() == mscl::MipTypes::CH_Y)
      {
        m_publishers->m_imu_msg.angular_velocity.y = point.as_float();
      }
      else if(point.qualifier() == mscl::MipTypes::CH_Z)
      {
        m_publishers->m_imu_msg.angular_velocity.z = point.as_float();
      }
    }break;

    //Scaled Mag
    case mscl::MipTypes::CH_FIELD_SENSOR_SCALED_MAG_VEC:
    {
      has_mag = true;

      if(point.qualifier() == mscl::MipTypes::CH_X)
      {
        m_curr_imu_mag_x           = point.as_float();
        m_publishers->m_mag_msg.magnetic_field.x = m_curr_imu_mag_x;
      }
      else if(point.qualifier() == mscl::MipTypes::CH_Y)
      {
        m_curr_imu_mag_y           = point.as_float();
        m_publishers->m_mag_msg.magnetic_field.y = m_curr_imu_mag_y;
      }
      else if(point.qualifier() == mscl::MipTypes::CH_Z)
      {
        m_curr_imu_mag_z           = point.as_float();
        m_publishers->m_mag_msg.magnetic_field.z = m_curr_imu_mag_z;
      }
    }break;

    //Orientation Quaternion
    case mscl::MipTypes::CH_FIELD_SENSOR_ORIENTATION_QUATERNION:
    {
      has_quat = true;

      if(point.qualifier() == mscl::MipTypes::CH_QUATERNION)
      {
        mscl::Vector quaternion  = point.as_Vector();
        m_curr_filter_quaternion = quaternion;

        if(m_config->m_use_enu_frame)
        {
          tf2::Quaternion q_ned2enu, qbody2ned(quaternion.as_floatAt(1), quaternion.as_floatAt(2), quaternion.as_floatAt(3), quaternion.as_floatAt(0));
          m_config->m_t_ned2enu.getRotation(q_ned2enu);
          m_publishers->m_imu_msg.orientation = tf2::toMsg(q_ned2enu*qbody2ned);
        }
        else
        {
  
          m_publishers->m_imu_msg.orientation.x = quaternion.as_floatAt(1);
          m_publishers->m_imu_msg.orientation.y = quaternion.as_floatAt(2);
          m_publishers->m_imu_msg.orientation.z = quaternion.as_floatAt(3);
          m_publishers->m_imu_msg.orientation.w = quaternion.as_floatAt(0);
        }
      }
    }break;

    //GPS Corr
    case mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_GPS_CORRELATION_TIMESTAMP:
    {
      // for some reason point.qualifier() == mscl::MipTypes::CH_WEEK_NUMBER and
      // point.qualifier() == mscl::MipTypes::CH_FLAGS always returned false so I used
      // an iterator and manually incremented it to access all the elements
      m_publishers->m_gps_corr_msg.gps_cor.gps_tow = point_iter->as_double();
      point_iter++;
      m_publishers->m_gps_corr_msg.gps_cor.gps_week_number = point_iter->as_uint16();
      point_iter++;
      m_publishers->m_gps_corr_msg.gps_cor.timestamp_flags = point_iter->as_uint16();
    }break;
    }
  }

  if(has_accel)
  {
    //Since the sensor does not produce a covariance for linear acceleration, set it based on our pulled in parameters.
    std::copy(m_config->m_imu_linear_cov.begin(), m_config->m_imu_linear_cov.end(), m_publishers->m_imu_msg.linear_acceleration_covariance.begin());
  }

  if(has_gyro)
  {
    //Since the sensor does not produce a covariance for angular velocity, set it based on our pulled in parameters.
    std::copy(m_config->m_imu_angular_cov.begin(), m_config->m_imu_angular_cov.end(), m_publishers->m_imu_msg.angular_velocity_covariance.begin());
  }

  if(has_quat)
  {
    //Since the MIP_AHRS data does not contain uncertainty values we have to set them based on the parameter values.
    std::copy(m_config->m_imu_orientation_cov.begin(), m_config->m_imu_orientation_cov.end(), m_publishers->m_imu_msg.orientation_covariance.begin());
  }

  //Publish
  if(m_config->m_publish_gps_corr)
    m_publishers->m_gps_corr_pub->publish(m_publishers->m_gps_corr_msg);

  if(m_config->m_publish_imu)
  {
    m_publishers->m_imu_pub->publish(m_publishers->m_imu_msg);
  
    if(has_mag)
      m_publishers->m_mag_pub->publish(m_publishers->m_mag_msg);
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// MIP Filter Packet Parsing Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

void MicrostrainParser::parse_filter_packet(const mscl::MipDataPacket &packet)
{
  bool gnss_aiding_status_received[NUM_GNSS] = {false};
  bool gnss_dual_antenna_status_received = false;
  int  i;

  //Update diagnostics
  m_filter_valid_packet_count++;

  //Handle time
  uint64_t time = packet.collectedTimestamp().nanoseconds();
  
  //Check if the user wants to use the device timestamp instead of PC collected time
  if(packet.hasDeviceTime() && m_config->m_use_device_timestamp) 
  {
     time = packet.deviceTimestamp().nanoseconds();
  }

  //Filtered IMU timestamp and frame
  set_seq(&m_publishers->m_filtered_imu_msg.header, m_filter_valid_packet_count);
  m_publishers->m_filtered_imu_msg.header.stamp    = to_ros_time(time);
  m_publishers->m_filtered_imu_msg.header.frame_id = m_config->m_filter_frame_id;
  
  //Nav odom timestamp and frame
  set_seq(&m_publishers->m_filter_msg.header, m_filter_valid_packet_count);
  m_publishers->m_filter_msg.header.stamp    = to_ros_time(time);
  m_publishers->m_filter_msg.header.frame_id = m_config->m_filter_frame_id;

  //Nav relative position odom timestamp and frame (note: Relative position frame is NED for both pos and vel)
  set_seq(&m_publishers->m_filter_relative_pos_msg.header, m_filter_valid_packet_count);
  m_publishers->m_filter_relative_pos_msg.header.stamp    = to_ros_time(time);
  m_publishers->m_filter_relative_pos_msg.header.frame_id = m_config->m_filter_child_frame_id;
  m_publishers->m_filter_relative_pos_msg.child_frame_id  = m_config->m_filter_child_frame_id;

  //Get the list of data elements
  const mscl::MipDataPoints &points = packet.data();
 
  //Loop over data elements and map them
  for(mscl::MipDataPoint point : points)
  {
    switch(point.field())
    {
    case mscl::MipTypes::CH_FIELD_ESTFILTER_FILTER_STATUS:
    {
      if(point.qualifier() == mscl::MipTypes::CH_FILTER_STATE) 
      {
        m_publishers->m_filter_status_msg.filter_state = point.as_uint16();
      } 
      else if(point.qualifier() == mscl::MipTypes::CH_DYNAMICS_MODE)
      {
        m_publishers->m_filter_status_msg.dynamics_mode = point.as_uint16();
      }
      else if(point.qualifier() == mscl::MipTypes::CH_FLAGS)
      {
        m_publishers->m_filter_status_msg.status_flags = point.as_uint16();
      }
    }break;  
        
    //Estimated LLH Position
    case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_LLH_POS:
    {
      m_publishers->m_filter_msg.child_frame_id = m_config->m_filter_child_frame_id;

      if(point.qualifier() == mscl::MipTypes::CH_LATITUDE)
      {
        m_curr_filter_pos_lat = point.as_double();

        if(m_config->m_use_enu_frame)
        {
          m_publishers->m_filter_msg.pose.pose.position.y = m_curr_filter_pos_lat;
        }
        else
        {
          m_publishers->m_filter_msg.pose.pose.position.x = m_curr_filter_pos_lat;          
        }
        
      }
      else if(point.qualifier() == mscl::MipTypes::CH_LONGITUDE)
      {
        m_curr_filter_pos_long = point.as_double();

        if(m_config->m_use_enu_frame)
        {
          m_publishers->m_filter_msg.pose.pose.position.x = m_curr_filter_pos_long;
        }
        else
        {          
          m_publishers->m_filter_msg.pose.pose.position.y = m_curr_filter_pos_long;
        }
      }
      else if(point.qualifier() == mscl::MipTypes::CH_HEIGHT_ABOVE_ELLIPSOID)
      {
        m_curr_filter_pos_height          = point.as_double();
        m_publishers->m_filter_msg.pose.pose.position.z = m_curr_filter_pos_height;
      }
    }break;

    //Estimated NED Velocity
    case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_NED_VELOCITY: 
    {
      if(point.qualifier() == mscl::MipTypes::CH_NORTH)
      {
        m_curr_filter_vel_north = point.as_float();

        if(m_config->m_use_enu_frame)
        {
          m_publishers->m_filter_msg.twist.twist.linear.y              = m_curr_filter_vel_north;
          m_publishers->m_filter_relative_pos_msg.twist.twist.linear.y = m_curr_filter_vel_north;          
        }
        else
        {
          m_publishers->m_filter_msg.twist.twist.linear.x              = m_curr_filter_vel_north;
          m_publishers->m_filter_relative_pos_msg.twist.twist.linear.x = m_curr_filter_vel_north;          
        }        
      }
      else if(point.qualifier() == mscl::MipTypes::CH_EAST)
      {
        m_curr_filter_vel_east = point.as_float();

        if(m_config->m_use_enu_frame)
        {
          m_publishers->m_filter_msg.twist.twist.linear.x              = m_curr_filter_vel_east;
          m_publishers->m_filter_relative_pos_msg.twist.twist.linear.x = m_curr_filter_vel_east;
        }
        else
        {
          m_publishers->m_filter_msg.twist.twist.linear.y              = m_curr_filter_vel_east;
          m_publishers->m_filter_relative_pos_msg.twist.twist.linear.y = m_curr_filter_vel_east;
        }
      }
      else if(point.qualifier() == mscl::MipTypes::CH_DOWN)
      {
        m_curr_filter_vel_down = point.as_float();

        if(m_config->m_use_enu_frame)
        {
          m_publishers->m_filter_msg.twist.twist.linear.z              = -m_curr_filter_vel_down;
          m_publishers->m_filter_relative_pos_msg.twist.twist.linear.z = -m_curr_filter_vel_down;
        }
        else
        {
          m_publishers->m_filter_msg.twist.twist.linear.z              = m_curr_filter_vel_down;
          m_publishers->m_filter_relative_pos_msg.twist.twist.linear.z = m_curr_filter_vel_down;
        }
      }
    }break;

    case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_EULER:
    {
      if(point.qualifier() == mscl::MipTypes::CH_ROLL)
      {
        m_curr_filter_roll = point.as_float();
      }
      else if(point.qualifier() == mscl::MipTypes::CH_PITCH)
      {
        m_curr_filter_pitch = point.as_float();
      }
      else if(point.qualifier() == mscl::MipTypes::CH_YAW)
      {
        m_curr_filter_yaw = point.as_float();

        if(m_config->m_use_enu_frame)
        {

        }
        else
        {
          m_publishers->m_filter_heading_msg.heading_deg = m_curr_filter_yaw*180.0/3.14;
          m_publishers->m_filter_heading_msg.heading_rad = m_curr_filter_yaw;
        }
      }
      else if(point.qualifier() == mscl::MipTypes::CH_FLAGS)
      {
        m_publishers->m_filter_heading_msg.status_flags = point.as_uint16();
      }
    }break;

    case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_QUATERNION:
    { 
      mscl::Vector quaternion  = point.as_Vector();
      m_curr_filter_quaternion = quaternion;
      
      if(m_config->m_use_enu_frame)
      {
        tf2::Quaternion q_ned2enu, qbody2ned(quaternion.as_floatAt(1), quaternion.as_floatAt(2), quaternion.as_floatAt(3), quaternion.as_floatAt(0));
        m_config->m_t_ned2enu.getRotation(q_ned2enu);
        m_publishers->m_filter_msg.pose.pose.orientation = tf2::toMsg(q_ned2enu*qbody2ned);
      }
      else
      {
        m_publishers->m_filter_msg.pose.pose.orientation.x  = quaternion.as_floatAt(1);
        m_publishers->m_filter_msg.pose.pose.orientation.y  = quaternion.as_floatAt(2);
        m_publishers->m_filter_msg.pose.pose.orientation.z  = quaternion.as_floatAt(3);
        m_publishers->m_filter_msg.pose.pose.orientation.w  = quaternion.as_floatAt(0); 
      }
      
      m_publishers->m_filtered_imu_msg.orientation                  = m_publishers->m_filter_msg.pose.pose.orientation;
      m_publishers->m_filter_relative_pos_msg.pose.pose.orientation = m_publishers->m_filter_msg.pose.pose.orientation;
    }break;

    case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ANGULAR_RATE:
    {
      if(point.qualifier() == mscl::MipTypes::CH_X)
      {
        m_curr_filter_angular_rate_x              = point.as_float();
        m_publishers->m_filter_msg.twist.twist.angular.x        = m_curr_filter_angular_rate_x;
        m_publishers->m_filtered_imu_msg.angular_velocity.x     = m_curr_filter_angular_rate_x;
        m_publishers->m_filter_relative_pos_msg.twist.twist.angular.x = m_curr_filter_angular_rate_x;
      }
      else if(point.qualifier() == mscl::MipTypes::CH_Y)
      {
        m_curr_filter_angular_rate_y              = point.as_float();
        m_publishers->m_filter_msg.twist.twist.angular.y        = m_curr_filter_angular_rate_y;
        m_publishers->m_filtered_imu_msg.angular_velocity.y     = m_curr_filter_angular_rate_y;
        m_publishers->m_filter_relative_pos_msg.twist.twist.angular.y = m_curr_filter_angular_rate_y;
      }
      else if(point.qualifier() == mscl::MipTypes::CH_Z)
      {
        m_curr_filter_angular_rate_z              = point.as_float();
        m_publishers->m_filter_msg.twist.twist.angular.z        = m_curr_filter_angular_rate_z;
        m_publishers->m_filtered_imu_msg.angular_velocity.z     = m_curr_filter_angular_rate_z;
        m_publishers->m_filter_relative_pos_msg.twist.twist.angular.z = m_curr_filter_angular_rate_z;
      }
    }break;
    
    case mscl::MipTypes::CH_FIELD_ESTFILTER_COMPENSATED_ACCEL:
    {
      if (point.qualifier() == mscl::MipTypes::CH_X)
      {
        m_publishers->m_filtered_imu_msg.linear_acceleration.x = point.as_float();
      }
      else if (point.qualifier() == mscl::MipTypes::CH_Y)
      {
        m_publishers->m_filtered_imu_msg.linear_acceleration.y = point.as_float();
      }
      else if (point.qualifier() == mscl::MipTypes::CH_Z)
      {
        m_publishers->m_filtered_imu_msg.linear_acceleration.z = point.as_float();
      }
    }break;

    case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_LLH_UNCERT:
    {
      if(point.qualifier() == mscl::MipTypes::CH_NORTH)
      {
        m_curr_filter_pos_uncert_north = point.as_float();

        if(m_config->m_use_enu_frame)
        {
          m_publishers->m_filter_msg.pose.covariance[7]              = pow(m_curr_filter_pos_uncert_north, 2);
          m_publishers->m_filter_relative_pos_msg.pose.covariance[7] = m_publishers->m_filter_msg.pose.covariance[7];
        }
        else
        {
          m_publishers->m_filter_msg.pose.covariance[0]              = pow(m_curr_filter_pos_uncert_north, 2);
          m_publishers->m_filter_relative_pos_msg.pose.covariance[0] = m_publishers->m_filter_msg.pose.covariance[0];
        }
      }
      else if(point.qualifier() == mscl::MipTypes::CH_EAST)
      {
        m_curr_filter_pos_uncert_east = point.as_float();

        if(m_config->m_use_enu_frame)
        {
          m_publishers->m_filter_msg.pose.covariance[0]              = pow(m_curr_filter_pos_uncert_east, 2);
          m_publishers->m_filter_relative_pos_msg.pose.covariance[0] = m_publishers->m_filter_msg.pose.covariance[0];
        }
        else
        {
          m_publishers->m_filter_msg.pose.covariance[7]              = pow(m_curr_filter_pos_uncert_east, 2);
          m_publishers->m_filter_relative_pos_msg.pose.covariance[7] = m_publishers->m_filter_msg.pose.covariance[7];
        }
      }
      else if(point.qualifier() == mscl::MipTypes::CH_DOWN)
      {
        m_curr_filter_pos_uncert_down    = point.as_float();
        m_publishers->m_filter_msg.pose.covariance[14] = pow(m_curr_filter_pos_uncert_down, 2);
        m_publishers->m_filter_relative_pos_msg.pose.covariance[14] = m_publishers->m_filter_msg.pose.covariance[14];
      }
    }
    break;

    case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_NED_UNCERT:
    {
      if(point.qualifier() == mscl::MipTypes::CH_NORTH)
      {
        m_curr_filter_vel_uncert_north = point.as_float();

        if(m_config->m_use_enu_frame)
        {
          m_publishers->m_filter_msg.twist.covariance[0]              = pow(m_curr_filter_vel_uncert_north, 2);
          m_publishers->m_filter_relative_pos_msg.twist.covariance[0] = m_publishers->m_filter_msg.twist.covariance[0];
        }
        else
        {
          m_publishers->m_filter_msg.twist.covariance[7]              = pow(m_curr_filter_vel_uncert_north, 2);
          m_publishers->m_filter_relative_pos_msg.twist.covariance[7] = m_publishers->m_filter_msg.twist.covariance[7];          
        }
        
      }
      else if(point.qualifier() == mscl::MipTypes::CH_EAST)
      {
        m_curr_filter_vel_uncert_east = point.as_float();

        if(m_config->m_use_enu_frame)
        {
          m_publishers->m_filter_msg.twist.covariance[7]              = pow(m_curr_filter_vel_uncert_east, 2);
          m_publishers->m_filter_relative_pos_msg.twist.covariance[7] = m_publishers->m_filter_msg.twist.covariance[7];
        }
        else
        {
          m_publishers->m_filter_msg.twist.covariance[0]              = pow(m_curr_filter_vel_uncert_east, 2);
          m_publishers->m_filter_relative_pos_msg.twist.covariance[0] = m_publishers->m_filter_msg.twist.covariance[0];         
        }
        
      }
      else if(point.qualifier() == mscl::MipTypes::CH_DOWN)
      {
        m_curr_filter_vel_uncert_down     = point.as_float();
        m_publishers->m_filter_msg.twist.covariance[14] = pow(m_curr_filter_vel_uncert_down, 2);
        m_publishers->m_filter_relative_pos_msg.twist.covariance[14] = m_publishers->m_filter_msg.twist.covariance[14];
      }
    }
    break;

    case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ATT_UNCERT_EULER:
    {
      if(point.qualifier() == mscl::MipTypes::CH_ROLL)
      {
        m_curr_filter_att_uncert_roll                 = point.as_float();
        m_publishers->m_filter_msg.pose.covariance[21]              = pow(m_curr_filter_att_uncert_roll, 2);
        m_publishers->m_filtered_imu_msg.orientation_covariance[0]  = m_publishers->m_filter_msg.pose.covariance[21];
        m_publishers->m_filter_relative_pos_msg.pose.covariance[21] = m_publishers->m_filter_msg.pose.covariance[21];
      }
      else if(point.qualifier() == mscl::MipTypes::CH_PITCH)
      {
        m_curr_filter_att_uncert_pitch                = point.as_float();
        m_publishers->m_filter_msg.pose.covariance[28]              = pow(m_curr_filter_att_uncert_pitch, 2);
        m_publishers->m_filtered_imu_msg.orientation_covariance[4]  = m_publishers->m_filter_msg.pose.covariance[28];
        m_publishers->m_filter_relative_pos_msg.pose.covariance[28] = m_publishers->m_filter_msg.pose.covariance[28];
      }
      else if(point.qualifier() == mscl::MipTypes::CH_YAW)
      {
        m_curr_filter_att_uncert_yaw                  = point.as_float();
        m_publishers->m_filter_msg.pose.covariance[35]              = pow(m_curr_filter_att_uncert_yaw, 2);
        m_publishers->m_filtered_imu_msg.orientation_covariance[8]  = m_publishers->m_filter_msg.pose.covariance[35];
        m_publishers->m_filter_relative_pos_msg.pose.covariance[35] = m_publishers->m_filter_msg.pose.covariance[35];
      }
    }break;


    case mscl::MipTypes::CH_FIELD_ESTFILTER_HEADING_UPDATE_SOURCE: 
    {
      if(point.qualifier() == mscl::MipTypes::CH_HEADING) 
      {
        m_publishers->m_filter_heading_state_msg.heading_rad = point.as_float();
      } 
      else if(point.qualifier() == mscl::MipTypes::CH_HEADING_UNCERTAINTY)
      {
        m_publishers->m_filter_heading_state_msg.heading_uncertainty = point.as_float();
      }
      else if(point.qualifier() == mscl::MipTypes::CH_SOURCE)
      {
        m_publishers->m_filter_heading_state_msg.source = point.as_uint16();
      }
      else if(point.qualifier() == mscl::MipTypes::CH_FLAGS)
      {
        m_publishers->m_filter_heading_state_msg.status_flags = point.as_uint16();
      }
    }break;  

    case mscl::MipTypes::CH_FIELD_ESTFILTER_NED_RELATIVE_POS: 
    {
      if(point.qualifier() == mscl::MipTypes::CH_X)
      {
        double rel_pos_north = point.as_double();

        if(m_config->m_use_enu_frame)
          m_publishers->m_filter_relative_pos_msg.pose.pose.position.y = rel_pos_north;
        else
          m_publishers->m_filter_relative_pos_msg.pose.pose.position.x = rel_pos_north;
      }
      else if(point.qualifier() == mscl::MipTypes::CH_Y)
      {
        double rel_pos_east = point.as_double();

        if(m_config->m_use_enu_frame)
          m_publishers->m_filter_relative_pos_msg.pose.pose.position.x = rel_pos_east;
        else
          m_publishers->m_filter_relative_pos_msg.pose.pose.position.y = rel_pos_east;
      }
      else if(point.qualifier() == mscl::MipTypes::CH_Z)
      {
        double rel_pos_down = point.as_double();

        if(m_config->m_use_enu_frame)
          m_publishers->m_filter_relative_pos_msg.pose.pose.position.z = -rel_pos_down;
        else
          m_publishers->m_filter_relative_pos_msg.pose.pose.position.z = rel_pos_down;
      }
    }break;

    case mscl::MipTypes::CH_FIELD_ESTFILTER_POSITION_AIDING_STATUS:
    {
      if(point.hasAddlIdentifiers())
      {
        int gnss_id = static_cast<int>(point.addlIdentifiers()[0].id()) - 1;

        if(gnss_id < 0 || gnss_id >= NUM_GNSS)
        {}
        else if(point.qualifier() == mscl::MipTypes::CH_TIME_OF_WEEK)
        {        
          m_publishers->m_gnss_aiding_status_msg[gnss_id].gps_tow = point.as_double();
          gnss_aiding_status_received[gnss_id] = true;
        }
        else if(point.qualifier() == mscl::MipTypes::CH_STATUS)
        {
          uint16_t status_flags = point.as_uint16(); 
           
          m_publishers->m_gnss_aiding_status_msg[gnss_id].has_position_fix         = (status_flags & mscl::InertialTypes::GnssAidingStatus::GNSS_AIDING_NO_FIX) == 0;
          m_publishers->m_gnss_aiding_status_msg[gnss_id].tight_coupling           = status_flags & mscl::InertialTypes::GnssAidingStatus::GNSS_AIDING_TIGHT_COUPLING;
          m_publishers->m_gnss_aiding_status_msg[gnss_id].differential_corrections = status_flags & mscl::InertialTypes::GnssAidingStatus::GNSS_AIDING_DIFFERENTIAL;
          m_publishers->m_gnss_aiding_status_msg[gnss_id].integer_fix              = status_flags & mscl::InertialTypes::GnssAidingStatus::GNSS_AIDING_INTEGER_FIX;
          m_publishers->m_gnss_aiding_status_msg[gnss_id].using_gps                = status_flags & mscl::InertialTypes::GnssAidingStatus::GNSS_AIDING_GPS;
          m_publishers->m_gnss_aiding_status_msg[gnss_id].using_glonass            = status_flags & mscl::InertialTypes::GnssAidingStatus::GNSS_AIDING_GLONASS;
          m_publishers->m_gnss_aiding_status_msg[gnss_id].using_galileo            = status_flags & mscl::InertialTypes::GnssAidingStatus::GNSS_AIDING_GALILEO;
          m_publishers->m_gnss_aiding_status_msg[gnss_id].using_beidou             = status_flags & mscl::InertialTypes::GnssAidingStatus::GNSS_AIDING_BEIDOU;        
        }
        else
        {
          MICROSTRAIN_INFO(m_node, "Point Qualifier %d", (int)point.qualifier());
        }
      }
    }break;

    case mscl::MipTypes::CH_FIELD_ESTFILTER_GNSS_DUAL_ANTENNA_STATUS:
    {
      if (point.qualifier() == mscl::MipTypes::CH_TIME_OF_WEEK)
      {
        gnss_dual_antenna_status_received = true;
        m_publishers->m_gnss_dual_antenna_status_msg.gps_tow = point.as_float();
      }
      else if (point.qualifier() == mscl::MipTypes::CH_HEADING)
      {
        m_publishers->m_gnss_dual_antenna_status_msg.heading = point.as_float();
      }
      else if (point.qualifier() == mscl::MipTypes::CH_HEADING_UNCERTAINTY)
      {
        m_publishers->m_gnss_dual_antenna_status_msg.heading_uncertainty = point.as_float();
      }
      else if (point.qualifier() == mscl::MipTypes::CH_FIX_TYPE)
      {
        m_publishers->m_gnss_dual_antenna_status_msg.fix_type = point.as_uint8();
      }
      else if (point.qualifier() == mscl::MipTypes::CH_STATUS)
      {
        uint16_t status_flags = point.as_uint16();

        m_publishers->m_gnss_dual_antenna_status_msg.rcv_1_valid           = status_flags & mscl::InertialTypes::DualAntennaStatusFlags::DATA_VALID_REC_1;
        m_publishers->m_gnss_dual_antenna_status_msg.rcv_2_valid           = status_flags & mscl::InertialTypes::DualAntennaStatusFlags::DATA_VALID_REC_2;
        m_publishers->m_gnss_dual_antenna_status_msg.antenna_offsets_valid = status_flags & mscl::InertialTypes::DualAntennaStatusFlags::ANTENNA_OFFSETS_VALID;
      }
    }break;
    
    default: break;
    }
  }

  //Copy fixed covariances to the filtered IMU message
  std::copy(m_config->m_imu_linear_cov.begin(),  m_config->m_imu_linear_cov.end(),  m_publishers->m_filtered_imu_msg.linear_acceleration_covariance.begin());
  std::copy(m_config->m_imu_angular_cov.begin(), m_config->m_imu_angular_cov.end(), m_publishers->m_filtered_imu_msg.angular_velocity_covariance.begin());

  //Publish
  if(m_config->m_publish_filter)
  {
    m_publishers->m_filtered_imu_pub->publish(m_publishers->m_filtered_imu_msg);
    m_publishers->m_filter_pub->publish(m_publishers->m_filter_msg);
    m_publishers->m_filter_status_pub->publish(m_publishers->m_filter_status_msg);
    m_publishers->m_filter_heading_pub->publish(m_publishers->m_filter_heading_msg);
    m_publishers->m_filter_heading_state_pub->publish(m_publishers->m_filter_heading_state_msg);

    if(m_config->m_publish_gnss_dual_antenna_status && gnss_dual_antenna_status_received)
      m_publishers->m_gnss_dual_antenna_status_pub->publish(m_publishers->m_gnss_dual_antenna_status_msg);
  }
  
  if(m_config->m_publish_filter_relative_pos)
    m_publishers->m_filter_relative_pos_pub->publish(m_publishers->m_filter_relative_pos_msg); 

  for(i=0; i<NUM_GNSS; i++)
  {
    if(m_config->m_publish_gnss_aiding_status[i] && gnss_aiding_status_received[i])
      m_publishers->m_gnss_aiding_status_pub[i]->publish(m_publishers->m_gnss_aiding_status_msg[i]); 
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// MIP GNSS Packet Parsing Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

void MicrostrainParser::parse_gnss_packet(const mscl::MipDataPacket &packet, int gnss_id)
{
  //Rnage-check id
  if(gnss_id >= NUM_GNSS)
    return;

  //Update diagnostics
  m_gnss_valid_packet_count[gnss_id]++;
  
  //Handle time
  uint64_t time       = packet.collectedTimestamp().nanoseconds();
  bool     time_valid = false;

  //Check if the user wants to use the device timestamp instead of PC collected time
  if(packet.hasDeviceTime() && m_config->m_use_device_timestamp) 
  {
     time       = packet.deviceTimestamp().nanoseconds();
     time_valid = true;
  }

  //GPS Fix time
  set_seq(&m_publishers->m_gnss_msg[gnss_id].header, m_gnss_valid_packet_count[gnss_id]);
  m_publishers->m_gnss_msg[gnss_id].header.stamp    = to_ros_time(time);
  m_publishers->m_gnss_msg[gnss_id].header.frame_id = m_config->m_gnss_frame_id[gnss_id];
  
  //GPS Odom time
  set_seq(&m_publishers->m_gnss_odom_msg[gnss_id].header, m_gnss_valid_packet_count[gnss_id]);
  m_publishers->m_gnss_odom_msg[gnss_id].header.stamp    = to_ros_time(time);
  m_publishers->m_gnss_odom_msg[gnss_id].header.frame_id = m_config->m_gnss_frame_id[gnss_id];
  //m_publishers->m_gnss_odom_msg[gnss_id].child_frame_id  = m_config->m_gnss_odom_child_frame_id[gnss_id];

  //GPS Time reference
  set_seq(&m_publishers->m_gnss_time_msg[gnss_id].header, m_gnss_valid_packet_count[gnss_id]);
  m_publishers->m_gnss_time_msg[gnss_id].header.stamp    = ros_time_now();
  m_publishers->m_gnss_time_msg[gnss_id].header.frame_id = m_config->m_gnss_frame_id[gnss_id];
  m_publishers->m_gnss_time_msg[gnss_id].time_ref        = to_ros_time(time);

  //Get the list of data elements
  const mscl::MipDataPoints &points = packet.data();

  //Loop over data elements and map them
  for(mscl::MipDataPoint point : points)
  {
    switch(point.field())
    {
    //LLH Position
    case mscl::MipTypes::CH_FIELD_GNSS_LLH_POSITION:
    case mscl::MipTypes::CH_FIELD_GNSS_1_LLH_POSITION:
    case mscl::MipTypes::CH_FIELD_GNSS_2_LLH_POSITION:
    {
      if(point.qualifier() == mscl::MipTypes::CH_LATITUDE)
      {
        m_publishers->m_gnss_msg[gnss_id].latitude = point.as_double();

        if(m_config->m_use_enu_frame)
          m_publishers->m_gnss_odom_msg[gnss_id].pose.pose.position.y = m_publishers->m_gnss_msg[gnss_id].latitude;
        else
          m_publishers->m_gnss_odom_msg[gnss_id].pose.pose.position.x = m_publishers->m_gnss_msg[gnss_id].latitude;
          
      }
      else if(point.qualifier() == mscl::MipTypes::CH_LONGITUDE)
      {
        m_publishers->m_gnss_msg[gnss_id].longitude = point.as_double();

        if(m_config->m_use_enu_frame)
          m_publishers->m_gnss_odom_msg[gnss_id].pose.pose.position.x = m_publishers->m_gnss_msg[gnss_id].longitude;
        else
          m_publishers->m_gnss_odom_msg[gnss_id].pose.pose.position.y = m_publishers->m_gnss_msg[gnss_id].longitude;
      }
      else if(point.qualifier() == mscl::MipTypes::CH_HEIGHT_ABOVE_ELLIPSOID)
      {
        m_publishers->m_gnss_msg[gnss_id].altitude                  = point.as_double();
        m_publishers->m_gnss_odom_msg[gnss_id].pose.pose.position.z = m_publishers->m_gnss_msg[gnss_id].altitude;
      }
      else if(point.qualifier() == mscl::MipTypes::CH_HORIZONTAL_ACCURACY)
      {
        //Horizontal covariance maps to lat and lon
        m_publishers->m_gnss_msg[gnss_id].position_covariance[0]  = pow(point.as_float(), 2);
        m_publishers->m_gnss_msg[gnss_id].position_covariance[4]  = m_publishers->m_gnss_msg[gnss_id].position_covariance[0];
        m_publishers->m_gnss_odom_msg[gnss_id].pose.covariance[0] = m_publishers->m_gnss_msg[gnss_id].position_covariance[0];
        m_publishers->m_gnss_odom_msg[gnss_id].pose.covariance[7] = m_publishers->m_gnss_msg[gnss_id].position_covariance[0];
      }
      else if(point.qualifier() == mscl::MipTypes::CH_VERTICAL_ACCURACY)
      {
        m_publishers->m_gnss_msg[gnss_id].position_covariance[8]   = pow(point.as_float(), 2);
        m_publishers->m_gnss_odom_msg[gnss_id].pose.covariance[14] = m_publishers->m_gnss_msg[gnss_id].position_covariance[8];
      }

      m_publishers->m_gnss_msg[gnss_id].status.service           = 1;
      m_publishers->m_gnss_msg[gnss_id].position_covariance_type = 2;
    }break;
    
    case mscl::MipTypes::ChannelField::CH_FIELD_GNSS_NED_VELOCITY:
    case mscl::MipTypes::ChannelField::CH_FIELD_GNSS_1_NED_VELOCITY:
    case mscl::MipTypes::ChannelField::CH_FIELD_GNSS_2_NED_VELOCITY:
    {
      if(point.qualifier() == mscl::MipTypes::CH_NORTH)
      {
        float north_velocity = point.as_float();

        if(m_config->m_use_enu_frame)
          m_publishers->m_gnss_odom_msg[gnss_id].twist.twist.linear.y = north_velocity;
        else
          m_publishers->m_gnss_odom_msg[gnss_id].twist.twist.linear.x = north_velocity;
      }
      else if(point.qualifier() == mscl::MipTypes::CH_EAST)
      {
        float east_velocity = point.as_float();
        
        if(m_config->m_use_enu_frame)
          m_publishers->m_gnss_odom_msg[gnss_id].twist.twist.linear.x = east_velocity;
        else
          m_publishers->m_gnss_odom_msg[gnss_id].twist.twist.linear.y = east_velocity;
      }
      else if(point.qualifier() == mscl::MipTypes::CH_DOWN)
      {
        float down_velocity = point.as_float();

        if(m_config->m_use_enu_frame)
          m_publishers->m_gnss_odom_msg[gnss_id].twist.twist.linear.z = -down_velocity;
        else
          m_publishers->m_gnss_odom_msg[gnss_id].twist.twist.linear.z = down_velocity;
      }
    }break;
    }
  }

  //Publish
  if(m_config->m_publish_gnss[gnss_id])
  {
    m_publishers->m_gnss_pub[gnss_id]->publish(m_publishers->m_gnss_msg[gnss_id]);
    m_publishers->m_gnss_odom_pub[gnss_id]->publish(m_publishers->m_gnss_odom_msg[gnss_id]);

    if(time_valid)
      m_publishers->m_gnss_time_pub[gnss_id]->publish(m_publishers->m_gnss_time_msg[gnss_id]);
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// MIP RTK Packet Parsing Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

void MicrostrainParser::parse_rtk_packet(const mscl::MipDataPacket& packet)
{
  //Update diagnostics
  m_rtk_valid_packet_count++;
  
  //Handle time
  uint64_t time = packet.collectedTimestamp().nanoseconds();

  //Check if the user wants to use the device timestamp instead of PC collected time
  if(packet.hasDeviceTime() && m_config->m_use_device_timestamp) 
    time = packet.deviceTimestamp().nanoseconds();

  //Get the list of data elements
  const mscl::MipDataPoints &points = packet.data();

  //Loop over data elements and map them
  for(mscl::MipDataPoint point : points)
  {
   switch(point.field())
    {
      //RTK Correction Status
      case mscl::MipTypes::CH_FIELD_GNSS_3_RTK_CORRECTIONS_STATUS:
      {
        if(point.qualifier() == mscl::MipTypes::CH_TIME_OF_WEEK)
        {        
          m_publishers->m_rtk_msg.gps_tow = point.as_double();
        }
        else if(point.qualifier() == mscl::MipTypes::CH_WEEK_NUMBER)
        {        
          m_publishers->m_rtk_msg.gps_week = point.as_uint16();
        }
        else if(point.qualifier() == mscl::MipTypes::CH_STATUS)
        {        
          m_publishers->m_rtk_msg.epoch_status = point.as_uint16();
        }
        else if(point.qualifier() == mscl::MipTypes::CH_FLAGS)
        {        
          //Decode dongle status
          mscl::RTKDeviceStatusFlags dongle_status(point.as_uint32());

          m_publishers->m_rtk_msg.dongle_controller_state  = dongle_status.controllerState(); 
          m_publishers->m_rtk_msg.dongle_platform_state 	 = dongle_status.platformState(); 
          m_publishers->m_rtk_msg.dongle_controller_status = dongle_status.controllerStatusCode(); 
          m_publishers->m_rtk_msg.dongle_platform_status 	 = dongle_status.platformStatusCode(); 
          m_publishers->m_rtk_msg.dongle_reset_reason 	   = dongle_status.resetReason(); 
          m_publishers->m_rtk_msg.dongle_signal_quality		 = dongle_status.signalQuality(); 
        }
        else if(point.qualifier() == mscl::MipTypes::CH_GPS_CORRECTION_LATENCY)
        {
          m_publishers->m_rtk_msg.gps_correction_latency = point.as_float();
        }
        else if(point.qualifier() == mscl::MipTypes::CH_GLONASS_CORRECTION_LATENCY)
        {
          m_publishers->m_rtk_msg.glonass_correction_latency = point.as_float();
        }
        else if(point.qualifier() == mscl::MipTypes::CH_GALILEO_CORRECTION_LATENCY)
        {
          m_publishers->m_rtk_msg.galileo_correction_latency = point.as_float();
        }
        else if(point.qualifier() == mscl::MipTypes::CH_BEIDOU_CORRECTION_LATENCY)
        {
          m_publishers->m_rtk_msg.beidou_correction_latency = point.as_float();
        }
      }break;
    }
  }
 
  //Publish
  if(m_config->m_publish_rtk)
    m_publishers->m_rtk_pub->publish(m_publishers->m_rtk_msg);
}

void MicrostrainParser::print_packet_stats()
{
  if(m_config->m_inertial_device)
  {    
    return;
  }
  
  if(m_config->m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_DEVICE_STATUS))
  {
    if(m_config->m_inertial_device->features().supportedStatusSelectors().size() > 1) 
    {
      mscl::DeviceStatusData status = m_config->m_inertial_device->getDiagnosticDeviceStatus();

      m_imu_valid_packet_count          = status.imuMessageInfo().messagesRead;
      m_imu_checksum_error_packet_count = status.imuMessageInfo().messageParsingErrors;
      m_imu_timeout_packet_count        = status.imuStreamInfo().outgoingPacketsDropped;
      m_filter_timeout_packet_count     = status.estimationFilterStreamInfo().outgoingPacketsDropped;

      MICROSTRAIN_DEBUG_THROTTLE(m_node, 1.0, "%u IMU (%u errors) Packets",
                           m_imu_valid_packet_count, m_imu_timeout_packet_count + m_imu_checksum_error_packet_count);
      
      m_gnss_checksum_error_packet_count[GNSS1_ID] = status.gnssMessageInfo().messageParsingErrors;
      m_gnss_valid_packet_count[GNSS1_ID]          = status.gnssMessageInfo().messagesRead;
      m_gnss_timeout_packet_count[GNSS1_ID]        = status.gnssStreamInfo().outgoingPacketsDropped;

      MICROSTRAIN_DEBUG_THROTTLE(m_node, 1.0, "%u FILTER (%u errors)    %u IMU (%u errors)    %u GPS (%u errors) Packets",
                         m_filter_valid_packet_count,  m_filter_timeout_packet_count,
                         m_imu_valid_packet_count,     m_imu_timeout_packet_count + m_imu_checksum_error_packet_count,
                         m_gnss_valid_packet_count[GNSS1_ID], m_gnss_timeout_packet_count[GNSS1_ID] + m_gnss_checksum_error_packet_count[GNSS1_ID]);

            MICROSTRAIN_DEBUG_THROTTLE(m_node, 1.0, "%u FILTER (%u errors)    %u IMU (%u errors) Packets",
                         m_filter_valid_packet_count, m_filter_timeout_packet_count,
                         m_imu_valid_packet_count,    m_imu_timeout_packet_count + m_imu_checksum_error_packet_count);
    }
  }
}

} // namespace Microstrain
