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
#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_PUBLISHERS_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_PUBLISHERS_H

#include "mip/mip_all.hpp"

#include "microstrain_inertial_driver_common/utils/ros_compat.h"
#include "microstrain_inertial_driver_common/config.h"

namespace microstrain
{

/**
 * Contains ROS messages and the publishers that will publish them
 */
class Publishers
{
public:
  /**
   * \brief Default Constructor
   */
  Publishers() = default;

  /**
   * \brief Constructs this class with a reference to the node, and a config object
   * \param node  Reference to a node that will be saved to this class and used to log and interact with ROS
   * \param config Reference to the config object that will be saved to this class and used to determine whether or not to publish
   */
  Publishers(RosNodeType* node, Config* config);

  /**
   * \brief Configures the publishers. After this function is called, the publishers will be created, but (ROS2 only) will not be activated
   * \return true if configuration was successful and false if configuration failed
   */
  bool configure();

  /**
   * \brief Only technically needed for ROS2 lifecycle publishers, this will activate the publishers, so they are ready to be published on
   * \return true if activation was successful and false if activation failed
   */
  bool activate();

  /**
   * \brief Only technically needed for ROS2 lifecycle publishers, this will deactivate the publishers, so they can be reconfigured and reactivated
   * \return true if deactivation was successful and false if deactivation failed
   */
  bool deactivate();

  /**
   * \brief Publishes all messages that have changed and are configured to be published
   */
  void publish();

  template<typename MessageType>
  class Publisher
  {
   public:
    using SharedPtr = std::shared_ptr<Publisher<MessageType>>;
    using SharedPtrVec = std::vector<SharedPtr>;

    Publisher(const std::string& topic) : topic_(topic)
    {
      message_ = std::make_shared<MessageType>();
    }

    static SharedPtr initialize(const std::string& topic)
    {
      return std::make_shared<Publisher<MessageType>>(topic);
    }

    static SharedPtrVec initializeVec(const std::vector<std::string>& topics)
    {
      SharedPtrVec ptrs;
      for (const auto& topic : topics)
        ptrs.push_back(initialize(topic));
      return ptrs;
    }

    operator bool() const
    {
      return publisher_ != nullptr;
    }

    void configure(RosNodeType* node)
    {
      publisher_ = create_publisher<MessageType>(node, topic_, 100);
    }

    void configure(RosNodeType* node, Config* config)
    {
      if (config->mip_publisher_mapping_->shouldPublish(topic_))
        configure(node);
    }

    void activate()
    {
      if (publisher_ != nullptr)
        publisher_->on_activate();
    }

    void deactivate()
    {
      if (publisher_ != nullptr)
        publisher_->on_deactivate();
    }

    void publish()
    {
      if (publisher_ != nullptr && message_ != nullptr && updated_)
      {
        publisher_->publish(*message_);
        updated_ = false;
      }
    }

    std::string topic() const
    {
      return topic_;
    }

    bool updated() const
    {
      return updated_;
    }

    typename RosPubType<MessageType>::MessageSharedPtr getMessage()
    {
      return message_;
    }

    typename RosPubType<MessageType>::MessageSharedPtr getMessageToUpdate()
    {
      updated_ = true;
      return getMessage();
    }


   private:
    const std::string topic_;
    bool updated_;

    typename RosPubType<MessageType>::MessageSharedPtr message_;
    typename RosPubType<MessageType>::SharedPtr publisher_;
  };


  // IMU Publishers
  Publisher<ImuMsg>::SharedPtr                            imu_pub_      = Publisher<ImuMsg>::initialize(IMU_DATA_TOPIC);
  Publisher<MagneticFieldMsg>::SharedPtr                  mag_pub_      = Publisher<MagneticFieldMsg>::initialize(IMU_MAG_TOPIC);
  Publisher<GPSCorrelationTimestampStampedMsg>::SharedPtr gps_corr_pub_ = Publisher<GPSCorrelationTimestampStampedMsg>::initialize(IMU_GPS_CORR_TOPIC);

  // GNSS publishers
  Publisher<NavSatFixMsg>::SharedPtrVec        gnss_pub_               = Publisher<NavSatFixMsg>::initializeVec({GNSS1_NAVSATFIX_TOPIC, GNSS2_NAVSATFIX_TOPIC});
  Publisher<OdometryMsg>::SharedPtrVec         gnss_odom_pub_          = Publisher<OdometryMsg>::initializeVec({GNSS1_ODOM_TOPIC, GNSS2_ODOM_TOPIC});
  Publisher<TimeReferenceMsg>::SharedPtrVec    gnss_time_pub_          = Publisher<TimeReferenceMsg>::initializeVec({GNSS1_TIME_REF_TOPIC, GNSS2_TIME_REF_TOPIC});
  Publisher<GNSSAidingStatusMsg>::SharedPtrVec gnss_aiding_status_pub_ = Publisher<GNSSAidingStatusMsg>::initializeVec({GNSS1_AIDING_STATUS_TOPIC, GNSS2_AIDING_STATUS_TOPIC});
  Publisher<GNSSFixInfoMsg>::SharedPtrVec      gnss_fix_info_pub_      = Publisher<GNSSFixInfoMsg>::initializeVec({GNSS1_FIX_INFO_TOPIC, GNSS2_FIX_INFO_TOPIC});

  // RTK publishers
  Publisher<RTKStatusMsg>::SharedPtr   rtk_pub_    = Publisher<RTKStatusMsg>::initialize(RTK_STATUS_TOPIC);
  Publisher<RTKStatusMsgV1>::SharedPtr rtk_pub_v1_ = Publisher<RTKStatusMsgV1>::initialize(RTK_STATUS_V1_TOPIC);

  // Filter publishers
  Publisher<FilterStatusMsg>::SharedPtr                   filter_status_pub_                     = Publisher<FilterStatusMsg>::initialize(FILTER_STATUS_TOPIC);
  Publisher<FilterHeadingMsg>::SharedPtr                  filter_heading_pub_                    = Publisher<FilterHeadingMsg>::initialize(FILTER_HEADING_TOPIC);
  Publisher<FilterHeadingStateMsg>::SharedPtr             filter_heading_state_pub_              = Publisher<FilterHeadingStateMsg>::initialize(FILTER_HEADING_STATE_TOPIC);
  Publisher<FilterAidingMeasurementSummaryMsg>::SharedPtr filter_aiding_mesaurement_summary_pub_ = Publisher<FilterAidingMeasurementSummaryMsg>::initialize(FILTER_AIDING_SUMMARY_TOPIC);
  Publisher<OdometryMsg>::SharedPtr                       filter_odom_pub_                       = Publisher<OdometryMsg>::initialize(FILTER_ODOM_TOPIC);
  Publisher<OdometryMsg>::SharedPtr                       filter_relative_odom_pub_              = Publisher<OdometryMsg>::initialize(FILTER_RELATIVE_ODOM_TOPIC);
  Publisher<ImuMsg>::SharedPtr                            filter_imu_pub_                        = Publisher<ImuMsg>::initialize(FILTER_IMU_DATA_TOPIC);
  Publisher<GNSSDualAntennaStatusMsg>::SharedPtr          gnss_dual_antenna_status_pub_          = Publisher<GNSSDualAntennaStatusMsg>::initialize(FILTER_DUAL_ANTENNA_STATUS_TOPIC);

  // Device Status publishser
  Publisher<StatusMsg>::SharedPtr device_status_pub_ = Publisher<StatusMsg>::initialize(DEVICE_STATUS_TOPIC);

  // NMEA sentence publisher
  Publisher<NMEASentenceMsg>::SharedPtr nmea_sentence_pub_ = Publisher<NMEASentenceMsg>::initialize(NMEA_SENTENCE_TOPIC);

  // Transform Broadcaster
  TransformBroadcasterType transform_broadcaster_ = nullptr;

  // Published transforms
  TransformStampedMsg filter_relative_transform_msg_;

private:
  template<class DataField, void (Publishers::*Callback)(const DataField&, uint8_t, mip::Timestamp)>
  void registerDataCallback(const uint8_t descriptor_set = DataField::DESCRIPTOR_SET);

  void handleSharedEventSource(const mip::data_shared::EventSource& event_source, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleSharedTicks(const mip::data_shared::Ticks& ticks, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleSharedDeltaTicks(const mip::data_shared::DeltaTicks& delta_ticks, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleSharedGpsTimestamp(const mip::data_shared::GpsTimestamp& gps_timestamp, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleSharedDeltaTime(const mip::data_shared::DeltaTime& delta_time, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleSharedReferenceTimestamp(const mip::data_shared::ReferenceTimestamp& reference_timestamp, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleSharedReferenceTimeDelta(const mip::data_shared::ReferenceTimeDelta& reference_time_delta, const uint8_t descriptor_set, mip::Timestamp timestamp);

  void handleSensorScaledAccel(const mip::data_sensor::ScaledAccel& scaled_accel, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleSensorScaledGyro(const mip::data_sensor::ScaledGyro& scaled_gyro, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleSensorCompQuaternion(const mip::data_sensor::CompQuaternion& comp_quaternion, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleSensorScaledMag(const mip::data_sensor::ScaledMag& scaled_mag, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleSensorGpsTimestamp(const mip::data_sensor::GpsTimestamp& gps_timestamp, const uint8_t descriptor_set, mip::Timestamp timestamp);

  void handleGnssPosLlh(const mip::data_gnss::PosLlh& pos_llh, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleGnssVelNed(const mip::data_gnss::VelNed& vel_ned, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleGnssFixInfo(const mip::data_gnss::FixInfo& fix_info, const uint8_t descriptor_set, mip::Timestamp timestamp);

  void handleRtkCorrectionsStatus(const mip::data_gnss::RtkCorrectionsStatus& rtk_corrections_status, const uint8_t descriptor_set, mip::Timestamp timestamp);

  void handleFilterStatus(const mip::data_filter::Status& status, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterEulerAngles(const mip::data_filter::EulerAngles& euler_angles, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterHeadingUpdateState(const mip::data_filter::HeadingUpdateState& heading_update_state, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterPositionLlh(const mip::data_filter::PositionLlh& position_llh, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterPositionLlhUncertainty(const mip::data_filter::PositionLlhUncertainty& position_llh_uncertainty, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterAttitudeQuaternion(const mip::data_filter::AttitudeQuaternion& attitude_quaternion, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterEulerAnglesUncertainty(const mip::data_filter::EulerAnglesUncertainty& euler_angles_uncertainty, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterVelocityNed(const mip::data_filter::VelocityNed& velocity_ned, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterVelocityNedUncertainty(const mip::data_filter::VelocityNedUncertainty& velocity_ned_uncertainty, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterCompAngularRate(const mip::data_filter::CompAngularRate& comp_angular_rate, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterCompAccel(const mip::data_filter::CompAccel& comp_accel, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterLinearAccel(const mip::data_filter::LinearAccel& linear_accel, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterRelPosNed(const mip::data_filter::RelPosNed& rel_pos_ned, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterGnssPosAidStatus(const mip::data_filter::GnssPosAidStatus& gnss_pos_aid_status, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterGnssDualAntennaStatus(const mip::data_filter::GnssDualAntennaStatus& gnss_dual_antenna_status, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterAidingMeasurementSummary(const mip::data_filter::AidingMeasurementSummary& aiding_measurement_summary, const uint8_t descriptor_set, mip::Timestamp timestamp);

  void updateHeaderTime(RosHeaderType* header, uint8_t descriptor_set, mip::Timestamp timestamp);

  std::vector<std::shared_ptr<mip::C::mip_dispatch_handler>> mip_dispatch_handlers_;

  RosNodeType* node_;
  Config* config_;

  // Mapping between every shared data field and descriptor sets
  std::map<uint8_t, mip::data_shared::EventSource> event_source_mapping_;
  std::map<uint8_t, mip::data_shared::Ticks> ticks_mapping_;
  std::map<uint8_t, mip::data_shared::DeltaTicks> delta_ticks_mapping_;
  std::map<uint8_t, mip::data_shared::GpsTimestamp> gps_timestamp_mapping_;
  std::map<uint8_t, mip::data_shared::DeltaTime> delta_time_mapping_;
  std::map<uint8_t, mip::data_shared::ReferenceTimestamp> reference_timestamp_mapping_;
  std::map<uint8_t, mip::data_shared::ReferenceTimeDelta> reference_time_delta_mapping_;

  // Save the orientation information, as it is used by some other data to transform based on orientation
  tf2::Quaternion filter_attitude_quaternion_ = tf2::Quaternion(0, 0, 0, 1);

};  // struct Publishers

template<class DataField, void (Publishers::*Callback)(const DataField&, uint8_t, mip::Timestamp)>
void Publishers::registerDataCallback(const uint8_t descriptor_set)
{
  // Register a handler for the callback
  mip_dispatch_handlers_.push_back(std::make_shared<mip::C::mip_dispatch_handler>());

  // Pass to the MIP SDK
  config_->mip_device_->device_->registerDataCallback<DataField, Publishers, Callback>(*(mip_dispatch_handlers_.back()), this, descriptor_set);
}

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_PUBLISHERS_H
