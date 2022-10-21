#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MAPPINGS_MIP_PUBLISHER_MAPPING_H_
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MAPPINGS_MIP_PUBLISHER_MAPPING_H_

#include <string>
#include <memory>

#include <mip/mip_all.hpp>

#include "microstrain_inertial_driver_common/utils/mappings/mip_mapping.h"
#include "microstrain_inertial_driver_common/utils/ros_compat.h"
#include "microstrain_inertial_driver_common/utils/mip/mip_device_wrapper.h"

namespace microstrain
{

// Topic names
static constexpr auto IMU_DATA_TOPIC = "imu/data";
static constexpr auto IMU_INTERNAL_TIME_REF_TOPIC = "imu/internal_time_ref";
static constexpr auto IMU_MAG_TOPIC = "mag";
static constexpr auto IMU_GPS_CORR_TOPIC = "gps_corr";

static constexpr auto GNSS1_NAVSATFIX_TOPIC = "gnss1/fix";
static constexpr auto GNSS1_ODOM_TOPIC = "gnss1/odom";
static constexpr auto GNSS1_TIME_REF_TOPIC = "gnss1/time_ref";
static constexpr auto GNSS1_FIX_INFO_TOPIC = "gnss1/fix_info";
static constexpr auto GNSS1_AIDING_STATUS_TOPIC = "gnss1/aiding_status";

static constexpr auto GNSS2_NAVSATFIX_TOPIC = "gnss2/fix";
static constexpr auto GNSS2_ODOM_TOPIC = "gnss2/odom";
static constexpr auto GNSS2_TIME_REF_TOPIC = "gnss2/time_ref";
static constexpr auto GNSS2_FIX_INFO_TOPIC = "gnss2/fix_info";
static constexpr auto GNSS2_AIDING_STATUS_TOPIC = "gnss2/aiding_status";

static constexpr auto RTK_STATUS_TOPIC = "rtk/status";
static constexpr auto RTK_STATUS_V1_TOPIC = "rtk/status_v1";

static constexpr auto FILTER_STATUS_TOPIC = "nav/status";
static constexpr auto FILTER_HEADING_TOPIC = "nav/heading";
static constexpr auto FILTER_HEADING_STATE_TOPIC = "nav/heading_state";
static constexpr auto FILTER_ODOM_TOPIC = "nav/odom";
static constexpr auto FILTER_IMU_DATA_TOPIC = "nav/filtered_imu/data";
static constexpr auto FILTER_RELATIVE_ODOM_TOPIC = "nav/relative_pos/odom";
static constexpr auto FILTER_DUAL_ANTENNA_STATUS_TOPIC = "nav/dual_antenna_status";
static constexpr auto FILTER_AIDING_SUMMARY_TOPIC = "nav/aiding_summary";

static constexpr auto DEVICE_STATUS_TOPIC = "device/status";
static constexpr auto NMEA_SENTENCE_TOPIC = "nmea/sentence";

// Some other constants
static constexpr int32_t FIELD_DATA_RATE_USE_DATA_CLASS = -1;
static constexpr int32_t DATA_CLASS_DATA_RATE_DO_NOT_STREAM = 0;

/**
 * Container for both a descriptor set and field descriptor
 */
struct MIPDescriptor
{
  uint8_t descriptor_set;  /// Descriptor set
  uint8_t field_descriptor;  /// Field descriptor within the descriptor_set
};

/**
 * Container that will hold information associated with a topic
 */
struct MIPPublisherMappingInfo
{
  std::vector<uint8_t> descriptor_sets = {};  /// Descriptor sets used by this topic
  std::vector<MIPDescriptor> descriptors = {};  /// Descriptors streamed by this topic
  int32_t data_rate = DATA_CLASS_DATA_RATE_DO_NOT_STREAM;  /// Data rate that this topic is streamed at
};

/**
 * Helper class used to lookup MIP or device information given a topic name
 */
class MIPPublisherMapping
{
 public:
  /**
   * \brief Default constructor
   */
  MIPPublisherMapping() = default;

  /**
   * \brief Constructs the mapping with a reference to the ROS node and the device. The reference to the ROS node will be saved as a member variable for later usage
   * \param node  The ROS node that is constructing this object
   * \param inertial_device  Pointer to the inertial device that we will use to read information from the device 
   */
  MIPPublisherMapping(RosNodeType* node, const std::shared_ptr<DeviceInterface> inertial_device);

  /**
   * \brief Configures the data rates associated with the topics. Updates the map with a data rate for each topic
   * \param config_node  ROS node to read the config from
   * \return True if the configuration was successful, false otherwise
   */
  bool configure(RosNodeType* config_node);

  /**
   * \brief Gets the data classes (descriptor sets) that are used by the topic. Will only return the data classes supported by the device passed into the constructor
   * \param topic  Name of the topic to search for
   * \return List of data classes for the requested topic, or an empty vector if the topic cannot be found or is not supported by the device
   */
  std::vector<uint8_t> getDescriptorSets(const std::string& topic) const;

  /**
   * \brief Gets the descriptors that are used by the topic. Will only return the channel fields supported by the device passed into the constructor
   * \param topic  Name of the topic to search for
   * \return List of descriptors for the requested topic, or an empty vector if the topic cannot be found or is not supported by the device
   */
  std::vector<MIPDescriptor> getDescriptors(const std::string& topic) const;

  /**
   * \brief Gets the data rate of the associated topic. Will only return a valid number if called after "configure"
   * \param topic  Name of the topic to search for
   * \return Data rate in hertz that the data is being streamed at
   */
  int32_t getDataRate(const std::string& topic) const;

  /**
   * \brief Gets the maximum data rate among all topics. Will only return a valid number if called after "configure"
   * \param descriptor_set  Descriptor set to search for the max rate of. If set to the shared descriptor set, will return the highest data rate out of all descriptors
   * \return Maximum data rate among all topics in hertz
   */
  int32_t getMaxDataRate(uint8_t descriptor_set = mip::data_shared::DESCRIPTOR_SET) const;

  /**
   * \brief Returns whether a topic is able to be published by a device. This will return true if the device supports the channel fields required by the topic
   * \param topic  Name of the topic to check if the device can publish
   * \return true if the device can publish the topic, false otherwise
   */
  bool canPublish(const std::string& topic) const;

  /**
   * \brief Returns whether a topic is configured to be published. This will return true if the device supports the channel fields required by the topic, AND if the user requested the topic to be published
   * \param topic  Name of the topic to check if the device should publish
   * \return true if the device can publish the topic, false otherwise
   */
  bool shouldPublish(const std::string& topic) const;

 private:
  /**
   * \brief Streams the desired descriptor for all descriptor sets that support it at the highest rate of the descriptor sets.
   * \param field_descriptor  The field descriptor from the share descriptor set to stream
   */
  void streamSharedDescriptor(const uint8_t field_descriptor);

  RosNodeType* node_;
  std::shared_ptr<DeviceInterface> mip_device_;

  std::map<std::string, MIPPublisherMappingInfo> topic_info_mapping_;
  std::map<uint8_t, std::vector<mip::DescriptorRate>> streamed_descriptors_mapping_;

  // Static mappings for topics. Note that this map contains all possible topics regardless of what the device supports
  static const std::map<std::string, FieldWrapper::SharedPtrVec> topic_to_mip_type_mapping_;  /// Mapping between topics and MIP types which can be used to lookup the descriptor set and field descriptors for a topic.
  static const std::map<std::string, std::string> topic_to_data_rate_config_key_mapping_;  /// Mapping between topics and the keys in the config used to configure their data rates

  // Static mappings for descriptor sets. Note that this map contains all possible descriptor sets regardless of what the device supports
  static const std::map<uint8_t, std::string> descriptor_set_to_data_rate_config_key_mapping_;  /// Mapping between descriptor sets and the keys in the config used to configure their data rates if no more specific topic option was provided
};

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MAPPINGS_MIP_PUBLISHER_MAPPING_H