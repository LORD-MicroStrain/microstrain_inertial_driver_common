/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Inertial Device Driver Implementation File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2020, Parker Hannifin Corp
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <map>
#include <string>
#include <memory>
#include <vector>
#include <algorithm>

#include "microstrain_inertial_driver_common/utils/mappings/mip_publisher_mapping.h"

namespace microstrain
{

MipPublisherMapping::MipPublisherMapping(RosNodeType* node, const std::shared_ptr<RosMipDeviceMain> inertial_device) : node_(node), mip_device_(inertial_device)
{
  // Add all supported descriptors to the supported mapping
  for (const auto& mip_type_mapping : static_topic_to_mip_type_mapping_)
  {
    const std::string& topic = mip_type_mapping.first;
    const FieldWrapper::SharedPtrVec& fields = mip_type_mapping.second;

    // Check if any of the fields are supported, and add any supported fields to the list of topic info
    for (const auto& field : fields)
    {
      const uint8_t descriptor_set = field->descriptorSet();
      const uint8_t field_descriptor = field->fieldDescriptor();
      if (mip_device_->supportsDescriptor(descriptor_set, field_descriptor))
      {
        // Add the descriptor to the mapping for the topic
        if (topic_info_mapping_.find(topic) == topic_info_mapping_.end())
          topic_info_mapping_[topic] = MipPublisherMappingInfo();

        auto& topic_info = topic_info_mapping_[topic];
        topic_info.descriptors.push_back({descriptor_set, field_descriptor});
        if (std::find(topic_info.descriptor_sets.begin(), topic_info.descriptor_sets.end(), descriptor_set) == topic_info.descriptor_sets.end())
          topic_info.descriptor_sets.push_back(descriptor_set);
      }
      else if (mip_device_->supportsDescriptorSet(descriptor_set))
      {
        MICROSTRAIN_DEBUG(node_, "Note: The device does not support field 0x%02x%02x associated with topic %s", descriptor_set, field_descriptor, topic.c_str());
      }
    }
    if (topic_info_mapping_.find(topic) == topic_info_mapping_.end())
      MICROSTRAIN_INFO(node_, "Note: The device does not support publishing the topic %s", topic.c_str());
  }
}

bool MipPublisherMapping::configure(RosNodeType* config_node)
{
  // Add the data rates to the topic info map
  for (auto& mapping : topic_info_mapping_)
  {
    const auto& topic = mapping.first;
    auto& topic_info = mapping.second;

    // Get the data rate for the topic, and if it is not the default, use it, otherwise use the data class data rate
    if (static_topic_to_data_rate_config_key_mapping_.find(topic) != static_topic_to_data_rate_config_key_mapping_.end())
    {
      getParamFloat(config_node, static_topic_to_data_rate_config_key_mapping_.at(topic), topic_info.data_rate, FIELD_DATA_RATE_USE_DATA_CLASS);
      if (topic_info.data_rate == FIELD_DATA_RATE_USE_DATA_CLASS)
      {
        // Get the maximum data rate for the data classes associated with the topic if no specific field data rate was configured
        std::vector<float> descriptor_set_rates;
        for (const uint8_t descriptor_set : topic_info.descriptor_sets)
        {
          if (static_descriptor_set_to_data_rate_config_key_mapping_.find(descriptor_set) != static_descriptor_set_to_data_rate_config_key_mapping_.end())
          {
            float descriptor_set_rate;
            getParamFloat(config_node, static_descriptor_set_to_data_rate_config_key_mapping_.at(descriptor_set), descriptor_set_rate, DATA_CLASS_DATA_RATE_DO_NOT_STREAM);
            descriptor_set_rates.push_back(descriptor_set_rate);
          }
          else
          {
            MICROSTRAIN_ERROR(node_, "Descriptor sets 0x%02x used by topic %s does not have an associated data rate. This should be added to the 'static_descriptor_set_to_data_rate_config_key_mapping_' map", descriptor_set, topic.c_str());
            return false;
          }
        }
        if (!descriptor_set_rates.empty())
          topic_info.data_rate = *std::max_element(descriptor_set_rates.begin(), descriptor_set_rates.end());
        else
          topic_info.data_rate = DATA_CLASS_DATA_RATE_DO_NOT_STREAM;
      }
    }
    else
    {
      MICROSTRAIN_ERROR(node_, "Topic %s does not have an associated data rate, this should be added to the 'static_topic_to_data_rate_config_key_mapping_' map", topic.c_str());
      return false;
    }

    // Get the decimation for the topic and add it to the map
    MICROSTRAIN_DEBUG(node_, "Configuring topic %s to stream at %.04f hz", topic.c_str(), topic_info.data_rate);
    for (const auto& descriptor : topic_info.descriptors)
    {
      const uint8_t descriptor_set = descriptor.descriptor_set;
      const uint8_t field_descriptor = descriptor.field_descriptor;
      const uint16_t decimation = mip_device_->getDecimationFromHertz(descriptor_set, topic_info.data_rate);

      // Append the channel to the proper entry for it's descriptor set
      if (streamed_descriptors_mapping_.find(descriptor_set) == streamed_descriptors_mapping_.end())
        streamed_descriptors_mapping_[descriptor_set] = {};
      auto& descriptor_rates = streamed_descriptors_mapping_[descriptor_set];

      // If the data rate is 0, do not stream any data
      if (topic_info.data_rate == 0)
      {
        MICROSTRAIN_DEBUG(node_, "Not configuring descriptor 0x%02x%02x to stream because it's data rate is set to 0", descriptor_set, field_descriptor);
        continue;
      }

      // If the channel has already been added, just update the rate of the existing entry
      auto existing_descriptor_rate = std::find_if(descriptor_rates.begin(), descriptor_rates.end(), [field_descriptor](const mip::DescriptorRate& d)
      {
        return d.descriptor == field_descriptor;
      }
      );
      if (existing_descriptor_rate != descriptor_rates.end())
      {
        if (existing_descriptor_rate->decimation > decimation)  // Lower decimation means faster data
        {
          MICROSTRAIN_DEBUG(node_, "Updating descriptor 0x%02x%02x to stream at %.04f hz", descriptor_set, field_descriptor, topic_info.data_rate);
          existing_descriptor_rate->decimation = decimation;
        }
        else
        {
          MICROSTRAIN_DEBUG(node_, "Descriptor 0x%02x%02x is already streaming faster than %.04f hz, so we are not updating it", descriptor_set, field_descriptor, topic_info.data_rate);
        }
      }
      else
      {
        MICROSTRAIN_DEBUG(node_, "Streaming descriptor 0x%02x%02x at a rate of %.04f hz", descriptor_set, field_descriptor, topic_info.data_rate);
        descriptor_rates.push_back({ field_descriptor, decimation });
      }
    }
  }

  // Add shared descriptors if they were requested
  bool use_device_timestamp;
  getParam<bool>(config_node, "use_device_timestamp", use_device_timestamp, false);
  if (use_device_timestamp)
  {
    if (mip_device_->supportsDescriptor(mip::data_sensor::DESCRIPTOR_SET, mip::data_shared::DATA_GPS_TIME))
    {
      // Prospect devices
      streamSharedDescriptor<mip::data_shared::GpsTimestamp>();
    }
    else
    {
      // Philo devices
      streamAtDescriptorSetRate<mip::data_sensor::GpsTimestamp>();
      streamAtDescriptorSetRate<mip::data_gnss::GpsTime>();
      streamAtDescriptorSetRate<mip::data_filter::Timestamp>();
    }
  }

  // Enable each of the descriptor sets and save the message format
  for (const auto& streamed_descriptor_mapping : streamed_descriptors_mapping_)
  {
    mip::CmdResult mip_cmd_result;
    const uint8_t descriptor_set = streamed_descriptor_mapping.first;
    const std::vector<mip::DescriptorRate> descriptor_rates = streamed_descriptor_mapping.second;
    if (!(mip_cmd_result = mip_device_->writeMessageFormat(descriptor_set, descriptor_rates.size(), descriptor_rates.data())))
    {
      MICROSTRAIN_ERROR(node_, "Failed to write message format for descriptor set 0x%02x", descriptor_set);
      MICROSTRAIN_ERROR(node_, "  Error(%d): %s", mip_cmd_result.value, mip_cmd_result.name());
      return false;
    }
    if (!(mip_cmd_result = mip_device_->writeDatastreamControl(descriptor_set, true)))
    {
      MICROSTRAIN_ERROR(node_, "Failed to enable descriptor set 0x%02x", descriptor_set);
      MICROSTRAIN_ERROR(node_, "  Error(%d): %s", mip_cmd_result.value, mip_cmd_result.name());
      return false;
    }
  }
  return true;
}

std::vector<uint8_t> MipPublisherMapping::getDescriptorSets(const std::string& topic) const
{
  if (topic_info_mapping_.find(topic) != topic_info_mapping_.end())
    return topic_info_mapping_.at(topic).descriptor_sets;
  else
    return {};
}

std::vector<MipDescriptor> MipPublisherMapping::getDescriptors(const std::string& topic) const
{
  if (topic_info_mapping_.find(topic) != topic_info_mapping_.end())
    return topic_info_mapping_.at(topic).descriptors;
  else
    return {};
}

float MipPublisherMapping::getDataRate(const std::string& topic) const
{
  if (topic_info_mapping_.find(topic) != topic_info_mapping_.end())
    return topic_info_mapping_.at(topic).data_rate;
  else
    return DATA_CLASS_DATA_RATE_DO_NOT_STREAM;
}

float MipPublisherMapping::getMaxDataRate(uint8_t descriptor_set) const
{
  std::vector<float> data_rates;
  for (const auto& element : static_topic_to_mip_type_mapping_)
  {
    const std::vector<uint8_t> descriptor_sets = getDescriptorSets(element.first);
    if (descriptor_set == mip::data_shared::DESCRIPTOR_SET || std::find(descriptor_sets.begin(), descriptor_sets.end(), descriptor_set) != descriptor_sets.end())
      data_rates.push_back(getDataRate(element.first));
  }
  return *std::max_element(data_rates.begin(), data_rates.end());
}

bool MipPublisherMapping::canPublish(const std::string& topic) const
{
  return !getDescriptors(topic).empty();
}

bool MipPublisherMapping::shouldPublish(const std::string& topic) const
{
  return canPublish(topic) && getDataRate(topic) != DATA_CLASS_DATA_RATE_DO_NOT_STREAM;
}

const std::map<std::string, FieldWrapper::SharedPtrVec> MipPublisherMapping::static_topic_to_mip_type_mapping_ =
{
  // IMU topic mappings
  {IMU_DATA_TOPIC, {
    FieldWrapperType<mip::data_sensor::ScaledAccel>::initialize(),
    FieldWrapperType<mip::data_sensor::ScaledGyro>::initialize(),
    FieldWrapperType<mip::data_sensor::CompQuaternion>::initialize(),
  }},
  {IMU_MAG_TOPIC, {
    FieldWrapperType<mip::data_sensor::ScaledMag>::initialize(),
  }},
  {IMU_GPS_CORR_TOPIC, {
    FieldWrapperType<mip::data_sensor::GpsTimestamp>::initialize(),
  }},
  {IMU_OVERRANGE_STATUS_TOPIC, {
    FieldWrapperType<mip::data_sensor::OverrangeStatus>::initialize(),
  }},

  // GNSS1 topic mappings. Note that each of these topics will contain a field for both the GNSS and GNSS1 descriptor set
  {GNSS1_NAVSATFIX_TOPIC, {
    FieldWrapperType<mip::data_gnss::PosLlh, mip::data_gnss::DESCRIPTOR_SET>::initialize(),
    FieldWrapperType<mip::data_gnss::PosLlh, mip::data_gnss::MIP_GNSS1_DATA_DESC_SET>::initialize(),
  }},
  {GNSS1_ODOM_TOPIC, {
    FieldWrapperType<mip::data_gnss::PosLlh, mip::data_gnss::DESCRIPTOR_SET>::initialize(),
    FieldWrapperType<mip::data_gnss::PosLlh, mip::data_gnss::MIP_GNSS1_DATA_DESC_SET>::initialize(),
    FieldWrapperType<mip::data_gnss::VelNed, mip::data_gnss::DESCRIPTOR_SET>::initialize(),
    FieldWrapperType<mip::data_gnss::VelNed, mip::data_gnss::MIP_GNSS1_DATA_DESC_SET>::initialize(),
  }},
  {GNSS1_TIME_REF_TOPIC, {
    FieldWrapperType<mip::data_gnss::GpsTime, mip::data_gnss::DESCRIPTOR_SET>::initialize(),
    FieldWrapperType<mip::data_gnss::GpsTime, mip::data_gnss::MIP_GNSS1_DATA_DESC_SET>::initialize(),
  }},
  {GNSS1_FIX_INFO_TOPIC, {
    FieldWrapperType<mip::data_gnss::FixInfo, mip::data_gnss::DESCRIPTOR_SET>::initialize(),
    FieldWrapperType<mip::data_gnss::FixInfo, mip::data_gnss::MIP_GNSS1_DATA_DESC_SET>::initialize(),
  }},
  {GNSS1_SBAS_INFO_TOPIC, {
    FieldWrapperType<mip::data_gnss::SbasInfo, mip::data_gnss::MIP_GNSS1_DATA_DESC_SET>::initialize(),
  }},
  {GNSS1_RF_ERROR_DETECTION_TOPIC, {
    FieldWrapperType<mip::data_gnss::RfErrorDetection, mip::data_gnss::MIP_GNSS1_DATA_DESC_SET>::initialize()
  }},

  // GNSS2 topic mappings.
  {GNSS2_NAVSATFIX_TOPIC, {
    FieldWrapperType<mip::data_gnss::PosLlh, mip::data_gnss::MIP_GNSS2_DATA_DESC_SET>::initialize(),
  }},
  {GNSS2_ODOM_TOPIC, {
    FieldWrapperType<mip::data_gnss::PosLlh, mip::data_gnss::MIP_GNSS2_DATA_DESC_SET>::initialize(),
    FieldWrapperType<mip::data_gnss::VelNed, mip::data_gnss::MIP_GNSS2_DATA_DESC_SET>::initialize(),
  }},
  {GNSS2_TIME_REF_TOPIC, {
    FieldWrapperType<mip::data_gnss::GpsTime, mip::data_gnss::MIP_GNSS2_DATA_DESC_SET>::initialize(),
  }},
  {GNSS2_FIX_INFO_TOPIC, {
    FieldWrapperType<mip::data_gnss::FixInfo, mip::data_gnss::MIP_GNSS2_DATA_DESC_SET>::initialize(),
  }},
  {GNSS2_SBAS_INFO_TOPIC, {
    FieldWrapperType<mip::data_gnss::SbasInfo, mip::data_gnss::MIP_GNSS2_DATA_DESC_SET>::initialize(),
  }},
  {GNSS2_RF_ERROR_DETECTION_TOPIC, {
    FieldWrapperType<mip::data_gnss::RfErrorDetection, mip::data_gnss::MIP_GNSS2_DATA_DESC_SET>::initialize()
  }},

  // RTK topic mappings
  {RTK_STATUS_TOPIC, {
    FieldWrapperType<mip::data_gnss::RtkCorrectionsStatus, mip::data_gnss::MIP_GNSS3_DATA_DESC_SET>::initialize(),
  }},
  {RTK_STATUS_V1_TOPIC, {
    FieldWrapperType<mip::data_gnss::RtkCorrectionsStatus, mip::data_gnss::MIP_GNSS3_DATA_DESC_SET>::initialize(),
  }},

  // Filter topic mappings
  {FILTER_STATUS_TOPIC, {
    FieldWrapperType<mip::data_filter::Status>::initialize(),
  }},
  {FILTER_HEADING_TOPIC, {
    FieldWrapperType<mip::data_filter::EulerAngles>::initialize(),
  }},
  {FILTER_HEADING_STATE_TOPIC, {
    FieldWrapperType<mip::data_filter::HeadingUpdateState>::initialize(),
  }},
  {FILTER_ODOM_TOPIC, {
    FieldWrapperType<mip::data_filter::PositionLlh>::initialize(),
    FieldWrapperType<mip::data_filter::PositionLlhUncertainty>::initialize(),
    FieldWrapperType<mip::data_filter::AttitudeQuaternion>::initialize(),
    FieldWrapperType<mip::data_filter::EulerAnglesUncertainty>::initialize(),
    FieldWrapperType<mip::data_filter::VelocityNed>::initialize(),
    FieldWrapperType<mip::data_filter::VelocityNedUncertainty>::initialize(),
    FieldWrapperType<mip::data_filter::CompAngularRate>::initialize(),
  }},
  {FILTER_IMU_DATA_TOPIC, {
    FieldWrapperType<mip::data_filter::AttitudeQuaternion>::initialize(),
    FieldWrapperType<mip::data_filter::CompAngularRate>::initialize(),
    FieldWrapperType<mip::data_filter::CompAccel>::initialize(),
    FieldWrapperType<mip::data_filter::LinearAccel>::initialize(),
    FieldWrapperType<mip::data_filter::EulerAnglesUncertainty>::initialize(),
  }},
  {FILTER_RELATIVE_ODOM_TOPIC, {
    FieldWrapperType<mip::data_filter::RelPosNed>::initialize(),
    FieldWrapperType<mip::data_filter::PositionLlhUncertainty>::initialize(),
    FieldWrapperType<mip::data_filter::AttitudeQuaternion>::initialize(),
    FieldWrapperType<mip::data_filter::EulerAnglesUncertainty>::initialize(),
    FieldWrapperType<mip::data_filter::VelocityNed>::initialize(),
    FieldWrapperType<mip::data_filter::VelocityNedUncertainty>::initialize(),
    FieldWrapperType<mip::data_filter::CompAngularRate>::initialize(),
  }},
  {GNSS1_AIDING_STATUS_TOPIC, {
    FieldWrapperType<mip::data_filter::GnssPosAidStatus>::initialize(),
  }},
  {GNSS2_AIDING_STATUS_TOPIC, {
    FieldWrapperType<mip::data_filter::GnssPosAidStatus>::initialize(),
  }},
  {FILTER_DUAL_ANTENNA_STATUS_TOPIC, {
    FieldWrapperType<mip::data_filter::GnssDualAntennaStatus>::initialize(),
  }},
  {FILTER_AIDING_SUMMARY_TOPIC, {
    FieldWrapperType<mip::data_filter::AidingMeasurementSummary>::initialize(),
  }}
};

const std::map<std::string, std::string> MipPublisherMapping::static_topic_to_data_rate_config_key_mapping_ =
{
  // IMU data rates
  {IMU_DATA_TOPIC,              "imu_raw_data_rate"},
  {IMU_MAG_TOPIC,               "imu_mag_data_rate"},
  {IMU_GPS_CORR_TOPIC,          "imu_gps_corr_data_rate"},
  {IMU_OVERRANGE_STATUS_TOPIC,  "imu_overrange_status_data_rate"},

  // GNSS/GNSS1 data rates
  {GNSS1_NAVSATFIX_TOPIC,          "gnss1_nav_sat_fix_data_rate"},
  {GNSS1_ODOM_TOPIC,               "gnss1_odom_data_rate"},
  {GNSS1_TIME_REF_TOPIC,           "gnss1_time_reference_data_rate"},
  {GNSS1_FIX_INFO_TOPIC,           "gnss1_fix_info_data_rate"},
  {GNSS1_AIDING_STATUS_TOPIC,      "filter_aiding_status_data_rate"},
  {GNSS1_SBAS_INFO_TOPIC,          "gnss1_sbas_info_data_rate"},
  {GNSS1_RF_ERROR_DETECTION_TOPIC, "gnss1_rf_error_detection_data_rate"},

  // GNSS2 data rates
  {GNSS2_NAVSATFIX_TOPIC,          "gnss2_nav_sat_fix_data_rate"},
  {GNSS2_ODOM_TOPIC,               "gnss2_odom_data_rate"},
  {GNSS2_TIME_REF_TOPIC,           "gnss2_time_reference_data_rate"},
  {GNSS2_FIX_INFO_TOPIC,           "gnss2_fix_info_data_rate"},
  {GNSS2_AIDING_STATUS_TOPIC,      "filter_aiding_status_data_rate"},
  {GNSS2_SBAS_INFO_TOPIC,          "gnss2_sbas_info_data_rate"},
  {GNSS2_RF_ERROR_DETECTION_TOPIC, "gnss2_rf_error_detection_data_rate"},

  // RTK data rates
  {RTK_STATUS_TOPIC,    "rtk_status_data_rate"},
  {RTK_STATUS_V1_TOPIC, "rtk_status_data_rate"},

  // Filter data rates
  {FILTER_STATUS_TOPIC,              "filter_status_data_rate"},
  {FILTER_HEADING_TOPIC,             "filter_heading_data_rate"},
  {FILTER_HEADING_STATE_TOPIC,       "filter_heading_state_data_rate"},
  {FILTER_ODOM_TOPIC,                "filter_heading_state_data_rate"},
  {FILTER_IMU_DATA_TOPIC,            "filter_imu_data_rate"},
  {FILTER_RELATIVE_ODOM_TOPIC,       "filter_relative_odom_data_rate"},
  {FILTER_DUAL_ANTENNA_STATUS_TOPIC, "filter_gnss_dual_antenna_data_rate"},
  {FILTER_AIDING_SUMMARY_TOPIC,      "filter_aiding_measurement_summary_data_rate"},
};

const std::map<uint8_t, std::string> MipPublisherMapping::static_descriptor_set_to_data_rate_config_key_mapping_ =
{
  {mip::data_sensor::DESCRIPTOR_SET,        "imu_data_rate"},
  {mip::data_gnss::DESCRIPTOR_SET,          "gnss1_data_rate"},
  {mip::data_gnss::MIP_GNSS1_DATA_DESC_SET, "gnss1_data_rate"},
  {mip::data_gnss::MIP_GNSS2_DATA_DESC_SET, "gnss2_data_rate"},
  {mip::data_gnss::MIP_GNSS3_DATA_DESC_SET, "rtk_data_rate"},
  {mip::data_filter::DESCRIPTOR_SET,        "filter_data_rate"},
};

}  // namespace microstrain
