#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_EVENTS_YAML_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_EVENTS_YAML_H

#include "microstrain_inertial_driver_common/utils/ros_compat.h"
#include "microstrain_inertial_driver_common/utils/mip/ros_mip_device_main.h"
#include "microstrain_inertial_driver_common/utils/mappings/mip_publisher_mapping.h"

#include <yaml-cpp/yaml.h>


namespace microstrain
{

class EventsYaml
{
 public:
  EventsYaml(RosNodeType* node, std::shared_ptr<MipPublisherMapping> mip_publisher_mapping);

  bool parseAndWriteEventConfig(std::shared_ptr<RosMipDeviceMain>& mip_device, const std::string& events_yaml_file_path);
 protected:
  bool parseAndWriteEventTriggerConfig(std::shared_ptr<RosMipDeviceMain>& mip_device, YAML::Node triggers_yaml);
  bool parseAndWriteEventActionConfig(std::shared_ptr<RosMipDeviceMain>& mip_device, YAML::Node actions_yaml);

  bool parseEventGpioTriggerConfig(const YAML::Node& gpio_triggers_yaml, std::vector<mip::commands_3dm::EventTrigger>* gpio_event_triggers);
  bool parseEventThresholdTriggerConfig(const YAML::Node& threshold_triggers_yaml, std::vector<mip::commands_3dm::EventTrigger>* threshold_event_triggers);
  bool parseEventCombinationTriggerConfig(const YAML::Node& combination_triggers_yaml, std::vector<mip::commands_3dm::EventTrigger>* combination_event_triggers);

  bool parseEventGpioActionConfig(const YAML::Node& gpio_actions_yaml, std::vector<mip::commands_3dm::EventAction>* gpio_event_actions);
  bool parseEventMessageActionConfig(std::shared_ptr<RosMipDeviceMain>& mip_device, const YAML::Node& message_actions_yaml, std::vector<mip::commands_3dm::EventAction>* message_event_actions);
  bool parseEventRosMesasgeActionConfig(std::shared_ptr<RosMipDeviceMain>& mip_device, const YAML::Node& ros_message_actions_yaml, std::vector<mip::commands_3dm::EventAction>* ros_message_event_actions);

  template<typename T>
  T getRequiredKeyFromYaml(const YAML::Node& node, const std::string& key);

  void printEventTrigger(const mip::commands_3dm::EventTrigger& trigger);
  void printEventAction(const mip::commands_3dm::EventAction& action);

  RosNodeType* node_;
  std::shared_ptr<MipPublisherMapping> mip_publisher_mapping_;
};

}

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_EVENTS_YAML_H