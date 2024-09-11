#include <stdexcept>

#include "microstrain_inertial_driver_common/utils/events_yaml.h"

namespace microstrain
{

inline std::string getYamlTypeString(const YAML::Node& node)
{
  switch (node.Type())
  {
    case YAML::NodeType::Map:
      return "map";
    case YAML::NodeType::Scalar:
      return "scalar";
    case YAML::NodeType::Sequence:
      return "sequence";
    case YAML::NodeType::Undefined:
      return "undefined";
    case YAML::NodeType::Null:
    default:
      return "null";
  }
}

EventsYaml::EventsYaml(RosNodeType* node) : node_(node)
{
}

bool EventsYaml::parseAndWriteEventConfig(std::shared_ptr<RosMipDeviceMain>& mip_device, const std::string& events_yaml_file_path)
{
  // First thing to do is to make sure that the path we were handed can be opened and parsed
  YAML::Node events_yaml;
  try
  {
    MICROSTRAIN_INFO(node_, "Reading event config from %s", events_yaml_file_path.c_str());
    events_yaml = YAML::LoadFile(events_yaml_file_path);
  }
  catch (const YAML::BadFile& b)
  {
    MICROSTRAIN_ERROR(node_, "Unable to open events yaml file located at %s", events_yaml_file_path.c_str());
    MICROSTRAIN_ERROR(node_, "  Exception: %s", b.what());
    return false;
  }
  catch (const YAML::ParserException& p)
  {
    MICROSTRAIN_ERROR(node_, "Opened, but failed to parse events yaml file located at %s", events_yaml_file_path.c_str());
    MICROSTRAIN_ERROR(node_, "  Exception: %s", p.what());
    return false;
  }

  // Make sure we can find the required information
  if (!mip_device->supportsDescriptor(mip::commands_3dm::DESCRIPTOR_SET, mip::commands_3dm::CMD_EVENT_SUPPORT))
  {
    MICROSTRAIN_ERROR(node_, "Device does not support the event 'Get Supported Events' command. We are unable to configure events");
    return false;
  }

  // All upper level keys are optional, so if we can't find one, just log and move on
  YAML::Node events_yaml_events;
  if ((events_yaml_events = events_yaml["events"]))
  {
    MICROSTRAIN_DEBUG(node_, "Parsing 'events' section of events config");
    YAML::Node triggers_yaml, actions_yaml;
    if ((triggers_yaml = events_yaml_events["triggers"]))
    {
      if (!parseAndWriteEventTriggerConfig(mip_device, triggers_yaml))
        return false;
    }
    else
    {
      MICROSTRAIN_INFO(node_, "Events yaml did not contain a 'triggers' key, so no triggers will be configured");
    }
    if ((actions_yaml = events_yaml_events["actions"]))
    {
      if (!parseAndWriteEventActionConfig(mip_device, actions_yaml))
        return false;
    }
    else
    {
      MICROSTRAIN_INFO(node_, "Events yaml did not contain a 'actions' key, so no actions will be configured");
    }
  }
  else
  {
    MICROSTRAIN_WARN(node_, "Events yaml did not contain an 'events' key, so no events will be configured");
  }

  // Looks like everything worked
  return true;
}

bool EventsYaml::parseAndWriteEventTriggerConfig(std::shared_ptr<RosMipDeviceMain>& mip_device, const YAML::Node& triggers_yaml)
{
  // Before we parse the yaml, check that we support each type, and check how many we support
  mip::CmdResult mip_cmd_result;
  uint8_t max_triggers, num_entries, max_entries = 255;
  mip::commands_3dm::GetEventSupport::Info entries[max_entries];
  if (!(mip_cmd_result = mip::commands_3dm::getEventSupport(*mip_device, mip::commands_3dm::GetEventSupport::Query::TRIGGER_TYPES, &max_triggers, &num_entries, max_entries, entries)))
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Unable to determine the max number of supported triggers");
    return false;
  }
  MICROSTRAIN_DEBUG(node_, "The device supports up to %u triggers", max_triggers);

  // Parse the individual configuration options
  MICROSTRAIN_DEBUG(node_, "Parsing 'triggers' section of events config");
  std::vector<mip::commands_3dm::EventTrigger> event_triggers;
  YAML::Node gpio_triggers_yaml, threshold_triggers_yaml, combination_triggers_yaml;
  if ((gpio_triggers_yaml = triggers_yaml["gpio"]))
  {
    if (!parseEventGpioTriggerConfig(gpio_triggers_yaml, &event_triggers))
      return false;
  }
  else
  {
    MICROSTRAIN_DEBUG(node_, "No 'gpio' section of 'triggers' config, not configuring any GPIO triggers");
  }
  if ((threshold_triggers_yaml = triggers_yaml["threshold"]))
  {
    if (!parseEventThresholdTriggerConfig(threshold_triggers_yaml, &event_triggers))
      return false;
  }
  else
  {
    MICROSTRAIN_DEBUG(node_, "No 'threshold' section of 'triggers' config, not configuring any threshold triggers");
  }
  if ((combination_triggers_yaml = triggers_yaml["combination"]))
  {
    if (!parseEventCombinationTriggerConfig(combination_triggers_yaml, &event_triggers))
      return false;
  }
  else
  {
    MICROSTRAIN_DEBUG(node_, "No 'combination' section of 'triggers' config, not configuring any combination triggers");
  }

  // Make sure that we don't have too many triggers
  if (event_triggers.size() > max_triggers)
  {
    MICROSTRAIN_ERROR(node_, "Device only supports %u triggers, but you attempted to configure %lu triggers", max_triggers, event_triggers.size());
    return false;
  }

  // Finally configure the triggers
  for (auto& event_trigger : event_triggers)
  {
    event_trigger.function = mip::FunctionSelector::WRITE;
    MICROSTRAIN_INFO(node_, "Configuring event trigger instance %u", event_trigger.instance);
    printEventTrigger(event_trigger);
    if (!(mip_cmd_result = mip_device->device().runCommand<mip::commands_3dm::EventTrigger>(event_trigger)))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to write event trigger configuration");
      return false;
    }
  }
  return true;
}

bool EventsYaml::parseAndWriteEventActionConfig(std::shared_ptr<RosMipDeviceMain>& mip_device, const YAML::Node& actions_yaml)
{
  // Before we parse the yaml, check that we support each type, and check how many we support
  mip::CmdResult mip_cmd_result;
  uint8_t max_actions, num_entries, max_entries = 255;
  mip::commands_3dm::GetEventSupport::Info entries[max_entries];
  if (!(mip_cmd_result = mip::commands_3dm::getEventSupport(*mip_device, mip::commands_3dm::GetEventSupport::Query::ACTION_TYPES, &max_actions, &num_entries, max_entries, entries)))
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Unable to determine the max number of supported actions");
    return false;
  }
  MICROSTRAIN_DEBUG(node_, "The device supports up to %u actions", max_actions);

  // Parse the individual configuration sections
  MICROSTRAIN_DEBUG(node_, "Parsing 'actions' section of events config");
  std::vector<mip::commands_3dm::EventAction> event_actions;
  YAML::Node gpio_actions_yaml, message_actions_yaml;
  if ((gpio_actions_yaml = actions_yaml["gpio"]))
  {
    if (!parseEventGpioActionConfig(gpio_actions_yaml, &event_actions))
      return false;
  }
  else
  {
    MICROSTRAIN_DEBUG(node_, "No 'gpio' section of 'actions' config, not configuring any GPIO actions");
  }
  if ((message_actions_yaml = actions_yaml["message"]))
  {
    if (!parseEventMessageActionConfig(mip_device, message_actions_yaml, &event_actions))
      return false;
  }
  else
  {
    MICROSTRAIN_DEBUG(node_, "No 'message' section of 'actions' config, not configuring any message actions");
  }

  // Make sure that we don't have too many actions
  if (event_actions.size() > max_actions)
  {
    MICROSTRAIN_ERROR(node_, "Device only supports %u actions, but you attempted to configure %lu triggers", max_actions, event_actions.size());
    return false;
  }

  // Finally configure the actions
  for (auto& event_action : event_actions)
  {
    event_action.function = mip::FunctionSelector::WRITE;
    MICROSTRAIN_INFO(node_, "Configuring event action instance %u", event_action.instance);
    printEventAction(event_action);
    if (!(mip_cmd_result = mip_device->device().runCommand<mip::commands_3dm::EventAction>(event_action)))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to write event action configuration");
      return false;
    }
  }
  return true;
}

bool EventsYaml::parseEventGpioTriggerConfig(const YAML::Node& gpio_triggers_yaml, std::vector<mip::commands_3dm::EventTrigger>* gpio_event_triggers)
{
  // Make sure that the type is a sequence or else we won't be able to iterate properly
  if (!gpio_triggers_yaml.IsSequence())
  {
    MICROSTRAIN_ERROR(node_, "Event triggers 'gpio' key must contain a sequence, but was of type %s", getYamlTypeString(gpio_triggers_yaml).c_str());
    return false;
  }

  // Iterate and extract the triggers into the MIP objects
  for (const YAML::Node& gpio_trigger_yaml : gpio_triggers_yaml)
  {
    // Make sure we can find all the required keys, and they are the right type
    mip::commands_3dm::EventTrigger gpio_trigger;
    gpio_trigger.type = mip::commands_3dm::EventTrigger::Type::GPIO;
    try
    {
      gpio_trigger.instance = getRequiredKeyFromYaml<uint16_t>(gpio_trigger_yaml, "instance");
      gpio_trigger.parameters.gpio.pin = getRequiredKeyFromYaml<uint16_t>(gpio_trigger_yaml, "pin");
      gpio_trigger.parameters.gpio.mode = static_cast<mip::commands_3dm::EventTrigger::GpioParams::Mode>(getRequiredKeyFromYaml<uint16_t>(gpio_trigger_yaml, "mode"));
      gpio_event_triggers->push_back(gpio_trigger);
    }
    catch (const std::runtime_error& e)
    {
      return false;
    }
  }

  return true;
}

bool EventsYaml::parseEventThresholdTriggerConfig(const YAML::Node& threshold_triggers_yaml, std::vector<mip::commands_3dm::EventTrigger>* threshold_event_triggers)
{
  // Make sure that the type is a sequence or else we won't be able to iterate properly
  if (!threshold_triggers_yaml.IsSequence())
  {
    MICROSTRAIN_ERROR(node_, "Event triggers 'threshold' key must contain a sequence, but was of type %s", getYamlTypeString(threshold_triggers_yaml).c_str());
    return false;
  }

  // Iterate and extract the triggers into the MIP objects
  for (const YAML::Node& threshold_trigger_yaml : threshold_triggers_yaml)
  {
    // Make sure we can find all the required keys, and they are the right type
    mip::commands_3dm::EventTrigger threshold_trigger;
    threshold_trigger.type = mip::commands_3dm::EventTrigger::Type::THRESHOLD;
    try
    {
      threshold_trigger.instance = getRequiredKeyFromYaml<uint16_t>(threshold_trigger_yaml, "instance");
      threshold_trigger.parameters.threshold.desc_set = getRequiredKeyFromYaml<uint16_t>(threshold_trigger_yaml, "descriptor_set");
      threshold_trigger.parameters.threshold.field_desc = getRequiredKeyFromYaml<uint16_t>(threshold_trigger_yaml, "field_descriptor");
      threshold_trigger.parameters.threshold.param_id = getRequiredKeyFromYaml<uint16_t>(threshold_trigger_yaml, "param_id");
      threshold_trigger.parameters.threshold.type = static_cast<mip::commands_3dm::EventTrigger::ThresholdParams::Type>(getRequiredKeyFromYaml<uint16_t>(threshold_trigger_yaml, "type"));
      switch (threshold_trigger.parameters.threshold.type)
      {
        case mip::commands_3dm::EventTrigger::ThresholdParams::Type::WINDOW:
          threshold_trigger.parameters.threshold.low_thres = getRequiredKeyFromYaml<float>(threshold_trigger_yaml, "low_threshold");
          threshold_trigger.parameters.threshold.high_thres = getRequiredKeyFromYaml<float>(threshold_trigger_yaml, "high_threshold");
          break;
        case mip::commands_3dm::EventTrigger::ThresholdParams::Type::INTERVAL:
          threshold_trigger.parameters.threshold.int_thres = getRequiredKeyFromYaml<float>(threshold_trigger_yaml, "interval_threshold");
          threshold_trigger.parameters.threshold.interval = getRequiredKeyFromYaml<float>(threshold_trigger_yaml, "interval");
          break;
        default:
          MICROSTRAIN_ERROR(node_, "Invalid threshold type %u", static_cast<uint8_t>(threshold_trigger.parameters.threshold.type));
          return false;
      }
      threshold_event_triggers->push_back(threshold_trigger);
    }
    catch (const std::runtime_error& e)
    {
      return false;
    }
  }

  return true;
}

bool EventsYaml::parseEventCombinationTriggerConfig(const YAML::Node& combination_triggers_yaml, std::vector<mip::commands_3dm::EventTrigger>* combination_event_triggers)
{
  // Make sure that the type is a sequence or else we won't be able to iterate properly
  if (!combination_triggers_yaml.IsSequence())
  {
    MICROSTRAIN_ERROR(node_, "Event triggers 'combination' key must contain a sequence, but was of type %s", getYamlTypeString(combination_triggers_yaml).c_str());
    return false;
  }

  // Iterate and extract the triggers into the MIP objects
  for (const YAML::Node& combination_trigger_yaml : combination_triggers_yaml)
  {
    // Make sure we can find all the required keys, and they are the right type
    mip::commands_3dm::EventTrigger combination_trigger;
    combination_trigger.type = mip::commands_3dm::EventTrigger::Type::COMBINATION;
    try
    {
      combination_trigger.instance = getRequiredKeyFromYaml<uint16_t>(combination_trigger_yaml, "instance");
      combination_trigger.parameters.combination.logic_table = getRequiredKeyFromYaml<uint16_t>(combination_trigger_yaml, "logic_table");

      YAML::Node input_triggers_yaml;
      const size_t max_input_triggers = sizeof(combination_trigger.parameters.combination.input_triggers) / sizeof(combination_trigger.parameters.combination.input_triggers[0]);
      if ((input_triggers_yaml = combination_trigger_yaml["input_triggers"]))
      {
        if (input_triggers_yaml.IsSequence() && input_triggers_yaml.size() <= max_input_triggers)
        {
          uint8_t i = 0;
          memset(combination_trigger.parameters.combination.input_triggers, 0, max_input_triggers);
          for (const auto& input_trigger_yaml : input_triggers_yaml)
          {
            try
            {
              combination_trigger.parameters.combination.input_triggers[i++] = input_trigger_yaml.as<uint16_t>();
            }
            catch(const YAML::TypedBadConversion<uint16_t>& t)
            {
              MICROSTRAIN_ERROR(node_, "Combination trigger parameter 'input_triggers' must only contain numbers");
              return false;
            }
          }
        }
        else
        {
          MICROSTRAIN_ERROR(node_, "Combination trigger parameter 'input_triggers' must be a sequence of size %lu or less", max_input_triggers);
          return false;
        }
      }
      else
      {
        MICROSTRAIN_ERROR(node_, "Combination trigger missing 'input_triggers' parameter");
        return false;
      }
      combination_event_triggers->push_back(combination_trigger);
    }
    catch (const std::runtime_error& e)
    {
      return false;
    }
  }

  return true;
}

bool EventsYaml::parseEventGpioActionConfig(const YAML::Node& gpio_actions_yaml, std::vector<mip::commands_3dm::EventAction>* gpio_event_actions)
{
  // Make sure that the type is a sequence or else we won't be able to iterate properly
  if (!gpio_actions_yaml.IsSequence())
  {
    MICROSTRAIN_ERROR(node_, "Event actions 'gpio' key must contain a sequence, but was of type %s", getYamlTypeString(gpio_actions_yaml).c_str());
    return false;
  }

  // Iterate the yaml and parse the entries into the MIP objects
  for (const YAML::Node& gpio_action_yaml : gpio_actions_yaml)
  {
    mip::commands_3dm::EventAction gpio_action;
    gpio_action.type = mip::commands_3dm::EventAction::Type::GPIO;
    try
    {
      gpio_action.instance = getRequiredKeyFromYaml<uint16_t>(gpio_action_yaml, "instance");
      gpio_action.trigger = getRequiredKeyFromYaml<uint16_t>(gpio_action_yaml, "trigger_instance");
      gpio_action.parameters.gpio.pin = getRequiredKeyFromYaml<uint16_t>(gpio_action_yaml, "pin");
      gpio_action.parameters.gpio.mode = static_cast<mip::commands_3dm::EventAction::GpioParams::Mode>(getRequiredKeyFromYaml<uint16_t>(gpio_action_yaml, "mode"));
      gpio_event_actions->push_back(gpio_action);
    }
    catch(const std::runtime_error& r)
    {
      return false;
    }
  }

  return true;
}

bool EventsYaml::parseEventMessageActionConfig(std::shared_ptr<RosMipDeviceMain>& mip_device, const YAML::Node& message_actions_yaml, std::vector<mip::commands_3dm::EventAction>* message_event_actions)
{
  // Make sure that the type is a sequence or else we won't be able to iterate properly
  if (!message_actions_yaml.IsSequence())
  {
    MICROSTRAIN_ERROR(node_, "Event actions 'message' key must contain a sequence, but was of type %s", getYamlTypeString(message_actions_yaml).c_str());
    return false;
  }

  // Iterate the yaml and parse the entries into the MIP objects
  for (const YAML::Node& message_action_yaml : message_actions_yaml)
  {
    mip::commands_3dm::EventAction message_action;
    message_action.type = mip::commands_3dm::EventAction::Type::MESSAGE;
    try
    {
      message_action.instance = getRequiredKeyFromYaml<uint16_t>(message_action_yaml, "instance");
      message_action.trigger = getRequiredKeyFromYaml<uint16_t>(message_action_yaml, "trigger_instance");
      message_action.parameters.message.desc_set = getRequiredKeyFromYaml<uint16_t>(message_action_yaml, "descriptor_set");

      double actual_hertz;
      float desired_hertz = getRequiredKeyFromYaml<float>(message_action_yaml, "hertz");
      message_action.parameters.message.decimation = mip_device->getDecimationFromHertz(message_action.parameters.message.desc_set, desired_hertz, &actual_hertz);
      if (actual_hertz != desired_hertz)
        MICROSTRAIN_WARN(node_, "Descriptor set 0x%02x does not support running at the %f hertz. The closest we can do is %f", message_action.parameters.message.desc_set, desired_hertz, actual_hertz);

      YAML::Node descriptors_yaml;
      const size_t max_descriptors = sizeof(message_action.parameters.message.descriptors) / sizeof(message_action.parameters.message.descriptors[0]);
      if ((descriptors_yaml = message_action_yaml["descriptors"]))
      {
        if (descriptors_yaml.IsSequence() && descriptors_yaml.size() <= max_descriptors)
        {
          uint8_t i = 0;
          for (const YAML::Node& descriptor_yaml : descriptors_yaml)
          {
            try
            {
              message_action.parameters.message.descriptors[i++] = descriptor_yaml.as<uint16_t>();
            }
            catch(const YAML::TypedBadConversion<uint16_t>& t)
            {
              MICROSTRAIN_ERROR(node_, "Message action parameter 'descriptors' must only contain numbers");
              return false;
            }
          }
          message_action.parameters.message.num_fields = descriptors_yaml.size();
        }
        else
        {
          MICROSTRAIN_ERROR(node_, "Message action parameter 'descriptors' must be a sequence of size no greater than %lu", max_descriptors);
          return false;
        }
      }
      else
      {
        MICROSTRAIN_ERROR(node_, "Message action missing required parameter 'descriptors'");
        return false;
      }

      message_event_actions->push_back(message_action);
    }
    catch (const std::runtime_error& r)
    {
      return false;
    }
  }

  return true;
}

template<typename T>
T EventsYaml::getRequiredKeyFromYaml(const YAML::Node& node, const std::string& key)
{
  if (!node[key])
  {
    YAML::Emitter emitter; emitter << node;
    MICROSTRAIN_ERROR(node_, "Missing required key '%s' in %s", key.c_str(), emitter.c_str());
    throw std::runtime_error("");
  }
  try
  {
    return node[key].as<T>();
  }
  catch(const YAML::TypedBadConversion<T>& t)
  {
    YAML::Emitter emitter; emitter << node;
    MICROSTRAIN_ERROR(node_, "Unable to get required key '%s' of type %s in %s", key.c_str(), typeid(T).name(), emitter.c_str());
    MICROSTRAIN_ERROR(node_, "  Exception: %s", t.what());
    throw std::runtime_error("");
  }
}

void EventsYaml::printEventTrigger(const mip::commands_3dm::EventTrigger& trigger)
{
  MICROSTRAIN_INFO(node_, "  instance = %u", trigger.instance);
  MICROSTRAIN_INFO(node_, "  type = %u", static_cast<uint8_t>(trigger.type));
  switch (trigger.type)
  {
    case mip::commands_3dm::EventTrigger::Type::GPIO:
      MICROSTRAIN_INFO(node_, "  pin = %u", trigger.parameters.gpio.pin);
      MICROSTRAIN_INFO(node_, "  mode = %u", static_cast<uint8_t>(trigger.parameters.gpio.mode));
      break;
    case mip::commands_3dm::EventTrigger::Type::THRESHOLD:
      MICROSTRAIN_INFO(node_, "  descriptor_set = 0x%02X", trigger.parameters.threshold.desc_set);
      MICROSTRAIN_INFO(node_, "  field_descriptor = 0x%02X", trigger.parameters.threshold.field_desc);
      MICROSTRAIN_INFO(node_, "  param_id = %u", trigger.parameters.threshold.param_id);
      MICROSTRAIN_INFO(node_, "  threshold_type = %u", static_cast<uint8_t>(trigger.parameters.threshold.type));
      switch (trigger.parameters.threshold.type)
      {
        case mip::commands_3dm::EventTrigger::ThresholdParams::Type::WINDOW:
          MICROSTRAIN_INFO(node_, "  low_threshold = %f", trigger.parameters.threshold.low_thres);
          MICROSTRAIN_INFO(node_, "  high_threshold = %f", trigger.parameters.threshold.high_thres);
          break;
        case mip::commands_3dm::EventTrigger::ThresholdParams::Type::INTERVAL:
          MICROSTRAIN_INFO(node_, "  interval_threshold = %f", trigger.parameters.threshold.int_thres);
          MICROSTRAIN_INFO(node_, "  interval = %f", trigger.parameters.threshold.interval);
          break;
      }
      break;
    case mip::commands_3dm::EventTrigger::Type::COMBINATION:
      MICROSTRAIN_INFO(node_, "  logic_table = 0x%04X", trigger.parameters.combination.logic_table);
      std::stringstream ss; ss << "[";
      const size_t max_input_triggers = sizeof(trigger.parameters.combination.input_triggers) / sizeof(trigger.parameters.combination.input_triggers[0]);
      for (int i = 0; i < max_input_triggers; i++)
      {
        ss << static_cast<int>(trigger.parameters.combination.input_triggers[i]);
        if (i < max_input_triggers - 1)
          ss << ", ";
      }
      ss << "]";
      MICROSTRAIN_INFO(node_, "  input_triggers = %s", ss.str().c_str());
      break;
  }
}

void EventsYaml::printEventAction(const mip::commands_3dm::EventAction& action)
{
  MICROSTRAIN_INFO(node_, "  instance = %u", action.instance);
  MICROSTRAIN_INFO(node_, "  trigger = %u", action.trigger);
  MICROSTRAIN_INFO(node_, "  type = %u", static_cast<uint8_t>(action.type));
  switch (action.type)
  {
    case mip::commands_3dm::EventAction::Type::GPIO:
      MICROSTRAIN_INFO(node_, "  pin = %u", action.parameters.gpio.pin);
      MICROSTRAIN_INFO(node_, "  mode = %u", static_cast<uint8_t>(action.parameters.gpio.mode));
      break;
    case mip::commands_3dm::EventAction::Type::MESSAGE:
      MICROSTRAIN_INFO(node_, "  descriptor_set = 0x%02x", action.parameters.message.desc_set);
      MICROSTRAIN_INFO(node_, "  decimation = %u", action.parameters.message.decimation);
      MICROSTRAIN_INFO(node_, "  num_fields = %u", action.parameters.message.num_fields);
      std::stringstream ss; ss << "[";
      for (int i = 0; i < action.parameters.message.num_fields; i++)
      {
        ss << "0x" << std::uppercase << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(action.parameters.message.descriptors[i]);
        if (i < action.parameters.message.num_fields - 1)
          ss << ", ";
      }
      ss << "]";
      MICROSTRAIN_INFO(node_, "  descriptors = %s", ss.str().c_str());
      break;
  }
}

}