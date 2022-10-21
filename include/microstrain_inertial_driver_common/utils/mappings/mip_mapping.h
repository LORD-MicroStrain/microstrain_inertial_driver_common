#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MAPPINGS_MIP_MAPPING_H_
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MAPPINGS_MIP_MAPPING_H_

#include <stdint.h>

#include <memory>
#include <vector>

namespace microstrain
{

/**
 * Wrapper for MIP fields which allows them to be stored in the same container
 */
class FieldWrapper
{
 public:
  using SharedPtr = std::shared_ptr<FieldWrapper>;
  using SharedPtrVec = std::vector<SharedPtr>;

  virtual uint8_t descriptorSet() const = 0;
  virtual uint8_t fieldDescriptor() const = 0;
};

/**
 * Extension of FieldWrapper
 */
template<typename DataType, uint8_t DescriptorSet = DataType::DESCRIPTOR_SET>
class FieldWrapperType : public FieldWrapper
{
 public:
  using SharedPtr = std::shared_ptr<FieldWrapperType<DataType, DescriptorSet>>;
  using SharedPtrVec = std::vector<SharedPtr>;

  uint8_t descriptorSet() const override
  {
    return DescriptorSet;
  }

  uint8_t fieldDescriptor() const override
  {
    return DataType::FIELD_DESCRIPTOR;
  }

  static SharedPtr initialize()
  {
    return std::make_shared<FieldWrapperType<DataType, DescriptorSet>>();
  }
};

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MAPPINGS_MIP_MAPPING_H