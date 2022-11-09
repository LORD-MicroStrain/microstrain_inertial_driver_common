/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Driver Definition File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2020, Parker Hannifin Corp
//
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

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

  /**
   * \brief Will be overridden in the implementing class to return descriptor set for the associated field
   * \return descriptor set of the MIP field
   */
  virtual uint8_t descriptorSet() const = 0;

  /**
   * \brief Will be overridden in the implementing class to return field descriptor for the associated field
   * \return field descriptor of the MIP field
   */
  virtual uint8_t fieldDescriptor() const = 0;
};

/**
 * Template extension of FieldWrapper.
 * \tparam DataType Data type class to use for this field wrapper. Should be a class in one of the mip::data_* namespaces
 * \tparam DescriptorSet Optional descriptor set to use instead of the descriptor set on the data field class
 */
template<typename DataType, uint8_t DescriptorSet = DataType::DESCRIPTOR_SET>
class FieldWrapperType : public FieldWrapper
{
 public:
  using SharedPtr = std::shared_ptr<FieldWrapperType<DataType, DescriptorSet>>;
  using SharedPtrVec = std::vector<SharedPtr>;

  /**
   * \brief Returns the descriptor set of the DataType, or optionaly the DescriptorSet passed
   * \return The descriptor set that this template type was initialized with
   */
  uint8_t descriptorSet() const final
  {
    return DescriptorSet;
  }

  /**
   * \brief Returns the field descriptor of the DataType
   * \return The field descriptor that this template type was initialized with
   */
  uint8_t fieldDescriptor() const final
  {
    return DataType::FIELD_DESCRIPTOR;
  }

  /**
   * \brief Initializes an instance of this class
   * \return Shared pointer of this templated class type
   */
  static SharedPtr initialize()
  {
    return std::make_shared<FieldWrapperType<DataType, DescriptorSet>>();
  }
};

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MAPPINGS_MIP_MAPPING_H
