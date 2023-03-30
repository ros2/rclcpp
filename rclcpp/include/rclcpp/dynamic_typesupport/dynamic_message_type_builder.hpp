// Copyright 2023 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RCLCPP__DYNAMIC_TYPESUPPORT__DYNAMIC_MESSAGE_TYPE_BUILDER_HPP_
#define RCLCPP__DYNAMIC_TYPESUPPORT__DYNAMIC_MESSAGE_TYPE_BUILDER_HPP_

#include <memory>
#include <string>

#include "rclcpp/dynamic_typesupport/dynamic_serialization_support.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"

#include <rosidl_dynamic_typesupport/types.h>


namespace rclcpp
{
namespace dynamic_typesupport
{

class DynamicMessage;
class DynamicMessageType;

/// Utility wrapper class for rosidl_dynamic_typesupport_dynamic_type_builder_t *
/**
 * This class:
 * - Manages the lifetime of the raw pointer.
 * - Exposes getter methods to get the raw pointer and shared pointers
 * - Exposes the underlying serialization support API
 *
 * Ownership:
 * - This class borrows the rosidl_dynamic_typesupport_serialization_support_t stored in the passed
 *   DynamicSerializationSupport. So it cannot outlive the DynamicSerializationSupport.
 * - The DynamicSerializationSupport's rosidl_dynamic_typesupport_serialization_support_t pointer
 *   must point to the same location in memory as the stored raw pointer!
 *
 * Note: This class is meant to map to rosidl_dynamic_typesupport_dynamic_type_builder_t,
 *       facilitating the construction of dynamic types bottom-up in the C++ layer.
 *
 *       The usual method of obtaining a DynamicMessageType is through construction of
 *       rosidl_message_type_support_t via rcl_dynamic_message_typesupport_handle_init(), then taking
 *       ownership of its contents. But DynamicMessageTypeBuilder can also be used to obtain
 *       DynamicMessageType by constructing it bottom-up instead, since it exposes the lower_level
 *       rosidl methods.
 */
class DynamicMessageTypeBuilder : public std::enable_shared_from_this<DynamicMessageTypeBuilder>
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(DynamicMessageTypeBuilder)

  // CONSTRUCTION ==================================================================================
  // All constructors require a passed in DynamicSerializationSupport::SharedPtr, to extend the
  // lifetime of the serialization support.
  //
  // In cases where a dynamic type builder pointer is passed, the serialization support composed by
  // the builder should be the exact same object managed by the DynamicSerializationSupport,
  // otherwise the lifetime management will not work properly.

  /// Construct a new DynamicMessageTypeBuilder with the provided serialization support
  RCLCPP_PUBLIC
  DynamicMessageTypeBuilder(
    DynamicSerializationSupport::SharedPtr serialization_support,
    const std::string & name);

  /// Assume ownership of raw pointer
  RCLCPP_PUBLIC
  DynamicMessageTypeBuilder(
    DynamicSerializationSupport::SharedPtr serialization_support,
    rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder);

  /// Copy shared pointer
  RCLCPP_PUBLIC
  DynamicMessageTypeBuilder(
    DynamicSerializationSupport::SharedPtr serialization_support,
    std::shared_ptr<rosidl_dynamic_typesupport_dynamic_type_builder_t> dynamic_type_builder);

  /// Copy constructor
  RCLCPP_PUBLIC
  DynamicMessageTypeBuilder(const DynamicMessageTypeBuilder & other);

  /// Move constructor
  RCLCPP_PUBLIC
  DynamicMessageTypeBuilder(DynamicMessageTypeBuilder && other) noexcept;

  /// Copy assignment
  RCLCPP_PUBLIC
  DynamicMessageTypeBuilder & operator=(const DynamicMessageTypeBuilder & other);

  /// Move assignment
  RCLCPP_PUBLIC
  DynamicMessageTypeBuilder & operator=(DynamicMessageTypeBuilder && other) noexcept;

  /// From description
  RCLCPP_PUBLIC
  DynamicMessageTypeBuilder(
    DynamicSerializationSupport::SharedPtr serialization_support,
    const rosidl_runtime_c__type_description__TypeDescription & description);

  RCLCPP_PUBLIC
  virtual ~DynamicMessageTypeBuilder();

  /// Swaps the serialization support if serialization_support is populated
  RCLCPP_PUBLIC
  void
  init_from_description(
    const rosidl_runtime_c__type_description__TypeDescription & description,
    DynamicSerializationSupport::SharedPtr serialization_support = nullptr);


  // GETTERS =======================================================================================
  RCLCPP_PUBLIC
  const std::string
  get_library_identifier() const;

  RCLCPP_PUBLIC
  const std::string
  get_name() const;

  RCLCPP_PUBLIC
  rosidl_dynamic_typesupport_dynamic_type_builder_t *
  get_rosidl_dynamic_type_builder();

  RCLCPP_PUBLIC
  const rosidl_dynamic_typesupport_dynamic_type_builder_t *
  get_rosidl_dynamic_type_builder() const;

  RCLCPP_PUBLIC
  std::shared_ptr<rosidl_dynamic_typesupport_dynamic_type_builder_t>
  get_shared_rosidl_dynamic_type_builder();

  RCLCPP_PUBLIC
  std::shared_ptr<const rosidl_dynamic_typesupport_dynamic_type_builder_t>
  get_shared_rosidl_dynamic_type_builder() const;

  RCLCPP_PUBLIC
  DynamicSerializationSupport::SharedPtr
  get_shared_dynamic_serialization_support();

  RCLCPP_PUBLIC
  DynamicSerializationSupport::ConstSharedPtr
  get_shared_dynamic_serialization_support() const;


  // METHODS =======================================================================================
  RCLCPP_PUBLIC
  void
  set_name(const std::string & name);

  RCLCPP_PUBLIC
  DynamicMessageTypeBuilder
  clone() const;

  RCLCPP_PUBLIC
  DynamicMessageTypeBuilder::SharedPtr
  clone_shared() const;

  RCLCPP_PUBLIC
  void
  clear();

  RCLCPP_PUBLIC
  DynamicMessage
  build_dynamic_message();

  RCLCPP_PUBLIC
  DynamicMessage::SharedPtr
  build_dynamic_message_shared();

  RCLCPP_PUBLIC
  DynamicMessageType
  build_dynamic_message_type();

  RCLCPP_PUBLIC
  DynamicMessageType::SharedPtr
  build_dynamic_message_type_shared();


  // ADD MEMBERS TEMPLATES =========================================================================
  /**
   * Since we're in a ROS layer, these should support all ROS interface C++ types as found in:
   * https://docs.ros.org/en/rolling/Concepts/About-ROS-Interfaces.html
   *
   * Explicitly:
   * - Basic types:        bool, byte, char
   * - Float types:        float, double
   * - Int types:          int8_t, int16_t, int32_t, int64_t
   * - Unsigned int types: uint8_t, uint16_t, uint32_t, uint64_t
   * - String types:       std::string, std::u16string
   */

  template<typename MemberT>
  void
  add_member(
    rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
    const std::string & default_value = "");

  template<typename MemberT>
  void
  add_array_member(
    rosidl_dynamic_typesupport_member_id_t id, const std::string & name, size_t array_length,
    const std::string & default_value = "");

  template<typename MemberT>
  void
  add_unbounded_sequence_member(
    rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
    const std::string & default_value = "");

  template<typename MemberT>
  void
  add_bounded_sequence_member(
    rosidl_dynamic_typesupport_member_id_t id, const std::string & name, size_t sequence_bound,
    const std::string & default_value = "");


  // ADD FIXED STRING MEMBERS ======================================================================
  RCLCPP_PUBLIC
  void
  add_fixed_string_member(
    rosidl_dynamic_typesupport_member_id_t id, const std::string & name, size_t string_length,
    const std::string & default_value = "");

  RCLCPP_PUBLIC
  void
  add_fixed_wstring_member(
    rosidl_dynamic_typesupport_member_id_t id, const std::string & name, size_t wstring_length,
    const std::string & default_value = "");

  RCLCPP_PUBLIC
  void
  add_fixed_string_array_member(
    rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
    size_t string_length, size_t array_length, const std::string & default_value = "");

  RCLCPP_PUBLIC
  void
  add_fixed_wstring_array_member(
    rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
    size_t wstring_length, size_t array_length, const std::string & default_value = "");

  RCLCPP_PUBLIC
  void
  add_fixed_string_unbounded_sequence_member(
    rosidl_dynamic_typesupport_member_id_t id, const std::string & name, size_t string_length,
    const std::string & default_value = "");

  RCLCPP_PUBLIC
  void
  add_fixed_wstring_unbounded_sequence_member(
    rosidl_dynamic_typesupport_member_id_t id, const std::string & name, size_t wstring_length,
    const std::string & default_value = "");

  RCLCPP_PUBLIC
  void
  add_fixed_string_bounded_sequence_member(
    rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
    size_t string_length, size_t sequence_bound, const std::string & default_value = "");

  RCLCPP_PUBLIC
  void
  add_fixed_wstring_bounded_sequence_member(
    rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
    size_t wstring_length, size_t sequence_bound, const std::string & default_value = "");


  // ADD BOUNDED STRING MEMBERS ====================================================================
  RCLCPP_PUBLIC
  void
  add_bounded_string_member(
    rosidl_dynamic_typesupport_member_id_t id, const std::string & name, size_t string_bound,
    const std::string & default_value = "");

  RCLCPP_PUBLIC
  void
  add_bounded_wstring_member(
    rosidl_dynamic_typesupport_member_id_t id, const std::string & name, size_t wstring_bound,
    const std::string & default_value = "");

  RCLCPP_PUBLIC
  void
  add_bounded_string_array_member(
    rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
    size_t string_bound, size_t array_length, const std::string & default_value = "");

  RCLCPP_PUBLIC
  void
  add_bounded_wstring_array_member(
    rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
    size_t wstring_bound, size_t array_length, const std::string & default_value = "");

  RCLCPP_PUBLIC
  void
  add_bounded_string_unbounded_sequence_member(
    rosidl_dynamic_typesupport_member_id_t id, const std::string & name, size_t string_bound,
    const std::string & default_value = "");

  RCLCPP_PUBLIC
  void
  add_bounded_wstring_unbounded_sequence_member(
    rosidl_dynamic_typesupport_member_id_t id, const std::string & name, size_t wstring_bound,
    const std::string & default_value = "");

  RCLCPP_PUBLIC
  void
  add_bounded_string_bounded_sequence_member(
    rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
    size_t string_bound, size_t sequence_bound, const std::string & default_value = "");

  RCLCPP_PUBLIC
  void
  add_bounded_wstring_bounded_sequence_member(
    rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
    size_t wstring_bound, size_t sequence_bound, const std::string & default_value = "");


  // ADD NESTED MEMBERS ============================================================================
  RCLCPP_PUBLIC
  void
  add_complex_member(
    rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
    DynamicMessageType & nested_type, const std::string & default_value = "");

  RCLCPP_PUBLIC
  void
  add_complex_array_member(
    rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
    DynamicMessageType & nested_type, size_t array_length, const std::string & default_value = "");

  RCLCPP_PUBLIC
  void
  add_complex_unbounded_sequence_member(
    rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
    DynamicMessageType & nested_type, const std::string & default_value = "");

  RCLCPP_PUBLIC
  void
  add_complex_bounded_sequence_member(
    rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
    DynamicMessageType & nested_type, size_t sequence_bound, const std::string & default_value = "");

  RCLCPP_PUBLIC
  void
  add_complex_member_builder(
    rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
    DynamicMessageTypeBuilder & nested_type_builder, const std::string & default_value = "");

  RCLCPP_PUBLIC
  void
  add_complex_array_member_builder(
    rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
    DynamicMessageTypeBuilder & nested_type_builder, size_t array_length,
    const std::string & default_value = "");

  RCLCPP_PUBLIC
  void
  add_complex_unbounded_sequence_member_builder(
    rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
    DynamicMessageTypeBuilder & nested_type_builder, const std::string & default_value = "");

  RCLCPP_PUBLIC
  void
  add_complex_bounded_sequence_member_builder(
    rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
    DynamicMessageTypeBuilder & nested_type_builder, size_t sequence_bound,
    const std::string & default_value = "");

protected:
  // NOTE(methylDragon):
  // This is just here to extend the lifetime of the serialization support
  // It isn't actually used by the builder since the builder should compose its own support
  //
  // ... Though ideally it should be the exact same support as the one stored in the
  // DynamicSerializationSupport
  DynamicSerializationSupport::SharedPtr serialization_support_;

  std::shared_ptr<rosidl_dynamic_typesupport_dynamic_type_builder_t> rosidl_dynamic_type_builder_;

private:
  RCLCPP_PUBLIC
  DynamicMessageTypeBuilder();

  RCLCPP_PUBLIC
  void
  init_from_serialization_support_(
    DynamicSerializationSupport::SharedPtr serialization_support,
    const std::string & name);

  RCLCPP_PUBLIC
  bool
  match_serialization_support_(
    const DynamicSerializationSupport & serialization_support,
    const rosidl_dynamic_typesupport_dynamic_type_builder_t & dynamic_type_builder);
};


}  // namespace dynamic_typesupport
}  // namespace rclcpp


#endif  // RCLCPP__DYNAMIC_TYPESUPPORT__DYNAMIC_MESSAGE_TYPE_BUILDER_HPP_
