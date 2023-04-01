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

#ifndef RCLCPP__DYNAMIC_TYPESUPPORT__DYNAMIC_MESSAGE_HPP_
#define RCLCPP__DYNAMIC_TYPESUPPORT__DYNAMIC_MESSAGE_HPP_

#include <rcl/types.h>
#include <rosidl_dynamic_typesupport/types.h>

#include <memory>
#include <string>

#include "rclcpp/dynamic_typesupport/dynamic_message_type.hpp"
#include "rclcpp/dynamic_typesupport/dynamic_serialization_support.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"


namespace rclcpp
{
namespace dynamic_typesupport
{


class DynamicMessageType;
class DynamicMessageTypeBuilder;

/// Utility wrapper class for rosidl_dynamic_typesupport_dynamic_data_t *
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
 * Note: This class is meant to map to the lower level rosidl_dynamic_typesupport_dynamic_data_t,
 *       even though rosidl_dynamic_typesupport_dynamic_data_t is equivalent to
 *       rosidl_dynamic_typesupport_dynamic_data_t, exposing the fundamental methods available to
 *       rosidl_dynamic_typesupport_dynamic_data_t, allowing the user to access the data's fields.
 */
class DynamicMessage : public std::enable_shared_from_this<DynamicMessage>
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(DynamicMessage)

  // CONSTRUCTION ==================================================================================
  // Most constructors require a passed in DynamicSerializationSupport::SharedPtr, to extend the
  // lifetime of the serialization support (if the constructor cannot otherwise get it from args).
  //
  // In cases where a dynamic data pointer is passed, the serialization support composed by
  // the data should be the exact same object managed by the DynamicSerializationSupport,
  // otherwise the lifetime management will not work properly.

  /// Construct a new DynamicMessage with the provided dynamic type builder
  RCLCPP_PUBLIC
  explicit DynamicMessage(std::shared_ptr<DynamicMessageTypeBuilder> dynamic_type_builder);

  /// Construct a new DynamicMessage with the provided dynamic type
  RCLCPP_PUBLIC
  explicit DynamicMessage(std::shared_ptr<DynamicMessageType> dynamic_type);

  /// Assume ownership of raw pointer
  RCLCPP_PUBLIC
  DynamicMessage(
    DynamicSerializationSupport::SharedPtr serialization_support,
    rosidl_dynamic_typesupport_dynamic_data_t * rosidl_dynamic_data);

  /// Copy shared pointer
  RCLCPP_PUBLIC
  DynamicMessage(
    DynamicSerializationSupport::SharedPtr serialization_support,
    std::shared_ptr<rosidl_dynamic_typesupport_dynamic_data_t> rosidl_dynamic_data);

  /// Loaning constructor
  /// Must only be called with raw ptr obtained from loaning!
  // NOTE(methylDragon): I'd put this in protected, but I need this exposed to
  //                     enable_shared_from_this...
  RCLCPP_PUBLIC
  DynamicMessage(
    DynamicMessage::SharedPtr parent_data,
    rosidl_dynamic_typesupport_dynamic_data_t * rosidl_loaned_data);

  // NOTE(methylDragon): Deliberately no constructor from description to nudge users towards using
  //                     construction from dynamic type/builder, which is more efficient

  /// Copy constructor
  RCLCPP_PUBLIC
  DynamicMessage(const DynamicMessage & other);

  /// Move constructor
  RCLCPP_PUBLIC
  DynamicMessage(DynamicMessage && other) noexcept;

  /// Copy assignment
  RCLCPP_PUBLIC
  DynamicMessage & operator=(const DynamicMessage & other);

  /// Move assignment
  RCLCPP_PUBLIC
  DynamicMessage & operator=(DynamicMessage && other) noexcept;

  RCLCPP_PUBLIC
  virtual ~DynamicMessage();


  // GETTERS =======================================================================================
  RCLCPP_PUBLIC
  const std::string
  get_library_identifier() const;

  RCLCPP_PUBLIC
  const std::string
  get_name() const;

  RCLCPP_PUBLIC
  rosidl_dynamic_typesupport_dynamic_data_t *
  get_rosidl_dynamic_data();

  RCLCPP_PUBLIC
  const rosidl_dynamic_typesupport_dynamic_data_t *
  get_rosidl_dynamic_data() const;

  RCLCPP_PUBLIC
  std::shared_ptr<rosidl_dynamic_typesupport_dynamic_data_t>
  get_shared_rosidl_dynamic_data();

  RCLCPP_PUBLIC
  std::shared_ptr<const rosidl_dynamic_typesupport_dynamic_data_t>
  get_shared_rosidl_dynamic_data() const;

  RCLCPP_PUBLIC
  DynamicSerializationSupport::SharedPtr
  get_shared_dynamic_serialization_support();

  RCLCPP_PUBLIC
  DynamicSerializationSupport::ConstSharedPtr
  get_shared_dynamic_serialization_support() const;

  RCLCPP_PUBLIC
  size_t
  get_item_count() const;

  RCLCPP_PUBLIC
  rosidl_dynamic_typesupport_member_id_t
  get_member_id(size_t index) const;

  RCLCPP_PUBLIC
  rosidl_dynamic_typesupport_member_id_t
  get_member_id(const std::string & name) const;

  RCLCPP_PUBLIC
  rosidl_dynamic_typesupport_member_id_t
  get_array_index(size_t index) const;

  RCLCPP_PUBLIC
  rosidl_dynamic_typesupport_member_id_t
  get_array_index(const std::string & name) const;


  // METHODS =======================================================================================
  RCLCPP_PUBLIC
  DynamicMessage
  clone() const;

  RCLCPP_PUBLIC
  DynamicMessage::SharedPtr
  clone_shared() const;

  RCLCPP_PUBLIC
  DynamicMessage
  init_from_type(DynamicMessageType & type) const;

  RCLCPP_PUBLIC
  DynamicMessage::SharedPtr
  init_from_type_shared(DynamicMessageType & type) const;

  RCLCPP_PUBLIC
  bool
  equals(const DynamicMessage & other) const;

  RCLCPP_PUBLIC
  DynamicMessage::SharedPtr
  loan_value(rosidl_dynamic_typesupport_member_id_t id);

  RCLCPP_PUBLIC
  DynamicMessage::SharedPtr
  loan_value(const std::string & name);

  RCLCPP_PUBLIC
  void
  clear_all_values();

  RCLCPP_PUBLIC
  void
  clear_nonkey_values();

  RCLCPP_PUBLIC
  void
  clear_value(rosidl_dynamic_typesupport_member_id_t id);

  RCLCPP_PUBLIC
  void
  clear_value(const std::string & name);

  RCLCPP_PUBLIC
  void
  clear_sequence();

  RCLCPP_PUBLIC
  rosidl_dynamic_typesupport_member_id_t
  insert_sequence_data();

  RCLCPP_PUBLIC
  void
  remove_sequence_data(rosidl_dynamic_typesupport_member_id_t index);

  RCLCPP_PUBLIC
  void
  print() const;

  RCLCPP_PUBLIC
  bool
  serialize(rcl_serialized_message_t * buffer);

  RCLCPP_PUBLIC
  bool
  deserialize(rcl_serialized_message_t * buffer);


  // MEMBER ACCESS TEMPLATES =======================================================================
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

  template<typename ValueT>
  ValueT
  get_value(rosidl_dynamic_typesupport_member_id_t id);

  template<typename ValueT>
  ValueT
  get_value(const std::string & name);

  template<typename ValueT>
  void
  set_value(rosidl_dynamic_typesupport_member_id_t id, ValueT value);

  template<typename ValueT>
  void
  set_value(const std::string & name, ValueT value);

  template<typename ValueT>
  rosidl_dynamic_typesupport_member_id_t
  insert_value(ValueT value);


  // FIXED STRING MEMBER ACCESS ====================================================================
  RCLCPP_PUBLIC
  const std::string
  get_fixed_string_value(rosidl_dynamic_typesupport_member_id_t id, size_t string_length);

  RCLCPP_PUBLIC
  const std::string
  get_fixed_string_value(const std::string & name, size_t string_length);

  RCLCPP_PUBLIC
  const std::u16string
  get_fixed_wstring_value(rosidl_dynamic_typesupport_member_id_t id, size_t wstring_length);

  RCLCPP_PUBLIC
  const std::u16string
  get_fixed_wstring_value(const std::string & name, size_t wstring_length);

  RCLCPP_PUBLIC
  void
  set_fixed_string_value(
    rosidl_dynamic_typesupport_member_id_t id, const std::string value, size_t string_length);

  RCLCPP_PUBLIC
  void
  set_fixed_string_value(const std::string & name, const std::string value, size_t string_length);

  RCLCPP_PUBLIC
  void
  set_fixed_wstring_value(
    rosidl_dynamic_typesupport_member_id_t id, const std::u16string value, size_t wstring_length);

  RCLCPP_PUBLIC
  void
  set_fixed_wstring_value(
    const std::string & name, const std::u16string value, size_t wstring_length);

  RCLCPP_PUBLIC
  rosidl_dynamic_typesupport_member_id_t
  insert_fixed_string_value(const std::string value, size_t string_length);

  RCLCPP_PUBLIC
  rosidl_dynamic_typesupport_member_id_t
  insert_fixed_wstring_value(const std::u16string value, size_t wstring_length);


  // BOUNDED STRING MEMBER ACCESS ==================================================================
  RCLCPP_PUBLIC
  const std::string
  get_bounded_string_value(rosidl_dynamic_typesupport_member_id_t id, size_t string_bound);

  RCLCPP_PUBLIC
  const std::string
  get_bounded_string_value(const std::string & name, size_t string_bound);

  RCLCPP_PUBLIC
  const std::u16string
  get_bounded_wstring_value(rosidl_dynamic_typesupport_member_id_t id, size_t wstring_bound);

  RCLCPP_PUBLIC
  const std::u16string
  get_bounded_wstring_value(const std::string & name, size_t wstring_bound);

  RCLCPP_PUBLIC
  void
  set_bounded_string_value(
    rosidl_dynamic_typesupport_member_id_t id, const std::string value, size_t string_bound);

  RCLCPP_PUBLIC
  void
  set_bounded_string_value(const std::string & name, const std::string value, size_t string_bound);

  RCLCPP_PUBLIC
  void
  set_bounded_wstring_value(
    rosidl_dynamic_typesupport_member_id_t id, const std::u16string value, size_t wstring_bound);

  RCLCPP_PUBLIC
  void
  set_bounded_wstring_value(
    const std::string & name, const std::u16string value, size_t wstring_bound);

  RCLCPP_PUBLIC
  rosidl_dynamic_typesupport_member_id_t
  insert_bounded_string_value(const std::string value, size_t string_bound);

  RCLCPP_PUBLIC
  rosidl_dynamic_typesupport_member_id_t
  insert_bounded_wstring_value(const std::u16string value, size_t wstring_bound);


  // NESTED MEMBER ACCESS ==========================================================================
  RCLCPP_PUBLIC
  DynamicMessage
  get_complex_value(rosidl_dynamic_typesupport_member_id_t id);

  RCLCPP_PUBLIC
  DynamicMessage
  get_complex_value(const std::string & name);

  RCLCPP_PUBLIC
  DynamicMessage::SharedPtr
  get_complex_value_shared(rosidl_dynamic_typesupport_member_id_t id);

  RCLCPP_PUBLIC
  DynamicMessage::SharedPtr
  get_complex_value_shared(const std::string & name);

  RCLCPP_PUBLIC
  void
  set_complex_value(rosidl_dynamic_typesupport_member_id_t id, DynamicMessage & value);

  RCLCPP_PUBLIC
  void
  set_complex_value(const std::string & name, DynamicMessage & value);

  RCLCPP_PUBLIC
  rosidl_dynamic_typesupport_member_id_t
  insert_complex_value_copy(const DynamicMessage & value);

  RCLCPP_PUBLIC
  rosidl_dynamic_typesupport_member_id_t
  insert_complex_value(DynamicMessage & value);

protected:
  // NOTE(methylDragon):
  // This is just here to extend the lifetime of the serialization support
  // It isn't actually used by the builder since the builder should compose its own support
  //
  // ... Though ideally it should be the exact same support as the one stored in the
  // DynamicSerializationSupport
  DynamicSerializationSupport::SharedPtr serialization_support_;

  std::shared_ptr<rosidl_dynamic_typesupport_dynamic_data_t> rosidl_dynamic_data_;

  bool is_loaned_;

  // Used for returning the loaned value, and lifetime management
  DynamicMessage::SharedPtr parent_data_;

private:
  RCLCPP_PUBLIC
  DynamicMessage();

  RCLCPP_PUBLIC
  bool
  match_serialization_support_(
    const DynamicSerializationSupport & serialization_support,
    const rosidl_dynamic_typesupport_dynamic_data_t & dynamic_data);
};


}  // namespace dynamic_typesupport
}  // namespace rclcpp

#endif  // RCLCPP__DYNAMIC_TYPESUPPORT__DYNAMIC_MESSAGE_HPP_
