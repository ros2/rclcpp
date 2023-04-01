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

#ifndef RCLCPP__DYNAMIC_TYPESUPPORT__DETAIL__DYNAMIC_MESSAGE_TYPE_BUILDER_IMPL_HPP_
#define RCLCPP__DYNAMIC_TYPESUPPORT__DETAIL__DYNAMIC_MESSAGE_TYPE_BUILDER_IMPL_HPP_

#include <rosidl_dynamic_typesupport/types.h>
#include <rosidl_dynamic_typesupport/api/dynamic_type.h>

#include <cstdint>
#include <cstddef>
#include <memory>
#include <string>

#include "rclcpp/exceptions.hpp"

#ifndef RCLCPP__DYNAMIC_TYPESUPPORT__DYNAMIC_MESSAGE_TYPE_BUILDER_HPP_
#include "rclcpp/dynamic_typesupport/dynamic_message_type_builder.hpp"
#endif


#define __DYNAMIC_MESSAGE_TYPE_BUILDER_ADD_MEMBER_FN(MemberT, FunctionT) \
  template<> \
  void \
  DynamicMessageTypeBuilder::add_member<MemberT>( \
    rosidl_dynamic_typesupport_member_id_t id, \
    const std::string & name, \
    const std::string & default_value) \
  { \
    rosidl_dynamic_typesupport_dynamic_type_builder_add_ ## FunctionT ## _member( \
      rosidl_dynamic_type_builder_.get(), \
      id, name.c_str(), name.size(), default_value.c_str(), default_value.size()); \
  }

#define __DYNAMIC_MESSAGE_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(MemberT, FunctionT) \
  template<> \
  void \
  DynamicMessageTypeBuilder::add_array_member<MemberT>( \
    rosidl_dynamic_typesupport_member_id_t id, const std::string & name, \
    size_t array_length, \
    const std::string & default_value) \
  { \
    rosidl_dynamic_typesupport_dynamic_type_builder_add_ ## FunctionT ## _array_member( \
      rosidl_dynamic_type_builder_.get(), \
      id, name.c_str(), name.size(), default_value.c_str(), default_value.size(), \
      array_length); \
  }

#define __DYNAMIC_MESSAGE_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(MemberT, FunctionT) \
  template<> \
  void \
  DynamicMessageTypeBuilder::add_unbounded_sequence_member<MemberT>( \
    rosidl_dynamic_typesupport_member_id_t id, \
    const std::string & name, \
    const std::string & default_value) \
  { \
    rosidl_dynamic_typesupport_dynamic_type_builder_add_ ## FunctionT ## \
    _unbounded_sequence_member( \
      rosidl_dynamic_type_builder_.get(), \
      id, name.c_str(), name.size(), default_value.c_str(), default_value.size()); \
  }

#define __DYNAMIC_MESSAGE_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(MemberT, FunctionT) \
  template<> \
  void \
  DynamicMessageTypeBuilder::add_bounded_sequence_member<MemberT>( \
    rosidl_dynamic_typesupport_member_id_t id, \
    const std::string & name, \
    size_t sequence_bound, \
    const std::string & default_value) \
  { \
    rosidl_dynamic_typesupport_dynamic_type_builder_add_ ## FunctionT ## _bounded_sequence_member( \
      rosidl_dynamic_type_builder_.get(), \
      id, name.c_str(), name.size(), default_value.c_str(), default_value.size(), \
      sequence_bound); \
  }

#define DYNAMIC_MESSAGE_TYPE_BUILDER_DEFINITIONS(MemberT, FunctionT) \
  __DYNAMIC_MESSAGE_TYPE_BUILDER_ADD_MEMBER_FN(MemberT, FunctionT) \
  __DYNAMIC_MESSAGE_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(MemberT, FunctionT) \
  __DYNAMIC_MESSAGE_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(MemberT, FunctionT) \
  __DYNAMIC_MESSAGE_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(MemberT, FunctionT) \


namespace rclcpp
{
namespace dynamic_typesupport
{

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

DYNAMIC_MESSAGE_TYPE_BUILDER_DEFINITIONS(bool, bool);
DYNAMIC_MESSAGE_TYPE_BUILDER_DEFINITIONS(std::byte, byte);
DYNAMIC_MESSAGE_TYPE_BUILDER_DEFINITIONS(char, char);
DYNAMIC_MESSAGE_TYPE_BUILDER_DEFINITIONS(float, float32);
DYNAMIC_MESSAGE_TYPE_BUILDER_DEFINITIONS(double, float64);
DYNAMIC_MESSAGE_TYPE_BUILDER_DEFINITIONS(int8_t, int8);
DYNAMIC_MESSAGE_TYPE_BUILDER_DEFINITIONS(int16_t, int16);
DYNAMIC_MESSAGE_TYPE_BUILDER_DEFINITIONS(int32_t, int32);
DYNAMIC_MESSAGE_TYPE_BUILDER_DEFINITIONS(int64_t, int64);
DYNAMIC_MESSAGE_TYPE_BUILDER_DEFINITIONS(uint8_t, uint8);
DYNAMIC_MESSAGE_TYPE_BUILDER_DEFINITIONS(uint16_t, uint16);
DYNAMIC_MESSAGE_TYPE_BUILDER_DEFINITIONS(uint32_t, uint32);
DYNAMIC_MESSAGE_TYPE_BUILDER_DEFINITIONS(uint64_t, uint64);
DYNAMIC_MESSAGE_TYPE_BUILDER_DEFINITIONS(std::string, string);
DYNAMIC_MESSAGE_TYPE_BUILDER_DEFINITIONS(std::u16string, wstring);


// THROW FOR UNSUPPORTED TYPES =====================================================================
template<typename MemberT>
void
DynamicMessageTypeBuilder::add_member(
  rosidl_dynamic_typesupport_member_id_t id,
  const std::string & name,
  const std::string & default_value)
{
  throw rclcpp::exceptions::UnimplementedError(
          "add_member is not implemented for input type");
}


template<typename MemberT>
void
DynamicMessageTypeBuilder::add_array_member(
  rosidl_dynamic_typesupport_member_id_t id,
  const std::string & name,
  size_t array_length, const std::string & default_value)
{
  throw rclcpp::exceptions::UnimplementedError(
          "add_array_member is not implemented for input type");
}


template<typename MemberT>
void
DynamicMessageTypeBuilder::add_unbounded_sequence_member(
  rosidl_dynamic_typesupport_member_id_t id,
  const std::string & name,
  const std::string & default_value)
{
  throw rclcpp::exceptions::UnimplementedError(
          "add_unbounded_sequence_member is not implemented for input type");
}


template<typename MemberT>
void
DynamicMessageTypeBuilder::add_bounded_sequence_member(
  rosidl_dynamic_typesupport_member_id_t id,
  const std::string & name,
  size_t sequence_bound,
  const std::string & default_value)
{
  throw rclcpp::exceptions::UnimplementedError(
          "add_bounded_sequence_member is not implemented for input type");
}


}  // namespace dynamic_typesupport
}  // namespace rclcpp

#undef __DYNAMIC_MESSAGE_TYPE_BUILDER_ADD_MEMBER_FN
#undef __DYNAMIC_MESSAGE_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN
#undef __DYNAMIC_MESSAGE_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN
#undef __DYNAMIC_MESSAGE_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN
#undef DYNAMIC_MESSAGE_TYPE_BUILDER_DEFINITIONS

#endif  // RCLCPP__DYNAMIC_TYPESUPPORT__DETAIL__DYNAMIC_MESSAGE_TYPE_BUILDER_IMPL_HPP_
