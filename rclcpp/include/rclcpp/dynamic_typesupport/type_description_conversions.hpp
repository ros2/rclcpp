// Copyright 2025 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__DYNAMIC_TYPESUPPORT__TYPE_DESCRIPTION_CONVERSIONS_HPP_
#define RCLCPP__DYNAMIC_TYPESUPPORT__TYPE_DESCRIPTION_CONVERSIONS_HPP_

#include "rosidl_runtime_cpp/type_description/individual_type_description__struct.hpp"
#include "rosidl_runtime_cpp/type_description/type_description__struct.hpp"
#include "rosidl_runtime_cpp/type_description/type_source__struct.hpp"
#include "type_description_interfaces/msg/individual_type_description.hpp"
#include "type_description_interfaces/msg/type_description.hpp"
#include "type_description_interfaces/msg/type_source.hpp"

namespace rclcpp
{
namespace dynamic_typesupport
{

// IndividualTypeDescription =======================================================================

/// Convert a runtime individual type description struct to its corresponding message.
/**
 * This function converts a rosidl_runtime_cpp::type_description::IndividualTypeDescription
 * struct to the corresponding type_description_interfaces/msg/IndividualTypeDescription
 * msg.
 *
 * \param[in] runtime_individual_description the runtime struct to convert
 * \return the converted individual type description msg
 */
template<
  typename ToAllocatorT = std::allocator<void>,
  typename FromAllocatorT
>
type_description_interfaces::msg::IndividualTypeDescription_<ToAllocatorT>
convert_individual_type_description_runtime_to_msg(
  const rosidl_runtime_cpp::type_description::IndividualTypeDescription_<FromAllocatorT> &
  runtime_individual_description, const ToAllocatorT & alloc = ToAllocatorT())
{
  type_description_interfaces::msg::IndividualTypeDescription_<ToAllocatorT> out(alloc);
  out.type_name = runtime_individual_description.type_name;

  for (const auto & field : runtime_individual_description.fields) {
    out.fields.emplace_back();
    out.fields.back().name = field.name;
    out.fields.back().default_value = field.default_value;

    out.fields.back().type.type_id = field.type.type_id;
    out.fields.back().type.capacity = field.type.capacity;
    out.fields.back().type.string_capacity = field.type.string_capacity;
    out.fields.back().type.nested_type_name = field.type.nested_type_name;
  }
  return out;
}

/// Convert a individual type description message to its corresponding runtime struct.
/**
 * This function converts a type_description_interfaces/msg/IndividualTypeDescription msg
 * to the corresponding rosidl_runtime_cpp::type_description::IndividualTypeDescription
 * struct.
 *
 * \param[in] description_msg the message to convert
 * \return the converted runtime struct
 */
template<
  typename ToAllocatorT = std::allocator<void>,
  typename FromAllocatorT
>
rosidl_runtime_cpp::type_description::IndividualTypeDescription_<ToAllocatorT>
convert_individual_type_description_msg_to_runtime(
  const type_description_interfaces::msg::IndividualTypeDescription_<FromAllocatorT> &
  individual_description_msg, const ToAllocatorT & alloc = ToAllocatorT())
{
  rosidl_runtime_cpp::type_description::IndividualTypeDescription_<ToAllocatorT> out(alloc);
  out.type_name = individual_description_msg.type_name;

  for (const auto & field : individual_description_msg.fields) {
    out.fields.emplace_back();
    out.fields.back().name = field.name;
    out.fields.back().default_value = field.default_value;

    out.fields.back().type.type_id = field.type.type_id;
    out.fields.back().type.capacity = field.type.capacity;
    out.fields.back().type.string_capacity = field.type.string_capacity;
    out.fields.back().type.nested_type_name = field.type.nested_type_name;
  }
  return out;
}

// TypeDescription =================================================================================

/// Convert a runtime type description struct to its corresponding message.
/**
 * This function converts a rosidl_runtime_cpp::type_description::TypeDescription
 * struct to the corresponding type_description_interfaces/msg/TypeDescription
 * msg.
 *
 * \param[in] runtime_description the runtime struct to convert
 * \return the converted message
 */
template<
  typename ToAllocatorT = std::allocator<void>,
  typename FromAllocatorT
>
type_description_interfaces::msg::TypeDescription_<ToAllocatorT>
convert_type_description_runtime_to_msg(
  const rosidl_runtime_cpp::type_description::TypeDescription_<FromAllocatorT> & runtime_description,
  const ToAllocatorT & alloc = ToAllocatorT())
{
  type_description_interfaces::msg::TypeDescription_<ToAllocatorT> out(alloc);
  out.type_description =
    convert_individual_type_description_runtime_to_msg(runtime_description.type_description, alloc);

  for (const auto & referenced_type_description :
    runtime_description.referenced_type_descriptions)
  {
    out.referenced_type_descriptions.push_back(convert_individual_type_description_runtime_to_msg(
          referenced_type_description, alloc));
  }
  return out;
}

/// Convert a type description message to its corresponding runtime struct.
/**
 * This function converts a type_description_interfaces/msg/TypeDescription msg
 * to the corresponding rosidl_runtime_cpp::type_description::TypeDescription
 * struct.
 *
 * \param[in] description_msg the message to convert
 * \return the converted runtime struct
 */
template<
  typename ToAllocatorT = std::allocator<void>,
  typename FromAllocatorT
>
rosidl_runtime_cpp::type_description::TypeDescription_<ToAllocatorT>
convert_type_description_msg_to_runtime(
  const type_description_interfaces::msg::TypeDescription_<FromAllocatorT> & description_msg,
  const ToAllocatorT & alloc = ToAllocatorT())
{
  rosidl_runtime_cpp::type_description::TypeDescription_<ToAllocatorT> out(alloc);
  out.type_description =
    convert_individual_type_description_msg_to_runtime(description_msg.type_description, alloc);

  for (const auto & referenced_type_description : description_msg.referenced_type_descriptions) {
    out.referenced_type_descriptions.push_back(convert_individual_type_description_msg_to_runtime(
          referenced_type_description, alloc));
  }
  return out;
}

// TypeSource ======================================================================================

/// Convert a runtime type source struct to its corresponding message.
/**
 * This function converts a rosidl_runtime_cpp::type_description::TypeSource
 * struct to the corresponding type_description_interfaces/msg/TypeSource msg.
 *
 * \param[in] runtime_type_source the runtime struct to convert
 * \return the converted message
 */
template<
  typename ToAllocatorT = std::allocator<void>,
  typename FromAllocatorT
>
type_description_interfaces::msg::TypeSource_<ToAllocatorT>
convert_type_source_sequence_runtime_to_msg(
  const rosidl_runtime_cpp::type_description::TypeSource_<FromAllocatorT> & runtime_type_source,
  const ToAllocatorT & alloc = ToAllocatorT())
{
  type_description_interfaces::msg::TypeSource_<ToAllocatorT> out(alloc);
  out.type_name = runtime_type_source.type_name;
  out.encoding = runtime_type_source.encoding;
  out.raw_file_contents = runtime_type_source.raw_file_contents;
  return out;
}

/// Convert a type source message to its corresponding runtime struct.
/**
 * This function converts a type_description_interfaces/msg/TypeSource msg to
 * the corresponding rosidl_runtime_cpp::type_description::TypeSource struct.
 *
 * \param[in] type_source_msg the message to convert
 * \return the converted runtime struct
 */
template<
  typename ToAllocatorT = std::allocator<void>,
  typename FromAllocatorT
>
rosidl_runtime_cpp::type_description::TypeSource_<ToAllocatorT>
convert_type_source_sequence_msg_to_runtime(
  const type_description_interfaces::msg::TypeSource_<FromAllocatorT> & type_source_msg,
  const ToAllocatorT & alloc = ToAllocatorT())
{
  rosidl_runtime_cpp::type_description::TypeSource_<ToAllocatorT> out(alloc);
  out.type_name = type_source_msg.type_name;
  out.encoding = type_source_msg.encoding;
  out.raw_file_contents = type_source_msg.raw_file_contents;
  return out;
}

}  // namespace dynamic_typesupport
}  // namespace rclcpp

#endif  // RCLCPP__DYNAMIC_TYPESUPPORT__TYPE_DESCRIPTION_CONVERSIONS_HPP_
