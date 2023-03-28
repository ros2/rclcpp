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

#ifndef RCLCPP__DYNAMIC_TYPESUPPORT__DETAIL__DYNAMIC_DATA_IMPL_HPP_
#define RCLCPP__DYNAMIC_TYPESUPPORT__DETAIL__DYNAMIC_DATA_IMPL_HPP_

#include <cstdint>
#include <cstddef>
#include <memory>
#include <string>

#include <rosidl_dynamic_typesupport/types.h>
#include <rosidl_dynamic_typesupport/api/dynamic_data.h>
#include "rclcpp/exceptions.hpp"

#ifndef RCLCPP__DYNAMIC_TYPESUPPORT__DYNAMIC_DATA_HPP_
#include "rclcpp/dynamic_typesupport/dynamic_data.hpp"
#endif


#define __DYNAMIC_DATA_GET_VALUE_BY_ID_FN(ValueT, FunctionT) \
  template<> \
  ValueT \
  DynamicData::get_value<ValueT>(rosidl_dynamic_typesupport_member_id_t id) \
  { \
    ValueT out; \
    rosidl_dynamic_typesupport_dynamic_data_get_ ## FunctionT ## _value( \
      rosidl_dynamic_data_.get(), id, &out); \
    return out; \
  }

#define __DYNAMIC_DATA_GET_VALUE_BY_NAME_FN(ValueT, FunctionT) \
  template<> \
  ValueT \
  DynamicData::get_value<ValueT>(const std::string & name) \
  { \
    return get_value<ValueT>(get_member_id(name)); \
  }

#define __DYNAMIC_DATA_SET_VALUE_BY_ID_FN(ValueT, FunctionT) \
  template<> \
  void \
  DynamicData::set_value<ValueT>(rosidl_dynamic_typesupport_member_id_t id, ValueT value) \
  { \
    rosidl_dynamic_typesupport_dynamic_data_set_ ## FunctionT ## _value( \
      rosidl_dynamic_data_.get(), id, value); \
  }

#define __DYNAMIC_DATA_SET_VALUE_BY_NAME_FN(ValueT, FunctionT) \
  template<> \
  void \
  DynamicData::set_value<ValueT>(const std::string & name, ValueT value) \
  { \
    set_value<ValueT>(get_member_id(name), value); \
  }

#define __DYNAMIC_DATA_INSERT_VALUE(ValueT, FunctionT) \
  template<> \
  rosidl_dynamic_typesupport_member_id_t \
  DynamicData::insert_value<ValueT>(ValueT value) \
  { \
    rosidl_dynamic_typesupport_member_id_t out; \
    rosidl_dynamic_typesupport_dynamic_data_insert_ ## FunctionT ## _value( \
      rosidl_dynamic_data_.get(), value, &out); \
    return out; \
  }

#define DYNAMIC_DATA_DEFINITIONS(ValueT, FunctionT) \
  __DYNAMIC_DATA_GET_VALUE_BY_ID_FN(ValueT, FunctionT) \
  __DYNAMIC_DATA_GET_VALUE_BY_NAME_FN(ValueT, FunctionT) \
  __DYNAMIC_DATA_SET_VALUE_BY_ID_FN(ValueT, FunctionT) \
  __DYNAMIC_DATA_SET_VALUE_BY_NAME_FN(ValueT, FunctionT) \
  __DYNAMIC_DATA_INSERT_VALUE(ValueT, FunctionT)


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

DYNAMIC_DATA_DEFINITIONS(bool, bool);
// DYNAMIC_DATA_DEFINITIONS(std::byte, byte);
DYNAMIC_DATA_DEFINITIONS(char, char);
DYNAMIC_DATA_DEFINITIONS(float, float32);
DYNAMIC_DATA_DEFINITIONS(double, float64);
DYNAMIC_DATA_DEFINITIONS(int8_t, int8);
DYNAMIC_DATA_DEFINITIONS(int16_t, int16);
DYNAMIC_DATA_DEFINITIONS(int32_t, int32);
DYNAMIC_DATA_DEFINITIONS(int64_t, int64);
DYNAMIC_DATA_DEFINITIONS(uint8_t, uint8);
DYNAMIC_DATA_DEFINITIONS(uint16_t, uint16);
DYNAMIC_DATA_DEFINITIONS(uint32_t, uint32);
DYNAMIC_DATA_DEFINITIONS(uint64_t, uint64);
// DYNAMIC_DATA_DEFINITIONS(std::string, std::string);
// DYNAMIC_DATA_DEFINITIONS(std::u16string, std::u16string);

// Byte and String getters have a different implementation and are defined below


// BYTE ============================================================================================
template<>
std::byte
DynamicData::get_value<std::byte>(rosidl_dynamic_typesupport_member_id_t id)
{
  unsigned char out;
  rosidl_dynamic_typesupport_dynamic_data_get_byte_value(get_rosidl_dynamic_data(), id, &out);
  return static_cast<std::byte>(out);
}


template<>
std::byte
DynamicData::get_value<std::byte>(const std::string & name)
{
  return get_value<std::byte>(get_member_id(name));
}


template<>
void
DynamicData::set_value<std::byte>(
  rosidl_dynamic_typesupport_member_id_t id, const std::byte value)
{
  rosidl_dynamic_typesupport_dynamic_data_set_byte_value(
    rosidl_dynamic_data_.get(), id, static_cast<unsigned char>(value));
}


template<>
void
DynamicData::set_value<std::byte>(const std::string & name, const std::byte value)
{
  set_value<std::byte>(get_member_id(name), value);
}


template<>
rosidl_dynamic_typesupport_member_id_t
DynamicData::insert_value<std::byte>(const std::byte value)
{
  rosidl_dynamic_typesupport_member_id_t out;
  rosidl_dynamic_typesupport_dynamic_data_insert_byte_value(
    rosidl_dynamic_data_.get(), static_cast<unsigned char>(value), &out);
  return out;
}


// STRINGS =========================================================================================
template<>
std::string
DynamicData::get_value<std::string>(rosidl_dynamic_typesupport_member_id_t id)
{
  size_t buf_length;
  char * buf = nullptr;
  rosidl_dynamic_typesupport_dynamic_data_get_string_value(
    get_rosidl_dynamic_data(), id, &buf, &buf_length);
  auto out = std::string(buf, buf_length);
  delete buf;
  return out;
}


template<>
std::u16string
DynamicData::get_value<std::u16string>(rosidl_dynamic_typesupport_member_id_t id)
{
  size_t buf_length;
  char16_t * buf = nullptr;
  rosidl_dynamic_typesupport_dynamic_data_get_wstring_value(
    get_rosidl_dynamic_data(), id, &buf, &buf_length);
  auto out = std::u16string(buf, buf_length);
  delete buf;
  return out;
}


template<>
std::string
DynamicData::get_value<std::string>(const std::string & name)
{
  return get_value<std::string>(get_member_id(name));
}


template<>
std::u16string
DynamicData::get_value<std::u16string>(const std::string & name)
{
  return get_value<std::u16string>(get_member_id(name));
}


template<>
void
DynamicData::set_value<std::string>(
  rosidl_dynamic_typesupport_member_id_t id, const std::string value)
{
  rosidl_dynamic_typesupport_dynamic_data_set_string_value(
    rosidl_dynamic_data_.get(), id, value.c_str(), value.size());
}


template<>
void
DynamicData::set_value<std::u16string>(
  rosidl_dynamic_typesupport_member_id_t id, const std::u16string value)
{
  rosidl_dynamic_typesupport_dynamic_data_set_wstring_value(
    rosidl_dynamic_data_.get(), id, value.c_str(), value.size());
}


template<>
void
DynamicData::set_value<std::string>(const std::string & name, const std::string value)
{
  set_value<std::string>(get_member_id(name), value);
}


template<>
void
DynamicData::set_value<std::u16string>(const std::string & name, const std::u16string value)
{
  set_value<std::u16string>(get_member_id(name), value);
}


template<>
rosidl_dynamic_typesupport_member_id_t
DynamicData::insert_value<std::string>(const std::string value)
{
  rosidl_dynamic_typesupport_member_id_t out;
  rosidl_dynamic_typesupport_dynamic_data_insert_string_value(
    rosidl_dynamic_data_.get(), value.c_str(), value.size(), &out);
  return out;
}


template<>
rosidl_dynamic_typesupport_member_id_t
DynamicData::insert_value<std::u16string>(const std::u16string value)
{
  rosidl_dynamic_typesupport_member_id_t out;
  rosidl_dynamic_typesupport_dynamic_data_insert_wstring_value(
    rosidl_dynamic_data_.get(), value.c_str(), value.size(), &out);
  return out;
}


// THROW FOR UNSUPPORTED TYPES =====================================================================
template<typename ValueT>
ValueT
DynamicData::get_value(rosidl_dynamic_typesupport_member_id_t id)
{
  throw rclcpp::exceptions::UnimplementedError("get_value is not implemented for input type");
}


template<typename ValueT>
ValueT
DynamicData::get_value(const std::string & name)
{
  throw rclcpp::exceptions::UnimplementedError("get_value is not implemented for input type");
}


template<typename ValueT>
void
DynamicData::set_value(
  rosidl_dynamic_typesupport_member_id_t id, ValueT value)
{
  throw rclcpp::exceptions::UnimplementedError("set_value is not implemented for input type");
}


template<typename ValueT>
void
DynamicData::set_value(const std::string & name, ValueT value)
{
  throw rclcpp::exceptions::UnimplementedError("set_value is not implemented for input type");
}


template<typename ValueT>
rosidl_dynamic_typesupport_member_id_t
DynamicData::insert_value(ValueT value)
{
  throw rclcpp::exceptions::UnimplementedError("insert_value is not implemented for input type");
}


}  // namespace dynamic_typesupport
}  // namespace rclcpp

#undef __DYNAMIC_DATA_GET_VALUE_BY_ID_FN
#undef __DYNAMIC_DATA_GET_VALUE_BY_NAME_FN
#undef __DYNAMIC_DATA_SET_VALUE_BY_ID_FN
#undef __DYNAMIC_DATA_SET_VALUE_BY_NAME_FN
#undef __DYNAMIC_DATA_INSERT_VALUE
#undef DYNAMIC_DATA_DEFINITIONS

#endif  // RCLCPP__DYNAMIC_TYPESUPPORT__DETAIL__DYNAMIC_DATA_IMPL_HPP_
