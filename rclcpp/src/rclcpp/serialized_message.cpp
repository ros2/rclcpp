// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/serialized_message.hpp"

#include <cstring>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"

#include "rmw/types.h"

namespace rclcpp
{

inline void copy_rcl_message(const rcl_serialized_message_t & from, rcl_serialized_message_t & to)
{
  const auto ret = rmw_serialized_message_init(
    &to, from.buffer_capacity, &from.allocator);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  // do not call memcpy if the pointer is "static"
  if (to.buffer != from.buffer) {
    std::memcpy(to.buffer, from.buffer, from.buffer_length);
  }
  to.buffer_length = from.buffer_length;
}

/// Object oriented version of rcl_serialized_message_t with destructor to avoid memory leaks
SerializedMessage::SerializedMessage(const rcl_allocator_t & allocator)
: SerializedMessage(0u, allocator)
{}

SerializedMessage::SerializedMessage(
  size_t initial_capacity, const rcl_allocator_t & allocator)
: serialized_message_(rmw_get_zero_initialized_serialized_message())
{
  const auto ret = rmw_serialized_message_init(
    &serialized_message_, initial_capacity, &allocator);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

SerializedMessage::SerializedMessage(const SerializedMessage & other)
: SerializedMessage(other.serialized_message_)
{}

SerializedMessage::SerializedMessage(const rcl_serialized_message_t & other)
: serialized_message_(rmw_get_zero_initialized_serialized_message())
{
  copy_rcl_message(other, serialized_message_);
}

SerializedMessage::SerializedMessage(SerializedMessage && other)
: serialized_message_(
    std::exchange(other.serialized_message_, rmw_get_zero_initialized_serialized_message()))
{}

SerializedMessage::SerializedMessage(rcl_serialized_message_t && other)
: serialized_message_(
    std::exchange(other, rmw_get_zero_initialized_serialized_message()))
{}

SerializedMessage & SerializedMessage::operator=(const SerializedMessage & other)
{
  if (this != &other) {
    serialized_message_ = rmw_get_zero_initialized_serialized_message();
    copy_rcl_message(other.serialized_message_, serialized_message_);
  }

  return *this;
}

SerializedMessage & SerializedMessage::operator=(const rcl_serialized_message_t & other)
{
  if (&serialized_message_ != &other) {
    serialized_message_ = rmw_get_zero_initialized_serialized_message();
    copy_rcl_message(other, serialized_message_);
  }

  return *this;
}

SerializedMessage & SerializedMessage::operator=(SerializedMessage && other)
{
  if (this != &other) {
    serialized_message_ =
      std::exchange(other.serialized_message_, rmw_get_zero_initialized_serialized_message());
  }

  return *this;
}

SerializedMessage & SerializedMessage::operator=(rcl_serialized_message_t && other)
{
  if (&serialized_message_ != &other) {
    serialized_message_ =
      std::exchange(other, rmw_get_zero_initialized_serialized_message());
  }
  return *this;
}

SerializedMessage::~SerializedMessage()
{
  if (nullptr != serialized_message_.buffer) {
    const auto fini_ret = rmw_serialized_message_fini(&serialized_message_);
    if (RCL_RET_OK != fini_ret) {
      RCLCPP_ERROR(
        get_logger("rclcpp"),
        "Failed to destroy serialized message: %s", rcl_get_error_string().str);
    }
  }
}

rcl_serialized_message_t & SerializedMessage::get_rcl_serialized_message()
{
  return serialized_message_;
}

const rcl_serialized_message_t & SerializedMessage::get_rcl_serialized_message() const
{
  return serialized_message_;
}

size_t SerializedMessage::size() const
{
  return serialized_message_.buffer_length;
}

size_t SerializedMessage::capacity() const
{
  return serialized_message_.buffer_capacity;
}

void SerializedMessage::reserve(size_t capacity)
{
  auto ret = rmw_serialized_message_resize(&serialized_message_, capacity);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

rcl_serialized_message_t SerializedMessage::release_rcl_serialized_message()
{
  auto ret = serialized_message_;
  serialized_message_ = rmw_get_zero_initialized_serialized_message();

  return ret;
}
}  // namespace rclcpp
