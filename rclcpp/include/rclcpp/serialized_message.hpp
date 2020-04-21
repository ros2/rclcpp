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

#ifndef RCLCPP__SERIALIZED_MESSAGE_HPP_
#define RCLCPP__SERIALIZED_MESSAGE_HPP_

#include <rclcpp/exceptions.hpp>

#include <cstring>

#include "rcutils/logging_macros.h"

#include "rmw/serialized_message.h"

namespace rclcpp
{

/// Object oriented version of rcl_serialized_message_t with destructor to avoid memory leaks
class SerializedMessage : public rcl_serialized_message_t
{
public:
  SerializedMessage()
  : rcl_serialized_message_t(rmw_get_zero_initialized_serialized_message())
  {}

  explicit SerializedMessage(const SerializedMessage & serialized_message)
  : SerializedMessage(static_cast<const rcl_serialized_message_t>(serialized_message))
  {}

  explicit SerializedMessage(const rcl_serialized_message_t & serialized_message)
  : rcl_serialized_message_t(rmw_get_zero_initialized_serialized_message())
  {
    const auto ret = rmw_serialized_message_init(
      this, serialized_message.buffer_length,
      &serialized_message.allocator);
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(ret);
    }

    // do not call memcpy if the pointer is "static"
    if (buffer != serialized_message.buffer) {
      std::memcpy(buffer, serialized_message.buffer, serialized_message.buffer_length);
    }
    buffer_length = serialized_message.buffer_length;
  }

  explicit SerializedMessage(rcl_serialized_message_t && msg)
  : rcl_serialized_message_t(msg)
  {
    // reset buffer to prevent double free
    msg = rmw_get_zero_initialized_serialized_message();
  }

  ~SerializedMessage()
  {
    if (nullptr != buffer) {
      const auto fini_ret = rmw_serialized_message_fini(this);
      if (fini_ret != RCL_RET_OK) {
        RCUTILS_LOG_ERROR_NAMED(
          "rclcpp",
          "Failed to destroy serialized message: %s", rcl_get_error_string().str);
      }
    }
  }
};

}  // namespace rclcpp

#endif  // RCLCPP__SERIALIZED_MESSAGE_HPP_
