// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXPERIMENTAL__SERIALIZATION_HPP_
#define RCLCPP__EXPERIMENTAL__SERIALIZATION_HPP_

#include "rmw/rmw.h"

#include <memory>
#include <string>

#include "rcl/error_handling.h"

namespace rclcpp
{
namespace experimental
{

/// Interface to (de)serialize a message
class SerializationBase
{
public:
  virtual ~SerializationBase() {}

  virtual std::shared_ptr<rcl_serialized_message_t> serialize_message(const void * message) = 0;

  virtual void deserialize_message(
    const rcl_serialized_message_t & serialized_message,
    void * msg) = 0;
};

/// Default implementation to (de)serialize a message by using rmw_(de)serialize
class Serialization : public SerializationBase
{
public:
  Serialization(
    const rosidl_message_type_support_t & type_support,
    const rcutils_allocator_t allocator = rcutils_get_default_allocator())
  : type_support_(type_support), rcutils_allocator_(allocator)
  {}

  std::shared_ptr<rcl_serialized_message_t> serialize_message(const void * message) override
  {
    auto serialized_message = new rcl_serialized_message_t;
    *serialized_message = rmw_get_zero_initialized_serialized_message();
    const auto ret = rmw_serialized_message_init(serialized_message, 0, &rcutils_allocator_);
    if (ret != RCUTILS_RET_OK) {
      throw std::runtime_error(
              "Error allocating resources for serialized message: " +
              std::string(rcutils_get_error_string().str));
    }

    if (message) {
      const auto error = rmw_serialize(
        message,
        &type_support_,
        serialized_message);
      if (error != RCL_RET_OK) {
        throw std::runtime_error("Failed to serialize.");
      }
    }

    auto shared_serialized_msg = std::shared_ptr<rcl_serialized_message_t>(
      serialized_message,
      [](rcl_serialized_message_t * msg) {
        auto fini_ret = rmw_serialized_message_fini(msg);
        delete msg;
        if (fini_ret != RCL_RET_OK) {
          RCUTILS_LOG_ERROR_NAMED(
            "rclcpp",
            "failed to destroy serialized message: %s", rcl_get_error_string().str);
        }
      });

    return shared_serialized_msg;
  }

  void deserialize_message(const rcl_serialized_message_t & serialized_message, void * msg) override
  {
    if (serialized_message.buffer_capacity == 0 ||
      serialized_message.buffer_length == 0 ||
      !serialized_message.buffer)
    {
      throw std::runtime_error("Failed to deserialize nullptr serialized message.");
    }

    const auto ret = rmw_deserialize(&serialized_message, &type_support_, msg);
    if (ret != RMW_RET_OK) {
      throw std::runtime_error("Failed to deserialize serialized message.");
    }
  }

private:
  rosidl_message_type_support_t type_support_;
  rcutils_allocator_t rcutils_allocator_;
};

}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__SERIALIZATION_HPP_
