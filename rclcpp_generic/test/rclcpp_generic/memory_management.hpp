// Copyright 2018, Bosch Software Innovations GmbH.
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

#ifndef ROSBAG2_TEST_COMMON__MEMORY_MANAGEMENT_HPP_
#define ROSBAG2_TEST_COMMON__MEMORY_MANAGEMENT_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/error_handling.h"

namespace rosbag2_test_common
{
class MemoryManagement
{
public:
  MemoryManagement()
  {
    rcutils_allocator_ = rcutils_get_default_allocator();
  }

  ~MemoryManagement() = default;

  template<typename T>
  inline
  std::shared_ptr<rmw_serialized_message_t> serialize_message(std::shared_ptr<T> message)
  {
    auto serialized_message = get_initialized_serialized_message(0);
    auto error = rmw_serialize(
      message.get(),
      get_message_typesupport(message),
      serialized_message.get());
    if (error != RCL_RET_OK) {
      throw std::runtime_error("Failed to serialize");
    }
    return serialized_message;
  }

  template<typename T>
  inline
  std::shared_ptr<T> deserialize_message(std::shared_ptr<rmw_serialized_message_t> serialized_msg)
  {
    auto message = std::make_shared<T>();
    auto error = rmw_deserialize(
      serialized_msg.get(),
      get_message_typesupport(message),
      message.get());
    if (error != RCL_RET_OK) {
      RCUTILS_LOG_ERROR_NAMED(
        "rosbag2_test_common", "Leaking memory. Error: %s",
        rcutils_get_error_string().str);
    }
    return message;
  }

  std::shared_ptr<rmw_serialized_message_t> make_initialized_message()
  {
    return get_initialized_serialized_message(0);
  }

private:
  template<typename T>
  inline
  const rosidl_message_type_support_t * get_message_typesupport(std::shared_ptr<T>)
  {
    return rosidl_typesupport_cpp::get_message_type_support_handle<T>();
  }

  std::shared_ptr<rmw_serialized_message_t>
  get_initialized_serialized_message(size_t capacity)
  {
    auto msg = new rmw_serialized_message_t;
    *msg = rmw_get_zero_initialized_serialized_message();
    auto ret = rmw_serialized_message_init(msg, capacity, &rcutils_allocator_);
    if (ret != RCUTILS_RET_OK) {
      throw std::runtime_error(
              "Error allocating resources for serialized message: " +
              std::string(rcutils_get_error_string().str));
    }

    auto serialized_message = std::shared_ptr<rmw_serialized_message_t>(
      msg,
      [](rmw_serialized_message_t * msg) {
        int error = rmw_serialized_message_fini(msg);
        delete msg;
        if (error != RCUTILS_RET_OK) {
          RCUTILS_LOG_ERROR_NAMED(
            "rosbag2_test_common", "Leaking memory. Error: %s",
            rcutils_get_error_string().str);
        }
      });
    return serialized_message;
  }

  rcutils_allocator_t rcutils_allocator_;
};

}  // namespace rosbag2_test_common

#endif  // ROSBAG2_TEST_COMMON__MEMORY_MANAGEMENT_HPP_
