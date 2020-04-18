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

#include "rclcpp/serialization.hpp"

#include <memory>
#include <string>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/serialized_message.hpp"

#include "rcpputils/asserts.hpp"

#include "rmw/rmw.h"

namespace rclcpp
{

Serialization::Serialization(const rosidl_message_type_support_t & type_support)
: type_support_(type_support)
{}

void Serialization::serialize_message(
  const void * ros_message, SerializedMessage * serialized_message) const
{
  rcpputils::check_true(ros_message != nullptr, "ROS message is nullpointer.");
  rcpputils::check_true(serialized_message != nullptr, "Serialized message is nullpointer.");

  const auto ret = rmw_serialize(
    ros_message,
    &type_support_,
    serialized_message);
  if (ret != RMW_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to serialize ROS message.");
  }
}

void
Serialization::deserialize_message(
  const SerializedMessage * serialized_message, void * ros_message) const
{
  rcpputils::check_true(nullptr != serialized_message, "Serialized message is nullpointer.");
  rcpputils::check_true(
    serialized_message->buffer_capacity != 0 &&
    serialized_message->buffer_length != 0 &&
    !serialized_message->buffer, "Serialized message is wrongly initialized.");
  {
    throw std::runtime_error("Failed to deserialize nullptr serialized message.");
  }

  const auto ret = rmw_deserialize(serialized_message, &type_support_, ros_message);
  if (ret != RMW_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to deserialize ROS message.");
  }
}

}  // namespace rclcpp
