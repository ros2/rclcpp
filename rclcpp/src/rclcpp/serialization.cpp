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

SerializationBase::SerializationBase(const rosidl_message_type_support_t * type_support)
: type_support_(type_support)
{
  rcpputils::check_true(nullptr != type_support, "Typesupport is nullpointer.");
}

void SerializationBase::serialize_message(
  const void * ros_message, SerializedMessage * serialized_message) const
{
  rcpputils::check_true(nullptr != type_support_, "Typesupport is nullpointer.");
  rcpputils::check_true(nullptr != ros_message, "ROS message is nullpointer.");
  rcpputils::check_true(nullptr != serialized_message, "Serialized message is nullpointer.");

  const auto ret = rmw_serialize(
    ros_message,
    type_support_,
    &serialized_message->get_rcl_serialized_message());
  if (ret != RMW_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to serialize ROS message.");
  }
}

void SerializationBase::deserialize_message(
  const SerializedMessage * serialized_message, void * ros_message) const
{
  rcpputils::check_true(nullptr != type_support_, "Typesupport is nullpointer.");
  rcpputils::check_true(nullptr != serialized_message, "Serialized message is nullpointer.");
  rcpputils::check_true(
    0u != serialized_message->capacity(),
    "Wrongly initialized. Serialized message has a capacity of zero.");
  rcpputils::check_true(
    0u != serialized_message->size(),
    "Wrongly initialized. Serialized message has a size of zero.");
  rcpputils::check_true(nullptr != ros_message, "ROS message is a nullpointer.");

  const auto ret = rmw_deserialize(
    &serialized_message->get_rcl_serialized_message(), type_support_, ros_message);
  if (ret != RMW_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to deserialize ROS message.");
  }
}

}  // namespace rclcpp
