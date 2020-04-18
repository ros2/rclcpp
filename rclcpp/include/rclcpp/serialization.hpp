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

#ifndef RCLCPP__SERIALIZATION_HPP_
#define RCLCPP__SERIALIZATION_HPP_

#include <memory>
#include <string>

#include "rosidl_runtime_c/message_type_support_struct.h"

namespace rclcpp
{

class SerializedMessage;

/// Interface to (de)serialize a message
class SerializationBase
{
public:
  virtual ~SerializationBase() = default;

  /// Serialize a ROS2 message to a serialized stream
  /**
   * \param[in] message The ROS2 message which is read and serialized by rmw.
   */
  virtual void serialize_message(
    const void * ros_message, SerializedMessage * serialized_message) const = 0;

  /// Deserialize a serialized stream to a ROS message
  /**
   * \param[in] serialized_message The serialized message to be converted to ROS2 by rmw.
   * \param[out] message The deserialized ROS2 message.
   */
  virtual void deserialize_message(
    const SerializedMessage * serialized_message, void * ros_message) const = 0;
};

/// Default implementation to (de)serialize a message by using rmw_(de)serialize
class Serialization : public SerializationBase
{
public:
  Serialization(const rosidl_message_type_support_t & type_support);

  void serialize_message(
    const void * ros_message, SerializedMessage * serialized_message) const override;

  void deserialize_message(
    const SerializedMessage * serialized_message, void * ros_message) const override;

private:
  rosidl_message_type_support_t type_support_;
};

}  // namespace rclcpp

#endif  // RCLCPP__SERIALIZATION_HPP_
