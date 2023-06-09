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
#include <stdexcept>
#include <string>

#include "rclcpp/visibility_control.hpp"

#include "rcl/types.h"

#include "rosidl_runtime_c/message_type_support_struct.h"

#include "rosidl_typesupport_cpp/message_type_support.hpp"

namespace rclcpp
{

class SerializedMessage;

namespace serialization_traits
{
// trait to check if type is the object oriented serialized message
template<typename T>
struct is_serialized_message_class : std::false_type
{};

template<>
struct is_serialized_message_class<SerializedMessage>: std::true_type
{};
}  // namespace serialization_traits

/// Interface to (de)serialize a message
class RCLCPP_PUBLIC_TYPE SerializationBase
{
public:
  /// Constructor of SerializationBase
  /**
   * \param[in] type_support handle for the message type support
   * to be used for serialization and deserialization.
   */
  explicit SerializationBase(const rosidl_message_type_support_t * type_support);

  /// Destructor of SerializationBase
  virtual ~SerializationBase() = default;

  /// Serialize a ROS2 message to a serialized stream
  /**
   * \param[in] ros_message The ROS2 message which is read and serialized by rmw.
   * \param[out] serialized_message The serialized message.
   */
  void serialize_message(
    const void * ros_message, SerializedMessage * serialized_message) const;

  /// Deserialize a serialized stream to a ROS message
  /**
   * \param[in] serialized_message The serialized message to be converted to ROS2 by rmw.
   * \param[out] ros_message The deserialized ROS2 message.
   */
  void deserialize_message(
    const SerializedMessage * serialized_message, void * ros_message) const;

private:
  const rosidl_message_type_support_t * type_support_;
};

/// Default implementation to (de)serialize a message by using rmw_(de)serialize
template<typename MessageT>
class Serialization : public SerializationBase
{
public:
  /// Constructor of Serialization
  Serialization()
  : SerializationBase(rosidl_typesupport_cpp::get_message_type_support_handle<MessageT>())
  {
    static_assert(
      !serialization_traits::is_serialized_message_class<MessageT>::value,
      "Serialization of serialized message to serialized message is not possible.");
  }
};

}  // namespace rclcpp

#endif  // RCLCPP__SERIALIZATION_HPP_
