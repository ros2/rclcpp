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

/// Interface to (de)serialize a message
class RCLCPP_PUBLIC_TYPE SerializationBase
{
protected:
  explicit SerializationBase(const rosidl_message_type_support_t * type_support);

  virtual ~SerializationBase() = default;

  /// Serialize a ROS2 message to a serialized stream
  /**
   * \param[in] message The ROS2 message which is read and serialized by rmw.
   * \param[out] serialized_message The serialized message.
   */
  void serialize_message(
    const void * ros_message, SerializedMessage * serialized_message) const;

  /// Deserialize a serialized stream to a ROS message
  /**
   * \param[in] serialized_message The serialized message to be converted to ROS2 by rmw.
   * \param[out] message The deserialized ROS2 message.
   */
  void deserialize_message(
    const SerializedMessage * serialized_message, void * ros_message) const;

  const rosidl_message_type_support_t * type_support_;
};

/// Default implementation to (de)serialize a message by using rmw_(de)serialize
template<typename MessageT>
class Serialization : public SerializationBase
{
public:
  Serialization()
  : SerializationBase(rosidl_typesupport_cpp::get_message_type_support_handle<MessageT>())
  {}

  void serialize_message(
    const MessageT & ros_message, SerializedMessage & serialized_message) const
  {
    SerializationBase::serialize_message(
      reinterpret_cast<const void *>(&ros_message),
      &serialized_message);
  }

  void deserialize_message(
    const SerializedMessage & serialized_message, MessageT & ros_message) const
  {
    SerializationBase::deserialize_message(
      &serialized_message,
      reinterpret_cast<void *>(&ros_message));
  }
};

template<>
class Serialization<SerializedMessage>: public SerializationBase
{
public:
  Serialization()
  : SerializationBase(nullptr)
  {}

  void serialize_message(
    const SerializedMessage & ros_message,
    SerializedMessage & serialized_message) const
  {
    (void)ros_message;
    (void)serialized_message;
    throw std::runtime_error(
            "Serialization of serialized message to serialized message is not possible.");
  }

  void deserialize_message(
    const SerializedMessage & serialized_message,
    SerializedMessage & ros_message) const
  {
    (void)ros_message;
    (void)serialized_message;
    throw std::runtime_error(
            "Deserialization of serialized message to serialized message is not possible.");
  }
};

template<>
class Serialization<rcl_serialized_message_t>: public SerializationBase
{
public:
  Serialization()
  : SerializationBase(nullptr)
  {}

  void serialize_message(
    rcl_serialized_message_t & ros_message,
    const SerializedMessage & serialized_message) const
  {
    (void)ros_message;
    (void)serialized_message;
    throw std::runtime_error(
            "Serialization of serialized message to serialized message is not possible.");
  }

  void deserialize_message(
    const SerializedMessage & serialized_message,
    rcl_serialized_message_t & ros_message) const
  {
    (void)ros_message;
    (void)serialized_message;
    throw std::runtime_error(
            "Deserialization of serialized message to serialized message is not possible.");
  }
};

// trait to check if type is the object oriented serialized message
template<typename T>
struct is_serialized_message_class : std::false_type
{};

template<>
struct is_serialized_message_class<rcl_serialized_message_t>: std::false_type
{};

template<>
struct is_serialized_message_class<SerializedMessage>
  : std::true_type
{};

}  // namespace rclcpp

#endif  // RCLCPP__SERIALIZATION_HPP_
