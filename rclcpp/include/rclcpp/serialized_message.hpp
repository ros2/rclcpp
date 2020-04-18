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

#include "rcl/allocator.h"
#include "rcl/types.h"

namespace rclcpp
{

/// Object oriented version of rcl_serialized_message_t with destructor to avoid memory leaks
class SerializedMessage : public rcl_serialized_message_t
{
public:
  /// Default constructor for a SerializedMessage
  /**
   * Default constructs a serialized message and initalizes its
   * capacity with 0.
   *
   * \param[in] allocator The allocator to be used for the initialzation.
   */
  explicit SerializedMessage(
    const rcl_allocator_t & allocator = rcl_get_default_allocator());

  /// Default constructor for a SerializedMessage
  /**
   * Default constructs a serialized message and initalizes its
   * capacity with 0.
   *
   * \param[in] initial_capacity The amount of memory to be allocated.
   * \param[in] allocator The allocator to be used for the initialzation.
   */
  explicit SerializedMessage(
    size_t initial_capacity,
    const rcl_allocator_t & allocator = rcl_get_default_allocator());

  /// Copy Constructor for a SerializedMessage
  explicit SerializedMessage(const SerializedMessage & serialized_message);

  /// Copy Constructor for a SerializedMessage from a rcl_serialized_message_t
  explicit SerializedMessage(const rcl_serialized_message_t & serialized_message);

  /// Move Constructor for a SerializedMessage
  explicit SerializedMessage(SerializedMessage && serialized_message);

  /// Move Constructor for a SerializedMessage from a rcl_serialized_message_t
  explicit SerializedMessage(rcl_serialized_message_t && serialized_message);

  /// Destructor for a SerializedMessage
  ~SerializedMessage();
};

}  // namespace rclcpp

#endif  // RCLCPP__SERIALIZED_MESSAGE_HPP_
