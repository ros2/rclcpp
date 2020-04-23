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

#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/// Object oriented version of rcl_serialized_message_t with destructor to avoid memory leaks
class RCLCPP_PUBLIC_TYPE SerializedMessage
{
public:
  /// Default constructor for a SerializedMessage
  /**
   * Default constructs a serialized message and initalizes it
   * with initial capacity of 0.
   * The allocator defaults to `rcl_get_default_allocator()`.
   *
   * \param[in] allocator The allocator to be used for the initialization.
   */
  explicit SerializedMessage(
    const rcl_allocator_t & allocator = rcl_get_default_allocator());

  /// Default constructor for a SerializedMessage
  /**
   * Default constructs a serialized message and initalizes it
   * with the provided capacity.
   * The allocator defaults to `rcl_get_default_allocator()`.
   *
   * \param[in] initial_capacity The amount of memory to be allocated.
   * \param[in] allocator The allocator to be used for the initialization.
   */
  SerializedMessage(
    size_t initial_capacity,
    const rcl_allocator_t & allocator = rcl_get_default_allocator());

  /// Copy Constructor for a SerializedMessage
  SerializedMessage(const SerializedMessage & other);

  /// Constructor for a SerializedMessage from a rcl_serialized_message_t
  explicit SerializedMessage(const rcl_serialized_message_t & other);

  /// Move Constructor for a SerializedMessage
  SerializedMessage(SerializedMessage && other);

  /// Constructor for a SerializedMessage from a moved rcl_serialized_message_t
  explicit SerializedMessage(rcl_serialized_message_t && other);

  /// Copy assignment operator
  SerializedMessage & operator=(const SerializedMessage & other);

  /// Copy assignment operator from a rcl_serialized_message_t
  SerializedMessage & operator=(const rcl_serialized_message_t & other);

  /// Move assignment operator
  SerializedMessage & operator=(SerializedMessage && other);

  /// Move assignment operator from a rcl_serialized_message_t
  SerializedMessage & operator=(rcl_serialized_message_t && other);

  /// Destructor for a SerializedMessage
  virtual ~SerializedMessage();

  /// Get the underlying rcl_serialized_t handle
  rcl_serialized_message_t & get_rcl_serialized_message();

  // Get a const handle to the underlying rcl_serialized_message_t
  const rcl_serialized_message_t & get_rcl_serialized_message() const;

  /// Get the size of the serialized data buffer
  /**
   * Note, this is different from the actual amount of allocated memory.
   * This can be obtained via a call to `capacity`.
   */
  size_t size() const;

  /// Get the size of allocated memory for the data buffer
  /**
   * Note, this is different from the amount of content in the buffer.
   * This can be obtained via a call to `size`.
   */
  size_t capacity() const;

  /// Allocate memory in the data buffer
  /**
   * The data buffer of the underlying rcl_serialized_message_t will be resized.
   * This might change the data layout and invalidates all pointers to the data.
   */
  void reserve(size_t capacity);

  /// Release the underlying rcl_serialized_message_t
  /**
   * The memory (i.e. the data buffer) of the serialized message will no longer
   * be managed by this instance and the memory won't be deallocated on destruction.
   */
  rcl_serialized_message_t release_rcl_serialized_message();

private:
  rcl_serialized_message_t serialized_message_;
};

}  // namespace rclcpp

#endif  // RCLCPP__SERIALIZED_MESSAGE_HPP_
