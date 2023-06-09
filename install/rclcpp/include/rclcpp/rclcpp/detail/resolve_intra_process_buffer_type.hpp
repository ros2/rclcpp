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

#ifndef RCLCPP__DETAIL__RESOLVE_INTRA_PROCESS_BUFFER_TYPE_HPP_
#define RCLCPP__DETAIL__RESOLVE_INTRA_PROCESS_BUFFER_TYPE_HPP_

#include <stdexcept>

#include "rclcpp/any_subscription_callback.hpp"
#include "rclcpp/intra_process_buffer_type.hpp"

namespace rclcpp
{

namespace detail
{

/// Return the buffer type, resolving the "CallbackDefault" type to an actual type if needed.
template<typename CallbackMessageT, typename AllocatorT>
rclcpp::IntraProcessBufferType
resolve_intra_process_buffer_type(
  const rclcpp::IntraProcessBufferType buffer_type,
  const rclcpp::AnySubscriptionCallback<CallbackMessageT, AllocatorT> & any_subscription_callback)
{
  rclcpp::IntraProcessBufferType resolved_buffer_type = buffer_type;

  // If the user has not specified a type for the intra-process buffer, use the callback's type.
  if (resolved_buffer_type == IntraProcessBufferType::CallbackDefault) {
    if (any_subscription_callback.use_take_shared_method()) {
      resolved_buffer_type = IntraProcessBufferType::SharedPtr;
    } else {
      resolved_buffer_type = IntraProcessBufferType::UniquePtr;
    }
  }

  return resolved_buffer_type;
}

}  // namespace detail

}  // namespace rclcpp

#endif  // RCLCPP__DETAIL__RESOLVE_INTRA_PROCESS_BUFFER_TYPE_HPP_
