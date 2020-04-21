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

#ifndef RCLCPP__GENERIC_PUBLISHER_HPP_
#define RCLCPP__GENERIC_PUBLISHER_HPP_

#include "rcl/error_handling.h"
#include "rcl/publisher.h"

#include "rclcpp/exceptions.hpp"
#include "rclcpp/serialized_message.hpp"

namespace rclcpp
{

namespace generic_publisher
{

template<typename MessageT>
inline void
do_inter_process_publish(rcl_publisher_t & publisher_handle, const MessageT & msg)
{
  auto status = rcl_publish(&publisher_handle, &msg, nullptr);

  if (RCL_RET_PUBLISHER_INVALID == status) {
    rcl_reset_error();  // next call will reset error message if not context
    if (rcl_publisher_is_valid_except_context(&publisher_handle)) {
      rcl_context_t * context = rcl_publisher_get_context(&publisher_handle);
      if (nullptr != context && !rcl_context_is_valid(context)) {
        // publisher is invalid due to context being shutdown
        return;
      }
    }
  }
  if (RCL_RET_OK != status) {
    rclcpp::exceptions::throw_from_rcl_error(status, "failed to publish message");
  }
}

template<>
inline void
do_inter_process_publish<SerializedMessage>(
  rcl_publisher_t & publisher_handle,
  const SerializedMessage & serialized_msg)
{
  auto status = rcl_publish_serialized_message(&publisher_handle, &serialized_msg, nullptr);
  if (RCL_RET_OK != status) {
    rclcpp::exceptions::throw_from_rcl_error(status, "failed to publish serialized message");
  }
}

}  // namespace generic_publisher

}  // namespace rclcpp

#endif  // RCLCPP__GENERIC_PUBLISHER_HPP_
