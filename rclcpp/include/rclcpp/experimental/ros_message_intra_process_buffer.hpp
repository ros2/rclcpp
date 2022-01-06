// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXPERIMENTAL__ROS_MESSAGE_INTRA_PROCESS_BUFFER_HPP_
#define RCLCPP__EXPERIMENTAL__ROS_MESSAGE_INTRA_PROCESS_BUFFER_HPP_

#include <memory>
#include <string>

#include "rcl/error_handling.h"

#include "rclcpp/any_subscription_callback.hpp"
#include "rclcpp/context.hpp"
#include "rclcpp/experimental/subscription_intra_process_base.hpp"
#include "tracetools/tracetools.h"

namespace rclcpp
{
namespace experimental
{

template<
  typename RosMessageT,
  typename Alloc = std::allocator<void>,
  typename Deleter = std::default_delete<void>
>
class SubscriptionROSMsgIntraProcessBuffer : public SubscriptionIntraProcessBase
{
public:
  using ROSMessageTypeAllocatorTraits = allocator::AllocRebind<RosMessageT, Alloc>;
  using ROSMessageTypeAllocator = typename ROSMessageTypeAllocatorTraits::allocator_type;
  using ROSMessageTypeDeleter = allocator::Deleter<ROSMessageTypeAllocator, RosMessageT>;

  using ConstMessageSharedPtr = std::shared_ptr<const RosMessageT>;
  using MessageUniquePtr = std::unique_ptr<RosMessageT, ROSMessageTypeDeleter>;

  SubscriptionROSMsgIntraProcessBuffer(
    rclcpp::Context::SharedPtr context,
    const std::string & topic_name,
    const rclcpp::QoS & qos_profile)
  : SubscriptionIntraProcessBase(context, topic_name, qos_profile)
  {}

  virtual ~SubscriptionROSMsgIntraProcessBuffer()
  {}

  virtual void
  provide_intra_process_message(ConstMessageSharedPtr message) = 0;

  virtual void
  provide_intra_process_message(MessageUniquePtr message) = 0;
};

}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__ROS_MESSAGE_INTRA_PROCESS_BUFFER_HPP_
