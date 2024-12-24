// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP_LIFECYCLE__LIFECYCLE_SUBSCRIPTION_HPP_
#define RCLCPP_LIFECYCLE__LIFECYCLE_SUBSCRIPTION_HPP_

#include <memory>
#include <string>

#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/qos.hpp"

#include "rclcpp_lifecycle/managed_entity.hpp"


namespace rclcpp_lifecycle
{

/// @brief Child class of rclcpp::Subscription that adds lifecycle functionality.
/**
 * This class is a child class of rclcpp::Subscription that adds a lifecycle
 * check to the callback function. If the node is in an inactive state, the
 * callback will not be called.
 */
template<
  typename MessageT,
  typename AllocatorT = std::allocator<void>,
  typename SubscribedT = typename rclcpp::TypeAdapter<MessageT>::custom_type,
  typename ROSMessageT = typename rclcpp::TypeAdapter<MessageT>::ros_message_type,
  typename MessageMemoryStrategyT = rclcpp::message_memory_strategy::MessageMemoryStrategy<
    ROSMessageT,
    AllocatorT
  >>
class LifecycleSubscription : public SimpleManagedEntity,
  public rclcpp::Subscription<MessageT, AllocatorT>
{
private:
  using SubscriptionTopicStatisticsSharedPtr =
    std::shared_ptr<rclcpp::topic_statistics::SubscriptionTopicStatistics>;

public:
  LifecycleSubscription(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    const rosidl_message_type_support_t & type_support_handle,
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    rclcpp::AnySubscriptionCallback<MessageT, AllocatorT> callback,
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options,
    typename MessageMemoryStrategyT::SharedPtr message_memory_strategy,
    SubscriptionTopicStatisticsSharedPtr subscription_topic_statistics = nullptr)
  :  rclcpp::Subscription<MessageT>(
      node_base,
      type_support_handle,
      topic_name,
      qos,
      callback,
      options,
      message_memory_strategy,
      subscription_topic_statistics)
  {
  }

  /// TODO: Hold onto the data that arrives before activation, and deliver that on activation.
  /// Check if we need to handle the message, and execute the callback if we do.
  void handle_message(
    std::shared_ptr<void> & message, const rclcpp::MessageInfo & message_info) override
  {
    if (!this->is_activated()) {
      return;
    }
    rclcpp::Subscription<MessageT, AllocatorT>::handle_message(message, message_info);
  }
};

}  // namespace rclcpp_lifecycle

#endif  // RCLCPP_LIFECYCLE__LIFECYCLE_SUBSCRIPTION_HPP_
