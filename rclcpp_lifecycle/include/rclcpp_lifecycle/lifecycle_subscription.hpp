// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#include <functional>
#include <memory>

#include "rclcpp/subscription.hpp"

namespace rclcpp_lifecycle
{

class LifecycleSubscriptionInterface
{
public:
  virtual void on_activate() = 0;
  virtual void on_deactivate() = 0;
  virtual bool is_activated() = 0;
};

/// Subscription implementation, templated on the type of message this subscription receives.
template<typename MessageT, typename Alloc = std::allocator<void>>
class LifecycleSubscription : public LifecycleSubscriptionInterface,
  public rclcpp::subscription::Subscription<MessageT, Alloc>
{
public:
  //using MessageAllocTraits = rclcpp::allocator::AllocRebind<MessageT, Alloc>;
  //using MessageAlloc = typename MessageAllocTraits::allocator_type;
  //using MessageDeleter = rclcpp::allocator::Deleter<MessageAlloc, MessageT>;
  //using MessageUniquePtr = std::unique_ptr<MessageT, MessageDeleter>;

  //RCLCPP_SMART_PTR_DEFINITIONS(LifecycleSubscription)

  /// Default constructor.
  /**
   * The constructor for a subscription is almost never called directly. Instead, subscriptions
   * should be instantiated through Node::create_subscription.
   * \param[in] node_handle rcl representation of the node that owns this subscription.
   * \param[in] topic_name Name of the topic to subscribe to.
   * \param[in] subscription_options options for the subscription.
   * \param[in] callback User defined callback to call when a message is received.
   * \param[in] memory_strategy The memory strategy to be used for managing message memory.
   */
  LifecycleSubscription(
    std::shared_ptr<rcl_node_t> node_handle,
    const std::string & topic_name,
    const rcl_subscription_options_t & subscription_options,
    rclcpp::any_subscription_callback::AnySubscriptionCallback<MessageT, Alloc> callback,
    typename rclcpp::message_memory_strategy::MessageMemoryStrategy<MessageT, Alloc>::SharedPtr
    memory_strategy = rclcpp::message_memory_strategy::MessageMemoryStrategy<MessageT,
    Alloc>::create_default())
  : rclcpp::subscription::Subscription<MessageT, Alloc>(
      node_handle,
      topic_name,
      subscription_options,
      callback,
      memory_strategy),
    enabled_(false)
  { }

  /// Check if we need to handle the message, and execute the callback if we do.
  /**
   * \param[in] message Shared pointer to the message to handle.
   * \param[in] message_info Metadata associated with this message.
   */
  virtual void
  handle_message(std::shared_ptr<void> & message, const rmw_message_info_t & message_info)
  {
    fprintf(stderr, "Handle Message callled.\n");
    if (!enabled_) {
      return;
    }

    rclcpp::subscription::Subscription<MessageT, Alloc>::handle_message(message, message_info);
  }

  void
  on_activate()
  {
    enabled_ = true;
  }

  void
  on_deactivate()
  {
    enabled_ = false;
  }

  bool
  is_activated()
  {
    return enabled_;
  }

private:
  bool enabled_ = false;
};

}  // namespace rclcpp_lifecycle

#endif  // RCLCPP_LIFECYCLE__LIFECYCLE_SUBSCRIPTION_HPP_
