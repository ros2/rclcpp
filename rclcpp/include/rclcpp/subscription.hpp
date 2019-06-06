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

#ifndef RCLCPP__SUBSCRIPTION_HPP_
#define RCLCPP__SUBSCRIPTION_HPP_

#include <rmw/error_handling.h>
#include <rmw/rmw.h>

#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>


#include "rcl/error_handling.h"
#include "rcl/subscription.h"

#include "rcl_interfaces/msg/intra_process_message.hpp"

#include "rclcpp/any_subscription_callback.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rclcpp/intra_process_manager.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/message_memory_strategy.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/subscription_traits.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/waitable.hpp"

namespace rclcpp
{

namespace node_interfaces
{
class NodeTopicsInterface;
}  // namespace node_interfaces

/// Subscription implementation, templated on the type of message this subscription receives.
template<
  typename CallbackMessageT,
  typename Alloc = std::allocator<void>>
class Subscription : public SubscriptionBase
{
  friend class rclcpp::node_interfaces::NodeTopicsInterface;

public:
  using MessageAllocTraits = allocator::AllocRebind<CallbackMessageT, Alloc>;
  using MessageAlloc = typename MessageAllocTraits::allocator_type;
  using MessageDeleter = allocator::Deleter<MessageAlloc, CallbackMessageT>;
  using ConstMessageSharedPtr = std::shared_ptr<const CallbackMessageT>;
  using MessageUniquePtr = std::unique_ptr<CallbackMessageT, MessageDeleter>;

  RCLCPP_SMART_PTR_DEFINITIONS(Subscription)

  /// Default constructor.
  /**
   * The constructor for a subscription is almost never called directly. Instead, subscriptions
   * should be instantiated through Node::create_subscription.
   * \param[in] node_handle rcl representation of the node that owns this subscription.
   * \param[in] type_support_handle rosidl type support struct, for the Message type of the topic.
   * \param[in] topic_name Name of the topic to subscribe to.
   * \param[in] subscription_options options for the subscription.
   * \param[in] callback User defined callback to call when a message is received.
   * \param[in] memory_strategy The memory strategy to be used for managing message memory.
   */
  Subscription(
    std::shared_ptr<rcl_node_t> node_handle,
    const rosidl_message_type_support_t & type_support_handle,
    const std::string & topic_name,
    const rcl_subscription_options_t & subscription_options,
    AnySubscriptionCallback<CallbackMessageT, Alloc> callback,
    const SubscriptionEventCallbacks & event_callbacks,
    typename message_memory_strategy::MessageMemoryStrategy<CallbackMessageT, Alloc>::SharedPtr
    memory_strategy = message_memory_strategy::MessageMemoryStrategy<CallbackMessageT,
    Alloc>::create_default())
  : SubscriptionBase(
      node_handle,
      type_support_handle,
      topic_name,
      subscription_options,
      rclcpp::subscription_traits::is_serialized_subscription_argument<CallbackMessageT>::value),
    any_callback_(callback),
    message_memory_strategy_(memory_strategy)
  {
    if (event_callbacks.deadline_callback) {
      this->add_event_handler(event_callbacks.deadline_callback,
        RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED);
    }
    if (event_callbacks.liveliness_callback) {
      this->add_event_handler(event_callbacks.liveliness_callback,
        RCL_SUBSCRIPTION_LIVELINESS_CHANGED);
    }
  }

  /// Support dynamically setting the message memory strategy.
  /**
   * Behavior may be undefined if called while the subscription could be executing.
   * \param[in] message_memory_strategy Shared pointer to the memory strategy to set.
   */
  void set_message_memory_strategy(
    typename message_memory_strategy::MessageMemoryStrategy<CallbackMessageT,
    Alloc>::SharedPtr message_memory_strategy)
  {
    message_memory_strategy_ = message_memory_strategy;
  }

  std::shared_ptr<void> create_message()
  {
    /* The default message memory strategy provides a dynamically allocated message on each call to
     * create_message, though alternative memory strategies that re-use a preallocated message may be
     * used (see rclcpp/strategies/message_pool_memory_strategy.hpp).
     */
    return message_memory_strategy_->borrow_message();
  }

  std::shared_ptr<rcl_serialized_message_t> create_serialized_message()
  {
    return message_memory_strategy_->borrow_serialized_message();
  }

  void handle_message(std::shared_ptr<void> & message, const rmw_message_info_t & message_info)
  {
    if (matches_any_intra_process_publishers(&message_info.publisher_gid)) {
      // In this case, the message will be delivered via intra process and
      // we should ignore this copy of the message.
      return;
    }
    auto typed_message = std::static_pointer_cast<CallbackMessageT>(message);
    any_callback_.dispatch(typed_message, message_info);
  }

  /// Return the loaned message.
  /** \param message message to be returned */
  void return_message(std::shared_ptr<void> & message)
  {
    auto typed_message = std::static_pointer_cast<CallbackMessageT>(message);
    message_memory_strategy_->return_message(typed_message);
  }

  void return_serialized_message(std::shared_ptr<rcl_serialized_message_t> & message)
  {
    message_memory_strategy_->return_serialized_message(message);
  }

  void handle_intra_process_message(
    rcl_interfaces::msg::IntraProcessMessage & ipm,
    const rmw_message_info_t & message_info)
  {
    if (!use_intra_process_) {
      // throw std::runtime_error(
      //   "handle_intra_process_message called before setup_intra_process");
      // TODO(wjwwood): for now, this could mean that intra process was just not enabled.
      // However, this can only really happen if this node has it disabled, but the other doesn't.
      return;
    }

    if (!matches_any_intra_process_publishers(&message_info.publisher_gid)) {
      // This intra-process message has not been created by a publisher from this context.
      // we should ignore this copy of the message.
      return;
    }

    if (any_callback_.use_take_shared_method()) {
      ConstMessageSharedPtr msg;
      take_intra_process_message(
        ipm.publisher_id,
        ipm.message_sequence,
        intra_process_subscription_id_,
        msg);
      if (!msg) {
        // This can happen when having two nodes in different process both using intraprocess
        // communication. It could happen too if the publisher no longer exists or the requested
        // message is not longer being stored.
        // TODO(ivanpauno): Print a warn message in the last two cases described above,
        // but not in the first one.
        return;
      }
      any_callback_.dispatch_intra_process(msg, message_info);
    } else {
      MessageUniquePtr msg;
      take_intra_process_message(
        ipm.publisher_id,
        ipm.message_sequence,
        intra_process_subscription_id_,
        msg);
      if (!msg) {
        // This can happen when having two nodes in different process both using intraprocess
        // communication. It could happen too if the publisher no longer exists or the requested
        // message is not longer being stored.
        // TODO(ivanpauno): Print a warn message in the last two cases described above,
        // but not in the first one.
        return;
      }
      any_callback_.dispatch_intra_process(std::move(msg), message_info);
    }
  }

  /// Implemenation detail.
  const std::shared_ptr<rcl_subscription_t>
  get_intra_process_subscription_handle() const
  {
    if (!use_intra_process_) {
      return nullptr;
    }
    return intra_process_subscription_handle_;
  }

private:
  void
  take_intra_process_message(
    uint64_t publisher_id,
    uint64_t message_sequence,
    uint64_t subscription_id,
    MessageUniquePtr & message)
  {
    auto ipm = weak_ipm_.lock();
    if (!ipm) {
      throw std::runtime_error(
              "intra process take called after destruction of intra process manager");
    }
    ipm->template take_intra_process_message<CallbackMessageT, Alloc>(
      publisher_id, message_sequence, subscription_id, message);
  }

  void
  take_intra_process_message(
    uint64_t publisher_id,
    uint64_t message_sequence,
    uint64_t subscription_id,
    ConstMessageSharedPtr & message)
  {
    auto ipm = weak_ipm_.lock();
    if (!ipm) {
      throw std::runtime_error(
              "intra process take called after destruction of intra process manager");
    }
    ipm->template take_intra_process_message<CallbackMessageT, Alloc>(
      publisher_id, message_sequence, subscription_id, message);
  }

  bool
  matches_any_intra_process_publishers(const rmw_gid_t * sender_gid)
  {
    if (!use_intra_process_) {
      return false;
    }
    auto ipm = weak_ipm_.lock();
    if (!ipm) {
      throw std::runtime_error(
              "intra process publisher check called "
              "after destruction of intra process manager");
    }
    return ipm->matches_any_publishers(sender_gid);
  }

  RCLCPP_DISABLE_COPY(Subscription)

  AnySubscriptionCallback<CallbackMessageT, Alloc> any_callback_;
  typename message_memory_strategy::MessageMemoryStrategy<CallbackMessageT, Alloc>::SharedPtr
    message_memory_strategy_;
};

}  // namespace rclcpp

#endif  // RCLCPP__SUBSCRIPTION_HPP_
