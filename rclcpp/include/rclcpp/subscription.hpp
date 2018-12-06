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

#include "rcl/error_handling.h"
#include "rcl/subscription.h"

#include "rcl_interfaces/msg/intra_process_message.hpp"

#include "rclcpp/any_subscription_callback.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/message_memory_strategy.hpp"
#include "rclcpp/subscription_traits.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

namespace node_interfaces
{
class NodeTopicsInterface;
}  // namespace node_interfaces

/// Virtual base class for subscriptions. This pattern allows us to iterate over different template
/// specializations of Subscription, among other things.
class SubscriptionBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(SubscriptionBase)

  /// Default constructor.
  /**
   * \param[in] node_handle The rcl representation of the node that owns this subscription.
   * \param[in] type_support_handle rosidl type support struct, for the Message type of the topic.
   * \param[in] topic_name Name of the topic to subscribe to.
   * \param[in] subscription_options options for the subscription.
   * \param[in] is_serialized is true if the message will be delivered still serialized
   */
  RCLCPP_PUBLIC
  SubscriptionBase(
    std::shared_ptr<rcl_node_t> node_handle,
    const rosidl_message_type_support_t & type_support_handle,
    const std::string & topic_name,
    const rcl_subscription_options_t & subscription_options,
    bool is_serialized = false);

  /// Default destructor.
  RCLCPP_PUBLIC
  virtual ~SubscriptionBase();

  /// Get the topic that this subscription is subscribed on.
  RCLCPP_PUBLIC
  const char *
  get_topic_name() const;

  RCLCPP_PUBLIC
  std::shared_ptr<rcl_subscription_t>
  get_subscription_handle();

  RCLCPP_PUBLIC
  const std::shared_ptr<rcl_subscription_t>
  get_subscription_handle() const;

  RCLCPP_PUBLIC
  virtual const std::shared_ptr<rcl_subscription_t>
  get_intra_process_subscription_handle() const;

  /// Borrow a new message.
  /** \return Shared pointer to the fresh message. */
  virtual std::shared_ptr<void>
  create_message() = 0;

  /// Borrow a new serialized message
  /** \return Shared pointer to a rcl_message_serialized_t. */
  virtual std::shared_ptr<rcl_serialized_message_t>
  create_serialized_message() = 0;

  /// Check if we need to handle the message, and execute the callback if we do.
  /**
   * \param[in] message Shared pointer to the message to handle.
   * \param[in] message_info Metadata associated with this message.
   */
  virtual void
  handle_message(std::shared_ptr<void> & message, const rmw_message_info_t & message_info) = 0;

  /// Return the message borrowed in create_message.
  /** \param[in] message Shared pointer to the returned message. */
  virtual void
  return_message(std::shared_ptr<void> & message) = 0;

  /// Return the message borrowed in create_serialized_message.
  /** \param[in] message Shared pointer to the returned message. */
  virtual void
  return_serialized_message(std::shared_ptr<rcl_serialized_message_t> & message) = 0;

  virtual void
  handle_intra_process_message(
    rcl_interfaces::msg::IntraProcessMessage & ipm,
    const rmw_message_info_t & message_info) = 0;

  const rosidl_message_type_support_t &
  get_message_type_support_handle() const;

  bool
  is_serialized() const;

protected:
  std::shared_ptr<rcl_subscription_t> intra_process_subscription_handle_;
  std::shared_ptr<rcl_subscription_t> subscription_handle_;
  std::shared_ptr<rcl_node_t> node_handle_;

private:
  RCLCPP_DISABLE_COPY(SubscriptionBase)

  rosidl_message_type_support_t type_support_;
  bool is_serialized_;
};

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
    message_memory_strategy_(memory_strategy),
    get_intra_process_message_callback_(nullptr),
    matches_any_intra_process_publishers_(nullptr)
  {}

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
    if (matches_any_intra_process_publishers_) {
      if (matches_any_intra_process_publishers_(&message_info.publisher_gid)) {
        // In this case, the message will be delivered via intra process and
        // we should ignore this copy of the message.
        return;
      }
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
    if (!get_intra_process_message_callback_) {
      // throw std::runtime_error(
      //   "handle_intra_process_message called before setup_intra_process");
      // TODO(wjwwood): for now, this could mean that intra process was just not enabled.
      // However, this can only really happen if this node has it disabled, but the other doesn't.
      return;
    }
    MessageUniquePtr msg;
    get_intra_process_message_callback_(
      ipm.publisher_id,
      ipm.message_sequence,
      intra_process_subscription_id_,
      msg);
    if (!msg) {
      // This either occurred because the publisher no longer exists or the
      // message requested is no longer being stored.
      // TODO(wjwwood): should we notify someone of this? log error, log warning?
      return;
    }
    any_callback_.dispatch_intra_process(msg, message_info);
  }

  using GetMessageCallbackType =
    std::function<void (uint64_t, uint64_t, uint64_t, MessageUniquePtr &)>;
  using MatchesAnyPublishersCallbackType = std::function<bool (const rmw_gid_t *)>;

  /// Implemenation detail.
  void setup_intra_process(
    uint64_t intra_process_subscription_id,
    GetMessageCallbackType get_message_callback,
    MatchesAnyPublishersCallbackType matches_any_publisher_callback,
    const rcl_subscription_options_t & intra_process_options)
  {
    std::string intra_process_topic_name = std::string(get_topic_name()) + "/_intra";
    rcl_ret_t ret = rcl_subscription_init(
      intra_process_subscription_handle_.get(),
      node_handle_.get(),
      rclcpp::type_support::get_intra_process_message_msg_type_support(),
      intra_process_topic_name.c_str(),
      &intra_process_options);
    if (ret != RCL_RET_OK) {
      if (ret == RCL_RET_TOPIC_NAME_INVALID) {
        auto rcl_node_handle = node_handle_.get();
        // this will throw on any validation problem
        rcl_reset_error();
        expand_topic_or_service_name(
          intra_process_topic_name,
          rcl_node_get_name(rcl_node_handle),
          rcl_node_get_namespace(rcl_node_handle));
      }

      rclcpp::exceptions::throw_from_rcl_error(ret, "could not create intra process subscription");
    }

    intra_process_subscription_id_ = intra_process_subscription_id;
    get_intra_process_message_callback_ = get_message_callback;
    matches_any_intra_process_publishers_ = matches_any_publisher_callback;
  }

  /// Implemenation detail.
  const std::shared_ptr<rcl_subscription_t>
  get_intra_process_subscription_handle() const
  {
    if (!get_intra_process_message_callback_) {
      return nullptr;
    }
    return intra_process_subscription_handle_;
  }

private:
  RCLCPP_DISABLE_COPY(Subscription)

  AnySubscriptionCallback<CallbackMessageT, Alloc> any_callback_;
  typename message_memory_strategy::MessageMemoryStrategy<CallbackMessageT, Alloc>::SharedPtr
    message_memory_strategy_;

  GetMessageCallbackType get_intra_process_message_callback_;
  MatchesAnyPublishersCallbackType matches_any_intra_process_publishers_;
  uint64_t intra_process_subscription_id_;
};

}  // namespace rclcpp

#endif  // RCLCPP__SUBSCRIPTION_HPP_
