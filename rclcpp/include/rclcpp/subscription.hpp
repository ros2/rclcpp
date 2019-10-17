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

#include "rclcpp/any_subscription_callback.hpp"
#include "rclcpp/detail/resolve_use_intra_process.hpp"
#include "rclcpp/detail/resolve_intra_process_buffer_type.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rclcpp/intra_process_manager.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/message_memory_strategy.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/subscription_intra_process.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/subscription_traits.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/waitable.hpp"
#include "tracetools/tracetools.h"

namespace rclcpp
{

namespace node_interfaces
{
class NodeTopicsInterface;
}  // namespace node_interfaces

/// Subscription implementation, templated on the type of message this subscription receives.
template<
  typename CallbackMessageT,
  typename AllocatorT = std::allocator<void>,
  typename MessageMemoryStrategyT = rclcpp::message_memory_strategy::MessageMemoryStrategy<
    CallbackMessageT,
    AllocatorT
  >>
class Subscription : public SubscriptionBase
{
  friend class rclcpp::node_interfaces::NodeTopicsInterface;

public:
  using MessageAllocatorTraits = allocator::AllocRebind<CallbackMessageT, AllocatorT>;
  using MessageAllocator = typename MessageAllocatorTraits::allocator_type;
  using MessageDeleter = allocator::Deleter<MessageAllocator, CallbackMessageT>;
  using ConstMessageSharedPtr = std::shared_ptr<const CallbackMessageT>;
  using MessageUniquePtr = std::unique_ptr<CallbackMessageT, MessageDeleter>;

  RCLCPP_SMART_PTR_DEFINITIONS(Subscription)

  /// Default constructor.
  /**
   * The constructor for a subscription is almost never called directly.
   * Instead, subscriptions should be instantiated through the function
   * rclcpp::create_subscription().
   *
   * \param[in] node_base NodeBaseInterface pointer that is used in part of the setup.
   * \param[in] type_support_handle rosidl type support struct, for the Message type of the topic.
   * \param[in] topic_name Name of the topic to subscribe to.
   * \param[in] callback User defined callback to call when a message is received.
   * \param[in] options options for the subscription.
   * \param[in] message_memory_strategy The memory strategy to be used for managing message memory.
   */
  Subscription(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    const rosidl_message_type_support_t & type_support_handle,
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    AnySubscriptionCallback<CallbackMessageT, AllocatorT> callback,
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options,
    typename MessageMemoryStrategyT::SharedPtr message_memory_strategy)
  : SubscriptionBase(
      node_base,
      type_support_handle,
      topic_name,
      options.template to_rcl_subscription_options<CallbackMessageT>(qos),
      rclcpp::subscription_traits::is_serialized_subscription_argument<CallbackMessageT>::value),
    any_callback_(callback),
    options_(options),
    message_memory_strategy_(message_memory_strategy)
  {
    if (options.event_callbacks.deadline_callback) {
      this->add_event_handler(
        options.event_callbacks.deadline_callback,
        RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED);
    }
    if (options.event_callbacks.liveliness_callback) {
      this->add_event_handler(
        options.event_callbacks.liveliness_callback,
        RCL_SUBSCRIPTION_LIVELINESS_CHANGED);
    }

    // Setup intra process publishing if requested.
    if (rclcpp::detail::resolve_use_intra_process(options, *node_base)) {
      using rclcpp::detail::resolve_intra_process_buffer_type;

      // First create a SubscriptionIntraProcess which will be given to the intra-process manager.
      auto context = node_base->get_context();
      auto subscription_intra_process =
        std::make_shared<rclcpp::SubscriptionIntraProcess<CallbackMessageT, AllocatorT>>(
          callback,
          options.get_allocator(),
          context,
          topic_name,
          qos.get_rmw_qos_profile(),
          resolve_intra_process_buffer_type(options.intra_process_buffer_type, callback)
        );

      // Add it to the intra process manager.
      using rclcpp::intra_process_manager::IntraProcessManager;
      auto ipm = context->get_sub_context<IntraProcessManager>();
      uint64_t intra_process_subscription_id = ipm->add_subscription(subscription_intra_process);
      this->setup_intra_process(intra_process_subscription_id, ipm);
    }

    TRACEPOINT(
      rclcpp_subscription_callback_added,
      (const void *)get_subscription_handle().get(),
      (const void *)&any_callback_);
    // The callback object gets copied, so if registration is done too early/before this point
    // (e.g. in `AnySubscriptionCallback::set()`), its address won't match any address used later
    // in subsequent tracepoints.
    any_callback_.register_callback_for_tracing();
  }

  /// Called after construction to continue setup that requires shared_from_this().
  void post_init_setup(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    const rclcpp::QoS & qos,
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options)
  {
    (void)node_base;
    (void)qos;
    (void)options;
  }

  /// Support dynamically setting the message memory strategy.
  /**
   * Behavior may be undefined if called while the subscription could be executing.
   * \param[in] message_memory_strategy Shared pointer to the memory strategy to set.
   */
  void set_message_memory_strategy(
    typename message_memory_strategy::MessageMemoryStrategy<CallbackMessageT,
    AllocatorT>::SharedPtr message_memory_strategy)
  {
    message_memory_strategy_ = message_memory_strategy;
  }

  std::shared_ptr<void> create_message() override
  {
    /* The default message memory strategy provides a dynamically allocated message on each call to
     * create_message, though alternative memory strategies that re-use a preallocated message may be
     * used (see rclcpp/strategies/message_pool_memory_strategy.hpp).
     */
    return message_memory_strategy_->borrow_message();
  }

  std::shared_ptr<rcl_serialized_message_t> create_serialized_message() override
  {
    return message_memory_strategy_->borrow_serialized_message();
  }

  void handle_message(
    std::shared_ptr<void> & message, const rmw_message_info_t & message_info) override
  {
    if (matches_any_intra_process_publishers(&message_info.publisher_gid)) {
      // In this case, the message will be delivered via intra process and
      // we should ignore this copy of the message.
      return;
    }
    auto typed_message = std::static_pointer_cast<CallbackMessageT>(message);
    any_callback_.dispatch(typed_message, message_info);
  }

  void
  handle_loaned_message(
    void * loaned_message, const rmw_message_info_t & message_info) override
  {
    auto typed_message = static_cast<CallbackMessageT *>(loaned_message);
    // message is loaned, so we have to make sure that the deleter does not deallocate the message
    auto sptr = std::shared_ptr<CallbackMessageT>(
      typed_message, [](CallbackMessageT * msg) {(void) msg;});
    any_callback_.dispatch(sptr, message_info);
  }

  /// Return the borrowed message.
  /** \param message message to be returned */
  void return_message(std::shared_ptr<void> & message) override
  {
    auto typed_message = std::static_pointer_cast<CallbackMessageT>(message);
    message_memory_strategy_->return_message(typed_message);
  }

  void return_serialized_message(std::shared_ptr<rcl_serialized_message_t> & message) override
  {
    message_memory_strategy_->return_serialized_message(message);
  }

  bool
  use_take_shared_method() const
  {
    return any_callback_.use_take_shared_method();
  }

private:
  RCLCPP_DISABLE_COPY(Subscription)

  AnySubscriptionCallback<CallbackMessageT, AllocatorT> any_callback_;
  /// Copy of original options passed during construction.
  /**
   * It is important to save a copy of this so that the rmw payload which it
   * may contain is kept alive for the duration of the subscription.
   */
  const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> options_;
  typename message_memory_strategy::MessageMemoryStrategy<CallbackMessageT, AllocatorT>::SharedPtr
    message_memory_strategy_;
};

}  // namespace rclcpp

#endif  // RCLCPP__SUBSCRIPTION_HPP_
