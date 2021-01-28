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

#include <chrono>
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
#include "rclcpp/experimental/intra_process_manager.hpp"
#include "rclcpp/experimental/subscription_intra_process.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/message_info.hpp"
#include "rclcpp/message_memory_strategy.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/subscription_traits.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/waitable.hpp"
#include "rclcpp/topic_statistics/subscription_topic_statistics.hpp"
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
  using SubscriptionTopicStatisticsSharedPtr =
    std::shared_ptr<rclcpp::topic_statistics::SubscriptionTopicStatistics<CallbackMessageT>>;

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
   * \param[in] qos QoS profile for Subcription.
   * \param[in] callback User defined callback to call when a message is received.
   * \param[in] options Options for the subscription.
   * \param[in] message_memory_strategy The memory strategy to be used for managing message memory.
   * \param[in] subscription_topic_statistics Optional pointer to a topic statistics subcription.
   * \throws std::invalid_argument if the QoS is uncompatible with intra-process (if one
   *   of the following conditions are true: qos_profile.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL,
   *   qos_profile.depth == 0 or qos_profile.durability != RMW_QOS_POLICY_DURABILITY_VOLATILE).
   */
  Subscription(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    const rosidl_message_type_support_t & type_support_handle,
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    AnySubscriptionCallback<CallbackMessageT, AllocatorT> callback,
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options,
    typename MessageMemoryStrategyT::SharedPtr message_memory_strategy,
    SubscriptionTopicStatisticsSharedPtr subscription_topic_statistics = nullptr)
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
    if (options.event_callbacks.incompatible_qos_callback) {
      this->add_event_handler(
        options.event_callbacks.incompatible_qos_callback,
        RCL_SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS);
    } else if (options_.use_default_callbacks) {
      // Register default callback when not specified
      try {
        this->add_event_handler(
          [this](QOSRequestedIncompatibleQoSInfo & info) {
            this->default_incompatible_qos_callback(info);
          },
          RCL_SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS);
      } catch (UnsupportedEventTypeException & /*exc*/) {
        // pass
      }
    }
    if (options.event_callbacks.message_lost_callback) {
      this->add_event_handler(
        options.event_callbacks.message_lost_callback,
        RCL_SUBSCRIPTION_MESSAGE_LOST);
    }

    // Setup intra process publishing if requested.
    if (rclcpp::detail::resolve_use_intra_process(options, *node_base)) {
      using rclcpp::detail::resolve_intra_process_buffer_type;

      // Check if the QoS is compatible with intra-process.
      rmw_qos_profile_t qos_profile = get_actual_qos().get_rmw_qos_profile();
      if (qos_profile.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL) {
        throw std::invalid_argument(
                "intraprocess communication is not allowed with keep all history qos policy");
      }
      if (qos_profile.depth == 0) {
        throw std::invalid_argument(
                "intraprocess communication is not allowed with 0 depth qos policy");
      }
      if (qos_profile.durability != RMW_QOS_POLICY_DURABILITY_VOLATILE) {
        throw std::invalid_argument(
                "intraprocess communication allowed only with volatile durability");
      }

      // First create a SubscriptionIntraProcess which will be given to the intra-process manager.
      auto context = node_base->get_context();
      subscription_intra_process_ = std::make_shared<SubscriptionIntraProcessT>(
        callback,
        options.get_allocator(),
        context,
        this->get_topic_name(),  // important to get like this, as it has the fully-qualified name
        qos_profile,
        resolve_intra_process_buffer_type(options.intra_process_buffer_type, callback));
      TRACEPOINT(
        rclcpp_subscription_init,
        static_cast<const void *>(get_subscription_handle().get()),
        static_cast<const void *>(subscription_intra_process_.get()));

      // Add it to the intra process manager.
      using rclcpp::experimental::IntraProcessManager;
      auto ipm = context->get_sub_context<IntraProcessManager>();
      uint64_t intra_process_subscription_id = ipm->add_subscription(subscription_intra_process_);
      this->setup_intra_process(intra_process_subscription_id, ipm);
    }

    if (subscription_topic_statistics != nullptr) {
      this->subscription_topic_statistics_ = std::move(subscription_topic_statistics);
    }

    TRACEPOINT(
      rclcpp_subscription_init,
      static_cast<const void *>(get_subscription_handle().get()),
      static_cast<const void *>(this));
    TRACEPOINT(
      rclcpp_subscription_callback_added,
      static_cast<const void *>(this),
      static_cast<const void *>(&any_callback_));
    // The callback object gets copied, so if registration is done too early/before this point
    // (e.g. in `AnySubscriptionCallback::set()`), its address won't match any address used later
    // in subsequent tracepoints.
#ifndef TRACETOOLS_DISABLED
    any_callback_.register_callback_for_tracing();
#endif
  }

  /// Called after construction to continue setup that requires shared_from_this().
  void
  post_init_setup(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    const rclcpp::QoS & qos,
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options)
  {
    (void)node_base;
    (void)qos;
    (void)options;
  }

  /// Take the next message from the inter-process subscription.
  /**
   * Data may be taken (written) into the message_out and message_info_out even
   * if false is returned.
   * Specifically in the case of dropping redundant intra-process data, where
   * data is received via both intra-process and inter-process (due to the
   * underlying middleware being unabled to avoid this duplicate delivery) and
   * so inter-process data from those intra-process publishers is ignored, but
   * it has to be taken to know if it came from an intra-process publisher or
   * not, and therefore could be dropped.
   *
   * \sa SubscriptionBase::take_type_erased()
   *
   * \param[out] message_out The message into which take will copy the data.
   * \param[out] message_info_out The message info for the taken message.
   * \returns true if data was taken and is valid, otherwise false
   * \throws any rcl errors from rcl_take, \sa rclcpp::exceptions::throw_from_rcl_error()
   */
  bool
  take(CallbackMessageT & message_out, rclcpp::MessageInfo & message_info_out)
  {
    return this->take_type_erased(static_cast<void *>(&message_out), message_info_out);
  }

  std::shared_ptr<void>
  create_message() override
  {
    /* The default message memory strategy provides a dynamically allocated message on each call to
     * create_message, though alternative memory strategies that re-use a preallocated message may be
     * used (see rclcpp/strategies/message_pool_memory_strategy.hpp).
     */
    return message_memory_strategy_->borrow_message();
  }

  std::shared_ptr<rclcpp::SerializedMessage>
  create_serialized_message() override
  {
    return message_memory_strategy_->borrow_serialized_message();
  }

  void
  handle_message(
    std::shared_ptr<void> & message,
    const rclcpp::MessageInfo & message_info) override
  {
    if (matches_any_intra_process_publishers(&message_info.get_rmw_message_info().publisher_gid)) {
      // In this case, the message will be delivered via intra process and
      // we should ignore this copy of the message.
      return;
    }
    auto typed_message = std::static_pointer_cast<CallbackMessageT>(message);

    std::chrono::time_point<std::chrono::system_clock> now;
    if (subscription_topic_statistics_) {
      // get current time before executing callback to
      // exclude callback duration from topic statistics result.
      now = std::chrono::system_clock::now();
    }

    any_callback_.dispatch(typed_message, message_info);

    if (subscription_topic_statistics_) {
      const auto nanos = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
      const auto time = rclcpp::Time(nanos.time_since_epoch().count());
      subscription_topic_statistics_->handle_message(*typed_message, time);
    }
  }

  void
  handle_loaned_message(
    void * loaned_message,
    const rclcpp::MessageInfo & message_info) override
  {
    auto typed_message = static_cast<CallbackMessageT *>(loaned_message);
    // message is loaned, so we have to make sure that the deleter does not deallocate the message
    auto sptr = std::shared_ptr<CallbackMessageT>(
      typed_message, [](CallbackMessageT * msg) {(void) msg;});
    any_callback_.dispatch(sptr, message_info);
  }

  /// Return the borrowed message.
  /**
   * \param[inout] message message to be returned
   */
  void
  return_message(std::shared_ptr<void> & message) override
  {
    auto typed_message = std::static_pointer_cast<CallbackMessageT>(message);
    message_memory_strategy_->return_message(typed_message);
  }

  /// Return the borrowed serialized message.
  /**
   * \param[inout] message serialized message to be returned
   */
  void
  return_serialized_message(std::shared_ptr<rclcpp::SerializedMessage> & message) override
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
  /// Component which computes and publishes topic statistics for this subscriber
  SubscriptionTopicStatisticsSharedPtr subscription_topic_statistics_{nullptr};
  using SubscriptionIntraProcessT = rclcpp::experimental::SubscriptionIntraProcess<
    CallbackMessageT,
    AllocatorT,
    typename MessageUniquePtr::deleter_type>;
  std::shared_ptr<SubscriptionIntraProcessT> subscription_intra_process_;
};

}  // namespace rclcpp

#endif  // RCLCPP__SUBSCRIPTION_HPP_
