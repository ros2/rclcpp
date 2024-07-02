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

#ifndef RCLCPP__EXPERIMENTAL__SUBSCRIPTION_INTRA_PROCESS_HPP_
#define RCLCPP__EXPERIMENTAL__SUBSCRIPTION_INTRA_PROCESS_HPP_

#include <rmw/types.h>

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#include "rcl/types.h"

#include "rclcpp/any_subscription_callback.hpp"
#include "rclcpp/context.hpp"
#include "rclcpp/experimental/buffers/intra_process_buffer.hpp"
#include "rclcpp/experimental/subscription_intra_process_buffer.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "tracetools/tracetools.h"

namespace rclcpp
{
namespace experimental
{

template<
  typename MessageT,
  typename SubscribedType,
  typename SubscribedTypeAlloc = std::allocator<SubscribedType>,
  typename SubscribedTypeDeleter = std::default_delete<SubscribedType>,
  typename ROSMessageType = SubscribedType,
  typename Alloc = std::allocator<void>
>
class SubscriptionIntraProcess
  : public SubscriptionIntraProcessBuffer<
    SubscribedType,
    SubscribedTypeAlloc,
    SubscribedTypeDeleter,
    ROSMessageType
  >
{
  using SubscriptionIntraProcessBufferT = SubscriptionIntraProcessBuffer<
    SubscribedType,
    SubscribedTypeAlloc,
    SubscribedTypeDeleter,
    ROSMessageType
  >;

public:
  RCLCPP_SMART_PTR_DEFINITIONS(SubscriptionIntraProcess)

  using MessageAllocTraits =
    typename SubscriptionIntraProcessBufferT::SubscribedTypeAllocatorTraits;
  using MessageAlloc = typename SubscriptionIntraProcessBufferT::SubscribedTypeAllocator;
  using ConstMessageSharedPtr = typename SubscriptionIntraProcessBufferT::ConstDataSharedPtr;
  using MessageUniquePtr = typename SubscriptionIntraProcessBufferT::SubscribedTypeUniquePtr;
  using BufferUniquePtr = typename SubscriptionIntraProcessBufferT::BufferUniquePtr;
  using StatsHandlerFn = std::function<void(const rmw_message_info_t &, const rclcpp::Time &)>;

  SubscriptionIntraProcess(
    AnySubscriptionCallback<MessageT, Alloc> callback,
    std::shared_ptr<Alloc> allocator,
    rclcpp::Context::SharedPtr context,
    const std::string & topic_name,
    const rclcpp::QoS & qos_profile,
    rclcpp::IntraProcessBufferType buffer_type,
    StatsHandlerFn stats_handler = nullptr)
  : SubscriptionIntraProcessBuffer<SubscribedType, SubscribedTypeAlloc,
      SubscribedTypeDeleter, ROSMessageType>(
      std::make_shared<SubscribedTypeAlloc>(*allocator),
      context,
      topic_name,
      qos_profile,
      buffer_type),
    any_callback_(callback),
    stats_handler_(std::move(stats_handler))
  {
    TRACETOOLS_TRACEPOINT(
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

  virtual ~SubscriptionIntraProcess() = default;

  void
  add_to_wait_set(rcl_wait_set_t & wait_set) override
  {
    // This block is necessary when the guard condition wakes the wait set, but
    // the intra process waitable was not handled before the wait set is waited
    // on again.
    // Basically we're keeping the guard condition triggered so long as there is
    // data in the buffer.
    if (this->buffer_->has_data()) {
      // If there is data still to be processed, indicate to the
      // executor or waitset by triggering the guard condition.
      this->trigger_guard_condition();
    }
    // Let the parent classes handle the rest of the work:
    return SubscriptionIntraProcessBufferT::add_to_wait_set(wait_set);
  }

  std::shared_ptr<void>
  take_data() override
  {
    auto data = this->buffer_->consume();
    if (data.message.index() == std::variant_npos) {
      return nullptr;
    }

    if (this->buffer_->has_data()) {
      // If there is data still to be processed, indicate to the
      // executor or waitset by triggering the guard condition.
      this->trigger_guard_condition();
    }

    return std::static_pointer_cast<void>(
      std::make_shared<
        typename SubscriptionIntraProcessBufferT::IntraProcessBuffer::Data>(
        std::move(data))
    );
  }

  void execute(const std::shared_ptr<void> & data) override
  {
    execute_impl<SubscribedType>(data);
  }

  /// Disable callbacks from being called
  /**
    * This method will block, until any subscription's callbacks currently being executed are
    * finished.
    * This method is thread safe, and provides a safe way to atomically disable the callbacks.
    */
  void disable_callbacks() override
  {
    SubscriptionIntraProcessBase::disable_callbacks();
    any_callback_.disable();
  }

  /// Enable the callbacks to be called
  /**
    * This method is thread safe, and provides a safe way to atomically enable the callbacks
    * in a multithreaded environment.
    */
  void enable_callbacks() override
  {
    SubscriptionIntraProcessBase::enable_callbacks();
    any_callback_.enable();
  }

  bool
  use_take_shared_method() const override
  {
    if (this->buffer_->buffer_type() == IntraProcessBufferType::CallbackDefault) {
      return any_callback_.use_take_shared_method();
    } else {
      return this->buffer_->buffer_type() == IntraProcessBufferType::SharedPtr;
    }
  }

protected:
  template<typename T>
  typename std::enable_if<std::is_same<T, rcl_serialized_message_t>::value, void>::type
  execute_impl(const std::shared_ptr<void> &)
  {
    throw std::runtime_error("Subscription intra-process can't handle serialized messages");
  }

  template<class T>
  typename std::enable_if<!std::is_same<T, rcl_serialized_message_t>::value, void>::type
  execute_impl(const std::shared_ptr<void> & data)
  {
    if (nullptr == data) {
      return;
    }

    auto shared_ptr = std::static_pointer_cast<
      typename SubscriptionIntraProcessBufferT::IntraProcessBuffer::Data>(
      data);

    // Copy the message info out before the callback (potentially) moves the message, since
    // the stats handler below is invoked after the callback has run.
    const rmw_message_info_t message_info = shared_ptr->message_info;

    std::visit(
      [&shared_ptr, this](auto && msg) {
        any_callback_.dispatch_intra_process(std::move(msg), shared_ptr->message_info);
      }, shared_ptr->message);

    shared_ptr.reset();

    if (stats_handler_) {
      stats_handler_(message_info, rclcpp::Time(message_info.source_timestamp));
    }
  }

  AnySubscriptionCallback<MessageT, Alloc> any_callback_;
  StatsHandlerFn stats_handler_;
};

}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__SUBSCRIPTION_INTRA_PROCESS_HPP_
