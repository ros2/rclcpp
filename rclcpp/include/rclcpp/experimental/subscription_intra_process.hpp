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

  SubscriptionIntraProcess(
    AnySubscriptionCallback<MessageT, Alloc> callback,
    std::shared_ptr<Alloc> allocator,
    rclcpp::Context::SharedPtr context,
    const std::string & topic_name,
    const rclcpp::QoS & qos_profile,
    rclcpp::IntraProcessBufferType buffer_type)
  : SubscriptionIntraProcessBuffer<SubscribedType, SubscribedTypeAlloc,
      SubscribedTypeDeleter, ROSMessageType>(
      std::make_shared<SubscribedTypeAlloc>(*allocator),
      context,
      topic_name,
      qos_profile,
      buffer_type),
    any_callback_(callback)
  {
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

  virtual ~SubscriptionIntraProcess() = default;

  std::shared_ptr<void>
  take_data() override
  {
    ConstMessageSharedPtr shared_msg;
    MessageUniquePtr unique_msg;

    if (any_callback_.use_take_shared_method()) {
      shared_msg = this->buffer_->consume_shared();
    } else {
      unique_msg = this->buffer_->consume_unique();
    }
    return std::static_pointer_cast<void>(
      std::make_shared<std::pair<ConstMessageSharedPtr, MessageUniquePtr>>(
        std::pair<ConstMessageSharedPtr, MessageUniquePtr>(
          shared_msg, std::move(unique_msg)))
    );
  }

  void execute(std::shared_ptr<void> & data) override
  {
    execute_impl<SubscribedType>(data);
  }

protected:
  template<typename T>
  typename std::enable_if<std::is_same<T, rcl_serialized_message_t>::value, void>::type
  execute_impl(std::shared_ptr<void> & data)
  {
    (void)data;
    throw std::runtime_error("Subscription intra-process can't handle serialized messages");
  }

  template<class T>
  typename std::enable_if<!std::is_same<T, rcl_serialized_message_t>::value, void>::type
  execute_impl(std::shared_ptr<void> & data)
  {
    if (!data) {
      throw std::runtime_error("'data' is empty");
    }

    rmw_message_info_t msg_info;
    msg_info.publisher_gid = {0, {0}};
    msg_info.from_intra_process = true;

    auto shared_ptr = std::static_pointer_cast<std::pair<ConstMessageSharedPtr, MessageUniquePtr>>(
      data);

    if (any_callback_.use_take_shared_method()) {
      ConstMessageSharedPtr shared_msg = shared_ptr->first;
      any_callback_.dispatch_intra_process(shared_msg, msg_info);
    } else {
      MessageUniquePtr unique_msg = std::move(shared_ptr->second);
      any_callback_.dispatch_intra_process(std::move(unique_msg), msg_info);
    }
    shared_ptr.reset();
  }

  AnySubscriptionCallback<MessageT, Alloc> any_callback_;
};

}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__SUBSCRIPTION_INTRA_PROCESS_HPP_
