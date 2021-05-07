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

#include <rmw/rmw.h>

#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rcl/error_handling.h"

#include "rclcpp/any_subscription_callback.hpp"
#include "rclcpp/experimental/buffers/intra_process_buffer.hpp"
#include "rclcpp/experimental/create_intra_process_buffer.hpp"
#include "rclcpp/experimental/subscription_intra_process_base.hpp"
#include "rclcpp/experimental/subscription_intra_process_buffer.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/waitable.hpp"
#include "tracetools/tracetools.h"

namespace rclcpp
{
namespace experimental
{

template<
  typename MessageT,
  typename Alloc = std::allocator<void>,
  typename Deleter = std::default_delete<MessageT>,
  typename CallbackMessageT = MessageT>
class SubscriptionIntraProcess
  : public SubscriptionIntraProcessBuffer<
    MessageT,
    Alloc,
    Deleter
  >
{
  using SubscriptionIntraProcessBufferT = SubscriptionIntraProcessBuffer<
    MessageT,
    Alloc,
    Deleter
  >;

public:
  RCLCPP_SMART_PTR_DEFINITIONS(SubscriptionIntraProcess)

  using MessageAllocTraits = typename SubscriptionIntraProcessBufferT::MessageAllocTraits;
  using MessageAlloc = typename SubscriptionIntraProcessBufferT::MessageAlloc;
  using ConstMessageSharedPtr = typename SubscriptionIntraProcessBufferT::ConstMessageSharedPtr;
  using MessageUniquePtr = typename SubscriptionIntraProcessBufferT::MessageUniquePtr;
  using BufferUniquePtr = typename SubscriptionIntraProcessBufferT::BufferUniquePtr;

  SubscriptionIntraProcess(
    AnySubscriptionCallback<CallbackMessageT, Alloc> callback,
    std::shared_ptr<Alloc> allocator,
    rclcpp::Context::SharedPtr context,
    const std::string & topic_name,
    rmw_qos_profile_t qos_profile,
    rclcpp::IntraProcessBufferType buffer_type)
  : SubscriptionIntraProcessBuffer<MessageT, Alloc, Deleter>(
      allocator,
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
  take_data()
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

  void execute(std::shared_ptr<void> & data)
  {
    execute_impl<MessageT>(data);
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

  AnySubscriptionCallback<CallbackMessageT, Alloc> any_callback_;
};

}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__SUBSCRIPTION_INTRA_PROCESS_HPP_
