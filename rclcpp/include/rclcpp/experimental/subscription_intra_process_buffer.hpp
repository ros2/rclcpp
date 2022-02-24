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

#ifndef RCLCPP__EXPERIMENTAL__SUBSCRIPTION_INTRA_PROCESS_BUFFER_HPP_
#define RCLCPP__EXPERIMENTAL__SUBSCRIPTION_INTRA_PROCESS_BUFFER_HPP_

#include <memory>
#include <string>
#include <stdexcept>
#include <utility>

#include "rcl/error_handling.h"
#include "rcl/guard_condition.h"
#include "rcl/wait.h"

#include "rclcpp/experimental/buffers/intra_process_buffer.hpp"
#include "rclcpp/experimental/create_intra_process_buffer.hpp"
#include "rclcpp/experimental/subscription_intra_process_base.hpp"
#include "rclcpp/experimental/ros_message_intra_process_buffer.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/type_support_decl.hpp"

namespace rclcpp
{
namespace experimental
{

template<
  typename SubscribedType,
  typename Alloc = std::allocator<SubscribedType>,
  typename Deleter = std::default_delete<SubscribedType>,
  /// MessageT::ros_message_type if MessageT is a TypeAdapter,
  /// otherwise just MessageT.
  typename ROSMessageType = SubscribedType
>
class SubscriptionIntraProcessBuffer : public SubscriptionROSMsgIntraProcessBuffer<ROSMessageType,
    typename allocator::AllocRebind<ROSMessageType, Alloc>::allocator_type,
    allocator::Deleter<typename allocator::AllocRebind<ROSMessageType, Alloc>::allocator_type,
    ROSMessageType>>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(SubscriptionIntraProcessBuffer)

  using SubscribedTypeAllocatorTraits = allocator::AllocRebind<SubscribedType, Alloc>;
  using SubscribedTypeAllocator = typename SubscribedTypeAllocatorTraits::allocator_type;
  using SubscribedTypeDeleter = allocator::Deleter<SubscribedTypeAllocator, SubscribedType>;

  using ROSMessageTypeAllocatorTraits = allocator::AllocRebind<ROSMessageType, Alloc>;
  using ROSMessageTypeAllocator = typename ROSMessageTypeAllocatorTraits::allocator_type;
  using ROSMessageTypeDeleter = allocator::Deleter<ROSMessageTypeAllocator, ROSMessageType>;

  using ConstMessageSharedPtr = std::shared_ptr<const ROSMessageType>;
  using MessageUniquePtr = std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter>;

  using ConstDataSharedPtr = std::shared_ptr<const SubscribedType>;
  using SubscribedTypeUniquePtr = std::unique_ptr<SubscribedType, SubscribedTypeDeleter>;

  using BufferUniquePtr = typename rclcpp::experimental::buffers::IntraProcessBuffer<
    SubscribedType,
    Alloc,
    SubscribedTypeDeleter
    >::UniquePtr;

  SubscriptionIntraProcessBuffer(
    std::shared_ptr<Alloc> allocator,
    rclcpp::Context::SharedPtr context,
    const std::string & topic_name,
    const rclcpp::QoS & qos_profile,
    rclcpp::IntraProcessBufferType buffer_type)
  : SubscriptionROSMsgIntraProcessBuffer<ROSMessageType, ROSMessageTypeAllocator,
      ROSMessageTypeDeleter>(
      context, topic_name, qos_profile),
    subscribed_type_allocator_(*allocator)
  {
    allocator::set_allocator_for_deleter(&subscribed_type_deleter_, &subscribed_type_allocator_);

    // Create the intra-process buffer.
    buffer_ = rclcpp::experimental::create_intra_process_buffer<SubscribedType, Alloc,
        SubscribedTypeDeleter>(
      buffer_type,
      qos_profile,
      std::make_shared<Alloc>(subscribed_type_allocator_));
  }

  bool
  is_ready(rcl_wait_set_t * wait_set) override
  {
    (void) wait_set;
    return buffer_->has_data();
  }

  SubscribedTypeUniquePtr
  convert_ros_message_to_subscribed_type_unique_ptr(const ROSMessageType & msg)
  {
    if constexpr (!std::is_same<SubscribedType, ROSMessageType>::value) {
      auto ptr = SubscribedTypeAllocatorTraits::allocate(subscribed_type_allocator_, 1);
      SubscribedTypeAllocatorTraits::construct(subscribed_type_allocator_, ptr);
      rclcpp::TypeAdapter<SubscribedType, ROSMessageType>::convert_to_custom(msg, *ptr);
      return SubscribedTypeUniquePtr(ptr, subscribed_type_deleter_);
    } else {
      throw std::runtime_error(
              "convert_ros_message_to_subscribed_type_unique_ptr "
              "unexpectedly called without TypeAdapter");
    }
  }

  void
  provide_intra_process_message(ConstMessageSharedPtr message) override
  {
    if constexpr (std::is_same<SubscribedType, ROSMessageType>::value) {
      buffer_->add_shared(std::move(message));
      trigger_guard_condition();
    } else {
      buffer_->add_shared(convert_ros_message_to_subscribed_type_unique_ptr(*message));
      trigger_guard_condition();
    }
    this->invoke_on_new_message();
  }

  void
  provide_intra_process_message(MessageUniquePtr message) override
  {
    if constexpr (std::is_same<SubscribedType, ROSMessageType>::value) {
      buffer_->add_unique(std::move(message));
      trigger_guard_condition();
    } else {
      buffer_->add_unique(convert_ros_message_to_subscribed_type_unique_ptr(*message));
      trigger_guard_condition();
    }
    this->invoke_on_new_message();
  }

  void
  provide_intra_process_data(ConstDataSharedPtr message)
  {
    buffer_->add_shared(std::move(message));
    trigger_guard_condition();
    this->invoke_on_new_message();
  }

  void
  provide_intra_process_data(SubscribedTypeUniquePtr message)
  {
    buffer_->add_unique(std::move(message));
    trigger_guard_condition();
    this->invoke_on_new_message();
  }

  bool
  use_take_shared_method() const override
  {
    return buffer_->use_take_shared_method();
  }

protected:
  void
  trigger_guard_condition() override
  {
    this->gc_.trigger();
  }

  BufferUniquePtr buffer_;
  SubscribedTypeAllocator subscribed_type_allocator_;
  SubscribedTypeDeleter subscribed_type_deleter_;
};

}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__SUBSCRIPTION_INTRA_PROCESS_BUFFER_HPP_
