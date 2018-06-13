// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__MESSAGE_MEMORY_STRATEGY_HPP_
#define RCLCPP__MESSAGE_MEMORY_STRATEGY_HPP_

#include <memory>
#include <stdexcept>

#include "rcl/types.h"

#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"

#include "rmw/raw_message.h"

namespace rclcpp
{
namespace message_memory_strategy
{

/// Default allocation strategy for messages received by subscriptions.
/** A message memory strategy must be templated on the type of the subscription it belongs to. */
template<typename MessageT, typename Alloc = std::allocator<void>>
class MessageMemoryStrategy
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(MessageMemoryStrategy)

  using MessageAllocTraits = allocator::AllocRebind<MessageT, Alloc>;
  using MessageAlloc = typename MessageAllocTraits::allocator_type;
  using MessageDeleter = allocator::Deleter<MessageAlloc, MessageT>;

  using RawMessageAllocTraits = allocator::AllocRebind<rcl_message_raw_t, Alloc>;
  using RawMessageAlloc = typename RawMessageAllocTraits::allocator_type;
  using RawMessageDeleter = allocator::Deleter<RawMessageAlloc, rcl_message_raw_t>;

  using BufferAllocTraits = allocator::AllocRebind<char, Alloc>;
  using BufferAlloc = typename BufferAllocTraits::allocator_type;
  using BufferDeleter = allocator::Deleter<BufferAlloc, char>;

  MessageMemoryStrategy()
  {
    message_allocator_ = std::make_shared<MessageAlloc>();
    raw_message_allocator_ = std::make_shared<RawMessageAlloc>();
    buffer_allocator_ = std::make_shared<BufferAlloc>();
    rcutils_allocator_ = allocator::get_rcl_allocator<char, BufferAlloc>(*buffer_allocator_.get());
  }

  explicit MessageMemoryStrategy(std::shared_ptr<Alloc> allocator)
  {
    message_allocator_ = std::make_shared<MessageAlloc>(*allocator.get());
    raw_message_allocator_ = std::make_shared<RawMessageAlloc>(*allocator.get());
    buffer_allocator_ = std::make_shared<BufferAlloc>(*allocator.get());
    rcutils_allocator_ = allocator::get_rcl_allocator<char, BufferAlloc>(*buffer_allocator_.get());
  }

  /// Default factory method
  static SharedPtr create_default()
  {
    return std::make_shared<MessageMemoryStrategy<MessageT, Alloc>>(std::make_shared<Alloc>());
  }

  /// By default, dynamically allocate a new message.
  /** \return Shared pointer to the new message. */
  virtual std::shared_ptr<MessageT> borrow_message()
  {
    return std::allocate_shared<MessageT, MessageAlloc>(*message_allocator_.get());
  }

  virtual std::shared_ptr<rcl_message_raw_t> borrow_raw_message(unsigned int capacity)
  {
    // get rcutils_default allocator
    // initialize raw message
    // does this have to be a shared pointer ?!
    //auto raw_msg = std::make_shared<rcl_message_raw_t>(rmw_get_zero_initialized_raw_message());
    auto raw_msg = std::make_shared<rcl_message_raw_t>();
    raw_msg->buffer_length = 0;
    raw_msg->buffer_capacity = 0;
    raw_msg->buffer = NULL;
    rmw_initialize_raw_message(raw_msg.get(), 0, &rcutils_allocator_);
    rmw_raw_message_resize(raw_msg.get(), capacity);
    return raw_msg;
  }

  virtual std::shared_ptr<rcl_message_raw_t> borrow_raw_message()
  {
    return borrow_raw_message(default_buffer_capacity_);
  }

  virtual void set_default_buffer_capacity(unsigned int capacity)
  {
    default_buffer_capacity_ = capacity;
  }

  /// Release ownership of the message, which will deallocate it if it has no more owners.
  /** \param[in] msg Shared pointer to the message we are returning. */
  virtual void return_message(std::shared_ptr<MessageT> & msg)
  {
    msg.reset();
  }

  virtual void return_raw_message(std::shared_ptr<rcl_message_raw_t> & raw_msg)
  {
    rmw_raw_message_fini(raw_msg.get());
    raw_msg.reset();
  }

  std::shared_ptr<MessageAlloc> message_allocator_;
  MessageDeleter message_deleter_;

  std::shared_ptr<RawMessageAlloc> raw_message_allocator_;
  RawMessageDeleter raw_message_deleter_;

  std::shared_ptr<BufferAlloc> buffer_allocator_;
  BufferDeleter buffer_deleter_;
  unsigned int default_buffer_capacity_ = 0;

  rcutils_allocator_t rcutils_allocator_;
};

}  // namespace message_memory_strategy
}  // namespace rclcpp

#endif  // RCLCPP__MESSAGE_MEMORY_STRATEGY_HPP_
