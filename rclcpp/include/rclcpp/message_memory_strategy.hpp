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

#ifndef RCLCPP_RCLCPP_MSG_MEMORY_STRATEGY_HPP_
#define RCLCPP_RCLCPP_MSG_MEMORY_STRATEGY_HPP_

#include <memory>

#include <rclcpp/allocator/allocator_common.hpp>
#include <rclcpp/macros.hpp>

namespace rclcpp
{
namespace message_memory_strategy
{

/// Default allocation strategy for messages received by subscriptions.
// A message memory strategy must be templated on the type of the subscription it belongs to.
template<typename MessageT, typename Alloc = std::allocator<void>>
class MessageMemoryStrategy
{

public:
  RCLCPP_SMART_PTR_DEFINITIONS(MessageMemoryStrategy);

  using MessageAlloc = allocator::AllocRebind<MessageT, Alloc>;
  using MessageDeleter = allocator::Deleter<typename MessageAlloc::allocator_type, MessageT>;

  MessageMemoryStrategy()
  {
    message_allocator_ = new typename MessageAlloc::allocator_type();
  }

  MessageMemoryStrategy(std::shared_ptr<Alloc> allocator)
  {
    message_allocator_ = new typename MessageAlloc::allocator_type(*allocator.get());
  }

  /// Default factory method
  static SharedPtr create_default()
  {
    return std::make_shared<MessageMemoryStrategy<MessageT, Alloc>>(std::make_shared<Alloc>());
  }

  /// By default, dynamically allocate a new message.
  // \return Shared pointer to the new message.
  virtual std::shared_ptr<MessageT> borrow_message()
  {
    auto ptr = MessageAlloc::allocate(*message_allocator_, 1);
    MessageAlloc::construct(*message_allocator_, ptr);
    return std::shared_ptr<MessageT>(ptr, message_deleter_);
  }

  /// Release ownership of the message, which will deallocate it if it has no more owners.
  // \param[in] Shared pointer to the message we are returning.
  virtual void return_message(std::shared_ptr<MessageT> & msg)
  {
    msg.reset();
  }

  typename MessageAlloc::allocator_type * message_allocator_;
  MessageDeleter message_deleter_;
};

}  /* message_memory_strategy */
}  /* rclcpp */

#endif  /* RCLCPP_RCLCPP_MSG_MEMORY_STRATEGY_HPP_ */
