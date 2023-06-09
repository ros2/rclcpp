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

#ifndef RCLCPP__STRATEGIES__MESSAGE_POOL_MEMORY_STRATEGY_HPP_
#define RCLCPP__STRATEGIES__MESSAGE_POOL_MEMORY_STRATEGY_HPP_

#include <memory>

#include "rosidl_runtime_cpp/traits.hpp"

#include "rclcpp/macros.hpp"
#include "rclcpp/message_memory_strategy.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace strategies
{
namespace message_pool_memory_strategy
{

/// Completely static memory allocation strategy for messages.
/**
 * Templated on the type of message pooled by this class and the size of the message pool.
 * Templating allows the program to determine the memory required for this object at compile time.
 * The size of the message pool should be at least the largest number of concurrent accesses to
 * the subscription (usually the number of threads).
 */
template<
  typename MessageT,
  size_t Size,
  typename std::enable_if<
    rosidl_generator_traits::has_fixed_size<MessageT>::value
  >::type * = nullptr
>
class MessagePoolMemoryStrategy
  : public message_memory_strategy::MessageMemoryStrategy<MessageT>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(MessagePoolMemoryStrategy)

  /// Default constructor
  MessagePoolMemoryStrategy()
  : next_array_index_(0)
  {
    for (size_t i = 0; i < Size; ++i) {
      pool_[i].msg_ptr_ = std::make_shared<MessageT>();
      pool_[i].used = false;
    }
  }

  /// Borrow a message from the message pool.
  /**
   * Manage the message pool ring buffer.
   * Throw an exception if the next message was not available.
   * \return Shared pointer to the borrowed message.
   */
  std::shared_ptr<MessageT> borrow_message()
  {
    size_t current_index = next_array_index_;
    next_array_index_ = (next_array_index_ + 1) % Size;
    if (pool_[current_index].used) {
      throw std::runtime_error("Tried to access message that was still in use! Abort.");
    }
    pool_[current_index].msg_ptr_->~MessageT();
    new (pool_[current_index].msg_ptr_.get())MessageT;

    pool_[current_index].used = true;
    return pool_[current_index].msg_ptr_;
  }

  /// Return a message to the message pool.
  /**
   * Manage metadata in the message pool ring buffer to release the message.
   * \param[in] msg Shared pointer to the message to return.
   */
  void return_message(std::shared_ptr<MessageT> & msg)
  {
    for (size_t i = 0; i < Size; ++i) {
      if (pool_[i].msg_ptr_ == msg) {
        pool_[i].used = false;
        return;
      }
    }
    throw std::runtime_error("Unrecognized message ptr in return_message.");
  }

protected:
  struct PoolMember
  {
    std::shared_ptr<MessageT> msg_ptr_;
    bool used;
  };

  std::array<PoolMember, Size> pool_;
  size_t next_array_index_;
};

}  // namespace message_pool_memory_strategy
}  // namespace strategies
}  // namespace rclcpp

#endif  // RCLCPP__STRATEGIES__MESSAGE_POOL_MEMORY_STRATEGY_HPP_
