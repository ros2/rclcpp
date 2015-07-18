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

#ifndef RCLCPP_RCLCPP_MSG_POOL_MEMORY_STRATEGY_HPP_
#define RCLCPP_RCLCPP_MSG_POOL_MEMORY_STRATEGY_HPP_

#include <rclcpp/macros.hpp>
#include <rclcpp/message_memory_strategy.hpp>

namespace rclcpp
{
namespace strategies
{
namespace message_pool_memory_strategy
{

template<typename MessageT, size_t size,
typename std::enable_if<rosidl_generator_traits::has_fixed_size<MessageT>::value>::type * =
nullptr>
class MessagePoolMemoryStrategy
  : public message_memory_strategy::MessageMemoryStrategy<MessageT>
{
public:
  RCLCPP_MAKE_SHARED_DEFINITIONS(MessagePoolMemoryStrategy);
  MessagePoolMemoryStrategy()
  : next_array_index_(0)
  {
    for (size_t i = 0; i < size; ++i) {
      pool_[i].msg_ptr_ = std::make_shared<MessageT>();
      pool_[i].used = false;
    }
  }

  std::shared_ptr<MessageT> borrow_message()
  {
    size_t current_index = next_array_index_;
    next_array_index_ = (next_array_index_ + 1) % size;
    if (pool_[current_index].used) {
      throw std::runtime_error("Tried to access message that was still in use! Abort.");
    }
    pool_[current_index].msg_ptr_->~MessageT();
    new (pool_[current_index].msg_ptr_.get())MessageT;

    pool_[current_index].used = true;
    return pool_[current_index].msg_ptr_;
  }

  void return_message(std::shared_ptr<MessageT> & msg)
  {
    for (size_t i = 0; i < size; ++i) {
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

  std::array<PoolMember, size> pool_;
  size_t next_array_index_;

};

}  /* message_pool_memory_strategy */
}  /* strategies */
}  /* rclcpp */
#endif  /* RCLCPP_RCLCPP_MSG_POOL_MEMORY_STRATEGY_HPP_ */
