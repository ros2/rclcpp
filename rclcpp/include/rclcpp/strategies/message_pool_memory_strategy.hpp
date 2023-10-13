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

#include <array>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <type_traits>

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
  {
    for (size_t i = 0; i < Size; ++i) {
      pool_[i] = std::make_shared<MessageT>();
      free_list_.push_back(i);
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
    std::lock_guard<std::mutex> lock(pool_mutex_);
    if (free_list_.size() == 0) {
      for (size_t i = 0; i < Size; ++i) {
        if (pool_[i].use_count() == 1) {
          free_list_.push_back(i);
          break;
        }
      }
      if (free_list_.size() == 0) {
        throw std::runtime_error("No more free slots in the pool");
      }
    }

    size_t current_index = free_list_.pop_front();

    pool_[current_index]->~MessageT();
    new (pool_[current_index].get())MessageT;

    return pool_[current_index];
  }

  /// Return a message to the message pool.
  /**
   * Manage metadata in the message pool ring buffer to release the message.
   * \param[in] msg Shared pointer to the message to return.
   */
  void return_message(std::shared_ptr<MessageT> & msg)
  {
    (void)msg;

    // What we really want to do here is to figure out whether the user has taken an additional
    // reference to the message, and only add it to the free list if that is *not* the case.
    // However, we can't really do that for the currently passed-in msg; it can have an arbitrary
    // reference count due to the mechanisms of rclcpp.  Instead, we look at all the rest of the
    // pointers, and add the ones that the user has released into the free pool.
    // We do the same thing in borrow_message(), so if the user has a pool of size 1
    // (or only one free slot), we'll always find it.

    std::lock_guard<std::mutex> lock(pool_mutex_);
    if (free_list_.size() == 0) {
      for (size_t i = 0; i < Size; ++i) {
        if (pool_[i].use_count() == 1) {
          free_list_.push_back(i);
        }
      }
    }
  }

protected:
  template<size_t N>
  class CyclicSizeTArray
  {
public:
    void push_back(const size_t v)
    {
      if (size_ + 1 > N) {
        throw std::runtime_error("Tried to push too many items into the array");
      }
      array_[(front_ + size_) % N] = v;
      ++size_;
    }

    size_t pop_front()
    {
      if (size_ < 1) {
        throw std::runtime_error("Tried to pop item from empty array");
      }

      size_t val = array_[front_];

      front_ = (front_ + 1) % N;
      --size_;

      return val;
    }

    size_t size() const
    {
      return size_;
    }

private:
    size_t front_ = 0;
    size_t size_ = 0;
    std::array<size_t, N> array_;
  };

  std::mutex pool_mutex_;
  std::array<std::shared_ptr<MessageT>, Size> pool_;
  CyclicSizeTArray<Size> free_list_;
};

}  // namespace message_pool_memory_strategy
}  // namespace strategies
}  // namespace rclcpp

#endif  // RCLCPP__STRATEGIES__MESSAGE_POOL_MEMORY_STRATEGY_HPP_
