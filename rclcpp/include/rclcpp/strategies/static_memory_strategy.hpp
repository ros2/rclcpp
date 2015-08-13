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

#ifndef RCLCPP_RCLCPP_STATIC_MEMORY_STRATEGY_HPP_
#define RCLCPP_RCLCPP_STATIC_MEMORY_STRATEGY_HPP_

#include <unordered_map>

#include <rclcpp/memory_strategy.hpp>

namespace rclcpp
{

namespace memory_strategies
{

namespace static_memory_strategy
{
struct ObjectPoolBounds
{
public:
  size_t max_subscriptions_;
  size_t max_services_;
  size_t max_clients_;
  size_t max_executables_;
  size_t pool_size_;

  ObjectPoolBounds(size_t subs = 10, size_t services = 10, size_t clients = 10,
    size_t executables = 1, size_t pool = 1024)
  : max_subscriptions_(subs), max_services_(services), max_clients_(clients), max_executables_(
      executables), pool_size_(pool)
  {}
};


class StaticMemoryStrategy : public memory_strategy::MemoryStrategy
{
public:
  StaticMemoryStrategy(ObjectPoolBounds bounds = ObjectPoolBounds())
  : bounds_(bounds)
  {
    memory_pool_ = static_cast<void **>(malloc(bounds_.pool_size_));
    subscription_pool_ = static_cast<void **>(malloc(bounds_.max_subscriptions_));
    service_pool_ = static_cast<void **>(malloc(bounds_.max_services_));
    client_pool_ = static_cast<void **>(malloc(bounds_.max_clients_));
    executable_pool_ = static_cast<executor::AnyExecutable *>(malloc(bounds_.max_executables_));

    memset(memory_pool_, 0, bounds_.pool_size_);
    memset(subscription_pool_, 0, bounds_.max_subscriptions_);
    memset(service_pool_, 0, bounds_.max_services_);
    memset(client_pool_, 0, bounds_.max_clients_);
    memset(executable_pool_, 0, bounds_.max_executables_);
    pool_seq_ = 0;
    exec_seq_ = 0;

    // Reserve pool_size_ buckets in the memory map.
    memory_map_.reserve(bounds_.pool_size_);
    for (size_t i = 0; i < bounds_.pool_size_; ++i) {
      memory_map_[memory_pool_[i]] = 0;
    }
  }

  void ** borrow_handles(HandleType type, size_t number_of_handles)
  {
    switch (type) {
      case HandleType::subscription_handle:
        if (number_of_handles > bounds_.max_subscriptions_) {
          throw std::runtime_error("Requested size exceeded maximum subscriptions.");
        }

        return subscription_pool_;
      case HandleType::service_handle:
        if (number_of_handles > bounds_.max_services_) {
          throw std::runtime_error("Requested size exceeded maximum services.");
        }

        return service_pool_;
      case HandleType::client_handle:
        if (number_of_handles > bounds_.max_clients_) {
          throw std::runtime_error("Requested size exceeded maximum clients.");
        }

        return client_pool_;
      default:
        break;
    }
    throw std::runtime_error("Unrecognized enum, could not borrow handle memory.");
  }

  void return_handles(HandleType type, void ** handles)
  {
    (void)handles;
    switch (type) {
      case HandleType::subscription_handle:
        memset(subscription_pool_, 0, bounds_.max_subscriptions_);
        break;
      case HandleType::service_handle:
        memset(service_pool_, 0, bounds_.max_services_);
        break;
      case HandleType::client_handle:
        memset(client_pool_, 0, bounds_.max_clients_);
        break;
      default:
        throw std::runtime_error("Unrecognized enum, could not return handle memory.");
    }
  }

  executor::AnyExecutable::SharedPtr instantiate_next_executable()
  {
    if (exec_seq_ >= bounds_.max_executables_) {
      // wrap around
      exec_seq_ = 0;
    }
    size_t prev_exec_seq_ = exec_seq_;
    ++exec_seq_;

    return std::make_shared<executor::AnyExecutable>(executable_pool_[prev_exec_seq_]);
  }

  void * alloc(size_t size)
  {
    // Extremely naive static allocation strategy
    // Keep track of block size at a given pointer
    if (pool_seq_ + size > bounds_.pool_size_) {
      // Start at 0
      pool_seq_ = 0;
    }
    void * ptr = memory_pool_[pool_seq_];
    if (memory_map_.count(ptr) == 0) {
      // We expect to have the state for all blocks pre-mapped into memory_map_
      throw std::runtime_error("Unexpected pointer in rcl_malloc.");
    }
    memory_map_[ptr] = size;
    size_t prev_pool_seq = pool_seq_;
    pool_seq_ += size;
    return memory_pool_[prev_pool_seq];
  }

  void free(void * ptr)
  {
    if (memory_map_.count(ptr) == 0) {
      // We expect to have the state for all blocks pre-mapped into memory_map_
      throw std::runtime_error("Unexpected pointer in rcl_free.");
    }

    memset(ptr, 0, memory_map_[ptr]);
  }

private:
  ObjectPoolBounds bounds_;

  void ** memory_pool_;
  void ** subscription_pool_;
  void ** service_pool_;
  void ** client_pool_;
  executor::AnyExecutable * executable_pool_;

  size_t pool_seq_;
  size_t exec_seq_;

  std::unordered_map<void *, size_t> memory_map_;
};

}  /* static_memory_strategy */

}  /* memory_strategies */

}  /* rclcpp */

#endif
