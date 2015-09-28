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

#ifndef RCLCPP_RCLCPP_STACK_POOL_MEMORY_STRATEGY_HPP_
#define RCLCPP_RCLCPP_STACK_POOL_MEMORY_STRATEGY_HPP_

#include <unordered_map>

#include <rclcpp/memory_strategy.hpp>

namespace rclcpp
{

namespace memory_strategies
{

namespace stack_pool_memory_strategy
{

/// Stack-based memory pool allocation strategy, an alternative to the default memory strategy.
/**
 * The memory managed by this class is allocated statically (on the heap) in the constructor.
 * StackPoolMemoryStrategy puts a hard limit on the number of subscriptions, etc. that can be
 * executed in one iteration of `Executor::spin`. Thus it allows for memory allocation optimization
 * for situations where a limit on the number of such entities is known.
 * Because this class is templated on the sizes of the memory pools for each entity, the amount of
 * memory required by this class is known at compile time.
 */
template<size_t MaxSubscriptions = 10, size_t MaxServices = 10, size_t MaxClients = 10,
size_t MaxExecutables = 1, size_t MaxGuardConditions = 2, size_t PoolSize = 0>
class StackPoolMemoryStrategy : public memory_strategy::MemoryStrategy
{
public:
  StackPoolMemoryStrategy()
  {
    if (PoolSize) {
      memory_pool_.fill(0);
    }

    if (MaxSubscriptions) {
      subscription_pool_.fill(0);
    }

    if (MaxServices) {
      service_pool_.fill(0);
    }

    if (MaxClients) {
      client_pool_.fill(0);
    }

    if (MaxGuardConditions) {
      guard_condition_pool_.fill(0);
    }

    for (size_t i = 0; i < MaxExecutables; ++i) {
      executable_pool_[i] = std::make_shared<executor::AnyExecutable>();
    }

    pool_seq_ = 0;
    exec_seq_ = 0;

    // Reserve pool_size_ buckets in the memory map.
    memory_map_.reserve(PoolSize);
    for (size_t i = 0; i < PoolSize; ++i) {
      memory_map_[memory_pool_[i]] = 0;
    }
    subs.reserve(MaxSubscriptions);
    clients.reserve(MaxClients);
    services.reserve(MaxServices);
  }

  void ** borrow_handles(HandleType type, size_t number_of_handles)
  {
    switch (type) {
      case HandleType::subscription_handle:
        if (number_of_handles > MaxSubscriptions) {
          throw std::runtime_error("Requested size exceeded maximum subscriptions.");
        }

        return subscription_pool_.data();
      case HandleType::service_handle:
        if (number_of_handles > MaxServices) {
          throw std::runtime_error("Requested size exceeded maximum services.");
        }

        return service_pool_.data();
      case HandleType::client_handle:
        if (number_of_handles > MaxClients) {
          throw std::runtime_error("Requested size exceeded maximum clients.");
        }

        return client_pool_.data();
      case HandleType::guard_condition_handle:
        if (number_of_handles > MaxGuardConditions) {
          throw std::runtime_error("Requested size exceeded maximum guard_conditions.");
        }

        return guard_condition_pool_.data();
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
        if (MaxSubscriptions) {
          subscription_pool_.fill(0);
        }
        break;
      case HandleType::service_handle:
        if (MaxServices) {
          service_pool_.fill(0);
        }
        break;
      case HandleType::client_handle:
        if (MaxClients) {
          client_pool_.fill(0);
        }
        break;
      case HandleType::guard_condition_handle:
        if (MaxGuardConditions) {
          guard_condition_pool_.fill(0);
        }
        break;
      default:
        throw std::runtime_error("Unrecognized enum, could not return handle memory.");
    }
  }

  executor::AnyExecutable::SharedPtr instantiate_next_executable()
  {
    if (exec_seq_ >= MaxExecutables) {
      // wrap around
      exec_seq_ = 0;
    }
    size_t prev_exec_seq_ = exec_seq_;
    ++exec_seq_;

    if (!executable_pool_[prev_exec_seq_]) {
      throw std::runtime_error("Executable pool member was NULL");
    }

    // Make sure to clear the executable fields.
    executable_pool_[prev_exec_seq_]->subscription.reset();
    executable_pool_[prev_exec_seq_]->timer.reset();
    executable_pool_[prev_exec_seq_]->service.reset();
    executable_pool_[prev_exec_seq_]->client.reset();
    executable_pool_[prev_exec_seq_]->callback_group.reset();
    executable_pool_[prev_exec_seq_]->node.reset();

    return executable_pool_[prev_exec_seq_];
  }

  void * alloc(size_t size)
  {
    // Extremely naive static allocation strategy
    // Keep track of block size at a given pointer
    if (pool_seq_ + size > PoolSize) {
      // Start at 0
      pool_seq_ = 0;
    }
    void * ptr = memory_pool_[pool_seq_];
    if (memory_map_.count(ptr) == 0) {
      // We expect to have the state for all blocks pre-mapped into memory_map_
      throw std::runtime_error("Unexpected pointer in StackPoolMemoryStrategy::alloc.");
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
      throw std::runtime_error("Unexpected pointer in StackPoolMemoryStrategy::free.");
    }

    memset(ptr, 0, memory_map_[ptr]);
  }

private:
  std::array<void *, PoolSize> memory_pool_;
  std::array<void *, MaxSubscriptions> subscription_pool_;
  std::array<void *, MaxServices> service_pool_;
  std::array<void *, MaxClients> client_pool_;
  std::array<void *, MaxGuardConditions> guard_condition_pool_;
  std::array<executor::AnyExecutable::SharedPtr, MaxExecutables> executable_pool_;

  size_t pool_seq_;
  size_t exec_seq_;

  std::unordered_map<void *, size_t> memory_map_;
};

}  /* stack_pool_memory_strategy */

}  /* memory_strategies */

}  /* rclcpp */

#endif
