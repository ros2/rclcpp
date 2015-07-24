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

namespace memory_strategy
{

template<typename T, size_t S>
class StaticContainerInterface : public ContainerInterface<T>
{
public:
  RCLCPP_MAKE_SHARED_DEFINITIONS(StaticContainerInterface);
  T& operator[](size_t pos)
  {
    return container_[pos];
  }
  T& at(size_t pos)
  {
    return container_.at(pos);
  }
  size_t size() const
  {
    return container_.size();
  }

  T* data()
  {
    return container_.data();
  }

  T* begin()
  {
    return data();
  }

  T* end()
  {
    return data() + seq;
  }

  void add_vector(std::vector<T> &vec)
  {
    if (vec.size() > size())
    {
      throw std::runtime_error("Requested size exceeded maximum number of subscribers.");
    }
    for (size_t i = 0; i < vec.size(); ++i)
    {
      at(i) = vec[i];
    }
    if (vec.size() > 0)
      seq = vec.size() - 1;
    else
      seq = 0;
  }

  size_t seq = 0;

private:
  std::array<T, S> container_;
};

}

namespace memory_strategies
{

namespace static_memory_strategy
{

class StaticMemoryStrategy : public memory_strategy::MemoryStrategy
{
public:
  StaticMemoryStrategy()
  {
    memset(memory_pool_, 0, pool_size_);
    memset(subscriber_pool_, 0, max_subscribers_);
    memset(service_pool_, 0, max_services_);
    memset(client_pool_, 0, max_clients_);
    memset(guard_condition_pool_, 0, max_guard_conditions_);
    pool_seq_ = 0;
    exec_seq_ = 0;

    // Reserve pool_size_ buckets in the memory map.
    memory_map_.reserve(pool_size_);
    for (size_t i = 0; i < pool_size_; ++i) {
      memory_map_[memory_pool_[i]] = 0;
    }
  }

  void ** borrow_handles(HandleType type, size_t number_of_handles)
  {
    switch (type) {
      case HandleType::subscriber_handle:
        if (number_of_handles > max_subscribers_) {
          throw std::runtime_error("Requested size exceeded maximum subscribers.");
        }

        return subscriber_pool_;
      case HandleType::service_handle:
        if (number_of_handles > max_services_) {
          throw std::runtime_error("Requested size exceeded maximum services.");
        }

        return service_pool_;
      case HandleType::client_handle:
        if (number_of_handles > max_clients_) {
          throw std::runtime_error("Requested size exceeded maximum clients.");
        }

        return client_pool_;
      case HandleType::guard_condition_handle:
        if (number_of_handles > max_guard_conditions_) {
          throw std::runtime_error("Requested size exceeded maximum guard conditions.");
        }

        return guard_condition_pool_;
      default:
        break;
    }
    throw std::runtime_error("Unrecognized enum, could not borrow handle memory.");
  }

  void return_handles(HandleType type, void ** handles)
  {
    (void)handles;
    switch (type) {
      case HandleType::subscriber_handle:
        memset(subscriber_pool_, 0, max_subscribers_);
        break;
      case HandleType::service_handle:
        memset(service_pool_, 0, max_services_);
        break;
      case HandleType::client_handle:
        memset(client_pool_, 0, max_clients_);
        break;
      case HandleType::guard_condition_handle:
        memset(guard_condition_pool_, 0, max_guard_conditions_);
        break;
      default:
        throw std::runtime_error("Unrecognized enum, could not return handle memory.");
    }
  }

  executor::AnyExecutable::SharedPtr instantiate_next_executable()
  {
    if (exec_seq_ >= max_executables_) {
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
    if (pool_seq_ + size > pool_size_) {
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

  std::shared_ptr<memory_strategy::ContainerInterface<subscription::SubscriptionBase::SharedPtr>>
  get_subscription_container_interface()
  {
    return std::make_shared<memory_strategy::ContainerInterface<subscription::SubscriptionBase::SharedPtr>>(subscription_container_);
  }

  std::shared_ptr<memory_strategy::ContainerInterface<service::ServiceBase::SharedPtr>>
  get_service_container_interface()
  {
    return std::make_shared<memory_strategy::ContainerInterface<service::ServiceBase::SharedPtr>>(services_container_);
  }

  std::shared_ptr<memory_strategy::ContainerInterface<client::ClientBase::SharedPtr>>
  get_client_container_interface()
  {
    return std::make_shared<memory_strategy::ContainerInterface<client::ClientBase::SharedPtr>>(clients_container_);
  }

  std::shared_ptr<memory_strategy::ContainerInterface<timer::TimerBase::SharedPtr>>
  get_timer_container_interface()
  {
    return std::make_shared<memory_strategy::ContainerInterface<timer::TimerBase::SharedPtr>>(timers_container_);
  }

  template<typename T>
  void return_container_interface(std::shared_ptr<void> container)
  {
    auto static_container = dynamic_cast<memory_strategy::StaticContainerInterface<T>>(container);
    if (!static_container)
    {
      throw std::runtime_error("Failed to downcast container to static type");
    }
    static_container->seq = 0;
  }


private:
  static const size_t pool_size_ = 1024;
  static const size_t max_subscribers_ = 10;
  static const size_t max_services_ = 5;
  static const size_t max_clients_ = 10;
  static const size_t max_guard_conditions_ = 50;
  static const size_t max_executables_ = 1;

  void * memory_pool_[pool_size_];
  void * subscriber_pool_[max_subscribers_];
  void * service_pool_[max_services_];
  void * client_pool_[max_clients_];
  void * guard_condition_pool_[max_guard_conditions_];
  executor::AnyExecutable executable_pool_[max_executables_];

  size_t pool_seq_;
  size_t exec_seq_;

  std::unordered_map<void *, size_t> memory_map_;

  memory_strategy::StaticContainerInterface<subscription::SubscriptionBase::SharedPtr, max_subscribers_> subscription_container_;
  memory_strategy::StaticContainerInterface<service::ServiceBase::SharedPtr, max_services_> services_container_;
  memory_strategy::StaticContainerInterface<client::ClientBase::SharedPtr, max_clients_> clients_container_;
  memory_strategy::StaticContainerInterface<timer::TimerBase::SharedPtr, max_guard_conditions_> timers_container_;
};

}  /* static_memory_strategy */

}  /* memory_strategies */

}  /* rclcpp */

#endif
