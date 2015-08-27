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

/// Representation of the upper bounds on the memory pools managed by StaticMemoryStrategy.
struct ObjectPoolBounds
{
public:
  size_t max_subscriptions;
  size_t max_services;
  size_t max_clients;
  size_t max_executables;
  size_t max_guard_conditions;
  size_t pool_size;

  /// Default constructor attempts to set reasonable default limits on the fields.
  ObjectPoolBounds()
  : max_subscriptions(10), max_services(10), max_clients(10),
    max_executables(1), max_guard_conditions(2), pool_size(1024)
  {}

  // Setters implement named parameter idiom/method chaining

  /// Set the maximum number of subscriptions.
  /* \param[in] subscriptions Maximum number of subscriptions.
   * \return Reference to this object, for method chaining.
   */
  ObjectPoolBounds & set_max_subscriptions(size_t subscriptions)
  {
    max_subscriptions = subscriptions;
    return *this;
  }

  /// Set the maximum number of services.
  /* \param[in] services Maximum number of services.
   * \return Reference to this object, for method chaining.
   */
  ObjectPoolBounds & set_max_services(size_t services)
  {
    max_services = services;
    return *this;
  }

  /// Set the maximum number of clients.
  /* \param[in] clients Maximum number of clients.
   * \return Reference to this object, for method chaining.
   */
  ObjectPoolBounds & set_max_clients(size_t clients)
  {
    max_clients = clients;
    return *this;
  }

  /// Set the maximum number of guard conditions.
  /* \param[in] guard conditions Maximum number of guard conditions.
   * \return Reference to this object, for method chaining.
   */
  ObjectPoolBounds & set_max_guard_conditions(size_t guard_conditions)
  {
    max_guard_conditions = guard_conditions;
    return *this;
  }

  /// Set the maximum number of executables.
  /* \param[in] executables Maximum number of executables.
   * \return Reference to this object, for method chaining.
   */
  ObjectPoolBounds & set_max_executables(size_t executables)
  {
    max_executables = executables;
    return *this;
  }

  /// Set the maximum memory pool size.
  /* \param[in] executables Maximum memory pool size.
   * \return Reference to this object, for method chaining.
   */
  ObjectPoolBounds & set_memory_pool_size(size_t pool)
  {
    pool_size = pool;
    return *this;
  }
};


/// Static memory allocation alternative to the default memory strategy.
/* The name is a bit of a misnomer. The memory managed by this class is actually allocated
 * dynamically in the constructor, but no subsequent accesses to the class (besides the destructor)
 * allocate or free memory.
 * StaticMemoryStrategy puts a hard limit on the number of subscriptions, etc. that can be executed
 * in one iteration of `Executor::spin`. Thus it allows for memory allocation optimization for
 * situations where a limit on the number of such entities is known.
 */
class StaticMemoryStrategy : public memory_strategy::MemoryStrategy
{
public:
  /// Default constructor.
  // \param[in] bounds Representation of the limits on memory managed by this class.
  StaticMemoryStrategy(ObjectPoolBounds bounds = ObjectPoolBounds())
  : bounds_(bounds), memory_pool_(nullptr), subscription_pool_(nullptr),
    service_pool_(nullptr), guard_condition_pool_(nullptr), executable_pool_(nullptr)
  {
    if (bounds_.pool_size) {
      memory_pool_ = new void *[bounds_.pool_size];
      memset(memory_pool_, 0, bounds_.pool_size * sizeof(void *));
    }

    if (bounds_.max_subscriptions) {
      subscription_pool_ = new void *[bounds_.max_subscriptions];
      memset(subscription_pool_, 0, bounds_.max_subscriptions * sizeof(void *));
    }

    if (bounds_.max_services) {
      service_pool_ = new void *[bounds_.max_services];
      memset(service_pool_, 0, bounds_.max_services * sizeof(void *));
    }

    if (bounds_.max_clients) {
      client_pool_ = new void *[bounds_.max_clients];
      memset(client_pool_, 0, bounds_.max_clients * sizeof(void *));
    }

    if (bounds_.max_guard_conditions) {
      guard_condition_pool_ = new void *[bounds_.max_guard_conditions];
      memset(guard_condition_pool_, 0, bounds_.max_guard_conditions * sizeof(void *));
    }

    if (bounds_.max_executables) {
      executable_pool_ = new executor::AnyExecutable::SharedPtr[bounds_.max_executables];
    }

    for (size_t i = 0; i < bounds_.max_executables; ++i) {
      executable_pool_[i] = std::make_shared<executor::AnyExecutable>();
    }

    pool_seq_ = 0;
    exec_seq_ = 0;

    // Reserve pool_size_ buckets in the memory map.
    memory_map_.reserve(bounds_.pool_size);
    for (size_t i = 0; i < bounds_.pool_size; ++i) {
      memory_map_[memory_pool_[i]] = 0;
    }
  }

  /// Default destructor. Free all allocated memory.
  ~StaticMemoryStrategy()
  {
    if (bounds_.pool_size) {
      delete[] memory_pool_;
    }
    if (bounds_.max_subscriptions) {
      delete[] subscription_pool_;
    }
    if (bounds_.max_services) {
      delete[] service_pool_;
    }
    if (bounds_.max_clients) {
      delete[] client_pool_;
    }
    if (bounds_.max_guard_conditions) {
      delete[] guard_condition_pool_;
    }
  }

  /// Borrow handles by returning a pointer to the preallocated object pool for the specified type.
  /* \param[in] The type of entity that this function is requesting for.
   * \param[in] The number of handles to borrow.
   * \return Pointer to the allocated handles.
   */
  void ** borrow_handles(HandleType type, size_t number_of_handles)
  {
    switch (type) {
      case HandleType::subscription_handle:
        if (number_of_handles > bounds_.max_subscriptions) {
          throw std::runtime_error("Requested size exceeded maximum subscriptions.");
        }

        return subscription_pool_;
      case HandleType::service_handle:
        if (number_of_handles > bounds_.max_services) {
          throw std::runtime_error("Requested size exceeded maximum services.");
        }

        return service_pool_;
      case HandleType::client_handle:
        if (number_of_handles > bounds_.max_clients) {
          throw std::runtime_error("Requested size exceeded maximum clients.");
        }

        return client_pool_;
      case HandleType::guard_condition_handle:
        if (number_of_handles > bounds_.max_guard_conditions) {
          throw std::runtime_error("Requested size exceeded maximum guard_conditions.");
        }

        return guard_condition_pool_;
      default:
        break;
    }
    throw std::runtime_error("Unrecognized enum, could not borrow handle memory.");
  }

  /// Return the borrowed handles by clearing the object pool for the correspondign type.
  /* \param[in] The type of entity that this function is returning.
   * \param[in] Pointer to the handles returned.
   */
  void return_handles(HandleType type, void ** handles)
  {
    (void)handles;
    switch (type) {
      case HandleType::subscription_handle:
        if (bounds_.max_subscriptions) {
          memset(subscription_pool_, 0, bounds_.max_subscriptions * sizeof(void *));
        }
        break;
      case HandleType::service_handle:
        if (bounds_.max_services) {
          memset(service_pool_, 0, bounds_.max_services * sizeof(void *));
        }
        break;
      case HandleType::client_handle:
        if (bounds_.max_clients) {
          memset(client_pool_, 0, bounds_.max_clients * sizeof(void *));
        }
        break;
      case HandleType::guard_condition_handle:
        if (bounds_.max_guard_conditions) {
          memset(guard_condition_pool_, 0, bounds_.max_guard_conditions * sizeof(void *));
        }
        break;
      default:
        throw std::runtime_error("Unrecognized enum, could not return handle memory.");
    }
  }

  /// Instantiate the next executable by borrowing space from the preallocated executables pool.
  // \return Shared pointer to the executable.
  executor::AnyExecutable::SharedPtr instantiate_next_executable()
  {
    if (exec_seq_ >= bounds_.max_executables) {
      // wrap around (ring buffer logic)
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

  /// General allocate: reserve space in the memory pool reserved by this function.
  /* \param[in] size Number of bytes to allocate.
   * \return Pointer to the allocated chunk of memory.
   */
  void * alloc(size_t size)
  {
    // Extremely naive static allocation strategy
    // Keep track of block size at a given pointer
    if (pool_seq_ + size > bounds_.pool_size) {
      // Start at 0
      pool_seq_ = 0;
    }
    void * ptr = memory_pool_[pool_seq_];
    if (memory_map_.count(ptr) == 0) {
      // We expect to have the state for all blocks pre-mapped into memory_map_
      throw std::runtime_error("Unexpected pointer in StaticMemoryStrategy::alloc.");
    }
    memory_map_[ptr] = size;
    size_t prev_pool_seq = pool_seq_;
    pool_seq_ += size;
    return memory_pool_[prev_pool_seq];
  }

  /// Release the allocated memory in the memory pool.
  /* \param[in] Pointer to deallocate.
   */
  void free(void * ptr)
  {
    if (memory_map_.count(ptr) == 0) {
      // We expect to have the state for all blocks pre-mapped into memory_map_
      throw std::runtime_error("Unexpected pointer in StaticMemoryStrategy::free.");
    }

    memset(ptr, 0, memory_map_[ptr]);
  }

private:
  ObjectPoolBounds bounds_;

  void ** memory_pool_;
  void ** subscription_pool_;
  void ** service_pool_;
  void ** client_pool_;
  void ** guard_condition_pool_;
  executor::AnyExecutable::SharedPtr * executable_pool_;

  size_t pool_seq_;
  size_t exec_seq_;

  std::unordered_map<void *, size_t> memory_map_;
};

}  /* static_memory_strategy */

}  /* memory_strategies */

}  /* rclcpp */

#endif
