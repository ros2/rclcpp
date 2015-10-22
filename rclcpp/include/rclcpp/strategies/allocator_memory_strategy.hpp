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

#ifndef RCLCPP_RCLCPP_ALLOCATOR_MEMORY_STRATEGY_HPP_
#define RCLCPP_RCLCPP_ALLOCATOR_MEMORY_STRATEGY_HPP_

#include <rclcpp/allocator/allocator_common.hpp>
#include <rclcpp/memory_strategy.hpp>

namespace rclcpp
{


namespace memory_strategies
{

namespace allocator_memory_strategy
{

/// Delegate for handling memory allocations while the Executor is executing.
/**
 * By default, the memory strategy dynamically allocates memory for structures that come in from
 * the rmw implementation after the executor waits for work, based on the number of entities that
 * come through.
 */
template<typename Alloc>
class AllocatorMemoryStrategy : public memory_strategy::MemoryStrategy
{

public:
  RCLCPP_SMART_PTR_DEFINITIONS(AllocatorMemoryStrategy<Alloc>);

  using ExecAllocTraits = allocator::AllocRebind<executor::AnyExecutable, Alloc>;
  using ExecAlloc = typename ExecAllocTraits::allocator_type;
  using ExecDeleter = allocator::Deleter<ExecAlloc, executor::AnyExecutable>;
  using VoidAllocTraits = typename allocator::AllocRebind<void *, Alloc>;
  using VoidAlloc = typename VoidAllocTraits::allocator_type;

  AllocatorMemoryStrategy(std::shared_ptr<Alloc> allocator)
  {
    executable_allocator_ = std::make_shared<ExecAlloc>(*allocator.get());
    allocator_ = std::make_shared<VoidAlloc>(*allocator.get());
  }

  /// Borrow memory for storing data for subscriptions, services, clients, or guard conditions.
  /**
   * The default implementation stores std::vectors for each handle type and resizes the vectors
   * as necessary based on the requested number of handles.
   * \param[in] The type of entity that this function is requesting for.
   * \param[in] The number of handles to borrow.
   * \return Pointer to the allocated handles.
   */
  virtual void ** borrow_handles(HandleType type, size_t number_of_handles)
  {
    switch (type) {
      case HandleType::subscription_handle:
        if (subscription_handles.size() < number_of_handles) {
          subscription_handles.resize(number_of_handles, 0);
        }
        return static_cast<void **>(subscription_handles.data());
      case HandleType::service_handle:
        if (service_handles.size() < number_of_handles) {
          service_handles.resize(number_of_handles, 0);
        }
        return static_cast<void **>(service_handles.data());
      case HandleType::client_handle:
        if (client_handles.size() < number_of_handles) {
          client_handles.resize(number_of_handles, 0);
        }
        return static_cast<void **>(client_handles.data());
      case HandleType::guard_condition_handle:
        if (number_of_handles > 2) {
          throw std::runtime_error("Too many guard condition handles requested!");
        }
        return guard_cond_handles.data();
      default:
        throw std::runtime_error("Unknown HandleType " + std::to_string(static_cast<int>(type)) +
                ", could not borrow handle memory.");
    }
  }

  /// Return the memory borrowed in borrow_handles.
  /**
   * return_handles should always mirror the way memory was borrowed in borrow_handles.
   * \param[in] The type of entity that this function is returning.
   * \param[in] Pointer to the handles returned.
   */
  virtual void return_handles(HandleType type, void ** handles)
  {
    switch (type) {
      case HandleType::subscription_handle:
        if (handles != subscription_handles.data()) {
          throw std::runtime_error(
                  "tried to return memory that isn't handled by this AllocatorMemoryStrategy");
        }
        memset(handles, 0, subscription_handles.size());
        break;
      case HandleType::service_handle:
        if (handles != service_handles.data()) {
          throw std::runtime_error(
                  "tried to return memory that isn't handled by this AllocatorMemoryStrategy");
        }
        memset(handles, 0, service_handles.size());
        break;
      case HandleType::client_handle:
        if (handles != client_handles.data()) {
          throw std::runtime_error(
                  "tried to return memory that isn't handled by this AllocatorMemoryStrategy");
        }
        memset(handles, 0, client_handles.size());
        break;
      case HandleType::guard_condition_handle:
        if (handles != guard_cond_handles.data()) {
          throw std::runtime_error(
                  "tried to return memory that isn't handled by this AllocatorMemoryStrategy");
        }
        guard_cond_handles.fill(0);
        break;
      default:
        throw std::runtime_error("Unknown HandleType " + std::to_string(static_cast<int>(type)) +
                ", could not borrow handle memory.");
    }
  }

  /// Provide a newly initialized AnyExecutable object.
  // \return Shared pointer to the fresh executable.
  virtual executor::AnyExecutable::SharedPtr instantiate_next_executable()
  {
    //return std::make_shared<executor::AnyExecutable>();
    auto ptr = ExecAllocTraits::allocate(*executable_allocator_.get(), 1);
    ExecAllocTraits::construct(*executable_allocator_.get(), ptr);
    return std::shared_ptr<executor::AnyExecutable>(ptr, executable_deleter_);
  }

  /// Implementation of a general-purpose allocation function.
  /**
   * \param[in] size Number of bytes to allocate.
   * \return Pointer to the allocated chunk of memory.
   */
  virtual void * alloc(size_t size)
  {
    if (size == 0) {
      return NULL;
    }
    auto ptr = VoidAllocTraits::allocate(*allocator_.get(), size);
    alloc_map[ptr] = size;
    return ptr;
  }

  /// Implementation of a general-purpose free.
  /**
   * \param[in] Pointer to deallocate.
   */
  virtual void free(void * ptr)
  {
    if (alloc_map.count(ptr) == 0) {
      // do nothing, the pointer is not in the alloc'd map
      return;
    }
    VoidAllocTraits::deallocate(*allocator_.get(), &ptr, alloc_map[ptr]);
  }

  template<typename U>
  using VectorRebind = typename std::allocator_traits<Alloc>::template rebind_alloc<U>;

  std::vector<rclcpp::subscription::SubscriptionBase::SharedPtr,
  VectorRebind<rclcpp::subscription::SubscriptionBase::SharedPtr>> subs;
  std::vector<rclcpp::service::ServiceBase::SharedPtr,
  VectorRebind<rclcpp::service::ServiceBase::SharedPtr>> services;
  std::vector<rclcpp::client::ClientBase::SharedPtr,
  VectorRebind<rclcpp::client::ClientBase::SharedPtr>> clients;

  std::vector<void *, VoidAlloc> subscription_handles;
  std::vector<void *, VoidAlloc> service_handles;
  std::vector<void *, VoidAlloc> client_handles;
  std::array<void *, 2> guard_cond_handles;

  std::unordered_map<void *, size_t> alloc_map;

private:
  std::shared_ptr<ExecAlloc> executable_allocator_;
  ExecDeleter executable_deleter_;
  std::shared_ptr<VoidAlloc> allocator_;
};

}  /* allocator_memory_strategy */
}  /* memory_strategies */

}  /* rclcpp */

#endif
