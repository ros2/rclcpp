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

class StaticMemoryStrategy : public memory_strategy::MemoryStrategy
{

public:
  StaticMemoryStrategy()
  {
    memset(_memory_pool, 0, _pool_size);
    memset(_subscriber_pool, 0, _max_subscribers);
    memset(_service_pool, 0, _max_services);
    memset(_client_pool, 0, _max_clients);
    memset(_guard_condition_pool, 0, _max_guard_conditions);
    _pool_seq = 0;
    _exec_seq = 0;

    // Reserve _pool_size buckets in the memory map.
    _memory_map.reserve(_pool_size);
    for (size_t i = 0; i < _pool_size; ++i) {
      _memory_map[_memory_pool[i]] = 0;
    }

    for (size_t i = 0; i < _max_executables; ++i) {
      _executable_pool[i] = std::make_shared<executor::AnyExecutable>(executor::AnyExecutable());
    }
  }

  void ** borrow_handles(HandleType type, size_t number_of_handles)
  {
    switch (type) {
      case HandleType::subscriber_handle:
        return _subscriber_pool;
      case HandleType::service_handle:
        return _service_pool;
      case HandleType::client_handle:
        return _client_pool;
      case HandleType::guard_condition_handle:
        return _guard_condition_pool;
      default:
        break;
    }
    throw std::runtime_error("Unrecognized enum, could not borrow handle memory.");
  }

  void return_handles(HandleType type, void ** handles)
  {
    switch (type) {
      case HandleType::subscriber_handle:
        memset(_subscriber_pool, 0, _max_subscribers);
        break;
      case HandleType::service_handle:
        memset(_service_pool, 0, _max_services);
        break;
      case HandleType::client_handle:
        memset(_client_pool, 0, _max_clients);
        break;
      case HandleType::guard_condition_handle:
        memset(_guard_condition_pool, 0, _max_guard_conditions);
        break;
      default:
        throw std::runtime_error("Unrecognized enum, could not return handle memory.");
        break;
    }
  }

  executor::AnyExecutable::SharedPtr instantiate_next_executable()
  {
    if (_exec_seq >= _max_executables) {
      // wrap around
      _exec_seq = 0;
    }
    size_t prev_exec_seq = _exec_seq;
    ++_exec_seq;

    return _executable_pool[prev_exec_seq];
  }

  void * alloc(size_t size)
  {
    // Extremely naive static allocation strategy
    // Keep track of block size at a given pointer
    if (_pool_seq + size > _pool_size) {
      // Start at 0
      _pool_seq = 0;
    }
    void * ptr = _memory_pool[_pool_seq];
    if (_memory_map.count(ptr) == 0) {
      // We expect to have the state for all blocks pre-mapped into _memory_map
      throw std::runtime_error("Unexpected pointer in rcl_malloc.");
    }
    _memory_map[ptr] = size;
    size_t prev_pool_seq = _pool_seq;
    _pool_seq += size;
    return _memory_pool[prev_pool_seq];
  }

  void free(void * ptr)
  {
    if (_memory_map.count(ptr) == 0) {
      // We expect to have the state for all blocks pre-mapped into _memory_map
      throw std::runtime_error("Unexpected pointer in rcl_free.");
    }

    memset(ptr, 0, _memory_map[ptr]);
  }

protected:
private:
  static const size_t _pool_size = 1024;
  static const size_t _max_subscribers = 10;
  static const size_t _max_services = 5;
  static const size_t _max_clients = 10;
  static const size_t _max_guard_conditions = 50;
  static const size_t _max_executables = 1;

  void * _memory_pool[_pool_size];
  void * _subscriber_pool[_max_subscribers];
  void * _service_pool[_max_services];
  void * _client_pool[_max_clients];
  void * _guard_condition_pool[_max_guard_conditions];
  executor::AnyExecutable::SharedPtr _executable_pool[_max_executables];

  size_t _pool_seq;
  size_t _exec_seq;

  std::unordered_map<void *, size_t> _memory_map;
};

}  /* static_memory_strategy */

}  /* memory_strategies */

}  /* rclcpp */

#endif
