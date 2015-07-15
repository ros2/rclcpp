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

#ifndef RCLCPP_RCLCPP_MEMORY_STRATEGY_HPP_
#define RCLCPP_RCLCPP_MEMORY_STRATEGY_HPP_
#include <rclcpp/any_executable.hpp>

namespace rclcpp
{

// TODO move HandleType somewhere where it makes sense
enum class HandleType {subscriber_handle, service_handle, client_handle, guard_condition_handle};

namespace executor
{
class Executor;
}

namespace memory_strategy
{

class MemoryStrategy
{

  friend class executor::Executor;

public:
  RCLCPP_MAKE_SHARED_DEFINITIONS(MemoryStrategy);
  virtual void ** borrow_handles(HandleType type, size_t number_of_handles)
  {
    return static_cast<void **>(alloc(sizeof(void *) * number_of_handles));
  }

  virtual void return_handles(HandleType type, void ** handles)
  {
    this->free(handles);
  }

  virtual executor::AnyExecutable::SharedPtr instantiate_next_executable()
  {
    return executor::AnyExecutable::SharedPtr(new executor::AnyExecutable);
  }

  virtual void * alloc(size_t size)
  {
    return std::malloc(size);
  }

  virtual void free(void * ptr)
  {
    return std::free(ptr);
  }
};


MemoryStrategy::SharedPtr create_default_strategy()
{
  return std::make_shared<MemoryStrategy>(MemoryStrategy());
}

}  /* memory_strategy */

}  /* rclcpp */

#endif
