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

#ifndef RCLCPP_RCLCPP_DYNAMIC_MEMORY_STRATEGY_HPP_
#define RCLCPP_RCLCPP_DYNAMIC_MEMORY_STRATEGY_HPP_

#include <rclcpp/memory_strategy.hpp>

namespace rclcpp
{

namespace memory_strategies
{

namespace dynamic_memory_strategy
{

class DynamicMemoryStrategy : public memory_strategy::MemoryStrategy
{

public:

  DynamicMemoryStrategy() {}

  void** borrow_handles(HandleType /*type*/, size_t number_of_handles)
  {
    return static_cast<void**>(rcl_malloc(sizeof(void *) * number_of_handles));
  }

  void return_handles(HandleType /*type*/, void** handles)
  {
    this->rcl_free(handles);
  }

  void *rcl_malloc(size_t size)
  {
    return std::malloc(size);
  }

  void rcl_free(void *ptr)
  {
    return std::free(ptr);
  }

protected:


private:

};

}

}  /* memory_strategies */

}  /* rclcpp */

#endif
