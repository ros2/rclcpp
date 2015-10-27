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

#include <memory>
#include <vector>

#include <rclcpp/any_executable.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/node.hpp>

namespace rclcpp
{

namespace memory_strategy
{

/// Delegate for handling memory allocations while the Executor is executing.
/**
 * By default, the memory strategy dynamically allocates memory for structures that come in from
 * the rmw implementation after the executor waits for work, based on the number of entities that
 * come through.
 */
class MemoryStrategy
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(MemoryStrategy);
  using WeakNodeVector = std::vector<std::weak_ptr<rclcpp::node::Node>>>;

  // return the new number of subscribers
  virtual size_t fill_subscriber_handles(void ** ptr) = 0;

  // return the new number of services
  virtual size_t fill_service_handles(void ** ptr) = 0;

  // return the new number of clients
  virtual size_t fill_client_handles(void ** ptr) = 0;

  virtual void clear_active_entities() = 0;

  virtual void clear_handles() = 0;
  virtual bool collect_entities(const WeakNodeVector & weak_nodes) = 0;

  /// Provide a newly initialized AnyExecutable object.
  // \return Shared pointer to the fresh executable.
  virtual executor::AnyExecutable::SharedPtr instantiate_next_executable() = 0;

  virtual void
  get_next_subscription(executor::AnyExecutable::SharedPtr any_exec,
    const WeakNodeVector & weak_nodes) = 0;

  virtual void
  get_next_service(executor::AnyExecutable::SharedPtr any_exec,
    const WeakNodeVector & weak_nodes) = 0;

  virtual void
  get_next_client(executor::AnyExecutable::SharedPtr any_exec,
    const WeakNodeVector & weak_nodes) = 0;


};

}  /* memory_strategy */

}  /* rclcpp */

#endif
