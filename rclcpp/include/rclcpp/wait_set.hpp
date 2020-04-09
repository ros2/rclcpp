// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__WAIT_SET_HPP_
#define RCLCPP__WAIT_SET_HPP_

#include <memory>

#include "rcl/wait.h"

#include "rclcpp/guard_condition.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/wait_set_policies/dynamic_storage.hpp"
#include "rclcpp/wait_set_policies/sequential_synchronization.hpp"
#include "rclcpp/wait_set_policies/static_storage.hpp"
#include "rclcpp/wait_set_policies/thread_safe_synchronization.hpp"
#include "rclcpp/wait_set_template.hpp"

namespace rclcpp
{

/// Most common user configuration of a WaitSet, which is dynamic but not thread-safe.
/**
 * This wait set allows you to add and remove items dynamically, and it will
 * automatically remove items that are let out of scope each time wait() or
 * prune_destroyed_entities() is called.
 *
 * It will not, however, provide thread-safety for adding and removing entities
 * while waiting.
 *
 * \sa rclcpp::WaitSetTemplate for API documentation
 */
using WaitSet = rclcpp::WaitSetTemplate<
  rclcpp::wait_set_policies::SequentialSynchronization,
  rclcpp::wait_set_policies::DynamicStorage
>;

/// WaitSet configuration which does not allow changes after construction.
/**
 * This wait set requires that you specify all entities at construction, and
 * prevents you from calling the typical add and remove functions.
 * It also requires that you specify how many of each item there will be as a
 * template argument.
 *
 * It will share ownership of the entities until destroyed, therefore it will
 * prevent the destruction of entities so long as the wait set exists, even if
 * the user lets their copy of the shared pointer to the entity go out of scope.
 *
 * Since the wait set cannot be mutated, it does not need to be thread-safe.
 *
 * \sa rclcpp::WaitSetTemplate for API documentation
 */
template<
  std::size_t NumberOfSubscriptions,
  std::size_t NumberOfGuardCondtions,
  std::size_t NumberOfTimers,
  std::size_t NumberOfClients,
  std::size_t NumberOfServices,
  std::size_t NumberOfWaitables
>
using StaticWaitSet = rclcpp::WaitSetTemplate<
  rclcpp::wait_set_policies::SequentialSynchronization,
  rclcpp::wait_set_policies::StaticStorage<
    NumberOfSubscriptions,
    NumberOfGuardCondtions,
    NumberOfTimers,
    NumberOfClients,
    NumberOfServices,
    NumberOfWaitables
  >
>;

/// Like WaitSet, this configuration is dynamic, but is also thread-safe.
/**
 * This wait set allows you to add and remove items dynamically, and it will
 * automatically remove items that are let out of scope each time wait() or
 * prune_destroyed_entities() is called.
 *
 * It will also ensure that adding and removing items explicitly from the
 * wait set is done in a thread-safe way, protecting against concurrent add and
 * deletes, as well as add and deletes during a wait().
 * This thread-safety comes at some overhead and the use of thread
 * synchronization primitives.
 *
 * \sa rclcpp::WaitSetTemplate for API documentation
 */
using ThreadSafeWaitSet = rclcpp::WaitSetTemplate<
  rclcpp::wait_set_policies::ThreadSafeSynchronization,
  rclcpp::wait_set_policies::DynamicStorage
>;

}  // namespace rclcpp

#endif  // RCLCPP__WAIT_SET_HPP_
