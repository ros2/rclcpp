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

#ifndef RCLCPP__LOGGING_MUTEX_HPP_
#define RCLCPP__LOGGING_MUTEX_HPP_

#include <memory>
#include <mutex>

#include "rclcpp/visibility_control.hpp"

/// Global logging mutex
/**
 * This mutex is locked in the following situations:
 *   - In initialization/destruction of contexts.
 *   - In initialization/destruction of nodes.
 *   - In the rcl logging output handler installed by rclcpp,
 *     i.e.: in all calls to the logger macros, including RCUTILS_* ones.
 */
// Implementation detail:
// A shared pointer to the mutex is used, so that objects that need to use
// it at destruction time can hold it alive.
// In that way, a destruction ordering problem between static objects is avoided.
RCLCPP_LOCAL
std::shared_ptr<std::recursive_mutex>
get_global_logging_mutex();

#endif  // RCLCPP__LOGGING_MUTEX_HPP_
