// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__UTILITIES_HPP_
#define RCLCPP__UTILITIES_HPP_

#include <chrono>

#include "rclcpp/visibility_control.hpp"
#include "rmw/macros.h"
#include "rmw/rmw.h"

namespace rclcpp
{
namespace utilities
{

/// Initialize communications via the rmw implementation and set up a global signal handler.
/**
 * \param[in] argc Number of arguments.
 * \param[in] argv Argument vector. Will eventually be used for passing options to rclcpp.
 */
RCLCPP_PUBLIC
void
init(int argc, char * argv[]);

/// Check rclcpp's status.
// \return True if SIGINT hasn't fired yet, false otherwise.
RCLCPP_PUBLIC
bool
ok();

/// Notify the signal handler and rmw that rclcpp is shutting down.
RCLCPP_PUBLIC
void
shutdown();


/// Get a handle to the rmw guard condition that manages the signal handler.
RCLCPP_PUBLIC
rmw_guard_condition_t *
get_global_sigint_guard_condition();

/// Use the global condition variable to block for the specified amount of time.
/**
 * \param[in] nanoseconds A std::chrono::duration representing how long to sleep for.
 * \return True if the condition variable did not timeout.
 */
RCLCPP_PUBLIC
bool
sleep_for(const std::chrono::nanoseconds & nanoseconds);

}  // namespace utilities
}  // namespace rclcpp

#endif  // RCLCPP__UTILITIES_HPP_
