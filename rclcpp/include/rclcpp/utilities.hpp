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
#include <functional>

#include "rclcpp/visibility_control.hpp"

#include "rcl/guard_condition.h"
#include "rcl/wait.h"

#include "rmw/macros.h"
#include "rmw/rmw.h"

#ifdef ANDROID
#include <string>
#include <sstream>

namespace std
{
template<typename T>
std::string to_string(T value)
{
  std::ostringstream os;
  os << value;
  return os.str();
}
}
#endif

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

/// Register a function to be called when shutdown is called.
/* Calling the callbacks is the last thing shutdown() does. */
RCLCPP_PUBLIC
void
on_shutdown(std::function<void(void)> callback);

/// Get a handle to the rmw guard condition that manages the signal handler.
/**
 * The first time that this function is called for a given waitset a new guard
 * condition will be created and returned; thereafter the same guard condition
 * will be returned for the same waitset. This mechanism is designed to ensure
 * that the same guard condition is not reused across waitsets (e.g., when
 * using multiple executors in the same process). Will throw an exception if
 * initialization of the guard condition fails.
 * \param[waitset] waitset Pointer to the rcl_wait_set_t that will be using the
 * resulting guard condition.
 * \return Pointer to the guard condition.
 */
RCLCPP_PUBLIC
rcl_guard_condition_t *
get_sigint_guard_condition(rcl_wait_set_t * waitset);

/// Release the previously allocated guard condition that manages the signal handler.
/**
 * If you previously called get_sigint_guard_condition() for a given waitset
 * to get a sigint guard condition, then you should call release_sigint_guard_condition()
 * when you're done, to free that condition.  Will throw an exception if
 * get_sigint_guard_condition() wasn't previously called for the given waitset.
 * \param[waitset] waitset Pointer to the rcl_wait_set_t that was using the
 * resulting guard condition.
 */
RCLCPP_PUBLIC
void
release_sigint_guard_condition(rcl_wait_set_t * waitset);

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
