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
#include <limits>
#include <vector>

#include "rclcpp/context.hpp"
#include "rclcpp/init_options.hpp"
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
/// Initialize communications via the rmw implementation and set up a global signal handler.
/**
 * \param[in] argc Number of arguments.
 * \param[in] argv Argument array which may contain arguments intended for ROS.
 * \param[in] init_options Initialization options for rclcpp and underlying layers.
 */
RCLCPP_PUBLIC
void
init(int argc, char const * const argv[], const InitOptions & init_options = InitOptions());

/// Initialize communications via the rmw implementation for a local context.
/**
 * Unlike rclcpp::init(), this function does not initialize signal handling.
 * Use rclcpp::install_signal_handlers() to setup signal handling manually.
 *
 * \param[in] argc Number of arguments.
 * \param[in] argv Argument array which may contain arguments intended for ROS.
 * \param[out] context_out Argument array which may contain arguments intended for ROS.
 * \param[in] init_options Initialization options for rclcpp and underlying layers.
 */
RCLCPP_PUBLIC
void
init_local(
  int argc,
  char const * const argv[],
  rclcpp::Context::SharedPtr context_out,
  const InitOptions & init_options = InitOptions());

/// Convenience version of init_local with a templated Context type.
template<typename ContextT = rclcpp::Context, typename... Args>
std::shared_ptr<ContextT>
init_local(
  int argc,
  char const * const argv[],
  Args &&... args,
  const InitOptions & init_options = InitOptions())
{
  auto result = std::make_shared<ContextT>(std::forward<Args>(args)...);
  init_local(argc, argv, result, init_options);
  return result;
}

/// Install the global signal handler for rclcpp.
/**
 * This function should only need to be run one time per process, and is
 * implicitly run by rclcpp::init(), and therefore this function does not need
 * to be run manually if rclcpp::init() has already been run.
 *
 * This function is thread-safe.
 *
 * \return true if signal handler was installed by this function, false if already installed.
 */
RCLCPP_PUBLIC
bool
install_signal_handlers();

/// Return true if the signal handlers are installed, otherwise false.
RCLCPP_PUBLIC
bool
signal_handlers_installed();

/// Uninstall the global signal handler for rclcpp.
/**
 * This function does not necessarily need to be called, but can be used to
 * undo what rclcpp::install_signal_handlers() or rclcpp::init() do with
 * respect to signal handling.
 * If you choose to use it, this function only needs to be run one time, and is
 * implicitly run by rclcpp::shutdown(), and therefore this function does not
 * need to be run manually if rclcpp::shutdown() has already been run.
 *
 * This function is thread-safe.
 *
 * \return true if signal handler was uninstalled by this function, false if was not installed.
 */
RCLCPP_PUBLIC
bool
uninstall_signal_handlers();

/// Initialize communications via the rmw implementation and set up a global signal handler.
/**
 * Additionally removes ROS-specific arguments from the argument vector.
 * \param[in] argc Number of arguments.
 * \param[in] argv Argument vector.
 * \param[in] init_options Initialization options for rclcpp and underlying layers.
 * \returns Members of the argument vector that are not ROS arguments.
 */
RCLCPP_PUBLIC
std::vector<std::string>
init_and_remove_ros_arguments(
  int argc,
  char const * const argv[],
  const InitOptions & init_options = InitOptions());

/// Remove ROS-specific arguments from argument vector.
/**
 * Some arguments may not have been intended as ROS arguments.
 * This function populates the arguments in a vector.
 * Since the first argument is always assumed to be a process name, the vector
 * will always contain the process name.
 *
 * \param[in] argc Number of arguments.
 * \param[in] argv Argument vector.
 * \returns Members of the argument vector that are not ROS arguments.
 */
RCLCPP_PUBLIC
std::vector<std::string>
remove_ros_arguments(int argc, char const * const argv[]);

/// Check rclcpp's status.
/**
 * This may return false for a context which has been shutdown, or for a
 * context that was shutdown due to SIGINT being received by the rclcpp signal
 * handler.
 *
 * If nullptr is given for the context, then the global context is used, i.e.
 * the context initialized by rclcpp::init().
 *
 * \param[in] context to check for shutdown of
 * \return true if shutdown has been called, false otherwise
 */
RCLCPP_PUBLIC
bool
ok(rclcpp::Context::SharedPtr context = nullptr);

/// Return true if init() has already been called for the given context.
/**
 * If nullptr is given for the context, then the global context is used, i.e.
 * the context initialized by rclcpp::init().
 *
 * Deprecated, as it is no longer different from rcl_ok().
 *
 * \param[in] context to check for shutdown of
 * \return true if the context is initialized, and false otherwise
 */
RCLCPP_PUBLIC
bool
is_initialized(rclcpp::Context::SharedPtr context = nullptr);

/// Shutdown rclcpp context, invalidating it for derived entities.
/**
 * If nullptr is given for the context, then the global context is used, i.e.
 * the context initialized by rclcpp::init().
 *
 * This will also cause the "on_shutdown" callbacks to be called.
 *
 * \param[in] context to be shutdown
 */
RCLCPP_PUBLIC
void
shutdown(rclcpp::Context::SharedPtr context = nullptr);

/// Register a function to be called when shutdown is called on the context.
/**
 * Calling the callbacks is the last thing shutdown() does.
 *
 * The callbacks receive a pointer to the rclcpp::Context with which they were
 * associated.
 *
 * If nullptr is given for the context, then the global context is used, i.e.
 * the context initialized by rclcpp::init().
 *
 * \param[in] callback to be called when the given context is shutdown
 * \param[in] context with which to associate the context
 */
RCLCPP_PUBLIC
void
on_shutdown(
  std::function<void(rclcpp::Context::SharedPtr)> callback,
  rclcpp::Context::SharedPtr context = nullptr);

/// Get a handle to the rmw guard condition that manages the signal handler.
/**
 * The first time that this function is called for a given wait set a new guard
 * condition will be created and returned; thereafter the same guard condition
 * will be returned for the same wait set. This mechanism is designed to ensure
 * that the same guard condition is not reused across wait sets (e.g., when
 * using multiple executors in the same process). Will throw an exception if
 * initialization of the guard condition fails.
 *
 * \param[in] wait_set Pointer to the rcl_wait_set_t that will be using the
 *   resulting guard condition.
 * \param[in] context to check for shutdown of
 * \return Pointer to the guard condition.
 */
RCLCPP_PUBLIC
rcl_guard_condition_t *
get_sigint_guard_condition(rcl_wait_set_t * wait_set, rclcpp::Context::SharedPtr context);

/// Release the previously allocated guard condition that manages the signal handler.
/**
 * If you previously called get_sigint_guard_condition() for a given wait set
 * to get a sigint guard condition, then you should call
 * release_sigint_guard_condition() when you're done, to free that condition.
 * Will throw an exception if get_sigint_guard_condition() wasn't previously
 * called for the given wait set.
 *
 * If nullptr is given for the context, then the global context is used, i.e.
 * the context initialized by rclcpp::init().
 *
 * \param[in] wait_set Pointer to the rcl_wait_set_t that was using the
 *   resulting guard condition.
 */
RCLCPP_PUBLIC
void
release_sigint_guard_condition(rcl_wait_set_t * wait_set);

/// Use the global condition variable to block for the specified amount of time.
/**
 * This function can be interrupted early if the associated context becomes
 * invalid due to rclcpp::shutdown() or the signal handler.
 *
 * If nullptr is given for the context, then the global context is used, i.e.
 * the context initialized by rclcpp::init().
 *
 * \param[in] nanoseconds A std::chrono::duration representing how long to sleep for.
 * \param[in] context which may interrupt this sleep
 * \return True if the condition variable did not timeout.
 */
RCLCPP_PUBLIC
bool
sleep_for(
  const std::chrono::nanoseconds & nanoseconds,
  rclcpp::Context::SharedPtr context = nullptr);

/// Safely check if addition will overflow.
/**
 * The type of the operands, T, should have defined
 * std::numeric_limits<T>::max(), `>`, `<` and `-` operators.
 *
 * \param[in] x is the first addend.
 * \param[in] y is the second addend.
 * \tparam T is type of the operands.
 * \return True if the x + y sum is greater than T::max value.
 */
template<typename T>
bool
add_will_overflow(const T x, const T y)
{
  return (y > 0) && (x > (std::numeric_limits<T>::max() - y));
}

/// Safely check if addition will underflow.
/**
 * The type of the operands, T, should have defined
 * std::numeric_limits<T>::min(), `>`, `<` and `-` operators.
 *
 * \param[in] x is the first addend.
 * \param[in] y is the second addend.
 * \tparam T is type of the operands.
 * \return True if the x + y sum is less than T::min value.
 */
template<typename T>
bool
add_will_underflow(const T x, const T y)
{
  return (y < 0) && (x < (std::numeric_limits<T>::min() - y));
}

/// Safely check if subtraction will overflow.
/**
 * The type of the operands, T, should have defined
 * std::numeric_limits<T>::max(), `>`, `<` and `+` operators.
 *
 * \param[in] x is the minuend.
 * \param[in] y is the subtrahend.
 * \tparam T is type of the operands.
 * \return True if the difference `x - y` sum is grater than T::max value.
 */
template<typename T>
bool
sub_will_overflow(const T x, const T y)
{
  return (y < 0) && (x > (std::numeric_limits<T>::max() + y));
}

/// Safely check if subtraction will underflow.
/**
 * The type of the operands, T, should have defined
 * std::numeric_limits<T>::min(), `>`, `<` and `+` operators.
 *
 * \param[in] x is the minuend.
 * \param[in] y is the subtrahend.
 * \tparam T is type of the operands.
 * \return True if the difference `x - y` sum is less than T::min value.
 */
template<typename T>
bool
sub_will_underflow(const T x, const T y)
{
  return (y > 0) && (x < (std::numeric_limits<T>::min() + y));
}

/// Return the given string.
/**
 * This function is overloaded to transform any string to C-style string.
 *
 * \param[in] string_in is the string to be returned
 * \return the given string
 */
RCLCPP_PUBLIC
const char *
get_c_string(const char * string_in);

/// Return the C string from the given std::string.
/**
 * \param[in] string_in is a std::string
 * \return the C string from the std::string
 */
RCLCPP_PUBLIC
const char *
get_c_string(const std::string & string_in);

}  // namespace rclcpp

#endif  // RCLCPP__UTILITIES_HPP_
