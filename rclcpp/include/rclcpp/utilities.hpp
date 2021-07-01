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
#include <string>
#include <vector>

#include "rclcpp/context.hpp"
#include "rclcpp/init_options.hpp"
#include "rclcpp/visibility_control.hpp"

#ifdef ANDROID
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
 * Initializes the global context which is accessible via the function
 * rclcpp::contexts::get_global_default_context().
 * Also, installs the global signal handlers with the function
 * rclcpp::install_signal_handlers().
 *
 * \sa rclcpp::Context::init() for more details on arguments and possible exceptions
 */
RCLCPP_PUBLIC
void
init(int argc, char const * const argv[], const InitOptions & init_options = InitOptions());

/// Install the global signal handler for rclcpp.
/**
 * This function should only need to be run one time per process.
 * It is implicitly run by rclcpp::init(), and therefore this function does not
 * need to be run manually if rclcpp::init() has already been run.
 *
 * The signal handler will shutdown all initialized context.
 * It will also interrupt any blocking functions in ROS allowing them react to
 * any changes in the state of the system (like shutdown).
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
 * If you choose to use it, this function only needs to be run one time.
 * It is implicitly run by rclcpp::shutdown(), and therefore this function does
 * not need to be run manually if rclcpp::shutdown() has already been run.
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
 *
 * \sa rclcpp::Context::init() for more details on arguments and possible exceptions
 * \returns Members of the argument vector that are not ROS arguments.
 * \throws anything remove_ros_arguments can throw
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
 * \throws anything throw_from_rcl_error can throw
 * \throws rclcpp::exceptions::RCLError if the parsing fails
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
 * \param[in] context Optional check for shutdown of this Context.
 * \return true if shutdown has been called, false otherwise
 */
RCLCPP_PUBLIC
bool
ok(rclcpp::Context::SharedPtr context = nullptr);

/// Shutdown rclcpp context, invalidating it for derived entities.
/**
 * If nullptr is given for the context, then the global context is used, i.e.
 * the context initialized by rclcpp::init().
 *
 * If the global context is used, then the signal handlers are also uninstalled.
 *
 * This will also cause the "on_shutdown" callbacks to be called.
 *
 * \sa rclcpp::Context::shutdown()
 * \param[in] context Optional to be shutdown
 * \param[in] reason Optional string passed to the context shutdown method
 * \return true if shutdown was successful, false if context was already shutdown
 */
RCLCPP_PUBLIC
bool
shutdown(
  rclcpp::Context::SharedPtr context = nullptr,
  const std::string & reason = "user called rclcpp::shutdown()");

/// Register a function to be called when shutdown is called on the context.
/**
 * If nullptr is given for the context, then the global context is used, i.e.
 * the context initialized by rclcpp::init().
 *
 * These callbacks are called when the associated Context is shutdown with the
 * Context::shutdown() method.
 * When shutdown by the SIGINT handler, shutdown, and therefore these callbacks,
 * is called asynchronously from the dedicated signal handling thread, at some
 * point after the SIGINT signal is received.
 *
 * \sa rclcpp::Context::on_shutdown()
 * \param[in] callback to be called when the given context is shutdown
 * \param[in] context with which to associate the context
 */
RCLCPP_PUBLIC
void
on_shutdown(std::function<void()> callback, rclcpp::Context::SharedPtr context = nullptr);

/// Use the global condition variable to block for the specified amount of time.
/**
 * This function can be interrupted early if the associated context becomes
 * invalid due to shutdown() or the signal handler.
 * \sa rclcpp::Context::sleep_for
 *
 * If nullptr is given for the context, then the global context is used, i.e.
 * the context initialized by rclcpp::init().
 *
 * \param[in] nanoseconds A std::chrono::duration representing how long to sleep for.
 * \param[in] context Optional which may interrupt this sleep
 * \return true if the condition variable did not timeout.
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
