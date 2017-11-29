// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__LOGGER_HPP_
#define RCLCPP__LOGGER_HPP_

#include <memory>
#include <string>

#include "rclcpp/visibility_control.hpp"

/**
 * \def RCLCPP_LOGGING_ENABLED
 * When this define evaluates to true (default), logger factory functions will
 * behave normally.
 * When false, logger factory functions will create dummy loggers to avoid
 * computational expense in manipulating objects.
 * This should be used in combination with `RCLCPP_LOG_MIN_SEVERITY` to compile
 * out logging macros.
 */
// TODO(dhood): determine this automatically from `RCLCPP_LOG_MIN_SEVERITY`
#ifndef RCLCPP_LOGGING_ENABLED
#define RCLCPP_LOGGING_ENABLED 1
#endif

namespace rclcpp
{

class Logger
{
private:
  friend Logger rclcpp::get_logger(const std::string & name);

  /// Constructor of a dummy logger.
  /**
   * This is used when logging is disabled: see `RCLCPP_LOGGING_ENABLED`.
   * This cannot be called directly, see `rclcpp::get_logger` instead.
   */
  Logger()
  : name_(nullptr) {}

  /// Constructor of a named logger.
  /**
   * This cannot be called directly, see `rclcpp::get_logger` instead.
   */
  explicit Logger(const std::string & name)
  : name_(new std::string(name)) {}

  std::shared_ptr<const std::string> name_;

public:
  RCLCPP_PUBLIC
  Logger(const Logger &) = default;

  /// Get the name of this logger.
  /**
   * \return the full name of the logger including any prefixes, or
   * \return `nullptr` if this logger is invalid (e.g. because logging is
   *   disabled).
   */
  RCLCPP_PUBLIC
  const char * get_name() const
  {
    if (!name_) {
      return nullptr;
    }
    return name_->c_str();
  }

  /// Return a logger that is a descendant of this logger.
  /**
   * The child logger's full name will include any hierarchy conventions that
   * indicate it is a descendant of this logger.
   * For example, ```get_logger('abc').get_child('def')``` will return a logger
   * with name `abc.def`.
   *
   * \param[in] suffix the child logger's suffix
   * \return a logger with the fully-qualified name including the suffix, or
   * \return a dummy logger if this logger is invalid (e.g. because logging is
   *   disabled).
   */
  RCLCPP_PUBLIC
  Logger get_child(const std::string & suffix)
  {
    if (!name_) {
      return Logger();
    }
    return Logger(*name_ + "." + suffix);
  }
};

/// Return a named logger.
/**
 * The returned logger's name will include any naming conventions, such as a
 * name prefix.
 * Currently there are no such naming conventions but they may be introduced in
 * the future.
 *
 * \param[in] name the name of the logger
 * \return a logger with the fully-qualified name including naming conventions, or
 * \return a dummy logger if logging is disabled.
 */
inline Logger get_logger(const std::string & name)
{
#if RCLCPP_LOGGING_ENABLED
  return rclcpp::Logger(name);
#else
  (void)name;
  return rclcpp::Logger();
#endif
}

namespace logging_macro_utilities
{

/// Helper function to give useful compiler errors in logging macros.
/**
 * This is not intended for regular use: the `Logger::get_name` method should be
 * used.
 * This will provide a compiler error that includes the `Logger` class if an
 * incorrect type is passed as a parameter, e.g. through incorrect usage of
 * logging macros such as `RCLCPP_INFO()`.
 *
 * \param[in] logger the logger to get the name of
 * \return the name of the logger
 */
inline const char * get_logger_name(const Logger & logger)
{
  return logger.get_name();
}

}  // namespace logging_macro_utilities

}  // namespace rclcpp

#endif  // RCLCPP__LOGGER_HPP_
