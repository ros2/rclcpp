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

#include "rclcpp/visibility_control.hpp"

// When this define evaluates to true (default), logger factory functions will be enabled.
// When false, logger factory functions will create dummy loggers to avoid computational expense.
// This should be used in combination with `RCLCPP_LOG_MIN_SEVERITY` to compile out logging macros.
// TODO(dhood): determine this automatically from `RCLCPP_LOG_MIN_SEVERITY`
#ifndef RCLCPP_LOGGING_ENABLED
#define RCLCPP_LOGGING_ENABLED 1
#endif

namespace rclcpp
{

class Logger
{
// Prevent users from calling constructors directly in favour of factory functions.
private:
  friend Logger rclcpp::get_logger(const std::string & name);
  Logger() : name_(nullptr) {}  // used by factory when logging is disabled globally
  Logger(const std::string & name) : name_(new std::string(name)) {}
  std::shared_ptr<const std::string> name_;

public:
  RCLCPP_PUBLIC
  Logger(const Logger &) = default;

  RCLCPP_PUBLIC
  const char * get_name() const
  {
    if (!name_)
    {
      return nullptr;
    }
    return name_->c_str();
  }

  RCLCPP_PUBLIC
  Logger sublogger(const std::string & suffix) {
    if (!name_) {
      return Logger();
    }
    return Logger(*name_ + "." + suffix);
  }
};

inline Logger get_logger(const std::string & name) {
#ifdef RCLCPP_LOGGING_ENABLED
  return rclcpp::Logger(name);
#else
  return rclcpp::Logger();
#endif
}

namespace logging_macro_utilities {

/// Helper function to give useful compiler errors in logging macros if passed incorrect type.
inline const char * get_logger_name(const Logger & logger)
{
  return logger.get_name();
}

}  // namespace logging_macro_utilities

}  // namespace rclcpp

#endif  // RCLCPP__LOGGER_HPP_
