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
  return rclcpp::Logger(name);
}

}  // namespace rclcpp

#endif  // RCLCPP__LOGGER_HPP_
