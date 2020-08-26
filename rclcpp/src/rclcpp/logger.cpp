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

#include <string>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

namespace rclcpp
{

Logger
get_logger(const std::string & name)
{
#if RCLCPP_LOGGING_ENABLED
  return rclcpp::Logger(name);
#else
  (void)name;
  return rclcpp::Logger();
#endif
}

Logger
get_node_logger(const rcl_node_t * node)
{
  const char * logger_name = rcl_node_get_logger_name(node);
  if (nullptr == logger_name) {
    auto logger = rclcpp::get_logger("rclcpp");
    RCLCPP_ERROR(logger, "failed to get logger name from node at address %p", node);
    return logger;
  }
  return rclcpp::get_logger(logger_name);
}

void
Logger::set_level(Level level)
{
  if (rcutils_logging_set_logger_level(
      get_name(),
      static_cast<RCUTILS_LOG_SEVERITY>(level)) != RCUTILS_RET_OK)
  {
    exceptions::throw_from_rcl_error(
      RCL_RET_ERROR, "Couldn't set logger level",
      rcutils_get_error_state(), rcutils_reset_error);
  }
}

}  // namespace rclcpp
