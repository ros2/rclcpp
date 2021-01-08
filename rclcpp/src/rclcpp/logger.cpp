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

#include "rcl_logging_interface/rcl_logging_interface.h"

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
    RCLCPP_ERROR(
      logger, "failed to get logger name from node at address %p",
      static_cast<void *>(const_cast<rcl_node_t *>(node)));
    return logger;
  }
  return rclcpp::get_logger(logger_name);
}

rcpputils::fs::path
get_logging_directory()
{
  char * log_dir = NULL;
  auto allocator = rcutils_get_default_allocator();
  rcl_logging_ret_t ret = rcl_logging_get_logging_directory(allocator, &log_dir);
  if (RCL_LOGGING_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
  std::string path{log_dir};
  allocator.deallocate(log_dir, allocator.state);
  return path;
}

void
Logger::set_level(Level level)
{
  rcutils_ret_t rcutils_ret = rcutils_logging_set_logger_level(
    get_name(),
    static_cast<RCUTILS_LOG_SEVERITY>(level));
  if (rcutils_ret != RCUTILS_RET_OK) {
    if (rcutils_ret == RCUTILS_RET_INVALID_ARGUMENT) {
      exceptions::throw_from_rcl_error(
        RCL_RET_INVALID_ARGUMENT, "Invalid parameter",
        rcutils_get_error_state(), rcutils_reset_error);
    }
    exceptions::throw_from_rcl_error(
      RCL_RET_ERROR, "Couldn't set logger level",
      rcutils_get_error_state(), rcutils_reset_error);
  }
}

}  // namespace rclcpp
