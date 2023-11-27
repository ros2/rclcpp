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

#include <memory>
#include <string>
#include <utility>

#include "rcl_logging_interface/rcl_logging_interface.h"
#include "rcl/error_handling.h"
#include "rcl/logging_rosout.h"

#include "rclcpp/exceptions.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "./logging_mutex.hpp"

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

Logger
Logger::get_child(const std::string & suffix)
{
  if (!name_) {
    return Logger();
  }

  rcl_ret_t rcl_ret = RCL_RET_OK;
  std::shared_ptr<std::recursive_mutex> logging_mutex;
  logging_mutex = get_global_logging_mutex();
  {
    std::lock_guard<std::recursive_mutex> guard(*logging_mutex);
    rcl_ret = rcl_logging_rosout_add_sublogger((*name_).c_str(), suffix.c_str());
    if (RCL_RET_NOT_FOUND == rcl_ret) {
      rcl_reset_error();
    } else if (RCL_RET_OK != rcl_ret) {
      exceptions::throw_from_rcl_error(
        rcl_ret, "failed to call rcl_logging_rosout_add_sublogger",
        rcl_get_error_state(), rcl_reset_error);
    }
  }

  Logger logger(*name_ + RCUTILS_LOGGING_SEPARATOR_STRING + suffix);
  if (RCL_RET_OK == rcl_ret) {
    logger.logger_sublogger_pairname_.reset(
      new std::pair<std::string, std::string>({*name_, suffix}),
      [logging_mutex](std::pair<std::string, std::string> * logger_sublogger_pairname_ptr) {
        std::lock_guard<std::recursive_mutex> guard(*logging_mutex);
        rcl_ret_t rcl_ret = rcl_logging_rosout_remove_sublogger(
          logger_sublogger_pairname_ptr->first.c_str(),
          logger_sublogger_pairname_ptr->second.c_str());
        delete logger_sublogger_pairname_ptr;
        if (RCL_RET_OK != rcl_ret) {
          rcl_reset_error();
        }
      });
  }
  return logger;
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

Logger::Level
Logger::get_effective_level() const
{
  int logger_level = rcutils_logging_get_logger_effective_level(get_name());

  if (logger_level < 0) {
    exceptions::throw_from_rcl_error(
      RCL_RET_ERROR, "Couldn't get logger level",
      rcutils_get_error_state(), rcutils_reset_error);
  }

  return static_cast<Level>(logger_level);
}

}  // namespace rclcpp
