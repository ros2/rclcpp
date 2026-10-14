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

#include <filesystem>
#include <string>

#include "rcl_logging_interface/rcl_logging_interface.h"

#include "rclcpp/exceptions.hpp"
#include "rclcpp/log_directory.hpp"

namespace rclcpp
{

std::filesystem::path
get_log_directory()
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

}  // namespace rclcpp
