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

#ifndef RCLCPP__LOG_DIRECTORY_HPP_
#define RCLCPP__LOG_DIRECTORY_HPP_

#include <filesystem>

#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/// Get the current logging directory.
/**
 * For more details of how the logging directory is determined,
 * see rcl_logging_get_logging_directory().
 *
 * \returns the logging directory being used.
 * \throws rclcpp::exceptions::RCLError if an unexpected error occurs.
 */
RCLCPP_PUBLIC
std::filesystem::path
get_log_directory();

}  // namespace rclcpp

#endif  // RCLCPP__LOG_DIRECTORY_HPP_
