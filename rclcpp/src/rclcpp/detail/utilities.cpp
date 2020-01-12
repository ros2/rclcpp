// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/detail/utilities.hpp"

#include <cassert>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/exceptions.hpp"

#include "rcl/allocator.h"
#include "rcl/arguments.h"

namespace rclcpp
{
namespace detail
{

std::vector<std::string>
get_unparsed_ros_arguments(
  int argc, char const * const argv[],
  rcl_arguments_t * arguments,
  rcl_allocator_t allocator)
{
  (void)argc;
  std::vector<std::string> unparsed_ros_arguments;
  int unparsed_ros_args_count = rcl_arguments_get_count_unparsed_ros(arguments);
  if (unparsed_ros_args_count > 0) {
    int * unparsed_ros_args_indices = nullptr;
    rcl_ret_t ret =
      rcl_arguments_get_unparsed_ros(arguments, allocator, &unparsed_ros_args_indices);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to get unparsed ROS arguments");
    }
    try {
      for (int i = 0; i < unparsed_ros_args_count; ++i) {
        assert(unparsed_ros_args_indices[i] >= 0);
        assert(unparsed_ros_args_indices[i] < argc);
        unparsed_ros_arguments.push_back(argv[unparsed_ros_args_indices[i]]);
      }
      allocator.deallocate(unparsed_ros_args_indices, allocator.state);
    } catch (...) {
      allocator.deallocate(unparsed_ros_args_indices, allocator.state);
      throw;
    }
  }
  return unparsed_ros_arguments;
}

}  // namespace detail
}  // namespace rclcpp
