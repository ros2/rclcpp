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

#ifndef RCLCPP__DETAIL__UTILITIES_HPP_
#define RCLCPP__DETAIL__UTILITIES_HPP_

#include "rclcpp/detail/utilities.hpp"

#include <string>
#include <vector>

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
  rcl_allocator_t allocator);

}  // namespace detail
}  // namespace rclcpp

#endif  // RCLCPP__DETAIL__UTILITIES_HPP_
