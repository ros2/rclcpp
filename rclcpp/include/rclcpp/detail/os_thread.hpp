// Copyright 2025 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__DETAIL__OS_THREAD_HPP_
#define RCLCPP__DETAIL__OS_THREAD_HPP_

#include <string>

#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace detail
{

/// Set the current thread name
/**
 * \param[in] name the name to set for the current thread. May be truncated depending on platform.
 */
RCLCPP_PUBLIC void set_thread_name(const std::string & name);

/// Get the current thread name
/**
 * \return the name of the current thread.
 */
RCLCPP_PUBLIC std::string get_thread_name();

}  // namespace detail
}  // namespace rclcpp

#endif  // RCLCPP__DETAIL__OS_THREAD_HPP_
