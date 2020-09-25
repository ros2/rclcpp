// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__FUTURE_RETURN_CODE_HPP_
#define RCLCPP__FUTURE_RETURN_CODE_HPP_

#include <iostream>
#include <string>

#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/// Return codes to be used with spin_until_future_complete.
/**
 * SUCCESS: The future is complete and can be accessed with "get" without blocking.
 *          This does not indicate that the operation succeeded; "get" may still throw an exception.
 * INTERRUPTED: The future is not complete, spinning was interrupted by Ctrl-C or another error.
 * TIMEOUT: Spinning timed out.
 */
enum class FutureReturnCode {SUCCESS, INTERRUPTED, TIMEOUT};

/// Stream operator for FutureReturnCode.
RCLCPP_PUBLIC
std::ostream &
operator<<(std::ostream & os, const FutureReturnCode & future_return_code);

/// String conversion function for FutureReturnCode.
RCLCPP_PUBLIC
std::string
to_string(const FutureReturnCode & future_return_code);

}  // namespace rclcpp

#endif  // RCLCPP__FUTURE_RETURN_CODE_HPP_
