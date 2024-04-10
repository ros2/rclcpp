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

#ifndef RCLCPP__SPIN_UNTIL_COMPLETE_RETURN_CODE_HPP_
#define RCLCPP__SPIN_UNTIL_COMPLETE_RETURN_CODE_HPP_

#include <iostream>
#include <string>

#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/// Return codes to be used with spin_until_complete.
/**
 * SUCCESS: The condition wait is complete. This does not indicate that the operation succeeded.
 * INTERRUPTED: The condition wait is not complete, spinning was interrupted by Ctrl-C or another
 *              error.
 * TIMEOUT: Spinning timed out.
 */
enum class SpinUntilCompleteReturnCode {SUCCESS, INTERRUPTED, TIMEOUT};

/// Stream operator for SpinUntilCompleteReturnCode.
RCLCPP_PUBLIC
std::ostream &
operator<<(std::ostream & os, const SpinUntilCompleteReturnCode & spin_until_complete_return_code);

/// String conversion function for SpinUntilCompleteReturnCode.
RCLCPP_PUBLIC
std::string
to_string(const SpinUntilCompleteReturnCode & spin_until_complete_return_code);

}  // namespace rclcpp

#endif  // RCLCPP__SPIN_UNTIL_COMPLETE_RETURN_CODE_HPP_
