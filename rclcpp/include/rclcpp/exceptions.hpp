// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXCEPTIONS_HPP_
#define RCLCPP__EXCEPTIONS_HPP_

#include <stdexcept>
#include <string>

#include "rcl/error_handling.h"
#include "rcl/types.h"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace exceptions
{

/// Throw a C++ std::exception which was created based on an rcl error.
/*
 * \param ret the return code for the current error state
 * \param prefix string to prefix to the error if applicable (not all errors have custom messages)
 * \param reset_error if true rcl_reset_error() is called before returning
 * \throws std::invalid_argument if ret is RCL_RET_OK
 * \throws std::runtime_error if the rcl_get_error_state returns 0
 * \throws RCLErrorBase some child class exception based on ret
 */
RCLCPP_PUBLIC
void
throw_from_rcl_error(rcl_ret_t ret, const std::string & prefix = "", bool reset_error = true);

class RCLErrorBase
{
public:
  RCLCPP_PUBLIC
  RCLErrorBase(rcl_ret_t ret, const rcl_error_state_t * error_state);
  virtual ~RCLErrorBase() {}

  rcl_ret_t ret;
  std::string message;
  std::string file;
  size_t line;
  std::string formatted_message;
};

/// Created when the return code does not match one of the other specialized exceptions.
class RCLError : public RCLErrorBase, public std::runtime_error
{
public:
  RCLCPP_PUBLIC
  RCLError(rcl_ret_t ret, const rcl_error_state_t * error_state, const std::string & prefix);
  RCLCPP_PUBLIC
  RCLError(const RCLErrorBase & base_exc, const std::string & prefix);
};

/// Created when the ret is RCL_RET_BAD_ALLOC.
class RCLBadAlloc : public RCLErrorBase, public std::bad_alloc
{
public:
  RCLCPP_PUBLIC
  RCLBadAlloc(rcl_ret_t ret, const rcl_error_state_t * error_state);
  RCLCPP_PUBLIC
  explicit RCLBadAlloc(const RCLErrorBase & base_exc);
};

/// Created when the ret is RCL_RET_INVALID_ARGUMENT.
class RCLInvalidArgument : public RCLErrorBase, public std::invalid_argument
{
public:
  RCLCPP_PUBLIC
  RCLInvalidArgument(
    rcl_ret_t ret,
    const rcl_error_state_t * error_state,
    const std::string & prefix);
  RCLCPP_PUBLIC
  RCLInvalidArgument(const RCLErrorBase & base_exc, const std::string & prefix);
};

}  // namespace exceptions
}  // namespace rclcpp

#endif  // RCLCPP__EXCEPTIONS_HPP_
