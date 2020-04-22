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

#include "rclcpp/exceptions.hpp"

#include <cstdio>
#include <functional>
#include <string>
#include <vector>

using namespace std::string_literals;

namespace rclcpp
{
namespace exceptions
{

std::string
NameValidationError::format_error(
  const char * name_type,
  const char * name,
  const char * error_msg,
  size_t invalid_index)
{
  std::string msg = "";
  msg += "Invalid "s + name_type + ": " + error_msg + ":\n";
  msg += "  '"s + name + "'\n";
  msg += "   "s + std::string(invalid_index, ' ') + "^\n";
  return msg;
}

std::exception_ptr
from_rcl_error(
  rcl_ret_t ret,
  const std::string & prefix,
  const rcl_error_state_t * error_state,
  void (* reset_error)())
{
  if (RCL_RET_OK == ret) {
    throw std::invalid_argument("ret is RCL_RET_OK");
  }
  if (!error_state) {
    error_state = rcl_get_error_state();
  }
  if (!error_state) {
    throw std::runtime_error("rcl error state is not set");
  }
  std::string formatted_prefix = prefix;
  if (!prefix.empty()) {
    formatted_prefix += ": ";
  }
  RCLErrorBase base_exc(ret, error_state);
  if (reset_error) {
    reset_error();
  }
  switch (ret) {
    case RCL_RET_BAD_ALLOC:
      return std::make_exception_ptr(RCLBadAlloc(base_exc));
    case RCL_RET_INVALID_ARGUMENT:
      return std::make_exception_ptr(RCLInvalidArgument(base_exc, formatted_prefix));
    case RCL_RET_INVALID_ROS_ARGS:
      return std::make_exception_ptr(RCLInvalidROSArgsError(base_exc, formatted_prefix));
    default:
      return std::make_exception_ptr(RCLError(base_exc, formatted_prefix));
  }
}

void
throw_from_rcl_error(
  rcl_ret_t ret,
  const std::string & prefix,
  const rcl_error_state_t * error_state,
  void (* reset_error)())
{
  // We expect this to either throw a standard error,
  // or to generate an error pointer (which is caught
  // in err, and immediately thrown)
  auto err = from_rcl_error(ret, prefix, error_state, reset_error);
  std::rethrow_exception(err);
}

RCLErrorBase::RCLErrorBase(rcl_ret_t ret, const rcl_error_state_t * error_state)
: ret(ret), message(error_state->message), file(error_state->file), line(error_state->line_number),
  formatted_message(rcl_get_error_string().str)
{}

RCLError::RCLError(
  rcl_ret_t ret,
  const rcl_error_state_t * error_state,
  const std::string & prefix)
: RCLError(RCLErrorBase(ret, error_state), prefix)
{}

RCLError::RCLError(
  const RCLErrorBase & base_exc,
  const std::string & prefix)
: RCLErrorBase(base_exc), std::runtime_error(prefix + base_exc.formatted_message)
{}

RCLBadAlloc::RCLBadAlloc(rcl_ret_t ret, const rcl_error_state_t * error_state)
: RCLBadAlloc(RCLErrorBase(ret, error_state))
{}

RCLBadAlloc::RCLBadAlloc(const RCLErrorBase & base_exc)
: RCLErrorBase(base_exc), std::bad_alloc()
{}

RCLInvalidArgument::RCLInvalidArgument(
  rcl_ret_t ret,
  const rcl_error_state_t * error_state,
  const std::string & prefix)
: RCLInvalidArgument(RCLErrorBase(ret, error_state), prefix)
{}

RCLInvalidArgument::RCLInvalidArgument(
  const RCLErrorBase & base_exc,
  const std::string & prefix)
: RCLErrorBase(base_exc), std::invalid_argument(prefix + base_exc.formatted_message)
{}

RCLInvalidROSArgsError::RCLInvalidROSArgsError(
  rcl_ret_t ret,
  const rcl_error_state_t * error_state,
  const std::string & prefix)
: RCLInvalidROSArgsError(RCLErrorBase(ret, error_state), prefix)
{}

RCLInvalidROSArgsError::RCLInvalidROSArgsError(
  const RCLErrorBase & base_exc,
  const std::string & prefix)
: RCLErrorBase(base_exc), std::runtime_error(prefix + base_exc.formatted_message)
{}

}  // namespace exceptions
}  // namespace rclcpp
