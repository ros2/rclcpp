// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/utilities.hpp"

#include <chrono>
#include <functional>
#include <string>
#include <vector>

#include "./signal_handler.hpp"
#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/detail/utilities.hpp"
#include "rclcpp/exceptions.hpp"

#include "rcl/error_handling.h"
#include "rcl/rcl.h"

namespace rclcpp
{

void
init(int argc, char const * const argv[], const InitOptions & init_options)
{
  using rclcpp::contexts::get_global_default_context;
  get_global_default_context()->init(argc, argv, init_options);
  // Install the signal handlers.
  install_signal_handlers();
}

bool
install_signal_handlers()
{
  return SignalHandler::get_global_signal_handler().install();
}

bool
signal_handlers_installed()
{
  return SignalHandler::get_global_signal_handler().is_installed();
}

bool
uninstall_signal_handlers()
{
  return SignalHandler::get_global_signal_handler().uninstall();
}

static
std::vector<std::string>
_remove_ros_arguments(
  char const * const argv[],
  const rcl_arguments_t * args,
  rcl_allocator_t alloc)
{
  rcl_ret_t ret;
  int nonros_argc = 0;
  const char ** nonros_argv = NULL;

  ret = rcl_remove_ros_arguments(
    argv,
    args,
    alloc,
    &nonros_argc,
    &nonros_argv);

  if (RCL_RET_OK != ret || nonros_argc < 0) {
    // Not using throw_from_rcl_error, because we may need to append deallocation failures.
    exceptions::RCLError exc(ret, rcl_get_error_state(), "");
    rcl_reset_error();
    if (NULL != nonros_argv) {
      alloc.deallocate(nonros_argv, alloc.state);
    }
    throw exc;
  }

  std::vector<std::string> return_arguments(static_cast<size_t>(nonros_argc));

  for (size_t ii = 0; ii < static_cast<size_t>(nonros_argc); ++ii) {
    return_arguments[ii] = std::string(nonros_argv[ii]);
  }

  if (NULL != nonros_argv) {
    alloc.deallocate(nonros_argv, alloc.state);
  }

  return return_arguments;
}

std::vector<std::string>
init_and_remove_ros_arguments(
  int argc,
  char const * const argv[],
  const InitOptions & init_options)
{
  init(argc, argv, init_options);

  using rclcpp::contexts::get_global_default_context;
  auto rcl_context = get_global_default_context()->get_rcl_context();
  return _remove_ros_arguments(argv, &(rcl_context->global_arguments), rcl_get_default_allocator());
}

std::vector<std::string>
remove_ros_arguments(int argc, char const * const argv[])
{
  rcl_allocator_t alloc = rcl_get_default_allocator();
  rcl_arguments_t parsed_args = rcl_get_zero_initialized_arguments();

  rcl_ret_t ret;

  ret = rcl_parse_arguments(argc, argv, alloc, &parsed_args);
  if (RCL_RET_OK != ret) {
    exceptions::throw_from_rcl_error(ret, "failed to parse arguments");
  }

  std::vector<std::string> return_arguments;
  try {
    return_arguments = _remove_ros_arguments(argv, &parsed_args, alloc);
  } catch (exceptions::RCLError & exc) {
    if (RCL_RET_OK != rcl_arguments_fini(&parsed_args)) {
      exc.formatted_message += std::string(
        ", failed also to cleanup parsed arguments, leaking memory: ") +
        rcl_get_error_string().str;
      rcl_reset_error();
    }
    throw exc;
  }

  ret = rcl_arguments_fini(&parsed_args);
  if (RCL_RET_OK != ret) {
    exceptions::throw_from_rcl_error(
      ret, "failed to cleanup parsed arguments, leaking memory");
  }

  return return_arguments;
}

bool
ok(Context::SharedPtr context)
{
  using rclcpp::contexts::get_global_default_context;
  if (nullptr == context) {
    context = get_global_default_context();
  }
  return context->is_valid();
}

bool
is_initialized(Context::SharedPtr context)
{
  return ok(context);
}

bool
shutdown(Context::SharedPtr context, const std::string & reason)
{
  using rclcpp::contexts::get_global_default_context;
  auto default_context = get_global_default_context();
  if (nullptr == context) {
    context = default_context;
  }
  bool ret = context->shutdown(reason);
  if (context == default_context) {
    uninstall_signal_handlers();
  }
  return ret;
}

void
on_shutdown(std::function<void()> callback, Context::SharedPtr context)
{
  using rclcpp::contexts::get_global_default_context;
  if (nullptr == context) {
    context = get_global_default_context();
  }
  context->on_shutdown(callback);
}

bool
sleep_for(const std::chrono::nanoseconds & nanoseconds, Context::SharedPtr context)
{
  using rclcpp::contexts::get_global_default_context;
  if (nullptr == context) {
    context = get_global_default_context();
  }
  return context->sleep_for(nanoseconds);
}

const char *
get_c_string(const char * string_in)
{
  return string_in;
}

const char *
get_c_string(const std::string & string_in)
{
  return string_in.c_str();
}

}  // namespace rclcpp
