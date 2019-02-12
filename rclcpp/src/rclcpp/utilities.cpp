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

#include <string>
#include <vector>

#include "./signal_handler.hpp"
#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/exceptions.hpp"

#include "rcl/error_handling.h"
#include "rcl/rcl.h"

void
rclcpp::init(int argc, char const * const argv[], const rclcpp::InitOptions & init_options)
{
  using rclcpp::contexts::default_context::get_global_default_context;
  get_global_default_context()->init(argc, argv, init_options);
  // Install the signal handlers.
  rclcpp::install_signal_handlers();
}

bool
rclcpp::install_signal_handlers()
{
  return rclcpp::SignalHandler::get_global_signal_handler().install();
}

bool
rclcpp::signal_handlers_installed()
{
  return rclcpp::SignalHandler::get_global_signal_handler().is_installed();
}

bool
rclcpp::uninstall_signal_handlers()
{
  return rclcpp::SignalHandler::get_global_signal_handler().uninstall();
}

std::vector<std::string>
rclcpp::init_and_remove_ros_arguments(
  int argc,
  char const * const argv[],
  const rclcpp::InitOptions & init_options)
{
  rclcpp::init(argc, argv, init_options);
  return rclcpp::remove_ros_arguments(argc, argv);
}

std::vector<std::string>
rclcpp::remove_ros_arguments(int argc, char const * const argv[])
{
  rcl_allocator_t alloc = rcl_get_default_allocator();
  rcl_arguments_t parsed_args = rcl_get_zero_initialized_arguments();

  rcl_ret_t ret;

  ret = rcl_parse_arguments(argc, argv, alloc, &parsed_args);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to parse arguments");
  }

  int nonros_argc = 0;
  const char ** nonros_argv = NULL;

  ret = rcl_remove_ros_arguments(
    argv,
    &parsed_args,
    alloc,
    &nonros_argc,
    &nonros_argv);

  if (RCL_RET_OK != ret) {
    // Not using throw_from_rcl_error, because we may need to append deallocation failures.
    rclcpp::exceptions::RCLErrorBase base_exc(ret, rcl_get_error_state());
    rcl_reset_error();
    if (NULL != nonros_argv) {
      alloc.deallocate(nonros_argv, alloc.state);
    }
    if (RCL_RET_OK != rcl_arguments_fini(&parsed_args)) {
      base_exc.formatted_message += std::string(
        ", failed also to cleanup parsed arguments, leaking memory: ") +
        rcl_get_error_string().str;
      rcl_reset_error();
    }
    throw rclcpp::exceptions::RCLError(base_exc, "");
  }

  std::vector<std::string> return_arguments;
  return_arguments.resize(nonros_argc);

  for (int ii = 0; ii < nonros_argc; ++ii) {
    return_arguments[ii] = std::string(nonros_argv[ii]);
  }

  if (NULL != nonros_argv) {
    alloc.deallocate(nonros_argv, alloc.state);
  }

  ret = rcl_arguments_fini(&parsed_args);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(
      ret, "failed to cleanup parsed arguments, leaking memory");
  }

  return return_arguments;
}

bool
rclcpp::ok(rclcpp::Context::SharedPtr context)
{
  using rclcpp::contexts::default_context::get_global_default_context;
  if (nullptr == context) {
    context = get_global_default_context();
  }
  return context->is_valid();
}

bool
rclcpp::is_initialized(rclcpp::Context::SharedPtr context)
{
  return rclcpp::ok(context);
}

bool
rclcpp::shutdown(rclcpp::Context::SharedPtr context, const std::string & reason)
{
  using rclcpp::contexts::default_context::get_global_default_context;
  auto default_context = get_global_default_context();
  if (nullptr == context) {
    context = default_context;
  }
  bool ret = context->shutdown(reason);
  if (context == default_context) {
    rclcpp::uninstall_signal_handlers();
  }
  return ret;
}

void
rclcpp::on_shutdown(std::function<void()> callback, rclcpp::Context::SharedPtr context)
{
  using rclcpp::contexts::default_context::get_global_default_context;
  if (nullptr == context) {
    context = get_global_default_context();
  }
  context->on_shutdown(callback);
}

bool
rclcpp::sleep_for(const std::chrono::nanoseconds & nanoseconds, rclcpp::Context::SharedPtr context)
{
  using rclcpp::contexts::default_context::get_global_default_context;
  if (nullptr == context) {
    context = get_global_default_context();
  }
  return context->sleep_for(nanoseconds);
}

const char *
rclcpp::get_c_string(const char * string_in)
{
  return string_in;
}

const char *
rclcpp::get_c_string(const std::string & string_in)
{
  return string_in.c_str();
}
