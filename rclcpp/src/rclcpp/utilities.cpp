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

#include <atomic>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/scope_exit.hpp"

#include "rcl/error_handling.h"
#include "rcl/rcl.h"

#include "rmw/error_handling.h"
#include "rmw/rmw.h"

#include "rcutils/logging_macros.h"

// Determine if sigaction is available
#if __APPLE__ || _POSIX_C_SOURCE >= 1 || _XOPEN_SOURCE || _POSIX_SOURCE
#define HAS_SIGACTION
#endif

namespace rclcpp
{

class SignalHandler final
{
public:
  static
  SignalHandler &
  get_global_signal_handler()
  {
    static SignalHandler singleton;
    return singleton;
  }

  bool
  install()
  {
    bool already_installed = installed_.exchange(true);
    if (already_installed) {
      return false;
    }
    try {
      signal_received.exchange(false);
#ifdef HAS_SIGACTION
      struct sigaction action;
      memset(&action, 0, sizeof(action));
      sigemptyset(&action.sa_mask);
      action.sa_sigaction = signal_handler;
      action.sa_flags = SA_SIGINFO;
      old_action = set_sigaction(SIGINT, action);
#else
      old_signal_handler = set_signal_handler(SIGINT, signal_handler);
#endif
      signal_handler_thread_ =
        std::thread(&SignalHandler::deferred_signal_handler, this);
    } catch (...) {
      installed_.exchange(false);
      throw;
    }
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "signal handler installed");
    return true;
  }

  bool
  uninstall()
  {
    bool installed = installed_.exchange(false);
    if (!installed) {
      return false;
    }
    try {
#ifdef HAS_SIGACTION
      set_sigaction(SIGINT, old_action);
#else
      set_signal_handler(SIGINT, old_signal_handler);
#endif
      events_condition_variable.notify_one();
      signal_handler_thread_.join();
    } catch (...) {
      installed_.exchange(true);
      throw;
    }
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "signal handler uninstalled");
    return true;
  }

  bool
  is_installed()
  {
    return installed_.load();
  }

private:
  SignalHandler() = default;

  ~SignalHandler()
  {
    uninstall();
  }

  // A mutex to lock event signaling.
  static std::mutex events_mutex;
  // A cond var to wait on for event signaling.
  static std::condition_variable events_condition_variable;
  // Whether a signal has been received or not.
  static std::atomic<bool> signal_received;

  // POSIX signal handler structure.
#ifdef HAS_SIGACTION
  static struct sigaction old_action;
#else
  typedef void (* signal_handler_t)(int);
  static signal_handler_t old_signal_handler;
#endif

  static
#ifdef HAS_SIGACTION
  struct sigaction
  set_sigaction(int signal_value, const struct sigaction & action)
#else
  signal_handler_t
  set_signal_handler(int signal_value, signal_handler_t signal_handler)
#endif
  {
#ifdef HAS_SIGACTION
    struct sigaction old_action;
    ssize_t ret = sigaction(signal_value, &action, &old_action);
    if (ret == -1)
#else
    signal_handler_t old_signal_handler = std::signal(signal_value, signal_handler);
    // NOLINTNEXTLINE(readability/braces)
    if (old_signal_handler == SIG_ERR)
#endif
    {
      const size_t error_length = 1024;
      // NOLINTNEXTLINE(runtime/arrays)
      char error_string[error_length];
#ifndef _WIN32
#if (defined(_GNU_SOURCE) && !defined(ANDROID))
      char * msg = strerror_r(errno, error_string, error_length);
      if (msg != error_string) {
        strncpy(error_string, msg, error_length);
        msg[error_length - 1] = '\0';
      }
#else
      int error_status = strerror_r(errno, error_string, error_length);
      if (error_status != 0) {
        throw std::runtime_error("Failed to get error string for errno: " + std::to_string(errno));
      }
#endif
#else
      strerror_s(error_string, error_length, errno);
#endif
      // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
      throw std::runtime_error(
        std::string("Failed to set SIGINT signal handler: (" + std::to_string(errno) + ")") +
        error_string);
      // *INDENT-ON*
    }

#ifdef HAS_SIGACTION
    return old_action;
#else
    return old_signal_handler;
#endif
  }

  static
  void
#ifdef HAS_SIGACTION
  signal_handler(int signal_value, siginfo_t * siginfo, void * context)
#else
  signal_handler(int signal_value)
#endif
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "signal_handler(signal_value=%d)", signal_value);

#ifdef HAS_SIGACTION
    if (old_action.sa_flags & SA_SIGINFO) {
      if (old_action.sa_sigaction != NULL) {
        old_action.sa_sigaction(signal_value, siginfo, context);
      }
    } else {
      if (
        old_action.sa_handler != NULL &&  // Is set
        old_action.sa_handler != SIG_DFL &&  // Is not default
        old_action.sa_handler != SIG_IGN)  // Is not ignored
      {
        old_action.sa_handler(signal_value);
      }
    }
#else
    if (old_signal_handler) {
      old_signal_handler(signal_value);
    }
#endif
    signal_received.exchange(true);
    events_condition_variable.notify_one();
  }

  void
  deferred_signal_handler()
  {
    while (true) {
      if (signal_received.exchange(false)) {
        for (auto context_ptr : rclcpp::get_contexts()) {
          if (context_ptr->get_init_options().shutdown_on_sigint) {
            context_ptr->shutdown("signal handler");
          }
        }
      }
      if (!is_installed()) {
        break;
      }
      std::unique_lock<std::mutex> events_lock(events_mutex);
      events_condition_variable.wait(events_lock);
    }
  }

  // Whether this handler has been installed or not.
  std::atomic<bool> installed_{false};
  // A thread to defer signal handling tasks to.
  std::thread signal_handler_thread_;
};

}  // namespace rclcpp

// Declare static class variables for SignalHandler
std::mutex rclcpp::SignalHandler::events_mutex;
std::condition_variable rclcpp::SignalHandler::events_condition_variable;
std::atomic<bool> rclcpp::SignalHandler::signal_received;
#ifdef HAS_SIGACTION
struct sigaction rclcpp::SignalHandler::old_action;
#else
typedef void (* signal_handler_t)(int);
signal_handler_t rclcpp::SignalHandler::old_signal_handler = 0;
#endif

/// Mutex to protect g_sigint_guard_cond_handles
static std::mutex g_sigint_guard_cond_handles_mutex;
/// Guard conditions for interrupting the rmw implementation when the global interrupt signal fired.
static std::map<rcl_wait_set_t *, rcl_guard_condition_t> g_sigint_guard_cond_handles;

/// Condition variable for timed sleep (see sleep_for).
static std::condition_variable g_interrupt_condition_variable;
/// Mutex for protecting the global condition variable.
static std::mutex g_interrupt_mutex;

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
