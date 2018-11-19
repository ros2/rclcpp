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
#include <vector>

#include "./rcl_context_wrapper.hpp"
#include "rclcpp/contexts/default_context.hpp"
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

/// Represent the status of the global interrupt signal.
static volatile sig_atomic_t g_signal_status = 0;
/// Mutex to protect installation and uninstallation of signal handlers.
static std::mutex g_signal_handlers_mutex;
/// Atomic bool to control setup of signal handlers.
static std::atomic<bool> g_signal_handlers_installed(false);

/// Mutex to protect g_sigint_guard_cond_handles
static std::mutex g_sigint_guard_cond_handles_mutex;
/// Guard conditions for interrupting the rmw implementation when the global interrupt signal fired.
static std::map<rcl_wait_set_t *, rcl_guard_condition_t> g_sigint_guard_cond_handles;

/// Condition variable for timed sleep (see sleep_for).
static std::condition_variable g_interrupt_condition_variable;
/// Mutex for protecting the global condition variable.
static std::mutex g_interrupt_mutex;

#ifdef HAS_SIGACTION
static struct sigaction old_action;
#else
typedef void (* signal_handler_t)(int);
static signal_handler_t old_signal_handler = 0;
#endif

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

void
trigger_interrupt_guard_condition(int signal_value)
{
  g_signal_status = signal_value;
  {
    std::lock_guard<std::mutex> lock(g_sigint_guard_cond_handles_mutex);
    for (auto & kv : g_sigint_guard_cond_handles) {
      rcl_ret_t status = rcl_trigger_guard_condition(&(kv.second));
      if (status != RCL_RET_OK) {
        RCUTILS_LOG_ERROR_NAMED(
          "rclcpp",
          "failed to trigger guard condition: %s", rcl_get_error_string().str);
      }
    }
  }
  g_interrupt_condition_variable.notify_all();
}

void
#ifdef HAS_SIGACTION
signal_handler(int signal_value, siginfo_t * siginfo, void * context)
#else
signal_handler(int signal_value)
#endif
{
  // TODO(wjwwood): remove? move to console logging at some point?
  printf("signal_handler(%d)\n", signal_value);

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

  trigger_interrupt_guard_condition(signal_value);
}

void
rclcpp::init(int argc, char const * const argv[], const rclcpp::InitOptions & init_options)
{
  using rclcpp::contexts::default_context::get_global_default_context;
  init_local(argc, argv, get_global_default_context(), init_options);
  // Install the signal handlers.
  rclcpp::install_signal_handlers();
}

void
rclcpp::init_local(
  int argc,
  char const * const argv[],
  rclcpp::Context::SharedPtr context_out,
  const rclcpp::InitOptions & init_options)
{
  auto rcl_context_wrapper = context_out->get_sub_context<RclContextWrapper>();
  rcl_ret_t ret = rcl_init(
    argc, argv, init_options.get_rcl_init_options(), rcl_context_wrapper->get_context().get());
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to initialize rcl: ");
  }
}

bool
rclcpp::install_signal_handlers()
{
  std::lock_guard<std::mutex> lock(g_signal_handlers_mutex);
  bool already_installed = g_signal_handlers_installed.exchange(true);
  if (already_installed) {
    return false;
  }
#ifdef HAS_SIGACTION
  struct sigaction action;
  memset(&action, 0, sizeof(action));
  sigemptyset(&action.sa_mask);
  action.sa_sigaction = ::signal_handler;
  action.sa_flags = SA_SIGINFO;
  ::old_action = set_sigaction(SIGINT, action);
#else
  ::old_signal_handler = set_signal_handler(SIGINT, ::signal_handler);
#endif
  return true;
}

bool
rclcpp::signal_handlers_installed()
{
  return g_signal_handlers_installed.load();
}

bool
rclcpp::uninstall_signal_handlers()
{
  std::lock_guard<std::mutex> lock(g_signal_handlers_mutex);
  bool installed = g_signal_handlers_installed.exchange(false);
  if (!installed) {
    return false;
  }
#ifdef HAS_SIGACTION
  set_sigaction(SIGINT, ::old_action);
#else
  set_signal_handler(SIGINT, ::old_signal_handler);
#endif
  return true;
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
  if (nullptr != context) {
    context = get_global_default_context();
  }
  auto rcl_context_wrapper = context->get_sub_context<RclContextWrapper>();
  return rcl_context_is_valid(rcl_context_wrapper->get_context().get());
}

bool
rclcpp::is_initialized(rclcpp::Context::SharedPtr context)
{
  using rclcpp::contexts::default_context::get_global_default_context;
  if (nullptr != context) {
    context = get_global_default_context();
  }
  auto rcl_context_wrapper = context->get_sub_context<RclContextWrapper>();
  return rcl_context_is_valid(rcl_context_wrapper->get_context().get());
}

static std::mutex on_shutdown_mutex_;

static
std::vector<std::pair<rclcpp::Context::SharedPtr, std::function<void(rclcpp::Context::SharedPtr)>>>
on_shutdown_callbacks_;

void
rclcpp::shutdown(rclcpp::Context::SharedPtr context)
{
  trigger_interrupt_guard_condition(SIGINT);

  using rclcpp::contexts::default_context::get_global_default_context;
  if (nullptr != context) {
    context = get_global_default_context();
  }
  auto rcl_context_wrapper = context->get_sub_context<RclContextWrapper>();
  auto ret = rcl_shutdown(rcl_context_wrapper->get_context().get());
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
  rcl_context_wrapper->reset();

  // Uninstall the signal handlers.
  uninstall_signal_handlers();

  // Execute on shutdown callbacks.
  {
    std::lock_guard<std::mutex> lock(on_shutdown_mutex_);
    for (auto & on_shutdown_context_callback_pair : on_shutdown_callbacks_) {
      if (context == on_shutdown_context_callback_pair.first) {
        on_shutdown_context_callback_pair.second(on_shutdown_context_callback_pair.first);
      }
    }
  }
}

void
rclcpp::on_shutdown(
  std::function<void(rclcpp::Context::SharedPtr)> callback,
  rclcpp::Context::SharedPtr context)
{
  std::lock_guard<std::mutex> lock(on_shutdown_mutex_);
  on_shutdown_callbacks_.push_back({context, callback});
}

rcl_guard_condition_t *
rclcpp::get_sigint_guard_condition(rcl_wait_set_t * wait_set, rclcpp::Context::SharedPtr context)
{
  std::lock_guard<std::mutex> lock(g_sigint_guard_cond_handles_mutex);
  auto kv = g_sigint_guard_cond_handles.find(wait_set);
  if (kv != g_sigint_guard_cond_handles.end()) {
    return &kv->second;
  } else {
    using rclcpp::contexts::default_context::get_global_default_context;
    if (nullptr != context) {
      context = get_global_default_context();
    }
    auto rcl_context_wrapper = context->get_sub_context<RclContextWrapper>();
    rcl_guard_condition_t handle = rcl_get_zero_initialized_guard_condition();
    rcl_guard_condition_options_t options = rcl_guard_condition_get_default_options();
    auto ret = rcl_guard_condition_init(&handle, rcl_context_wrapper->get_context().get(), options);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "Couldn't initialize guard condition: ");
    }
    g_sigint_guard_cond_handles[wait_set] = handle;
    return &g_sigint_guard_cond_handles[wait_set];
  }
}

void
rclcpp::release_sigint_guard_condition(rcl_wait_set_t * wait_set)
{
  std::lock_guard<std::mutex> lock(g_sigint_guard_cond_handles_mutex);
  auto kv = g_sigint_guard_cond_handles.find(wait_set);
  if (kv != g_sigint_guard_cond_handles.end()) {
    if (rcl_guard_condition_fini(&kv->second) != RCL_RET_OK) {
      // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
      throw std::runtime_error(std::string(
        "Failed to destroy sigint guard condition: ") +
        rcl_get_error_string().str);
      // *INDENT-ON*
    }
    g_sigint_guard_cond_handles.erase(kv);
  } else {
    // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
    throw std::runtime_error(std::string(
      "Tried to release sigint guard condition for nonexistent wait set"));
    // *INDENT-ON*
  }
}

bool
rclcpp::sleep_for(const std::chrono::nanoseconds & nanoseconds, rclcpp::Context::SharedPtr context)
{
  std::chrono::nanoseconds time_left = nanoseconds;
  {
    std::unique_lock<std::mutex> lock(::g_interrupt_mutex);
    auto start = std::chrono::steady_clock::now();
    // this will release the lock while waiting
    ::g_interrupt_condition_variable.wait_for(lock, nanoseconds);
    time_left -= std::chrono::steady_clock::now() - start;
  }
  if (time_left > std::chrono::nanoseconds::zero() && ok(context)) {
    return sleep_for(time_left);
  }
  // Return true if the timeout elapsed successfully, otherwise false.
  return ok(context);
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
