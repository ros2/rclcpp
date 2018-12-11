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
#include "rmw/impl/cpp/demangle.hpp"
#include "rmw/rmw.h"

#include "rcutils/logging_macros.h"

// Determine if sigaction is available
#if __APPLE__ || _POSIX_C_SOURCE >= 1 || _XOPEN_SOURCE || _POSIX_SOURCE
#define HAS_SIGACTION
#endif

rclcpp::Logger g_logger = rclcpp::get_logger("rclcpp");

namespace rclcpp
{

// This setups anything that is necessary for wait_for_signal() or notify_signal_handler().
// This must be called before wait_for_signal() or notify_signal_handler().
// This is not thread-safe.
RCLCPP_LOCAL
void
setup_wait_for_signal();

// Undo all setup done in setup_wait_for_signal().
// Must not call wait_for_signal() or notify_signal_handler() after calling this.
// This is not thread-safe.
RCLCPP_LOCAL
void
teardown_wait_for_signal() noexcept;

// This waits for a notification from notify_signal_handler() in a signal safe way.
// This is not thread-safe.
RCLCPP_LOCAL
void
wait_for_signal() noexcept;

// This notifies the deferred_signal_handler() thread to wake up in a signal safe way.
// This is thread-safe.
RCLCPP_LOCAL
void
notify_signal_handler() noexcept;

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
    std::lock_guard<std::mutex> lock(install_mutex_);
    bool already_installed = installed_.exchange(true);
    if (already_installed) {
      return false;
    }
    try {
      setup_wait_for_signal();
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
    RCLCPP_DEBUG(g_logger, "signal handler installed");
    return true;
  }

  bool
  uninstall()
  {
    std::lock_guard<std::mutex> lock(install_mutex_);
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
      RCLCPP_DEBUG(g_logger, "SignalHandler::uninstall(): notifying deferred signal handler");
      notify_signal_handler();
      signal_handler_thread_.join();
      teardown_wait_for_signal();
    } catch (...) {
      installed_.exchange(true);
      throw;
    }
    RCLCPP_DEBUG(g_logger, "signal handler uninstalled");
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
    try {
      uninstall();
    } catch (const std::exception & exc) {
      RCLCPP_ERROR(
        g_logger,
        "caught %s exception when uninstalling the sigint handler in rclcpp::~SignalHandler: %s",
        rmw::impl::cpp::demangle(exc).c_str(), exc.what());
    } catch (...) {
      RCLCPP_ERROR(
        g_logger,
        "caught unknown exception when uninstalling the sigint handler in rclcpp::~SignalHandler");
    }
  }

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
    RCLCPP_INFO(g_logger, "signal_handler(signal_value=%d)", signal_value);

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
    RCLCPP_DEBUG(g_logger, "signal_handler(): SIGINT received, notifying deferred signal handler");
    notify_signal_handler();
  }

  void
  deferred_signal_handler()
  {
    while (true) {
      if (signal_received.exchange(false)) {
        RCLCPP_DEBUG(g_logger, "deferred_signal_handler(): SIGINT received, shutting down");
        for (auto context_ptr : rclcpp::get_contexts()) {
          if (context_ptr->get_init_options().shutdown_on_sigint) {
            RCLCPP_DEBUG(
              g_logger,
              "deferred_signal_handler(): "
              "shutting down rclcpp::Context @ %p, because it had shutdown_on_sigint == true",
              context_ptr.get());
            context_ptr->shutdown("signal handler");
          }
        }
      }
      if (!is_installed()) {
        RCLCPP_DEBUG(g_logger, "deferred_signal_handler(): signal handling uninstalled");
        break;
      }
      RCLCPP_DEBUG(g_logger, "deferred_signal_handler(): waiting for SIGINT or uninstall");
      wait_for_signal();
      RCLCPP_DEBUG(g_logger, "deferred_signal_handler(): woken up due to SIGINT or uninstall");
    }
  }

  // Whether a signal has been received or not.
  static std::atomic<bool> signal_received;
  // A thread to defer signal handling tasks to.
  std::thread signal_handler_thread_;

  // A mutex to synchronize the install() and uninstall() methods.
  std::mutex install_mutex_;
  // Whether this handler has been installed or not.
  std::atomic<bool> installed_{false};
};

}  // namespace rclcpp

// Declare static class variables for SignalHandler
std::atomic<bool> rclcpp::SignalHandler::signal_received;
#ifdef HAS_SIGACTION
struct sigaction rclcpp::SignalHandler::old_action;
#else
typedef void (* signal_handler_t)(int);
signal_handler_t rclcpp::SignalHandler::old_signal_handler = 0;
#endif

#if defined(_WIN32)
#include <windows.h>
#elif defined(__APPLE__)
#include <dispatch/dispatch.h>
#else  // posix
#include <semaphore.h>
#endif

static std::atomic_bool g_wait_for_signal_is_setup = ATOMIC_VAR_INIT(false);
#if defined(_WIN32)
HANDLE g_signal_handler_sem;
#elif defined(__APPLE__)
dispatch_semaphore_t g_signal_handler_sem;
#else  // posix
sem_t g_signal_handler_sem;
#endif

void
rclcpp::setup_wait_for_signal()
{
#if defined(_WIN32)
  g_signal_handler_sem = CreateSemaphore(
    NULL,  // default security attributes
    0,  // initial semaphore count
    1,  // maximum semaphore count
    NULL);  // unnamed semaphore
  if (NULL == g_signal_handler_sem) {
    throw std::runtime_error("CreateSemaphore() failed in setup_wait_for_signal()");
  }
#elif defined(__APPLE__)
  g_signal_handler_sem = dispatch_semaphore_create(0);
#else  // posix
  if (-1 == sem_init(&g_signal_handler_sem, 0, 0)) {
    throw std::runtime_error(std::string("sem_init() failed: ") + strerror(errno));
  }
#endif
  g_wait_for_signal_is_setup.store(true);
}

void
rclcpp::teardown_wait_for_signal() noexcept
{
  if (!g_wait_for_signal_is_setup.exchange(false)) {
    return;
  }
#if defined(_WIN32)
  CloseHandle(g_signal_handler_sem);
#elif defined(__APPLE__)
  dispatch_release(g_signal_handler_sem);
#else  // posix
  if (EINVAL == sem_destroy(&g_signal_handler_sem)) {
    RCLCPP_ERROR(g_logger, "invalid semaphore in teardown_wait_for_signal()");
  }
#endif
}

void
rclcpp::wait_for_signal() noexcept
{
  if (!g_wait_for_signal_is_setup.load()) {
    RCLCPP_ERROR(g_logger, "called wait_for_signal() before setup_wait_for_signal()");
    return;
  }
#if defined(_WIN32)
  DWORD dw_wait_result = WaitForSingleObject(g_signal_handler_sem, INFINITE);
  switch (dw_wait_result) {
    case WAIT_ABANDONED:
      RCLCPP_ERROR(
        g_logger, "WaitForSingleObject() failed in wait_for_signal() with WAIT_ABANDONED: %s",
        GetLastError());
      break;
    case WAIT_OBJECT_0:
      // successful
      break;
    case WAIT_TIMEOUT:
      RCLCPP_ERROR(g_logger, "WaitForSingleObject() timedout out in wait_for_signal()");
      break;
    case WAIT_FAILED:
      RCLCPP_ERROR(
        g_logger, "WaitForSingleObject() failed in wait_for_signal(): %s", GetLastError());
      break;
    default:
      RCLCPP_ERROR(
        g_logger, "WaitForSingleObject() gave unknown return in wait_for_signal(): %s",
        GetLastError());
  }
#elif defined(__APPLE__)
  dispatch_semaphore_wait(g_signal_handler_sem, DISPATCH_TIME_FOREVER);
#else  // posix
  int s;
  do {
    s = sem_wait(&g_signal_handler_sem);
  } while (-1 == s && EINTR == errno);
#endif
}

void
rclcpp::notify_signal_handler() noexcept
{
  if (!g_wait_for_signal_is_setup.load()) {
    return;
  }
#if defined(_WIN32)
  if (!ReleaseSemaphore(g_signal_handler_sem, 1, NULL)) {
    RCLCPP_ERROR(
      g_logger, "ReleaseSemaphore() failed in notify_signal_handler(): %s", GetLastError());
  }
#elif defined(__APPLE__)
  dispatch_semaphore_signal(g_signal_handler_sem);
#else  // posix
  if (-1 == sem_post(&g_signal_handler_sem)) {
    RCLCPP_ERROR(g_logger, "sem_post failed in notify_signal_handler()");
  }
#endif
}

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
