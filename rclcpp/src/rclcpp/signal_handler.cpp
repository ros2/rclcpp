// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include "./signal_handler.hpp"

#include <atomic>
#include <csignal>
#include <mutex>
#include <string>
#include <thread>

// includes for semaphore notification code
#if defined(_WIN32)
#include <windows.h>
#elif defined(__APPLE__)
#include <dispatch/dispatch.h>
#else  // posix
#include <semaphore.h>
#endif

#include "rclcpp/logging.hpp"
#include "rcutils/strerror.h"
#include "rmw/impl/cpp/demangle.hpp"

using rclcpp::SignalHandler;

// initialize static storage in SignalHandler class
SignalHandler::signal_handler_type SignalHandler::old_signal_handler_;
std::atomic_bool SignalHandler::signal_received_ = ATOMIC_VAR_INIT(false);
std::atomic_bool SignalHandler::wait_for_signal_is_setup_ = ATOMIC_VAR_INIT(false);
#if defined(_WIN32)
HANDLE SignalHandler::signal_handler_sem_;
#elif defined(__APPLE__)
dispatch_semaphore_t SignalHandler::signal_handler_sem_;
#else  // posix
sem_t SignalHandler::signal_handler_sem_;
#endif

// The logger must be initialized before the local static variable signal_handler,
// from the method get_global_signal_handler(), so that it is destructed after
// it, because the destructor of SignalHandler uses this logger object.
static rclcpp::Logger g_logger = rclcpp::get_logger("rclcpp");

rclcpp::Logger &
SignalHandler::get_logger()
{
  return g_logger;
}

SignalHandler &
SignalHandler::get_global_signal_handler()
{
  // This is initialized after the g_logger static global, ensuring
  // SignalHandler::get_logger() may be called from the destructor of
  // SignalHandler, according to this:
  //
  //   Variables declared at block scope with the specifier static have static
  //   storage duration but are initialized the first time control passes
  //   through their declaration (unless their initialization is zero- or
  //   constant-initialization, which can be performed before the block is
  //   first entered). On all further calls, the declaration is skipped.
  //
  // -- https://en.cppreference.com/w/cpp/language/storage_duration#Static_local_variables
  //
  // Which is guaranteed to occur after static initialization for global (see:
  // https://en.cppreference.com/w/cpp/language/initialization#Static_initialization),
  // which is when g_logger will be initialized.
  // And destruction will occur in the reverse order.
  static SignalHandler signal_handler;
  return signal_handler;
}

bool
SignalHandler::install()
{
  std::lock_guard<std::mutex> lock(install_mutex_);
  bool already_installed = installed_.exchange(true);
  if (already_installed) {
    return false;
  }
  try {
    setup_wait_for_signal();
    signal_received_.store(false);

    SignalHandler::signal_handler_type signal_handler_argument;
#if defined(RCLCPP_HAS_SIGACTION)
    memset(&signal_handler_argument, 0, sizeof(signal_handler_argument));
    sigemptyset(&signal_handler_argument.sa_mask);
    signal_handler_argument.sa_sigaction = signal_handler;
    signal_handler_argument.sa_flags = SA_SIGINFO;
#else
    signal_handler_argument = signal_handler;
#endif

    old_signal_handler_ = SignalHandler::set_signal_handler(SIGINT, signal_handler_argument);

    signal_handler_thread_ = std::thread(&SignalHandler::deferred_signal_handler, this);
  } catch (...) {
    installed_.store(false);
    throw;
  }
  RCLCPP_DEBUG(get_logger(), "signal handler installed");
  return true;
}

bool
SignalHandler::uninstall()
{
  std::lock_guard<std::mutex> lock(install_mutex_);
  bool installed = installed_.exchange(false);
  if (!installed) {
    return false;
  }
  try {
    // TODO(wjwwood): what happens if someone overrides our signal handler then calls uninstall?
    //   I think we need to assert that we're the current signal handler, and mitigate if not.
    set_signal_handler(SIGINT, old_signal_handler_);
    RCLCPP_DEBUG(get_logger(), "SignalHandler::uninstall(): notifying deferred signal handler");
    notify_signal_handler();
    signal_handler_thread_.join();
    teardown_wait_for_signal();
  } catch (...) {
    installed_.exchange(true);
    throw;
  }
  RCLCPP_DEBUG(get_logger(), "signal handler uninstalled");
  return true;
}

bool
SignalHandler::is_installed()
{
  return installed_.load();
}

SignalHandler::~SignalHandler()
{
  try {
    uninstall();
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(
      get_logger(),
      "caught %s exception when uninstalling the sigint handler in rclcpp::~SignalHandler: %s",
      rmw::impl::cpp::demangle(exc).c_str(), exc.what());
  } catch (...) {
    RCLCPP_ERROR(
      get_logger(),
      "caught unknown exception when uninstalling the sigint handler in rclcpp::~SignalHandler");
  }
}

SignalHandler::signal_handler_type
SignalHandler::set_signal_handler(
  int signal_value,
  const SignalHandler::signal_handler_type & signal_handler)
{
  bool signal_handler_install_failed;
  SignalHandler::signal_handler_type old_signal_handler;
#if defined(RCLCPP_HAS_SIGACTION)
  ssize_t ret = sigaction(signal_value, &signal_handler, &old_signal_handler);
  signal_handler_install_failed = (ret == -1);
#else
  old_signal_handler = std::signal(signal_value, signal_handler);
  signal_handler_install_failed = (old_signal_handler == SIG_ERR);
#endif
  if (signal_handler_install_failed) {
    char error_string[1024];
    rcutils_strerror(error_string, sizeof(error_string));
    auto msg =
      "Failed to set SIGINT signal handler (" + std::to_string(errno) + "): " + error_string;
    throw std::runtime_error(msg);
  }

  return old_signal_handler;
}

void
SignalHandler::signal_handler_common()
{
  signal_received_.store(true);
  RCLCPP_DEBUG(
    get_logger(),
    "signal_handler(): SIGINT received, notifying deferred signal handler");
  notify_signal_handler();
}

#if defined(RCLCPP_HAS_SIGACTION)
void
SignalHandler::signal_handler(int signal_value, siginfo_t * siginfo, void * context)
{
  RCLCPP_INFO(get_logger(), "signal_handler(signal_value=%d)", signal_value);

  if (old_signal_handler_.sa_flags & SA_SIGINFO) {
    if (old_signal_handler_.sa_sigaction != NULL) {
      old_signal_handler_.sa_sigaction(signal_value, siginfo, context);
    }
  } else {
    if (
      old_signal_handler_.sa_handler != NULL &&  // Is set
      old_signal_handler_.sa_handler != SIG_DFL &&  // Is not default
      old_signal_handler_.sa_handler != SIG_IGN)  // Is not ignored
    {
      old_signal_handler_.sa_handler(signal_value);
    }
  }

  signal_handler_common();
}
#else
void
SignalHandler::signal_handler(int signal_value)
{
  RCLCPP_INFO(get_logger(), "signal_handler(signal_value=%d)", signal_value);

  if (old_signal_handler_) {
    old_signal_handler_(signal_value);
  }

  signal_handler_common();
}
#endif

void
SignalHandler::deferred_signal_handler()
{
  while (true) {
    if (signal_received_.exchange(false)) {
      RCLCPP_DEBUG(get_logger(), "deferred_signal_handler(): SIGINT received, shutting down");
      for (auto context_ptr : rclcpp::get_contexts()) {
        if (context_ptr->get_init_options().shutdown_on_sigint) {
          RCLCPP_DEBUG(
            get_logger(),
            "deferred_signal_handler(): "
            "shutting down rclcpp::Context @ %p, because it had shutdown_on_sigint == true",
            static_cast<void *>(context_ptr.get()));
          context_ptr->shutdown("signal handler");
        }
      }
    }
    if (!is_installed()) {
      RCLCPP_DEBUG(get_logger(), "deferred_signal_handler(): signal handling uninstalled");
      break;
    }
    RCLCPP_DEBUG(get_logger(), "deferred_signal_handler(): waiting for SIGINT or uninstall");
    wait_for_signal();
    RCLCPP_DEBUG(get_logger(), "deferred_signal_handler(): woken up due to SIGINT or uninstall");
  }
}

void
SignalHandler::setup_wait_for_signal()
{
#if defined(_WIN32)
  signal_handler_sem_ = CreateSemaphore(
    NULL,  // default security attributes
    0,  // initial semaphore count
    1,  // maximum semaphore count
    NULL);  // unnamed semaphore
  if (NULL == signal_handler_sem_) {
    throw std::runtime_error("CreateSemaphore() failed in setup_wait_for_signal()");
  }
#elif defined(__APPLE__)
  signal_handler_sem_ = dispatch_semaphore_create(0);
#else  // posix
  if (-1 == sem_init(&signal_handler_sem_, 0, 0)) {
    throw std::runtime_error(std::string("sem_init() failed: ") + strerror(errno));
  }
#endif
  wait_for_signal_is_setup_.store(true);
}

void
SignalHandler::teardown_wait_for_signal() noexcept
{
  if (!wait_for_signal_is_setup_.exchange(false)) {
    return;
  }
#if defined(_WIN32)
  CloseHandle(signal_handler_sem_);
#elif defined(__APPLE__)
  dispatch_release(signal_handler_sem_);
#else  // posix
  if (-1 == sem_destroy(&signal_handler_sem_)) {
    RCLCPP_ERROR(get_logger(), "invalid semaphore in teardown_wait_for_signal()");
  }
#endif
}

void
SignalHandler::wait_for_signal()
{
  if (!wait_for_signal_is_setup_.load()) {
    RCLCPP_ERROR(get_logger(), "called wait_for_signal() before setup_wait_for_signal()");
    return;
  }
#if defined(_WIN32)
  DWORD dw_wait_result = WaitForSingleObject(signal_handler_sem_, INFINITE);
  switch (dw_wait_result) {
    case WAIT_ABANDONED:
      RCLCPP_ERROR(
        get_logger(), "WaitForSingleObject() failed in wait_for_signal() with WAIT_ABANDONED: %s",
        GetLastError());
      break;
    case WAIT_OBJECT_0:
      // successful
      break;
    case WAIT_TIMEOUT:
      RCLCPP_ERROR(get_logger(), "WaitForSingleObject() timedout out in wait_for_signal()");
      break;
    case WAIT_FAILED:
      RCLCPP_ERROR(
        get_logger(), "WaitForSingleObject() failed in wait_for_signal(): %s", GetLastError());
      break;
    default:
      RCLCPP_ERROR(
        get_logger(), "WaitForSingleObject() gave unknown return in wait_for_signal(): %s",
        GetLastError());
  }
#elif defined(__APPLE__)
  dispatch_semaphore_wait(signal_handler_sem_, DISPATCH_TIME_FOREVER);
#else  // posix
  int s;
  do {
    s = sem_wait(&signal_handler_sem_);
  } while (-1 == s && EINTR == errno);
#endif
}

void
SignalHandler::notify_signal_handler() noexcept
{
  if (!wait_for_signal_is_setup_.load()) {
    return;
  }
#if defined(_WIN32)
  if (!ReleaseSemaphore(signal_handler_sem_, 1, NULL)) {
    RCLCPP_ERROR(
      get_logger(), "ReleaseSemaphore() failed in notify_signal_handler(): %s", GetLastError());
  }
#elif defined(__APPLE__)
  dispatch_semaphore_signal(signal_handler_sem_);
#else  // posix
  if (-1 == sem_post(&signal_handler_sem_)) {
    RCLCPP_ERROR(get_logger(), "sem_post failed in notify_signal_handler()");
  }
#endif
}
