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

#include "signal_handler.hpp"

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
#include "rclcpp/utilities.hpp"
#include "rcutils/strerror.h"
#include "rmw/impl/cpp/demangle.hpp"

using rclcpp::SignalHandler;
using rclcpp::SignalHandlerOptions;

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
      "Failed to set signal handler (" + std::to_string(errno) + "): " + error_string;
    throw std::runtime_error(msg);
  }
  return old_signal_handler;
}

// Unfortunately macros (or duplicated code) are needed here,
// as the signal handler must be a function pointer.
#if defined(RCLCPP_HAS_SIGACTION)
void
SignalHandler::signal_handler(
  int signum, siginfo_t * siginfo, void * context)
{
  RCLCPP_INFO(SignalHandler::get_logger(), "signal_handler(signum=%d)", signum);
  auto & instance = SignalHandler::get_global_signal_handler();

  auto old_signal_handler = instance.get_old_signal_handler(signum);
  if (old_signal_handler.sa_flags & SA_SIGINFO) {
    if (old_signal_handler.sa_sigaction != NULL) {
      old_signal_handler.sa_sigaction(signum, siginfo, context);
    }
  } else {
    if (
      old_signal_handler.sa_handler != NULL &&  /* Is set */
      old_signal_handler.sa_handler != SIG_DFL &&  /* Is not default*/
      old_signal_handler.sa_handler != SIG_IGN)  /* Is not ignored */
    {
      old_signal_handler.sa_handler(signum);
    }
  }
  instance.signal_handler_common();
}
#else
void
SignalHandler::signal_handler(int signum)
{
  RCLCPP_INFO(SignalHandler::get_logger(), "signal_handler(signum=%d)", signum);
  auto & instance = SignalHandler::get_global_signal_handler();
  auto old_signal_handler = instance.get_old_signal_handler(signum);
  if (
    SIG_ERR != old_signal_handler && SIG_IGN != old_signal_handler &&
    SIG_DFL != old_signal_handler)
  {
    old_signal_handler(signum);
  }
  instance.signal_handler_common();
}
#endif

rclcpp::Logger &
SignalHandler::get_logger()
{
  return SignalHandler::get_global_signal_handler().logger_;
}

SignalHandler &
SignalHandler::get_global_signal_handler()
{
  static SignalHandler signal_handler;
  return signal_handler;
}

bool
SignalHandler::install(SignalHandlerOptions signal_handler_options)
{
  std::lock_guard<std::mutex> lock(install_mutex_);
  bool already_installed = installed_.exchange(true);
  if (already_installed) {
    return false;
  }
  if (signal_handler_options == SignalHandlerOptions::None) {
    return true;
  }
  signal_handlers_options_ = signal_handler_options;
  try {
    setup_wait_for_signal();
    signal_received_.store(false);

    SignalHandler::signal_handler_type handler_argument;
#if defined(RCLCPP_HAS_SIGACTION)
    memset(&handler_argument, 0, sizeof(handler_argument));
    sigemptyset(&handler_argument.sa_mask);
    handler_argument.sa_sigaction = &this->signal_handler;
    handler_argument.sa_flags = SA_SIGINFO;
#else
    handler_argument = &this->signal_handler;
#endif
    if (
      signal_handler_options == SignalHandlerOptions::SigInt ||
      signal_handler_options == SignalHandlerOptions::All)
    {
      old_sigint_handler_ = set_signal_handler(SIGINT, handler_argument);
    }

    if (
      signal_handler_options == SignalHandlerOptions::SigTerm ||
      signal_handler_options == SignalHandlerOptions::All)
    {
      old_sigterm_handler_ = set_signal_handler(SIGTERM, handler_argument);
    }

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
    if (
      SignalHandlerOptions::SigInt == signal_handlers_options_ ||
      SignalHandlerOptions::All == signal_handlers_options_)
    {
      set_signal_handler(SIGINT, old_sigint_handler_);
    }
    if (
      SignalHandlerOptions::SigTerm == signal_handlers_options_ ||
      SignalHandlerOptions::All == signal_handlers_options_)
    {
      set_signal_handler(SIGTERM, old_sigterm_handler_);
    }
    signal_handlers_options_ = SignalHandlerOptions::None;
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
      "caught %s exception when uninstalling signal handlers in rclcpp::~SignalHandler: %s",
      rmw::impl::cpp::demangle(exc).c_str(), exc.what());
  } catch (...) {
    RCLCPP_ERROR(
      get_logger(),
      "caught unknown exception when uninstalling signal handlers in rclcpp::~SignalHandler");
  }
}

SignalHandler::signal_handler_type
SignalHandler::get_old_signal_handler(int signum)
{
  if (SIGINT == signum) {
    return old_sigint_handler_;
  } else if (SIGTERM == signum) {
    return old_sigterm_handler_;
  }
#if defined(RCLCPP_HAS_SIGACTION)
  SignalHandler::signal_handler_type ret;
  memset(&ret, 0, sizeof(ret));
  sigemptyset(&ret.sa_mask);
  ret.sa_handler = SIG_DFL;
  return ret;
#else
  return SIG_DFL;
#endif
}

void
SignalHandler::signal_handler_common()
{
  auto & instance = SignalHandler::get_global_signal_handler();
  instance.signal_received_.store(true);
  RCLCPP_DEBUG(
    get_logger(),
    "signal_handler(): notifying deferred signal handler");
  instance.notify_signal_handler();
}

void
SignalHandler::deferred_signal_handler()
{
  while (true) {
    if (signal_received_.exchange(false)) {
      RCLCPP_DEBUG(get_logger(), "deferred_signal_handler(): shutting down");
      for (auto context_ptr : rclcpp::get_contexts()) {
        if (context_ptr->get_init_options().shutdown_on_signal) {
          RCLCPP_DEBUG(
            get_logger(),
            "deferred_signal_handler(): "
            "shutting down rclcpp::Context @ %p, because it had shutdown_on_signal == true",
            static_cast<void *>(context_ptr.get()));
          context_ptr->shutdown("signal handler");
        }
      }
    }
    if (!is_installed()) {
      RCLCPP_DEBUG(get_logger(), "deferred_signal_handler(): signal handling uninstalled");
      break;
    }
    RCLCPP_DEBUG(
      get_logger(), "deferred_signal_handler(): waiting for SIGINT/SIGTERM or uninstall");
    wait_for_signal();
    RCLCPP_DEBUG(
      get_logger(), "deferred_signal_handler(): woken up due to SIGINT/SIGTERM or uninstall");
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

rclcpp::SignalHandlerOptions
SignalHandler::get_current_signal_handler_options()
{
  return signal_handlers_options_;
}
