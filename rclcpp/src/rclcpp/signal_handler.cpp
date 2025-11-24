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
#include <future>
#include <optional>
#include <mutex>
#include <string>
#include <thread>

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
  instance.signal_handler_common(signum);
}
#else
void
SignalHandler::signal_handler(int signum)
{
  auto & instance = SignalHandler::get_global_signal_handler();
  auto old_signal_handler = instance.get_old_signal_handler(signum);
  if (
    SIG_ERR != old_signal_handler && SIG_IGN != old_signal_handler &&
    SIG_DFL != old_signal_handler)
  {
    old_signal_handler(signum);
  }
  instance.signal_handler_common(signum);
}
#endif

void rclcpp::SignalHandler::signal_handler_common(int signum) noexcept
{
  switch(signum) {
    case SIGTERM:
      notify_deferred_handler(Input::SigTerm);
      break;
    case SIGINT:
      notify_deferred_handler(Input::SigInt);
      break;
    default:
      break;
  }
}


rclcpp::Logger &
SignalHandler::get_logger()
{
  return SignalHandler::get_global_signal_handler().logger_;
}

SignalHandler &
SignalHandler::get_global_signal_handler()
{
  static SignalHandler & signal_handler = *new SignalHandler();
  return signal_handler;
}

bool
SignalHandler::install(SignalHandlerOptions signal_handler_options)
{
  std::lock_guard<std::mutex> lock(install_mutex_);
  if (installed_) {
    return false;
  }
  if (signal_handler_options == SignalHandlerOptions::None) {
    return true;
  }

  // Reset state in case someone uninstalls and reinstall handlers
  got_sig_int = false;
  got_sig_term = false;
  terminate_handler_ = false;

  signal_handlers_options_ = signal_handler_options;
  try {
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
    throw;
  }
  installed_ = true;

  RCLCPP_DEBUG(get_logger(), "signal handler installed");
  return true;
}

bool
SignalHandler::uninstall()
{
  std::lock_guard<std::mutex> lock(install_mutex_);
  if (!installed_) {
    return false;
  }

  RCLCPP_DEBUG(get_logger(), "SignalHandler::uninstall(): shutting down deferred signal handler");
  notify_deferred_handler(Input::TerminateHandler);
  if (signal_handler_thread_.joinable()) {
    signal_handler_thread_.join();
  }

  try {
    RCLCPP_DEBUG(get_logger(), "SignalHandler::uninstall(): restoring signal handlers");
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
  } catch (...) {
    throw;
  }
  installed_ = false;
  RCLCPP_DEBUG(get_logger(), "signal handler uninstalled");
  return true;
}

bool
SignalHandler::is_installed()
{
  std::lock_guard<std::mutex> lock(install_mutex_);
  return installed_;
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
SignalHandler::deferred_signal_handler()
{
  bool running = true;
  while (running) {
    std::optional<Input> next;

    RCLCPP_DEBUG(
      get_logger(), "deferred_signal_handler(): waiting for SIGINT/SIGTERM or uninstall");
    {
      std::unique_lock l(signal_mutex_);
      signal_conditional_.wait(l, [this] () {
          return terminate_handler_ || got_sig_int || got_sig_term;
      });

      if(terminate_handler_.exchange(false)) {
        next = Input::TerminateHandler;
      }
      if(got_sig_int.exchange(false)) {
        RCLCPP_INFO(SignalHandler::get_logger(), "signal_handler(SIGINT)");
        next = Input::SigInt;
      }
      if(got_sig_term.exchange(false)) {
        RCLCPP_INFO(SignalHandler::get_logger(), "signal_handler(SIGTERM)");
        next = Input::SigTerm;
      }
    }
    RCLCPP_DEBUG(
      get_logger(), "deferred_signal_handler(): woken up due to SIGINT/SIGTERM or uninstall");

    // Note 'next' must always be valid at this point, if not
    // a throw is the expected behaviour
    switch(next.value()) {
      case Input::SigInt:
        [[fallthrough]];
      case Input::SigTerm:
        {
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
          break;
        }
      case Input::TerminateHandler:
        running = false;
        break;
    }
  }
}

void
SignalHandler::notify_deferred_handler(Input input) noexcept
{
  switch(input) {
    case Input::SigInt:
      got_sig_int.exchange(true);
      break;
    case Input::SigTerm:
      got_sig_term.exchange(true);
      break;
    case Input::TerminateHandler:
      terminate_handler_.exchange(true);
      break;
  }

  signal_conditional_.notify_one();
}

rclcpp::SignalHandlerOptions
SignalHandler::get_current_signal_handler_options()
{
  return signal_handlers_options_;
}
