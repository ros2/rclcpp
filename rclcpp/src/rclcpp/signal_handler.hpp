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

#ifndef RCLCPP__SIGNAL_HANDLER_HPP_
#define RCLCPP__SIGNAL_HANDLER_HPP_

#include <atomic>
#include <csignal>
#include <thread>

#include "rclcpp/logging.hpp"
#include "rclcpp/utilities.hpp"

#include "rcpputils/mutex.hpp"

// includes for semaphore notification code
#if defined(_WIN32)
#include <windows.h>
#elif defined(__APPLE__)
#include <dispatch/dispatch.h>
#else  // posix
#include <semaphore.h>
#endif

// Determine if sigaction is available
#if __APPLE__ || _POSIX_C_SOURCE >= 1 || _XOPEN_SOURCE || _POSIX_SOURCE
#define RCLCPP_HAS_SIGACTION
#endif

namespace rclcpp
{

/// Responsible for managing the SIGINT/SIGTERM signal handling.
/**
 * This class is responsible for:
 *
 * - installing the signal handler for SIGINT/SIGTERM
 * - uninstalling the signal handler for SIGINT/SIGTERM
 * - creating a thread to execute "on signal" work outside of the signal handler
 * - safely notifying the dedicated signal handling thread when receiving SIGINT/SIGTERM
 * - implementation of all of the signal handling work, like shutting down contexts
 *
 * \internal
 */
class SignalHandler final
{
public:
  /// Return the global singleton of this class.
  static
  SignalHandler &
  get_global_signal_handler();

  /// Return a global singleton logger to avoid needing to create it everywhere.
  static
  rclcpp::Logger &
  get_logger();

  /// Install the signal handler for SIGINT/SIGTERM and start the dedicated signal handling thread.
  /**
   * Also stores the current signal handler to be called on signal and to
   * restore when uninstalling this signal handler.
   *
   * \param signal_handler_options option to indicate which signal handlers should be installed.
   */
  bool
  install(SignalHandlerOptions signal_handler_options = SignalHandlerOptions::All);

  /// Uninstall the signal handler for SIGINT/SIGTERM and join the dedicated singal handling
  /// thread.
  /**
   * Also restores the previous signal handler.
   */
  bool
  uninstall();

  /// Return true if installed, false otherwise.
  bool
  is_installed();

  /// Get the current signal handler options.
  /**
   * If no signal handler is installed, SignalHandlerOptions::None is returned.
   */
  rclcpp::SignalHandlerOptions
  get_current_signal_handler_options();

private:
  /// Signal handler type, platform dependent.
#if defined(RCLCPP_HAS_SIGACTION)
  using signal_handler_type = struct sigaction;
#else
  using signal_handler_type = void (*)(int);
#endif


  SignalHandler() = default;

  ~SignalHandler();

  SignalHandler(const SignalHandler &) = delete;
  SignalHandler(SignalHandler &&) = delete;
  SignalHandler &
  operator=(const SignalHandler &) = delete;
  SignalHandler &&
  operator=(SignalHandler &&) = delete;

  /// Common signal handler code between sigaction and non-sigaction versions.
  void
  signal_handler_common();

#if defined(RCLCPP_HAS_SIGACTION)
  /// Signal handler function.
  static
  void
  signal_handler(int signal_value, siginfo_t * siginfo, void * context);
#else
  /// Signal handler function.
  static
  void
  signal_handler(int signal_value);
#endif

  /// Target of the dedicated signal handling thread.
  void
  deferred_signal_handler();

  /// Setup anything that is necessary for wait_for_signal() or notify_signal_handler().
  /**
   * This must be called before wait_for_signal() or notify_signal_handler().
   * This is not thread-safe.
   */
  void
  setup_wait_for_signal();

  /// Undo all setup done in setup_wait_for_signal().
  /**
   * Must not call wait_for_signal() or notify_signal_handler() after calling this.
   *
   * This is not thread-safe.
   */
  void
  teardown_wait_for_signal() noexcept;

  /// Wait for a notification from notify_signal_handler() in a signal safe way.
  /**
   * This static method may throw if posting the semaphore fails.
   *
   * This is not thread-safe.
   */
  void
  wait_for_signal();

  /// Notify blocking wait_for_signal() calls in a signal safe way.
  /**
   * This is used to notify the deferred_signal_handler() thread to start work
   * from the signal handler.
   *
   * This is thread-safe.
   */
  void
  notify_signal_handler() noexcept;

  static
  signal_handler_type
  set_signal_handler(
    int signal_value,
    const signal_handler_type & signal_handler);

  signal_handler_type
  get_old_signal_handler(int signum);

  rclcpp::SignalHandlerOptions signal_handlers_options_ = rclcpp::SignalHandlerOptions::None;

  signal_handler_type old_sigint_handler_;
  signal_handler_type old_sigterm_handler_;

  // logger instance
  rclcpp::Logger logger_ = rclcpp::get_logger("rclcpp");

  // Whether or not a signal has been received.
  std::atomic_bool signal_received_ = false;
  // A thread to which singal handling tasks are deferred.
  std::thread signal_handler_thread_;

  // A mutex used to synchronize the install() and uninstall() methods.
  rcpputils::PIMutex install_mutex_;
  // Whether or not the signal handler has been installed.
  std::atomic_bool installed_ = false;

  // Whether or not the semaphore for wait_for_signal is setup.
  std::atomic_bool wait_for_signal_is_setup_;
  // Storage for the wait_for_signal semaphore.
#if defined(_WIN32)
  HANDLE signal_handler_sem_;
#elif defined(__APPLE__)
  dispatch_semaphore_t signal_handler_sem_;
#else  // posix
  sem_t signal_handler_sem_;
#endif
};

}  // namespace rclcpp

#endif  // RCLCPP__SIGNAL_HANDLER_HPP_
