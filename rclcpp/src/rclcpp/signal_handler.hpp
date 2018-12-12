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
#include <mutex>
#include <thread>

#include "rclcpp/logging.hpp"

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

/// Responsible for manaaging the SIGINT signal handling.
/**
 * This class is responsible for:
 *
 * - installing the signal handler for SIGINT
 * - uninstalling the signal handler for SIGINT
 * - creating a thread to execute "on sigint" work outside of the signal handler
 * - safely notifying the dedicated signal handling thread when receiving SIGINT
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

  /// Install the signal handler for SIGINT and start the dedicated signal handling thread.
  /**
   * Also stores the current signal handler to be called on SIGINT and to
   * restore when uninstalling this signal handler.
   */
  bool
  install();

  /// Uninstall the signal handler for SIGINT and join the dedicated singal handling thread.
  /**
   * Also restores the previous signal handler.
   */
  bool
  uninstall();

  /// Return true if installed, false otherwise.
  bool
  is_installed();

private:
  SignalHandler() = default;

  ~SignalHandler();

#if defined(RCLCPP_HAS_SIGACTION)
  using signal_handler_type = struct sigaction;
#else
  using signal_handler_type = void (*)(int);
#endif
  // POSIX signal handler structure storage for the existing signal handler.
  static SignalHandler::signal_handler_type old_signal_handler_;

  /// Set the signal handler function.
  static
  SignalHandler::signal_handler_type
  set_signal_handler(int signal_value, const SignalHandler::signal_handler_type & signal_handler);

  /// Common signal handler code between sigaction and non-sigaction versions.
  static
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
  static
  void
  setup_wait_for_signal();

  /// Undo all setup done in setup_wait_for_signal().
  /**
   * Must not call wait_for_signal() or notify_signal_handler() after calling this.
   *
   * This is not thread-safe.
   */
  static
  void
  teardown_wait_for_signal() noexcept;

  /// Wait for a notification from notify_signal_handler() in a signal safe way.
  /**
   * This static method may throw if posting the semaphore fails.
   *
   * This is not thread-safe.
   */
  static
  void
  wait_for_signal();

  /// Notify blocking wait_for_signal() calls in a signal safe way.
  /**
   * This is used to notify the deferred_signal_handler() thread to start work
   * from the signal handler.
   *
   * This is thread-safe.
   */
  static
  void
  notify_signal_handler() noexcept;

  // Whether or not a signal has been received.
  static std::atomic_bool signal_received_;
  // A thread to which singal handling tasks are deferred.
  std::thread signal_handler_thread_;

  // A mutex used to synchronize the install() and uninstall() methods.
  std::mutex install_mutex_;
  // Whether or not the signal handler has been installed.
  std::atomic_bool installed_{false};

  // Whether or not the semaphore for wait_for_signal is setup.
  static std::atomic_bool wait_for_signal_is_setup_;
  // Storage for the wait_for_signal semaphore.
#if defined(_WIN32)
  static HANDLE signal_handler_sem_;
#elif defined(__APPLE__)
  static dispatch_semaphore_t signal_handler_sem_;
#else  // posix
  static sem_t signal_handler_sem_;
#endif
};

}  // namespace rclcpp

#endif  // RCLCPP__SIGNAL_HANDLER_HPP_
