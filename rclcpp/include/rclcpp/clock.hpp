// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__CLOCK_HPP_
#define RCLCPP__CLOCK_HPP_

#include <functional>
#include <memory>
#include <mutex>

#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/visibility_control.hpp"

#include "rcl/time.h"
#include "rcutils/time.h"
#include "rcutils/types/rcutils_ret.h"

namespace rclcpp
{

class TimeSource;

class JumpHandler
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(JumpHandler)

  using pre_callback_t = std::function<void ()>;
  using post_callback_t = std::function<void (const rcl_time_jump_t &)>;

  JumpHandler(
    pre_callback_t pre_callback,
    post_callback_t post_callback,
    const rcl_jump_threshold_t & threshold);

  pre_callback_t pre_callback;
  post_callback_t post_callback;
  rcl_jump_threshold_t notice_threshold;
};

class Clock
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Clock)

  /// Default c'tor
  /**
   * Initializes the clock instance with the given clock_type.
   *
   * WARNING Don't instantiate a clock using RCL_ROS_TIME directly,
   * unless you really know what you are doing. By default no TimeSource
   * is attached to a new clock. This will lead to the unexpected behavior,
   * that your RCL_ROS_TIME will run always on system time. If you want
   * a RCL_ROS_TIME use Node::get_clock(), or make sure to attach a
   * TimeSource yourself.
   *
   * \param clock_type type of the clock.
   * \throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   */
  RCLCPP_PUBLIC
  explicit Clock(rcl_clock_type_t clock_type = RCL_SYSTEM_TIME);

  RCLCPP_PUBLIC
  ~Clock();

  /**
   * Returns current time from the time source specified by clock_type.
   *
   * \return current time.
   * \throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   */
  RCLCPP_PUBLIC
  Time
  now() const;

  /**
   * Sleep until a specified Time, according to clock type.
   *
   * Notes for RCL_ROS_TIME clock type:
   *   - Can sleep forever if ros time is active and received clock never reaches `until`
   *   - If ROS time enabled state changes during the sleep, this method will immediately return
   *     false. There is not a consistent choice of sleeping time when the time source changes,
   *     so this is up to the caller to call again if needed.
   *
   * \warning When using gcc < 10 or when using gcc >= 10 and pthreads lacks the function
   *    `pthread_cond_clockwait`, steady clocks may sleep using the system clock.
   *    If so, steady clock sleep times can be affected by system clock time jumps.
   *    Depending on the steady clock's epoch and resolution in comparison to the system clock's,
   *    an overflow when converting steady clock durations to system clock times may cause
   *    undefined behavior.
   *    For more info see these issues:
   *    https://gcc.gnu.org/bugzilla/show_bug.cgi?id=41861
   *    https://gcc.gnu.org/bugzilla/show_bug.cgi?id=58931
   *
   * \param until absolute time according to current clock type to sleep until.
   * \param context the rclcpp context the clock should use to check that ROS is still initialized.
   * \return true immediately if `until` is in the past
   * \return true when the time `until` is reached
   * \return false if time cannot be reached reliably, for example from shutdown or a change
   *    of time source.
   * \throws std::runtime_error if the context is invalid
   * \throws std::runtime_error if `until` has a different clock type from this clock
   */
  RCLCPP_PUBLIC
  bool
  sleep_until(
    Time until,
    Context::SharedPtr context = contexts::get_global_default_context());

  /**
   * Sleep for a specified Duration.
   *
   * Equivalent to
   *
   * ```cpp
   * clock->sleep_until(clock->now() + rel_time, context)
   * ```
   *
   * The function will return immediately if `rel_time` is zero or negative.
   *
   * \param rel_time the length of time to sleep for.
   * \param context the rclcpp context the clock should use to check that ROS is still initialized.
   * \return true when the end time is reached
   * \return false if time cannot be reached reliably, for example from shutdown or a change
   *    of time source.
   * \throws std::runtime_error if the context is invalid
   */
  RCLCPP_PUBLIC
  bool
  sleep_for(
    Duration rel_time,
    Context::SharedPtr context = contexts::get_global_default_context());

  /**
   * Check if the clock is started.
   *
   * A started clock is a clock that reflects non-zero time.
   * Typically a clock will be unstarted if it is using RCL_ROS_TIME with ROS time and
   * nothing has been published on the clock topic yet.
   *
   * \return true if clock is started
   * \throws std::runtime_error if the clock is not rcl_clock_valid
   */
  RCLCPP_PUBLIC
  bool
  started();

  /**
   * Wait until clock to start.
   *
   * \rclcpp::Clock::started
   * \param context the context to wait in
   * \return true if clock was already started or became started
   * \throws std::runtime_error if the context is invalid or clock is not rcl_clock_valid
   */
  RCLCPP_PUBLIC
  bool
  wait_until_started(Context::SharedPtr context = contexts::get_global_default_context());

  /**
   * Wait for clock to start, with timeout.
   *
   * The timeout is waited in steady time.
   *
   * \rclcpp::Clock::started
   * \param timeout the maximum time to wait for.
   * \param context the context to wait in.
   * \param wait_tick_ns the time to wait between each iteration of the wait loop (in nanoseconds).
   * \return true if clock was or became valid
   * \throws std::runtime_error if the context is invalid or clock is not rcl_clock_valid
   */
  RCLCPP_PUBLIC
  bool
  wait_until_started(
    const rclcpp::Duration & timeout,
    Context::SharedPtr context = contexts::get_global_default_context(),
    const rclcpp::Duration & wait_tick_ns = rclcpp::Duration(0, static_cast<uint32_t>(1e7)));

  /**
   * Returns the clock of the type `RCL_ROS_TIME` is active.
   *
   * \return true if the clock is active
   * \throws anything rclcpp::exceptions::throw_from_rcl_error can throw if
   * the current clock does not have the clock_type `RCL_ROS_TIME`.
   */
  RCLCPP_PUBLIC
  bool
  ros_time_is_active();

  /**
   * Deprecated. This API is broken, as there is no way to get a deep
   * copy of a clock. Therefore one can experience spurious wakeups triggered
   * by some other instance of a clock.
   *
   * Cancels an ongoing or future sleep operation of one thread.
   *
   * This function can be used by one thread, to wakeup another thread that is
   * blocked using any of the sleep_ or wait_ methods of this class.
   */
  [[deprecated("Use ClockConditionalVariable")]]
  RCLCPP_PUBLIC
  void
  cancel_sleep_or_wait();

  /// Return the rcl_clock_t clock handle
  RCLCPP_PUBLIC
  rcl_clock_t *
  get_clock_handle() noexcept;

  RCLCPP_PUBLIC
  rcl_clock_type_t
  get_clock_type() const noexcept;

  /// Get the clock's mutex
  RCLCPP_PUBLIC
  std::mutex &
  get_clock_mutex() noexcept;

  /// Add a callback to invoke if the jump threshold is exceeded.
  /**
   * These callback functions must remain valid as long as the
   * returned shared pointer is valid.
   *
   * Function will register callbacks to the callback queue. On time jump all
   * callbacks will be executed whose threshold is greater than the time jump;
   * The logic will first call selected pre_callbacks and then all selected
   * post_callbacks.
   *
   * Function is only applicable if the clock_type is `RCL_ROS_TIME`
   *
   * \param pre_callback Must be non-throwing
   * \param post_callback Must be non-throwing.
   * \param threshold Callbacks will be triggered if the time jump is greater
   * than the threshold.
   * \throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   * \throws std::bad_alloc if the allocation of the JumpHandler fails.
   * \warning the instance of the clock must remain valid as long as any created
   * JumpHandler.
   */
  RCLCPP_PUBLIC
  JumpHandler::SharedPtr
  create_jump_callback(
    JumpHandler::pre_callback_t pre_callback,
    JumpHandler::post_callback_t post_callback,
    const rcl_jump_threshold_t & threshold);

private:
  // Invoke time jump callback
  RCLCPP_PUBLIC
  static void
  on_time_jump(
    const rcl_time_jump_t * time_jump,
    bool before_jump,
    void * user_data);

  /// Private internal storage
  class Impl;

  std::shared_ptr<Impl> impl_;
};

/**
 * A synchronization primitive, equal to std::conditional_variable,
 * that works with the rclcpp::Clock.
 *
 * For more information on the API see https://en.cppreference.com/w/cpp/thread/condition_variable.
 *
 * Note, this class does not handle shutdowns, if you want to
 * haven them handles as well, use ClockConditionalVariable.
 */
class ClockWaiter
{
private:
  class ClockWaiterImpl;
  std::unique_ptr<ClockWaiterImpl> impl_;

public:
  RCLCPP_SMART_PTR_DEFINITIONS(ClockWaiter)

  RCLCPP_PUBLIC
  explicit ClockWaiter(const rclcpp::Clock::SharedPtr & clock);

  RCLCPP_PUBLIC
  ~ClockWaiter();

  /**
   * Calling this function will block the current thread, until abs_time is reached,
   * or pred returns true.
   * @param lock A locked lock. The lock must be locked at call time, or this method will throw.
   *             The lock will be atomically released and this thread will blocked.
   * @param abs_time The time until which this thread shall be blocked.
   * @param pred may be called in cased of spurious wakeups, but must be called every time
   *             notify_one() was called. During the call to pred, the given lock will be locked.
   *             This method will return, if pred returns true.
   */
  RCLCPP_PUBLIC
  bool
  wait_until(
    std::unique_lock<std::mutex> & lock,
    const rclcpp::Time & abs_time, const std::function<bool ()> & pred);

  /**
   * Notify the blocked thread, that it should reevaluate the wakeup condition.
   * The given pred function in wait_until will be reevaluated and wait_until
   * will return if it evaluates to true.
   */
  RCLCPP_PUBLIC
  void
  notify_one();
};


/**
 * A synchronization primitive, similar to std::conditional_variable,
 * that works with the rclcpp::Clock.
 *
 * For more information on the API see https://en.cppreference.com/w/cpp/thread/condition_variable.
 *
 * This primitive will wake up if the context was shut down.
 */
class ClockConditionalVariable
{
  class Impl;
  std::unique_ptr<Impl> impl_;

public:
  RCLCPP_SMART_PTR_DEFINITIONS(ClockConditionalVariable)

  RCLCPP_PUBLIC
  ClockConditionalVariable(
    const rclcpp::Clock::SharedPtr & clock,
    rclcpp::Context::SharedPtr context = rclcpp::contexts::get_global_default_context());
  RCLCPP_PUBLIC
  ~ClockConditionalVariable();

  /**
   * Calling this function will block the current thread, until abs_time is reached,
   * or pred returns true.
   * @param lock A locked lock. The lock must be locked at call time, or this method will throw.
   *             The lock will be atomically released and this thread will blocked.
   *             The given lock must be created using the mutex returned my mutex().
   * @param abs_time The time until which this thread shall be blocked.
   * @param pred may be called in cased of spurious wakeups, but must be called every time
   *             notify_one() was called. During the call to pred, the given lock will be locked.
   *             This method will return, if pred returns true.
   *
   * @return true if until was reached.
   */
  RCLCPP_PUBLIC
  bool
  wait_until(
    std::unique_lock<std::mutex> & lock, rclcpp::Time until,
    const std::function<bool ()> & pred);

  /**
   * Notify the blocked thread, that is should reevaluate the wakeup condition.
   * E.g. the given pred function in wait_until shall be reevaluated.
   */
  RCLCPP_PUBLIC
  void
  notify_one();

  /**
   * Returns the internal mutex. In order to be race free with the context shutdown,
   * this mutex must be used for the wait_until call.
   */
  RCLCPP_PUBLIC
  std::mutex &
  mutex();
};


}  // namespace rclcpp

#endif  // RCLCPP__CLOCK_HPP_
