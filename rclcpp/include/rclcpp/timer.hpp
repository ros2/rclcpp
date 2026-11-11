// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__TIMER_HPP_
#define RCLCPP__TIMER_HPP_

#include <atomic>
#include <chrono>
#include <functional>
#include <optional>
#include <memory>
#include <thread>
#include <type_traits>
#include <utility>

#include "rclcpp/clock.hpp"
#include "rclcpp/context.hpp"
#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/function_traits.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/visibility_control.hpp"
#include "tracetools/tracetools.h"
#include "tracetools/utils.hpp"

#include "rcl/error_handling.h"
#include "rcl/timer.h"

#include "rmw/rmw.h"

namespace rclcpp
{

struct TimerInfo
{
  Time expected_call_time;
  Time actual_call_time;
};

/// Compute a phase-aligned start time for a periodic timer.
/**
 * The result is the smallest time greater than or equal to `clock.now()`
 * of the form `k * interval + phase` for some non-negative integer `k`.
 *
 * This is useful for synchronizing periodic timers across multiple nodes
 * or processes that share a common, synchronized clock (e.g. ROS time)
 * without needing to exchange an explicit start time out-of-band: any two
 * callers with a synchronized clock computing this function with the same
 * interval and phase will agree on the same sequence of aligned instants.
 *
 * \param[in] clock clock used to obtain the current time
 * \param[in] interval alignment interval; must be greater than zero
 * \param[in] phase offset added to each interval boundary
 * \return the computed, phase-aligned start time, using clock's clock type
 * \throws std::invalid_argument if interval is not greater than zero
 */
RCLCPP_PUBLIC
Time
compute_phase_aligned_time(
  const Clock & clock,
  std::chrono::nanoseconds interval,
  std::chrono::nanoseconds phase = std::chrono::nanoseconds(0));

class TimerBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(TimerBase)

  /// TimerBase constructor
  /**
   * \param clock A clock to use for time and sleeping
   * \param period The interval at which the timer fires
   * \param context node context
   * \param autostart timer state on initialization
   *
   * In order to activate a timer that is not started on initialization,
   * user should call the reset() method.
   */
  RCLCPP_PUBLIC
  explicit TimerBase(
    Clock::SharedPtr clock,
    std::chrono::nanoseconds period,
    rclcpp::Context::SharedPtr context,
    bool autostart = true);

  /// TimerBase destructor
  RCLCPP_PUBLIC
  virtual
  ~TimerBase();

  /// Cancel the timer.
  /**
   * \throws std::runtime_error if the rcl_timer_cancel returns a failure
   */
  RCLCPP_PUBLIC
  void
  cancel();

  /// Return the timer cancellation state.
  /**
   * \return true if the timer has been cancelled, false otherwise
   * \throws std::runtime_error if the rcl_get_error_state returns 0
   * \throws rclcpp::exceptions::RCLError some child class exception based on ret
   */
  RCLCPP_PUBLIC
  bool
  is_canceled();

  /// Reset the timer.
  /**
   * \throws std::runtime_error if the rcl_timer_reset returns a failure
   */
  RCLCPP_PUBLIC
  void
  reset();

  /// Indicate that we're about to execute the callback.
  /**
   * The multithreaded executor takes advantage of this to avoid scheduling
   * the callback multiple times.
   *
   * \return a valid shared_ptr if the callback should be executed,
   *         an invalid shared_ptr (nullptr) if the timer was canceled.
   */
  RCLCPP_PUBLIC
  virtual std::shared_ptr<void>
  call() = 0;

  /// Call the callback function when the timer signal is emitted.
  /**
   * \param[in] data the pointer returned by the call function
   */
  RCLCPP_PUBLIC
  virtual void
  execute_callback(const std::shared_ptr<void> & data) = 0;

  RCLCPP_PUBLIC
  std::shared_ptr<const rcl_timer_t>
  get_timer_handle();

  /// Check how long the timer has until its next scheduled callback.
  /**
   * \return A std::chrono::duration representing the relative time until the next callback
   * or std::chrono::nanoseconds::max() if the timer is canceled.
   * \throws std::runtime_error if the rcl_timer_get_time_until_next_call returns a failure
   */
  RCLCPP_PUBLIC
  std::chrono::nanoseconds
  time_until_trigger();

  /// Is the clock steady (i.e. is the time between ticks constant?)
  /** \return True if the clock used by this timer is steady. */
  virtual bool is_steady() = 0;

  /// Check if the timer is ready to trigger the callback.
  /**
   * This function expects its caller to immediately trigger the callback after this function,
   * since it maintains the last time the callback was triggered.
   * \return True if the timer needs to trigger.
   * \throws std::runtime_error if it failed to check timer
   */
  RCLCPP_PUBLIC
  bool is_ready();

  /// Exchange the "in use by wait set" state for this timer.
  /**
   * This is used to ensure this timer is not used by multiple
   * wait sets at the same time.
   *
   * \param[in] in_use_state the new state to exchange into the state, true
   *   indicates it is now in use by a wait set, and false is that it is no
   *   longer in use by a wait set.
   * \returns the previous state.
   */
  RCLCPP_PUBLIC
  bool
  exchange_in_use_by_wait_set_state(bool in_use_state);

  /// Set a callback to be called when the timer is reset
  /**
   * You should aim to make this callback fast and not blocking.
   * If you need to do a lot of work or wait for some other event, you should
   * spin it off to another thread.
   *
   * Calling it again will override any previously set callback.
   * An exception will be thrown if the callback is not callable.
   *
   * This function is thread-safe.
   *
   * If you want more information available in the callback,
   * you may use a lambda with captures or std::bind.
   *
   * \param[in] callback functor to be called whenever timer is reset
   */
  RCLCPP_PUBLIC
  void
  set_on_reset_callback(const std::function<void(size_t)> & callback);

  /// Unset the callback registered for reset timer
  RCLCPP_PUBLIC
  void
  clear_on_reset_callback();

  /// Returns the clock this timer uses
  RCLCPP_PUBLIC
  const Clock::SharedPtr & get_clock() const;

protected:
  /// Constructor for subclasses that initialize `timer_handle_` themselves.
  /**
   * Subclasses that use an alternate rcl timer initialization function can delegate to this
   * constructor and call `create_timer_handle(context)` to allocate the zero-initialized
   * `timer_handle_` before initializing the rcl timer.
   *
   * \param clock A clock to use for time and sleeping
   */
  RCLCPP_PUBLIC
  explicit TimerBase(Clock::SharedPtr clock);

  /// Allocate a zero-initialized `rcl_timer_t` shared_ptr with its cleanup deleter.
  RCLCPP_PUBLIC
  std::shared_ptr<rcl_timer_t>
  create_timer_handle(rclcpp::Context::SharedPtr context);

  std::recursive_mutex callback_mutex_;
  // Declare callback before timer_handle_, so on destruction
  // the callback is destroyed last. Otherwise, the rcl timer
  // callback would point briefly to a destroyed function.
  // Clearing the callback on timer destructor also makes sure
  // the rcl callback is cleared before on_reset_callback_.
  std::function<void(size_t)> on_reset_callback_{nullptr};

  Clock::SharedPtr clock_;
  std::shared_ptr<rcl_timer_t> timer_handle_;

  std::atomic<bool> in_use_by_wait_set_{false};

  RCLCPP_PUBLIC
  void
  set_on_reset_callback(rcl_event_callback_t callback, const void * user_data);
};

using VoidCallbackType = std::function<void ()>;
using TimerCallbackType = std::function<void (TimerBase &)>;
using TimerInfoCallbackType = std::function<void (const TimerInfo &)>;

/// Generic timer. Periodically executes a user-specified callback.
template<
  typename FunctorT,
  typename std::enable_if<
    rclcpp::function_traits::same_arguments<FunctorT, VoidCallbackType>::value ||
    rclcpp::function_traits::same_arguments<FunctorT, TimerCallbackType>::value ||
    rclcpp::function_traits::same_arguments<FunctorT, TimerInfoCallbackType>::value
  >::type * = nullptr
>
class GenericTimer : public TimerBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(GenericTimer)

  /// Default constructor.
  /**
   * \param[in] clock The clock providing the current time.
   * \param[in] period The interval at which the timer fires.
   * \param[in] callback User-specified callback function.
   * \param[in] context custom context to be used.
   * \param autostart timer state on initialization
   */
  explicit GenericTimer(
    Clock::SharedPtr clock, std::chrono::nanoseconds period, FunctorT && callback,
    rclcpp::Context::SharedPtr context, bool autostart = true
  )
  : TimerBase(clock, period, context, autostart), callback_(std::forward<FunctorT>(callback))
  {
    TRACETOOLS_TRACEPOINT(
      rclcpp_timer_callback_added,
      static_cast<const void *>(get_timer_handle().get()),
      reinterpret_cast<const void *>(&callback_));
#ifndef TRACETOOLS_DISABLED
    if (TRACETOOLS_TRACEPOINT_ENABLED(rclcpp_callback_register)) {
      char * symbol = tracetools::get_symbol(callback_);
      TRACETOOLS_DO_TRACEPOINT(
        rclcpp_callback_register,
        reinterpret_cast<const void *>(&callback_),
        symbol);
      std::free(symbol);
    }
#endif
  }

  /// Default destructor.
  virtual ~GenericTimer()
  {
    // Stop the timer from running.
    if (timer_handle_) {
      cancel();
    }
  }

  /**
   * \sa rclcpp::TimerBase::call
   * \throws std::runtime_error if it failed to notify timer that callback will occurr
   */
  std::shared_ptr<void>
  call() override
  {
    auto timer_call_info_ = std::make_shared<rcl_timer_call_info_t>();
    rcl_ret_t ret = rcl_timer_call_with_info(timer_handle_.get(), timer_call_info_.get());
    if (ret == RCL_RET_TIMER_CANCELED) {
      return nullptr;
    }
    if (ret != RCL_RET_OK) {
      throw std::runtime_error("Failed to notify timer that callback occurred");
    }
    return timer_call_info_;
  }

  /**
   * \sa rclcpp::TimerBase::execute_callback
   */
  void
  execute_callback(const std::shared_ptr<void> & data) override
  {
    TRACETOOLS_TRACEPOINT(callback_start, reinterpret_cast<const void *>(&callback_), false);
    execute_callback_delegate<>(*static_cast<rcl_timer_call_info_t *>(data.get()));
    TRACETOOLS_TRACEPOINT(callback_end, reinterpret_cast<const void *>(&callback_));
  }

  // void specialization
  template<
    typename CallbackT = FunctorT,
    typename std::enable_if<
      rclcpp::function_traits::same_arguments<CallbackT, VoidCallbackType>::value
    >::type * = nullptr
  >
  void
  execute_callback_delegate(const rcl_timer_call_info_t &)
  {
    callback_();
  }

  template<
    typename CallbackT = FunctorT,
    typename std::enable_if<
      rclcpp::function_traits::same_arguments<CallbackT, TimerCallbackType>::value
    >::type * = nullptr
  >
  void
  execute_callback_delegate(const rcl_timer_call_info_t &)
  {
    callback_(*this);
  }


  template<
    typename CallbackT = FunctorT,
    typename std::enable_if<
      rclcpp::function_traits::same_arguments<CallbackT, TimerInfoCallbackType>::value
    >::type * = nullptr
  >
  void
  execute_callback_delegate(const rcl_timer_call_info_t & timer_call_info)
  {
    const TimerInfo info{Time{timer_call_info.expected_call_time, clock_->get_clock_type()},
      Time{timer_call_info.actual_call_time, clock_->get_clock_type()}};
    callback_(info);
  }

  /// Is the clock steady (i.e. is the time between ticks constant?)
  /** \return True if the clock used by this timer is steady. */
  bool
  is_steady() override
  {
    return clock_->get_clock_type() == RCL_STEADY_TIME;
  }

protected:
  RCLCPP_DISABLE_COPY(GenericTimer)

  /// Constructor for subclasses that initialize `timer_handle_` themselves.
  explicit GenericTimer(Clock::SharedPtr clock, FunctorT && callback)
  : TimerBase(clock), callback_(std::forward<FunctorT>(callback))
  {}

  FunctorT callback_;
};

template<
  typename FunctorT,
  typename std::enable_if<
    rclcpp::function_traits::same_arguments<FunctorT, VoidCallbackType>::value ||
    rclcpp::function_traits::same_arguments<FunctorT, TimerCallbackType>::value ||
    rclcpp::function_traits::same_arguments<FunctorT, TimerInfoCallbackType>::value
  >::type * = nullptr
>
class WallTimer : public GenericTimer<FunctorT>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(WallTimer)

  /// Wall timer constructor
  /**
   * \param period The interval at which the timer fires
   * \param callback The callback function to execute every interval
   * \param context node context
   * \param autostart timer state on initialization
   */
  WallTimer(
    std::chrono::nanoseconds period,
    FunctorT && callback,
    rclcpp::Context::SharedPtr context,
    bool autostart = true)
  : GenericTimer<FunctorT>(
      std::make_shared<Clock>(RCL_STEADY_TIME), period, std::move(callback), context, autostart)
  {}

protected:
  RCLCPP_DISABLE_COPY(WallTimer)
};

/// Timer with an explicit start time or phase alignment that is preserved across reset() calls.
/**
 * Unlike GenericTimer, whose first trigger is at `now() + period` and whose `reset()` method
 * re-anchors the schedule to `now() + period`, AlignedTimer is initialized with an explicit
 * initial trigger time (or a period and phase offset) via `rcl_timer_init_with_start_time()`
 * and `reset()` preserves its schedule phase.
 */
template<
  typename FunctorT,
  typename std::enable_if<
    rclcpp::function_traits::same_arguments<FunctorT, VoidCallbackType>::value ||
    rclcpp::function_traits::same_arguments<FunctorT, TimerCallbackType>::value ||
    rclcpp::function_traits::same_arguments<FunctorT, TimerInfoCallbackType>::value
  >::type * = nullptr
>
class AlignedTimer : public GenericTimer<FunctorT>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(AlignedTimer)

  /// Constructor with an explicit initial trigger time.
  /**
   * \param[in] clock The clock providing the current time.
   * \param[in] initial_call_time The time at which the callback should be initially triggered.
   * \param[in] period The interval at which the timer fires.
   * \param[in] callback User-specified callback function.
   * \param[in] context custom context to be used.
   * \param[in] autostart timer state on initialization
   *
   * \throws std::runtime_error if initial_call_time's clock type does not match clock's
   */
  explicit AlignedTimer(
    Clock::SharedPtr clock, Time initial_call_time, std::chrono::nanoseconds period,
    FunctorT && callback, rclcpp::Context::SharedPtr context, bool autostart = true
  )
  : GenericTimer<FunctorT>(clock, std::forward<FunctorT>(callback))
  {
    // An uninitialized clock is rejected below by rcl_timer_init_with_start_time itself; skip
    // the type comparison here so that pre-existing error path is still what surfaces in that case.
    if (this->clock_->get_clock_type() != RCL_CLOCK_UNINITIALIZED &&
      initial_call_time.get_clock_type() != this->clock_->get_clock_type())
    {
      throw std::runtime_error(
              "initial_call_time's clock type does not match clock's clock type");
    }

    if (nullptr == context) {
      context = rclcpp::contexts::get_global_default_context();
    }
    auto rcl_context = context->get_rcl_context();
    auto timer_handle = this->create_timer_handle(context);
    rcl_clock_t * clock_handle = this->clock_->get_clock_handle();
    {
      std::lock_guard<std::mutex> clock_guard(this->clock_->get_clock_mutex());
      rcl_ret_t ret = rcl_timer_init_with_start_time(
        timer_handle.get(), clock_handle, rcl_context.get(),
        initial_call_time.nanoseconds(), period.count(), nullptr,
        rcl_get_default_allocator(), autostart);
      if (ret != RCL_RET_OK) {
        rclcpp::exceptions::throw_from_rcl_error(ret, "Couldn't initialize rcl timer handle");
      }
    }
    this->timer_handle_ = std::move(timer_handle);

    TRACETOOLS_TRACEPOINT(
      rclcpp_timer_callback_added,
      static_cast<const void *>(this->get_timer_handle().get()),
      reinterpret_cast<const void *>(&this->callback_));
#ifndef TRACETOOLS_DISABLED
    if (TRACETOOLS_TRACEPOINT_ENABLED(rclcpp_callback_register)) {
      char * symbol = tracetools::get_symbol(this->callback_);
      TRACETOOLS_DO_TRACEPOINT(
        rclcpp_callback_register,
        reinterpret_cast<const void *>(&this->callback_),
        symbol);
      std::free(symbol);
    }
#endif
  }

  /// Convenience constructor that computes a phase-aligned initial trigger time from period and
  /// phase offset.
  /**
   * The initial trigger time is computed via `compute_phase_aligned_time(*clock, period, phase)`
   * as the smallest time `>= clock->now()` of the form `k * period + phase`.
   *
   * \param[in] clock The clock providing the current time.
   * \param[in] period The interval at which the timer fires (and alignment interval; must be > 0).
   * \param[in] phase Offset added to each interval boundary.
   * \param[in] callback User-specified callback function.
   * \param[in] context custom context to be used.
   * \param[in] autostart timer state on initialization
   *
   * \throws std::invalid_argument if period is not greater than zero
   */
  explicit AlignedTimer(
    Clock::SharedPtr clock, std::chrono::nanoseconds period, std::chrono::nanoseconds phase,
    FunctorT && callback, rclcpp::Context::SharedPtr context, bool autostart = true
  )
  : AlignedTimer(
      clock,
      compute_initial_call_time(clock, period, phase),
      period,
      std::forward<FunctorT>(callback),
      context,
      autostart)
  {}

protected:
  RCLCPP_DISABLE_COPY(AlignedTimer)

private:
  static Time
  compute_initial_call_time(
    const Clock::SharedPtr & clock,
    std::chrono::nanoseconds period,
    std::chrono::nanoseconds phase)
  {
    if (nullptr == clock) {
      throw std::invalid_argument("clock cannot be null");
    }
    if (clock->get_clock_type() == RCL_CLOCK_UNINITIALIZED) {
      return Time(static_cast<int64_t>(0), RCL_CLOCK_UNINITIALIZED);
    }
    return compute_phase_aligned_time(*clock, period, phase);
  }
};

/// Wall-clock timer with an explicit start time or phase alignment preserved across reset() calls.
template<
  typename FunctorT,
  typename std::enable_if<
    rclcpp::function_traits::same_arguments<FunctorT, VoidCallbackType>::value ||
    rclcpp::function_traits::same_arguments<FunctorT, TimerCallbackType>::value ||
    rclcpp::function_traits::same_arguments<FunctorT, TimerInfoCallbackType>::value
  >::type * = nullptr
>
class AlignedWallTimer : public AlignedTimer<FunctorT>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(AlignedWallTimer)

  /// AlignedWallTimer constructor with an explicit initial trigger time.
  /**
   * \param[in] initial_call_time The time at which the callback should be initially triggered.
   * \param[in] period The interval at which the timer fires.
   * \param[in] callback The callback function to execute every interval.
   * \param[in] context node context.
   * \param[in] autostart timer state on initialization.
   *
   * \throws std::runtime_error if initial_call_time's clock type is not RCL_STEADY_TIME
   */
  AlignedWallTimer(
    Time initial_call_time,
    std::chrono::nanoseconds period,
    FunctorT && callback,
    rclcpp::Context::SharedPtr context,
    bool autostart = true)
  : AlignedTimer<FunctorT>(
      std::make_shared<Clock>(RCL_STEADY_TIME), initial_call_time, period,
      std::move(callback), context, autostart)
  {}

  /// AlignedWallTimer convenience constructor with period and phase offset.
  /**
   * \param[in] period The interval at which the timer fires (and alignment interval; must be > 0).
   * \param[in] phase Offset added to each interval boundary.
   * \param[in] callback The callback function to execute every interval.
   * \param[in] context node context.
   * \param[in] autostart timer state on initialization.
   *
   * \throws std::invalid_argument if period is not greater than zero
   */
  AlignedWallTimer(
    std::chrono::nanoseconds period,
    std::chrono::nanoseconds phase,
    FunctorT && callback,
    rclcpp::Context::SharedPtr context,
    bool autostart = true)
  : AlignedTimer<FunctorT>(
      std::make_shared<Clock>(RCL_STEADY_TIME), period, phase,
      std::move(callback), context, autostart)
  {}

protected:
  RCLCPP_DISABLE_COPY(AlignedWallTimer)
};

}  // namespace rclcpp

#endif  // RCLCPP__TIMER_HPP_
