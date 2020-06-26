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
#include <memory>
#include <sstream>
#include <thread>
#include <type_traits>
#include <utility>

#include "rclcpp/clock.hpp"
#include "rclcpp/context.hpp"
#include "rclcpp/function_traits.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/visibility_control.hpp"
#include "tracetools/tracetools.h"
#include "tracetools/utils.hpp"

#include "rcl/error_handling.h"
#include "rcl/timer.h"

#include "rmw/error_handling.h"
#include "rmw/rmw.h"

namespace rclcpp
{

class TimerBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(TimerBase)

  /// TimerBase constructor
  /**
   * \param clock A clock to use for time and sleeping
   * \param period The interval at which the timer fires
   * \param context node context
   */
  RCLCPP_PUBLIC
  explicit TimerBase(
    Clock::SharedPtr clock,
    std::chrono::nanoseconds period,
    rclcpp::Context::SharedPtr context);

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

  /// Call the callback function when the timer signal is emitted.
  RCLCPP_PUBLIC
  virtual void
  execute_callback() = 0;

  RCLCPP_PUBLIC
  std::shared_ptr<const rcl_timer_t>
  get_timer_handle();

  /// Check how long the timer has until its next scheduled callback.
  /**
   * \return A std::chrono::duration representing the relative time until the next callback.
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

protected:
  Clock::SharedPtr clock_;
  std::shared_ptr<rcl_timer_t> timer_handle_;

  std::atomic<bool> in_use_by_wait_set_{false};
};


using VoidCallbackType = std::function<void ()>;
using TimerCallbackType = std::function<void (TimerBase &)>;

/// Generic timer. Periodically executes a user-specified callback.
template<
  typename FunctorT,
  typename std::enable_if<
    rclcpp::function_traits::same_arguments<FunctorT, VoidCallbackType>::value ||
    rclcpp::function_traits::same_arguments<FunctorT, TimerCallbackType>::value
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
   */
  explicit GenericTimer(
    Clock::SharedPtr clock, std::chrono::nanoseconds period, FunctorT && callback,
    rclcpp::Context::SharedPtr context
  )
  : TimerBase(clock, period, context), callback_(std::forward<FunctorT>(callback))
  {
    TRACEPOINT(
      rclcpp_timer_callback_added,
      (const void *)get_timer_handle().get(),
      (const void *)&callback_);
    TRACEPOINT(
      rclcpp_callback_register,
      (const void *)&callback_,
      get_symbol(callback_));
  }

  /// Default destructor.
  virtual ~GenericTimer()
  {
    // Stop the timer from running.
    cancel();
  }

  /**
   * \sa rclcpp::TimerBase::execute_callback
   * \throws std::runtime_error if it failed to notify timer that callback occurred
   */
  void
  execute_callback() override
  {
    rcl_ret_t ret = rcl_timer_call(timer_handle_.get());
    if (ret == RCL_RET_TIMER_CANCELED) {
      return;
    }
    if (ret != RCL_RET_OK) {
      throw std::runtime_error("Failed to notify timer that callback occurred");
    }
    TRACEPOINT(callback_start, (const void *)&callback_, false);
    execute_callback_delegate<>();
    TRACEPOINT(callback_end, (const void *)&callback_);
  }

  // void specialization
  template<
    typename CallbackT = FunctorT,
    typename std::enable_if<
      rclcpp::function_traits::same_arguments<CallbackT, VoidCallbackType>::value
    >::type * = nullptr
  >
  void
  execute_callback_delegate()
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
  execute_callback_delegate()
  {
    callback_(*this);
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

  FunctorT callback_;
};

template<
  typename FunctorT,
  typename std::enable_if<
    rclcpp::function_traits::same_arguments<FunctorT, VoidCallbackType>::value ||
    rclcpp::function_traits::same_arguments<FunctorT, TimerCallbackType>::value
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
   */
  WallTimer(
    std::chrono::nanoseconds period,
    FunctorT && callback,
    rclcpp::Context::SharedPtr context)
  : GenericTimer<FunctorT>(
      std::make_shared<Clock>(RCL_STEADY_TIME), period, std::move(callback), context)
  {}

protected:
  RCLCPP_DISABLE_COPY(WallTimer)
};

}  // namespace rclcpp

#endif  // RCLCPP__TIMER_HPP_
