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

#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <thread>
#include <type_traits>
#include <utility>

#include "rclcpp/function_traits.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/visibility_control.hpp"

#include "rcl/error_handling.h"
#include "rcl/timer.h"

#include "rmw/error_handling.h"
#include "rmw/rmw.h"

namespace rclcpp
{
namespace timer
{

class TimerBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(TimerBase)

  RCLCPP_PUBLIC
  explicit TimerBase(std::chrono::nanoseconds period);

  RCLCPP_PUBLIC
  ~TimerBase();

  RCLCPP_PUBLIC
  void
  cancel();

  RCLCPP_PUBLIC
  virtual void
  execute_callback() = 0;

  RCLCPP_PUBLIC
  const rcl_timer_t *
  get_timer_handle();

  /// Check how long the timer has until its next scheduled callback.
  // \return A std::chrono::duration representing the relative time until the next callback.
  RCLCPP_PUBLIC
  std::chrono::nanoseconds
  time_until_trigger();

  /// Is the clock steady (i.e. is the time between ticks constant?)
  // \return True if the clock used by this timer is steady.
  virtual bool is_steady() = 0;

  /// Check if the timer is ready to trigger the callback.
  /**
   * This function expects its caller to immediately trigger the callback after this function,
   * since it maintains the last time the callback was triggered.
   * \return True if the timer needs to trigger.
   */
  RCLCPP_PUBLIC
  bool is_ready();

protected:
  rcl_timer_t timer_handle_ = rcl_get_zero_initialized_timer();
};


using VoidCallbackType = std::function<void()>;
using TimerCallbackType = std::function<void(TimerBase &)>;

/// Generic timer templated on the clock type. Periodically executes a user-specified callback.
template<
  typename FunctorT,
  class Clock,
  typename std::enable_if<
    (rclcpp::function_traits::same_arguments<FunctorT, VoidCallbackType>::value ||
    rclcpp::function_traits::same_arguments<FunctorT, TimerCallbackType>::value) &&
    Clock::is_steady
  >::type * = nullptr
>
class GenericTimer : public TimerBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(GenericTimer)

  /// Default constructor.
  /**
   * \param[in] period The interval at which the timer fires.
   * \param[in] callback User-specified callback function.
   */
  GenericTimer(std::chrono::nanoseconds period, FunctorT && callback)
  : TimerBase(period), callback_(std::forward<FunctorT>(callback))
  {
  }

  /// Default destructor.
  virtual ~GenericTimer()
  {
    // Stop the timer from running.
    cancel();
    if (rcl_timer_fini(&timer_handle_) != RCL_RET_OK) {
      fprintf(stderr, "Failed to clean up rcl timer handle: %s\n", rcl_get_error_string_safe());
    }
  }

  void
  execute_callback()
  {
    rcl_ret_t ret = rcl_timer_call(&timer_handle_);
    if (ret == RCL_RET_TIMER_CANCELED) {
      return;
    }
    if (ret != RCL_RET_OK) {
      throw std::runtime_error("Failed to notify timer that callback occurred");
    }
    execute_callback_delegate<>();
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

  virtual bool
  is_steady()
  {
    return Clock::is_steady;
  }

protected:
  RCLCPP_DISABLE_COPY(GenericTimer)

  FunctorT callback_;
};

template<typename CallbackType>
using WallTimer = GenericTimer<CallbackType, std::chrono::steady_clock>;

}  // namespace timer
}  // namespace rclcpp

#endif  // RCLCPP__TIMER_HPP_
