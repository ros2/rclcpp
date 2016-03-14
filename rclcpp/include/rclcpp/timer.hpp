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

#include "rclcpp/function_traits.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/visibility_control.hpp"

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
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(TimerBase);

  RCLCPP_PUBLIC
  explicit TimerBase();

  RCLCPP_PUBLIC
  virtual ~TimerBase();

  RCLCPP_PUBLIC
  void
  cancel();

  RCLCPP_PUBLIC
  void
  execute_callback();

  /// Check how long the timer has until its next scheduled callback.
  // \return A std::chrono::duration representing the relative time until the next callback.
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
  bool is_ready();

protected:
  rcl_timer_t timer_handle_;
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
  RCLCPP_SMART_PTR_DEFINITIONS(GenericTimer);

  /// Default constructor.
  /**
   * \param[in] period The interval at which the timer fires.
   * \param[in] callback User-specified callback function.
   */
  GenericTimer(std::chrono::nanoseconds period, FunctorT && callback)
  : TimerBase(period), callback_(std::forward<FunctorT>(callback))
  {
    initialize_rcl_handle(period);
  }

  /// Default destructor.
  virtual ~GenericTimer()
  {
    // Stop the timer from running.
    cancel();
    rcl_timer_fini(&timer_handle_);
  }

  // void specialization
  template<
    typename CallbackT = FunctorT,
    typename std::enable_if<
      rclcpp::function_traits::same_arguments<CallbackT, VoidCallbackType>::value
    >::type * = nullptr
  >
  void
  initialize_rcl_handle(std::chrono::nanoseconds & period)
  {
    auto rcl_callback = std::function<void(rcl_timer_t *, uint64_t)>([this](rcl_timer_t * timer, uint64_t last_call_time) {
        callback_();
      });
    rcl_timer_init(&timer_handle_, period.count(), rcl_callback.target<void(rcl_timer_t *, uint64_t)>(), rcl_get_default_allocator());
  }

  template<
    typename CallbackT = FunctorT,
    typename std::enable_if<
      rclcpp::function_traits::same_arguments<CallbackT, TimerCallbackType>::value
    >::type * = nullptr
  >
  void
  initialize_rcl_handle(std::chrono::nanoseconds & period)
  {
    auto rcl_callback = std::function<void(rcl_timer_t *, uint64_t)>([this](rcl_timer_t * timer, uint64_t last_call_time) {
        callback_(*this);
      });
    rcl_timer_init(&timer_handle_, period.count(), rcl_callback.target<void(rcl_timer_t *, uint64_t)>(), rcl_get_default_allocator());
  }

  virtual bool
  is_steady()
  {
    return Clock::is_steady;
  }

protected:
  RCLCPP_DISABLE_COPY(GenericTimer);

  FunctorT callback_;
};

template<typename CallbackType>
using WallTimer = GenericTimer<CallbackType, std::chrono::steady_clock>;

}  // namespace timer
}  // namespace rclcpp

#endif  // RCLCPP__TIMER_HPP_
