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

#include "rclcpp/macros.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"

namespace rclcpp
{
namespace timer
{

using CallbackType = std::function<void()>;

class TimerBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(TimerBase);

  RCLCPP_PUBLIC
  TimerBase(std::chrono::nanoseconds period, CallbackType callback);

  RCLCPP_PUBLIC
  virtual ~TimerBase();

  RCLCPP_PUBLIC
  void
  cancel();

  RCLCPP_PUBLIC
  void
  execute_callback() const;

  RCLCPP_PUBLIC
  const CallbackType &
  get_callback() const;

  /// Check how long the timer has until its next scheduled callback.
  // \return A std::chrono::duration representing the relative time until the next callback.
  virtual std::chrono::nanoseconds
  time_until_trigger() = 0;

  /// Is the clock steady (i.e. is the time between ticks constant?)
  // \return True if the clock used by this timer is steady.
  virtual bool is_steady() = 0;

  /// Check if the timer needs to trigger the callback.
  /**
   * This function expects its caller to immediately trigger the callback after this function,
   * since it maintains the last time the callback was triggered.
   * \return True if the timer needs to trigger.
   */
  virtual bool check_and_trigger() = 0;

protected:
  std::chrono::nanoseconds period_;
  CallbackType callback_;

  bool canceled_;
};

/// Generic timer templated on the clock type. Periodically executes a user-specified callback.
template<class Clock = std::chrono::high_resolution_clock>
class GenericTimer : public TimerBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(GenericTimer);

  /// Default constructor.
  /**
   * \param[in] period The interval at which the timer fires.
   * \param[in] callback User-specified callback function.
   */
  GenericTimer(std::chrono::nanoseconds period, CallbackType callback)
  : TimerBase(period, callback), loop_rate_(period)
  {
    /* Set last_triggered_time_ so that the timer fires at least one period after being created. */
    last_triggered_time_ = Clock::now();
  }

  /// Default destructor.
  virtual ~GenericTimer()
  {
    // Stop the timer from running.
    cancel();
  }

  bool
  check_and_trigger()
  {
    if (canceled_) {
      return false;
    }
    if (Clock::now() < last_triggered_time_) {
      return false;
    }
    if (std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - last_triggered_time_) >=
      loop_rate_.period())
    {
      last_triggered_time_ = Clock::now();
      return true;
    }
    return false;
  }

  std::chrono::nanoseconds
  time_until_trigger()
  {
    std::chrono::nanoseconds time_until_trigger;
    // Calculate the time between the next trigger and the current time
    if (last_triggered_time_ + loop_rate_.period() < Clock::now()) {
      // time is overdue, need to trigger immediately
      time_until_trigger = std::chrono::nanoseconds::zero();
    } else {
      time_until_trigger = std::chrono::duration_cast<std::chrono::nanoseconds>(
        last_triggered_time_ - Clock::now()) + loop_rate_.period();
    }
    return time_until_trigger;
  }

  virtual bool
  is_steady()
  {
    return Clock::is_steady;
  }

private:
  RCLCPP_DISABLE_COPY(GenericTimer);

  rclcpp::rate::GenericRate<Clock> loop_rate_;
  std::chrono::time_point<Clock> last_triggered_time_;
};

using WallTimer = GenericTimer<std::chrono::steady_clock>;

}  // namespace timer
}  // namespace rclcpp

#endif  // RCLCPP__TIMER_HPP_
