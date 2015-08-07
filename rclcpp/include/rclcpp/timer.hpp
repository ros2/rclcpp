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

#ifndef RCLCPP_RCLCPP_TIMER_HPP_
#define RCLCPP_RCLCPP_TIMER_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <thread>

#include <rmw/error_handling.h>
#include <rmw/rmw.h>

#include <rclcpp/macros.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>

namespace rclcpp
{

// Forward declaration is for friend statement in GenericTimer
namespace executor
{
class Executor;
} // namespace executor

namespace timer
{

typedef std::function<void ()> CallbackType;

class TimerBase
{
  friend class rclcpp::executor::Executor;

public:
  RCLCPP_MAKE_SHARED_DEFINITIONS(TimerBase);
  typedef std::weak_ptr<TimerBase> WeakPtr;

  TimerBase(std::chrono::nanoseconds period, CallbackType callback)
  : period_(period),
    callback_(callback),
    canceled_(false)
  {
  }

  virtual ~TimerBase()
  {
  }

  void
  cancel()
  {
    this->canceled_ = true;
  }

  virtual std::chrono::nanoseconds
  time_until_trigger() = 0;

  virtual bool is_steady() = 0;

  // Interface for externally triggering the timer event
  virtual bool check_and_trigger() = 0;

protected:
  std::chrono::nanoseconds period_;
  CallbackType callback_;

  bool canceled_;

};

template<class Clock = std::chrono::high_resolution_clock>
class GenericTimer : public TimerBase
{
  friend class rclcpp::executor::Executor;

public:
  RCLCPP_MAKE_SHARED_DEFINITIONS(GenericTimer);
  typedef std::weak_ptr<GenericTimer> WeakPtr;

  GenericTimer(std::chrono::nanoseconds period, CallbackType callback)
  : TimerBase(period, callback), loop_rate_(period)
  {
    /* Subtracting the loop rate period ensures that the callback gets triggered
       on the first call to check_and_trigger. */
    last_triggered_time_ = Clock::now() - period;
  }

  virtual ~GenericTimer()
  {
    cancel();
  }

  // return: true to trigger callback on the next "execute_timer" call in executor
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

typedef GenericTimer<std::chrono::steady_clock> WallTimer;

} /* namespace timer */
} /* namespace rclcpp */

#endif /* RCLCPP_RCLCPP_TIMER_HPP_ */
