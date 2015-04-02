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
#include <thread>

#include <rmw/rmw.h>

#include <rclcpp/macros.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>

namespace rclcpp
{

// Forward declaration is for friend statement in GenericTimer
namespace executor {class Executor;}

namespace timer
{

typedef std::function<void()> CallbackType;

class TimerBase
{
  friend class rclcpp::executor::Executor;
public:
  RCLCPP_MAKE_SHARED_DEFINITIONS(TimerBase);

  TimerBase(std::chrono::nanoseconds period, CallbackType callback)
    : period_(period),
      callback_(callback),
      canceled_(false)
  {
    guard_condition_ = rmw_create_guard_condition();
  }

  void
  cancel()
  {
    this->canceled_ = true;
  }

  virtual bool is_steady() = 0;

protected:
  std::chrono::nanoseconds period_;
  CallbackType callback_;
  rmw_guard_condition_t * guard_condition_;

  bool canceled_;

};

template<class Clock = std::chrono::high_resolution_clock>
class GenericTimer : public TimerBase
{
  friend class rclcpp::executor::Executor;
public:
  RCLCPP_MAKE_SHARED_DEFINITIONS(GenericTimer);

  GenericTimer(std::chrono::nanoseconds period, CallbackType callback)
    : TimerBase(period, callback), loop_rate_(period)
  {
    thread_ = std::thread(&GenericTimer<Clock>::run, this);
  }

  ~GenericTimer()
  {
    cancel();
  }

  void
  run()
  {
    while (rclcpp::utilities::ok() && !this->canceled_)
    {
      loop_rate_.sleep();
      if (!rclcpp::utilities::ok())
      {
        return;
      }
      rmw_trigger_guard_condition(guard_condition_);
    }
  }

  virtual bool
  is_steady()
  {
    return Clock::is_steady;
  }

private:
  RCLCPP_DISABLE_COPY(GenericTimer);

  std::thread thread_;
  rclcpp::rate::GenericRate<Clock> loop_rate_;

};

typedef GenericTimer<std::chrono::steady_clock> WallTimer;

} /* namespace timer */
} /* namespace rclcpp */

#endif /* RCLCPP_RCLCPP_TIMER_HPP_ */
