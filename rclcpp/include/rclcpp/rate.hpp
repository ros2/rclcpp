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

#ifndef RCLCPP__RATE_HPP_
#define RCLCPP__RATE_HPP_

#include <chrono>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

class RateBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(RateBase)

  virtual ~RateBase() {}
  virtual bool sleep() = 0;
  virtual rcl_clock_type_t get_type() const = 0;
  virtual void reset() = 0;
};

template<class Clock = std::chrono::high_resolution_clock>
class [[deprecated("use rclcpp::Rate class instead of GenericRate")]] GenericRate : public RateBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(GenericRate)

  explicit GenericRate(double) {}
  explicit GenericRate(std::chrono::nanoseconds) {}

  virtual bool sleep() {return false;}
  virtual rcl_clock_type_t get_type() const {return RCL_CLOCK_UNINITIALIZED;}
  virtual void reset() {}
};

class Rate : public RateBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Rate)

  explicit Rate(
    const double rate,
    Clock::SharedPtr clock = std::make_shared<Clock>(RCL_SYSTEM_TIME))
  : Rate(
      Duration::from_seconds(1.0 / rate), clock)
  {}
  explicit Rate(
    const Duration & period,
    Clock::SharedPtr clock = std::make_shared<Clock>(RCL_SYSTEM_TIME))
  : clock_(clock), period_(period), last_interval_(clock_->now())
  {}

  virtual bool
  sleep()
  {
    // Time coming into sleep
    auto now = clock_->now();
    // Time of next interval
    auto next_interval = last_interval_ + period_;
    // Detect backwards time flow
    if (now < last_interval_) {
      // Best thing to do is to set the next_interval to now + period
      next_interval = now + period_;
    }
    // Update the interval
    last_interval_ += period_;
    // If the time_to_sleep is negative or zero, don't sleep
    if (next_interval <= now) {
      // If an entire cycle was missed then reset next interval.
      // This might happen if the loop took more than a cycle.
      // Or if time jumps forward.
      if (now > next_interval + period_) {
        last_interval_ = now + period_;
      }
      // Either way do not sleep and return false
      return false;
    }
    // Calculate the time to sleep
    auto time_to_sleep = next_interval - now;
    // Sleep (will get interrupted by ctrl-c, may not sleep full time)
    clock_->sleep_for(time_to_sleep);
    return true;
  }

  virtual rcl_clock_type_t
  get_type() const
  {
    return clock_->get_clock_type();
  }

  virtual void
  reset()
  {
    last_interval_ = clock_->now();
  }

  Duration period() const
  {
    return period_;
  }

private:
  RCLCPP_DISABLE_COPY(Rate)

  Clock::SharedPtr clock_;
  Duration period_;
  Time last_interval_;
};

class WallRate : public Rate
{
public:
  explicit WallRate(const double rate)
  : Rate(rate, std::make_shared<Clock>(RCL_STEADY_TIME))
  {}

  explicit WallRate(const Duration & period)
  : Rate(period, std::make_shared<Clock>(RCL_STEADY_TIME))
  {}
};

class ROSRate : public Rate
{
public:
  explicit ROSRate(const double rate)
  : Rate(rate, std::make_shared<Clock>(RCL_ROS_TIME))
  {}

  explicit ROSRate(const Duration & period)
  : Rate(period, std::make_shared<Clock>(RCL_ROS_TIME))
  {}
};

}  // namespace rclcpp

#endif  // RCLCPP__RATE_HPP_
