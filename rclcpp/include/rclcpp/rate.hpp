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
#include <thread>

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

  RCLCPP_PUBLIC
  virtual ~RateBase() {}

  RCLCPP_PUBLIC
  virtual bool sleep() = 0;

  [[deprecated("use get_type() instead")]]
  RCLCPP_PUBLIC
  virtual bool is_steady() const = 0;

  RCLCPP_PUBLIC
  virtual rcl_clock_type_t get_type() const = 0;

  RCLCPP_PUBLIC
  virtual void reset() = 0;
};

using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::nanoseconds;

template<class Clock = std::chrono::high_resolution_clock>
class [[deprecated("use rclcpp::Rate class instead of GenericRate")]] GenericRate : public RateBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(GenericRate)

  explicit GenericRate(double rate)
  : period_(duration_cast<nanoseconds>(duration<double>(1.0 / rate))), last_interval_(Clock::now())
  {}
  explicit GenericRate(std::chrono::nanoseconds period)
  : period_(period), last_interval_(Clock::now())
  {}

  virtual bool
  sleep()
  {
    // Time coming into sleep
    auto now = Clock::now();
    // Time of next interval
    auto next_interval = last_interval_ + period_;
    // Detect backwards time flow
    if (now < last_interval_) {
      // Best thing to do is to set the next_interval to now + period
      next_interval = now + period_;
    }
    // Calculate the time to sleep
    auto time_to_sleep = next_interval - now;
    // Update the interval
    last_interval_ += period_;
    // If the time_to_sleep is negative or zero, don't sleep
    if (time_to_sleep <= std::chrono::seconds(0)) {
      // If an entire cycle was missed then reset next interval.
      // This might happen if the loop took more than a cycle.
      // Or if time jumps forward.
      if (now > next_interval + period_) {
        last_interval_ = now + period_;
      }
      // Either way do not sleep and return false
      return false;
    }
    // Sleep (will get interrupted by ctrl-c, may not sleep full time)
    rclcpp::sleep_for(time_to_sleep);
    return true;
  }

  [[deprecated("use get_type() instead")]]
  virtual bool
  is_steady() const
  {
    return Clock::is_steady;
  }

  virtual rcl_clock_type_t get_type() const
  {
    return Clock::is_steady ? RCL_STEADY_TIME : RCL_SYSTEM_TIME;
  }

  virtual void
  reset()
  {
    last_interval_ = Clock::now();
  }

  std::chrono::nanoseconds period() const
  {
    return period_;
  }

private:
  RCLCPP_DISABLE_COPY(GenericRate)

  std::chrono::nanoseconds period_;
  using ClockDurationNano = std::chrono::duration<typename Clock::rep, std::nano>;
  std::chrono::time_point<Clock, ClockDurationNano> last_interval_;
};

class Rate : public RateBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Rate)

  RCLCPP_PUBLIC
  explicit Rate(
    const double rate,
    Clock::SharedPtr clock = std::make_shared<Clock>(RCL_SYSTEM_TIME));

  RCLCPP_PUBLIC
  explicit Rate(
    const Duration & period,
    Clock::SharedPtr clock = std::make_shared<Clock>(RCL_SYSTEM_TIME));

  RCLCPP_PUBLIC
  virtual bool
  sleep();

  [[deprecated("use get_type() instead")]]
  RCLCPP_PUBLIC
  virtual bool
  is_steady() const;

  RCLCPP_PUBLIC
  virtual rcl_clock_type_t
  get_type() const;

  RCLCPP_PUBLIC
  virtual void
  reset();

  RCLCPP_PUBLIC
  std::chrono::nanoseconds
  period() const;

private:
  RCLCPP_DISABLE_COPY(Rate)

  Clock::SharedPtr clock_;
  Duration period_;
  Time last_interval_;
};

class WallRate : public Rate
{
public:
  RCLCPP_PUBLIC
  explicit WallRate(const double rate);

  RCLCPP_PUBLIC
  explicit WallRate(const Duration & period);
};

}  // namespace rclcpp

#endif  // RCLCPP__RATE_HPP_
