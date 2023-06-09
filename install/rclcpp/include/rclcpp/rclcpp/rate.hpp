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
  virtual bool is_steady() const = 0;
  virtual void reset() = 0;
};

using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::nanoseconds;

template<class Clock = std::chrono::high_resolution_clock>
class GenericRate : public RateBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(GenericRate)

  explicit GenericRate(double rate)
  : GenericRate<Clock>(
      duration_cast<nanoseconds>(duration<double>(1.0 / rate)))
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

  virtual bool
  is_steady() const
  {
    return Clock::is_steady;
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

using Rate = GenericRate<std::chrono::system_clock>;
using WallRate = GenericRate<std::chrono::steady_clock>;

}  // namespace rclcpp

#endif  // RCLCPP__RATE_HPP_
