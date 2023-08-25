// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/rate.hpp"

namespace rclcpp
{

bool
Rate::sleep()
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
  return clock_->sleep_for(time_to_sleep);
}

bool
Rate::is_steady() const
{
  return clock_->get_clock_type() == RCL_STEADY_TIME;
}

rcl_clock_type_t
Rate::get_type() const
{
  return clock_->get_clock_type();
}

void
Rate::reset()
{
  last_interval_ = clock_->now();
}

Duration
Rate::period() const
{
  return period_;
}

}  // namespace rclcpp
