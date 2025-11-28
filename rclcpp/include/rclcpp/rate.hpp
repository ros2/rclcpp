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

  RCLCPP_PUBLIC
  virtual rcl_clock_type_t get_type() const = 0;

  RCLCPP_PUBLIC
  virtual void reset() = 0;
};

using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::nanoseconds;

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
