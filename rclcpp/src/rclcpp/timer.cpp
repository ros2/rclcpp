// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/timer.hpp"

#include <chrono>

using rclcpp::timer::CallbackType;
using rclcpp::timer::TimerBase;

TimerBase::TimerBase(std::chrono::nanoseconds period, CallbackType callback)
: period_(period),
  callback_(callback),
  canceled_(false)
{}

TimerBase::~TimerBase()
{}

void
TimerBase::cancel()
{
  this->canceled_ = true;
}

void
TimerBase::execute_callback() const
{
  callback_();
}

const CallbackType &
TimerBase::get_callback() const
{
  return callback_;
}
