// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/event.hpp"

namespace rclcpp
{

Event::Event()
: state_(false) {}

bool
Event::set()
{
  return state_.exchange(true);
}

bool
Event::check()
{
  return state_.load();
}

bool
Event::check_and_clear()
{
  return state_.exchange(false);
}

}  // namespace rclcpp
