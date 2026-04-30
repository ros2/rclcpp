// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/waitable.hpp"

using rclcpp::Waitable;

size_t
Waitable::get_number_of_ready_subscriptions()
{
  return 0u;
}

size_t
Waitable::get_number_of_ready_timers()
{
  return 0u;
}

size_t
Waitable::get_number_of_ready_clients()
{
  return 0u;
}

size_t
Waitable::get_number_of_ready_events()
{
  return 0u;
}

size_t
Waitable::get_number_of_ready_services()
{
  return 0u;
}

size_t
Waitable::get_number_of_ready_guard_conditions()
{
  return 0u;
}

bool
Waitable::exchange_in_use_by_wait_set_state(bool in_use_state)
{
  return in_use_by_wait_set_.exchange(in_use_state);
}
