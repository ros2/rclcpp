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

#include <stdexcept>

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

std::shared_ptr<void>
Waitable::take_data_by_entity_id(size_t id)
{
  (void)id;
  throw std::runtime_error(
          "Custom waitables should override take_data_by_entity_id "
          "if they want to use it.");
}

bool
Waitable::exchange_in_use_by_wait_set_state(bool in_use_state)
{
  return in_use_by_wait_set_.exchange(in_use_state);
}

void
Waitable::set_on_ready_callback(std::function<void(size_t, int)> callback)
{
  (void)callback;

  throw std::runtime_error(
          "Custom waitables should override set_on_ready_callback "
          "if they want to use it.");
}

void
Waitable::clear_on_ready_callback()
{
  throw std::runtime_error(
          "Custom waitables should override clear_on_ready_callback if they "
          "want to use it and make sure to call it on the waitable destructor.");
}

void
Waitable::add_to_wait_set(rcl_wait_set_t * wait_set)
{
  this->add_to_wait_set(*wait_set);
}

void
Waitable::add_to_wait_set(rcl_wait_set_t & wait_set)
{
#if !defined(_WIN32)
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#else  // !defined(_WIN32)
# pragma warning(push)
# pragma warning(disable: 4996)
#endif
  this->add_to_wait_set(&wait_set);
// remove warning suppression
#if !defined(_WIN32)
# pragma GCC diagnostic pop
#else  // !defined(_WIN32)
# pragma warning(pop)
#endif
}

bool
Waitable::is_ready(rcl_wait_set_t * wait_set)
{
  const rcl_wait_set_t & const_wait_set_ref = *wait_set;
  return this->is_ready(const_wait_set_ref);
}

bool
Waitable::is_ready(const rcl_wait_set_t & wait_set)
{
#if !defined(_WIN32)
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#else  // !defined(_WIN32)
# pragma warning(push)
# pragma warning(disable: 4996)
#endif
  // note this const cast is only required to support a deprecated function
  return this->is_ready(&const_cast<rcl_wait_set_t &>(wait_set));
// remove warning suppression
#if !defined(_WIN32)
# pragma GCC diagnostic pop
#else  // !defined(_WIN32)
# pragma warning(pop)
#endif
}

void
Waitable::execute(std::shared_ptr<void> & data)
{
  const std::shared_ptr<void> & const_data = data;
  this->execute(const_data);
}

void
Waitable::execute(const std::shared_ptr<void> & data)
{
#if !defined(_WIN32)
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#else  // !defined(_WIN32)
# pragma warning(push)
# pragma warning(disable: 4996)
#endif
  // note this const cast is only required to support a deprecated function
  this->execute(const_cast<std::shared_ptr<void> &>(data));
// remove warning suppression
#if !defined(_WIN32)
# pragma GCC diagnostic pop
#else  // !defined(_WIN32)
# pragma warning(pop)
#endif
}
