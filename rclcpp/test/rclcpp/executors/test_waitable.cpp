// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/detail/add_guard_condition_to_rcl_wait_set.hpp"
#include "rclcpp/waitable.hpp"

#include "rcl/wait.h"

#include "test_waitable.hpp"

using namespace std::chrono_literals;

void
TestWaitable::add_to_wait_set(rcl_wait_set_t & wait_set)
{
  if (trigger_count_ > 0) {
    // Keep the gc triggered until the trigger count is reduced back to zero.
    // This is necessary if trigger() results in the wait set waking, but not
    // executing this waitable, in which case it needs to be re-triggered.
    gc_.trigger();
  }
  rclcpp::detail::add_guard_condition_to_rcl_wait_set(wait_set, gc_);
}

void TestWaitable::trigger()
{
  trigger_count_++;
  gc_.trigger();
}

bool
TestWaitable::is_ready(const rcl_wait_set_t & wait_set)
{
  is_ready_count_++;
  for (size_t i = 0; i < wait_set.size_of_guard_conditions; ++i) {
    auto rcl_guard_condition = wait_set.guard_conditions[i];
    if (&gc_.get_rcl_guard_condition() == rcl_guard_condition) {
      return true;
    }
  }
  return false;
}

std::shared_ptr<void>
TestWaitable::take_data()
{
  return nullptr;
}

std::shared_ptr<void>
TestWaitable::take_data_by_entity_id(size_t id)
{
  (void) id;
  return nullptr;
}

void
TestWaitable::execute(const std::shared_ptr<void> &)
{
  trigger_count_--;
  count_++;
  if (nullptr != on_execute_callback_) {
    on_execute_callback_();
  } else {
    // TODO(wjwwood): I don't know why this was here, but probably it should
    //   not be there, or test cases where that is important should use the
    //   on_execute_callback?
    std::this_thread::sleep_for(3ms);
  }
}

void
TestWaitable::set_on_execute_callback(std::function<void()> on_execute_callback)
{
  on_execute_callback_ = on_execute_callback;
}

void
TestWaitable::set_on_ready_callback(std::function<void(size_t, int)> callback)
{
  auto gc_callback = [callback](size_t count) {
      callback(count, 0);
    };
  gc_.set_on_trigger_callback(gc_callback);
}

void
TestWaitable::clear_on_ready_callback()
{
  gc_.set_on_trigger_callback(nullptr);
}

size_t
TestWaitable::get_number_of_ready_guard_conditions()
{
  return 1;
}

size_t
TestWaitable::get_count() const
{
  return count_;
}

size_t
TestWaitable::get_is_ready_call_count() const
{
  return is_ready_count_;
}
