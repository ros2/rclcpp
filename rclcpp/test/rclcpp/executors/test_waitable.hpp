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

#ifndef RCLCPP__EXECUTORS__TEST_WAITABLE_HPP_
#define RCLCPP__EXECUTORS__TEST_WAITABLE_HPP_

#include <atomic>
#include <functional>
#include <memory>

#include "rclcpp/waitable.hpp"
#include "rclcpp/guard_condition.hpp"

#include "rcl/wait.h"

class TestWaitable : public rclcpp::Waitable
{
public:
  TestWaitable() = default;

  void
  add_to_wait_set(rcl_wait_set_t & wait_set) override;

  void trigger();

  bool
  is_ready(const rcl_wait_set_t & wait_set) override;

  std::shared_ptr<void>
  take_data() override;

  std::shared_ptr<void>
  take_data_by_entity_id(size_t id) override;

  void
  execute(const std::shared_ptr<void> &) override;

  void
  set_on_execute_callback(std::function<void()> on_execute_callback);

  void
  set_on_ready_callback(std::function<void(size_t, int)> callback) override;

  void
  clear_on_ready_callback() override;

  size_t
  get_number_of_ready_guard_conditions() override;

  size_t
  get_count() const;

  size_t
  get_is_ready_call_count() const;

private:
  std::atomic<size_t> trigger_count_ = 0;
  std::atomic<size_t> is_ready_count_ = 0;
  std::atomic<size_t> count_ = 0;
  rclcpp::GuardCondition gc_;
  std::function<void()> on_execute_callback_ = nullptr;
};

#endif  // RCLCPP__EXECUTORS__TEST_WAITABLE_HPP_
