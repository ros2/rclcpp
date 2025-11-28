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

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <stdexcept>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rcpputils/scope_exit.hpp"

#include "rclcpp/executors/executor_notify_waitable.hpp"

#include "../../utils/rclcpp_gtest_macros.hpp"


class TestExecutorNotifyWaitable : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }
};

TEST_F(TestExecutorNotifyWaitable, construct_destruct) {
  {
    auto waitable = std::make_shared<rclcpp::executors::ExecutorNotifyWaitable>();
    waitable.reset();
  }
  {
    auto on_execute_callback = []() {};
    auto waitable =
      std::make_shared<rclcpp::executors::ExecutorNotifyWaitable>(on_execute_callback);
    waitable.reset();
  }
}

TEST_F(TestExecutorNotifyWaitable, add_remove_guard_conditions) {
  auto on_execute_callback = []() {};
  auto waitable =
    std::make_shared<rclcpp::executors::ExecutorNotifyWaitable>(on_execute_callback);

  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  auto notify_guard_condition =

    node->get_node_base_interface()->get_shared_notify_guard_condition();
  EXPECT_NO_THROW(waitable->add_guard_condition(notify_guard_condition));
  EXPECT_NO_THROW(waitable->remove_guard_condition(notify_guard_condition));
}

TEST_F(TestExecutorNotifyWaitable, wait) {
  int on_execute_calls = 0;
  auto on_execute_callback = [&on_execute_calls]() {on_execute_calls++;};

  auto waitable =
    std::make_shared<rclcpp::executors::ExecutorNotifyWaitable>(on_execute_callback);

  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  auto notify_guard_condition = std::make_shared<rclcpp::GuardCondition>();
  EXPECT_NO_THROW(waitable->add_guard_condition(notify_guard_condition));

  auto default_cbg = node->get_node_base_interface()->get_default_callback_group();
  ASSERT_NE(nullptr, default_cbg->get_notify_guard_condition());

  auto waitables = node->get_node_waitables_interface();
  waitables->add_waitable(std::static_pointer_cast<rclcpp::Waitable>(waitable), default_cbg);

  // notify the guard condition, this should trigger the on_execute_callback
  notify_guard_condition->trigger();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin_all(std::chrono::seconds(1));
  EXPECT_EQ(1u, on_execute_calls);

  // no further trigger, therefore no further callback
  executor.spin_all(std::chrono::seconds(1));
  EXPECT_EQ(1u, on_execute_calls);
}
