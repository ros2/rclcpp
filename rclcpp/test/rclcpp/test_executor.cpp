// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <string>

#include "rcl/error_handling.h"
#include "rcl/time.h"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class TestExecutors : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp()
  {
    node = std::make_shared<rclcpp::Node>("my_node");
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
};

// Make sure that executors detach from nodes when destructing
TEST_F(TestExecutors, detachOnDestruction) {
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
  }
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    EXPECT_NO_THROW(executor.add_node(node));
  }
}

// Make sure that the executor can automatically remove expired nodes correctly
TEST_F(TestExecutors, addTemporaryNode) {
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(std::make_shared<rclcpp::Node>("temporary_node"));
  EXPECT_NO_THROW(executor.spin_some());
}

// Make sure that the spin_until_future_complete works correctly with std::future
TEST_F(TestExecutors, testSpinUntilFutureComplete) {
  rclcpp::executors::SingleThreadedExecutor executor;
  std::future<void> future;
  rclcpp::FutureReturnCode ret;

  // test success
  future = std::async(
    []() {
      return;
    });
  ret = executor.spin_until_future_complete(future, 1s);
  EXPECT_EQ(rclcpp::FutureReturnCode::SUCCESS, ret);

  // test timeout
  future = std::async(
    []() {
      std::this_thread::sleep_for(1s);
      return;
    });
  ret = executor.spin_until_future_complete(future, 100ms);
  EXPECT_EQ(rclcpp::FutureReturnCode::TIMEOUT, ret);
}

// Make sure that the spin_until_future_complete works correctly with std::shared_future
TEST_F(TestExecutors, testSpinUntilFutureCompleteSharedFuture) {
  rclcpp::executors::SingleThreadedExecutor executor;
  std::future<void> future;
  rclcpp::FutureReturnCode ret;

  // test success
  future = std::async(
    []() {
      return;
    });
  ret = executor.spin_until_future_complete(future.share(), 1s);
  EXPECT_EQ(rclcpp::FutureReturnCode::SUCCESS, ret);

  // test timeout
  future = std::async(
    []() {
      std::this_thread::sleep_for(1s);
      return;
    });
  ret = executor.spin_until_future_complete(future.share(), 100ms);
  EXPECT_EQ(rclcpp::FutureReturnCode::TIMEOUT, ret);
}

class TestWaitable : public rclcpp::Waitable
{
public:
  TestWaitable()
  {
    rcl_guard_condition_options_t guard_condition_options =
      rcl_guard_condition_get_default_options();

    gc_ = rcl_get_zero_initialized_guard_condition();
    rcl_ret_t ret = rcl_guard_condition_init(
      &gc_,
      rclcpp::contexts::get_global_default_context()->get_rcl_context().get(),
      guard_condition_options);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret);
    }
  }

  bool
  add_to_wait_set(rcl_wait_set_t * wait_set) override
  {
    rcl_ret_t ret = rcl_wait_set_add_guard_condition(wait_set, &gc_, NULL);
    if (RCL_RET_OK != ret) {
      return false;
    }
    ret = rcl_trigger_guard_condition(&gc_);
    return RCL_RET_OK == ret;
  }

  bool
  is_ready(rcl_wait_set_t * wait_set) override
  {
    (void)wait_set;
    return true;
  }

  void
  execute() override
  {
    count_++;
    std::this_thread::sleep_for(100ms);
  }

  void
  take_data() override
  {
      throw std::runtime_error("Test executor should not be called");
  }

  size_t
  get_number_of_ready_guard_conditions() override {return 1;}

  size_t
  get_count()
  {
    return count_;
  }

private:
  size_t count_ = 0;
  rcl_guard_condition_t gc_;
};

TEST_F(TestExecutors, testSpinAllvsSpinSome) {
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    auto waitable_interfaces = node->get_node_waitables_interface();
    auto my_waitable = std::make_shared<TestWaitable>();
    waitable_interfaces->add_waitable(my_waitable, nullptr);
    executor.add_node(node);
    executor.spin_all(1s);
    executor.remove_node(node);
    EXPECT_GT(my_waitable->get_count(), 1u);
    waitable_interfaces->remove_waitable(my_waitable, nullptr);
  }
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    auto waitable_interfaces = node->get_node_waitables_interface();
    auto my_waitable = std::make_shared<TestWaitable>();
    waitable_interfaces->add_waitable(my_waitable, nullptr);
    executor.add_node(node);
    executor.spin_some(1s);
    executor.remove_node(node);
    EXPECT_EQ(my_waitable->get_count(), 1u);
    waitable_interfaces->remove_waitable(my_waitable, nullptr);
  }
}
