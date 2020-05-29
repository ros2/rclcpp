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
      std::this_thread::sleep_for(20ms);
      return;
    });
  ret = executor.spin_until_future_complete(future, 10ms);
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
      std::this_thread::sleep_for(20ms);
      return;
    });
  ret = executor.spin_until_future_complete(future.share(), 10ms);
  EXPECT_EQ(rclcpp::FutureReturnCode::TIMEOUT, ret);
}
