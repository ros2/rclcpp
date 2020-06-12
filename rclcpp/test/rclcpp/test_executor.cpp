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
