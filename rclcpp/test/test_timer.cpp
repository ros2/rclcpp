// Copyright 2019 Open Source Robotics Foundation, Inc.
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
#include <exception>
#include <memory>

#include "rcl/timer.h"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class TestTimer : public ::testing::Test
{
protected:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
    test_node = std::make_shared<rclcpp::Node>("test_timer_node");
  }

  void TearDown()
  {
    test_node.reset();
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr test_node;
};

TEST_F(TestTimer, test_is_cancelled)
{
  try {
    auto timer = test_node->create_wall_timer(1s,
        []() -> void {
          printf("this is a test\n");
        }
    );

    // start and cancel
    ASSERT_FALSE(timer->is_cancelled());
    timer->cancel();
    ASSERT_TRUE(timer->is_cancelled());

    // reset and cancel
    timer->reset();
    ASSERT_FALSE(timer->is_cancelled());
    timer->cancel();
    ASSERT_TRUE(timer->is_cancelled());
  } catch (std::exception & e) {
    FAIL() << e.what();
  }
}
