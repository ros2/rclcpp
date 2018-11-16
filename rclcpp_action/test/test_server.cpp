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

#include <rclcpp/exceptions.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <test_msgs/action/fibonacci.hpp>

#include <gtest/gtest.h>

#include <memory>

#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server.hpp"


class TestServer : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }
};

TEST_F(TestServer, construction_and_destruction)
{
  auto node = std::make_shared<rclcpp::Node>("my_node", "/rclcpp_action/test/server");

  using GoalHandle = rclcpp_action::ServerGoalHandle<test_msgs::action::Fibonacci>;
  auto as = rclcpp_action::create_server<test_msgs::action::Fibonacci>(node.get(), "fibonacci",
      [](rcl_action_goal_info_t &, test_msgs::action::Fibonacci::Goal *) {},
      [](std::shared_ptr<GoalHandle>) {});
  (void)as;
}
