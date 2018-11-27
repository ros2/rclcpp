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
  auto node = std::make_shared<rclcpp::Node>("construct_node", "/rclcpp_action/construct");

  using GoalHandle = rclcpp_action::ServerGoalHandle<test_msgs::action::Fibonacci>;
  auto as = rclcpp_action::create_server<test_msgs::action::Fibonacci>(node.get(), "fibonacci",
      [](rcl_action_goal_info_t &, std::shared_ptr<test_msgs::action::Fibonacci::Goal>) {
        return rclcpp_action::GoalResponse::REJECT;
      },
      [](std::shared_ptr<GoalHandle>) {},
      [](std::shared_ptr<GoalHandle>) {});
  (void)as;
}

TEST_F(TestServer, handle_goal_called)
{
  auto node = std::make_shared<rclcpp::Node>("handle_goal_node", "/rclcpp_action/handle_goal");
  rcl_action_goal_info_t received_info;

  auto handle_goal = [&received_info](
      rcl_action_goal_info_t & info, std::shared_ptr<test_msgs::action::Fibonacci::Goal>)
    {
      received_info = info;
      return rclcpp_action::GoalResponse::REJECT;
    };

  using GoalHandle = rclcpp_action::ServerGoalHandle<test_msgs::action::Fibonacci>;
  auto as = rclcpp_action::create_server<test_msgs::action::Fibonacci>(node.get(), "fibonacci",
    handle_goal,
    [](std::shared_ptr<GoalHandle>) {},
    [](std::shared_ptr<GoalHandle>) {});
  (void)as;

  // Create a client that calls the goal request service
  // Make sure the UUID received is the same as the one sent

  auto client = node->create_client<test_msgs::action::Fibonacci::GoalRequestService>(
    "fibonacci/_action/send_goal");

  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(20)));

  auto request = std::make_shared<test_msgs::action::Fibonacci::GoalRequestService::Request>();

  const std::array<uint8_t, 16> uuid = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
  request->uuid = uuid;

  auto future = client->async_send_request(request);
  ASSERT_EQ(
    rclcpp::executor::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, future));

  for (size_t i = 0; i < 16; ++i) {
    EXPECT_EQ(uuid[i], received_info.uuid[i]) << "at idx " << i;
  }
}

TEST_F(TestServer, handle_execute_called)
{
  auto node = std::make_shared<rclcpp::Node>("handle_exec_node", "/rclcpp_action/handle_execute");

  auto handle_goal = [](
      rcl_action_goal_info_t &, std::shared_ptr<test_msgs::action::Fibonacci::Goal>)
    {
      return rclcpp_action::GoalResponse::ACCEPT;
    };

  using GoalHandle = rclcpp_action::ServerGoalHandle<test_msgs::action::Fibonacci>;

  std::shared_ptr<GoalHandle> received_handle;
  auto handle_execute = [&received_handle](std::shared_ptr<GoalHandle> handle)
    {
      received_handle = handle;
    };

  auto as = rclcpp_action::create_server<test_msgs::action::Fibonacci>(node.get(), "fibonacci",
    handle_goal,
    [](std::shared_ptr<GoalHandle>) {},
    handle_execute);
  (void)as;

  // Create a client that calls the goal request service
  // Make sure the UUID received is the same as the one sent

  auto client = node->create_client<test_msgs::action::Fibonacci::GoalRequestService>(
    "fibonacci/_action/send_goal");

  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(20)));

  auto request = std::make_shared<test_msgs::action::Fibonacci::GoalRequestService::Request>();

  const std::array<uint8_t, 16> uuid = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
  request->uuid = uuid;

  auto future = client->async_send_request(request);
  ASSERT_EQ(
    rclcpp::executor::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, future));

  ASSERT_TRUE(received_handle);
  EXPECT_EQ(uuid, received_handle->uuid_);
  EXPECT_EQ(*request, *(received_handle->goal_));
}
