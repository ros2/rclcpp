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

  std::shared_ptr<test_msgs::action::Fibonacci::GoalRequestService::Request>
  send_goal_request(rclcpp::Node::SharedPtr node, std::array<uint8_t, 16> uuid)
  {
    auto client = node->create_client<test_msgs::action::Fibonacci::GoalRequestService>(
      "fibonacci/_action/send_goal");
    if (!client->wait_for_service(std::chrono::seconds(20))) {
      throw std::runtime_error("send goal service didn't become available");
    }
    auto request = std::make_shared<test_msgs::action::Fibonacci::GoalRequestService::Request>();
    request->uuid = uuid;
    auto future = client->async_send_request(request);
    if (rclcpp::executor::FutureReturnCode::SUCCESS !=
        rclcpp::spin_until_future_complete(node, future)) {
      throw std::runtime_error("send goal future didn't complete succesfully");
    }
    return request;
  }

  void
  send_cancel_request(rclcpp::Node::SharedPtr node, std::array<uint8_t, 16> uuid)
  {
    auto cancel_client = node->create_client<test_msgs::action::Fibonacci::CancelGoalService>(
      "fibonacci/_action/cancel_goal");
    if (!cancel_client->wait_for_service(std::chrono::seconds(20))) {
      throw std::runtime_error("cancel goal service didn't become available");
    }
    auto request = std::make_shared<test_msgs::action::Fibonacci::CancelGoalService::Request>();
    request->goal_info.uuid = uuid;
    auto future = cancel_client->async_send_request(request);
    if (rclcpp::executor::FutureReturnCode::SUCCESS !=
        rclcpp::spin_until_future_complete(node, future)) {
      throw std::runtime_error("cancel goal future didn't complete succesfully");
    }
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
      [](std::shared_ptr<GoalHandle>) {
        return rclcpp_action::CancelResponse::REJECT;
      },
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
    [](std::shared_ptr<GoalHandle>) {
      return rclcpp_action::CancelResponse::REJECT;
    },
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
  const std::array<uint8_t, 16> uuid = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

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
    [](std::shared_ptr<GoalHandle>) {
      return rclcpp_action::CancelResponse::REJECT;
    },
    handle_execute);
  (void)as;

  auto request = send_goal_request(node, uuid);

  ASSERT_TRUE(received_handle);
  EXPECT_EQ(uuid, received_handle->uuid_);
  EXPECT_EQ(*request, *(received_handle->goal_));
}

TEST_F(TestServer, handle_cancel_called)
{
  auto node = std::make_shared<rclcpp::Node>("handle_cancel_node", "/rclcpp_action/handle_cancel");
  const std::array<uint8_t, 16> uuid = {10, 20, 30, 40, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

  auto handle_goal = [](
      rcl_action_goal_info_t &, std::shared_ptr<test_msgs::action::Fibonacci::Goal>)
    {
      return rclcpp_action::GoalResponse::ACCEPT;
    };

  using GoalHandle = rclcpp_action::ServerGoalHandle<test_msgs::action::Fibonacci>;

  auto handle_cancel = [](std::shared_ptr<GoalHandle>)
    {
      return rclcpp_action::CancelResponse::ACCEPT;
    };

  std::shared_ptr<GoalHandle> received_handle;
  auto handle_execute = [&received_handle](std::shared_ptr<GoalHandle> handle)
    {
      received_handle = handle;
    };

  auto as = rclcpp_action::create_server<test_msgs::action::Fibonacci>(node.get(), "fibonacci",
    handle_goal,
    handle_cancel,
    handle_execute);
  (void)as;

  send_goal_request(node, uuid);

  ASSERT_TRUE(received_handle);
  EXPECT_EQ(uuid, received_handle->uuid_);
  EXPECT_FALSE(received_handle->is_cancel_request());

  send_cancel_request(node, uuid);
  EXPECT_TRUE(received_handle->is_cancel_request());
}

TEST_F(TestServer, publish_status_accepted)
{
  auto node = std::make_shared<rclcpp::Node>("status_accept_node", "/rclcpp_action/status_accept");
  const std::array<uint8_t, 16> uuid = {1, 2, 3, 4, 5, 6, 7, 8, 9, 100, 110, 120, 13, 14, 15, 16};

  auto handle_goal = [](
      rcl_action_goal_info_t &, std::shared_ptr<test_msgs::action::Fibonacci::Goal>)
    {
      return rclcpp_action::GoalResponse::ACCEPT;
    };

  using GoalHandle = rclcpp_action::ServerGoalHandle<test_msgs::action::Fibonacci>;

  auto handle_cancel = [](std::shared_ptr<GoalHandle>)
    {
      return rclcpp_action::CancelResponse::REJECT;
    };

  std::shared_ptr<GoalHandle> received_handle;
  auto handle_execute = [&received_handle](std::shared_ptr<GoalHandle> handle)
    {
      received_handle = handle;
    };

  auto as = rclcpp_action::create_server<test_msgs::action::Fibonacci>(node.get(), "fibonacci",
    handle_goal,
    handle_cancel,
    handle_execute);
  (void)as;

  // Subscribe to status messages
  std::vector<action_msgs::msg::GoalStatusArray::SharedPtr> received_msgs;
  auto subscriber = node->create_subscription<action_msgs::msg::GoalStatusArray>(
    "fibonacci/_action/status", [&received_msgs](action_msgs::msg::GoalStatusArray::SharedPtr list)
      {
        received_msgs.push_back(list);
      });

  send_goal_request(node, uuid);

  // 10 seconds
  const size_t max_tries = 10 * 1000 / 100;
  for (size_t retry = 0; retry < max_tries && received_msgs.size() == 0u; ++retry) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(node);
  }

  ASSERT_EQ(1u, received_msgs.size());
  ASSERT_EQ(1u, received_msgs.at(0)->status_list.size());
  EXPECT_EQ(
    action_msgs::msg::GoalStatus::STATUS_ACCEPTED, received_msgs.at(0)->status_list.at(0).status);
  EXPECT_EQ(uuid, received_msgs.at(0)->status_list.at(0).goal_info.uuid);
}

TEST_F(TestServer, publish_status_canceling)
{
  auto node = std::make_shared<rclcpp::Node>("status_cancel_node", "/rclcpp_action/status_cancel");
  const std::array<uint8_t, 16> uuid = {1, 2, 3, 40, 5, 6, 7, 80, 9, 10, 11, 120, 13, 14, 15, 160};

  auto handle_goal = [](
      rcl_action_goal_info_t &, std::shared_ptr<test_msgs::action::Fibonacci::Goal>)
    {
      return rclcpp_action::GoalResponse::ACCEPT;
    };

  using GoalHandle = rclcpp_action::ServerGoalHandle<test_msgs::action::Fibonacci>;

  auto handle_cancel = [](std::shared_ptr<GoalHandle>)
    {
      return rclcpp_action::CancelResponse::ACCEPT;
    };

  std::shared_ptr<GoalHandle> received_handle;
  auto handle_execute = [&received_handle](std::shared_ptr<GoalHandle> handle)
    {
      received_handle = handle;
    };

  auto as = rclcpp_action::create_server<test_msgs::action::Fibonacci>(node.get(), "fibonacci",
    handle_goal,
    handle_cancel,
    handle_execute);
  (void)as;

  // Subscribe to status messages
  std::vector<action_msgs::msg::GoalStatusArray::SharedPtr> received_msgs;
  auto subscriber = node->create_subscription<action_msgs::msg::GoalStatusArray>(
    "fibonacci/_action/status", [&received_msgs](action_msgs::msg::GoalStatusArray::SharedPtr list)
      {
        received_msgs.push_back(list);
      });

  send_goal_request(node, uuid);
  send_cancel_request(node, uuid);

  // 10 seconds
  const size_t max_tries = 10 * 1000 / 100;
  for (size_t retry = 0; retry < max_tries && received_msgs.size() != 2u; ++retry) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(node);
  }

  ASSERT_EQ(2u, received_msgs.size());
  ASSERT_EQ(1u, received_msgs.at(0)->status_list.size());
  EXPECT_EQ(
    action_msgs::msg::GoalStatus::STATUS_CANCELING, received_msgs.at(1)->status_list.at(0).status);
  EXPECT_EQ(uuid, received_msgs.at(1)->status_list.at(0).goal_info.uuid);
}

// TODO publish_status set_aborted, set_suceeded, set_cancelled
// TODO request result service
