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
#include <vector>

#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server.hpp"

using Fibonacci = test_msgs::action::Fibonacci;
using GoalID = rclcpp_action::GoalID;

class TestServer : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  std::shared_ptr<Fibonacci::Impl::SendGoalService::Request>
  send_goal_request(rclcpp::Node::SharedPtr node, GoalID uuid)
  {
    auto client = node->create_client<Fibonacci::Impl::SendGoalService>(
      "fibonacci/_action/send_goal");
    if (!client->wait_for_service(std::chrono::seconds(20))) {
      throw std::runtime_error("send goal service didn't become available");
    }
    auto request = std::make_shared<Fibonacci::Impl::SendGoalService::Request>();
    request->goal_id = uuid;
    auto future = client->async_send_request(request);
    if (rclcpp::executor::FutureReturnCode::SUCCESS !=
      rclcpp::spin_until_future_complete(node, future))
    {
      throw std::runtime_error("send goal future didn't complete succesfully");
    }
    return request;
  }

  void
  send_cancel_request(rclcpp::Node::SharedPtr node, GoalID uuid)
  {
    auto cancel_client = node->create_client<Fibonacci::Impl::CancelGoalService>(
      "fibonacci/_action/cancel_goal");
    if (!cancel_client->wait_for_service(std::chrono::seconds(20))) {
      throw std::runtime_error("cancel goal service didn't become available");
    }
    auto request = std::make_shared<Fibonacci::Impl::CancelGoalService::Request>();
    request->goal_info.goal_id.uuid = uuid;
    auto future = cancel_client->async_send_request(request);
    if (rclcpp::executor::FutureReturnCode::SUCCESS !=
      rclcpp::spin_until_future_complete(node, future))
    {
      throw std::runtime_error("cancel goal future didn't complete succesfully");
    }
  }
};

TEST_F(TestServer, construction_and_destruction)
{
  auto node = std::make_shared<rclcpp::Node>("construct_node", "/rclcpp_action/construct");

  using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;
  auto as = rclcpp_action::create_server<Fibonacci>(node, "fibonacci",
      [](const GoalID &, std::shared_ptr<const Fibonacci::Goal>) {
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
  GoalID received_uuid;

  auto handle_goal = [&received_uuid](
    const GoalID & uuid, std::shared_ptr<const Fibonacci::Goal>)
    {
      received_uuid = uuid;
      return rclcpp_action::GoalResponse::REJECT;
    };

  using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;
  auto as = rclcpp_action::create_server<Fibonacci>(node, "fibonacci",
      handle_goal,
      [](std::shared_ptr<GoalHandle>) {
        return rclcpp_action::CancelResponse::REJECT;
      },
      [](std::shared_ptr<GoalHandle>) {});
  (void)as;

  // Create a client that calls the goal request service
  // Make sure the UUID received is the same as the one sent

  auto client = node->create_client<Fibonacci::Impl::SendGoalService>(
    "fibonacci/_action/send_goal");

  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(20)));

  auto request = std::make_shared<Fibonacci::Impl::SendGoalService::Request>();

  const GoalID uuid{{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16}};
  request->goal_id = uuid;

  auto future = client->async_send_request(request);
  ASSERT_EQ(
    rclcpp::executor::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, future));

  ASSERT_EQ(uuid, received_uuid);
}

TEST_F(TestServer, handle_accepted_called)
{
  auto node = std::make_shared<rclcpp::Node>("handle_exec_node", "/rclcpp_action/handle_accepted");
  const GoalID uuid{{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16}};

  auto handle_goal = [](
    const GoalID &, std::shared_ptr<const Fibonacci::Goal>)
    {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

  using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;

  std::shared_ptr<GoalHandle> received_handle;
  auto handle_accepted = [&received_handle](std::shared_ptr<GoalHandle> handle)
    {
      received_handle = handle;
    };

  auto as = rclcpp_action::create_server<Fibonacci>(node, "fibonacci",
      handle_goal,
      [](std::shared_ptr<GoalHandle>) {
        return rclcpp_action::CancelResponse::REJECT;
      },
      handle_accepted);
  (void)as;

  auto request = send_goal_request(node, uuid);

  ASSERT_TRUE(received_handle);
  ASSERT_TRUE(received_handle->is_active());
  EXPECT_EQ(uuid, received_handle->get_goal_id());
  EXPECT_EQ(request->goal, *(received_handle->get_goal()));
}

TEST_F(TestServer, handle_cancel_called)
{
  auto node = std::make_shared<rclcpp::Node>("handle_cancel_node", "/rclcpp_action/handle_cancel");
  const GoalID uuid{{10, 20, 30, 40, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16}};

  auto handle_goal = [](
    const GoalID &, std::shared_ptr<const Fibonacci::Goal>)
    {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

  using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;

  auto handle_cancel = [](std::shared_ptr<GoalHandle>)
    {
      return rclcpp_action::CancelResponse::ACCEPT;
    };

  std::shared_ptr<GoalHandle> received_handle;
  auto handle_accepted = [&received_handle](std::shared_ptr<GoalHandle> handle)
    {
      received_handle = handle;
    };

  auto as = rclcpp_action::create_server<Fibonacci>(node, "fibonacci",
      handle_goal,
      handle_cancel,
      handle_accepted);
  (void)as;

  send_goal_request(node, uuid);

  ASSERT_TRUE(received_handle);
  EXPECT_EQ(uuid, received_handle->get_goal_id());
  EXPECT_FALSE(received_handle->is_canceling());

  send_cancel_request(node, uuid);
  EXPECT_TRUE(received_handle->is_canceling());
}

TEST_F(TestServer, publish_status_accepted)
{
  auto node = std::make_shared<rclcpp::Node>("status_accept_node", "/rclcpp_action/status_accept");
  const GoalID uuid{{1, 2, 3, 4, 5, 6, 7, 8, 9, 100, 110, 120, 13, 14, 15, 16}};

  auto handle_goal = [](
    const GoalID &, std::shared_ptr<const Fibonacci::Goal>)
    {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

  using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;

  auto handle_cancel = [](std::shared_ptr<GoalHandle>)
    {
      return rclcpp_action::CancelResponse::REJECT;
    };

  std::shared_ptr<GoalHandle> received_handle;
  auto handle_accepted = [&received_handle](std::shared_ptr<GoalHandle> handle)
    {
      received_handle = handle;
    };

  auto as = rclcpp_action::create_server<Fibonacci>(node, "fibonacci",
      handle_goal,
      handle_cancel,
      handle_accepted);
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
  for (size_t retry = 0; retry < max_tries && received_msgs.size() != 1u; ++retry) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(node);
  }

  ASSERT_LT(0u, received_msgs.size());
  // Not sure whether accepted will come through because not sure when subscriber will match
  for (auto & msg : received_msgs) {
    ASSERT_EQ(1u, msg->status_list.size());
    EXPECT_EQ(uuid, msg->status_list.at(0).goal_info.goal_id.uuid);
    auto status = msg->status_list.at(0).status;
    if (action_msgs::msg::GoalStatus::STATUS_ACCEPTED == status) {
      EXPECT_EQ(action_msgs::msg::GoalStatus::STATUS_ACCEPTED, status);
    } else {
      EXPECT_EQ(action_msgs::msg::GoalStatus::STATUS_EXECUTING, status);
    }
  }
}

TEST_F(TestServer, publish_status_canceling)
{
  auto node = std::make_shared<rclcpp::Node>("status_cancel_node", "/rclcpp_action/status_cancel");
  const GoalID uuid{{1, 2, 3, 40, 5, 6, 7, 80, 9, 10, 11, 120, 13, 14, 15, 160}};

  auto handle_goal = [](
    const GoalID &, std::shared_ptr<const Fibonacci::Goal>)
    {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

  using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;

  auto handle_cancel = [](std::shared_ptr<GoalHandle>)
    {
      return rclcpp_action::CancelResponse::ACCEPT;
    };

  std::shared_ptr<GoalHandle> received_handle;
  auto handle_accepted = [&received_handle](std::shared_ptr<GoalHandle> handle)
    {
      received_handle = handle;
    };

  auto as = rclcpp_action::create_server<Fibonacci>(node, "fibonacci",
      handle_goal,
      handle_cancel,
      handle_accepted);
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
  for (size_t retry = 0; retry < max_tries && received_msgs.size() < 2u; ++retry) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(node);
  }

  ASSERT_LT(0u, received_msgs.size());
  auto & msg = received_msgs.back();
  ASSERT_EQ(1u, msg->status_list.size());
  EXPECT_EQ(action_msgs::msg::GoalStatus::STATUS_CANCELING, msg->status_list.at(0).status);
  EXPECT_EQ(uuid, msg->status_list.at(0).goal_info.goal_id.uuid);
}

TEST_F(TestServer, publish_status_canceled)
{
  auto node = std::make_shared<rclcpp::Node>("status_canceled", "/rclcpp_action/status_canceled");
  const GoalID uuid{{1, 2, 3, 40, 5, 6, 70, 8, 9, 1, 11, 120, 13, 140, 15, 160}};

  auto handle_goal = [](
    const GoalID &, std::shared_ptr<const Fibonacci::Goal>)
    {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

  using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;

  auto handle_cancel = [](std::shared_ptr<GoalHandle>)
    {
      return rclcpp_action::CancelResponse::ACCEPT;
    };

  std::shared_ptr<GoalHandle> received_handle;
  auto handle_accepted = [&received_handle](std::shared_ptr<GoalHandle> handle)
    {
      received_handle = handle;
    };

  auto as = rclcpp_action::create_server<Fibonacci>(node, "fibonacci",
      handle_goal,
      handle_cancel,
      handle_accepted);
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

  received_handle->set_canceled(std::make_shared<Fibonacci::Impl::GetResultService::Response>());

  // 10 seconds
  const size_t max_tries = 10 * 1000 / 100;
  for (size_t retry = 0; retry < max_tries && received_msgs.size() < 3u; ++retry) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(node);
  }

  ASSERT_LT(0u, received_msgs.size());
  auto & msg = received_msgs.back();
  ASSERT_EQ(1u, msg->status_list.size());
  EXPECT_EQ(action_msgs::msg::GoalStatus::STATUS_CANCELED, msg->status_list.at(0).status);
  EXPECT_EQ(uuid, msg->status_list.at(0).goal_info.goal_id.uuid);
}

TEST_F(TestServer, publish_status_succeeded)
{
  auto node = std::make_shared<rclcpp::Node>("status_succeeded", "/rclcpp_action/status_succeeded");
  const GoalID uuid{{1, 2, 3, 40, 5, 6, 70, 8, 9, 1, 11, 120, 13, 140, 15, 160}};

  auto handle_goal = [](
    const GoalID &, std::shared_ptr<const Fibonacci::Goal>)
    {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

  using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;

  auto handle_cancel = [](std::shared_ptr<GoalHandle>)
    {
      return rclcpp_action::CancelResponse::REJECT;
    };

  std::shared_ptr<GoalHandle> received_handle;
  auto handle_accepted = [&received_handle](std::shared_ptr<GoalHandle> handle)
    {
      received_handle = handle;
    };

  auto as = rclcpp_action::create_server<Fibonacci>(node, "fibonacci",
      handle_goal,
      handle_cancel,
      handle_accepted);
  (void)as;

  // Subscribe to status messages
  std::vector<action_msgs::msg::GoalStatusArray::SharedPtr> received_msgs;
  auto subscriber = node->create_subscription<action_msgs::msg::GoalStatusArray>(
    "fibonacci/_action/status", [&received_msgs](action_msgs::msg::GoalStatusArray::SharedPtr list)
    {
      received_msgs.push_back(list);
    });

  send_goal_request(node, uuid);
  received_handle->set_succeeded(std::make_shared<Fibonacci::Impl::GetResultService::Response>());

  // 10 seconds
  const size_t max_tries = 10 * 1000 / 100;
  for (size_t retry = 0; retry < max_tries && received_msgs.size() < 2u; ++retry) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(node);
  }

  ASSERT_LT(0u, received_msgs.size());
  auto & msg = received_msgs.back();
  ASSERT_EQ(1u, msg->status_list.size());
  EXPECT_EQ(action_msgs::msg::GoalStatus::STATUS_SUCCEEDED, msg->status_list.at(0).status);
  EXPECT_EQ(uuid, msg->status_list.at(0).goal_info.goal_id.uuid);
}

TEST_F(TestServer, publish_status_aborted)
{
  auto node = std::make_shared<rclcpp::Node>("status_aborted", "/rclcpp_action/status_aborted");
  const GoalID uuid{{1, 2, 3, 40, 5, 6, 70, 8, 9, 1, 11, 120, 13, 140, 15, 160}};

  auto handle_goal = [](
    const GoalID &, std::shared_ptr<const Fibonacci::Goal>)
    {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

  using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;

  auto handle_cancel = [](std::shared_ptr<GoalHandle>)
    {
      return rclcpp_action::CancelResponse::REJECT;
    };

  std::shared_ptr<GoalHandle> received_handle;
  auto handle_accepted = [&received_handle](std::shared_ptr<GoalHandle> handle)
    {
      received_handle = handle;
    };

  auto as = rclcpp_action::create_server<Fibonacci>(node, "fibonacci",
      handle_goal,
      handle_cancel,
      handle_accepted);
  (void)as;

  // Subscribe to status messages
  std::vector<action_msgs::msg::GoalStatusArray::SharedPtr> received_msgs;
  auto subscriber = node->create_subscription<action_msgs::msg::GoalStatusArray>(
    "fibonacci/_action/status", [&received_msgs](action_msgs::msg::GoalStatusArray::SharedPtr list)
    {
      received_msgs.push_back(list);
    });

  send_goal_request(node, uuid);
  received_handle->set_aborted(std::make_shared<Fibonacci::Impl::GetResultService::Response>());

  // 10 seconds
  const size_t max_tries = 10 * 1000 / 100;
  for (size_t retry = 0; retry < max_tries && received_msgs.size() < 2u; ++retry) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(node);
  }

  ASSERT_LT(0u, received_msgs.size());
  auto & msg = received_msgs.back();
  ASSERT_EQ(1u, msg->status_list.size());
  EXPECT_EQ(action_msgs::msg::GoalStatus::STATUS_ABORTED, msg->status_list.at(0).status);
  EXPECT_EQ(uuid, msg->status_list.at(0).goal_info.goal_id.uuid);
}

TEST_F(TestServer, publish_feedback)
{
  auto node = std::make_shared<rclcpp::Node>("pub_feedback", "/rclcpp_action/pub_feedback");
  const GoalID uuid{{1, 20, 30, 4, 5, 6, 70, 8, 9, 1, 11, 120, 13, 14, 15, 160}};

  auto handle_goal = [](
    const GoalID &, std::shared_ptr<const Fibonacci::Goal>)
    {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

  using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;

  auto handle_cancel = [](std::shared_ptr<GoalHandle>)
    {
      return rclcpp_action::CancelResponse::REJECT;
    };

  std::shared_ptr<GoalHandle> received_handle;
  auto handle_accepted = [&received_handle](std::shared_ptr<GoalHandle> handle)
    {
      received_handle = handle;
    };

  auto as = rclcpp_action::create_server<Fibonacci>(node, "fibonacci",
      handle_goal,
      handle_cancel,
      handle_accepted);
  (void)as;

  // Subscribe to feedback messages
  using FeedbackT = Fibonacci::Impl::FeedbackMessage;
  std::vector<FeedbackT::SharedPtr> received_msgs;
  auto subscriber = node->create_subscription<FeedbackT>(
    "fibonacci/_action/feedback", [&received_msgs](FeedbackT::SharedPtr msg)
    {
      received_msgs.push_back(msg);
    });

  send_goal_request(node, uuid);

  auto sent_message = std::make_shared<FeedbackT>();
  sent_message->feedback.sequence = {1, 1, 2, 3, 5};
  received_handle->publish_feedback(sent_message);

  // 10 seconds
  const size_t max_tries = 10 * 1000 / 100;
  for (size_t retry = 0; retry < max_tries && received_msgs.size() < 1u; ++retry) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(node);
  }

  ASSERT_EQ(1u, received_msgs.size());
  auto & msg = received_msgs.back();
  ASSERT_EQ(sent_message->feedback.sequence, msg->feedback.sequence);
}

TEST_F(TestServer, get_result)
{
  auto node = std::make_shared<rclcpp::Node>("get_result", "/rclcpp_action/get_result");
  const GoalID uuid{{1, 2, 3, 4, 5, 6, 7, 80, 90, 10, 11, 12, 13, 14, 15, 160}};

  auto handle_goal = [](
    const GoalID &, std::shared_ptr<const Fibonacci::Goal>)
    {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

  using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;

  auto handle_cancel = [](std::shared_ptr<GoalHandle>)
    {
      return rclcpp_action::CancelResponse::REJECT;
    };

  std::shared_ptr<GoalHandle> received_handle;
  auto handle_accepted = [&received_handle](std::shared_ptr<GoalHandle> handle)
    {
      received_handle = handle;
    };

  auto as = rclcpp_action::create_server<Fibonacci>(node, "fibonacci",
      handle_goal,
      handle_cancel,
      handle_accepted);
  (void)as;

  send_goal_request(node, uuid);

  // Send result request
  auto result_client = node->create_client<Fibonacci::Impl::GetResultService>(
    "fibonacci/_action/get_result");
  if (!result_client->wait_for_service(std::chrono::seconds(20))) {
    throw std::runtime_error("get result service didn't become available");
  }
  auto request = std::make_shared<Fibonacci::Impl::GetResultService::Request>();
  request->goal_id = uuid;
  auto future = result_client->async_send_request(request);

  // Send a result
  auto result = std::make_shared<Fibonacci::Impl::GetResultService::Response>();
  result->result.sequence = {5, 8, 13, 21};
  received_handle->set_succeeded(result);

  // Wait for the result request to be received
  ASSERT_EQ(rclcpp::executor::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, future));

  auto response = future.get();
  EXPECT_EQ(action_msgs::msg::GoalStatus::STATUS_SUCCEEDED, response->status);
  EXPECT_EQ(result->result.sequence, response->result.sequence);
}

TEST_F(TestServer, deferred_execution)
{
  auto node = std::make_shared<rclcpp::Node>("defer_exec", "/rclcpp_action/defer_exec");
  const GoalID uuid{{1, 2, 3, 40, 5, 6, 70, 8, 9, 1, 11, 120, 13, 140, 15, 160}};

  auto handle_goal = [](
    const GoalID &, std::shared_ptr<const Fibonacci::Goal>)
    {
      return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
    };

  using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;

  auto handle_cancel = [](std::shared_ptr<GoalHandle>)
    {
      return rclcpp_action::CancelResponse::REJECT;
    };

  std::shared_ptr<GoalHandle> received_handle;
  auto handle_accepted = [&received_handle](std::shared_ptr<GoalHandle> handle)
    {
      received_handle = handle;
    };

  auto as = rclcpp_action::create_server<test_msgs::action::Fibonacci>(node, "fibonacci",
      handle_goal,
      handle_cancel,
      handle_accepted);
  (void)as;

  send_goal_request(node, uuid);

  EXPECT_TRUE(received_handle->is_active());
  EXPECT_FALSE(received_handle->is_executing());
  received_handle->set_executing();
  EXPECT_TRUE(received_handle->is_executing());
}
