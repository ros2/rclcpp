// Copyright 2026 TriOrb, Inc.
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
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rclcpp_action/create_client.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/graph.hpp"

#include "test_msgs/action/fibonacci.hpp"

using namespace std::chrono_literals;

using Fibonacci = test_msgs::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

namespace
{

constexpr char kActionName[] = "fibonacci";
constexpr char kFqActionName[] = "/ns/fibonacci";

bool wait_for_event(
  std::shared_ptr<rclcpp::Node> node,
  std::function<bool()> predicate,
  std::chrono::milliseconds timeout = 3s,
  std::chrono::milliseconds sleep_period = 100ms)
{
  auto start = std::chrono::steady_clock::now();
  std::chrono::milliseconds time_slept(0);

  bool predicate_result;
  while (!(predicate_result = predicate()) &&
    time_slept < std::chrono::duration_cast<std::chrono::milliseconds>(timeout))
  {
    rclcpp::Event::SharedPtr graph_event = node->get_graph_event();
    node->wait_for_graph_change(graph_event, sleep_period);
    time_slept = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start);
  }
  return predicate_result;
}

}  // namespace

class TestActionGraph : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp()
  {
    node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  }

  void TearDown()
  {
    node.reset();
  }

  std::shared_ptr<rclcpp::Node> node;
};

TEST_F(TestActionGraph, get_action_clients_servers_info_by_action)
{
  auto node_base = node->get_node_base_interface();
  // Lists should be empty
  EXPECT_TRUE(
    rclcpp_action::get_action_clients_info_by_action(node_base, kFqActionName).empty());
  EXPECT_TRUE(
    rclcpp_action::get_action_servers_info_by_action(node_base, kFqActionName).empty());

  // Add an action client
  auto action_client = rclcpp_action::create_client<Fibonacci>(node, kActionName);
  // Wait for the action client to appear in the graph
  ASSERT_TRUE(
    wait_for_event(
      node,
      [&]() {
        return rclcpp_action::get_action_clients_info_by_action(
          node_base, kFqActionName).size() == 1u;
      }));
  // Server list should still be empty
  EXPECT_TRUE(
    rclcpp_action::get_action_servers_info_by_action(node_base, kFqActionName).empty());

  // Verify the client list has the right data
  auto client_list = rclcpp_action::get_action_clients_info_by_action(node_base, kFqActionName);
  ASSERT_EQ(1u, client_list.size());
  EXPECT_EQ(node->get_name(), client_list[0].node_name());
  EXPECT_EQ(node->get_namespace(), client_list[0].node_namespace());
  EXPECT_EQ("test_msgs/action/Fibonacci", client_list[0].action_type());
  EXPECT_EQ(rclcpp::EndpointType::Client, client_list[0].endpoint_type());
  // Verify the underlying entities of the action client
  const rclcpp::ServiceEndpointInfo & goal_info = client_list[0].goal_service_info();
  EXPECT_EQ("test_msgs/action/Fibonacci_SendGoal", goal_info.service_type());
  EXPECT_EQ(rclcpp::EndpointType::Client, goal_info.endpoint_type());
  EXPECT_TRUE(goal_info.endpoint_count() == 1u || goal_info.endpoint_count() == 2u);
  ASSERT_TRUE(client_list[0].cancel_service_info().has_value());
  EXPECT_EQ(
    "action_msgs/srv/CancelGoal", client_list[0].cancel_service_info()->service_type());
  ASSERT_TRUE(client_list[0].result_service_info().has_value());
  EXPECT_EQ(
    "test_msgs/action/Fibonacci_GetResult", client_list[0].result_service_info()->service_type());
  ASSERT_TRUE(client_list[0].feedback_topic_info().has_value());
  EXPECT_EQ(
    "test_msgs/action/Fibonacci_FeedbackMessage",
    client_list[0].feedback_topic_info()->topic_type());
  EXPECT_EQ(
    rclcpp::EndpointType::Subscription, client_list[0].feedback_topic_info()->endpoint_type());
  ASSERT_TRUE(client_list[0].status_topic_info().has_value());
  EXPECT_EQ(
    "action_msgs/msg/GoalStatusArray", client_list[0].status_topic_info()->topic_type());
  EXPECT_EQ(
    rclcpp::EndpointType::Subscription, client_list[0].status_topic_info()->endpoint_type());

  // Add an action server
  auto handle_goal =
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };
  auto handle_cancel =
    [](const std::shared_ptr<GoalHandleFibonacci>) {
      return rclcpp_action::CancelResponse::REJECT;
    };
  auto handle_accepted = [](const std::shared_ptr<GoalHandleFibonacci>) {};
  auto action_server = rclcpp_action::create_server<Fibonacci>(
    node, kActionName, handle_goal, handle_cancel, handle_accepted);
  // Wait for the action server to appear in the graph
  ASSERT_TRUE(
    wait_for_event(
      node,
      [&]() {
        return rclcpp_action::get_action_servers_info_by_action(
          node_base, kFqActionName).size() == 1u;
      }));

  // Verify the server list has the right data
  auto server_list = rclcpp_action::get_action_servers_info_by_action(node_base, kFqActionName);
  ASSERT_EQ(1u, server_list.size());
  EXPECT_EQ(node->get_name(), server_list[0].node_name());
  EXPECT_EQ(node->get_namespace(), server_list[0].node_namespace());
  EXPECT_EQ("test_msgs/action/Fibonacci", server_list[0].action_type());
  EXPECT_EQ(rclcpp::EndpointType::Server, server_list[0].endpoint_type());
  // Verify the underlying entities of the action server
  const rclcpp::ServiceEndpointInfo & server_goal_info = server_list[0].goal_service_info();
  EXPECT_EQ("test_msgs/action/Fibonacci_SendGoal", server_goal_info.service_type());
  EXPECT_EQ(rclcpp::EndpointType::Server, server_goal_info.endpoint_type());
  EXPECT_TRUE(
    server_goal_info.endpoint_count() == 1u || server_goal_info.endpoint_count() == 2u);
  ASSERT_TRUE(server_list[0].cancel_service_info().has_value());
  EXPECT_EQ(
    "action_msgs/srv/CancelGoal", server_list[0].cancel_service_info()->service_type());
  ASSERT_TRUE(server_list[0].result_service_info().has_value());
  EXPECT_EQ(
    "test_msgs/action/Fibonacci_GetResult", server_list[0].result_service_info()->service_type());
  ASSERT_TRUE(server_list[0].feedback_topic_info().has_value());
  EXPECT_EQ(
    "test_msgs/action/Fibonacci_FeedbackMessage",
    server_list[0].feedback_topic_info()->topic_type());
  EXPECT_EQ(
    rclcpp::EndpointType::Publisher, server_list[0].feedback_topic_info()->endpoint_type());
  ASSERT_TRUE(server_list[0].status_topic_info().has_value());
  EXPECT_EQ(
    "action_msgs/msg/GoalStatusArray", server_list[0].status_topic_info()->topic_type());
  EXPECT_EQ(
    rclcpp::EndpointType::Publisher, server_list[0].status_topic_info()->endpoint_type());

  // Error case: invalid action name
  EXPECT_THROW(
    rclcpp_action::get_action_clients_info_by_action(node_base, "13"),
    rclcpp::exceptions::InvalidTopicNameError);
  EXPECT_THROW(
    rclcpp_action::get_action_servers_info_by_action(node_base, "13"),
    rclcpp::exceptions::InvalidTopicNameError);
}

TEST_F(TestActionGraph, count_action_clients_servers)
{
  auto node_base = node->get_node_base_interface();
  EXPECT_EQ(0u, rclcpp_action::count_action_clients(node_base, kActionName));
  EXPECT_EQ(0u, rclcpp_action::count_action_servers(node_base, kActionName));

  auto action_client = rclcpp_action::create_client<Fibonacci>(node, kActionName);
  ASSERT_TRUE(
    wait_for_event(
      node,
      [&]() {
        return rclcpp_action::count_action_clients(node_base, kActionName) == 1u;
      }));
  EXPECT_EQ(1u, rclcpp_action::count_action_clients(node_base, kFqActionName));
  EXPECT_EQ(0u, rclcpp_action::count_action_servers(node_base, kActionName));

  auto handle_goal =
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };
  auto handle_cancel =
    [](const std::shared_ptr<GoalHandleFibonacci>) {
      return rclcpp_action::CancelResponse::REJECT;
    };
  auto handle_accepted = [](const std::shared_ptr<GoalHandleFibonacci>) {};
  auto action_server = rclcpp_action::create_server<Fibonacci>(
    node, kActionName, handle_goal, handle_cancel, handle_accepted);
  ASSERT_TRUE(
    wait_for_event(
      node,
      [&]() {
        return rclcpp_action::count_action_servers(node_base, kActionName) == 1u;
      }));
  EXPECT_EQ(1u, rclcpp_action::count_action_clients(node_base, kFqActionName));
  EXPECT_EQ(1u, rclcpp_action::count_action_servers(node_base, kFqActionName));

  // Error case: invalid action name
  EXPECT_THROW(
    rclcpp_action::count_action_clients(node_base, "13"),
    rclcpp::exceptions::InvalidTopicNameError);
  EXPECT_THROW(
    rclcpp_action::count_action_servers(node_base, "13"),
    rclcpp::exceptions::InvalidTopicNameError);
}
