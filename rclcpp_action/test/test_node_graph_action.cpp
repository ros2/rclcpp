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

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rcl/graph.h"
#include "rcl/node_options.h"
#include "rcl/remap.h"
#include "rcl_action/graph.h"
#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/node_base.hpp"
#include "rclcpp/node_interfaces/node_graph.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/create_client.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rcutils/strdup.h"
#include "test_msgs/msg/empty.h"
#include "test_msgs/msg/empty.hpp"
#include "test_msgs/srv/empty.hpp"
#include "test_msgs/action/fibonacci.hpp"

#include "mocking_utils/patch.hpp"

using ActionType = test_msgs::action::Fibonacci;
using GoalHandle = rclcpp_action::ServerGoalHandle<ActionType>;

rclcpp_action::GoalResponse handle_goal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ActionType::Goal> goal)
{
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse handle_cancel(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  (void) goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  (void)goal_handle;
}

namespace
{

constexpr char node_name[] = "node";
constexpr char node_namespace[] = "ns";
constexpr char absolute_namespace[] = "/ns";

}  // namespace

class TestNodeGraph : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>(node_name, node_namespace);

    // This dynamic cast is not necessary for the unittests, but instead is used to ensure
    // the proper type is being tested and covered.
    node_graph_ =
      dynamic_cast<rclcpp::node_interfaces::NodeGraph *>(node_->get_node_graph_interface().get());
    ASSERT_NE(nullptr, node_graph_);
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<rclcpp::Node> node() {return node_;}

  const rclcpp::node_interfaces::NodeGraph * node_graph() const {return node_graph_;}

  size_t get_num_graph_things(std::function<size_t()> predicate)
  {
    constexpr std::chrono::milliseconds timeout(100);

    size_t tries = 0;
    size_t num_things = 0;
    while (tries++ < 5) {
      num_things = predicate();
      if (num_things >= 1) {
        break;
      }

      auto event = node()->get_graph_event();
      EXPECT_NO_THROW(node()->wait_for_graph_change(event, timeout));
    }

    return num_things;
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::node_interfaces::NodeGraph * node_graph_;
};

TEST_F(TestNodeGraph, get_action_names_and_types)
{
  ASSERT_EQ(0u, node_graph()->get_action_names_and_types().size());
}

TEST_F(TestNodeGraph, get_action_names_and_types_rcl_error)
{
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp_action", rcl_action_get_names_and_types, RCL_RET_ERROR);
  EXPECT_THROW(
    node_graph()->get_action_names_and_types(),
    rclcpp::exceptions::RCLError);
}

TEST_F(TestNodeGraph, get_action_names_and_types_rcl_fini_error)
{
  auto mock_info_array_fini = mocking_utils::patch_and_return(
    "lib:rclcpp_action", rcl_names_and_types_fini, RCL_RET_ERROR);
  EXPECT_THROW(
    node_graph()->get_action_names_and_types(),
    rclcpp::exceptions::RCLError);
}

TEST_F(TestNodeGraph, get_action_client_names_and_types_by_node)
{
  auto client = rclcpp_action::create_client<ActionType>(node(), "node1_client");

  const std::string node2_name = "node2";
  auto node2 = std::make_shared<rclcpp::Node>(node2_name, node_namespace);

  // rcl_action_get_client_names_and_types_by_node() expects the node to exist, otherwise it fails
  EXPECT_THROW(
    node_graph()->get_action_client_names_and_types_by_node("not_a_node", "not_absolute_namespace"),
    std::runtime_error);

  auto clients_of_node1 =
    node_graph()->get_action_client_names_and_types_by_node(node_name, absolute_namespace);
  auto clients_of_node2 =
    node_graph()->get_action_client_names_and_types_by_node(node2_name, absolute_namespace);

  // Begin microbenchmarking of the action names
  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(3)) {
    clients_of_node1 =
      node_graph()->get_action_client_names_and_types_by_node(node_name, absolute_namespace);
    clients_of_node2 =
      node_graph()->get_action_client_names_and_types_by_node(node2_name, absolute_namespace);
    if (clients_of_node1.find("/ns/node1_client") != clients_of_node1.end()) {
      break;
    }
  }

  EXPECT_TRUE(clients_of_node1.find("/ns/node1_client") != clients_of_node1.end());
  EXPECT_FALSE(clients_of_node2.find("/ns/node1_client") != clients_of_node2.end());
}

TEST_F(TestNodeGraph, get_action_client_names_and_types_by_node_rcl_error)
{
  auto client = rclcpp_action::create_client<ActionType>(node(), "node1_client");

  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp_action", rcl_action_get_client_names_and_types_by_node, RCL_RET_ERROR);
  auto mock_names_fini = mocking_utils::patch_and_return(
    "lib:rclcpp_action", rcl_names_and_types_fini, RCL_RET_ERROR);

  EXPECT_THROW(
    node_graph()->get_action_client_names_and_types_by_node(node_name, node_namespace),
    decltype(std::runtime_error(
      "failed to get action client names and types by node: error not set")));
}

TEST_F(TestNodeGraph, get_action_client_names_and_types_by_node_rcl_fini_error)
{
  auto client = rclcpp_action::create_client<ActionType>(node(), "node1_client");

  auto mock_names_fini = mocking_utils::patch_and_return(
    "lib:rclcpp_action", rcl_names_and_types_fini, RCL_RET_ERROR);
  EXPECT_NO_THROW(
    node_graph()->get_action_client_names_and_types_by_node(node_name, absolute_namespace));
}

TEST_F(TestNodeGraph, get_action_server_names_and_types_by_node)
{
  auto server = rclcpp_action::create_server<ActionType>(
    node(),
    node_name,
    handle_goal,
    handle_cancel,
    handle_accepted
  );

  const std::string node2_name = "node2";
  auto node2 = std::make_shared<rclcpp::Node>(node2_name, node_namespace);

  // rcl_action_get_server_names_and_types_by_node() expects the node to exist, otherwise it fails
  EXPECT_THROW(
    node_graph()->get_action_server_names_and_types_by_node("not_a_node", "not_absolute_namespace"),
    rclcpp::exceptions::RCLError);

  // Check that node1_topic exists for node1 but not node2. This shouldn't exercise graph
  // discovery as node_graph belongs to node1 anyway. This is just to test the API itself.
  auto topics_of_node1 =
    node_graph()->get_action_server_names_and_types_by_node(node_name, absolute_namespace);
  auto topics_of_node2 =
    node_graph()->get_action_server_names_and_types_by_node(node2_name, absolute_namespace);

  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(3)) {
    topics_of_node1 =
      node_graph()->get_action_server_names_and_types_by_node(node_name, absolute_namespace);
    topics_of_node2 =
      node_graph()->get_action_server_names_and_types_by_node(node2_name, absolute_namespace);
    if (topics_of_node1.find("/ns/node1_topic") != topics_of_node1.end()) {
      break;
    }
  }

  EXPECT_TRUE(topics_of_node1.find("/ns/node1_topic") != topics_of_node1.end());
  EXPECT_FALSE(topics_of_node2.find("/ns/node1_topic") != topics_of_node2.end());
}

TEST_F(TestNodeGraph, get_action_server_names_and_types_by_node_rcl_error)
{
  auto server = rclcpp_action::create_server<ActionType>(
    node(),
    node_name,
    handle_goal,
    handle_cancel,
    handle_accepted
  );

  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp_action", rcl_action_get_server_names_and_types_by_node, RCL_RET_ERROR);
  auto mock_names_fini = mocking_utils::patch_and_return(
    "lib:rclcpp_action", rcl_names_and_types_fini, RCL_RET_ERROR);
  EXPECT_THROW(
    node_graph()->get_action_server_names_and_types_by_node(node_name, node_namespace),
    decltype(std::runtime_error(
      "failed to get action server names and types by node: error not set")));
}

TEST_F(TestNodeGraph, get_action_server_names_and_types_by_node_rcl_fini_error)
{
  auto server = rclcpp_action::create_server<ActionType>(
    node(),
    node_name,
    handle_goal,
    handle_cancel,
    handle_accepted
  );

  auto mock_names_fini = mocking_utils::patch_and_return(
    "lib:rclcpp_action", rcl_names_and_types_fini, RCL_RET_ERROR);

  EXPECT_NO_THROW(
    node_graph()->get_action_server_names_and_types_by_node(node_name, absolute_namespace));
}
