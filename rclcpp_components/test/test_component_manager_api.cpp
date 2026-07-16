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
#include <memory>
#include <string>

#include "composition_interfaces/srv/load_node.hpp"
#include "composition_interfaces/srv/unload_node.hpp"
#include "composition_interfaces/srv/list_nodes.hpp"

#include "rclcpp_components/component_manager.hpp"
#include "rclcpp_components/component_manager_isolated.hpp"

using namespace std::chrono_literals;

class TestComponentManager : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }
};

// TODO(hidmic): split up tests once Node bring up/tear down races
//               are solved https://github.com/ros2/rclcpp/issues/863
void test_components_api(bool use_dedicated_executor)
{
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto node = rclcpp::Node::make_shared("test_component_manager");
  std::shared_ptr<rclcpp_components::ComponentManager> manager;
  if (use_dedicated_executor) {
    using ComponentManagerIsolated =
      rclcpp_components::ComponentManagerIsolated<rclcpp::executors::SingleThreadedExecutor>;
    manager = std::make_shared<ComponentManagerIsolated>(exec);
  } else {
    manager = std::make_shared<rclcpp_components::ComponentManager>(exec);
  }

  exec->add_node(manager);
  exec->add_node(node);

  auto composition_client = node->create_client<composition_interfaces::srv::LoadNode>(
    "/ComponentManager/_container/load_node");

  if (!composition_client->wait_for_service(20s)) {
    ASSERT_TRUE(false) << "service not available after waiting";
  }

  {
    auto request = std::make_shared<composition_interfaces::srv::LoadNode::Request>();
    request->package_name = "rclcpp_components";
    request->plugin_name = "test_rclcpp_components::TestComponentFoo";

    auto future = composition_client->async_send_request(request);
    auto ret = exec->spin_until_future_complete(future, 5s);  // Wait for the result.
    auto result = future.get();
    EXPECT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_EQ(result->success, true);
    EXPECT_EQ(result->error_message, "");
    EXPECT_EQ(result->full_node_name, "/test_component_foo");
    EXPECT_EQ(result->unique_id, 1u);
  }

  {
    auto request = std::make_shared<composition_interfaces::srv::LoadNode::Request>();
    request->package_name = "rclcpp_components";
    request->plugin_name = "test_rclcpp_components::TestComponentBar";

    auto future = composition_client->async_send_request(request);
    auto ret = exec->spin_until_future_complete(future, 5s);  // Wait for the result.
    auto result = future.get();
    EXPECT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_EQ(result->success, true);
    EXPECT_EQ(result->error_message, "");
    EXPECT_EQ(result->full_node_name, "/test_component_bar");
    EXPECT_EQ(result->unique_id, 2u);
  }

  // Test remapping the node name
  {
    auto request = std::make_shared<composition_interfaces::srv::LoadNode::Request>();
    request->package_name = "rclcpp_components";
    request->plugin_name = "test_rclcpp_components::TestComponentFoo";
    request->node_name = "test_component_baz";

    auto future = composition_client->async_send_request(request);
    auto ret = exec->spin_until_future_complete(future, 5s);  // Wait for the result.
    auto result = future.get();
    EXPECT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_EQ(result->success, true);
    EXPECT_EQ(result->error_message, "");
    EXPECT_EQ(result->full_node_name, "/test_component_baz");
    EXPECT_EQ(result->unique_id, 3u);
  }

  // Test remapping the node namespace
  {
    auto request = std::make_shared<composition_interfaces::srv::LoadNode::Request>();
    request->package_name = "rclcpp_components";
    request->plugin_name = "test_rclcpp_components::TestComponentFoo";
    request->node_namespace = "/ns";
    request->node_name = "test_component_bing";

    auto future = composition_client->async_send_request(request);
    auto ret = exec->spin_until_future_complete(future, 5s);  // Wait for the result.
    auto result = future.get();
    EXPECT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_EQ(result->success, true);
    EXPECT_EQ(result->error_message, "");
    EXPECT_EQ(result->full_node_name, "/ns/test_component_bing");
    EXPECT_EQ(result->unique_id, 4u);
  }

  {
    // Valid package, but invalid class name.
    auto request = std::make_shared<composition_interfaces::srv::LoadNode::Request>();
    request->package_name = "rclcpp_components";
    request->plugin_name = "test_rclcpp_components::TestComponent";

    auto future = composition_client->async_send_request(request);
    auto ret = exec->spin_until_future_complete(future, 5s);  // Wait for the result.
    EXPECT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
    auto result = future.get();
    EXPECT_EQ(result->success, false);
    EXPECT_EQ(result->error_message, "Failed to find class with the requested plugin name.");
    EXPECT_EQ(result->full_node_name, "");
    EXPECT_EQ(result->unique_id, 0u);
  }

  {
    // Invalid package, but valid class name.
    auto request = std::make_shared<composition_interfaces::srv::LoadNode::Request>();
    request->package_name = "rclcpp_components_foo";
    request->plugin_name = "test_rclcpp_components::TestComponentFoo";

    auto future = composition_client->async_send_request(request);
    auto ret = exec->spin_until_future_complete(future, 5s);  // Wait for the result.
    EXPECT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
    auto result = future.get();
    EXPECT_EQ(result->success, false);
    EXPECT_EQ(result->error_message, "Could not find requested resource in ament index");
    EXPECT_EQ(result->full_node_name, "");
    EXPECT_EQ(result->unique_id, 0u);
  }

  {
    // Remap rules
    auto request = std::make_shared<composition_interfaces::srv::LoadNode::Request>();
    request->package_name = "rclcpp_components";
    request->plugin_name = "test_rclcpp_components::TestComponentFoo";
    request->node_name = "test_component_remap";
    request->remap_rules.push_back("alice:=bob");

    auto future = composition_client->async_send_request(request);
    auto ret = exec->spin_until_future_complete(future, 5s);  // Wait for the result.
    auto result = future.get();
    EXPECT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_EQ(result->success, true);
    EXPECT_EQ(result->error_message, "");
    EXPECT_EQ(result->full_node_name, "/test_component_remap");
    EXPECT_EQ(result->unique_id, 5u);
  }

  {
    // use_intra_process_comms
    auto request = std::make_shared<composition_interfaces::srv::LoadNode::Request>();
    request->package_name = "rclcpp_components";
    request->plugin_name = "test_rclcpp_components::TestComponentFoo";
    request->node_name = "test_component_intra_process";
    rclcpp::Parameter use_intraprocess_comms("use_intra_process_comms",
      rclcpp::ParameterValue(true));
    request->extra_arguments.push_back(use_intraprocess_comms.to_parameter_msg());

    auto future = composition_client->async_send_request(request);
    auto ret = exec->spin_until_future_complete(future, 5s);  // Wait for the result.
    auto result = future.get();
    EXPECT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_EQ(result->success, true);
    EXPECT_EQ(result->error_message, "");
    std::cout << result->full_node_name << std::endl;
    EXPECT_EQ(result->full_node_name, "/test_component_intra_process");
    EXPECT_EQ(result->unique_id, 6u);
  }

  {
    // use_intra_process_comms is not a bool type parameter
    auto request = std::make_shared<composition_interfaces::srv::LoadNode::Request>();
    request->package_name = "rclcpp_components";
    request->plugin_name = "test_rclcpp_components::TestComponentFoo";
    request->node_name = "test_component_intra_process_str";

    rclcpp::Parameter use_intraprocess_comms("use_intra_process_comms",
      rclcpp::ParameterValue("hello"));
    request->extra_arguments.push_back(use_intraprocess_comms.to_parameter_msg());

    auto future = composition_client->async_send_request(request);
    auto ret = exec->spin_until_future_complete(future, 5s);  // Wait for the result.
    auto result = future.get();
    EXPECT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_EQ(result->success, false);
    EXPECT_EQ(
      result->error_message,
      "Extra component argument 'use_intra_process_comms' must be a boolean");
    EXPECT_EQ(result->full_node_name, "");
    EXPECT_EQ(result->unique_id, 0u);
  }

  {
    // forward_global_arguments
    auto request = std::make_shared<composition_interfaces::srv::LoadNode::Request>();
    request->package_name = "rclcpp_components";
    request->plugin_name = "test_rclcpp_components::TestComponentFoo";
    request->node_name = "test_component_global_arguments";
    rclcpp::Parameter forward_global_arguments("forward_global_arguments",
      rclcpp::ParameterValue(true));
    request->extra_arguments.push_back(forward_global_arguments.to_parameter_msg());

    auto future = composition_client->async_send_request(request);
    auto ret = exec->spin_until_future_complete(future, 5s);  // Wait for the result.
    auto result = future.get();
    EXPECT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_EQ(result->success, true);
    EXPECT_EQ(result->error_message, "");
    EXPECT_EQ(result->full_node_name, "/test_component_global_arguments");
    EXPECT_EQ(result->unique_id, 7u);
  }

  {
    // forward_global_arguments is not a bool type parameter
    auto request = std::make_shared<composition_interfaces::srv::LoadNode::Request>();
    request->package_name = "rclcpp_components";
    request->plugin_name = "test_rclcpp_components::TestComponentFoo";
    request->node_name = "test_component_global_arguments_str";

    rclcpp::Parameter forward_global_arguments("forward_global_arguments",
      rclcpp::ParameterValue("hello"));
    request->extra_arguments.push_back(forward_global_arguments.to_parameter_msg());

    auto future = composition_client->async_send_request(request);
    auto ret = exec->spin_until_future_complete(future, 5s);  // Wait for the result.
    auto result = future.get();
    EXPECT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_EQ(result->success, false);
    EXPECT_EQ(
      result->error_message,
      "Extra component argument 'forward_global_arguments' must be a boolean");
    EXPECT_EQ(result->full_node_name, "");
    EXPECT_EQ(result->unique_id, 0u);
  }

  auto node_names = node->get_node_names();

  auto find_in_nodes = [node_names](std::string name) {
      return std::find(node_names.begin(), node_names.end(), name) != node_names.end();
    };

  EXPECT_TRUE(find_in_nodes("/test_component_foo"));
  EXPECT_TRUE(find_in_nodes("/test_component_bar"));
  EXPECT_TRUE(find_in_nodes("/test_component_baz"));
  EXPECT_TRUE(find_in_nodes("/ns/test_component_bing"));

  {
    auto client = node->create_client<composition_interfaces::srv::ListNodes>(
      "/ComponentManager/_container/list_nodes");

    if (!client->wait_for_service(20s)) {
      ASSERT_TRUE(false) << "service not available after waiting";
    }

    {
      auto request = std::make_shared<composition_interfaces::srv::ListNodes::Request>();
      auto future = client->async_send_request(request);
      auto ret = exec->spin_until_future_complete(future, 5s);  // Wait for the result.
      EXPECT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
      auto result = future.get();
      auto result_node_names = result->full_node_names;
      auto result_unique_ids = result->unique_ids;

      EXPECT_EQ(result_node_names.size(), 7u);
      EXPECT_EQ(result_node_names[0], "/test_component_foo");
      EXPECT_EQ(result_node_names[1], "/test_component_bar");
      EXPECT_EQ(result_node_names[2], "/test_component_baz");
      EXPECT_EQ(result_node_names[3], "/ns/test_component_bing");
      EXPECT_EQ(result_node_names[4], "/test_component_remap");
      EXPECT_EQ(result_node_names[5], "/test_component_intra_process");
      EXPECT_EQ(result_node_names[6], "/test_component_global_arguments");
      EXPECT_EQ(result_unique_ids.size(), 7u);
      EXPECT_EQ(result_unique_ids[0], 1u);
      EXPECT_EQ(result_unique_ids[1], 2u);
      EXPECT_EQ(result_unique_ids[2], 3u);
      EXPECT_EQ(result_unique_ids[3], 4u);
      EXPECT_EQ(result_unique_ids[4], 5u);
      EXPECT_EQ(result_unique_ids[5], 6u);
      EXPECT_EQ(result_unique_ids[6], 7u);
    }
  }

  {
    auto client = node->create_client<composition_interfaces::srv::UnloadNode>(
      "/ComponentManager/_container/unload_node");

    if (!client->wait_for_service(20s)) {
      ASSERT_TRUE(false) << "service not available after waiting";
    }

    {
      auto request = std::make_shared<composition_interfaces::srv::UnloadNode::Request>();
      request->unique_id = 1;

      auto future = client->async_send_request(request);
      auto ret = exec->spin_until_future_complete(future, 5s);  // Wait for the result.
      auto result = future.get();
      EXPECT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
      EXPECT_EQ(result->success, true);
      EXPECT_EQ(result->error_message, "");
    }

    {
      auto request = std::make_shared<composition_interfaces::srv::UnloadNode::Request>();
      request->unique_id = 1;

      auto future = client->async_send_request(request);
      auto ret = exec->spin_until_future_complete(future, 5s);  // Wait for the result.
      auto result = future.get();
      EXPECT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
      EXPECT_EQ(result->success, false);
      EXPECT_EQ(result->error_message, "No node found with unique_id: 1");
    }
  }

  {
    auto client = node->create_client<composition_interfaces::srv::ListNodes>(
      "/ComponentManager/_container/list_nodes");

    if (!client->wait_for_service(20s)) {
      ASSERT_TRUE(false) << "service not available after waiting";
    }

    {
      auto request = std::make_shared<composition_interfaces::srv::ListNodes::Request>();
      auto future = client->async_send_request(request);
      auto ret = exec->spin_until_future_complete(future, 5s);  // Wait for the result.
      EXPECT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
      auto result = future.get();
      auto result_node_names = result->full_node_names;
      auto result_unique_ids = result->unique_ids;

      EXPECT_EQ(result_node_names.size(), 6u);
      EXPECT_EQ(result_node_names[0], "/test_component_bar");
      EXPECT_EQ(result_node_names[1], "/test_component_baz");
      EXPECT_EQ(result_node_names[2], "/ns/test_component_bing");
      EXPECT_EQ(result_node_names[3], "/test_component_remap");
      EXPECT_EQ(result_node_names[4], "/test_component_intra_process");
      EXPECT_EQ(result_node_names[5], "/test_component_global_arguments");
      EXPECT_EQ(result_unique_ids.size(), 6u);
      EXPECT_EQ(result_unique_ids[0], 2u);
      EXPECT_EQ(result_unique_ids[1], 3u);
      EXPECT_EQ(result_unique_ids[2], 4u);
      EXPECT_EQ(result_unique_ids[3], 5u);
      EXPECT_EQ(result_unique_ids[4], 6u);
      EXPECT_EQ(result_unique_ids[5], 7u);
    }
  }
}

TEST_F(TestComponentManager, components_api)
{
  {
    SCOPED_TRACE("ComponentManager");
    test_components_api(false);
  }
  {
    SCOPED_TRACE("ComponentManagerIsolated");
    test_components_api(true);
  }
}
