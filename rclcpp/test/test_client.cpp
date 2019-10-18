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

#include <string>
#include <memory>
#include <chrono>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rcl_interfaces/srv/list_parameters.hpp"

using namespace std::chrono_literals;

class TestClient : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp()
  {
    node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
};

class TestClientSub : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
  }

  void SetUp()
  {
    node = std::make_shared<rclcpp::Node>("my_node", "/ns");
    subnode = node->create_sub_node("sub_ns");
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::Node::SharedPtr subnode;
};

/*
   Testing client construction and destruction.
 */
TEST_F(TestClient, construction_and_destruction) {
  using rcl_interfaces::srv::ListParameters;
  {
    auto client = node->create_client<ListParameters>("service");
  }

  {
    ASSERT_THROW(
    {
      auto client = node->create_client<ListParameters>("invalid_service?");
    }, rclcpp::exceptions::InvalidServiceNameError);
  }
}

TEST_F(TestClient, construction_with_free_function) {
  {
    auto client = rclcpp::create_client<rcl_interfaces::srv::ListParameters>(
      node->get_node_base_interface(),
      node->get_node_graph_interface(),
      node->get_node_services_interface(),
      "service",
      rmw_qos_profile_services_default,
      nullptr);
  }
  {
    ASSERT_THROW(
    {
      auto client = rclcpp::create_client<rcl_interfaces::srv::ListParameters>(
        node->get_node_base_interface(),
        node->get_node_graph_interface(),
        node->get_node_services_interface(),
        "invalid_?service",
        rmw_qos_profile_services_default,
        nullptr);
    }, rclcpp::exceptions::InvalidServiceNameError);
  }
}

/*
   Testing client construction and destruction for subnodes.
 */
TEST_F(TestClientSub, construction_and_destruction) {
  using rcl_interfaces::srv::ListParameters;
  {
    auto client = subnode->create_client<ListParameters>("service");
    EXPECT_STREQ(client->get_service_name(), "/ns/sub_ns/service");
  }

  {
    ASSERT_THROW(
    {
      auto client = node->create_client<ListParameters>("invalid_service?");
    }, rclcpp::exceptions::InvalidServiceNameError);
  }
}

/*
   Basic nominal test of a client.
 */
TEST_F(TestClient, test_client_nominal) {
  // Initialize the client.
  const std::string topic_name = "test_client";
  const std::string request_str = "test message";

  using RequestType = rcl_interfaces::srv::GetParameters::Request;
  using ResponseType = rcl_interfaces::srv::GetParameters::Response;

  auto server_cb = [request_str](const std::shared_ptr<RequestType> request,
      std::shared_ptr<ResponseType> response) -> void
  {
    EXPECT_EQ(request->names.size(), 1U);
    EXPECT_EQ(request->names.front(), request_str);

    rcl_interfaces::msg::ParameterValue resp;
    resp.string_value = request_str;
    response->values.push_back(resp);
    return;
  };

  auto client = node->create_client<rcl_interfaces::srv::GetParameters>(topic_name);
  auto server = node->create_service<rcl_interfaces::srv::GetParameters>(topic_name, server_cb);
  auto request = std::make_shared<RequestType>();
  request->names.emplace_back(request_str);

  EXPECT_TRUE(client->wait_for_service(1s));

  auto result = client->async_send_request(request);

  EXPECT_EQ(rclcpp::spin_until_future_complete(node, result),
    rclcpp::executor::FutureReturnCode::SUCCESS);

  EXPECT_EQ(request_str, result.get()->values.front().string_value);
}
