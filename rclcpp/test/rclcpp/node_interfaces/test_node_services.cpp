// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <string>
#include <vector>

#include "rcl/node_options.h"
#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/node_services.hpp"
#include "rclcpp/rclcpp.hpp"

#include "../../mocking_utils/patch.hpp"
#include "../../utils/rclcpp_gtest_macros.hpp"

class TestService : public rclcpp::ServiceBase
{
public:
  explicit TestService(rclcpp::Node * node)
  : rclcpp::ServiceBase(node->get_node_base_interface()->get_shared_rcl_node_handle()) {}

  std::shared_ptr<void> create_request() override {return nullptr;}
  std::shared_ptr<rmw_request_id_t> create_request_header() override {return nullptr;}
  void handle_request(std::shared_ptr<rmw_request_id_t>, std::shared_ptr<void>) override {}
};

class TestClient : public rclcpp::ClientBase
{
public:
  explicit TestClient(rclcpp::Node * node)
  : rclcpp::ClientBase(node->get_node_base_interface().get(), node->get_node_graph_interface()) {}

  std::shared_ptr<void> create_response() override {return nullptr;}
  std::shared_ptr<rmw_request_id_t> create_request_header() override {return nullptr;}
  void handle_response(
    std::shared_ptr<rmw_request_id_t>, std::shared_ptr<void>) override {}
};

class TestNodeService : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);

    rclcpp::NodeOptions options{};
    options.arguments(std::vector<std::string>{"-r", "foo:=bar"});
    node = std::make_shared<rclcpp::Node>("node", "ns", options);

    // This dynamic cast is not necessary for the unittest itself, but instead is used to ensure
    // the proper type is being tested and covered.
    node_services =
      dynamic_cast<rclcpp::node_interfaces::NodeServices *>(
      node->get_node_services_interface().get());
    ASSERT_NE(nullptr, node_services);
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::node_interfaces::NodeServices * node_services;
};

TEST_F(TestNodeService, add_service)
{
  auto service = std::make_shared<TestService>(node.get());
  auto callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  EXPECT_NO_THROW(
    node_services->add_service(service, callback_group));

  // Check that adding a service from node to a callback group of different_node throws exception.
  std::shared_ptr<rclcpp::Node> different_node = std::make_shared<rclcpp::Node>("node2", "ns");

  auto callback_group_in_different_node =
    different_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  RCLCPP_EXPECT_THROW_EQ(
    node_services->add_service(service, callback_group_in_different_node),
    std::runtime_error("Cannot create service, group not in node."));
}

TEST_F(TestNodeService, add_service_rcl_trigger_guard_condition_error)
{
  auto service = std::make_shared<TestService>(node.get());
  auto callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_trigger_guard_condition, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    node_services->add_service(service, callback_group),
    std::runtime_error("failed to notify wait set on service creation: error not set"));
}

TEST_F(TestNodeService, add_client)
{
  auto client = std::make_shared<TestClient>(node.get());
  auto callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  EXPECT_NO_THROW(node_services->add_client(client, callback_group));

  // Check that adding a client from node to a callback group of different_node throws exception.
  std::shared_ptr<rclcpp::Node> different_node = std::make_shared<rclcpp::Node>("node2", "ns");

  auto callback_group_in_different_node =
    different_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  RCLCPP_EXPECT_THROW_EQ(
    node_services->add_client(client, callback_group_in_different_node),
    std::runtime_error("Cannot create client, group not in node."));
}

TEST_F(TestNodeService, add_client_rcl_trigger_guard_condition_error)
{
  auto client = std::make_shared<TestClient>(node.get());
  auto callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_trigger_guard_condition, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    node_services->add_client(client, callback_group),
    std::runtime_error("failed to notify wait set on client creation: error not set"));
}

TEST_F(TestNodeService, resolve_service_name)
{
  EXPECT_EQ("/ns/bar", node_services->resolve_service_name("foo", false));
  EXPECT_EQ("/ns/foo", node_services->resolve_service_name("foo", true));
  EXPECT_EQ("/foo", node_services->resolve_service_name("/foo", true));
  EXPECT_THROW(
    node_services->resolve_service_name("this is not a valid name!~>", true),
    rclcpp::exceptions::RCLError);
}
