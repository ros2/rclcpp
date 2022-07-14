// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/node_interfaces/node_service_introspection.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rcl/service_introspection.h"

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
class TestNodeServiceIntrospection : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }
};

TEST_F(TestNodeServiceIntrospection, construct_from_node)
{
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>(
    "node", "ns",
    rclcpp::NodeOptions().enable_service_introspection(true));

  // This dynamic cast is not necessary for the unittest itself, but instead is used to ensure
  // the proper type is being tested and covered.
  auto * node_service_introspection =
    dynamic_cast<rclcpp::node_interfaces::NodeServiceIntrospection *>(
    node->get_node_service_introspection_interface().get());

  ASSERT_NE(nullptr, node_service_introspection);
  ASSERT_TRUE(node->get_parameter(RCL_SERVICE_INTROSPECTION_PUBLISH_CLIENT_PARAMETER));
  ASSERT_TRUE(node->get_parameter(RCL_SERVICE_INTROSPECTION_PUBLISH_SERVICE_PARAMETER));
  ASSERT_TRUE(
    node->get_parameter(RCL_SERVICE_INTROSPECTION_PUBLISH_CLIENT_EVENT_CONTENT_PARAMETER));
  ASSERT_TRUE(
    node->get_parameter(RCL_SERVICE_INTROSPECTION_PUBLISH_SERVICE_EVENT_CONTENT_PARAMETER));
}

TEST_F(TestNodeServiceIntrospection, register_services_and_clients)
{
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>(
    "node", "ns",
    rclcpp::NodeOptions().enable_service_introspection(true));

  // This dynamic cast is not necessary for the unittest itself, but instead is used to ensure
  // the proper type is being tested and covered.
  auto * node_service_introspection =
    dynamic_cast<rclcpp::node_interfaces::NodeServiceIntrospection *>(
    node->get_node_service_introspection_interface().get());

  auto service = std::make_shared<TestService>(node.get());
  auto client = std::make_shared<TestClient>(node.get());

  ASSERT_EQ(1, node_service_introspection->register_service(service));
  ASSERT_EQ(1, node_service_introspection->register_client(client));
}
