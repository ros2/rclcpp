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

#include "rcl/node_options.h"
#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/node_services.hpp"
#include "rclcpp/rclcpp.hpp"

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

TEST(TestNodeService, add_service)
{
  rclcpp::init(0, nullptr);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("node", "ns");

  // This dynamic cast is not necessary for the unittest itself, but the coverage utility lcov
  // reports these functions uncovered otherwise.
  auto * node_services =
    dynamic_cast<rclcpp::node_interfaces::NodeServices *>(
    node->get_node_services_interface().get());
  ASSERT_NE(nullptr, node_services);

  std::shared_ptr<rclcpp::Node> node2 = std::make_shared<rclcpp::Node>("node2", "ns");

  auto callback_group = node2->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto service = std::make_shared<TestService>(node.get());
  EXPECT_THROW(
    node_services->add_service(service, callback_group),
    std::runtime_error);

  rclcpp::shutdown();
}

TEST(TestNodeService, add_client)
{
  rclcpp::init(0, nullptr);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("node", "ns");

  // This dynamic cast is not necessary for the unittest itself, but the coverage utility lcov
  // reports these functions uncovered otherwise.
  auto * node_services =
    dynamic_cast<rclcpp::node_interfaces::NodeServices *>(
    node->get_node_services_interface().get());
  ASSERT_NE(nullptr, node_services);

  std::shared_ptr<rclcpp::Node> node2 = std::make_shared<rclcpp::Node>("node2", "ns");

  auto callback_group = node2->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto client = std::make_shared<TestClient>(node.get());
  EXPECT_THROW(
    node_services->add_client(client, callback_group),
    std::runtime_error);

  rclcpp::shutdown();
}
