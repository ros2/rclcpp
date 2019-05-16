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

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "rcl_interfaces/msg/parameter_event.hpp"

class TestParameterClient : public ::testing::Test
{
public:
  void OnMessage(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
  {
    (void)event;
  }

protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp()
  {
    node = std::make_shared<rclcpp::Node>("test_parameter_client", "/ns");
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
};

/*
   Testing async parameter client construction and destruction.
 */
TEST_F(TestParameterClient, async_construction_and_destruction) {
  {
    auto asynchronous_client = std::make_shared<rclcpp::AsyncParametersClient>(node);
    (void)asynchronous_client;
  }

  {
    auto asynchronous_client = std::make_shared<rclcpp::AsyncParametersClient>(
      node->get_node_base_interface(),
      node->get_node_topics_interface(),
      node->get_node_graph_interface(),
      node->get_node_services_interface());
    (void)asynchronous_client;
  }

  {
    ASSERT_THROW({
      std::make_shared<rclcpp::AsyncParametersClient>(node, "invalid_remote_node?");
    }, rclcpp::exceptions::InvalidServiceNameError);
  }
}

/*
   Testing sync parameter client construction and destruction.
 */
TEST_F(TestParameterClient, sync_construction_and_destruction) {
  {
    auto synchronous_client = std::make_shared<rclcpp::SyncParametersClient>(node);
    (void)synchronous_client;
  }

  {
    auto synchronous_client = std::make_shared<rclcpp::SyncParametersClient>(
      std::make_shared<rclcpp::executors::SingleThreadedExecutor>(),
      node);
    (void)synchronous_client;
  }

  {
    auto synchronous_client = std::make_shared<rclcpp::SyncParametersClient>(
      std::make_shared<rclcpp::executors::SingleThreadedExecutor>(),
      node->get_node_base_interface(),
      node->get_node_topics_interface(),
      node->get_node_graph_interface(),
      node->get_node_services_interface());
    (void)synchronous_client;
  }

  {
    ASSERT_THROW({
      std::make_shared<rclcpp::SyncParametersClient>(node, "invalid_remote_node?");
    }, rclcpp::exceptions::InvalidServiceNameError);
  }
}

/*
   Testing different methods for parameter event subscription from asynchronous clients.
 */
TEST_F(TestParameterClient, async_parameter_event_subscription) {
  auto callback = std::bind(&TestParameterClient::OnMessage, this, std::placeholders::_1);
  {
    auto asynchronous_client = std::make_shared<rclcpp::AsyncParametersClient>(node);
    auto event_sub = asynchronous_client->on_parameter_event(callback);
    (void)event_sub;
  }

  {
    auto event_sub = rclcpp::AsyncParametersClient::on_parameter_event(node, callback);
    (void)event_sub;
  }

  {
    auto event_sub = rclcpp::AsyncParametersClient::on_parameter_event(
      node->get_node_topics_interface(),
      callback);
    (void)event_sub;
  }
}

/*
   Testing different methods for parameter event subscription from synchronous clients.
 */
TEST_F(TestParameterClient, sync_parameter_event_subscription) {
  auto callback = std::bind(&TestParameterClient::OnMessage, this, std::placeholders::_1);
  {
    auto synchronous_client = std::make_shared<rclcpp::SyncParametersClient>(node);
    auto event_sub = synchronous_client->on_parameter_event(callback);
    (void)event_sub;
  }

  {
    auto event_sub = rclcpp::SyncParametersClient::on_parameter_event(node, callback);
    (void)event_sub;
  }

  {
    auto event_sub = rclcpp::SyncParametersClient::on_parameter_event(
      node->get_node_topics_interface(),
      callback);
    (void)event_sub;
  }
}
