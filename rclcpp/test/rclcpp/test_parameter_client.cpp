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
#include <functional>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "../utils/rclcpp_gtest_macros.hpp"

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
    ASSERT_THROW(
    {
      auto asynchronous_client = std::make_shared<rclcpp::AsyncParametersClient>(
        node, "invalid_remote_node?");
      (void)asynchronous_client;
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
    ASSERT_THROW(
    {
      auto synchronous_client = std::make_shared<rclcpp::SyncParametersClient>(
        node, "invalid_remote_node?");
      (void)synchronous_client;
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

/*
   Coverage for simple get_parameter methods
 */
TEST_F(TestParameterClient, sync_parameter_get_parameter) {
  rclcpp::SyncParametersClient client(node);
  EXPECT_EQ(10, client.get_parameter("not_a_parameter", 10));

  RCLCPP_EXPECT_THROW_EQ(
    client.get_parameter<int>("not_a_parameter"),
    std::runtime_error("Parameter 'not_a_parameter' is not set"));
}

/*
   Coverage for async waiting/is_ready
 */
TEST_F(TestParameterClient, sync_parameter_is_ready) {
  rclcpp::SyncParametersClient client(node);
  EXPECT_TRUE(client.wait_for_service());
  EXPECT_TRUE(client.service_is_ready());
}

/*
  Coverage for async get_parameter_types
 */
TEST_F(TestParameterClient, async_parameter_get_parameter_types) {
  auto asynchronous_client = std::make_shared<rclcpp::AsyncParametersClient>(node);
  bool callback_called = false;
  auto callback = [&callback_called](std::shared_future<std::vector<rclcpp::ParameterType>> result)
    {
      // We expect the result to be empty since we tried to get a parameter that didn't exist.
      if (result.valid() && result.get().size() == 0) {
        callback_called = true;
      }
    };
  std::vector<std::string> names{"foo"};
  std::shared_future<std::vector<rclcpp::ParameterType>> future =
    asynchronous_client->get_parameter_types(names, callback);
  auto return_code = rclcpp::spin_until_future_complete(
    node, future, std::chrono::milliseconds(100));
  ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, return_code);
  ASSERT_TRUE(callback_called);
}

/*
  Coverage for async get_parameters
 */
TEST_F(TestParameterClient, async_parameter_get_parameters) {
  auto asynchronous_client = std::make_shared<rclcpp::AsyncParametersClient>(node);
  bool callback_called = false;
  auto callback = [&callback_called](std::shared_future<std::vector<rclcpp::Parameter>> result)
    {
      if (result.valid() && result.get().size() == 1 && result.get()[0].get_name() == "foo") {
        callback_called = true;
      }
    };
  std::vector<std::string> names{"foo"};
  std::shared_future<std::vector<rclcpp::Parameter>> future = asynchronous_client->get_parameters(
    names, callback);
  auto return_code = rclcpp::spin_until_future_complete(
    node, future, std::chrono::milliseconds(100));
  ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, return_code);
  ASSERT_TRUE(callback_called);
}

/*
  Coverage for async set_parameters_atomically
 */
TEST_F(TestParameterClient, async_parameter_set_parameters_atomically) {
  auto asynchronous_client = std::make_shared<rclcpp::AsyncParametersClient>(node);
  bool callback_called = false;
  auto callback =
    [&callback_called](std::shared_future<rcl_interfaces::msg::SetParametersResult> result)
    {
      // We expect this to fail since we didn't declare the parameter first.
      if (result.valid() && !result.get().successful &&
        result.get().reason == "One or more parameters were not declared before setting")
      {
        callback_called = true;
      }
    };
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back(rclcpp::Parameter("foo"));
  std::shared_future<rcl_interfaces::msg::SetParametersResult> future =
    asynchronous_client->set_parameters_atomically(parameters, callback);
  auto return_code = rclcpp::spin_until_future_complete(
    node, future, std::chrono::milliseconds(100));
  ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, return_code);
  ASSERT_TRUE(callback_called);
}

/*
  Coverage for async list_parameters
 */
TEST_F(TestParameterClient, async_parameter_list_parameters) {
  auto asynchronous_client = std::make_shared<rclcpp::AsyncParametersClient>(node);
  bool callback_called = false;
  auto callback =
    [&callback_called](std::shared_future<rcl_interfaces::msg::ListParametersResult> result)
    {
      if (result.valid() && result.get().names.size() == 0 && result.get().prefixes.size() == 0) {
        callback_called = true;
      }
    };
  std::vector<std::string> prefixes{"foo"};
  std::shared_future<rcl_interfaces::msg::ListParametersResult> future =
    asynchronous_client->list_parameters(prefixes, 0, callback);
  auto return_code = rclcpp::spin_until_future_complete(
    node, future, std::chrono::milliseconds(100));
  ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, return_code);
  ASSERT_TRUE(callback_called);
}

/*
  Coverage for sync get_parameter_types
 */
TEST_F(TestParameterClient, sync_parameter_get_parameter_types) {
  node->declare_parameter("foo", 4);
  auto synchronous_client = std::make_shared<rclcpp::SyncParametersClient>(node);
  std::vector<std::string> names{"foo"};
  std::vector<rclcpp::ParameterType> parameter_types =
    synchronous_client->get_parameter_types(names);
  ASSERT_EQ(1u, parameter_types.size());
  ASSERT_EQ(rclcpp::ParameterType::PARAMETER_INTEGER, parameter_types[0]);
}
