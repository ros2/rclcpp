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

using namespace std::chrono_literals;

class TestParameterClient : public ::testing::Test
{
public:
  void OnMessage(rcl_interfaces::msg::ParameterEvent::ConstSharedPtr event)
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
    node_with_option =
      std::make_shared<rclcpp::Node>(
      "test_parameter_client_allow_undeclare", "/ns",
      rclcpp::NodeOptions().allow_undeclared_parameters(true));
  }

  void TearDown()
  {
    node.reset();
    node_with_option.reset();
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::Node::SharedPtr node_with_option;
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
  Coverage for async get_parameter_types with allow_undeclared_ enabled
 */
TEST_F(TestParameterClient, async_parameter_get_parameter_types_allow_undeclared) {
  auto asynchronous_client =
    std::make_shared<rclcpp::AsyncParametersClient>(node_with_option);
  bool callback_called = false;
  auto callback = [&callback_called](std::shared_future<std::vector<rclcpp::ParameterType>> result)
    {
      if (result.valid() && result.get().size() == 1 &&
        result.get()[0] == rclcpp::PARAMETER_NOT_SET)
      {
        callback_called = true;
      }
    };
  std::vector<std::string> names{"foo"};
  std::shared_future<std::vector<rclcpp::ParameterType>> future =
    asynchronous_client->get_parameter_types(names, callback);
  auto return_code = rclcpp::spin_until_future_complete(
    node_with_option, future, std::chrono::milliseconds(100));
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
      // We expect the result to be empty since we tried to get a parameter that didn't exist.
      if (result.valid() && result.get().size() == 0) {
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
  Coverage for async get_parameters with allow_undeclared_ enabled
 */
TEST_F(TestParameterClient, async_parameter_get_parameters_allow_undeclared) {
  auto asynchronous_client =
    std::make_shared<rclcpp::AsyncParametersClient>(node_with_option);
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
    node_with_option, future, std::chrono::milliseconds(100));
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
  node->declare_parameter("bar", "this is bar");
  auto synchronous_client = std::make_shared<rclcpp::SyncParametersClient>(node);

  {
    std::vector<std::string> names{"none"};
    std::vector<rclcpp::ParameterType> parameter_types =
      synchronous_client->get_parameter_types(names, 10s);
    ASSERT_EQ(0u, parameter_types.size());
  }

  {
    std::vector<std::string> names{"none", "foo", "bar"};
    std::vector<rclcpp::ParameterType> parameter_types =
      synchronous_client->get_parameter_types(names, 10s);
    ASSERT_EQ(0u, parameter_types.size());
  }

  {
    std::vector<std::string> names{"foo"};
    std::vector<rclcpp::ParameterType> parameter_types =
      synchronous_client->get_parameter_types(names, 10s);
    ASSERT_EQ(1u, parameter_types.size());
    ASSERT_EQ(rclcpp::ParameterType::PARAMETER_INTEGER, parameter_types[0]);
  }

  {
    std::vector<std::string> names{"bar"};
    std::vector<rclcpp::ParameterType> parameter_types =
      synchronous_client->get_parameter_types(names, 10s);
    ASSERT_EQ(1u, parameter_types.size());
    ASSERT_EQ(rclcpp::ParameterType::PARAMETER_STRING, parameter_types[0]);
  }

  {
    std::vector<std::string> names{"foo", "bar"};
    std::vector<rclcpp::ParameterType> parameter_types =
      synchronous_client->get_parameter_types(names, 10s);
    ASSERT_EQ(2u, parameter_types.size());
    ASSERT_EQ(rclcpp::ParameterType::PARAMETER_INTEGER, parameter_types[0]);
    ASSERT_EQ(rclcpp::ParameterType::PARAMETER_STRING, parameter_types[1]);
  }
}

/*
  Coverage for sync get_parameter_types with allow_undeclared_ enabled
 */
TEST_F(TestParameterClient, sync_parameter_get_parameter_types_allow_undeclared) {
  node_with_option->declare_parameter("foo", 4);
  node_with_option->declare_parameter("bar", "this is bar");
  auto synchronous_client = std::make_shared<rclcpp::SyncParametersClient>(node_with_option);

  {
    std::vector<std::string> names{"none"};
    std::vector<rclcpp::ParameterType> parameter_types =
      synchronous_client->get_parameter_types(names, 10s);
    ASSERT_EQ(1u, parameter_types.size());
    ASSERT_EQ(rclcpp::ParameterType::PARAMETER_NOT_SET, parameter_types[0]);
  }

  {
    std::vector<std::string> names{"none", "foo", "bar"};
    std::vector<rclcpp::ParameterType> parameter_types =
      synchronous_client->get_parameter_types(names, 10s);
    ASSERT_EQ(3u, parameter_types.size());
    ASSERT_EQ(rclcpp::ParameterType::PARAMETER_NOT_SET, parameter_types[0]);
    ASSERT_EQ(rclcpp::ParameterType::PARAMETER_INTEGER, parameter_types[1]);
    ASSERT_EQ(rclcpp::ParameterType::PARAMETER_STRING, parameter_types[2]);
  }

  {
    std::vector<std::string> names{"foo"};
    std::vector<rclcpp::ParameterType> parameter_types =
      synchronous_client->get_parameter_types(names, 10s);
    ASSERT_EQ(1u, parameter_types.size());
    ASSERT_EQ(rclcpp::ParameterType::PARAMETER_INTEGER, parameter_types[0]);
  }

  {
    std::vector<std::string> names{"bar"};
    std::vector<rclcpp::ParameterType> parameter_types =
      synchronous_client->get_parameter_types(names, 10s);
    ASSERT_EQ(1u, parameter_types.size());
    ASSERT_EQ(rclcpp::ParameterType::PARAMETER_STRING, parameter_types[0]);
  }

  {
    std::vector<std::string> names{"foo", "bar"};
    std::vector<rclcpp::ParameterType> parameter_types =
      synchronous_client->get_parameter_types(names, 10s);
    ASSERT_EQ(2u, parameter_types.size());
    ASSERT_EQ(rclcpp::ParameterType::PARAMETER_INTEGER, parameter_types[0]);
    ASSERT_EQ(rclcpp::ParameterType::PARAMETER_STRING, parameter_types[1]);
  }
}

/*
  Coverage for sync get_parameters
 */
TEST_F(TestParameterClient, sync_parameter_get_parameters) {
  node->declare_parameter("foo", 4);
  node->declare_parameter("bar", "this is bar");
  auto synchronous_client = std::make_shared<rclcpp::SyncParametersClient>(node);

  {
    std::vector<std::string> names{"none"};
    std::vector<rclcpp::Parameter> parameters = synchronous_client->get_parameters(names, 10s);
    ASSERT_EQ(0u, parameters.size());
  }

  {
    std::vector<std::string> names{"none", "foo", "bar"};
    std::vector<rclcpp::Parameter> parameters = synchronous_client->get_parameters(names, 10s);
    ASSERT_EQ(0u, parameters.size());
  }

  {
    std::vector<std::string> names{"foo"};
    std::vector<rclcpp::Parameter> parameters = synchronous_client->get_parameters(names, 10s);
    ASSERT_EQ(1u, parameters.size());
    ASSERT_EQ("foo", parameters[0].get_name());
    ASSERT_EQ(4u, parameters[0].as_int());
  }

  {
    std::vector<std::string> names{"bar"};
    std::vector<rclcpp::Parameter> parameters = synchronous_client->get_parameters(names, 10s);
    ASSERT_EQ(1u, parameters.size());
    ASSERT_EQ("bar", parameters[0].get_name());
    ASSERT_EQ("this is bar", parameters[0].as_string());
  }

  {
    std::vector<std::string> names{"foo", "bar"};
    std::vector<rclcpp::Parameter> parameters = synchronous_client->get_parameters(names, 10s);
    ASSERT_EQ(2u, parameters.size());
    ASSERT_EQ("foo", parameters[0].get_name());
    ASSERT_EQ(4u, parameters[0].as_int());
    ASSERT_EQ("bar", parameters[1].get_name());
    ASSERT_EQ("this is bar", parameters[1].as_string());
  }
}

/*
  Coverage for sync get_parameters with allow_undeclared_ enabled
 */
TEST_F(TestParameterClient, sync_parameter_get_parameters_allow_undeclared) {
  node_with_option->declare_parameter("foo", 4);
  node_with_option->declare_parameter("bar", "this is bar");
  auto synchronous_client = std::make_shared<rclcpp::SyncParametersClient>(node_with_option);

  {
    std::vector<std::string> names{"none"};
    std::vector<rclcpp::Parameter> parameters = synchronous_client->get_parameters(names, 10s);
    ASSERT_EQ(1u, parameters.size());
  }

  {
    std::vector<std::string> names{"none", "foo", "bar"};
    std::vector<rclcpp::Parameter> parameters = synchronous_client->get_parameters(names, 10s);
    ASSERT_EQ(3u, parameters.size());
    ASSERT_EQ("foo", parameters[1].get_name());
    ASSERT_EQ(4u, parameters[1].as_int());
    ASSERT_EQ("bar", parameters[2].get_name());
    ASSERT_EQ("this is bar", parameters[2].as_string());
  }

  {
    std::vector<std::string> names{"foo"};
    std::vector<rclcpp::Parameter> parameters = synchronous_client->get_parameters(names, 10s);
    ASSERT_EQ(1u, parameters.size());
    ASSERT_EQ("foo", parameters[0].get_name());
    ASSERT_EQ(4u, parameters[0].as_int());
  }

  {
    std::vector<std::string> names{"bar"};
    std::vector<rclcpp::Parameter> parameters = synchronous_client->get_parameters(names, 10s);
    ASSERT_EQ(1u, parameters.size());
    ASSERT_EQ("bar", parameters[0].get_name());
    ASSERT_EQ("this is bar", parameters[0].as_string());
  }

  {
    std::vector<std::string> names{"foo", "bar"};
    std::vector<rclcpp::Parameter> parameters = synchronous_client->get_parameters(names, 10s);
    ASSERT_EQ(2u, parameters.size());
    ASSERT_EQ("foo", parameters[0].get_name());
    ASSERT_EQ(4u, parameters[0].as_int());
    ASSERT_EQ("bar", parameters[1].get_name());
    ASSERT_EQ("this is bar", parameters[1].as_string());
  }
}

/*
  Coverage for async describe_parameters
 */
TEST_F(TestParameterClient, async_parameter_describe_parameters) {
  node->declare_parameter("foo", 4);
  node->declare_parameter("bar", "this is bar");
  auto asynchronous_client = std::make_shared<rclcpp::AsyncParametersClient>(node);

  {
    bool callback_called = false;
    auto callback = [&callback_called](
      std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>> result)
      {
        // We expect the result to be empty since we tried to get a parameter that didn't exist.
        if (result.valid() && result.get().size() == 0) {
          callback_called = true;
        }
      };
    std::vector<std::string> names{"none"};
    std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>> future =
      asynchronous_client->describe_parameters(names, callback);
    auto return_code = rclcpp::spin_until_future_complete(
      node, future, std::chrono::milliseconds(100));
    ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, return_code);
    ASSERT_TRUE(callback_called);
  }

  {
    bool callback_called = false;
    auto callback = [&callback_called](
      std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>> result)
      {
        if (result.valid() && result.get().size() == 1) {
          callback_called = true;
        }
      };
    std::vector<std::string> names{"foo"};
    std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>> future =
      asynchronous_client->describe_parameters(names, callback);
    auto return_code = rclcpp::spin_until_future_complete(
      node, future, std::chrono::milliseconds(100));
    ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, return_code);
    std::vector<rcl_interfaces::msg::ParameterDescriptor> parameter_descs = future.get();
    ASSERT_EQ(1u, parameter_descs.size());
    ASSERT_EQ("foo", parameter_descs[0].name);
    ASSERT_EQ(rclcpp::ParameterType::PARAMETER_INTEGER, parameter_descs[0].type);
    ASSERT_EQ("", parameter_descs[0].description);
    ASSERT_EQ("", parameter_descs[0].additional_constraints);
    ASSERT_FALSE(parameter_descs[0].read_only);
    ASSERT_TRUE(callback_called);
  }

  {
    bool callback_called = false;
    auto callback = [&callback_called](
      std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>> result)
      {
        // We expect the result to be empty since we tried to get a parameter that didn't exist.
        if (result.valid() && result.get().size() == 0) {
          callback_called = true;
        }
      };
    std::vector<std::string> names{"foo", "baz"};
    std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>> future =
      asynchronous_client->describe_parameters(names, callback);
    auto return_code = rclcpp::spin_until_future_complete(
      node, future, std::chrono::milliseconds(100));
    ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, return_code);
    ASSERT_TRUE(callback_called);
  }

  {
    bool callback_called = false;
    auto callback = [&callback_called](
      std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>> result)
      {
        // We expect the result to be empty since we tried to get a parameter that didn't exist.
        if (result.valid() && result.get().size() == 0) {
          callback_called = true;
        }
      };
    std::vector<std::string> names{"baz", "foo"};
    std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>> future =
      asynchronous_client->describe_parameters(names, callback);
    auto return_code = rclcpp::spin_until_future_complete(
      node, future, std::chrono::milliseconds(100));
    ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, return_code);
    ASSERT_TRUE(callback_called);
  }

  {
    bool callback_called = false;
    auto callback = [&callback_called](
      std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>> result)
      {
        if (result.valid() && result.get().size() == 2) {
          callback_called = true;
        }
      };
    std::vector<std::string> names{"foo", "bar"};
    std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>> future =
      asynchronous_client->describe_parameters(names, callback);
    auto return_code = rclcpp::spin_until_future_complete(
      node, future, std::chrono::milliseconds(100));
    ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, return_code);
    std::vector<rcl_interfaces::msg::ParameterDescriptor> parameter_descs = future.get();
    ASSERT_EQ(2u, parameter_descs.size());
    ASSERT_EQ("foo", parameter_descs[0].name);
    ASSERT_EQ(rclcpp::ParameterType::PARAMETER_INTEGER, parameter_descs[0].type);
    ASSERT_EQ("", parameter_descs[0].description);
    ASSERT_EQ("", parameter_descs[0].additional_constraints);
    ASSERT_FALSE(parameter_descs[0].read_only);
    ASSERT_EQ("bar", parameter_descs[1].name);
    ASSERT_EQ(rclcpp::ParameterType::PARAMETER_STRING, parameter_descs[1].type);
    ASSERT_EQ("", parameter_descs[1].description);
    ASSERT_EQ("", parameter_descs[1].additional_constraints);
    ASSERT_FALSE(parameter_descs[1].read_only);
    ASSERT_TRUE(callback_called);
  }
}
/*
  Coverage for sync describe_parameters
 */
TEST_F(TestParameterClient, sync_parameter_describe_parameters) {
  node->declare_parameter("foo", 4);
  node->declare_parameter("bar", "this is bar");
  auto synchronous_client = std::make_shared<rclcpp::SyncParametersClient>(node);

  {
    std::vector<std::string> names{"none"};
    std::vector<rcl_interfaces::msg::ParameterDescriptor> parameter_descs =
      synchronous_client->describe_parameters(names, 10s);
    ASSERT_EQ(0u, parameter_descs.size());
  }

  {
    std::vector<std::string> names{"foo"};
    std::vector<rcl_interfaces::msg::ParameterDescriptor> parameter_descs =
      synchronous_client->describe_parameters(names, 10s);
    ASSERT_EQ(1u, parameter_descs.size());
    ASSERT_EQ("foo", parameter_descs[0].name);
    ASSERT_EQ(rclcpp::ParameterType::PARAMETER_INTEGER, parameter_descs[0].type);
    ASSERT_EQ("", parameter_descs[0].description);
    ASSERT_EQ("", parameter_descs[0].additional_constraints);
    ASSERT_FALSE(parameter_descs[0].read_only);
  }

  {
    std::vector<std::string> names{"foo", "baz"};
    std::vector<rcl_interfaces::msg::ParameterDescriptor> parameter_descs =
      synchronous_client->describe_parameters(names, 10s);
    ASSERT_EQ(0u, parameter_descs.size());
  }

  {
    std::vector<std::string> names{"baz", "foo"};
    std::vector<rcl_interfaces::msg::ParameterDescriptor> parameter_descs =
      synchronous_client->describe_parameters(names, 10s);
    ASSERT_EQ(0u, parameter_descs.size());
  }

  {
    std::vector<std::string> names{"foo", "bar"};
    std::vector<rcl_interfaces::msg::ParameterDescriptor> parameter_descs =
      synchronous_client->describe_parameters(names, 10s);
    ASSERT_EQ(2u, parameter_descs.size());
    ASSERT_EQ("foo", parameter_descs[0].name);
    ASSERT_EQ(rclcpp::ParameterType::PARAMETER_INTEGER, parameter_descs[0].type);
    ASSERT_EQ("", parameter_descs[0].description);
    ASSERT_EQ("", parameter_descs[0].additional_constraints);
    ASSERT_FALSE(parameter_descs[0].read_only);
    ASSERT_EQ("bar", parameter_descs[1].name);
    ASSERT_EQ(rclcpp::ParameterType::PARAMETER_STRING, parameter_descs[1].type);
    ASSERT_EQ("", parameter_descs[1].description);
    ASSERT_EQ("", parameter_descs[1].additional_constraints);
    ASSERT_FALSE(parameter_descs[1].read_only);
  }
}

/*
  Coverage for async describe_parameters with allow_undeclared_ enabled
 */
TEST_F(TestParameterClient, async_parameter_describe_parameters_allow_undeclared) {
  node_with_option->declare_parameter("foo", 4);
  node_with_option->declare_parameter("bar", "this is bar");
  auto asynchronous_client =
    std::make_shared<rclcpp::AsyncParametersClient>(node_with_option);

  {
    bool callback_called = false;
    auto callback = [&callback_called](
      std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>> result)
      {
        // We expect the result to be defaut since we tried to get a parameter that didn't exist.
        if (result.valid() && result.get().size() == 1) {
          callback_called = true;
        }
      };
    std::vector<std::string> names{"none"};
    std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>> future =
      asynchronous_client->describe_parameters(names, callback);
    auto return_code = rclcpp::spin_until_future_complete(
      node_with_option, future, std::chrono::milliseconds(100));
    ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, return_code);
    std::vector<rcl_interfaces::msg::ParameterDescriptor> parameter_descs = future.get();
    ASSERT_EQ(1u, parameter_descs.size());
    ASSERT_EQ("none", parameter_descs[0].name);
    ASSERT_EQ(rclcpp::ParameterType::PARAMETER_NOT_SET, parameter_descs[0].type);
    ASSERT_EQ("", parameter_descs[0].description);
    ASSERT_EQ("", parameter_descs[0].additional_constraints);
    ASSERT_FALSE(parameter_descs[0].read_only);
    ASSERT_TRUE(callback_called);
  }

  {
    bool callback_called = false;
    auto callback = [&callback_called](
      std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>> result)
      {
        if (result.valid() && result.get().size() == 2) {
          callback_called = true;
        }
      };
    std::vector<std::string> names{"foo", "baz"};
    std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>> future =
      asynchronous_client->describe_parameters(names, callback);
    auto return_code = rclcpp::spin_until_future_complete(
      node_with_option, future, std::chrono::milliseconds(100));
    ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, return_code);
    std::vector<rcl_interfaces::msg::ParameterDescriptor> parameter_descs = future.get();
    ASSERT_EQ(2u, parameter_descs.size());
    ASSERT_EQ("foo", parameter_descs[0].name);
    ASSERT_EQ(rclcpp::ParameterType::PARAMETER_INTEGER, parameter_descs[0].type);
    ASSERT_EQ("", parameter_descs[0].description);
    ASSERT_EQ("", parameter_descs[0].additional_constraints);
    ASSERT_FALSE(parameter_descs[0].read_only);
    ASSERT_EQ("baz", parameter_descs[1].name);
    ASSERT_EQ(rclcpp::ParameterType::PARAMETER_NOT_SET, parameter_descs[1].type);
    ASSERT_EQ("", parameter_descs[1].description);
    ASSERT_EQ("", parameter_descs[1].additional_constraints);
    ASSERT_FALSE(parameter_descs[1].read_only);
    ASSERT_TRUE(callback_called);
  }

  {
    bool callback_called = false;
    auto callback = [&callback_called](
      std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>> result)
      {
        if (result.valid() && result.get().size() == 2) {
          callback_called = true;
        }
      };
    std::vector<std::string> names{"baz", "foo"};
    std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>> future =
      asynchronous_client->describe_parameters(names, callback);
    auto return_code = rclcpp::spin_until_future_complete(
      node_with_option, future, std::chrono::milliseconds(100));
    ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, return_code);
    std::vector<rcl_interfaces::msg::ParameterDescriptor> parameter_descs = future.get();
    ASSERT_EQ(2u, parameter_descs.size());
    ASSERT_EQ("baz", parameter_descs[0].name);
    ASSERT_EQ(rclcpp::ParameterType::PARAMETER_NOT_SET, parameter_descs[0].type);
    ASSERT_EQ("", parameter_descs[0].description);
    ASSERT_EQ("", parameter_descs[0].additional_constraints);
    ASSERT_FALSE(parameter_descs[0].read_only);
    ASSERT_EQ("foo", parameter_descs[1].name);
    ASSERT_EQ(rclcpp::ParameterType::PARAMETER_INTEGER, parameter_descs[1].type);
    ASSERT_EQ("", parameter_descs[1].description);
    ASSERT_EQ("", parameter_descs[1].additional_constraints);
    ASSERT_FALSE(parameter_descs[1].read_only);
    ASSERT_TRUE(callback_called);
  }
}
/*
  Coverage for sync describe_parameters with allow_undeclared_ enabled
 */
TEST_F(TestParameterClient, sync_parameter_describe_parameters_allow_undeclared) {
  node_with_option->declare_parameter("foo", 4);
  node_with_option->declare_parameter("bar", "this is bar");
  auto synchronous_client =
    std::make_shared<rclcpp::SyncParametersClient>(node_with_option);

  {
    std::vector<std::string> names{"none"};
    std::vector<rcl_interfaces::msg::ParameterDescriptor> parameter_descs =
      synchronous_client->describe_parameters(names, 10s);
    ASSERT_EQ(1u, parameter_descs.size());
    ASSERT_EQ("none", parameter_descs[0].name);
    ASSERT_EQ(rclcpp::ParameterType::PARAMETER_NOT_SET, parameter_descs[0].type);
    ASSERT_EQ("", parameter_descs[0].description);
    ASSERT_EQ("", parameter_descs[0].additional_constraints);
    ASSERT_FALSE(parameter_descs[0].read_only);
  }

  {
    std::vector<std::string> names{"foo", "baz"};
    std::vector<rcl_interfaces::msg::ParameterDescriptor> parameter_descs =
      synchronous_client->describe_parameters(names, 10s);
    ASSERT_EQ(2u, parameter_descs.size());
    ASSERT_EQ("foo", parameter_descs[0].name);
    ASSERT_EQ(rclcpp::ParameterType::PARAMETER_INTEGER, parameter_descs[0].type);
    ASSERT_EQ("", parameter_descs[0].description);
    ASSERT_EQ("", parameter_descs[0].additional_constraints);
    ASSERT_FALSE(parameter_descs[0].read_only);
    ASSERT_EQ("baz", parameter_descs[1].name);
    ASSERT_EQ(rclcpp::ParameterType::PARAMETER_NOT_SET, parameter_descs[1].type);
    ASSERT_EQ("", parameter_descs[1].description);
    ASSERT_EQ("", parameter_descs[1].additional_constraints);
    ASSERT_FALSE(parameter_descs[1].read_only);
  }

  {
    std::vector<std::string> names{"baz", "foo"};
    std::vector<rcl_interfaces::msg::ParameterDescriptor> parameter_descs =
      synchronous_client->describe_parameters(names, 10s);
    ASSERT_EQ(2u, parameter_descs.size());
    ASSERT_EQ("baz", parameter_descs[0].name);
    ASSERT_EQ(rclcpp::ParameterType::PARAMETER_NOT_SET, parameter_descs[0].type);
    ASSERT_EQ("", parameter_descs[0].description);
    ASSERT_EQ("", parameter_descs[0].additional_constraints);
    ASSERT_FALSE(parameter_descs[0].read_only);
    ASSERT_EQ("foo", parameter_descs[1].name);
    ASSERT_EQ(rclcpp::ParameterType::PARAMETER_INTEGER, parameter_descs[1].type);
    ASSERT_EQ("", parameter_descs[1].description);
    ASSERT_EQ("", parameter_descs[1].additional_constraints);
    ASSERT_FALSE(parameter_descs[1].read_only);
  }
}

/*
  Coverage for async delete_parameters
 */
TEST_F(TestParameterClient, async_parameter_delete_parameters) {
  auto asynchronous_client =
    std::make_shared<rclcpp::AsyncParametersClient>(node_with_option);
  // set parameter
  auto set_future = asynchronous_client->set_parameters({rclcpp::Parameter("foo", 4)});
  rclcpp::spin_until_future_complete(
    node_with_option, set_future, std::chrono::milliseconds(100));
  ASSERT_EQ(set_future.get()[0].successful, true);
  // delete one parameter
  auto delete_future = asynchronous_client->delete_parameters({"foo"});
  rclcpp::spin_until_future_complete(
    node_with_option, delete_future, std::chrono::milliseconds(100));
  ASSERT_EQ(delete_future.get()[0].successful, true);
  // check that deleted parameter isn't set
  auto get_future2 = asynchronous_client->get_parameters({"foo"});
  rclcpp::spin_until_future_complete(
    node_with_option, get_future2, std::chrono::milliseconds(100));
  ASSERT_EQ(
    get_future2.get()[0].get_type(),
    rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET);
}
/*
  Coverage for sync delete_parameters
 */
TEST_F(TestParameterClient, sync_parameter_delete_parameters) {
  auto synchronous_client =
    std::make_shared<rclcpp::SyncParametersClient>(node_with_option);
  // set parameter
  auto set_result = synchronous_client->set_parameters({rclcpp::Parameter("foo", 4)});
  // delete one parameter
  auto delete_result = synchronous_client->delete_parameters({"foo"});
  // check that deleted parameter isn't set
  auto get_result = synchronous_client->get_parameters({"foo"});
  ASSERT_EQ(
    get_result[0].get_type(),
    rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET);
}

/*
  Coverage for async load_parameters
 */
TEST_F(TestParameterClient, async_parameter_load_parameters) {
  auto load_node = std::make_shared<rclcpp::Node>(
    "load_node",
    "namespace",
    rclcpp::NodeOptions().allow_undeclared_parameters(true));
  auto asynchronous_client =
    std::make_shared<rclcpp::AsyncParametersClient>(load_node, "/namespace/load_node");
  // load parameters
  rcpputils::fs::path test_resources_path{TEST_RESOURCES_DIRECTORY};
  const std::string parameters_filepath = (
    test_resources_path / "test_node" / "load_parameters.yaml").string();
  auto load_future = asynchronous_client->load_parameters(parameters_filepath);
  auto result_code = rclcpp::spin_until_future_complete(
    load_node, load_future, std::chrono::milliseconds(100));
  ASSERT_EQ(result_code, rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_EQ(load_future.get()[0].successful, true);
  // list parameters
  auto list_parameters = asynchronous_client->list_parameters({}, 3);
  rclcpp::spin_until_future_complete(
    load_node, list_parameters, std::chrono::milliseconds(100));
  ASSERT_EQ(list_parameters.get().names.size(), static_cast<uint64_t>(5));
}
/*
  Coverage for sync load_parameters
 */
TEST_F(TestParameterClient, sync_parameter_load_parameters) {
  auto load_node = std::make_shared<rclcpp::Node>(
    "load_node",
    "namespace",
    rclcpp::NodeOptions().allow_undeclared_parameters(true));
  auto synchronous_client =
    std::make_shared<rclcpp::SyncParametersClient>(load_node);
  // load parameters
  rcpputils::fs::path test_resources_path{TEST_RESOURCES_DIRECTORY};
  const std::string parameters_filepath = (
    test_resources_path / "test_node" / "load_parameters.yaml").string();
  auto load_future = synchronous_client->load_parameters(parameters_filepath);
  ASSERT_EQ(load_future[0].successful, true);
  // list parameters
  auto list_parameters = synchronous_client->list_parameters({}, 3);
  ASSERT_EQ(list_parameters.names.size(), static_cast<uint64_t>(5));
}
