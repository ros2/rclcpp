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

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "../../src/rclcpp/parameter_service_names.hpp"

using namespace std::chrono_literals;

// This tests the ParameterService as it is included in an rclcpp::Node. Creating a separate
// ParameterService would interfere with the built-in one
class TestParameterService : public ::testing::Test
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
    node = std::make_shared<rclcpp::Node>("test_parameter_service", "/ns");
    client = std::make_shared<rclcpp::SyncParametersClient>(node);
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::SyncParametersClient::SharedPtr client;
};

TEST_F(TestParameterService, get_parameters) {
  node->declare_parameter("parameter1", rclcpp::ParameterValue(42));
  EXPECT_EQ(42, client->get_parameter("parameter1", 0));

  EXPECT_EQ(-42, client->get_parameter("undeclared_parameter", -42));
}

TEST_F(TestParameterService, get_parameter_types) {
  node->declare_parameter("parameter1", rclcpp::ParameterValue(42));

  const std::vector<std::string> declared_parameters = {"parameter1"};
  const auto parameter_types = client->get_parameter_types(declared_parameters, 10s);
  ASSERT_EQ(1u, parameter_types.size());
  EXPECT_EQ(rclcpp::ParameterType::PARAMETER_INTEGER, parameter_types[0]);

  const std::vector<std::string> undeclared_parameters = {"parameter2"};
  const auto undeclared_parameter_types = client->get_parameter_types(undeclared_parameters, 10s);
  EXPECT_EQ(0u, undeclared_parameter_types.size());
}

TEST_F(TestParameterService, set_parameters) {
  node->declare_parameter("parameter1", rclcpp::ParameterValue(42));
  ASSERT_EQ(42, client->get_parameter("parameter1", 0));

  const std::vector<rclcpp::Parameter> parameters = {
    rclcpp::Parameter("parameter1", 0),
  };
  client->set_parameters(parameters, 10s);
  EXPECT_EQ(0, client->get_parameter("parameter1", 100));
}

TEST_F(TestParameterService, set_parameters_atomically) {
  node->declare_parameter("parameter1", rclcpp::ParameterValue(42));
  ASSERT_EQ(42, client->get_parameter("parameter1", 0));

  const std::vector<rclcpp::Parameter> parameters = {
    rclcpp::Parameter("parameter1", 0),
  };
  client->set_parameters_atomically(parameters, 10s);
  EXPECT_EQ(0, client->get_parameter("parameter1", 100));
}

TEST_F(TestParameterService, list_parameters) {
  const size_t number_parameters_in_basic_node = client->list_parameters({}, 1, 10s).names.size();
  node->declare_parameter("parameter1", rclcpp::ParameterValue(42));

  const auto list_result = client->list_parameters({}, 1, 10s);
  EXPECT_EQ(1u + number_parameters_in_basic_node, list_result.names.size());
}

TEST_F(TestParameterService, describe_parameters) {
  node->declare_parameter("parameter1", rclcpp::ParameterValue(42));

  {
    const std::vector<std::string> names{"parameter1"};
    std::vector<rcl_interfaces::msg::ParameterDescriptor> parameter_descs =
      client->describe_parameters(names, 10s);
    ASSERT_EQ(1u, parameter_descs.size());
    EXPECT_EQ("parameter1", parameter_descs[0].name);
    EXPECT_EQ(rclcpp::ParameterType::PARAMETER_INTEGER, parameter_descs[0].type);
  }

  {
    const std::vector<std::string> names{"undeclared_parameter"};
    std::vector<rcl_interfaces::msg::ParameterDescriptor> parameter_descs =
      client->describe_parameters(names, 10s);
    EXPECT_EQ(0u, parameter_descs.size());
  }
}
