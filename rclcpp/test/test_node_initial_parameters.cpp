// Copyright 2018 Open Source Robotics Foundation, Inc.
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
#include <vector>

#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"

class TestNodeWithInitialValues : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, NULL);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }
};

TEST_F(TestNodeWithInitialValues, no_initial_values) {
  auto options = rclcpp::NodeOptions()
    .use_intra_process_comms(false)
    .use_global_arguments(false);

  auto node = rclcpp::Node::make_shared("node_name", options);
  auto list_params_result = node->list_parameters({}, 0);
  // Has use_sim_time parameter
  EXPECT_EQ(1u, list_params_result.names.size());
}

TEST_F(TestNodeWithInitialValues, multiple_undeclared_initial_values) {
  const std::vector<rclcpp::Parameter> initial_values = {
    rclcpp::Parameter("foo", true),
    rclcpp::Parameter("bar", "hello world"),
    rclcpp::Parameter("baz", std::vector<double>{3.14, 2.718})
  };

  auto options = rclcpp::NodeOptions()
    .initial_parameters(initial_values)
    .use_global_arguments(false)
    .use_intra_process_comms(false);

  auto node = rclcpp::Node::make_shared("node_name", options);
  auto list_params_result = node->list_parameters({}, 0);
  // Has use_sim_time parameter
  EXPECT_EQ(1u, list_params_result.names.size());
}

TEST_F(TestNodeWithInitialValues, multiple_declared_initial_values) {
  const std::vector<rclcpp::Parameter> initial_values = {
    rclcpp::Parameter("foo", true),
    rclcpp::Parameter("bar", "hello world"),
    rclcpp::Parameter("baz", std::vector<double>{3.14, 2.718})
  };

  auto options = rclcpp::NodeOptions()
    .initial_parameters(initial_values)
    .use_global_arguments(false);
  auto node = rclcpp::Node::make_shared("node_name", options);

  node->declare_parameter("foo");
  node->declare_parameter("bar");
  node->declare_parameter("baz");

  auto list_params_result = node->list_parameters({}, 0);
  EXPECT_TRUE(node->get_parameter("foo").get_value<bool>());
  EXPECT_STREQ("hello world", node->get_parameter("bar").get_value<std::string>().c_str());
  std::vector<double> double_array = node->get_parameter("baz").get_value<std::vector<double>>();
  ASSERT_EQ(2u, double_array.size());
  EXPECT_DOUBLE_EQ(3.14, double_array.at(0));
  EXPECT_DOUBLE_EQ(2.718, double_array.at(1));
}

TEST_F(TestNodeWithInitialValues, multiple_undeclared_initial_values_allowed) {
  const std::vector<rclcpp::Parameter> initial_values = {
    rclcpp::Parameter("foo", true),
    rclcpp::Parameter("bar", "hello world"),
    rclcpp::Parameter("baz", std::vector<double>{3.14, 2.718})
  };

  auto options = rclcpp::NodeOptions()
    .initial_parameters(initial_values)
    .use_global_arguments(false)
    .allow_undeclared_parameters(true);
  auto node = rclcpp::Node::make_shared("node_name", options);

  auto list_params_result = node->list_parameters({}, 0);
  EXPECT_TRUE(node->get_parameter("foo").get_value<bool>());
  EXPECT_STREQ("hello world", node->get_parameter("bar").get_value<std::string>().c_str());
  std::vector<double> double_array = node->get_parameter("baz").get_value<std::vector<double>>();
  ASSERT_EQ(2u, double_array.size());
  EXPECT_DOUBLE_EQ(3.14, double_array.at(0));
  EXPECT_DOUBLE_EQ(2.718, double_array.at(1));
}
