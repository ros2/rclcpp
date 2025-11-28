// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/node_type_descriptions.hpp"

class TestNodeTypeDescriptions : public ::testing::Test
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

TEST_F(TestNodeTypeDescriptions, interface_created)
{
  rclcpp::Node node{"node", "ns"};
  ASSERT_NE(nullptr, node.get_node_type_descriptions_interface());
}

TEST_F(TestNodeTypeDescriptions, automatically_declare_parameters_from_overrides)
{
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.append_parameter_override("start_type_description_service", false);
  ASSERT_NO_THROW(
  {
    auto node = std::make_shared<rclcpp::Node>("node", "ns", node_options);
    (void) node;
  });
}

TEST_F(TestNodeTypeDescriptions, bad_parameter_type)
{
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("start_type_description_service", "unexpected_type");

  ASSERT_THROW(
  {
    node_options.automatically_declare_parameters_from_overrides(false);
    auto node = std::make_shared<rclcpp::Node>("node", "ns", node_options);
    (void) node;
  }, rclcpp::exceptions::InvalidParameterTypeException);

  ASSERT_THROW(
  {
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("node", "ns", node_options);
    (void) node;
  }, rclcpp::exceptions::InvalidParameterTypeException);
}

TEST_F(TestNodeTypeDescriptions, disabled_no_service)
{
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("start_type_description_service", false);
  rclcpp::Node node{"node", "ns", node_options};

  auto services = node.get_node_graph_interface()->get_service_names_and_types_by_node(
    "node", "/ns");
  EXPECT_TRUE(services.find("/ns/node/get_type_description") == services.end());
}

TEST_F(TestNodeTypeDescriptions, enabled_creates_service)
{
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("start_type_description_service", true);
  rclcpp::Node node{"node", "ns", node_options};

  auto services = node.get_node_graph_interface()->get_service_names_and_types_by_node(
    "node", "/ns");

  EXPECT_TRUE(services.find("/ns/node/get_type_description") != services.end());
}
