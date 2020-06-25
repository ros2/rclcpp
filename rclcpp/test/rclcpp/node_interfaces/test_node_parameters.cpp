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

/**
 * NodeParameters is a complicated interface with lots of code, but it is tested elsewhere
 * very thoroughly. This currently just includes unittests for the currently uncovered
 * functionality.
 */

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/node_parameters.hpp"

TEST(TestNodeParameters, list_parameters)
{
  rclcpp::init(0, nullptr);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("node", "ns");

  // This dynamic cast is not necessary for the unittest itself, but the coverage utility lcov
  // reports these functions uncovered otherwise.
  auto * node_parameters =
    dynamic_cast<rclcpp::node_interfaces::NodeParameters *>(
    node->get_node_parameters_interface().get());
  ASSERT_NE(nullptr, node_parameters);

  std::vector<std::string> prefixes;
  const auto list_result = node_parameters->list_parameters(prefixes, 1u);

  // Currently the only default parameter is 'use_sim_time', but that may change.
  EXPECT_GE(1u, list_result.names.size());
  size_t number_of_parameters = list_result.names.size();

  const std::string parameter_name = "new_parameter";
  const rclcpp::ParameterValue value(true);
  const rcl_interfaces::msg::ParameterDescriptor descriptor;
  const auto added_parameter_value =
    node_parameters->declare_parameter(parameter_name, value, descriptor, false);
  EXPECT_EQ(value.get<bool>(), added_parameter_value.get<bool>());

  auto list_result2 = node_parameters->list_parameters(prefixes, 1u);
  EXPECT_EQ(number_of_parameters + 1u, list_result2.names.size());

  bool parameter_added = false;
  for (const auto & name : list_result2.names) {
    if (name.compare(parameter_name) == 0) {
      parameter_added = true;
      break;
    }
  }

  EXPECT_TRUE(parameter_added);
  rclcpp::shutdown();
}

TEST(TestNodeParameters, parameter_overrides)
{
  rclcpp::init(0, nullptr);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("node", "ns");

  auto * node_parameters =
    dynamic_cast<rclcpp::node_interfaces::NodeParameters *>(
    node->get_node_parameters_interface().get());
  ASSERT_NE(nullptr, node_parameters);

  const auto & parameter_overrides = node_parameters->get_parameter_overrides();
  EXPECT_EQ(0u, parameter_overrides.size());
  rclcpp::shutdown();
}
