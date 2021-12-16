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

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/node_parameters.hpp"

#include "../../mocking_utils/patch.hpp"
#include "../../utils/rclcpp_gtest_macros.hpp"

#include "rcpputils/filesystem_helper.hpp"

class TestNodeParameters : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    node = std::make_shared<rclcpp::Node>("node", "ns", options);

    // This dynamic cast is not necessary for the unittest itself, but instead is used to ensure
    // the proper type is being tested and covered.
    node_parameters =
      dynamic_cast<rclcpp::node_interfaces::NodeParameters *>(
      node->get_node_parameters_interface().get());
    ASSERT_NE(nullptr, node_parameters);
    test_resources_path /= "test_node_parameters";
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::node_interfaces::NodeParameters * node_parameters;

  rcpputils::fs::path test_resources_path{TEST_RESOURCES_DIRECTORY};
};

TEST_F(TestNodeParameters, construct_destruct_rcl_errors) {
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_arguments_get_param_overrides, RCL_RET_ERROR);
  EXPECT_THROW(
    std::make_shared<rclcpp::Node>("node2", "ns").reset(),
    rclcpp::exceptions::RCLError);
}

TEST_F(TestNodeParameters, list_parameters)
{
  std::vector<std::string> prefixes;
  const auto list_result = node_parameters->list_parameters(prefixes, 1u);

  // Currently the only default parameter is 'use_sim_time', but that may change.
  size_t number_of_parameters = list_result.names.size();
  EXPECT_GE(1u, number_of_parameters);

  const std::string parameter_name = "new_parameter";
  const rclcpp::ParameterValue value(true);
  const rcl_interfaces::msg::ParameterDescriptor descriptor;
  const auto added_parameter_value =
    node_parameters->declare_parameter(parameter_name, value, descriptor, false);
  EXPECT_EQ(value.get<bool>(), added_parameter_value.get<bool>());

  auto list_result2 = node_parameters->list_parameters(prefixes, 1u);
  EXPECT_EQ(number_of_parameters + 1u, list_result2.names.size());

  EXPECT_NE(
    std::find(list_result2.names.begin(), list_result2.names.end(), parameter_name),
    list_result2.names.end());

  // Check prefixes
  const std::string parameter_name2 = "prefix.new_parameter";
  const rclcpp::ParameterValue value2(true);
  const rcl_interfaces::msg::ParameterDescriptor descriptor2;
  const auto added_parameter_value2 =
    node_parameters->declare_parameter(parameter_name2, value2, descriptor2, false);
  EXPECT_EQ(value.get<bool>(), added_parameter_value.get<bool>());
  prefixes = {"prefix"};
  auto list_result3 = node_parameters->list_parameters(prefixes, 2u);
  EXPECT_EQ(1u, list_result3.names.size());
  EXPECT_NE(
    std::find(list_result3.names.begin(), list_result3.names.end(), parameter_name2),
    list_result3.names.end());

  // Check if prefix equals parameter name
  prefixes = {"new_parameter"};
  auto list_result4 = node_parameters->list_parameters(prefixes, 2u);
  EXPECT_EQ(1u, list_result4.names.size());
  EXPECT_NE(
    std::find(list_result4.names.begin(), list_result4.names.end(), parameter_name),
    list_result4.names.end());
}

TEST_F(TestNodeParameters, parameter_overrides)
{
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.append_parameter_override("param1", true);
  node_options.append_parameter_override("param2", 42);

  std::shared_ptr<rclcpp::Node> node2 = std::make_shared<rclcpp::Node>("node2", "ns", node_options);

  auto * node_parameters_interface =
    dynamic_cast<rclcpp::node_interfaces::NodeParameters *>(
    node2->get_node_parameters_interface().get());
  ASSERT_NE(nullptr, node_parameters_interface);

  const auto & parameter_overrides = node_parameters_interface->get_parameter_overrides();
  EXPECT_EQ(2u, parameter_overrides.size());
}

TEST_F(TestNodeParameters, set_parameters) {
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);

  rcl_interfaces::msg::ParameterDescriptor bool_descriptor;
  bool_descriptor.name = "bool_parameter";
  bool_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  bool_descriptor.read_only = false;
  node_parameters->declare_parameter(
    "bool_parameter", rclcpp::ParameterValue(false), bool_descriptor, false);

  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.name = "read_only_parameter";
  read_only_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  read_only_descriptor.read_only = true;
  node_parameters->declare_parameter(
    "read_only_parameter", rclcpp::ParameterValue(42), read_only_descriptor, false);

  const std::vector<rclcpp::Parameter> parameters = {
    rclcpp::Parameter("bool_parameter", true),
    rclcpp::Parameter("read_only_parameter", 42),
  };
  auto result = node_parameters->set_parameters(parameters);
  ASSERT_EQ(parameters.size(), result.size());
  EXPECT_TRUE(result[0].successful);
  EXPECT_FALSE(result[1].successful);
  EXPECT_STREQ(
    "parameter 'read_only_parameter' cannot be set because it is read-only",
    result[1].reason.c_str());

  RCLCPP_EXPECT_THROW_EQ(
    node_parameters->set_parameters({rclcpp::Parameter("", true)}),
    rclcpp::exceptions::InvalidParametersException("parameter name must not be empty"));

  result = node_parameters->set_parameters({rclcpp::Parameter("undeclared_parameter", 3.14159)});
  ASSERT_EQ(1u, result.size());
  EXPECT_TRUE(result[0].successful);
}

TEST_F(TestNodeParameters, add_remove_parameters_callback) {
  rcl_interfaces::msg::ParameterDescriptor bool_descriptor;
  bool_descriptor.name = "bool_parameter";
  bool_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  bool_descriptor.read_only = false;
  node_parameters->declare_parameter(
    "bool_parameter", rclcpp::ParameterValue(false), bool_descriptor, false);
  const std::vector<rclcpp::Parameter> parameters = {rclcpp::Parameter("bool_parameter", true)};

  const std::string reason = "some totally not made up reason";
  auto callback = [reason](const std::vector<rclcpp::Parameter> &) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = false;
      result.reason = reason;
      return result;
    };

  auto handle = node_parameters->add_on_set_parameters_callback(callback);
  auto result = node_parameters->set_parameters(parameters);
  ASSERT_EQ(1u, result.size());
  EXPECT_FALSE(result[0].successful);
  EXPECT_EQ(reason, result[0].reason);

  EXPECT_NO_THROW(node_parameters->remove_on_set_parameters_callback(handle.get()));

  RCLCPP_EXPECT_THROW_EQ(
    node_parameters->remove_on_set_parameters_callback(handle.get()),
    std::runtime_error("Callback doesn't exist"));
}

TEST_F(TestNodeParameters, wildcard_with_namespace)
{
  rclcpp::NodeOptions opts;
  opts.arguments(
  {
    "--ros-args",
    "--params-file", (test_resources_path / "wildcards.yaml").string()
  });

  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("node2", "ns", opts);

  auto * node_parameters =
    dynamic_cast<rclcpp::node_interfaces::NodeParameters *>(
    node->get_node_parameters_interface().get());
  ASSERT_NE(nullptr, node_parameters);

  const auto & parameter_overrides = node_parameters->get_parameter_overrides();
  EXPECT_EQ(7u, parameter_overrides.size());
  EXPECT_EQ(parameter_overrides.at("full_wild").get<std::string>(), "full_wild");
  EXPECT_EQ(parameter_overrides.at("namespace_wild").get<std::string>(), "namespace_wild");
  EXPECT_EQ(
    parameter_overrides.at("namespace_wild_another").get<std::string>(),
    "namespace_wild_another");
  EXPECT_EQ(
    parameter_overrides.at("namespace_wild_one_star").get<std::string>(),
    "namespace_wild_one_star");
  EXPECT_EQ(parameter_overrides.at("node_wild_in_ns").get<std::string>(), "node_wild_in_ns");
  EXPECT_EQ(
    parameter_overrides.at("node_wild_in_ns_another").get<std::string>(),
    "node_wild_in_ns_another");
  EXPECT_EQ(parameter_overrides.at("explicit_in_ns").get<std::string>(), "explicit_in_ns");
  EXPECT_EQ(parameter_overrides.count("should_not_appear"), 0u);
}

TEST_F(TestNodeParameters, wildcard_no_namespace)
{
  rclcpp::NodeOptions opts;
  opts.arguments(
  {
    "--ros-args",
    "--params-file", (test_resources_path / "wildcards.yaml").string()
  });

  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("node2", opts);

  auto * node_parameters =
    dynamic_cast<rclcpp::node_interfaces::NodeParameters *>(
    node->get_node_parameters_interface().get());
  ASSERT_NE(nullptr, node_parameters);

  const auto & parameter_overrides = node_parameters->get_parameter_overrides();
  EXPECT_EQ(5u, parameter_overrides.size());
  EXPECT_EQ(parameter_overrides.at("full_wild").get<std::string>(), "full_wild");
  EXPECT_EQ(parameter_overrides.at("namespace_wild").get<std::string>(), "namespace_wild");
  EXPECT_EQ(
    parameter_overrides.at("namespace_wild_another").get<std::string>(),
    "namespace_wild_another");
  EXPECT_EQ(parameter_overrides.at("node_wild_no_ns").get<std::string>(), "node_wild_no_ns");
  EXPECT_EQ(parameter_overrides.at("explicit_no_ns").get<std::string>(), "explicit_no_ns");
  EXPECT_EQ(parameter_overrides.count("should_not_appear"), 0u);
  // "/*" match exactly one token, not expect to get `namespace_wild_one_star`
  EXPECT_EQ(parameter_overrides.count("namespace_wild_one_star"), 0u);
}

TEST_F(TestNodeParameters, params_by_order)
{
  rclcpp::NodeOptions opts;
  opts.arguments(
  {
    "--ros-args",
    "--params-file", (test_resources_path / "params_by_order.yaml").string()
  });

  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("node2", "ns", opts);

  auto * node_parameters =
    dynamic_cast<rclcpp::node_interfaces::NodeParameters *>(
    node->get_node_parameters_interface().get());
  ASSERT_NE(nullptr, node_parameters);

  const auto & parameter_overrides = node_parameters->get_parameter_overrides();
  EXPECT_EQ(3u, parameter_overrides.size());
  EXPECT_EQ(parameter_overrides.at("a_value").get<std::string>(), "last_one_win");
  EXPECT_EQ(parameter_overrides.at("foo").get<std::string>(), "foo");
  EXPECT_EQ(parameter_overrides.at("bar").get<std::string>(), "bar");
}

TEST_F(TestNodeParameters, complicated_wildcards)
{
  rclcpp::NodeOptions opts;
  opts.arguments(
  {
    "--ros-args",
    "--params-file", (test_resources_path / "complicated_wildcards.yaml").string()
  });

  {
    // regex matched: /**/foo/*/bar
    std::shared_ptr<rclcpp::Node> node =
      std::make_shared<rclcpp::Node>("node2", "/a/b/c/foo/d/bar", opts);

    auto * node_parameters =
      dynamic_cast<rclcpp::node_interfaces::NodeParameters *>(
      node->get_node_parameters_interface().get());
    ASSERT_NE(nullptr, node_parameters);

    const auto & parameter_overrides = node_parameters->get_parameter_overrides();
    EXPECT_EQ(2u, parameter_overrides.size());
    EXPECT_EQ(parameter_overrides.at("foo").get<std::string>(), "foo");
    EXPECT_EQ(parameter_overrides.at("bar").get<std::string>(), "bar");
  }

  {
    // regex not matched: /**/foo/*/bar
    std::shared_ptr<rclcpp::Node> node =
      std::make_shared<rclcpp::Node>("node2", "/a/b/c/foo/bar", opts);

    auto * node_parameters =
      dynamic_cast<rclcpp::node_interfaces::NodeParameters *>(
      node->get_node_parameters_interface().get());
    ASSERT_NE(nullptr, node_parameters);

    const auto & parameter_overrides = node_parameters->get_parameter_overrides();
    EXPECT_EQ(0u, parameter_overrides.size());
  }
}
