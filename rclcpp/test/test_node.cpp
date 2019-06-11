// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include <map>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/scope_exit.hpp"
#include "rclcpp/rclcpp.hpp"

class TestNode : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }
};

/*
   Testing node construction and destruction.
 */
TEST_F(TestNode, construction_and_destruction) {
  {
    std::make_shared<rclcpp::Node>("my_node", "/ns");
  }

  {
    ASSERT_THROW({
      std::make_shared<rclcpp::Node>("invalid_node?", "/ns");
    }, rclcpp::exceptions::InvalidNodeNameError);
  }

  {
    ASSERT_THROW({
      std::make_shared<rclcpp::Node>("my_node", "/invalid_ns?");
    }, rclcpp::exceptions::InvalidNamespaceError);
  }
}

TEST_F(TestNode, get_name_and_namespace) {
  {
    auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
    EXPECT_STREQ("my_node", node->get_name());
    EXPECT_STREQ("/ns", node->get_namespace());
    EXPECT_STREQ("/ns/my_node", node->get_fully_qualified_name());
  }
  {
    auto options = rclcpp::NodeOptions()
      .arguments({"__ns:=/another_ns"});
    auto node = std::make_shared<rclcpp::Node>("my_node", "/ns", options);
    EXPECT_STREQ("my_node", node->get_name());
    EXPECT_STREQ("/another_ns", node->get_namespace());
    EXPECT_STREQ("/another_ns/my_node", node->get_fully_qualified_name());
  }
  {
    auto node = std::make_shared<rclcpp::Node>("my_node", "ns");
    EXPECT_STREQ("my_node", node->get_name());
    EXPECT_STREQ("/ns", node->get_namespace());
    EXPECT_STREQ("/ns/my_node", node->get_fully_qualified_name());
  }
  {
    auto node = std::make_shared<rclcpp::Node>("my_node");
    EXPECT_STREQ("my_node", node->get_name());
    EXPECT_STREQ("/", node->get_namespace());
    EXPECT_STREQ("/my_node", node->get_fully_qualified_name());
  }
  {
    auto node = std::make_shared<rclcpp::Node>("my_node", "");
    EXPECT_STREQ("my_node", node->get_name());
    EXPECT_STREQ("/", node->get_namespace());
    EXPECT_STREQ("/my_node", node->get_fully_qualified_name());
  }
  {
    auto node = std::make_shared<rclcpp::Node>("my_node", "/my/ns");
    EXPECT_STREQ("my_node", node->get_name());
    EXPECT_STREQ("/my/ns", node->get_namespace());
    EXPECT_STREQ("/my/ns/my_node", node->get_fully_qualified_name());
  }
  {
    auto node = std::make_shared<rclcpp::Node>("my_node", "my/ns");
    EXPECT_STREQ("my_node", node->get_name());
    EXPECT_STREQ("/my/ns", node->get_namespace());
    EXPECT_STREQ("/my/ns/my_node", node->get_fully_qualified_name());
  }
  {
    auto node1 = std::make_shared<rclcpp::Node>("my_node1", "my/ns");
    auto node2 = std::make_shared<rclcpp::Node>("my_node2", "my/ns");
    auto node3 = std::make_shared<rclcpp::Node>("my_node3", "/ns2");
    auto node4 = std::make_shared<rclcpp::Node>("my_node4", "my/ns3");
    auto names_and_namespaces = node1->get_node_names();
    auto name_namespace_set = std::unordered_set<std::string>(names_and_namespaces.begin(),
        names_and_namespaces.end());
    std::function<bool(std::string)> Set_Contains = [&](std::string string_key)
      {
        return name_namespace_set.find(string_key) != name_namespace_set.end();
      };
    EXPECT_TRUE(Set_Contains("/my/ns/my_node1"));
    EXPECT_TRUE(Set_Contains("/my/ns/my_node2"));
    EXPECT_TRUE(Set_Contains("/ns2/my_node3"));
    EXPECT_TRUE(Set_Contains("/my/ns3/my_node4"));
  }
}

TEST_F(TestNode, subnode_get_name_and_namespace) {
  {
    auto node = std::make_shared<rclcpp::Node>("my_node", "ns");
    auto subnode = node->create_sub_node("sub_ns");
    EXPECT_STREQ("my_node", subnode->get_name());
    EXPECT_STREQ("/ns", subnode->get_namespace());
    EXPECT_STREQ("sub_ns", subnode->get_sub_namespace().c_str());
    EXPECT_STREQ("/ns/sub_ns", subnode->get_effective_namespace().c_str());
  }
  {
    auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
    auto subnode = node->create_sub_node("sub_ns");
    EXPECT_STREQ("my_node", subnode->get_name());
    EXPECT_STREQ("/ns", subnode->get_namespace());
    EXPECT_STREQ("sub_ns", subnode->get_sub_namespace().c_str());
    EXPECT_STREQ("/ns/sub_ns", subnode->get_effective_namespace().c_str());
  }
  {
    auto node = std::make_shared<rclcpp::Node>("my_node");
    auto subnode = node->create_sub_node("sub_ns");
    EXPECT_STREQ("my_node", subnode->get_name());
    EXPECT_STREQ("/", subnode->get_namespace());
    EXPECT_STREQ("sub_ns", subnode->get_sub_namespace().c_str());
    EXPECT_STREQ("/sub_ns", subnode->get_effective_namespace().c_str());
  }
  {
    auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
    auto subnode = node->create_sub_node("sub_ns");
    EXPECT_STREQ("my_node", subnode->get_name());
    EXPECT_STREQ("/ns", subnode->get_namespace());
    EXPECT_STREQ("sub_ns", subnode->get_sub_namespace().c_str());
    EXPECT_STREQ("/ns/sub_ns", subnode->get_effective_namespace().c_str());
    auto subnode2 = subnode->create_sub_node("sub_ns2");
    EXPECT_STREQ("my_node", subnode2->get_name());
    EXPECT_STREQ("/ns", subnode2->get_namespace());
    EXPECT_STREQ("sub_ns/sub_ns2", subnode2->get_sub_namespace().c_str());
    EXPECT_STREQ("/ns/sub_ns/sub_ns2", subnode2->get_effective_namespace().c_str());
  }
  {
    auto node = std::make_shared<rclcpp::Node>("my_node");
    auto subnode = node->create_sub_node("sub_ns");
    EXPECT_STREQ("my_node", subnode->get_name());
    EXPECT_STREQ("/", subnode->get_namespace());
    EXPECT_STREQ("sub_ns", subnode->get_sub_namespace().c_str());
    EXPECT_STREQ("/sub_ns", subnode->get_effective_namespace().c_str());
    auto subnode2 = subnode->create_sub_node("sub_ns2");
    EXPECT_STREQ("my_node", subnode2->get_name());
    EXPECT_STREQ("/", subnode2->get_namespace());
    EXPECT_STREQ("sub_ns/sub_ns2", subnode2->get_sub_namespace().c_str());
    EXPECT_STREQ("/sub_ns/sub_ns2", subnode2->get_effective_namespace().c_str());
  }
  {
    auto node = std::make_shared<rclcpp::Node>("my_node");
    ASSERT_THROW({
      auto subnode = node->create_sub_node("/sub_ns");
    }, rclcpp::exceptions::NameValidationError);
  }
}
/*
   Testing node construction and destruction.
 */
TEST_F(TestNode, subnode_construction_and_destruction) {
  {
    ASSERT_NO_THROW({
      auto node = std::make_shared<rclcpp::Node>("my_node", "ns");
      auto subnode = node->create_sub_node("sub_ns");
    });
  }
  {
    ASSERT_THROW({
      auto node = std::make_shared<rclcpp::Node>("my_node", "ns");
      auto subnode = node->create_sub_node("invalid_ns?");
    }, rclcpp::exceptions::InvalidNamespaceError);
  }
  {
    ASSERT_THROW({
      auto node = std::make_shared<rclcpp::Node>("my_node", "ns/");
    }, rclcpp::exceptions::InvalidNamespaceError);
  }
  {
    ASSERT_THROW({
      auto node = std::make_shared<rclcpp::Node>("my_node", "ns/");
      auto subnode = node->create_sub_node("/sub_ns");
    }, rclcpp::exceptions::InvalidNamespaceError);
  }
  {
    ASSERT_THROW({
      auto node = std::make_shared<rclcpp::Node>("my_node", "ns");
      auto subnode = node->create_sub_node("/sub_ns");
    }, rclcpp::exceptions::NameValidationError);
  }
  {
    ASSERT_THROW({
      auto node = std::make_shared<rclcpp::Node>("my_node", "ns");
      auto subnode = node->create_sub_node("~sub_ns");
    }, rclcpp::exceptions::InvalidNamespaceError);
  }
  {
    ASSERT_THROW({
      auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
      auto subnode = node->create_sub_node("invalid_ns?");
    }, rclcpp::exceptions::InvalidNamespaceError);
  }
  {
    ASSERT_NO_THROW({
      auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
      auto subnode = node->create_sub_node("sub_ns");
    });
  }
  {
    ASSERT_THROW({
      auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
      auto subnode = node->create_sub_node("/sub_ns");
    }, rclcpp::exceptions::NameValidationError);
  }
  {
    ASSERT_THROW({
      auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
      auto subnode = node->create_sub_node("~sub_ns");
    }, rclcpp::exceptions::InvalidNamespaceError);
  }
}

TEST_F(TestNode, get_logger) {
  {
    auto node = std::make_shared<rclcpp::Node>("my_node");
    EXPECT_STREQ("my_node", node->get_logger().get_name());
  }
  {
    auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
    EXPECT_STREQ("ns.my_node", node->get_logger().get_name());
  }
  {
    auto node = std::make_shared<rclcpp::Node>("my_node", "ns");
    EXPECT_STREQ("ns.my_node", node->get_logger().get_name());
  }
  {
    auto node = std::make_shared<rclcpp::Node>("my_node", "/my/ns");
    EXPECT_STREQ("my.ns.my_node", node->get_logger().get_name());
  }
  {
    auto node = std::make_shared<rclcpp::Node>("my_node", "my/ns");
    EXPECT_STREQ("my.ns.my_node", node->get_logger().get_name());
  }
}

TEST_F(TestNode, get_clock) {
  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  auto ros_clock = node->get_clock();
  EXPECT_TRUE(ros_clock != nullptr);
  EXPECT_EQ(ros_clock->get_clock_type(), RCL_ROS_TIME);
}

TEST_F(TestNode, now) {
  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  auto clock = node->get_clock();
  auto now_builtin = node->now().nanoseconds();
  auto now_external = clock->now().nanoseconds();
  EXPECT_GE(now_external, now_builtin);
  EXPECT_LT(now_external - now_builtin, 5000000L);
}

std::string
operator"" _unq(const char * prefix, size_t prefix_length)
{
  static uint64_t count = 0;
  return std::string(prefix, prefix_length) + "_" + std::to_string(++count);
}

TEST_F(TestNode, declare_parameter_with_no_initial_values) {
  // test cases without initial values
  auto node = std::make_shared<rclcpp::Node>("test_declare_parameter_node"_unq);
  {
    // no default, no initial
    rclcpp::ParameterValue value = node->declare_parameter("parameter"_unq);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_NOT_SET);
  }
  {
    // int default, no initial
    rclcpp::ParameterValue default_value(42);
    rclcpp::ParameterValue value = node->declare_parameter("parameter"_unq, default_value);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_INTEGER);
    EXPECT_EQ(value.get<int>(), default_value.get<int>());
  }
  {
    // int default, no initial, custom parameter descriptor
    auto name = "parameter"_unq;
    rclcpp::ParameterValue default_value(42);
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    rclcpp::ParameterValue value =
      node->declare_parameter(name, default_value, descriptor);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_INTEGER);
    EXPECT_EQ(value.get<int>(), default_value.get<int>());
    rcl_interfaces::msg::ParameterDescriptor actual_descriptor =
      node->describe_parameter(name);
    EXPECT_EQ(actual_descriptor.read_only, descriptor.read_only);
  }
  {
    // int default, no initial, implicit template specialization
    int default_value = 42;
    EXPECT_EQ(node->declare_parameter("parameter"_unq, default_value), default_value);
  }
  {
    // parameter already declared throws
    auto name = "parameter"_unq;
    node->declare_parameter(name);
    EXPECT_THROW(
      {node->declare_parameter(name);},
      rclcpp::exceptions::ParameterAlreadyDeclaredException);
  }
  {
    // parameter name invalid throws
    EXPECT_THROW(
      {node->declare_parameter("");},
      rclcpp::exceptions::InvalidParametersException);
  }
  {
    // parameter rejected throws
    RCLCPP_SCOPE_EXIT({node->set_on_parameters_set_callback(nullptr);});    // always reset
    auto name = "parameter"_unq;
    auto on_set_parameters =
      [&name](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & parameter : parameters) {
          if (
            parameter.get_name() == name &&
            parameter.get_type() != rclcpp::PARAMETER_INTEGER)
          {
            result.successful = false;
            result.reason = "'" + name + "' must be an integer";
          }
        }
        return result;
      };
    EXPECT_EQ(node->set_on_parameters_set_callback(on_set_parameters), nullptr);
    EXPECT_THROW(
      {node->declare_parameter<std::string>(name, "not an int");},
      rclcpp::exceptions::InvalidParameterValueException);
  }
}

TEST_F(TestNode, declare_parameter_with_initial_values) {
  // test cases with initial values
  rclcpp::NodeOptions no;
  no.parameter_overrides({
    {"parameter_no_default", 42},
    {"parameter_no_default_set", 42},
    {"parameter_no_default_set_cvref", 42},
    {"parameter_and_default", 42},
    {"parameter_custom", 42},
    {"parameter_template", 42},
    {"parameter_already_declared", 42},
    {"parameter_rejected", 42},
    {"parameter_type_mismatch", "not an int"},
  });
  auto node = std::make_shared<rclcpp::Node>("test_declare_parameter_node"_unq, no);
  {
    // no default, with initial
    rclcpp::ParameterValue value = node->declare_parameter("parameter_no_default");
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_INTEGER);
    EXPECT_EQ(value.get<int>(), 42);
  }
  {
    // no default, with initial, and set after
    rclcpp::ParameterValue value = node->declare_parameter("parameter_no_default_set");
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_INTEGER);
    EXPECT_EQ(value.get<int>(), 42);
    // check that the value is changed after a set
    node->set_parameter({"parameter_no_default_set", 44});
    EXPECT_EQ(node->get_parameter("parameter_no_default_set").get_value<int>(), 44);
  }
  {
    // no default, with initial
    const rclcpp::ParameterValue & value =
      node->declare_parameter("parameter_no_default_set_cvref");
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_INTEGER);
    EXPECT_EQ(value.get<int>(), 42);
    // check that the value is changed after a set
    node->set_parameter({"parameter_no_default_set_cvref", 44});
    EXPECT_EQ(value.get<int>(), 44);
  }
  {
    // int default, with initial
    rclcpp::ParameterValue default_value(43);
    rclcpp::ParameterValue value = node->declare_parameter("parameter_and_default", default_value);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_INTEGER);
    EXPECT_EQ(value.get<int>(), 42);  // and not 43 which is the default value
  }
  {
    // int default, with initial, custom parameter descriptor
    rclcpp::ParameterValue default_value(43);
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    rclcpp::ParameterValue value =
      node->declare_parameter("parameter_custom", default_value, descriptor);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_INTEGER);
    EXPECT_EQ(value.get<int>(), 42);  // and not 43 which is the default value
    rcl_interfaces::msg::ParameterDescriptor actual_descriptor =
      node->describe_parameter("parameter_custom");
    EXPECT_EQ(actual_descriptor.read_only, descriptor.read_only);
  }
  {
    // int default, with initial, implicit template specialization
    int default_value = 43;
    // is equal to 42, not 43 which is the default value
    EXPECT_EQ(node->declare_parameter("parameter_template", default_value), 42);
  }
  {
    // parameter already declared throws
    auto name = "parameter_already_declared";
    node->declare_parameter(name);
    EXPECT_THROW(
      {node->declare_parameter(name);},
      rclcpp::exceptions::ParameterAlreadyDeclaredException);
  }
  {
    // parameter name invalid throws
    EXPECT_THROW(
      {node->declare_parameter("");},
      rclcpp::exceptions::InvalidParametersException);
  }
  {
    // parameter rejected throws, with initial value
    RCLCPP_SCOPE_EXIT({node->set_on_parameters_set_callback(nullptr);});    // always reset
    auto name = std::string("parameter_rejected");
    auto on_set_parameters =
      [&name](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & parameter : parameters) {
          if (
            parameter.get_name() == name &&
            parameter.get_type() == rclcpp::PARAMETER_INTEGER)
          {
            if (parameter.get_value<int>() < 43) {
              result.successful = false;
              result.reason = "'" + name + "' must be an integer and less than 43";
            }
          }
        }
        return result;
      };
    EXPECT_EQ(node->set_on_parameters_set_callback(on_set_parameters), nullptr);
    EXPECT_THROW(
      {node->declare_parameter<int>(name, 43);},
      rclcpp::exceptions::InvalidParameterValueException);
  }
  {
    // default type and initial value type do not match
    EXPECT_THROW(
      {node->declare_parameter("parameter_type_mismatch", 42);},
      rclcpp::ParameterTypeException);
  }
}

TEST_F(TestNode, declare_parameters_with_no_initial_values) {
  // test cases without initial values
  auto node = std::make_shared<rclcpp::Node>("test_declare_parameters_node"_unq);
  {
    // with namespace, defaults, no custom descriptors, no initial
    int64_t bigger_than_int = INT64_MAX - 42;
    auto values = node->declare_parameters<int64_t>("namespace1", {
      {"parameter_a", 42},
      {"parameter_b", 256},
      {"parameter_c", bigger_than_int},
    });
    std::vector<int64_t> expected = {42, 256, bigger_than_int};
    EXPECT_EQ(values, expected);
    EXPECT_TRUE(node->has_parameter("namespace1.parameter_a"));
    EXPECT_TRUE(node->has_parameter("namespace1.parameter_b"));
    EXPECT_FALSE(node->has_parameter("namespace1"));
  }
  {
    // without namespace, defaults, no custom descriptors, no initial
    auto values = node->declare_parameters<int64_t>("", {
      {"parameter_without_ns_a", 42},
      {"parameter_without_ns_b", 256},
    });
    std::vector<int64_t> expected = {42, 256};
    EXPECT_EQ(values, expected);
    EXPECT_TRUE(node->has_parameter("parameter_without_ns_a"));
    EXPECT_TRUE(node->has_parameter("parameter_without_ns_b"));
  }
  {
    // with namespace, defaults, custom descriptors, no initial
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    auto values = node->declare_parameters<int64_t>("namespace2", {
      {"parameter_a", {42, descriptor}},
      {"parameter_b", {256, descriptor}},
    });
    std::vector<int64_t> expected = {42, 256};
    EXPECT_EQ(values, expected);
    EXPECT_TRUE(node->has_parameter("namespace2.parameter_a"));
    EXPECT_TRUE(node->has_parameter("namespace2.parameter_b"));
    EXPECT_FALSE(node->has_parameter("namespace2"));
  }
  {
    // without namespace, defaults, custom descriptors, no initial
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    auto values = node->declare_parameters<int64_t>("", {
      {"parameter_without_ns_c", {42, descriptor}},
      {"parameter_without_ns_d", {256, descriptor}},
    });
    std::vector<int64_t> expected = {42, 256};
    EXPECT_EQ(values, expected);
    EXPECT_TRUE(node->has_parameter("parameter_without_ns_c"));
    EXPECT_TRUE(node->has_parameter("parameter_without_ns_d"));
  }
  {
    // empty parameters
    auto values = node->declare_parameters<int64_t>("", {});
    std::vector<int64_t> expected {};
    EXPECT_EQ(values, expected);
  }
  {
    // parameter already declared throws, even with not_set type
    auto name = "parameter"_unq;
    node->declare_parameter(name);
    EXPECT_THROW(
      {node->declare_parameters<int64_t>("", {{name, 42}});},
      rclcpp::exceptions::ParameterAlreadyDeclaredException);
  }
  {
    // parameter name invalid throws
    EXPECT_THROW(
      {node->declare_parameters<int64_t>("", {{"", 42}});},
      rclcpp::exceptions::InvalidParametersException);
  }
  {
    // parameter rejected throws
    RCLCPP_SCOPE_EXIT({node->set_on_parameters_set_callback(nullptr);});    // always reset
    auto name = "parameter"_unq;
    auto on_set_parameters =
      [&name](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & parameter : parameters) {
          if (
            parameter.get_name() == name &&
            parameter.get_type() != rclcpp::PARAMETER_INTEGER)
          {
            result.successful = false;
            result.reason = "'" + name + "' must be an integer";
          }
        }
        return result;
      };
    EXPECT_EQ(node->set_on_parameters_set_callback(on_set_parameters), nullptr);
    EXPECT_THROW(
      {node->declare_parameters<std::string>("", {{name, "not an int"}});},
      rclcpp::exceptions::InvalidParameterValueException);
  }
}

TEST_F(TestNode, undeclare_parameter) {
  auto node = std::make_shared<rclcpp::Node>("test_undeclare_parameter_node"_unq);
  {
    // normal use
    auto name = "parameter"_unq;
    node->declare_parameter(name);
    EXPECT_TRUE(node->has_parameter(name));
    node->undeclare_parameter(name);
    EXPECT_FALSE(node->has_parameter(name));
  }
  {
    // not declared throws
    auto name = "parameter"_unq;
    EXPECT_FALSE(node->has_parameter(name));
    EXPECT_THROW(
      {node->undeclare_parameter(name);},
      rclcpp::exceptions::ParameterNotDeclaredException);
  }
  {
    // read only parameter throws
    auto name = "parameter"_unq;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    node->declare_parameter(name, 42, descriptor);
    EXPECT_TRUE(node->has_parameter(name));
    EXPECT_THROW(
      {node->undeclare_parameter(name);},
      rclcpp::exceptions::ParameterImmutableException);
    EXPECT_TRUE(node->has_parameter(name));
  }
}

TEST_F(TestNode, has_parameter) {
  auto node = std::make_shared<rclcpp::Node>("test_has_parameter_node"_unq);
  {
    // normal use
    auto name = "parameter"_unq;
    EXPECT_FALSE(node->has_parameter(name));
    node->declare_parameter(name);
    EXPECT_TRUE(node->has_parameter(name));
    node->undeclare_parameter(name);
    EXPECT_FALSE(node->has_parameter(name));
  }
}

TEST_F(TestNode, set_parameter_undeclared_parameters_not_allowed) {
  auto node = std::make_shared<rclcpp::Node>(
    "test_set_parameter_node"_unq,
    rclcpp::NodeOptions().allow_undeclared_parameters(false));
  {
    // normal use
    auto name = "parameter"_unq;
    EXPECT_FALSE(node->has_parameter(name));
    node->declare_parameter(name, 42);
    EXPECT_TRUE(node->has_parameter(name));
    EXPECT_EQ(node->get_parameter(name).get_value<int>(), 42);
    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name, 43)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<int>(), 43);
  }
  {
    // normal use, change type
    auto name = "parameter"_unq;
    EXPECT_FALSE(node->has_parameter(name));
    node->declare_parameter(name, 42);
    EXPECT_TRUE(node->has_parameter(name));
    EXPECT_EQ(node->get_parameter(name).get_value<int>(), 42);
    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name, "not an integer")).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<std::string>(), std::string("not an integer"));
  }
  {
    // normal use, multiple parameters
    auto name1 = "parameter"_unq;
    auto name2 = "parameter"_unq;
    auto name3 = "parameter"_unq;
    node->declare_parameter(name1, 42);
    EXPECT_TRUE(node->has_parameter(name1));
    node->declare_parameter(name2, true);
    EXPECT_TRUE(node->has_parameter(name2));
    node->declare_parameter<std::string>(name3, "something");
    EXPECT_TRUE(node->has_parameter(name3));

    EXPECT_EQ(node->get_parameter(name1).get_value<int>(), 42);
    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name1, "not an integer")).successful);
    EXPECT_EQ(node->get_parameter(name1).get_value<std::string>(), std::string("not an integer"));

    EXPECT_EQ(node->get_parameter(name2).get_value<bool>(), true);
    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name2, false)).successful);
    EXPECT_EQ(node->get_parameter(name2).get_value<bool>(), false);
  }
  {
    // setting an undeclared parameter throws
    auto name = "parameter"_unq;
    EXPECT_FALSE(node->has_parameter(name));
    EXPECT_THROW(
      {node->set_parameter(rclcpp::Parameter(name, 42));},
      rclcpp::exceptions::ParameterNotDeclaredException);
    EXPECT_FALSE(node->has_parameter(name));
  }
  {
    // rejecting parameter does not throw, but fails
    auto name = "parameter"_unq;
    node->declare_parameter(name, 42);

    RCLCPP_SCOPE_EXIT({node->set_on_parameters_set_callback(nullptr);});    // always reset
    auto on_set_parameters =
      [](const std::vector<rclcpp::Parameter> &) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "no parameter may not be set right now";
        return result;
      };
    node->set_on_parameters_set_callback(on_set_parameters);

    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 43)).successful);
  }
  {
    // setting type of rclcpp::PARAMETER_NOT_SET, when already not set, does not undeclare
    auto name = "parameter"_unq;
    auto value = node->declare_parameter(name);
    EXPECT_TRUE(node->has_parameter(name));
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_NOT_SET);

    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name)).successful);

    EXPECT_TRUE(node->has_parameter(name));
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_NOT_SET);
  }
  {
    // setting type of rclcpp::PARAMETER_NOT_SET, when already to another type, will undeclare
    auto name = "parameter"_unq;
    node->declare_parameter(name, 42);
    EXPECT_TRUE(node->has_parameter(name));
    auto value = node->get_parameter(name);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_INTEGER);

    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name)).successful);

    EXPECT_FALSE(node->has_parameter(name));
  }
  {
    // setting a parameter with integer range descriptor
    auto name = "parameter"_unq;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.integer_range.resize(1);
    auto & integer_range = descriptor.integer_range.at(0);
    integer_range.from_value = 10;
    integer_range.to_value = 18;
    integer_range.step = 2;
    node->declare_parameter(name, 10, descriptor);
    EXPECT_TRUE(node->has_parameter(name));
    auto value = node->get_parameter(name);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_INTEGER);
    EXPECT_EQ(value.get_value<int64_t>(), 10);

    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name, 14)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<int64_t>(), 14);
    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name, 18)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<int64_t>(), 18);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 15)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<int64_t>(), 18);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 20)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<int64_t>(), 18);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 8)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<int64_t>(), 18);
  }
  {
    // setting a parameter with integer range descriptor, from_value > to_value
    auto name = "parameter"_unq;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.integer_range.resize(1);
    auto & integer_range = descriptor.integer_range.at(0);
    integer_range.from_value = 20;
    integer_range.to_value = 18;
    integer_range.step = 1;
    node->declare_parameter(name, 20, descriptor);
    EXPECT_TRUE(node->has_parameter(name));
    auto value = node->get_parameter(name);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_INTEGER);
    EXPECT_EQ(value.get_value<int64_t>(), 20);

    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name, 18)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<int64_t>(), 18);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 19)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<int64_t>(), 18);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 10)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<int64_t>(), 18);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 25)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<int64_t>(), 18);
  }
  {
    // setting a parameter with integer range descriptor, from_value = to_value
    auto name = "parameter"_unq;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.integer_range.resize(1);
    auto & integer_range = descriptor.integer_range.at(0);
    integer_range.from_value = 18;
    integer_range.to_value = 18;
    integer_range.step = 1;
    node->declare_parameter(name, 18, descriptor);
    EXPECT_TRUE(node->has_parameter(name));
    auto value = node->get_parameter(name);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_INTEGER);
    EXPECT_EQ(value.get_value<int64_t>(), 18);

    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 17)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<int64_t>(), 18);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 19)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<int64_t>(), 18);
  }
  {
    // setting a parameter with integer range descriptor, step > distance(from_value, to_value)
    auto name = "parameter"_unq;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.integer_range.resize(1);
    auto & integer_range = descriptor.integer_range.at(0);
    integer_range.from_value = 18;
    integer_range.to_value = 25;
    integer_range.step = 10;
    node->declare_parameter(name, 18, descriptor);
    EXPECT_TRUE(node->has_parameter(name));
    auto value = node->get_parameter(name);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_INTEGER);
    EXPECT_EQ(value.get_value<int64_t>(), 18);

    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name, 25)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<int64_t>(), 25);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 17)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<int64_t>(), 25);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 19)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<int64_t>(), 25);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 26)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<int64_t>(), 25);
  }
  {
    // setting a parameter with integer range descriptor, distance not multiple of the step.
    auto name = "parameter"_unq;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.integer_range.resize(1);
    auto & integer_range = descriptor.integer_range.at(0);
    integer_range.from_value = 18;
    integer_range.to_value = 28;
    integer_range.step = 7;
    node->declare_parameter(name, 18, descriptor);
    EXPECT_TRUE(node->has_parameter(name));
    auto value = node->get_parameter(name);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_INTEGER);
    EXPECT_EQ(value.get_value<int64_t>(), 18);

    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name, 28)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<int64_t>(), 28);
    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name, 25)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<int64_t>(), 25);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 17)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<int64_t>(), 25);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 19)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<int64_t>(), 25);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 32)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<int64_t>(), 25);
  }
  {
    // setting a parameter with integer range descriptor, step=0
    auto name = "parameter"_unq;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.integer_range.resize(1);
    auto & integer_range = descriptor.integer_range.at(0);
    integer_range.from_value = 10;
    integer_range.to_value = 18;
    integer_range.step = 0;
    node->declare_parameter(name, 10, descriptor);
    EXPECT_TRUE(node->has_parameter(name));
    auto value = node->get_parameter(name);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_INTEGER);
    EXPECT_EQ(value.get_value<int64_t>(), 10);

    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name, 11)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<int64_t>(), 11);
    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name, 15)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<int64_t>(), 15);
    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name, 18)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<int64_t>(), 18);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 9)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<int64_t>(), 18);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 19)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<int64_t>(), 18);
  }
  {
    // setting a parameter with integer range descriptor and wrong default value will throw
    auto name = "parameter"_unq;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.integer_range.resize(1);
    auto & integer_range = descriptor.integer_range.at(0);
    integer_range.from_value = 10;
    integer_range.to_value = 18;
    integer_range.step = 2;
    ASSERT_THROW(
      node->declare_parameter(name, 42, descriptor),
      rclcpp::exceptions::InvalidParameterValueException);
  }
  {
    // setting a parameter with floating point range descriptor
    auto name = "parameter"_unq;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.floating_point_range.resize(1);
    auto & floating_point_range = descriptor.floating_point_range.at(0);
    floating_point_range.from_value = 10.0;
    floating_point_range.to_value = 11.0;
    floating_point_range.step = 0.2;
    node->declare_parameter(name, 10.0, descriptor);
    EXPECT_TRUE(node->has_parameter(name));
    auto value = node->get_parameter(name);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_DOUBLE);
    EXPECT_EQ(value.get_value<double>(), 10.0);

    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name, 10.2)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<double>(), 10.2);
    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name, 11.0)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<double>(), 11.0);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 11.3)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<double>(), 11.0);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 12.0)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<double>(), 11.0);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 9.4)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<double>(), 11.0);
  }
  {
    // setting a parameter with floating point range descriptor, negative step
    auto name = "parameter"_unq;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.floating_point_range.resize(1);
    auto & floating_point_range = descriptor.floating_point_range.at(0);
    floating_point_range.from_value = 10.0;
    floating_point_range.to_value = 11.0;
    floating_point_range.step = -0.2;
    node->declare_parameter(name, 10.0, descriptor);
    EXPECT_TRUE(node->has_parameter(name));
    auto value = node->get_parameter(name);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_DOUBLE);
    EXPECT_EQ(value.get_value<double>(), 10.0);

    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name, 10.2)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<double>(), 10.2);
    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name, 11.0)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<double>(), 11.0);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 11.3)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<double>(), 11.0);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 12.0)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<double>(), 11.0);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 9.4)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<double>(), 11.0);
  }
  {
    // setting a parameter with floating point range descriptor, from_value > to_value
    auto name = "parameter"_unq;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.floating_point_range.resize(1);
    auto & floating_point_range = descriptor.floating_point_range.at(0);
    floating_point_range.from_value = 11.0;
    floating_point_range.to_value = 10.0;
    floating_point_range.step = 0.2;
    node->declare_parameter(name, 10.0, descriptor);
    EXPECT_TRUE(node->has_parameter(name));
    auto value = node->get_parameter(name);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_DOUBLE);
    EXPECT_EQ(value.get_value<double>(), 10.0);

    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name, 11.0)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<double>(), 11.0);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 11.2)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<double>(), 11.0);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 12.0)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<double>(), 11.0);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 9.4)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<double>(), 11.0);
  }
  {
    // setting a parameter with floating point range descriptor, from_value = to_value
    auto name = "parameter"_unq;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.floating_point_range.resize(1);
    auto & floating_point_range = descriptor.floating_point_range.at(0);
    floating_point_range.from_value = 10.0;
    floating_point_range.to_value = 10.0;
    floating_point_range.step = 0.2;
    node->declare_parameter(name, 10.0, descriptor);
    EXPECT_TRUE(node->has_parameter(name));
    auto value = node->get_parameter(name);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_DOUBLE);
    EXPECT_EQ(value.get_value<double>(), 10.0);

    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 11.2)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<double>(), 10.0);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 12.0)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<double>(), 10.0);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 9.4)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<double>(), 10.0);
  }
  {
    // setting a parameter with floating point range descriptor, step > distance
    auto name = "parameter"_unq;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.floating_point_range.resize(1);
    auto & floating_point_range = descriptor.floating_point_range.at(0);
    floating_point_range.from_value = 10.0;
    floating_point_range.to_value = 11.0;
    floating_point_range.step = 2.2;
    node->declare_parameter(name, 10.0, descriptor);
    EXPECT_TRUE(node->has_parameter(name));
    auto value = node->get_parameter(name);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_DOUBLE);
    EXPECT_EQ(value.get_value<double>(), 10.0);

    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name, 11.0)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<double>(), 11.0);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 12.2)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<double>(), 11.0);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 7.8)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<double>(), 11.0);
  }
  {
    // setting a parameter with floating point range descriptor, distance not multiple of the step.
    auto name = "parameter"_unq;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.floating_point_range.resize(1);
    auto & floating_point_range = descriptor.floating_point_range.at(0);
    floating_point_range.from_value = 10.0;
    floating_point_range.to_value = 11.0;
    floating_point_range.step = 0.7;
    node->declare_parameter(name, 10.0, descriptor);
    EXPECT_TRUE(node->has_parameter(name));
    auto value = node->get_parameter(name);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_DOUBLE);
    EXPECT_EQ(value.get_value<double>(), 10.0);

    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name, 11.0)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<double>(), 11.0);
    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name, 10.7)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<double>(), 10.7);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 11.4)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<double>(), 10.7);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 9.3)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<double>(), 10.7);
  }
  {
    // setting a parameter with floating point range descriptor, step=0
    auto name = "parameter"_unq;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.floating_point_range.resize(1);
    auto & floating_point_range = descriptor.floating_point_range.at(0);
    floating_point_range.from_value = 10.0;
    floating_point_range.to_value = 11.0;
    floating_point_range.step = 0.0;
    node->declare_parameter(name, 10.0, descriptor);
    EXPECT_TRUE(node->has_parameter(name));
    auto value = node->get_parameter(name);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_DOUBLE);
    EXPECT_EQ(value.get_value<double>(), 10.0);

    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name, 10.0001)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<double>(), 10.0001);
    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name, 10.5479051)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<double>(), 10.5479051);
    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name, 11.0)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<double>(), 11.0);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 11.001)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<double>(), 11.0);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 9.999)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<double>(), 11.0);
  }
  {
    // setting a parameter with a different type is still possible
    // when having a descriptor specifying a type (type is a status, not a constraint).
    auto name = "parameter"_unq;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rclcpp::PARAMETER_INTEGER;
    node->declare_parameter(name, 42, descriptor);
    EXPECT_TRUE(node->has_parameter(name));
    auto value = node->get_parameter(name);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_INTEGER);
    EXPECT_EQ(value.get_value<int64_t>(), 42);

    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name, "asd")).successful);
    EXPECT_TRUE(node->has_parameter(name));
    value = node->get_parameter(name);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_STRING);
    EXPECT_EQ(value.get_value<std::string>(), "asd");
  }
}

TEST_F(TestNode, set_parameter_undeclared_parameters_allowed) {
  rclcpp::NodeOptions no;
  no.parameter_overrides({
    {"parameter_with_override", 30},
  });
  no.allow_undeclared_parameters(true);
  auto node = std::make_shared<rclcpp::Node>("test_set_parameter_node"_unq, no);
  {
    // overrides are ignored when not declaring a parameter
    auto name = "parameter_with_override";
    EXPECT_FALSE(node->has_parameter(name));
    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name, 43)).successful);
    EXPECT_TRUE(node->has_parameter(name));
    EXPECT_EQ(node->get_parameter(name).get_value<int>(), 43);
  }
  {
    // normal use (declare first) still works with this true
    auto name = "parameter"_unq;
    EXPECT_FALSE(node->has_parameter(name));
    node->declare_parameter(name, 42);
    EXPECT_TRUE(node->has_parameter(name));
    EXPECT_EQ(node->get_parameter(name).get_value<int>(), 42);
    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name, 43)).successful);
    EXPECT_EQ(node->get_parameter(name).get_value<int>(), 43);
  }
  {
    // setting a parameter that is not declared implicitly declares it
    auto name = "parameter"_unq;
    EXPECT_FALSE(node->has_parameter(name));

    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name, 43)).successful);
    EXPECT_TRUE(node->has_parameter(name));
    EXPECT_EQ(node->get_parameter(name).get_value<int>(), 43);
  }
}

TEST_F(TestNode, set_parameters_undeclared_parameters_not_allowed) {
  auto node = std::make_shared<rclcpp::Node>(
    "test_set_parameters_node"_unq,
    rclcpp::NodeOptions().allow_undeclared_parameters(false));
  {
    // normal use
    auto name1 = "parameter"_unq;
    auto name2 = "parameter"_unq;
    auto name3 = "parameter"_unq;
    node->declare_parameter(name1, 1);
    node->declare_parameter(name2, true);
    node->declare_parameter<std::string>(name3, "blue");

    auto rets = node->set_parameters({
      {name1, 2},
      {name2, false},
      {name3, "red"},
    });
    EXPECT_TRUE(std::all_of(rets.begin(), rets.end(), [](auto & r) {return r.successful;}));
    EXPECT_TRUE(node->has_parameter(name1));
    EXPECT_TRUE(node->has_parameter(name2));
    EXPECT_TRUE(node->has_parameter(name3));
  }
  {
    // overwrite and order of setting
    auto name = "parameter"_unq;
    node->declare_parameter(name, 1);

    auto rets = node->set_parameters({
      {name, 42},
      {name, 2},
    });
    EXPECT_TRUE(std::all_of(rets.begin(), rets.end(), [](auto & r) {return r.successful;}));
    EXPECT_TRUE(node->has_parameter(name));
    EXPECT_EQ(node->get_parameter(name).get_value<int>(), 2);
  }
  {
    // undeclared parameter throws,
    // and preceding parameters are still set, but proceeding values are not
    auto name1 = "parameter"_unq;
    auto name2 = "parameter"_unq;
    auto name3 = "parameter"_unq;
    node->declare_parameter(name1, 1);
    node->declare_parameter(name3, 100);

    EXPECT_THROW(
    {
      node->set_parameters({
        {name1, 2},
        {name2, "not declared :("},
        {name3, 101},
      });
    },
      rclcpp::exceptions::ParameterNotDeclaredException);

    EXPECT_TRUE(node->has_parameter(name1));
    EXPECT_FALSE(node->has_parameter(name2));
    EXPECT_TRUE(node->has_parameter(name3));
    EXPECT_EQ(node->get_parameter(name1).get_value<int>(), 2);
    EXPECT_EQ(node->get_parameter(name3).get_value<int>(), 100);
  }
  {
    // rejecting parameter does not throw, but fails
    // all parameters to be set are attempted to be set
    auto name1 = "parameter"_unq;
    auto name2 = "parameter"_unq;
    auto name3 = "parameter"_unq;
    node->declare_parameter(name1, 1);
    node->declare_parameter(name2, true);
    node->declare_parameter<std::string>(name3, "blue");

    RCLCPP_SCOPE_EXIT({node->set_on_parameters_set_callback(nullptr);});    // always reset
    auto on_set_parameters =
      [&name2](const std::vector<rclcpp::Parameter> & ps) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        if (std::any_of(ps.begin(), ps.end(), [&](auto & p) {return p.get_name() == name2;})) {
          result.successful = false;
          result.reason = "parameter '" + name2 + "' may not be set right now";
        }
        return result;
      };
    node->set_on_parameters_set_callback(on_set_parameters);

    auto rets = node->set_parameters({
      {name1, 2},
      {name2, false},
      {name3, "red"},
    });
    EXPECT_EQ(rets.size(), 3U);
    EXPECT_TRUE(rets[0].successful);
    EXPECT_FALSE(rets[1].successful);
    EXPECT_NE(rets[1].reason.find("may not be set right now"), std::string::npos);
    EXPECT_TRUE(rets[2].successful);
    EXPECT_EQ(node->get_parameter(name1).get_value<int>(), 2);
    EXPECT_EQ(node->get_parameter(name2).get_value<bool>(), true);  // old value
    EXPECT_EQ(node->get_parameter(name3).get_value<std::string>(), "red");
  }
  {
    // setting type of rclcpp::PARAMETER_NOT_SET, when already not set, does not undeclare
    auto name = "parameter"_unq;
    auto value = node->declare_parameter(name);
    EXPECT_TRUE(node->has_parameter(name));
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_NOT_SET);

    EXPECT_TRUE(node->set_parameters({rclcpp::Parameter(name)})[0].successful);

    EXPECT_TRUE(node->has_parameter(name));
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_NOT_SET);
  }
  {
    // setting type of rclcpp::PARAMETER_NOT_SET, when already to another type, will undeclare
    auto name = "parameter"_unq;
    node->declare_parameter(name, 42);
    EXPECT_TRUE(node->has_parameter(name));
    auto value = node->get_parameter(name);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_INTEGER);

    EXPECT_TRUE(node->set_parameters({rclcpp::Parameter(name)})[0].successful);

    EXPECT_FALSE(node->has_parameter(name));
  }
}

// test set_parameters with undeclared allowed
TEST_F(TestNode, set_parameters_undeclared_parameters_allowed) {
  auto node = std::make_shared<rclcpp::Node>(
    "test_set_parameters_node"_unq,
    rclcpp::NodeOptions().allow_undeclared_parameters(true));
  {
    // normal use (declare first) still works with this true
    auto name1 = "parameter"_unq;
    auto name2 = "parameter"_unq;

    EXPECT_FALSE(node->has_parameter(name1));
    EXPECT_FALSE(node->has_parameter(name2));

    node->declare_parameter(name1, 42);
    EXPECT_TRUE(node->has_parameter(name1));
    EXPECT_EQ(node->get_parameter(name1).get_value<int>(), 42);
    node->declare_parameter<std::string>(name2, "test");
    EXPECT_TRUE(node->has_parameter(name2));
    EXPECT_EQ(node->get_parameter(name2).get_value<std::string>(), "test");

    auto rets = node->set_parameters({
      rclcpp::Parameter(name1, 43),
      rclcpp::Parameter(name2, "other"),
    });
    EXPECT_TRUE(std::all_of(rets.begin(), rets.end(), [](auto & r) {return r.successful;}));
    EXPECT_EQ(node->get_parameter(name1).get_value<int>(), 43);
    EXPECT_EQ(node->get_parameter(name2).get_value<std::string>(), "other");
  }
  {
    // setting a parameter that is not declared implicitly declares it
    auto name1 = "parameter"_unq;
    auto name2 = "parameter"_unq;

    EXPECT_FALSE(node->has_parameter(name1));
    EXPECT_FALSE(node->has_parameter(name2));

    auto rets = node->set_parameters({
      rclcpp::Parameter(name1, 42),
      rclcpp::Parameter(name2, "test"),
    });
    EXPECT_TRUE(std::all_of(rets.begin(), rets.end(), [](auto & r) {return r.successful;}));
    EXPECT_EQ(node->get_parameter(name1).get_value<int>(), 42);
    EXPECT_EQ(node->get_parameter(name2).get_value<std::string>(), "test");
  }
}

TEST_F(TestNode, set_parameters_atomically_undeclared_parameters_not_allowed) {
  auto node = std::make_shared<rclcpp::Node>(
    "test_set_parameters_atomically_node"_unq,
    rclcpp::NodeOptions().allow_undeclared_parameters(false));
  {
    // normal use
    auto name1 = "parameter"_unq;
    auto name2 = "parameter"_unq;
    auto name3 = "parameter"_unq;
    node->declare_parameter(name1, 1);
    node->declare_parameter(name2, true);
    node->declare_parameter<std::string>(name3, "blue");

    auto ret = node->set_parameters_atomically({
      {name1, 2},
      {name2, false},
      {name3, "red"},
    });
    EXPECT_TRUE(ret.successful);
    EXPECT_TRUE(node->has_parameter(name1));
    EXPECT_TRUE(node->has_parameter(name2));
    EXPECT_TRUE(node->has_parameter(name3));
  }
  {
    // overwrite and order of setting
    auto name = "parameter"_unq;
    node->declare_parameter(name, 1);

    auto ret = node->set_parameters_atomically({
      {name, 42},
      {name, 2},
    });
    EXPECT_TRUE(ret.successful);
    EXPECT_TRUE(node->has_parameter(name));
    EXPECT_EQ(node->get_parameter(name).get_value<int>(), 2);
  }
  {
    // undeclared parameter throws,
    // and no parameters were changed
    auto name1 = "parameter"_unq;
    auto name2 = "parameter"_unq;
    auto name3 = "parameter"_unq;
    node->declare_parameter(name1, 1);
    node->declare_parameter(name3, 100);

    EXPECT_THROW(
    {
      node->set_parameters_atomically({
        {name1, 2},
        {name2, "not declared :("},
        {name3, 101},
      });
    },
      rclcpp::exceptions::ParameterNotDeclaredException);

    EXPECT_TRUE(node->has_parameter(name1));
    EXPECT_FALSE(node->has_parameter(name2));
    EXPECT_TRUE(node->has_parameter(name3));
    // both have old values
    EXPECT_EQ(node->get_parameter(name1).get_value<int>(), 1);
    EXPECT_EQ(node->get_parameter(name3).get_value<int>(), 100);
  }
  {
    // rejecting parameter does not throw, but fails
    // and no parameters are changed
    auto name1 = "parameter"_unq;
    auto name2 = "parameter"_unq;
    auto name3 = "parameter"_unq;
    node->declare_parameter(name1, 1);
    node->declare_parameter(name2, true);
    node->declare_parameter<std::string>(name3, "blue");

    RCLCPP_SCOPE_EXIT({node->set_on_parameters_set_callback(nullptr);});    // always reset
    auto on_set_parameters =
      [&name2](const std::vector<rclcpp::Parameter> & ps) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        if (std::any_of(ps.begin(), ps.end(), [&](auto & p) {return p.get_name() == name2;})) {
          result.successful = false;
          result.reason = "parameter '" + name2 + "' may not be set right now";
        }
        return result;
      };
    node->set_on_parameters_set_callback(on_set_parameters);

    auto ret = node->set_parameters_atomically({
      {name1, 2},
      {name2, false},  // should fail to be set, failing the whole operation
      {name3, "red"},
    });
    EXPECT_FALSE(ret.successful);
    // all have old values
    EXPECT_EQ(node->get_parameter(name1).get_value<int>(), 1);
    EXPECT_EQ(node->get_parameter(name2).get_value<bool>(), true);
    EXPECT_EQ(node->get_parameter(name3).get_value<std::string>(), "blue");
  }
  {
    // setting type of rclcpp::PARAMETER_NOT_SET, when already not set, does not undeclare
    auto name = "parameter"_unq;
    auto value = node->declare_parameter(name);
    EXPECT_TRUE(node->has_parameter(name));
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_NOT_SET);

    EXPECT_TRUE(node->set_parameters_atomically({rclcpp::Parameter(name)}).successful);

    EXPECT_TRUE(node->has_parameter(name));
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_NOT_SET);
  }
  {
    // setting type of rclcpp::PARAMETER_NOT_SET, when already to another type, will undeclare
    auto name = "parameter"_unq;
    node->declare_parameter(name, 42);
    EXPECT_TRUE(node->has_parameter(name));
    auto value = node->get_parameter(name);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_INTEGER);

    EXPECT_TRUE(node->set_parameters_atomically({rclcpp::Parameter(name)}).successful);

    EXPECT_FALSE(node->has_parameter(name));
  }
}

// test set_parameters with undeclared allowed
TEST_F(TestNode, set_parameters_atomically_undeclared_parameters_allowed) {
  auto node = std::make_shared<rclcpp::Node>(
    "test_set_parameters_atomically_node"_unq,
    rclcpp::NodeOptions().allow_undeclared_parameters(true));
  {
    // normal use (declare first) still works with this true
    auto name1 = "parameter"_unq;
    auto name2 = "parameter"_unq;

    EXPECT_FALSE(node->has_parameter(name1));
    EXPECT_FALSE(node->has_parameter(name2));

    node->declare_parameter(name1, 42);
    EXPECT_TRUE(node->has_parameter(name1));
    EXPECT_EQ(node->get_parameter(name1).get_value<int>(), 42);
    node->declare_parameter<std::string>(name2, "test");
    EXPECT_TRUE(node->has_parameter(name2));
    EXPECT_EQ(node->get_parameter(name2).get_value<std::string>(), "test");

    auto ret = node->set_parameters_atomically({
      rclcpp::Parameter(name1, 43),
      rclcpp::Parameter(name2, "other"),
    });
    EXPECT_TRUE(ret.successful);
    EXPECT_EQ(node->get_parameter(name1).get_value<int>(), 43);
    EXPECT_EQ(node->get_parameter(name2).get_value<std::string>(), "other");
  }
  {
    // setting a parameter that is not declared implicitly declares it
    auto name1 = "parameter"_unq;
    auto name2 = "parameter"_unq;

    EXPECT_FALSE(node->has_parameter(name1));
    EXPECT_FALSE(node->has_parameter(name2));

    auto ret = node->set_parameters_atomically({
      rclcpp::Parameter(name1, 42),
      rclcpp::Parameter(name2, "test"),
    });
    EXPECT_TRUE(ret.successful);
    EXPECT_EQ(node->get_parameter(name1).get_value<int>(), 42);
    EXPECT_EQ(node->get_parameter(name2).get_value<std::string>(), "test");
  }
  {
    // if an undeclared parameter is implicitly declared, but a later parameter set fails,
    // then the implicitly "to be" declared parameter remains undeclared
    auto name1 = "parameter"_unq;
    auto name2 = "parameter"_unq;
    auto name3 = "parameter"_unq;

    node->declare_parameter(name1, 42);
    EXPECT_TRUE(node->has_parameter(name1));
    EXPECT_EQ(node->get_parameter(name1).get_value<int>(), 42);
    node->declare_parameter<std::string>(name3, "test");
    EXPECT_TRUE(node->has_parameter(name3));
    EXPECT_EQ(node->get_parameter(name3).get_value<std::string>(), "test");

    RCLCPP_SCOPE_EXIT({node->set_on_parameters_set_callback(nullptr);});    // always reset
    auto on_set_parameters =
      [&name3](const std::vector<rclcpp::Parameter> & ps) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        if (std::any_of(ps.begin(), ps.end(), [&](auto & p) {return p.get_name() == name3;})) {
          result.successful = false;
          result.reason = "parameter '" + name3 + "' may not be set right now";
        }
        return result;
      };
    node->set_on_parameters_set_callback(on_set_parameters);

    auto ret = node->set_parameters_atomically({
      rclcpp::Parameter(name1, 43),
      rclcpp::Parameter(name2, true),  // this would cause implicit declaration
      rclcpp::Parameter(name3, "other"),  // this set should fail, and fail the whole operation
    });
    EXPECT_FALSE(ret.successful);
    // name1 and name2 remain with the old values
    EXPECT_EQ(node->get_parameter(name1).get_value<int>(), 42);
    EXPECT_FALSE(node->has_parameter(name2));  // important! name2 remains undeclared
    EXPECT_EQ(node->get_parameter(name3).get_value<std::string>(), "test");
  }
}

// test get_parameter with undeclared not allowed
TEST_F(TestNode, get_parameter_undeclared_parameters_not_allowed) {
  auto node = std::make_shared<rclcpp::Node>(
    "test_get_parameter_node"_unq,
    rclcpp::NodeOptions().allow_undeclared_parameters(false));
  {
    // normal use
    auto name = "parameter"_unq;

    node->declare_parameter(name, 42);
    EXPECT_TRUE(node->has_parameter(name));

    // version that throws on undeclared
    EXPECT_EQ(node->get_parameter(name).get_value<int>(), 42);
    // version that returns bool and never throws, and stores in rclcpp::Parameter
    {
      rclcpp::Parameter parameter;
      EXPECT_TRUE(node->get_parameter(name, parameter));
      EXPECT_EQ(parameter.get_value<int>(), 42);
    }
    // version that returns bool and never throws, but is templated to store in a primitive type
    {
      int value;
      EXPECT_TRUE(node->get_parameter(name, value));
      EXPECT_EQ(value, 42);
    }
  }
  {
    // getting an undeclared parameter throws
    auto name = "parameter"_unq;
    EXPECT_FALSE(node->has_parameter(name));

    EXPECT_THROW({node->get_parameter(name);}, rclcpp::exceptions::ParameterNotDeclaredException);
    {
      rclcpp::Parameter parameter;
      EXPECT_FALSE(node->get_parameter(name, parameter));
    }
    {
      int value;
      EXPECT_FALSE(node->get_parameter(name, value));
    }
  }
  {
    // for templated version, throws if the parameter type doesn't match the requested type
    auto name = "parameter"_unq;

    node->declare_parameter<std::string>(name, "not an int");

    EXPECT_THROW(
    {
      int value;
      node->get_parameter(name, value);
    },
      rclcpp::ParameterTypeException);
  }
}

// test get_parameter with undeclared allowed
TEST_F(TestNode, get_parameter_undeclared_parameters_allowed) {
  auto node = std::make_shared<rclcpp::Node>(
    "test_get_parameter_node"_unq,
    rclcpp::NodeOptions().allow_undeclared_parameters(true));
  {
    // normal use (declare first) still works
    auto name = "parameter"_unq;

    node->declare_parameter(name, 42);
    EXPECT_TRUE(node->has_parameter(name));

    // version that throws on undeclared
    EXPECT_EQ(node->get_parameter(name).get_value<int>(), 42);
    // version that returns bool and never throws, and stores in rclcpp::Parameter
    {
      rclcpp::Parameter parameter;
      EXPECT_TRUE(node->get_parameter(name, parameter));
      EXPECT_EQ(parameter.get_value<int>(), 42);
    }
    // version that returns bool and never throws, but is templated to store in a primitive type
    {
      int value;
      EXPECT_TRUE(node->get_parameter(name, value));
      EXPECT_EQ(value, 42);
    }
  }
  {
    // getting an undeclared parameter returns default constructed rclcpp::Parameter or false
    auto name = "parameter"_unq;
    EXPECT_FALSE(node->has_parameter(name));

    EXPECT_EQ(node->get_parameter(name).get_type(), rclcpp::PARAMETER_NOT_SET);
    {
      rclcpp::Parameter parameter;
      EXPECT_FALSE(node->get_parameter(name, parameter));
    }
    {
      int value;
      EXPECT_FALSE(node->get_parameter(name, value));
    }
  }
  {
    // for templated version, return false if the parameter not declared
    auto name = "parameter"_unq;

    EXPECT_EQ(node->get_parameter(name).get_type(), rclcpp::PARAMETER_NOT_SET);
    int value = 42;
    EXPECT_FALSE(node->get_parameter<int>(name, value));
    EXPECT_EQ(value, 42);
  }
}

// test get_parameter_or with undeclared not allowed
TEST_F(TestNode, get_parameter_or_undeclared_parameters_not_allowed) {
  auto node = std::make_shared<rclcpp::Node>(
    "test_get_parameter_or_node"_unq,
    rclcpp::NodeOptions().allow_undeclared_parameters(false));
  {
    // normal use (declare first) still works
    auto name = "parameter"_unq;

    node->declare_parameter(name, 42);
    EXPECT_TRUE(node->has_parameter(name));

    {
      int value;
      EXPECT_TRUE(node->get_parameter_or(name, value, 43));
      EXPECT_EQ(value, 42);
    }
  }
  {
    // normal use, no declare first
    auto name = "parameter"_unq;

    {
      int value;
      EXPECT_FALSE(node->get_parameter_or(name, value, 43));
      EXPECT_EQ(value, 43);
    }
  }
}

// test get_parameter_or with undeclared allowed
TEST_F(TestNode, get_parameter_or_undeclared_parameters_allowed) {
  auto node = std::make_shared<rclcpp::Node>(
    "test_get_parameter_node"_unq,
    rclcpp::NodeOptions().allow_undeclared_parameters(true));
  {
    // normal use (declare first) still works
    auto name = "parameter"_unq;

    node->declare_parameter(name, 42);
    EXPECT_TRUE(node->has_parameter(name));

    {
      int value;
      EXPECT_TRUE(node->get_parameter_or(name, value, 43));
      EXPECT_EQ(value, 42);
    }
  }
  {
    // normal use, no declare first
    auto name = "parameter"_unq;

    {
      int value;
      EXPECT_FALSE(node->get_parameter_or(name, value, 43));
      EXPECT_EQ(value, 43);
    }
  }
}

// test get_parameters with undeclared not allowed
TEST_F(TestNode, get_parameters_undeclared_parameters_not_allowed) {
  auto node = std::make_shared<rclcpp::Node>(
    "test_get_parameters_node"_unq,
    rclcpp::NodeOptions().allow_undeclared_parameters(false));
  {
    // normal use
    auto base_name1 = "parameter"_unq;
    auto name1 = "ints." + base_name1;
    auto base_name2 = "parameter"_unq;
    auto name2 = "strings." + base_name2;
    auto base_name3 = "parameter"_unq;
    auto name3 = "ints." + base_name3;

    node->declare_parameter(name1, 42);
    node->declare_parameter<std::string>(name2, "test");
    node->declare_parameter(name3, 100);
    EXPECT_TRUE(node->has_parameter(name1));
    EXPECT_TRUE(node->has_parameter(name2));
    EXPECT_TRUE(node->has_parameter(name3));

    // non-templated version, get all
    {
      std::vector<rclcpp::Parameter> expected = {
        {name1, 42},
        {name2, "test"},
        {name3, 100},
      };
      EXPECT_EQ(node->get_parameters({name1, name2, name3}), expected);
    }
    // non-templated version, get some
    {
      std::vector<rclcpp::Parameter> expected = {
        {name1, 42},
        {name3, 100},
      };
      EXPECT_EQ(node->get_parameters({name1, name3}), expected);
    }
    // non-templated version, get some, different types
    {
      std::vector<rclcpp::Parameter> expected = {
        {name1, 42},
        {name2, "test"},
      };
      EXPECT_EQ(node->get_parameters({name1, name2}), expected);
    }
    // non-templated version, get some, wrong order (request order preserved)
    {
      std::vector<rclcpp::Parameter> expected = {
        {name3, 100},
        {name1, 42},
      };
      EXPECT_EQ(node->get_parameters({name3, name1}), expected);
    }
    // templated version, get all int's
    {
      std::map<std::string, int64_t> expected = {
        {base_name1, 42},
        {base_name3, 100},
      };
      std::map<std::string, int64_t> actual;
      EXPECT_TRUE(node->get_parameters("ints", actual));
      EXPECT_EQ(actual, expected);
    }
    // templated version, get the one string
    {
      std::map<std::string, std::string> expected = {
        {base_name2, "test"},
      };
      std::map<std::string, std::string> actual;
      EXPECT_TRUE(node->get_parameters("strings", actual));
      EXPECT_EQ(actual, expected);
    }
  }
  {
    // getting an undeclared parameter throws, or in the alternative signature returns false
    auto name = "prefix.parameter"_unq;
    EXPECT_FALSE(node->has_parameter(name));

    EXPECT_THROW(
      {node->get_parameters({name});},
      rclcpp::exceptions::ParameterNotDeclaredException);
    {
      std::map<std::string, int64_t> values;
      EXPECT_TRUE(values.empty());
      EXPECT_FALSE(node->get_parameters("prefix", values));
      EXPECT_TRUE(values.empty());
    }
  }
  {
    // templated version with empty prefix will get all parameters
    auto node_local = std::make_shared<rclcpp::Node>("test_get_parameters_node"_unq);
    auto name1 = "prefix1.parameter"_unq;
    auto name2 = "prefix2.parameter"_unq;

    node_local->declare_parameter(name1, 42);
    node_local->declare_parameter(name2, 100);
    // undeclare so that it doesn't interfere with the test
    node_local->undeclare_parameter("use_sim_time");

    {
      std::map<std::string, int64_t> actual;
      EXPECT_TRUE(node_local->get_parameters("", actual));
      EXPECT_NE(actual.find(name1), actual.end());
      EXPECT_NE(actual.find(name2), actual.end());
    }

    // will throw if set of parameters is non-homogeneous
    auto name3 = "prefix2.parameter"_unq;
    node_local->declare_parameter<std::string>(name3, "not an int");

    {
      std::map<std::string, int64_t> actual;
      EXPECT_THROW(
      {
        node_local->get_parameters("", actual);
      },
        rclcpp::ParameterTypeException);
    }
  }
}

// test get_parameters with undeclared allowed
TEST_F(TestNode, get_parameters_undeclared_parameters_allowed) {
  auto node = std::make_shared<rclcpp::Node>(
    "test_get_parameters_node"_unq,
    rclcpp::NodeOptions().allow_undeclared_parameters(true));
  {
    // normal use
    auto base_name1 = "parameter"_unq;
    auto name1 = "ints." + base_name1;
    auto base_name2 = "parameter"_unq;
    auto name2 = "strings." + base_name2;
    auto base_name3 = "parameter"_unq;
    auto name3 = "ints." + base_name3;

    EXPECT_FALSE(node->has_parameter(name1));
    EXPECT_FALSE(node->has_parameter(name2));
    EXPECT_FALSE(node->has_parameter(name3));

    {
      // non-templated version, get all, none set, no throw
      std::vector<rclcpp::Parameter> expected = {
        {name1, {}},
        {name2, {}},
        {name3, {}},
      };
      EXPECT_EQ(node->get_parameters({name1, name2, name3}), expected);
    }
    {
      // templated version, get all int's, none set, no throw
      std::map<std::string, int64_t> actual;
      EXPECT_FALSE(node->get_parameters("ints", actual));
      EXPECT_TRUE(actual.empty());
    }
    {
      // templated version, get the one string, none set, no throw
      std::map<std::string, std::string> actual;
      EXPECT_FALSE(node->get_parameters("strings", actual));
      EXPECT_TRUE(actual.empty());
    }
  }
}

// test describe parameter with undeclared not allowed
TEST_F(TestNode, describe_parameter_undeclared_parameters_not_allowed) {
  auto node = std::make_shared<rclcpp::Node>(
    "test_get_parameter_node"_unq,
    rclcpp::NodeOptions().allow_undeclared_parameters(false));
  {
    // normal use
    auto name1 = "parameter"_unq;
    auto name2 = "parameter"_unq;

    node->declare_parameter(name1, 42);
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    node->declare_parameter<std::string>(name2, "test", descriptor);

    {
      auto result = node->describe_parameter(name1);
      EXPECT_EQ(result.name, name1);
      EXPECT_EQ(result.type, rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER);
      EXPECT_FALSE(result.read_only);
    }
    {
      auto result = node->describe_parameter(name2);
      EXPECT_EQ(result.name, name2);
      EXPECT_EQ(result.type, rcl_interfaces::msg::ParameterType::PARAMETER_STRING);
      EXPECT_TRUE(result.read_only);
    }
  }
  {
    // non-existent parameter throws
    auto name = "parameter"_unq;

    {
      EXPECT_THROW({
        node->describe_parameter(name);
      }, rclcpp::exceptions::ParameterNotDeclaredException);
    }
  }
}

// test describe parameter with undeclared allowed
TEST_F(TestNode, describe_parameter_undeclared_parameters_allowed) {
  auto node = std::make_shared<rclcpp::Node>(
    "test_get_parameter_node"_unq,
    rclcpp::NodeOptions().allow_undeclared_parameters(true));
  {
    // normal use still works
    auto name1 = "parameter"_unq;
    auto name2 = "parameter"_unq;

    node->declare_parameter(name1, 42);
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    node->declare_parameter<std::string>(name2, "test", descriptor);

    {
      auto result = node->describe_parameter(name1);
      EXPECT_EQ(result.name, name1);
      EXPECT_EQ(result.type, rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER);
      EXPECT_FALSE(result.read_only);
    }
    {
      auto result = node->describe_parameter(name2);
      EXPECT_EQ(result.name, name2);
      EXPECT_EQ(result.type, rcl_interfaces::msg::ParameterType::PARAMETER_STRING);
      EXPECT_TRUE(result.read_only);
    }
  }
  {
    // non-existent parameter does not throw, but returns default constructed one
    auto name = "parameter"_unq;

    {
      auto result = node->describe_parameter(name);
      rcl_interfaces::msg::ParameterDescriptor expected;
      expected.name = name;
      EXPECT_EQ(result, expected);
    }
  }
}

// test describe parameters with undeclared not allowed
TEST_F(TestNode, describe_parameters_undeclared_parameters_not_allowed) {
  auto node = std::make_shared<rclcpp::Node>(
    "test_get_parameters_node"_unq,
    rclcpp::NodeOptions().allow_undeclared_parameters(false));
  {
    // normal use
    auto name1 = "parameter"_unq;
    auto name2 = "parameter"_unq;

    node->declare_parameter(name1, 42);
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    node->declare_parameter<std::string>(name2, "test", descriptor);

    auto results = node->describe_parameters({name1, name2});

    EXPECT_EQ(results.size(), 2u);

    EXPECT_EQ(results[0].name, name1);
    EXPECT_EQ(results[0].type, rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER);
    EXPECT_FALSE(results[0].read_only);

    EXPECT_EQ(results[1].name, name2);
    EXPECT_EQ(results[1].type, rcl_interfaces::msg::ParameterType::PARAMETER_STRING);
    EXPECT_TRUE(results[1].read_only);
  }
  {
    // non-existent parameter throws
    auto name = "parameter"_unq;

    {
      EXPECT_THROW({
        node->describe_parameters({name});
      }, rclcpp::exceptions::ParameterNotDeclaredException);
    }
  }
  {
    // non-existent parameter throws, even with existing parameters in the list requested
    auto name1 = "parameter"_unq;
    auto name2 = "parameter"_unq;

    node->declare_parameter(name1, 42);

    {
      EXPECT_THROW({
        node->describe_parameters({name1, name2});
      }, rclcpp::exceptions::ParameterNotDeclaredException);
    }
  }
  {
    // check that repeated names in input work, and that output is stable (same order as input)
    auto name1 = "parameter"_unq;
    auto name2 = "parameter"_unq;

    node->declare_parameter(name1, 42);
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    node->declare_parameter<std::string>(name2, "test", descriptor);

    auto results = node->describe_parameters({name2, name1, name2});

    EXPECT_EQ(results.size(), 3u);

    EXPECT_EQ(results[0].name, name2);
    EXPECT_EQ(results[0].type, rcl_interfaces::msg::ParameterType::PARAMETER_STRING);
    EXPECT_TRUE(results[0].read_only);

    EXPECT_EQ(results[1].name, name1);
    EXPECT_EQ(results[1].type, rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER);
    EXPECT_FALSE(results[1].read_only);

    EXPECT_EQ(results[2].name, name2);
    EXPECT_EQ(results[2].type, rcl_interfaces::msg::ParameterType::PARAMETER_STRING);
    EXPECT_TRUE(results[2].read_only);
  }
}

// test describe parameters with undeclared allowed
TEST_F(TestNode, describe_parameters_undeclared_parameters_allowed) {
  auto node = std::make_shared<rclcpp::Node>(
    "test_get_parameters_node"_unq,
    rclcpp::NodeOptions().allow_undeclared_parameters(true));
  {
    // normal use still works (declare first)
    auto name1 = "parameter"_unq;
    auto name2 = "parameter"_unq;

    node->declare_parameter(name1, 42);
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    node->declare_parameter<std::string>(name2, "test", descriptor);

    auto results = node->describe_parameters({name1, name2});

    EXPECT_EQ(results.size(), 2u);

    EXPECT_EQ(results[0].name, name1);
    EXPECT_EQ(results[0].type, rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER);
    EXPECT_FALSE(results[0].read_only);

    EXPECT_EQ(results[1].name, name2);
    EXPECT_EQ(results[1].type, rcl_interfaces::msg::ParameterType::PARAMETER_STRING);
    EXPECT_TRUE(results[1].read_only);
  }
  {
    // non-existent parameter does not throw
    auto name = "parameter"_unq;

    auto results = node->describe_parameters({name});

    EXPECT_EQ(results.size(), 1u);

    EXPECT_EQ(results[0].name, name);
    EXPECT_EQ(results[0].type, rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET);
    EXPECT_FALSE(results[0].read_only);
  }
  {
    // check that repeated names in input work, and that output is stable (same order as input)
    auto name1 = "parameter"_unq;
    auto name2 = "parameter"_unq;

    auto results = node->describe_parameters({name2, name1, name2});

    EXPECT_EQ(results.size(), 3u);

    EXPECT_EQ(results[0].name, name2);
    EXPECT_EQ(results[0].type, rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET);
    EXPECT_FALSE(results[0].read_only);

    EXPECT_EQ(results[1].name, name1);
    EXPECT_EQ(results[1].type, rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET);
    EXPECT_FALSE(results[1].read_only);

    EXPECT_EQ(results[2].name, name2);
    EXPECT_EQ(results[2].type, rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET);
    EXPECT_FALSE(results[2].read_only);
  }
}

// test get parameter types with undeclared not allowed
TEST_F(TestNode, get_parameter_types_undeclared_parameters_not_allowed) {
  auto node = std::make_shared<rclcpp::Node>(
    "test_get_parameter_types_node"_unq,
    rclcpp::NodeOptions().allow_undeclared_parameters(false));
  {
    // normal use
    auto name1 = "parameter"_unq;
    auto name2 = "parameter"_unq;

    node->declare_parameter(name1, 42);
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    node->declare_parameter<std::string>(name2, "test", descriptor);

    auto results = node->get_parameter_types({name1, name2});

    EXPECT_EQ(results.size(), 2u);

    EXPECT_EQ(results[0], rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER);

    EXPECT_EQ(results[1], rcl_interfaces::msg::ParameterType::PARAMETER_STRING);
  }
  {
    // non-existent parameter throws
    auto name = "parameter"_unq;

    {
      EXPECT_THROW({
        node->get_parameter_types({name});
      }, rclcpp::exceptions::ParameterNotDeclaredException);
    }
  }
  {
    // check that repeated names in input work, and that output is stable (same order as input)
    auto name1 = "parameter"_unq;
    auto name2 = "parameter"_unq;

    node->declare_parameter(name1, 42);
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    node->declare_parameter<std::string>(name2, "test", descriptor);

    auto results = node->get_parameter_types({name2, name1, name2});

    EXPECT_EQ(results.size(), 3u);

    EXPECT_EQ(results[0], rcl_interfaces::msg::ParameterType::PARAMETER_STRING);

    EXPECT_EQ(results[1], rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER);

    EXPECT_EQ(results[2], rcl_interfaces::msg::ParameterType::PARAMETER_STRING);
  }
}

// test get parameter types with undeclared allowed
TEST_F(TestNode, get_parameter_types_undeclared_parameters_allowed) {
  auto node = std::make_shared<rclcpp::Node>(
    "test_get_parameter_types_node"_unq,
    rclcpp::NodeOptions().allow_undeclared_parameters(true));
  {
    // normal use still works (declare first)
    auto name1 = "parameter"_unq;
    auto name2 = "parameter"_unq;

    node->declare_parameter(name1, 42);
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    node->declare_parameter<std::string>(name2, "test", descriptor);

    auto results = node->get_parameter_types({name1, name2});

    EXPECT_EQ(results.size(), 2u);

    EXPECT_EQ(results[0], rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER);

    EXPECT_EQ(results[1], rcl_interfaces::msg::ParameterType::PARAMETER_STRING);
  }
  {
    // non-existent parameter does not throw
    auto name = "parameter"_unq;

    auto results = node->get_parameter_types({name});

    EXPECT_EQ(results.size(), 1u);

    EXPECT_EQ(results[0], rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET);
  }
  {
    // check that repeated names in input work, and that output is stable (same order as input)
    auto name1 = "parameter"_unq;
    auto name2 = "parameter"_unq;

    auto results = node->get_parameter_types({name2, name1, name2});

    EXPECT_EQ(results.size(), 3u);

    EXPECT_EQ(results[0], rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET);

    EXPECT_EQ(results[1], rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET);

    EXPECT_EQ(results[2], rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET);
  }
}
