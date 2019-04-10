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

#include <string>
#include <memory>
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
  }
  {
    auto node = std::make_shared<rclcpp::Node>("my_node", "ns");
    EXPECT_STREQ("my_node", node->get_name());
    EXPECT_STREQ("/ns", node->get_namespace());
  }
  {
    auto node = std::make_shared<rclcpp::Node>("my_node", "/my/ns");
    EXPECT_STREQ("my_node", node->get_name());
    EXPECT_STREQ("/my/ns", node->get_namespace());
  }
  {
    auto node = std::make_shared<rclcpp::Node>("my_node", "my/ns");
    EXPECT_STREQ("my_node", node->get_name());
    EXPECT_STREQ("/my/ns", node->get_namespace());
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
operator"" _unq(const char * prefix, size_t prefix_length) {
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
    RCLCPP_SCOPE_EXIT({ node->set_on_parameters_set_callback(nullptr); });  // always reset
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
  no.initial_parameters({
    {"parameter_no_default", 42},
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
    RCLCPP_SCOPE_EXIT({ node->set_on_parameters_set_callback(nullptr); });  // always reset
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
    auto values = node->declare_parameters<int>("namespace1", {
      {"parameter_a", 42},
      {"parameter_b", 256},
    });
    std::vector<int> expected = {42, 256};
    EXPECT_EQ(values, expected);
    EXPECT_TRUE(node->has_parameter("namespace1.parameter_a"));
    EXPECT_TRUE(node->has_parameter("namespace1.parameter_b"));
    EXPECT_FALSE(node->has_parameter("namespace1"));
  }
  {
    // without namespace, defaults, no custom descriptors, no initial
    auto values = node->declare_parameters<int>("", {
      {"parameter_without_ns_a", 42},
      {"parameter_without_ns_b", 256},
    });
    std::vector<int> expected = {42, 256};
    EXPECT_EQ(values, expected);
    EXPECT_TRUE(node->has_parameter("parameter_without_ns_a"));
    EXPECT_TRUE(node->has_parameter("parameter_without_ns_b"));
  }
  {
    // with namespace, defaults, custom descriptors, no initial
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    auto values = node->declare_parameters<int>("namespace2", {
      {"parameter_a", {42, descriptor}},
      {"parameter_b", {256, descriptor}},
    });
    std::vector<int> expected = {42, 256};
    EXPECT_EQ(values, expected);
    EXPECT_TRUE(node->has_parameter("namespace2.parameter_a"));
    EXPECT_TRUE(node->has_parameter("namespace2.parameter_b"));
    EXPECT_FALSE(node->has_parameter("namespace2"));
  }
  {
    // without namespace, defaults, custom descriptors, no initial
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    auto values = node->declare_parameters<int>("", {
      {"parameter_without_ns_c", {42, descriptor}},
      {"parameter_without_ns_d", {256, descriptor}},
    });
    std::vector<int> expected = {42, 256};
    EXPECT_EQ(values, expected);
    EXPECT_TRUE(node->has_parameter("parameter_without_ns_c"));
    EXPECT_TRUE(node->has_parameter("parameter_without_ns_d"));
  }
  {
    // empty parameters
    auto values = node->declare_parameters<int>("", {});
    std::vector<int> expected {};
    EXPECT_EQ(values, expected);
  }
  {
    // parameter already declared throws, even with not_set type
    auto name = "parameter"_unq;
    node->declare_parameter(name);
    EXPECT_THROW(
      {node->declare_parameters<int>("", {{name, 42}});},
      rclcpp::exceptions::ParameterAlreadyDeclaredException);
  }
  {
    // parameter name invalid throws
    EXPECT_THROW(
      {node->declare_parameters<int>("", {{"", 42}});},
      rclcpp::exceptions::InvalidParametersException);
  }
  {
    // parameter rejected throws
    RCLCPP_SCOPE_EXIT({ node->set_on_parameters_set_callback(nullptr); });  // always reset
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

    RCLCPP_SCOPE_EXIT({ node->set_on_parameters_set_callback(nullptr); });  // always reset
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
}

TEST_F(TestNode, set_parameter_undeclared_parameters_allowed) {
  auto node = std::make_shared<rclcpp::Node>(
    "test_set_parameter_node"_unq,
    rclcpp::NodeOptions().allow_undeclared_parameters(true));
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

    RCLCPP_SCOPE_EXIT({ node->set_on_parameters_set_callback(nullptr); });  // always reset
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
