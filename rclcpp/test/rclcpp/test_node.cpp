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

#include <chrono>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rcpputils/filesystem_helper.hpp"
#include "rcpputils/scope_exit.hpp"

#include "rmw/validate_namespace.h"

#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/msg/empty.hpp"
#include "test_msgs/srv/empty.hpp"

#include "../mocking_utils/patch.hpp"

class TestNode : public ::testing::Test
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

  void SetUp() override
  {
    test_resources_path /= "test_node";
  }

  rcpputils::fs::path test_resources_path{TEST_RESOURCES_DIRECTORY};
};

/*
   Testing node construction and destruction.
 */
TEST_F(TestNode, construction_and_destruction) {
  {
    auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
    EXPECT_NE(nullptr, node->get_node_base_interface());
    EXPECT_NE(nullptr, node->get_node_clock_interface());
    EXPECT_NE(nullptr, node->get_node_graph_interface());
    EXPECT_NE(nullptr, node->get_node_logging_interface());
    EXPECT_NE(nullptr, node->get_node_time_source_interface());
    EXPECT_NE(nullptr, node->get_node_timers_interface());
    EXPECT_NE(nullptr, node->get_node_topics_interface());
    EXPECT_NE(nullptr, node->get_node_services_interface());
    EXPECT_NE(nullptr, node->get_node_parameters_interface());
    EXPECT_NE(nullptr, node->get_node_waitables_interface());
    EXPECT_NE(nullptr, node->get_node_options().get_rcl_node_options());
    EXPECT_NE(nullptr, node->get_graph_event());
    EXPECT_NE(nullptr, node->get_clock());
  }

  {
    ASSERT_THROW(
    {
      auto node = std::make_shared<rclcpp::Node>("invalid_node?", "/ns");
      (void)node;
    }, rclcpp::exceptions::InvalidNodeNameError);
  }

  {
    ASSERT_THROW(
    {
      auto node = std::make_shared<rclcpp::Node>("my_node", "/invalid_ns?");
      (void)node;
    }, rclcpp::exceptions::InvalidNamespaceError);
  }
}

/*
   Testing lifecycles of subscriptions and publishers after node dies
 */
TEST_F(TestNode, pub_and_sub_lifecycles) {
  using test_msgs::msg::Empty;
  rclcpp::Publisher<Empty>::SharedPtr pub;
  rclcpp::Subscription<Empty>::SharedPtr sub;
  const auto callback = [](Empty::ConstSharedPtr) {};

  {
    // Create the node and context in a nested scope so that their
    // std::shared_ptrs expire before we use pub and sub
    auto context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr);
    rclcpp::NodeOptions options;
    options.context(context);

    const auto node = std::make_shared<rclcpp::Node>("my_node", "/ns", options);
    pub = node->create_publisher<Empty>("topic", 10);
    sub = node->create_subscription<Empty>("topic", 10, callback);
  }

  pub->publish(Empty());
}

TEST_F(TestNode, get_name_and_namespace) {
  {
    auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
    EXPECT_STREQ("my_node", node->get_name());
    EXPECT_STREQ("/ns", node->get_namespace());
    EXPECT_STREQ("/ns", node->get_effective_namespace().c_str());
    EXPECT_STREQ("/ns/my_node", node->get_fully_qualified_name());
  }
  {
    auto options = rclcpp::NodeOptions()
      .arguments({"--ros-args", "-r", "__ns:=/another_ns"});
    auto node = std::make_shared<rclcpp::Node>("my_node", "/ns", options);
    EXPECT_STREQ("my_node", node->get_name());
    EXPECT_STREQ("/another_ns", node->get_namespace());
    EXPECT_STREQ("/another_ns/my_node", node->get_fully_qualified_name());
  }
  {
    auto node = std::make_shared<rclcpp::Node>("my_node", "ns");
    EXPECT_STREQ("my_node", node->get_name());
    EXPECT_STREQ("/ns", node->get_namespace());
    EXPECT_STREQ("/ns", node->get_effective_namespace().c_str());
    EXPECT_STREQ("/ns/my_node", node->get_fully_qualified_name());
  }
  {
    auto node = std::make_shared<rclcpp::Node>("my_node");
    EXPECT_STREQ("my_node", node->get_name());
    EXPECT_STREQ("/", node->get_namespace());
    EXPECT_STREQ("/", node->get_effective_namespace().c_str());
    EXPECT_STREQ("/my_node", node->get_fully_qualified_name());
  }
  {
    auto node = std::make_shared<rclcpp::Node>("my_node", "");
    EXPECT_STREQ("my_node", node->get_name());
    EXPECT_STREQ("/", node->get_namespace());
    EXPECT_STREQ("/", node->get_effective_namespace().c_str());
    EXPECT_STREQ("/my_node", node->get_fully_qualified_name());
  }
  {
    auto node = std::make_shared<rclcpp::Node>("my_node", "/my/ns");
    EXPECT_STREQ("my_node", node->get_name());
    EXPECT_STREQ("/my/ns", node->get_namespace());
    EXPECT_STREQ("/my/ns", node->get_effective_namespace().c_str());
    EXPECT_STREQ("/my/ns/my_node", node->get_fully_qualified_name());
  }
  {
    auto node = std::make_shared<rclcpp::Node>("my_node", "my/ns");
    EXPECT_STREQ("my_node", node->get_name());
    EXPECT_STREQ("/my/ns", node->get_namespace());
    EXPECT_STREQ("/my/ns", node->get_effective_namespace().c_str());
    EXPECT_STREQ("/my/ns/my_node", node->get_fully_qualified_name());
  }
  {
    auto node1 = std::make_shared<rclcpp::Node>("my_node1", "my/ns");
    auto node2 = std::make_shared<rclcpp::Node>("my_node2", "my/ns");
    auto node3 = std::make_shared<rclcpp::Node>("my_node3", "/ns2");
    auto node4 = std::make_shared<rclcpp::Node>("my_node4", "my/ns3");
    auto names_and_namespaces = node1->get_node_names();
    auto name_namespace_set = std::unordered_set<std::string>(
      names_and_namespaces.begin(),
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
    ASSERT_THROW(
    {
      auto subnode = node->create_sub_node("/sub_ns");
    }, rclcpp::exceptions::NameValidationError);
  }
}
/*
   Testing node construction and destruction.
 */
TEST_F(TestNode, subnode_construction_and_destruction) {
  {
    ASSERT_NO_THROW(
    {
      auto node = std::make_shared<rclcpp::Node>("my_node", "ns");
      auto subnode = node->create_sub_node("sub_ns");
    });
  }
  {
    ASSERT_THROW(
    {
      auto node = std::make_shared<rclcpp::Node>("my_node", "ns");
      auto subnode = node->create_sub_node("invalid_ns?");
    }, rclcpp::exceptions::InvalidNamespaceError);
  }
  {
    ASSERT_THROW(
    {
      auto node = std::make_shared<rclcpp::Node>("my_node", "ns/");
    }, rclcpp::exceptions::InvalidNamespaceError);
  }
  {
    ASSERT_THROW(
    {
      auto node = std::make_shared<rclcpp::Node>("my_node", "ns/");
      auto subnode = node->create_sub_node("/sub_ns");
    }, rclcpp::exceptions::InvalidNamespaceError);
  }
  {
    ASSERT_THROW(
    {
      auto node = std::make_shared<rclcpp::Node>("my_node", "ns");
      auto subnode = node->create_sub_node("/sub_ns");
    }, rclcpp::exceptions::NameValidationError);
  }
  {
    ASSERT_THROW(
    {
      auto node = std::make_shared<rclcpp::Node>("my_node", "ns");
      auto subnode = node->create_sub_node("~sub_ns");
    }, rclcpp::exceptions::InvalidNamespaceError);
  }
  {
    ASSERT_THROW(
    {
      auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
      auto subnode = node->create_sub_node("invalid_ns?");
    }, rclcpp::exceptions::InvalidNamespaceError);
  }
  {
    ASSERT_NO_THROW(
    {
      auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
      auto subnode = node->create_sub_node("sub_ns");
    });
  }
  {
    ASSERT_THROW(
    {
      auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
      auto subnode = node->create_sub_node("/sub_ns");
    }, rclcpp::exceptions::NameValidationError);
  }
  {
    ASSERT_THROW(
    {
      auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
      auto subnode = node->create_sub_node("~sub_ns");
    }, rclcpp::exceptions::InvalidNamespaceError);
  }
  {
    ASSERT_THROW(
    {
      auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
      auto subnode = node->create_sub_node("");
    }, rclcpp::exceptions::NameValidationError);
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
  EXPECT_NE(nullptr, ros_clock);
  EXPECT_EQ(ros_clock->get_clock_type(), RCL_ROS_TIME);

  const rclcpp::Node & const_node = *node.get();
  EXPECT_NE(nullptr, const_node.get_clock());
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
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;
    // no default, no initial
    const std::string parameter_name = "parameter"_unq;
    rclcpp::ParameterValue value = node->declare_parameter(
      parameter_name, rclcpp::ParameterValue{}, descriptor);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_NOT_SET);
    // Does not throw if unset before access
    EXPECT_EQ(
      rclcpp::PARAMETER_NOT_SET,
      node->get_parameter(parameter_name).get_parameter_value().get_type());
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
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;
    node->declare_parameter(name, rclcpp::ParameterValue{}, descriptor);
    EXPECT_THROW(
    {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.dynamic_typing = true;
      node->declare_parameter(name, rclcpp::ParameterValue{}, descriptor);
    },
      rclcpp::exceptions::ParameterAlreadyDeclaredException);
  }
  {
    // parameter name invalid throws
    EXPECT_THROW(
      {node->declare_parameter("", 5);},
      rclcpp::exceptions::InvalidParametersException);
  }
  {
    // parameter rejected throws
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
    auto handler = node->add_on_set_parameters_callback(on_set_parameters);
    RCPPUTILS_SCOPE_EXIT(
      {node->remove_on_set_parameters_callback(handler.get());});  // always reset
    EXPECT_THROW(
      {node->declare_parameter<std::string>(name, "not an int");},
      rclcpp::exceptions::InvalidParameterValueException);
  }
}

TEST_F(TestNode, declare_parameter_with_allow_undeclared_parameters) {
  // test cases without initial values
  auto node = std::make_shared<rclcpp::Node>(
    "test_declare_parameter_node"_unq, "/",
    rclcpp::NodeOptions{}.allow_undeclared_parameters(true));
  {
    // declared parameters static typing is still enforced
    auto param_name = "parameter"_unq;
    auto value = node->declare_parameter(param_name, 5);
    EXPECT_EQ(value, 5);
    EXPECT_FALSE(node->set_parameter({param_name, "asd"}).successful);
    auto param = node->get_parameter(param_name);
    EXPECT_EQ(param.get_type(), rclcpp::PARAMETER_INTEGER);
    EXPECT_EQ(param.get_value<int64_t>(), 5);
  }
  {
    // not for automatically declared parameters
    auto param_name = "parameter"_unq;
    EXPECT_TRUE(node->set_parameter({param_name, 5}).successful);
    auto param = node->get_parameter(param_name);
    EXPECT_EQ(param.get_type(), rclcpp::PARAMETER_INTEGER);
    EXPECT_EQ(param.get_value<int64_t>(), 5);
    EXPECT_TRUE(node->set_parameter({param_name, "asd"}).successful);
    param = node->get_parameter(param_name);
    EXPECT_EQ(param.get_type(), rclcpp::PARAMETER_STRING);
    EXPECT_EQ(param.get_value<std::string>(), "asd");
  }
  {
    // declare after set is invalid
    auto param_name = "parameter"_unq;
    EXPECT_TRUE(node->set_parameter({param_name, 5}).successful);
    auto param = node->get_parameter(param_name);
    EXPECT_EQ(param.get_type(), rclcpp::PARAMETER_INTEGER);
    EXPECT_EQ(param.get_value<int64_t>(), 5);
    EXPECT_THROW(
      node->declare_parameter(param_name, 5),
      rclcpp::exceptions::ParameterAlreadyDeclaredException);
  }
}

auto get_fixed_on_parameter_set_callback(const std::string & name, bool successful)
{
  return
    [name, successful](const std::vector<rclcpp::Parameter> & parameters) {
      (void)parameters;
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = successful;
      return result;
    };
}

TEST_F(TestNode, test_registering_multiple_callbacks_api) {
  auto node = std::make_shared<rclcpp::Node>("test_declare_parameter_node"_unq);
  {
    int64_t default_value{42};
    auto name1 = "parameter"_unq;
    auto scoped_callback1 = node->add_on_set_parameters_callback(
      get_fixed_on_parameter_set_callback(name1, true));
    EXPECT_NE(scoped_callback1, nullptr);
    int64_t value{node->declare_parameter(name1, default_value)};
    EXPECT_EQ(value, default_value);

    auto name2 = "parameter"_unq;
    auto scoped_callback2 = node->add_on_set_parameters_callback(
      get_fixed_on_parameter_set_callback(name2, false));
    EXPECT_NE(scoped_callback2, nullptr);
    EXPECT_THROW(
      {node->declare_parameter(name2, default_value);},
      rclcpp::exceptions::InvalidParameterValueException);

    auto name3 = "parameter"_unq;
    scoped_callback2.reset();
    value = node->declare_parameter(name3, default_value);
    EXPECT_EQ(value, default_value);
  }
  {
    int64_t default_value{42};
    auto name1 = "parameter"_unq;
    auto scoped_callback1 = node->add_on_set_parameters_callback(
      get_fixed_on_parameter_set_callback(name1, true));
    EXPECT_NE(scoped_callback1, nullptr);
    int64_t value{node->declare_parameter(name1, default_value)};
    EXPECT_EQ(value, default_value);

    auto name2 = "parameter"_unq;
    auto scoped_callback2 = node->add_on_set_parameters_callback(
      get_fixed_on_parameter_set_callback(name2, false));
    EXPECT_NE(scoped_callback2, nullptr);
    EXPECT_THROW(
      {node->declare_parameter(name2, default_value);},
      rclcpp::exceptions::InvalidParameterValueException);

    auto name3 = "parameter"_unq;
    node->remove_on_set_parameters_callback(scoped_callback2.get());
    value = node->declare_parameter(name3, default_value);
    EXPECT_EQ(value, default_value);
  }
  {
    int64_t default_value{42};
    auto name1 = "parameter"_unq;
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr scoped_callback(
      node->add_on_set_parameters_callback(
        get_fixed_on_parameter_set_callback(name1, false)));
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr scoped_callback_copy(scoped_callback);
    scoped_callback.reset();

    EXPECT_THROW(
      {node->declare_parameter("parameter"_unq, default_value);},
      rclcpp::exceptions::InvalidParameterValueException);

    scoped_callback_copy.reset();
    // All the shared_ptr has been reset
    int64_t value = node->declare_parameter("parameter"_unq, default_value);
    EXPECT_EQ(value, default_value);
  }
}

TEST_F(TestNode, declare_parameter_with_overrides) {
  // test cases with overrides
  rclcpp::NodeOptions no;
  no.parameter_overrides(
  {
    {"parameter_no_default", 42},
    {"parameter_no_default_set", 42},
    {"parameter_no_default_set_cvref", 42},
    {"parameter_and_default", 42},
    {"parameter_and_default_ignore_override", 42},
    {"parameter_custom", 42},
    {"parameter_template", 42},
    {"parameter_already_declared", 42},
    {"parameter_rejected", 42},
    {"parameter_type_mismatch", "not an int"},
  });

  auto node = std::make_shared<rclcpp::Node>("test_declare_parameter_node"_unq, no);
  {
    // no default, with override
    rclcpp::ParameterValue value = node->declare_parameter(
      "parameter_no_default", rclcpp::ParameterType::PARAMETER_INTEGER);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_INTEGER);
    EXPECT_EQ(value.get<int>(), 42);
  }
  {
    // no default, with override, and set after
    rclcpp::ParameterValue value = node->declare_parameter(
      "parameter_no_default_set", rclcpp::ParameterType::PARAMETER_INTEGER);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_INTEGER);
    EXPECT_EQ(value.get<int>(), 42);
    // check that the value is changed after a set
    node->set_parameter({"parameter_no_default_set", 44});
    EXPECT_EQ(node->get_parameter("parameter_no_default_set").get_value<int>(), 44);
  }
  {
    // no default, with override
    const rclcpp::ParameterValue & value = node->declare_parameter(
      "parameter_no_default_set_cvref", rclcpp::ParameterType::PARAMETER_INTEGER);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_INTEGER);
    EXPECT_EQ(value.get<int>(), 42);
    // check that the value is changed after a set
    node->set_parameter({"parameter_no_default_set_cvref", 44});
    EXPECT_EQ(value.get<int>(), 44);
  }
  {
    // int default, with override
    rclcpp::ParameterValue default_value(43);
    rclcpp::ParameterValue value = node->declare_parameter("parameter_and_default", default_value);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_INTEGER);
    EXPECT_EQ(value.get<int>(), 42);  // and not 43 which is the default value
  }
  {
    // int default, with override and ignoring it
    rclcpp::ParameterValue default_value(43);
    rclcpp::ParameterValue value = node->declare_parameter(
      "parameter_and_default_ignore_override",
      default_value,
      rcl_interfaces::msg::ParameterDescriptor(),
      true);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_INTEGER);
    EXPECT_EQ(value.get<int>(), 43);  // and not 42, the parameter override is ignored.
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
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;
    node->declare_parameter(name, rclcpp::ParameterValue{}, descriptor);
    EXPECT_THROW(
    {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.dynamic_typing = true;
      node->declare_parameter(name, rclcpp::ParameterValue{}, descriptor);
    },
      rclcpp::exceptions::ParameterAlreadyDeclaredException);
  }
  {
    // parameter name invalid throws
    EXPECT_THROW(
      {node->declare_parameter("", 5);},
      rclcpp::exceptions::InvalidParametersException);
  }
  {
    // parameter rejected throws, with initial value
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
    auto handler = node->add_on_set_parameters_callback(on_set_parameters);
    RCPPUTILS_SCOPE_EXIT(
      {node->remove_on_set_parameters_callback(handler.get());});  // always reset
    EXPECT_THROW(
      {node->declare_parameter<int>(name, 43);},
      rclcpp::exceptions::InvalidParameterValueException);
  }
  {
    // default type and initial value type do not match
    EXPECT_THROW(
      {node->declare_parameter("parameter_type_mismatch", 42);},
      rclcpp::exceptions::InvalidParameterTypeException);
  }
  {
    // default type and expected type do not match
    EXPECT_THROW(
      {node->declare_parameter(
          "parameter_type_mismatch", rclcpp::ParameterType::PARAMETER_INTEGER);},
      rclcpp::exceptions::InvalidParameterTypeException);
  }
  {
    // statically typed parameter must be initialized
    EXPECT_THROW(
      {node->declare_parameter<std::string>("static_and_uninitialized");},
      rclcpp::exceptions::UninitializedStaticallyTypedParameterException);
  }
  {
    // cannot pass an expected type and a descriptor with dynamic_typing=True
    rcl_interfaces::msg::ParameterDescriptor descriptor{};
    descriptor.dynamic_typing = true;
    EXPECT_THROW(
      {node->declare_parameter(
          "invalid_argument", rclcpp::ParameterType::PARAMETER_INTEGER, descriptor);},
      std::invalid_argument);
  }
}

TEST_F(TestNode, declare_parameters_with_no_initial_values) {
  // test cases without initial values
  auto node = std::make_shared<rclcpp::Node>("test_declare_parameters_node"_unq);
  {
    // with namespace, defaults, no custom descriptors, no initial
    int64_t bigger_than_int = INT64_MAX - 42;
    auto values = node->declare_parameters<int64_t>(
      "namespace1", {
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
    auto values = node->declare_parameters<int64_t>(
      "", {
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
    auto values = node->declare_parameters<int64_t>(
      "namespace2", {
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
    auto values = node->declare_parameters<int64_t>(
      "", {
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
    node->declare_parameter(name, 42);
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
    auto handler = node->add_on_set_parameters_callback(on_set_parameters);
    RCPPUTILS_SCOPE_EXIT(
      {node->remove_on_set_parameters_callback(handler.get());});  // always reset
    EXPECT_THROW(
      {node->declare_parameters<std::string>("", {{name, "not an int"}});},
      rclcpp::exceptions::InvalidParameterValueException);
  }
}

TEST_F(TestNode, declare_parameter_with_cli_overrides) {
  const std::string parameters_filepath = (
    test_resources_path / "test_parameters.yaml").string();
  // test cases with overrides
  rclcpp::NodeOptions no;
  no.arguments(
  {
    "--ros-args",
    "-p", "parameter_bool:=false",
    "-p", "parameter_int:=42",
    "-p", "parameter_double:=0.42",
    "-p", "parameter_string:=foo",
    "--params-file", parameters_filepath.c_str(),
    "-p", "parameter_bool_array:=[false, true]",
    "-p", "parameter_int_array:=[-21, 42]",
    "-p", "parameter_double_array:=[-1.0, .42]",
    "-p", "parameter_string_array:=[foo, bar]"
  });

  // To match parameters YAML file content, use a well-known node name for this test only.
  auto node = std::make_shared<rclcpp::Node>("test_declare_parameter_node", no);
  {
    rclcpp::ParameterValue value = node->declare_parameter(
      "parameter_bool", rclcpp::ParameterType::PARAMETER_BOOL);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_BOOL);
    EXPECT_EQ(value.get<bool>(), true);
  }
  {
    rclcpp::ParameterValue value = node->declare_parameter(
      "parameter_int", rclcpp::ParameterType::PARAMETER_INTEGER);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_INTEGER);
    EXPECT_EQ(value.get<int64_t>(), 21);  // set to 42 in CLI, overriden by file
  }
  {
    rclcpp::ParameterValue value = node->declare_parameter(
      "parameter_double", rclcpp::ParameterType::PARAMETER_DOUBLE);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_DOUBLE);
    EXPECT_EQ(value.get<double>(), 0.42);
  }
  {
    rclcpp::ParameterValue value = node->declare_parameter(
      "parameter_string", rclcpp::ParameterType::PARAMETER_STRING);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_STRING);
    EXPECT_EQ(value.get<std::string>(), "foo");
  }
  {
    rclcpp::ParameterValue value = node->declare_parameter(
      "parameter_bool_array", rclcpp::ParameterType::PARAMETER_BOOL_ARRAY);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_BOOL_ARRAY);
    std::vector<bool> expected_value{false, true};
    EXPECT_EQ(value.get<std::vector<bool>>(), expected_value);
  }
  {
    rclcpp::ParameterValue value = node->declare_parameter(
      "parameter_int_array", rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_INTEGER_ARRAY);
    std::vector<int64_t> expected_value{-21, 42};
    EXPECT_EQ(value.get<std::vector<int64_t>>(), expected_value);
  }
  {
    rclcpp::ParameterValue value = node->declare_parameter(
      "parameter_double_array", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_DOUBLE_ARRAY);
    std::vector<double> expected_value{-1.0, 0.42};
    EXPECT_EQ(value.get<std::vector<double>>(), expected_value);
  }
  {
    rclcpp::ParameterValue value = node->declare_parameter(
      "parameter_string_array", rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_STRING_ARRAY);
    std::vector<std::string> expected_value{"foo", "bar"};
    // set to [baz, baz, baz] in file, overriden by CLI
    EXPECT_EQ(value.get<std::vector<std::string>>(), expected_value);
  }
}

TEST_F(TestNode, undeclare_parameter) {
  auto node = std::make_shared<rclcpp::Node>("test_undeclare_parameter_node"_unq);
  {
    // normal use
    auto name = "parameter"_unq;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;
    node->declare_parameter(name, rclcpp::ParameterValue{}, descriptor);
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
    // statically typed parameter throws
    auto name = "parameter"_unq;
    node->declare_parameter(name, 42);
    EXPECT_TRUE(node->has_parameter(name));
    EXPECT_THROW(
      {node->undeclare_parameter(name);},
      rclcpp::exceptions::InvalidParameterTypeException);
    EXPECT_TRUE(node->has_parameter(name));
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
  // normal use
  auto name = "parameter"_unq;
  EXPECT_FALSE(node->has_parameter(name));
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.dynamic_typing = true;
  node->declare_parameter(name, rclcpp::ParameterValue{}, descriptor);
  EXPECT_TRUE(node->has_parameter(name));
  node->undeclare_parameter(name);
  EXPECT_FALSE(node->has_parameter(name));
}

TEST_F(TestNode, list_parameters) {
  auto node = std::make_shared<rclcpp::Node>("test_list_parameter_node"_unq);
  // normal use
  auto name = "parameter"_unq;
  const size_t before_size = node->list_parameters({}, 1u).names.size();
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.dynamic_typing = true;
  node->declare_parameter(name, rclcpp::ParameterValue{}, descriptor);
  EXPECT_EQ(1u + before_size, node->list_parameters({}, 1u).names.size());
  node->undeclare_parameter(name);
  EXPECT_EQ(before_size, node->list_parameters({}, 1u).names.size());
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
    // normal use, change type not allowed
    auto name = "parameter"_unq;
    EXPECT_FALSE(node->has_parameter(name));
    node->declare_parameter(name, 42);
    EXPECT_TRUE(node->has_parameter(name));
    EXPECT_EQ(node->get_parameter(name).get_value<int>(), 42);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, "not an integer")).successful);
  }
  {
    // normal use, change type
    auto name = "parameter"_unq;
    EXPECT_FALSE(node->has_parameter(name));
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;
    node->declare_parameter(name, 42, descriptor);
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

    auto on_set_parameters =
      [](const std::vector<rclcpp::Parameter> &) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "no parameter may not be set right now";
        return result;
      };
    auto handler = node->add_on_set_parameters_callback(on_set_parameters);
    RCPPUTILS_SCOPE_EXIT(
      {node->remove_on_set_parameters_callback(handler.get());});  // always reset

    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 43)).successful);
  }
  {
    // setting type of rclcpp::PARAMETER_NOT_SET, when already not set, does not undeclare
    auto name = "parameter"_unq;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;
    auto value = node->declare_parameter(name, rclcpp::ParameterValue{}, descriptor);
    EXPECT_TRUE(node->has_parameter(name));
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_NOT_SET);

    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name)).successful);

    EXPECT_TRUE(node->has_parameter(name));
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_NOT_SET);
  }
  {
    // setting type of rclcpp::PARAMETER_NOT_SET, when already to another type, will fail
    auto name = "parameter"_unq;
    node->declare_parameter(name, 42);
    EXPECT_TRUE(node->has_parameter(name));
    auto value = node->get_parameter(name);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_INTEGER);

    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name)).successful);
  }
  {
    // setting type of rclcpp::PARAMETER_NOT_SET,
    // when dynamic typing is allowing and already set to another type, will undeclare
    auto name = "parameter"_unq;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;
    node->declare_parameter(name, 42, descriptor);
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
    node->declare_parameter(name, "asd", descriptor);
    EXPECT_TRUE(node->has_parameter(name));
    auto value = node->get_parameter(name);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_STRING);
    EXPECT_EQ(value.get_value<std::string>(), "asd");
  }
}

TEST_F(TestNode, set_parameter_undeclared_parameters_allowed) {
  rclcpp::NodeOptions no;
  no.parameter_overrides(
  {
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

    auto rets = node->set_parameters(
    {
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

    auto rets = node->set_parameters(
    {
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
      node->set_parameters(
      {
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
    auto handler = node->add_on_set_parameters_callback(on_set_parameters);
    RCPPUTILS_SCOPE_EXIT(
      {node->remove_on_set_parameters_callback(handler.get());});  // always reset

    auto rets = node->set_parameters(
    {
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
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;
    auto value = node->declare_parameter(name, rclcpp::ParameterValue{}, descriptor);
    EXPECT_TRUE(node->has_parameter(name));
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_NOT_SET);

    EXPECT_TRUE(node->set_parameters({rclcpp::Parameter(name)})[0].successful);

    EXPECT_TRUE(node->has_parameter(name));
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_NOT_SET);
  }
  {
    // setting type of rclcpp::PARAMETER_NOT_SET, when already to another type, will fail
    auto name = "parameter"_unq;
    node->declare_parameter(name, 42);
    EXPECT_TRUE(node->has_parameter(name));
    auto value = node->get_parameter(name);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_INTEGER);

    EXPECT_FALSE(node->set_parameters({rclcpp::Parameter(name)})[0].successful);
  }
  {
    // setting type of rclcpp::PARAMETER_NOT_SET,
    // when already to another type and dynamic typic allowed, will undeclare
    auto name = "parameter"_unq;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;
    node->declare_parameter(name, 42, descriptor);
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

    auto rets = node->set_parameters(
    {
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

    auto rets = node->set_parameters(
    {
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

    auto ret = node->set_parameters_atomically(
    {
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

    auto ret = node->set_parameters_atomically(
    {
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
      node->set_parameters_atomically(
      {
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
    auto handler = node->add_on_set_parameters_callback(on_set_parameters);
    RCPPUTILS_SCOPE_EXIT(
      {node->remove_on_set_parameters_callback(handler.get());});  // always reset

    auto ret = node->set_parameters_atomically(
    {
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
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;
    auto value = node->declare_parameter(name, rclcpp::ParameterValue{}, descriptor);
    EXPECT_TRUE(node->has_parameter(name));
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_NOT_SET);

    EXPECT_TRUE(node->set_parameters_atomically({rclcpp::Parameter(name)}).successful);

    EXPECT_TRUE(node->has_parameter(name));
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_NOT_SET);
  }
  {
    // setting type of rclcpp::PARAMETER_NOT_SET, when already to another type, will fail
    auto name = "parameter"_unq;
    node->declare_parameter(name, 42);
    EXPECT_TRUE(node->has_parameter(name));
    auto value = node->get_parameter(name);
    EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_INTEGER);

    EXPECT_FALSE(node->set_parameters_atomically({rclcpp::Parameter(name)}).successful);
  }
  {
    // setting type of rclcpp::PARAMETER_NOT_SET,
    // when dynamic typing is allowed and already declared to another type, will undeclare
    auto name = "parameter"_unq;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;
    node->declare_parameter(name, 42, descriptor);
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

    auto ret = node->set_parameters_atomically(
    {
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

    auto ret = node->set_parameters_atomically(
    {
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
    auto handler = node->add_on_set_parameters_callback(on_set_parameters);
    RCPPUTILS_SCOPE_EXIT(
      {node->remove_on_set_parameters_callback(handler.get());});  // always reset

    auto ret = node->set_parameters_atomically(
    {
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
      rclcpp::exceptions::InvalidParameterTypeException);
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

// test get_parameter_or with return value
TEST_F(TestNode, get_parameter_or_with_return_value) {
  auto node = std::make_shared<rclcpp::Node>(
    "test_get_parameter_or_node"_unq);
  {
    // normal use (declare first) still works
    auto name = "parameter"_unq;

    node->declare_parameter(name, 42);
    EXPECT_TRUE(node->has_parameter(name));

    {
      const int value = node->get_parameter_or(name, 43);
      EXPECT_EQ(value, 42);
    }
  }
  {
    // normal use, no declare first
    auto name = "parameter"_unq;

    {
      const int value = node->get_parameter_or(name, 43);
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
    auto name1 = "parameter"_unq;
    auto name2 = "parameter"_unq;

    node_local->declare_parameter("prefix." + name1, 42);
    node_local->declare_parameter("prefix." + name2, 100);

    {
      std::map<std::string, int64_t> actual;
      EXPECT_TRUE(node_local->get_parameters("prefix", actual));
      EXPECT_NE(actual.find(name1), actual.end());
      EXPECT_NE(actual.find(name2), actual.end());
    }

    // will throw if set of parameters is non-homogeneous
    auto name3 = "prefix1.parameter"_unq;
    node_local->declare_parameter<std::string>("prefix." + name3, "not an int");

    {
      std::map<std::string, int64_t> actual;
      EXPECT_THROW(
      {
        node_local->get_parameters("prefix", actual);
      },
        rclcpp::exceptions::InvalidParameterTypeException);
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
      EXPECT_THROW(
      {
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
      EXPECT_THROW(
      {
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
      EXPECT_THROW(
      {
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
      EXPECT_THROW(
      {
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

// test declare parameter with int, int64_t, float and double vector
TEST_F(TestNode, declare_parameter_with_vector) {
  auto node = std::make_shared<rclcpp::Node>(
    "test_declare_parameter_with_vector"_unq,
    rclcpp::NodeOptions().allow_undeclared_parameters(true));
  {
    // declare parameter and then get types to check
    auto name1 = "parameter"_unq;
    auto name2 = "parameter"_unq;
    auto name3 = "parameter"_unq;
    auto name4 = "parameter"_unq;

    node->declare_parameter(name1, std::vector<int>{});
    node->declare_parameter(name2, std::vector<int64_t>{});
    node->declare_parameter(name3, std::vector<float>{});
    node->declare_parameter(name4, std::vector<double>{});

    EXPECT_TRUE(node->has_parameter(name1));
    EXPECT_TRUE(node->has_parameter(name2));
    EXPECT_TRUE(node->has_parameter(name3));
    EXPECT_TRUE(node->has_parameter(name4));

    auto results = node->get_parameter_types({name1, name2, name3, name4});
    EXPECT_EQ(results.size(), 4u);
    EXPECT_EQ(results[0], rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY);
    EXPECT_EQ(results[1], rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY);
    EXPECT_EQ(results[2], rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY);
    EXPECT_EQ(results[3], rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY);
  }
  {
    // declare parameter and then get values to check
    auto name1 = "parameter"_unq;
    auto name2 = "parameter"_unq;
    auto name3 = "parameter"_unq;
    auto name4 = "parameter"_unq;

    int64_t bigger_than_int = INT64_MAX - 42;
    double bigger_than_float = std::numeric_limits<double>::max() - 42;
    node->declare_parameter(name1, std::vector<int>{1, 2});
    node->declare_parameter(name2, std::vector<int64_t>{3, bigger_than_int});
    node->declare_parameter(name3, std::vector<float>{1.5f, 2.8f});
    node->declare_parameter(name4, std::vector<double>{3.0, bigger_than_float});

    std::vector<rclcpp::Parameter> expected = {
      {name1, std::vector<int>{1, 2}},
      {name2, std::vector<int64_t>{3, bigger_than_int}},
      {name3, std::vector<float>{1.5f, 2.8f}},
      {name4, std::vector<double>{3.0, bigger_than_float}},
    };
    EXPECT_EQ(node->get_parameters({name1, name2, name3, name4}), expected);
  }
}

// test non-array data types for declare parameter function templates that are explicitly defined
TEST_F(TestNode, declare_parameter_allowed_simple_types_function_templates) {
  auto node = std::make_shared<rclcpp::Node>(
    "test_declare_parameter_allowed_simple_types_function_templates"_unq);
  {
    // declare parameter and then get types to check
    auto name1 = "parameter"_unq;
    auto name2 = "parameter"_unq;
    auto name3 = "parameter"_unq;
    auto name4 = "parameter"_unq;
    auto name5 = "parameter"_unq;
    auto name6 = "parameter"_unq;
    auto name7 = "parameter"_unq;

    node->declare_parameter<bool>(name1, false);
    node->declare_parameter<int>(name2, 1234);
    node->declare_parameter<int64_t>(name3, 12340);
    node->declare_parameter<float>(name4, static_cast<float>(12.34));
    node->declare_parameter<double>(name5, 12.34);  // called float64 in ros2 design parameters page
    node->declare_parameter<std::string>(name6, "test string");
    auto str = "test param";
    node->declare_parameter<const char *>(name7, str);

    EXPECT_TRUE(node->has_parameter(name1));
    EXPECT_TRUE(node->has_parameter(name2));
    EXPECT_TRUE(node->has_parameter(name3));
    EXPECT_TRUE(node->has_parameter(name4));
    EXPECT_TRUE(node->has_parameter(name5));
    EXPECT_TRUE(node->has_parameter(name6));
    EXPECT_TRUE(node->has_parameter(name7));

    auto results = node->get_parameter_types({name1, name2, name3, name4, name5, name6, name7});
    EXPECT_EQ(results.size(), 7u);
    EXPECT_EQ(results[0], rcl_interfaces::msg::ParameterType::PARAMETER_BOOL);
    EXPECT_EQ(results[1], rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER);
    EXPECT_EQ(results[2], rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER);
    EXPECT_EQ(results[3], rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);
    EXPECT_EQ(results[4], rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);
    EXPECT_EQ(results[5], rcl_interfaces::msg::ParameterType::PARAMETER_STRING);
    EXPECT_EQ(results[6], rcl_interfaces::msg::ParameterType::PARAMETER_STRING);
  }
  {
    // declare parameter and then get values to check
    auto name1 = "parameter"_unq;
    auto name2 = "parameter"_unq;
    auto name3 = "parameter"_unq;
    auto name4 = "parameter"_unq;
    auto name5 = "parameter"_unq;
    auto name6 = "parameter"_unq;
    auto name7 = "parameter"_unq;

    node->declare_parameter<bool>(name1, false);
    node->declare_parameter<int>(name2, 4321);
    node->declare_parameter<int64_t>(name3, 43210);
    node->declare_parameter<float>(name4, static_cast<float>(43.21));
    node->declare_parameter<double>(name5, 12.34);  // called float64 in ros2 design parameters page
    node->declare_parameter<std::string>(name6, "test string");
    auto str = "test param";
    node->declare_parameter<const char *>(name7, str);

    std::vector<rclcpp::Parameter> expected = {
      {name1, false},
      {name2, 4321},
      {name3, 43210},
      {name4, static_cast<float>(43.21)},
      {name5, 12.34},
      {name6, "test string"},
      {name7, str}
    };
    EXPECT_EQ(node->get_parameters({name1, name2, name3, name4, name5, name6, name7}), expected);
  }
}

// test array data types for declare parameter function templates that are explicitly defined
TEST_F(TestNode, declare_parameter_allowed_array_types_function_templates) {
  auto node = std::make_shared<rclcpp::Node>(
    "test_declare_parameter_allowed_array_types_function_templates"_unq);
  {
    // declare parameter and then get types to check
    auto name1 = "parameter"_unq;
    auto name2 = "parameter"_unq;
    auto name3 = "parameter"_unq;
    auto name4 = "parameter"_unq;
    auto name5 = "parameter"_unq;
    auto name6 = "parameter"_unq;
    auto name7 = "parameter"_unq;

    node->declare_parameter<std::vector<uint8_t>>(name1, std::vector<uint8_t>{3, 4, 5, 7, 9});
    node->declare_parameter<std::vector<bool>>(name2, std::vector<bool>{false, true});
    node->declare_parameter<std::vector<int>>(name3, std::vector<int>{1234, 2345});
    node->declare_parameter<std::vector<int64_t>>(name4, std::vector<int64_t>{12340, 9876});
    node->declare_parameter<std::vector<float>>(
      name5, std::vector<float>{static_cast<float>(12.34),
        static_cast<float>(98.78)});
    node->declare_parameter<std::vector<double>>(
      name6,
      std::vector<double>{12.34, 55.66});  // called float64 in ros2 design parameters page
    node->declare_parameter<std::vector<std::string>>(
      name7, std::vector<std::string>{"test string",
        "another test str"});

    EXPECT_TRUE(node->has_parameter(name1));
    EXPECT_TRUE(node->has_parameter(name2));
    EXPECT_TRUE(node->has_parameter(name3));
    EXPECT_TRUE(node->has_parameter(name4));
    EXPECT_TRUE(node->has_parameter(name5));
    EXPECT_TRUE(node->has_parameter(name6));
    EXPECT_TRUE(node->has_parameter(name7));

    auto results = node->get_parameter_types({name1, name2, name3, name4, name5, name6, name7});
    EXPECT_EQ(results.size(), 7u);
    EXPECT_EQ(results[0], rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY);
    EXPECT_EQ(results[1], rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY);
    EXPECT_EQ(results[2], rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY);
    EXPECT_EQ(results[3], rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY);
    EXPECT_EQ(results[4], rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY);
    EXPECT_EQ(results[5], rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY);
    EXPECT_EQ(results[6], rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY);
  }
  {
    // declare parameter and then get values to check
    auto name1 = "parameter"_unq;
    auto name2 = "parameter"_unq;
    auto name3 = "parameter"_unq;
    auto name4 = "parameter"_unq;
    auto name5 = "parameter"_unq;
    auto name6 = "parameter"_unq;
    auto name7 = "parameter"_unq;

    std::vector<uint8_t> byte_arr = {0xD, 0xE, 0xA, 0xD};
    node->declare_parameter<std::vector<uint8_t>>(name1, byte_arr);
    node->declare_parameter<std::vector<bool>>(name2, std::vector<bool>{true, false, true});
    node->declare_parameter<std::vector<int>>(name3, std::vector<int>{22, 33, 55, 77});
    node->declare_parameter<std::vector<int64_t>>(name4, std::vector<int64_t>{456, 765});
    node->declare_parameter<std::vector<float>>(
      name5, std::vector<float>{static_cast<float>(99.11),
        static_cast<float>(11.99)});
    node->declare_parameter<std::vector<double>>(
      name6,
      std::vector<double>{12.21, 55.55, 98.89});  // called float64 in ros2 design parameters page
    node->declare_parameter<std::vector<std::string>>(
      name7, std::vector<std::string>{"ros2",
        "colcon", "ignition"});

    std::vector<rclcpp::Parameter> expected = {
      {name1, std::vector<uint8_t>{0xD, 0xE, 0xA, 0xD}},
      {name2, std::vector<bool>{true, false, true}},
      {name3, std::vector<int>{22, 33, 55, 77}},
      {name4, std::vector<int64_t>{456, 765}},
      {name5, std::vector<float>{static_cast<float>(99.11), static_cast<float>(11.99)}},
      {name6, std::vector<double>{12.21, 55.55, 98.89}},
      {name7, std::vector<std::string>{"ros2", "colcon", "ignition"}}
    };
    EXPECT_EQ(node->get_parameters({name1, name2, name3, name4, name5, name6, name7}), expected);
  }
}

void expect_qos_profile_eq(
  const rmw_qos_profile_t & qos1, const rmw_qos_profile_t & qos2, bool is_publisher)
{
  // Depth and history are skipped because they are not retrieved.
  EXPECT_EQ(qos1.reliability, qos2.reliability);
  EXPECT_EQ(qos1.durability, qos2.durability);
  EXPECT_EQ(qos1.deadline.sec, qos2.deadline.sec);
  EXPECT_EQ(qos1.deadline.nsec, qos2.deadline.nsec);
  if (is_publisher) {
    EXPECT_EQ(qos1.lifespan.sec, qos2.lifespan.sec);
    EXPECT_EQ(qos1.lifespan.nsec, qos2.lifespan.nsec);
  }
  EXPECT_EQ(qos1.liveliness, qos2.liveliness);
  EXPECT_EQ(qos1.liveliness_lease_duration.sec, qos2.liveliness_lease_duration.sec);
  EXPECT_EQ(qos1.liveliness_lease_duration.nsec, qos2.liveliness_lease_duration.nsec);
}

namespace
{

constexpr std::chrono::nanoseconds DEFAULT_EVENT_TIMEOUT = std::chrono::seconds(3);

constexpr std::chrono::nanoseconds DEFAULT_EVENT_SLEEP_PERIOD = std::chrono::milliseconds(100);

bool wait_for_event(
  std::shared_ptr<rclcpp::Node> node,
  std::function<bool()> predicate,
  std::chrono::nanoseconds timeout = DEFAULT_EVENT_TIMEOUT,
  std::chrono::nanoseconds sleep_period = DEFAULT_EVENT_SLEEP_PERIOD)
{
  auto start = std::chrono::steady_clock::now();
  std::chrono::nanoseconds time_slept(0);

  bool predicate_result;
  while (!(predicate_result = predicate()) && time_slept < timeout) {
    rclcpp::Event::SharedPtr graph_event = node->get_graph_event();
    node->wait_for_graph_change(graph_event, sleep_period);
    time_slept = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - start);
  }
  return predicate_result;
}

}  // namespace

// test that calling get_publishers_info_by_topic and get_subscriptions_info_by_topic
TEST_F(TestNode, get_publishers_subscriptions_info_by_topic) {
  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  std::string topic_name = "test_topic_info";
  std::string fq_topic_name = rclcpp::expand_topic_or_service_name(
    topic_name, node->get_name(), node->get_namespace());

  // Lists should be empty
  EXPECT_TRUE(node->get_publishers_info_by_topic(fq_topic_name).empty());
  EXPECT_TRUE(node->get_subscriptions_info_by_topic(fq_topic_name).empty());

  // Add a publisher
  rclcpp::QoSInitialization qos_initialization =
  {
    RMW_QOS_POLICY_HISTORY_KEEP_ALL,
    10
  };
  rmw_qos_profile_t rmw_qos_profile_default =
  {
    RMW_QOS_POLICY_HISTORY_KEEP_ALL,
    10,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
    {1, 12345},
    {20, 9887665},
    RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC,
    {5, 23456},
    false
  };
  rclcpp::QoS qos = rclcpp::QoS(qos_initialization, rmw_qos_profile_default);
  auto publisher = node->create_publisher<test_msgs::msg::BasicTypes>(topic_name, qos);
  // Wait for the underlying RMW implementation to catch up with graph changes
  auto topic_is_published =
    [&]() {return node->get_publishers_info_by_topic(fq_topic_name).size() > 0u;};
  ASSERT_TRUE(wait_for_event(node, topic_is_published));
  // List should have one item
  auto publisher_list = node->get_publishers_info_by_topic(fq_topic_name);
  ASSERT_EQ(publisher_list.size(), (size_t)1);
  // Subscription list should be empty
  EXPECT_TRUE(node->get_subscriptions_info_by_topic(fq_topic_name).empty());
  // Verify publisher list has the right data.
  EXPECT_EQ(node->get_name(), publisher_list[0].node_name());
  EXPECT_EQ(node->get_namespace(), publisher_list[0].node_namespace());
  EXPECT_EQ("test_msgs/msg/BasicTypes", publisher_list[0].topic_type());
  EXPECT_EQ(rclcpp::EndpointType::Publisher, publisher_list[0].endpoint_type());
  auto actual_qos_profile = publisher_list[0].qos_profile().get_rmw_qos_profile();
  {
    SCOPED_TRACE("Publisher QOS 1");
    expect_qos_profile_eq(qos.get_rmw_qos_profile(), actual_qos_profile, true);
  }

  // Add a subscription
  rclcpp::QoSInitialization qos_initialization2 =
  {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    0
  };
  rmw_qos_profile_t rmw_qos_profile_default2 =
  {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    0,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    {15, 1678},
    {29, 2345},
    RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
    {5, 23456},
    false
  };
  rclcpp::QoS qos2 = rclcpp::QoS(qos_initialization2, rmw_qos_profile_default2);
  auto callback = [](test_msgs::msg::BasicTypes::ConstSharedPtr msg) {
      (void)msg;
    };
  auto subscriber =
    node->create_subscription<test_msgs::msg::BasicTypes>(topic_name, qos2, callback);
  // Wait for the underlying RMW implementation to catch up with graph changes
  auto topic_is_subscribed =
    [&]() {return node->get_subscriptions_info_by_topic(fq_topic_name).size() > 0u;};
  ASSERT_TRUE(wait_for_event(node, topic_is_subscribed));
  // Both lists should have one item
  publisher_list = node->get_publishers_info_by_topic(fq_topic_name);
  auto subscription_list = node->get_subscriptions_info_by_topic(fq_topic_name);
  EXPECT_EQ(publisher_list.size(), (size_t)1);
  EXPECT_EQ(subscription_list.size(), (size_t)1);

  // Verify publisher and subscription list has the right data.
  EXPECT_EQ(node->get_name(), publisher_list[0].node_name());
  EXPECT_EQ(node->get_namespace(), publisher_list[0].node_namespace());
  EXPECT_EQ("test_msgs/msg/BasicTypes", publisher_list[0].topic_type());
  EXPECT_EQ(rclcpp::EndpointType::Publisher, publisher_list[0].endpoint_type());
  auto publisher_qos_profile = publisher_list[0].qos_profile().get_rmw_qos_profile();
  {
    SCOPED_TRACE("Publisher QOS 2");
    expect_qos_profile_eq(qos.get_rmw_qos_profile(), publisher_qos_profile, true);
  }

  EXPECT_EQ(node->get_name(), subscription_list[0].node_name());
  EXPECT_EQ(node->get_namespace(), subscription_list[0].node_namespace());
  EXPECT_EQ("test_msgs/msg/BasicTypes", subscription_list[0].topic_type());
  EXPECT_EQ(rclcpp::EndpointType::Subscription, subscription_list[0].endpoint_type());
  auto subscription_qos_profile = subscription_list[0].qos_profile().get_rmw_qos_profile();
  {
    SCOPED_TRACE("Subscription QOS");
    expect_qos_profile_eq(qos2.get_rmw_qos_profile(), subscription_qos_profile, false);
  }

  // Error cases
  EXPECT_THROW(
  {
    publisher_list = node->get_publishers_info_by_topic("13");
  }, rclcpp::exceptions::InvalidTopicNameError);
  EXPECT_THROW(
  {
    subscription_list = node->get_subscriptions_info_by_topic("13");
  }, rclcpp::exceptions::InvalidTopicNameError);
}

TEST_F(TestNode, callback_groups) {
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  size_t num_callback_groups_in_basic_node = 0;
  node->for_each_callback_group(
    [&num_callback_groups_in_basic_node](rclcpp::CallbackGroup::SharedPtr group)
    {
      (void)group;
      num_callback_groups_in_basic_node++;
    });

  auto group1 = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  size_t num_callback_groups = 0;
  node->for_each_callback_group(
    [&num_callback_groups](rclcpp::CallbackGroup::SharedPtr group)
    {
      (void)group;
      num_callback_groups++;
    });
  EXPECT_EQ(1u + num_callback_groups_in_basic_node, num_callback_groups);

  auto group2 = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  size_t num_callback_groups2 = 0;
  node->for_each_callback_group(
    [&num_callback_groups2](rclcpp::CallbackGroup::SharedPtr group)
    {
      (void)group;
      num_callback_groups2++;
    });
  EXPECT_EQ(2u + num_callback_groups_in_basic_node, num_callback_groups2);
}

// This is tested more thoroughly in node_interfaces/test_node_graph
TEST_F(TestNode, get_entity_names) {
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  const auto node_names = node->get_node_names();
  EXPECT_NE(
    node_names.end(),
    std::find(node_names.begin(), node_names.end(), node->get_fully_qualified_name()));

  const auto topic_names_and_types = node->get_topic_names_and_types();
  EXPECT_EQ(topic_names_and_types.end(), topic_names_and_types.find("topic"));

  EXPECT_EQ(0u, node->count_publishers("topic"));
  EXPECT_EQ(0u, node->count_subscribers("topic"));

  const auto service_names_and_types = node->get_service_names_and_types();
  EXPECT_EQ(service_names_and_types.end(), service_names_and_types.find("service"));

  const auto service_names_and_types_by_node =
    node->get_service_names_and_types_by_node("node", "/ns");
  EXPECT_EQ(
    service_names_and_types_by_node.end(),
    service_names_and_types_by_node.find("service"));
}

TEST_F(TestNode, wait_for_graph_event) {
  // Even though this node is only used in the std::thread below, it's here to ensure there is no
  // race condition in its destruction and modification of the node_graph
  auto node = std::make_shared<rclcpp::Node>("node", "ns");

  constexpr std::chrono::seconds timeout(10);
  auto thread_start = std::chrono::steady_clock::now();
  auto thread_completion = thread_start;

  // This runs until the graph is updated
  std::thread graph_event_wait_thread([&thread_completion, node, timeout]() {
      auto event = node->get_graph_event();
      EXPECT_NO_THROW(node->wait_for_graph_change(event, timeout));
      thread_completion = std::chrono::steady_clock::now();
    });

  // Start creating nodes until at least one event triggers in graph_event_wait_thread or until 100
  // nodes have been created (at which point this is a failure)
  std::vector<std::shared_ptr<rclcpp::Node>> nodes;
  while (thread_completion == thread_start && nodes.size() < 100) {
    nodes.emplace_back(std::make_shared<rclcpp::Node>("node"_unq, "ns"));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  graph_event_wait_thread.join();
  // Nodes will probably only be of size 1
  EXPECT_LT(0u, nodes.size());
  EXPECT_GT(100u, nodes.size());
  EXPECT_NE(thread_start, thread_completion);
  EXPECT_GT(timeout, thread_completion - thread_start);
}

TEST_F(TestNode, create_sub_node_rmw_validate_namespace_error) {
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rmw_validate_namespace, RMW_RET_INVALID_ARGUMENT);

    // reset() is not necessary for this exception, but it handles unused return value warning
    EXPECT_THROW(
      node->create_sub_node("ns").reset(),
      rclcpp::exceptions::RCLInvalidArgument);
  }
  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rmw_validate_namespace, RMW_RET_ERROR);
    EXPECT_THROW(
      node->create_sub_node("ns").reset(),
      rclcpp::exceptions::RCLError);
  }
}

TEST_F(TestNode, static_and_dynamic_typing) {
  rclcpp::NodeOptions no;
  no.parameter_overrides(
  {
    {"integer_parameter_override_ok", 43},
    {"string_parameter_override_should_throw", 43},
    {"integer_must_provide_override", 43},
    {"cool_way_of_declaring_a_string_without_a_default", "hello!"}
  });
  auto node = std::make_shared<rclcpp::Node>("node", "ns", no);
  {
    auto param = node->declare_parameter("an_int", 42);
    EXPECT_EQ(42, param);
    auto result = node->set_parameter({"an_int", "string value"});
    EXPECT_FALSE(result.successful);
    result = node->set_parameter({"an_int", 43});
    EXPECT_TRUE(result.successful);
    EXPECT_TRUE(node->get_parameter("an_int", param));
    EXPECT_EQ(43, param);
  }
  {
    auto param = node->declare_parameter("integer_parameter_override_ok", 42);
    EXPECT_EQ(43, param);
  }
  {
    EXPECT_THROW(
      node->declare_parameter("string_parameter_override_should_throw", "a string"),
      rclcpp::exceptions::InvalidParameterTypeException);
  }
  {
    auto param = node->declare_parameter(
      "integer_must_provide_override", rclcpp::PARAMETER_INTEGER);
    EXPECT_EQ(43, param.get<int64_t>());
  }
  {
    auto param = node->declare_parameter<std::string>(
      "cool_way_of_declaring_a_string_without_a_default");
    EXPECT_EQ("hello!", param);
  }
  {
    auto param = node->declare_parameter("integer_override_not_given", rclcpp::PARAMETER_INTEGER);
    EXPECT_EQ(rclcpp::PARAMETER_NOT_SET, param.get_type());
    // Throws if not set before access
    EXPECT_THROW(
      node->get_parameter("integer_override_not_given"),
      rclcpp::exceptions::ParameterUninitializedException);
  }
  {
    auto param = node->declare_parameter("integer_set_after_declare", rclcpp::PARAMETER_INTEGER);
    EXPECT_EQ(rclcpp::PARAMETER_NOT_SET, param.get_type());
    auto result = node->set_parameter(rclcpp::Parameter{"integer_set_after_declare", 44});
    ASSERT_TRUE(result.successful) << result.reason;
    auto get_param = node->get_parameter("integer_set_after_declare");
    EXPECT_EQ(44, get_param.as_int());
  }
  {
    EXPECT_THROW(
      node->declare_parameter("parameter_not_set_is_not_a_valid_type", rclcpp::PARAMETER_NOT_SET),
      std::invalid_argument);
  }
  {
    EXPECT_THROW(
      node->declare_parameter(
        "uninitialized_not_valid_except_dynamic_typing", rclcpp::ParameterValue{}),
      rclcpp::exceptions::InvalidParameterTypeException);
  }
}
