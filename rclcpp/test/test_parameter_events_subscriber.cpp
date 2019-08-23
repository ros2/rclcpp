// Copyright (c) 2019 Intel Corporation
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

#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_events_subscriber.hpp"

class TestParameterEventsSubscriber : public rclcpp::ParameterEventsSubscriber
{
public:
  explicit TestParameterEventsSubscriber(rclcpp::Node::SharedPtr node)
  : ParameterEventsSubscriber(node)
  {}

  void test_event(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
  {
    event_callback(event);
  }
};

class TestNode : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp()
  {
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);

    node = std::make_shared<rclcpp::Node>(
      "test_parameter_events_subscriber", options);

    remote_node_name = "/remote_node";
    diff_ns_name = "/ns/remote_node";

    ParamSubscriber = std::make_shared<TestParameterEventsSubscriber>(node);

    same_node_int = std::make_shared<rcl_interfaces::msg::ParameterEvent>();
    same_node_double = std::make_shared<rcl_interfaces::msg::ParameterEvent>();
    multiple = std::make_shared<rcl_interfaces::msg::ParameterEvent>();
    remote_node_string = std::make_shared<rcl_interfaces::msg::ParameterEvent>();
    diff_ns_bool = std::make_shared<rcl_interfaces::msg::ParameterEvent>();

    same_node_int->node = node->get_fully_qualified_name();
    same_node_double->node = node->get_fully_qualified_name();
    multiple->node = node->get_fully_qualified_name();
    remote_node_string->node = remote_node_name;
    diff_ns_bool->node = diff_ns_name;

    rcl_interfaces::msg::Parameter p;
    p.name = "my_int";
    p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    p.value.integer_value = 1;
    same_node_int->changed_parameters.push_back(p);
    multiple->changed_parameters.push_back(p);

    p.name = "my_double";
    p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    p.value.double_value = 1.0;
    same_node_double->changed_parameters.push_back(p);
    multiple->changed_parameters.push_back(p);

    p.name = "my_string";
    p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    p.value.string_value = "test";
    remote_node_string->changed_parameters.push_back(p);

    p.name = "my_bool";
    p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    p.value.bool_value = true;
    diff_ns_bool->changed_parameters.push_back(p);
  }

  void TearDown()
  {
    node.reset();
    ParamSubscriber.reset();
  }

  rcl_interfaces::msg::ParameterEvent::SharedPtr same_node_int;
  rcl_interfaces::msg::ParameterEvent::SharedPtr same_node_double;
  rcl_interfaces::msg::ParameterEvent::SharedPtr remote_node_string;
  rcl_interfaces::msg::ParameterEvent::SharedPtr multiple;
  rcl_interfaces::msg::ParameterEvent::SharedPtr diff_ns_bool;

  rclcpp::Node::SharedPtr node;
  std::string remote_node_name;
  std::string diff_ns_name;
  std::shared_ptr<TestParameterEventsSubscriber> ParamSubscriber;
};

TEST_F(TestNode, RegisterParameterCallback)
{
  bool received;
  auto cb = [&received](const rclcpp::Parameter &) {received = true;};

  ParamSubscriber->register_parameter_callback("my_double", cb);  // same node
  ParamSubscriber->register_parameter_callback("my_int", cb);  // same node
  ParamSubscriber->register_parameter_callback("my_string", cb, remote_node_name);  // remote node
  ParamSubscriber->register_parameter_callback("my_bool", cb, diff_ns_name);  // different namespace

  received = false;
  ParamSubscriber->test_event(same_node_double);
  EXPECT_EQ(received, true);

  received = false;
  ParamSubscriber->test_event(same_node_int);
  EXPECT_EQ(received, true);

  received = false;
  ParamSubscriber->test_event(remote_node_string);
  EXPECT_EQ(received, true);

  received = false;
  ParamSubscriber->test_event(diff_ns_bool);
  EXPECT_EQ(received, true);
}

TEST_F(TestNode, RegisterParameterUpdate)
{
  double double_param;
  int int_param;
  bool bool_param;
  std::string string_param;

  // Set individual parameters
  ParamSubscriber->register_parameter_update("my_double", double_param);
  ParamSubscriber->register_parameter_update("my_int", int_param);
  ParamSubscriber->register_parameter_update("my_string", string_param, remote_node_name);
  ParamSubscriber->register_parameter_update("my_bool", bool_param, diff_ns_name);

  ParamSubscriber->test_event(same_node_double);
  ParamSubscriber->test_event(same_node_int);
  ParamSubscriber->test_event(remote_node_string);
  ParamSubscriber->test_event(diff_ns_bool);

  EXPECT_EQ(double_param, 1.0);
  EXPECT_EQ(int_param, 1);
  EXPECT_EQ(string_param, "test");
  EXPECT_EQ(bool_param, true);

  // Set multiple parameters atomically
  double_param = 0;
  int_param = 0;

  ParamSubscriber->test_event(multiple);
  EXPECT_EQ(double_param, 1.0);
  EXPECT_EQ(int_param, 1);
}

TEST_F(TestNode, UserCallback)
{
  double double_param = 0.0;
  int int_param = 0;
  bool received = false;

  double product;
  auto cb =
    [&int_param, &double_param, &product, &received,
      this](const rcl_interfaces::msg::ParameterEvent::SharedPtr & event)
    {
      auto node_name = node->get_fully_qualified_name();

      if (event->node == node_name) {
        received = true;
      }

      rclcpp::Parameter p;
      if (ParamSubscriber->get_parameter_from_event(event, p, "my_int")) {
        int_param = p.get_value<int>();
      }

      if (ParamSubscriber->get_parameter_from_event(event, p, "my_double")) {
        double_param = p.get_value<double>();
      }

      product = int_param * double_param;
    };

  ParamSubscriber->set_event_callback(cb);

  ParamSubscriber->test_event(diff_ns_bool);
  EXPECT_EQ(received, false);

  ParamSubscriber->test_event(multiple);
  EXPECT_EQ(product, 1.0);
}
