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
    diff_node_int = std::make_shared<rcl_interfaces::msg::ParameterEvent>();

    same_node_int->node = node->get_fully_qualified_name();
    same_node_double->node = node->get_fully_qualified_name();
    multiple->node = node->get_fully_qualified_name();
    remote_node_string->node = remote_node_name;
    diff_ns_bool->node = diff_ns_name;
    diff_node_int->node = remote_node_name;

    rcl_interfaces::msg::Parameter p;
    p.name = "my_int";
    p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    p.value.integer_value = 1;
    same_node_int->changed_parameters.push_back(p);
    diff_node_int->changed_parameters.push_back(p);
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
  rcl_interfaces::msg::ParameterEvent::SharedPtr diff_node_int;
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

  auto h1 = ParamSubscriber->add_parameter_callback("my_double", cb);
  auto h2 = ParamSubscriber->add_parameter_callback("my_int", cb);
  auto h3 = ParamSubscriber->add_parameter_callback("my_string", cb, remote_node_name);
  auto h4 = ParamSubscriber->add_parameter_callback("my_bool", cb, diff_ns_name);

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

TEST_F(TestNode, SameParameterDifferentNode)
{
  int int_param_node1;
  int int_param_node2;

  auto cb1 = [&int_param_node1](const rclcpp::Parameter & p) {
      int_param_node1 = p.get_value<int>();
    };
  auto cb2 = [&int_param_node2](const rclcpp::Parameter & p) {
      int_param_node2 = p.get_value<int>();
    };

  // Set individual parameters
  auto h1 = ParamSubscriber->add_parameter_callback("my_int", cb1);
  auto h2 = ParamSubscriber->add_parameter_callback("my_int", cb2, remote_node_name);

  ParamSubscriber->test_event(same_node_int);
  EXPECT_EQ(int_param_node1, 1);
  EXPECT_NE(int_param_node2, 1);

  int_param_node1 = 0;
  int_param_node2 = 0;

  ParamSubscriber->test_event(diff_node_int);
  EXPECT_NE(int_param_node1, 1);
  EXPECT_EQ(int_param_node2, 1);
}

TEST_F(TestNode, EventCallback)
{
  using rclcpp::ParameterEventsSubscriber;

  double double_param = 0.0;
  int int_param = 0;
  bool bool_param{false};
  bool received{false};

  double product;
  auto cb =
    [&int_param, &double_param, &product, &bool_param, &received,
      this](const rcl_interfaces::msg::ParameterEvent::SharedPtr & event)
    {
      auto node_name = node->get_fully_qualified_name();

      if (event->node == node_name) {
        received = true;
      }

      rclcpp::Parameter p;
      if (ParameterEventsSubscriber::get_parameter_from_event(event, p, "my_int", node_name)) {
        int_param = p.get_value<int>();
      }
      try {
        p = ParameterEventsSubscriber::get_parameter_from_event(event, "my_double", node_name);
        double_param = p.get_value<double>();
      } catch (...) {
      }

      product = int_param * double_param;
    };

  auto cb2 =
    [&bool_param, this](const rcl_interfaces::msg::ParameterEvent::SharedPtr & event)
    {
      rclcpp::Parameter p;
      if (event->node == diff_ns_name) {
        if (ParameterEventsSubscriber::get_parameter_from_event(event, p, "my_bool",
          diff_ns_name))
        {
          bool_param = p.get_value<bool>();
        }
      }
    };

  auto event_handle = ParamSubscriber->add_parameter_event_callback(cb);
  auto event_handle2 = ParamSubscriber->add_parameter_event_callback(cb2);

  bool_param = false;
  ParamSubscriber->test_event(multiple);
  EXPECT_EQ(received, true);
  EXPECT_EQ(product, 1.0);
  EXPECT_EQ(bool_param, false);

  ParamSubscriber->test_event(diff_ns_bool);
  EXPECT_EQ(bool_param, true);

  // Test removal of event callback
  received = false;
  bool_param = false;
  ParamSubscriber->remove_parameter_event_callback(event_handle.get());
  ParamSubscriber->test_event(multiple);
  ParamSubscriber->test_event(diff_ns_bool);
  EXPECT_EQ(received, false);
  EXPECT_EQ(bool_param, true);
}

TEST_F(TestNode, MultipleParameterCallbacks)
{
  bool received_1{false};
  bool received_2{false};

  auto cb1 = [&received_1](const rclcpp::Parameter &) {received_1 = true;};
  auto cb2 = [&received_2](const rclcpp::Parameter &) {received_2 = true;};
  auto cb3 = [](const rclcpp::Parameter &) { /*do nothing*/};

  auto h1 = ParamSubscriber->add_parameter_callback("my_int", cb1);
  auto h2 = ParamSubscriber->add_parameter_callback("my_int", cb2);
  auto h3 = ParamSubscriber->add_parameter_callback("my_double", cb3);

  // Test multiple callbacks per parameter
  ParamSubscriber->test_event(same_node_int);
  EXPECT_EQ(received_1, true);
  EXPECT_EQ(received_2, true);

  // Test removal of parameter callback by callback handle
  received_1 = false;
  received_2 = false;
  ParamSubscriber->remove_parameter_callback(h1.get());
  ParamSubscriber->test_event(same_node_int);
  EXPECT_EQ(received_1, false);
  EXPECT_EQ(received_2, true);

  // Test removal of parameter callback by name
  received_2 = false;
  ParamSubscriber->remove_parameter_callback("my_int");
  ParamSubscriber->test_event(same_node_int);
  EXPECT_EQ(received_2, false);
}
