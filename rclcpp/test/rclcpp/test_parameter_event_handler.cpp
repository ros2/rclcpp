// Copyright 2019 Intel Corporation
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

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

class TestParameterEventHandler : public rclcpp::ParameterEventHandler
{
public:
  explicit TestParameterEventHandler(rclcpp::Node::SharedPtr node)
  : ParameterEventHandler(node)
  {}

  void test_event(rcl_interfaces::msg::ParameterEvent::ConstSharedPtr event)
  {
    callbacks_->event_callback(*event);
  }

  size_t num_event_callbacks()
  {
    return callbacks_->event_callbacks_.size();
  }

  size_t num_parameter_callbacks()
  {
    return callbacks_->parameter_callbacks_.size();
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
    node = std::make_shared<rclcpp::Node>(
      "test_parameter_events_subscriber", options);

    remote_node_name = "/remote_node";
    diff_ns_name = "/ns/remote_node";

    param_handler = std::make_shared<TestParameterEventHandler>(node);

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
    param_handler.reset();
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
  std::shared_ptr<TestParameterEventHandler> param_handler;
};

TEST_F(TestNode, RegisterParameterCallback)
{
  bool received;
  auto cb = [&received](const rclcpp::Parameter &) {received = true;};

  auto h1 = param_handler->add_parameter_callback("my_double", cb);
  auto h2 = param_handler->add_parameter_callback("my_int", cb);
  auto h3 = param_handler->add_parameter_callback("my_string", cb, remote_node_name);
  auto h4 = param_handler->add_parameter_callback("my_bool", cb, diff_ns_name);

  received = false;
  param_handler->test_event(same_node_double);
  EXPECT_EQ(received, true);

  received = false;
  param_handler->test_event(same_node_int);
  EXPECT_EQ(received, true);

  received = false;
  param_handler->test_event(remote_node_string);
  EXPECT_EQ(received, true);

  received = false;
  param_handler->test_event(diff_ns_bool);
  EXPECT_EQ(received, true);
}

TEST_F(TestNode, SameParameterDifferentNode)
{
  int64_t int_param_node1{0};
  int64_t int_param_node2{0};

  auto cb1 = [&int_param_node1](const rclcpp::Parameter & p) {
      int_param_node1 = p.get_value<int64_t>();
    };
  auto cb2 = [&int_param_node2](const rclcpp::Parameter & p) {
      int_param_node2 = p.get_value<int64_t>();
    };

  // Set individual parameters
  auto h1 = param_handler->add_parameter_callback("my_int", cb1);
  auto h2 = param_handler->add_parameter_callback("my_int", cb2, remote_node_name);

  param_handler->test_event(same_node_int);
  EXPECT_EQ(int_param_node1, 1);
  EXPECT_NE(int_param_node2, 1);

  int_param_node1 = 0;
  int_param_node2 = 0;

  param_handler->test_event(diff_node_int);
  EXPECT_NE(int_param_node1, 1);
  EXPECT_EQ(int_param_node2, 1);

  param_handler->remove_parameter_callback(h1);
  param_handler->remove_parameter_callback(h2);
  EXPECT_EQ(param_handler->num_parameter_callbacks(), 0UL);
}

TEST_F(TestNode, GetParameterFromEvent)
{
  using rclcpp::ParameterEventHandler;
  std::string node_name = node->get_fully_qualified_name();
  std::string wrong_name = "/wrong_node_name";

  rclcpp::Parameter p;
  EXPECT_TRUE(
    ParameterEventHandler::get_parameter_from_event(*multiple, p, "my_int", node_name));
  EXPECT_EQ(p.get_value<int>(), 1);
  // False if parameter not with correct node name
  EXPECT_FALSE(
    ParameterEventHandler::get_parameter_from_event(*multiple, p, "my_int", wrong_name));
  // False if parameter not part of event
  EXPECT_FALSE(
    ParameterEventHandler::get_parameter_from_event(*diff_ns_bool, p, "my_int", node_name));

  EXPECT_NO_THROW(
    ParameterEventHandler::get_parameter_from_event(*multiple, "my_int", node_name));
  // Throws if parameter not with correct node name
  EXPECT_THROW(
    ParameterEventHandler::get_parameter_from_event(*multiple, "my_int", wrong_name),
    std::runtime_error);
  // Throws if parameter not part of event
  EXPECT_THROW(
    ParameterEventHandler::get_parameter_from_event(*diff_ns_bool, "my_int", node_name),
    std::runtime_error);
}

TEST_F(TestNode, GetParametersFromEvent)
{
  using rclcpp::ParameterEventHandler;
  std::string node_name = node->get_fully_qualified_name();

  auto params = ParameterEventHandler::get_parameters_from_event(*multiple);
  EXPECT_EQ(params.size(), 2u);
  bool found_int = false;
  bool found_double = false;
  for (auto & p : params) {
    if (p.get_name() == std::string("my_int")) {
      found_int = true;
      EXPECT_EQ(p.get_value<int>(), 1);
    } else if (p.get_name() == std::string("my_double")) {
      found_double = true;
      EXPECT_EQ(p.get_value<double>(), 1.0);
    }
  }
  EXPECT_EQ(found_int, true);
  EXPECT_EQ(found_double, true);

  params = ParameterEventHandler::get_parameters_from_event(*remote_node_string);
  EXPECT_EQ(params.size(), 1u);
  bool found_string = false;
  for (auto & p : params) {
    if (p.get_name() == std::string("my_string")) {
      found_string = true;
      EXPECT_EQ(p.get_value<std::string>(), std::string("test"));
    }
  }
  EXPECT_EQ(found_string, true);

  params = ParameterEventHandler::get_parameters_from_event(*diff_ns_bool);
  EXPECT_EQ(params.size(), 1u);
  bool found_bool = false;
  for (auto & p : params) {
    if (p.get_name() == std::string("my_bool")) {
      found_bool = true;
      EXPECT_EQ(p.get_value<bool>(), true);
    }
  }
  EXPECT_EQ(found_bool, true);
}

TEST_F(TestNode, EventCallback)
{
  using rclcpp::ParameterEventHandler;

  double double_param = 0.0;
  int64_t int_param = 0;
  bool bool_param{false};
  bool received{false};

  double product;
  auto cb =
    [&int_param, &double_param, &product, &received,
      this](const rcl_interfaces::msg::ParameterEvent & event)
    {
      auto node_name = node->get_fully_qualified_name();

      if (event.node == node_name) {
        received = true;
      }

      rclcpp::Parameter p;
      if (ParameterEventHandler::get_parameter_from_event(event, p, "my_int", node_name)) {
        int_param = p.get_value<int64_t>();
      }
      try {
        p = ParameterEventHandler::get_parameter_from_event(event, "my_double", node_name);
        double_param = p.get_value<double>();
      } catch (...) {
      }

      product = static_cast<double>(int_param) * double_param;
    };

  auto cb2 =
    [&bool_param, this](const rcl_interfaces::msg::ParameterEvent & event)
    {
      rclcpp::Parameter p;
      if (event.node == diff_ns_name) {
        if (ParameterEventHandler::get_parameter_from_event(
            event, p, "my_bool", diff_ns_name))
        {
          bool_param = p.get_value<bool>();
        }
      }
    };

  auto event_handle1 = param_handler->add_parameter_event_callback(cb);
  auto event_handle2 = param_handler->add_parameter_event_callback(cb2);

  bool_param = false;
  param_handler->test_event(multiple);
  EXPECT_EQ(received, true);
  EXPECT_EQ(product, 1.0);
  EXPECT_EQ(bool_param, false);

  param_handler->test_event(diff_ns_bool);
  EXPECT_EQ(bool_param, true);

  // Test removal of event callback
  received = false;
  bool_param = false;
  param_handler->remove_parameter_event_callback(event_handle1);
  param_handler->test_event(multiple);
  param_handler->test_event(diff_ns_bool);
  EXPECT_EQ(received, false);
  EXPECT_EQ(bool_param, true);

  // Should throw if callback handle no longer exists or already removed
  EXPECT_THROW(
    param_handler->remove_parameter_event_callback(event_handle1), std::runtime_error);
}

TEST_F(TestNode, MultipleParameterCallbacks)
{
  bool received_1{false};
  bool received_2{false};

  auto cb1 = [&received_1](const rclcpp::Parameter &) {received_1 = true;};
  auto cb2 = [&received_2](const rclcpp::Parameter &) {received_2 = true;};
  auto cb3 = [](const rclcpp::Parameter &) { /*do nothing*/};

  auto h1 = param_handler->add_parameter_callback("my_int", cb1);
  auto h2 = param_handler->add_parameter_callback("my_int", cb2);
  auto h3 = param_handler->add_parameter_callback("my_double", cb3);

  // Test multiple callbacks per parameter
  param_handler->test_event(same_node_int);
  EXPECT_EQ(received_1, true);
  EXPECT_EQ(received_2, true);

  // Test removal of parameter callback by callback handle
  received_1 = false;
  received_2 = false;
  param_handler->remove_parameter_callback(h1);
  param_handler->test_event(same_node_int);
  EXPECT_EQ(received_1, false);
  EXPECT_EQ(received_2, true);

  // Test removal of parameter callback by name
  received_2 = false;
  param_handler->remove_parameter_callback(h2);
  param_handler->test_event(same_node_int);
  EXPECT_EQ(received_2, false);

  // Should throw if callback handle no longer exists or already removed
  EXPECT_THROW(param_handler->remove_parameter_callback(h1), std::runtime_error);
  EXPECT_THROW(param_handler->remove_parameter_callback(h2), std::runtime_error);

  param_handler->remove_parameter_callback(h3);

  // All callbacks should have been removed
  EXPECT_EQ(received_2, 0);
  EXPECT_EQ(param_handler->num_event_callbacks(), 0UL);
}

TEST_F(TestNode, LastInFirstCallForParameterCallbacks)
{
  rclcpp::Time time_1;
  rclcpp::Time time_2;

  // The callbacks will log the current time for comparison purposes. Add a bit of a stall
  // to ensure that the time noted in the back-to-back calls isn't the same
  auto cb1 = [this, &time_1](const rclcpp::Parameter &) {
      time_1 = node->now();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    };
  auto cb2 = [this, &time_2](const rclcpp::Parameter &) {
      time_2 = node->now();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    };

  auto h1 = param_handler->add_parameter_callback("my_int", cb1);
  auto h2 = param_handler->add_parameter_callback("my_int", cb2);

  // Test multiple callbacks per parameter
  param_handler->test_event(same_node_int);

  // The most-recently install handler should be called first
  EXPECT_EQ(time_2 < time_1, true);

  param_handler->remove_parameter_callback(h1);
  param_handler->remove_parameter_callback(h2);
  EXPECT_EQ(param_handler->num_parameter_callbacks(), 0UL);
}

TEST_F(TestNode, LastInFirstCallForParameterEventCallbacks)
{
  rclcpp::Time time_1;
  rclcpp::Time time_2;

  // The callbacks will log the current time for comparison purposes. Add a bit of a stall
  // to ensure that the time noted in the back-to-back calls isn't the same
  auto cb1 =
    [this, &time_1](const rcl_interfaces::msg::ParameterEvent &)
    {
      time_1 = node->now();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    };
  auto cb2 =
    [this, &time_2](const rcl_interfaces::msg::ParameterEvent &)
    {
      time_2 = node->now();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    };

  auto h1 = param_handler->add_parameter_event_callback(cb1);
  auto h2 = param_handler->add_parameter_event_callback(cb2);

  // Test multiple callbacks per parameter
  param_handler->test_event(same_node_int);

  // The most-recently install handler should be called first
  EXPECT_EQ(time_2 < time_1, true);

  param_handler->remove_parameter_event_callback(h1);
  param_handler->remove_parameter_event_callback(h2);
  EXPECT_EQ(param_handler->num_event_callbacks(), 0UL);
}
