// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/node_interfaces/get_node_topics_interface.hpp"
#include "rclcpp/rclcpp.hpp"

#include "./node_wrapper.hpp"

static const std::string node_suffix = "test_get_node_interfaces";  // NOLINT

class TestGetNodeInterfaces : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>(node_suffix);
    wrapped_node = std::make_shared<NodeWrapper>("wrapped_" + node_suffix);
  }

  static void TearDownTestCase()
  {
    node.reset();
    wrapped_node.reset();
  }

  static rclcpp::Node::SharedPtr node;
  static std::shared_ptr<NodeWrapper> wrapped_node;
};

rclcpp::Node::SharedPtr TestGetNodeInterfaces::node = nullptr;
std::shared_ptr<NodeWrapper> TestGetNodeInterfaces::wrapped_node = nullptr;

TEST_F(TestGetNodeInterfaces, rclcpp_node_shared_ptr) {
  auto result = rclcpp::node_interfaces::get_node_topics_interface(this->node);
  static_assert(
    std::is_same<
      rclcpp::node_interfaces::NodeTopicsInterface *,
      decltype(result)
    >::value, "expected rclcpp::node_interfaces::NodeTopicsInterface *");
}

TEST_F(TestGetNodeInterfaces, node_shared_ptr) {
  auto result = rclcpp::node_interfaces::get_node_topics_interface(this->wrapped_node);
  static_assert(
    std::is_same<
      rclcpp::node_interfaces::NodeTopicsInterface *,
      decltype(result)
    >::value, "expected rclcpp::node_interfaces::NodeTopicsInterface *");
}

TEST_F(TestGetNodeInterfaces, rclcpp_node_reference) {
  rclcpp::Node & node_reference = *this->node;
  auto result = rclcpp::node_interfaces::get_node_topics_interface(node_reference);
  static_assert(
    std::is_same<
      rclcpp::node_interfaces::NodeTopicsInterface *,
      decltype(result)
    >::value, "expected rclcpp::node_interfaces::NodeTopicsInterface *");
}

TEST_F(TestGetNodeInterfaces, node_reference) {
  NodeWrapper & wrapped_node_reference = *this->wrapped_node;
  auto result = rclcpp::node_interfaces::get_node_topics_interface(wrapped_node_reference);
  static_assert(
    std::is_same<
      rclcpp::node_interfaces::NodeTopicsInterface *,
      decltype(result)
    >::value, "expected rclcpp::node_interfaces::NodeTopicsInterface *");
}

TEST_F(TestGetNodeInterfaces, rclcpp_node_pointer) {
  rclcpp::Node * node_pointer = this->node.get();
  auto result = rclcpp::node_interfaces::get_node_topics_interface(node_pointer);
  static_assert(
    std::is_same<
      rclcpp::node_interfaces::NodeTopicsInterface *,
      decltype(result)
    >::value, "expected rclcpp::node_interfaces::NodeTopicsInterface *");
}

TEST_F(TestGetNodeInterfaces, node_pointer) {
  NodeWrapper * wrapped_node_pointer = this->wrapped_node.get();
  auto result = rclcpp::node_interfaces::get_node_topics_interface(wrapped_node_pointer);
  static_assert(
    std::is_same<
      rclcpp::node_interfaces::NodeTopicsInterface *,
      decltype(result)
    >::value, "expected rclcpp::node_interfaces::NodeTopicsInterface *");
}

TEST_F(TestGetNodeInterfaces, interface_shared_pointer) {
  std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> interface_shared_ptr =
    this->node->get_node_topics_interface();
  auto result = rclcpp::node_interfaces::get_node_topics_interface(interface_shared_ptr);
  static_assert(
    std::is_same<
      rclcpp::node_interfaces::NodeTopicsInterface *,
      decltype(result)
    >::value, "expected rclcpp::node_interfaces::NodeTopicsInterface *");
}
