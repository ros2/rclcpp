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

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "rcl/node_options.h"
#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/node_base.hpp"
#include "rclcpp/rclcpp.hpp"

TEST(TestNodeBase, construct_from_node)
{
  rclcpp::init(0, nullptr);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("node", "ns");

  // This dynamic cast is not necessary for the unittest itself, but the coverage utility lcov
  // reports these functions uncovered otherwise.
  auto * node_base =
    dynamic_cast<rclcpp::node_interfaces::NodeBase *>(node->get_node_base_interface().get());
  ASSERT_NE(nullptr, node_base);

  EXPECT_STREQ("node", node_base->get_name());
  EXPECT_STREQ("/ns", node_base->get_namespace());

  std::string expected_fully_qualified_name = "/ns/node";
  EXPECT_STREQ(expected_fully_qualified_name.c_str(), node_base->get_fully_qualified_name());
  EXPECT_NE(nullptr, node_base->get_context());
  EXPECT_NE(nullptr, node_base->get_rcl_node_handle());
  EXPECT_NE(nullptr, node_base->get_shared_rcl_node_handle());

  const auto * const_node_base = node_base;
  EXPECT_NE(nullptr, const_node_base->get_rcl_node_handle());
  EXPECT_NE(nullptr, const_node_base->get_shared_rcl_node_handle());
  rclcpp::shutdown();
}
