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

#include "rclcpp/exceptions.hpp"
#include "rclcpp/node.hpp"
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
    auto node = std::make_shared<rclcpp::node::Node>("my_node", "/ns");
  }

  {
    ASSERT_THROW({
      auto node = std::make_shared<rclcpp::node::Node>("invalid_node?", "/ns");
    }, rclcpp::exceptions::InvalidNodeNameError);
  }

  {
    ASSERT_THROW({
      auto node = std::make_shared<rclcpp::node::Node>("my_node", "/invalid_ns?");
    }, rclcpp::exceptions::InvalidNamespaceError);
  }
}

TEST_F(TestNode, get_name_and_namespace) {
  {
    auto node = std::make_shared<rclcpp::node::Node>("my_node", "/ns");
    EXPECT_STREQ("my_node", node->get_name());
    EXPECT_STREQ("/ns", node->get_namespace());
  }
  {
    auto node = std::make_shared<rclcpp::node::Node>("my_node", "ns");
    EXPECT_STREQ("my_node", node->get_name());
    EXPECT_STREQ("/ns", node->get_namespace());
  }
  {
    auto node = std::make_shared<rclcpp::node::Node>("my_node", "/my/ns");
    EXPECT_STREQ("my_node", node->get_name());
    EXPECT_STREQ("/my/ns", node->get_namespace());
  }
  {
    auto node = std::make_shared<rclcpp::node::Node>("my_node", "my/ns");
    EXPECT_STREQ("my_node", node->get_name());
    EXPECT_STREQ("/my/ns", node->get_namespace());
  }
}
