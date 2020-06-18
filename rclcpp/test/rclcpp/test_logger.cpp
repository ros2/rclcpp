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

#include <memory>
#include <string>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"

TEST(TestLogger, factory_functions) {
  rclcpp::Logger logger = rclcpp::get_logger("test_logger");
  EXPECT_STREQ("test_logger", logger.get_name());
  rclcpp::Logger logger_copy = rclcpp::Logger(logger);
  EXPECT_STREQ("test_logger", logger_copy.get_name());
}

TEST(TestLogger, hierarchy) {
  rclcpp::Logger logger = rclcpp::get_logger("test_logger");
  rclcpp::Logger sublogger = logger.get_child("child");
  EXPECT_STREQ("test_logger.child", sublogger.get_name());
  rclcpp::Logger subsublogger = sublogger.get_child("grandchild");
  EXPECT_STREQ("test_logger.child.grandchild", subsublogger.get_name());
}

TEST(TestLogger, get_node_logger) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  auto node_base = rclcpp::node_interfaces::get_node_base_interface(node);
  auto logger = rclcpp::get_node_logger(node_base->get_rcl_node_handle());
  EXPECT_STREQ(logger.get_name(), "ns.my_node");

  logger = rclcpp::get_node_logger(nullptr);
  EXPECT_STREQ(logger.get_name(), "rclcpp");
  rclcpp::shutdown();
}
