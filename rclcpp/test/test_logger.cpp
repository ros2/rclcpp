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

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

TEST(TestLogger, factory_functions) {
  rclcpp::logger::Logger logger = rclcpp::logger::get_logger("test_logger");
  EXPECT_STREQ("test_logger", logger.get_name());
  rclcpp::logger::Logger logger_copy = rclcpp::logger::Logger(logger);
  EXPECT_STREQ("test_logger", logger_copy.get_name());
}

TEST(TestLogger, hierarchy) {
  rclcpp::logger::Logger logger = rclcpp::logger::get_logger("test_logger");
  rclcpp::logger::Logger sublogger = logger.get_child("child");
  EXPECT_STREQ("test_logger.child", sublogger.get_name());
  rclcpp::logger::Logger subsublogger = sublogger.get_child("grandchild");
  EXPECT_STREQ("test_logger.child.grandchild", subsublogger.get_name());
}

TEST(TestLogger, helper_functions) {
  rclcpp::logger::Logger logger = rclcpp::logger::get_logger("test_logger");
  EXPECT_STREQ(logger.get_name(), rclcpp::logging_macro_utilities::_get_logger_name(logger));
}
