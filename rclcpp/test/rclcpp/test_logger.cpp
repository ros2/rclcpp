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

#include "rcutils/env.h"

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
  rcl_reset_error();
  EXPECT_STREQ(logger.get_name(), "rclcpp");
  rclcpp::shutdown();
}

struct LogEvent
{
  bool console_output_handler_called;
  std::string message;
};
LogEvent g_last_log_event;

TEST(TestLogger, set_level) {
  ASSERT_EQ(RCUTILS_RET_OK, rcutils_logging_initialize());

  rclcpp::Logger logger = rclcpp::get_logger("test_logger");
  EXPECT_THROW(
  {
    logger.set_level(static_cast<rclcpp::Logger::Level>(99999));
  }, rclcpp::exceptions::RCLInvalidArgument);

  auto rcutils_logging_console_output_handler = [](
    const rcutils_log_location_t *,
    int, const char *, rcutils_time_point_value_t,
    const char * format, va_list * args) -> void
    {
      g_last_log_event.console_output_handler_called = true;
      char buffer[1024];
      vsnprintf(buffer, sizeof(buffer), format, *args);
      g_last_log_event.message = buffer;
    };

  rcutils_logging_output_handler_t previous_output_handler = rcutils_logging_get_output_handler();
  rcutils_logging_set_output_handler(rcutils_logging_console_output_handler);

  // default
  RCLCPP_DEBUG(logger, "message %s", "debug");
  EXPECT_FALSE(g_last_log_event.console_output_handler_called);
  RCLCPP_INFO(logger, "message %s", "info");
  EXPECT_TRUE(g_last_log_event.console_output_handler_called);
  EXPECT_EQ("message info", g_last_log_event.message);

  // unset
  g_last_log_event.console_output_handler_called = false;
  logger.set_level(rclcpp::Logger::Level::Unset);
  RCLCPP_DEBUG(logger, "message");
  RCLCPP_DEBUG(logger, "message %s", "debug");
  EXPECT_FALSE(g_last_log_event.console_output_handler_called);
  RCLCPP_INFO(logger, "message");
  RCLCPP_INFO(logger, "message %s", "info");
  EXPECT_TRUE(g_last_log_event.console_output_handler_called);
  EXPECT_EQ("message info", g_last_log_event.message);

  // debug
  g_last_log_event.console_output_handler_called = false;
  logger.set_level(rclcpp::Logger::Level::Debug);
  RCLCPP_DEBUG(logger, "message %s", "debug");
  EXPECT_TRUE(g_last_log_event.console_output_handler_called);
  EXPECT_EQ("message debug", g_last_log_event.message);
  RCLCPP_INFO(logger, "message %s", "info");
  EXPECT_EQ("message info", g_last_log_event.message);

  // info
  g_last_log_event.console_output_handler_called = false;
  logger.set_level(rclcpp::Logger::Level::Info);
  RCLCPP_DEBUG(logger, "message %s", "debug");
  EXPECT_FALSE(g_last_log_event.console_output_handler_called);
  RCLCPP_INFO(logger, "message %s", "info");
  EXPECT_TRUE(g_last_log_event.console_output_handler_called);
  EXPECT_EQ("message info", g_last_log_event.message);

  // warn
  g_last_log_event.console_output_handler_called = false;
  logger.set_level(rclcpp::Logger::Level::Warn);
  RCLCPP_DEBUG(logger, "message %s", "debug");
  EXPECT_FALSE(g_last_log_event.console_output_handler_called);
  RCLCPP_INFO(logger, "message %s", "info");
  EXPECT_FALSE(g_last_log_event.console_output_handler_called);
  RCLCPP_WARN(logger, "message %s", "warn");
  EXPECT_TRUE(g_last_log_event.console_output_handler_called);
  EXPECT_EQ("message warn", g_last_log_event.message);

  // error
  g_last_log_event.console_output_handler_called = false;
  logger.set_level(rclcpp::Logger::Level::Error);
  RCLCPP_DEBUG(logger, "message %s", "debug");
  EXPECT_FALSE(g_last_log_event.console_output_handler_called);
  RCLCPP_INFO(logger, "message %s", "info");
  EXPECT_FALSE(g_last_log_event.console_output_handler_called);
  RCLCPP_WARN(logger, "message %s", "warn");
  EXPECT_FALSE(g_last_log_event.console_output_handler_called);
  RCLCPP_ERROR(logger, "message %s", "error");
  EXPECT_TRUE(g_last_log_event.console_output_handler_called);
  EXPECT_EQ("message error", g_last_log_event.message);

  // fatal
  g_last_log_event.console_output_handler_called = false;
  logger.set_level(rclcpp::Logger::Level::Fatal);
  RCLCPP_DEBUG(logger, "message %s", "debug");
  EXPECT_FALSE(g_last_log_event.console_output_handler_called);
  RCLCPP_INFO(logger, "message %s", "info");
  EXPECT_FALSE(g_last_log_event.console_output_handler_called);
  RCLCPP_WARN(logger, "message %s", "warn");
  EXPECT_FALSE(g_last_log_event.console_output_handler_called);
  RCLCPP_ERROR(logger, "message %s", "error");
  EXPECT_FALSE(g_last_log_event.console_output_handler_called);
  RCLCPP_FATAL(logger, "message %s", "fatal");
  EXPECT_TRUE(g_last_log_event.console_output_handler_called);
  EXPECT_EQ("message fatal", g_last_log_event.message);

  rcutils_logging_set_output_handler(previous_output_handler);
  EXPECT_EQ(RCUTILS_RET_OK, rcutils_logging_shutdown());
}

TEST(TestLogger, get_logging_directory) {
  ASSERT_EQ(true, rcutils_set_env("HOME", "/fake_home_dir"));
  ASSERT_EQ(true, rcutils_set_env("USERPROFILE", nullptr));
  ASSERT_EQ(true, rcutils_set_env("ROS_LOG_DIR", nullptr));
  ASSERT_EQ(true, rcutils_set_env("ROS_HOME", nullptr));

  auto path = rclcpp::get_logging_directory();
  auto expected_path = rcpputils::fs::path{"/fake_home_dir"} / ".ros" / "log";

  // TODO(ivanpauno): Add operator== to rcpputils::fs::path
  auto it = path.cbegin();
  auto eit = expected_path.cbegin();
  for (; it != path.cend() && eit != expected_path.cend(); ++it, ++eit) {
    EXPECT_EQ(*eit, *it);
  }
  EXPECT_EQ(it, path.cend());
  EXPECT_EQ(eit, expected_path.cend());
}
