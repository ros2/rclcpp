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

#include <gmock/gmock.h>

#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rcutils/logging.h"
#include "rcutils/time.h"

using ::testing::EndsWith;

size_t g_log_calls = 0;
rclcpp::Logger g_logger = rclcpp::get_logger("name");

struct LogEvent
{
  const rcutils_log_location_t * location;
  int level;
  std::string name;
  rcutils_time_point_value_t timestamp;
  std::string message;
};
LogEvent g_last_log_event;

class TestLoggingMacros : public ::testing::Test
{
public:
  rcutils_logging_output_handler_t previous_output_handler;
  void SetUp()
  {
    g_log_calls = 0;
    ASSERT_EQ(RCUTILS_RET_OK, rcutils_logging_initialize());
    rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_DEBUG);

    auto rcutils_logging_console_output_handler = [](
      const rcutils_log_location_t * location,
      int level, const char * name, rcutils_time_point_value_t timestamp,
      const char * format, va_list * args) -> void
      {
        g_log_calls += 1;
        g_last_log_event.location = location;
        g_last_log_event.level = level;
        g_last_log_event.name = name ? name : "";
        g_last_log_event.timestamp = timestamp;
        char buffer[1024];
        vsnprintf(buffer, sizeof(buffer), format, *args);
        g_last_log_event.message = buffer;
      };

    this->previous_output_handler = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(rcutils_logging_console_output_handler);
  }

  void TearDown()
  {
    rcutils_logging_set_output_handler(this->previous_output_handler);
    ASSERT_EQ(RCUTILS_RET_OK, rcutils_logging_shutdown());
    EXPECT_FALSE(g_rcutils_logging_initialized);
  }
};

class DummyNode
{
public:
  DummyNode()
  {
    clock_ = rclcpp::Clock::make_shared(RCL_ROS_TIME);
  }
  rclcpp::Clock::SharedPtr get_clock()
  {
    return clock_;
  }

private:
  rclcpp::Clock::SharedPtr clock_;
};

TEST_F(TestLoggingMacros, test_logging_named) {
  for (int i : {1, 2, 3}) {
    RCLCPP_DEBUG(g_logger, "message %d", i);
  }
  size_t expected_location = __LINE__ - 2u;
  EXPECT_EQ(3u, g_log_calls);
  EXPECT_TRUE(g_last_log_event.location != NULL);
  if (g_last_log_event.location) {
    EXPECT_STREQ("TestBody", g_last_log_event.location->function_name);
    EXPECT_THAT(g_last_log_event.location->file_name, EndsWith("test_logging.cpp"));
    EXPECT_EQ(expected_location, g_last_log_event.location->line_number);
  }
  EXPECT_EQ(RCUTILS_LOG_SEVERITY_DEBUG, g_last_log_event.level);
  EXPECT_EQ("name", g_last_log_event.name);
  EXPECT_EQ("message 3", g_last_log_event.message);
}

TEST_F(TestLoggingMacros, test_logging_string) {
  for (std::string i : {"one", "two", "three"}) {
    RCLCPP_DEBUG(g_logger, "message " + i);
  }
  EXPECT_EQ(3u, g_log_calls);
  EXPECT_EQ("message three", g_last_log_event.message);

  RCLCPP_DEBUG(g_logger, "message " "four");
  EXPECT_EQ("message four", g_last_log_event.message);

  RCLCPP_DEBUG(g_logger, std::string("message " "five"));
  EXPECT_EQ("message five", g_last_log_event.message);

  RCLCPP_DEBUG(g_logger, std::string("message %s"), "six");
  EXPECT_EQ("message six", g_last_log_event.message);

  RCLCPP_DEBUG(g_logger, "message seven");
  EXPECT_EQ("message seven", g_last_log_event.message);
}

TEST_F(TestLoggingMacros, test_logging_stream) {
  for (std::string i : {"one", "two", "three"}) {
    RCLCPP_DEBUG_STREAM(g_logger, "message " << i);
  }
  EXPECT_EQ(3u, g_log_calls);
  EXPECT_EQ("message three", g_last_log_event.message);

  RCLCPP_DEBUG_STREAM(g_logger, 4 << "th message");
  EXPECT_EQ("4th message", g_last_log_event.message);

  RCLCPP_DEBUG_STREAM(g_logger, "message " << 5);
  EXPECT_EQ("message 5", g_last_log_event.message);
}

TEST_F(TestLoggingMacros, test_logging_once) {
  for (int i : {1, 2, 3}) {
    RCLCPP_INFO_ONCE(g_logger, "message %d", i);
  }
  EXPECT_EQ(1u, g_log_calls);
  EXPECT_EQ(RCUTILS_LOG_SEVERITY_INFO, g_last_log_event.level);
  EXPECT_EQ("name", g_last_log_event.name);
  EXPECT_EQ("message 1", g_last_log_event.message);

  // Check that another instance has a context that's independent to the call above's
  g_log_calls = 0;
  for (int i : {1, 2, 3}) {
    RCLCPP_INFO_ONCE(g_logger, "second message %d", i);
  }
  EXPECT_EQ(1u, g_log_calls);
  EXPECT_EQ(RCUTILS_LOG_SEVERITY_INFO, g_last_log_event.level);
  EXPECT_EQ("name", g_last_log_event.name);
  EXPECT_EQ("second message 1", g_last_log_event.message);
}

TEST_F(TestLoggingMacros, test_logging_expression) {
  for (int i : {1, 2, 3, 4, 5, 6}) {
    RCLCPP_INFO_EXPRESSION(g_logger, i % 3, "message %d", i);
  }
  EXPECT_EQ(4u, g_log_calls);
  EXPECT_EQ("message 5", g_last_log_event.message);
}

int g_counter = 0;

bool mod3()
{
  return (g_counter % 3) != 0;
}

TEST_F(TestLoggingMacros, test_logging_function) {
  for (int i : {1, 2, 3, 4, 5, 6}) {
    g_counter = i;
    RCLCPP_INFO_FUNCTION(g_logger, &mod3, "message %d", i);
  }
  EXPECT_EQ(4u, g_log_calls);
  EXPECT_EQ("message 5", g_last_log_event.message);
}

TEST_F(TestLoggingMacros, test_logging_skipfirst) {
  for (uint32_t i : {1, 2, 3, 4, 5}) {
    RCLCPP_WARN_SKIPFIRST(g_logger, "message %u", i);
    EXPECT_EQ(i - 1, g_log_calls);
  }
}

TEST_F(TestLoggingMacros, test_throttle) {
  using namespace std::chrono_literals;
  rclcpp::Clock steady_clock(RCL_STEADY_TIME);
  for (uint64_t i = 0; i < 3; ++i) {
    RCLCPP_DEBUG_THROTTLE(g_logger, steady_clock, 10000, "Throttling");
  }
  EXPECT_EQ(1u, g_log_calls);
  RCLCPP_DEBUG_SKIPFIRST_THROTTLE(g_logger, steady_clock, 1, "Skip first throttling");
  EXPECT_EQ(1u, g_log_calls);
  for (uint64_t i = 0; i < 6; ++i) {
    RCLCPP_DEBUG_THROTTLE(g_logger, steady_clock, 100, "Throttling");
    RCLCPP_DEBUG_SKIPFIRST_THROTTLE(g_logger, steady_clock, 400, "Throttling");
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_EQ(4u, g_log_calls);
  rclcpp::Clock ros_clock(RCL_ROS_TIME);
  ASSERT_EQ(RCL_RET_OK, rcl_enable_ros_time_override(ros_clock.get_clock_handle()));
  RCLCPP_DEBUG_THROTTLE(g_logger, ros_clock, 10000, "Throttling");
  rcl_clock_t * clock = ros_clock.get_clock_handle();
  ASSERT_TRUE(clock);
  EXPECT_EQ(4u, g_log_calls);
  EXPECT_EQ(RCL_RET_OK, rcl_set_ros_time_override(clock, RCUTILS_MS_TO_NS(10)));
  for (uint64_t i = 0; i < 2; ++i) {
    RCLCPP_DEBUG_THROTTLE(g_logger, ros_clock, 10, "Throttling");
    if (i == 0) {
      EXPECT_EQ(5u, g_log_calls);
      rcl_time_point_value_t clock_ns = ros_clock.now().nanoseconds() + RCUTILS_MS_TO_NS(10);
      EXPECT_EQ(RCL_RET_OK, rcl_set_ros_time_override(clock, clock_ns));
    } else {
      EXPECT_EQ(6u, g_log_calls);
    }
  }
  DummyNode node;
  rcl_clock_t * node_clock = node.get_clock()->get_clock_handle();
  ASSERT_TRUE(node_clock);
  ASSERT_EQ(RCL_RET_OK, rcl_enable_ros_time_override(node_clock));
  EXPECT_EQ(6u, g_log_calls);
  EXPECT_EQ(RCL_RET_OK, rcl_set_ros_time_override(node_clock, RCUTILS_MS_TO_NS(10)));
  for (uint64_t i = 0; i < 3; ++i) {
    RCLCPP_DEBUG_THROTTLE(g_logger, *node.get_clock(), 10, "Throttling");
    if (i == 0) {
      EXPECT_EQ(7u, g_log_calls);
      rcl_time_point_value_t clock_ns = node.get_clock()->now().nanoseconds() + RCUTILS_MS_TO_NS(5);
      EXPECT_EQ(RCL_RET_OK, rcl_set_ros_time_override(node_clock, clock_ns));
    } else if (i == 1) {
      EXPECT_EQ(7u, g_log_calls);
      rcl_time_point_value_t clock_ns = node.get_clock()->now().nanoseconds() + RCUTILS_MS_TO_NS(5);
      EXPECT_EQ(RCL_RET_OK, rcl_set_ros_time_override(node_clock, clock_ns));
    } else {
      EXPECT_EQ(8u, g_log_calls);
    }
  }
}

TEST_F(TestLoggingMacros, test_parameter_expression) {
  RCLCPP_DEBUG_STREAM(*&g_logger, "message");
  EXPECT_EQ(1u, g_log_calls);
  EXPECT_EQ("message", g_last_log_event.message);
}

bool log_function(rclcpp::Logger logger)
{
  RCLCPP_INFO(logger, "successful log");
  return true;
}

bool log_function_const(const rclcpp::Logger logger)
{
  RCLCPP_INFO(logger, "successful log");
  return true;
}

bool log_function_const_ref(const rclcpp::Logger & logger)
{
  RCLCPP_INFO(logger, "successful log");
  return true;
}

TEST_F(TestLoggingMacros, test_log_from_node) {
  auto logger = rclcpp::get_logger("test_logging_logger");
  EXPECT_TRUE(log_function(logger));
  EXPECT_TRUE(log_function_const(logger));
  EXPECT_TRUE(log_function_const_ref(logger));
}
