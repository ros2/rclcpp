// Copyright 2023 Sony Group Corporation.
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
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rcl_interfaces/srv/get_logger_levels.hpp"
#include "rcl_interfaces/srv/set_logger_levels.hpp"

using namespace std::chrono_literals;

class TestLoggingService : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp()
  {
    rclcpp::NodeOptions options = rclcpp::NodeOptions();
    options.enable_log_service(true);
    node = std::make_shared<rclcpp::Node>("test_logging_service", "/ns", options);
  }

  rclcpp::Node::SharedPtr node;
};

TEST_F(TestLoggingService, get_logger_levels) {
  auto client = node->create_client<rcl_interfaces::srv::GetLoggerLevels>(
    "/ns/test_logging_service/get_logger_levels");
  ASSERT_TRUE(client->wait_for_service(1s));
  auto request = std::make_shared<rcl_interfaces::srv::GetLoggerLevels::Request>();
  request->names = {"/ns/test_logging_service"};
  auto result = client->async_send_request(request);
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(node, result),
    rclcpp::FutureReturnCode::SUCCESS);
  auto result_get = result.get();
  ASSERT_EQ(result_get->levels.size(), 1u);
  ASSERT_STREQ(result_get->levels[0].name.c_str(), "/ns/test_logging_service");
  ASSERT_EQ(result_get->levels[0].level, 0u);
}

TEST_F(TestLoggingService, set_logger_levels) {
  auto client = node->create_client<rcl_interfaces::srv::SetLoggerLevels>(
    "/ns/test_logging_service/set_logger_levels");
  ASSERT_TRUE(client->wait_for_service(1s));

  {
    // set log level with right value
    auto request = std::make_shared<rcl_interfaces::srv::SetLoggerLevels::Request>();
    auto level = rcl_interfaces::msg::LoggerLevel();
    level.name = "/ns/test_logging_service";
    level.level = 10u;
    request->levels.push_back(level);
    auto result = client->async_send_request(request);
    ASSERT_EQ(
      rclcpp::spin_until_future_complete(node, result),
      rclcpp::FutureReturnCode::SUCCESS);
    auto result_get = result.get();
    ASSERT_EQ(result_get->results.size(), 1u);
    ASSERT_TRUE(result_get->results[0].successful);
  }

  {
    // set log level with wrong value
    auto request = std::make_shared<rcl_interfaces::srv::SetLoggerLevels::Request>();
    auto level = rcl_interfaces::msg::LoggerLevel();
    level.name = "/ns/test_logging_service";
    level.level = 11u;
    request->levels.push_back(level);
    auto result = client->async_send_request(request);
    ASSERT_EQ(
      rclcpp::spin_until_future_complete(node, result),
      rclcpp::FutureReturnCode::SUCCESS);
    auto result_get = result.get();
    ASSERT_EQ(result_get->results.size(), 1u);
    ASSERT_FALSE(result_get->results[0].successful);
  }
}
