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

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rcl_interfaces/srv/get_logger_levels.hpp"
#include "rcl_interfaces/srv/set_logger_levels.hpp"

using namespace std::chrono_literals;

class TestLoggerService : public ::testing::Test
{
protected:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options = rclcpp::NodeOptions();
    options.enable_logger_service(true);
    node_ = std::make_shared<rclcpp::Node>("test_logger_service", "/test", options);
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node_;
  std::thread thread_;
};

TEST_F(TestLoggerService, check_connect_get_logger_service) {
  auto client = node_->create_client<rcl_interfaces::srv::GetLoggerLevels>(
    "/test/test_logger_service/get_logger_levels");
  ASSERT_TRUE(client->wait_for_service(2s));
}

TEST_F(TestLoggerService, check_connect_set_logger_service) {
  auto client = node_->create_client<rcl_interfaces::srv::SetLoggerLevels>(
    "/test/test_logger_service/set_logger_levels");
  ASSERT_TRUE(client->wait_for_service(2s));
}

TEST_F(TestLoggerService, test_set_and_get_one_logging_level) {
  std::string test_logger_name = "rcl";
  uint8_t test_logger_level = 20;
  {
    auto client = node_->create_client<rcl_interfaces::srv::SetLoggerLevels>(
      "/test/test_logger_service/set_logger_levels");
    ASSERT_TRUE(client->wait_for_service(1s));
    auto request = std::make_shared<rcl_interfaces::srv::SetLoggerLevels::Request>();
    auto level = rcl_interfaces::msg::LoggerLevel();
    level.name = test_logger_name;
    level.level = test_logger_level;
    request->levels.push_back(level);
    auto result = client->async_send_request(request);
    ASSERT_EQ(
      rclcpp::spin_until_future_complete(node_, result),
      rclcpp::FutureReturnCode::SUCCESS);
    auto result_get = result.get();
    ASSERT_EQ(result_get->results.size(), 1u);
    ASSERT_TRUE(result_get->results[0].successful);
    ASSERT_STREQ(result_get->results[0].reason.c_str(), "");
  }

  {
    auto client = node_->create_client<rcl_interfaces::srv::GetLoggerLevels>(
      "/test/test_logger_service/get_logger_levels");
    ASSERT_TRUE(client->wait_for_service(1s));
    auto request = std::make_shared<rcl_interfaces::srv::GetLoggerLevels::Request>();
    request->names.emplace_back(test_logger_name);
    auto result = client->async_send_request(request);
    ASSERT_EQ(
      rclcpp::spin_until_future_complete(node_, result),
      rclcpp::FutureReturnCode::SUCCESS);
    auto result_get = result.get();
    ASSERT_EQ(result_get->levels.size(), 1u);
    ASSERT_STREQ(result_get->levels[0].name.c_str(), test_logger_name.c_str());
    ASSERT_EQ(result_get->levels[0].level, test_logger_level);
  }
}

TEST_F(TestLoggerService, test_set_and_get_multi_logging_level) {
  std::vector<std::pair<std::string, uint8_t>> test_data {
    {"rcl", 30},
    {"rclcpp", 40},
    {"/test/test_logger_service", 50}
  };

  // Set multi log levels
  {
    auto client = node_->create_client<rcl_interfaces::srv::SetLoggerLevels>(
      "/test/test_logger_service/set_logger_levels");
    ASSERT_TRUE(client->wait_for_service(1s));
    auto request = std::make_shared<rcl_interfaces::srv::SetLoggerLevels::Request>();
    for (auto & set_level : test_data) {
      auto level = rcl_interfaces::msg::LoggerLevel();
      level.name = std::get<0>(set_level);
      level.level = std::get<1>(set_level);
      request->levels.push_back(level);
    }
    auto result = client->async_send_request(request);
    ASSERT_EQ(
      rclcpp::spin_until_future_complete(node_, result),
      rclcpp::FutureReturnCode::SUCCESS);
    auto result_get = result.get();
    ASSERT_EQ(result_get->results.size(), test_data.size());
    for (uint32_t i = 0; i < test_data.size(); i++) {
      ASSERT_TRUE(result_get->results[0].successful);
    }
  }

  // Get multi log levels
  {
    auto client = node_->create_client<rcl_interfaces::srv::GetLoggerLevels>(
      "/test/test_logger_service/get_logger_levels");
    ASSERT_TRUE(client->wait_for_service(1s));
    auto request = std::make_shared<rcl_interfaces::srv::GetLoggerLevels::Request>();
    for (auto & set_level : test_data) {
      request->names.emplace_back(std::get<0>(set_level));
    }
    auto result = client->async_send_request(request);
    ASSERT_EQ(
      rclcpp::spin_until_future_complete(node_, result),
      rclcpp::FutureReturnCode::SUCCESS);
    auto result_get = result.get();
    ASSERT_EQ(result_get->levels.size(), test_data.size());
    for (uint32_t i = 0; i < test_data.size(); i++) {
      ASSERT_STREQ(result_get->levels[i].name.c_str(), std::get<0>(test_data[i]).c_str());
      ASSERT_EQ(result_get->levels[i].level, std::get<1>(test_data[i]));
    }
  }
}

TEST_F(TestLoggerService, test_set_logging_level_with_invalid_param) {
  std::vector<std::pair<std::string, uint8_t>> test_data {
    {"rcl", 12},
    {"/test/test_logger_service", 22}
  };

  // Set multi log levels
  {
    auto client = node_->create_client<rcl_interfaces::srv::SetLoggerLevels>(
      "/test/test_logger_service/set_logger_levels");
    ASSERT_TRUE(client->wait_for_service(1s));
    auto request = std::make_shared<rcl_interfaces::srv::SetLoggerLevels::Request>();
    for (auto & set_level : test_data) {
      auto level = rcl_interfaces::msg::LoggerLevel();
      level.name = std::get<0>(set_level);
      level.level = std::get<1>(set_level);
      request->levels.push_back(level);
    }
    auto result = client->async_send_request(request);
    ASSERT_EQ(
      rclcpp::spin_until_future_complete(node_, result),
      rclcpp::FutureReturnCode::SUCCESS);
    auto result_get = result.get();
    ASSERT_EQ(result_get->results.size(), test_data.size());
    for (uint32_t i = 0; i < test_data.size(); i++) {
      ASSERT_FALSE(result_get->results[i].successful);
      // Check string starts with prefix
      ASSERT_EQ(
        result_get->results[i].reason.rfind("Unable to determine severity_string for severity", 0),
        0);
    }
  }
}

TEST_F(TestLoggerService, test_set_logging_level_with_partial_invalid_param) {
  std::vector<std::pair<std::string, uint8_t>> test_data {
    {"rcl", 20},
    {"rclcpp", 22},
    {"/test/test_logger_service", 30}
  };

  // Set multi log levels
  {
    auto client = node_->create_client<rcl_interfaces::srv::SetLoggerLevels>(
      "/test/test_logger_service/set_logger_levels");
    ASSERT_TRUE(client->wait_for_service(1s));
    auto request = std::make_shared<rcl_interfaces::srv::SetLoggerLevels::Request>();
    for (auto & set_level : test_data) {
      auto level = rcl_interfaces::msg::LoggerLevel();
      level.name = std::get<0>(set_level);
      level.level = std::get<1>(set_level);
      request->levels.push_back(level);
    }
    auto result = client->async_send_request(request);
    ASSERT_EQ(
      rclcpp::spin_until_future_complete(node_, result),
      rclcpp::FutureReturnCode::SUCCESS);
    auto result_get = result.get();
    ASSERT_EQ(result_get->results.size(), test_data.size());
    ASSERT_TRUE(result_get->results[0].successful);
    ASSERT_FALSE(result_get->results[1].successful);
    ASSERT_TRUE(result_get->results[2].successful);
  }
}
