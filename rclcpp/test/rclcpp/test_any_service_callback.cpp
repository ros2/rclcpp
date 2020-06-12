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

// This file includes basic API tests for the AnyServiceCallback class.
// It is also tested in test_externally_defined_services.cpp

#include <gtest/gtest.h>

#include <functional>
#include <memory>
#include <utility>

#include "rclcpp/any_service_callback.hpp"
#include "test_msgs/srv/empty.hpp"
#include "test_msgs/srv/empty.h"

TEST(TestAnyServiceCallback, set_and_dispatch_throw) {
  rclcpp::AnyServiceCallback<test_msgs::srv::Empty> cb;
  auto request_header = std::make_shared<rmw_request_id_t>();
  auto request = std::make_shared<test_msgs::srv::Empty::Request>();
  auto response = std::make_shared<test_msgs::srv::Empty::Response>();

  EXPECT_THROW(cb.dispatch(request_header, request, response), std::runtime_error);
}

TEST(TestAnyServiceCallback, set_and_dispatch_no_header) {
  rclcpp::AnyServiceCallback<test_msgs::srv::Empty> cb;
  auto request_header = std::make_shared<rmw_request_id_t>();
  auto request = std::make_shared<test_msgs::srv::Empty::Request>();
  auto response = std::make_shared<test_msgs::srv::Empty::Response>();

  int callback_calls = 0;
  auto callback = [&callback_calls](const std::shared_ptr<test_msgs::srv::Empty::Request>,
      std::shared_ptr<test_msgs::srv::Empty::Response>) {
      callback_calls++;
    };

  cb.set(std::forward<decltype(callback)>(callback));
  EXPECT_NO_THROW(
    cb.dispatch(request_header, request, response));
  EXPECT_EQ(callback_calls, 1);
}


TEST(TestAnyServiceCallback, set_and_dispatch_header) {
  rclcpp::AnyServiceCallback<test_msgs::srv::Empty> cb;
  auto request_header = std::make_shared<rmw_request_id_t>();
  auto request = std::make_shared<test_msgs::srv::Empty::Request>();
  auto response = std::make_shared<test_msgs::srv::Empty::Response>();

  int callback_with_header_calls = 0;
  auto callback_with_header =
    [&callback_with_header_calls](const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<test_msgs::srv::Empty::Request>,
      std::shared_ptr<test_msgs::srv::Empty::Response>) {
      callback_with_header_calls++;
    };

  cb.set(std::forward<decltype(callback_with_header)>(callback_with_header));
  EXPECT_NO_THROW(
    cb.dispatch(request_header, request, response));
  EXPECT_EQ(callback_with_header_calls, 1);
}
