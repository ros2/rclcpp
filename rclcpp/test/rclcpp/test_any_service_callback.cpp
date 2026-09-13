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
#include "rclcpp/service.hpp"
#include "test_msgs/srv/basic_types.hpp"
#include "test_msgs/srv/empty.hpp"

class TestAnyServiceCallback : public ::testing::Test
{
public:
  void SetUp()
  {
    request_header_ = std::make_shared<rmw_request_id_t>();
    request_ = std::make_shared<test_msgs::srv::Empty::Request>();
    response_ = std::make_shared<test_msgs::srv::Empty::Response>();
  }

protected:
  rclcpp::AnyServiceCallback<test_msgs::srv::Empty> any_service_callback_;
  std::shared_ptr<rmw_request_id_t> request_header_;
  std::shared_ptr<test_msgs::srv::Empty::Request> request_;
  std::shared_ptr<test_msgs::srv::Empty::Response> response_;
};

TEST_F(TestAnyServiceCallback, no_set_and_dispatch_throw) {
  EXPECT_THROW(
    any_service_callback_.dispatch(nullptr, request_header_, request_),
    std::runtime_error);
}

TEST_F(TestAnyServiceCallback, set_and_dispatch_no_header) {
  int callback_calls = 0;
  auto callback = [&callback_calls](
    const std::shared_ptr<test_msgs::srv::Empty::Request>,
    std::shared_ptr<test_msgs::srv::Empty::Response>)
    {
      callback_calls++;
    };

  any_service_callback_.set(callback);
  EXPECT_NO_THROW(
    EXPECT_NE(nullptr, any_service_callback_.dispatch(nullptr, request_header_, request_)));
  EXPECT_EQ(callback_calls, 1);
}


TEST_F(TestAnyServiceCallback, set_and_dispatch_header) {
  int callback_with_header_calls = 0;
  auto callback_with_header = [&callback_with_header_calls](
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<test_msgs::srv::Empty::Request>,
    std::shared_ptr<test_msgs::srv::Empty::Response>)
    {
      callback_with_header_calls++;
    };

  any_service_callback_.set(callback_with_header);
  EXPECT_NO_THROW(
    EXPECT_NE(nullptr, any_service_callback_.dispatch(nullptr, request_header_, request_)));
  EXPECT_EQ(callback_with_header_calls, 1);
}

TEST_F(TestAnyServiceCallback, set_and_dispatch_defered) {
  int callback_with_header_calls = 0;
  auto callback_with_header = [&callback_with_header_calls](
    const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<test_msgs::srv::Empty::Request>)
    {
      callback_with_header_calls++;
    };

  any_service_callback_.set(callback_with_header);
  EXPECT_NO_THROW(
    EXPECT_EQ(nullptr, any_service_callback_.dispatch(nullptr, request_header_, request_)));
  EXPECT_EQ(callback_with_header_calls, 1);
}

TEST_F(TestAnyServiceCallback, set_and_dispatch_defered_with_service_handle) {
  int callback_with_header_calls = 0;
  auto callback_with_header = [&callback_with_header_calls](
    std::shared_ptr<rclcpp::Service<test_msgs::srv::Empty>>,
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<test_msgs::srv::Empty::Request>)
    {
      callback_with_header_calls++;
    };

  any_service_callback_.set(callback_with_header);
  EXPECT_NO_THROW(
    EXPECT_EQ(nullptr, any_service_callback_.dispatch(nullptr, request_header_, request_)));
  EXPECT_EQ(callback_with_header_calls, 1);
}

TEST_F(TestAnyServiceCallback, set_and_dispatch_const_ref_no_header) {
  int callback_calls = 0;
  auto callback = [&callback_calls](
    const test_msgs::srv::Empty::Request &,
    test_msgs::srv::Empty::Response &)
    {
      callback_calls++;
    };

  any_service_callback_.set(callback);
  EXPECT_NO_THROW(
    EXPECT_NE(nullptr, any_service_callback_.dispatch(nullptr, request_header_, request_)));
  EXPECT_EQ(callback_calls, 1);
}

TEST_F(TestAnyServiceCallback, set_and_dispatch_const_ref_header) {
  int callback_calls = 0;
  int64_t seen_sequence_number = 0;
  request_header_->sequence_number = 42;
  auto callback = [&callback_calls, &seen_sequence_number](
    const rmw_request_id_t & request_header,
    const test_msgs::srv::Empty::Request &,
    test_msgs::srv::Empty::Response &)
    {
      callback_calls++;
      seen_sequence_number = request_header.sequence_number;
    };

  any_service_callback_.set(callback);
  EXPECT_NO_THROW(
    EXPECT_NE(nullptr, any_service_callback_.dispatch(nullptr, request_header_, request_)));
  EXPECT_EQ(callback_calls, 1);
  EXPECT_EQ(seen_sequence_number, 42);
}

TEST_F(TestAnyServiceCallback, const_ref_response_is_returned) {
  rclcpp::AnyServiceCallback<test_msgs::srv::BasicTypes> any_service_callback;
  auto request = std::make_shared<test_msgs::srv::BasicTypes::Request>();
  request->int64_value = 7;
  request->string_value = "ping";

  auto callback = [](
    const test_msgs::srv::BasicTypes::Request & req,
    test_msgs::srv::BasicTypes::Response & res)
    {
      res.int64_value = req.int64_value * 2;
      res.string_value = req.string_value + "-pong";
    };

  any_service_callback.set(callback);
  auto response = any_service_callback.dispatch(nullptr, request_header_, request);
  ASSERT_NE(nullptr, response);
  EXPECT_EQ(response->int64_value, 14);
  EXPECT_EQ(response->string_value, "ping-pong");
}

TEST_F(TestAnyServiceCallback, set_and_dispatch_const_ref_std_bind) {
  struct Handler
  {
    int calls = 0;
    void no_header(const test_msgs::srv::Empty::Request &, test_msgs::srv::Empty::Response &)
    {
      calls++;
    }
    void header(
      const rmw_request_id_t &,
      const test_msgs::srv::Empty::Request &,
      test_msgs::srv::Empty::Response &)
    {
      calls++;
    }
  };
  Handler handler;

  any_service_callback_.set(
    std::bind(&Handler::no_header, &handler, std::placeholders::_1, std::placeholders::_2));
  EXPECT_NE(nullptr, any_service_callback_.dispatch(nullptr, request_header_, request_));

  rclcpp::AnyServiceCallback<test_msgs::srv::Empty> any_service_callback_with_header;
  any_service_callback_with_header.set(
    std::bind(
      &Handler::header, &handler,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  EXPECT_NE(
    nullptr, any_service_callback_with_header.dispatch(nullptr, request_header_, request_));
  EXPECT_EQ(handler.calls, 2);
}

TEST_F(TestAnyServiceCallback, const_ref_dispatch_null_arguments_throw) {
  any_service_callback_.set(
    [](const test_msgs::srv::Empty::Request &, test_msgs::srv::Empty::Response &) {});
  EXPECT_THROW(
    any_service_callback_.dispatch(nullptr, request_header_, nullptr),
    std::runtime_error);

  rclcpp::AnyServiceCallback<test_msgs::srv::Empty> any_service_callback_with_header;
  any_service_callback_with_header.set(
    [](
      const rmw_request_id_t &,
      const test_msgs::srv::Empty::Request &,
      test_msgs::srv::Empty::Response &) {});
  EXPECT_THROW(
    any_service_callback_with_header.dispatch(nullptr, nullptr, request_),
    std::runtime_error);
  EXPECT_THROW(
    any_service_callback_with_header.dispatch(nullptr, request_header_, nullptr),
    std::runtime_error);
}
