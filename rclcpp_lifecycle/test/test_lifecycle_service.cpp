// Copyright 2021 Open Source Robotics Foundation, Inc.
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
#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_service.hpp"
#include "test_msgs/srv/empty.hpp"

using namespace std::literals::chrono_literals;

class TestLifecycleService : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
    node_ = rclcpp_lifecycle::LifecycleNode::make_shared("node");
    service_ = node_->create_service<test_msgs::srv::Empty>(
      std::string("service"), request_callback_.AsStdFunction());
    client_ = node_->create_client<test_msgs::srv::Empty>(std::string("service"));
  }

  void TearDown() {rclcpp::shutdown();}

protected:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp_lifecycle::LifecycleService<test_msgs::srv::Empty>::SharedPtr service_;
  rclcpp::Client<test_msgs::srv::Empty>::SharedPtr client_;

  ::testing::MockFunction<void(
      const test_msgs::srv::Empty::Request::SharedPtr, test_msgs::srv::Empty::Response::SharedPtr)>
  request_callback_;
};

TEST_F(TestLifecycleService, request_on_active_service)
{
  service_->on_activate();
  EXPECT_TRUE(service_->is_activated());

  ::testing::MockFunction<void(std::shared_future<std::shared_ptr<test_msgs::srv::Empty_Response>>)>
  response_callback;

  auto request = std::make_shared<test_msgs::srv::Empty::Request>();
  client_->wait_for_service();
  auto future = client_->async_send_request(request, response_callback.AsStdFunction());

  EXPECT_CALL(request_callback_, Call(::testing::_, ::testing::_)).WillOnce(::testing::Return());
  EXPECT_CALL(response_callback, Call(::testing::_)).WillOnce(::testing::Return());

  rclcpp::spin_until_future_complete(node_->get_node_base_interface(), future, 1s);
}

TEST_F(TestLifecycleService, request_on_inactive_service)
{
  service_->on_deactivate();
  EXPECT_FALSE(service_->is_activated());

  ::testing::MockFunction<void(std::shared_future<std::shared_ptr<test_msgs::srv::Empty_Response>>)>
  response_callback;

  auto request = std::make_shared<test_msgs::srv::Empty::Request>();
  client_->wait_for_service();
  auto future = client_->async_send_request(request, response_callback.AsStdFunction());

  EXPECT_CALL(request_callback_, Call(::testing::_, ::testing::_)).Times(0);
  EXPECT_CALL(response_callback, Call(::testing::_)).Times(0);

  rclcpp::spin_until_future_complete(node_->get_node_base_interface(), future, 1s);
}
