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

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "rcl_interfaces/msg/log.hpp"

using namespace std::chrono_literals;

#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

class CLASSNAME (TestRosoutSubscription, RMW_IMPLEMENTATION) : public ::testing::Test
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
    node = std::make_shared<rclcpp::Node>("test_rosout_subscription", "/ns");
    sub = node->create_subscription<rcl_interfaces::msg::Log>(
      "/rosout", 10, [this](rcl_interfaces::msg::Log::ConstSharedPtr msg) {
        if (msg->msg == this->rosout_msg_data &&
        msg->name == this->rosout_msg_name)
        {
          received_msg_promise.set_value(true);
        }
      });
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr sub;
  std::promise<bool> received_msg_promise;
  std::string rosout_msg_data;
  std::string rosout_msg_name;
};

TEST_F(TestRosoutSubscription, test_rosoutsubscription_getchild) {
  std::string logger_name = "ns.test_rosout_subscription.child";
  this->rosout_msg_data = "SOMETHING";
  this->rosout_msg_name = logger_name;
  {
    // before calling get_child of Logger
    {
      RCLCPP_INFO(
        rclcpp::get_logger(logger_name), this->rosout_msg_data.c_str());
      auto future = received_msg_promise.get_future();
      auto return_code = rclcpp::spin_until_future_complete(this->node, future, 3s);
      ASSERT_EQ(rclcpp::FutureReturnCode::TIMEOUT, return_code);
      received_msg_promise = {};
    }

    rclcpp::Logger child_logger = this->node->get_logger().get_child("child");
    ASSERT_EQ(child_logger.get_name(), logger_name);

    // after calling get_child of Logger
    // 1. use child_logger directly
    {
      RCLCPP_INFO(child_logger, this->rosout_msg_data.c_str());
      auto future = received_msg_promise.get_future();
      auto return_code = rclcpp::spin_until_future_complete(this->node, future, 3s);
      ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, return_code);
      EXPECT_TRUE(future.get());
      received_msg_promise = {};
    }

    // 2. use rclcpp::get_logger
    {
      RCLCPP_INFO(rclcpp::get_logger(logger_name), this->rosout_msg_data.c_str());
      auto future = received_msg_promise.get_future();
      auto return_code = rclcpp::spin_until_future_complete(this->node, future, 3s);
      ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, return_code);
      EXPECT_TRUE(future.get());
      received_msg_promise = {};
    }
  }

  // `child_logger` is end of life, there is no sublogger
  {
    RCLCPP_INFO(rclcpp::get_logger(logger_name), this->rosout_msg_data.c_str());
    auto future = received_msg_promise.get_future();
    auto return_code = rclcpp::spin_until_future_complete(this->node, future, 3s);
    ASSERT_EQ(rclcpp::FutureReturnCode::TIMEOUT, return_code);
    received_msg_promise = {};
  }
}

TEST_F(TestRosoutSubscription, test_rosoutsubscription_getchild_hierarchy) {
  std::string logger_name = "ns.test_rosout_subscription.child.grandchild";
  this->rosout_msg_data = "SOMETHING";
  this->rosout_msg_name = logger_name;

  rclcpp::Logger grandchild_logger =
    this->node->get_logger().get_child("child").get_child("grandchild");
  ASSERT_EQ(grandchild_logger.get_name(), logger_name);
  RCLCPP_INFO(grandchild_logger, this->rosout_msg_data.c_str());
  auto future = received_msg_promise.get_future();
  auto return_code = rclcpp::spin_until_future_complete(this->node, future, 3s);
  ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, return_code);
  EXPECT_TRUE(future.get());
  received_msg_promise = {};
}
