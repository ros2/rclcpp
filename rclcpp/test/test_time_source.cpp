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

#include <algorithm>
#include <limits>
#include <memory>
#include <string>

#include "rcl/error_handling.h"
#include "rcl/time.h"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/time_source.hpp"

class TestTimeSource : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp()
  {
    node = std::make_shared<rclcpp::node::Node>("my_node");
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::node::Node::SharedPtr node;
};


TEST_F(TestTimeSource, detachUnattached) {
  rclcpp::TimeSource ts;

  ASSERT_NO_THROW(ts.detachNode());

  // Try multiple detach to see if error
  ASSERT_NO_THROW(ts.detachNode());
}

TEST_F(TestTimeSource, reattach) {
  rclcpp::TimeSource ts;
  // Try reattach
  ASSERT_NO_THROW(ts.attachNode(node));
  ASSERT_NO_THROW(ts.attachNode(node));
}

TEST_F(TestTimeSource, ROS_time_valid) {
  rclcpp::TimeSource ts;
  // Try reattach
  // TODO(tfoote) reenabled using external clock
  auto ros_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  EXPECT_FALSE(ros_clock->isROSTimeActive());
  ts.attachClock(ros_clock);
  auto now = ros_clock->now();

  EXPECT_FALSE(ros_clock->isROSTimeActive());

  ts.attachNode(node);

  EXPECT_FALSE(ros_clock->isROSTimeActive());
}

TEST_F(TestTimeSource, clock) {
  rclcpp::TimeSource ts(node);
  auto ros_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  EXPECT_FALSE(ros_clock->isROSTimeActive());
  ts.attachClock(ros_clock);
  EXPECT_FALSE(ros_clock->isROSTimeActive());

  // builtin_interfaces::msg::Time::SharedPtr last_msg;
  // auto clock_sub = node->create_subscription<builtin_interfaces::msg::Time>(
  //   "clock", [&](builtin_interfaces::msg::Time::SharedPtr msg) {last_msg = msg;},
  //   rmw_qos_profile_default);

  auto clock_pub = node->create_publisher<builtin_interfaces::msg::Time>("clock",
      rmw_qos_profile_default);

  rclcpp::WallRate loop_rate(10);
  for (int i = 0; i < 10; ++i) {
    if (!rclcpp::ok()) {
      break;  // Break for ctrl-c
    }
    auto msg = std::make_shared<builtin_interfaces::msg::Time>();
    msg->sec = i;
    msg->nanosec = 1000;
    clock_pub->publish(msg);
    // std::cout << "Publishing: '" << msg->sec << ".000000" << msg->nanosec << "'" << std::endl;
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  auto t_low = rclcpp::Time(1, 0, RCL_ROS_TIME);
  auto t_high = rclcpp::Time(10, 100000, RCL_ROS_TIME);

  // Now that we've recieved a message it should be active with parameter unset
  EXPECT_TRUE(ros_clock->isROSTimeActive());

  auto t_out = ros_clock->now();

  EXPECT_NE(0, t_out.nanoseconds());
  EXPECT_LT(t_low.nanoseconds(), t_out.nanoseconds());
  EXPECT_GT(t_high.nanoseconds(), t_out.nanoseconds());
}

TEST_F(TestTimeSource, parameter_activation) {
  rclcpp::TimeSource ts(node);
  auto ros_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  EXPECT_FALSE(ros_clock->isROSTimeActive());

  ts.attachClock(ros_clock);
  EXPECT_FALSE(ros_clock->isROSTimeActive());

  auto parameter_service = std::make_shared<rclcpp::parameter_service::ParameterService>(node);
  auto parameters_client = std::make_shared<rclcpp::parameter_client::SyncParametersClient>(node);

  using namespace std::chrono_literals;
  EXPECT_TRUE(parameters_client->wait_for_service(2s));
  auto set_parameters_results = parameters_client->set_parameters({
    rclcpp::parameter::ParameterVariant("use_sim_time", true)
  });
  for (auto & result : set_parameters_results) {
    EXPECT_TRUE(result.successful);
  }
  rclcpp::spin_some(node);
  EXPECT_TRUE(ros_clock->isROSTimeActive());


  set_parameters_results = parameters_client->set_parameters({
    rclcpp::parameter::ParameterVariant("use_sim_time", rclcpp::parameter::PARAMETER_NOT_SET)
  });
  for (auto & result : set_parameters_results) {
    EXPECT_TRUE(result.successful);
  }
  EXPECT_TRUE(ros_clock->isROSTimeActive());

  set_parameters_results = parameters_client->set_parameters({
    rclcpp::parameter::ParameterVariant("use_sim_time", false)
  });
  for (auto & result : set_parameters_results) {
    EXPECT_TRUE(result.successful);
  }
  EXPECT_FALSE(ros_clock->isROSTimeActive());

  set_parameters_results = parameters_client->set_parameters({
    rclcpp::parameter::ParameterVariant("use_sim_time", rclcpp::parameter::PARAMETER_NOT_SET)
  });
  for (auto & result : set_parameters_results) {
    EXPECT_TRUE(result.successful);
  }
  EXPECT_FALSE(ros_clock->isROSTimeActive());
}
