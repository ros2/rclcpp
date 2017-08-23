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
#include <string>

#include "rcl/error_handling.h"
#include "rcl/time.h"
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


TEST_F(TestTimeSource, tripwire){
  rclcpp::TimeSource ts(node);
  

}

TEST_F(TestTimeSource, clock)
{
  rclcpp::TimeSource ts(node);
  
  auto clock_pub = node->create_publisher<builtin_interfaces::msg::Time>("clock", rmw_qos_profile_default);
  
  rclcpp::WallRate loop_rate(1);
  for (int i = 0; i < 10; ++i)
  {
    if (!rclcpp::ok()) break; // Break for ctrl-c
    auto msg = std::make_shared<builtin_interfaces::msg::Time>();
    msg->sec = i;
    msg->nanosec = 1000;
    clock_pub->publish(msg);
    rclcpp::spin_some(node);
    std::cout << "Publishing: '" << msg->sec << ".000000" << msg->nanosec << "'" << std::endl;
    loop_rate.sleep();
  }
}
