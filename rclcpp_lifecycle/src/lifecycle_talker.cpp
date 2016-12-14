// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "std_msgs/msg/string.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::node::LifecycleNode> lc_node = std::make_shared<rclcpp::node::LifecycleNode>("lc_talker");
 
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 7;

  std::shared_ptr<rclcpp::publisher::LifecyclePublisher<std_msgs::msg::String>> chatter_pub =
    lc_node->create_publisher<std_msgs::msg::String>("chatter", custom_qos_profile);

  lc_node->print_state_machine();

  if (!lc_node->on_configure())
  {
    printf("Could not configure node. Going to error state.\n");
  }

  rclcpp::WallRate loop_rate(2);

  auto msg = std::make_shared<std_msgs::msg::String>();
  auto i = 1;

  while (rclcpp::ok() && i <= 10) {
    msg->data = "Hello World: " + std::to_string(i++);
    //std::cout << "Publishing: '" << msg->data << "'" << std::endl;
    chatter_pub->publish(msg);
    rclcpp::spin_some(lc_node);
    loop_rate.sleep();
  }

  printf("Calling activate\n");
  if (!lc_node->on_activate())
  {
    return -1;
  }

  while (rclcpp::ok() && i <= 20) {
    msg->data = "Hello World: " + std::to_string(i++);
    //std::cout << "Publishing: '" << msg->data << "'" << std::endl;
    chatter_pub->publish(msg);
    rclcpp::spin_some(lc_node);
    loop_rate.sleep();
  }

  printf("Calling deactivate\n");
  if (!lc_node->on_deactivate())
  {
    return -1;
  }

  while (rclcpp::ok() && i <= 30) {
    msg->data = "Hello World: " + std::to_string(i++);
    //std::cout << "Publishing: '" << msg->data << "'" << std::endl;
    chatter_pub->publish(msg);
    rclcpp::spin_some(lc_node);
    loop_rate.sleep();
  }
  return 0;
  
}
