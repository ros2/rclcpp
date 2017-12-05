// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/executors.hpp"

void
rclcpp::spin_some(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr)
{
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.spin_node_some(node_ptr);
}

void
rclcpp::spin_some(rclcpp::Node::SharedPtr node_ptr)
{
  rclcpp::spin_some(node_ptr->get_node_base_interface());
}

void
rclcpp::spin(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr)
{
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_ptr);
  exec.spin();
  exec.remove_node(node_ptr);
}

void
rclcpp::spin(rclcpp::Node::SharedPtr node_ptr)
{
  rclcpp::spin(node_ptr->get_node_base_interface());
}
