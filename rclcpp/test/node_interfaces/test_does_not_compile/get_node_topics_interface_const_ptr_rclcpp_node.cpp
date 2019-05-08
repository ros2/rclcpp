// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_interfaces/get_node_topics_interface.hpp"

int main(void)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  std::shared_ptr<const rclcpp::Node> const_node_ptr = node;
  // Should fail because a const node cannot have a non-const method called on it.
  rclcpp::node_interfaces::NodeTopicsInterface * result =
    rclcpp::node_interfaces::get_node_topics_interface(const_node_ptr);
  (void)result;
}
