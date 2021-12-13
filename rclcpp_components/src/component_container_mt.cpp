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

#include "rclcpp_components/component_manager.hpp"

int main(int argc, char * argv[])
{
  /// Component container with a multi-threaded executor.
  rclcpp::init(argc, argv);

  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto node = std::make_shared<rclcpp_components::ComponentManager>();
  if (node->has_parameter("thread_num")) {
    const auto thread_num = node->get_parameter("thread_num").as_int();
    exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions{}, thread_num);
    node->set_executor(exec);
  } else {
    node->set_executor(exec);
  }
  exec->add_node(node);
  exec->spin();
}
