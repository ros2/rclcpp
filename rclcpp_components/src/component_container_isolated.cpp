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

#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_components/component_manager_isolated.hpp"

int main(int argc, char * argv[])
{
  /// Component container with dedicated single-threaded executors for each components.
  rclcpp::init(argc, argv);
  // parse arguments
  bool use_multi_threaded_executor{false};
  std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
  for (auto & arg : args) {
    if (arg == std::string("--use_multi_threaded_executor")) {
      use_multi_threaded_executor = true;
    }
  }
  // create executor and component manager
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  rclcpp::Node::SharedPtr node;
  if (use_multi_threaded_executor) {
    using ComponentManagerIsolated =
      rclcpp_components::ComponentManagerIsolated<rclcpp::executors::MultiThreadedExecutor>;
    node = std::make_shared<ComponentManagerIsolated>(exec);
  } else {
    using ComponentManagerIsolated =
      rclcpp_components::ComponentManagerIsolated<rclcpp::executors::SingleThreadedExecutor>;
    node = std::make_shared<ComponentManagerIsolated>(exec);
  }
  exec->add_node(node);
  exec->spin();
}
