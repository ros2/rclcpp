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
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_components/component_manager_isolated.hpp"

int main(int argc, char * argv[])
{
  /// Component container with dedicated executor for each component
  rclcpp::init(argc, argv);

  // parse arguments
  // valid entries: --executor-type single-threaded, --executor-type multi-threaded, --executor-type events
  std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);

  std::string executor_type = "single-threaded";  // default
  for (size_t i = 0; i < args.size(); ++i) {
    if (args[i] == "--executor-type") {
      if (i + 1 < args.size()) {
        executor_type = args[i + 1];
        break;
      }
    } else if (args[i] == "--use_multi_threaded_executor") { // backward compatibility
      executor_type = "multi-threaded";
    }
  }

  // create executor and component manager
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<rclcpp::Executor> exec;
  if (executor_type == "events") {
    using executor = rclcpp::experimental::executors::EventsExecutor;
    using ComponentManagerIsolated = rclcpp_components::ComponentManagerIsolated<executor>;
    exec = std::make_shared<executor>();
    node = std::make_shared<ComponentManagerIsolated>(exec);
  } else if (executor_type == "multi-threaded") {
    using executor = rclcpp::executors::MultiThreadedExecutor;
    using ComponentManagerIsolated = rclcpp_components::ComponentManagerIsolated<executor>;
    exec = std::make_shared<executor>();
    node = std::make_shared<ComponentManagerIsolated>(exec);
  } else if (executor_type == "single-threaded") {
    using executor = rclcpp::executors::SingleThreadedExecutor;
    using ComponentManagerIsolated = rclcpp_components::ComponentManagerIsolated<executor>;
    exec = std::make_shared<executor>();
    node = std::make_shared<ComponentManagerIsolated>(exec);
  } else {
    std::cerr << "Invalid executor type: " << executor_type << std::endl;
    return 1;
  }

  exec->add_node(node);
  exec->spin();
}
