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
  /// Component container with a single-threaded executor.
  rclcpp::init(argc, argv);
<<<<<<< HEAD
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto node = std::make_shared<rclcpp_components::ComponentManager>(exec);
=======

  std::vector<std::string> arg_strs = rclcpp::remove_ros_arguments(argc, argv);
  auto args = parse_args(arg_strs);

  if (args.help) {
    print_usage();
    return 0;
  }

  if (args.invalid) {
    RCUTILS_LOG_ERROR_NAMED("component_container", "%s", args.error_message.c_str());
    print_usage();
    return 1;
  }

  // Declare the executor after the manager, so on destruction the executor is
  // destroyed first. Otherwise, the executor could release component entities
  // after the manager has unloaded their libraries.
  std::shared_ptr<rclcpp_components::ComponentManager> node;
  std::shared_ptr<rclcpp::Executor> exec;

  // Create the manager once, as its final type. The isolated managers resolve the
  // `thread_num` parameter themselves when a component is loaded (0 == auto).
  if (args.isolated) {
    switch (args.executor_type) {
      case ExecutorType::MultiThreaded:
        node = std::make_shared<
          rclcpp_components::ComponentManagerIsolated<rclcpp::executors::MultiThreadedExecutor>>(
          rclcpp::ExecutorOptions(), 0);
        break;
      case ExecutorType::EventsCBG:
        node = std::make_shared<
          rclcpp_components::ComponentManagerIsolated<rclcpp::executors::EventsCBGExecutor>>(
          rclcpp::ExecutorOptions(), 0);
        break;
      default:
        node = std::make_shared<
          rclcpp_components::ComponentManagerIsolated<rclcpp::executors::SingleThreadedExecutor>>();
        break;
    }
  } else {
    node = std::make_shared<rclcpp_components::ComponentManager>();
  }

  const int64_t num_threads = (node->has_parameter("thread_num")) ?
    node->get_parameter("thread_num").as_int() :
    std::thread::hardware_concurrency();

  if (args.executor_type == ExecutorType::SingleThreaded &&
    num_threads > 0 &&
    num_threads != std::thread::hardware_concurrency())
  {
    RCUTILS_LOG_WARN_NAMED("component_container",
      "thread_num is not supported by the SingleThreadedExecutor. Ignoring...");
  }

  std::string debug_msg;
  if (args.isolated) {
    // The outer executor runs only the container manager's load/unload services.
    // Each loaded component gets its own dedicated executor of the requested type.
    exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    debug_msg = "Creating isolated component container with the following per-node settings: "
      "executor_type: " + executor_type_to_string(args.executor_type);
    if (args.executor_type != ExecutorType::SingleThreaded) {
      debug_msg += ", num_threads: " + std::to_string(num_threads);
    }
  } else {
    exec = make_executor(args.executor_type, rclcpp::ExecutorOptions(), num_threads);

    debug_msg = "Creating non-isolated component container with executor_type: " +
      executor_type_to_string(args.executor_type);
    if (args.executor_type != ExecutorType::SingleThreaded) {
      debug_msg += ", num_threads: " + std::to_string(num_threads);
    }
  }

  RCUTILS_LOG_DEBUG_NAMED("component_container", "%s", debug_msg.c_str());

  node->set_executor(exec);
>>>>>>> efd18d6 (Destroy the executor before unloading component libraries (#3319))
  exec->add_node(node);
  exec->spin();
}
