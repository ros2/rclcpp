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
  auto vargv = rclcpp::init_and_remove_ros_arguments(argc, argv);
  size_t number_of_threads{0};
  rclcpp::Logger logger{rclcpp::get_logger("component_container_mt")};

  if (vargv.size() == 3 && (vargv.at(1) == "--thread-num" || vargv.at(1) == "-t")) {
    try {
      number_of_threads = static_cast<size_t>(std::stoi(vargv.at(2)));
      RCLCPP_INFO_STREAM(logger, "number of threads: " << number_of_threads);
    } catch (const std::invalid_argument & ex) {
      RCLCPP_ERROR_STREAM(logger, ex.what());
    } catch (const std::out_of_range & ex) {
      RCLCPP_ERROR_STREAM(logger, ex.what());
    }
  }

  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
    rclcpp::ExecutorOptions(), number_of_threads);
  auto node = std::make_shared<rclcpp_components::ComponentManager>(exec);
  exec->add_node(node);
  exec->spin();
}
