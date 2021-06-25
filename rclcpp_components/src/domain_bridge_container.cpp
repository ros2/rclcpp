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

#include "domain_bridge/domain_bridge.hpp"
#include "domain_bridge/parse_domain_bridge_yaml_config.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rclcpp_components/component_manager.hpp"

int main(int argc, char * argv[])
{
  // Domain bridge container with a single-threaded executor
  rclcpp::init(argc, argv);
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto node = std::make_shared<rclcpp_components::ComponentManager>(exec);

  if (argc < 2) {
    std::cerr << "error: Expected YAML config file" << std::endl;
    return 1;
  }

  std::string yaml_config = argv[1];
  domain_bridge::DomainBridgeConfig domain_bridge_config =
    domain_bridge::parse_domain_bridge_yaml_config(yaml_config);

  domain_bridge::DomainBridge domain_bridge(domain_bridge_config);

  exec->add_node(node);
  domain_bridge.add_to_executor(*exec);
  exec->spin();
}
