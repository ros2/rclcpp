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

#include <memory>
#include <string>
#include <vector>

#include "class_loader/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory.hpp"


#define LINKTIME_COMPOSITION_LOGGER_NAME "node_main"

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::Logger logger = rclcpp::get_logger(LINKTIME_COMPOSITION_LOGGER_NAME);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  std::vector<class_loader::ClassLoader *> loaders;
  std::vector<rclcpp_components::NodeInstanceWrapper> node_wrappers;

  std::vector<std::string> libraries = {
    // all classes from libraries linked by the linker (rather then dlopen)
    // are registered under the library_path ""
    "",
  };
  for (auto library : libraries) {
    RCLCPP_INFO(logger, "Load library %s", library.c_str());
    auto loader = new class_loader::ClassLoader(library);
    auto classes = loader->getAvailableClasses<rclcpp_components::NodeFactory>();
    for (auto clazz : classes) {
      RCLCPP_INFO(logger, "Instantiate class %s", clazz.c_str());
      auto node_factory = loader->createInstance<rclcpp_components::NodeFactory>(clazz);
      auto wrapper = node_factory->create_node_instance(options);
      auto node = wrapper.get_node_base_interface();
      node_wrappers.push_back(wrapper);
      exec.add_node(node);
    }
    loaders.push_back(loader);
  }

  exec.spin();

  for (auto wrapper : node_wrappers) {
    exec.remove_node(wrapper.get_node_base_interface());
  }
  node_wrappers.clear();

  rclcpp::shutdown();

  return 0;
}
