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

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_resource.hpp"

#include "component_manager.hpp"
#include "filesystem_helper.hpp"
#include "split.hpp"

using namespace std::placeholders;

namespace rclcpp_components
{

ComponentManager::ComponentManager(std::weak_ptr<rclcpp::executor::Executor> executor)
: Node("ComponentManager"),
  executor_(executor)
{
  loadNode_srv_ = create_service<LoadNode>("~/_container/load_node",
      std::bind(&ComponentManager::OnLoadNode, this, _1, _2, _3));
  unloadNode_srv_ = create_service<UnloadNode>("~/_container/unload_node",
      std::bind(&ComponentManager::OnUnloadNode, this, _1, _2, _3));
  listNodes_srv_ = create_service<ListNodes>("~/_container/list_nodes",
      std::bind(&ComponentManager::OnListNodes, this, _1, _2, _3));
}

void
ComponentManager::OnLoadNode(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<LoadNode::Request> request,
  std::shared_ptr<LoadNode::Response> response)
{
  (void) request_header;

  std::string content;
  std::string base_path;
  if (!ament_index_cpp::get_resource(
      "rclcpp_components", request->package_name, content, &base_path))
  {
    RCLCPP_ERROR(get_logger(), "Could not find requested resource in ament index");
    response->error_message = "Could not find requested resource in ament index";
    response->success = false;
    return;
  }

  std::vector<std::string> lines = split(content, '\n', true);
  for (const auto &  line : lines) {
    std::vector<std::string> parts = split(line, ';');
    if (parts.size() != 2) {
      RCLCPP_ERROR(get_logger(), "Invalid resource entry");
      response->error_message = "Invalid resource entry";
      response->success = false;
      return;
    }
    // match plugin name with the same rmw suffix as this executable
    if (parts[0] != request->plugin_name) {
      continue;
    }

    std::string class_name = parts[0];
    std::string fq_class_name = "rclcpp_components::NodeFactoryTemplate<" + class_name + ">";

    // load node plugin
    std::string library_path = parts[1];
    if (!fs::path(library_path).is_absolute()) {
      library_path = base_path + "/" + library_path;
    }
    class_loader::ClassLoader * loader;

    if (loaders_.find(library_path) != loaders_.end()) {
      loader = loaders_[library_path].get();
    } else {
      RCLCPP_INFO(get_logger(), "Load library %s", library_path.c_str());
      try {
        loader = new class_loader::ClassLoader(library_path);
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(get_logger(), "Failed to load library: %s", ex.what());
        response->error_message = "Failed to load library";
        response->success = false;
        return;
      } catch (...) {
        RCLCPP_ERROR(get_logger(), "Failed to load library");
        response->error_message = "Failed to load library";
        response->success = false;
        return;
      }
    }
    auto classes = loader->getAvailableClasses<rclcpp_components::NodeFactory>();
    for (const auto & clazz : classes) {
      RCLCPP_INFO(get_logger(), "Found class %s", class_name.c_str());
      if (clazz == class_name || clazz == fq_class_name) {
        RCLCPP_INFO(get_logger(), "Instantiate class %s", class_name.c_str());

        loaders_[library_path] = loader;

        auto node_factory = loader->createInstance<rclcpp_components::NodeFactory>(clazz);

        std::vector<rclcpp::Parameter> parameters;
        for (const auto & p : request->parameters) {
          parameters.push_back(rclcpp::Parameter::from_parameter_msg(p));
        }

        std::vector<std::string> remap_rules {request->remap_rules};

        if (!request->node_name.empty()) {
          remap_rules.push_back("__node:=" + request->node_name);
        }

        if (!request->node_namespace.empty()) {
          remap_rules.push_back("__ns:=" + request->node_namespace);
        }

        auto options = rclcpp::NodeOptions()
          .initial_parameters(parameters)
          .arguments(remap_rules);

        auto node_id = unique_id++;

        node_wrappers_[node_id] = node_factory->create_node_instance(options);

        auto node = node_wrappers_[node_id].get_node_base_interface();
        if (auto exec = executor_.lock()) {
          exec->add_node(node, true);
        }
        response->full_node_name = node->get_fully_qualified_name();
        response->unique_id = node_id;
        response->success = true;
        return;
      }
    }

    // no matching class found in loader
    delete loader;
    RCLCPP_ERROR(
      get_logger(), "Failed to find class with the requested plugin name '%s' in "
      "the loaded library",
      request->plugin_name.c_str());
    response->success = false;
    return;
  }

  RCLCPP_ERROR(
    get_logger(), "Failed to find plugin name '%s' in prefix '%s'",
    request->plugin_name.c_str(), base_path.c_str());
  response->success = false;
}

void
ComponentManager::OnUnloadNode(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<UnloadNode::Request> request,
  std::shared_ptr<UnloadNode::Response> response)
{
  (void) request_header;

  auto wrapper = node_wrappers_.find(request->unique_id);

  if (wrapper == node_wrappers_.end()) {
    response->success = false;
    std::stringstream ss;
    ss << "No node found with unique_id: " << request->unique_id;
    response->error_message = ss.str();
    RCLCPP_WARN(get_logger(), ss.str());
  } else {
    if (auto exec = executor_.lock()) {
      exec->remove_node(wrapper->second.get_node_base_interface());
    }
    node_wrappers_.erase(wrapper);
    response->success = true;
  }
}

void
ComponentManager::OnListNodes(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<ListNodes::Request> request,
  std::shared_ptr<ListNodes::Response> response)
{
  (void) request_header;
  (void) request;

  for (auto & wrapper : node_wrappers_) {
    response->unique_ids.push_back(wrapper.first);
    response->full_node_names.push_back(
      wrapper.second.get_node_base_interface()->get_fully_qualified_name());
  }
}

}  // namespace rclcpp_components
