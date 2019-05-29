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

#include "component_manager.hpp"

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_resource.hpp"
#include "rcpputils/filesystem_helper.hpp"
#include "rcpputils/split.hpp"

using namespace std::placeholders;

namespace rclcpp_components
{

ComponentManager::ComponentManager(
  std::weak_ptr<rclcpp::executor::Executor> executor)
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

ComponentManager::~ComponentManager()
{
  if (node_wrappers_.size()) {
    RCLCPP_DEBUG(get_logger(), "Removing components from executor");
    if (auto exec = executor_.lock()) {
      for (auto & wrapper : node_wrappers_) {
        exec->remove_node(wrapper.second.get_node_base_interface());
      }
    }
  }
}

std::vector<ComponentManager::ComponentResource>
ComponentManager::get_component_resources(const std::string & package_name) const
{
  std::string content;
  std::string base_path;
  if (!ament_index_cpp::get_resource(
      "rclcpp_components", package_name, content, &base_path))
  {
    throw ComponentManagerException("Could not find requested resource in ament index");
  }

  std::vector<ComponentResource> resources;
  std::vector<std::string> lines = rcpputils::split(content, '\n', true);
  for (const auto & line : lines) {
    std::vector<std::string> parts = rcpputils::split(line, ';');
    if (parts.size() != 2) {
      throw ComponentManagerException("Invalid resource entry");
    }

    std::string library_path = parts[1];
    if (!rcpputils::fs::path(library_path).is_absolute()) {
      library_path = base_path + "/" + library_path;
    }
    resources.push_back({parts[0], library_path});
  }
  return resources;
}

std::shared_ptr<rclcpp_components::NodeFactory>
ComponentManager::create_component_factory(const ComponentResource & resource)
{
  std::string library_path = resource.second;
  std::string class_name = resource.first;
  std::string fq_class_name = "rclcpp_components::NodeFactoryTemplate<" + class_name + ">";

  class_loader::ClassLoader * loader;
  if (loaders_.find(library_path) == loaders_.end()) {
    RCLCPP_INFO(get_logger(), "Load Library: %s", library_path.c_str());
    try {
      loaders_[library_path] = std::make_unique<class_loader::ClassLoader>(library_path);
    } catch (const std::exception & ex) {
      throw ComponentManagerException("Failed to load library: " + std::string(ex.what()));
    } catch (...) {
      throw ComponentManagerException("Failed to load library");
    }
  }
  loader = loaders_[library_path].get();

  auto classes = loader->getAvailableClasses<rclcpp_components::NodeFactory>();
  for (const auto & clazz : classes) {
    RCLCPP_INFO(get_logger(), "Found class: %s", clazz.c_str());
    if (clazz == class_name || clazz == fq_class_name) {
      RCLCPP_INFO(get_logger(), "Instantiate class: %s", clazz.c_str());
      return loader->createInstance<rclcpp_components::NodeFactory>(clazz);
    }
  }
  return {};
}

void
ComponentManager::OnLoadNode(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<LoadNode::Request> request,
  std::shared_ptr<LoadNode::Response> response)
{
  (void) request_header;

  try {
    auto resources = get_component_resources(request->package_name);

    for (const auto & resource : resources) {
      if (resource.first != request->plugin_name) {
        continue;
      }
      auto factory = create_component_factory(resource);

      if (factory == nullptr) {
        continue;
      }

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
        .use_global_arguments(false)
        .parameter_overrides(parameters)
        .arguments(remap_rules);

      auto node_id = unique_id++;

      if (0 == node_id) {
        // This puts a technical limit on the number of times you can add a component.
        // But even if you could add (and remove) them at 1 kHz (very optimistic rate)
        // it would still be a very long time before you could exhaust the pool of id's:
        //   2^64 / 1000 times per sec / 60 sec / 60 min / 24 hours / 365 days = 584,942,417 years
        // So around 585 million years. Even at 1 GHz, it would take 585 years.
        // I think it's safe to avoid trying to handle overflow.
        // If we roll over then it's most likely a bug.
        throw std::overflow_error("exhausted the unique ids for components in this process");
      }

      try {
        node_wrappers_[node_id] = factory->create_node_instance(options);
      } catch (...) {
        // In the case that the component constructor throws an exception,
        // rethrow into the following catch block.
        throw ComponentManagerException("Component constructor threw an exception");
      }

      auto node = node_wrappers_[node_id].get_node_base_interface();
      if (auto exec = executor_.lock()) {
        exec->add_node(node, true);
      }
      response->full_node_name = node->get_fully_qualified_name();
      response->unique_id = node_id;
      response->success = true;
      return;
    }
    RCLCPP_ERROR(
      get_logger(), "Failed to find class with the requested plugin name '%s' in "
      "the loaded library",
      request->plugin_name.c_str());
    response->error_message = "Failed to find class with the requested plugin name.";
    response->success = false;
  } catch (const ComponentManagerException & ex) {
    RCLCPP_ERROR(get_logger(), ex.what());
    response->error_message = ex.what();
    response->success = false;
  }
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
