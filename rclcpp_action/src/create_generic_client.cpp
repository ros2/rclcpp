// Copyright 2025 Sony Group Corporation.
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
#include <sstream>
#include <string>
#include <tuple>

#include "rclcpp/node.hpp"
#include "rclcpp/typesupport_helpers.hpp"
#include "rcpputils/shared_library.hpp"
#include "rosidl_runtime_c/action_type_support_struct.h"

#include "rclcpp_action/create_generic_client.hpp"
#include "rclcpp_action/generic_client.hpp"

namespace rclcpp_action
{

namespace
{
// Local implementation of get_action_typesupport_handle for jazzy compatibility.
// This function is available in rclcpp on rolling but not on jazzy.
const rosidl_action_type_support_t * get_action_typesupport_handle(
  const std::string & type,
  const std::string & typesupport_identifier,
  rcpputils::SharedLibrary & library)
{
  std::string package_name;
  std::string middle_module;
  std::string type_name;
  std::tie(package_name, middle_module, type_name) = rclcpp::extract_type_identifier(type);

  if (middle_module.empty()) {
    middle_module = "action";
  }

  auto mk_error = [&package_name, &type_name](auto reason) {
      std::stringstream rcutils_dynamic_loading_error;
      rcutils_dynamic_loading_error <<
        "Something went wrong loading the typesupport library for action type " <<
        package_name << "/" << type_name << ". " << reason;
      return rcutils_dynamic_loading_error.str();
    };

  try {
    std::string symbol_name = typesupport_identifier + "__get_action_type_support_handle__" +
      package_name + "__" + middle_module + "__" + type_name;
    const rosidl_action_type_support_t * (* get_ts)() = nullptr;
    // This will throw runtime_error if the symbol was not found.
    get_ts = reinterpret_cast<decltype(get_ts)>(library.get_symbol(symbol_name));
    return get_ts();
  } catch (std::runtime_error &) {
    throw std::runtime_error{mk_error("Library could not be found.")};
  }
}
}  // namespace

typename GenericClient::SharedPtr
create_generic_client(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
  const std::string & name,
  const std::string & type,
  rclcpp::CallbackGroup::SharedPtr group,
  const rcl_action_client_options_t & options)
{
  std::weak_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> weak_node =
    node_waitables_interface;
  std::weak_ptr<rclcpp::CallbackGroup> weak_group = group;
  bool group_is_null = (nullptr == group.get());

  auto deleter = [weak_node, weak_group, group_is_null](GenericClient * ptr)
    {
      if (nullptr == ptr) {
        return;
      }
      auto shared_node = weak_node.lock();
      if (shared_node) {
        // API expects a shared pointer, give it one with a deleter that does nothing.
        std::shared_ptr<GenericClient> fake_shared_ptr(ptr, [](GenericClient *) {});

        if (group_is_null) {
          // Was added to default group
          shared_node->remove_waitable(fake_shared_ptr, nullptr);
        } else {
          // Was added to a specific group
          auto shared_group = weak_group.lock();
          if (shared_group) {
            shared_node->remove_waitable(fake_shared_ptr, shared_group);
          }
        }
      }
      delete ptr;
    };

  auto typesupport_lib = rclcpp::get_typesupport_library(type, "rosidl_typesupport_cpp");
  auto action_typesupport_handle = get_action_typesupport_handle(
    type, "rosidl_typesupport_cpp", *typesupport_lib);

  std::shared_ptr<GenericClient> action_client(
    new GenericClient(
      node_base_interface,
      node_graph_interface,
      node_logging_interface,
      name,
      typesupport_lib,
      action_typesupport_handle,
      options),
    deleter);

  node_waitables_interface->add_waitable(action_client, group);
  return action_client;
}

}  // namespace rclcpp_action
