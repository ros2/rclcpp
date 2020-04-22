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

#ifndef RCLCPP__CREATE_CLIENT_HPP_
#define RCLCPP__CREATE_CLIENT_HPP_

#include <memory>
#include <string>

#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"
#include "rmw/rmw.h"

namespace rclcpp
{

/// Create a service client with a given type.
/// \internal
template<typename ServiceT>
typename rclcpp::Client<ServiceT>::SharedPtr
create_client(
  std::shared_ptr<node_interfaces::NodeBaseInterface> node_base,
  std::shared_ptr<node_interfaces::NodeGraphInterface> node_graph,
  std::shared_ptr<node_interfaces::NodeServicesInterface> node_services,
  const std::string & service_name,
  const rmw_qos_profile_t & qos_profile,
  rclcpp::CallbackGroup::SharedPtr group)
{
  rcl_client_options_t options = rcl_client_get_default_options();
  options.qos = qos_profile;

  auto cli = rclcpp::Client<ServiceT>::make_shared(
    node_base.get(),
    node_graph,
    service_name,
    options);

  auto cli_base_ptr = std::dynamic_pointer_cast<rclcpp::ClientBase>(cli);
  node_services->add_client(cli_base_ptr, group);
  return cli;
}

}  // namespace rclcpp

#endif  // RCLCPP__CREATE_CLIENT_HPP_
