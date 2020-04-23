// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__CREATE_SERVICE_HPP_
#define RCLCPP__CREATE_SERVICE_HPP_

#include <memory>
#include <string>
#include <utility>

#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rmw/rmw.h"

namespace rclcpp
{

/// Create a service with a given type.
/// \internal
template<typename ServiceT, typename CallbackT>
typename rclcpp::Service<ServiceT>::SharedPtr
create_service(
  std::shared_ptr<node_interfaces::NodeBaseInterface> node_base,
  std::shared_ptr<node_interfaces::NodeServicesInterface> node_services,
  const std::string & service_name,
  CallbackT && callback,
  const rmw_qos_profile_t & qos_profile,
  rclcpp::CallbackGroup::SharedPtr group)
{
  rclcpp::AnyServiceCallback<ServiceT> any_service_callback;
  any_service_callback.set(std::forward<CallbackT>(callback));

  rcl_service_options_t service_options = rcl_service_get_default_options();
  service_options.qos = qos_profile;

  auto serv = Service<ServiceT>::make_shared(
    node_base->get_shared_rcl_node_handle(),
    service_name, any_service_callback, service_options);
  auto serv_base_ptr = std::dynamic_pointer_cast<ServiceBase>(serv);
  node_services->add_service(serv_base_ptr, group);
  return serv;
}

}  // namespace rclcpp

#endif  // RCLCPP__CREATE_SERVICE_HPP_
