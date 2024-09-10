// Copyright 2024 Sony Group Corporation.
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

#ifndef RCLCPP__CREATE_GENERIC_SERVICE_HPP_
#define RCLCPP__CREATE_GENERIC_SERVICE_HPP_

#include <memory>
#include <string>
#include <utility>

#include "rclcpp/generic_service.hpp"
#include "rclcpp/node_interfaces/get_node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/get_node_services_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rmw/rmw.h"

namespace rclcpp
{
/// Create a generic service with a given type.
/**
 * \param[in] node_base NodeBaseInterface implementation of the node on which
 *  to create the generic service.
 * \param[in] node_services NodeServicesInterface implementation of the node on
 *  which to create the service.
 * \param[in] service_name The name on which the service is accessible.
 * \param[in] service_type The name of service type, e.g. "std_srvs/srv/SetBool".
 * \param[in] callback The callback to call when the service gets a request.
 * \param[in] qos Quality of service profile for the service.
 * \param[in] group Callback group to handle the reply to service calls.
 * \return Shared pointer to the created service.
 */
template<typename CallbackT>
typename rclcpp::GenericService::SharedPtr
create_generic_service(
  std::shared_ptr<node_interfaces::NodeBaseInterface> node_base,
  std::shared_ptr<node_interfaces::NodeServicesInterface> node_services,
  const std::string & service_name,
  const std::string & service_type,
  CallbackT && callback,
  const rclcpp::QoS & qos,
  rclcpp::CallbackGroup::SharedPtr group)
{
  rclcpp::GenericServiceCallback any_service_callback;
  any_service_callback.set(std::forward<CallbackT>(callback));

  rcl_service_options_t service_options = rcl_service_get_default_options();
  service_options.qos = qos.get_rmw_qos_profile();

  auto serv = GenericService::make_shared(
    node_base->get_shared_rcl_node_handle(),
    service_name, service_type, any_service_callback, service_options);
  auto serv_base_ptr = std::dynamic_pointer_cast<ServiceBase>(serv);
  node_services->add_service(serv_base_ptr, group);
  return serv;
}

/// Create a generic service with a given type.
/**
 * The NodeT type needs to have NodeBaseInterface implementation and NodeServicesInterface
 * implementation of the node which to create the generic service.
 *
 * \param[in] node The node on which to create the generic service.
 * \param[in] service_name The name on which the service is accessible.
 * \param[in] service_type The name of service type, e.g. "std_srvs/srv/SetBool".
 * \param[in] callback The callback to call when the service gets a request.
 * \param[in] qos Quality of service profile for the service.
 * \param[in] group Callback group to handle the reply to service calls.
 * \return Shared pointer to the created service.
 */
template<typename NodeT, typename CallbackT>
typename rclcpp::GenericService::SharedPtr
create_generic_service(
  NodeT node,
  const std::string & service_name,
  const std::string & service_type,
  CallbackT && callback,
  const rclcpp::QoS & qos,
  rclcpp::CallbackGroup::SharedPtr group)
{
  return create_generic_service<CallbackT>(
    rclcpp::node_interfaces::get_node_base_interface(node),
    rclcpp::node_interfaces::get_node_services_interface(node),
    service_name,
    service_type,
    std::forward<CallbackT>(callback), qos.get_rmw_qos_profile(), group);
}
}  // namespace rclcpp

#endif  // RCLCPP__CREATE_GENERIC_SERVICE_HPP_
