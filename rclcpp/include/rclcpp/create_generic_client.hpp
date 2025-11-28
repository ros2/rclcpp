// Copyright 2023 Sony Group Corporation.
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

#ifndef RCLCPP__CREATE_GENERIC_CLIENT_HPP_
#define RCLCPP__CREATE_GENERIC_CLIENT_HPP_

#include <memory>
#include <string>

#include "rclcpp/generic_client.hpp"
#include "rclcpp/node_interfaces/get_node_base_interface.hpp"
#include "rclcpp/node_interfaces/get_node_graph_interface.hpp"
#include "rclcpp/node_interfaces/get_node_services_interface.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"
#include "rclcpp/qos.hpp"

namespace rclcpp
{
/// Create a generic service client with a name of given type.
/**
 * \param[in] node_base NodeBaseInterface implementation of the node on which
 *  to create the client.
 * \param[in] node_graph NodeGraphInterface implementation of the node on which
 *  to create the client.
 * \param[in] node_services NodeServicesInterface implementation of the node on
 *  which to create the client.
 * \param[in] service_name The name on which the service is accessible.
 * \param[in] service_type The name of service type, e.g. "test_msgs/srv/BasicTypes"
 * \param[in] qos Quality of service profile for client.
 * \param[in] group Callback group to handle the reply to service calls.
 * \return Shared pointer to the created client.
 */
RCLCPP_PUBLIC
rclcpp::GenericClient::SharedPtr
create_generic_client(
  std::shared_ptr<node_interfaces::NodeBaseInterface> node_base,
  std::shared_ptr<node_interfaces::NodeGraphInterface> node_graph,
  std::shared_ptr<node_interfaces::NodeServicesInterface> node_services,
  const std::string & service_name,
  const std::string & service_type,
  const rclcpp::QoS & qos = rclcpp::ServicesQoS(),
  rclcpp::CallbackGroup::SharedPtr group = nullptr);

/// Create a generic service client with a name of given type.
/**
 * The NodeT type needs to have NodeBaseInterface implementation, NodeGraphInterface implementation
 * and NodeServicesInterface implementation of the node which to create the client.
 *
 * \param[in] node The node on which to create the client.
 * \param[in] service_name The name on which the service is accessible.
 * \param[in] service_type The name of service type, e.g. "test_msgs/srv/BasicTypes"
 * \param[in] qos Quality of service profile for client.
 * \param[in] group Callback group to handle the reply to service calls.
 * \return Shared pointer to the created client.
 */
template<typename NodeT>
rclcpp::GenericClient::SharedPtr
create_generic_client(
  NodeT node,
  const std::string & service_name,
  const std::string & service_type,
  const rclcpp::QoS & qos = rclcpp::ServicesQoS(),
  rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  return create_generic_client(
    rclcpp::node_interfaces::get_node_base_interface(node),
    rclcpp::node_interfaces::get_node_graph_interface(node),
    rclcpp::node_interfaces::get_node_services_interface(node),
    service_name,
    service_type,
    qos,
    group
  );
}
}  // namespace rclcpp

#endif  // RCLCPP__CREATE_GENERIC_CLIENT_HPP_
