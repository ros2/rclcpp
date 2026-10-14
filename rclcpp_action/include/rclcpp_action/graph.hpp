// Copyright 2026 TriOrb, Inc.
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

#ifndef RCLCPP_ACTION__GRAPH_HPP_
#define RCLCPP_ACTION__GRAPH_HPP_

#include <optional>
#include <string>
#include <vector>

#include "rcl_action/graph.h"

#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"

#include "rclcpp_action/visibility_control.hpp"

namespace rclcpp_action
{

/// Endpoint information of an action client or an action server.
/**
 * An action is built on top of three services and two topics:
 *
 * - the goal service (`<action_name>/_action/send_goal`)
 * - the cancel service (`<action_name>/_action/cancel_goal`)
 * - the result service (`<action_name>/_action/get_result`)
 * - the feedback topic (`<action_name>/_action/feedback`)
 * - the status topic (`<action_name>/_action/status`)
 *
 * This class aggregates the endpoint information of all the underlying
 * entities of one action client or one action server.
 * The goal service endpoint is used as the canonical identity of an action
 * client or an action server, so the goal service endpoint information is
 * always populated and the node name, node namespace, action type, and
 * endpoint type are derived from it.
 * The remaining endpoint information is correlated to the goal service
 * endpoint by the node name and node namespace, and is `std::nullopt` if the
 * underlying entity has not been discovered.
 */
class ActionEndpointInfo
{
public:
  /// Construct from a rcl_action_endpoint_info_t.
  /**
   * \param[in] info the rcl endpoint information to construct from, its
   *   `goal_service_info` member must be populated.
   * \throws std::invalid_argument if the goal service info is not populated.
   */
  RCLCPP_ACTION_PUBLIC
  explicit ActionEndpointInfo(const rcl_action_endpoint_info_t & info);

  /// Get the node name of the action endpoint.
  RCLCPP_ACTION_PUBLIC
  const std::string &
  node_name() const;

  /// Get the node namespace of the action endpoint.
  RCLCPP_ACTION_PUBLIC
  const std::string &
  node_namespace() const;

  /// Get the action type, derived from the goal service type.
  RCLCPP_ACTION_PUBLIC
  std::string
  action_type() const;

  /// Get the endpoint type (client or server) of the action endpoint.
  RCLCPP_ACTION_PUBLIC
  rclcpp::EndpointType
  endpoint_type() const;

  /// Get a mutable reference to the endpoint information of the goal service.
  RCLCPP_ACTION_PUBLIC
  rclcpp::ServiceEndpointInfo &
  goal_service_info();

  /// Get a const reference to the endpoint information of the goal service.
  RCLCPP_ACTION_PUBLIC
  const rclcpp::ServiceEndpointInfo &
  goal_service_info() const;

  /// Get a mutable reference to the endpoint information of the cancel service.
  RCLCPP_ACTION_PUBLIC
  std::optional<rclcpp::ServiceEndpointInfo> &
  cancel_service_info();

  /// Get a const reference to the endpoint information of the cancel service.
  RCLCPP_ACTION_PUBLIC
  const std::optional<rclcpp::ServiceEndpointInfo> &
  cancel_service_info() const;

  /// Get a mutable reference to the endpoint information of the result service.
  RCLCPP_ACTION_PUBLIC
  std::optional<rclcpp::ServiceEndpointInfo> &
  result_service_info();

  /// Get a const reference to the endpoint information of the result service.
  RCLCPP_ACTION_PUBLIC
  const std::optional<rclcpp::ServiceEndpointInfo> &
  result_service_info() const;

  /// Get a mutable reference to the endpoint information of the feedback topic.
  RCLCPP_ACTION_PUBLIC
  std::optional<rclcpp::TopicEndpointInfo> &
  feedback_topic_info();

  /// Get a const reference to the endpoint information of the feedback topic.
  RCLCPP_ACTION_PUBLIC
  const std::optional<rclcpp::TopicEndpointInfo> &
  feedback_topic_info() const;

  /// Get a mutable reference to the endpoint information of the status topic.
  RCLCPP_ACTION_PUBLIC
  std::optional<rclcpp::TopicEndpointInfo> &
  status_topic_info();

  /// Get a const reference to the endpoint information of the status topic.
  RCLCPP_ACTION_PUBLIC
  const std::optional<rclcpp::TopicEndpointInfo> &
  status_topic_info() const;

private:
  rclcpp::ServiceEndpointInfo goal_service_info_;
  std::optional<rclcpp::ServiceEndpointInfo> cancel_service_info_;
  std::optional<rclcpp::ServiceEndpointInfo> result_service_info_;
  std::optional<rclcpp::TopicEndpointInfo> feedback_topic_info_;
  std::optional<rclcpp::TopicEndpointInfo> status_topic_info_;
};

/// Return a list of action clients on a given action.
/**
 * The returned parameter is a list of ActionEndpointInfo objects, where each
 * aggregates the endpoint information of all the underlying entities of one
 * action client, i.e. the clients of the goal, cancel, and result services
 * and the subscriptions on the feedback and status topics.
 *
 * The `action_name` may be a relative, private, or fully qualified action
 * name.
 * A relative or private action name will be expanded using the node's
 * namespace and name.
 * The queried `action_name` is not remapped.
 *
 * \param[in] node_base the node base interface of the node used to query the ROS graph.
 * \param[in] action_name name of the action to query.
 * \return a list of ActionEndpointInfo representing all the action clients on this action.
 * \throws std::invalid_argument if the action name is invalid.
 * \throws std::runtime_error if the list cannot be retrieved.
 */
RCLCPP_ACTION_PUBLIC
std::vector<ActionEndpointInfo>
get_action_clients_info_by_action(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base,
  const std::string & action_name);

/// Return a list of action servers on a given action.
/**
 * The returned parameter is a list of ActionEndpointInfo objects, where each
 * aggregates the endpoint information of all the underlying entities of one
 * action server, i.e. the servers of the goal, cancel, and result services
 * and the publishers on the feedback and status topics.
 *
 * The `action_name` may be a relative, private, or fully qualified action
 * name.
 * A relative or private action name will be expanded using the node's
 * namespace and name.
 * The queried `action_name` is not remapped.
 *
 * \param[in] node_base the node base interface of the node used to query the ROS graph.
 * \param[in] action_name name of the action to query.
 * \return a list of ActionEndpointInfo representing all the action servers on this action.
 * \throws std::invalid_argument if the action name is invalid.
 * \throws std::runtime_error if the list cannot be retrieved.
 */
RCLCPP_ACTION_PUBLIC
std::vector<ActionEndpointInfo>
get_action_servers_info_by_action(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base,
  const std::string & action_name);

/// Return the number of action clients on a given action.
/**
 * The `action_name` may be a relative, private, or fully qualified action
 * name.
 * A relative or private action name will be expanded using the node's
 * namespace and name.
 * The queried `action_name` is not remapped.
 *
 * \param[in] node_base the node base interface of the node used to query the ROS graph.
 * \param[in] action_name name of the action to count the number of action clients.
 * \return the number of action clients on the action.
 * \throws std::invalid_argument if the action name is invalid.
 * \throws std::runtime_error if the count cannot be retrieved.
 */
RCLCPP_ACTION_PUBLIC
size_t
count_action_clients(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base,
  const std::string & action_name);

/// Return the number of action servers on a given action.
/**
 * The `action_name` may be a relative, private, or fully qualified action
 * name.
 * A relative or private action name will be expanded using the node's
 * namespace and name.
 * The queried `action_name` is not remapped.
 *
 * \param[in] node_base the node base interface of the node used to query the ROS graph.
 * \param[in] action_name name of the action to count the number of action servers.
 * \return the number of action servers on the action.
 * \throws std::invalid_argument if the action name is invalid.
 * \throws std::runtime_error if the count cannot be retrieved.
 */
RCLCPP_ACTION_PUBLIC
size_t
count_action_servers(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base,
  const std::string & action_name);

}  // namespace rclcpp_action

#endif  // RCLCPP_ACTION__GRAPH_HPP_
