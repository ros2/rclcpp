// Copyright 2016-2017 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__NODE_INTERFACES__NODE_GRAPH_INTERFACE_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_GRAPH_INTERFACE_HPP_

#include <algorithm>
#include <array>
#include <chrono>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "rcl/graph.h"
#include "rcl/guard_condition.h"

#include "rclcpp/event.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

enum class EndpointType
{
  Invalid = RMW_ENDPOINT_INVALID,
  Publisher = RMW_ENDPOINT_PUBLISHER,
  Subscription = RMW_ENDPOINT_SUBSCRIPTION
};

/**
 * Struct that contains topic endpoint information like the associated node name, node namespace,
 * topic type, endpoint type, endpoint GID, and its QoS.
 */
class TopicEndpointInfo
{
public:
  /// Construct a TopicEndpointInfo from a rcl_topic_endpoint_info_t.
  RCLCPP_PUBLIC
  explicit TopicEndpointInfo(const rcl_topic_endpoint_info_t & info)
  : node_name_(info.node_name),
    node_namespace_(info.node_namespace),
    topic_type_(info.topic_type),
    endpoint_type_(static_cast<rclcpp::EndpointType>(info.endpoint_type)),
    qos_profile_({info.qos_profile.history, info.qos_profile.depth}, info.qos_profile)
  {
    std::copy(info.endpoint_gid, info.endpoint_gid + RMW_GID_STORAGE_SIZE, endpoint_gid_.begin());
  }

  /// Get a mutable reference to the node name.
  RCLCPP_PUBLIC
  std::string &
  node_name();

  /// Get a const reference to the node name.
  RCLCPP_PUBLIC
  const std::string &
  node_name() const;

  /// Get a mutable reference to the node namespace.
  RCLCPP_PUBLIC
  std::string &
  node_namespace();

  /// Get a const reference to the node namespace.
  RCLCPP_PUBLIC
  const std::string &
  node_namespace() const;

  /// Get a mutable reference to the topic type string.
  RCLCPP_PUBLIC
  std::string &
  topic_type();

  /// Get a const reference to the topic type string.
  RCLCPP_PUBLIC
  const std::string &
  topic_type() const;

  /// Get a mutable reference to the topic endpoint type.
  RCLCPP_PUBLIC
  rclcpp::EndpointType &
  endpoint_type();

  /// Get a const reference to the topic endpoint type.
  RCLCPP_PUBLIC
  const rclcpp::EndpointType &
  endpoint_type() const;

  /// Get a mutable reference to the GID of the topic endpoint.
  RCLCPP_PUBLIC
  std::array<uint8_t, RMW_GID_STORAGE_SIZE> &
  endpoint_gid();

  /// Get a const reference to the GID of the topic endpoint.
  RCLCPP_PUBLIC
  const std::array<uint8_t, RMW_GID_STORAGE_SIZE> &
  endpoint_gid() const;

  /// Get a mutable reference to the QoS profile of the topic endpoint.
  RCLCPP_PUBLIC
  rclcpp::QoS &
  qos_profile();

  /// Get a const reference to the QoS profile of the topic endpoint.
  RCLCPP_PUBLIC
  const rclcpp::QoS &
  qos_profile() const;

private:
  std::string node_name_;
  std::string node_namespace_;
  std::string topic_type_;
  rclcpp::EndpointType endpoint_type_;
  std::array<uint8_t, RMW_GID_STORAGE_SIZE> endpoint_gid_;
  rclcpp::QoS qos_profile_;
};

namespace node_interfaces
{

/// Pure virtual interface class for the NodeGraph part of the Node API.
class NodeGraphInterface
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeGraphInterface)

  RCLCPP_PUBLIC
  virtual
  ~NodeGraphInterface() = default;

  /// Return a map of existing topic names to list of topic types.
  /**
   * A topic is considered to exist when at least one publisher or subscriber
   * exists for it, whether they be local or remote to this process.
   * The returned names are the actual names used and do not have remap rules applied.
   *
   * \param[in] no_demangle if true, topic names and types are not demangled
   */
  RCLCPP_PUBLIC
  virtual
  std::map<std::string, std::vector<std::string>>
  get_topic_names_and_types(bool no_demangle = false) const = 0;

  /// Return a map of existing service names to list of service types.
  /**
   * A service is considered to exist when at least one service server or
   * service client exists for it, whether they be local or remote to this
   * process.
   * The returned names are the actual names used and do not have remap rules applied.
   */
  RCLCPP_PUBLIC
  virtual
  std::map<std::string, std::vector<std::string>>
  get_service_names_and_types() const = 0;

  /// Return a map of existing service names to list of service types for a specific node.
  /**
   * This function only considers services - not clients.
   * The returned names are the actual names used and do not have remap rules applied.
   *
   * \param[in] node_name name of the node
   * \param[in] namespace_ namespace of the node
   */
  RCLCPP_PUBLIC
  virtual
  std::map<std::string, std::vector<std::string>>
  get_service_names_and_types_by_node(
    const std::string & node_name,
    const std::string & namespace_) const = 0;

  /// Return a vector of existing node names (string).
  /*
   * The returned names are the actual names used and do not have remap rules applied.
   */
  RCLCPP_PUBLIC
  virtual
  std::vector<std::string>
  get_node_names() const = 0;

  /// Return a vector of existing node names and namespaces (pair of string).
  /*
   * The returned names are the actual names used and do not have remap rules applied.
   */
  RCLCPP_PUBLIC
  virtual
  std::vector<std::pair<std::string, std::string>>
  get_node_names_and_namespaces() const = 0;

  /// Return the number of publishers that are advertised on a given topic.
  /*
   * \param[in] topic_name the actual topic name used; it will not be automatically remapped.
   */
  RCLCPP_PUBLIC
  virtual
  size_t
  count_publishers(const std::string & topic_name) const = 0;

  /// Return the number of subscribers who have created a subscription for a given topic.
  /*
   * \param[in] topic_name the actual topic name used; it will not be automatically remapped.
   */
  RCLCPP_PUBLIC
  virtual
  size_t
  count_subscribers(const std::string & topic_name) const = 0;

  /// Return the rcl guard condition which is triggered when the ROS graph changes.
  RCLCPP_PUBLIC
  virtual
  const rcl_guard_condition_t *
  get_graph_guard_condition() const = 0;

  /// Notify threads waiting on graph changes.
  /**
   * Affects threads waiting on the notify guard condition, see:
   * get_notify_guard_condition(), as well as the threads waiting on graph
   * changes using a graph Event, see: wait_for_graph_change().
   *
   * This is typically only used by the rclcpp::graph_listener::GraphListener.
   *
   * \throws RCLBaseError (a child of that exception) when an rcl error occurs
   */
  RCLCPP_PUBLIC
  virtual
  void
  notify_graph_change() = 0;

  /// Notify any and all blocking node actions that shutdown has occurred.
  RCLCPP_PUBLIC
  virtual
  void
  notify_shutdown() = 0;

  /// Return a graph event, which will be set anytime a graph change occurs.
  /**
   * The graph Event object is a loan which must be returned.
   * The Event object is scoped and therefore to return the load just let it go
   * out of scope.
   */
  RCLCPP_PUBLIC
  virtual
  rclcpp::Event::SharedPtr
  get_graph_event() = 0;

  /// Wait for a graph event to occur by waiting on an Event to become set.
  /**
   * The given Event must be acquire through the get_graph_event() method.
   *
   * \throws InvalidEventError if the given event is nullptr
   * \throws EventNotRegisteredError if the given event was not acquired with
   *   get_graph_event().
   */
  RCLCPP_PUBLIC
  virtual
  void
  wait_for_graph_change(
    rclcpp::Event::SharedPtr event,
    std::chrono::nanoseconds timeout) = 0;

  /// Return the number of on loan graph events, see get_graph_event().
  /**
   * This is typically only used by the rclcpp::graph_listener::GraphListener.
   */
  RCLCPP_PUBLIC
  virtual
  size_t
  count_graph_users() const = 0;

  /// Return the topic endpoint information about publishers on a given topic.
  /**
   * \param[in] topic_name the actual topic name used; it will not be automatically remapped.
   * \sa rclcpp::Node::get_publishers_info_by_topic
   */
  RCLCPP_PUBLIC
  virtual
  std::vector<rclcpp::TopicEndpointInfo>
  get_publishers_info_by_topic(const std::string & topic_name, bool no_mangle = false) const = 0;

  /// Return the topic endpoint information about subscriptions on a given topic.
  /**
   * \param[in] topic_name the actual topic name used; it will not be automatically remapped.
   * \sa rclcpp::Node::get_subscriptions_info_by_topic
   */
  RCLCPP_PUBLIC
  virtual
  std::vector<rclcpp::TopicEndpointInfo>
  get_subscriptions_info_by_topic(const std::string & topic_name, bool no_mangle = false) const = 0;
};

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_GRAPH_INTERFACE_HPP_
