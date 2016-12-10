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

#ifndef RCLCPP__NODE_INTERFACES__NODE_GRAPH_INTERFACE_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_GRAPH_INTERFACE_HPP_

#include <chrono>
#include <map>
#include <string>

#include "rcl/guard_condition.h"

#include "rclcpp/event.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace node_interfaces
{

/// Pure virtual interface class for the NodeGraph part of the Node API.
class NodeGraphInterface
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeGraphInterface)

  /// Return a map of existing topic names (string) to topic types (string).
  /* A topic is considered to exist when at least one publisher or subscriber
   * exists for it, whether they be local or remote to this process.
   */
  RCLCPP_PUBLIC
  virtual
  std::map<std::string, std::string>
  get_topic_names_and_types() const = 0;

  /// Return the number of publishers that are advertised on a given topic.
  RCLCPP_PUBLIC
  virtual
  size_t
  count_publishers(const std::string & topic_name) const = 0;

  /// Return the number of subscribers who have created a subscription for a given topic.
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
  /* Affects threads waiting on the notify guard condition, see:
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
  /* The graph Event object is a loan which must be returned.
   * The Event object is scoped and therefore to return the load just let it go
   * out of scope.
   */
  RCLCPP_PUBLIC
  virtual
  rclcpp::event::Event::SharedPtr
  get_graph_event() = 0;

  /// Wait for a graph event to occur by waiting on an Event to become set.
  /* The given Event must be acquire through the get_graph_event() method.
   *
   * \throws InvalidEventError if the given event is nullptr
   * \throws EventNotRegisteredError if the given event was not acquired with
   *   get_graph_event().
   */
  RCLCPP_PUBLIC
  virtual
  void
  wait_for_graph_change(
    rclcpp::event::Event::SharedPtr event,
    std::chrono::nanoseconds timeout) = 0;

  /// Return the number of on loan graph events, see get_graph_event().
  /* This is typically only used by the rclcpp::graph_listener::GraphListener.
   */
  RCLCPP_PUBLIC
  virtual
  size_t
  count_graph_users() = 0;
};

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_GRAPH_INTERFACE_HPP_
