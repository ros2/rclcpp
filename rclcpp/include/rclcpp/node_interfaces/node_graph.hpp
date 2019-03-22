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

#ifndef RCLCPP__NODE_INTERFACES__NODE_GRAPH_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_GRAPH_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "rcl/guard_condition.h"

#include "rclcpp/event.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

namespace graph_listener
{
class GraphListener;
}  // namespace graph_listener

namespace node_interfaces
{

/// Implementation the NodeGraph part of the Node API.
class NodeGraph : public NodeGraphInterface
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeGraph)

  RCLCPP_PUBLIC
  explicit NodeGraph(rclcpp::node_interfaces::NodeBaseInterface * node_base);

  RCLCPP_PUBLIC
  virtual
  ~NodeGraph();

  RCLCPP_PUBLIC
  virtual
  std::map<std::string, std::vector<std::string>>
  get_topic_names_and_types(bool no_demangle = false) const;

  RCLCPP_PUBLIC
  virtual
  std::map<std::string, std::vector<std::string>>
  get_service_names_and_types() const;

  RCLCPP_PUBLIC
  virtual
  std::vector<std::string>
  get_node_names() const;

  RCLCPP_PUBLIC
  virtual
  std::vector<std::pair<std::string, std::string>>
  get_node_names_and_namespaces() const;

  RCLCPP_PUBLIC
  virtual
  size_t
  count_publishers(const std::string & topic_name) const;

  RCLCPP_PUBLIC
  virtual
  size_t
  count_subscribers(const std::string & topic_name) const;

  RCLCPP_PUBLIC
  virtual
  const rcl_guard_condition_t *
  get_graph_guard_condition() const;

  RCLCPP_PUBLIC
  virtual
  void
  notify_graph_change();

  RCLCPP_PUBLIC
  virtual
  void
  notify_shutdown();

  RCLCPP_PUBLIC
  virtual
  rclcpp::Event::SharedPtr
  get_graph_event();

  RCLCPP_PUBLIC
  virtual
  void
  wait_for_graph_change(
    rclcpp::Event::SharedPtr event,
    std::chrono::nanoseconds timeout);

  RCLCPP_PUBLIC
  virtual
  size_t
  count_graph_users();

private:
  RCLCPP_DISABLE_COPY(NodeGraph)

  /// Handle to the NodeBaseInterface given in the constructor.
  rclcpp::node_interfaces::NodeBaseInterface * node_base_;

  /// Graph Listener which waits on graph changes for the node and is shared across nodes.
  std::shared_ptr<rclcpp::graph_listener::GraphListener> graph_listener_;
  /// Whether or not this node needs to be added to the graph listener.
  std::atomic_bool should_add_to_graph_listener_;

  /// Mutex to guard the graph event related data structures.
  mutable std::mutex graph_mutex_;
  /// For notifying waiting threads (wait_for_graph_change()) on changes (notify_graph_change()).
  std::condition_variable graph_cv_;
  /// Weak references to graph events out on loan.
  std::vector<rclcpp::Event::WeakPtr> graph_events_;
  /// Number of graph events out on loan, used to determine if the graph should be monitored.
  /** graph_users_count_ is atomic so that it can be accessed without acquiring the graph_mutex_ */
  std::atomic_size_t graph_users_count_;
};

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_GRAPH_HPP_
