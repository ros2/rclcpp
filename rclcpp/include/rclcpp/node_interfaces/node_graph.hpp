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

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "rcl/guard_condition.h"

#include "rclcpp/event.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/visibility_control.hpp"

#include "rcpputils/mutex.hpp"

#include "rmw/topic_endpoint_info_array.h"

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
  std::map<std::string, std::vector<std::string>>
  get_topic_names_and_types(bool no_demangle = false) const override;

  RCLCPP_PUBLIC
  std::map<std::string, std::vector<std::string>>
  get_service_names_and_types() const override;

  RCLCPP_PUBLIC
  std::map<std::string, std::vector<std::string>>
  get_service_names_and_types_by_node(
    const std::string & node_name,
    const std::string & namespace_) const override;

  RCLCPP_PUBLIC
  std::map<std::string, std::vector<std::string>>
  get_client_names_and_types_by_node(
    const std::string & node_name,
    const std::string & namespace_) const override;

  RCLCPP_PUBLIC
  std::map<std::string, std::vector<std::string>>
  get_publisher_names_and_types_by_node(
    const std::string & node_name,
    const std::string & namespace_,
    bool no_demangle = false) const override;

  RCLCPP_PUBLIC
  std::map<std::string, std::vector<std::string>>
  get_subscriber_names_and_types_by_node(
    const std::string & node_name,
    const std::string & namespace_,
    bool no_demangle = false) const override;

  RCLCPP_PUBLIC
  std::vector<std::string>
  get_node_names() const override;

  RCLCPP_PUBLIC
  std::vector<std::tuple<std::string, std::string, std::string>>
  get_node_names_with_enclaves() const override;

  RCLCPP_PUBLIC
  std::vector<std::pair<std::string, std::string>>
  get_node_names_and_namespaces() const override;

  RCLCPP_PUBLIC
  size_t
  count_publishers(const std::string & topic_name) const override;

  RCLCPP_PUBLIC
  size_t
  count_subscribers(const std::string & topic_name) const override;

  RCLCPP_PUBLIC
  const rcl_guard_condition_t *
  get_graph_guard_condition() const override;

  RCLCPP_PUBLIC
  void
  notify_graph_change() override;

  RCLCPP_PUBLIC
  void
  notify_shutdown() override;

  RCLCPP_PUBLIC
  rclcpp::Event::SharedPtr
  get_graph_event() override;

  RCLCPP_PUBLIC
  void
  wait_for_graph_change(
    rclcpp::Event::SharedPtr event,
    std::chrono::nanoseconds timeout) override;

  RCLCPP_PUBLIC
  size_t
  count_graph_users() const override;

  RCLCPP_PUBLIC
  std::vector<rclcpp::TopicEndpointInfo>
  get_publishers_info_by_topic(
    const std::string & topic_name,
    bool no_mangle = false) const override;

  RCLCPP_PUBLIC
  std::vector<rclcpp::TopicEndpointInfo>
  get_subscriptions_info_by_topic(
    const std::string & topic_name,
    bool no_mangle = false) const override;

private:
  RCLCPP_DISABLE_COPY(NodeGraph)

  /// Handle to the NodeBaseInterface given in the constructor.
  rclcpp::node_interfaces::NodeBaseInterface * node_base_;

  /// Graph Listener which waits on graph changes for the node and is shared across nodes.
  std::shared_ptr<rclcpp::graph_listener::GraphListener> graph_listener_;
  /// Whether or not this node needs to be added to the graph listener.
  std::atomic_bool should_add_to_graph_listener_;

  /// Mutex to guard the graph event related data structures.
  mutable rcpputils::PIMutex graph_mutex_;
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
