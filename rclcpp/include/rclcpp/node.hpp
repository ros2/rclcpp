// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__NODE_HPP_
#define RCLCPP__NODE_HPP_

#include <atomic>
#include <condition_variable>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <vector>

#include "rcl/error_handling.h"
#include "rcl/node.h"

#include "rcl_interfaces/msg/list_parameters_result.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "rclcpp/callback_group.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/context.hpp"
#include "rclcpp/event.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/message_memory_strategy.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rcl
{
struct rcl_node_t;
}  // namespace rcl

namespace rclcpp
{

namespace graph_listener
{
class GraphListener;
}  // namespace graph_listener

namespace node
{

class InvalidEventError : public std::runtime_error
{
public:
  InvalidEventError()
  : std::runtime_error("event is invalid") {}
};

class EventNotRegisteredError : public std::runtime_error
{
public:
  EventNotRegisteredError()
  : std::runtime_error("event already registered") {}
};

/// Node is the single point of entry for creating publishers and subscribers.
class Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Node);

  /// Create a new node with the specified name.
  /**
   * \param[in] node_name Name of the node.
   * \param[in] use_intra_process_comms True to use the optimized intra-process communication
   * pipeline to pass messages between nodes in the same process using shared memory.
   */
  RCLCPP_PUBLIC
  explicit Node(const std::string & node_name, bool use_intra_process_comms = false);

  /// Create a node based on the node name and a rclcpp::context::Context.
  /**
   * \param[in] node_name Name of the node.
   * \param[in] context The context for the node (usually represents the state of a process).
   * \param[in] use_intra_process_comms True to use the optimized intra-process communication
   * pipeline to pass messages between nodes in the same process using shared memory.
   */
  RCLCPP_PUBLIC
  Node(
    const std::string & node_name, rclcpp::context::Context::SharedPtr context,
    bool use_intra_process_comms = false);

  RCLCPP_PUBLIC
  virtual ~Node();

  /// Get the name of the node.
  // \return The name of the node.
  RCLCPP_PUBLIC
  const std::string &
  get_name() const;

  /// Create and return a callback group.
  RCLCPP_PUBLIC
  rclcpp::callback_group::CallbackGroup::SharedPtr
  create_callback_group(rclcpp::callback_group::CallbackGroupType group_type);

  /// Create and return a Publisher.
  /**
   * \param[in] topic_name The topic for this publisher to publish on.
   * \param[in] qos_history_depth The depth of the publisher message queue.
   * \return Shared pointer to the created publisher.
   */
  template<typename MessageT, typename Alloc = std::allocator<void>>
  typename rclcpp::publisher::Publisher<MessageT, Alloc>::SharedPtr
  create_publisher(
    const std::string & topic_name, size_t qos_history_depth,
    std::shared_ptr<Alloc> allocator = nullptr);

  /// Create and return a Publisher.
  /**
   * \param[in] topic_name The topic for this publisher to publish on.
   * \param[in] qos_profile The quality of service profile to pass on to the rmw implementation.
   * \return Shared pointer to the created publisher.
   */
  template<typename MessageT, typename Alloc = std::allocator<void>>
  typename rclcpp::publisher::Publisher<MessageT, Alloc>::SharedPtr
  create_publisher(
    const std::string & topic_name,
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_default,
    std::shared_ptr<Alloc> allocator = nullptr);

  /// Create and return a Subscription.
  /**
   * \param[in] topic_name The topic to subscribe on.
   * \param[in] callback The user-defined callback function.
   * \param[in] qos_profile The quality of service profile to pass on to the rmw implementation.
   * \param[in] group The callback group for this subscription. NULL for no callback group.
   * \param[in] ignore_local_publications True to ignore local publications.
   * \param[in] msg_mem_strat The message memory strategy to use for allocating messages.
   * \return Shared pointer to the created subscription.
   */
  /* TODO(jacquelinekay):
     Windows build breaks when static member function passed as default
     argument to msg_mem_strat, nullptr is a workaround.
   */
  template<typename MessageT, typename CallbackT, typename Alloc = std::allocator<void>>
  typename rclcpp::subscription::Subscription<MessageT, Alloc>::SharedPtr
  create_subscription(
    const std::string & topic_name,
    CallbackT && callback,
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_default,
    rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr,
    bool ignore_local_publications = false,
    typename rclcpp::message_memory_strategy::MessageMemoryStrategy<MessageT, Alloc>::SharedPtr
    msg_mem_strat = nullptr,
    std::shared_ptr<Alloc> allocator = nullptr);

  /// Create and return a Subscription.
  /**
   * \param[in] topic_name The topic to subscribe on.
   * \param[in] qos_history_depth The depth of the subscription's incoming message queue.
   * \param[in] callback The user-defined callback function.
   * \param[in] group The callback group for this subscription. NULL for no callback group.
   * \param[in] ignore_local_publications True to ignore local publications.
   * \param[in] msg_mem_strat The message memory strategy to use for allocating messages.
   * \return Shared pointer to the created subscription.
   */
  /* TODO(jacquelinekay):
     Windows build breaks when static member function passed as default
     argument to msg_mem_strat, nullptr is a workaround.
   */
  template<typename MessageT, typename CallbackT, typename Alloc = std::allocator<void>>
  typename rclcpp::subscription::Subscription<MessageT, Alloc>::SharedPtr
  create_subscription(
    const std::string & topic_name,
    size_t qos_history_depth,
    CallbackT && callback,
    rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr,
    bool ignore_local_publications = false,
    typename rclcpp::message_memory_strategy::MessageMemoryStrategy<MessageT, Alloc>::SharedPtr
    msg_mem_strat = nullptr,
    std::shared_ptr<Alloc> allocator = nullptr);

  /// Create a timer.
  /**
   * \param[in] period Time interval between triggers of the callback.
   * \param[in] callback User-defined callback function.
   * \param[in] group Callback group to execute this timer's callback in.
   */
  template<typename CallbackType>
  typename rclcpp::timer::WallTimer<CallbackType>::SharedPtr
  create_wall_timer(
    std::chrono::nanoseconds period,
    CallbackType callback,
    rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr);

  /// Create a timer.
  /**
   * \param[in] period Time interval between triggers of the callback.
   * \param[in] callback User-defined callback function.
   * \param[in] group Callback group to execute this timer's callback in.
   */
  // TODO(wjwwood): reenable this once I figure out why the demo doesn't build with it.
  // rclcpp::timer::WallTimer::SharedPtr
  // create_wall_timer(
  //   std::chrono::duration<long double, std::nano> period,
  //   rclcpp::timer::CallbackType callback,
  //   rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr);

  using CallbackGroup = rclcpp::callback_group::CallbackGroup;
  using CallbackGroupWeakPtr = std::weak_ptr<CallbackGroup>;
  using CallbackGroupWeakPtrList = std::list<CallbackGroupWeakPtr>;

  /* Create and return a Client. */
  template<typename ServiceT>
  typename rclcpp::client::Client<ServiceT>::SharedPtr
  create_client(
    const std::string & service_name,
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_services_default,
    rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr);

  /* Create and return a Service. */
  template<typename ServiceT, typename CallbackT>
  typename rclcpp::service::Service<ServiceT>::SharedPtr
  create_service(
    const std::string & service_name,
    CallbackT && callback,
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_services_default,
    rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr);

  RCLCPP_PUBLIC
  std::vector<rcl_interfaces::msg::SetParametersResult>
  set_parameters(const std::vector<rclcpp::parameter::ParameterVariant> & parameters);

  RCLCPP_PUBLIC
  rcl_interfaces::msg::SetParametersResult
  set_parameters_atomically(const std::vector<rclcpp::parameter::ParameterVariant> & parameters);

  RCLCPP_PUBLIC
  std::vector<rclcpp::parameter::ParameterVariant>
  get_parameters(const std::vector<std::string> & names) const;

  RCLCPP_PUBLIC
  std::vector<rcl_interfaces::msg::ParameterDescriptor>
  describe_parameters(const std::vector<std::string> & names) const;

  RCLCPP_PUBLIC
  std::vector<uint8_t>
  get_parameter_types(const std::vector<std::string> & names) const;

  RCLCPP_PUBLIC
  rcl_interfaces::msg::ListParametersResult
  list_parameters(const std::vector<std::string> & prefixes, uint64_t depth) const;

  RCLCPP_PUBLIC
  std::map<std::string, std::string>
  get_topic_names_and_types() const;

  RCLCPP_PUBLIC
  size_t
  count_publishers(const std::string & topic_name) const;

  RCLCPP_PUBLIC
  size_t
  count_subscribers(const std::string & topic_name) const;

  RCLCPP_PUBLIC
  const CallbackGroupWeakPtrList &
  get_callback_groups() const;

  RCLCPP_PUBLIC
  const rcl_guard_condition_t *
  get_notify_guard_condition() const;

  RCLCPP_PUBLIC
  const rcl_guard_condition_t *
  get_graph_guard_condition() const;

  RCLCPP_PUBLIC
  const rcl_node_t *
  get_rcl_node_handle() const;

  /// Return the rcl_node_t node handle (non-const version).
  RCLCPP_PUBLIC
  rcl_node_t *
  get_rcl_node_handle();

  /// Return the rcl_node_t node handle in a std::shared_ptr.
  /* This handle remains valid after the Node is destroyed.
   * The actual rcl node is not finalized until it is out of scope everywhere.
   */
  RCLCPP_PUBLIC
  std::shared_ptr<rcl_node_t>
  get_shared_node_handle();

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
  void
  notify_graph_change();

  /// Return a graph event, which will be set anytime a graph change occurs.
  /* The graph Event object is a loan which must be returned.
   * The Event object is scoped and therefore to return the load just let it go
   * out of scope.
   */
  RCLCPP_PUBLIC
  rclcpp::event::Event::SharedPtr
  get_graph_event();

  /// Wait for a graph event to occur by waiting on an Event to become set.
  /* The given Event must be acquire through the get_graph_event() method.
   *
   * \throws InvalidEventError if the given event is nullptr
   * \throws EventNotRegisteredError if the given event was not acquired with
   *   get_graph_event().
   */
  RCLCPP_PUBLIC
  void
  wait_for_graph_change(
    rclcpp::event::Event::SharedPtr event,
    std::chrono::nanoseconds timeout);

  /// Return the number of on loan graph events, see get_graph_event().
  /* This is typically only used by the rclcpp::graph_listener::GraphListener.
   */
  RCLCPP_PUBLIC
  size_t
  count_graph_users();

  std::atomic_bool has_executor;

private:
  RCLCPP_DISABLE_COPY(Node);

  RCLCPP_PUBLIC
  bool
  group_in_node(callback_group::CallbackGroup::SharedPtr group);

  std::string name_;

  std::shared_ptr<rcl_node_t> node_handle_;

  rclcpp::context::Context::SharedPtr context_;

  CallbackGroup::SharedPtr default_callback_group_;
  CallbackGroupWeakPtrList callback_groups_;

  size_t number_of_subscriptions_;
  size_t number_of_timers_;
  size_t number_of_services_;
  size_t number_of_clients_;

  bool use_intra_process_comms_;

  mutable std::mutex mutex_;

  /// Guard condition for notifying the Executor of changes to this node.
  rcl_guard_condition_t notify_guard_condition_ = rcl_get_zero_initialized_guard_condition();

  /// Graph Listener which waits on graph changes for the node and is shared across nodes.
  std::shared_ptr<rclcpp::graph_listener::GraphListener> graph_listener_;
  /// Whether or not this node has been added to the graph listener yet.
  bool added_to_graph_listener_;

  /// Mutex to guard the graph event related data structures.
  std::mutex graph_mutex_;
  /// For notifying waiting threads (wait_for_graph_change()) on changes (notify_graph_change()).
  std::condition_variable graph_cv_;
  /// Weak references to graph events out on loan.
  std::vector<rclcpp::event::Event::WeakPtr> graph_events_;
  /// Number of graph events out on loan, used to determine if the graph should be monitored.
  /* graph_users_count_ is atomic so that it can be accessed without acquiring the graph_mutex_ */
  std::atomic_size_t graph_users_count_;

  std::map<std::string, rclcpp::parameter::ParameterVariant> parameters_;

  publisher::Publisher<rcl_interfaces::msg::ParameterEvent>::SharedPtr events_publisher_;
};

}  // namespace node
}  // namespace rclcpp

#ifndef RCLCPP__NODE_IMPL_HPP_
// Template implementations
#include "node_impl.hpp"
#endif

#endif  // RCLCPP__NODE_HPP_
