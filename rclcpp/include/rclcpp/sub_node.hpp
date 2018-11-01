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

#ifndef RCLCPP__SUB_NODE_HPP_
#define RCLCPP__SUB_NODE_HPP_

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
#include "rclcpp/clock.hpp"
#include "rclcpp/context.hpp"
#include "rclcpp/event.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/message_memory_strategy.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_clock_interface.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"
#include "rclcpp/node_interfaces/node_timers_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/subscription_traits.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

class Node;

/// Node is the single point of entry for creating publishers and subscribers.
class SubNode : public std::enable_shared_from_this<SubNode>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(SubNode)

  /// Create a new node with the specified name.
  /**
   * \param[in] node_name Name of the node.
   * \param[in] namespace_ Namespace of the node.
   * \param[in] use_intra_process_comms True to use the optimized intra-process communication
   * pipeline to pass messages between nodes in the same process using shared memory.
   */

  RCLCPP_PUBLIC
  virtual ~SubNode();

  /// Get the name of the node.
  /** \return The name of the node. */
  RCLCPP_PUBLIC
  const char *
  get_name() const;

  /// Get the namespace of the node.
  /** \return The namespace of the node. */
  RCLCPP_PUBLIC
  const char *
  get_namespace() const;

  /// Get the logger of the node.
  /** \return The logger of the node. */
  RCLCPP_PUBLIC
  rclcpp::Logger
  get_logger() const;

  /// Create and return a callback group.
  RCLCPP_PUBLIC
  rclcpp::callback_group::CallbackGroup::SharedPtr
  create_callback_group(rclcpp::callback_group::CallbackGroupType group_type);

  /// Return the list of callback groups in the node.
  RCLCPP_PUBLIC
  const std::vector<rclcpp::callback_group::CallbackGroup::WeakPtr> &
  get_callback_groups() const;

  /// Create and return a Publisher.
  /**
   * \param[in] topic_name The topic for this publisher to publish on.
   * \param[in] qos_history_depth The depth of the publisher message queue.
   * \param[in] allocator Optional custom allocator.
   * \return Shared pointer to the created publisher.
   */
  template<
    typename MessageT, typename Alloc = std::allocator<void>,
    typename PublisherT = ::rclcpp::Publisher<MessageT, Alloc>>
  std::shared_ptr<PublisherT>
  create_publisher(
    const std::string & topic_name, size_t qos_history_depth,
    std::shared_ptr<Alloc> allocator = nullptr);

  /// Create and return a Publisher.
  /**
   * \param[in] topic_name The topic for this publisher to publish on.
   * \param[in] qos_profile The quality of service profile to pass on to the rmw implementation.
   * \param[in] allocator Optional custom allocator.
   * \return Shared pointer to the created publisher.
   */
  template<
    typename MessageT, typename Alloc = std::allocator<void>,
    typename PublisherT = ::rclcpp::Publisher<MessageT, Alloc>>
  std::shared_ptr<PublisherT>
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
   * \param[in] allocator Optional custom allocator.
   * \return Shared pointer to the created subscription.
   */
  /* TODO(jacquelinekay):
     Windows build breaks when static member function passed as default
     argument to msg_mem_strat, nullptr is a workaround.
   */
  template<
    typename MessageT,
    typename CallbackT,
    typename Alloc = std::allocator<void>,
    typename SubscriptionT = rclcpp::Subscription<
      typename rclcpp::subscription_traits::has_message_type<CallbackT>::type, Alloc>>
  std::shared_ptr<SubscriptionT>
  create_subscription(
    const std::string & topic_name,
    CallbackT && callback,
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_default,
    rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr,
    bool ignore_local_publications = false,
    typename rclcpp::message_memory_strategy::MessageMemoryStrategy<
      typename rclcpp::subscription_traits::has_message_type<CallbackT>::type, Alloc>::SharedPtr
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
   * \param[in] allocator Optional custom allocator.
   * \return Shared pointer to the created subscription.
   */
  /* TODO(jacquelinekay):
     Windows build breaks when static member function passed as default
     argument to msg_mem_strat, nullptr is a workaround.
   */
  template<
    typename MessageT,
    typename CallbackT,
    typename Alloc = std::allocator<void>,
    typename SubscriptionT = rclcpp::Subscription<
      typename rclcpp::subscription_traits::has_message_type<CallbackT>::type, Alloc>>
  std::shared_ptr<SubscriptionT>
  create_subscription(
    const std::string & topic_name,
    CallbackT && callback,
    size_t qos_history_depth,
    rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr,
    bool ignore_local_publications = false,
    typename rclcpp::message_memory_strategy::MessageMemoryStrategy<
      typename rclcpp::subscription_traits::has_message_type<CallbackT>::type, Alloc>::SharedPtr
    msg_mem_strat = nullptr,
    std::shared_ptr<Alloc> allocator = nullptr);

  /// Create a timer.
  /**
   * \param[in] period Time interval between triggers of the callback.
   * \param[in] callback User-defined callback function.
   * \param[in] group Callback group to execute this timer's callback in.
   */
  template<typename DurationT = std::milli, typename CallbackT>
  typename rclcpp::WallTimer<CallbackT>::SharedPtr
  create_wall_timer(
    std::chrono::duration<int64_t, DurationT> period,
    CallbackT callback,
    rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr);

  /* Create and return a Client. */
  template<typename ServiceT>
  typename rclcpp::Client<ServiceT>::SharedPtr
  create_client(
    const std::string & service_name,
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_services_default,
    rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr);

  /* Create and return a Service. */
  template<typename ServiceT, typename CallbackT>
  typename rclcpp::Service<ServiceT>::SharedPtr
  create_service(
    const std::string & service_name,
    CallbackT && callback,
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_services_default,
    rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr);

  RCLCPP_PUBLIC
  std::vector<rcl_interfaces::msg::SetParametersResult>
  set_parameters(const std::vector<rclcpp::Parameter> & parameters);

  RCLCPP_PUBLIC
  rcl_interfaces::msg::SetParametersResult
  set_parameters_atomically(const std::vector<rclcpp::Parameter> & parameters);

  template<typename ParameterT>
  void
  set_parameter_if_not_set(
    const std::string & name,
    const ParameterT & value);

  RCLCPP_PUBLIC
  std::vector<rclcpp::Parameter>
  get_parameters(const std::vector<std::string> & names) const;

  RCLCPP_PUBLIC
  rclcpp::Parameter
  get_parameter(const std::string & name) const;

  RCLCPP_PUBLIC
  bool
  get_parameter(
    const std::string & name,
    rclcpp::Parameter & parameter) const;

  /// Assign the value of the parameter if set into the parameter argument.
  /**
   * If the parameter was not set, then the "parameter" argument is never assigned a value.
   *
   * \param[in] name The name of the parameter to get.
   * \param[out] parameter The output where the value of the parameter should be assigned.
   * \returns true if the parameter was set, false otherwise
   */
  template<typename ParameterT>
  bool
  get_parameter(const std::string & name, ParameterT & parameter) const;

  /// Get the parameter value, or the "alternative value" if not set, and assign it to "value".
  /**
   * If the parameter was not set, then the "value" argument is assigned
   * the "alternative_value".
   * In all cases, the parameter remains not set after this function is called.
   *
   * \param[in] name The name of the parameter to get.
   * \param[out] value The output where the value of the parameter should be assigned.
   * \param[in] alternative_value Value to be stored in output if the parameter was not set.
   * \returns true if the parameter was set, false otherwise
   */
  template<typename ParameterT>
  bool
  get_parameter_or(
    const std::string & name,
    ParameterT & value,
    const ParameterT & alternative_value) const;

  /// Get the parameter value; if not set, set the "alternative value" and store it in the node.
  /**
   * If the parameter is set, then the "value" argument is assigned the value
   * in the parameter.
   * If the parameter is not set, then the "value" argument is assigned the "alternative_value",
   * and the parameter is set to the "alternative_value" on the node.
   *
   * \param[in] name The name of the parameter to get.
   * \param[out] value The output where the value of the parameter should be assigned.
   * \param[in] alternative_value Value to be stored in output and parameter if the parameter was not set.
   */
  template<typename ParameterT>
  void
  get_parameter_or_set(
    const std::string & name,
    ParameterT & value,
    const ParameterT & alternative_value);

  RCLCPP_PUBLIC
  std::vector<rcl_interfaces::msg::ParameterDescriptor>
  describe_parameters(const std::vector<std::string> & names) const;

  RCLCPP_PUBLIC
  std::vector<uint8_t>
  get_parameter_types(const std::vector<std::string> & names) const;

  RCLCPP_PUBLIC
  rcl_interfaces::msg::ListParametersResult
  list_parameters(const std::vector<std::string> & prefixes, uint64_t depth) const;

  /// Register the callback for parameter changes
  /**
   * \param[in] callback User defined callback function.
   *   It is expected to atomically set parameters.
   * \note Repeated invocations of this function will overwrite previous callbacks
   */
  template<typename CallbackT>
  void
  register_param_change_callback(CallbackT && callback);

  RCLCPP_PUBLIC
  std::vector<std::string>
  get_node_names() const;

  RCLCPP_PUBLIC
  std::map<std::string, std::vector<std::string>>
  get_topic_names_and_types() const;

  RCLCPP_PUBLIC
  std::map<std::string, std::vector<std::string>>
  get_service_names_and_types() const;

  RCLCPP_PUBLIC
  size_t
  count_publishers(const std::string & topic_name) const;

  RCLCPP_PUBLIC
  size_t
  count_subscribers(const std::string & topic_name) const;

  /// Return a graph event, which will be set anytime a graph change occurs.
  /* The graph Event object is a loan which must be returned.
   * The Event object is scoped and therefore to return the loan just let it go
   * out of scope.
   */
  RCLCPP_PUBLIC
  rclcpp::Event::SharedPtr
  get_graph_event();

  /// Wait for a graph event to occur by waiting on an Event to become set.
  /**
   * The given Event must be acquire through the get_graph_event() method.
   *
   * \throws InvalidEventError if the given event is nullptr
   * \throws EventNotRegisteredError if the given event was not acquired with
   *   get_graph_event().
   */
  RCLCPP_PUBLIC
  void
  wait_for_graph_change(
    rclcpp::Event::SharedPtr event,
    std::chrono::nanoseconds timeout);

  RCLCPP_PUBLIC
  rclcpp::Clock::SharedPtr
  get_clock();

  RCLCPP_PUBLIC
  Time
  now();

  /// Return the Node's internal NodeBaseInterface implementation.
  RCLCPP_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface();

  /// Return the Node's internal NodeClockInterface implementation.
  RCLCPP_PUBLIC
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr
  get_node_clock_interface();

  /// Return the Node's internal NodeGraphInterface implementation.
  RCLCPP_PUBLIC
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr
  get_node_graph_interface();

  /// Return the Node's internal NodeTimersInterface implementation.
  RCLCPP_PUBLIC
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr
  get_node_timers_interface();

  /// Return the Node's internal NodeTopicsInterface implementation.
  RCLCPP_PUBLIC
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr
  get_node_topics_interface();

  /// Return the Node's internal NodeServicesInterface implementation.
  RCLCPP_PUBLIC
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr
  get_node_services_interface();

  /// Return the Node's internal NodeParametersInterface implementation.
  RCLCPP_PUBLIC
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
  get_node_parameters_interface();

  RCLCPP_PUBLIC
  std::shared_ptr<rclcpp::SubNode>
  create_sub_node(const std::string & namespace_);

private:
  RCLCPP_DISABLE_COPY(SubNode)

  RCLCPP_PUBLIC
  explicit SubNode(
    const Node::SharedPtr original,
    const std::string & namespace_);

  RCLCPP_PUBLIC
  explicit SubNode(
    const SubNode::SharedPtr other,
    const std::string & namespace_);

  RCLCPP_PUBLIC
  bool
  group_in_node(callback_group::CallbackGroup::SharedPtr group);

  RCLCPP_PUBLIC
  const std::string validated_extended_ns(const std::string & extended_ns) const;

  RCLCPP_PUBLIC
  const std::string add_extended_ns_prefix(const std::string & original_name) const;

  Node::SharedPtr original_;

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_;

  bool use_intra_process_comms_;

  std::string extended_namespace_;

  friend class Node;
};

}  // namespace rclcpp

#ifndef RCLCPP__SUB_NODE_IMPL_HPP_
// Template implementations
#include "sub_node_impl.hpp"
#endif

#endif  // RCLCPP__NODE_HPP_
