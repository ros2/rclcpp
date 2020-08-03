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

/** \mainpage rclcpp_lifecycle: Package containing a prototype for lifecycle implementation.
 *
 * - Lifecycle states: Define the State class. There are 4 primary states: Unconfigured, Inactive,
 *   Active and Finalized. There are also 6 transition states which are intermediate states during
 *   a requested transition. Configuring, CleaningUp, ShuttingDown, Activating, Deactivating and
 *   ErrorProcessing.
 *   - rclcpp_lifecycle/state.hpp
 * - Lifecycle transitions Define the Transition class. There are 7 transitions exposed to a
 *   supervisory process, they are: create, configure, cleanup, activate, deactivate, shutdown and
 *   destroy.
 *   - rclcpp_lifecycle/transition.hpp
 * - Lifecycle publisher creates a publisher that allows enabling and disabling message publication.
 *   - rclcpp_lifecycle/publisher.hpp
 * - Lifecycle node: An optional interface class for life cycle node implementations.
 *   - rclcpp_lifecycle/lifecycle_node.hpp
 *
 * Some useful internal abstractions and utilities:
 * - Macros for controlling symbol visibility on the library
 *   - rclcpp_lifecycle/visibility_control.h
 */

#ifndef RCLCPP_LIFECYCLE__LIFECYCLE_NODE_HPP_
#define RCLCPP_LIFECYCLE__LIFECYCLE_NODE_HPP_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rcutils/macros.h"

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
#include "rclcpp/node_options.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_clock_interface.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"
#include "rclcpp/node_interfaces/node_time_source_interface.hpp"
#include "rclcpp/node_interfaces/node_timers_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/node_interfaces/node_waitables_interface.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/transition.hpp"
#include "rclcpp_lifecycle/visibility_control.h"

namespace rclcpp_lifecycle
{

// include these here to work around an esoteric Windows error where the namespace
// cannot be used in the function declaration below without getting an error like:
//   'rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>':
//     no appropriate default constructor available
template<typename AllocatorT>
using PublisherOptionsWithAllocator = rclcpp::PublisherOptionsWithAllocator<AllocatorT>;
template<typename AllocatorT>
using SubscriptionOptionsWithAllocator = rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>;

template<typename AllocatorT>
PublisherOptionsWithAllocator<AllocatorT>
create_default_publisher_options()
{
  return rclcpp::PublisherOptionsWithAllocator<AllocatorT>();
}

template<typename AllocatorT>
SubscriptionOptionsWithAllocator<AllocatorT>
create_default_subscription_options()
{
  return rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>();
}

/// LifecycleNode for creating lifecycle components
/**
 * has lifecycle nodeinterface for configuring this node.
 */
class LifecycleNode : public node_interfaces::LifecycleNodeInterface,
  public std::enable_shared_from_this<LifecycleNode>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(LifecycleNode)

  /// Create a new lifecycle node with the specified name.
  /**
   * \param[in] node_name Name of the node.
   * \param[in] options Additional options to control creation of the node.
   */
  RCLCPP_LIFECYCLE_PUBLIC
  explicit LifecycleNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// Create a node based on the node name and a rclcpp::Context.
  /**
   * \param[in] node_name Name of the node.
   * \param[in] namespace_ Namespace of the node.
   * \param[in] options Additional options to control creation of the node.
   */
  RCLCPP_LIFECYCLE_PUBLIC
  LifecycleNode(
    const std::string & node_name,
    const std::string & namespace_,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  RCLCPP_LIFECYCLE_PUBLIC
  virtual ~LifecycleNode();

  /// Get the name of the node.
  /**
   * \return The name of the node.
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const char *
  get_name() const;

  /// Get the namespace of the node
  /**
   * \return The namespace of the node.
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const char *
  get_namespace() const;

  /// Get the logger of the node.
  /**
   * \return The logger of the node.
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::Logger
  get_logger() const;

  /// Create and return a callback group.
  /**
   * \param[in] group_type callback group type to create by this method.
   * \return a callback group
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::CallbackGroup::SharedPtr
  create_callback_group(rclcpp::CallbackGroupType group_type);

  /// Return the list of callback groups in the node.
  /**
   * \return list of callback groups in the node.
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const std::vector<rclcpp::CallbackGroup::WeakPtr> &
  get_callback_groups() const;

  /// Create and return a Publisher.
  /**
   * \param[in] topic_name The topic for this publisher to publish on.
   * \param[in] qos The Quality of Service settings for this publisher.
   * \param[in] options The publisher options for this publisher.
   * \return Shared pointer to the created lifecycle publisher.
   */
  template<typename MessageT, typename AllocatorT = std::allocator<void>>
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MessageT, AllocatorT>>
  create_publisher(
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    const PublisherOptionsWithAllocator<AllocatorT> & options = (
      create_default_publisher_options<AllocatorT>()
    )
  );

  /// Create and return a Subscription.
  /**
   * \param[in] topic_name The topic to subscribe on.
   * \param[in] callback The user-defined callback function.
   * \param[in] qos The quality of service for this subscription.
   * \param[in] options The subscription options for this subscription.
   * \param[in] msg_mem_strat The message memory strategy to use for allocating messages.
   * \return Shared pointer to the created subscription.
   */
  template<
    typename MessageT,
    typename CallbackT,
    typename AllocatorT = std::allocator<void>,
    typename CallbackMessageT =
    typename rclcpp::subscription_traits::has_message_type<CallbackT>::type,
    typename SubscriptionT = rclcpp::Subscription<MessageT, AllocatorT>,
    typename MessageMemoryStrategyT = rclcpp::message_memory_strategy::MessageMemoryStrategy<
      CallbackMessageT,
      AllocatorT
    >>
  std::shared_ptr<SubscriptionT>
  create_subscription(
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    CallbackT && callback,
    const SubscriptionOptionsWithAllocator<AllocatorT> & options =
    create_default_subscription_options<AllocatorT>(),
    typename MessageMemoryStrategyT::SharedPtr msg_mem_strat = (
      MessageMemoryStrategyT::create_default()
    )
  );

  /// Create a timer.
  /**
   * \param[in] period Time interval between triggers of the callback.
   * \param[in] callback User-defined callback function.
   * \param[in] group Callback group to execute this timer's callback in.
   */
  template<typename DurationRepT = int64_t, typename DurationT = std::milli, typename CallbackT>
  typename rclcpp::WallTimer<CallbackT>::SharedPtr
  create_wall_timer(
    std::chrono::duration<DurationRepT, DurationT> period,
    CallbackT callback,
    rclcpp::CallbackGroup::SharedPtr group = nullptr);

  /// Create and return a Client.
  /**
   * \sa rclcpp::Node::create_client
   */
  template<typename ServiceT>
  typename rclcpp::Client<ServiceT>::SharedPtr
  create_client(
    const std::string & service_name,
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_services_default,
    rclcpp::CallbackGroup::SharedPtr group = nullptr);

  /// Create and return a Service.
  /**
   * \sa rclcpp::Node::create_service
   */
  template<typename ServiceT, typename CallbackT>
  typename rclcpp::Service<ServiceT>::SharedPtr
  create_service(
    const std::string & service_name,
    CallbackT && callback,
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_services_default,
    rclcpp::CallbackGroup::SharedPtr group = nullptr);

  /// Declare and initialize a parameter, return the effective value.
  /**
   * \sa rclcpp::Node::declare_parameter
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const rclcpp::ParameterValue &
  declare_parameter(
    const std::string & name,
    const rclcpp::ParameterValue & default_value = rclcpp::ParameterValue(),
    const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
    rcl_interfaces::msg::ParameterDescriptor());

  /// Declare and initialize a parameter with a type.
  /**
   * \sa rclcpp::Node::declare_parameter
   */
  template<typename ParameterT>
  auto
  declare_parameter(
    const std::string & name,
    const ParameterT & default_value,
    const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
    rcl_interfaces::msg::ParameterDescriptor());

  /// Declare and initialize several parameters with the same namespace and type.
  /**
   * \sa rclcpp::Node::declare_parameters
   */
  template<typename ParameterT>
  std::vector<ParameterT>
  declare_parameters(
    const std::string & namespace_,
    const std::map<std::string, ParameterT> & parameters);

  /// Declare and initialize several parameters with the same namespace and type.
  /**
   * \sa rclcpp::Node::declare_parameters
   */
  template<typename ParameterT>
  std::vector<ParameterT>
  declare_parameters(
    const std::string & namespace_,
    const std::map<
      std::string,
      std::pair<ParameterT, rcl_interfaces::msg::ParameterDescriptor>
    > & parameters);

  /// Undeclare a previously declared parameter.
  /**
   * \sa rclcpp::Node::undeclare_parameter
   */
  RCLCPP_LIFECYCLE_PUBLIC
  void
  undeclare_parameter(const std::string & name);

  /// Return true if a given parameter is declared.
  /**
   * \sa rclcpp::Node::has_parameter
   */
  RCLCPP_LIFECYCLE_PUBLIC
  bool
  has_parameter(const std::string & name) const;

  /// Set a single parameter.
  /**
   * \sa rclcpp::Node::set_parameter
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rcl_interfaces::msg::SetParametersResult
  set_parameter(const rclcpp::Parameter & parameter);

  /// Set one or more parameters, one at a time.
  /**
   * \sa rclcpp::Node::set_parameters
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::vector<rcl_interfaces::msg::SetParametersResult>
  set_parameters(const std::vector<rclcpp::Parameter> & parameters);

  /// Set one or more parameters, all at once.
  /**
   * \sa rclcpp::Node::set_parameters_atomically
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rcl_interfaces::msg::SetParametersResult
  set_parameters_atomically(const std::vector<rclcpp::Parameter> & parameters);

  /// Return the parameter by the given name.
  /**
   * \sa rclcpp::Node::get_parameter
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::Parameter
  get_parameter(const std::string & name) const;

  /// Get the value of a parameter by the given name, and return true if it was set.
  /**
   * \sa rclcpp::Node::get_parameter
   */
  RCLCPP_LIFECYCLE_PUBLIC
  bool
  get_parameter(
    const std::string & name,
    rclcpp::Parameter & parameter) const;

  /// Get the value of a parameter by the given name, and return true if it was set.
  /**
   * \sa rclcpp::Node::get_parameter
   */
  template<typename ParameterT>
  bool
  get_parameter(const std::string & name, ParameterT & parameter) const;

  /// Get the parameter value, or the "alternative_value" if not set, and assign it to "parameter".
  /**
   * \sa rclcpp::Node::get_parameter_or
   */
  template<typename ParameterT>
  bool
  get_parameter_or(
    const std::string & name,
    ParameterT & value,
    const ParameterT & alternative_value) const;

  /// Return the parameters by the given parameter names.
  /**
   * \sa rclcpp::Node::get_parameters
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::vector<rclcpp::Parameter>
  get_parameters(const std::vector<std::string> & names) const;

  /// Get the parameter values for all parameters that have a given prefix.
  /**
   * \sa rclcpp::Node::get_parameters
   */
  template<typename MapValueT>
  bool
  get_parameters(
    const std::string & prefix,
    std::map<std::string, MapValueT> & values) const;

  /// Return the parameter descriptor for the given parameter name.
  /**
   * \sa rclcpp::Node::describe_parameter
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rcl_interfaces::msg::ParameterDescriptor
  describe_parameter(const std::string & name) const;

  /// Return a vector of parameter descriptors, one for each of the given names.
  /**
   * \sa rclcpp::Node::describe_parameters
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::vector<rcl_interfaces::msg::ParameterDescriptor>
  describe_parameters(const std::vector<std::string> & names) const;

  /// Return a vector of parameter types, one for each of the given names.
  /**
   * \sa rclcpp::Node::get_parameter_types
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::vector<uint8_t>
  get_parameter_types(const std::vector<std::string> & names) const;

  /// Return a list of parameters with any of the given prefixes, up to the given depth.
  /**
   * \sa rclcpp::Node::list_parameters
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rcl_interfaces::msg::ListParametersResult
  list_parameters(const std::vector<std::string> & prefixes, uint64_t depth) const;

  using OnSetParametersCallbackHandle =
    rclcpp::node_interfaces::OnSetParametersCallbackHandle;
  using OnParametersSetCallbackType =
    rclcpp::node_interfaces::NodeParametersInterface::OnParametersSetCallbackType;

  /// Add a callback for when parameters are being set.
  /**
   * \sa rclcpp::Node::add_on_set_parameters_callback
   */
  RCLCPP_LIFECYCLE_PUBLIC
  RCUTILS_WARN_UNUSED
  rclcpp_lifecycle::LifecycleNode::OnSetParametersCallbackHandle::SharedPtr
  add_on_set_parameters_callback(
    rclcpp_lifecycle::LifecycleNode::OnParametersSetCallbackType callback);

  /// Remove a callback registered with `add_on_set_parameters_callback`.
  /**
   * \sa rclcpp::Node::remove_on_set_parameters_callback
   */
  RCLCPP_LIFECYCLE_PUBLIC
  void
  remove_on_set_parameters_callback(
    const rclcpp_lifecycle::LifecycleNode::OnSetParametersCallbackHandle * const handler);

  /// Register a callback to be called anytime a parameter is about to be changed.
  /**
   * \deprecated Use add_on_set_parameters_callback instead.
   * \sa rclcpp::Node::set_on_parameters_set_callback
   */
  [[deprecated("use add_on_set_parameters_callback(OnParametersSetCallbackType callback) instead")]]
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp_lifecycle::LifecycleNode::OnParametersSetCallbackType
  set_on_parameters_set_callback(
    rclcpp_lifecycle::LifecycleNode::OnParametersSetCallbackType callback);

  /// Return a vector of existing node names (string).
  /**
   * \sa rclcpp::Node::get_node_names
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::vector<std::string>
  get_node_names() const;

  /// Return a map of existing topic names to list of topic types.
  /**
   * \sa rclcpp::Node::get_topic_names_and_types
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::map<std::string, std::vector<std::string>>
  get_topic_names_and_types(bool no_demangle = false) const;

  /// Return a map of existing service names to list of topic types.
  /**
   * \sa rclcpp::Node::get_service_names_and_types
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::map<std::string, std::vector<std::string>>
  get_service_names_and_types() const;

  /// Return a map of existing service names to list of service types for a specific node.
  /**
   * This function only considers services - not clients.
   *
   * \param[in] node_name name of the node
   * \param[in] namespace_ namespace of the node
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::map<std::string, std::vector<std::string>>
  get_service_names_and_types_by_node(
    const std::string & node_name,
    const std::string & namespace_) const;

  /// Return the number of publishers that are advertised on a given topic.
  /**
   * \sa rclcpp::Node::count_publishers
   */
  RCLCPP_LIFECYCLE_PUBLIC
  size_t
  count_publishers(const std::string & topic_name) const;

  /// Return the number of subscribers who have created a subscription for a given topic.
  /**
   * \sa rclcpp::Node::count_subscribers
   */
  RCLCPP_LIFECYCLE_PUBLIC
  size_t
  count_subscribers(const std::string & topic_name) const;

  /// Return the topic endpoint information about publishers on a given topic.
  /**
   * \sa rclcpp::Node::get_publishers_info_by_topic
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::vector<rclcpp::TopicEndpointInfo>
  get_publishers_info_by_topic(const std::string & topic_name, bool no_mangle = false) const;

  /// Return the topic endpoint information about subscriptions on a given topic.
  /**
   * \sa rclcpp::Node::get_subscriptions_info_by_topic
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::vector<rclcpp::TopicEndpointInfo>
  get_subscriptions_info_by_topic(const std::string & topic_name, bool no_mangle = false) const;

  /// Return a graph event, which will be set anytime a graph change occurs.
  /* The graph Event object is a loan which must be returned.
   * The Event object is scoped and therefore to return the load just let it go
   * out of scope.
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::Event::SharedPtr
  get_graph_event();

  /// Wait for a graph event to occur by waiting on an Event to become set.
  /* The given Event must be acquire through the get_graph_event() method.
   *
   * \throws InvalidEventError if the given event is nullptr
   * \throws EventNotRegisteredError if the given event was not acquired with
   *   get_graph_event().
   */
  RCLCPP_LIFECYCLE_PUBLIC
  void
  wait_for_graph_change(
    rclcpp::Event::SharedPtr event,
    std::chrono::nanoseconds timeout);

  /// Get a clock as a non-const shared pointer which is managed by the node.
  /**
   * \sa rclcpp::node_interfaces::NodeClock::get_clock
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::Clock::SharedPtr
  get_clock();

  /// Get a clock as a const shared pointer which is managed by the node.
  /**
   * \sa rclcpp::node_interfaces::NodeClock::get_clock
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::Clock::ConstSharedPtr
  get_clock() const;

  /// Returns current time from the time source specified by clock_type.
  /**
   * \sa rclcpp::Clock::now
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::Time
  now() const;

  /// Return the Node's internal NodeBaseInterface implementation.
  /**
   * \sa rclcpp::Node::get_node_base_interface
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface();

  /// Return the Node's internal NodeClockInterface implementation.
  /**
   * \sa rclcpp::Node::get_node_clock_interface
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr
  get_node_clock_interface();

  /// Return the Node's internal NodeGraphInterface implementation.
  /**
   * \sa rclcpp::Node::get_node_graph_interface
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr
  get_node_graph_interface();

  /// Return the Node's internal NodeLoggingInterface implementation.
  /**
   * \sa rclcpp::Node::get_node_logging_interface
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr
  get_node_logging_interface();

  /// Return the Node's internal NodeTimersInterface implementation.
  /**
   * \sa rclcpp::Node::get_node_timers_interface
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr
  get_node_timers_interface();

  /// Return the Node's internal NodeTopicsInterface implementation.
  /**
   * \sa rclcpp::Node::get_node_topics_interface
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr
  get_node_topics_interface();

  /// Return the Node's internal NodeServicesInterface implementation.
  /**
   * \sa rclcpp::Node::get_node_services_interface
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr
  get_node_services_interface();

  /// Return the Node's internal NodeParametersInterface implementation.
  /**
   * \sa rclcpp::Node::get_node_parameters_interface
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
  get_node_parameters_interface();

  /// Return the Node's internal NodeParametersInterface implementation.
  /**
   * \sa rclcpp::Node::get_node_time_source_interface
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::node_interfaces::NodeTimeSourceInterface::SharedPtr
  get_node_time_source_interface();

  /// Return the Node's internal NodeWaitablesInterface implementation.
  /**
   * \sa rclcpp::Node::get_node_waitables_interface
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr
  get_node_waitables_interface();

  /// Return the NodeOptions used when creating this node.
  /**
   * \sa rclcpp::Node::get_node_options
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const rclcpp::NodeOptions &
  get_node_options() const;

  //
  // LIFECYCLE COMPONENTS
  //
  /// Return the current State.
  /**
   * \return the current state
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const State &
  get_current_state();

  /// Return a list with the available states.
  /**
   * \return list with the available states.
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::vector<State>
  get_available_states();

  /// Return a list with the available transitions.
  /**
   * \return list with the available transitions.
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::vector<Transition>
  get_available_transitions();

  /// Trigger the specified transition.
  /*
   * \return the new state after this transition
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const State &
  trigger_transition(const Transition & transition);

  /// Trigger the specified transition and get the callback return code.
  /*
   * \param[out] cb_return_code transition callback return code
   * \return the new state after this transition
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const State &
  trigger_transition(
    const Transition & transition, LifecycleNodeInterface::CallbackReturn & cb_return_code);

  /// Trigger the specified transition based on an id.
  /*
   * \return the new state after this transition
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const State &
  trigger_transition(uint8_t transition_id);

  /// Trigger the specified transition based on an id and get the callback return code.
  /*
   * \param[out] cb_return_code transition callback return code
   * \return the new state after this transition
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const State &
  trigger_transition(
    uint8_t transition_id, LifecycleNodeInterface::CallbackReturn & cb_return_code);

  /// Trigger the configure transition
  /*
   * \param[out] cb_return_code transition callback return code.
   * \return the new state after this transition
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const State &
  configure();

  /// Trigger the configure transition and get the callback return code.
  /*
   * \param[out] cb_return_code transition callback return code
   * \return the new state after this transition
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const State &
  configure(LifecycleNodeInterface::CallbackReturn & cb_return_code);

  /// Trigger the cleanup transition.
  /*
   * \return the new state after this transition
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const State &
  cleanup();

  /// Trigger the cleanup transition and get the callback return code.
  /*
   * \param[out] cb_return_code transition callback return code
   * \return the new state after this transition
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const State &
  cleanup(LifecycleNodeInterface::CallbackReturn & cb_return_code);

  /// Trigger the activate transition.
  /*
   * \return the new state after this transition
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const State &
  activate();

  /// Trigger the activate transition and get the callback return code.
  /*
   * \param[out] cb_return_code transition callback return code
   * \return the new state after this transition
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const State &
  activate(LifecycleNodeInterface::CallbackReturn & cb_return_code);

  /// Trigger the deactivate transition
  /*
   * \return the new state after this transition
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const State &
  deactivate();

  /// Trigger the deactivate transition and get the callback return code.
  /*
   * \param[out] cb_return_code transition callback return code
   * \return the new state after this transition
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const State &
  deactivate(LifecycleNodeInterface::CallbackReturn & cb_return_code);

  /// Trigger the shutdown transition
  /*
   * \return the new state after this transition
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const State &
  shutdown();

  /// Trigger the shutdown transition and get the callback return code.
  /*
   * \param[out] cb_return_code transition callback return code
   * \return the new state after this transition
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const State &
  shutdown(LifecycleNodeInterface::CallbackReturn & cb_return_code);

  /// Register the configure callback
  /**
   * This callback will be called when the transition to this state is triggered
   * \param[in] fcn callback function to call
   * \return always true
   */
  RCLCPP_LIFECYCLE_PUBLIC
  bool
  register_on_configure(std::function<LifecycleNodeInterface::CallbackReturn(const State &)> fcn);

  /// Register the cleanup callback
  /**
   * This callback will be called when the transition to this state is triggered
   * \param[in] fcn callback function to call
   * \return always true
   */
  RCLCPP_LIFECYCLE_PUBLIC
  bool
  register_on_cleanup(std::function<LifecycleNodeInterface::CallbackReturn(const State &)> fcn);

  /// Register the shutdown callback
  /**
   * This callback will be called when the transition to this state is triggered
   * \param[in] fcn callback function to call
   * \return always true
   */
  RCLCPP_LIFECYCLE_PUBLIC
  bool
  register_on_shutdown(std::function<LifecycleNodeInterface::CallbackReturn(const State &)> fcn);

  /// Register the activate callback
  /**
   * This callback will be called when the transition to this state is triggered
   * \param[in] fcn callback function to call
   * \return always true
   */
  RCLCPP_LIFECYCLE_PUBLIC
  bool
  register_on_activate(std::function<LifecycleNodeInterface::CallbackReturn(const State &)> fcn);

  /// Register the deactivate callback
  /**
   * This callback will be called when the transition to this state is triggered
   * \param[in] fcn callback function to call
   * \return always true
   */
  RCLCPP_LIFECYCLE_PUBLIC
  bool
  register_on_deactivate(std::function<LifecycleNodeInterface::CallbackReturn(const State &)> fcn);

  /// Register the error callback
  /**
   * This callback will be called when the transition to this state is triggered
   * \param[in] fcn callback function to call
   * \return always true
   */
  RCLCPP_LIFECYCLE_PUBLIC
  bool
  register_on_error(std::function<LifecycleNodeInterface::CallbackReturn(const State &)> fcn);

protected:
  RCLCPP_LIFECYCLE_PUBLIC
  void
  add_publisher_handle(std::shared_ptr<rclcpp_lifecycle::LifecyclePublisherInterface> pub);

  RCLCPP_LIFECYCLE_PUBLIC
  void
  add_timer_handle(std::shared_ptr<rclcpp::TimerBase> timer);

private:
  RCLCPP_DISABLE_COPY(LifecycleNode)

  RCLCPP_LIFECYCLE_PUBLIC
  bool
  group_in_node(rclcpp::CallbackGroup::SharedPtr group);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;
  rclcpp::node_interfaces::NodeTimeSourceInterface::SharedPtr node_time_source_;
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_;

  const rclcpp::NodeOptions node_options_;

  class LifecycleNodeInterfaceImpl;
  std::unique_ptr<LifecycleNodeInterfaceImpl> impl_;
};

}  // namespace rclcpp_lifecycle

#ifndef RCLCPP_LIFECYCLE__LIFECYCLE_NODE_IMPL_HPP_
// Template implementations
#include "rclcpp_lifecycle/lifecycle_node_impl.hpp"
#endif

#endif  // RCLCPP_LIFECYCLE__LIFECYCLE_NODE_HPP_
