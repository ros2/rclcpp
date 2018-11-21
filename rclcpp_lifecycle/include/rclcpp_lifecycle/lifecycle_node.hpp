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

#ifndef RCLCPP_LIFECYCLE__LIFECYCLE_NODE_HPP_
#define RCLCPP_LIFECYCLE__LIFECYCLE_NODE_HPP_

#include <string>
#include <map>
#include <vector>
#include <memory>

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
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_clock_interface.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"
#include "rclcpp/node_interfaces/node_timers_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/node_interfaces/node_waitables_interface.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/transition.hpp"
#include "rclcpp_lifecycle/visibility_control.h"

namespace rclcpp_lifecycle
{

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
   * \param[in] namespace_ Namespace of the node.
   * \param[in] use_intra_process_comms True to use the optimized intra-process communication
   * pipeline to pass messages between nodes in the same process using shared memory.
   */
  RCLCPP_LIFECYCLE_PUBLIC
  explicit LifecycleNode(
    const std::string & node_name,
    const std::string & namespace_ = "",
    bool use_intra_process_comms = false);

  /// Create a node based on the node name and a rclcpp::Context.
  /**
   * \param[in] node_name Name of the node.
   * \param[in] namespace_ Namespace of the node.
   * \param[in] context The context for the node (usually represents the state of a process).
   * \param[in] arguments Command line arguments that should apply only to this node.
   * \param[in] initial_parameters a list of initial values for parameters on the node.
   * This can be used to provide remapping rules that only affect one instance.
   * \param[in] use_global_arguments False to prevent node using arguments passed to the process.
   * \param[in] use_intra_process_comms True to use the optimized intra-process communication
   * pipeline to pass messages between nodes in the same process using shared memory.
   * \param[in] start_parameter_services True to setup ROS interfaces for accessing parameters
   * in the node.
   */
  RCLCPP_LIFECYCLE_PUBLIC
  LifecycleNode(
    const std::string & node_name,
    const std::string & namespace_,
    rclcpp::Context::SharedPtr context,
    const std::vector<std::string> & arguments,
    const std::vector<rclcpp::Parameter> & initial_parameters,
    bool use_global_arguments = true,
    bool use_intra_process_comms = false,
    bool start_parameter_services = true);

  RCLCPP_LIFECYCLE_PUBLIC
  virtual ~LifecycleNode();

  /// Get the name of the node.
  // \return The name of the node.
  RCLCPP_LIFECYCLE_PUBLIC
  const char *
  get_name() const;

  /// Get the namespace of the node.
  // \return The namespace of the node.
  RCLCPP_LIFECYCLE_PUBLIC
  const char *
  get_namespace() const;

  /// Get the logger of the node.
  /** \return The logger of the node. */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::Logger
  get_logger() const;

  /// Create and return a callback group.
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::callback_group::CallbackGroup::SharedPtr
  create_callback_group(rclcpp::callback_group::CallbackGroupType group_type);

  /// Return the list of callback groups in the node.
  RCLCPP_LIFECYCLE_PUBLIC
  const std::vector<rclcpp::callback_group::CallbackGroup::WeakPtr> &
  get_callback_groups() const;

  /// Create and return a Publisher.
  /**
   * \param[in] topic_name The topic for this publisher to publish on.
   * \param[in] qos_history_depth The depth of the publisher message queue.
   * \param[in] allocator allocator to use during publishing activities.
   * \return Shared pointer to the created publisher.
   */
  template<typename MessageT, typename Alloc = std::allocator<void>>
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MessageT, Alloc>>
  create_publisher(
    const std::string & topic_name, size_t qos_history_depth,
    std::shared_ptr<Alloc> allocator = nullptr);

  /// Create and return a LifecyclePublisher.
  /**
   * \param[in] topic_name The topic for this publisher to publish on.
   * \param[in] qos_profile The QoS settings for this publisher.
   * \param[in] allocator allocator to use during publishing activities.
   * \return Shared pointer to the created publisher.
   */
  template<typename MessageT, typename Alloc = std::allocator<void>>
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MessageT, Alloc>>
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
  template<
    typename MessageT,
    typename CallbackT,
    typename Alloc = std::allocator<void>,
    typename SubscriptionT = rclcpp::Subscription<MessageT, Alloc>>
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
   * \param[in] allocator allocator to be used during handling of subscription callbacks.
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
    typename SubscriptionT = rclcpp::Subscription<MessageT, Alloc>>
  std::shared_ptr<SubscriptionT>
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

  RCLCPP_LIFECYCLE_PUBLIC
  std::vector<rcl_interfaces::msg::SetParametersResult>
  set_parameters(const std::vector<rclcpp::Parameter> & parameters);

  RCLCPP_LIFECYCLE_PUBLIC
  rcl_interfaces::msg::SetParametersResult
  set_parameters_atomically(const std::vector<rclcpp::Parameter> & parameters);

  RCLCPP_LIFECYCLE_PUBLIC
  std::vector<rclcpp::Parameter>
  get_parameters(const std::vector<std::string> & names) const;

  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::Parameter
  get_parameter(const std::string & name) const;

  RCLCPP_LIFECYCLE_PUBLIC
  bool
  get_parameter(
    const std::string & name,
    rclcpp::Parameter & parameter) const;

  template<typename ParameterT>
  bool
  get_parameter(const std::string & name, ParameterT & parameter) const;

  RCLCPP_LIFECYCLE_PUBLIC
  std::vector<rcl_interfaces::msg::ParameterDescriptor>
  describe_parameters(const std::vector<std::string> & names) const;

  RCLCPP_LIFECYCLE_PUBLIC
  std::vector<uint8_t>
  get_parameter_types(const std::vector<std::string> & names) const;

  RCLCPP_LIFECYCLE_PUBLIC
  rcl_interfaces::msg::ListParametersResult
  list_parameters(const std::vector<std::string> & prefixes, uint64_t depth) const;

  /// Register the callback for parameter changes
  /**
   * \param[in] callback User defined function which is expected to atomically set parameters.
   * \note Repeated invocations of this function will overwrite previous callbacks
   */
  template<typename CallbackT>
  void
  register_param_change_callback(CallbackT && callback);

  RCLCPP_LIFECYCLE_PUBLIC
  std::vector<std::string>
  get_node_names() const;

  RCLCPP_LIFECYCLE_PUBLIC
  std::map<std::string, std::vector<std::string>>
  get_topic_names_and_types(bool no_demangle = false) const;

  RCLCPP_LIFECYCLE_PUBLIC
  std::map<std::string, std::vector<std::string>>
  get_service_names_and_types() const;

  RCLCPP_LIFECYCLE_PUBLIC
  size_t
  count_publishers(const std::string & topic_name) const;

  RCLCPP_LIFECYCLE_PUBLIC
  size_t
  count_subscribers(const std::string & topic_name) const;

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

  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::Clock::SharedPtr
  get_clock();

  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::Time
  now();

  /// Return the Node's internal NodeBaseInterface implementation.
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface();

  /// Return the Node's internal NodeClockInterface implementation.
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr
  get_node_clock_interface();

  /// Return the Node's internal NodeGraphInterface implementation.
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr
  get_node_graph_interface();

  /// Return the Node's internal NodeTimersInterface implementation.
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr
  get_node_timers_interface();

  /// Return the Node's internal NodeTopicsInterface implementation.
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr
  get_node_topics_interface();

  /// Return the Node's internal NodeServicesInterface implementation.
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr
  get_node_services_interface();

  /// Return the Node's internal NodeParametersInterface implementation.
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
  get_node_parameters_interface();

  /// Return the Node's internal NodeWaitablesInterface implementation.
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr
  get_node_waitables_interface();

  //
  // LIFECYCLE COMPONENTS
  //
  RCLCPP_LIFECYCLE_PUBLIC
  const State &
  get_current_state();

  RCLCPP_LIFECYCLE_PUBLIC
  std::vector<State>
  get_available_states();

  RCLCPP_LIFECYCLE_PUBLIC
  std::vector<Transition>
  get_available_transitions();

  /// trigger the specified transition
  /*
   * return the new state after this transition
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const State &
  trigger_transition(const Transition & transition);

  RCLCPP_LIFECYCLE_PUBLIC
  const State &
  trigger_transition(
    const Transition & transition, LifecycleNodeInterface::CallbackReturn & cb_return_code);

  RCLCPP_LIFECYCLE_PUBLIC
  const State &
  trigger_transition(uint8_t transition_id);

  RCLCPP_LIFECYCLE_PUBLIC
  const State &
  trigger_transition(
    uint8_t transition_id, LifecycleNodeInterface::CallbackReturn & cb_return_code);

  RCLCPP_LIFECYCLE_PUBLIC
  const State &
  configure();

  RCLCPP_LIFECYCLE_PUBLIC
  const State &
  configure(LifecycleNodeInterface::CallbackReturn & cb_return_code);

  RCLCPP_LIFECYCLE_PUBLIC
  const State &
  cleanup();

  RCLCPP_LIFECYCLE_PUBLIC
  const State &
  cleanup(LifecycleNodeInterface::CallbackReturn & cb_return_code);

  RCLCPP_LIFECYCLE_PUBLIC
  const State &
  activate();

  RCLCPP_LIFECYCLE_PUBLIC
  const State &
  activate(LifecycleNodeInterface::CallbackReturn & cb_return_code);

  RCLCPP_LIFECYCLE_PUBLIC
  const State &
  deactivate();

  RCLCPP_LIFECYCLE_PUBLIC
  const State &
  deactivate(LifecycleNodeInterface::CallbackReturn & cb_return_code);

  RCLCPP_LIFECYCLE_PUBLIC
  const State &
  shutdown();

  RCLCPP_LIFECYCLE_PUBLIC
  const State &
  shutdown(LifecycleNodeInterface::CallbackReturn & cb_return_code);

  RCLCPP_LIFECYCLE_PUBLIC
  bool
  register_on_configure(std::function<LifecycleNodeInterface::CallbackReturn(const State &)> fcn);

  RCLCPP_LIFECYCLE_PUBLIC
  bool
  register_on_cleanup(std::function<LifecycleNodeInterface::CallbackReturn(const State &)> fcn);

  RCLCPP_LIFECYCLE_PUBLIC
  bool
  register_on_shutdown(std::function<LifecycleNodeInterface::CallbackReturn(const State &)> fcn);

  RCLCPP_LIFECYCLE_PUBLIC
  bool
  register_on_activate(std::function<LifecycleNodeInterface::CallbackReturn(const State &)> fcn);

  RCLCPP_LIFECYCLE_PUBLIC
  bool
  register_on_deactivate(std::function<LifecycleNodeInterface::CallbackReturn(const State &)> fcn);

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
  group_in_node(rclcpp::callback_group::CallbackGroup::SharedPtr group);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_;
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_;

  bool use_intra_process_comms_;

  class LifecycleNodeInterfaceImpl;
  std::unique_ptr<LifecycleNodeInterfaceImpl> impl_;
};

}  // namespace rclcpp_lifecycle

#ifndef RCLCPP_LIFECYCLE__LIFECYCLE_NODE_IMPL_HPP_
// Template implementations
#include "rclcpp_lifecycle/lifecycle_node_impl.hpp"
#endif

#endif  // RCLCPP_LIFECYCLE__LIFECYCLE_NODE_HPP_
