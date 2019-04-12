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
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/subscription_traits.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/// Node is the single point of entry for creating publishers and subscribers.
class Node : public std::enable_shared_from_this<Node>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Node)

  /// Create a new node with the specified name.
  /**
   * \param[in] node_name Name of the node.
   * \param[in] options Additional options to control creation of the node.
   */
  RCLCPP_PUBLIC
  explicit Node(
    const std::string & node_name,
    const NodeOptions & options = NodeOptions());

  /// Create a new node with the specified name.
  /**
   * \param[in] node_name Name of the node.
   * \param[in] namespace_ Namespace of the node.
   * \param[in] options Additional options to control creation of the node.
   */
  RCLCPP_PUBLIC
  explicit Node(
    const std::string & node_name,
    const std::string & namespace_,
    const NodeOptions & options = NodeOptions());

  RCLCPP_PUBLIC
  virtual ~Node();

  /// Get the name of the node.
  /** \return The name of the node. */
  RCLCPP_PUBLIC
  const char *
  get_name() const;

  /// Get the namespace of the node.
  /**
   * This namespace is the "node's" namespace, and therefore is not affected
   * by any sub-namespace's that may affect entities created with this instance.
   * Use get_effective_namespace() to get the full namespace used by entities.
   *
   * \sa get_sub_namespace()
   * \sa get_effective_namespace()
   * \return The namespace of the node.
   */
  RCLCPP_PUBLIC
  const char *
  get_namespace() const;

  /// Get the fully-qualified name of the node.
  /**
   * The fully-qualified name includes the local namespace and name of the node.
   */
  RCLCPP_PUBLIC
  const char *
  get_fully_qualified_name() const;

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
   * \param[in] options Additional options for the created Publisher.
   * \return Shared pointer to the created publisher.
   */
  template<
    typename MessageT,
    typename AllocatorT = std::allocator<void>,
    typename PublisherT = ::rclcpp::Publisher<MessageT, AllocatorT>>
  std::shared_ptr<PublisherT>
  create_publisher(
    const std::string & topic_name,
    size_t qos_history_depth,
    const PublisherOptionsWithAllocator<AllocatorT> &
    options = PublisherOptionsWithAllocator<AllocatorT>());

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
  // cppcheck-suppress syntaxError // bug in cppcheck 1.82 for [[deprecated]] on templated function
  [[deprecated(
    "use the create_publisher(const std::string &, size_t, const PublisherOptions<Alloc> & = "
    "PublisherOptions<Alloc>()) signature instead")]]
  std::shared_ptr<PublisherT>
  create_publisher(
    const std::string & topic_name,
    size_t qos_history_depth,
    std::shared_ptr<Alloc> allocator,
    IntraProcessSetting use_intra_process_comm = IntraProcessSetting::NodeDefault);

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
    std::shared_ptr<Alloc> allocator = nullptr,
    IntraProcessSetting use_intra_process_comm = IntraProcessSetting::NodeDefault);

  /// Create and return a Subscription.
  /**
   * \param[in] topic_name The topic to subscribe on.
   * \param[in] callback The user-defined callback function to receive a message
   * \param[in] qos_history_depth The depth of the subscription's incoming message queue.
   * \param[in] options Additional options for the creation of the Subscription.
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
    typename AllocatorT = std::allocator<void>,
    typename SubscriptionT = rclcpp::Subscription<
      typename rclcpp::subscription_traits::has_message_type<CallbackT>::type, AllocatorT>>
  std::shared_ptr<SubscriptionT>
  create_subscription(
    const std::string & topic_name,
    CallbackT && callback,
    size_t qos_history_depth,
    const SubscriptionOptionsWithAllocator<AllocatorT> &
    options = SubscriptionOptionsWithAllocator<AllocatorT>(),
    typename rclcpp::message_memory_strategy::MessageMemoryStrategy<
      typename rclcpp::subscription_traits::has_message_type<CallbackT>::type, AllocatorT
    >::SharedPtr
    msg_mem_strat = nullptr);

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
    std::shared_ptr<Alloc> allocator = nullptr,
    IntraProcessSetting use_intra_process_comm = IntraProcessSetting::NodeDefault);

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
  [[deprecated(
    "use the create_subscription(const std::string &, CallbackT &&, size_t, "
    "const SubscriptionOptions<Alloc> & = SubscriptionOptions<Alloc>(), ...) signature instead")]]
  std::shared_ptr<SubscriptionT>
  create_subscription(
    const std::string & topic_name,
    CallbackT && callback,
    size_t qos_history_depth,
    rclcpp::callback_group::CallbackGroup::SharedPtr group,
    bool ignore_local_publications = false,
    typename rclcpp::message_memory_strategy::MessageMemoryStrategy<
      typename rclcpp::subscription_traits::has_message_type<CallbackT>::type, Alloc>::SharedPtr
    msg_mem_strat = nullptr,
    std::shared_ptr<Alloc> allocator = nullptr,
    IntraProcessSetting use_intra_process_comm = IntraProcessSetting::NodeDefault);

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

  /// Set a map of parameters with the same prefix.
  /**
   * For each key in the map, a parameter with a name of "name.key" will be set
   * to the value in the map.
   *
   * \param[in] name The prefix of the parameters to set.
   * \param[in] values The parameters to set in the given prefix.
   */
  template<typename MapValueT>
  void
  set_parameters_if_not_set(
    const std::string & name,
    const std::map<std::string, MapValueT> & values);

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

  /// Assign the value of the map parameter if set into the values argument.
  /**
   * Parameter names that are part of a map are of the form "name.member".
   * This API gets all parameters that begin with "name", storing them into the
   * map with the name of the parameter and their value.
   * If there are no members in the named map, then the "values" argument is not changed.
   *
   * \param[in] name The prefix of the parameters to get.
   * \param[out] values The map of output values, with one std::string,MapValueT
   *                    per parameter.
   * \returns true if values was changed, false otherwise
   */
  template<typename MapValueT>
  bool
  get_parameters(
    const std::string & name,
    std::map<std::string, MapValueT> & values) const;

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

  /// Return the Node's internal NodeLoggingInterface implementation.
  RCLCPP_PUBLIC
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr
  get_node_logging_interface();

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

  /// Return the Node's internal NodeWaitablesInterface implementation.
  RCLCPP_PUBLIC
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr
  get_node_waitables_interface();

  /// Return the Node's internal NodeParametersInterface implementation.
  RCLCPP_PUBLIC
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
  get_node_parameters_interface();

  /// Return the Node's internal NodeParametersInterface implementation.
  RCLCPP_PUBLIC
  rclcpp::node_interfaces::NodeTimeSourceInterface::SharedPtr
  get_node_time_source_interface();

  /// Return the sub-namespace, if this is a sub-node, otherwise an empty string.
  /**
   * The returned sub-namespace is either the accumulated sub-namespaces which
   * were given to one-to-many create_sub_node() calls, or an empty string if
   * this is an original node instance, i.e. not a sub-node.
   *
   * For example, consider:
   *
   *   auto node = std::make_shared<rclcpp::Node>("my_node", "my_ns");
   *   node->get_sub_namespace();  // -> ""
   *   auto sub_node1 = node->create_sub_node("a");
   *   sub_node1->get_sub_namespace();  // -> "a"
   *   auto sub_node2 = sub_node1->create_sub_node("b");
   *   sub_node2->get_sub_namespace();  // -> "a/b"
   *   auto sub_node3 = node->create_sub_node("foo");
   *   sub_node3->get_sub_namespace();  // -> "foo"
   *   node->get_sub_namespace();  // -> ""
   *
   * get_namespace() will return the original node namespace, and will not
   * include the sub-namespace if one exists.
   * To get that you need to call the get_effective_namespace() method.
   *
   * \sa get_namespace()
   * \sa get_effective_namespace()
   * \return the sub-namespace string, not including the node's original namespace
   */
  RCLCPP_PUBLIC
  const std::string &
  get_sub_namespace() const;

  /// Return the effective namespace that is used when creating entities.
  /**
   * The returned namespace is a concatenation of the node namespace and the
   * accumulated sub-namespaces, which is used as the namespace when creating
   * entities which have relative names.
   *
   * For example, consider:
   *
   *   auto node = std::make_shared<rclcpp::Node>("my_node", "my_ns");
   *   node->get_effective_namespace();  // -> "/my_ns"
   *   auto sub_node1 = node->create_sub_node("a");
   *   sub_node1->get_effective_namespace();  // -> "/my_ns/a"
   *   auto sub_node2 = sub_node1->create_sub_node("b");
   *   sub_node2->get_effective_namespace();  // -> "/my_ns/a/b"
   *   auto sub_node3 = node->create_sub_node("foo");
   *   sub_node3->get_effective_namespace();  // -> "/my_ns/foo"
   *   node->get_effective_namespace();  // -> "/my_ns"
   *
   * \sa get_namespace()
   * \sa get_sub_namespace()
   * \return the sub-namespace string, not including the node's original namespace
   */
  RCLCPP_PUBLIC
  const std::string &
  get_effective_namespace() const;

  /// Create a sub-node, which will extend the namespace of all entities created with it.
  /**
   * A sub-node (short for subordinate node) is an instance of this class
   * which has been created using an existing instance of this class, but which
   * has an additional sub-namespace (short for subordinate namespace)
   * associated with it.
   * The sub-namespace will extend the node's namespace for the purpose of
   * creating additional entities, such as Publishers, Subscriptions, Service
   * Clients and Servers, and so on.
   *
   * By default, when an instance of this class is created using one of the
   * public constructors, it has no sub-namespace associated with it, and
   * therefore is not a sub-node.
   * That "normal" node instance may, however, be used to create further
   * instances of this class, based on the original instance, which have an
   * additional sub-namespace associated with them.
   * This may be done by using this method, create_sub_node().
   *
   * Furthermore, a sub-node may be used to create additional sub-node's, in
   * which case the sub-namespace passed to this function will further
   * extend the sub-namespace of the existing sub-node.
   * See get_sub_namespace() and get_effective_namespace() for examples.
   *
   * Note that entities which use absolute names are not affected by any
   * namespaces, neither the normal node namespace nor any sub-namespace.
   * Note also that the fully qualified node name is unaffected by a
   * sub-namespace.
   *
   * The sub-namespace should be relative, and an exception will be thrown if
   * the sub-namespace is absolute, i.e. if it starts with a leading '/'.
   *
   * \sa get_sub_namespace()
   * \sa get_effective_namespace()
   * \param[in] sub_namespace sub-namespace of the sub-node.
   * \return newly created sub-node
   * \throws NameValidationError if the sub-namespace is absolute, i.e. starts
   *   with a leading '/'.
   */
  RCLCPP_PUBLIC
  Node::SharedPtr
  create_sub_node(const std::string & sub_namespace);

  /// Return the NodeOptions used when creating this node.
  RCLCPP_PUBLIC
  const NodeOptions &
  get_node_options() const;

protected:
  /// Construct a sub-node, which will extend the namespace of all entities created with it.
  /**
   * \sa create_sub_node()
   *
   * \param[in] other The node from which a new sub-node is created.
   * \param[in] sub_namespace The sub-namespace of the sub-node.
   */
  RCLCPP_PUBLIC
  Node(
    const Node & other,
    const std::string & sub_namespace);

private:
  RCLCPP_DISABLE_COPY(Node)

  RCLCPP_PUBLIC
  bool
  group_in_node(callback_group::CallbackGroup::SharedPtr group);

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

  const NodeOptions node_options_;
  const std::string sub_namespace_;
  const std::string effective_namespace_;
};

}  // namespace rclcpp

#ifndef RCLCPP__NODE_IMPL_HPP_
// Template implementations
#include "node_impl.hpp"
#endif

#endif  // RCLCPP__NODE_HPP_
