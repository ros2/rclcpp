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
#include <functional>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
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
#include "rclcpp/generic_publisher.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/message_memory_strategy.hpp"
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
#include "rclcpp/node_options.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/qos.hpp"
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
   * \throws InvalidNamespaceError if the namespace is invalid
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
   * \throws InvalidNamespaceError if the namespace is invalid
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
   * \return fully-qualified name of the node.
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
  rclcpp::CallbackGroup::SharedPtr
  create_callback_group(
    rclcpp::CallbackGroupType group_type,
    bool automatically_add_to_executor_with_node = true);

  /// Return the list of callback groups in the node.
  RCLCPP_PUBLIC
  const std::vector<rclcpp::CallbackGroup::WeakPtr> &
  get_callback_groups() const;

  /// Iterate over the callback groups in the node, calling the given function on each valid one.
  /**
   * This method is called in a thread-safe way, and also makes sure to only call the given
   * function on those items that are still valid.
   *
   * \param[in] func The callback function to call on each valid callback group.
   */
  RCLCPP_PUBLIC
  void
  for_each_callback_group(const node_interfaces::NodeBaseInterface::CallbackGroupFunction & func);

  /// Create and return a Publisher.
  /**
   * The rclcpp::QoS has several convenient constructors, including a
   * conversion constructor for size_t, which mimics older API's that
   * allows just a string and size_t to create a publisher.
   *
   * For example, all of these cases will work:
   *
   * ```cpp
   * pub = node->create_publisher<MsgT>("chatter", 10);  // implicitly KeepLast
   * pub = node->create_publisher<MsgT>("chatter", QoS(10));  // implicitly KeepLast
   * pub = node->create_publisher<MsgT>("chatter", QoS(KeepLast(10)));
   * pub = node->create_publisher<MsgT>("chatter", QoS(KeepAll()));
   * pub = node->create_publisher<MsgT>("chatter", QoS(1).best_effort().durability_volatile());
   * {
   *   rclcpp::QoS custom_qos(KeepLast(10), rmw_qos_profile_sensor_data);
   *   pub = node->create_publisher<MsgT>("chatter", custom_qos);
   * }
   * ```
   *
   * The publisher options may optionally be passed as the third argument for
   * any of the above cases.
   *
   * \param[in] topic_name The topic for this publisher to publish on.
   * \param[in] qos The Quality of Service settings for the publisher.
   * \param[in] options Additional options for the created Publisher.
   * \return Shared pointer to the created publisher.
   */
  template<
    typename MessageT,
    typename AllocatorT = std::allocator<void>,
    typename PublisherT = rclcpp::Publisher<MessageT, AllocatorT>>
  std::shared_ptr<PublisherT>
  create_publisher(
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    const PublisherOptionsWithAllocator<AllocatorT> & options =
    PublisherOptionsWithAllocator<AllocatorT>()
  );

  /// Create and return a Subscription.
  /**
   * \param[in] topic_name The topic to subscribe on.
   * \param[in] qos QoS profile for Subcription.
   * \param[in] callback The user-defined callback function to receive a message
   * \param[in] options Additional options for the creation of the Subscription.
   * \param[in] msg_mem_strat The message memory strategy to use for allocating messages.
   * \return Shared pointer to the created subscription.
   */
  template<
    typename MessageT,
    typename CallbackT,
    typename AllocatorT = std::allocator<void>,
    typename CallbackMessageT =
    typename rclcpp::subscription_traits::has_message_type<CallbackT>::type,
    typename SubscriptionT = rclcpp::Subscription<CallbackMessageT, AllocatorT>,
    typename MessageMemoryStrategyT = rclcpp::message_memory_strategy::MessageMemoryStrategy<
      CallbackMessageT,
      AllocatorT
    >
  >
  std::shared_ptr<SubscriptionT>
  create_subscription(
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    CallbackT && callback,
    const SubscriptionOptionsWithAllocator<AllocatorT> & options =
    SubscriptionOptionsWithAllocator<AllocatorT>(),
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
   * \param[in] service_name The topic to service on.
   * \param[in] qos_profile rmw_qos_profile_t Quality of service profile for client.
   * \param[in] group Callback group to call the service.
   * \return Shared pointer to the created client.
   */
  template<typename ServiceT>
  typename rclcpp::Client<ServiceT>::SharedPtr
  create_client(
    const std::string & service_name,
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_services_default,
    rclcpp::CallbackGroup::SharedPtr group = nullptr);

  /// Create and return a Service.
  /**
   * \param[in] service_name The topic to service on.
   * \param[in] callback User-defined callback function.
   * \param[in] qos_profile rmw_qos_profile_t Quality of service profile for client.
   * \param[in] group Callback group to call the service.
   * \return Shared pointer to the created service.
   */
  template<typename ServiceT, typename CallbackT>
  typename rclcpp::Service<ServiceT>::SharedPtr
  create_service(
    const std::string & service_name,
    CallbackT && callback,
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_services_default,
    rclcpp::CallbackGroup::SharedPtr group = nullptr);

  /// Create and return a GenericPublisher.
  /**
   * The returned pointer will never be empty, but this function can throw various exceptions, for
   * instance when the message's package can not be found on the AMENT_PREFIX_PATH.
   *
   * \param[in] topic_name Topic name
   * \param[in] topic_type Topic type
   * \param[in] qos %QoS settings
   * \param options %Publisher options.
   * Not all publisher options are currently respected, the only relevant options for this
   * publisher are `event_callbacks`, `use_default_callbacks`, and `%callback_group`.
   * \return Shared pointer to the created generic publisher.
   */
  template<typename AllocatorT = std::allocator<void>>
  std::shared_ptr<rclcpp::GenericPublisher> create_generic_publisher(
    const std::string & topic_name,
    const std::string & topic_type,
    const rclcpp::QoS & qos,
    const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options = (
      rclcpp::PublisherOptionsWithAllocator<AllocatorT>()
    )
  );

  /// Create and return a GenericSubscription.
  /**
   * The returned pointer will never be empty, but this function can throw various exceptions, for
   * instance when the message's package can not be found on the AMENT_PREFIX_PATH.
   *
   * \param[in] topic_name Topic name
   * \param[in] topic_type Topic type
   * \param[in] qos %QoS settings
   * \param[in] callback Callback for new messages of serialized form
   * \param[in] options %Subscription options.
   * Not all subscription options are currently respected, the only relevant options for this
 * subscription are `event_callbacks`, `use_default_callbacks`, `ignore_local_publications`, and
 * `%callback_group`.
   * \return Shared pointer to the created generic subscription.
   */
  template<typename AllocatorT = std::allocator<void>>
  std::shared_ptr<rclcpp::GenericSubscription> create_generic_subscription(
    const std::string & topic_name,
    const std::string & topic_type,
    const rclcpp::QoS & qos,
    std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback,
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options = (
      rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>()
    )
  );

  /// Declare and initialize a parameter, return the effective value.
  /**
   * This method is used to declare that a parameter exists on this node.
   * If, at run-time, the user has provided an initial value then it will be
   * set in this method, otherwise the given default_value will be set.
   * In either case, the resulting value is returned, whether or not it is
   * based on the default value or the user provided initial value.
   *
   * If no parameter_descriptor is given, then the default values from the
   * message definition will be used, e.g. read_only will be false.
   *
   * The name and type in the given rcl_interfaces::msg::ParameterDescriptor
   * are ignored, and should be specified using the name argument to this
   * function and the default value's type instead.
   *
   * If `ignore_override` is `true`, the parameter override will be ignored.
   *
   * This method, if successful, will result in any callback registered with
   * add_on_set_parameters_callback to be called.
   * If that callback prevents the initial value for the parameter from being
   * set then rclcpp::exceptions::InvalidParameterValueException is thrown.
   *
   * The returned reference will remain valid until the parameter is
   * undeclared.
   *
   * \param[in] name The name of the parameter.
   * \param[in] default_value An initial value to be used if at run-time user
   *   did not override it.
   * \param[in] parameter_descriptor An optional, custom description for
   *   the parameter.
   * \param[in] ignore_override When `true`, the parameter override is ignored.
   *    Default to `false`.
   * \return A const reference to the value of the parameter.
   * \throws rclcpp::exceptions::ParameterAlreadyDeclaredException if parameter
   *   has already been declared.
   * \throws rclcpp::exceptions::InvalidParametersException if a parameter
   *   name is invalid.
   * \throws rclcpp::exceptions::InvalidParameterValueException if initial
   *   value fails to be set.
   * \throws rclcpp::exceptions::InvalidParameterTypeException
   *   if the type of the default value or override is wrong.
   */
  RCLCPP_PUBLIC
  const rclcpp::ParameterValue &
  declare_parameter(
    const std::string & name,
    const rclcpp::ParameterValue & default_value,
    const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
    rcl_interfaces::msg::ParameterDescriptor(),
    bool ignore_override = false);

  /// Declare and initialize a parameter, return the effective value.
  /**
   * Same as the previous one, but a default value is not provided and the user
   * must provide a parameter override of the correct type.
   *
   * \param[in] name The name of the parameter.
   * \param[in] type Desired type of the parameter, which will enforced at runtime.
   * \param[in] parameter_descriptor An optional, custom description for
   *   the parameter.
   * \param[in] ignore_override When `true`, the parameter override is ignored.
   *    Default to `false`.
   * \return A const reference to the value of the parameter.
   * \throws Same as the previous overload taking a default value.
   * \throws rclcpp::exceptions::InvalidParameterTypeException
   *   if an override is not provided or the provided override is of the wrong type.
   */
  RCLCPP_PUBLIC
  const rclcpp::ParameterValue &
  declare_parameter(
    const std::string & name,
    rclcpp::ParameterType type,
    const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
    rcl_interfaces::msg::ParameterDescriptor{},
    bool ignore_override = false);

  /// Declare a parameter
  [[deprecated(
    "declare_parameter() with only a name is deprecated and will be deleted in the future.\n" \
    "If you want to declare a parameter that won't change type without a default value use:\n" \
    "`node->declare_parameter<ParameterT>(name)`, where e.g. ParameterT=int64_t.\n\n" \
    "If you want to declare a parameter that can dynamically change type use:\n" \
    "```\n" \
    "rcl_interfaces::msg::ParameterDescriptor descriptor;\n" \
    "descriptor.dynamic_typing = true;\n" \
    "node->declare_parameter(name, rclcpp::ParameterValue{}, descriptor);\n" \
    "```"
  )]]
  RCLCPP_PUBLIC
  const rclcpp::ParameterValue &
  declare_parameter(const std::string & name);

  /// Declare and initialize a parameter with a type.
  /**
   * See the non-templated declare_parameter() on this class for details.
   *
   * If the type of the default value, and therefore also the type of return
   * value, differs from the initial value provided in the node options, then
   * a rclcpp::exceptions::InvalidParameterTypeException may be thrown.
   * To avoid this, use the declare_parameter() method which returns an
   * rclcpp::ParameterValue instead.
   *
   * Note, this method cannot return a const reference, because extending the
   * lifetime of a temporary only works recursively with member initializers,
   * and cannot be extended to members of a class returned.
   * The return value of this class is a copy of the member of a ParameterValue
   * which is returned by the other version of declare_parameter().
   * See also:
   *
   *   - https://en.cppreference.com/w/cpp/language/lifetime
   *   - https://herbsutter.com/2008/01/01/gotw-88-a-candidate-for-the-most-important-const/
   *   - https://www.youtube.com/watch?v=uQyT-5iWUow (cppnow 2018 presentation)
   */
  template<typename ParameterT>
  auto
  declare_parameter(
    const std::string & name,
    const ParameterT & default_value,
    const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
    rcl_interfaces::msg::ParameterDescriptor(),
    bool ignore_override = false);

  /// Declare and initialize a parameter with a type.
  /**
   * See the non-templated declare_parameter() on this class for details.
   */
  template<typename ParameterT>
  auto
  declare_parameter(
    const std::string & name,
    const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
    rcl_interfaces::msg::ParameterDescriptor(),
    bool ignore_override = false);

  /// Declare and initialize several parameters with the same namespace and type.
  /**
   * For each key in the map, a parameter with a name of "namespace.key"
   * will be set to the value in the map.
   * The resulting value for each declared parameter will be returned.
   *
   * The name expansion is naive, so if you set the namespace to be "foo.",
   * then the resulting parameter names will be like "foo..key".
   * However, if the namespace is an empty string, then no leading '.' will be
   * placed before each key, which would have been the case when naively
   * expanding "namespace.key".
   * This allows you to declare several parameters at once without a namespace.
   *
   * The map contains default values for parameters.
   * There is another overload which takes the std::pair with the default value
   * and descriptor.
   *
   * If `ignore_overrides` is `true`, all the overrides of the parameters declared
   * by the function call will be ignored.
   *
   * This method, if successful, will result in any callback registered with
   * add_on_set_parameters_callback to be called, once for each parameter.
   * If that callback prevents the initial value for any parameter from being
   * set then rclcpp::exceptions::InvalidParameterValueException is thrown.
   *
   * \param[in] namespace_ The namespace in which to declare the parameters.
   * \param[in] parameters The parameters to set in the given namespace.
   * \param[in] ignore_overrides When `true`, the parameters overrides are ignored.
   *    Default to `false`.
   * \throws rclcpp::exceptions::ParameterAlreadyDeclaredException if parameter
   *   has already been declared.
   * \throws rclcpp::exceptions::InvalidParametersException if a parameter
   *   name is invalid.
   * \throws rclcpp::exceptions::InvalidParameterValueException if initial
   *   value fails to be set.
   */
  template<typename ParameterT>
  std::vector<ParameterT>
  declare_parameters(
    const std::string & namespace_,
    const std::map<std::string, ParameterT> & parameters,
    bool ignore_overrides = false);

  /// Declare and initialize several parameters with the same namespace and type.
  /**
   * This version will take a map where the value is a pair, with the default
   * parameter value as the first item and a parameter descriptor as the second.
   *
   * See the simpler declare_parameters() on this class for more details.
   */
  template<typename ParameterT>
  std::vector<ParameterT>
  declare_parameters(
    const std::string & namespace_,
    const std::map<
      std::string,
      std::pair<ParameterT, rcl_interfaces::msg::ParameterDescriptor>
    > & parameters,
    bool ignore_overrides = false);

  /// Undeclare a previously declared parameter.
  /**
   * This method will not cause a callback registered with
   * add_on_set_parameters_callback to be called.
   *
   * \param[in] name The name of the parameter to be undeclared.
   * \throws rclcpp::exceptions::ParameterNotDeclaredException if the parameter
   *   has not been declared.
   * \throws rclcpp::exceptions::ParameterImmutableException if the parameter
   *   was create as read_only (immutable).
   */
  RCLCPP_PUBLIC
  void
  undeclare_parameter(const std::string & name);

  /// Return true if a given parameter is declared.
  /**
   * \param[in] name The name of the parameter to check for being declared.
   * \return true if the parameter name has been declared, otherwise false.
   */
  RCLCPP_PUBLIC
  bool
  has_parameter(const std::string & name) const;

  /// Set a single parameter.
  /**
   * Set the given parameter and then return result of the set action.
   *
   * If the parameter has not been declared this function may throw the
   * rclcpp::exceptions::ParameterNotDeclaredException exception, but only if
   * the node was not created with the
   * rclcpp::NodeOptions::allow_undeclared_parameters set to true.
   * If undeclared parameters are allowed, then the parameter is implicitly
   * declared with the default parameter meta data before being set.
   * Parameter overrides are ignored by set_parameter.
   *
   * This method will result in any callback registered with
   * add_on_set_parameters_callback to be called.
   * If the callback prevents the parameter from being set, then it will be
   * reflected in the SetParametersResult that is returned, but no exception
   * will be thrown.
   *
   * If the value type of the parameter is rclcpp::PARAMETER_NOT_SET, and the
   * existing parameter type is something else, then the parameter will be
   * implicitly undeclared.
   * This will result in a parameter event indicating that the parameter was
   * deleted.
   *
   * \param[in] parameter The parameter to be set.
   * \return The result of the set action.
   * \throws rclcpp::exceptions::ParameterNotDeclaredException if the parameter
   *   has not been declared and undeclared parameters are not allowed.
   */
  RCLCPP_PUBLIC
  rcl_interfaces::msg::SetParametersResult
  set_parameter(const rclcpp::Parameter & parameter);

  /// Set one or more parameters, one at a time.
  /**
   * Set the given parameters, one at a time, and then return result of each
   * set action.
   *
   * Parameters are set in the order they are given within the input vector.
   *
   * Like set_parameter, if any of the parameters to be set have not first been
   * declared, and undeclared parameters are not allowed (the default), then
   * this method will throw rclcpp::exceptions::ParameterNotDeclaredException.
   *
   * If setting a parameter fails due to not being declared, then the
   * parameters which have already been set will stay set, and no attempt will
   * be made to set the parameters which come after.
   *
   * If a parameter fails to be set due to any other reason, like being
   * rejected by the user's callback (basically any reason other than not
   * having been declared beforehand), then that is reflected in the
   * corresponding SetParametersResult in the vector returned by this function.
   *
   * This method will result in any callback registered with
   * add_on_set_parameters_callback to be called, once for each parameter.
   * If the callback prevents the parameter from being set, then, as mentioned
   * before, it will be reflected in the corresponding SetParametersResult
   * that is returned, but no exception will be thrown.
   *
   * Like set_parameter() this method will implicitly undeclare parameters
   * with the type rclcpp::PARAMETER_NOT_SET.
   *
   * \param[in] parameters The vector of parameters to be set.
   * \return The results for each set action as a vector.
   * \throws rclcpp::exceptions::ParameterNotDeclaredException if any parameter
   *   has not been declared and undeclared parameters are not allowed.
   */
  RCLCPP_PUBLIC
  std::vector<rcl_interfaces::msg::SetParametersResult>
  set_parameters(const std::vector<rclcpp::Parameter> & parameters);

  /// Set one or more parameters, all at once.
  /**
   * Set the given parameters, all at one time, and then aggregate result.
   *
   * Behaves like set_parameter, except that it sets multiple parameters,
   * failing all if just one of the parameters are unsuccessfully set.
   * Either all of the parameters are set or none of them are set.
   *
   * Like set_parameter and set_parameters, this method may throw an
   * rclcpp::exceptions::ParameterNotDeclaredException exception if any of the
   * parameters to be set have not first been declared.
   * If the exception is thrown then none of the parameters will have been set.
   *
   * This method will result in any callback registered with
   * add_on_set_parameters_callback to be called, just one time.
   * If the callback prevents the parameters from being set, then it will be
   * reflected in the SetParametersResult which is returned, but no exception
   * will be thrown.
   *
   * If you pass multiple rclcpp::Parameter instances with the same name, then
   * only the last one in the vector (forward iteration) will be set.
   *
   * Like set_parameter() this method will implicitly undeclare parameters
   * with the type rclcpp::PARAMETER_NOT_SET.
   *
   * \param[in] parameters The vector of parameters to be set.
   * \return The aggregate result of setting all the parameters atomically.
   * \throws rclcpp::exceptions::ParameterNotDeclaredException if any parameter
   *   has not been declared and undeclared parameters are not allowed.
   */
  RCLCPP_PUBLIC
  rcl_interfaces::msg::SetParametersResult
  set_parameters_atomically(const std::vector<rclcpp::Parameter> & parameters);

  /// Return the parameter by the given name.
  /**
   * If the parameter has not been declared, then this method may throw the
   * rclcpp::exceptions::ParameterNotDeclaredException exception.
   *
   * If undeclared parameters are allowed, see the node option
   * rclcpp::NodeOptions::allow_undeclared_parameters, then this method will
   * not throw an exception, and instead return a default initialized
   * rclcpp::Parameter, which has a type of
   * rclcpp::ParameterType::PARAMETER_NOT_SET.
   *
   * \param[in] name The name of the parameter to get.
   * \return The requested parameter inside of a rclcpp parameter object.
   * \throws rclcpp::exceptions::ParameterNotDeclaredException if the parameter
   *   has not been declared and undeclared parameters are not allowed.
   */
  RCLCPP_PUBLIC
  rclcpp::Parameter
  get_parameter(const std::string & name) const;

  /// Get the value of a parameter by the given name, and return true if it was set.
  /**
   * This method will never throw the
   * rclcpp::exceptions::ParameterNotDeclaredException exception, but will
   * instead return false if the parameter has not be previously declared.
   *
   * If the parameter was not declared, then the output argument for this
   * method which is called "parameter" will not be assigned a value.
   * If the parameter was declared, and therefore has a value, then it is
   * assigned into the "parameter" argument of this method.
   *
   * \param[in] name The name of the parameter to get.
   * \param[out] parameter The output storage for the parameter being retrieved.
   * \return true if the parameter was previously declared, otherwise false.
   */
  RCLCPP_PUBLIC
  bool
  get_parameter(const std::string & name, rclcpp::Parameter & parameter) const;

  /// Get the value of a parameter by the given name, and return true if it was set.
  /**
   * Identical to the non-templated version of this method, except that when
   * assigning the output argument called "parameter", this method will attempt
   * to coerce the parameter value into the type requested by the given
   * template argument, which may fail and throw an exception.
   *
   * If the parameter has not been declared, it will not attempt to coerce the
   * value into the requested type, as it is known that the type is not set.
   *
   * \throws rclcpp::ParameterTypeException if the requested type does not
   *   match the value of the parameter which is stored.
   */
  template<typename ParameterT>
  bool
  get_parameter(const std::string & name, ParameterT & parameter) const;

  /// Get the parameter value, or the "alternative_value" if not set, and assign it to "parameter".
  /**
   * If the parameter was not set, then the "parameter" argument is assigned
   * the "alternative_value".
   *
   * Like the version of get_parameter() which returns a bool, this method will
   * not throw the rclcpp::exceptions::ParameterNotDeclaredException exception.
   *
   * In all cases, the parameter is never set or declared within the node.
   *
   * \param[in] name The name of the parameter to get.
   * \param[out] parameter The output where the value of the parameter should be assigned.
   * \param[in] alternative_value Value to be stored in output if the parameter was not set.
   * \returns true if the parameter was set, false otherwise.
   */
  template<typename ParameterT>
  bool
  get_parameter_or(
    const std::string & name,
    ParameterT & parameter,
    const ParameterT & alternative_value) const;

  /// Return the parameters by the given parameter names.
  /**
   * Like get_parameters(), this method may throw the
   * rclcpp::exceptions::ParameterNotDeclaredException exception if the
   * requested parameter has not been declared and undeclared parameters are
   * not allowed.
   *
   * Also like get_parameters(), if undeclared parameters are allowed and the
   * parameter has not been declared, then the corresponding rclcpp::Parameter
   * will be default initialized and therefore have the type
   * rclcpp::ParameterType::PARAMETER_NOT_SET.
   *
   * \param[in] names The names of the parameters to be retrieved.
   * \return The parameters that were retrieved.
   * \throws rclcpp::exceptions::ParameterNotDeclaredException if any of the
   *   parameters have not been declared and undeclared parameters are not
   *   allowed.
   */
  RCLCPP_PUBLIC
  std::vector<rclcpp::Parameter>
  get_parameters(const std::vector<std::string> & names) const;

  /// Get the parameter values for all parameters that have a given prefix.
  /**
   * The "prefix" argument is used to list the parameters which are prefixed
   * with that prefix, see also list_parameters().
   *
   * The resulting list of parameter names are used to get the values of the
   * parameters.
   *
   * The names which are used as keys in the values map have the prefix removed.
   * For example, if you use the prefix "foo" and the parameters "foo.ping" and
   * "foo.pong" exist, then the returned map will have the keys "ping" and
   * "pong".
   *
   * An empty string for the prefix will match all parameters.
   *
   * If no parameters with the prefix are found, then the output parameter
   * "values" will be unchanged and false will be returned.
   * Otherwise, the parameter names and values will be stored in the map and
   * true will be returned to indicate "values" was mutated.
   *
   * This method will never throw the
   * rclcpp::exceptions::ParameterNotDeclaredException exception because the
   * action of listing the parameters is done atomically with getting the
   * values, and therefore they are only listed if already declared and cannot
   * be undeclared before being retrieved.
   *
   * Like the templated get_parameter() variant, this method will attempt to
   * coerce the parameter values into the type requested by the given
   * template argument, which may fail and throw an exception.
   *
   * \param[in] prefix The prefix of the parameters to get.
   * \param[out] values The map used to store the parameter names and values,
   *   respectively, with one entry per parameter matching prefix.
   * \returns true if output "values" was changed, false otherwise.
   * \throws rclcpp::ParameterTypeException if the requested type does not
   *   match the value of the parameter which is stored.
   */
  template<typename ParameterT>
  bool
  get_parameters(
    const std::string & prefix,
    std::map<std::string, ParameterT> & values) const;

  /// Return the parameter descriptor for the given parameter name.
  /**
   * Like get_parameters(), this method may throw the
   * rclcpp::exceptions::ParameterNotDeclaredException exception if the
   * requested parameter has not been declared and undeclared parameters are
   * not allowed.
   *
   * If undeclared parameters are allowed, then a default initialized
   * descriptor will be returned.
   *
   * \param[in] name The name of the parameter to describe.
   * \return The descriptor for the given parameter name.
   * \throws rclcpp::exceptions::ParameterNotDeclaredException if the
   *   parameter has not been declared and undeclared parameters are not
   *   allowed.
   * \throws std::runtime_error if the number of described parameters is more than one
   */
  RCLCPP_PUBLIC
  rcl_interfaces::msg::ParameterDescriptor
  describe_parameter(const std::string & name) const;

  /// Return a vector of parameter descriptors, one for each of the given names.
  /**
   * Like get_parameters(), this method may throw the
   * rclcpp::exceptions::ParameterNotDeclaredException exception if any of the
   * requested parameters have not been declared and undeclared parameters are
   * not allowed.
   *
   * If undeclared parameters are allowed, then a default initialized
   * descriptor will be returned for the undeclared parameter's descriptor.
   *
   * If the names vector is empty, then an empty vector will be returned.
   *
   * \param[in] names The list of parameter names to describe.
   * \return A list of parameter descriptors, one for each parameter given.
   * \throws rclcpp::exceptions::ParameterNotDeclaredException if any of the
   *   parameters have not been declared and undeclared parameters are not
   *   allowed.
   * \throws std::runtime_error if the number of described parameters is more than one
   */
  RCLCPP_PUBLIC
  std::vector<rcl_interfaces::msg::ParameterDescriptor>
  describe_parameters(const std::vector<std::string> & names) const;

  /// Return a vector of parameter types, one for each of the given names.
  /**
   * Like get_parameters(), this method may throw the
   * rclcpp::exceptions::ParameterNotDeclaredException exception if any of the
   * requested parameters have not been declared and undeclared parameters are
   * not allowed.
   *
   * If undeclared parameters are allowed, then the default type
   * rclcpp::ParameterType::PARAMETER_NOT_SET will be returned.
   *
   * \param[in] names The list of parameter names to get the types.
   * \return A list of parameter types, one for each parameter given.
   * \throws rclcpp::exceptions::ParameterNotDeclaredException if any of the
   *   parameters have not been declared and undeclared parameters are not
   *   allowed.
   */
  RCLCPP_PUBLIC
  std::vector<uint8_t>
  get_parameter_types(const std::vector<std::string> & names) const;

  /// Return a list of parameters with any of the given prefixes, up to the given depth.
  /**
   * \todo: properly document and test this method.
   */
  RCLCPP_PUBLIC
  rcl_interfaces::msg::ListParametersResult
  list_parameters(const std::vector<std::string> & prefixes, uint64_t depth) const;

  using OnSetParametersCallbackHandle =
    rclcpp::node_interfaces::OnSetParametersCallbackHandle;
  using OnParametersSetCallbackType =
    rclcpp::node_interfaces::NodeParametersInterface::OnParametersSetCallbackType;

  /// Add a callback for when parameters are being set.
  /**
   * The callback signature is designed to allow handling of any of the above
   * `set_parameter*` or `declare_parameter*` methods, and so it takes a const
   * reference to a vector of parameters to be set, and returns an instance of
   * rcl_interfaces::msg::SetParametersResult to indicate whether or not the
   * parameter should be set or not, and if not why.
   *
   * For an example callback:
   *
   * ```cpp
   * rcl_interfaces::msg::SetParametersResult
   * my_callback(const std::vector<rclcpp::Parameter> & parameters)
   * {
   *   rcl_interfaces::msg::SetParametersResult result;
   *   result.successful = true;
   *   for (const auto & parameter : parameters) {
   *     if (!some_condition) {
   *       result.successful = false;
   *       result.reason = "the reason it could not be allowed";
   *     }
   *   }
   *   return result;
   * }
   * ```
   *
   * You can see that the SetParametersResult is a boolean flag for success
   * and an optional reason that can be used in error reporting when it fails.
   *
   * This allows the node developer to control which parameters may be changed.
   *
   * It is considered bad practice to reject changes for "unknown" parameters as this prevents
   * other parts of the node (that may be aware of these parameters) from handling them.
   *
   * Note that the callback is called when declare_parameter() and its variants
   * are called, and so you cannot assume the parameter has been set before
   * this callback, so when checking a new value against the existing one, you
   * must account for the case where the parameter is not yet set.
   *
   * Some constraints like read_only are enforced before the callback is called.
   *
   * The callback may introspect other already set parameters (by calling any
   * of the {get,list,describe}_parameter() methods), but may *not* modify
   * other parameters (by calling any of the {set,declare}_parameter() methods)
   * or modify the registered callback itself (by calling the
   * add_on_set_parameters_callback() method).  If a callback tries to do any
   * of the latter things,
   * rclcpp::exceptions::ParameterModifiedInCallbackException will be thrown.
   *
   * The callback functions must remain valid as long as the
   * returned smart pointer is valid.
   * The returned smart pointer can be promoted to a shared version.
   *
   * Resetting or letting the smart pointer go out of scope unregisters the callback.
   * `remove_on_set_parameters_callback` can also be used.
   *
   * The registered callbacks are called when a parameter is set.
   * When a callback returns a not successful result, the remaining callbacks aren't called.
   * The order of the callback is the reverse from the registration order.
   *
   * \param callback The callback to register.
   * \returns A shared pointer. The callback is valid as long as the smart pointer is alive.
   * \throws std::bad_alloc if the allocation of the OnSetParametersCallbackHandle fails.
   */
  RCLCPP_PUBLIC
  RCUTILS_WARN_UNUSED
  OnSetParametersCallbackHandle::SharedPtr
  add_on_set_parameters_callback(OnParametersSetCallbackType callback);

  /// Remove a callback registered with `add_on_set_parameters_callback`.
  /**
   * Delete a handler returned by `add_on_set_parameters_callback`.
   *
   * e.g.:
   *
   *    `remove_on_set_parameters_callback(scoped_callback.get())`
   *
   * As an alternative, the smart pointer can be reset:
   *
   *    `scoped_callback.reset()`
   *
   * Supposing that `scoped_callback` was the only owner.
   *
   * Calling `remove_on_set_parameters_callback` more than once with the same handler,
   * or calling it after the shared pointer has been reset is an error.
   * Resetting or letting the smart pointer go out of scope after calling
   * `remove_on_set_parameters_callback` is not a problem.
   *
   * \param handler The callback handler to remove.
   * \throws std::runtime_error if the handler was not created with `add_on_set_parameters_callback`,
   *   or if it has been removed before.
   */
  RCLCPP_PUBLIC
  void
  remove_on_set_parameters_callback(const OnSetParametersCallbackHandle * const handler);

  /// Get the fully-qualified names of all available nodes.
  /**
   * The fully-qualified name includes the local namespace and name of the node.
   * \return A vector of fully-qualified names of nodes.
   */
  RCLCPP_PUBLIC
  std::vector<std::string>
  get_node_names() const;

  /// Return a map of existing topic names to list of topic types.
  /**
   * \return a map of existing topic names to list of topic types.
   * \throws std::runtime_error anything that rcl_error can throw
   */
  RCLCPP_PUBLIC
  std::map<std::string, std::vector<std::string>>
  get_topic_names_and_types() const;

  /// Return a map of existing service names to list of service types.
  /**
   * \return a map of existing service names to list of service types.
   * \throws std::runtime_error anything that rcl_error can throw
   */
  RCLCPP_PUBLIC
  std::map<std::string, std::vector<std::string>>
  get_service_names_and_types() const;

  /// Return a map of existing service names to list of service types for a specific node.
  /**
   * This function only considers services - not clients.
   * The returned names are the actual names used and do not have remap rules applied.
   *
   * \param[in] node_name name of the node.
   * \param[in] namespace_ namespace of the node.
   * \return a map of existing service names to list of service types.
   * \throws std::runtime_error anything that rcl_error can throw.
   */
  RCLCPP_PUBLIC
  std::map<std::string, std::vector<std::string>>
  get_service_names_and_types_by_node(
    const std::string & node_name,
    const std::string & namespace_) const;

  /// Return the number of publishers created for a given topic.
  /**
   * \param[in] topic_name the actual topic name used; it will not be automatically remapped.
   * \return number of publishers that have been created for the given topic.
   * \throws std::runtime_error if publishers could not be counted
   */
  RCLCPP_PUBLIC
  size_t
  count_publishers(const std::string & topic_name) const;

  /// Return the number of subscribers created for a given topic.
  /**
   * \param[in] topic_name the actual topic name used; it will not be automatically remapped.
   * \return number of subscribers that have been created for the given topic.
   * \throws std::runtime_error if subscribers could not be counted
   */
  RCLCPP_PUBLIC
  size_t
  count_subscribers(const std::string & topic_name) const;

  /// Return the topic endpoint information about publishers on a given topic.
  /**
   * The returned parameter is a list of topic endpoint information, where each item will contain
   * the node name, node namespace, topic type, endpoint type, topic endpoint's GID, and its QoS
   * profile.
   *
   * When the `no_mangle` parameter is `true`, the provided `topic_name` should be a valid topic
   * name for the middleware (useful when combining ROS with native middleware (e.g. DDS) apps).
   * When the `no_mangle` parameter is `false`, the provided `topic_name` should follow
   * ROS topic name conventions.
   *
   * `topic_name` may be a relative, private, or fully qualified topic name.
   * A relative or private topic will be expanded using this node's namespace and name.
   * The queried `topic_name` is not remapped.
   *
   * \param[in] topic_name the actual topic name used; it will not be automatically remapped.
   * \param[in] no_mangle if `true`, `topic_name` needs to be a valid middleware topic name,
   *   otherwise it should be a valid ROS topic name. Defaults to `false`.
   * \return a list of TopicEndpointInfo representing all the publishers on this topic.
   * \throws InvalidTopicNameError if the given topic_name is invalid.
   * \throws std::runtime_error if internal error happens.
   */
  RCLCPP_PUBLIC
  std::vector<rclcpp::TopicEndpointInfo>
  get_publishers_info_by_topic(const std::string & topic_name, bool no_mangle = false) const;

  /// Return the topic endpoint information about subscriptions on a given topic.
  /**
   * The returned parameter is a list of topic endpoint information, where each item will contain
   * the node name, node namespace, topic type, endpoint type, topic endpoint's GID, and its QoS
   * profile.
   *
   * When the `no_mangle` parameter is `true`, the provided `topic_name` should be a valid topic
   * name for the middleware (useful when combining ROS with native middleware (e.g. DDS) apps).
   * When the `no_mangle` parameter is `false`, the provided `topic_name` should follow
   * ROS topic name conventions.
   *
   * `topic_name` may be a relative, private, or fully qualified topic name.
   * A relative or private topic will be expanded using this node's namespace and name.
   * The queried `topic_name` is not remapped.
   *
   * \param[in] topic_name the actual topic name used; it will not be automatically remapped.
   * \param[in] no_mangle if `true`, `topic_name` needs to be a valid middleware topic name,
   *   otherwise it should be a valid ROS topic name. Defaults to `false`.
   * \return a list of TopicEndpointInfo representing all the subscriptions on this topic.
   * \throws InvalidTopicNameError if the given topic_name is invalid.
   * \throws std::runtime_error if internal error happens.
   */
  RCLCPP_PUBLIC
  std::vector<rclcpp::TopicEndpointInfo>
  get_subscriptions_info_by_topic(const std::string & topic_name, bool no_mangle = false) const;

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
   * \param[in] event pointer to an Event to wait for
   * \param[in] timeout nanoseconds to wait for the Event to change the state
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

  /// Get a clock as a non-const shared pointer which is managed by the node.
  /**
   * \sa rclcpp::node_interfaces::NodeClock::get_clock
   */
  RCLCPP_PUBLIC
  rclcpp::Clock::SharedPtr
  get_clock();

  /// Get a clock as a const shared pointer which is managed by the node.
  /**
   * \sa rclcpp::node_interfaces::NodeClock::get_clock
   */
  RCLCPP_PUBLIC
  rclcpp::Clock::ConstSharedPtr
  get_clock() const;

  /// Returns current time from the time source specified by clock_type.
  /**
   * \sa rclcpp::Clock::now
   */
  RCLCPP_PUBLIC
  Time
  now() const;

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

  /// Return the Node's internal NodeTimeSourceInterface implementation.
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
   * ```cpp
   * auto node = std::make_shared<rclcpp::Node>("my_node", "my_ns");
   * node->get_sub_namespace();  // -> ""
   * auto sub_node1 = node->create_sub_node("a");
   * sub_node1->get_sub_namespace();  // -> "a"
   * auto sub_node2 = sub_node1->create_sub_node("b");
   * sub_node2->get_sub_namespace();  // -> "a/b"
   * auto sub_node3 = node->create_sub_node("foo");
   * sub_node3->get_sub_namespace();  // -> "foo"
   * node->get_sub_namespace();  // -> ""
   * ```
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
   * ```cpp
   * auto node = std::make_shared<rclcpp::Node>("my_node", "my_ns");
   * node->get_effective_namespace();  // -> "/my_ns"
   * auto sub_node1 = node->create_sub_node("a");
   * sub_node1->get_effective_namespace();  // -> "/my_ns/a"
   * auto sub_node2 = sub_node1->create_sub_node("b");
   * sub_node2->get_effective_namespace();  // -> "/my_ns/a/b"
   * auto sub_node3 = node->create_sub_node("foo");
   * sub_node3->get_effective_namespace();  // -> "/my_ns/foo"
   * node->get_effective_namespace();  // -> "/my_ns"
   * ```
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
  rclcpp::Node::SharedPtr
  create_sub_node(const std::string & sub_namespace);

  /// Return the NodeOptions used when creating this node.
  RCLCPP_PUBLIC
  const rclcpp::NodeOptions &
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
  const std::string sub_namespace_;
  const std::string effective_namespace_;
};

}  // namespace rclcpp

#ifndef RCLCPP__NODE_IMPL_HPP_
// Template implementations
#include "node_impl.hpp"
#endif

#endif  // RCLCPP__NODE_HPP_
