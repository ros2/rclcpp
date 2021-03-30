// Copyright 2019 Intel Corporation
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

#ifndef RCLCPP__PARAMETER_EVENT_HANDLER_HPP_
#define RCLCPP__PARAMETER_EVENT_HANDLER_HPP_

#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/create_subscription.hpp"
#include "rclcpp/node_interfaces/get_node_base_interface.hpp"
#include "rclcpp/node_interfaces/get_node_topics_interface.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"

namespace rclcpp
{

struct ParameterCallbackHandle
{
  RCLCPP_SMART_PTR_DEFINITIONS(ParameterCallbackHandle)

  using ParameterCallbackType = std::function<void (const rclcpp::Parameter &)>;

  std::string parameter_name;
  std::string node_name;
  ParameterCallbackType callback;
};

struct ParameterEventCallbackHandle
{
  RCLCPP_SMART_PTR_DEFINITIONS(ParameterEventCallbackHandle)

  using ParameterEventCallbackType =
    std::function<void (const rcl_interfaces::msg::ParameterEvent &)>;

  ParameterEventCallbackType callback;
};

/// A class used to "handle" (monitor and respond to) changes to parameters.
/**
 * The ParameterEventHandler class allows for the monitoring of changes to node parameters,
 * either a node's own parameters or parameters owned by other nodes in the system.
 * Multiple parameter callbacks can be set and will be invoked when the specified parameter
 * changes.
 *
 * The first step is to instantiate a ParameterEventHandler, providing a ROS node to use
 * to create any required subscriptions:
 *
 *   auto param_handler = std::make_shared<rclcpp::ParameterEventHandler>(node);
 *
 * Next, you can supply a callback to the add_parameter_callback method, as follows:
 *
 *   auto cb1 = [&node](const rclcpp::Parameter & p) {
 *      RCLCPP_INFO(
 *        node->get_logger(),
 *        "cb1: Received an update to parameter \"%s\" of type %s: \"%ld\"",
 *        p.get_name().c_str(),
 *        p.get_type_name().c_str(),
 *        p.as_int());
 *   };
 *   auto handle1 = param_handler->add_parameter_callback("an_int_param", cb1);
 *
 * In this case, we didn't supply a node name (the third, optional, parameter) so the
 * default will be to monitor for changes to the "an_int_param" parameter associated with
 * the ROS node supplied in the ParameterEventHandler constructor.
 * The callback, a lambda function in this case, simply prints out the value of the parameter.
 *
 * You may also monitor for changes to parameters in other nodes by supplying the node
 * name to add_parameter_callback:
 *
 *   auto cb2 = [&node](const rclcpp::Parameter & p) {
 *       RCLCPP_INFO(
 *         node->get_logger(),
 *         "cb2: Received an update to parameter \"%s\" of type: %s: \"%s\"",
 *         p.get_name().c_str(),
 *         p.get_type_name().c_str(),
 *         p.as_string().c_str());
 *     };
 *   auto handle2 = param_handler->add_parameter_callback(
 *     "some_remote_param_name", cb2, "some_remote_node_name");
 *
 * In this case, the callback will be invoked whenever "some_remote_param_name" changes
 * on remote node "some_remote_node_name".
 *
 * To remove a parameter callback, call remove_parameter_callback, passing the handle returned
 * from add_parameter_callback:
 *
 *   param_handler->remove_parameter_callback(handle2);
 *
 * You can also monitor for *all* parameter changes, using add_parameter_event_callback.
 * In this case, the callback will be invoked whenever any parameter changes in the system.
 * You are likely interested in a subset of these parameter changes, so in the callback it
 * is convenient to use a regular expression on the node names or namespaces of interest.
 * For example:
 *
 *   auto cb3 =
 *     [fqn, remote_param_name, &node](const rcl_interfaces::msg::ParameterEvent & event) {
 *       // Look for any updates to parameters in "/a_namespace" as well as any parameter changes
 *       // to our own node ("this_node")
 *       std::regex re("(/a_namespace/.*)|(/this_node)");
 *       if (regex_match(event.node, re)) {
 *         // Now that we know the event matches the regular expression we scanned for, we can
 *         // use 'get_parameter_from_event' to get a specific parameter name that we're looking for
 *         rclcpp::Parameter p;
 *         if (rclcpp::ParameterEventsSubscriber::get_parameter_from_event(
 *             event, p, remote_param_name, fqn))
 *         {
 *           RCLCPP_INFO(
 *             node->get_logger(),
 *             "cb3: Received an update to parameter \"%s\" of type: %s: \"%s\"",
 *             p.get_name().c_str(),
 *             p.get_type_name().c_str(),
 *             p.as_string().c_str());
 *         }
 *
 *         // You can also use 'get_parameter*s*_from_event' to enumerate all changes that came
 *         // in on this event
 *         auto params = rclcpp::ParameterEventsSubscriber::get_parameters_from_event(event);
 *         for (auto & p : params) {
 *           RCLCPP_INFO(
 *             node->get_logger(),
 *             "cb3: Received an update to parameter \"%s\" of type: %s: \"%s\"",
 *             p.get_name().c_str(),
 *             p.get_type_name().c_str(),
 *             p.value_to_string().c_str());
 *         }
 *       }
 *     };
 *   auto handle3 = param_handler->add_parameter_event_callback(cb3);
 *
 * For both parameter callbacks and parameter event callbacks, when multiple callbacks are added,
 * the callbacks are invoked last-in, first-called order (LIFO).
 *
 * To remove a parameter event callback, use:
 *
 *   param_handler->remove_event_parameter_callback(handle);
 */
class ParameterEventHandler
{
public:
  /// Construct a parameter events monitor.
  /**
   * \param[in] node The node to use to create any required subscribers.
   * \param[in] qos The QoS settings to use for any subscriptions.
   */
  template<typename NodeT>
  ParameterEventHandler(
    NodeT node,
    const rclcpp::QoS & qos =
    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_parameter_events)))
  {
    node_base_ = rclcpp::node_interfaces::get_node_base_interface(node);
    auto node_topics = rclcpp::node_interfaces::get_node_topics_interface(node);

    event_subscription_ = rclcpp::create_subscription<rcl_interfaces::msg::ParameterEvent>(
      node_topics, "/parameter_events", qos,
      std::bind(&ParameterEventHandler::event_callback, this, std::placeholders::_1));
  }

  using ParameterEventCallbackType =
    ParameterEventCallbackHandle::ParameterEventCallbackType;

  /// Set a callback for all parameter events.
  /**
   * This function may be called multiple times to set multiple parameter event callbacks.
   *
   * \param[in] callback Function callback to be invoked on parameter updates.
   * \returns A handle used to refer to the callback.
   */
  RCLCPP_PUBLIC
  ParameterEventCallbackHandle::SharedPtr
  add_parameter_event_callback(
    ParameterEventCallbackType callback);

  /// Remove parameter event callback registered with add_parameter_event_callback.
  /**
   * \param[in] callback_handle Handle of the callback to remove.
   */
  RCLCPP_PUBLIC
  void
  remove_parameter_event_callback(
    ParameterEventCallbackHandle::SharedPtr callback_handle);

  using ParameterCallbackType = ParameterCallbackHandle::ParameterCallbackType;

  /// Add a callback for a specified parameter.
  /**
   * If a node_name is not provided, defaults to the current node.
   *
   * \param[in] parameter_name Name of parameter to monitor.
   * \param[in] callback Function callback to be invoked upon parameter update.
   * \param[in] node_name Name of node which hosts the parameter.
   * \returns A handle used to refer to the callback.
   */
  RCLCPP_PUBLIC
  ParameterCallbackHandle::SharedPtr
  add_parameter_callback(
    const std::string & parameter_name,
    ParameterCallbackType callback,
    const std::string & node_name = "");

  /// Remove a parameter callback registered with add_parameter_callback.
  /**
   * The parameter name and node name are inspected from the callback handle. The callback handle
   * is erased from the list of callback handles on the {parameter_name, node_name} in the map.
   * An error is thrown if the handle does not exist and/or was already removed.
   *
   * \param[in] callback_handle Handle of the callback to remove.
   */
  RCLCPP_PUBLIC
  void
  remove_parameter_callback(
    ParameterCallbackHandle::SharedPtr callback_handle);

  /// Get an rclcpp::Parameter from a parameter event.
  /**
   * If a node_name is not provided, defaults to the current node.
   *
   * \param[in] event Event msg to be inspected.
   * \param[out] parameter Reference to rclcpp::Parameter to be assigned.
   * \param[in] parameter_name Name of parameter.
   * \param[in] node_name Name of node which hosts the parameter.
   * \returns Output parameter is set with requested parameter info and returns true if
   * requested parameter name and node is in event. Otherwise, returns false.
   */
  RCLCPP_PUBLIC
  static bool
  get_parameter_from_event(
    const rcl_interfaces::msg::ParameterEvent & event,
    rclcpp::Parameter & parameter,
    const std::string parameter_name,
    const std::string node_name = "");

  /// Get an rclcpp::Parameter from parameter event
  /**
   * If a node_name is not provided, defaults to the current node.
   *
   * The user is responsible to check if the returned parameter has been properly assigned.
   * By default, if the requested parameter is not found in the event, the returned parameter
   * has parameter value of type rclcpp::PARAMETER_NOT_SET.
   *
   * \param[in] event Event msg to be inspected.
   * \param[in] parameter_name Name of parameter.
   * \param[in] node_name Name of node which hosts the parameter.
   * \returns The resultant rclcpp::Parameter from the event.
   */
  RCLCPP_PUBLIC
  static rclcpp::Parameter
  get_parameter_from_event(
    const rcl_interfaces::msg::ParameterEvent & event,
    const std::string parameter_name,
    const std::string node_name = "");

  /// Get all rclcpp::Parameter values from a parameter event
  /**
   * \param[in] event Event msg to be inspected.
   * \returns A vector rclcpp::Parameter values from the event.
   */
  RCLCPP_PUBLIC
  static std::vector<rclcpp::Parameter>
  get_parameters_from_event(
    const rcl_interfaces::msg::ParameterEvent & event);

  using CallbacksContainerType = std::list<ParameterCallbackHandle::WeakPtr>;

protected:
  /// Callback for parameter events subscriptions.
  RCLCPP_PUBLIC
  void
  event_callback(const rcl_interfaces::msg::ParameterEvent & event);

  // Utility function for resolving node path.
  std::string resolve_path(const std::string & path);

  // Node interface used for base functionality
  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_;

  // *INDENT-OFF* Uncrustify doesn't handle indented public/private labels
  // Hash function for string pair required in std::unordered_map
  // See: https://stackoverflow.com/questions/35985960/c-why-is-boosthash-combine-the-best-way-to-combine-hash-values
  class StringPairHash
  {
  public:
    template<typename T>
    inline void hash_combine(std::size_t & seed, const T & v) const
    {
      std::hash<T> hasher;
      seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }

    inline size_t operator()(const std::pair<std::string, std::string> & s) const
    {
      size_t seed = 0;
      hash_combine(seed, s.first);
      hash_combine(seed, s.second);
      return seed;
    }
  };
  // *INDENT-ON*

  // Map container for registered parameters
  std::unordered_map<
    std::pair<std::string, std::string>,
    CallbacksContainerType,
    StringPairHash
  > parameter_callbacks_;

  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr event_subscription_;

  std::list<ParameterEventCallbackHandle::WeakPtr> event_callbacks_;

  std::recursive_mutex mutex_;
};

}  // namespace rclcpp

#endif  // RCLCPP__PARAMETER_EVENT_HANDLER_HPP_
