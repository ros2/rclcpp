// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__NODE_INTERFACES__NODE_INTERFACE_HANDLE_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_INTERFACE_HANDLE_HPP_

#include <memory>
#include <type_traits>

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

namespace rclcpp
{
namespace node_interfaces
{

/// A helper class for aggregating node interfaces
template<typename ... InterfaceTs>
class NodeInterfaceHandle : public std::enable_shared_from_this<NodeInterfaceHandle<InterfaceTs...>>
{
  static_assert(0 != sizeof ...(InterfaceTs), "Template parameters must be populated!")

public:
  RCLCPP_SMART_PTR_DEFINITIONS(NodeInterfaceHandle)

  /// Create a new node handle with no bound node interfaces.
  RCLCPP_PUBLIC
  NodeInterfaceHandle()
  : InterfaceTs()... {}

  /// Create a new node handle bound with the passed in node-like object's interfaces.
  /**
   * Specify which interfaces you want to bind using the template parameters by specifying
   * interface support classes to use. Any unmentioned interfaces will be unavailable to bind.
   *
   * You may use any of the available support classes in
   * node_interfaces/node_interface_handle_helpers.hpp:
   *   - Base:       Supports NodeBaseInterface
   *   - Clock:      Supports NodeClockInterface
   *   - Graph:      Supports NodeGraphInterface
   *   - Logging:    Supports NodeLoggingInterface
   *   - Parameters: Supports NodeParametersInterface
   *   - Services:   Supports NodeServicesInterface
   *   - TimeSource: Supports NodeTimeSourceInterface
   *   - Timers:     Supports NodeTimersInterface
   *   - Topics:     Supports NodeTopicsInterface
   *   - Waitables:  Supports NodeWaitablesInterface
   *
   * Or you can define your own interface support classes!
   *
   * Each of the support classes should define:
   *   - Default constructor
   *   - Templated constructor taking NodeT
   *   - get_node_<interface_name>_interface()
   *   - set_node_<interface_name>_interface()
   *
   * Usage example:
   *   - ```NodeInterfaceHandle<rclcpp::node_interfaces::Base>(node)```
   *     will bind just the NodeBaseInterface.
   *   - ```NodeInterfaceHandle<
   *          rclcpp::node_interfaces::Base, rclcpp::node_interfaces::Clock>(node)```
   *     will bind both the NodeBaseInterface and NodeClockInterface.
   *
   * \param[in] node Node-like object to bind the interfaces of.
   */
  template<typename NodeT>
  explicit NodeInterfaceHandle(const NodeT & node)
  : InterfaceTs(node)... {}
};


/// Create a new node handle bound with no node interfaces.
/**
 * Specify which interfaces you want to bind using the template parameters by specifying
 * interface support classes to use. Any unmentioned interfaces will be unavailable to bind.
 *
 * This method will return a NodeInterfaceHandle with no bound interfaces. You must set them using
 * ```NodeInterfaceHandle->set_<interface_name>_interface(InterfaceT::SharedPtr interface)```
 *
 * You may use any of the available support classes in
 * node_interfaces/node_interface_handle_helpers.hpp:
 *   - Base:       Supports NodeBaseInterface
 *   - Clock:      Supports NodeClockInterface
 *   - Graph:      Supports NodeGraphInterface
 *   - Logging:    Supports NodeLoggingInterface
 *   - Parameters: Supports NodeParametersInterface
 *   - Services:   Supports NodeServicesInterface
 *   - TimeSource: Supports NodeTimeSourceInterface
 *   - Timers:     Supports NodeTimersInterface
 *   - Topics:     Supports NodeTopicsInterface
 *   - Waitables:  Supports NodeWaitablesInterface
 *
 * Or you can define your own interface support classes!
 *
 * Each of the support classes should define:
 *   - Default constructor
 *   - Templated constructor taking NodeT
 *   - get_node_<interface_name>_interface()
 *   - set_node_<interface_name>_interface()
 *
 * Usage example:
 *   - ```NodeInterfaceHandle<rclcpp::node_interfaces::Base>(node)```
 *     will bind just the NodeBaseInterface.
 *   - ```NodeInterfaceHandle<
 *          rclcpp::node_interfaces::Base, rclcpp::node_interfaces::Clock>(node)```
 *     will bind both the NodeBaseInterface and NodeClockInterface.
 *
 * \sa rclcpp::node_interfaces::NodeInterfaceHandle
 * \param[in] node Node-like object to bind the interfaces of.
 * \returns a NodeInterfaceHandle::SharedPtr supporting the stated interfaces, but bound with none
 *          of them
 */
template<typename ... InterfaceTs>
typename NodeInterfaceHandle<InterfaceTs...>::SharedPtr
get_node_interface_handle()
{
  static_assert(0 != sizeof ...(InterfaceTs), "Template parameters must be populated!")
  return std::make_shared<NodeInterfaceHandle<InterfaceTs...>>();
}

/// Create a new node handle bound with the passed in node-like object's interfaces.
/**
 * Specify which interfaces you want to bind using the template parameters by specifying
 * interface support classes to use. Any unmentioned interfaces will be unavailable to bind.
 *
 * You may use any of the available support classes in
 * node_interfaces/node_interface_handle_helpers.hpp:
 *   - Base:       Supports NodeBaseInterface
 *   - Clock:      Supports NodeClockInterface
 *   - Graph:      Supports NodeGraphInterface
 *   - Logging:    Supports NodeLoggingInterface
 *   - Parameters: Supports NodeParametersInterface
 *   - Services:   Supports NodeServicesInterface
 *   - TimeSource: Supports NodeTimeSourceInterface
 *   - Timers:     Supports NodeTimersInterface
 *   - Topics:     Supports NodeTopicsInterface
 *   - Waitables:  Supports NodeWaitablesInterface
 *
 * Or you can define your own interface support classes!
 *
 * Each of the support classes should define:
 *   - Default constructor
 *   - Templated constructor taking NodeT
 *   - get_node_<interface_name>_interface()
 *   - set_node_<interface_name>_interface()
 *
 * Usage example:
 *   - ```NodeInterfaceHandle<rclcpp::node_interfaces::Base>(node)```
 *     will bind just the NodeBaseInterface.
 *   - ```NodeInterfaceHandle<
 *          rclcpp::node_interfaces::Base, rclcpp::node_interfaces::Clock>(node)```
 *     will bind both the NodeBaseInterface and NodeClockInterface.
 *
 * \sa rclcpp::node_interfaces::NodeInterfaceHandle
 * \param[in] node Node-like object to bind the interfaces of.
 * \returns a NodeInterfaceHandle::SharedPtr bound with the node-like objects's interfaces
 */
template<typename ... InterfaceTs, typename NodeT>
typename NodeInterfaceHandle<InterfaceTs...>::SharedPtr
get_node_interface_handle(const NodeT & node)
{
  static_assert(0 != sizeof ...(InterfaceTs), "Template parameters must be populated!")
  return std::make_shared<NodeInterfaceHandle<InterfaceTs...>>(node);
}

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_INTERFACE_HANDLE_HPP_
