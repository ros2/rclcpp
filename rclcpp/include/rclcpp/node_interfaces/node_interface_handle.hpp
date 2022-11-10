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
public:
  RCLCPP_SMART_PTR_DEFINITIONS(NodeInterfaceHandle)

  /// Create a new node handle with no bound node interfaces.
  RCLCPP_PUBLIC
  NodeInterfaceHandle()
  : base_(nullptr),
    clock_(nullptr),
    graph_(nullptr),
    logging_(nullptr),
    timers_(nullptr),
    topics_(nullptr),
    services_(nullptr),
    waitables_(nullptr),
    parameters_(nullptr),
    time_source_(nullptr)
  {}

  /// Create a new node handle bound with the passed in node-like object's interfaces.
  /**
   * Specify which interfaces you want to bind using the temlplate parameters. Any unbound
   * interfaces will be nullptr.
   *
   * Additionally, to bind all interfaces, specify AllInterfaces.
   *
   * For example:
   *   - ```NodeInterfaceHandle<>(node)``` will bind no interfaces.
   *   - ```NodeInterfaceHandle<rclcpp::node_interfaces::NodeBaseInterface>(node)```
   *     will bind just the NodeBaseInterface.
   *
   * \param[in] node Node-like object to bind the interfaces of.
   */
  template<typename NodeT>
  explicit NodeInterfaceHandle(const NodeT & node)
  {
    if constexpr (0 == sizeof ...(InterfaceTs)) {
      base_ = nullptr;
      clock_ = nullptr;
      graph_ = nullptr;
      logging_ = nullptr;
      timers_ = nullptr;
      topics_ = nullptr;
      services_ = nullptr;
      waitables_ = nullptr;
      parameters_ = nullptr;
      time_source_ = nullptr;
    } else {
      base_ = (
        (std::is_same_v<node_interfaces::NodeBaseInterface, InterfaceTs>|| ...) ||
        (std::is_same_v<node_interfaces::NodeBaseInterface::SharedPtr, InterfaceTs>|| ...)
        ) ? node->get_node_base_interface() : nullptr;
      clock_ = (
        (std::is_same_v<node_interfaces::NodeClockInterface, InterfaceTs>|| ...) ||
        (std::is_same_v<node_interfaces::NodeClockInterface::SharedPtr, InterfaceTs>|| ...)
        ) ? node->get_node_clock_interface() : nullptr;
      graph_ = (
        (std::is_same_v<node_interfaces::NodeGraphInterface, InterfaceTs>|| ...) ||
        (std::is_same_v<node_interfaces::NodeGraphInterface::SharedPtr, InterfaceTs>|| ...)
        ) ? node->get_node_graph_interface() : nullptr;
      logging_ = (
        (std::is_same_v<node_interfaces::NodeLoggingInterface, InterfaceTs>|| ...) ||
        (std::is_same_v<node_interfaces::NodeLoggingInterface::SharedPtr, InterfaceTs>|| ...)
        ) ? node->get_node_logging_interface() : nullptr;
      timers_ = (
        (std::is_same_v<node_interfaces::NodeTimersInterface, InterfaceTs>|| ...) ||
        (std::is_same_v<node_interfaces::NodeTimersInterface::SharedPtr, InterfaceTs>|| ...)
        ) ? node->get_node_timers_interface() : nullptr;
      topics_ = (
        (std::is_same_v<node_interfaces::NodeTopicsInterface, InterfaceTs>|| ...) ||
        (std::is_same_v<node_interfaces::NodeTopicsInterface::SharedPtr, InterfaceTs>|| ...)
        ) ? node->get_node_topics_interface() : nullptr;
      services_ = (
        (std::is_same_v<node_interfaces::NodeServicesInterface, InterfaceTs>|| ...) ||
        (std::is_same_v<node_interfaces::NodeServicesInterface::SharedPtr, InterfaceTs>|| ...)
        ) ? node->get_node_services_interface() : nullptr;
      waitables_ = (
        (std::is_same_v<node_interfaces::NodeWaitablesInterface, InterfaceTs>|| ...) ||
        (std::is_same_v<node_interfaces::NodeWaitablesInterface::SharedPtr, InterfaceTs>|| ...)
        ) ? node->get_node_waitables_interface() : nullptr;
      parameters_ = (
        (std::is_same_v<node_interfaces::NodeParametersInterface, InterfaceTs>|| ...) ||
        (std::is_same_v<node_interfaces::NodeParametersInterface::SharedPtr, InterfaceTs>|| ...)
        ) ? node->get_node_parameters_interface() : nullptr;
      time_source_ = (
        (std::is_same_v<node_interfaces::NodeTimeSourceInterface, InterfaceTs>|| ...) ||
        (std::is_same_v<node_interfaces::NodeTimeSourceInterface::SharedPtr, InterfaceTs>|| ...)
        ) ? node->get_node_time_source_interface() : nullptr;
    }
  }


  // Getters

  /// Return the bound NodeBaseInterface
  inline node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface() {return base_;}

  /// Return the bound NodeClockInterface
  inline node_interfaces::NodeClockInterface::SharedPtr
  get_node_clock_interface() {return clock_;}

  /// Return the bound NodeGraphInterface
  inline node_interfaces::NodeGraphInterface::SharedPtr
  get_node_graph_interface() {return graph_;}

  /// Return the bound NodeLoggingInterface
  inline node_interfaces::NodeLoggingInterface::SharedPtr
  get_node_logging_interface() {return logging_;}

  /// Return the bound NodeTimersInterface
  inline node_interfaces::NodeTimersInterface::SharedPtr
  get_node_timers_interface() {return timers_;}

  /// Return the bound NodeTopicsInterface
  inline node_interfaces::NodeTopicsInterface::SharedPtr
  get_node_topics_interface() {return topics_;}

  /// Return the bound NodeServicesInterface
  inline node_interfaces::NodeServicesInterface::SharedPtr
  get_node_services_interface() {return services_;}

  /// Return the bound NodeWaitablesInterface
  inline node_interfaces::NodeWaitablesInterface::SharedPtr
  get_node_waitables_interface() {return waitables_;}

  /// Return the bound NodeParametersInterface
  inline node_interfaces::NodeParametersInterface::SharedPtr
  get_node_parameters_interface() {return parameters_;}

  /// Return the bound NodeTimeSourceInterface
  inline node_interfaces::NodeTimeSourceInterface::SharedPtr
  get_node_time_source_interface() {return time_source_;}


  // Setters

  /// Set the bound NodeBaseInterface
  inline void
  set_node_base_interface(node_interfaces::NodeBaseInterface::SharedPtr interface)
  {
    base_ = interface;
  }

  /// Set the bound NodeClockInterface
  inline void
  set_node_clock_interface(node_interfaces::NodeClockInterface::SharedPtr interface)
  {
    clock_ = interface;
  }

  /// Set the bound NodeGraphInterface
  inline void
  set_node_graph_interface(node_interfaces::NodeGraphInterface::SharedPtr interface)
  {
    graph_ = interface;
  }

  /// Set the bound NodeLoggingInterface
  inline void
  set_node_logging_interface(node_interfaces::NodeLoggingInterface::SharedPtr interface)
  {
    logging_ = interface;
  }

  /// Set the bound NodeTimersInterface
  inline void
  set_node_timers_interface(node_interfaces::NodeTimersInterface::SharedPtr interface)
  {
    timers_ = interface;
  }

  /// Set the bound NodeTopicsInterface
  inline void
  set_node_topics_interface(node_interfaces::NodeTopicsInterface::SharedPtr interface)
  {
    topics_ = interface;
  }

  /// Set the bound NodeServicesInterface
  inline void
  set_node_services_interface(node_interfaces::NodeServicesInterface::SharedPtr interface)
  {
    services_ = interface;
  }

  /// Set the bound NodeWaitablesInterface
  inline void
  set_node_waitables_interface(node_interfaces::NodeWaitablesInterface::SharedPtr interface)
  {
    waitables_ = interface;
  }

  /// Set the bound NodeParametersInterface
  inline void
  set_node_parameters_interface(node_interfaces::NodeParametersInterface::SharedPtr interface)
  {
    parameters_ = interface;
  }

  /// Set the bound NodeTimeSourceInterface
  inline void
  set_node_time_source_interface(node_interfaces::NodeTimeSourceInterface::SharedPtr interface)
  {
    time_source_ = interface;
  }

private:
  node_interfaces::NodeBaseInterface::SharedPtr base_;
  node_interfaces::NodeClockInterface::SharedPtr clock_;
  node_interfaces::NodeGraphInterface::SharedPtr graph_;
  node_interfaces::NodeLoggingInterface::SharedPtr logging_;
  node_interfaces::NodeTimersInterface::SharedPtr timers_;
  node_interfaces::NodeTopicsInterface::SharedPtr topics_;
  node_interfaces::NodeServicesInterface::SharedPtr services_;
  node_interfaces::NodeWaitablesInterface::SharedPtr waitables_;
  node_interfaces::NodeParametersInterface::SharedPtr parameters_;
  node_interfaces::NodeTimeSourceInterface::SharedPtr time_source_;
};


/// Create a new node handle bound with no node interfaces.
RCLCPP_PUBLIC
inline NodeInterfaceHandle<>::SharedPtr get_node_handle()
{
  return std::make_shared<NodeInterfaceHandle<>>();
}


/// Create a new node handle bound with the passed in node-like object's interfaces.
/**
 * This overload binds all interfaces, if you want to bind a subset of interfaces, specify them
 * in the template parameters.
 *
 * \param[in] node Node-like object to bind the interfaces of.
 */
template<typename NodeT>
typename NodeInterfaceHandle<>::SharedPtr
get_node_handle(const NodeT & node)
{
  return std::make_shared<NodeInterfaceHandle<>>(node);
}


/// Create a new node handle bound with the passed in node-like object's interfaces.
/**
 * You may specify in the template parameters the interfaces to bind. Any unbound interfaces
 * will be nullptr.
 *
 * For example: ```NodeInterfaceHandle<BaseInterface>(node)``` will bind just the NodeBaseInterface.
 *
 * \param[in] node Node-like object to bind the interfaces of.
 */
template<typename ... InterfaceTs, typename NodeT>
typename NodeInterfaceHandle<InterfaceTs...>::SharedPtr
get_node_handle(const NodeT & node)
{
  if constexpr (0 == sizeof...(InterfaceTs)) {
    return get_node_handle(node);
  } else {
    return std::make_shared<NodeInterfaceHandle<InterfaceTs...>>(node);
  }
}

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_INTERFACE_HANDLE_HPP_
