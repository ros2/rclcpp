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

#ifndef RCLCPP__NODE_HANDLE_HPP_
#define RCLCPP__NODE_HANDLE_HPP_

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

using BaseInterface = node_interfaces::NodeBaseInterface::SharedPtr;
using ClockInterface = node_interfaces::NodeClockInterface::SharedPtr;
using GraphInterface = node_interfaces::NodeGraphInterface::SharedPtr;
using LoggingInterface = node_interfaces::NodeLoggingInterface::SharedPtr;
using TimersInterface = node_interfaces::NodeTimersInterface::SharedPtr;
using TopicsInterface = node_interfaces::NodeTopicsInterface::SharedPtr;
using ServicesInterface = node_interfaces::NodeServicesInterface::SharedPtr;
using WaitablesInterface = node_interfaces::NodeWaitablesInterface::SharedPtr;
using ParametersInterface = node_interfaces::NodeParametersInterface::SharedPtr;
using TimeSourceInterface = node_interfaces::NodeTimeSourceInterface::SharedPtr;
struct AllInterfaces {};

/// A helper class for aggregating node interfaces
template<typename ... InterfaceTs>
class NodeHandle : public std::enable_shared_from_this<NodeHandle<InterfaceTs...>>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(NodeHandle)

  /// Create a new node handle with no bound node interfaces.
  RCLCPP_PUBLIC
  NodeHandle()
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
   *   - ```NodeHandle<>(node)``` will bind no interfaces.
   *   - ```NodeHandle<BaseInterface>(node)``` will bind just the NodeBaseInterface.
   *   - ```NodeHandle<AllInterfaces>(node)``` will bind all interfaces.
   * \param[in] node Node-like object to bind the interfaces of.
   */
  template<typename NodeT>
  explicit NodeHandle(const NodeT & node)
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

      // NOTE(methylDragon): Extra parentheses to force parameter pack expansion
    } else if constexpr ((std::is_same_v<AllInterfaces, InterfaceTs>|| ...)) {
      base_ = node->get_node_base_interface();
      clock_ = node->get_node_clock_interface();
      graph_ = node->get_node_graph_interface();
      logging_ = node->get_node_logging_interface();
      timers_ = node->get_node_timers_interface();
      topics_ = node->get_node_topics_interface();
      services_ = node->get_node_services_interface();
      waitables_ = node->get_node_waitables_interface();
      parameters_ = node->get_node_parameters_interface();
      time_source_ = node->get_node_time_source_interface();
    } else {
      base_ = (std::is_same_v<BaseInterface, InterfaceTs>|| ...) ?
        node->get_node_base_interface() : nullptr;
      clock_ = (std::is_same_v<ClockInterface, InterfaceTs>|| ...) ?
        node->get_node_clock_interface() : nullptr;
      graph_ = (std::is_same_v<GraphInterface, InterfaceTs>|| ...) ?
        node->get_node_graph_interface() : nullptr;
      logging_ = (std::is_same_v<LoggingInterface, InterfaceTs>|| ...) ?
        node->get_node_logging_interface() : nullptr;
      timers_ = (std::is_same_v<TimersInterface, InterfaceTs>|| ...) ?
        node->get_node_timers_interface() : nullptr;
      topics_ = (std::is_same_v<TopicsInterface, InterfaceTs>|| ...) ?
        node->get_node_topics_interface() : nullptr;
      services_ = (std::is_same_v<ServicesInterface, InterfaceTs>|| ...) ?
        node->get_node_services_interface() : nullptr;
      waitables_ = (std::is_same_v<WaitablesInterface, InterfaceTs>|| ...) ?
        node->get_node_waitables_interface() : nullptr;
      parameters_ = (std::is_same_v<ParametersInterface, InterfaceTs>|| ...) ?
        node->get_node_parameters_interface() : nullptr;
      time_source_ = (std::is_same_v<TimeSourceInterface, InterfaceTs>|| ...) ?
        node->get_node_time_source_interface() : nullptr;
    }
  }

  // Getters and setters

  /// Return the bound NodeBaseInterface
  inline BaseInterface base() {return base_;}

  /// Set the bound NodeBaseInterface
  inline void base(BaseInterface interface) {base_ = interface;}

  /// Return the bound NodeClockInterface
  inline ClockInterface clock() {return clock_;}

  /// Set the bound NodeClockInterface
  inline void clock(ClockInterface interface) {clock_ = interface;}

  /// Return the bound NodeGraphInterface
  inline GraphInterface graph() {return graph_;}

  /// Set the bound NodeGraphInterface
  inline void graph(GraphInterface interface) {graph_ = interface;}

  /// Return the bound NodeLoggingInterface
  inline LoggingInterface logging() {return logging_;}

  /// Set the bound NodeLoggingInterface
  inline void logging(LoggingInterface interface) {logging_ = interface;}

  /// Return the bound NodeTimersInterface
  inline TimersInterface timers() {return timers_;}

  /// Set the bound NodeTimersInterface
  inline void timers(TimersInterface interface) {timers_ = interface;}

  /// Return the bound NodeTopicsInterface
  inline TopicsInterface topics() {return topics_;}

  /// Set the bound NodeTopicsInterface
  inline void topics(TopicsInterface interface) {topics_ = interface;}

  /// Return the bound NodeServicesInterface
  inline ServicesInterface services() {return services_;}

  /// Set the bound NodeServicesInterface
  inline void services(ServicesInterface interface) {services_ = interface;}

  /// Return the bound NodeWaitablesInterface
  inline WaitablesInterface waitables() {return waitables_;}

  /// Set the bound NodeWaitablesInterface
  inline void waitables(WaitablesInterface interface) {waitables_ = interface;}

  /// Return the bound NodeParametersInterface
  inline ParametersInterface parameters() {return parameters_;}

  /// Set the bound NodeParametersInterface
  inline void parameters(ParametersInterface interface) {parameters_ = interface;}

  /// Return the bound NodeTimeSourceInterface
  inline TimeSourceInterface time_source() {return time_source_;}

  /// Set the bound NodeTimeSourceInterface
  inline void time_source(TimeSourceInterface interface) {time_source_ = interface;}


  // Alterate getters
  inline BaseInterface get_node_base_interface() {return base();}
  inline ClockInterface get_node_clock_interface() {return clock();}
  inline GraphInterface get_node_graph_interface() {return graph();}
  inline LoggingInterface get_node_logging_interface() {return logging();}
  inline TimersInterface get_node_timers_interface() {return timers();}
  inline TopicsInterface get_node_topics_interface() {return topics();}
  inline ServicesInterface get_node_services_interface() {return services();}
  inline WaitablesInterface get_node_waitables_interface() {return waitables();}
  inline ParametersInterface get_node_parameters_interface() {return parameters();}
  inline TimeSourceInterface get_node_time_source_interface() {return time_source();}


  // Alternate setters
  inline void set_node_base_interface(BaseInterface interface) {base(interface);}
  inline void set_node_clock_interface(ClockInterface interface) {clock(interface);}
  inline void set_node_graph_interface(GraphInterface interface) {graph(interface);}
  inline void set_node_logging_interface(LoggingInterface interface) {logging(interface);}
  inline void set_node_timers_interface(TimersInterface interface) {timers(interface);}
  inline void set_node_topics_interface(TopicsInterface interface) {topics(interface);}
  inline void set_node_services_interface(ServicesInterface interface) {services(interface);}
  inline void set_node_waitables_interface(WaitablesInterface interface) {waitables(interface);}
  inline void set_node_parameters_interface(ParametersInterface interface) {parameters(interface);}
  inline void set_node_time_source_interface(TimeSourceInterface interface)
  {
    time_source(interface);
  }

private:
  BaseInterface base_;
  ClockInterface clock_;
  GraphInterface graph_;
  LoggingInterface logging_;
  TimersInterface timers_;
  TopicsInterface topics_;
  ServicesInterface services_;
  WaitablesInterface waitables_;
  ParametersInterface parameters_;
  TimeSourceInterface time_source_;
};


/// Create a new node handle bound with no node interfaces.
RCLCPP_PUBLIC
inline NodeHandle<>::SharedPtr get_node_handle() {return std::make_shared<NodeHandle<>>();}


/// Create a new node handle bound with the passed in node-like object's interfaces.
/**
 * This overload binds all interfaces, if you want to bind a subset of interfaces, specify them
 * in the template parameters.
 *
 * \param[in] node Node-like object to bind the interfaces of.
 */
template<typename NodeT>
typename NodeHandle<>::SharedPtr
get_node_handle(const NodeT & node)
{
  return std::make_shared<NodeHandle<>>(node);
}


/// Create a new node handle bound with the passed in node-like object's interfaces.
/**
 * You may specify in the template parameters the interfaces to bind. Any unbound interfaces
 * will be nullptr.
 *
 * For example: ```NodeHandle<BaseInterface>(node)``` will bind just the NodeBaseInterface.
 *
 * \param[in] node Node-like object to bind the interfaces of.
 */
template<typename ... InterfaceTs, typename NodeT>
typename NodeHandle<InterfaceTs...>::SharedPtr
get_node_handle(const NodeT & node)
{
  if constexpr (0 == sizeof...(InterfaceTs)) {
    return get_node_handle(node);
  } else {
    return std::make_shared<NodeHandle<InterfaceTs...>>(node);
  }
}

}  // namespace rclcpp

#endif  // RCLCPP__NODE_HANDLE_HPP_
