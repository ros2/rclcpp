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

#ifndef RCLCPP__NODE_INTERFACES__NODE_INTERFACE_HANDLE_HELPERS_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_INTERFACE_HANDLE_HELPERS_HPP_

#include <memory>

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

// Helper classes to be inherited by NodeInterfaceHandle to support node interface aggregation
// via multiple inheritance.

// These also provide a more terse way to configure the supported interfaces!


/// NodeInterfaceHandle support for NodeBaseInterface
class Base
{
public:
  /// Default constructor with no bound NodeBaseInterface
  Base() {impl_ = nullptr;}

  /// Bind the passed in node-like object's NodeBaseInterface
  template<class NodeT>
  explicit Base(const NodeT & node) {impl_ = node->get_node_base_interface();}

  /// Return the bound NodeBaseInterface
  inline NodeBaseInterface::SharedPtr get_node_base_interface() {return impl_;}

  /// Set the bound NodeBaseInterface
  inline void set_node_base_interface(NodeBaseInterface::SharedPtr interface) {impl_ = interface;}

private:
  NodeBaseInterface::SharedPtr impl_;
};


/// NodeInterfaceHandle support for NodeClockInterface
class Clock
{
public:
  /// Default constructor with no bound NodeClockInterface
  Clock() {impl_ = nullptr;}

  /// Bind the passed in node-like object's NodeClockInterface
  template<class NodeT>
  explicit Clock(const NodeT & node) {impl_ = node->get_node_clock_interface();}

  /// Return the bound NodeClockInterface
  inline NodeClockInterface::SharedPtr get_node_clock_interface() {return impl_;}

  /// Set the bound NodeClockInterface
  inline void set_node_clock_interface(NodeClockInterface::SharedPtr interface) {impl_ = interface;}

private:
  NodeClockInterface::SharedPtr impl_;
};


/// NodeInterfaceHandle support for NodeGraphInterface
class Graph
{
public:
  /// Default constructor with no bound NodeGraphInterface
  Graph() {impl_ = nullptr;}

  /// Bind the passed in node-like object's NodeGraphInterface
  template<class NodeT>
  explicit Graph(const NodeT & node) {impl_ = node->get_node_graph_interface();}

  /// Return the bound NodeGraphInterface
  inline NodeGraphInterface::SharedPtr get_node_graph_interface() {return impl_;}

  /// Set the bound NodeGraphInterface
  inline void set_node_graph_interface(NodeGraphInterface::SharedPtr interface) {impl_ = interface;}

private:
  NodeGraphInterface::SharedPtr impl_;
};


/// NodeInterfaceHandle support for NodeLoggingInterface
class Logging
{
public:
  /// Default constructor with no bound NodeLoggingInterface
  Logging() {impl_ = nullptr;}

  /// Bind the passed in node-like object's NodeLoggingInterface
  template<class NodeT>
  explicit Logging(const NodeT & node) {impl_ = node->get_node_logging_interface();}

  /// Return the bound NodeLoggingInterface
  inline NodeLoggingInterface::SharedPtr get_node_logging_interface() {return impl_;}

  /// Set the bound NodeLoggingInterface
  inline void
  set_node_logging_interface(NodeLoggingInterface::SharedPtr interface) {impl_ = interface;}

private:
  NodeLoggingInterface::SharedPtr impl_;
};


/// NodeInterfaceHandle support for NodeParametersInterface
class Parameters
{
public:
  /// Default constructor with no bound NodeParametersInterface
  Parameters() {impl_ = nullptr;}

  /// Bind the passed in node-like object's NodeParametersInterface
  template<class NodeT>
  explicit Parameters(const NodeT & node) {impl_ = node->get_node_parameters_interface();}

  /// Return the bound NodeParametersInterface
  inline NodeParametersInterface::SharedPtr get_node_parameters_interface() {return impl_;}

  /// Set the bound NodeParametersInterface
  inline void
  set_node_parameters_interface(NodeParametersInterface::SharedPtr interface) {impl_ = interface;}

private:
  NodeParametersInterface::SharedPtr impl_;
};


/// NodeInterfaceHandle support for NodeServicesInterface
class Services
{
public:
  /// Default constructor with no bound NodeServicesInterface
  Services() {impl_ = nullptr;}

  /// Bind the passed in node-like object's NodeServicesInterface
  template<class NodeT>
  explicit Services(const NodeT & node) {impl_ = node->get_node_services_interface();}

  /// Return the bound NodeServicesInterface
  inline NodeServicesInterface::SharedPtr get_node_services_interface() {return impl_;}

  /// Set the bound NodeServicesInterface
  inline void
  set_node_services_interface(NodeServicesInterface::SharedPtr interface) {impl_ = interface;}

private:
  NodeServicesInterface::SharedPtr impl_;
};


/// NodeInterfaceHandle support for NodeTimeSourceInterface
class TimeSource
{
public:
  /// Default constructor with no bound NodeTimeSourceInterface
  TimeSource() {impl_ = nullptr;}

  /// Bind the passed in node-like object's NodeTimeSourceInterface
  template<class NodeT>
  explicit TimeSource(const NodeT & node) {impl_ = node->get_node_time_source_interface();}

  /// Return the bound NodeTimeSourceInterface
  inline NodeTimeSourceInterface::SharedPtr get_node_time_source_interface() {return impl_;}

  /// Set the bound NodeTimeSourceInterface
  inline void
  set_node_time_source_interface(NodeTimeSourceInterface::SharedPtr interface) {impl_ = interface;}

private:
  NodeTimeSourceInterface::SharedPtr impl_;
};


/// NodeInterfaceHandle support for NodeTimersInterface
class Timers
{
public:
  /// Default constructor with no bound NodeTimersInterface
  Timers() {impl_ = nullptr;}

  /// Bind the passed in node-like object's NodeTimersInterface
  template<class NodeT>
  explicit Timers(const NodeT & node) {impl_ = node->get_node_timers_interface();}

  /// Return the bound NodeTimersInterface
  inline NodeTimersInterface::SharedPtr get_node_timers_interface() {return impl_;}

  /// Set the bound NodeTimersInterface
  inline void
  set_node_timers_interface(NodeTimersInterface::SharedPtr interface) {impl_ = interface;}

private:
  NodeTimersInterface::SharedPtr impl_;
};


/// NodeInterfaceHandle support for NodeTopicsInterface
class Topics
{
public:
  /// Default constructor with no bound NodeTopicsInterface
  Topics() {impl_ = nullptr;}

  /// Bind the passed in node-like object's NodeTopicsInterface
  template<class NodeT>
  explicit Topics(const NodeT & node) {impl_ = node->get_node_topics_interface();}

  /// Return the bound NodeTopicsInterface
  inline NodeTopicsInterface::SharedPtr get_node_topics_interface() {return impl_;}

  /// Set the bound NodeTopicsInterface
  inline void
  set_node_topics_interface(NodeTopicsInterface::SharedPtr interface) {impl_ = interface;}

private:
  NodeTopicsInterface::SharedPtr impl_;
};


/// NodeInterfaceHandle support for NodeWaitablesInterface
class Waitables
{
public:
  /// Default constructor with no bound NodeWaitablesInterface
  Waitables() {impl_ = nullptr;}

  /// Bind the passed in node-like object's NodeWaitablesInterface
  template<class NodeT>
  explicit Waitables(const NodeT & node) {impl_ = node->get_node_waitables_interface();}

  /// Return the bound NodeWaitablesInterface
  inline NodeWaitablesInterface::SharedPtr get_node_waitables_interface() {return impl_;}

  /// Set the bound NodeWaitablesInterface
  inline void
  set_node_waitables_interface(NodeWaitablesInterface::SharedPtr interface) {impl_ = interface;}

private:
  NodeWaitablesInterface::SharedPtr impl_;
};


}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_INTERFACE_HANDLE_HELPERS_HPP_
