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

#ifndef RCLCPP__NODE_INTERFACES__NODE_INTERFACES_HELPERS_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_INTERFACES_HELPERS_HPP_

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
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace node_interfaces
{

/**
 * Helper classes to be inherited by NodeInterfaces to support node interface aggregation
 * via multiple inheritance.
 *
 * \sa rclcpp::node_interfaces::NodeInterfaces
 */

// These also provide a more terse way to configure the supported interfaces!

// NOTE: These helpers deliberately do not feature methods to change the internally stored node
//       interface, since it would make thread-safe code more difficult.

/// NodeInterfaces support for NodeBaseInterface
class Base
{
public:
  /// Default constructor with no bound NodeBaseInterface
  RCLCPP_PUBLIC
  Base() {impl_ = nullptr;}

  /// Bind the passed in node-like object's NodeBaseInterface
  template<class NodeT>
  explicit Base(NodeT & node) {impl_ = node.get_node_base_interface();}

  /// Bind the passed in node-like shared_ptr's NodeBaseInterface
  template<class NodeT>
  explicit Base(std::shared_ptr<NodeT> node) {impl_ = node->get_node_base_interface();}

  /// Bind a passed in NodeBaseInterface::SharedPtr
  RCLCPP_PUBLIC
  explicit Base(NodeBaseInterface::SharedPtr interface) {impl_ = interface;}

  /// Return the bound NodeBaseInterface
  inline NodeBaseInterface::SharedPtr get_node_base_interface() {return impl_;}

private:
  NodeBaseInterface::SharedPtr impl_;
};


/// NodeInterfaces support for NodeClockInterface
class Clock
{
public:
  /// Default constructor with no bound NodeClockInterface
  RCLCPP_PUBLIC
  Clock() {impl_ = nullptr;}

  /// Bind the passed in node-like object's NodeClockInterface
  template<class NodeT>
  explicit Clock(NodeT & node) {impl_ = node.get_node_clock_interface();}

  /// Bind the passed in node-like shared_ptr's NodeClockInterface
  template<class NodeT>
  explicit Clock(std::shared_ptr<NodeT> node) {impl_ = node->get_node_clock_interface();}

  /// Bind a passed in NodeClockInterface::SharedPtr
  RCLCPP_PUBLIC
  explicit Clock(NodeClockInterface::SharedPtr interface) {impl_ = interface;}

  /// Return the bound NodeClockInterface
  inline NodeClockInterface::SharedPtr get_node_clock_interface() {return impl_;}

private:
  NodeClockInterface::SharedPtr impl_;
};


/// NodeInterfaces support for NodeGraphInterface
class Graph
{
public:
  /// Default constructor with no bound NodeGraphInterface
  RCLCPP_PUBLIC
  Graph() {impl_ = nullptr;}

  /// Bind the passed in node-like object's NodeGraphInterface
  template<class NodeT>
  explicit Graph(NodeT & node) {impl_ = node.get_node_graph_interface();}

  /// Bind the passed in node-like shared_ptr's NodeGraphInterface
  template<class NodeT>
  explicit Graph(std::shared_ptr<NodeT> node) {impl_ = node->get_node_graph_interface();}

  /// Bind a passed in NodeGraphInterface::SharedPtr
  RCLCPP_PUBLIC
  explicit Graph(NodeGraphInterface::SharedPtr interface) {impl_ = interface;}

  /// Return the bound NodeGraphInterface
  inline NodeGraphInterface::SharedPtr get_node_graph_interface() {return impl_;}

private:
  NodeGraphInterface::SharedPtr impl_;
};


/// NodeInterfaces support for NodeLoggingInterface
class Logging
{
public:
  /// Default constructor with no bound NodeLoggingInterface
  RCLCPP_PUBLIC
  Logging() {impl_ = nullptr;}

  /// Bind the passed in node-like object's NodeLoggingInterface
  template<class NodeT>
  explicit Logging(NodeT & node) {impl_ = node.get_node_logging_interface();}

  /// Bind the passed in node-like shared_ptr's NodeLoggingInterface
  template<class NodeT>
  explicit Logging(std::shared_ptr<NodeT> node) {impl_ = node->get_node_logging_interface();}

  /// Bind a passed in NodeLoggingInterface::SharedPtr
  RCLCPP_PUBLIC
  explicit Logging(NodeLoggingInterface::SharedPtr interface) {impl_ = interface;}

  /// Return the bound NodeLoggingInterface
  inline NodeLoggingInterface::SharedPtr get_node_logging_interface() {return impl_;}

private:
  NodeLoggingInterface::SharedPtr impl_;
};


/// NodeInterfaces support for NodeParametersInterface
class Parameters
{
public:
  /// Default constructor with no bound NodeParametersInterface
  RCLCPP_PUBLIC
  Parameters() {impl_ = nullptr;}

  /// Bind the passed in node-like object's NodeParametersInterface
  template<class NodeT>
  explicit Parameters(NodeT & node) {impl_ = node.get_node_parameters_interface();}

  /// Bind the passed in node-like shared_ptr's NodeParametersInterface
  template<class NodeT>
  explicit Parameters(std::shared_ptr<NodeT> node) {impl_ = node->get_node_parameters_interface();}

  /// Bind a passed in NodeParametersInterface::SharedPtr
  RCLCPP_PUBLIC
  explicit Parameters(NodeParametersInterface::SharedPtr interface) {impl_ = interface;}

  /// Return the bound NodeParametersInterface
  inline NodeParametersInterface::SharedPtr get_node_parameters_interface() {return impl_;}

private:
  NodeParametersInterface::SharedPtr impl_;
};


/// NodeInterfaces support for NodeServicesInterface
class Services
{
public:
  /// Default constructor with no bound NodeServicesInterface
  RCLCPP_PUBLIC
  Services() {impl_ = nullptr;}

  /// Bind the passed in node-like object's NodeServicesInterface
  template<class NodeT>
  explicit Services(NodeT & node) {impl_ = node.get_node_services_interface();}

  /// Bind the passed in node-like shared_ptr's NodeServicesInterface
  template<class NodeT>
  explicit Services(std::shared_ptr<NodeT> node) {impl_ = node->get_node_services_interface();}

  /// Bind a passed in NodeServicesInterface::SharedPtr
  RCLCPP_PUBLIC
  explicit Services(NodeServicesInterface::SharedPtr interface) {impl_ = interface;}

  /// Return the bound NodeServicesInterface
  inline NodeServicesInterface::SharedPtr get_node_services_interface() {return impl_;}

private:
  NodeServicesInterface::SharedPtr impl_;
};


/// NodeInterfaces support for NodeTimeSourceInterface
class TimeSource
{
public:
  /// Default constructor with no bound NodeTimeSourceInterface
  RCLCPP_PUBLIC
  TimeSource() {impl_ = nullptr;}

  /// Bind the passed in node-like object's NodeTimeSourceInterface
  template<class NodeT>
  explicit TimeSource(NodeT & node) {impl_ = node.get_node_time_source_interface();}

  /// Bind the passed in node-like shared_ptr's NodeTimeSourceInterface
  template<class NodeT>
  explicit TimeSource(std::shared_ptr<NodeT> node) {impl_ = node->get_node_time_source_interface();}

  /// Bind a passed in NodeTimeSourceInterface::SharedPtr
  RCLCPP_PUBLIC
  explicit TimeSource(NodeTimeSourceInterface::SharedPtr interface) {impl_ = interface;}

  /// Return the bound NodeTimeSourceInterface
  inline NodeTimeSourceInterface::SharedPtr get_node_time_source_interface() {return impl_;}

private:
  NodeTimeSourceInterface::SharedPtr impl_;
};


/// NodeInterfaces support for NodeTimersInterface
class Timers
{
public:
  /// Default constructor with no bound NodeTimersInterface
  RCLCPP_PUBLIC
  Timers() {impl_ = nullptr;}

  /// Bind the passed in node-like object's NodeTimersInterface
  template<class NodeT>
  explicit Timers(NodeT & node) {impl_ = node.get_node_timers_interface();}

  /// Bind the passed in node-like shared_ptr's NodeTimersInterface
  template<class NodeT>
  explicit Timers(std::shared_ptr<NodeT> node) {impl_ = node->get_node_timers_interface();}

  /// Bind a passed in NodeTimersInterface::SharedPtr
  RCLCPP_PUBLIC
  explicit Timers(NodeTimersInterface::SharedPtr interface) {impl_ = interface;}

  /// Return the bound NodeTimersInterface
  inline NodeTimersInterface::SharedPtr get_node_timers_interface() {return impl_;}

private:
  NodeTimersInterface::SharedPtr impl_;
};


/// NodeInterfaces support for NodeTopicsInterface
class Topics
{
public:
  /// Default constructor with no bound NodeTopicsInterface
  RCLCPP_PUBLIC
  Topics() {impl_ = nullptr;}

  /// Bind the passed in node-like object's NodeTopicsInterface
  template<class NodeT>
  explicit Topics(NodeT & node) {impl_ = node.get_node_topics_interface();}

  /// Bind the passed in node-like shared_ptr's NodeTopicsInterface
  template<class NodeT>
  explicit Topics(std::shared_ptr<NodeT> node) {impl_ = node->get_node_topics_interface();}

  /// Bind a passed in NodeTopicsInterface::SharedPtr
  RCLCPP_PUBLIC
  explicit Topics(NodeTopicsInterface::SharedPtr interface) {impl_ = interface;}

  /// Return the bound NodeTopicsInterface
  inline NodeTopicsInterface::SharedPtr get_node_topics_interface() {return impl_;}

private:
  NodeTopicsInterface::SharedPtr impl_;
};


/// NodeInterfaces support for NodeWaitablesInterface
class Waitables
{
public:
  /// Default constructor with no bound NodeWaitablesInterface
  RCLCPP_PUBLIC
  Waitables() {impl_ = nullptr;}

  /// Bind the passed in node-like object's NodeWaitablesInterface
  template<class NodeT>
  explicit Waitables(NodeT & node) {impl_ = node.get_node_waitables_interface();}

  /// Bind the passed in node-like shared_ptr's NodeWaitablesInterface
  template<class NodeT>
  explicit Waitables(std::shared_ptr<NodeT> node) {impl_ = node->get_node_waitables_interface();}

  /// Bind a passed in NodeWaitablesInterface::SharedPtr
  RCLCPP_PUBLIC
  explicit Waitables(NodeWaitablesInterface::SharedPtr interface) {impl_ = interface;}

  /// Return the bound NodeWaitablesInterface
  inline NodeWaitablesInterface::SharedPtr get_node_waitables_interface() {return impl_;}

private:
  NodeWaitablesInterface::SharedPtr impl_;
};


}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_INTERFACES_HELPERS_HPP_
