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

#ifndef RCLCPP__GRAPH_LISTENER_HPP_
#define RCLCPP__GRAPH_LISTENER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "rcl/guard_condition.h"
#include "rcl/wait.h"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

namespace node
{
class Node;
}  // namespace node

namespace graph_listener
{

/// Thrown when a function is called on a GraphListener that is already shutdown.
class GraphListenerShutdownError : public std::runtime_error
{
public:
  GraphListenerShutdownError()
  : std::runtime_error("GraphListener already shutdown") {}
};

/// Thrown when a node has already been added to the GraphListener.
class NodeAlreadyAddedError : public std::runtime_error
{
public:
  NodeAlreadyAddedError()
  : std::runtime_error("node already added") {}
};

/// Thrown when the given node is not in the GraphListener.
class NodeNotFoundError : public std::runtime_error
{
public:
  NodeNotFoundError()
  : std::runtime_error("node not found") {}
};

/// Notifies many nodes of graph changes by listening in a thread.
class GraphListener
{
public:
  RCLCPP_PUBLIC
  GraphListener();

  RCLCPP_PUBLIC
  virtual ~GraphListener();

  /// Start the graph listener's listen thread if it hasn't been started.
  /* This function is thread-safe.
   *
   * \throws GraphListenerShutdownError if the GraphListener is shutdown
   */
  RCLCPP_PUBLIC
  virtual
  void
  start_if_not_started();

  /// Add a node to the graph listener's list of nodes.
  /*
   * \throws GraphListenerShutdownError if the GraphListener is shutdown
   * \throws NodeAlreadyAddedError if the given node is already in the list
   * \throws std::invalid_argument if node is nullptr
   * \throws std::system_error anything std::mutex::lock() throws
   */
  RCLCPP_PUBLIC
  virtual
  void
  add_node(rclcpp::node::Node * node);

  /// Return true if the given node is in the graph listener's list of nodes.
  /* Also return false if given nullptr.
   *
   * \throws std::system_error anything std::mutex::lock() throws
   */
  RCLCPP_PUBLIC
  virtual
  bool
  has_node(rclcpp::node::Node * node) const;

  /// Remove a node from the graph listener's list of nodes.
  /*
   * \throws NodeNotFoundError if the given node is not in the list
   * \throws std::invalid_argument if node is nullptr
   * \throws std::system_error anything std::mutex::lock() throws
   */
  RCLCPP_PUBLIC
  virtual
  void
  remove_node(rclcpp::node::Node * node);

  /// Interrupt the listening thread so it can consider changes or shutdown.
  /*
   * \throws GraphListenerShutdownError if the GraphListener is shutdown
   * \throws rclcpp::execptions::RCLError from rcl_trigger_guard_condition()
   * \throws std::system_error anything std::mutex::lock() throws
   */
  RCLCPP_PUBLIC
  virtual
  void
  interrupt();

  /// Stop the listening thread.
  /* The thread cannot be restarted, and the class is defunct after calling.
   * This function is called by the ~GraphListener() and does nothing if
   * shutdown() was already called.
   * This function exists separately from the ~GraphListener() so that it can
   * be called before and exceptions can be caught.
   *
   * If start_if_not_started() was never called, this function still succeeds,
   * but start_if_not_started() still cannot be called after this function.
   *
   * \throws rclcpp::execptions::RCLError from rcl_guard_condition_fini()
   * \throws rclcpp::execptions::RCLError from rcl_wait_set_fini()
   * \throws std::system_error anything std::mutex::lock() throws
   */
  RCLCPP_PUBLIC
  virtual
  void
  shutdown();

protected:
  /// Main function for the listening thread.
  RCLCPP_PUBLIC
  virtual
  void
  run();

  RCLCPP_PUBLIC
  virtual
  void
  run_loop();

private:
  std::thread listener_thread_;
  bool is_started_;
  std::atomic_bool is_shutdown_;
  mutable std::mutex shutdown_mutex_;

  mutable std::mutex nodes_mutex_;
  std::vector<rclcpp::node::Node *> nodes_;

  rcl_guard_condition_t interrupt_guard_condition_ = rcl_get_zero_initialized_guard_condition();
  rcl_wait_set_t wait_set_ = rcl_get_zero_initialized_wait_set();
};

}  // namespace graph_listener
}  // namespace rclcpp

#endif  // RCLCPP__GRAPH_LISTENER_HPP_
