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
#include <thread>
#include <vector>

#include "rcl/guard_condition.h"
#include "rcl/wait.h"
#include "rclcpp/context.hpp"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rcpputils/mutex.hpp"

namespace rclcpp
{

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
class GraphListener : public std::enable_shared_from_this<GraphListener>
{
public:
  RCLCPP_PUBLIC
  explicit GraphListener(const rclcpp::Context::SharedPtr & parent_context);

  RCLCPP_PUBLIC
  virtual ~GraphListener();

  /// Start the graph listener's listen thread if it hasn't been started.
  /**
   * This function is thread-safe.
   *
   * \throws GraphListenerShutdownError if the GraphListener is shutdown
   * \throws std::runtime if the parent context was destroyed
   * \throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   */
  RCLCPP_PUBLIC
  virtual
  void
  start_if_not_started();

  /// Add a node to the graph listener's list of nodes.
  /**
   * \throws GraphListenerShutdownError if the GraphListener is shutdown
   * \throws NodeAlreadyAddedError if the given node is already in the list
   * \throws std::invalid_argument if node is nullptr
   * \throws std::system_error anything std::mutex::lock() throws
   */
  RCLCPP_PUBLIC
  virtual
  void
  add_node(rclcpp::node_interfaces::NodeGraphInterface * node_graph);

  /// Return true if the given node is in the graph listener's list of nodes.
  /**
   * Also return false if given nullptr.
   *
   * \throws std::system_error anything std::mutex::lock() throws
   */
  RCLCPP_PUBLIC
  virtual
  bool
  has_node(rclcpp::node_interfaces::NodeGraphInterface * node_graph);

  /// Remove a node from the graph listener's list of nodes.
  /**
   *
   * \throws NodeNotFoundError if the given node is not in the list
   * \throws std::invalid_argument if node is nullptr
   * \throws std::system_error anything std::mutex::lock() throws
   */
  RCLCPP_PUBLIC
  virtual
  void
  remove_node(rclcpp::node_interfaces::NodeGraphInterface * node_graph);

  /// Stop the listening thread.
  /**
   * The thread cannot be restarted, and the class is defunct after calling.
   * This function is called by the ~GraphListener() and does nothing if
   * shutdown() was already called.
   * This function exists separately from the ~GraphListener() so that it can
   * be called before and exceptions can be caught.
   *
   * If start_if_not_started() was never called, this function still succeeds,
   * but start_if_not_started() still cannot be called after this function.
   *
   * Note that if you override this method, but leave shutdown to be called in
   * the destruction of this base class, it will not call the overridden
   * version from your base class.
   * So you need to ensure you call your class's shutdown() in its destructor.
   *
   * \throws rclcpp::execptions::RCLError from rcl_guard_condition_fini()
   * \throws rclcpp::execptions::RCLError from rcl_wait_set_fini()
   * \throws std::system_error anything std::mutex::lock() throws
   */
  RCLCPP_PUBLIC
  virtual
  void
  shutdown();

  /// Nothrow version of shutdown(), logs to RCLCPP_ERROR instead.
  RCLCPP_PUBLIC
  virtual
  void
  shutdown(const std::nothrow_t &) noexcept;

  /// Return true if shutdown() has been called, else false.
  RCLCPP_PUBLIC
  virtual
  bool
  is_shutdown();

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

  RCLCPP_PUBLIC
  void
  init_wait_set();

  RCLCPP_PUBLIC
  void
  cleanup_wait_set();

private:
  RCLCPP_DISABLE_COPY(GraphListener)

  /** \internal */
  void
  __shutdown();

  std::weak_ptr<rclcpp::Context> weak_parent_context_;
  std::shared_ptr<rcl_context_t> rcl_parent_context_;

  std::thread listener_thread_;
  bool is_started_;
  std::atomic_bool is_shutdown_;
  mutable rcpputils::PIMutex shutdown_mutex_;

  mutable rcpputils::PIMutex node_graph_interfaces_barrier_mutex_;
  mutable rcpputils::PIMutex node_graph_interfaces_mutex_;
  std::vector<rclcpp::node_interfaces::NodeGraphInterface *> node_graph_interfaces_;

  rclcpp::GuardCondition interrupt_guard_condition_;
  rcl_wait_set_t wait_set_ = rcl_get_zero_initialized_wait_set();
};

}  // namespace graph_listener
}  // namespace rclcpp

#endif  // RCLCPP__GRAPH_LISTENER_HPP_
