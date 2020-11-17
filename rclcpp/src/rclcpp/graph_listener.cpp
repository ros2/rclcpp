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

#include "rclcpp/graph_listener.hpp"

#include <cstdio>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include "rcl/error_handling.h"
#include "rcl/types.h"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rmw/impl/cpp/demangle.hpp"

#include "rcutils/logging_macros.h"

using rclcpp::exceptions::throw_from_rcl_error;

namespace rclcpp
{
namespace graph_listener
{

GraphListener::GraphListener(const std::shared_ptr<Context> & parent_context)
: weak_parent_context_(parent_context),
  rcl_parent_context_(parent_context->get_rcl_context()),
  is_started_(false),
  is_shutdown_(false)
{
  // TODO(wjwwood): make a guard condition class in rclcpp so this can be tracked
  //   automatically with the rcl guard condition
  // hold on to this context to prevent it from going out of scope while this
  // guard condition is using it.
  rcl_ret_t ret = rcl_guard_condition_init(
    &interrupt_guard_condition_,
    rcl_parent_context_.get(),
    rcl_guard_condition_get_default_options());
  if (RCL_RET_OK != ret) {
    throw_from_rcl_error(ret, "failed to create interrupt guard condition");
  }
}

GraphListener::~GraphListener()
{
  this->shutdown(std::nothrow);
}

void GraphListener::init_wait_set()
{
  rcl_ret_t ret = rcl_wait_set_init(
    &wait_set_,
    0,  // number_of_subscriptions
    2,  // number_of_guard_conditions
    0,  // number_of_timers
    0,  // number_of_clients
    0,  // number_of_services
    0,  // number_of_events
    rcl_parent_context_.get(),
    rcl_get_default_allocator());
  if (RCL_RET_OK != ret) {
    throw_from_rcl_error(ret, "failed to initialize wait set");
  }
}

void
GraphListener::start_if_not_started()
{
  std::lock_guard<std::mutex> shutdown_lock(shutdown_mutex_);
  if (is_shutdown_.load()) {
    throw GraphListenerShutdownError();
  }
  auto parent_context = weak_parent_context_.lock();
  if (!is_started_ && parent_context) {
    // Register an on_shutdown hook to shtudown the graph listener.
    // This is important to ensure that the wait set is finalized before
    // destruction of static objects occurs.
    std::weak_ptr<GraphListener> weak_this = shared_from_this();
    parent_context->on_shutdown(
      [weak_this]() {
        auto shared_this = weak_this.lock();
        if (shared_this) {
          // should not throw from on_shutdown if it can be avoided
          shared_this->shutdown(std::nothrow);
        }
      });
    // Initialize the wait set before starting.
    init_wait_set();
    // Start the listener thread.
    listener_thread_ = std::thread(&GraphListener::run, this);
    is_started_ = true;
  }
}

void
GraphListener::run()
{
  try {
    run_loop();
  } catch (const std::exception & exc) {
    RCUTILS_LOG_ERROR_NAMED(
      "rclcpp",
      "caught %s exception in GraphListener thread: %s",
      rmw::impl::cpp::demangle(exc).c_str(),
      exc.what());
    std::rethrow_exception(std::current_exception());
  } catch (...) {
    RCUTILS_LOG_ERROR_NAMED(
      "rclcpp",
      "unknown error in GraphListener thread");
    std::rethrow_exception(std::current_exception());
  }
}

void
GraphListener::run_loop()
{
  while (true) {
    // If shutdown() was called, exit.
    if (is_shutdown_.load()) {
      return;
    }
    rcl_ret_t ret;
    {
      // This "barrier" lock ensures that other functions can acquire the
      // node_graph_interfaces_mutex_ after waking up rcl_wait.
      std::lock_guard<std::mutex> nodes_barrier_lock(node_graph_interfaces_barrier_mutex_);
      // This is ownership is passed to nodes_lock in the next line.
      node_graph_interfaces_mutex_.lock();
    }
    // This lock is released when the loop continues or exits.
    std::lock_guard<std::mutex> nodes_lock(node_graph_interfaces_mutex_, std::adopt_lock);
    // Resize the wait set if necessary.
    const size_t node_graph_interfaces_size = node_graph_interfaces_.size();
    // Add 2 for the interrupt and shutdown guard conditions
    if (wait_set_.size_of_guard_conditions < (node_graph_interfaces_size + 2)) {
      ret = rcl_wait_set_resize(&wait_set_, 0, node_graph_interfaces_size + 2, 0, 0, 0, 0);
      if (RCL_RET_OK != ret) {
        throw_from_rcl_error(ret, "failed to resize wait set");
      }
    }
    // Clear the wait set.
    ret = rcl_wait_set_clear(&wait_set_);
    if (RCL_RET_OK != ret) {
      throw_from_rcl_error(ret, "failed to clear wait set");
    }
    // Put the interrupt guard condition in the wait set.
    ret = rcl_wait_set_add_guard_condition(&wait_set_, &interrupt_guard_condition_, NULL);
    if (RCL_RET_OK != ret) {
      throw_from_rcl_error(ret, "failed to add interrupt guard condition to wait set");
    }

    // Put graph guard conditions for each node into the wait set.
    std::vector<size_t> graph_gc_indexes(node_graph_interfaces_size, 0u);
    for (size_t i = 0u; i < node_graph_interfaces_size; ++i) {
      auto node_ptr = node_graph_interfaces_[i];
      // Only wait on graph changes if some user of the node is watching.
      if (node_ptr->count_graph_users() == 0) {
        continue;
      }
      // Add the graph guard condition for the node to the wait set.
      auto graph_gc = node_ptr->get_graph_guard_condition();
      if (!graph_gc) {
        throw_from_rcl_error(RCL_RET_ERROR, "failed to get graph guard condition");
      }
      ret = rcl_wait_set_add_guard_condition(&wait_set_, graph_gc, &graph_gc_indexes[i]);
      if (RCL_RET_OK != ret) {
        throw_from_rcl_error(ret, "failed to add graph guard condition to wait set");
      }
    }

    // Wait for: graph changes, interrupt, or shutdown/SIGINT
    ret = rcl_wait(&wait_set_, -1);  // block for ever until a guard condition is triggered
    if (RCL_RET_TIMEOUT == ret) {
      throw std::runtime_error("rcl_wait unexpectedly timed out");
    }
    if (RCL_RET_OK != ret) {
      throw_from_rcl_error(ret, "failed to wait on wait set");
    }

    // Notify nodes who's guard conditions are set (triggered).
    for (size_t i = 0u; i < node_graph_interfaces_size; ++i) {
      const auto node_ptr = node_graph_interfaces_[i];
      auto graph_gc = node_ptr->get_graph_guard_condition();
      if (!graph_gc) {
        throw_from_rcl_error(RCL_RET_ERROR, "failed to get graph guard condition");
      }
      if (graph_gc == wait_set_.guard_conditions[graph_gc_indexes[i]]) {
        node_ptr->notify_graph_change();
      }
      if (is_shutdown_) {
        // If shutdown, then notify the node of this as well.
        node_ptr->notify_shutdown();
      }
    }
  }  // while (true)
}

static void
interrupt_(rcl_guard_condition_t * interrupt_guard_condition)
{
  rcl_ret_t ret = rcl_trigger_guard_condition(interrupt_guard_condition);
  if (RCL_RET_OK != ret) {
    throw_from_rcl_error(ret, "failed to trigger the interrupt guard condition");
  }
}

static void
acquire_nodes_lock_(
  std::mutex * node_graph_interfaces_barrier_mutex,
  std::mutex * node_graph_interfaces_mutex,
  rcl_guard_condition_t * interrupt_guard_condition)
{
  {
    // Acquire this lock to prevent the run loop from re-locking the
    // nodes_mutext_ after being woken up.
    std::lock_guard<std::mutex> nodes_barrier_lock(*node_graph_interfaces_barrier_mutex);
    // Trigger the interrupt guard condition to wake up rcl_wait.
    interrupt_(interrupt_guard_condition);
    node_graph_interfaces_mutex->lock();
  }
}

static bool
has_node_(
  std::vector<rclcpp::node_interfaces::NodeGraphInterface *> * node_graph_interfaces,
  rclcpp::node_interfaces::NodeGraphInterface * node_graph)
{
  for (const auto node_ptr : (*node_graph_interfaces)) {
    if (node_graph == node_ptr) {
      return true;
    }
  }
  return false;
}

bool
GraphListener::has_node(rclcpp::node_interfaces::NodeGraphInterface * node_graph)
{
  if (!node_graph) {
    return false;
  }
  // Acquire the nodes mutex using the barrier to prevent the run loop from
  // re-locking the nodes mutex after being interrupted.
  acquire_nodes_lock_(
    &node_graph_interfaces_barrier_mutex_,
    &node_graph_interfaces_mutex_,
    &interrupt_guard_condition_);
  // Store the now acquired node_graph_interfaces_mutex_ in the scoped lock using adopt_lock.
  std::lock_guard<std::mutex> nodes_lock(node_graph_interfaces_mutex_, std::adopt_lock);
  return has_node_(&node_graph_interfaces_, node_graph);
}

void
GraphListener::add_node(rclcpp::node_interfaces::NodeGraphInterface * node_graph)
{
  if (!node_graph) {
    throw std::invalid_argument("node is nullptr");
  }
  std::lock_guard<std::mutex> shutdown_lock(shutdown_mutex_);
  if (is_shutdown_.load()) {
    throw GraphListenerShutdownError();
  }

  // Acquire the nodes mutex using the barrier to prevent the run loop from
  // re-locking the nodes mutex after being interrupted.
  acquire_nodes_lock_(
    &node_graph_interfaces_barrier_mutex_,
    &node_graph_interfaces_mutex_,
    &interrupt_guard_condition_);
  // Store the now acquired node_graph_interfaces_mutex_ in the scoped lock using adopt_lock.
  std::lock_guard<std::mutex> nodes_lock(node_graph_interfaces_mutex_, std::adopt_lock);
  if (has_node_(&node_graph_interfaces_, node_graph)) {
    throw NodeAlreadyAddedError();
  }
  node_graph_interfaces_.push_back(node_graph);
  // The run loop has already been interrupted by acquire_nodes_lock_() and
  // will evaluate the new node when nodes_lock releases the node_graph_interfaces_mutex_.
}

static void
remove_node_(
  std::vector<rclcpp::node_interfaces::NodeGraphInterface *> * node_graph_interfaces,
  rclcpp::node_interfaces::NodeGraphInterface * node_graph)
{
  // Remove the node if it is found.
  for (auto it = node_graph_interfaces->begin(); it != node_graph_interfaces->end(); ++it) {
    if (node_graph == *it) {
      // Found the node, remove it.
      node_graph_interfaces->erase(it);
      // Now trigger the interrupt guard condition to make sure
      return;
    }
  }
  // Not found in the loop.
  throw NodeNotFoundError();
}

void
GraphListener::remove_node(rclcpp::node_interfaces::NodeGraphInterface * node_graph)
{
  if (!node_graph) {
    throw std::invalid_argument("node is nullptr");
  }
  std::lock_guard<std::mutex> shutdown_lock(shutdown_mutex_);
  if (is_shutdown()) {
    // If shutdown, then the run loop has been joined, so we can remove them directly.
    return remove_node_(&node_graph_interfaces_, node_graph);
  }
  // Otherwise, first interrupt and lock against the run loop to safely remove the node.
  // Acquire the nodes mutex using the barrier to prevent the run loop from
  // re-locking the nodes mutex after being interrupted.
  acquire_nodes_lock_(
    &node_graph_interfaces_barrier_mutex_,
    &node_graph_interfaces_mutex_,
    &interrupt_guard_condition_);
  // Store the now acquired node_graph_interfaces_mutex_ in the scoped lock using adopt_lock.
  std::lock_guard<std::mutex> nodes_lock(node_graph_interfaces_mutex_, std::adopt_lock);
  remove_node_(&node_graph_interfaces_, node_graph);
}

void
GraphListener::cleanup_wait_set()
{
  rcl_ret_t ret = rcl_wait_set_fini(&wait_set_);
  if (RCL_RET_OK != ret) {
    throw_from_rcl_error(ret, "failed to finalize wait set");
  }
}

void
GraphListener::__shutdown()
{
  std::lock_guard<std::mutex> shutdown_lock(shutdown_mutex_);
  if (!is_shutdown_.exchange(true)) {
    if (is_started_) {
      interrupt_(&interrupt_guard_condition_);
      listener_thread_.join();
    }
    rcl_ret_t ret = rcl_guard_condition_fini(&interrupt_guard_condition_);
    if (RCL_RET_OK != ret) {
      throw_from_rcl_error(ret, "failed to finalize interrupt guard condition");
    }
    if (is_started_) {
      cleanup_wait_set();
    }
  }
}

void
GraphListener::shutdown()
{
  this->__shutdown();
}

void
GraphListener::shutdown(const std::nothrow_t &) noexcept
{
  try {
    this->__shutdown();
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(
      rclcpp::get_logger("rclcpp"),
      "caught %s exception when shutting down GraphListener: %s",
      rmw::impl::cpp::demangle(exc).c_str(), exc.what());
  } catch (...) {
    RCLCPP_ERROR(
      rclcpp::get_logger("rclcpp"),
      "caught unknown exception when shutting down GraphListener");
  }
}

bool
GraphListener::is_shutdown()
{
  return is_shutdown_.load();
}

}  // namespace graph_listener
}  // namespace rclcpp
