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
#include <string>
#include <vector>

#include "rcl/error_handling.h"
#include "rcl/types.h"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/node.hpp"
#include "rmw/impl/cpp/demangle.hpp"

using rclcpp::exceptions::throw_from_rcl_error;

namespace rclcpp
{
namespace graph_listener
{

GraphListener::GraphListener()
: is_started_(false), is_shutdown_(false), shutdown_guard_condition_(nullptr)
{
  rcl_ret_t ret = rcl_guard_condition_init(
    &interrupt_guard_condition_,
    rcl_guard_condition_get_default_options());
  if (RCL_RET_OK != ret) {
    throw_from_rcl_error(ret, "failed to create interrupt guard condition");
  }

  shutdown_guard_condition_ = rclcpp::utilities::get_sigint_guard_condition(&wait_set_);
}

GraphListener::~GraphListener()
{
  this->shutdown();
}

void
GraphListener::start_if_not_started()
{
  std::lock_guard<std::mutex> shutdown_lock(shutdown_mutex_);
  if (is_shutdown_.load()) {
    throw GraphListenerShutdownError();
  }
  if (!is_started_) {
    // Initialize the wait set before starting.
    rcl_ret_t ret = rcl_wait_set_init(
      &wait_set_,
      0,  // number_of_subscriptions
      2,  // number_of_guard_conditions
      0,  // number_of_timers
      0,  // number_of_clients
      0,  // number_of_services
      rcl_get_default_allocator());
    if (RCL_RET_OK != ret) {
      throw_from_rcl_error(ret, "failed to initialize wait set");
    }
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
    fprintf(stderr,
      "[rclcpp] caught %s exception in GraphListener thread: %s\n",
      rmw::impl::cpp::demangle(exc).c_str(),
      exc.what());
    std::rethrow_exception(std::current_exception());
  } catch (...) {
    fprintf(stderr, "[rclcpp] unknown error in GraphListener thread\n");
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
      // nodes_mutex_ after waking up rcl_wait.
      std::lock_guard<std::mutex> nodes_barrier_lock(nodes_barrier_mutex_);
      // This is ownership is passed to nodes_lock in the next line.
      nodes_mutex_.lock();
    }
    // This lock is released when the loop continues or exits.
    std::lock_guard<std::mutex> nodes_lock(nodes_mutex_, std::adopt_lock);

    // Resize the wait set if necessary.
    if (wait_set_.size_of_guard_conditions < (nodes_.size() + 2)) {
      ret = rcl_wait_set_resize_guard_conditions(&wait_set_, nodes_.size() + 2);
      if (RCL_RET_OK != ret) {
        throw_from_rcl_error(ret, "failed to resize wait set");
      }
    }
    // Clear the wait set's guard conditions.
    ret = rcl_wait_set_clear_guard_conditions(&wait_set_);
    if (RCL_RET_OK != ret) {
      throw_from_rcl_error(ret, "failed to clear wait set");
    }
    // Put the interrupt guard condition in the wait set.
    ret = rcl_wait_set_add_guard_condition(&wait_set_, &interrupt_guard_condition_);
    if (RCL_RET_OK != ret) {
      throw_from_rcl_error(ret, "failed to add interrupt guard condition to wait set");
    }
    // Put the shutdown guard condition in the wait set.
    ret = rcl_wait_set_add_guard_condition(&wait_set_, shutdown_guard_condition_);
    if (RCL_RET_OK != ret) {
      throw_from_rcl_error(ret, "failed to add shutdown guard condition to wait set");
    }
    // Put graph guard conditions for each node into the wait set.
    for (const auto node_ptr : nodes_) {
      // Only wait on graph changes if some user of the node is watching.
      if (node_ptr->count_graph_users() == 0) {
        continue;
      }
      // Add the graph guard condition for the node to the wait set.
      auto graph_gc = rcl_node_get_graph_guard_condition(node_ptr->get_rcl_node_handle());
      if (!graph_gc) {
        throw_from_rcl_error(RCL_RET_ERROR, "failed to get graph guard condition");
      }
      ret = rcl_wait_set_add_guard_condition(&wait_set_, graph_gc);
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

    bool shutdown_guard_condition_triggered = false;
    // Check to see if the shutdown guard condition has been triggered.
    for (size_t i = 0; i < wait_set_.size_of_guard_conditions; ++i) {
      if (shutdown_guard_condition_ == wait_set_.guard_conditions[i]) {
        shutdown_guard_condition_triggered = true;
      }
    }
    // Notify nodes who's guard conditions are set (triggered).
    for (const auto node_ptr : nodes_) {
      auto graph_gc = rcl_node_get_graph_guard_condition(node_ptr->get_rcl_node_handle());
      if (!graph_gc) {
        throw_from_rcl_error(RCL_RET_ERROR, "failed to get graph guard condition");
      }
      for (size_t i = 0; i < wait_set_.size_of_guard_conditions; ++i) {
        if (graph_gc == wait_set_.guard_conditions[i]) {
          node_ptr->notify_graph_change();
        }
      }
      if (shutdown_guard_condition_triggered) {
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
  std::mutex * nodes_barrier_mutex,
  std::mutex * nodes_mutex,
  rcl_guard_condition_t * interrupt_guard_condition)
{
  {
    // Acquire this lock to prevent the run loop from re-locking the
    // nodes_mutext_ after being woken up.
    std::lock_guard<std::mutex> nodes_barrier_lock(*nodes_barrier_mutex);
    // Trigger the interrupt guard condition to wake up rcl_wait.
    interrupt_(interrupt_guard_condition);
    nodes_mutex->lock();
  }
}

static bool
has_node_(std::vector<rclcpp::node::Node *> * nodes, rclcpp::node::Node * node)
{
  for (const auto node_ptr : (*nodes)) {
    if (node == node_ptr) {
      return true;
    }
  }
  return false;
}

bool
GraphListener::has_node(rclcpp::node::Node * node)
{
  if (!node) {
    return false;
  }
  // Acquire the nodes mutex using the barrier to prevent the run loop from
  // re-locking the nodes mutex after being interrupted.
  acquire_nodes_lock_(&nodes_barrier_mutex_, &nodes_mutex_, &interrupt_guard_condition_);
  // Store the now acquired nodes_mutex_ in the scoped lock using adopt_lock.
  std::lock_guard<std::mutex> nodes_lock(nodes_mutex_, std::adopt_lock);
  return has_node_(&nodes_, node);
}

void
GraphListener::add_node(rclcpp::node::Node * node)
{
  if (!node) {
    throw std::invalid_argument("node is nullptr");
  }
  std::lock_guard<std::mutex> shutdown_lock(shutdown_mutex_);
  if (is_shutdown_.load()) {
    throw GraphListenerShutdownError();
  }

  // Acquire the nodes mutex using the barrier to prevent the run loop from
  // re-locking the nodes mutex after being interrupted.
  acquire_nodes_lock_(&nodes_barrier_mutex_, &nodes_mutex_, &interrupt_guard_condition_);
  // Store the now acquired nodes_mutex_ in the scoped lock using adopt_lock.
  std::lock_guard<std::mutex> nodes_lock(nodes_mutex_, std::adopt_lock);
  if (has_node_(&nodes_, node)) {
    throw NodeAlreadyAddedError();
  }
  nodes_.push_back(node);
  // The run loop has already been interrupted by acquire_nodes_lock_() and
  // will evaluate the new node when nodes_lock releases the nodes_mutex_.
}

static void
remove_node_(std::vector<rclcpp::node::Node *> * nodes, rclcpp::node::Node * node)
{
  // Remove the node if it is found.
  for (auto it = nodes->begin(); it != nodes->end(); ++it) {
    if (node == *it) {
      // Found the node, remove it.
      nodes->erase(it);
      // Now trigger the interrupt guard condition to make sure
      return;
    }
  }
  // Not found in the loop.
  throw NodeNotFoundError();
}

void
GraphListener::remove_node(rclcpp::node::Node * node)
{
  if (!node) {
    throw std::invalid_argument("node is nullptr");
  }
  std::lock_guard<std::mutex> shutdown_lock(shutdown_mutex_);
  if (is_shutdown()) {
    // If shutdown, then the run loop has been joined, so we can remove them directly.
    return remove_node_(&nodes_, node);
  }
  // Otherwise, first interrupt and lock against the run loop to safely remove the node.
  // Acquire the nodes mutex using the barrier to prevent the run loop from
  // re-locking the nodes mutex after being interrupted.
  acquire_nodes_lock_(&nodes_barrier_mutex_, &nodes_mutex_, &interrupt_guard_condition_);
  // Store the now acquired nodes_mutex_ in the scoped lock using adopt_lock.
  std::lock_guard<std::mutex> nodes_lock(nodes_mutex_, std::adopt_lock);
  remove_node_(&nodes_, node);
}

void
GraphListener::shutdown()
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
    if (shutdown_guard_condition_) {
      rclcpp::utilities::release_sigint_guard_condition(&wait_set_);
      shutdown_guard_condition_ = nullptr;
    }
    ret = rcl_wait_set_fini(&wait_set_);
    if (RCL_RET_OK != ret) {
      throw_from_rcl_error(ret, "failed to finalize wait set");
    }
  }
}

bool
GraphListener::is_shutdown()
{
  return is_shutdown_.load();
}

}  // namespace graph_listener
}  // namespace rclcpp
