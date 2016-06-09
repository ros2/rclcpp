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
#include <string>

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
: is_started_(false), is_shutdown_(false)
{
  rcl_ret_t ret = rcl_guard_condition_init(
    &interrupt_guard_condition_,
    rcl_guard_condition_get_default_options());
  if (RCL_RET_OK != ret) {
    throw_from_rcl_error(ret, "failed to create interrupt guard condition");
  }

  ret = rcl_wait_set_init(
    &wait_set_,
    0,  // number_of_subscriptions
    1,  // number_of_guard_conditions
    0,  // number_of_timers
    0,  // number_of_clients
    0,  // number_of_services
    rcl_get_default_allocator());
  if (RCL_RET_OK != ret) {
    throw_from_rcl_error(ret, "failed to initialize wait set");
  }
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
  } catch (...) {
    fprintf(stderr, "[rclcpp] unknown error in GraphListener thread\n");
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
      std::lock_guard<std::mutex> nodes_lock(nodes_mutex_);
      // Reseize the guard conditions set if it is smaller than the number of nodes + 1.
      // One slot is reserverd for the interrupt guard condition.
      if (wait_set_.size_of_guard_conditions < (nodes_.size() + 1)) {
        ret = rcl_wait_set_resize_guard_conditions(&wait_set_, nodes_.size() + 1);
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
    }  // release the nodes_ lock

    // Wait for graph changes or interrupt.
    ret = rcl_wait(&wait_set_, -1);  // block for ever until one of the
    if (RCL_RET_TIMEOUT == ret) {
      throw std::runtime_error("rcl_wait unexpectedly timed out");
    }
    if (RCL_RET_OK != ret) {
      throw_from_rcl_error(ret, "failed to wait on wait set");
    }
    {
      // Notify nodes who's guard conditions are set (triggered).
      std::lock_guard<std::mutex> nodes_lock(nodes_mutex_);
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
      }
    }  // release the nodes_ lock again
  }  // while (true)
}

bool
GraphListener::has_node(rclcpp::node::Node * node) const
{
  if (!node) {
    return false;
  }
  std::lock_guard<std::mutex> lock(nodes_mutex_);
  for (const auto node_ptr : nodes_) {
    if (node == node_ptr) {
      return true;
    }
  }
  return false;
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
  if (this->has_node(node)) {
    throw NodeAlreadyAddedError();
  }
  std::lock_guard<std::mutex> lock(nodes_mutex_);
  nodes_.push_back(node);
}

void
GraphListener::remove_node(rclcpp::node::Node * node)
{
  if (!node) {
    throw std::invalid_argument("node is nullptr");
  }
  std::lock_guard<std::mutex> lock(nodes_mutex_);
  for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
    if (node == *it) {
      nodes_.erase(it);
      return;
    }
  }
  // Not found in the loop.
  throw NodeNotFoundError();
}

static void
interrupt_(rcl_guard_condition_t * interrupt_guard_condition)
{
  rcl_ret_t ret = rcl_trigger_guard_condition(interrupt_guard_condition);
  if (RCL_RET_OK != ret) {
    throw_from_rcl_error(ret, "failed to trigger the interrupt guard condition");
  }
}

void
GraphListener::interrupt()
{
  std::lock_guard<std::mutex> shutdown_lock(shutdown_mutex_);
  if (is_shutdown_.load()) {
    throw GraphListenerShutdownError();
  }
  interrupt_(&interrupt_guard_condition_);
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
  }
  rcl_ret_t ret = rcl_guard_condition_fini(&interrupt_guard_condition_);
  if (RCL_RET_OK != ret) {
    throw_from_rcl_error(ret, "failed to finalize interrupt guard condition");
  }
  ret = rcl_wait_set_fini(&wait_set_);
  if (RCL_RET_OK != ret) {
    throw_from_rcl_error(ret, "failed to finalize wait set");
  }
}

}  // namespace graph_listener
}  // namespace rclcpp
