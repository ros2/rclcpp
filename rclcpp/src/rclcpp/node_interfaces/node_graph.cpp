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

#include "rclcpp/node_interfaces/node_graph.hpp"

#include <map>
#include <string>
#include <vector>

#include "rcl/graph.h"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/event.hpp"
#include "rclcpp/graph_listener.hpp"

using rclcpp::node_interfaces::NodeGraph;
using rclcpp::exceptions::throw_from_rcl_error;
using rclcpp::graph_listener::GraphListener;

NodeGraph::NodeGraph(rclcpp::node_interfaces::NodeBaseInterface * node_base)
: node_base_(node_base),
  graph_listener_(node_base->get_context()->get_sub_context<GraphListener>()),
  should_add_to_graph_listener_(true),
  graph_users_count_(0)
{}

NodeGraph::~NodeGraph()
{
  // Remove self from graph listener.
  // Exchange with false to prevent others from trying to add this node to the
  // graph listener after checking that it was not here.
  if (!should_add_to_graph_listener_.exchange(false)) {
    // If it was already false, then it needs to now be removed.
    graph_listener_->remove_node(this);
  }
}

std::map<std::string, std::string>
NodeGraph::get_topic_names_and_types() const
{
  rcl_topic_names_and_types_t topic_names_and_types =
    rcl_get_zero_initialized_topic_names_and_types();

  auto ret = rcl_get_topic_names_and_types(node_base_->get_rcl_node_handle(),
      &topic_names_and_types);
  if (ret != RMW_RET_OK) {
    // *INDENT-OFF*
    throw std::runtime_error(
      std::string("could not get topic names and types: ") + rmw_get_error_string_safe());
    // *INDENT-ON*
  }

  std::map<std::string, std::string> topics;
  for (size_t i = 0; i < topic_names_and_types.topic_count; ++i) {
    topics[topic_names_and_types.topic_names[i]] = topic_names_and_types.type_names[i];
  }

  ret = rmw_destroy_topic_names_and_types(&topic_names_and_types);
  if (ret != RMW_RET_OK) {
    // *INDENT-OFF*
    throw std::runtime_error(
      std::string("could not destroy topic names and types: ") + rmw_get_error_string_safe());
    // *INDENT-ON*
  }

  return topics;
}

std::vector<std::string>
NodeGraph::get_node_names() const
{
  rcl_string_array_t node_names_c =
    rcl_get_zero_initialized_string_array();

  auto ret = rcl_get_node_names(node_base_->get_rcl_node_handle(),
      &node_names_c);
  if (ret != RMW_RET_OK) {
    if (rmw_destroy_node_names(&node_names_c) != RMW_RET_OK) {
      RMW_SET_ERROR_MSG("Fatal: Leaking node_name memory.");
    }
    // *INDENT-OFF*
    throw std::runtime_error(
      std::string("could not get node names: ") + rmw_get_error_string_safe());
    // *INDENT-ON*
  }

  std::vector<std::string> node_names(&node_names_c.data[0],
    &node_names_c.data[0 + node_names_c.size]);
  ret = rmw_destroy_node_names(&node_names_c);
  if (ret != RMW_RET_OK) {
    // *INDENT-OFF*
    throw std::runtime_error(
      std::string("could not destroy node names: ") + rmw_get_error_string_safe());
    // *INDENT-ON*
  }

  return node_names;
}

size_t
NodeGraph::count_publishers(const std::string & topic_name) const
{
  size_t count;
  // TODO(wjwwood): use the rcl equivalent methods
  auto ret = rmw_count_publishers(rcl_node_get_rmw_handle(node_base_->get_rcl_node_handle()),
      topic_name.c_str(), &count);
  if (ret != RMW_RET_OK) {
    // *INDENT-OFF*
    throw std::runtime_error(
      std::string("could not count publishers: ") + rmw_get_error_string_safe());
    // *INDENT-ON*
  }
  return count;
}

size_t
NodeGraph::count_subscribers(const std::string & topic_name) const
{
  size_t count;
  // TODO(wjwwood): use the rcl equivalent methods
  auto ret = rmw_count_subscribers(rcl_node_get_rmw_handle(node_base_->get_rcl_node_handle()),
      topic_name.c_str(), &count);
  if (ret != RMW_RET_OK) {
    // *INDENT-OFF*
    throw std::runtime_error(
      std::string("could not count subscribers: ") + rmw_get_error_string_safe());
    // *INDENT-ON*
  }
  return count;
}

const rcl_guard_condition_t *
NodeGraph::get_graph_guard_condition() const
{
  return rcl_node_get_graph_guard_condition(node_base_->get_rcl_node_handle());
}

void
NodeGraph::notify_graph_change()
{
  {
    std::lock_guard<std::mutex> graph_changed_lock(graph_mutex_);
    bool bad_ptr_encountered = false;
    for (auto & event_wptr : graph_events_) {
      auto event_ptr = event_wptr.lock();
      if (event_ptr) {
        event_ptr->set();
      } else {
        bad_ptr_encountered = true;
      }
    }
    if (bad_ptr_encountered) {
      // remove invalid pointers with the erase-remove idiom
      graph_events_.erase(
        std::remove_if(
          graph_events_.begin(),
          graph_events_.end(),
          [](const rclcpp::event::Event::WeakPtr & wptr) {
        return wptr.expired();
      }),
        graph_events_.end());
      // update graph_users_count_
      graph_users_count_.store(graph_events_.size());
    }
  }
  graph_cv_.notify_all();
  {
    auto notify_condition_lock = node_base_->acquire_notify_guard_condition_lock();
    rcl_ret_t ret = rcl_trigger_guard_condition(node_base_->get_notify_guard_condition());
    if (RCL_RET_OK != ret) {
      throw_from_rcl_error(ret, "failed to trigger notify guard condition");
    }
  }
}

void
NodeGraph::notify_shutdown()
{
  // notify here anything that will not be woken up by ctrl-c or rclcpp::shutdown().
  graph_cv_.notify_all();
}

rclcpp::event::Event::SharedPtr
NodeGraph::get_graph_event()
{
  auto event = rclcpp::event::Event::make_shared();
  std::lock_guard<std::mutex> graph_changed_lock(graph_mutex_);
  // on first call, add node to graph_listener_
  if (should_add_to_graph_listener_.exchange(false)) {
    graph_listener_->add_node(this);
    graph_listener_->start_if_not_started();
  }
  graph_events_.push_back(event);
  graph_users_count_++;
  return event;
}

void
NodeGraph::wait_for_graph_change(
  rclcpp::event::Event::SharedPtr event,
  std::chrono::nanoseconds timeout)
{
  using rclcpp::exceptions::InvalidEventError;
  using rclcpp::exceptions::EventNotRegisteredError;
  if (!event) {
    throw InvalidEventError();
  }
  {
    std::lock_guard<std::mutex> graph_changed_lock(graph_mutex_);
    bool event_in_graph_events = false;
    for (const auto & event_wptr : graph_events_) {
      if (event == event_wptr.lock()) {
        event_in_graph_events = true;
        break;
      }
    }
    if (!event_in_graph_events) {
      throw EventNotRegisteredError();
    }
  }
  std::unique_lock<std::mutex> graph_lock(graph_mutex_);
  graph_cv_.wait_for(graph_lock, timeout, [&event]() {
    return event->check() || !rclcpp::utilities::ok();
  });
}

size_t
NodeGraph::count_graph_users()
{
  return graph_users_count_.load();
}
