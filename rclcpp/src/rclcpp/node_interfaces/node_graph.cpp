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

#include <algorithm>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "rcl/graph.h"
#include "rcl/remap.h"
#include "rclcpp/event.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rclcpp/graph_listener.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"

using rclcpp::node_interfaces::NodeGraph;
using rclcpp::exceptions::throw_from_rcl_error;
using rclcpp::graph_listener::GraphListener;

NodeGraph::NodeGraph(rclcpp::node_interfaces::NodeBaseInterface * node_base)
: node_base_(node_base),
  graph_listener_(
    node_base->get_context()->get_sub_context<GraphListener>(node_base->get_context())
  ),
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

std::map<std::string, std::vector<std::string>>
NodeGraph::get_topic_names_and_types(bool no_demangle) const
{
  rcl_names_and_types_t topic_names_and_types = rcl_get_zero_initialized_names_and_types();

  rcl_allocator_t allocator = rcl_get_default_allocator();
  auto ret = rcl_get_topic_names_and_types(
    node_base_->get_rcl_node_handle(),
    &allocator,
    no_demangle,
    &topic_names_and_types);
  if (ret != RCL_RET_OK) {
    auto error_msg = std::string("failed to get topic names and types: ") +
      rcl_get_error_string().str;
    rcl_reset_error();
    if (rcl_names_and_types_fini(&topic_names_and_types) != RCL_RET_OK) {
      error_msg += std::string(", failed also to cleanup topic names and types, leaking memory: ") +
        rcl_get_error_string().str;
      rcl_reset_error();
    }
    throw std::runtime_error(error_msg);
  }

  std::map<std::string, std::vector<std::string>> topics_and_types;
  for (size_t i = 0; i < topic_names_and_types.names.size; ++i) {
    std::string topic_name = topic_names_and_types.names.data[i];
    for (size_t j = 0; j < topic_names_and_types.types[i].size; ++j) {
      topics_and_types[topic_name].emplace_back(topic_names_and_types.types[i].data[j]);
    }
  }

  ret = rcl_names_and_types_fini(&topic_names_and_types);
  if (ret != RCL_RET_OK) {
    // *INDENT-OFF*
    throw std::runtime_error(
      std::string("could not destroy topic names and types: ") + rcl_get_error_string().str);
    // *INDENT-ON*
  }

  return topics_and_types;
}

std::map<std::string, std::vector<std::string>>
NodeGraph::get_service_names_and_types() const
{
  rcl_names_and_types_t service_names_and_types = rcl_get_zero_initialized_names_and_types();

  rcl_allocator_t allocator = rcl_get_default_allocator();
  auto ret = rcl_get_service_names_and_types(
    node_base_->get_rcl_node_handle(),
    &allocator,
    &service_names_and_types);
  if (ret != RCL_RET_OK) {
    auto error_msg = std::string("failed to get service names and types: ") +
      rcl_get_error_string().str;
    rcl_reset_error();
    if (rcl_names_and_types_fini(&service_names_and_types) != RCL_RET_OK) {
      error_msg +=
        std::string(", failed also to cleanup service names and types, leaking memory: ") +
        rcl_get_error_string().str;
      rcl_reset_error();
    }
    throw std::runtime_error(error_msg);
  }

  std::map<std::string, std::vector<std::string>> services_and_types;
  for (size_t i = 0; i < service_names_and_types.names.size; ++i) {
    std::string service_name = service_names_and_types.names.data[i];
    for (size_t j = 0; j < service_names_and_types.types[i].size; ++j) {
      services_and_types[service_name].emplace_back(service_names_and_types.types[i].data[j]);
    }
  }

  ret = rcl_names_and_types_fini(&service_names_and_types);
  if (ret != RCL_RET_OK) {
    // *INDENT-OFF*
    throw std::runtime_error(
      std::string("could not destroy service names and types: ") + rcl_get_error_string().str);
    // *INDENT-ON*
  }

  return services_and_types;
}

std::map<std::string, std::vector<std::string>>
NodeGraph::get_service_names_and_types_by_node(
  const std::string & node_name,
  const std::string & namespace_) const
{
  rcl_names_and_types_t service_names_and_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_get_service_names_and_types_by_node(
    node_base_->get_rcl_node_handle(),
    &allocator,
    node_name.c_str(),
    namespace_.c_str(),
    &service_names_and_types);
  if (ret != RCL_RET_OK) {
    auto error_msg = std::string("failed to get service names and types by node: ") +
      rcl_get_error_string().str;
    rcl_reset_error();
    if (rcl_names_and_types_fini(&service_names_and_types) != RCL_RET_OK) {
      error_msg +=
        std::string(", failed also to cleanup service names and types, leaking memory: ") +
        rcl_get_error_string().str;
      rcl_reset_error();
    }
    throw std::runtime_error(error_msg);
  }

  std::map<std::string, std::vector<std::string>> services_and_types;
  for (size_t i = 0; i < service_names_and_types.names.size; ++i) {
    std::string service_name = service_names_and_types.names.data[i];
    for (size_t j = 0; j < service_names_and_types.types[i].size; ++j) {
      services_and_types[service_name].emplace_back(service_names_and_types.types[i].data[j]);
    }
  }

  ret = rcl_names_and_types_fini(&service_names_and_types);
  if (ret != RCL_RET_OK) {
    throw_from_rcl_error(ret, "could not destroy service names and types");
  }

  return services_and_types;
}

std::vector<std::string>
NodeGraph::get_node_names() const
{
  std::vector<std::string> nodes;
  auto names_and_namespaces = get_node_names_and_namespaces();

  std::transform(
    names_and_namespaces.begin(),
    names_and_namespaces.end(),
    std::back_inserter(nodes),
    [](std::pair<std::string, std::string> nns) {
      std::string return_string;
      if (nns.second.back() == '/') {
        return_string = nns.second + nns.first;
      } else {
        return_string = nns.second + '/' + nns.first;
      }
      // Quick check to make sure that we start with a slash
      // Since fully-qualified strings need to
      if (return_string.front() != '/') {
        return_string = "/" + return_string;
      }
      return return_string;
    }
  );
  return nodes;
}

std::vector<std::pair<std::string, std::string>>
NodeGraph::get_node_names_and_namespaces() const
{
  rcutils_string_array_t node_names_c =
    rcutils_get_zero_initialized_string_array();
  rcutils_string_array_t node_namespaces_c =
    rcutils_get_zero_initialized_string_array();

  auto allocator = rcl_get_default_allocator();
  auto ret = rcl_get_node_names(
    node_base_->get_rcl_node_handle(),
    allocator,
    &node_names_c,
    &node_namespaces_c);
  if (ret != RCL_RET_OK) {
    auto error_msg = std::string("failed to get node names: ") + rcl_get_error_string().str;
    rcl_reset_error();
    if (rcutils_string_array_fini(&node_names_c) != RCUTILS_RET_OK) {
      error_msg += std::string(", failed also to cleanup node names, leaking memory: ") +
        rcl_get_error_string().str;
      rcl_reset_error();
    }
    if (rcutils_string_array_fini(&node_namespaces_c) != RCUTILS_RET_OK) {
      error_msg += std::string(", failed also to cleanup node namespaces, leaking memory: ") +
        rcl_get_error_string().str;
      rcl_reset_error();
    }
    // TODO(karsten1987): Append rcutils_error_message once it's in master
    throw std::runtime_error(error_msg);
  }


  std::vector<std::pair<std::string, std::string>> node_names;
  node_names.reserve(node_names_c.size);
  for (size_t i = 0; i < node_names_c.size; ++i) {
    if (node_names_c.data[i] && node_namespaces_c.data[i]) {
      node_names.emplace_back(node_names_c.data[i], node_namespaces_c.data[i]);
    }
  }

  std::string error;
  rcl_ret_t ret_names = rcutils_string_array_fini(&node_names_c);
  if (ret_names != RCUTILS_RET_OK) {
    // *INDENT-OFF*
    // TODO(karsten1987): Append rcutils_error_message once it's in master
    error = "could not destroy node names";
    // *INDENT-ON*
  }
  rcl_ret_t ret_ns = rcutils_string_array_fini(&node_namespaces_c);
  if (ret_ns != RCUTILS_RET_OK) {
    // *INDENT-OFF*
    // TODO(karsten1987): Append rcutils_error_message once it's in master
    error += ", could not destroy node namespaces";
    // *INDENT-ON*
  }

  if (ret_names != RCUTILS_RET_OK || ret_ns != RCUTILS_RET_OK) {
    throw std::runtime_error(error);
  }

  return node_names;
}

size_t
NodeGraph::count_publishers(const std::string & topic_name) const
{
  auto rcl_node_handle = node_base_->get_rcl_node_handle();

  auto fqdn = rclcpp::expand_topic_or_service_name(
    topic_name,
    rcl_node_get_name(rcl_node_handle),
    rcl_node_get_namespace(rcl_node_handle),
    false);    // false = not a service

  size_t count;
  auto ret = rcl_count_publishers(rcl_node_handle, fqdn.c_str(), &count);
  if (ret != RMW_RET_OK) {
    // *INDENT-OFF*
    throw std::runtime_error(
      std::string("could not count publishers: ") + rmw_get_error_string().str);
    // *INDENT-ON*
  }
  return count;
}

size_t
NodeGraph::count_subscribers(const std::string & topic_name) const
{
  auto rcl_node_handle = node_base_->get_rcl_node_handle();

  auto fqdn = rclcpp::expand_topic_or_service_name(
    topic_name,
    rcl_node_get_name(rcl_node_handle),
    rcl_node_get_namespace(rcl_node_handle),
    false);    // false = not a service

  size_t count;
  auto ret = rcl_count_subscribers(rcl_node_handle, fqdn.c_str(), &count);
  if (ret != RMW_RET_OK) {
    // *INDENT-OFF*
    throw std::runtime_error(
      std::string("could not count subscribers: ") + rmw_get_error_string().str);
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
          [](const rclcpp::Event::WeakPtr & wptr) {
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

rclcpp::Event::SharedPtr
NodeGraph::get_graph_event()
{
  auto event = rclcpp::Event::make_shared();
  {
    std::lock_guard<std::mutex> graph_changed_lock(graph_mutex_);
    graph_events_.push_back(event);
    graph_users_count_++;
  }
  // on first call, add node to graph_listener_
  if (should_add_to_graph_listener_.exchange(false)) {
    graph_listener_->add_node(this);
    graph_listener_->start_if_not_started();
  }
  return event;
}

void
NodeGraph::wait_for_graph_change(
  rclcpp::Event::SharedPtr event,
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
  auto pred = [&event, context = node_base_->get_context()]() {
      return event->check() || !rclcpp::ok(context);
    };
  std::unique_lock<std::mutex> graph_lock(graph_mutex_);
  if (!pred()) {
    graph_cv_.wait_for(graph_lock, timeout, pred);
  }
}

size_t
NodeGraph::count_graph_users() const
{
  return graph_users_count_.load();
}

static
std::vector<rclcpp::TopicEndpointInfo>
convert_to_topic_info_list(const rcl_topic_endpoint_info_array_t & info_array)
{
  std::vector<rclcpp::TopicEndpointInfo> topic_info_list;
  for (size_t i = 0; i < info_array.size; ++i) {
    topic_info_list.push_back(rclcpp::TopicEndpointInfo(info_array.info_array[i]));
  }
  return topic_info_list;
}

template<const char * EndpointType, typename FunctionT>
static std::vector<rclcpp::TopicEndpointInfo>
get_info_by_topic(
  rclcpp::node_interfaces::NodeBaseInterface * node_base,
  const std::string & topic_name,
  bool no_mangle,
  FunctionT rcl_get_info_by_topic)
{
  std::string fqdn;
  auto rcl_node_handle = node_base->get_rcl_node_handle();

  if (no_mangle) {
    fqdn = topic_name;
  } else {
    fqdn = rclcpp::expand_topic_or_service_name(
      topic_name,
      rcl_node_get_name(rcl_node_handle),
      rcl_node_get_namespace(rcl_node_handle),
      false);    // false = not a service

    // Get the node options
    const rcl_node_options_t * node_options = rcl_node_get_options(rcl_node_handle);
    if (nullptr == node_options) {
      throw std::runtime_error("Need valid node options in get_info_by_topic()");
    }
    const rcl_arguments_t * global_args = nullptr;
    if (node_options->use_global_arguments) {
      global_args = &(rcl_node_handle->context->global_arguments);
    }

    char * remapped_topic_name = nullptr;
    rcl_ret_t ret = rcl_remap_topic_name(
      &(node_options->arguments),
      global_args,
      fqdn.c_str(),
      rcl_node_get_name(rcl_node_handle),
      rcl_node_get_namespace(rcl_node_handle),
      node_options->allocator,
      &remapped_topic_name);
    if (RCL_RET_OK != ret) {
      throw_from_rcl_error(ret, std::string("Failed to remap topic name ") + fqdn);
    } else if (nullptr != remapped_topic_name) {
      fqdn = remapped_topic_name;
      node_options->allocator.deallocate(remapped_topic_name, node_options->allocator.state);
    }
  }

  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcl_topic_endpoint_info_array_t info_array = rcl_get_zero_initialized_topic_endpoint_info_array();
  rcl_ret_t ret =
    rcl_get_info_by_topic(rcl_node_handle, &allocator, fqdn.c_str(), no_mangle, &info_array);
  if (RCL_RET_OK != ret) {
    auto error_msg =
      std::string("Failed to get information by topic for ") + EndpointType + std::string(":");
    if (RCL_RET_UNSUPPORTED == ret) {
      error_msg += std::string("function not supported by RMW_IMPLEMENTATION");
    } else {
      error_msg += rcl_get_error_string().str;
    }
    rcl_reset_error();
    if (RCL_RET_OK != rcl_topic_endpoint_info_array_fini(&info_array, &allocator)) {
      error_msg += std::string(", failed also to cleanup topic info array, leaking memory: ") +
        rcl_get_error_string().str;
      rcl_reset_error();
    }
    throw_from_rcl_error(ret, error_msg);
  }

  std::vector<rclcpp::TopicEndpointInfo> topic_info_list = convert_to_topic_info_list(info_array);
  ret = rcl_topic_endpoint_info_array_fini(&info_array, &allocator);
  if (RCL_RET_OK != ret) {
    throw_from_rcl_error(ret, "rcl_topic_info_array_fini failed.");
  }

  return topic_info_list;
}

static constexpr char kPublisherEndpointTypeName[] = "publishers";
std::vector<rclcpp::TopicEndpointInfo>
NodeGraph::get_publishers_info_by_topic(
  const std::string & topic_name,
  bool no_mangle) const
{
  return get_info_by_topic<kPublisherEndpointTypeName>(
    node_base_,
    topic_name,
    no_mangle,
    rcl_get_publishers_info_by_topic);
}

static constexpr char kSubscriptionEndpointTypeName[] = "subscriptions";
std::vector<rclcpp::TopicEndpointInfo>
NodeGraph::get_subscriptions_info_by_topic(
  const std::string & topic_name,
  bool no_mangle) const
{
  return get_info_by_topic<kSubscriptionEndpointTypeName>(
    node_base_,
    topic_name,
    no_mangle,
    rcl_get_subscriptions_info_by_topic);
}

std::string &
rclcpp::TopicEndpointInfo::node_name()
{
  return node_name_;
}

const std::string &
rclcpp::TopicEndpointInfo::node_name() const
{
  return node_name_;
}

std::string &
rclcpp::TopicEndpointInfo::node_namespace()
{
  return node_namespace_;
}

const std::string &
rclcpp::TopicEndpointInfo::node_namespace() const
{
  return node_namespace_;
}

std::string &
rclcpp::TopicEndpointInfo::topic_type()
{
  return topic_type_;
}

const std::string &
rclcpp::TopicEndpointInfo::topic_type() const
{
  return topic_type_;
}

rclcpp::EndpointType &
rclcpp::TopicEndpointInfo::endpoint_type()
{
  return endpoint_type_;
}

const rclcpp::EndpointType &
rclcpp::TopicEndpointInfo::endpoint_type() const
{
  return endpoint_type_;
}

std::array<uint8_t, RMW_GID_STORAGE_SIZE> &
rclcpp::TopicEndpointInfo::endpoint_gid()
{
  return endpoint_gid_;
}

const std::array<uint8_t, RMW_GID_STORAGE_SIZE> &
rclcpp::TopicEndpointInfo::endpoint_gid() const
{
  return endpoint_gid_;
}

rclcpp::QoS &
rclcpp::TopicEndpointInfo::qos_profile()
{
  return qos_profile_;
}

const rclcpp::QoS &
rclcpp::TopicEndpointInfo::qos_profile() const
{
  return qos_profile_;
}
