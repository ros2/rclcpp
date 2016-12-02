// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/graph_listener.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/node_base.hpp"
#include "rclcpp/node_interfaces/node_graph.hpp"
#include "rclcpp/node_interfaces/node_topics.hpp"

using rclcpp::node::Node;
using rclcpp::exceptions::throw_from_rcl_error;

Node::Node(const std::string & node_name, bool use_intra_process_comms)
: Node(
    node_name,
    rclcpp::contexts::default_context::get_global_default_context(),
    use_intra_process_comms)
{}

Node::Node(
  const std::string & node_name,
  rclcpp::context::Context::SharedPtr context,
  bool use_intra_process_comms)
: node_base_(new rclcpp::node_interfaces::NodeBase(node_name, context)),
  node_topics_(new rclcpp::node_interfaces::NodeTopics(node_base_.get())),
  node_graph_(new rclcpp::node_interfaces::NodeGraph(node_base_.get())),
  number_of_timers_(0), number_of_services_(0), number_of_clients_(0),
  use_intra_process_comms_(use_intra_process_comms)
{
  events_publisher_ = create_publisher<rcl_interfaces::msg::ParameterEvent>(
    "parameter_events", rmw_qos_profile_parameter_events);
}

Node::~Node()
{}

const std::string &
Node::get_name() const
{
  return node_base_->get_name();
}

rclcpp::callback_group::CallbackGroup::SharedPtr
Node::create_callback_group(
  rclcpp::callback_group::CallbackGroupType group_type)
{
  return node_base_->create_callback_group(group_type);
}

bool
Node::group_in_node(rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  return node_base_->callback_group_in_node(group);
}

// TODO(wjwwood): reenable this once I figure out why the demo doesn't build with it.
// rclcpp::timer::WallTimer::SharedPtr
// Node::create_wall_timer(
//   std::chrono::duration<long double, std::nano> period,
//   rclcpp::timer::CallbackType callback,
//   rclcpp::callback_group::CallbackGroup::SharedPtr group)
// {
//   return create_wall_timer(
//     std::chrono::duration_cast<std::chrono::nanoseconds>(period),
//     callback,
//     group);
// }

std::vector<rcl_interfaces::msg::SetParametersResult>
Node::set_parameters(
  const std::vector<rclcpp::parameter::ParameterVariant> & parameters)
{
  std::vector<rcl_interfaces::msg::SetParametersResult> results;
  for (auto p : parameters) {
    auto result = set_parameters_atomically({{p}});
    results.push_back(result);
  }
  return results;
}

rcl_interfaces::msg::SetParametersResult
Node::set_parameters_atomically(
  const std::vector<rclcpp::parameter::ParameterVariant> & parameters)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::map<std::string, rclcpp::parameter::ParameterVariant> tmp_map;
  auto parameter_event = std::make_shared<rcl_interfaces::msg::ParameterEvent>();

  // TODO(jacquelinekay): handle parameter constraints
  rcl_interfaces::msg::SetParametersResult result;
  if (parameters_callback_) {
    result = parameters_callback_(parameters);
  } else {
    result.successful = true;
  }

  if (!result.successful) {
    return result;
  }

  for (auto p : parameters) {
    if (parameters_.find(p.get_name()) == parameters_.end()) {
      if (p.get_type() != rclcpp::parameter::ParameterType::PARAMETER_NOT_SET) {
        parameter_event->new_parameters.push_back(p.to_parameter());
      }
    } else if (p.get_type() != rclcpp::parameter::ParameterType::PARAMETER_NOT_SET) {
      parameter_event->changed_parameters.push_back(p.to_parameter());
    } else {
      parameter_event->deleted_parameters.push_back(p.to_parameter());
    }
    tmp_map[p.get_name()] = p;
  }
  // std::map::insert will not overwrite elements, so we'll keep the new
  // ones and add only those that already exist in the Node's internal map
  tmp_map.insert(parameters_.begin(), parameters_.end());
  std::swap(tmp_map, parameters_);

  events_publisher_->publish(parameter_event);

  return result;
}

std::vector<rclcpp::parameter::ParameterVariant>
Node::get_parameters(
  const std::vector<std::string> & names) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<rclcpp::parameter::ParameterVariant> results;

  for (auto & name : names) {
    if (std::any_of(parameters_.cbegin(), parameters_.cend(),
      [&name](const std::pair<std::string, rclcpp::parameter::ParameterVariant> & kv) {
      return name == kv.first;
    }))
    {
      results.push_back(parameters_.at(name));
    }
  }
  return results;
}

const rclcpp::parameter::ParameterVariant
Node::get_parameter(const std::string & name) const
{
  rclcpp::parameter::ParameterVariant parameter;

  if (get_parameter(name, parameter)) {
    return parameter;
  } else {
    throw std::out_of_range("Parameter '" + name + "' not set");
  }
}

bool Node::get_parameter(const std::string & name,
  rclcpp::parameter::ParameterVariant & parameter) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (parameters_.count(name)) {
    parameter = parameters_.at(name);
    return true;
  } else {
    return false;
  }
}

std::vector<rcl_interfaces::msg::ParameterDescriptor>
Node::describe_parameters(
  const std::vector<std::string> & names) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<rcl_interfaces::msg::ParameterDescriptor> results;
  for (auto & kv : parameters_) {
    if (std::any_of(names.cbegin(), names.cend(), [&kv](const std::string & name) {
      return name == kv.first;
    }))
    {
      rcl_interfaces::msg::ParameterDescriptor parameter_descriptor;
      parameter_descriptor.name = kv.first;
      parameter_descriptor.type = kv.second.get_type();
      results.push_back(parameter_descriptor);
    }
  }
  return results;
}

std::vector<uint8_t>
Node::get_parameter_types(
  const std::vector<std::string> & names) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<uint8_t> results;
  for (auto & kv : parameters_) {
    if (std::any_of(names.cbegin(), names.cend(), [&kv](const std::string & name) {
      return name == kv.first;
    }))
    {
      results.push_back(kv.second.get_type());
    } else {
      results.push_back(rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET);
    }
  }
  return results;
}

rcl_interfaces::msg::ListParametersResult
Node::list_parameters(
  const std::vector<std::string> & prefixes, uint64_t depth) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  rcl_interfaces::msg::ListParametersResult result;

  // TODO(esteve): define parameter separator, use "." for now
  for (auto & kv : parameters_) {
    if (((prefixes.size() == 0) &&
      ((depth == rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE) ||
      (static_cast<uint64_t>(std::count(kv.first.begin(), kv.first.end(), '.')) < depth))) ||
      (std::any_of(prefixes.cbegin(), prefixes.cend(), [&kv, &depth](const std::string & prefix) {
      if (kv.first == prefix) {
        return true;
      } else if (kv.first.find(prefix + ".") == 0) {
        size_t length = prefix.length();
        std::string substr = kv.first.substr(length);
        // Cast as unsigned integer to avoid warning
        return (depth == rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE) ||
        (static_cast<uint64_t>(std::count(substr.begin(), substr.end(), '.')) < depth);
      }
      return false;
    })))
    {
      result.names.push_back(kv.first);
      size_t last_separator = kv.first.find_last_of('.');
      if (std::string::npos != last_separator) {
        std::string prefix = kv.first.substr(0, last_separator);
        if (std::find(result.prefixes.cbegin(), result.prefixes.cend(), prefix) ==
          result.prefixes.cend())
        {
          result.prefixes.push_back(prefix);
        }
      }
    }
  }
  return result;
}

std::map<std::string, std::string>
Node::get_topic_names_and_types() const
{
  return node_graph_->get_topic_names_and_types();
}

size_t
Node::count_publishers(const std::string & topic_name) const
{
  return node_graph_->count_publishers(topic_name);
}

size_t
Node::count_subscribers(const std::string & topic_name) const
{
  return node_graph_->count_subscribers(topic_name);
}

const std::vector<Node::CallbackGroupWeakPtr> &
Node::get_callback_groups() const
{
  return node_base_->get_callback_groups();
}

const rcl_guard_condition_t *
Node::get_notify_guard_condition() const
{
  return node_base_->get_notify_guard_condition();
}

const rcl_guard_condition_t *
Node::get_graph_guard_condition() const
{
  return node_graph_->get_graph_guard_condition();
}

const rcl_node_t *
Node::get_rcl_node_handle() const
{
  return node_base_->get_rcl_node_handle();
}

rcl_node_t *
Node::get_rcl_node_handle()
{
  return node_base_->get_rcl_node_handle();
}

std::shared_ptr<rcl_node_t>
Node::get_shared_rcl_node_handle()
{
  return node_base_->get_shared_rcl_node_handle();
}

rclcpp::event::Event::SharedPtr
Node::get_graph_event()
{
  return node_graph_->get_graph_event();
}

void
Node::wait_for_graph_change(
  rclcpp::event::Event::SharedPtr event,
  std::chrono::nanoseconds timeout)
{
  node_graph_->wait_for_graph_change(event, timeout);
}

// PROTECTED IMPLEMENTATION
void
Node::add_service(rclcpp::service::ServiceBase::SharedPtr serv_base_ptr,
  rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  if (!serv_base_ptr) {
    throw std::runtime_error("Cannot add empty service to group");
  }

  if (group) {
    if (!group_in_node(group)) {
      // TODO(jacquelinekay): use custom exception
      throw std::runtime_error("Cannot create service, group not in node.");
    }
    group->add_service(serv_base_ptr);
  } else {
    node_base_->get_default_callback_group()->add_service(serv_base_ptr);
  }
  number_of_services_++;
  if (rcl_trigger_guard_condition(node_base_->get_notify_guard_condition()) != RCL_RET_OK) {
    throw std::runtime_error(
            std::string(
              "Failed to notify waitset on service creation: ") + rmw_get_error_string());
  }
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
Node::get_node_base_interface()
{
  return node_base_;
}

rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr
Node::get_node_topics_interface()
{
  return node_topics_;
}

rclcpp::node_interfaces::NodeGraphInterface::SharedPtr
Node::get_node_graph_interface()
{
  return node_graph_;
}
