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

#include "rclcpp/exceptions.hpp"
#include "rclcpp/graph_listener.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/node_base.hpp"
#include "rclcpp/node_interfaces/node_clock.hpp"
#include "rclcpp/node_interfaces/node_graph.hpp"
#include "rclcpp/node_interfaces/node_logging.hpp"
#include "rclcpp/node_interfaces/node_parameters.hpp"
#include "rclcpp/node_interfaces/node_services.hpp"
#include "rclcpp/node_interfaces/node_timers.hpp"
#include "rclcpp/node_interfaces/node_topics.hpp"
#include "rclcpp/node_interfaces/node_waitables.hpp"

#include "rmw/validate_namespace.h"

using rclcpp::Node;
using rclcpp::exceptions::throw_from_rcl_error;

Node::Node(
  const std::string & node_name,
  const std::string & namespace_,
  bool use_intra_process_comms)
: Node(
    node_name,
    namespace_,
    rclcpp::contexts::default_context::get_global_default_context(),
    {},
    {},
    true,
    use_intra_process_comms,
    true)
{}

Node::Node(
  const std::string & node_name,
  const std::string & namespace_,
  rclcpp::Context::SharedPtr context,
  const std::vector<std::string> & arguments,
  const std::vector<rclcpp::Parameter> & initial_parameters,
  bool use_global_arguments,
  bool use_intra_process_comms,
  bool start_parameter_services)
: node_base_(new rclcpp::node_interfaces::NodeBase(
      node_name, namespace_, context, arguments, use_global_arguments)),
  node_graph_(new rclcpp::node_interfaces::NodeGraph(node_base_.get())),
  node_logging_(new rclcpp::node_interfaces::NodeLogging(node_base_.get())),
  node_timers_(new rclcpp::node_interfaces::NodeTimers(node_base_.get())),
  node_topics_(new rclcpp::node_interfaces::NodeTopics(node_base_.get())),
  node_services_(new rclcpp::node_interfaces::NodeServices(node_base_.get())),
  node_clock_(new rclcpp::node_interfaces::NodeClock(
      node_base_,
      node_topics_,
      node_graph_,
      node_services_,
      node_logging_
    )),
  node_parameters_(new rclcpp::node_interfaces::NodeParameters(
      node_base_,
      node_topics_,
      node_services_,
      node_clock_,
      initial_parameters,
      use_intra_process_comms,
      start_parameter_services
    )),
  node_waitables_(new rclcpp::node_interfaces::NodeWaitables(node_base_.get())),
  use_intra_process_comms_(use_intra_process_comms),
  sub_namespace_("")
{
}

Node::Node(
  const Node & other,
  const std::string & sub_namespace)
: node_base_(other.node_base_),
  node_graph_(other.node_graph_),
  node_logging_(other.node_logging_),
  node_timers_(other.node_timers_),
  node_topics_(other.node_topics_),
  node_services_(other.node_services_),
  node_clock_(other.node_clock_),
  node_parameters_(other.node_parameters_),
  use_intra_process_comms_(other.use_intra_process_comms_),
  sub_namespace_("")
{
  sub_namespace_ = sub_namespace;

  if (sub_namespace_.front() == '/') {
    sub_namespace_.replace(0, 1, "");
  }

  if (sub_namespace_.back() == '/') {
    sub_namespace_ = sub_namespace_.substr(0, sub_namespace_.size() - 1);
  }

  if (other.sub_namespace_ != "") {
    sub_namespace_ = other.sub_namespace_ + "/" + sub_namespace;
  }

  std::string full_namespace(node_base_->get_namespace());

  if (full_namespace.back() == '/') {
    full_namespace += sub_namespace_;
  } else {
    full_namespace += "/" + sub_namespace_;
  }

  int validation_result;
  size_t invalid_index;
  rmw_ret_t rmw_ret =
    rmw_validate_namespace(full_namespace.c_str(), &validation_result, &invalid_index);

  if (rmw_ret != RMW_RET_OK) {
    if (rmw_ret == RMW_RET_INVALID_ARGUMENT) {
      throw_from_rcl_error(RCL_RET_INVALID_ARGUMENT, "failed to validate subnode namespace");
    }
    throw_from_rcl_error(RCL_RET_ERROR, "failed to validate subnode namespace");
  }

  if (validation_result != RMW_NAMESPACE_VALID) {
    throw rclcpp::exceptions::InvalidNamespaceError(
            full_namespace.c_str(),
            rmw_namespace_validation_result_string(validation_result),
            invalid_index);
  }
}

Node::~Node()
{}

const char *
Node::get_name() const
{
  return node_base_->get_name();
}

const char *
Node::get_namespace() const
{
  if (sub_namespace_ == "") {
    return node_base_->get_namespace();
  } else {
    std::string full_namespace(node_base_->get_namespace());

    if (full_namespace.back() == '/') {
      full_namespace += sub_namespace_;
    } else {
      full_namespace += "/" + sub_namespace_;
    }

    return full_namespace.c_str();
  }
}

rclcpp::Logger
Node::get_logger() const
{
  return node_logging_->get_logger();
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

std::vector<rcl_interfaces::msg::SetParametersResult>
Node::set_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  return node_parameters_->set_parameters(parameters);
}

rcl_interfaces::msg::SetParametersResult
Node::set_parameters_atomically(
  const std::vector<rclcpp::Parameter> & parameters)
{
  return node_parameters_->set_parameters_atomically(parameters);
}

std::vector<rclcpp::Parameter>
Node::get_parameters(
  const std::vector<std::string> & names) const
{
  return node_parameters_->get_parameters(names);
}

rclcpp::Parameter
Node::get_parameter(const std::string & name) const
{
  return node_parameters_->get_parameter(name);
}

bool Node::get_parameter(
  const std::string & name,
  rclcpp::Parameter & parameter) const
{
  return node_parameters_->get_parameter(name, parameter);
}

std::vector<rcl_interfaces::msg::ParameterDescriptor>
Node::describe_parameters(
  const std::vector<std::string> & names) const
{
  return node_parameters_->describe_parameters(names);
}

std::vector<uint8_t>
Node::get_parameter_types(
  const std::vector<std::string> & names) const
{
  return node_parameters_->get_parameter_types(names);
}

rcl_interfaces::msg::ListParametersResult
Node::list_parameters(
  const std::vector<std::string> & prefixes, uint64_t depth) const
{
  return node_parameters_->list_parameters(prefixes, depth);
}

std::vector<std::string>
Node::get_node_names() const
{
  return node_graph_->get_node_names();
}

std::map<std::string, std::vector<std::string>>
Node::get_topic_names_and_types() const
{
  return node_graph_->get_topic_names_and_types();
}

std::map<std::string, std::vector<std::string>>
Node::get_service_names_and_types() const
{
  return node_graph_->get_service_names_and_types();
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

const std::vector<rclcpp::callback_group::CallbackGroup::WeakPtr> &
Node::get_callback_groups() const
{
  return node_base_->get_callback_groups();
}

rclcpp::Event::SharedPtr
Node::get_graph_event()
{
  return node_graph_->get_graph_event();
}

void
Node::wait_for_graph_change(
  rclcpp::Event::SharedPtr event,
  std::chrono::nanoseconds timeout)
{
  node_graph_->wait_for_graph_change(event, timeout);
}

rclcpp::Clock::SharedPtr
Node::get_clock()
{
  return node_clock_->get_clock();
}

rclcpp::Time
Node::now()
{
  return node_clock_->get_clock()->now();
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
Node::get_node_base_interface()
{
  return node_base_;
}

rclcpp::node_interfaces::NodeClockInterface::SharedPtr
Node::get_node_clock_interface()
{
  return node_clock_;
}

rclcpp::node_interfaces::NodeGraphInterface::SharedPtr
Node::get_node_graph_interface()
{
  return node_graph_;
}

rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr
Node::get_node_logging_interface()
{
  return node_logging_;
}

rclcpp::node_interfaces::NodeTimersInterface::SharedPtr
Node::get_node_timers_interface()
{
  return node_timers_;
}

rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr
Node::get_node_topics_interface()
{
  return node_topics_;
}

rclcpp::node_interfaces::NodeServicesInterface::SharedPtr
Node::get_node_services_interface()
{
  return node_services_;
}

rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
Node::get_node_parameters_interface()
{
  return node_parameters_;
}

rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr
Node::get_node_waitables_interface()
{
  return node_waitables_;
}

Node::SharedPtr
Node::create_sub_node(const std::string & sub_namespace)
{
  auto new_node = std::make_shared<Node>(*this, sub_namespace);

  return new_node;
}
