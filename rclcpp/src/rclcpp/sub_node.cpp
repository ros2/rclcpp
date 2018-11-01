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
#include "rmw/validate_namespace.h"

#include "rclcpp/sub_node.hpp"


using rclcpp::SubNode;
using rclcpp::exceptions::throw_from_rcl_error;

SubNode::SubNode(
  rclcpp::Node::SharedPtr original,
  const std::string & namespace_)
: original_(original),
  node_base_(original->node_base_),
  node_graph_(original->node_graph_),
  node_logging_(original->node_logging_),
  node_timers_(original->node_timers_),
  node_topics_(original->node_topics_),
  node_services_(original->node_services_),
  node_parameters_(original->node_parameters_),
  node_clock_(original->node_clock_),
  use_intra_process_comms_(original->use_intra_process_comms_)
{
  extended_namespace_ = validated_extended_ns(namespace_);
}

SubNode::SubNode(
  const SubNode::SharedPtr other,
  const std::string & namespace_)
: original_(other->original_),
  node_base_(other->original_->node_base_),
  node_graph_(other->original_->node_graph_),
  node_logging_(other->original_->node_logging_),
  node_timers_(other->original_->node_timers_),
  node_topics_(other->original_->node_topics_),
  node_services_(other->original_->node_services_),
  node_parameters_(other->original_->node_parameters_),
  node_clock_(other->original_->node_clock_),
  use_intra_process_comms_(other->original_->use_intra_process_comms_)
{
  extended_namespace_ = validated_extended_ns(
    other->extended_namespace_+"/"+namespace_);
}

SubNode::~SubNode()
{}

const char *
SubNode::get_name() const
{
  return node_base_->get_name();
}

const char *
SubNode::get_namespace() const
{
  return node_base_->get_namespace();
}

rclcpp::Logger
SubNode::get_logger() const
{
  return node_logging_->get_logger();
}

rclcpp::callback_group::CallbackGroup::SharedPtr
SubNode::create_callback_group(
  rclcpp::callback_group::CallbackGroupType group_type)
{
  return node_base_->create_callback_group(group_type);
}

bool
SubNode::group_in_node(rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  return node_base_->callback_group_in_node(group);
}

std::vector<rcl_interfaces::msg::SetParametersResult>
SubNode::set_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  return node_parameters_->set_parameters(parameters);
}

rcl_interfaces::msg::SetParametersResult
SubNode::set_parameters_atomically(
  const std::vector<rclcpp::Parameter> & parameters)
{
  return node_parameters_->set_parameters_atomically(parameters);
}

std::vector<rclcpp::Parameter>
SubNode::get_parameters(
  const std::vector<std::string> & names) const
{
  return node_parameters_->get_parameters(names);
}

rclcpp::Parameter
SubNode::get_parameter(const std::string & name) const
{
  return node_parameters_->get_parameter(name);
}

bool SubNode::get_parameter(
  const std::string & name,
  rclcpp::Parameter & parameter) const
{
  return node_parameters_->get_parameter(name, parameter);
}

std::vector<rcl_interfaces::msg::ParameterDescriptor>
SubNode::describe_parameters(
  const std::vector<std::string> & names) const
{
  return node_parameters_->describe_parameters(names);
}

std::vector<uint8_t>
SubNode::get_parameter_types(
  const std::vector<std::string> & names) const
{
  return node_parameters_->get_parameter_types(names);
}

rcl_interfaces::msg::ListParametersResult
SubNode::list_parameters(
  const std::vector<std::string> & prefixes, uint64_t depth) const
{
  return node_parameters_->list_parameters(prefixes, depth);
}

std::vector<std::string>
SubNode::get_node_names() const
{
  return node_graph_->get_node_names();
}

std::map<std::string, std::vector<std::string>>
SubNode::get_topic_names_and_types() const
{
  return node_graph_->get_topic_names_and_types();
}

std::map<std::string, std::vector<std::string>>
SubNode::get_service_names_and_types() const
{
  return node_graph_->get_service_names_and_types();
}

size_t
SubNode::count_publishers(const std::string & topic_name) const
{
  return node_graph_->count_publishers(topic_name);
}

size_t
SubNode::count_subscribers(const std::string & topic_name) const
{
  return node_graph_->count_subscribers(topic_name);
}

const std::vector<rclcpp::callback_group::CallbackGroup::WeakPtr> &
SubNode::get_callback_groups() const
{
  return node_base_->get_callback_groups();
}

rclcpp::Event::SharedPtr
SubNode::get_graph_event()
{
  return node_graph_->get_graph_event();
}

void
SubNode::wait_for_graph_change(
  rclcpp::Event::SharedPtr event,
  std::chrono::nanoseconds timeout)
{
  node_graph_->wait_for_graph_change(event, timeout);
}

rclcpp::Clock::SharedPtr
SubNode::get_clock()
{
  return node_clock_->get_clock();
}

rclcpp::Time
SubNode::now()
{
  return node_clock_->get_clock()->now();
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
SubNode::get_node_base_interface()
{
  return node_base_;
}

rclcpp::node_interfaces::NodeClockInterface::SharedPtr
SubNode::get_node_clock_interface()
{
  return node_clock_;
}

rclcpp::node_interfaces::NodeGraphInterface::SharedPtr
SubNode::get_node_graph_interface()
{
  return node_graph_;
}

rclcpp::node_interfaces::NodeTimersInterface::SharedPtr
SubNode::get_node_timers_interface()
{
  return node_timers_;
}

rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr
SubNode::get_node_topics_interface()
{
  return node_topics_;
}

rclcpp::node_interfaces::NodeServicesInterface::SharedPtr
SubNode::get_node_services_interface()
{
  return node_services_;
}

rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
SubNode::get_node_parameters_interface()
{
  return node_parameters_;
}

const std::string
SubNode::validated_extended_ns(const std::string & extended_ns) const
{
  std::string full_namespace = std::string(original_->get_namespace())+"/"+extended_ns;

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

  return extended_ns;
}

RCLCPP_PUBLIC
const std::string
SubNode::add_extended_ns_prefix(const std::string & original_name) const
{
  return extended_namespace_+"/"+original_name;
}


std::shared_ptr<rclcpp::SubNode>
SubNode::create_sub_node(const std::string & namespace_)
{
  return rclcpp::SubNode::SharedPtr(
    new rclcpp::SubNode(shared_from_this(), namespace_));
}
