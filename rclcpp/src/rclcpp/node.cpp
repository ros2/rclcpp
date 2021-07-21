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
#include <array>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rcl/arguments.h"

#include "rclcpp/detail/qos_parameters.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/graph_listener.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/node_base.hpp"
#include "rclcpp/node_interfaces/node_clock.hpp"
#include "rclcpp/node_interfaces/node_graph.hpp"
#include "rclcpp/node_interfaces/node_logging.hpp"
#include "rclcpp/node_interfaces/node_parameters.hpp"
#include "rclcpp/node_interfaces/node_services.hpp"
#include "rclcpp/node_interfaces/node_time_source.hpp"
#include "rclcpp/node_interfaces/node_timers.hpp"
#include "rclcpp/node_interfaces/node_topics.hpp"
#include "rclcpp/node_interfaces/node_waitables.hpp"
#include "rclcpp/qos_overriding_options.hpp"

#include "rmw/validate_namespace.h"

#include "./detail/resolve_parameter_overrides.hpp"

using rclcpp::Node;
using rclcpp::NodeOptions;
using rclcpp::exceptions::throw_from_rcl_error;

RCLCPP_LOCAL
std::string
extend_sub_namespace(const std::string & existing_sub_namespace, const std::string & extension)
{
  // Assumption is that the existing_sub_namespace does not need checking
  // because it would be checked already when it was set with this function.

  // check if the new sub-namespace extension is absolute
  if (extension.front() == '/') {
    throw rclcpp::exceptions::NameValidationError(
            "sub_namespace",
            extension.c_str(),
            "a sub-namespace should not have a leading /",
            0);
  }

  std::string new_sub_namespace;
  if (existing_sub_namespace.empty()) {
    new_sub_namespace = extension;
  } else {
    new_sub_namespace = existing_sub_namespace + "/" + extension;
  }

  // remove any trailing `/` so that new extensions do no result in `//`
  if (new_sub_namespace.back() == '/') {
    new_sub_namespace = new_sub_namespace.substr(0, new_sub_namespace.size() - 1);
  }

  return new_sub_namespace;
}

RCLCPP_LOCAL
std::string
create_effective_namespace(const std::string & node_namespace, const std::string & sub_namespace)
{
  // Assumption is that both the node_namespace and sub_namespace are conforming
  // and do not need trimming of `/` and other things, as they were validated
  // in other functions already.

  if (node_namespace.back() == '/') {
    // this is the special case where node_namespace is just `/`
    return node_namespace + sub_namespace;
  } else {
    return node_namespace + "/" + sub_namespace;
  }
}

Node::Node(
  const std::string & node_name,
  const NodeOptions & options)
: Node(node_name, "", options)
{
}

static
rclcpp::QoS
get_parameter_events_qos(
  rclcpp::node_interfaces::NodeBaseInterface & node_base,
  const rclcpp::NodeOptions & options)
{
  auto final_qos = options.parameter_event_qos();
  const rcl_arguments_t * global_args = nullptr;
  auto * rcl_options = options.get_rcl_node_options();
  if (rcl_options->use_global_arguments) {
    auto context_ptr = node_base.get_context()->get_rcl_context();
    global_args = &(context_ptr->global_arguments);
  }

  auto parameter_overrides = rclcpp::detail::resolve_parameter_overrides(
    node_base.get_fully_qualified_name(),
    options.parameter_overrides(),
    &rcl_options->arguments,
    global_args);

  auto final_topic_name = node_base.resolve_topic_or_service_name("/parameter_events", false);
  auto prefix = "qos_overrides." + final_topic_name + ".";
  std::array<rclcpp::QosPolicyKind, 4> policies = {
    rclcpp::QosPolicyKind::Depth,
    rclcpp::QosPolicyKind::Durability,
    rclcpp::QosPolicyKind::History,
    rclcpp::QosPolicyKind::Reliability,
  };
  for (const auto & policy : policies) {
    auto param_name = prefix + rclcpp::qos_policy_kind_to_cstr(policy);
    auto it = parameter_overrides.find(param_name);
    auto value = it != parameter_overrides.end() ?
      it->second.value :
      rclcpp::detail::get_default_qos_param_value(policy, options.parameter_event_qos());
    rclcpp::detail::apply_qos_override(policy, value, final_qos);
  }
  return final_qos;
}

Node::Node(
  const std::string & node_name,
  const std::string & namespace_,
  const NodeOptions & options)
: node_base_(new rclcpp::node_interfaces::NodeBase(
      node_name,
      namespace_,
      options.context(),
      *(options.get_rcl_node_options()),
      options.use_intra_process_comms(),
      options.enable_topic_statistics())),
  node_graph_(new rclcpp::node_interfaces::NodeGraph(node_base_.get())),
  node_logging_(new rclcpp::node_interfaces::NodeLogging(node_base_.get())),
  node_timers_(new rclcpp::node_interfaces::NodeTimers(node_base_.get())),
  node_topics_(new rclcpp::node_interfaces::NodeTopics(node_base_.get(), node_timers_.get())),
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
      node_logging_,
      node_topics_,
      node_services_,
      node_clock_,
      options.parameter_overrides(),
      options.start_parameter_services(),
      options.start_parameter_event_publisher(),
      // This is needed in order to apply parameter overrides to the qos profile provided in
      // options.
      get_parameter_events_qos(*node_base_, options),
      options.parameter_event_publisher_options(),
      options.allow_undeclared_parameters(),
      options.automatically_declare_parameters_from_overrides()
    )),
  node_time_source_(new rclcpp::node_interfaces::NodeTimeSource(
      node_base_,
      node_topics_,
      node_graph_,
      node_services_,
      node_logging_,
      node_clock_,
      node_parameters_,
      options.clock_qos(),
      options.use_clock_thread()
    )),
  node_waitables_(new rclcpp::node_interfaces::NodeWaitables(node_base_.get())),
  node_options_(options),
  sub_namespace_(""),
  effective_namespace_(create_effective_namespace(this->get_namespace(), sub_namespace_))
{
  // we have got what we wanted directly from the overrides,
  // but declare the parameters anyway so they are visible.
  rclcpp::detail::declare_qos_parameters(
    rclcpp::QosOverridingOptions
  {
    QosPolicyKind::Depth,
    QosPolicyKind::Durability,
    QosPolicyKind::History,
    QosPolicyKind::Reliability,
  },
    node_parameters_,
    node_topics_->resolve_topic_name("/parameter_events"),
    options.parameter_event_qos(),
    rclcpp::detail::PublisherQosParametersTraits{});
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
  node_options_(other.node_options_),
  sub_namespace_(extend_sub_namespace(other.get_sub_namespace(), sub_namespace)),
  effective_namespace_(create_effective_namespace(other.get_namespace(), sub_namespace_))
{
  // Validate new effective namespace.
  int validation_result;
  size_t invalid_index;
  rmw_ret_t rmw_ret =
    rmw_validate_namespace(effective_namespace_.c_str(), &validation_result, &invalid_index);

  if (rmw_ret != RMW_RET_OK) {
    if (rmw_ret == RMW_RET_INVALID_ARGUMENT) {
      throw_from_rcl_error(RCL_RET_INVALID_ARGUMENT, "failed to validate subnode namespace");
    }
    throw_from_rcl_error(RCL_RET_ERROR, "failed to validate subnode namespace");
  }

  if (validation_result != RMW_NAMESPACE_VALID) {
    throw rclcpp::exceptions::InvalidNamespaceError(
            effective_namespace_.c_str(),
            rmw_namespace_validation_result_string(validation_result),
            invalid_index);
  }
}

Node::~Node()
{
  // release sub-interfaces in an order that allows them to consult with node_base during tear-down
  node_waitables_.reset();
  node_time_source_.reset();
  node_parameters_.reset();
  node_clock_.reset();
  node_services_.reset();
  node_topics_.reset();
  node_timers_.reset();
  node_logging_.reset();
  node_graph_.reset();
}

const char *
Node::get_name() const
{
  return node_base_->get_name();
}

const char *
Node::get_namespace() const
{
  return node_base_->get_namespace();
}

const char *
Node::get_fully_qualified_name() const
{
  return node_base_->get_fully_qualified_name();
}

rclcpp::Logger
Node::get_logger() const
{
  return node_logging_->get_logger();
}

rclcpp::CallbackGroup::SharedPtr
Node::create_callback_group(
  rclcpp::CallbackGroupType group_type,
  bool automatically_add_to_executor_with_node)
{
  return node_base_->create_callback_group(group_type, automatically_add_to_executor_with_node);
}

const rclcpp::ParameterValue &
Node::declare_parameter(const std::string & name)
{
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#else
# pragma warning(push)
# pragma warning(disable: 4996)
#endif
  return this->node_parameters_->declare_parameter(name);
#ifndef _WIN32
# pragma GCC diagnostic pop
#else
# pragma warning(pop)
#endif
}

const rclcpp::ParameterValue &
Node::declare_parameter(
  const std::string & name,
  const rclcpp::ParameterValue & default_value,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor,
  bool ignore_override)
{
  return this->node_parameters_->declare_parameter(
    name,
    default_value,
    parameter_descriptor,
    ignore_override);
}

const rclcpp::ParameterValue &
Node::declare_parameter(
  const std::string & name,
  rclcpp::ParameterType type,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor,
  bool ignore_override)
{
  return this->node_parameters_->declare_parameter(
    name,
    type,
    parameter_descriptor,
    ignore_override);
}

void
Node::undeclare_parameter(const std::string & name)
{
  this->node_parameters_->undeclare_parameter(name);
}

bool
Node::has_parameter(const std::string & name) const
{
  return this->node_parameters_->has_parameter(name);
}

rcl_interfaces::msg::SetParametersResult
Node::set_parameter(const rclcpp::Parameter & parameter)
{
  return this->set_parameters_atomically({parameter});
}

std::vector<rcl_interfaces::msg::SetParametersResult>
Node::set_parameters(const std::vector<rclcpp::Parameter> & parameters)
{
  return node_parameters_->set_parameters(parameters);
}

rcl_interfaces::msg::SetParametersResult
Node::set_parameters_atomically(const std::vector<rclcpp::Parameter> & parameters)
{
  return node_parameters_->set_parameters_atomically(parameters);
}

rclcpp::Parameter
Node::get_parameter(const std::string & name) const
{
  return node_parameters_->get_parameter(name);
}

bool
Node::get_parameter(const std::string & name, rclcpp::Parameter & parameter) const
{
  return node_parameters_->get_parameter(name, parameter);
}

std::vector<rclcpp::Parameter>
Node::get_parameters(
  const std::vector<std::string> & names) const
{
  return node_parameters_->get_parameters(names);
}

rcl_interfaces::msg::ParameterDescriptor
Node::describe_parameter(const std::string & name) const
{
  auto result = node_parameters_->describe_parameters({name});
  if (0 == result.size()) {
    throw rclcpp::exceptions::ParameterNotDeclaredException(name);
  }
  if (result.size() > 1) {
    throw std::runtime_error("number of described parameters unexpectedly more than one");
  }
  return result.front();
}

std::vector<rcl_interfaces::msg::ParameterDescriptor>
Node::describe_parameters(const std::vector<std::string> & names) const
{
  return node_parameters_->describe_parameters(names);
}

std::vector<uint8_t>
Node::get_parameter_types(const std::vector<std::string> & names) const
{
  return node_parameters_->get_parameter_types(names);
}

rcl_interfaces::msg::ListParametersResult
Node::list_parameters(const std::vector<std::string> & prefixes, uint64_t depth) const
{
  return node_parameters_->list_parameters(prefixes, depth);
}

rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr
Node::add_on_set_parameters_callback(OnParametersSetCallbackType callback)
{
  return node_parameters_->add_on_set_parameters_callback(callback);
}

void
Node::remove_on_set_parameters_callback(const OnSetParametersCallbackHandle * const callback)
{
  return node_parameters_->remove_on_set_parameters_callback(callback);
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

std::map<std::string, std::vector<std::string>>
Node::get_service_names_and_types_by_node(
  const std::string & node_name,
  const std::string & namespace_) const
{
  return node_graph_->get_service_names_and_types_by_node(
    node_name, namespace_);
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

std::vector<rclcpp::TopicEndpointInfo>
Node::get_publishers_info_by_topic(const std::string & topic_name, bool no_mangle) const
{
  return node_graph_->get_publishers_info_by_topic(topic_name, no_mangle);
}

std::vector<rclcpp::TopicEndpointInfo>
Node::get_subscriptions_info_by_topic(const std::string & topic_name, bool no_mangle) const
{
  return node_graph_->get_subscriptions_info_by_topic(topic_name, no_mangle);
}

const std::vector<rclcpp::CallbackGroup::WeakPtr> &
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

rclcpp::Clock::ConstSharedPtr
Node::get_clock() const
{
  return node_clock_->get_clock();
}

rclcpp::Time
Node::now() const
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

rclcpp::node_interfaces::NodeTimeSourceInterface::SharedPtr
Node::get_node_time_source_interface()
{
  return node_time_source_;
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

const std::string &
Node::get_sub_namespace() const
{
  return this->sub_namespace_;
}

const std::string &
Node::get_effective_namespace() const
{
  return this->effective_namespace_;
}

Node::SharedPtr
Node::create_sub_node(const std::string & sub_namespace)
{
  // Cannot use make_shared<Node>() here as it requires the constructor to be
  // public, and this constructor is intentionally protected instead.
  return std::shared_ptr<Node>(new Node(*this, sub_namespace));
}

const NodeOptions &
Node::get_node_options() const
{
  return this->node_options_;
}
