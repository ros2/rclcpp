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

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "rcl_interfaces/msg/list_parameters_result.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "rclcpp/exceptions.hpp"
#include "rclcpp/graph_listener.hpp"
#include "rclcpp/logger.hpp"
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
#include "rclcpp/node_interfaces/node_type_descriptions.hpp"
#include "rclcpp/node_interfaces/node_waitables.hpp"
#include "rclcpp/parameter_service.hpp"
#include "rclcpp/qos.hpp"

#include "lifecycle_node_interface_impl.hpp"  // implementation

namespace rclcpp_lifecycle
{

LifecycleNode::LifecycleNode(
  const std::string & node_name,
  const rclcpp::NodeOptions & options,
  bool enable_communication_interface)
: LifecycleNode(
    node_name,
    "",
    options,
    enable_communication_interface)
{}

LifecycleNode::LifecycleNode(
  const std::string & node_name,
  const std::string & namespace_,
  const rclcpp::NodeOptions & options,
  bool enable_communication_interface)
: node_base_(new rclcpp::node_interfaces::NodeBase(
      node_name,
      namespace_,
      options.context(),
      *(options.get_rcl_node_options()),
      options.use_intra_process_comms(),
      options.enable_topic_statistics())),
  node_graph_(new rclcpp::node_interfaces::NodeGraph(node_base_.get())),
  node_logging_(new rclcpp::node_interfaces::NodeLogging(node_base_)),
  node_timers_(new rclcpp::node_interfaces::NodeTimers(node_base_.get())),
  node_topics_(new rclcpp::node_interfaces::NodeTopics(node_base_.get(), node_timers_.get())),
  node_services_(new rclcpp::node_interfaces::NodeServices(node_base_.get())),
  node_clock_(new rclcpp::node_interfaces::NodeClock(
      node_base_,
      node_topics_,
      node_graph_,
      node_services_,
      node_logging_,
      options.clock_type()
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
      options.parameter_event_qos(),
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
  impl_(new LifecycleNodeInterfaceImpl(
      node_base_,
      node_logging_,
      node_parameters_,
      node_services_))
{
  impl_->init(enable_communication_interface);

  register_on_configure(
    std::bind(
      &LifecycleNodeInterface::on_configure, this,
      std::placeholders::_1));
  register_on_cleanup(std::bind(&LifecycleNodeInterface::on_cleanup, this, std::placeholders::_1));
  register_on_shutdown(
    std::bind(
      &LifecycleNodeInterface::on_shutdown, this,
      std::placeholders::_1));
  register_on_activate(
    std::bind(
      &LifecycleNodeInterface::on_activate, this,
      std::placeholders::_1));
  register_on_deactivate(
    std::bind(
      &LifecycleNodeInterface::on_deactivate, this,
      std::placeholders::_1));
  register_on_error(std::bind(&LifecycleNodeInterface::on_error, this, std::placeholders::_1));

  if (options.enable_logger_service()) {
    node_logging_->create_logger_services(node_services_);
  }
}

LifecycleNode::~LifecycleNode()
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
LifecycleNode::get_name() const
{
  return node_base_->get_name();
}

const char *
LifecycleNode::get_namespace() const
{
  return node_base_->get_namespace();
}

const char *
LifecycleNode::get_fully_qualified_name() const
{
  return node_base_->get_fully_qualified_name();
}

rclcpp::Logger
LifecycleNode::get_logger() const
{
  return node_logging_->get_logger();
}

rclcpp::CallbackGroup::SharedPtr
LifecycleNode::create_callback_group(
  rclcpp::CallbackGroupType group_type,
  bool automatically_add_to_executor_with_node)
{
  return node_base_->create_callback_group(group_type, automatically_add_to_executor_with_node);
}

const rclcpp::ParameterValue &
LifecycleNode::declare_parameter(
  const std::string & name,
  const rclcpp::ParameterValue & default_value,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor,
  bool ignore_override)
{
  return this->node_parameters_->declare_parameter(
    name, default_value, parameter_descriptor, ignore_override);
}

const rclcpp::ParameterValue &
LifecycleNode::declare_parameter(
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
LifecycleNode::undeclare_parameter(const std::string & name)
{
  this->node_parameters_->undeclare_parameter(name);
}

bool
LifecycleNode::has_parameter(const std::string & name) const
{
  return this->node_parameters_->has_parameter(name);
}

rcl_interfaces::msg::SetParametersResult
LifecycleNode::set_parameter(const rclcpp::Parameter & parameter)
{
  return this->set_parameters_atomically({parameter});
}

std::vector<rcl_interfaces::msg::SetParametersResult>
LifecycleNode::set_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  return node_parameters_->set_parameters(parameters);
}

rcl_interfaces::msg::SetParametersResult
LifecycleNode::set_parameters_atomically(
  const std::vector<rclcpp::Parameter> & parameters)
{
  return node_parameters_->set_parameters_atomically(parameters);
}

std::vector<rclcpp::Parameter>
LifecycleNode::get_parameters(
  const std::vector<std::string> & names) const
{
  return node_parameters_->get_parameters(names);
}

rclcpp::Parameter
LifecycleNode::get_parameter(const std::string & name) const
{
  return node_parameters_->get_parameter(name);
}

bool
LifecycleNode::get_parameter(
  const std::string & name,
  rclcpp::Parameter & parameter) const
{
  return node_parameters_->get_parameter(name, parameter);
}

rcl_interfaces::msg::ParameterDescriptor
LifecycleNode::describe_parameter(const std::string & name) const
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
LifecycleNode::describe_parameters(
  const std::vector<std::string> & names) const
{
  return node_parameters_->describe_parameters(names);
}

std::vector<uint8_t>
LifecycleNode::get_parameter_types(
  const std::vector<std::string> & names) const
{
  return node_parameters_->get_parameter_types(names);
}

rcl_interfaces::msg::ListParametersResult
LifecycleNode::list_parameters(
  const std::vector<std::string> & prefixes, uint64_t depth) const
{
  return node_parameters_->list_parameters(prefixes, depth);
}

rclcpp::Node::PreSetParametersCallbackHandle::SharedPtr
LifecycleNode::add_pre_set_parameters_callback(PreSetParametersCallbackType callback)
{
  return node_parameters_->add_pre_set_parameters_callback(callback);
}

rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr
LifecycleNode::add_on_set_parameters_callback(OnSetParametersCallbackType callback)
{
  return node_parameters_->add_on_set_parameters_callback(callback);
}

rclcpp::Node::PostSetParametersCallbackHandle::SharedPtr
LifecycleNode::add_post_set_parameters_callback(PostSetParametersCallbackType callback)
{
  return node_parameters_->add_post_set_parameters_callback(callback);
}

void
LifecycleNode::remove_pre_set_parameters_callback(
  const PreSetParametersCallbackHandle * const callback)
{
  node_parameters_->remove_pre_set_parameters_callback(callback);
}

void
LifecycleNode::remove_on_set_parameters_callback(
  const OnSetParametersCallbackHandle * const callback)
{
  node_parameters_->remove_on_set_parameters_callback(callback);
}

void
LifecycleNode::remove_post_set_parameters_callback(
  const PostSetParametersCallbackHandle * const callback)
{
  node_parameters_->remove_post_set_parameters_callback(callback);
}

std::vector<std::string>
LifecycleNode::get_node_names() const
{
  return node_graph_->get_node_names();
}

std::map<std::string, std::vector<std::string>>
LifecycleNode::get_topic_names_and_types(bool no_demangle) const
{
  return node_graph_->get_topic_names_and_types(no_demangle);
}

std::map<std::string, std::vector<std::string>>
LifecycleNode::get_service_names_and_types() const
{
  return node_graph_->get_service_names_and_types();
}

std::map<std::string, std::vector<std::string>>
LifecycleNode::get_service_names_and_types_by_node(
  const std::string & node_name,
  const std::string & namespace_) const
{
  return node_graph_->get_service_names_and_types_by_node(
    node_name, namespace_);
}

size_t
LifecycleNode::count_publishers(const std::string & topic_name) const
{
  return node_graph_->count_publishers(topic_name);
}

size_t
LifecycleNode::count_subscribers(const std::string & topic_name) const
{
  return node_graph_->count_subscribers(topic_name);
}

std::vector<rclcpp::TopicEndpointInfo>
LifecycleNode::get_publishers_info_by_topic(const std::string & topic_name, bool no_mangle) const
{
  return node_graph_->get_publishers_info_by_topic(topic_name, no_mangle);
}

std::vector<rclcpp::TopicEndpointInfo>
LifecycleNode::get_subscriptions_info_by_topic(const std::string & topic_name, bool no_mangle) const
{
  return node_graph_->get_subscriptions_info_by_topic(topic_name, no_mangle);
}

void
LifecycleNode::for_each_callback_group(
  const rclcpp::node_interfaces::NodeBaseInterface::CallbackGroupFunction & func)
{
  node_base_->for_each_callback_group(func);
}

rclcpp::Event::SharedPtr
LifecycleNode::get_graph_event()
{
  return node_graph_->get_graph_event();
}

void
LifecycleNode::wait_for_graph_change(
  rclcpp::Event::SharedPtr event,
  std::chrono::nanoseconds timeout)
{
  node_graph_->wait_for_graph_change(event, timeout);
}

rclcpp::Clock::SharedPtr
LifecycleNode::get_clock()
{
  return node_clock_->get_clock();
}

rclcpp::Clock::ConstSharedPtr
LifecycleNode::get_clock() const
{
  return node_clock_->get_clock();
}

rclcpp::Time
LifecycleNode::now() const
{
  return node_clock_->get_clock()->now();
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
LifecycleNode::get_node_base_interface()
{
  return node_base_;
}

rclcpp::node_interfaces::NodeClockInterface::SharedPtr
LifecycleNode::get_node_clock_interface()
{
  return node_clock_;
}

rclcpp::node_interfaces::NodeGraphInterface::SharedPtr
LifecycleNode::get_node_graph_interface()
{
  return node_graph_;
}

rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr
LifecycleNode::get_node_logging_interface()
{
  return node_logging_;
}

rclcpp::node_interfaces::NodeTimeSourceInterface::SharedPtr
LifecycleNode::get_node_time_source_interface()
{
  return node_time_source_;
}

rclcpp::node_interfaces::NodeTimersInterface::SharedPtr
LifecycleNode::get_node_timers_interface()
{
  return node_timers_;
}

rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr
LifecycleNode::get_node_topics_interface()
{
  return node_topics_;
}

rclcpp::node_interfaces::NodeServicesInterface::SharedPtr
LifecycleNode::get_node_services_interface()
{
  return node_services_;
}

rclcpp::node_interfaces::NodeTypeDescriptionsInterface::SharedPtr
LifecycleNode::get_node_type_descriptions_interface()
{
  return impl_->get_node_type_descriptions_interface();
}

rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
LifecycleNode::get_node_parameters_interface()
{
  return node_parameters_;
}

rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr
LifecycleNode::get_node_waitables_interface()
{
  return node_waitables_;
}

const rclcpp::NodeOptions &
LifecycleNode::get_node_options() const
{
  return node_options_;
}

////
bool
LifecycleNode::register_on_configure(
  std::function<LifecycleNodeInterface::CallbackReturn(const State &)> fcn)
{
  return impl_->register_callback(
    lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING, fcn);
}

bool
LifecycleNode::register_on_cleanup(
  std::function<LifecycleNodeInterface::CallbackReturn(const State &)> fcn)
{
  return impl_->register_callback(
    lifecycle_msgs::msg::State::TRANSITION_STATE_CLEANINGUP, fcn);
}

bool
LifecycleNode::register_on_shutdown(
  std::function<LifecycleNodeInterface::CallbackReturn(const State &)> fcn)
{
  return impl_->register_callback(
    lifecycle_msgs::msg::State::TRANSITION_STATE_SHUTTINGDOWN, fcn);
}

bool
LifecycleNode::register_on_activate(
  std::function<LifecycleNodeInterface::CallbackReturn(const State &)> fcn)
{
  return impl_->register_callback(
    lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING, fcn);
}

bool
LifecycleNode::register_on_deactivate(
  std::function<LifecycleNodeInterface::CallbackReturn(const State &)> fcn)
{
  return impl_->register_callback(
    lifecycle_msgs::msg::State::TRANSITION_STATE_DEACTIVATING, fcn);
}

bool
LifecycleNode::register_on_error(
  std::function<LifecycleNodeInterface::CallbackReturn(const State &)> fcn)
{
  return impl_->register_callback(
    lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING, fcn);
}

const State &
LifecycleNode::get_current_state() const
{
  return impl_->get_current_state();
}

std::vector<State>
LifecycleNode::get_available_states() const
{
  return impl_->get_available_states();
}

std::vector<Transition>
LifecycleNode::get_available_transitions() const
{
  return impl_->get_available_transitions();
}

std::vector<Transition>
LifecycleNode::get_transition_graph() const
{
  return impl_->get_transition_graph();
}

const State &
LifecycleNode::trigger_transition(const Transition & transition)
{
  return trigger_transition(transition.id());
}

const State &
LifecycleNode::trigger_transition(
  const Transition & transition, LifecycleNodeInterface::CallbackReturn & cb_return_code)
{
  return trigger_transition(transition.id(), cb_return_code);
}

const State &
LifecycleNode::trigger_transition(uint8_t transition_id)
{
  return impl_->trigger_transition(transition_id);
}

const State &
LifecycleNode::trigger_transition(
  uint8_t transition_id, LifecycleNodeInterface::CallbackReturn & cb_return_code)
{
  return impl_->trigger_transition(transition_id, cb_return_code);
}

const State &
LifecycleNode::configure()
{
  return impl_->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
}

const State &
LifecycleNode::configure(LifecycleNodeInterface::CallbackReturn & cb_return_code)
{
  return impl_->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, cb_return_code);
}

const State &
LifecycleNode::cleanup()
{
  return impl_->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
}

const State &
LifecycleNode::cleanup(LifecycleNodeInterface::CallbackReturn & cb_return_code)
{
  return impl_->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, cb_return_code);
}

const State &
LifecycleNode::activate()
{
  return impl_->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
}

const State &
LifecycleNode::activate(LifecycleNodeInterface::CallbackReturn & cb_return_code)
{
  return impl_->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, cb_return_code);
}

const State &
LifecycleNode::deactivate()
{
  return impl_->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
}

const State &
LifecycleNode::deactivate(LifecycleNodeInterface::CallbackReturn & cb_return_code)
{
  return impl_->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, cb_return_code);
}

const State &
LifecycleNode::shutdown()
{
  return impl_->trigger_transition(
    rcl_lifecycle_shutdown_label);
}

const State &
LifecycleNode::shutdown(LifecycleNodeInterface::CallbackReturn & cb_return_code)
{
  return impl_->trigger_transition(
    rcl_lifecycle_shutdown_label, cb_return_code);
}

node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleNode::on_activate(const State &)
{
  impl_->on_activate();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleNode::on_deactivate(const State &)
{
  impl_->on_deactivate();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void
LifecycleNode::add_managed_entity(
  std::weak_ptr<rclcpp_lifecycle::ManagedEntityInterface> managed_entity)
{
  impl_->add_managed_entity(managed_entity);
}

void
LifecycleNode::add_timer_handle(std::shared_ptr<rclcpp::TimerBase> timer)
{
  impl_->add_timer_handle(timer);
}

}  // namespace rclcpp_lifecycle
