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

#include <string>
#include <map>
#include <memory>
#include <vector>
#include <utility>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/exceptions.hpp"
#include "rclcpp/graph_listener.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/node_base.hpp"
#include "rclcpp/node_interfaces/node_graph.hpp"
#include "rclcpp/node_interfaces/node_parameters.hpp"
#include "rclcpp/node_interfaces/node_services.hpp"
#include "rclcpp/node_interfaces/node_timers.hpp"
#include "rclcpp/node_interfaces/node_topics.hpp"

#include "lifecycle_node_interface_impl.hpp"  // implementation

namespace rclcpp_lifecycle
{

LifecycleNode::LifecycleNode(
  const std::string & node_name,
  const std::string & namespace_,
  bool use_intra_process_comms)
: LifecycleNode(
    node_name,
    namespace_,
    rclcpp::contexts::default_context::get_global_default_context(),
    use_intra_process_comms)
{}

LifecycleNode::LifecycleNode(
  const std::string & node_name,
  const std::string & namespace_,
  rclcpp::context::Context::SharedPtr context,
  bool use_intra_process_comms)
: node_base_(new rclcpp::node_interfaces::NodeBase(node_name, namespace_, context)),
  node_graph_(new rclcpp::node_interfaces::NodeGraph(node_base_.get())),
  node_timers_(new rclcpp::node_interfaces::NodeTimers(node_base_.get())),
  node_topics_(new rclcpp::node_interfaces::NodeTopics(node_base_.get())),
  node_services_(new rclcpp::node_interfaces::NodeServices(node_base_.get())),
  node_parameters_(new rclcpp::node_interfaces::NodeParameters(
      node_topics_.get(),
      use_intra_process_comms
    )),
  use_intra_process_comms_(use_intra_process_comms),
  impl_(new LifecycleNodeInterfaceImpl(node_base_, node_services_))
{
  impl_->init();

  register_on_configure(std::bind(&LifecycleNodeInterface::on_configure, this,
    std::placeholders::_1));
  register_on_cleanup(std::bind(&LifecycleNodeInterface::on_cleanup, this, std::placeholders::_1));
  register_on_shutdown(std::bind(&LifecycleNodeInterface::on_shutdown, this,
    std::placeholders::_1));
  register_on_activate(std::bind(&LifecycleNodeInterface::on_activate, this,
    std::placeholders::_1));
  register_on_deactivate(std::bind(&LifecycleNodeInterface::on_deactivate, this,
    std::placeholders::_1));
  register_on_error(std::bind(&LifecycleNodeInterface::on_error, this, std::placeholders::_1));
}

LifecycleNode::~LifecycleNode()
{}

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

rclcpp::callback_group::CallbackGroup::SharedPtr
LifecycleNode::create_callback_group(
  rclcpp::callback_group::CallbackGroupType group_type)
{
  return node_base_->create_callback_group(group_type);
}

bool
LifecycleNode::group_in_node(rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  return node_base_->callback_group_in_node(group);
}

std::vector<rcl_interfaces::msg::SetParametersResult>
LifecycleNode::set_parameters(
  const std::vector<rclcpp::parameter::ParameterVariant> & parameters)
{
  return node_parameters_->set_parameters(parameters);
}

rcl_interfaces::msg::SetParametersResult
LifecycleNode::set_parameters_atomically(
  const std::vector<rclcpp::parameter::ParameterVariant> & parameters)
{
  return node_parameters_->set_parameters_atomically(parameters);
}

std::vector<rclcpp::parameter::ParameterVariant>
LifecycleNode::get_parameters(
  const std::vector<std::string> & names) const
{
  return node_parameters_->get_parameters(names);
}

rclcpp::parameter::ParameterVariant
LifecycleNode::get_parameter(const std::string & name) const
{
  return node_parameters_->get_parameter(name);
}

bool LifecycleNode::get_parameter(const std::string & name,
  rclcpp::parameter::ParameterVariant & parameter) const
{
  return node_parameters_->get_parameter(name, parameter);
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

const std::vector<rclcpp::callback_group::CallbackGroup::WeakPtr> &
LifecycleNode::get_callback_groups() const
{
  return node_base_->get_callback_groups();
}

rclcpp::event::Event::SharedPtr
LifecycleNode::get_graph_event()
{
  return node_graph_->get_graph_event();
}

void
LifecycleNode::wait_for_graph_change(
  rclcpp::event::Event::SharedPtr event,
  std::chrono::nanoseconds timeout)
{
  node_graph_->wait_for_graph_change(event, timeout);
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
LifecycleNode::get_node_base_interface()
{
  return node_base_;
}

rclcpp::node_interfaces::NodeGraphInterface::SharedPtr
LifecycleNode::get_node_graph_interface()
{
  return node_graph_;
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

rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
LifecycleNode::get_node_parameters_interface()
{
  return node_parameters_;
}


////
bool
LifecycleNode::register_on_configure(std::function<rcl_lifecycle_ret_t(const State &)> fcn)
{
  return impl_->register_callback(lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING, fcn);
}

bool
LifecycleNode::register_on_cleanup(std::function<rcl_lifecycle_ret_t(const State &)> fcn)
{
  return impl_->register_callback(lifecycle_msgs::msg::State::TRANSITION_STATE_CLEANINGUP, fcn);
}

bool
LifecycleNode::register_on_shutdown(std::function<rcl_lifecycle_ret_t(const State &)> fcn)
{
  return impl_->register_callback(lifecycle_msgs::msg::State::TRANSITION_STATE_SHUTTINGDOWN, fcn);
}

bool
LifecycleNode::register_on_activate(std::function<rcl_lifecycle_ret_t(const State &)> fcn)
{
  return impl_->register_callback(lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING, fcn);
}

bool
LifecycleNode::register_on_deactivate(std::function<rcl_lifecycle_ret_t(const State &)> fcn)
{
  return impl_->register_callback(lifecycle_msgs::msg::State::TRANSITION_STATE_DEACTIVATING, fcn);
}

bool
LifecycleNode::register_on_error(std::function<rcl_lifecycle_ret_t(const State &)> fcn)
{
  return impl_->register_callback(lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING,
           fcn);
}

const State &
LifecycleNode::get_current_state()
{
  return impl_->get_current_state();
}

std::vector<State>
LifecycleNode::get_available_states()
{
  return impl_->get_available_states();
}

std::vector<Transition>
LifecycleNode::get_available_transitions()
{
  return impl_->get_available_transitions();
}

const State &
LifecycleNode::trigger_transition(const Transition & transition)
{
  return trigger_transition(transition.id());
}

const State &
LifecycleNode::trigger_transition(const Transition & transition, rcl_ret_t & error)
{
  return trigger_transition(transition.id(), error);
}

const State &
LifecycleNode::trigger_transition(uint8_t transition_id)
{
  return impl_->trigger_transition(transition_id);
}

const State &
LifecycleNode::trigger_transition(uint8_t transition_id, rcl_ret_t & error)
{
  return impl_->trigger_transition(transition_id, error);
}

const State &
LifecycleNode::configure()
{
  return impl_->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
}

const State &
LifecycleNode::configure(rcl_lifecycle_ret_t & error)
{
  return impl_->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, error);
}

const State &
LifecycleNode::cleanup()
{
  return impl_->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
}

const State &
LifecycleNode::cleanup(rcl_lifecycle_ret_t & error)
{
  return impl_->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, error);
}

const State &
LifecycleNode::activate()
{
  return impl_->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
}

const State &
LifecycleNode::activate(rcl_lifecycle_ret_t & error)
{
  return impl_->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, error);
}

const State &
LifecycleNode::deactivate()
{
  return impl_->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
}

const State &
LifecycleNode::deactivate(rcl_lifecycle_ret_t & error)
{
  return impl_->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, error);
}

const State &
LifecycleNode::shutdown()
{
  return impl_->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_SHUTDOWN);
}

const State &
LifecycleNode::shutdown(rcl_lifecycle_ret_t & error)
{
  return impl_->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_SHUTDOWN, error);
}

void
LifecycleNode::add_publisher_handle(
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisherInterface> pub)
{
  impl_->add_publisher_handle(pub);
}

void
LifecycleNode::add_timer_handle(std::shared_ptr<rclcpp::timer::TimerBase> timer)
{
  impl_->add_timer_handle(timer);
}

}  // namespace rclcpp_lifecycle
