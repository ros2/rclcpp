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
#include "lifecycle_node_impl.hpp"  // implementation

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include <string>
#include <memory>

namespace rclcpp
{
namespace lifecycle
{

LifecycleNode::LifecycleNode(const std::string & node_name, bool use_intra_process_comms)
: base_node_handle_(std::make_shared<rclcpp::node::Node>(node_name, use_intra_process_comms)),
  communication_interface_(base_node_handle_),
  impl_(new LifecycleNodeImpl(base_node_handle_))
{
  impl_->init();

  register_on_configure(std::bind(&LifecycleNodeInterface::on_configure, this));
  register_on_cleanup(std::bind(&LifecycleNodeInterface::on_cleanup, this));
  register_on_shutdown(std::bind(&LifecycleNodeInterface::on_shutdown, this));
  register_on_activate(std::bind(&LifecycleNodeInterface::on_activate, this));
  register_on_deactivate(std::bind(&LifecycleNodeInterface::on_deactivate, this));
  register_on_error(std::bind(&LifecycleNodeInterface::on_error, this));
}

LifecycleNode::~LifecycleNode()
{}

bool
LifecycleNode::register_on_configure(std::function<bool(void)> fcn)
{
  return impl_->register_callback(lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING, fcn);
}

bool
LifecycleNode::register_on_cleanup(std::function<bool(void)> fcn)
{
  return impl_->register_callback(lifecycle_msgs::msg::State::TRANSITION_STATE_CLEANINGUP, fcn);
}

bool
LifecycleNode::register_on_shutdown(std::function<bool(void)> fcn)
{
  return impl_->register_callback(lifecycle_msgs::msg::State::TRANSITION_STATE_SHUTTINGDOWN, fcn);
}

bool
LifecycleNode::register_on_activate(std::function<bool(void)> fcn)
{
  return impl_->register_callback(lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING, fcn);
}

bool
LifecycleNode::register_on_deactivate(std::function<bool(void)> fcn)
{
  return impl_->register_callback(lifecycle_msgs::msg::State::TRANSITION_STATE_DEACTIVATING, fcn);
}

bool
LifecycleNode::register_on_error(std::function<bool(void)> fcn)
{
  return impl_->register_callback(lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING,
           fcn);
}

const State &
LifecycleNode::get_current_state()
{
  return impl_->get_current_state();
}

const State &
LifecycleNode::trigger_transition(const Transition & transition)
{
  return trigger_transition(transition.id());
}

const State &
LifecycleNode::trigger_transition(unsigned int transition_id)
{
  return impl_->trigger_transition(transition_id);
}

void
LifecycleNode::add_publisher_handle(
  std::shared_ptr<rclcpp::lifecycle::LifecyclePublisherInterface> pub)
{
  impl_->add_publisher_handle(pub);
}

void
LifecycleNode::add_timer_handle(std::shared_ptr<rclcpp::timer::TimerBase> timer)
{
  impl_->add_timer_handle(timer);
}

}  // namespace lifecycle
}  // namespace rclcpp
