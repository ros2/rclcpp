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

#include <string>
#include <memory>

#include <rcl_lifecycle/states.h>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_node_impl.hpp"  // implementation

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
{};

bool
LifecycleNode::register_on_configure(std::function<bool(void)> fcn)
{
  return impl_->register_callback(rcl_state_configuring.index, fcn);
}

bool
LifecycleNode::register_on_cleanup(std::function<bool(void)> fcn)
{
  return impl_->register_callback(rcl_state_cleaningup.index, fcn);
}

bool
LifecycleNode::register_on_shutdown(std::function<bool(void)> fcn)
{
  return impl_->register_callback(rcl_state_shuttingdown.index, fcn);
}

bool
LifecycleNode::register_on_activate(std::function<bool(void)> fcn)
{
  return impl_->register_callback(rcl_state_activating.index, fcn);
}

bool
LifecycleNode::register_on_deactivate(std::function<bool(void)> fcn)
{
  return impl_->register_callback(rcl_state_deactivating.index, fcn);
}

bool
LifecycleNode::register_on_error(std::function<bool(void)> fcn)
{
  return impl_->register_callback(rcl_state_errorprocessing.index, fcn);
}

void
LifecycleNode::add_publisher_handle(std::shared_ptr<rclcpp::lifecycle::LifecyclePublisherInterface> pub)
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
