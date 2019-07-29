// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"

#include "rcl/time.h"

#include "rclcpp/clock.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/time_source.hpp"

namespace rclcpp
{

TimeSource::TimeSource(std::shared_ptr<rclcpp::Node> node)
: logger_(rclcpp::get_logger("rclcpp")),
  clock_subscription_(nullptr),
  ros_time_active_(false)
{
  this->attachNode(node);
}

TimeSource::TimeSource()
: logger_(rclcpp::get_logger("rclcpp")),
  ros_time_active_(false)
{
}

void TimeSource::attachNode(rclcpp::Node::SharedPtr node)
{
  attachNode(
    node->get_node_base_interface(),
    node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface(),
    node->get_node_logging_interface(),
    node->get_node_clock_interface(),
    node->get_node_parameters_interface());
}

void TimeSource::attachNode(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface,
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface,
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_interface,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_interface)
{
  node_base_ = node_base_interface;
  node_topics_ = node_topics_interface;
  node_graph_ = node_graph_interface;
  node_services_ = node_services_interface;
  node_logging_ = node_logging_interface;
  node_clock_ = node_clock_interface;
  node_parameters_ = node_parameters_interface;
  // TODO(tfoote): Update QOS

  logger_ = node_logging_->get_logger();

  // Though this defaults to false, it can be overridden by initial parameter values for the node,
  // which may be given by the user at the node's construction or even by command-line arguments.
  rclcpp::ParameterValue use_sim_time_param;
  const char * use_sim_time_name = "use_sim_time";
  if (!node_parameters_->has_parameter(use_sim_time_name)) {
    use_sim_time_param = node_parameters_->declare_parameter(
      use_sim_time_name,
      rclcpp::ParameterValue(false),
      rcl_interfaces::msg::ParameterDescriptor());
  } else {
    use_sim_time_param = node_parameters_->get_parameter(use_sim_time_name).get_parameter_value();
  }
  if (use_sim_time_param.get_type() == rclcpp::PARAMETER_BOOL) {
    if (use_sim_time_param.get<bool>()) {
      parameter_state_ = SET_TRUE;
      enable_ros_time();
      create_clock_sub();
    }
  } else {
    // TODO(wjwwood): use set_on_parameters_set_callback to catch the type mismatch,
    //   before the use_sim_time parameter can ever be set to an invalid value
    RCLCPP_ERROR(logger_, "Invalid type '%s' for parameter 'use_sim_time', should be 'bool'",
      rclcpp::to_string(use_sim_time_param.get_type()).c_str());
  }

  // TODO(tfoote) use parameters interface not subscribe to events via topic ticketed #609
  parameter_subscription_ = rclcpp::AsyncParametersClient::on_parameter_event(
    node_topics_,
    std::bind(&TimeSource::on_parameter_event, this, std::placeholders::_1));
}

void TimeSource::detachNode()
{
  this->ros_time_active_ = false;
  clock_subscription_.reset();
  parameter_subscription_.reset();
  node_base_.reset();
  node_topics_.reset();
  node_graph_.reset();
  node_services_.reset();
  node_logging_.reset();
  node_clock_.reset();
  node_parameters_.reset();
  disable_ros_time();
}

void TimeSource::attachClock(std::shared_ptr<rclcpp::Clock> clock)
{
  if (clock->get_clock_type() != RCL_ROS_TIME) {
    throw std::invalid_argument("Cannot attach clock to a time source that's not a ROS clock");
  }

  std::lock_guard<std::mutex> guard(clock_list_lock_);
  associated_clocks_.push_back(clock);
  // Set the clock to zero unless there's a recently received message
  auto time_msg = std::make_shared<builtin_interfaces::msg::Time>();
  if (last_msg_set_) {
    time_msg = std::make_shared<builtin_interfaces::msg::Time>(last_msg_set_->clock);
  }
  set_clock(time_msg, ros_time_active_, clock);
}

void TimeSource::detachClock(std::shared_ptr<rclcpp::Clock> clock)
{
  std::lock_guard<std::mutex> guard(clock_list_lock_);
  auto result = std::find(associated_clocks_.begin(), associated_clocks_.end(), clock);
  if (result != associated_clocks_.end()) {
    associated_clocks_.erase(result);
  } else {
    RCLCPP_ERROR(logger_, "failed to remove clock");
  }
}

TimeSource::~TimeSource()
{
  if (node_base_ || node_topics_ || node_graph_ || node_services_ ||
    node_logging_ || node_clock_ || node_parameters_)
  {
    this->detachNode();
  }
}

void TimeSource::set_clock(
  const builtin_interfaces::msg::Time::SharedPtr msg, bool set_ros_time_enabled,
  std::shared_ptr<rclcpp::Clock> clock)
{
  // Do change
  if (!set_ros_time_enabled && clock->ros_time_is_active()) {
    disable_ros_time(clock);
  } else if (set_ros_time_enabled && !clock->ros_time_is_active()) {
    enable_ros_time(clock);
  }

  auto ret = rcl_set_ros_time_override(&(clock->rcl_clock_), rclcpp::Time(*msg).nanoseconds());
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(
      ret, "Failed to set ros_time_override_status");
  }
}

void TimeSource::clock_cb(const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
  if (!this->ros_time_active_ && SET_TRUE == this->parameter_state_) {
    enable_ros_time();
  }
  // Cache the last message in case a new clock is attached.
  last_msg_set_ = msg;
  auto time_msg = std::make_shared<builtin_interfaces::msg::Time>(msg->clock);

  if (SET_TRUE == this->parameter_state_) {
    std::lock_guard<std::mutex> guard(clock_list_lock_);
    for (auto it = associated_clocks_.begin(); it != associated_clocks_.end(); ++it) {
      set_clock(time_msg, true, *it);
    }
  }
}

void TimeSource::create_clock_sub()
{
  std::lock_guard<std::mutex> guard(clock_sub_lock_);
  if (clock_subscription_) {
    // Subscription already created.
    return;
  }

  clock_subscription_ = rclcpp::create_subscription<rosgraph_msgs::msg::Clock>(
    node_topics_,
    "/clock",
    rclcpp::QoS(QoSInitialization::from_rmw(rmw_qos_profile_default)),
    std::bind(&TimeSource::clock_cb, this, std::placeholders::_1)
  );
}

void TimeSource::destroy_clock_sub()
{
  std::lock_guard<std::mutex> guard(clock_sub_lock_);
  clock_subscription_.reset();
}

void TimeSource::on_parameter_event(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  // Filter out events on 'use_sim_time' parameter instances in other nodes.
  if (event->node != node_base_->get_fully_qualified_name()) {
    return;
  }
  // Filter for only 'use_sim_time' being added or changed.
  rclcpp::ParameterEventsFilter filter(event, {"use_sim_time"},
    {rclcpp::ParameterEventsFilter::EventType::NEW,
      rclcpp::ParameterEventsFilter::EventType::CHANGED});
  for (auto & it : filter.get_events()) {
    if (it.second->value.type != ParameterType::PARAMETER_BOOL) {
      RCLCPP_ERROR(logger_, "use_sim_time parameter cannot be set to anything but a bool");
      continue;
    }
    if (it.second->value.bool_value) {
      parameter_state_ = SET_TRUE;
      enable_ros_time();
      create_clock_sub();
    } else {
      parameter_state_ = SET_FALSE;
      disable_ros_time();
      destroy_clock_sub();
    }
  }
  // Handle the case that use_sim_time was deleted.
  rclcpp::ParameterEventsFilter deleted(event, {"use_sim_time"},
    {rclcpp::ParameterEventsFilter::EventType::DELETED});
  for (auto & it : deleted.get_events()) {
    (void) it;  // if there is a match it's already matched, don't bother reading it.
    // If the parameter is deleted mark it as unset but dont' change state.
    parameter_state_ = UNSET;
  }
}

void TimeSource::enable_ros_time(std::shared_ptr<rclcpp::Clock> clock)
{
  auto ret = rcl_enable_ros_time_override(&clock->rcl_clock_);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(
      ret, "Failed to enable ros_time_override_status");
  }
}

void TimeSource::disable_ros_time(std::shared_ptr<rclcpp::Clock> clock)
{
  auto ret = rcl_disable_ros_time_override(&clock->rcl_clock_);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(
      ret, "Failed to enable ros_time_override_status");
  }
}

void TimeSource::enable_ros_time()
{
  if (ros_time_active_) {
    // already enabled no-op
    return;
  }

  // Local storage
  ros_time_active_ = true;

  // Update all attached clocks to zero or last recorded time
  std::lock_guard<std::mutex> guard(clock_list_lock_);
  auto time_msg = std::make_shared<builtin_interfaces::msg::Time>();
  if (last_msg_set_) {
    time_msg = std::make_shared<builtin_interfaces::msg::Time>(last_msg_set_->clock);
  }
  for (auto it = associated_clocks_.begin(); it != associated_clocks_.end(); ++it) {
    set_clock(time_msg, true, *it);
  }
}

void TimeSource::disable_ros_time()
{
  if (!ros_time_active_) {
    // already disabled no-op
    return;
  }

  // Local storage
  ros_time_active_ = false;

  // Update all attached clocks
  std::lock_guard<std::mutex> guard(clock_list_lock_);
  for (auto it = associated_clocks_.begin(); it != associated_clocks_.end(); ++it) {
    auto msg = std::make_shared<builtin_interfaces::msg::Time>();
    set_clock(msg, false, *it);
  }
}

}  // namespace rclcpp
