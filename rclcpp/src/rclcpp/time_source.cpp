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
#include <utility>

#include "builtin_interfaces/msg/time.hpp"

#include "rcl/time.h"

#include "rcutils/logging_macros.h"

#include "rclcpp/clock.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/time_source.hpp"


namespace rclcpp
{

TimeSource::TimeSource(std::shared_ptr<rclcpp::node::Node> node)
: ros_time_active_(false)
{
  this->attachNode(node);
}

TimeSource::TimeSource()
: ros_time_active_(false)
{
}

void TimeSource::attachNode(std::shared_ptr<rclcpp::node::Node> node)
{
  node_ = node;
  // TODO(tfoote): Update QOS
  clock_subscription_ = node_->create_subscription<builtin_interfaces::msg::Time>(
    "clock", std::bind(&TimeSource::clock_cb, this, std::placeholders::_1),
    rmw_qos_profile_default);

  parameter_client_ = std::make_shared<rclcpp::parameter_client::AsyncParametersClient>(node);
  parameter_subscription_ =
    parameter_client_->on_parameter_event(std::bind(&TimeSource::on_parameter_event,
      this, std::placeholders::_1));
}

void TimeSource::detachNode()
{
  this->ros_time_active_ = false;
  clock_subscription_.reset();
  parameter_client_.reset();
  node_.reset();
  disable_ros_time();
}

void TimeSource::attachClock(std::shared_ptr<rclcpp::Clock> clock)
{
  if (clock->get_clock_type() != RCL_ROS_TIME) {
    throw std::invalid_argument("Cannot attach clock to a time source that's not a ROS clock");
  }

  std::lock_guard<std::mutex> guard(clock_list_lock_);
  associated_clocks_.push_back(clock);
  // Set the clock if there's already data for it
  if (last_msg_set_) {
    set_clock(last_msg_set_, ros_time_active_, clock);
  }
}

void TimeSource::detachClock(std::shared_ptr<rclcpp::Clock> clock)
{
  std::lock_guard<std::mutex> guard(clock_list_lock_);
  auto result = std::find(associated_clocks_.begin(), associated_clocks_.end(), clock);
  if (result != associated_clocks_.end()) {
    associated_clocks_.erase(result);
  } else {
    RCUTILS_LOG_ERROR("Failed to remove clock");
  }
}

TimeSource::~TimeSource()
{
  if (node_) {
    this->detachNode();
  }
}

void TimeSource::set_clock(
  const builtin_interfaces::msg::Time::SharedPtr msg, bool set_ros_time_enabled,
  std::shared_ptr<rclcpp::Clock> clock)
{
  // Compute diff
  rclcpp::Time msg_time = rclcpp::Time(*msg);
  rclcpp::Time now = clock->now();
  auto diff = now - msg_time;
  rclcpp::TimeJump jump;
  jump.delta_.nanoseconds = diff.nanoseconds();

  // Compute jump type
  if (clock->ros_time_is_active()) {
    if (set_ros_time_enabled) {
      jump.jump_type_ = TimeJump::ClockChange_t::ROS_TIME_NO_CHANGE;
    } else {
      jump.jump_type_ = TimeJump::ClockChange_t::ROS_TIME_DEACTIVATED;
    }
  } else if (!clock->ros_time_is_active()) {
    if (set_ros_time_enabled) {
      jump.jump_type_ = TimeJump::ClockChange_t::ROS_TIME_ACTIVATED;
    } else {
      jump.jump_type_ = TimeJump::ClockChange_t::SYSTEM_TIME_NO_CHANGE;
    }
  }

  if (jump.jump_type_ == TimeJump::ClockChange_t::SYSTEM_TIME_NO_CHANGE) {
    // No change/no updates don't act.
    return;
  }

  // TODO(tfoote) potential race condition with callback object going out of scope
  // Document to user requirments
  auto active_callbacks = clock->get_triggered_callback_handlers(jump);
  clock->invoke_prejump_callbacks(active_callbacks);

  // Do change
  if (jump.jump_type_ == TimeJump::ClockChange_t::ROS_TIME_DEACTIVATED) {
    disable_ros_time(clock);
  } else if (jump.jump_type_ == TimeJump::ClockChange_t::ROS_TIME_ACTIVATED) {
    enable_ros_time(clock);
  }

  if (jump.jump_type_ == TimeJump::ClockChange_t::ROS_TIME_ACTIVATED ||
    jump.jump_type_ == TimeJump::ClockChange_t::ROS_TIME_NO_CHANGE)
  {
    auto ret = rcl_set_ros_time_override(&(clock->rcl_clock_), msg_time.nanoseconds());
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(
        ret, "Failed to set ros_time_override_status");
    }
  }
  // Post change callbacks
  clock->invoke_postjump_callbacks(active_callbacks, jump);
}

void TimeSource::clock_cb(const builtin_interfaces::msg::Time::SharedPtr msg)
{
  if (!this->ros_time_active_) {
    enable_ros_time();
  }
  // Cache the last message in case a new clock is attached.
  last_msg_set_ = msg;

  std::lock_guard<std::mutex> guard(clock_list_lock_);
  for (auto it = associated_clocks_.begin(); it != associated_clocks_.end(); ++it) {
    set_clock(msg, true, *it);
  }
}

void TimeSource::on_parameter_event(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  for (auto & new_parameter : event->new_parameters) {
    if (new_parameter.value.type == rclcpp::parameter::ParameterType::PARAMETER_BOOL) {
      if (new_parameter.name == "use_sim_time") {
        bool value = new_parameter.value.bool_value;
        if (value) {
          parameter_state_ = SET_TRUE;
          enable_ros_time();
        } else {
          parameter_state_ = SET_FALSE;
          disable_ros_time();
        }
      }
    }
  }
  for (auto & changed_parameter : event->changed_parameters) {
    if (changed_parameter.value.type == rclcpp::parameter::ParameterType::PARAMETER_BOOL) {
      if (changed_parameter.name == "use_sim_time") {
        bool value = changed_parameter.value.bool_value;
        if (value) {
          parameter_state_ = SET_TRUE;
          enable_ros_time();
        } else {
          parameter_state_ = SET_FALSE;
          disable_ros_time();
        }
      }
    }
  }
  for (auto & deleted_parameter : event->deleted_parameters) {
    if (deleted_parameter.name == "use_sim_time") {
      // If the parameter is deleted mark it as unset but dont' change state.
      parameter_state_ = UNSET;
    }
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

  // Update all attached clocks
  std::lock_guard<std::mutex> guard(clock_list_lock_);
  for (auto it = associated_clocks_.begin(); it != associated_clocks_.end(); ++it) {
    auto msg = std::make_shared<builtin_interfaces::msg::Time>();
    msg->sec = 0;
    msg->nanosec = 0;
    set_clock(msg, true, *it);
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
