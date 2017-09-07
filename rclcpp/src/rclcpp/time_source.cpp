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

#include "rclcpp/exceptions.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/time_source.hpp"


namespace rclcpp
{

TimeSource::TimeSource(std::shared_ptr<rclcpp::node::Node> node)
: ros_time_valid_(false)
{
  this->initializeData();
  this->attachNode(node);
}

TimeSource::TimeSource()
: ros_time_valid_(false)
{
  this->initializeData();
}

void TimeSource::attachNode(std::shared_ptr<rclcpp::node::Node> node)
{
  node_ = node;
  // TODO(tfoote): Update QOS
  clock_subscription_ = node_->create_subscription<builtin_interfaces::msg::Time>(
    "clock", std::bind(&TimeSource::clock_cb, this, std::placeholders::_1),
    rmw_qos_profile_default);
  // TODO(tfoote): Check for time related parameters here too
  this->ros_time_valid_ = true;
}

void TimeSource::detachNode()
{
  this->ros_time_valid_ = false;
  clock_subscription_.reset();
  node_.reset();
  disableROSTime();
}

void TimeSource::attachClock(std::shared_ptr<rclcpp::Clock> clock)
{
  std::lock_guard<std::mutex> guard(clock_list_lock_);
  associated_clocks_.push_back(clock);
  // TODO(tfoote) cache state and set clock here
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

void TimeSource::initializeData()
{
  auto ret1 = rcl_clock_init(RCL_ROS_TIME, &ros_clock_);
  if (ret1 != RCL_RET_OK) {
    RCUTILS_LOG_ERROR("Failed to initialize ROS time source");
  }
}

TimeSource::~TimeSource()
{
  if (node_) {
    this->detachNode();
  }
  auto ret1 = rcl_clock_fini(&ros_clock_);
  if (ret1 != RCL_RET_OK) {
    RCUTILS_LOG_ERROR("Failed to fini ROS time source");
  }
}

void TimeSource::setClock(const builtin_interfaces::msg::Time::SharedPtr msg,
    std::shared_ptr<rclcpp::Clock> clock)
{
  // TODO(tfoote) Use a time import/export method from rclcpp Time pending
  rcl_time_point_t clock_time;
  auto ret = rcl_time_point_init(&clock_time, &(clock->rcl_clock_.type));
  if (ret != RCL_RET_OK) {
    RCUTILS_LOG_ERROR("Failed to init ros_time_point");
  }
  clock_time.nanoseconds = msg->sec * 1e9 + msg->nanosec;
  ret = rcl_set_ros_time_override(&(clock->rcl_clock_), clock_time.nanoseconds);
  if (ret != RCL_RET_OK) {
    RCUTILS_LOG_ERROR("Failed to set ros_time_override_status");
  }
  auto ret2 = rcl_time_point_fini(&clock_time);
  if (ret2 != RCL_RET_OK) {
    RCUTILS_LOG_ERROR("Failed to fini clock_time");
  }
}

void TimeSource::clock_cb(const builtin_interfaces::msg::Time::SharedPtr msg)
{
  // RCUTILS_LOG_INFO("Got clock message");
  enableROSTime();
  // TODO(tfoote) switch this to be based on if there are clock publishers or use_sim_time
  // TODO(tfoote) also setup disable

  std::lock_guard<std::mutex> guard(clock_list_lock_);
  for (auto it = associated_clocks_.begin(); it != associated_clocks_.end(); ++it) {
    setClock(msg, *it);
  }
}

void TimeSource::enableROSTime()
{
  bool is_enabled;
  auto ret = rcl_is_enabled_ros_time_override(&ros_clock_, &is_enabled);
  if (ret != RCL_RET_OK) {
    RCUTILS_LOG_ERROR("Failed to check ros_time_override_status");
  }
  if (!is_enabled) {
    ret = rcl_enable_ros_time_override(&ros_clock_);
    if (ret != RCL_RET_OK) {
      RCUTILS_LOG_ERROR("Failed to enable ros_time_override_status");
    }
  }
}

void TimeSource::disableROSTime()
{
  bool is_enabled;
  auto ret = rcl_is_enabled_ros_time_override(&ros_clock_, &is_enabled);
  if (ret != RCL_RET_OK) {
    RCUTILS_LOG_ERROR("Failed to check ros_time_override_status");
  }
  if (!is_enabled) {
    ret = rcl_disable_ros_time_override(&ros_clock_);
    if (ret != RCL_RET_OK) {
      RCUTILS_LOG_ERROR("Failed to disable ros_time_override_status");
    }
  }
}

}  // namespace rclcpp
