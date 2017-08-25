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

#include "rclcpp/node.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/time_source.hpp"


#include <utility>

#include "builtin_interfaces/msg/time.hpp"

#include "rcl/time.h"

#include "rclcpp/exceptions.hpp"

#include "rcutils/logging_macros.h"

namespace rclcpp
{

Time
TimeSource::now(rcl_time_source_type_t clock)
{
  rcl_time_point_t now_time_point;

  if (clock == RCL_ROS_TIME){
    if (!this->ros_time_valid_) {
      throw std::invalid_argument("Timesource ROS connection invalid, RCL_ROS_TIME cannot get now.");
    }

    auto ret = rcl_time_point_init(&now_time_point, &ros_time_source_);
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(
        ret, "could not get init time_point");
      }
  }
  else if (clock == RCL_SYSTEM_TIME){
    auto ret2 = rcl_time_point_init(&now_time_point, &system_time_source_);
    if (ret2 != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(
        ret2, "could not get init time_point");
      }
  }
  else {
    RCUTILS_LOG_ERROR("INVALID TIME TYPE");
  }

  auto ret3 = rcl_time_point_get_now(&now_time_point);
  if (ret3 != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(
      ret3, "could not get current time stamp");
  }

  Time now(now_time_point);
  return now;
}

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
    "clock", std::bind(&TimeSource::clock_cb, this, std::placeholders::_1), rmw_qos_profile_default);
  //TODO(tfoote): Check for time related parameters here too
  this->ros_time_valid_ = true;
}

void TimeSource::detachNode()
{
  this->ros_time_valid_ = false;
  clock_subscription_.reset();
  node_.reset();
  disableROSTime();
}

void TimeSource::initializeData(){
  auto ret1 = rcl_time_source_init(RCL_ROS_TIME, &ros_time_source_);
  if (ret1 != RCL_RET_OK) {
    RCUTILS_LOG_ERROR("Failed to initialize ROS time source");
  }
  auto ret2 = rcl_time_source_init(RCL_ROS_TIME, &system_time_source_);
  if (ret2 != RCL_RET_OK) {
    RCUTILS_LOG_ERROR("Failed to initialize SYSTEM time source");
  }
}

TimeSource::~TimeSource()
{
  if (node_) {
    this->detachNode();
  }
  auto ret1 = rcl_time_source_fini(&ros_time_source_);
  if (ret1 != RCL_RET_OK) {
    RCUTILS_LOG_ERROR("Failed to fini ROS time source");
  }
  auto ret2 = rcl_time_source_fini(&system_time_source_);
  if (ret2 != RCL_RET_OK) {
    RCUTILS_LOG_ERROR("Failed to fini SYSTEM time source");
  }
}

void TimeSource::clock_cb(const builtin_interfaces::msg::Time::SharedPtr msg)
{
  // RCUTILS_LOG_INFO("Got clock message");
  enableROSTime();
  //TODO(tfoote) switch this to be based on if there are clock publishers
  //TODO(tfoote) also setup disable

  //TODO(tfoote) Use a time import/export method from rclcpp Time pending
  rcl_time_point_t clock_time;
  auto ret = rcl_time_point_init(&clock_time, &ros_time_source_);
  if (ret != RCL_RET_OK) {
      RCUTILS_LOG_ERROR("Failed to init ros_time_point");
  }
  clock_time.nanoseconds = msg->sec * 1e9 + msg->nanosec;
  ret = rcl_set_ros_time_override(&ros_time_source_, clock_time.nanoseconds);
  if (ret != RCL_RET_OK) {
      RCUTILS_LOG_ERROR("Failed to set ros_time_override_status");
  }
}

void TimeSource::enableROSTime(){
  bool is_enabled;
  auto ret = rcl_is_enabled_ros_time_override(&ros_time_source_, &is_enabled);
  if (ret != RCL_RET_OK) {
      RCUTILS_LOG_ERROR("Failed to check ros_time_override_status");
  }
  if (!is_enabled)
  {
   ret = rcl_enable_ros_time_override(&ros_time_source_);
   if (ret != RCL_RET_OK) {
       RCUTILS_LOG_ERROR("Failed to enable ros_time_override_status");
   }
  }
}

void TimeSource::disableROSTime(){
  bool is_enabled;
  auto ret = rcl_is_enabled_ros_time_override(&ros_time_source_, &is_enabled);
  if (ret != RCL_RET_OK) {
      RCUTILS_LOG_ERROR("Failed to check ros_time_override_status");
  }
  if (!is_enabled)
  {
   ret = rcl_disable_ros_time_override(&ros_time_source_);
   if (ret != RCL_RET_OK) {
       RCUTILS_LOG_ERROR("Failed to disable ros_time_override_status");
   }
  }
}

}  // namespace rclcpp
