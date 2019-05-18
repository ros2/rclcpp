// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/qos.hpp"

#include <rmw/types.h>

namespace rclcpp
{

QoSInitialization::QoSInitialization(rmw_qos_history_policy_t history_policy_arg, size_t depth_arg)
: history_policy(history_policy_arg), depth(depth_arg)
{}

QoSInitialization
QoSInitialization::from_rmw(const rmw_qos_profile_t & rmw_qos)
{
  switch (rmw_qos.history) {
    case RMW_QOS_POLICY_HISTORY_KEEP_ALL:
      return KeepAll();
    case RMW_QOS_POLICY_HISTORY_KEEP_LAST:
    case RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT:
    case RMW_QOS_POLICY_HISTORY_UNKNOWN:
    default:
      return KeepLast(rmw_qos.depth);
  }
}

KeepAll::KeepAll()
: QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_ALL, 0)
{}

KeepLast::KeepLast(size_t depth)
: QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, depth)
{}

QoS::QoS(
  const QoSInitialization & qos_initialization,
  const rmw_qos_profile_t & initial_profile)
: rmw_qos_profile_(initial_profile)
{
  rmw_qos_profile_.history = qos_initialization.history_policy;
  rmw_qos_profile_.depth = qos_initialization.depth;
}

QoS::QoS(size_t history_depth)
: QoS(KeepLast(history_depth))
{}

rmw_qos_profile_t &
QoS::get_rmw_qos_profile()
{
  return rmw_qos_profile_;
}

const rmw_qos_profile_t &
QoS::get_rmw_qos_profile() const
{
  return rmw_qos_profile_;
}

QoS &
QoS::history(rmw_qos_history_policy_t history)
{
  rmw_qos_profile_.history = history;
  return *this;
}

QoS &
QoS::keep_last(size_t depth)
{
  rmw_qos_profile_.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  rmw_qos_profile_.depth = depth;
  return *this;
}

QoS &
QoS::keep_all()
{
  rmw_qos_profile_.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
  rmw_qos_profile_.depth = 0;
  return *this;
}

QoS &
QoS::reliability(rmw_qos_reliability_policy_t reliability)
{
  rmw_qos_profile_.reliability = reliability;
  return *this;
}

QoS &
QoS::reliable()
{
  return this->reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
}

QoS &
QoS::best_effort()
{
  return this->reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
}

QoS &
QoS::durability(rmw_qos_durability_policy_t durability)
{
  rmw_qos_profile_.durability = durability;
  return *this;
}

QoS &
QoS::durability_volatile()
{
  return this->durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

QoS &
QoS::transient_local()
{
  return this->durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
}

QoS &
QoS::deadline(rmw_time_t deadline)
{
  rmw_qos_profile_.deadline = deadline;
  return *this;
}

QoS &
QoS::deadline(const rclcpp::Duration & deadline)
{
  return this->deadline(deadline.to_rmw_time());
}

QoS &
QoS::lifespan(rmw_time_t lifespan)
{
  rmw_qos_profile_.lifespan = lifespan;
  return *this;
}

QoS &
QoS::lifespan(const rclcpp::Duration & lifespan)
{
  return this->lifespan(lifespan.to_rmw_time());
}

QoS &
QoS::liveliness(rmw_qos_liveliness_policy_t liveliness)
{
  rmw_qos_profile_.liveliness = liveliness;
  return *this;
}

QoS &
QoS::liveliness_lease_duration(rmw_time_t liveliness_lease_duration)
{
  rmw_qos_profile_.liveliness_lease_duration = liveliness_lease_duration;
  return *this;
}

QoS &
QoS::liveliness_lease_duration(const rclcpp::Duration & liveliness_lease_duration)
{
  return this->liveliness_lease_duration(liveliness_lease_duration.to_rmw_time());
}

QoS &
QoS::avoid_ros_namespace_conventions(bool avoid_ros_namespace_conventions)
{
  rmw_qos_profile_.avoid_ros_namespace_conventions = avoid_ros_namespace_conventions;
  return *this;
}

SensorDataQoS::SensorDataQoS(const QoSInitialization & qos_initialization)
: QoS(qos_initialization, rmw_qos_profile_sensor_data)
{}

ParametersQoS::ParametersQoS(const QoSInitialization & qos_initialization)
: QoS(qos_initialization, rmw_qos_profile_parameters)
{}

ServicesQoS::ServicesQoS(const QoSInitialization & qos_initialization)
: QoS(qos_initialization, rmw_qos_profile_services_default)
{}

ParameterEventsQoS::ParameterEventsQoS(const QoSInitialization & qos_initialization)
: QoS(qos_initialization, rmw_qos_profile_parameter_events)
{}

SystemDefaultsQoS::SystemDefaultsQoS(const QoSInitialization & qos_initialization)
: QoS(qos_initialization, rmw_qos_profile_system_default)
{}

}  // namespace rclcpp
