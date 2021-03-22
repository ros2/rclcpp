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

#include <string>

#include "rmw/error_handling.h"
#include "rmw/types.h"
#include "rmw/qos_profiles.h"

namespace rclcpp
{

std::string qos_policy_name_from_kind(rmw_qos_policy_kind_t policy_kind)
{
  switch (policy_kind) {
    case RMW_QOS_POLICY_DURABILITY:
      return "DURABILITY_QOS_POLICY";
    case RMW_QOS_POLICY_DEADLINE:
      return "DEADLINE_QOS_POLICY";
    case RMW_QOS_POLICY_LIVELINESS:
      return "LIVELINESS_QOS_POLICY";
    case RMW_QOS_POLICY_RELIABILITY:
      return "RELIABILITY_QOS_POLICY";
    case RMW_QOS_POLICY_HISTORY:
      return "HISTORY_QOS_POLICY";
    case RMW_QOS_POLICY_LIFESPAN:
      return "LIFESPAN_QOS_POLICY";
    default:
      return "INVALID_QOS_POLICY";
  }
}

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
QoS::history(HistoryPolicy history)
{
  rmw_qos_profile_.history = static_cast<rmw_qos_history_policy_t>(history);
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
QoS::reliability(ReliabilityPolicy reliability)
{
  rmw_qos_profile_.reliability = static_cast<rmw_qos_reliability_policy_t>(reliability);
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
QoS::durability(DurabilityPolicy durability)
{
  rmw_qos_profile_.durability = static_cast<rmw_qos_durability_policy_t>(durability);
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
QoS::liveliness(LivelinessPolicy liveliness)
{
  rmw_qos_profile_.liveliness = static_cast<rmw_qos_liveliness_policy_t>(liveliness);
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

HistoryPolicy
QoS::history() const
{
  return static_cast<HistoryPolicy>(rmw_qos_profile_.history);
}

size_t
QoS::depth() const {return rmw_qos_profile_.depth;}

ReliabilityPolicy
QoS::reliability() const
{
  return static_cast<ReliabilityPolicy>(rmw_qos_profile_.reliability);
}

DurabilityPolicy
QoS::durability() const
{
  return static_cast<DurabilityPolicy>(rmw_qos_profile_.durability);
}

Duration
QoS::deadline() const {return Duration::from_rmw_time(rmw_qos_profile_.deadline);}

Duration
QoS::lifespan() const {return Duration::from_rmw_time(rmw_qos_profile_.lifespan);}

LivelinessPolicy
QoS::liveliness() const
{
  return static_cast<LivelinessPolicy>(rmw_qos_profile_.liveliness);
}

Duration
QoS::liveliness_lease_duration() const
{
  return Duration::from_rmw_time(rmw_qos_profile_.liveliness_lease_duration);
}

bool
QoS::avoid_ros_namespace_conventions() const
{
  return rmw_qos_profile_.avoid_ros_namespace_conventions;
}

namespace
{
/// Check if two rmw_time_t have the same values.
bool operator==(const rmw_time_t & left, const rmw_time_t & right)
{
  return left.sec == right.sec && left.nsec == right.nsec;
}
}  // unnamed namespace

bool operator==(const QoS & left, const QoS & right)
{
  const auto & pl = left.get_rmw_qos_profile();
  const auto & pr = right.get_rmw_qos_profile();
  return pl.history == pr.history &&
         pl.depth == pr.depth &&
         pl.reliability == pr.reliability &&
         pl.durability == pr.durability &&
         pl.deadline == pr.deadline &&
         pl.lifespan == pr.lifespan &&
         pl.liveliness == pr.liveliness &&
         pl.liveliness_lease_duration == pr.liveliness_lease_duration &&
         pl.avoid_ros_namespace_conventions == pr.avoid_ros_namespace_conventions;
}

bool operator!=(const QoS & left, const QoS & right)
{
  return !(left == right);
}

QoSCheckCompatibleResult
qos_check_compatible(const QoS & publisher_qos, const QoS & subscription_qos)
{
  rmw_qos_compatibility_type_t compatible;
  const size_t reason_size = 2048u;
  char reason_c_str[reason_size] = "";
  rmw_ret_t ret = rmw_qos_profile_check_compatible(
    publisher_qos.get_rmw_qos_profile(),
    subscription_qos.get_rmw_qos_profile(),
    &compatible,
    reason_c_str,
    reason_size);
  if (RMW_RET_OK != ret) {
    std::string error_str(rmw_get_error_string().str);
    rmw_reset_error();
    throw rclcpp::exceptions::QoSCheckCompatibleException{error_str};
  }

  QoSCheckCompatibleResult result;
  result.reason = std::string(reason_c_str);

  switch (compatible) {
    case RMW_QOS_COMPATIBILITY_OK:
      result.compatibility = QoSCompatibility::Ok;
      break;
    case RMW_QOS_COMPATIBILITY_WARNING:
      result.compatibility = QoSCompatibility::Warning;
      break;
    case RMW_QOS_COMPATIBILITY_ERROR:
      result.compatibility = QoSCompatibility::Error;
      break;
    default:
      throw rclcpp::exceptions::QoSCheckCompatibleException{
              "Unexpected compatibility value returned by rmw '" + std::to_string(compatible) +
              "'"};
  }
  return result;
}

ClockQoS::ClockQoS(const QoSInitialization & qos_initialization)
// Using `rmw_qos_profile_sensor_data` intentionally.
// It's best effort and `qos_initialization` is overriding the depth to 1.
: QoS(qos_initialization, rmw_qos_profile_sensor_data)
{}

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

RosoutQoS::RosoutQoS(const QoSInitialization & rosout_initialization)
: QoS(rosout_initialization, rcl_qos_profile_rosout_default)
{}

SystemDefaultsQoS::SystemDefaultsQoS(const QoSInitialization & qos_initialization)
: QoS(qos_initialization, rmw_qos_profile_system_default)
{}

}  // namespace rclcpp
