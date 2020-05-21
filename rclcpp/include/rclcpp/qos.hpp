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

#ifndef RCLCPP__QOS_HPP_
#define RCLCPP__QOS_HPP_

#include <string>

#include "rclcpp/duration.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rmw/incompatible_qos_events_statuses.h"
#include "rmw/qos_profiles.h"
#include "rmw/types.h"

namespace rclcpp
{

std::string qos_policy_name_from_kind(rmw_qos_policy_kind_t policy_kind);

/// QoS initialization values, cannot be created directly, use KeepAll or KeepLast instead.
struct RCLCPP_PUBLIC QoSInitialization
{
  rmw_qos_history_policy_t history_policy;
  size_t depth;

  /// Constructor which takes both a history policy and a depth (even if it would be unused).
  QoSInitialization(rmw_qos_history_policy_t history_policy_arg, size_t depth_arg);

  /// Create a QoSInitialization from an existing rmw_qos_profile_t, using its history and depth.
  static
  QoSInitialization
  from_rmw(const rmw_qos_profile_t & rmw_qos);
};

/// Use to initialize the QoS with the keep_all history setting.
struct RCLCPP_PUBLIC KeepAll : public rclcpp::QoSInitialization
{
  KeepAll();
};

/// Use to initialize the QoS with the keep_last history setting and the given depth.
struct RCLCPP_PUBLIC KeepLast : public rclcpp::QoSInitialization
{
  explicit KeepLast(size_t depth);
};

/// Encapsulation of Quality of Service settings.
class RCLCPP_PUBLIC QoS
{
public:
  /// Constructor which allows you to construct a QoS by giving the only required settings.
  explicit
  QoS(
    const QoSInitialization & qos_initialization,
    const rmw_qos_profile_t & initial_profile = rmw_qos_profile_default);

  /// Conversion constructor to ease construction in the common case of just specifying depth.
  /**
   * Convenience constructor, equivalent to QoS(KeepLast(history_depth)).
   */
  // cppcheck-suppress noExplicitConstructor
  QoS(size_t history_depth);  // NOLINT(runtime/explicit): conversion constructor

  /// Return the rmw qos profile.
  rmw_qos_profile_t &
  get_rmw_qos_profile();

  /// Return the rmw qos profile.
  const rmw_qos_profile_t &
  get_rmw_qos_profile() const;

  /// Set the history policy.
  QoS &
  history(rmw_qos_history_policy_t history);

  /// Set the history to keep last.
  QoS &
  keep_last(size_t depth);

  /// Set the history to keep all.
  QoS &
  keep_all();

  /// Set the reliability setting.
  QoS &
  reliability(rmw_qos_reliability_policy_t reliability);

  /// Set the reliability setting to reliable.
  QoS &
  reliable();

  /// Set the reliability setting to best effort.
  QoS &
  best_effort();

  /// Set the durability setting.
  QoS &
  durability(rmw_qos_durability_policy_t durability);

  /// Set the durability setting to volatile.
  /**
    * Note that this cannot be named `volatile` because it is a C++ keyword.
    */
  QoS &
  durability_volatile();

  /// Set the durability setting to transient local.
  QoS &
  transient_local();

  /// Set the deadline setting.
  QoS &
  deadline(rmw_time_t deadline);

  /// Set the deadline setting, rclcpp::Duration.
  QoS &
  deadline(const rclcpp::Duration & deadline);

  /// Set the lifespan setting.
  QoS &
  lifespan(rmw_time_t lifespan);

  /// Set the lifespan setting, rclcpp::Duration.
  QoS &
  lifespan(const rclcpp::Duration & lifespan);

  /// Set the liveliness setting.
  QoS &
  liveliness(rmw_qos_liveliness_policy_t liveliness);

  /// Set the liveliness_lease_duration setting.
  QoS &
  liveliness_lease_duration(rmw_time_t liveliness_lease_duration);

  /// Set the liveliness_lease_duration setting, rclcpp::Duration.
  QoS &
  liveliness_lease_duration(const rclcpp::Duration & liveliness_lease_duration);

  /// Set the avoid_ros_namespace_conventions setting.
  QoS &
  avoid_ros_namespace_conventions(bool avoid_ros_namespace_conventions);

private:
  rmw_qos_profile_t rmw_qos_profile_;
};

/// Check if two QoS profiles are exactly equal in all policy values.
RCLCPP_PUBLIC
bool operator==(const QoS & left, const QoS & right);
RCLCPP_PUBLIC
bool operator!=(const QoS & left, const QoS & right);

/**
 * Sensor Data QoS class
 *    - History: Keep last,
 *    - Depth: 5,
 *    - Reliability: Best effort,
 *    - Durability: Volatile,
 *    - Deadline: Default,
 *    - Lifespan: Default,
 *    - Liveliness: System default,
 *    - Liveliness lease duration: default,
 *    - avoid ros namespace conventions: false
 */
class RCLCPP_PUBLIC SensorDataQoS : public QoS
{
public:
  explicit
  SensorDataQoS(
    const QoSInitialization & qos_initialization = (
      QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)
  ));
};

/**
 * Parameters QoS class
 *    - History: Keep last,
 *    - Depth: 1000,
 *    - Reliability: Reliable,
 *    - Durability: Volatile,
 *    - Deadline: Default,
 *    - Lifespan: Default,
 *    - Liveliness: System default,
 *    - Liveliness lease duration: default,
 *    - Avoid ros namespace conventions: false
 */
class RCLCPP_PUBLIC ParametersQoS : public QoS
{
public:
  explicit
  ParametersQoS(
    const QoSInitialization & qos_initialization = (
      QoSInitialization::from_rmw(rmw_qos_profile_parameters)
  ));
};

/**
 * Services QoS class
 *    - History: Keep last,
 *    - Depth: 10,
 *    - Reliability: Reliable,
 *    - Durability: Volatile,
 *    - Deadline: Default,
 *    - Lifespan: Default,
 *    - Liveliness: System default,
 *    - Liveliness lease duration: default,
 *    - Avoid ros namespace conventions: false
 */
class RCLCPP_PUBLIC ServicesQoS : public QoS
{
public:
  explicit
  ServicesQoS(
    const QoSInitialization & qos_initialization = (
      QoSInitialization::from_rmw(rmw_qos_profile_services_default)
  ));
};

/**
 * Parameter events QoS class
 *    - History: Keep last,
 *    - Depth: 1000,
 *    - Reliability: Reliable,
 *    - Durability: Volatile,
 *    - Deadline: Default,
 *    - Lifespan: Default,
 *    - Liveliness: System default,
 *    - Liveliness lease duration: default,
 *    - Avoid ros namespace conventions: false
 */
class RCLCPP_PUBLIC ParameterEventsQoS : public QoS
{
public:
  explicit
  ParameterEventsQoS(
    const QoSInitialization & qos_initialization = (
      QoSInitialization::from_rmw(rmw_qos_profile_parameter_events)
  ));
};

/**
 * System defaults QoS class
 *    - History: System default,
 *    - Depth: System default,
 *    - Reliability: System default,
 *    - Durability: System default,
 *    - Deadline: Default,
 *    - Lifespan: Default,
 *    - Liveliness: System default,
 *    - Liveliness lease duration: System default,
 *    - Avoid ros namespace conventions: false
 */
class RCLCPP_PUBLIC SystemDefaultsQoS : public QoS
{
public:
  explicit
  SystemDefaultsQoS(
    const QoSInitialization & qos_initialization = (
      QoSInitialization::from_rmw(rmw_qos_profile_system_default)
  ));
};

}  // namespace rclcpp

#endif  // RCLCPP__QOS_HPP_
