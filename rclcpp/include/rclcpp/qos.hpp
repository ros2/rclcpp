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
#include "rclcpp/exceptions.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rcl/logging_rosout.h"
#include "rmw/incompatible_qos_events_statuses.h"
#include "rmw/qos_profiles.h"
#include "rmw/types.h"

namespace rclcpp
{

RCLCPP_PUBLIC
std::string qos_policy_name_from_kind(rmw_qos_policy_kind_t policy_kind);

enum class HistoryPolicy
{
  KeepLast = RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  KeepAll = RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  SystemDefault = RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT,
  Unknown = RMW_QOS_POLICY_HISTORY_UNKNOWN,
};

enum class ReliabilityPolicy
{
  BestEffort = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
  Reliable = RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  SystemDefault = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
  Unknown = RMW_QOS_POLICY_RELIABILITY_UNKNOWN,
};

enum class DurabilityPolicy
{
  Volatile = RMW_QOS_POLICY_DURABILITY_VOLATILE,
  TransientLocal = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
  SystemDefault = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT,
  Unknown = RMW_QOS_POLICY_DURABILITY_UNKNOWN,
};

enum class LivelinessPolicy
{
  Automatic = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
  ManualByTopic = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC,
  SystemDefault = RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  Unknown = RMW_QOS_POLICY_LIVELINESS_UNKNOWN,
};

enum class QoSCompatibility
{
  Ok = RMW_QOS_COMPATIBILITY_OK,
  Warning = RMW_QOS_COMPATIBILITY_WARNING,
  Error = RMW_QOS_COMPATIBILITY_ERROR,
};

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
/**
 * Quality of Service settings control the behavior of publishers, subscriptions,
 * and other entities, and includes things like how data is sent or resent,
 * how data is buffered on the publishing and subscribing side, and other things.
 * See:
 *   <a href="https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html">
 *     https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html
 *   </a>
 */
class RCLCPP_PUBLIC QoS
{
public:
  /// Create a QoS by specifying only the history policy and history depth.
  /**
   * When using the default initial profile, the defaults will include:
   *
   *   - \link rclcpp::ReliabilityPolicy::Reliable ReliabilityPolicy::Reliable\endlink
   *   - \link rclcpp::DurabilityPolicy::Volatile DurabilityPolicy::Volatile\endlink
   *
   * See rmw_qos_profile_default for a full list of default settings.
   * If some other rmw_qos_profile_t is passed to initial_profile, then the defaults will derive from
   * that profile instead.
   *
   * \param[in] qos_initialization Specifies history policy and history depth.
   * \param[in] initial_profile The rmw_qos_profile_t instance on which to base the default settings.
   */
  explicit
  QoS(
    const QoSInitialization & qos_initialization,
    const rmw_qos_profile_t & initial_profile = rmw_qos_profile_default);

  /// Conversion constructor to ease construction in the common case of just specifying depth.
  /**
   * This is a convenience constructor that calls QoS(KeepLast(history_depth)).
   *
   * \param[in] history_depth How many messages can be queued when publishing
   *   with a Publisher, or how many messages can be queued before being replaced
   *   by a Subscription.
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
  history(HistoryPolicy history);

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

  /// Set the reliability setting.
  QoS &
  reliability(ReliabilityPolicy reliability);

  /// Set the reliability setting to reliable.
  QoS &
  reliable();

  /// Set the reliability setting to best effort.
  QoS &
  best_effort();

  /// Set the durability setting.
  QoS &
  durability(rmw_qos_durability_policy_t durability);

  /// Set the durability setting.
  QoS &
  durability(DurabilityPolicy durability);

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

  /// Set the liveliness setting.
  QoS &
  liveliness(LivelinessPolicy liveliness);

  /// Set the liveliness_lease_duration setting.
  QoS &
  liveliness_lease_duration(rmw_time_t liveliness_lease_duration);

  /// Set the liveliness_lease_duration setting, rclcpp::Duration.
  QoS &
  liveliness_lease_duration(const rclcpp::Duration & liveliness_lease_duration);

  /// Set the avoid_ros_namespace_conventions setting.
  QoS &
  avoid_ros_namespace_conventions(bool avoid_ros_namespace_conventions);

  /// Get the history qos policy.
  HistoryPolicy
  history() const;

  /// Get the history depth.
  size_t
  depth() const;

  /// Get the reliability policy.
  ReliabilityPolicy
  reliability() const;

  /// Get the durability policy.
  DurabilityPolicy
  durability() const;

  /// Get the deadline duration setting.
  rclcpp::Duration
  deadline() const;

  /// Get the lifespan duration setting.
  rclcpp::Duration
  lifespan() const;

  /// Get the liveliness policy.
  LivelinessPolicy
  liveliness() const;

  /// Get the liveliness lease duration setting.
  rclcpp::Duration
  liveliness_lease_duration() const;

  /// Get the `avoid ros namespace convention` setting.
  bool
  avoid_ros_namespace_conventions() const;

private:
  rmw_qos_profile_t rmw_qos_profile_;
};

/// Check if two QoS profiles are exactly equal in all policy values.
RCLCPP_PUBLIC
bool operator==(const QoS & left, const QoS & right);
RCLCPP_PUBLIC
bool operator!=(const QoS & left, const QoS & right);

/// Result type for checking QoS compatibility
/**
 * \see rclcpp::qos_check_compatible()
 */
struct QoSCheckCompatibleResult
{
  /// Compatibility result.
  QoSCompatibility compatibility;

  /// Reason for a (possible) incompatibility.
  /**
   * Set if compatiblity is QoSCompatibility::Warning or QoSCompatiblity::Error.
   * Not set if the QoS profiles are compatible.
   */
  std::string reason;
};

/// Check if two QoS profiles are compatible.
/**
 * Two QoS profiles are compatible if a publisher and subcription
 * using the QoS policies can communicate with each other.
 *
 * If any policies have value "system default" or "unknown" then it is possible that
 * compatiblity cannot be determined.
 * In this case, the value QoSCompatility::Warning is set as part of
 * the returned structure.
 *
 * Example usage:
 *
 * ```cpp
 * rclcpp::QoSCheckCompatibleResult result = rclcpp::qos_check_compatible(
 *   publisher_qos, subscription_qos);
 * if (rclcpp::QoSCompatibility::Error != result.compatibility) {
 *   // QoS not compatible ...
 *   // result.reason contains info about the incompatibility
 * } else if (rclcpp::QoSCompatibility::Warning != result.compatibility) {
 *   // QoS may not be compatible ...
 *   // result.reason contains info about the possible incompatibility
 * }
 * ```
 *
 * \param[in] publisher_qos: The QoS profile for a publisher.
 * \param[in] subscription_qos: The QoS profile for a subscription.
 * \return Struct with compatiblity set to QoSCompatibility::Ok if the QoS profiles are
 *   compatible, or
 * \return Struct with compatibility set to QoSCompatibility::Warning if there is a chance
 *   the QoS profiles are not compatible, or
 * \return Struct with compatibility set to QoSCompatibility::Error if the QoS profiles are
 *   not compatible.
 * \throws rclcpp::exceptions::QoSCheckCompatibilityException if an unexpected error occurs.
 */
RCLCPP_PUBLIC
QoSCheckCompatibleResult
qos_check_compatible(const QoS & publisher_qos, const QoS & subscription_qos);

/**
 * Clock QoS class
 *    - History: Keep last,
 *    - Depth: 1,
 *    - Reliability: Best effort,
 *    - Durability: Volatile,
 *    - Deadline: Default,
 *    - Lifespan: Default,
 *    - Liveliness: System default,
 *    - Liveliness lease duration: default,
 *    - avoid ros namespace conventions: false
 */
class RCLCPP_PUBLIC ClockQoS : public QoS
{
public:
  explicit
  ClockQoS(
    const QoSInitialization & qos_initialization = KeepLast(1));
};

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
 * Rosout QoS class
 *    - History: Keep last,
 *    - Depth: 1000,
 *    - Reliability: Reliable,
 *    - Durability: TRANSIENT_LOCAL,
 *    - Deadline: Default,
 *    - Lifespan: {10, 0},
 *    - Liveliness: System default,
 *    - Liveliness lease duration: default,
 *    - Avoid ros namespace conventions: false
 */
class RCLCPP_PUBLIC RosoutQoS : public QoS
{
public:
  explicit
  RosoutQoS(
    const QoSInitialization & rosout_qos_initialization = (
      QoSInitialization::from_rmw(rcl_qos_profile_rosout_default)
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
