// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__QOS_OVERRIDING_OPTIONS_HPP_
#define RCLCPP__QOS_OVERRIDING_OPTIONS_HPP_

#include <functional>
#include <initializer_list>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/qos.hpp"

namespace rclcpp
{

enum class QosPolicyKind
{
  AvoidRosNamespaceConventions,
  Deadline,
  Durability,
  History,
  HistoryDepth,
  Lifespan,
  Liveliness,
  LivelinessLeaseDuration,
  Reliability,
};

const char *
qos_policy_kind_to_cstr(const QosPolicyKind & qpk);

std::ostream &
operator<<(std::ostream & os, const QosPolicyKind & qpk);

using QosCallback = std::function<bool (const rclcpp::QoS &)>;

namespace detail
{
// forward declare
template<typename T>
class QosParameters;
}

/// Options that are passed in subscription/publisher constructor to specify QoSConfigurability.
/**
 * TODO: Write nice docs here.
 */
struct QosOverridingOptions
{
  /// Id of the entity requesting to create parameters.
  std::string id;
  /// Policy kinds that are allowed to be reconfigured.
  std::vector<QosPolicyKind> qos_policy_kinds;
  /// Validation callback that will be called to verify the profile.
  QosCallback validation_callback;

  /// Construct using default overriding options.
  /**
   * \param declare_default_parameters if `true`, the default set of qos that can be
   *   reconfigured will be declared. If `false`, qos aren't reconfigurable.
   * \param id id of the entity.
   */
  explicit QosOverridingOptions(bool declare_default_parameters = false, std::string id = {});

  /// Construct passing a list of qos policies that can be overriden.
  /**
   * This constructor is implicit, e.g.:
   * ```cpp
   * node->create_publisher(
   *   "topic_name",
   *   default_qos_profile,
   *   {{QosPolicyKind::Reliability}, "my_id"});
   * ```
   * \param policy_kinds list of policy kinds that will be reconfigurable.
   * \param id id of the entity.
   */

  QosOverridingOptions(std::initializer_list<QosPolicyKind> policy_kinds, std::string id = {});
  /// Construct passing a list of qos policies that and a verification callback.
  /**
   * This constructor is implicit, e.g.:
   * ```cpp
   * node->create_publisher(
   *   "topic_name",
   *   default_qos_profile,
   *   {
   *     {QosPolicyKind::Reliability},
   *     [] (auto && qos) {return check_qos_validity(qos)},
   *     "my_id"
   *   });
   * ```
   * \param policy_kinds list of policy kinds that will be reconfigurable.
   * \param validation_callback callbak that will be called to validate the validity of
   *   the qos profile set by the user.
   * \param id id of the entity.
   */
  QosOverridingOptions(
    std::initializer_list<QosPolicyKind> policy_kinds,
    QosCallback validation_callback,
    std::string id = {});

  /// Construct using default overriding options and passing a validation callback.
  /**
   * This constructor is implicit, e.g.:
   * ```cpp
   * node->create_publisher(
   *   "topic_name",
   *   default_qos_profile,
   *   {
   *     [] (auto && qos) {return check_qos_validity(qos)},
   *     "my_id"
   *   });
   * ```
   * \param validation_callback callbak that will be called to validate the validity of
   *   the qos profile set by the user.
   * \param id id of the entity.
   */
  QosOverridingOptions(QosCallback validation_callback, std::string id = {});  // NOLINT, implicit
};

}  // namespace rclcpp

#endif  // RCLCPP__QOS_OVERRIDING_OPTIONS_HPP_
