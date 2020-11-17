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
#include "rclcpp/visibility_control.hpp"

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rmw/qos_policy_kind.h"

namespace rclcpp
{

enum class RCLCPP_PUBLIC_TYPE QosPolicyKind
{
  AvoidRosNamespaceConventions = RMW_QOS_POLICY_AVOID_ROS_NAMESPACE_CONVENTIONS,
  Deadline = RMW_QOS_POLICY_DEADLINE,
  Depth = RMW_QOS_POLICY_DEPTH,
  Durability = RMW_QOS_POLICY_DURABILITY,
  History = RMW_QOS_POLICY_HISTORY,
  Lifespan = RMW_QOS_POLICY_LIFESPAN,
  Liveliness = RMW_QOS_POLICY_LIVELINESS,
  LivelinessLeaseDuration = RMW_QOS_POLICY_LIVELINESS_LEASE_DURATION,
  Reliability = RMW_QOS_POLICY_RELIABILITY,
  Invalid = RMW_QOS_POLICY_INVALID,
};

RCLCPP_PUBLIC
const char *
qos_policy_kind_to_cstr(const QosPolicyKind & qpk);

RCLCPP_PUBLIC
std::ostream &
operator<<(std::ostream & os, const QosPolicyKind & qpk);

using QosCallbackResult = rcl_interfaces::msg::SetParametersResult;
using QosCallback = std::function<QosCallbackResult(const rclcpp::QoS &)>;

namespace detail
{
// forward declare
template<typename T>
class QosParameters;
}

/// Options that are passed in subscription/publisher constructor to specify QoSConfigurability.
/**
 * This options struct allows configuring:
 * - Which policy kinds will have declared parameters.
 * - An optional callback, that will be called to validate the final qos profile.
 * - An optional id. In the case that different qos are desired for two publishers/subscriptions in
 *   the same topic, this id will allow disambiguating them.
 *
 * Example parameter file:
 *
 * ```yaml
 * my_node_name:
 *  ros__parameters:
 *    qos_overrides:
 *      /my/topic/name:
 *        publisher:  # publisher without provided id
 *          reliability: reliable
 *          depth: 100
 *        publisher_my_id:  # publisher with `id="my_id"
 *          reliability: reliable
 *          depth: 10
 * ```
 */
class QosOverridingOptions
{
public:
  /// Default constructor, no overrides allowed.
  RCLCPP_PUBLIC
  QosOverridingOptions() = default;

  /// Construct passing a list of QoS policies and a verification callback.
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
  RCLCPP_PUBLIC
  QosOverridingOptions(
    std::initializer_list<QosPolicyKind> policy_kinds,
    QosCallback validation_callback = nullptr,
    std::string id = {});

  RCLCPP_PUBLIC
  const std::string &
  get_id() const;

  RCLCPP_PUBLIC
  const std::vector<QosPolicyKind> &
  get_policy_kinds() const;

  RCLCPP_PUBLIC
  const QosCallback &
  get_validation_callback() const;

  /// Construct passing a list of QoS policies and a verification callback.
  /**
   * Same as `QosOverridingOptions` constructor, but only declares the default policies:
   *
   * History, Depth, Reliability.
   */
  RCLCPP_PUBLIC
  static
  QosOverridingOptions
  with_default_policies(QosCallback validation_callback = nullptr, std::string id = {});

private:
  /// \internal Id of the entity requesting to create parameters.
  std::string id_;
  /// \internal Policy kinds that are allowed to be reconfigured.
  std::vector<QosPolicyKind> policy_kinds_;
  /// \internal Validation callback that will be called to verify the profile.
  QosCallback validation_callback_;
};

}  // namespace rclcpp

#endif  // RCLCPP__QOS_OVERRIDING_OPTIONS_HPP_
