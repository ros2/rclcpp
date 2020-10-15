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

#ifndef RCLCPP__DETAIL__QOS_PARAMETERS_HPP_
#define RCLCPP__DETAIL__QOS_PARAMETERS_HPP_

#include <algorithm>
#include <array>
#include <functional>
#include <initializer_list>
#include <map>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"

#include "rclcpp/duration.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/qos_overriding_options.hpp"

namespace rclcpp
{
namespace detail
{

/// \internal Trait used to specialize `declare_qos_parameters()` for publishers.
struct PublisherQosParametersTraits
{
  static constexpr const char * entity_type() {return "publisher";}
  static constexpr auto allowed_policies()
  {
    return std::array<::rclcpp::QosPolicyKind, 9> {
      QosPolicyKind::AvoidRosNamespaceConventions,
      QosPolicyKind::Deadline,
      QosPolicyKind::Durability,
      QosPolicyKind::History,
      QosPolicyKind::HistoryDepth,
      QosPolicyKind::Lifespan,
      QosPolicyKind::Liveliness,
      QosPolicyKind::LivelinessLeaseDuration,
      QosPolicyKind::Reliability,
    };
  }
};

/// \internal Trait used to specialize `declare_qos_parameters()` for subscriptions.
struct SubscriptionQosParametersTraits
{
  static constexpr const char * entity_type() {return "subscription";}
  static constexpr auto allowed_policies()
  {
    return std::array<::rclcpp::QosPolicyKind, 8> {
      QosPolicyKind::AvoidRosNamespaceConventions,
      QosPolicyKind::Deadline,
      QosPolicyKind::Durability,
      QosPolicyKind::History,
      QosPolicyKind::HistoryDepth,
      QosPolicyKind::Liveliness,
      QosPolicyKind::LivelinessLeaseDuration,
      QosPolicyKind::Reliability,
    };
  }
};

/// \internal Declare qos parameters for the given entity.
/**
 * \tparam EntityQosParametersTraits A class with two static methods: `entity_type()` and
 *  `allowed_policies()`. See `PublisherQosParametersTraits` and `SubscriptionQosParametersTraits`.
 * \param options User provided options that indicate if qos parameter overrides should be
 *  declared or not, which policy can have overrides, and optionally a callback to validate the profile.
 * \param parameters_interface Parameters will be declared through this interface.
 * \param topic_name Name of the topic of the entity.
 * \param qos User provided qos. It will be used as a default for the parameters declared,
 *  and then overriden with the final parameter overrides.
 */
template<typename EntityQosParametersTraits>
inline
void declare_qos_parameters(
  const ::rclcpp::QosOverridingOptions & options,
  ::rclcpp::node_interfaces::NodeParametersInterface & parameters_interface,
  const std::string & topic_name,
  ::rclcpp::QoS & qos,
  EntityQosParametersTraits);

/// \internal Same as `declare_qos_parameters()` for a `Publisher`.
inline
void
declare_publisher_qos_parameters(
  const ::rclcpp::QosOverridingOptions & options,
  ::rclcpp::node_interfaces::NodeParametersInterface & parameters_interface,
  const std::string & topic_name,
  ::rclcpp::QoS & qos)
{
  declare_qos_parameters(
    options, parameters_interface, topic_name, qos, PublisherQosParametersTraits{});
}

/// \internal Same as `declare_qos_parameters()` for a `Subscription`.
inline
void
declare_subscription_qos_parameters(
  const ::rclcpp::QosOverridingOptions & options,
  ::rclcpp::node_interfaces::NodeParametersInterface & parameters_interface,
  const std::string & topic_name,
  ::rclcpp::QoS & qos)
{
  declare_qos_parameters(
    options, parameters_interface, topic_name, qos, SubscriptionQosParametersTraits{});
}

/// \internal Returns the given `policy` of the profile `qos` converted to a parameter value.
inline
::rclcpp::ParameterValue
get_default_qos_param_value(rclcpp::QosPolicyKind policy, const rclcpp::QoS & qos);

/// \internal Modify the given `policy` in `qos` to be `value`.
inline
void
apply_qos_override(
  rclcpp::QosPolicyKind policy, rclcpp::ParameterValue value, rclcpp::QoS & qos);

template<typename EntityQosParametersTraits>
inline
void declare_qos_parameters(
  const ::rclcpp::QosOverridingOptions & options,
  ::rclcpp::node_interfaces::NodeParametersInterface & parameters_interface,
  const std::string & topic_name,
  ::rclcpp::QoS & qos,
  EntityQosParametersTraits)
{
  std::string param_prefix;
  {
    std::ostringstream oss{"qos_overrides.", std::ios::ate};
    oss << topic_name << "." << EntityQosParametersTraits::entity_type();
    if (!options.id.empty()) {
      oss << "_" << options.id;
    }
    oss << ".";
    param_prefix = oss.str();
  }
  std::string param_description_suffix;
  {
    std::ostringstream oss{"} for ", std::ios::ate};
    oss << EntityQosParametersTraits::entity_type() << " {" << topic_name << "}";
    if (!options.id.empty()) {
      oss << " with id {" << options.id << "}";
    }
    param_description_suffix = oss.str();
  }
  for (auto policy : EntityQosParametersTraits::allowed_policies()) {
    if (
      std::count(options.policy_kinds.begin(), options.policy_kinds.end(), policy))
    {
      std::ostringstream param_name{param_prefix, std::ios::ate};
      param_name << qos_policy_kind_to_cstr(policy);
      std::ostringstream param_desciption{"qos policy {", std::ios::ate};
      param_desciption << qos_policy_kind_to_cstr(policy) << param_description_suffix;
      rcl_interfaces::msg::ParameterDescriptor descriptor{};
      descriptor.description = param_desciption.str();
      descriptor.read_only = true;
      auto value = parameters_interface.declare_parameter(
        param_name.str(), get_default_qos_param_value(policy, qos), descriptor);
      ::rclcpp::detail::apply_qos_override(policy, value, qos);
    }
  }
  if (options.validation_callback && !options.validation_callback(qos)) {
    throw rclcpp::exceptions::InvalidQosOverridesException{"validation callback failed"};
  }
}

/// \internal Get the `rmw_qos_*_policy_t` value from a given `str`, or raise a runtime_error.
template<typename RetT>
RetT
string_to_policy(const std::string & str);

template<>
inline
rmw_qos_durability_policy_t
string_to_policy(const std::string & str);

template<>
inline
rmw_qos_liveliness_policy_t
string_to_policy(const std::string & str);

template<>
inline
rmw_qos_history_policy_t
string_to_policy(const std::string & str);

template<>
inline
rmw_qos_reliability_policy_t
string_to_policy(const std::string & str);

inline
void
apply_qos_override(
  rclcpp::QosPolicyKind policy, rclcpp::ParameterValue value, rclcpp::QoS & qos)
{
  switch (policy) {
    case QosPolicyKind::AvoidRosNamespaceConventions:
      qos.avoid_ros_namespace_conventions(value.get<bool>());
      break;
    case QosPolicyKind::Deadline:
      qos.deadline(::rclcpp::Duration(value.get<int64_t>()));
      break;
    case QosPolicyKind::Durability:
      qos.durability(string_to_policy<rmw_qos_durability_policy_t>(value.get<std::string>()));
      break;
    case QosPolicyKind::History:
      qos.history(string_to_policy<rmw_qos_history_policy_t>(value.get<std::string>()));
      break;
    case QosPolicyKind::HistoryDepth:
      qos.get_rmw_qos_profile().depth = static_cast<size_t>(value.get<int64_t>());
      break;
    case QosPolicyKind::Lifespan:
      qos.lifespan(::rclcpp::Duration(value.get<int64_t>()));
      break;
    case QosPolicyKind::Liveliness:
      qos.liveliness(string_to_policy<rmw_qos_liveliness_policy_t>(value.get<std::string>()));
      break;
    case QosPolicyKind::LivelinessLeaseDuration:
      qos.liveliness_lease_duration(::rclcpp::Duration(value.get<int64_t>()));
      break;
    case QosPolicyKind::Reliability:
      qos.reliability(string_to_policy<rmw_qos_reliability_policy_t>(value.get<std::string>()));
      break;
    default:
      throw std::runtime_error{"unknown QosPolicyKind"};
  }
}

/// Convert the given policy to the corresponding string representation.
inline
const char *
policy_to_cstring(rmw_qos_durability_policy_t durability);

/// Convert the given policy to the corresponding string representation.
inline
const char *
policy_to_cstring(rmw_qos_history_policy_t history);

/// Convert the given policy to the corresponding string representation.
inline
const char *
policy_to_cstring(rmw_qos_liveliness_policy_t liveliness);

/// Convert the given policy to the corresponding string representation.
inline
const char *
policy_to_cstring(rmw_qos_reliability_policy_t reliability);

/// Convert `rmw_time_t` to `int64_t` that can be used as a parameter value.
inline
int64_t
rmw_duration_to_int64_t(rmw_time_t rmw_duration)
{
  return ::rclcpp::Duration(
    static_cast<int32_t>(rmw_duration.sec),
    static_cast<uint32_t>(rmw_duration.nsec)
  ).nanoseconds();
}

inline
::rclcpp::ParameterValue
get_default_qos_param_value(rclcpp::QosPolicyKind qpk, const rclcpp::QoS & qos)
{
  using ParameterValue = ::rclcpp::ParameterValue;
  const auto & rmw_qos = qos.get_rmw_qos_profile();
  switch (qpk) {
    case QosPolicyKind::AvoidRosNamespaceConventions:
      return ParameterValue(rmw_qos.avoid_ros_namespace_conventions);
    case QosPolicyKind::Deadline:
      return ParameterValue(rmw_duration_to_int64_t(rmw_qos.deadline));
    case QosPolicyKind::Durability:
      return ParameterValue(policy_to_cstring(rmw_qos.durability));
    case QosPolicyKind::History:
      return ParameterValue(policy_to_cstring(rmw_qos.history));
    case QosPolicyKind::HistoryDepth:
      return ParameterValue(static_cast<int64_t>(rmw_qos.depth));
    case QosPolicyKind::Lifespan:
      return ParameterValue(rmw_duration_to_int64_t(rmw_qos.lifespan));
    case QosPolicyKind::Liveliness:
      return ParameterValue(policy_to_cstring(rmw_qos.liveliness));
    case QosPolicyKind::LivelinessLeaseDuration:
      return ParameterValue(rmw_duration_to_int64_t(rmw_qos.liveliness_lease_duration));
    case QosPolicyKind::Reliability:
      return ParameterValue(policy_to_cstring(rmw_qos.reliability));
    default:
      throw std::invalid_argument{"unknown qos policy kind"};
  }
}

// TODO(ivanpauno): All `policy_to_cstring()` and `string_to_policy()` functions should be
// a wrapper of a `rcl` implemented function.
inline
const char *
policy_to_cstring(rmw_qos_durability_policy_t durability)
{
  switch (durability) {
    case RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT:
      return "system_default";
    case RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:
      return "transient_local";
    case RMW_QOS_POLICY_DURABILITY_VOLATILE:
      return "volatile";
    case RMW_QOS_POLICY_DURABILITY_UNKNOWN:  // fallthrough
    default:
      throw std::invalid_argument{"unknown durability qos policy value"};
  }
}

inline
const char *
policy_to_cstring(rmw_qos_history_policy_t history)
{
  switch (history) {
    case RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT:
      return "system_default";
    case RMW_QOS_POLICY_HISTORY_KEEP_LAST:
      return "keep_last";
    case RMW_QOS_POLICY_HISTORY_KEEP_ALL:
      return "keep_all";
    case RMW_QOS_POLICY_HISTORY_UNKNOWN:  // fallthrough
    default:
      throw std::invalid_argument{"unknown history qos policy value"};
  }
}

inline
const char *
policy_to_cstring(rmw_qos_liveliness_policy_t liveliness)
{
  switch (liveliness) {
    case RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT:
      return "system_default";
    case RMW_QOS_POLICY_LIVELINESS_AUTOMATIC:
      return "automatic";
    case RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC:
      return "manual_by_topic";
    case RMW_QOS_POLICY_LIVELINESS_UNKNOWN:  // fallthrough
    default:
      throw std::invalid_argument{"unknown liveliness qos policy value"};
  }
}

inline
const char *
policy_to_cstring(rmw_qos_reliability_policy_t reliability)
{
  switch (reliability) {
    case RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT:
      return "system_default";
    case RMW_QOS_POLICY_RELIABILITY_RELIABLE:
      return "reliable";
    case RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT:
      return "best_effort";
    case RMW_QOS_POLICY_RELIABILITY_UNKNOWN:  // fallthrough
    default:
      throw std::invalid_argument{"unknown reliability qos policy value"};
  }
}

template<>
inline
rmw_qos_durability_policy_t
string_to_policy(const std::string & str)
{
  if ("system_default" == str) {
    return RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
  }
  if ("transient_local" == str) {
    return RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  }
  if ("volatile" == str) {
    return RMW_QOS_POLICY_DURABILITY_VOLATILE;
  }
  throw std::invalid_argument{"unknown durability qos policy string"};
}

template<>
inline
rmw_qos_history_policy_t
string_to_policy(const std::string & str)
{
  if ("system_default" == str) {
    return RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT;
  }
  if ("keep_last" == str) {
    return RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  }
  if ("keep_all" == str) {
    return RMW_QOS_POLICY_HISTORY_KEEP_ALL;
  }
  throw std::invalid_argument{"unknown history qos policy string"};
}

template<>
inline
rmw_qos_liveliness_policy_t
string_to_policy(const std::string & str)
{
  if ("system_default" == str) {
    return RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT;
  }
  if ("automatic" == str) {
    return RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
  }
  if ("manual_by_topic" == str) {
    return RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
  }
  throw std::invalid_argument{"unknown liveliness qos policy string"};
}

template<>
inline
rmw_qos_reliability_policy_t
string_to_policy(const std::string & str)
{
  if ("system_default" == str) {
    return RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
  }
  if ("reliable" == str) {
    return RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  }
  if ("best_effort" == str) {
    return RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  }
  throw std::invalid_argument{"unknown reliability qos policy string"};
}

}  // namespace detail
}  // namespace rclcpp

#endif  // RCLCPP__DETAIL__QOS_PARAMETERS_HPP_
