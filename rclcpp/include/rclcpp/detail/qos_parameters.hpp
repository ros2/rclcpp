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
#include <type_traits>
#include <vector>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcpputils/pointer_traits.hpp"
#include "rmw/qos_string_conversions.h"

#include "rclcpp/duration.hpp"
#include "rclcpp/node_interfaces/get_node_parameters_interface.hpp"
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
      QosPolicyKind::Depth,
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
      QosPolicyKind::Depth,
      QosPolicyKind::Liveliness,
      QosPolicyKind::LivelinessLeaseDuration,
      QosPolicyKind::Reliability,
    };
  }
};

/// \internal Returns the given `policy` of the profile `qos` converted to a parameter value.
inline
::rclcpp::ParameterValue
get_default_qos_param_value(rclcpp::QosPolicyKind policy, const rclcpp::QoS & qos);

/// \internal Modify the given `policy` in `qos` to be `value`.
inline
void
apply_qos_override(
  rclcpp::QosPolicyKind policy, rclcpp::ParameterValue value, rclcpp::QoS & qos);

inline
rclcpp::ParameterValue
declare_parameter_or_get(
  rclcpp::node_interfaces::NodeParametersInterface & parameters_interface,
  const std::string & param_name,
  rclcpp::ParameterValue param_value,
  rcl_interfaces::msg::ParameterDescriptor descriptor)
{
  try {
    return parameters_interface.declare_parameter(
      param_name, param_value, descriptor);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    return parameters_interface.get_parameter(param_name).get_parameter_value();
  }
}

/// \internal Declare QoS parameters for the given entity.
/**
 * \tparam NodeT Node pointer or reference type.
 * \tparam EntityQosParametersTraits A class with two static methods: `entity_type()` and
 *  `allowed_policies()`. See `PublisherQosParametersTraits` and `SubscriptionQosParametersTraits`.
 * \param options User provided options that indicate if QoS parameter overrides should be
 *  declared or not, which policy can have overrides, and optionally a callback to validate the profile.
 * \param node Parameters will be declared using this node.
 * \param topic_name Name of the topic of the entity.
 * \param default_qos User provided qos. It will be used as a default for the parameters declared.
 * \return qos profile based on the user provided parameter overrides.
 */
template<typename NodeT, typename EntityQosParametersTraits>
std::enable_if_t<
  rclcpp::node_interfaces::has_node_parameters_interface<
    decltype(std::declval<typename rcpputils::remove_pointer<NodeT>::type>())>::value ||
  std::is_same<typename std::decay_t<NodeT>,
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr>::value,
  rclcpp::QoS>
declare_qos_parameters(
  const ::rclcpp::QosOverridingOptions & options,
  NodeT & node,
  const std::string & topic_name,
  const ::rclcpp::QoS & default_qos,
  EntityQosParametersTraits)
{
  auto & parameters_interface = *rclcpp::node_interfaces::get_node_parameters_interface(node);
  std::string param_prefix;
  const auto & id = options.get_id();
  {
    std::ostringstream oss{"qos_overrides.", std::ios::ate};
    oss << topic_name << "." << EntityQosParametersTraits::entity_type();
    if (!id.empty()) {
      oss << "_" << id;
    }
    oss << ".";
    param_prefix = oss.str();
  }
  std::string param_description_suffix;
  {
    std::ostringstream oss{"} for ", std::ios::ate};
    oss << EntityQosParametersTraits::entity_type() << " {" << topic_name << "}";
    if (!id.empty()) {
      oss << " with id {" << id << "}";
    }
    param_description_suffix = oss.str();
  }
  rclcpp::QoS qos = default_qos;
  for (auto policy : EntityQosParametersTraits::allowed_policies()) {
    if (
      std::count(options.get_policy_kinds().begin(), options.get_policy_kinds().end(), policy))
    {
      std::ostringstream param_name{param_prefix, std::ios::ate};
      param_name << qos_policy_kind_to_cstr(policy);
      std::ostringstream param_desciption{"qos policy {", std::ios::ate};
      param_desciption << qos_policy_kind_to_cstr(policy) << param_description_suffix;
      rcl_interfaces::msg::ParameterDescriptor descriptor{};
      descriptor.description = param_desciption.str();
      descriptor.read_only = true;
      auto value = declare_parameter_or_get(
        parameters_interface, param_name.str(),
        get_default_qos_param_value(policy, qos), descriptor);
      ::rclcpp::detail::apply_qos_override(policy, value, qos);
    }
  }
  const auto & validation_callback = options.get_validation_callback();
  if (validation_callback) {
    auto result = validation_callback(qos);
    if (!result.successful) {
      throw rclcpp::exceptions::InvalidQosOverridesException{
              "validation callback failed: " + result.reason};
    }
  }
  return qos;
}

// TODO(ivanpauno): This overload cannot declare the QoS parameters, as a node parameters interface
// was not provided.
template<typename NodeT, typename EntityQosParametersTraits>
std::enable_if_t<
  !(rclcpp::node_interfaces::has_node_parameters_interface<
    decltype(std::declval<typename rcpputils::remove_pointer<NodeT>::type>())>::value ||
  std::is_same<typename std::decay_t<NodeT>,
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr>::value),
  rclcpp::QoS>
declare_qos_parameters(
  const ::rclcpp::QosOverridingOptions & options,
  NodeT &,
  const std::string &,
  const ::rclcpp::QoS & default_qos,
  EntityQosParametersTraits)
{
  if (options.get_policy_kinds().size()) {
    std::runtime_error exc{
      "passed non-default qos overriding options without providing a parameters interface"};
    throw exc;
  }
  return default_qos;
}

/// \internal Helper function to get a rmw qos policy value from a string.
#define RCLCPP_DETAIL_APPLY_QOS_OVERRIDE_FROM_PARAMETER_STRING( \
    kind_lower, kind_upper, parameter_value, rclcpp_qos) \
  do { \
    auto policy_string = (parameter_value).get<std::string>(); \
    auto policy_value = rmw_qos_ ## kind_lower ## _policy_from_str(policy_string.c_str()); \
    if (RMW_QOS_POLICY_ ## kind_upper ## _UNKNOWN == policy_value) { \
      throw std::invalid_argument{"unknown QoS policy " #kind_lower " value: " + policy_string}; \
    } \
    ((rclcpp_qos).kind_lower)(policy_value); \
  } while (0)

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
      qos.deadline(::rclcpp::Duration::from_nanoseconds(value.get<int64_t>()));
      break;
    case QosPolicyKind::Durability:
      RCLCPP_DETAIL_APPLY_QOS_OVERRIDE_FROM_PARAMETER_STRING(
        durability, DURABILITY, value, qos);
      break;
    case QosPolicyKind::History:
      RCLCPP_DETAIL_APPLY_QOS_OVERRIDE_FROM_PARAMETER_STRING(
        history, HISTORY, value, qos);
      break;
    case QosPolicyKind::Depth:
      qos.get_rmw_qos_profile().depth = static_cast<size_t>(value.get<int64_t>());
      break;
    case QosPolicyKind::Lifespan:
      qos.lifespan(::rclcpp::Duration::from_nanoseconds(value.get<int64_t>()));
      break;
    case QosPolicyKind::Liveliness:
      RCLCPP_DETAIL_APPLY_QOS_OVERRIDE_FROM_PARAMETER_STRING(
        liveliness, LIVELINESS, value, qos);
      break;
    case QosPolicyKind::LivelinessLeaseDuration:
      qos.liveliness_lease_duration(::rclcpp::Duration::from_nanoseconds(value.get<int64_t>()));
      break;
    case QosPolicyKind::Reliability:
      RCLCPP_DETAIL_APPLY_QOS_OVERRIDE_FROM_PARAMETER_STRING(
        reliability, RELIABILITY, value, qos);
      break;
    default:
      throw std::invalid_argument{"unknown QosPolicyKind"};
  }
}

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

/// \internal Throw an exception if `policy_value_stringified` is NULL.
inline
const char *
check_if_stringified_policy_is_null(const char * policy_value_stringified, QosPolicyKind kind)
{
  if (!policy_value_stringified) {
    std::ostringstream oss{"unknown value for policy kind {", std::ios::ate};
    oss << kind << "}";
    throw std::invalid_argument{oss.str()};
  }
  return policy_value_stringified;
}

inline
::rclcpp::ParameterValue
get_default_qos_param_value(rclcpp::QosPolicyKind kind, const rclcpp::QoS & qos)
{
  using ParameterValue = ::rclcpp::ParameterValue;
  const auto & rmw_qos = qos.get_rmw_qos_profile();
  switch (kind) {
    case QosPolicyKind::AvoidRosNamespaceConventions:
      return ParameterValue(rmw_qos.avoid_ros_namespace_conventions);
    case QosPolicyKind::Deadline:
      return ParameterValue(rmw_duration_to_int64_t(rmw_qos.deadline));
    case QosPolicyKind::Durability:
      return ParameterValue(
        check_if_stringified_policy_is_null(
          rmw_qos_durability_policy_to_str(rmw_qos.durability), kind));
    case QosPolicyKind::History:
      return ParameterValue(
        check_if_stringified_policy_is_null(
          rmw_qos_history_policy_to_str(rmw_qos.history), kind));
    case QosPolicyKind::Depth:
      return ParameterValue(static_cast<int64_t>(rmw_qos.depth));
    case QosPolicyKind::Lifespan:
      return ParameterValue(rmw_duration_to_int64_t(rmw_qos.lifespan));
    case QosPolicyKind::Liveliness:
      return ParameterValue(
        check_if_stringified_policy_is_null(
          rmw_qos_liveliness_policy_to_str(rmw_qos.liveliness), kind));
    case QosPolicyKind::LivelinessLeaseDuration:
      return ParameterValue(rmw_duration_to_int64_t(rmw_qos.liveliness_lease_duration));
    case QosPolicyKind::Reliability:
      return ParameterValue(
        check_if_stringified_policy_is_null(
          rmw_qos_reliability_policy_to_str(rmw_qos.reliability), kind));
    default:
      throw std::invalid_argument{"unknown QoS policy kind"};
  }
}

}  // namespace detail
}  // namespace rclcpp

#endif  // RCLCPP__DETAIL__QOS_PARAMETERS_HPP_
