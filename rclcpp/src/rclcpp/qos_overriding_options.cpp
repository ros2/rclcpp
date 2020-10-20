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

#include "rclcpp/qos_overriding_options.hpp"

#include <initializer_list>
#include <ostream>
#include <stdexcept>
#include <string>
#include <utility>

#include "rmw/qos_policy_kind.h"
#include "rmw/qos_string_conversions.h"

namespace rclcpp
{

const char *
qos_policy_kind_to_cstr(const QosPolicyKind & qpk)
{
  const char * ret = rmw_qos_policy_kind_to_str(static_cast<rmw_qos_policy_kind_t>(qpk));
  if (!ret) {
    throw std::invalid_argument{"unknown qos policy kind"};
  }
  return ret;
}

std::ostream &
operator<<(std::ostream & oss, const QosPolicyKind & qpk)
{
  return oss << qos_policy_kind_to_cstr(qpk);
}

#define RCLCPP_QOS_OVERRIDING_OPTIONS_DEFAULT_QOS_POLICIES (std::initializer_list<QosPolicyKind> \
  {QosPolicyKind::History, QosPolicyKind::Depth, QosPolicyKind::Reliability})

QosOverridingOptions::QosOverridingOptions(bool declare_default_parameters, std::string id)
: id{std::move(id)},
  policy_kinds{declare_default_parameters ?
    RCLCPP_QOS_OVERRIDING_OPTIONS_DEFAULT_QOS_POLICIES :
    std::initializer_list<QosPolicyKind>{}}
{}

QosOverridingOptions::QosOverridingOptions(
  std::initializer_list<QosPolicyKind> policy_kinds, std::string id)
: id{std::move(id)},
  policy_kinds{policy_kinds}
{}

QosOverridingOptions::QosOverridingOptions(
  std::initializer_list<QosPolicyKind> policy_kinds,
  QosCallback validation_callback,
  std::string id)
: id{std::move(id)},
  policy_kinds{policy_kinds},
  validation_callback{std::move(validation_callback)}
{}

QosOverridingOptions::QosOverridingOptions(
  QosCallback validation_callback,
  std::string id)
: id{std::move(id)},
  policy_kinds{RCLCPP_QOS_OVERRIDING_OPTIONS_DEFAULT_QOS_POLICIES},
  validation_callback{std::move(validation_callback)}
{}

}  // namespace rclcpp
