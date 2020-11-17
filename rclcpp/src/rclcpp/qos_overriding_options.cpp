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
#include <vector>

#include "rmw/qos_policy_kind.h"
#include "rmw/qos_string_conversions.h"

namespace rclcpp
{

const char *
qos_policy_kind_to_cstr(const QosPolicyKind & qpk)
{
  const char * ret = rmw_qos_policy_kind_to_str(static_cast<rmw_qos_policy_kind_t>(qpk));
  if (!ret) {
    throw std::invalid_argument{"unknown QoS policy kind"};
  }
  return ret;
}

std::ostream &
operator<<(std::ostream & oss, const QosPolicyKind & qpk)
{
  return oss << qos_policy_kind_to_cstr(qpk);
}

static std::initializer_list<QosPolicyKind> kDefaultPolicies =
{QosPolicyKind::History, QosPolicyKind::Depth, QosPolicyKind::Reliability};

QosOverridingOptions::QosOverridingOptions(
  std::initializer_list<QosPolicyKind> policy_kinds,
  QosCallback validation_callback,
  std::string id)
: id_{std::move(id)},
  policy_kinds_{policy_kinds},
  validation_callback_{std::move(validation_callback)}
{}

QosOverridingOptions
QosOverridingOptions::with_default_policies(
  QosCallback validation_callback,
  std::string id)
{
  return QosOverridingOptions{kDefaultPolicies, validation_callback, id};
}

const std::string &
QosOverridingOptions::get_id() const
{
  return id_;
}

const std::vector<QosPolicyKind> &
QosOverridingOptions::get_policy_kinds() const
{
  return policy_kinds_;
}

const QosCallback &
QosOverridingOptions::get_validation_callback() const
{
  return validation_callback_;
}

}  // namespace rclcpp
