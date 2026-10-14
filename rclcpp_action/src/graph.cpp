// Copyright 2026 TriOrb, Inc.
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

#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "rcl/error_handling.h"
#include "rcl/graph.h"
#include "rcl_action/graph.h"
#include "rcutils/allocator.h"

#include "rclcpp/exceptions.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"

#include "rclcpp_action/graph.hpp"

using rclcpp_action::ActionEndpointInfo;

namespace
{

const rcl_action_endpoint_info_t &
validate_endpoint_info(const rcl_action_endpoint_info_t & info)
{
  if (!info.goal_service_info.node_name ||
    !info.goal_service_info.node_namespace ||
    !info.goal_service_info.service_type)
  {
    throw std::invalid_argument(
            "Constructing ActionEndpointInfo with invalid goal service endpoint info");
  }
  return info;
}

std::vector<ActionEndpointInfo>
convert_to_action_info_list(const rcl_action_endpoint_info_array_t & info_array)
{
  std::vector<ActionEndpointInfo> action_info_list;
  action_info_list.reserve(info_array.size);
  for (size_t i = 0; i < info_array.size; ++i) {
    const rcl_action_endpoint_info_t & info = info_array.info_array[i];
    // rcl_action guarantees the goal service info of each entry is populated,
    // skip malformed entries defensively instead of failing the whole query.
    if (!info.goal_service_info.node_name ||
      !info.goal_service_info.node_namespace ||
      !info.goal_service_info.service_type)
    {
      continue;
    }
    action_info_list.emplace_back(info);
  }
  return action_info_list;
}

template<const char * EndpointType, typename FunctionT>
std::vector<ActionEndpointInfo>
get_info_by_action(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base,
  const std::string & action_name,
  FunctionT rcl_action_get_info_by_action)
{
  auto rcl_node_handle = node_base->get_rcl_node_handle();

  std::string fqdn = rclcpp::expand_topic_or_service_name(
    action_name,
    rcl_node_get_name(rcl_node_handle),
    rcl_node_get_namespace(rcl_node_handle),
    false);    // false = not a service

  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcl_action_endpoint_info_array_t info_array =
    rcl_action_get_zero_initialized_endpoint_info_array();
  rcl_ret_t ret =
    rcl_action_get_info_by_action(rcl_node_handle, &allocator, fqdn.c_str(), &info_array);
  if (RCL_RET_OK != ret) {
    auto error_msg =
      std::string("Failed to get information by action for ") + EndpointType + std::string(":");
    if (RCL_RET_UNSUPPORTED == ret) {
      error_msg += std::string("function not supported by RMW_IMPLEMENTATION");
    } else {
      error_msg += rcl_get_error_string().str;
    }
    rcl_reset_error();
    if (RCL_RET_OK != rcl_action_endpoint_info_array_fini(&info_array, &allocator)) {
      error_msg += std::string(", failed also to cleanup endpoint info array, leaking memory: ") +
        rcl_get_error_string().str;
      rcl_reset_error();
    }
    rclcpp::exceptions::throw_from_rcl_error(ret, error_msg);
  }

  std::vector<ActionEndpointInfo> action_info_list = convert_to_action_info_list(info_array);
  ret = rcl_action_endpoint_info_array_fini(&info_array, &allocator);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "rcl_action_endpoint_info_array_fini failed.");
  }

  return action_info_list;
}

size_t
count_action_entities(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base,
  const std::string & action_name,
  bool count_clients)
{
  auto rcl_node_handle = node_base->get_rcl_node_handle();

  std::string fqdn = rclcpp::expand_topic_or_service_name(
    action_name,
    rcl_node_get_name(rcl_node_handle),
    rcl_node_get_namespace(rcl_node_handle),
    false);    // false = not a service

  size_t count = 0;
  rcl_ret_t ret;
  if (count_clients) {
    ret = rcl_action_count_clients(rcl_node_handle, fqdn.c_str(), &count);
  } else {
    ret = rcl_action_count_servers(rcl_node_handle, fqdn.c_str(), &count);
  }
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(
      ret, std::string("could not count ") + (count_clients ? "action clients" : "action servers"));
  }
  return count;
}

}  // namespace

namespace rclcpp_action
{

ActionEndpointInfo::ActionEndpointInfo(const rcl_action_endpoint_info_t & info)
: goal_service_info_(validate_endpoint_info(info).goal_service_info)
{
  if (info.cancel_service_info.node_name) {
    cancel_service_info_.emplace(info.cancel_service_info);
  }
  if (info.result_service_info.node_name) {
    result_service_info_.emplace(info.result_service_info);
  }
  if (info.feedback_topic_info.node_name) {
    feedback_topic_info_.emplace(info.feedback_topic_info);
  }
  if (info.status_topic_info.node_name) {
    status_topic_info_.emplace(info.status_topic_info);
  }
}

const std::string &
ActionEndpointInfo::node_name() const
{
  return goal_service_info_.node_name();
}

const std::string &
ActionEndpointInfo::node_namespace() const
{
  return goal_service_info_.node_namespace();
}

std::string
ActionEndpointInfo::action_type() const
{
  const std::string & goal_service_type = goal_service_info_.service_type();
  static constexpr char suffix[] = "_SendGoal";
  static constexpr size_t suffix_len = sizeof(suffix) - 1;
  if (goal_service_type.size() > suffix_len &&
    0 == goal_service_type.compare(
      goal_service_type.size() - suffix_len, suffix_len, suffix))
  {
    return goal_service_type.substr(0, goal_service_type.size() - suffix_len);
  }
  return goal_service_type;
}

rclcpp::EndpointType
ActionEndpointInfo::endpoint_type() const
{
  return goal_service_info_.endpoint_type();
}

rclcpp::ServiceEndpointInfo &
ActionEndpointInfo::goal_service_info()
{
  return goal_service_info_;
}

const rclcpp::ServiceEndpointInfo &
ActionEndpointInfo::goal_service_info() const
{
  return goal_service_info_;
}

std::optional<rclcpp::ServiceEndpointInfo> &
ActionEndpointInfo::cancel_service_info()
{
  return cancel_service_info_;
}

const std::optional<rclcpp::ServiceEndpointInfo> &
ActionEndpointInfo::cancel_service_info() const
{
  return cancel_service_info_;
}

std::optional<rclcpp::ServiceEndpointInfo> &
ActionEndpointInfo::result_service_info()
{
  return result_service_info_;
}

const std::optional<rclcpp::ServiceEndpointInfo> &
ActionEndpointInfo::result_service_info() const
{
  return result_service_info_;
}

std::optional<rclcpp::TopicEndpointInfo> &
ActionEndpointInfo::feedback_topic_info()
{
  return feedback_topic_info_;
}

const std::optional<rclcpp::TopicEndpointInfo> &
ActionEndpointInfo::feedback_topic_info() const
{
  return feedback_topic_info_;
}

std::optional<rclcpp::TopicEndpointInfo> &
ActionEndpointInfo::status_topic_info()
{
  return status_topic_info_;
}

const std::optional<rclcpp::TopicEndpointInfo> &
ActionEndpointInfo::status_topic_info() const
{
  return status_topic_info_;
}

static constexpr char kActionClientEndpointTypeName[] = "action clients";
std::vector<ActionEndpointInfo>
get_action_clients_info_by_action(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base,
  const std::string & action_name)
{
  return get_info_by_action<kActionClientEndpointTypeName>(
    node_base, action_name, rcl_action_get_clients_info_by_action);
}

static constexpr char kActionServerEndpointTypeName[] = "action servers";
std::vector<ActionEndpointInfo>
get_action_servers_info_by_action(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base,
  const std::string & action_name)
{
  return get_info_by_action<kActionServerEndpointTypeName>(
    node_base, action_name, rcl_action_get_servers_info_by_action);
}

size_t
count_action_clients(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base,
  const std::string & action_name)
{
  return count_action_entities(node_base, action_name, true);
}

size_t
count_action_servers(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base,
  const std::string & action_name)
{
  return count_action_entities(node_base, action_name, false);
}

}  // namespace rclcpp_action
