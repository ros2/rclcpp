// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/node_interfaces/node_parameters.hpp"

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rclcpp/create_publisher.hpp"
#include "rcutils/logging_macros.h"
#include "rmw/qos_profiles.h"

using rclcpp::node_interfaces::NodeParameters;

NodeParameters::NodeParameters(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
  const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services,
  bool use_intra_process,
  bool start_parameter_services)
{
  using MessageT = rcl_interfaces::msg::ParameterEvent;
  using PublisherT = rclcpp::Publisher<MessageT>;
  using AllocatorT = std::allocator<void>;
  // TODO(wjwwood): expose this allocator through the Parameter interface.
  auto allocator = std::make_shared<AllocatorT>();

  if (start_parameter_services) {
    parameter_service_ = std::make_shared<ParameterService>(node_base, node_services, this);
  }

  events_publisher_ = rclcpp::create_publisher<MessageT, AllocatorT, PublisherT>(
    node_topics.get(),
    "parameter_events",
    rmw_qos_profile_parameter_events,
    use_intra_process,
    allocator);
}

NodeParameters::~NodeParameters()
{}

std::vector<rcl_interfaces::msg::SetParametersResult>
NodeParameters::set_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::vector<rcl_interfaces::msg::SetParametersResult> results;
  for (auto p : parameters) {
    auto result = set_parameters_atomically({{p}});
    results.push_back(result);
  }
  return results;
}

rcl_interfaces::msg::SetParametersResult
NodeParameters::set_parameters_atomically(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::map<std::string, rclcpp::Parameter> tmp_map;
  auto parameter_event = std::make_shared<rcl_interfaces::msg::ParameterEvent>();

  // TODO(jacquelinekay): handle parameter constraints
  rcl_interfaces::msg::SetParametersResult result;
  if (parameters_callback_) {
    result = parameters_callback_(parameters);
  } else {
    result.successful = true;
  }

  if (!result.successful) {
    return result;
  }

  for (auto p : parameters) {
    if (parameters_.find(p.get_name()) == parameters_.end()) {
      if (p.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
        // case: parameter not set before, and input is something other than "NOT_SET"
        parameter_event->new_parameters.push_back(p.to_parameter_msg());
      }
    } else if (p.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
      // case: parameter was set before, and input is something other than "NOT_SET"
      parameter_event->changed_parameters.push_back(p.to_parameter_msg());
    } else {
      // case: parameter was set before, and input is "NOT_SET"
      // therefore we will "unset" the previously set parameter
      // it is not necessary to erase the parameter from parameters_
      // because the new value for this key (p.get_name()) will be a
      // Parameter with type "NOT_SET"
      parameter_event->deleted_parameters.push_back(p.to_parameter_msg());
    }
    tmp_map[p.get_name()] = p;
  }
  // std::map::insert will not overwrite elements, so we'll keep the new
  // ones and add only those that already exist in the Node's internal map
  tmp_map.insert(parameters_.begin(), parameters_.end());
  std::swap(tmp_map, parameters_);

  events_publisher_->publish(parameter_event);

  return result;
}

std::vector<rclcpp::Parameter>
NodeParameters::get_parameters(const std::vector<std::string> & names) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<rclcpp::Parameter> results;

  for (auto & name : names) {
    if (std::any_of(parameters_.cbegin(), parameters_.cend(),
      [&name](const std::pair<std::string, rclcpp::Parameter> & kv) {
        return name == kv.first;
      }))
    {
      results.push_back(parameters_.at(name));
    }
  }
  return results;
}

rclcpp::Parameter
NodeParameters::get_parameter(const std::string & name) const
{
  rclcpp::Parameter parameter;

  if (get_parameter(name, parameter)) {
    return parameter;
  } else {
    throw std::out_of_range("Parameter '" + name + "' not set");
  }
}

bool
NodeParameters::get_parameter(
  const std::string & name,
  rclcpp::Parameter & parameter) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (parameters_.count(name)) {
    parameter = parameters_.at(name);
    return true;
  } else {
    return false;
  }
}

std::vector<rcl_interfaces::msg::ParameterDescriptor>
NodeParameters::describe_parameters(const std::vector<std::string> & names) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<rcl_interfaces::msg::ParameterDescriptor> results;
  for (auto & kv : parameters_) {
    if (std::any_of(names.cbegin(), names.cend(), [&kv](const std::string & name) {
        return name == kv.first;
      }))
    {
      rcl_interfaces::msg::ParameterDescriptor parameter_descriptor;
      parameter_descriptor.name = kv.first;
      parameter_descriptor.type = kv.second.get_type();
      results.push_back(parameter_descriptor);
    }
  }
  return results;
}

std::vector<uint8_t>
NodeParameters::get_parameter_types(const std::vector<std::string> & names) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<uint8_t> results;
  for (auto & kv : parameters_) {
    if (std::any_of(names.cbegin(), names.cend(), [&kv](const std::string & name) {
        return name == kv.first;
      }))
    {
      results.push_back(kv.second.get_type());
    } else {
      results.push_back(rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET);
    }
  }
  return results;
}

rcl_interfaces::msg::ListParametersResult
NodeParameters::list_parameters(const std::vector<std::string> & prefixes, uint64_t depth) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  rcl_interfaces::msg::ListParametersResult result;

  // TODO(mikaelarguedas) define parameter separator different from "/" to avoid ambiguity
  // using "." for now
  const char * separator = ".";
  for (auto & kv : parameters_) {
    bool get_all = (prefixes.size() == 0) &&
      ((depth == rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE) ||
      (static_cast<uint64_t>(std::count(kv.first.begin(), kv.first.end(), *separator)) < depth));
    bool prefix_matches = std::any_of(prefixes.cbegin(), prefixes.cend(),
        [&kv, &depth, &separator](const std::string & prefix) {
          if (kv.first == prefix) {
            return true;
          } else if (kv.first.find(prefix + separator) == 0) {
            size_t length = prefix.length();
            std::string substr = kv.first.substr(length);
            // Cast as unsigned integer to avoid warning
            return (depth == rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE) ||
            (static_cast<uint64_t>(std::count(substr.begin(), substr.end(), *separator)) < depth);
          }
          return false;
        });
    if (get_all || prefix_matches) {
      result.names.push_back(kv.first);
      size_t last_separator = kv.first.find_last_of(separator);
      if (std::string::npos != last_separator) {
        std::string prefix = kv.first.substr(0, last_separator);
        if (std::find(result.prefixes.cbegin(), result.prefixes.cend(), prefix) ==
          result.prefixes.cend())
        {
          result.prefixes.push_back(prefix);
        }
      }
    }
  }
  return result;
}

void
NodeParameters::register_param_change_callback(ParametersCallbackFunction callback)
{
  if (parameters_callback_) {
    RCUTILS_LOG_WARN("param_change_callback already registered, "
      "overwriting previous callback")
  }
  parameters_callback_ = callback;
}
