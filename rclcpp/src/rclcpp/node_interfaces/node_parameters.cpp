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

#include <rcl_yaml_param_parser/parser.h>

#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rclcpp/create_publisher.hpp"
#include "rclcpp/parameter_map.hpp"
#include "rclcpp/scope_exit.hpp"
#include "rcutils/logging_macros.h"
#include "rmw/qos_profiles.h"

using rclcpp::node_interfaces::NodeParameters;

NodeParameters::NodeParameters(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
  const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services,
  const rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock,
  const std::vector<rclcpp::Parameter> & initial_parameters,
  bool use_intra_process,
  bool start_parameter_services,
  bool start_parameter_event_publisher,
  const rmw_qos_profile_t & parameter_event_qos_profile,
  bool allow_undeclared_parameters)
:
  allow_undeclared_(allow_undeclared_parameters),
  events_publisher_(nullptr),
  node_clock_(node_clock)
{
  using MessageT = rcl_interfaces::msg::ParameterEvent;
  using PublisherT = rclcpp::Publisher<MessageT>;
  using AllocatorT = std::allocator<void>;
  // TODO(wjwwood): expose this allocator through the Parameter interface.
  auto allocator = std::make_shared<AllocatorT>();

  if (start_parameter_services) {
    parameter_service_ = std::make_shared<ParameterService>(node_base, node_services, this);
  }

  if (start_parameter_event_publisher) {
    events_publisher_ = rclcpp::create_publisher<MessageT, AllocatorT, PublisherT>(
      node_topics.get(),
      "parameter_events",
      parameter_event_qos_profile,
      use_intra_process,
      allocator);
  }

  // Get the node options
  const rcl_node_t * node = node_base->get_rcl_node_handle();
  if (nullptr == node) {
    throw std::runtime_error("Need valid node handle in NodeParameters");
  }
  const rcl_node_options_t * options = rcl_node_get_options(node);
  if (nullptr == options) {
    throw std::runtime_error("Need valid node options in NodeParameters");
  }

  // Get paths to yaml files containing initial parameter values
  std::vector<std::string> yaml_paths;

  auto get_yaml_paths = [&yaml_paths, &options](const rcl_arguments_t * args) {
      int num_yaml_files = rcl_arguments_get_param_files_count(args);
      if (num_yaml_files > 0) {
        char ** param_files;
        rcl_ret_t ret = rcl_arguments_get_param_files(args, options->allocator, &param_files);
        if (RCL_RET_OK != ret) {
          rclcpp::exceptions::throw_from_rcl_error(ret);
        }
        auto cleanup_param_files = make_scope_exit(
          [&param_files, &num_yaml_files, &options]() {
            for (int i = 0; i < num_yaml_files; ++i) {
              options->allocator.deallocate(param_files[i], options->allocator.state);
            }
            options->allocator.deallocate(param_files, options->allocator.state);
          });
        for (int i = 0; i < num_yaml_files; ++i) {
          yaml_paths.emplace_back(param_files[i]);
        }
      }
    };

  // global before local so that local overwrites global
  if (options->use_global_arguments) {
    auto context_ptr = node_base->get_context()->get_rcl_context();
    get_yaml_paths(&(context_ptr->global_arguments));
  }
  get_yaml_paths(&(options->arguments));

  // Get fully qualified node name post-remapping to use to find node's params in yaml files
  const std::string node_name = node_base->get_name();
  const std::string node_namespace = node_base->get_namespace();
  if (0u == node_namespace.size() || 0u == node_name.size()) {
    // Should never happen
    throw std::runtime_error("Node name and namespace were not set");
  }

  if ('/' == node_namespace.at(node_namespace.size() - 1)) {
    combined_name_ = node_namespace + node_name;
  } else {
    combined_name_ = node_namespace + '/' + node_name;
  }

  // TODO(sloretz) use rcl to parse yaml when circular dependency is solved
  // See https://github.com/ros2/rcl/issues/252
  for (const std::string & yaml_path : yaml_paths) {
    rcl_params_t * yaml_params = rcl_yaml_node_struct_init(options->allocator);
    if (nullptr == yaml_params) {
      throw std::bad_alloc();
    }
    if (!rcl_parse_yaml_file(yaml_path.c_str(), yaml_params)) {
      std::ostringstream ss;
      ss << "Failed to parse parameters from file '" << yaml_path << "': " <<
        rcl_get_error_string().str;
      rcl_reset_error();
      throw std::runtime_error(ss.str());
    }

    rclcpp::ParameterMap initial_map = rclcpp::parameter_map_from(yaml_params);
    rcl_yaml_node_struct_fini(yaml_params);
    auto iter = initial_map.find(combined_name_);
    if (initial_map.end() == iter) {
      continue;
    }

    // Combine parameter yaml files, overwriting values in older ones
    for (auto & param : iter->second) {
      initial_parameter_values_[param.get_name()] =
        rclcpp::ParameterValue(param.get_value_message());
    }
  }

  // initial values passed to constructor overwrite yaml file sources
  for (auto & param : initial_parameters) {
    initial_parameter_values_[param.get_name()] =
      rclcpp::ParameterValue(param.get_value_message());
  }

  if (!initial_parameters.empty() && allow_undeclared_) {
    rcl_interfaces::msg::SetParametersResult result = set_parameters_atomically(initial_parameters);
    if (!result.successful) {
      throw std::runtime_error("Failed to set initial parameters");
    }
  }
}

NodeParameters::~NodeParameters()
{}

void
NodeParameters::create_parameter(
  const std::string & name,
  const rclcpp::ParameterValue & default_value,
  bool read_only)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // TODO(sloretz) parameter name validation
  if (name.empty()) {
    throw std::runtime_error("parameter name must not be empty");
  }

  // Error if this parameter has already been declared and is different
  auto param_iter = parameters_.find(name);
  if (param_iter != parameters_.end() && param_iter->second.is_declared) {
    if (
      param_iter->second.descriptor.type != default_value.get_type() ||
      param_iter->second.descriptor.read_only != read_only
    )
    {
      throw std::runtime_error("parameter '" + name + "' exists with conflicting description");
    }
  }

  // Check if run-time user passed an initial value, else use the default.
  rclcpp::ParameterValue initial_value = default_value;
  auto value_iter = initial_parameter_values_.find(name);
  if (value_iter != initial_parameter_values_.end()) {
    initial_value = value_iter->second;
  }

  // Save a description of the parameter
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.name = name;
  desc.type = initial_value.get_type();
  desc.read_only = read_only;

  rclcpp::node_interfaces::ParameterInfo_t pinfo;
  pinfo.is_declared = true;
  pinfo.value = initial_value;
  pinfo.descriptor = desc;

  // Add declared parameters to storage, even when they're not set.
  parameters_[name] = pinfo;

  // If it has an actual value then add it to 'new_parameters' event
  if (rclcpp::ParameterType::PARAMETER_NOT_SET != initial_value.get_type()) {
    auto parameter_event = std::make_shared<rcl_interfaces::msg::ParameterEvent>();
    parameter_event->new_parameters.push_back(
      rclcpp::Parameter(name, initial_value).to_parameter_msg());
    events_publisher_->publish(parameter_event);
  }
}

std::vector<rcl_interfaces::msg::SetParametersResult>
NodeParameters::set_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::vector<rcl_interfaces::msg::SetParametersResult> results;
  for (const auto & p : parameters) {
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
  std::map<std::string, rclcpp::node_interfaces::ParameterInfo_t> tmp_map;
  auto parameter_event = std::make_shared<rcl_interfaces::msg::ParameterEvent>();

  parameter_event->node = combined_name_;

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

  std::vector<std::string> to_delete;

  for (const auto & p : parameters) {
    auto param_iter = parameters_.find(p.get_name());
    bool exists = parameters_.end() != param_iter;
    bool want_to_delete = p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET;

    if (exists) {
      if (param_iter->second.descriptor.read_only) {
        result.successful = false;
        result.reason = "read_only parameter: '" + p.get_name() + "'";
        return result;
      }
      if (want_to_delete) {
        parameter_event->deleted_parameters.push_back(p.to_parameter_msg());
        if (param_iter->second.is_declared) {
          // clear declared parameter value, but don't delete it
          rclcpp::node_interfaces::ParameterInfo_t cleared_param_info = param_iter->second;
          cleared_param_info.value = rclcpp::ParameterValue();
          tmp_map[p.get_name()] = cleared_param_info;
        } else {
          // Truly delete an undeclared parameter
          to_delete.push_back(p.get_name());
        }
      } else {
        if (param_iter->second.value.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
          // case: setting a value on a declared parameter that currently is unset
          parameter_event->new_parameters.push_back(p.to_parameter_msg());
        } else {
          // case: changing a parameter value
          parameter_event->changed_parameters.push_back(p.to_parameter_msg());
        }
        rclcpp::node_interfaces::ParameterInfo_t changed_param_info = param_iter->second;
        // TODO(sloretz) Add accessor for ParameterValue on Parameter class
        changed_param_info.value = rclcpp::ParameterValue(p.get_value_message());
        tmp_map[p.get_name()] = changed_param_info;
      }
    } else {
      // case: parameter does not exist already
      if (!allow_undeclared_) {
        result.successful = false;
        result.reason = "undeclared parameter: '" + p.get_name() + "'";
        return result;
      } else if (want_to_delete) {
        result.successful = false;
        result.reason = "deleting parameter: '" + p.get_name() + "' that does not exist";
        return result;
      } else {
        // Create new undeclared parameter
        parameter_event->new_parameters.push_back(p.to_parameter_msg());

        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.name = p.get_name();
        desc.type = p.get_type();
        desc.read_only = false;

        rclcpp::node_interfaces::ParameterInfo_t new_param_info;
        new_param_info.is_declared = false;
        // TODO(sloretz) Add accessor for ParameterValue on Parameter class
        new_param_info.value = rclcpp::ParameterValue(p.get_value_message());
        new_param_info.descriptor = desc;

        tmp_map[p.get_name()] = new_param_info;
      }
    }
  }
  // std::map::insert will not overwrite elements, so we'll keep the new
  // ones and add only those that already exist in the Node's internal map
  tmp_map.insert(parameters_.begin(), parameters_.end());

  // remove truly deleted parameters
  for (const std::string & param_name : to_delete) {
    tmp_map.erase(param_name);
  }

  std::swap(tmp_map, parameters_);

  // events_publisher_ may be nullptr if it was disabled in constructor
  if (nullptr != events_publisher_) {
    parameter_event->stamp = node_clock_->get_clock()->now();
    events_publisher_->publish(parameter_event);
  }

  return result;
}

std::vector<rclcpp::Parameter>
NodeParameters::get_parameters(const std::vector<std::string> & names) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<rclcpp::Parameter> results;

  for (auto & name : names) {
    if (std::any_of(parameters_.cbegin(), parameters_.cend(),
      [&name](const std::pair<std::string, rclcpp::node_interfaces::ParameterInfo_t> & kv) {
        return name == kv.first;
      }))
    {
      results.emplace_back(name, parameters_.at(name).value);
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
    parameter = {name, parameters_.at(name).value};
    return true;
  } else {
    return false;
  }
}

bool
NodeParameters::get_parameters_by_prefix(
  const std::string & prefix,
  std::map<std::string, rclcpp::Parameter> & parameters) const
{
  std::string prefix_with_dot = prefix + ".";
  bool ret = false;

  std::lock_guard<std::mutex> lock(mutex_);

  for (const auto & param : parameters_) {
    if (param.first.find(prefix_with_dot) == 0 && param.first.length() > prefix_with_dot.length()) {
      // Found one!
      parameters[param.first.substr(prefix_with_dot.length())] =
        rclcpp::Parameter(param.second.descriptor.name, param.second.value);
      ret = true;
    }
  }

  return ret;
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
      results.push_back(kv.second.descriptor);
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
      results.push_back(kv.second.value.get_type());
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
      "overwriting previous callback");
  }
  parameters_callback_ = callback;
}
