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

#include "../rcl_context_wrapper.hpp"
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
  const std::vector<rclcpp::Parameter> & initial_parameters,
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
    auto rcl_context_wrapper = node_base->get_context()->get_sub_context<RclContextWrapper>();
    auto context_ptr = rcl_context_wrapper->get_context();
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
  std::string combined_name;
  if ('/' == node_namespace.at(node_namespace.size() - 1)) {
    combined_name = node_namespace + node_name;
  } else {
    combined_name = node_namespace + '/' + node_name;
  }

  std::map<std::string, rclcpp::Parameter> parameters;

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
    auto iter = initial_map.find(combined_name);
    if (initial_map.end() == iter) {
      continue;
    }

    // Combine parameter yaml files, overwriting values in older ones
    for (auto & param : iter->second) {
      parameters[param.get_name()] = param;
    }
  }

  // initial values passed to constructor overwrite yaml file sources
  for (auto & param : initial_parameters) {
    parameters[param.get_name()] = param;
  }

  std::vector<rclcpp::Parameter> combined_values;
  combined_values.reserve(parameters.size());
  for (auto & kv : parameters) {
    combined_values.emplace_back(kv.second);
  }

  // TODO(sloretz) store initial values and use them when a parameter is created ros2/rclcpp#475
  // Set initial parameter values
  if (!combined_values.empty()) {
    rcl_interfaces::msg::SetParametersResult result = set_parameters_atomically(combined_values);
    if (!result.successful) {
      throw std::runtime_error("Failed to set initial parameters");
    }
  }
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
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      if (parameters_.find(p.get_name()) != parameters_.end()) {
        // case: parameter was set before, and input is "NOT_SET"
        // therefore we will erase the parameter from parameters_ later
        parameter_event->deleted_parameters.push_back(p.to_parameter_msg());
      }
    } else {
      if (parameters_.find(p.get_name()) == parameters_.end()) {
        // case: parameter not set before, and input is something other than "NOT_SET"
        parameter_event->new_parameters.push_back(p.to_parameter_msg());
      } else {
        // case: parameter was set before, and input is something other than "NOT_SET"
        parameter_event->changed_parameters.push_back(p.to_parameter_msg());
      }
      tmp_map[p.get_name()] = p;
    }
  }
  // std::map::insert will not overwrite elements, so we'll keep the new
  // ones and add only those that already exist in the Node's internal map
  tmp_map.insert(parameters_.begin(), parameters_.end());

  // remove explicitly deleted parameters
  for (auto p : parameters) {
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      tmp_map.erase(p.get_name());
    }
  }

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
      "overwriting previous callback");
  }
  parameters_callback_ = callback;
}
