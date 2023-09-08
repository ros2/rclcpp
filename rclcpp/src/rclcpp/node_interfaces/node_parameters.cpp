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

#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rclcpp/create_publisher.hpp"
#include "rclcpp/parameter_map.hpp"
#include "rcutils/logging_macros.h"
#include "rmw/qos_profiles.h"

#include "../detail/resolve_parameter_overrides.hpp"

using rclcpp::node_interfaces::NodeParameters;

RCLCPP_LOCAL
void
local_perform_automatically_declare_parameters_from_overrides(
  const std::map<std::string, rclcpp::ParameterValue> & parameter_overrides,
  std::function<bool(const std::string &)> has_parameter,
  std::function<void(
    const std::string &,
    const rclcpp::ParameterValue &,
    const rcl_interfaces::msg::ParameterDescriptor &,
    bool)>
  declare_parameter)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.dynamic_typing = true;
  for (const auto & pair : parameter_overrides) {
    if (!has_parameter(pair.first)) {
      declare_parameter(
        pair.first,
        pair.second,
        descriptor,
        true);
    }
  }
}

NodeParameters::NodeParameters(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
  const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services,
  const rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock,
  const std::vector<rclcpp::Parameter> & parameter_overrides,
  bool start_parameter_services,
  bool start_parameter_event_publisher,
  const rclcpp::QoS & parameter_event_qos,
  const rclcpp::PublisherOptionsBase & parameter_event_publisher_options,
  bool allow_undeclared_parameters,
  bool automatically_declare_parameters_from_overrides)
: allow_undeclared_(allow_undeclared_parameters),
  events_publisher_(nullptr),
  node_logging_(node_logging),
  node_clock_(node_clock)
{
  using MessageT = rcl_interfaces::msg::ParameterEvent;
  using PublisherT = rclcpp::Publisher<MessageT>;
  using AllocatorT = std::allocator<void>;
  // TODO(wjwwood): expose this allocator through the Parameter interface.
  rclcpp::PublisherOptionsWithAllocator<AllocatorT> publisher_options(
    parameter_event_publisher_options);
  publisher_options.allocator = std::make_shared<AllocatorT>();

  if (start_parameter_services) {
    parameter_service_ = std::make_shared<ParameterService>(node_base, node_services, this);
  }

  if (start_parameter_event_publisher) {
    // TODO(ivanpauno): Qos of the `/parameters_event` topic should be somehow overridable.
    events_publisher_ = rclcpp::create_publisher<MessageT, AllocatorT, PublisherT>(
      node_topics,
      "/parameter_events",
      parameter_event_qos,
      publisher_options);
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

  const rcl_arguments_t * global_args = nullptr;
  if (options->use_global_arguments) {
    auto context_ptr = node_base->get_context()->get_rcl_context();
    global_args = &(context_ptr->global_arguments);
  }
  combined_name_ = node_base->get_fully_qualified_name();

  parameter_overrides_ = rclcpp::detail::resolve_parameter_overrides(
    combined_name_, parameter_overrides, &options->arguments, global_args);

  // If asked, initialize any parameters that ended up in the initial parameter values,
  // but did not get declared explcitily by this point.
  if (automatically_declare_parameters_from_overrides) {
    using namespace std::placeholders;
    local_perform_automatically_declare_parameters_from_overrides(
      this->get_parameter_overrides(),
      std::bind(&NodeParameters::has_parameter, this, _1),
      [this](
        const std::string & name,
        const rclcpp::ParameterValue & default_value,
        const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor,
        bool ignore_override)
      {
        NodeParameters::declare_parameter(
          name, default_value, parameter_descriptor, ignore_override);
      }
    );
  }
}

void
NodeParameters::perform_automatically_declare_parameters_from_overrides()
{
  local_perform_automatically_declare_parameters_from_overrides(
    this->get_parameter_overrides(),
    [this](const std::string & name) {
      return this->has_parameter(name);
    },
    [this](
      const std::string & name,
      const rclcpp::ParameterValue & default_value,
      const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor,
      bool ignore_override)
    {
      this->declare_parameter(
        name, default_value, parameter_descriptor, ignore_override);
    }
  );
}

NodeParameters::~NodeParameters()
{}

RCLCPP_LOCAL
bool
__lockless_has_parameter(
  const std::map<std::string, rclcpp::node_interfaces::ParameterInfo> & parameters,
  const std::string & name)
{
  return parameters.find(name) != parameters.end();
}

// see https://en.cppreference.com/w/cpp/types/numeric_limits/epsilon
RCLCPP_LOCAL
bool
__are_doubles_equal(double x, double y, double ulp = 100.0)
{
  return std::abs(x - y) <= std::numeric_limits<double>::epsilon() * std::abs(x + y) * ulp;
}

static
std::string
format_range_reason(const std::string & name, const char * range_type)
{
  std::ostringstream ss;
  ss << "Parameter {" << name << "} doesn't comply with " << range_type << " range.";
  return ss.str();
}

RCLCPP_LOCAL
rcl_interfaces::msg::SetParametersResult
__check_parameter_value_in_range(
  const rcl_interfaces::msg::ParameterDescriptor & descriptor,
  const rclcpp::ParameterValue & value)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  if (!descriptor.integer_range.empty() && value.get_type() == rclcpp::PARAMETER_INTEGER) {
    int64_t v = value.get<int64_t>();
    auto integer_range = descriptor.integer_range.at(0);
    if (v == integer_range.from_value || v == integer_range.to_value) {
      return result;
    }
    if ((v < integer_range.from_value) || (v > integer_range.to_value)) {
      result.successful = false;
      result.reason = format_range_reason(descriptor.name, "integer");
      return result;
    }
    if (integer_range.step == 0) {
      return result;
    }
    if (((v - integer_range.from_value) % integer_range.step) == 0) {
      return result;
    }
    result.successful = false;
    result.reason = format_range_reason(descriptor.name, "integer");
    return result;
  }

  if (!descriptor.floating_point_range.empty() && value.get_type() == rclcpp::PARAMETER_DOUBLE) {
    double v = value.get<double>();
    auto fp_range = descriptor.floating_point_range.at(0);
    if (__are_doubles_equal(v, fp_range.from_value) || __are_doubles_equal(v, fp_range.to_value)) {
      return result;
    }
    if ((v < fp_range.from_value) || (v > fp_range.to_value)) {
      result.successful = false;
      result.reason = format_range_reason(descriptor.name, "floating point");
      return result;
    }
    if (fp_range.step == 0.0) {
      return result;
    }
    double rounded_div = std::round((v - fp_range.from_value) / fp_range.step);
    if (__are_doubles_equal(v, fp_range.from_value + rounded_div * fp_range.step)) {
      return result;
    }
    result.successful = false;
    result.reason = format_range_reason(descriptor.name, "floating point");
    return result;
  }
  return result;
}

static
std::string
format_type_reason(
  const std::string & name, const std::string & old_type, const std::string & new_type)
{
  std::ostringstream ss;
  // WARN: A condition later depends on this message starting with "Wrong parameter type",
  // check `declare_parameter` if you modify this!
  ss << "Wrong parameter type, parameter {" << name << "} is of type {" << old_type <<
    "}, setting it to {" << new_type << "} is not allowed.";
  return ss.str();
}

// Return true if parameter values comply with the descriptors in parameter_infos.
RCLCPP_LOCAL
rcl_interfaces::msg::SetParametersResult
__check_parameters(
  std::map<std::string, rclcpp::node_interfaces::ParameterInfo> & parameter_infos,
  const std::vector<rclcpp::Parameter> & parameters,
  bool allow_undeclared)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const rclcpp::Parameter & parameter : parameters) {
    std::string name = parameter.get_name();
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    if (allow_undeclared) {
      auto it = parameter_infos.find(name);
      if (it != parameter_infos.cend()) {
        descriptor = it->second.descriptor;
      } else {
        // implicitly declared parameters are dinamically typed!
        descriptor.dynamic_typing = true;
      }
    } else {
      descriptor = parameter_infos[name].descriptor;
    }
    if (descriptor.name.empty()) {
      descriptor.name = name;
    }
    const auto new_type = parameter.get_type();
    const auto specified_type = static_cast<rclcpp::ParameterType>(descriptor.type);
    result.successful = descriptor.dynamic_typing || specified_type == new_type;
    if (!result.successful) {
      result.reason = format_type_reason(
        name, rclcpp::to_string(specified_type), rclcpp::to_string(new_type));
      return result;
    }
    result = __check_parameter_value_in_range(
      descriptor,
      parameter.get_parameter_value());
    if (!result.successful) {
      return result;
    }
  }
  return result;
}

using PreSetParametersCallbackType =
  rclcpp::node_interfaces::NodeParametersInterface::PreSetParametersCallbackType;
using PreSetParametersCallbackHandle =
  rclcpp::node_interfaces::PreSetParametersCallbackHandle;
using PreSetCallbacksHandleContainer =
  rclcpp::node_interfaces::NodeParameters::PreSetCallbacksHandleContainer;

using OnSetParametersCallbackType =
  rclcpp::node_interfaces::NodeParametersInterface::OnSetParametersCallbackType;
using OnSetParametersCallbackHandle =
  rclcpp::node_interfaces::OnSetParametersCallbackHandle;
using OnSetCallbacksHandleContainer =
  rclcpp::node_interfaces::NodeParameters::OnSetCallbacksHandleContainer;

using PostSetParametersCallbackType =
  rclcpp::node_interfaces::NodeParametersInterface::PostSetParametersCallbackType;
using PostSetParametersCallbackHandle =
  rclcpp::node_interfaces::PostSetParametersCallbackHandle;
using PostSetCallbacksHandleContainer =
  rclcpp::node_interfaces::NodeParameters::PostSetCallbacksHandleContainer;

RCLCPP_LOCAL
void
__call_pre_set_parameters_callbacks(
  std::vector<rclcpp::Parameter> & parameters,
  PreSetCallbacksHandleContainer & callback_container)
{
  if (callback_container.empty()) {
    return;
  }

  auto it = callback_container.begin();
  while (it != callback_container.end()) {
    auto shared_handle = it->lock();
    if (nullptr != shared_handle) {
      shared_handle->callback(parameters);
      it++;
    } else {
      it = callback_container.erase(it);
    }
  }
}

RCLCPP_LOCAL
rcl_interfaces::msg::SetParametersResult
__call_on_set_parameters_callbacks(
  const std::vector<rclcpp::Parameter> & parameters,
  OnSetCallbacksHandleContainer & callback_container)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  auto it = callback_container.begin();
  while (it != callback_container.end()) {
    auto shared_handle = it->lock();
    if (nullptr != shared_handle) {
      result = shared_handle->callback(parameters);
      if (!result.successful) {
        return result;
      }
      it++;
    } else {
      it = callback_container.erase(it);
    }
  }
  return result;
}

RCLCPP_LOCAL
void __call_post_set_parameters_callbacks(
  const std::vector<rclcpp::Parameter> & parameters,
  PostSetCallbacksHandleContainer & callback_container)
{
  if (callback_container.empty()) {
    return;
  }

  auto it = callback_container.begin();
  while (it != callback_container.end()) {
    auto shared_handle = it->lock();
    if (nullptr != shared_handle) {
      shared_handle->callback(parameters);
      it++;
    } else {
      it = callback_container.erase(it);
    }
  }
}

RCLCPP_LOCAL
rcl_interfaces::msg::SetParametersResult
__set_parameters_atomically_common(
  const std::vector<rclcpp::Parameter> & parameters,
  std::map<std::string, rclcpp::node_interfaces::ParameterInfo> & parameter_infos,
  OnSetCallbacksHandleContainer & on_set_callback_container,
  PostSetCallbacksHandleContainer & post_set_callback_container,
  bool allow_undeclared = false)
{
  // Check if the value being set complies with the descriptor.
  rcl_interfaces::msg::SetParametersResult result = __check_parameters(
    parameter_infos, parameters, allow_undeclared);
  if (!result.successful) {
    return result;
  }
  // Call the user callbacks to see if the new value(s) are allowed.
  result =
    __call_on_set_parameters_callbacks(parameters, on_set_callback_container);
  if (!result.successful) {
    return result;
  }
  // If accepted, actually set the values.
  if (result.successful) {
    for (size_t i = 0; i < parameters.size(); ++i) {
      const std::string & name = parameters[i].get_name();
      parameter_infos[name].descriptor.name = parameters[i].get_name();
      parameter_infos[name].descriptor.type = parameters[i].get_type();
      parameter_infos[name].value = parameters[i].get_parameter_value();
    }
    // Call the user post set parameter callback
    __call_post_set_parameters_callbacks(parameters, post_set_callback_container);
  }

  // Either way, return the result.
  return result;
}

RCLCPP_LOCAL
rcl_interfaces::msg::SetParametersResult
__declare_parameter_common(
  const std::string & name,
  const rclcpp::ParameterValue & default_value,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor,
  std::map<std::string, rclcpp::node_interfaces::ParameterInfo> & parameters_out,
  const std::map<std::string, rclcpp::ParameterValue> & overrides,
  OnSetCallbacksHandleContainer & on_set_callback_container,
  PostSetCallbacksHandleContainer & post_set_callback_container,
  rcl_interfaces::msg::ParameterEvent * parameter_event_out,
  bool ignore_override = false)
{
  using rclcpp::node_interfaces::ParameterInfo;
  std::map<std::string, ParameterInfo> parameter_infos {{name, ParameterInfo()}};
  parameter_infos.at(name).descriptor = parameter_descriptor;

  // Use the value from the overrides if available, otherwise use the default.
  const rclcpp::ParameterValue * initial_value = &default_value;
  auto overrides_it = overrides.find(name);
  if (!ignore_override && overrides_it != overrides.end()) {
    initial_value = &overrides_it->second;
  }

  // If there is no initial value, then skip initialization
  if (initial_value->get_type() == rclcpp::PARAMETER_NOT_SET) {
    // Add declared parameters to storage (without a value)
    parameter_infos[name].descriptor.name = name;
    if (parameter_descriptor.dynamic_typing) {
      parameter_infos[name].descriptor.type = rclcpp::PARAMETER_NOT_SET;
    } else {
      parameter_infos[name].descriptor.type = parameter_descriptor.type;
    }
    parameters_out[name] = parameter_infos.at(name);
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  // Check with the user's callbacks to see if the initial value can be set.
  std::vector<rclcpp::Parameter> parameter_wrappers {rclcpp::Parameter(name, *initial_value)};
  // This function also takes care of default vs initial value.
  auto result = __set_parameters_atomically_common(
    parameter_wrappers,
    parameter_infos,
    on_set_callback_container,
    post_set_callback_container
  );

  if (!result.successful) {
    return result;
  }

  // Add declared parameters to storage.
  parameters_out[name] = parameter_infos.at(name);

  // Extend the given parameter event, if valid.
  if (parameter_event_out) {
    parameter_event_out->new_parameters.push_back(parameter_wrappers[0].to_parameter_msg());
  }

  return result;
}

static
const rclcpp::ParameterValue &
declare_parameter_helper(
  const std::string & name,
  rclcpp::ParameterType type,
  const rclcpp::ParameterValue & default_value,
  rcl_interfaces::msg::ParameterDescriptor parameter_descriptor,
  bool ignore_override,
  std::map<std::string, rclcpp::node_interfaces::ParameterInfo> & parameters,
  const std::map<std::string, rclcpp::ParameterValue> & overrides,
  OnSetCallbacksHandleContainer & on_set_callback_container,
  PostSetCallbacksHandleContainer & post_set_callback_container,
  rclcpp::Publisher<rcl_interfaces::msg::ParameterEvent> * events_publisher,
  const std::string & combined_name,
  rclcpp::node_interfaces::NodeClockInterface & node_clock)
{
  // TODO(sloretz) parameter name validation
  if (name.empty()) {
    throw rclcpp::exceptions::InvalidParametersException("parameter name must not be empty");
  }

  // Error if this parameter has already been declared and is different
  if (__lockless_has_parameter(parameters, name)) {
    throw rclcpp::exceptions::ParameterAlreadyDeclaredException(
            "parameter '" + name + "' has already been declared");
  }

  if (!parameter_descriptor.dynamic_typing) {
    if (rclcpp::PARAMETER_NOT_SET == type) {
      type = default_value.get_type();
    }
    if (rclcpp::PARAMETER_NOT_SET == type) {
      throw rclcpp::exceptions::InvalidParameterTypeException{
              name,
              "cannot declare a statically typed parameter with an uninitialized value"
      };
    }
    parameter_descriptor.type = static_cast<uint8_t>(type);
  }

  rcl_interfaces::msg::ParameterEvent parameter_event;
  auto result = __declare_parameter_common(
    name,
    default_value,
    parameter_descriptor,
    parameters,
    overrides,
    on_set_callback_container,
    post_set_callback_container,
    &parameter_event,
    ignore_override);

  // If it failed to be set, then throw an exception.
  if (!result.successful) {
    constexpr const char type_error_msg_start[] = "Wrong parameter type";
    if (
      0u == std::strncmp(
        result.reason.c_str(), type_error_msg_start, sizeof(type_error_msg_start) - 1))
    {
      // TODO(ivanpauno): Refactor the logic so we don't need the above `strncmp` and we can
      // detect between both exceptions more elegantly.
      throw rclcpp::exceptions::InvalidParameterTypeException(name, result.reason);
    }
    throw rclcpp::exceptions::InvalidParameterValueException(
            "parameter '" + name + "' could not be set: " + result.reason);
  }

  // Publish if events_publisher_ is not nullptr, which may be if disabled in the constructor.
  if (nullptr != events_publisher) {
    parameter_event.node = combined_name;
    parameter_event.stamp = node_clock.get_clock()->now();
    events_publisher->publish(parameter_event);
  }

  return parameters.at(name).value;
}

const rclcpp::ParameterValue &
NodeParameters::declare_parameter(
  const std::string & name,
  const rclcpp::ParameterValue & default_value,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor,
  bool ignore_override)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  ParameterMutationRecursionGuard guard(parameter_modification_enabled_);

  return declare_parameter_helper(
    name,
    rclcpp::PARAMETER_NOT_SET,
    default_value,
    parameter_descriptor,
    ignore_override,
    parameters_,
    parameter_overrides_,
    on_set_parameters_callback_container_,
    post_set_parameters_callback_container_,
    events_publisher_.get(),
    combined_name_,
    *node_clock_);
}

const rclcpp::ParameterValue &
NodeParameters::declare_parameter(
  const std::string & name,
  rclcpp::ParameterType type,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor,
  bool ignore_override)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  ParameterMutationRecursionGuard guard(parameter_modification_enabled_);

  if (rclcpp::PARAMETER_NOT_SET == type) {
    throw std::invalid_argument{
            "declare_parameter(): the provided parameter type cannot be rclcpp::PARAMETER_NOT_SET"};
  }

  if (parameter_descriptor.dynamic_typing == true) {
    throw std::invalid_argument{
            "declare_parameter(): cannot declare parameter of specific type and pass descriptor"
            "with `dynamic_typing=true`"};
  }

  return declare_parameter_helper(
    name,
    type,
    rclcpp::ParameterValue{},
    parameter_descriptor,
    ignore_override,
    parameters_,
    parameter_overrides_,
    on_set_parameters_callback_container_,
    post_set_parameters_callback_container_,
    events_publisher_.get(),
    combined_name_,
    *node_clock_);
}

void
NodeParameters::undeclare_parameter(const std::string & name)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  ParameterMutationRecursionGuard guard(parameter_modification_enabled_);

  auto parameter_info = parameters_.find(name);
  if (parameter_info == parameters_.end()) {
    throw rclcpp::exceptions::ParameterNotDeclaredException(
            "cannot undeclare parameter '" + name + "' which has not yet been declared");
  }

  if (parameter_info->second.descriptor.read_only) {
    throw rclcpp::exceptions::ParameterImmutableException(
            "cannot undeclare parameter '" + name + "' because it is read-only");
  }
  if (!parameter_info->second.descriptor.dynamic_typing) {
    throw rclcpp::exceptions::InvalidParameterTypeException{
            name, "cannot undeclare a statically typed parameter"};
  }

  parameters_.erase(parameter_info);
}

bool
NodeParameters::has_parameter(const std::string & name) const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  return __lockless_has_parameter(parameters_, name);
}

std::vector<rcl_interfaces::msg::SetParametersResult>
NodeParameters::set_parameters(const std::vector<rclcpp::Parameter> & parameters)
{
  std::vector<rcl_interfaces::msg::SetParametersResult> results;
  results.reserve(parameters.size());

  for (const auto & p : parameters) {
    auto result = set_parameters_atomically({{p}});
    results.push_back(result);
  }

  return results;
}

template<typename ParameterVectorType>
auto
__find_parameter_by_name(
  ParameterVectorType & parameters,
  const std::string & name)
{
  return std::find_if(
    parameters.begin(),
    parameters.end(),
    [&](auto parameter) {return parameter.get_name() == name;});
}

rcl_interfaces::msg::SetParametersResult
NodeParameters::set_parameters_atomically(const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  ParameterMutationRecursionGuard guard(parameter_modification_enabled_);

  rcl_interfaces::msg::SetParametersResult result;

  // call any user registered pre set parameter callbacks
  // this callback can make changes to the original parameters list
  // also check if the changed parameter list is empty or not, if empty return
  std::vector<rclcpp::Parameter> parameters_after_pre_set_callback(parameters);
  __call_pre_set_parameters_callbacks(
    parameters_after_pre_set_callback,
    pre_set_parameters_callback_container_);

  if (parameters_after_pre_set_callback.empty()) {
    result.successful = false;
    result.reason = "parameter list cannot be empty, this might be due to "
      "pre_set_parameters_callback modifying the original parameters list.";
    return result;
  }

  // Check if any of the parameters are read-only, or if any parameters are not
  // declared.
  // If not declared, keep track of them in order to declare them later, when
  // undeclared parameters are allowed, and if they're not allowed, fail.
  std::vector<const rclcpp::Parameter *> parameters_to_be_declared;
  for (const auto & parameter : parameters_after_pre_set_callback) {
    const std::string & name = parameter.get_name();

    // Check to make sure the parameter name is valid.
    if (name.empty()) {
      throw rclcpp::exceptions::InvalidParametersException("parameter name must not be empty");
    }

    // Check to see if it is declared.
    auto parameter_info = parameters_.find(name);
    if (parameter_info == parameters_.end()) {
      // If not check to see if undeclared paramaters are allowed, ...
      if (allow_undeclared_) {
        // If so, mark the parameter to be declared for the user implicitly.
        parameters_to_be_declared.push_back(&parameter);
        // continue as it cannot be read-only, and because the declare will
        // implicitly set the parameter and parameter_infos is for setting only.
        continue;
      } else {
        // If not, then throw the exception as documented.
        throw rclcpp::exceptions::ParameterNotDeclaredException(
                "parameter '" + name + "' cannot be set because it was not declared");
      }
    }

    // Check to see if it is read-only.
    if (parameter_info->second.descriptor.read_only) {
      result.successful = false;
      result.reason = "parameter '" + name + "' cannot be set because it is read-only";
      return result;
    }
  }

  // Declare parameters into a temporary "staging area", incase one of the declares fail.
  // We will use the staged changes as input to the "set atomically" action.
  // We explicitly avoid calling the user callbacks here, so that it may be called once, with
  // all the other parameters to be set (already declared parameters).
  std::map<std::string, rclcpp::node_interfaces::ParameterInfo> staged_parameter_changes;
  rcl_interfaces::msg::ParameterEvent parameter_event_msg;
  parameter_event_msg.node = combined_name_;
  OnSetCallbacksHandleContainer empty_on_set_callback_container;
  PostSetCallbacksHandleContainer empty_post_set_callback_container;

  // Implicit declare uses dynamic type descriptor.
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.dynamic_typing = true;
  for (auto parameter_to_be_declared : parameters_to_be_declared) {
    // This should not throw, because we validated the name and checked that
    // the parameter was not already declared.
    result = __declare_parameter_common(
      parameter_to_be_declared->get_name(),
      parameter_to_be_declared->get_parameter_value(),
      descriptor,
      staged_parameter_changes,
      parameter_overrides_,
      // Only call callbacks once below
      empty_on_set_callback_container,   // callback_container is explicitly empty
      empty_post_set_callback_container,  // callback_container is explicitly empty
      &parameter_event_msg,
      true);
    if (!result.successful) {
      // Declare failed, return knowing that nothing was changed because the
      // staged changes were not applied.
      return result;
    }
  }

  // If there were implicitly declared parameters, then we may need to copy the input parameters
  // and then assign the value that was selected after the declare (could be affected by the
  // initial parameter values).
  const std::vector<rclcpp::Parameter> * parameters_to_be_set = &parameters_after_pre_set_callback;
  std::vector<rclcpp::Parameter> parameters_copy;
  if (0 != staged_parameter_changes.size()) {  // If there were any implicitly declared parameters.
    bool any_initial_values_used = false;
    for (const auto & staged_parameter_change : staged_parameter_changes) {
      auto it = __find_parameter_by_name(
        parameters_after_pre_set_callback,
        staged_parameter_change.first);
      if (it->get_parameter_value() != staged_parameter_change.second.value) {
        // In this case, the value of the staged parameter differs from the
        // input from the user, and therefore we need to update things before setting.
        any_initial_values_used = true;
        // No need to search further since at least one initial value needs to be used.
        break;
      }
    }
    if (any_initial_values_used) {
      parameters_copy = parameters_after_pre_set_callback;
      for (const auto & staged_parameter_change : staged_parameter_changes) {
        auto it = __find_parameter_by_name(parameters_copy, staged_parameter_change.first);
        *it = Parameter(staged_parameter_change.first, staged_parameter_change.second.value);
      }
      parameters_to_be_set = &parameters_copy;
    }
  }

  // Collect parameters who will have had their type changed to
  // rclcpp::PARAMETER_NOT_SET so they can later be implicitly undeclared.
  std::vector<const rclcpp::Parameter *> parameters_to_be_undeclared;
  for (const auto & parameter : *parameters_to_be_set) {
    if (rclcpp::PARAMETER_NOT_SET == parameter.get_type()) {
      auto it = parameters_.find(parameter.get_name());
      if (it != parameters_.end() && rclcpp::PARAMETER_NOT_SET != it->second.value.get_type()) {
        if (!it->second.descriptor.dynamic_typing) {
          result.reason = "cannot undeclare a statically typed parameter";
          result.successful = false;
          return result;
        }
        parameters_to_be_undeclared.push_back(&parameter);
      }
    }
  }

  // Set all of the parameters including the ones declared implicitly above.
  result = __set_parameters_atomically_common(
    // either the original parameters given by the user, or ones updated with initial values
    *parameters_to_be_set,
    // they are actually set on the official parameter storage
    parameters_,
    // These callbacks are called once. When a callback returns an unsuccessful result,
    // the remaining aren't called
    on_set_parameters_callback_container_,
    post_set_parameters_callback_container_,
    allow_undeclared_);  // allow undeclared

  // If not successful, then stop here.
  if (!result.successful) {
    return result;
  }

  // If successful, then update the parameter infos from the implicitly declared parameter's.
  for (const auto & kv_pair : staged_parameter_changes) {
    // assumption: the parameter is already present in parameters_ due to the above "set"
    assert(__lockless_has_parameter(parameters_, kv_pair.first));
    // assumption: the value in parameters_ is the same as the value resulting from the declare
    assert(parameters_[kv_pair.first].value == kv_pair.second.value);
    // This assignment should not change the name, type, or value, but may
    // change other things from the ParameterInfo.
    parameters_[kv_pair.first] = kv_pair.second;
  }

  // Undeclare parameters that need to be.
  for (auto parameter_to_undeclare : parameters_to_be_undeclared) {
    auto it = parameters_.find(parameter_to_undeclare->get_name());
    // assumption: the parameter to be undeclared should be in the parameter infos map
    assert(it != parameters_.end());
    if (it != parameters_.end()) {
      // Update the parameter event message and remove it.
      parameter_event_msg.deleted_parameters.push_back(
        rclcpp::Parameter(it->first, it->second.value).to_parameter_msg());
      parameters_.erase(it);
    }
  }

  // Update the parameter event message for any parameters which were only set,
  // and not either declared or undeclared.
  for (const auto & parameter : *parameters_to_be_set) {
    if (staged_parameter_changes.find(parameter.get_name()) != staged_parameter_changes.end()) {
      // This parameter was declared.
      continue;
    }
    auto it = std::find_if(
      parameters_to_be_undeclared.begin(),
      parameters_to_be_undeclared.end(),
      [&parameter](const auto & p) {return p->get_name() == parameter.get_name();});
    if (it != parameters_to_be_undeclared.end()) {
      // This parameter was undeclared (deleted).
      continue;
    }
    // This parameter was neither declared nor undeclared.
    parameter_event_msg.changed_parameters.push_back(parameter.to_parameter_msg());
  }

  // Publish if events_publisher_ is not nullptr, which may be if disabled in the constructor.
  if (nullptr != events_publisher_) {
    parameter_event_msg.stamp = node_clock_->get_clock()->now();
    events_publisher_->publish(parameter_event_msg);
  }
  return result;
}

std::vector<rclcpp::Parameter>
NodeParameters::get_parameters(const std::vector<std::string> & names) const
{
  std::vector<rclcpp::Parameter> results;
  results.reserve(names.size());

  std::lock_guard<std::recursive_mutex> lock(mutex_);
  for (auto & name : names) {
    results.emplace_back(this->get_parameter(name));
  }
  return results;
}

rclcpp::Parameter
NodeParameters::get_parameter(const std::string & name) const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  auto param_iter = parameters_.find(name);
  if (parameters_.end() != param_iter) {
    if (
      param_iter->second.value.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET ||
      param_iter->second.descriptor.dynamic_typing)
    {
      return rclcpp::Parameter{name, param_iter->second.value};
    }
    throw rclcpp::exceptions::ParameterUninitializedException(name);
  } else if (this->allow_undeclared_) {
    return rclcpp::Parameter{name};
  } else {
    throw rclcpp::exceptions::ParameterNotDeclaredException(name);
  }
}

bool
NodeParameters::get_parameter(
  const std::string & name,
  rclcpp::Parameter & parameter) const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  auto param_iter = parameters_.find(name);
  if (
    parameters_.end() != param_iter &&
    param_iter->second.value.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET)
  {
    parameter = {name, param_iter->second.value};
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
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  std::string prefix_with_dot = prefix.empty() ? prefix : prefix + ".";
  bool ret = false;

  for (const auto & param : parameters_) {
    if (param.first.find(prefix_with_dot) == 0 && param.first.length() > prefix_with_dot.length()) {
      // Found one!
      parameters[param.first.substr(prefix_with_dot.length())] = rclcpp::Parameter(param.second);
      ret = true;
    }
  }

  return ret;
}

std::vector<rcl_interfaces::msg::ParameterDescriptor>
NodeParameters::describe_parameters(const std::vector<std::string> & names) const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  std::vector<rcl_interfaces::msg::ParameterDescriptor> results;
  results.reserve(names.size());

  for (const auto & name : names) {
    auto it = parameters_.find(name);
    if (it != parameters_.cend()) {
      results.push_back(it->second.descriptor);
    } else if (allow_undeclared_) {
      // parameter not found, but undeclared allowed, so return empty
      rcl_interfaces::msg::ParameterDescriptor default_description;
      default_description.name = name;
      results.push_back(default_description);
    } else {
      throw rclcpp::exceptions::ParameterNotDeclaredException(name);
    }
  }

  if (results.size() != names.size()) {
    throw std::runtime_error("results and names unexpectedly different sizes");
  }

  return results;
}

std::vector<uint8_t>
NodeParameters::get_parameter_types(const std::vector<std::string> & names) const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  std::vector<uint8_t> results;
  results.reserve(names.size());

  for (const auto & name : names) {
    auto it = parameters_.find(name);
    if (it != parameters_.cend()) {
      results.push_back(it->second.value.get_type());
    } else if (allow_undeclared_) {
      // parameter not found, but undeclared allowed, so return not set
      results.push_back(rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET);
    } else {
      throw rclcpp::exceptions::ParameterNotDeclaredException(name);
    }
  }

  if (results.size() != names.size()) {
    throw std::runtime_error("results and names unexpectedly different sizes");
  }

  return results;
}

rcl_interfaces::msg::ListParametersResult
NodeParameters::list_parameters(const std::vector<std::string> & prefixes, uint64_t depth) const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  rcl_interfaces::msg::ListParametersResult result;

  // TODO(mikaelarguedas) define parameter separator different from "/" to avoid ambiguity
  // using "." for now
  const char * separator = ".";

  auto separators_less_than_depth = [&depth, &separator](const std::string & str) -> bool {
      return static_cast<uint64_t>(std::count(str.begin(), str.end(), *separator)) < depth;
    };

  bool recursive = (prefixes.size() == 0) &&
    (depth == rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE);

  for (const std::pair<const std::string, ParameterInfo> & kv : parameters_) {
    if (!recursive) {
      bool get_all = (prefixes.size() == 0) && separators_less_than_depth(kv.first);
      if (!get_all) {
        bool prefix_matches = std::any_of(
          prefixes.cbegin(), prefixes.cend(),
          [&kv, &depth, &separator, &separators_less_than_depth](const std::string & prefix) {
            if (kv.first == prefix) {
              return true;
            } else if (kv.first.find(prefix + separator) == 0) {
              if (depth == rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE) {
                return true;
              }
              std::string substr = kv.first.substr(prefix.length() + 1);
              return separators_less_than_depth(substr);
            }
            return false;
          });

        if (!prefix_matches) {
          continue;
        }
      }
    }

    result.names.push_back(kv.first);
    size_t last_separator = kv.first.find_last_of(separator);
    if (std::string::npos != last_separator) {
      std::string prefix = kv.first.substr(0, last_separator);
      if (
        std::find(result.prefixes.cbegin(), result.prefixes.cend(), prefix) ==
        result.prefixes.cend())
      {
        result.prefixes.push_back(prefix);
      }
    }
  }
  return result;
}

void
NodeParameters::remove_pre_set_parameters_callback(
  const PreSetParametersCallbackHandle * const handle)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  ParameterMutationRecursionGuard guard(parameter_modification_enabled_);

  auto it = std::find_if(
    pre_set_parameters_callback_container_.begin(),
    pre_set_parameters_callback_container_.end(),
    [handle](const auto & weak_handle) {
      return handle == weak_handle.lock().get();
    });
  if (it != pre_set_parameters_callback_container_.end()) {
    pre_set_parameters_callback_container_.erase(it);
  } else {
    throw std::runtime_error("Pre set parameter callback doesn't exist");
  }
}

void
NodeParameters::remove_on_set_parameters_callback(
  const OnSetParametersCallbackHandle * const handle)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  ParameterMutationRecursionGuard guard(parameter_modification_enabled_);

  auto it = std::find_if(
    on_set_parameters_callback_container_.begin(),
    on_set_parameters_callback_container_.end(),
    [handle](const auto & weak_handle) {
      return handle == weak_handle.lock().get();
    });
  if (it != on_set_parameters_callback_container_.end()) {
    on_set_parameters_callback_container_.erase(it);
  } else {
    throw std::runtime_error("On set parameter callback doesn't exist");
  }
}

void
NodeParameters::remove_post_set_parameters_callback(
  const PostSetParametersCallbackHandle * const handle)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  ParameterMutationRecursionGuard guard(parameter_modification_enabled_);

  auto it = std::find_if(
    post_set_parameters_callback_container_.begin(),
    post_set_parameters_callback_container_.end(),
    [handle](const auto & weak_handle) {
      return handle == weak_handle.lock().get();
    });
  if (it != post_set_parameters_callback_container_.end()) {
    post_set_parameters_callback_container_.erase(it);
  } else {
    throw std::runtime_error("Post set parameter callback doesn't exist");
  }
}

PreSetParametersCallbackHandle::SharedPtr
NodeParameters::add_pre_set_parameters_callback(PreSetParametersCallbackType callback)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  ParameterMutationRecursionGuard guard(parameter_modification_enabled_);

  auto handle = std::make_shared<PreSetParametersCallbackHandle>();
  handle->callback = callback;
  // the last callback registered is executed first.
  pre_set_parameters_callback_container_.emplace_front(handle);
  return handle;
}

OnSetParametersCallbackHandle::SharedPtr
NodeParameters::add_on_set_parameters_callback(OnSetParametersCallbackType callback)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  ParameterMutationRecursionGuard guard(parameter_modification_enabled_);

  auto handle = std::make_shared<OnSetParametersCallbackHandle>();
  handle->callback = callback;
  // the last callback registered is executed first.
  on_set_parameters_callback_container_.emplace_front(handle);
  return handle;
}

PostSetParametersCallbackHandle::SharedPtr
NodeParameters::add_post_set_parameters_callback(
  PostSetParametersCallbackType callback)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  ParameterMutationRecursionGuard guard(parameter_modification_enabled_);

  auto handle = std::make_shared<PostSetParametersCallbackHandle>();
  handle->callback = callback;
  // the last callback registered is executed first.
  post_set_parameters_callback_container_.emplace_front(handle);
  return handle;
}

const std::map<std::string, rclcpp::ParameterValue> &
NodeParameters::get_parameter_overrides() const
{
  return parameter_overrides_;
}
