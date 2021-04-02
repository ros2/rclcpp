// Copyright 2019 Intel Corporation
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

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/parameter_event_handler.hpp"
#include "rcpputils/join.hpp"

namespace rclcpp
{

ParameterEventCallbackHandle::SharedPtr
ParameterEventHandler::add_parameter_event_callback(
  ParameterEventCallbackType callback)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto handle = std::make_shared<ParameterEventCallbackHandle>();
  handle->callback = callback;
  event_callbacks_.emplace_front(handle);

  return handle;
}

void
ParameterEventHandler::remove_parameter_event_callback(
  ParameterEventCallbackHandle::SharedPtr callback_handle)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto it = std::find_if(
    event_callbacks_.begin(),
    event_callbacks_.end(),
    [callback_handle](const auto & weak_handle) {
      return callback_handle.get() == weak_handle.lock().get();
    });
  if (it != event_callbacks_.end()) {
    event_callbacks_.erase(it);
  } else {
    throw std::runtime_error("Callback doesn't exist");
  }
}

ParameterCallbackHandle::SharedPtr
ParameterEventHandler::add_parameter_callback(
  const std::string & parameter_name,
  ParameterCallbackType callback,
  const std::string & node_name)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto full_node_name = resolve_path(node_name);

  auto handle = std::make_shared<ParameterCallbackHandle>();
  handle->callback = callback;
  handle->parameter_name = parameter_name;
  handle->node_name = full_node_name;
  // the last callback registered is executed first.
  parameter_callbacks_[{parameter_name, full_node_name}].emplace_front(handle);

  return handle;
}

void
ParameterEventHandler::remove_parameter_callback(
  ParameterCallbackHandle::SharedPtr callback_handle)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto handle = callback_handle.get();
  auto & container = parameter_callbacks_[{handle->parameter_name, handle->node_name}];
  auto it = std::find_if(
    container.begin(),
    container.end(),
    [handle](const auto & weak_handle) {
      return handle == weak_handle.lock().get();
    });
  if (it != container.end()) {
    container.erase(it);
    if (container.empty()) {
      parameter_callbacks_.erase({handle->parameter_name, handle->node_name});
    }
  } else {
    throw std::runtime_error("Callback doesn't exist");
  }
}

bool
ParameterEventHandler::get_parameter_from_event(
  const rcl_interfaces::msg::ParameterEvent & event,
  rclcpp::Parameter & parameter,
  const std::string parameter_name,
  const std::string node_name)
{
  if (event.node != node_name) {
    return false;
  }

  for (auto & new_parameter : event.new_parameters) {
    if (new_parameter.name == parameter_name) {
      parameter = rclcpp::Parameter::from_parameter_msg(new_parameter);
      return true;
    }
  }

  for (auto & changed_parameter : event.changed_parameters) {
    if (changed_parameter.name == parameter_name) {
      parameter = rclcpp::Parameter::from_parameter_msg(changed_parameter);
      return true;
    }
  }

  return false;
}

rclcpp::Parameter
ParameterEventHandler::get_parameter_from_event(
  const rcl_interfaces::msg::ParameterEvent & event,
  const std::string parameter_name,
  const std::string node_name)
{
  rclcpp::Parameter p;
  if (!get_parameter_from_event(event, p, parameter_name, node_name)) {
    throw std::runtime_error(
            "Parameter '" + parameter_name + "' of node '" + node_name +
            "' is not part of parameter event");
  }
  return p;
}

std::vector<rclcpp::Parameter>
ParameterEventHandler::get_parameters_from_event(
  const rcl_interfaces::msg::ParameterEvent & event)
{
  std::vector<rclcpp::Parameter> params;

  for (auto & new_parameter : event.new_parameters) {
    params.push_back(rclcpp::Parameter::from_parameter_msg(new_parameter));
  }

  for (auto & changed_parameter : event.changed_parameters) {
    params.push_back(rclcpp::Parameter::from_parameter_msg(changed_parameter));
  }

  return params;
}

void
ParameterEventHandler::event_callback(const rcl_interfaces::msg::ParameterEvent & event)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  for (auto it = parameter_callbacks_.begin(); it != parameter_callbacks_.end(); ++it) {
    rclcpp::Parameter p;
    if (get_parameter_from_event(event, p, it->first.first, it->first.second)) {
      for (auto cb = it->second.begin(); cb != it->second.end(); ++cb) {
        auto shared_handle = cb->lock();
        if (nullptr != shared_handle) {
          shared_handle->callback(p);
        } else {
          cb = it->second.erase(cb);
        }
      }
    }
  }

  for (auto event_cb = event_callbacks_.begin(); event_cb != event_callbacks_.end(); ++event_cb) {
    auto shared_event_handle = event_cb->lock();
    if (nullptr != shared_event_handle) {
      shared_event_handle->callback(event);
    } else {
      event_cb = event_callbacks_.erase(event_cb);
    }
  }
}

std::string
ParameterEventHandler::resolve_path(const std::string & path)
{
  std::string full_path;

  if (path == "") {
    full_path = node_base_->get_fully_qualified_name();
  } else {
    full_path = path;
    if (*path.begin() != '/') {
      auto ns = node_base_->get_namespace();
      const std::vector<std::string> paths{ns, path};
      full_path = (ns == std::string("/")) ? ns + path : rcpputils::join(paths, "/");
    }
  }

  return full_path;
}

}  // namespace rclcpp
