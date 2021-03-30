// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/parameter_events_filter.hpp"

#include <memory>
#include <string>
#include <vector>

using rclcpp::ParameterEventsFilter;
using EventType = rclcpp::ParameterEventsFilter::EventType;
using EventPair = rclcpp::ParameterEventsFilter::EventPair;

ParameterEventsFilter::ParameterEventsFilter(
  std::shared_ptr<const rcl_interfaces::msg::ParameterEvent> event,
  const std::vector<std::string> & names,
  const std::vector<EventType> & types)
: event_(event)
{
  if (std::find(types.begin(), types.end(), EventType::NEW) != types.end()) {
    for (auto & new_parameter : event->new_parameters) {
      if (std::find(names.begin(), names.end(), new_parameter.name) != names.end()) {
        result_.push_back(
          EventPair(EventType::NEW, &new_parameter));
      }
    }
  }
  if (std::find(types.begin(), types.end(), EventType::CHANGED) != types.end()) {
    for (auto & changed_parameter : event->changed_parameters) {
      if (std::find(names.begin(), names.end(), changed_parameter.name) != names.end()) {
        result_.push_back(
          EventPair(EventType::CHANGED, &changed_parameter));
      }
    }
  }
  if (std::find(types.begin(), types.end(), EventType::DELETED) != types.end()) {
    for (auto & deleted_parameter : event->deleted_parameters) {
      if (std::find(names.begin(), names.end(), deleted_parameter.name) != names.end()) {
        result_.push_back(
          EventPair(EventType::DELETED, &deleted_parameter));
      }
    }
  }
}

const std::vector<EventPair> &
ParameterEventsFilter::get_events() const
{
  return result_;
}
