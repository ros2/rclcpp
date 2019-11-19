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

#ifndef RCLCPP__PARAMETER_EVENTS_FILTER_HPP_
#define RCLCPP__PARAMETER_EVENTS_FILTER_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rmw/rmw.h"

namespace rclcpp
{

class ParameterEventsFilter
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(ParameterEventsFilter)
  enum class EventType {NEW, DELETED, CHANGED};  ///< An enum for the type of event.
  /// Used for the listed results
  using EventPair = std::pair<EventType, rcl_interfaces::msg::Parameter *>;

  /// Construct a filtered view of a parameter event.
  /**
   * \param[in] event The parameter event message to filter.
   * \param[in] names A list of parameter names of interest.
   * \param[in] types A list of the types of parameter events of iterest.
   *    EventType NEW, DELETED, or CHANGED
   *
   * Example Usage:
   *
   * If you have recieved a parameter event and are only interested in parameters foo and
   * bar being added or changed but don't care about deletion.
   *
   * ```cpp
   * auto res = rclcpp::ParameterEventsFilter(
   *   event_shared_ptr,
   *   {"foo", "bar"},
   *   {rclcpp::ParameterEventsFilter::EventType::NEW, rclcpp::ParameterEventsFilter::EventType::CHANGED});
   * ```
   */
  RCLCPP_PUBLIC
  ParameterEventsFilter(
    rcl_interfaces::msg::ParameterEvent::SharedPtr event,
    const std::vector<std::string> & names,
    const std::vector<EventType> & types);

  /// Get the result of the filter
  /**
   * \return A std::vector<EventPair> of all matching parameter changes in this event.
   */
  RCLCPP_PUBLIC
  const std::vector<EventPair> & get_events() const;

private:
  // access only allowed via const accessor.
  std::vector<EventPair> result_;  ///< Storage of the resultant vector
  rcl_interfaces::msg::ParameterEvent::SharedPtr event_;  ///< Keep event in scope
};

}  // namespace rclcpp

#endif  // RCLCPP__PARAMETER_EVENTS_FILTER_HPP_
