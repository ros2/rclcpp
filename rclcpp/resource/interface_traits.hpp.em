// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__NODE_INTERFACES__INTERFACE_TRAITS_HPP_
#define RCLCPP__NODE_INTERFACES__INTERFACE_TRAITS_HPP_

#include <functional>
#include <type_traits>

@{
node_interfaces = [
  'node_base_interface',
  'node_clock_interface',
  'node_graph_interface',
  'node_logging_interface',
  'node_parameters_interface',
  'node_services_interface',
  'node_time_source_interface',
  'node_timers_interface',
  'node_topics_interface',
  'node_waitables_interface',
]

node_interface_types = [
  'NodeBaseInterface',
  'NodeClockInterface',
  'NodeGraphInterface',
  'NodeLoggingInterface',
  'NodeParametersInterface',
  'NodeServicesInterface',
  'NodeTimeSourceInterface',
  'NodeTimersInterface',
  'NodeTopicsInterface',
  'NodeWaitablesInterface',
]

assert (len(node_interfaces) == len(node_interface_types))
}@

@[for interface_ in node_interfaces]@
#include "rclcpp/node_interfaces/@(interface_).hpp"
@[end for]@

namespace rclcpp
{
namespace node_interfaces
{

@[for (interface_, type_) in zip(node_interfaces, node_interface_types)]@
using @(interface_)_getter_t = std::shared_ptr<rclcpp::node_interfaces::@(type_)>;

template<class T, typename = void>
struct has_@(interface_) : std::false_type
{};

template<class T>
struct has_@(interface_)<
  T, typename std::enable_if<
    std::is_same<
      @(interface_)_getter_t, decltype(std::declval<T>().get_@(interface_)())>::value>::type> : std::true_type
{};

@[end for]@
}  // namespace node_interfaces
}  // namespace rclcpp
#endif  // RCLCPP__NODE_INTERFACES__INTERFACE_TRAITS_HPP_
