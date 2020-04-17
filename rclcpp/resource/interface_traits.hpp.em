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
@{
uppercase_interface_name = interface_name.upper()
interface_typename = ''.join([part.capitalize() for part in interface_name.split('_')])
}@

#ifndef RCLCPP__NODE_INTERFACES__@(uppercase_interface_name)_TRAITS_HPP_
#define RCLCPP__NODE_INTERFACES__@(uppercase_interface_name)_TRAITS_HPP_

#include <functional>
#include <type_traits>

#include "rclcpp/node_interfaces/@(interface_name).hpp"

namespace rclcpp
{
namespace node_interfaces
{

template<class T, typename = void>
struct has_@(interface_name) : std::false_type
{};

template<class T>
struct has_@(interface_name)<
  T, typename std::enable_if<
    std::is_same<
      std::shared_ptr<rclcpp::node_interfaces::@(interface_typename)>,
      decltype(std::declval<T>().get_@(interface_name)())>::value>::type> : std::true_type
{};

template<class T>
struct has_@(interface_name)<
  T, typename std::enable_if<
    std::is_same<
      std::shared_ptr<rclcpp::node_interfaces::@(interface_typename)>,
      decltype(std::declval<T>()->get_@(interface_name)())>::value>::type> : std::true_type
{};

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__@(uppercase_interface_name)_TRAITS_HPP_
