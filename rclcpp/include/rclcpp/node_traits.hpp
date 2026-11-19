// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__NODE_TRAITS_HPP_
#define RCLCPP__NODE_TRAITS_HPP_

#include <type_traits>

/** \file
 * Compile-time checks for inheritance from rclcpp::Node.
 *
 * The traits ignore const, volatile, and references. Raw and smart pointers
 * are checked as their own types. Private, protected, and ambiguous bases count.
 * A non-union class argument must be complete unless it is rclcpp::Node.
 */

namespace rclcpp
{
class Node;

/// True for rclcpp::Node and classes derived from it.
template<typename T>
struct is_node : std::is_base_of<Node, std::remove_cvref_t<T>>
{};

/// Convenience variable for is_node<T>::value.
template<typename T>
inline constexpr bool is_node_v = is_node<T>::value;

}  // namespace rclcpp

#endif  // RCLCPP__NODE_TRAITS_HPP_
