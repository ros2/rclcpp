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
}@

#ifndef RCLCPP__NODE_INTERFACES__GET_@(uppercase_interface_name)_HPP_
#define RCLCPP__NODE_INTERFACES__GET_@(uppercase_interface_name)_HPP_

#include <memory>
#include <utility>
#include <type_traits>

#include "rcpputils/pointer_traits.hpp"

#include "rclcpp/node_interfaces/@(interface_name).hpp"
#include "rclcpp/node_interfaces/@(interface_name)_traits.hpp"

@{
interface_typename = ''.join([part.capitalize() for part in interface_name.split('_')])
}@

/// This header provides the get_@(interface_name)() template function.
/**
 * This function is useful for getting the @(interface_typename) pointer from
 * various kinds of Node-like classes.
 *
 * It's able to get a std::shared_ptr to a @(interface_typename) so long as the class
 * has a method called ``get_@(interface_name)()`` which returns one.
 */

namespace rclcpp
{
namespace node_interfaces
{
namespace detail
{

// If NodeType has a method called get_@(interface_name)() which returns a shared pointer.
template<
  typename NodeType,
  typename std::enable_if<has_@(interface_name)<
    typename rcpputils::remove_pointer<NodeType>::type
  >::value, int>::type = 0
>
std::shared_ptr<rclcpp::node_interfaces::@(interface_typename)>
get_@(interface_name)_from_pointer(NodeType node_pointer)
{
  if (!node_pointer) {
    throw std::invalid_argument("node cannot be nullptr");
  }
  return node_pointer->get_@(interface_name)();
}

}  // namespace detail

/// Get the @(interface_typename) as a shared pointer from a pointer to a "Node like" object.
template<
  typename NodeType,
  typename std::enable_if<
    rcpputils::is_pointer<NodeType>::value, int
  >::type = 0
>
inline
std::shared_ptr<rclcpp::node_interfaces::@(interface_typename)>
get_@(interface_name)(NodeType && node)
{
  // Forward pointers to detail implementation directly.
  return detail::get_@(interface_name)_from_pointer(node);
}

/// Get the @(interface_typename) as a shared pointer from a "Node like" object.
template<
  typename NodeType,
  typename std::enable_if<
    !rcpputils::is_pointer<NodeType>::value, int
  >::type = 0
>
inline
std::shared_ptr<rclcpp::node_interfaces::@(interface_typename)>
get_@(interface_name)(NodeType && node)
{
  // Forward references to detail implementation as a pointer.
  return detail::get_@(interface_name)_from_pointer(&node);
}

/// Keep the @(interface_typename) a shared pointer.
inline
std::shared_ptr<rclcpp::node_interfaces::@(interface_typename)>
get_@(interface_name)(
  std::shared_ptr<rclcpp::node_interfaces::@(interface_typename)> & node_interface)
{
  return node_interface;
}

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__GET_@(uppercase_interface_name)_HPP_
