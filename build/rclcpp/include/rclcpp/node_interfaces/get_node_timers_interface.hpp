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


#ifndef RCLCPP__NODE_INTERFACES__GET_NODE_TIMERS_INTERFACE_HPP_
#define RCLCPP__NODE_INTERFACES__GET_NODE_TIMERS_INTERFACE_HPP_

#include <memory>
#include <utility>
#include <type_traits>

#include "rcpputils/pointer_traits.hpp"

#include "rclcpp/node_interfaces/node_timers_interface.hpp"
#include "rclcpp/node_interfaces/node_timers_interface_traits.hpp"


/// This header provides the get_node_timers_interface() template function.
/**
 * This function is useful for getting the NodeTimersInterface pointer from
 * various kinds of Node-like classes.
 *
 * It's able to get a std::shared_ptr to a NodeTimersInterface so long as the class
 * has a method called ``get_node_timers_interface()`` which returns one.
 */

namespace rclcpp
{
namespace node_interfaces
{
namespace detail
{

// If NodeType has a method called get_node_timers_interface() which returns a shared pointer.
template<
  typename NodeType,
  typename std::enable_if<has_node_timers_interface<
    typename rcpputils::remove_pointer<NodeType>::type
  >::value, int>::type = 0
>
std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface>
get_node_timers_interface_from_pointer(NodeType node_pointer)
{
  if (!node_pointer) {
    throw std::invalid_argument("node cannot be nullptr");
  }
  return node_pointer->get_node_timers_interface();
}

}  // namespace detail

/// Get the NodeTimersInterface as a shared pointer from a pointer to a "Node like" object.
template<
  typename NodeType,
  typename std::enable_if<
    rcpputils::is_pointer<NodeType>::value, int
  >::type = 0
>
inline
std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface>
get_node_timers_interface(NodeType && node)
{
  // Forward pointers to detail implementation directly.
  return detail::get_node_timers_interface_from_pointer(node);
}

/// Get the NodeTimersInterface as a shared pointer from a "Node like" object.
template<
  typename NodeType,
  typename std::enable_if<
    !rcpputils::is_pointer<NodeType>::value, int
  >::type = 0
>
inline
std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface>
get_node_timers_interface(NodeType && node)
{
  // Forward references to detail implementation as a pointer.
  return detail::get_node_timers_interface_from_pointer(&node);
}

/// Keep the NodeTimersInterface a shared pointer.
inline
std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface>
get_node_timers_interface(
  std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> & node_interface)
{
  return node_interface;
}

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__GET_NODE_TIMERS_INTERFACE_HPP_
