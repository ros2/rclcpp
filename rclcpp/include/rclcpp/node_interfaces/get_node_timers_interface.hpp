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

#ifndef RCLCPP__NODE_INTERFACES__GET_NODE_TIMERS_INTERFACE_HPP_
#define RCLCPP__NODE_INTERFACES__GET_NODE_TIMERS_INTERFACE_HPP_

#include <memory>
#include <utility>
#include <type_traits>

#include "rclcpp/node_interfaces/node_timers_interface.hpp"

/// This header provides the get_node_timers_interface() template function.
/**
 * This function is useful for getting the NodeTimersInterface pointer from
 * various kinds of Node-like classes.
 *
 * It's able to get the NodeTimersInterface pointer so long as the class
 * has a method called ``get_node_timers_interface()`` which returns
 * either a pointer (const or not) to a NodeTimersInterface or a
 * std::shared_ptr to a NodeTimersInterface.
 */

namespace rclcpp
{
namespace node_interfaces
{

namespace detail
{

// This is a meta-programming checker for if a given Node-like object has a
// getter called get_node_timers_interface() which returns various types,
// e.g. const pointer or a shared pointer.
template<typename NodeType, typename ReturnType>
struct has_get_node_timers_interface
{
private:
  template<typename T>
  static constexpr
  auto
  check(T *)->typename std::is_same<
    decltype(std::declval<T>().get_node_timers_interface()),
    ReturnType
  >::type;

  template<typename>
  static constexpr
  std::false_type
  check(...);

public:
  using type = decltype(check<NodeType>(nullptr));
  static constexpr bool value = type::value;
};

// If NodeType is a pointer to NodeTimersInterface already (just normal function overload).
inline
rclcpp::node_interfaces::NodeTimersInterface *
get_node_timers_interface_from_pointer(rclcpp::node_interfaces::NodeTimersInterface * pointer)
{
  return pointer;
}

// If NodeType has a method called get_node_timers_interface() which returns a shared pointer.
template<
  typename NodeType,
  typename std::enable_if<has_get_node_timers_interface<
    typename std::remove_pointer<NodeType>::type,
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface>
  >::value, int>::type = 0
>
rclcpp::node_interfaces::NodeTimersInterface *
get_node_timers_interface_from_pointer(NodeType node_pointer)
{
  return node_pointer->get_node_timers_interface().get();
}

// If NodeType has a method called get_node_timers_interface() which returns a pointer.
template<
  typename NodeType,
  typename std::enable_if<has_get_node_timers_interface<
    typename std::remove_pointer<NodeType>::type,
    rclcpp::node_interfaces::NodeTimersInterface *
  >::value, int>::type = 0
>
rclcpp::node_interfaces::NodeTimersInterface *
get_node_timers_interface_from_pointer(NodeType node_pointer)
{
  return node_pointer->get_node_timers_interface();
}

// Forward shared_ptr's to const node pointer signatures.
template<
  typename NodeType,
  typename std::enable_if<std::is_same<
    NodeType,
    typename std::shared_ptr<typename std::remove_pointer<NodeType>::type::element_type> *
  >::value, int>::type = 0
>
rclcpp::node_interfaces::NodeTimersInterface *
get_node_timers_interface_from_pointer(NodeType node_shared_pointer)
{
  return get_node_timers_interface_from_pointer(node_shared_pointer->get());
}

}  // namespace detail

/// Get the NodeTimersInterface as a pointer from a pointer to a "Node like" object.
template<
  typename NodeType,
  typename std::enable_if<std::is_pointer<NodeType>::value, int>::type = 0
>
rclcpp::node_interfaces::NodeTimersInterface *
get_node_timers_interface(NodeType node_pointer)
{
  // Forward pointers to detail implmentation directly.
  return detail::get_node_timers_interface_from_pointer(node_pointer);
}

/// Get the NodeTimersInterface as a pointer from a "Node like" object.
template<
  typename NodeType,
  typename std::enable_if<
    !std::is_pointer<typename std::remove_reference<NodeType>::type>::value, int
  >::type = 0
>
rclcpp::node_interfaces::NodeTimersInterface *
get_node_timers_interface(NodeType && node_reference)
{
  // Forward references to detail implmentation as a pointer.
  return detail::get_node_timers_interface_from_pointer(&node_reference);
}

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__GET_NODE_TIMERS_INTERFACE_HPP_
