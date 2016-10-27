// Copyright 2014 Open Source Robotics Foundation, Inc.
// //
// // Licensed under the Apache License, Version 2.0 (the "License");
// // you may not use this file except in compliance with the License.
// // You may obtain a copy of the License at
// //
// //     http://www.apache.org/licenses/LICENSE-2.0
// //
// // Unless required by applicable law or agreed to in writing, software
// // distributed under the License is distributed on an "AS IS" BASIS,
// // WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// // See the License for the specific language governing permissions and
// // limitations under the License.

#ifndef RCLCPP__IS_MANAGEABLE_HPP_
#define RCLCPP__IS_MANAGEABLE_HPP_

#include <utility>
#include <type_traits>

template<class T, typename = void>
struct has_on_activate
{
  static constexpr bool value = false;
};

template<class T>
struct has_on_activate<T, typename std::enable_if<std::is_same<void, decltype(std::declval<T>().on_activate())>::value>::type>
{
  static constexpr bool value = true;
};

template<class T, typename = void>
struct has_on_deactivate
{
  static constexpr bool value = false;
};

template<class T>
struct has_on_deactivate<T, typename std::enable_if<std::is_same<void, decltype(std::declval<T>().on_deactivate())>::value>::type>
{
  static constexpr bool value = true;
};

template<class T, typename = void>
struct is_manageable_node : std::false_type
{};

template<class T>
struct is_manageable_node<T, typename std::enable_if< has_on_activate<T>::value
                                                   && has_on_deactivate<T>::value >::type> : std::true_type
{};

#endif
