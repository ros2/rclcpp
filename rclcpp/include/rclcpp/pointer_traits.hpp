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

#ifndef RCLCPP__POINTER_TRAITS_HPP_
#define RCLCPP__POINTER_TRAITS_HPP_

#include <memory>
#include <type_traits>

namespace rclcpp
{

template<class T>
struct is_smart_pointer_helper : std::false_type
{};

template<class T>
struct is_smart_pointer_helper<std::shared_ptr<T>>: std::true_type
{};

template<class T>
struct is_smart_pointer_helper<std::unique_ptr<T>>: std::true_type
{};

template<class T>
struct is_smart_pointer : is_smart_pointer_helper<typename std::remove_cv<T>::type>
{};

template<class T>
struct is_pointer
{
  static constexpr bool value = std::is_pointer<T>::value || is_smart_pointer<T>::value;
};

}  // namespace rclcpp

#endif  // RCLCPP__POINTER_TRAITS_HPP_
