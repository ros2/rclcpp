// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__DETAIL__TEMPLATE_CONTAINS_HPP_
#define RCLCPP__DETAIL__TEMPLATE_CONTAINS_HPP_

#include <type_traits>

namespace rclcpp
{
namespace detail
{

/// Template meta-function that checks if a given T is contained in the list Us.
template<typename T, typename ... Us>
struct template_contains;

template<typename ... Args>
inline constexpr bool template_contains_v = template_contains<Args ...>::value;

template<typename T, typename NextT, typename ... Us>
struct template_contains<T, NextT, Us ...>
{
  enum { value = (std::is_same_v<T, NextT>|| template_contains_v<T, Us ...>)};
};

template<typename T>
struct template_contains<T>
{
  enum { value = false };
};

}  // namespace detail
}  // namespace rclcpp

#endif  // RCLCPP__DETAIL__TEMPLATE_CONTAINS_HPP_
