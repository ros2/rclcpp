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

#ifndef RCLCPP__DETAIL__TEMPLATE_UNIQUE_HPP_
#define RCLCPP__DETAIL__TEMPLATE_UNIQUE_HPP_

#include <type_traits>

#include "rclcpp/detail/template_contains.hpp"

namespace rclcpp
{
namespace detail
{

/// Template meta-function that checks if a given list Ts contains unique types.
template<typename ... Ts>
struct template_unique;

template<typename ... Args>
inline constexpr bool template_unique_v = template_unique<Args ...>::value;

template<typename NextT, typename ... Ts>
struct template_unique<NextT, Ts ...>
{
  enum { value = !template_contains_v<NextT, Ts ...>&& template_unique_v<Ts ...>};
};

template<typename T>
struct template_unique<T>
{
  enum { value = true };
};

}  // namespace detail
}  // namespace rclcpp

#endif  // RCLCPP__DETAIL__TEMPLATE_UNIQUE_HPP_
