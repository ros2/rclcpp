// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__FUNCTION_TRAITS_HPP_
#define RCLCPP__FUNCTION_TRAITS_HPP_

#include <memory>
#include <tuple>

#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/* NOTE(esteve):
 * We support service callbacks that can optionally take the request id,
 * which should be possible with two overloaded create_service methods,
 * but unfortunately std::function's constructor on VS2015 is too greedy,
 * so we need a mechanism for checking the arity and the type of each argument
 * in a callback function.
 * See http://blogs.msdn.com/b/vcblog/archive/2015/06/19/c-11-14-17-features-in-vs-2015-rtm.aspx
 */

// Remove the first item in a tuple
template<typename T>
struct tuple_tail;

template<typename Head, typename ... Tail>
struct tuple_tail<std::tuple<Head, Tail ...>>
{
  using type = std::tuple<Tail ...>;
};

// std::function
template<typename FunctionT>
struct function_traits
{
  using arguments = typename tuple_tail<
      typename function_traits<decltype( & FunctionT::operator())>::arguments>::type;

  static constexpr std::size_t arity = std::tuple_size<arguments>::value;

  template<std::size_t N>
  using argument_type = typename std::tuple_element<N, arguments>::type;
};

// Free functions
template<typename ReturnTypeT, typename ... Args>
struct function_traits<ReturnTypeT(Args ...)>
{
  using arguments = std::tuple<Args ...>;

  static constexpr std::size_t arity = std::tuple_size<arguments>::value;

  template<std::size_t N>
  using argument_type = typename std::tuple_element<N, arguments>::type;
};

// Function pointers
template<typename ReturnTypeT, typename ... Args>
struct function_traits<ReturnTypeT (*)(Args ...)>: public function_traits<ReturnTypeT(Args ...)>
{};

// Lambdas
template<typename ClassT, typename ReturnTypeT, typename ... Args>
struct function_traits<ReturnTypeT (ClassT::*)(Args ...) const>
  : public function_traits<ReturnTypeT(ClassT &, Args ...)>
{};

/* NOTE(esteve):
 * VS2015 does not support expression SFINAE, so we're using this template to evaluate
 * the arity of a function.
 */
template<std::size_t Arity, typename FunctorT>
struct arity_comparator : std::integral_constant<
    bool, (Arity == function_traits<FunctorT>::arity)>{};

template<typename FunctorT, typename ... Args>
struct check_arguments : std::is_same<
    typename function_traits<FunctorT>::arguments,
    std::tuple<Args ...>
  >
{};

template<typename FunctorAT, typename FunctorBT>
struct same_arguments : std::is_same<
    typename function_traits<FunctorAT>::arguments,
    typename function_traits<FunctorBT>::arguments
  >
{};

}  // namespace rclcpp

#endif  // RCLCPP__FUNCTION_TRAITS_HPP_
