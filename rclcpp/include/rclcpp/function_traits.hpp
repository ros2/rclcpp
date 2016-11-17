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

#if defined _LIBCPP_VERSION  // libc++ (Clang)
#define RCLCPP__FUNCTION_TRAITS_BIND_OBJECT_MEMBER_SIGNATURE \
  std::__1::__bind<ReturnTypeT (ClassT::*)(Args ...), FArgs ...>
#elif defined __GLIBCXX__  // glibc++ (GNU C++)
#define RCLCPP__FUNCTION_TRAITS_BIND_OBJECT_MEMBER_SIGNATURE \
  std::_Bind<std::_Mem_fn<ReturnTypeT (ClassT::*)(Args ...)>(FArgs ...)>
#elif defined _MSC_VER  // MS Visual Studio
#define RCLCPP__FUNCTION_TRAITS_BIND_OBJECT_MEMBER_SIGNATURE \
  std::_Binder<std::_Unforced, ReturnTypeT(__cdecl ClassT::*)(Args ...), FArgs ...>
#else
#error "Unsupported C++ compiler / standard library"
#endif

#if defined _LIBCPP_VERSION  // libc++ (Clang)
#define RCLCPP__FUNCTION_TRAITS_BIND_FREE_FUNCTION_SIGNATURE \
  std::__1::__bind<ReturnTypeT( &)(Args ...), FArgs ...>
#elif defined __GLIBCXX__  // glibc++ (GNU C++)
#define RCLCPP__FUNCTION_TRAITS_BIND_FREE_FUNCTION_SIGNATURE \
  std::_Bind<ReturnTypeT(*(FArgs ...))(Args ...)>
#elif defined _MSC_VER  // MS Visual Studio
#define RCLCPP__FUNCTION_TRAITS_BIND_FREE_FUNCTION_SIGNATURE \
  std::_Binder<std::_Unforced, ReturnTypeT(__cdecl &)(Args ...), FArgs ...>
#else
#error "Unsupported C++ compiler / standard library"
#endif

#include <functional>
#include <memory>
#include <tuple>

namespace rclcpp
{

namespace function_traits
{

/* NOTE(esteve):
 * We support service callbacks that can optionally take the request id,
 * which should be possible with two overloaded create_service methods,
 * but unfortunately std::function's constructor on VS2015 is too greedy,
 * so we need a mechanism for checking the arity and the type of each argument
 * in a callback function.
 * See http://blogs.msdn.com/b/vcblog/archive/2015/06/19/c-11-14-17-features-in-vs-2015-rtm.aspx
 */

template<typename TupleA, typename TupleB>
struct tuple_concat;

template<typename ... TupleArgsA, typename ... TupleArgsB>
struct tuple_concat<std::tuple<TupleArgsA ...>, std::tuple<TupleArgsB ...>>
{
  using type = std::tuple<TupleArgsA ..., TupleArgsB ...>;
};

template<std::size_t N, typename T>
struct tuple_cut;

template<std::size_t N, typename Head, typename ... Tail>
struct tuple_cut<N, std::tuple<Head, Tail ...>>
{
  using type = typename tuple_concat<
      std::tuple<Head>,
      typename tuple_cut<N - 1, std::tuple<Tail ...>>::type
      >::type;
};

template<typename Head, typename ... Tail>
struct tuple_cut<1, std::tuple<Head, Tail ...>>
{
  using type = std::tuple<Head>;
};

// Remove the first item in a tuple
template<typename T>
struct tuple_tail;

template<typename Head, typename ... Tail>
struct tuple_tail<std::tuple<Head, Tail ...>>
{
  using type = std::tuple<Tail ...>;
};

struct strict_tag {};

struct relaxed_tag {};

template<typename T>
struct argument_checking_traits
{
  using tag = strict_tag;
};

template<typename ClassT, typename ReturnTypeT, typename ... Args, typename ... FArgs>
struct argument_checking_traits<RCLCPP__FUNCTION_TRAITS_BIND_OBJECT_MEMBER_SIGNATURE>
{
  using tag = relaxed_tag;
};

template<typename ReturnTypeT, typename ... Args, typename ... FArgs>
struct argument_checking_traits<RCLCPP__FUNCTION_TRAITS_BIND_FREE_FUNCTION_SIGNATURE>
{
  using tag = relaxed_tag;
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

  using return_type = typename function_traits<decltype( & FunctionT::operator())>::return_type;
};

// Free functions
template<typename ReturnTypeT, typename ... Args>
struct function_traits<ReturnTypeT(Args ...)>
{
  using arguments = std::tuple<Args ...>;

  static constexpr std::size_t arity = std::tuple_size<arguments>::value;

  template<std::size_t N>
  using argument_type = typename std::tuple_element<N, arguments>::type;

  using return_type = ReturnTypeT;
};

// Function pointers
template<typename ReturnTypeT, typename ... Args>
struct function_traits<ReturnTypeT (*)(Args ...)>: function_traits<ReturnTypeT(Args ...)>
{};

// std::bind for object methods
template<typename ClassT, typename ReturnTypeT, typename ... Args, typename ... FArgs>
struct function_traits<RCLCPP__FUNCTION_TRAITS_BIND_OBJECT_MEMBER_SIGNATURE>
  : function_traits<ReturnTypeT(Args ...)>
{};

// std::bind for free functions
template<typename ReturnTypeT, typename ... Args, typename ... FArgs>
struct function_traits<RCLCPP__FUNCTION_TRAITS_BIND_FREE_FUNCTION_SIGNATURE>
  : function_traits<ReturnTypeT(Args ...)>
{};

// Lambdas
template<typename ClassT, typename ReturnTypeT, typename ... Args>
struct function_traits<ReturnTypeT (ClassT::*)(Args ...) const>
  : function_traits<ReturnTypeT(ClassT &, Args ...)>
{};

template<typename FunctionT>
struct function_traits<FunctionT &>: function_traits<FunctionT>
{};

template<typename FunctionT>
struct function_traits<FunctionT &&>: function_traits<FunctionT>
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

template<typename FunctorAT, typename FunctorBT,
typename Tag = typename argument_checking_traits<FunctorAT>::tag>
struct same_arguments : std::is_same<
    typename function_traits<FunctorAT>::arguments,
    typename function_traits<FunctorBT>::arguments
  >
{};

template<typename FunctorAT, typename FunctorBT>
struct same_arguments<FunctorAT, FunctorBT, relaxed_tag>: std::is_same<
    typename tuple_cut<
      function_traits<FunctorBT>::arity,
      typename function_traits<FunctorAT>::arguments
    >::type,
    typename function_traits<FunctorBT>::arguments
  >
{};

}  // namespace function_traits

}  // namespace rclcpp

#endif  // RCLCPP__FUNCTION_TRAITS_HPP_
