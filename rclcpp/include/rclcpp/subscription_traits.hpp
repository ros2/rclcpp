// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__SUBSCRIPTION_TRAITS_HPP_
#define RCLCPP__SUBSCRIPTION_TRAITS_HPP_

#include "rclcpp/function_traits.hpp"

namespace rclcpp
{
namespace subscription_traits
{

template<typename T>
struct is_raw_subscription_argument : std::false_type
{};

template<>
struct is_raw_subscription_argument<rcl_message_raw_t> : std::true_type
{};

template<>
struct is_raw_subscription_argument<std::shared_ptr<rcl_message_raw_t>> : std::true_type
{};

template<typename T>
struct is_raw_subscription : is_raw_subscription_argument<T>
{};

template<typename CallbackT>
struct is_raw_callback
  : is_raw_subscription_argument<
    typename rclcpp::function_traits::function_traits<CallbackT>::template argument_type<0>>
{};

template<typename MessageT>
struct extract_message_type
{
  using type = typename std::remove_cv<MessageT>::type;
};

template<typename MessageT>
struct extract_message_type<std::shared_ptr<MessageT>> : extract_message_type<MessageT>
{};

template<typename MessageT>
struct extract_message_type<std::unique_ptr<MessageT>> : extract_message_type<MessageT>
{};

template<typename CallbackT>
struct has_message_type
: extract_message_type<
    typename rclcpp::function_traits::function_traits<CallbackT>::template argument_type<0>>
{};

}  // namespace subscription_traits
}  // namespace rclcpp

#endif  // RCLCPP__SUBSCRIPTION_TRAITS_HPP_
