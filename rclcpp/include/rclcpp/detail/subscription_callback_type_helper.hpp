// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__DETAIL__SUBSCRIPTION_CALLBACK_TYPE_HELPER_HPP_
#define RCLCPP__DETAIL__SUBSCRIPTION_CALLBACK_TYPE_HELPER_HPP_

#include <memory>
#include <type_traits>

#include "rclcpp/function_traits.hpp"
#include "rclcpp/message_info.hpp"

namespace rclcpp
{
namespace detail
{

/// Template metaprogramming helper used to resolve the callback argument into a std::function.
/**
 * Sometimes the CallbackT is a std::function already, but it could also be a
 * function pointer, lambda, bind, or some variant of those.
 * In some cases, like a lambda where the arguments can be converted between one
 * another, e.g. std::function<void (shared_ptr<...>)> and
 * std::function<void (unique_ptr<...>)>, you need to make that not ambiguous
 * by checking the arguments independently using function traits rather than
 * rely on overloading the two std::function types.
 *
 * This issue, with the lambda's, can be demonstrated with this minimal program:
 *
 *   #include <functional>
 *   #include <memory>
 *
 *   void f(std::function<void (std::shared_ptr<int>)>) {}
 *   void f(std::function<void (std::unique_ptr<int>)>) {}
 *
 *   int main() {
 *     // Fails to compile with an "ambiguous call" error.
 *     f([](std::shared_ptr<int>){});
 *
 *     // Works.
 *     std::function<void (std::shared_ptr<int>)> cb = [](std::shared_ptr<int>){};
 *     f(cb);
 *   }
 *
 * If this program ever starts working in a future version of C++, this class
 * may become redundant.
 *
 * This helper works by using SFINAE with rclcpp::function_traits::same_arguments<>
 * to narrow down the exact std::function<> type for the given CallbackT.
 */
template<typename MessageT, typename CallbackT, typename Enable = void>
struct SubscriptionCallbackTypeHelper
{
  using callback_type = typename rclcpp::function_traits::as_std_function<CallbackT>::type;
};

template<typename MessageT, typename CallbackT>
struct SubscriptionCallbackTypeHelper<
  MessageT,
  CallbackT,
  typename std::enable_if_t<
    rclcpp::function_traits::same_arguments<
      CallbackT,
      std::function<void(std::shared_ptr<const MessageT>)>
    >::value
  >
>
{
  using callback_type = std::function<void (std::shared_ptr<const MessageT>)>;
};

template<typename MessageT, typename CallbackT>
struct SubscriptionCallbackTypeHelper<
  MessageT,
  CallbackT,
  typename std::enable_if_t<
    rclcpp::function_traits::same_arguments<
      CallbackT,
      std::function<void(std::shared_ptr<const MessageT>, const rclcpp::MessageInfo &)>
    >::value
  >
>
{
  using callback_type =
    std::function<void (std::shared_ptr<const MessageT>, const rclcpp::MessageInfo &)>;
};

template<typename MessageT, typename CallbackT>
struct SubscriptionCallbackTypeHelper<
  MessageT,
  CallbackT,
  typename std::enable_if_t<
    rclcpp::function_traits::same_arguments<
      CallbackT,
      std::function<void(const std::shared_ptr<const MessageT> &)>
    >::value
  >
>
{
  using callback_type = std::function<void (const std::shared_ptr<const MessageT> &)>;
};

template<typename MessageT, typename CallbackT>
struct SubscriptionCallbackTypeHelper<
  MessageT,
  CallbackT,
  typename std::enable_if_t<
    rclcpp::function_traits::same_arguments<
      CallbackT,
      std::function<void(const std::shared_ptr<const MessageT> &, const rclcpp::MessageInfo &)>
    >::value
  >
>
{
  using callback_type =
    std::function<void (const std::shared_ptr<const MessageT> &, const rclcpp::MessageInfo &)>;
};

template<typename MessageT, typename CallbackT>
struct SubscriptionCallbackTypeHelper<
  MessageT,
  CallbackT,
  typename std::enable_if_t<
    rclcpp::function_traits::same_arguments<
      CallbackT,
      std::function<void(std::shared_ptr<MessageT>)>
    >::value
  >
>
{
  using callback_type = std::function<void (std::shared_ptr<MessageT>)>;
};

template<typename MessageT, typename CallbackT>
struct SubscriptionCallbackTypeHelper<
  MessageT,
  CallbackT,
  typename std::enable_if_t<
    rclcpp::function_traits::same_arguments<
      CallbackT,
      std::function<void(std::shared_ptr<MessageT>, const rclcpp::MessageInfo &)>
    >::value
  >
>
{
  using callback_type =
    std::function<void (std::shared_ptr<MessageT>, const rclcpp::MessageInfo &)>;
};

}  // namespace detail
}  // namespace rclcpp

#endif  // RCLCPP__DETAIL__SUBSCRIPTION_CALLBACK_TYPE_HELPER_HPP_
