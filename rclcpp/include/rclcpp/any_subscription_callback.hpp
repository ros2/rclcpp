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

#ifndef RCLCPP_RCLCPP_ANY_SUBSCRIPTION_CALLBACK_HPP_
#define RCLCPP_RCLCPP_ANY_SUBSCRIPTION_CALLBACK_HPP_

#include <rclcpp/function_traits.hpp>

#include <functional>
#include <memory>
#include <type_traits>

#include <rmw/types.h>

namespace rclcpp
{

namespace any_subscription_callback
{

template<typename MessageT>
struct AnySubscriptionCallback
{
  using SharedPtrCallback = std::function<void(const std::shared_ptr<MessageT>)>;
  using SharedPtrWithInfoCallback =
      std::function<void(const std::shared_ptr<MessageT>, const rmw_message_info_t &)>;
  using ConstSharedPtrCallback = std::function<void(const std::shared_ptr<const MessageT>)>;
  using ConstSharedPtrWithInfoCallback =
      std::function<void(const std::shared_ptr<const MessageT>, const rmw_message_info_t &)>;
  using UniquePtrCallback = std::function<void(std::unique_ptr<MessageT> &)>;
  using UniquePtrWithInfoCallback =
      std::function<void(std::unique_ptr<MessageT> &, const rmw_message_info_t &)>;

  SharedPtrCallback shared_ptr_callback;
  SharedPtrWithInfoCallback shared_ptr_with_info_callback;
  ConstSharedPtrCallback const_shared_ptr_callback;
  ConstSharedPtrWithInfoCallback const_shared_ptr_with_info_callback;
  UniquePtrCallback unique_ptr_callback;
  UniquePtrWithInfoCallback unique_ptr_with_info_callback;

  AnySubscriptionCallback()
  : shared_ptr_callback(nullptr), shared_ptr_with_info_callback(nullptr),
    const_shared_ptr_callback(nullptr), const_shared_ptr_with_info_callback(nullptr),
    unique_ptr_callback(nullptr), unique_ptr_with_info_callback(nullptr)
  {}

  AnySubscriptionCallback(const AnySubscriptionCallback &) = default;

  template<typename CallbackT,
  typename std::enable_if<
    function_traits<CallbackT>::arity == 1
  >::type * = nullptr,
  typename std::enable_if<
    std::is_same<
      typename function_traits<CallbackT>::template argument_type<0>,
      typename std::shared_ptr<MessageT>
    >::value
  >::type * = nullptr
  >
  void set(CallbackT callback)
  {
    shared_ptr_callback = callback;
  }

  template<typename CallbackT,
  typename std::enable_if<
    function_traits<CallbackT>::arity == 2
  >::type * = nullptr,
  typename std::enable_if<
    std::is_same<
      typename function_traits<CallbackT>::template argument_type<0>,
      typename std::shared_ptr<MessageT>
    >::value
  >::type * = nullptr
  >
  void set(CallbackT callback)
  {
    shared_ptr_with_info_callback = callback;
  }

  template<typename CallbackT,
  typename std::enable_if<
    function_traits<CallbackT>::arity == 1
  >::type * = nullptr,
  typename std::enable_if<
    std::is_same<
      typename function_traits<CallbackT>::template argument_type<0>,
      typename std::shared_ptr<const MessageT>
    >::value
  >::type * = nullptr
  >
  void set(CallbackT callback)
  {
    const_shared_ptr_callback = callback;
  }

  template<typename CallbackT,
  typename std::enable_if<
    function_traits<CallbackT>::arity == 2
  >::type * = nullptr,
  typename std::enable_if<
    std::is_same<
      typename function_traits<CallbackT>::template argument_type<0>,
      typename std::shared_ptr<const MessageT>
    >::value
  >::type * = nullptr
  >
  void set(CallbackT callback)
  {
    const_shared_ptr_with_info_callback = callback;
  }
/*
  template<typename CallbackT,
    typename std::enable_if<
      function_traits<CallbackT>::arity == 1
    >::type * = nullptr,
    typename std::enable_if<
      std::is_same<
        typename function_traits<CallbackT>::template argument_type<0>,
        typename std::unique_ptr<MessageT>
      >::value
    >::type * = nullptr
  >
  void set(CallbackT callback)
  {
    static_assert(std::is_same<
        typename function_traits<CallbackT>::template argument_type<0>,
        typename std::unique_ptr<MessageT>
      >::value, "Not a unique pointer");
    unique_ptr_callback = callback;
  }

  template<typename CallbackT,
    typename std::enable_if<
      function_traits<CallbackT>::arity == 2
    >::type * = nullptr,
    typename std::enable_if<
      std::is_same<
        typename function_traits<CallbackT>::template argument_type<0>,
        typename std::unique_ptr<MessageT>
      >::value
    >::type * = nullptr
  >
  void set(CallbackT callback)
  {
    unique_ptr_with_info_callback = callback;
  }
  */
};

} /* namespace any_subscription_callback */
} /* namespace rclcpp */

#endif /* RCLCPP_RCLCPP_ANY_SUBSCRIPTION_CALLBACK_HPP_ */
