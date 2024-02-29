// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__ANY_SERVICE_CALLBACK_HPP_
#define RCLCPP__ANY_SERVICE_CALLBACK_HPP_

#include <variant>
#include <functional>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "rosidl_runtime_cpp/traits.hpp"

#include "rclcpp/function_traits.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/type_adapter.hpp"
#include "rmw/types.h"
#include "tracetools/tracetools.h"
#include "tracetools/utils.hpp"

namespace rclcpp
{

namespace detail
{
template<typename T, typename = void>
struct can_be_nullptr : std::false_type {};

// Some lambdas define a comparison with nullptr,
// but we see a warning that they can never be null when using it.
// We also test if `T &` can be assigned to `nullptr` to avoid the issue.
template<typename T>
#ifdef __QNXNTO__
struct can_be_nullptr<T, std::void_t<
    decltype(std::declval<T>() == nullptr)>>: std::true_type {};
#else
struct can_be_nullptr<T, std::void_t<
    decltype(std::declval<T>() == nullptr), decltype(std::declval<T &>() = nullptr)>>
  : std::true_type {};
#endif
}  // namespace detail

// Forward declare
template<typename ServiceT>
class Service;

template<typename ServiceT>
class AnyServiceCallback
{
public:
  using ServiceRequestType =
    typename TypeAdapter<ServiceT>::custom_type::Request;
  /// ServiceT::ros_message_type::Request if ServiceT is a TypeAdapter, otherwise just the
  /// ServiceT::Request
  using ROSServiceRequestType =
    typename TypeAdapter<ServiceT>::ros_message_type::Request;
  /// ServiceT::custom_type::Response if ServiceT is a TypeAdapter, otherwise just the
  /// ServiceT::Response
  using ServiceResponseType =
    typename TypeAdapter<ServiceT>::custom_type::Response;
  /// ServiceT::ros_message_type::Response if ServiceT is a TypeAdapter, otherwise just the
  /// ServiceT::Response
  using ROSServiceResponseType =
    typename TypeAdapter<ServiceT>::ros_message_type::Response;

private:
  using ROSTotalSharedPtrCallback = std::function<
    void (
      std::shared_ptr<ROSServiceRequestType>,
      std::shared_ptr<ROSServiceResponseType>
    )>;
  using ROSTotalSharedPtrWithRequestHeaderCallback = std::function<
    void (
      std::shared_ptr<rmw_request_id_t>,
      std::shared_ptr<ROSServiceRequestType>,
      std::shared_ptr<ROSServiceResponseType>
    )>;
  using CustomTotalSharedPtrCallback = std::function<
    void (
      std::shared_ptr<ServiceRequestType>,
      std::shared_ptr<ServiceResponseType>
    )>;
  using CustomTotalSharedPtrWithRequestHeaderCallback = std::function<
    void (
      std::shared_ptr<rmw_request_id_t>,
      std::shared_ptr<ServiceRequestType>,
      std::shared_ptr<ServiceResponseType>
    )>;
  using ROSCustomSharedPtrCallback = std::function<
    void (
      std::shared_ptr<ROSServiceRequestType>,
      std::shared_ptr<ServiceResponseType>
    )>;
  using ROSCustomSharedPtrWithRequestHeaderCallback = std::function<
    void (
      std::shared_ptr<rmw_request_id_t>,
      std::shared_ptr<ROSServiceRequestType>,
      std::shared_ptr<ServiceResponseType>
    )>;
  using CustomROSSharedPtrCallback = std::function<
    void (
      std::shared_ptr<ServiceRequestType>,
      std::shared_ptr<ROSServiceResponseType>
    )>;
  using CustomROSSharedPtrWithRequestHeaderCallback = std::function<
    void (
      std::shared_ptr<rmw_request_id_t>,
      std::shared_ptr<ServiceRequestType>,
      std::shared_ptr<ROSServiceResponseType>
    )>;
  using ROSSharedPtrDeferResponseCallback = std::function<
    void (
      std::shared_ptr<rmw_request_id_t>,
      std::shared_ptr<ROSServiceRequestType>
    )>;
  using ROSSharedPtrDeferResponseCallbackWithServiceHandle = std::function<
    void (
      std::shared_ptr<rclcpp::Service<ServiceT>>,
      std::shared_ptr<rmw_request_id_t>,
      std::shared_ptr<ROSServiceRequestType>
    )>;
  using CustomSharedPtrDeferResponseCallback = std::function<
    void (
      std::shared_ptr<rmw_request_id_t>,
      std::shared_ptr<ServiceRequestType>
    )>;
  using CustomSharedPtrDeferResponseCallbackWithServiceHandle = std::function<
    void (
      std::shared_ptr<rclcpp::Service<ServiceT>>,
      std::shared_ptr<rmw_request_id_t>,
      std::shared_ptr<ServiceRequestType>
    )>;

  template<bool is_adapted_type = rclcpp::TypeAdapter<ServiceT>::is_specialized::value>
  struct AnyServiceCallbackHelper;

  template<>
  struct AnyServiceCallbackHelper<true>
  {
    using variant_type = std::variant<
      std::monostate,
      ROSTotalSharedPtrCallback,
      ROSTotalSharedPtrWithRequestHeaderCallback,
      CustomTotalSharedPtrCallback,
      CustomTotalSharedPtrWithRequestHeaderCallback,
      ROSCustomSharedPtrCallback,
      ROSCustomSharedPtrWithRequestHeaderCallback,
      CustomROSSharedPtrCallback,
      CustomROSSharedPtrWithRequestHeaderCallback,
      ROSSharedPtrDeferResponseCallback,
      ROSSharedPtrDeferResponseCallbackWithServiceHandle,
      CustomSharedPtrDeferResponseCallback,
      CustomSharedPtrDeferResponseCallbackWithServiceHandle>;
  };

  template<>
  struct AnyServiceCallbackHelper<false>
  {
    using variant_type = std::variant<
      std::monostate,
      ROSTotalSharedPtrCallback,
      ROSTotalSharedPtrWithRequestHeaderCallback,
      ROSSharedPtrDeferResponseCallback,
      ROSSharedPtrDeferResponseCallbackWithServiceHandle>;
  };

  using CallbackHelperT = AnyServiceCallbackHelper<>;

  typename CallbackHelperT::variant_type callback_;

public:
  AnyServiceCallback()
  : callback_(std::monostate{})
  {}

  template<
    typename CallbackT,
    typename std::enable_if_t<!detail::can_be_nullptr<CallbackT>::value, int> = 0>
  void
  set(CallbackT && callback)
  {
    // Workaround Windows issue with std::bind
    if constexpr (
      rclcpp::function_traits::same_arguments<
        CallbackT,
        ROSTotalSharedPtrCallback
      >::value)
    {
      callback_.template emplace<ROSTotalSharedPtrCallback>(callback);
    } else if constexpr (  // NOLINT, can't satisfy both cpplint and uncrustify
      rclcpp::function_traits::same_arguments<
        CallbackT,
        ROSTotalSharedPtrWithRequestHeaderCallback
      >::value)
    {
      callback_.template emplace<ROSTotalSharedPtrWithRequestHeaderCallback>(callback);
    } else if constexpr (  // NOLINT
      rclcpp::function_traits::same_arguments<
        CallbackT,
        CustomTotalSharedPtrCallback
      >::value)
    {
      callback_.template emplace<CustomTotalSharedPtrCallback>(callback);
    } else if constexpr (  // NOLINT
      rclcpp::function_traits::same_arguments<
        CallbackT,
        CustomTotalSharedPtrWithRequestHeaderCallback
      >::value)
    {
      callback_.template emplace<CustomTotalSharedPtrWithRequestHeaderCallback>(callback);
    } else if constexpr (  // NOLINT
      rclcpp::function_traits::same_arguments<
        CallbackT,
        ROSCustomSharedPtrCallback
      >::value)
    {
      callback_.template emplace<ROSCustomSharedPtrCallback>(callback);
    } else if constexpr (  // NOLINT
      rclcpp::function_traits::same_arguments<
        CallbackT,
        ROSCustomSharedPtrWithRequestHeaderCallback
      >::value)
    {
      callback_.template emplace<ROSCustomSharedPtrWithRequestHeaderCallback>(callback);
    } else if constexpr (  // NOLINT
      rclcpp::function_traits::same_arguments<
        CallbackT,
        CustomROSSharedPtrCallback
      >::value)
    {
      callback_.template emplace<CustomROSSharedPtrCallback>(callback);
    } else if constexpr (  // NOLINT
      rclcpp::function_traits::same_arguments<
        CallbackT,
        CustomROSSharedPtrWithRequestHeaderCallback
      >::value)
    {
      callback_.template emplace<CustomROSSharedPtrWithRequestHeaderCallback>(callback);
    } else if constexpr (  // NOLINT
      rclcpp::function_traits::same_arguments<
        CallbackT,
        ROSSharedPtrDeferResponseCallback
      >::value)
    {
      callback_.template emplace<ROSSharedPtrDeferResponseCallback>(callback);
    } else if constexpr (  // NOLINT
      rclcpp::function_traits::same_arguments<
        CallbackT,
        ROSSharedPtrDeferResponseCallbackWithServiceHandle
      >::value)
    {
      callback_.template emplace<
        ROSSharedPtrDeferResponseCallbackWithServiceHandle>(callback);
    } else if constexpr (  // NOLINT
      rclcpp::function_traits::same_arguments<
        CallbackT,
        CustomSharedPtrDeferResponseCallback
      >::value)
    {
      callback_.template emplace<CustomSharedPtrDeferResponseCallback>(callback);
    } else if constexpr (  // NOLINT
      rclcpp::function_traits::same_arguments<
        CallbackT,
        CustomSharedPtrDeferResponseCallbackWithServiceHandle
      >::value)
    {
      callback_.template emplace<
        CustomSharedPtrDeferResponseCallbackWithServiceHandle>(callback);
    } else {
      // the else clause is not needed, but anyways we should only be doing this instead
      // of all the above workaround ...
      callback_ = std::forward<CallbackT>(callback);
    }
  }

  template<
    typename CallbackT,
    typename std::enable_if_t<detail::can_be_nullptr<CallbackT>::value, int> = 0>
  void
  set(CallbackT && callback)
  {
    if (!callback) {
      throw std::invalid_argument("AnyServiceCallback::set(): callback cannot be nullptr");
    }
    // Workaround Windows issue with std::bind
    if constexpr (
      rclcpp::function_traits::same_arguments<
        CallbackT,
        ROSTotalSharedPtrCallback
      >::value)
    {
      callback_.template emplace<ROSTotalSharedPtrCallback>(callback);
    } else if constexpr (  // NOLINT, can't satisfy both cpplint and uncrustify
      rclcpp::function_traits::same_arguments<
        CallbackT,
        ROSTotalSharedPtrWithRequestHeaderCallback
      >::value)
    {
      callback_.template emplace<ROSTotalSharedPtrWithRequestHeaderCallback>(callback);
    } else if constexpr (  // NOLINT
      rclcpp::function_traits::same_arguments<
        CallbackT,
        CustomTotalSharedPtrCallback
      >::value)
    {
      callback_.template emplace<CustomTotalSharedPtrCallback>(callback);
    } else if constexpr (  // NOLINT
      rclcpp::function_traits::same_arguments<
        CallbackT,
        CustomTotalSharedPtrWithRequestHeaderCallback
      >::value)
    {
      callback_.template emplace<CustomTotalSharedPtrWithRequestHeaderCallback>(callback);
    } else if constexpr (  // NOLINT
      rclcpp::function_traits::same_arguments<
        CallbackT,
        ROSCustomSharedPtrCallback
      >::value)
    {
      callback_.template emplace<ROSCustomSharedPtrCallback>(callback);
    } else if constexpr (  // NOLINT
      rclcpp::function_traits::same_arguments<
        CallbackT,
        ROSCustomSharedPtrWithRequestHeaderCallback
      >::value)
    {
      callback_.template emplace<ROSCustomSharedPtrWithRequestHeaderCallback>(callback);
    } else if constexpr (  // NOLINT
      rclcpp::function_traits::same_arguments<
        CallbackT,
        CustomROSSharedPtrCallback
      >::value)
    {
      callback_.template emplace<CustomROSSharedPtrCallback>(callback);
    } else if constexpr (  // NOLINT
      rclcpp::function_traits::same_arguments<
        CallbackT,
        CustomROSSharedPtrWithRequestHeaderCallback
      >::value)
    {
      callback_.template emplace<CustomROSSharedPtrWithRequestHeaderCallback>(callback);
    } else if constexpr (  // NOLINT
      rclcpp::function_traits::same_arguments<
        CallbackT,
        ROSSharedPtrDeferResponseCallback
      >::value)
    {
      callback_.template emplace<ROSSharedPtrDeferResponseCallback>(callback);
    } else if constexpr (  // NOLINT
      rclcpp::function_traits::same_arguments<
        CallbackT,
        ROSSharedPtrDeferResponseCallbackWithServiceHandle
      >::value)
    {
      callback_.template emplace<
        ROSSharedPtrDeferResponseCallbackWithServiceHandle>(callback);
    } else if constexpr (  // NOLINT
      rclcpp::function_traits::same_arguments<
        CallbackT,
        CustomSharedPtrDeferResponseCallback
      >::value)
    {
      callback_.template emplace<CustomSharedPtrDeferResponseCallback>(callback);
    } else if constexpr (  // NOLINT
      rclcpp::function_traits::same_arguments<
        CallbackT,
        CustomSharedPtrDeferResponseCallbackWithServiceHandle
      >::value)
    {
      callback_.template emplace<
        CustomSharedPtrDeferResponseCallbackWithServiceHandle>(callback);
    } else {
      // the else clause is not needed, but anyways we should only be doing this instead
      // of all the above workaround ...
      callback_ = std::forward<CallbackT>(callback);
    }
  }

  // template<typename Allocator = std::allocator<typename ServiceT::Response>>
  template<typename T>
  typename std::enable_if_t<
    rosidl_generator_traits::is_message<T>::value &&
    std::is_same<T, ROSServiceRequestType>::value,
    std::shared_ptr<ROSServiceResponseType>>
  dispatch(
    const std::shared_ptr<rclcpp::Service<ServiceT>> & service_handle,
    const std::shared_ptr<rmw_request_id_t> & request_header,
    std::shared_ptr<T> request)
  {
    TRACETOOLS_TRACEPOINT(callback_start, static_cast<const void *>(this), false);
    if (std::holds_alternative<std::monostate>(callback_)) {
      // TODO(ivanpauno): Remove the set method, and force the users of this class
      // to pass a callback at construnciton.
      throw std::runtime_error{"unexpected request without any callback set"};
    }
    if (std::holds_alternative<ROSSharedPtrDeferResponseCallback>(callback_)) {
      const auto & cb = std::get<ROSSharedPtrDeferResponseCallback>(callback_);
      cb(request_header, std::move(request));
      return nullptr;
    }
    if (std::holds_alternative<ROSSharedPtrDeferResponseCallbackWithServiceHandle>(callback_)) {
      const auto & cb = std::get<ROSSharedPtrDeferResponseCallbackWithServiceHandle>(callback_);
      cb(service_handle, request_header, std::move(request));
      return nullptr;
    }
    // auto response = allocate_shared<typename ServiceT::Response, Allocator>();
    auto response = std::make_shared<ROSServiceResponseType>();
    if (std::holds_alternative<ROSTotalSharedPtrCallback>(callback_)) {
      (void)request_header;
      const auto & cb = std::get<ROSTotalSharedPtrCallback>(callback_);
      cb(std::move(request), response);
    } else if (std::holds_alternative<ROSTotalSharedPtrWithRequestHeaderCallback>(callback_)) {
      const auto & cb = std::get<ROSTotalSharedPtrWithRequestHeaderCallback>(callback_);
      cb(request_header, std::move(request), response);
    }
    TRACETOOLS_TRACEPOINT(callback_end, static_cast<const void *>(this));
    return response;
  }

  template<typename T>
  typename std::enable_if_t<
    !rosidl_generator_traits::is_message<T>::value &&
    std::is_same<T, ServiceRequestType>::value,
    std::shared_ptr<ServiceResponseType>>
  dispatch(
    const std::shared_ptr<rclcpp::Service<ServiceT>> & service_handle,
    const std::shared_ptr<rmw_request_id_t> & request_header,
    std::shared_ptr<T> request)
  {
    TRACETOOLS_TRACEPOINT(callback_start, static_cast<const void *>(this), false);
    if (std::holds_alternative<std::monostate>(callback_)) {
      // TODO(ivanpauno): Remove the set method, and force the users of this class
      // to pass a callback at construnciton.
      throw std::runtime_error{"unexpected request without any callback set"};
    }
    if (std::holds_alternative<CustomSharedPtrDeferResponseCallback>(callback_)) {
      const auto & cb = std::get<CustomSharedPtrDeferResponseCallback>(callback_);
      cb(request_header, std::move(request));
      return nullptr;
    }
    if (std::holds_alternative<CustomSharedPtrDeferResponseCallbackWithServiceHandle>(callback_)) {
      const auto & cb = std::get<CustomSharedPtrDeferResponseCallbackWithServiceHandle>(callback_);
      cb(service_handle, request_header, std::move(request));
      return nullptr;
    }
    // auto response = allocate_shared<typename ServiceT::Response, Allocator>();
    auto response = std::make_shared<ServiceResponseType>();
    if (std::holds_alternative<CustomTotalSharedPtrCallback>(callback_)) {
      (void)request_header;
      const auto & cb = std::get<CustomTotalSharedPtrCallback>(callback_);
      cb(std::move(request), response);
    } else if (std::holds_alternative<CustomTotalSharedPtrWithRequestHeaderCallback>(callback_)) {
      const auto & cb = std::get<CustomTotalSharedPtrWithRequestHeaderCallback>(callback_);
      cb(request_header, std::move(request), response);
    }
    TRACETOOLS_TRACEPOINT(callback_end, static_cast<const void *>(this));
    return response;
  }

  typename CallbackHelperT::variant_type &
  get_variant()
  {
    return callback_;
  }

  const typename CallbackHelperT::variant_type &
  get_variant() const
  {
    return callback_;
  }

  void register_callback_for_tracing()
  {
#ifndef TRACETOOLS_DISABLED
    std::visit(
      [this](auto && arg) {
        if (TRACETOOLS_TRACEPOINT_ENABLED(rclcpp_callback_register)) {
          char * symbol = tracetools::get_symbol(arg);
          TRACETOOLS_DO_TRACEPOINT(
            rclcpp_callback_register,
            static_cast<const void *>(this),
            symbol);
          std::free(symbol);
        }
      }, callback_);
#endif  // TRACETOOLS_DISABLED
  }
};

}  // namespace rclcpp

#endif  // RCLCPP__ANY_SERVICE_CALLBACK_HPP_
