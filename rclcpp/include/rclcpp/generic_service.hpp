// Copyright 2024 Sony Group Corporation.
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

#ifndef RCLCPP__GENERIC_SERVICE_HPP_
#define RCLCPP__GENERIC_SERVICE_HPP_

#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>

#include "rclcpp/typesupport_helpers.hpp"

#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"

#include "service.hpp"

namespace rclcpp
{
class GenericService;

class GenericServiceCallback
{
public:
  using SharedRequest = std::shared_ptr<void>;
  using SharedResponse = std::shared_ptr<void>;

  GenericServiceCallback()
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
        SharedPtrCallback
      >::value)
    {
      callback_.template emplace<SharedPtrCallback>(callback);
    } else if constexpr (  // NOLINT, can't satisfy both cpplint and uncrustify
      rclcpp::function_traits::same_arguments<
        CallbackT,
        SharedPtrWithRequestHeaderCallback
      >::value)
    {
      callback_.template emplace<SharedPtrWithRequestHeaderCallback>(callback);
    } else if constexpr (  // NOLINT
      rclcpp::function_traits::same_arguments<
        CallbackT,
        SharedPtrDeferResponseCallback
      >::value)
    {
      callback_.template emplace<SharedPtrDeferResponseCallback>(callback);
    } else if constexpr (  // NOLINT
      rclcpp::function_traits::same_arguments<
        CallbackT,
        SharedPtrDeferResponseCallbackWithServiceHandle
      >::value)
    {
      callback_.template emplace<SharedPtrDeferResponseCallbackWithServiceHandle>(callback);
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
        SharedPtrCallback
      >::value)
    {
      callback_.template emplace<SharedPtrCallback>(callback);
    } else if constexpr (  // NOLINT
      rclcpp::function_traits::same_arguments<
        CallbackT,
        SharedPtrWithRequestHeaderCallback
      >::value)
    {
      callback_.template emplace<SharedPtrWithRequestHeaderCallback>(callback);
    } else if constexpr (  // NOLINT
      rclcpp::function_traits::same_arguments<
        CallbackT,
        SharedPtrDeferResponseCallback
      >::value)
    {
      callback_.template emplace<SharedPtrDeferResponseCallback>(callback);
    } else if constexpr (  // NOLINT
      rclcpp::function_traits::same_arguments<
        CallbackT,
        SharedPtrDeferResponseCallbackWithServiceHandle
      >::value)
    {
      callback_.template emplace<SharedPtrDeferResponseCallbackWithServiceHandle>(callback);
    } else {
      // the else clause is not needed, but anyways we should only be doing this instead
      // of all the above workaround ...
      callback_ = std::forward<CallbackT>(callback);
    }
  }

  SharedResponse
  dispatch(
    const std::shared_ptr<rclcpp::GenericService> & service_handle,
    const std::shared_ptr<rmw_request_id_t> & request_header,
    SharedRequest request,
    SharedRequest response)
  {
    TRACETOOLS_TRACEPOINT(callback_start, static_cast<const void *>(this), false);
    if (std::holds_alternative<std::monostate>(callback_)) {
      // TODO(ivanpauno): Remove the set method, and force the users of this class
      // to pass a callback at construnciton.
      throw std::runtime_error{"unexpected request without any callback set"};
    }
    if (std::holds_alternative<SharedPtrDeferResponseCallback>(callback_)) {
      const auto & cb = std::get<SharedPtrDeferResponseCallback>(callback_);
      cb(request_header, std::move(request));
      return nullptr;
    }
    if (std::holds_alternative<SharedPtrDeferResponseCallbackWithServiceHandle>(callback_)) {
      const auto & cb = std::get<SharedPtrDeferResponseCallbackWithServiceHandle>(callback_);
      cb(service_handle, request_header, std::move(request));
      return nullptr;
    }

    if (std::holds_alternative<SharedPtrCallback>(callback_)) {
      (void)request_header;
      const auto & cb = std::get<SharedPtrCallback>(callback_);
      cb(std::move(request), std::move(response));
    } else if (std::holds_alternative<SharedPtrWithRequestHeaderCallback>(callback_)) {
      const auto & cb = std::get<SharedPtrWithRequestHeaderCallback>(callback_);
      cb(request_header, std::move(request), std::move(response));
    }
    TRACETOOLS_TRACEPOINT(callback_end, static_cast<const void *>(this));
    return response;
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

private:
  using SharedPtrCallback = std::function<void (SharedRequest, SharedResponse)>;
  using SharedPtrWithRequestHeaderCallback = std::function<
    void (
      std::shared_ptr<rmw_request_id_t>,
      SharedRequest,
      SharedResponse
    )>;
  using SharedPtrDeferResponseCallback = std::function<
    void (
      std::shared_ptr<rmw_request_id_t>,
      SharedRequest
    )>;
  using SharedPtrDeferResponseCallbackWithServiceHandle = std::function<
    void (
      std::shared_ptr<rclcpp::GenericService>,
      std::shared_ptr<rmw_request_id_t>,
      SharedRequest
    )>;

  std::variant<
    std::monostate,
    SharedPtrCallback,
    SharedPtrWithRequestHeaderCallback,
    SharedPtrDeferResponseCallback,
    SharedPtrDeferResponseCallbackWithServiceHandle> callback_;
};

class GenericService
  : public ServiceBase,
  public std::enable_shared_from_this<GenericService>
{
public:
  using Request = void *;  // Serialized/Deserialized data pointer of request message
  using Response = void *;  // Serialized/Deserialized data pointer of response message
  using SharedRequest = std::shared_ptr<void>;
  using SharedResponse = std::shared_ptr<void>;
  using CallbackType = std::function<void (const SharedRequest, SharedResponse)>;

  using CallbackWithHeaderType =
    std::function<void (const std::shared_ptr<rmw_request_id_t>,
      const SharedRequest,
      SharedResponse)>;

  RCLCPP_SMART_PTR_DEFINITIONS(GenericService)

  /// Default constructor.
  /**
   * The constructor for a Service is almost never called directly.
   * Instead, services should be instantiated through the function
   * rclcpp::create_service().
   *
   * \param[in] node_handle NodeBaseInterface pointer that is used in part of the setup.
   * \param[in] service_name Name of the topic to publish to.
   * \param[in] service_type The name of service type, e.g. "std_srvs/srv/SetBool".
   * \param[in] any_callback User defined callback to call when a client request is received.
   * \param[in] service_options options for the service.
   */
  RCLCPP_PUBLIC
  GenericService(
    std::shared_ptr<rcl_node_t> node_handle,
    const std::string & service_name,
    const std::string & service_type,
    GenericServiceCallback any_callback,
    rcl_service_options_t & service_options);

  GenericService() = delete;

  RCLCPP_PUBLIC
  virtual ~GenericService() {}

  /// Take the next request from the service.
  /**
   * \sa ServiceBase::take_type_erased_request().
   *
   * \param[out] request_out The reference to a service deserialized request object
   *   into which the middleware will copy the taken request.
   * \param[out] request_id_out The output id for the request which can be used
   *   to associate response with this request in the future.
   * \returns true if the request was taken, otherwise false.
   * \throws rclcpp::exceptions::RCLError based exceptions if the underlying
   *   rcl calls fail.
   */
  RCLCPP_PUBLIC
  bool
  take_request(SharedRequest request_out, rmw_request_id_t & request_id_out);

  RCLCPP_PUBLIC
  std::shared_ptr<void>
  create_request() override;

  RCLCPP_PUBLIC
  std::shared_ptr<void>
  create_response();

  RCLCPP_PUBLIC
  std::shared_ptr<rmw_request_id_t>
  create_request_header() override;

  RCLCPP_PUBLIC
  void
  handle_request(
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<void> request) override;

  RCLCPP_PUBLIC
  void
  send_response(rmw_request_id_t & req_id, SharedResponse & response);

private:
  RCLCPP_DISABLE_COPY(GenericService)

  GenericServiceCallback any_callback_;

  std::shared_ptr<rcpputils::SharedLibrary> ts_lib_;
  const rosidl_typesupport_introspection_cpp::MessageMembers * request_members_;
  const rosidl_typesupport_introspection_cpp::MessageMembers * response_members_;
};

}  // namespace rclcpp
#endif  // RCLCPP__GENERIC_SERVICE_HPP_
