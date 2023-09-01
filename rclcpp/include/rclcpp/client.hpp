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

#ifndef RCLCPP__CLIENT_HPP_
#define RCLCPP__CLIENT_HPP_

#include <sys/_types/_int64_t.h>

#include <atomic>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "is_ros_compatible_type.hpp"
#include "rcl/client.h"
#include "rcl/error_handling.h"
#include "rcl/event_callback.h"
#include "rcl/service_introspection.h"
#include "rcl/wait.h"

#include "rclcpp/clock.hpp"
#include "rclcpp/detail/cpp_callback_trampoline.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rclcpp/get_service_type_support_handle.hpp"
#include "rclcpp/function_traits.hpp"
#include "rclcpp/is_ros_compatible_type.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/type_adapter.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/visibility_control.hpp"

#include "rmw/error_handling.h"
#include "rmw/impl/cpp/demangle.hpp"
#include "rmw/rmw.h"

#include <rosidl_runtime_cpp/traits.hpp>

namespace rclcpp
{

namespace detail
{
template<typename FutureT>
struct FutureAndRequestId
{
  FutureT future;
  int64_t request_id;

  FutureAndRequestId(FutureT impl, int64_t req_id)
  : future(std::move(impl)), request_id(req_id)
  {}

  /// Allow implicit conversions to `std::future` by reference.
  operator FutureT &() {return this->future;}

  /// Deprecated, use the `future` member variable instead.
  /**
   * Allow implicit conversions to `std::future` by value.
   * \deprecated
   */
  [[deprecated("FutureAndRequestId: use .future instead of an implicit conversion")]]
  operator FutureT() {return this->future;}

  // delegate future like methods in the std::future impl_

  /// See std::future::get().
  auto get() {return this->future.get();}
  /// See std::future::valid().
  bool valid() const noexcept {return this->future.valid();}
  /// See std::future::wait().
  void wait() const {return this->future.wait();}
  /// See std::future::wait_for().
  template<class Rep, class Period>
  std::future_status wait_for(
    const std::chrono::duration<Rep, Period> & timeout_duration) const
  {
    return this->future.wait_for(timeout_duration);
  }
  /// See std::future::wait_until().
  template<class Clock, class Duration>
  std::future_status wait_until(
    const std::chrono::time_point<Clock, Duration> & timeout_time) const
  {
    return this->future.wait_until(timeout_time);
  }

  // Rule of five, we could use the rule of zero here, but better be explicit as some of the
  // methods are deleted.

  /// Move constructor.
  FutureAndRequestId(FutureAndRequestId && other) noexcept = default;
  /// Deleted copy constructor, each instance is a unique owner of the future.
  FutureAndRequestId(const FutureAndRequestId & other) = delete;
  /// Move assignment.
  FutureAndRequestId & operator=(FutureAndRequestId && other) noexcept = default;
  /// Deleted copy assignment, each instance is a unique owner of the future.
  FutureAndRequestId & operator=(const FutureAndRequestId & other) = delete;
  /// Destructor.
  ~FutureAndRequestId() = default;
};
}  // namespace detail

namespace node_interfaces
{
class NodeBaseInterface;
}  // namespace node_interfaces

class ClientBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(ClientBase)

  RCLCPP_PUBLIC
  ClientBase(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph);

  RCLCPP_PUBLIC
  virtual ~ClientBase() = default;

  /// Take the next response for this client as a type erased pointer.
  /**
   * The type erased pointer allows for this method to be used in a type
   * agnostic way along with ClientBase::create_response(),
   * ClientBase::create_request_header(), and ClientBase::handle_response().
   * The typed version of this can be used if the Service type is known,
   * \sa Client::take_response().
   *
   * \param[out] response_out The type erased pointer to a Service Response into
   *   which the middleware will copy the response being taken.
   * \param[out] request_header_out The request header to be filled by the
   *   middleware when taking, and which can be used to associte the response
   *   to a specific request.
   * \returns true if the response was taken, otherwise false.
   * \throws rclcpp::exceptions::RCLError based exceptions if the underlying
   *   rcl function fail.
   */
  RCLCPP_PUBLIC
  bool
  take_type_erased_response(void * response_out, rmw_request_id_t & request_header_out);

  /// Return the name of the service.
  /** \return The name of the service. */
  RCLCPP_PUBLIC
  const char *
  get_service_name() const;

  /// Return the rcl_client_t client handle in a std::shared_ptr.
  /**
   * This handle remains valid after the Client is destroyed.
   * The actual rcl client is not finalized until it is out of scope everywhere.
   */
  RCLCPP_PUBLIC
  std::shared_ptr<rcl_client_t>
  get_client_handle();

  /// Return the rcl_client_t client handle in a std::shared_ptr.
  /**
   * This handle remains valid after the Client is destroyed.
   * The actual rcl client is not finalized until it is out of scope everywhere.
   */
  RCLCPP_PUBLIC
  std::shared_ptr<const rcl_client_t>
  get_client_handle() const;

  /// Return if the service is ready.
  /**
   * \return `true` if the service is ready, `false` otherwise
   */
  RCLCPP_PUBLIC
  bool
  service_is_ready() const;

  /// Wait for a service to be ready.
  /**
   * \param timeout maximum time to wait
   * \return `true` if the service is ready and the timeout is not over, `false` otherwise
   */
  template<typename RepT = int64_t, typename RatioT = std::milli>
  bool
  wait_for_service(
    std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1))
  {
    return wait_for_service_nanoseconds(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timeout)
    );
  }

  virtual std::shared_ptr<void> create_response() = 0;
  virtual std::shared_ptr<rmw_request_id_t> create_request_header() = 0;
  virtual void handle_response(
    std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> response) = 0;

  /// Exchange the "in use by wait set" state for this client.
  /**
   * This is used to ensure this client is not used by multiple
   * wait sets at the same time.
   *
   * \param[in] in_use_state the new state to exchange into the state, true
   *   indicates it is now in use by a wait set, and false is that it is no
   *   longer in use by a wait set.
   * \returns the previous state.
   */
  RCLCPP_PUBLIC
  bool
  exchange_in_use_by_wait_set_state(bool in_use_state);

  /// Get the actual request publsher QoS settings, after the defaults have been determined.
  /**
   * The actual configuration applied when using RMW_QOS_POLICY_*_SYSTEM_DEFAULT
   * can only be resolved after the creation of the client, and it
   * depends on the underlying rmw implementation.
   * If the underlying setting in use can't be represented in ROS terms,
   * it will be set to RMW_QOS_POLICY_*_UNKNOWN.
   * May throw runtime_error when an unexpected error occurs.
   *
   * \return The actual request publsher qos settings.
   * \throws std::runtime_error if failed to get qos settings
   */
  RCLCPP_PUBLIC
  rclcpp::QoS
  get_request_publisher_actual_qos() const;

  /// Get the actual response subscription QoS settings, after the defaults have been determined.
  /**
   * The actual configuration applied when using RMW_QOS_POLICY_*_SYSTEM_DEFAULT
   * can only be resolved after the creation of the client, and it
   * depends on the underlying rmw implementation.
   * If the underlying setting in use can't be represented in ROS terms,
   * it will be set to RMW_QOS_POLICY_*_UNKNOWN.
   * May throw runtime_error when an unexpected error occurs.
   *
   * \return The actual response subscription qos settings.
   * \throws std::runtime_error if failed to get qos settings
   */
  RCLCPP_PUBLIC
  rclcpp::QoS
  get_response_subscription_actual_qos() const;

  /// Set a callback to be called when each new response is received.
  /**
   * The callback receives a size_t which is the number of responses received
   * since the last time this callback was called.
   * Normally this is 1, but can be > 1 if responses were received before any
   * callback was set.
   *
   * Since this callback is called from the middleware, you should aim to make
   * it fast and not blocking.
   * If you need to do a lot of work or wait for some other event, you should
   * spin it off to another thread, otherwise you risk blocking the middleware.
   *
   * Calling it again will clear any previously set callback.
   *
   * An exception will be thrown if the callback is not callable.
   *
   * This function is thread-safe.
   *
   * If you want more information available in the callback, like the client
   * or other information, you may use a lambda with captures or std::bind.
   *
   * \sa rmw_client_set_on_new_response_callback
   * \sa rcl_client_set_on_new_response_callback
   *
   * \param[in] callback functor to be called when a new response is received
   */
  void
  set_on_new_response_callback(std::function<void(size_t)> callback)
  {
    if (!callback) {
      throw std::invalid_argument(
              "The callback passed to set_on_new_response_callback "
              "is not callable.");
    }

    auto new_callback =
      [callback, this](size_t number_of_responses) {
        try {
          callback(number_of_responses);
        } catch (const std::exception & exception) {
          RCLCPP_ERROR_STREAM(
            node_logger_,
            "rclcpp::ClientBase@" << this <<
              " caught " << rmw::impl::cpp::demangle(exception) <<
              " exception in user-provided callback for the 'on new response' callback: " <<
              exception.what());
        } catch (...) {
          RCLCPP_ERROR_STREAM(
            node_logger_,
            "rclcpp::ClientBase@" << this <<
              " caught unhandled exception in user-provided callback " <<
              "for the 'on new response' callback");
        }
      };

    std::lock_guard<std::recursive_mutex> lock(callback_mutex_);

    // Set it temporarily to the new callback, while we replace the old one.
    // This two-step setting, prevents a gap where the old std::function has
    // been replaced but the middleware hasn't been told about the new one yet.
    set_on_new_response_callback(
      rclcpp::detail::cpp_callback_trampoline<decltype(new_callback), const void *, size_t>,
      static_cast<const void *>(&new_callback));

    // Store the std::function to keep it in scope, also overwrites the existing one.
    on_new_response_callback_ = new_callback;

    // Set it again, now using the permanent storage.
    set_on_new_response_callback(
      rclcpp::detail::cpp_callback_trampoline<
        decltype(on_new_response_callback_), const void *, size_t>,
      static_cast<const void *>(&on_new_response_callback_));
  }

  /// Unset the callback registered for new responses, if any.
  void
  clear_on_new_response_callback()
  {
    std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
    if (on_new_response_callback_) {
      set_on_new_response_callback(nullptr, nullptr);
      on_new_response_callback_ = nullptr;
    }
  }

protected:
  RCLCPP_DISABLE_COPY(ClientBase)

  RCLCPP_PUBLIC
  bool
  wait_for_service_nanoseconds(std::chrono::nanoseconds timeout);

  RCLCPP_PUBLIC
  rcl_node_t *
  get_rcl_node_handle();

  RCLCPP_PUBLIC
  const rcl_node_t *
  get_rcl_node_handle() const;

  RCLCPP_PUBLIC
  void
  set_on_new_response_callback(rcl_event_callback_t callback, const void * user_data);

  rclcpp::node_interfaces::NodeGraphInterface::WeakPtr node_graph_;
  std::shared_ptr<rcl_node_t> node_handle_;
  std::shared_ptr<rclcpp::Context> context_;
  rclcpp::Logger node_logger_;

  std::recursive_mutex callback_mutex_;
  // It is important to declare on_new_response_callback_ before
  // client_handle_, so on destruction the client is
  // destroyed first. Otherwise, the rmw client callback
  // would point briefly to a destroyed function.
  std::function<void(size_t)> on_new_response_callback_{nullptr};
  // Declare client_handle_ after callback
  std::shared_ptr<rcl_client_t> client_handle_;

  std::atomic<bool> in_use_by_wait_set_{false};
};

template<typename ServiceT>
class Client : public ClientBase
{
public:
  static_assert(
    rclcpp::is_ros_compatible_service_type<ServiceT>::value,
    "Service type is not compatible with ROS 2 and cannot be used with a Client");

  /// ServiceT::custom_type::Request if ServiceT is a TypeAdapter, otherwise just the
  /// ServiceT::Request
  using ServiceRequestType = typename rclcpp::TypeAdapter<ServiceT>::custom_type::Request;
  /// ServiceT::ros_message_type::Request if ServiceT is a TypeAdapter, otherwise just the
  /// ServiceT::Request
  using ROSServiceRequestType =
    typename rclcpp::TypeAdapter<ServiceT>::ros_message_type::Request;
  /// ServiceT::custom_type::Response if ServiceT is a TypeAdapter, otherwise just the
  /// ServiceT::Response
  using ServiceResponseType = typename rclcpp::TypeAdapter<ServiceT>::custom_type::Response;
  /// ServiceT::ros_message_type::Response if ServiceT is a TypeAdapter, otherwise just the
  /// ServiceT::Response
  using ROSServiceResponseType =
    typename rclcpp::TypeAdapter<ServiceT>::ros_message_type::Response;

  using CustomSharedRequest = typename std::shared_ptr<ServiceRequestType>;
  using CustomSharedResponse = typename std::shared_ptr<ServiceResponseType>;

  using ROSSharedRequest = typename ROSServiceRequestType::SharedPtr;
  using ROSSharedResponse = typename ROSServiceResponseType::SharedPtr;

  using SharedRequest [[deprecated("Use ROSSharedRequest instead of SharedRequest")]] =
    ROSSharedRequest;

  using SharedResponse [[deprecated("Use ROSSharedResponse instead of SharedResponse")]] =
    ROSSharedResponse;

  using CustomPromise = std::promise<CustomSharedResponse>;
  using ROSPromise = std::promise<ROSSharedResponse>;

  using Promise [[deprecated("Use ROSPromise instead of Promise")]] = ROSPromise;

  using CustomTotalPromiseWithRequest =
    std::promise<std::pair<CustomSharedRequest, CustomSharedResponse>>;
  using ROSTotalPromiseWithRequest =
    std::promise<std::pair<ROSSharedRequest, ROSSharedResponse>>;
  using CustomROSPromiseWithRequest =
    std::promise<std::pair<CustomSharedRequest, ROSSharedResponse>>;
  using ROSCustomPromiseWithRequest =
    std::promise<std::pair<ROSSharedRequest, CustomSharedResponse>>;

  using PromiseWithRequest
  [[deprecated("Use ROSTotalPromiseWithRequest instead of PromiseWithRequest")]] =
    ROSTotalPromiseWithRequest;

  using CustomSharedPromise = std::shared_ptr<CustomPromise>;
  using ROSSharedPromise = std::shared_ptr<ROSPromise>;

  using SharedPromise [[deprecated("Use ROSSharedPromise instead of SharedPromise")]] =
    ROSSharedPromise;

  using CustomTotalSharedPromiseWithRequest = std::shared_ptr<CustomTotalPromiseWithRequest>;
  using ROSTotalSharedPromiseWithRequest = std::shared_ptr<ROSTotalPromiseWithRequest>;
  using CustomROSSharedPromiseWithRequest = std::shared_ptr<CustomROSPromiseWithRequest>;
  using ROSCustomSharedPromiseWithRequest = std::shared_ptr<ROSCustomPromiseWithRequest>;

  using SharedPromiseWithRequest
  [[deprecated("Use ROSTotalSharedPromiseWithRequest instead fo SharedPromiseWithRequest")]] =
    ROSTotalSharedPromiseWithRequest;

  using CustomFuture = std::future<CustomSharedResponse>;
  using ROSFuture = std::future<ROSSharedResponse>;

  using Future [[deprecated("Use ROSFuture instead of Future")]] =
    ROSFuture;

  using CustomSharedFuture = std::shared_future<CustomSharedResponse>;
  using ROSSharedFuture = std::shared_future<ROSSharedResponse>;

  using SharedFuture [[deprecated("Use ROSSharedFuture instead of SharedFuture")]] =
    ROSSharedFuture;

  using CustomTotalSharedFutureWithRequest =
    std::shared_future<std::pair<CustomSharedRequest, CustomSharedResponse>>;
  using ROSTotalSharedFutureWithRequest =
    std::shared_future<std::pair<ROSSharedRequest, ROSSharedResponse>>;
  using CustomROSSharedFutureWithRequest =
    std::shared_future<std::pair<CustomSharedRequest, ROSSharedResponse>>;
  using ROSCustomSharedFutureWithRequest =
    std::shared_future<std::pair<ROSSharedRequest, CustomSharedResponse>>;

  using SharedFutureWithRequest
  [[deprecated("Use ROSTotalSharedFutureWithRequest instead of SharedFutureWithRequest")]] =
    ROSTotalSharedFutureWithRequest;

  using CustomCallbackType = std::function<void (CustomSharedFuture)>;
  using ROSCallbackType = std::function<void (ROSSharedFuture)>;

  using CallbackType [[deprecated("Use ROSCallbackType instead of CallbackType")]] =
    ROSCallbackType;

  using CustomTotalCallbackWithRequestType =
    std::function<void (CustomTotalSharedFutureWithRequest)>;
  using ROSTotalCallbackWithRequestType =
    std::function<void (ROSTotalSharedFutureWithRequest)>;
  using CustomROSCallbackWithRequestType =
    std::function<void (CustomROSSharedFutureWithRequest)>;
  using ROSCustomCallbackWithRequestType =
    std::function<void (ROSCustomSharedFutureWithRequest)>;

  using CallbackWithRequestType
  [[deprecated("Use ROSTotalCallbackWithRequestType instead of CallbackWithRequestType")]] =
    ROSTotalCallbackWithRequestType;

  RCLCPP_SMART_PTR_DEFINITIONS(Client)

  /// A convenient Client::Future and request id pair.
  /**
   * Public members:
   * - future: a std::future<SharedResponse>.
   *   where SharedResponse is either ROSSharedResponse or CustomSharedResponse
   * - request_id: the request id associated with the future.
   *
   * All the other methods are equivalent to the ones std::future provides.
   */
  struct [[deprecated("Use ROSFutureAndRequestId instead of FutureAndRequestId")]]
  FutureAndRequestId : detail::FutureAndRequestId<std::future<ROSSharedResponse>>
  {
    using detail::FutureAndRequestId<std::future<ROSSharedResponse>>::FutureAndRequestId;
    // delegate future like methods in the std::future impl_

    /// See std::future::share().
    ROSSharedFuture share() noexcept {return this->future.share();}
  };

  struct ROSFutureAndRequestId
    : detail::FutureAndRequestId<std::future<ROSSharedResponse>>
  {
    using detail::FutureAndRequestId<std::future<ROSSharedResponse>>::FutureAndRequestId;
    // delegate future like methods in the std::future impl_

    /// See std::future::share().
    ROSSharedFuture share() noexcept {return this->future.share();}
  };

  struct CustomFutureAndRequestId
    : detail::FutureAndRequestId<std::future<CustomSharedResponse>>
  {
    using detail::FutureAndRequestId<std::future<CustomSharedResponse>>::FutureAndRequestId;
    // delegate future like methods in the std::future impl_

    /// See std::future::share().
    CustomSharedFuture share() noexcept {return this->future.share();}
  };

  /// A convenient Client::SharedFuture and request id pair.
  /**
   * Public members:
   * - future: a std::shared_future<SharedResponse>.
   *   where SharedResponse is either ROSSharedResponse or CustomSharedResponse
   * - request_id: the request id associated with the future.
   *
   * All the other methods are equivalent to the ones std::shared_future provides.
   */
  struct [[deprecated("Use ROSSharedFutureAndRequestId instead of SharedFutureAndRequestId")]]
  SharedFutureAndRequestId
    : detail::FutureAndRequestId<std::shared_future<ROSSharedResponse>>
  {
    using detail::FutureAndRequestId<std::shared_future<ROSSharedResponse>>::FutureAndRequestId;
  };

  struct ROSSharedFutureAndRequestId
    : detail::FutureAndRequestId<std::shared_future<ROSSharedResponse>>
  {
    using detail::FutureAndRequestId<std::shared_future<ROSSharedResponse>>::FutureAndRequestId;
  };

  struct CustomSharedFutureAndRequestId
    : detail::FutureAndRequestId<std::shared_future<CustomSharedResponse>>
  {
    using detail::FutureAndRequestId<std::shared_future<CustomSharedResponse>>::FutureAndRequestId;
  };

  /// A convenient Client::SharedFutureWithRequest and request id pair.
  /**
   * Public members:
   * - future: a std::shared_future<std::pair<SharedRequest, SharedResponse>>.
   *   where SharedRequest, SharedResponse can be a mixture of ROS/Custom Typed
   * - request_id: the request id associated with the future.
   *
   * All the other methods are equivalent to the ones std::shared_future provides.
   */
  struct
  [[deprecated(
    "Use ROSTotalSharedFutureWithRequest instead of SharedFutureWithRequestAndRequestId")]]
  SharedFutureWithRequestAndRequestId
    : detail::FutureAndRequestId<std::shared_future<std::pair<SharedRequest, SharedResponse>>>
  {
    using detail::FutureAndRequestId<
      std::shared_future<std::pair<SharedRequest, SharedResponse>>
    >::FutureAndRequestId;
  };

  struct ROSTotalSharedFutureWithRequestAndRequestId
    : detail::FutureAndRequestId<std::shared_future<std::pair<ROSSharedRequest, ROSSharedResponse>>>
  {
    using detail::FutureAndRequestId<
      std::shared_future<std::pair<ROSSharedRequest, ROSSharedResponse>>
    >::FutureAndRequestId;
  };

  struct CustomTotalSharedFutureWithRequestAndRequestId
    : detail::FutureAndRequestId<std::shared_future<std::pair<CustomSharedRequest,
      CustomSharedResponse>>>
  {
    using detail::FutureAndRequestId<
      std::shared_future<std::pair<CustomSharedRequest, CustomSharedResponse>>
    >::FutureAndRequestId;
  };

  struct ROSCustomSharedFutureWithRequestAndRequestId
    : detail::FutureAndRequestId<std::shared_future<std::pair<ROSSharedRequest,
      CustomSharedResponse>>>
  {
    using detail::FutureAndRequestId<
      std::shared_future<std::pair<ROSSharedRequest, CustomSharedResponse>>
    >::FutureAndRequestId;
  };

  struct CustomROSSharedFutureWithRequestAndRequestId
    : detail::FutureAndRequestId<std::shared_future<std::pair<CustomSharedRequest,
      ROSSharedResponse>>>
  {
    using detail::FutureAndRequestId<
      std::shared_future<std::pair<CustomSharedRequest, ROSSharedResponse>>
    >::FutureAndRequestId;
  };

  /// Default constructor.
  /**
   * The constructor for a Client is almost never called directly.
   * Instead, clients should be instantiated through the function
   * rclcpp::create_client().
   *
   * \param[in] node_base NodeBaseInterface pointer that is used in part of the setup.
   * \param[in] node_graph The node graph interface of the corresponding node.
   * \param[in] service_name Name of the topic to publish to.
   * \param[in] client_options options for the subscription.
   */
  Client(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
    const std::string & service_name,
    rcl_client_options_t & client_options)
  : ClientBase(node_base, node_graph),
    srv_type_support_handle_(&rclcpp::get_service_type_support_handle<ServiceT>())
  {
    rcl_ret_t ret = rcl_client_init(
      this->get_client_handle().get(),
      this->get_rcl_node_handle(),
      srv_type_support_handle_,
      service_name.c_str(),
      &client_options);
    if (ret != RCL_RET_OK) {
      if (ret == RCL_RET_SERVICE_NAME_INVALID) {
        auto rcl_node_handle = this->get_rcl_node_handle();
        // this will throw on any validation problem
        rcl_reset_error();
        expand_topic_or_service_name(
          service_name,
          rcl_node_get_name(rcl_node_handle),
          rcl_node_get_namespace(rcl_node_handle),
          true);
      }
      rclcpp::exceptions::throw_from_rcl_error(ret, "could not create client");
    }
  }

  virtual ~Client()
  {
  }

  /// Take the next response for this client.
  /**
   * \sa ClientBase::take_type_erased_response().
   *
   * This signature is enabled if the service response is a ROSServiceResponseType
   * as opposed to the custom type of a respective TypeAdapter
   *
   * \param[out] response_out The reference to a Service Response into
   *   which the middleware will copy the response being taken.
   * \param[out] request_header_out The request header to be filled by the
   *   middleware when taking, and which can be used to associte the response
   *   to a specific request.
   * \returns true if the response was taken, otherwise false.
   * \throws rclcpp::exceptions::RCLError based exceptions if the underlying
   *   rcl function fail.
   */

  template<typename T>
  typename std::enable_if_t<
    rosidl_generator_traits::is_message<T>::value &&
    std::is_same<T, ROSServiceResponseType>::value,
    bool
  >
  take_response(T & response_out, rmw_request_id_t & request_header_out)
  {
    return this->take_type_erased_response(&response_out, request_header_out);
  }
  /// Take the next response for this client.
  /**
   * \sa ClientBase::take_type_erased_response().
   *
   * This signature is enabled if the service response is a ServiceResponseType
   * created with a TypeAdapter, matching its respective custom_type
   *
   * \param[out] response_out The reference to a Service Response into
   *   which the middleware will copy the response being taken.
   * \param[out] request_header_out The request header to be filled by the
   *   middleware when taking, and which can be used to associte the response
   *   to a specific request.
   * \returns true if the response was taken, otherwise false.
   * \throws rclcpp::exceptions::RCLError based exceptions if the underlying
   *   rcl function fail.
   */
  template<typename T>
  typename std::enable_if_t<
    rclcpp::TypeAdapter<ServiceT>::is_specialized::value &&
    std::is_same<T, ServiceResponseType>::value,
    bool
  >
  take_response(T & response_out, rmw_request_id_t & request_header_out)
  {
    ROSServiceResponseType ros_service_response_out;
    rclcpp::TypeAdapter<ServiceT>::convert_to_ros_service_response(
      response_out, ros_service_response_out);
    return this->take_type_erased_response(&ros_service_response_out, request_header_out);
  }

  /// Create a shared pointer with the response type
  /**
   * \return shared pointer with the response type
   */
  std::shared_ptr<void>
  create_response() override
  {
    return std::shared_ptr<void>(new ROSServiceResponseType());
  }

  /// Create a shared pointer with a rmw_request_id_t
  /**
   * \return shared pointer with a rmw_request_id_t
   */
  std::shared_ptr<rmw_request_id_t>
  create_request_header() override
  {
    // TODO(wjwwood): This should probably use rmw_request_id's allocator.
    //                (since it is a C type)
    return std::shared_ptr<rmw_request_id_t>(new rmw_request_id_t);
  }

  /// Handle a server response
  /**
    * \param[in] request_header used to check if the secuence number is valid
    * \param[in] response message with the server response
   */
  void
  handle_response(
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<void> response) override
  {
    std::optional<CallbackInfoVariant>
    optional_pending_request = this->get_and_erase_pending_request(request_header->sequence_number);
    if (!optional_pending_request) {
      return;
    }
    auto & value = *optional_pending_request;
    if (std::holds_alternative<ROSPromise>(value)) {
      auto typed_response = std::static_pointer_cast<ROSServiceResponseType>(
        std::move(response));
      auto & promise = std::get<ROSPromise>(value);
      promise.set_value(std::move(typed_response));
    } else if (std::holds_alternative<CustomPromise>(value)) {
      auto typed_response = std::static_pointer_cast<ServiceResponseType>(
        std::move(response));
      auto & promise = std::get<CustomPromise>(value);
      promise.set_value(std::move(typed_response));
    } else if (std::holds_alternative<CallbackTypeValueVariant>(value)) {
      auto & inner = std::get<CallbackTypeValueVariant>(value);
      if (inner.index() == 0) {
        auto typed_response = std::static_pointer_cast<ROSServiceResponseType>(
          std::move(response));
        auto & inner_tuple =
          std::get<std::tuple<ROSCallbackType, ROSSharedFuture, ROSPromise>>(inner);
        const auto & callback = std::get<ROSCallbackType>(inner_tuple);
        auto & promise = std::get<ROSPromise>(inner_tuple);
        auto & future = std::get<ROSSharedFuture>(inner_tuple);
        promise.set_value(std::move(typed_response));
        callback(std::move(future));
      } else if (inner.index() == 1) {
        auto typed_response = std::static_pointer_cast<ServiceResponseType>(
          std::move(response));
        auto & inner_tuple =
          std::get<std::tuple<CustomCallbackType, CustomSharedFuture, CustomPromise>>(inner);
        const auto & callback = std::get<CustomCallbackType>(inner_tuple);
        auto & promise = std::get<CustomPromise>(inner_tuple);
        auto & future = std::get<CustomSharedFuture>(inner_tuple);
        promise.set_value(std::move(typed_response));
        callback(std::move(future));
      }
    } else if (std::holds_alternative<CallbackWithRequestTypeValueVariant>(value)) {
      auto & inner = std::get<CallbackWithRequestTypeValueVariant>(value);
      if (inner.index() == 0) {
        auto typed_response = std::static_pointer_cast<ROSServiceResponseType>(
          std::move(response));
        auto & inner_tuple =
          std::get<std::tuple<ROSTotalCallbackWithRequestType, ROSSharedRequest,
            ROSTotalSharedFutureWithRequest, ROSTotalPromiseWithRequest>>(inner);
        const auto & callback = std::get<ROSTotalCallbackWithRequestType>(inner_tuple);
        auto & promise = std::get<ROSTotalPromiseWithRequest>(inner_tuple);
        auto & future = std::get<ROSTotalSharedFutureWithRequest>(inner_tuple);
        auto & request = std::get<ROSSharedRequest>(inner_tuple);
        promise.set_value(std::make_pair(std::move(request), std::move(typed_response)));
        callback(std::move(future));
      } else if (inner.index() == 1) {
        auto typed_response = std::static_pointer_cast<ServiceResponseType>(
          std::move(response));
        auto & inner_tuple =
          std::get<std::tuple<CustomTotalCallbackWithRequestType, CustomSharedRequest,
            CustomTotalSharedFutureWithRequest, CustomTotalPromiseWithRequest>>(inner);
        const auto & callback = std::get<CustomTotalCallbackWithRequestType>(inner_tuple);
        auto & promise = std::get<CustomTotalPromiseWithRequest>(inner_tuple);
        auto & future = std::get<CustomTotalSharedFutureWithRequest>(inner_tuple);
        auto & request = std::get<CustomSharedRequest>(inner_tuple);
        promise.set_value(std::make_pair(std::move(request), std::move(typed_response)));
        callback(std::move(future));
      }
    }
  }

  /// Send a request to the service server.
  /**
   * This method returns a `FutureAndRequestId` instance
   * that can be passed to Executor::spin_until_future_complete() to
   * wait until it has been completed.
   *
   * If the future never completes,
   * e.g. the call to Executor::spin_until_future_complete() times out,
   * Client::remove_pending_request() must be called to clean the client internal state.
   * Not doing so will make the `Client` instance to use more memory each time a response is not
   * received from the service server.
   *
   * ```cpp
   * auto future = client->async_send_request(my_request);
   * if (
   *   rclcpp::FutureReturnCode::TIMEOUT ==
   *   executor->spin_until_future_complete(future, timeout))
   * {
   *   client->remove_pending_request(future);
   *   // handle timeout
   * } else {
   *   handle_response(future.get());
   * }
   * ```
   *
   * \param[in] request request to be send.
   * \return a FutureAndRequestId instance.
   */
  template<typename T>
  typename std::enable_if_t<
    std::is_same<T, ROSSharedRequest>::value,
    ROSFutureAndRequestId
  >
  async_send_request(T request)
  {
    ROSPromise promise;
    auto future = promise.get_future();
    auto req_id = async_send_request_impl(
      *request,
      std::move(promise));
    return ROSFutureAndRequestId(std::move(future), req_id);
  }

  template<typename T>
  typename std::enable_if_t<
    rclcpp::TypeAdapter<ServiceT>::is_specialized::value &&
    std::is_same<T, CustomSharedRequest>::value,
    CustomFutureAndRequestId>
  async_send_request(T request)
  {
    CustomPromise promise;
    auto future = promise.get_future();
    auto req_id = async_send_request_impl(
      *request,
      std::move(promise));
    return CustomFutureAndRequestId(std::move(future), req_id);
  }

  /// Send a request to the service server and schedule a callback in the executor.
  /**
   * Similar to the previous overload, but a callback will automatically be called when a response is received.
   *
   * If the callback is never called, because we never got a reply for the service server, remove_pending_request()
   * has to be called with the returned request id or prune_pending_requests().
   * Not doing so will make the `Client` instance use more memory each time a response is not
   * received from the service server
   * In this case, it's convenient to setup a timer to cleanup the pending requests.
   * See for example the `examples_rclcpp_async_client` package in https://github.com/ros2/examples.
   *
   * \param[in] request request to be send.
   * \param[in] cb callback that will be called when we get a response for this request.
   * \return the request id representing the request just sent.
   */
  template<typename CallbackT>
  typename std::enable_if_t<
    rclcpp::function_traits::same_arguments<
      CallbackT,
      ROSCallbackType
    >::value,
    ROSSharedFutureAndRequestId
  >
  async_send_request(ROSSharedRequest request, CallbackT && cb)
  {
    ROSPromise promise;
    auto shared_future = promise.get_future().share();
    auto req_id = async_send_request_impl(
      *request,
      std::make_tuple(
        ROSCallbackType{std::forward<CallbackT>(cb)},
        shared_future,
        std::move(promise)));
    return ROSSharedFutureAndRequestId{std::move(shared_future), req_id};
  }

  template<typename CallbackT>
  typename std::enable_if_t<
    rclcpp::TypeAdapter<ServiceT>::is_specialized::value &&
    rclcpp::function_traits::same_arguments<
      CallbackT,
      CustomCallbackType
    >::value,
    CustomSharedFutureAndRequestId
  >
  async_send_request(CustomSharedRequest request, CallbackT && cb)
  {
    CustomPromise promise;
    auto shared_future = promise.get_future().share();
    auto req_id = async_send_request_impl(
      *request,
      std::make_tuple(
        CustomCallbackType{std::forward<CallbackT>(cb)},
        shared_future,
        std::move(promise)));
    return CustomSharedFutureAndRequestId{std::move(shared_future), req_id};
  }

  /// Send a request to the service server and schedule a callback in the executor.
  /**
   * Similar to the previous method, but you can get both the request and response in the callback.
   *
   * \param[in] request request to be send.
   * \param[in] cb callback that will be called when we get a response for this request.
   * \return the request id representing the request just sent.
   */
  template<typename CallbackT>
  typename std::enable_if_t<
    rclcpp::function_traits::same_arguments<
      CallbackT,
      ROSTotalCallbackWithRequestType
    >::value,
    ROSTotalSharedFutureWithRequestAndRequestId
  >
  async_send_request(ROSSharedRequest request, CallbackT && cb)
  {
    ROSTotalPromiseWithRequest promise;
    auto shared_future = promise.get_future().share();
    auto req_id = async_send_request_impl(
      *request,
      std::make_tuple(
        ROSTotalCallbackWithRequestType{std::forward<CallbackT>(cb)},
        request,
        shared_future,
        std::move(promise)));
    return ROSTotalSharedFutureWithRequestAndRequestId{std::move(shared_future), req_id};
  }

  template<typename CallbackT>
  typename std::enable_if_t<
    rclcpp::TypeAdapter<ServiceT>::is_specialized::value &&
    rclcpp::function_traits::same_arguments<
      CallbackT,
      CustomTotalCallbackWithRequestType
    >::value,
    CustomTotalSharedFutureWithRequestAndRequestId
  >
  async_send_request(CustomSharedRequest request, CallbackT && cb)
  {
    CustomTotalPromiseWithRequest promise;
    auto shared_future = promise.get_future().share();
    auto req_id = async_send_request_impl(
      *request,
      std::make_tuple(
        CustomTotalCallbackWithRequestType{std::forward<CallbackT>(cb)},
        request,
        shared_future,
        std::move(promise)));
    return CustomTotalSharedFutureWithRequestAndRequestId{std::move(shared_future), req_id};
  }

  /// Cleanup a pending request.
  /**
   * This notifies the client that we have waited long enough for a response from the server
   * to come, we have given up and we are not waiting for a response anymore.
   *
   * Not calling this will make the client start using more memory for each request
   * that never got a reply from the server.
   *
   * \param[in] request_id request id returned by async_send_request().
   * \return true when a pending request was removed, false if not (e.g. a response was received).
   */
  template<typename T>
  typename std::enable_if_t<std::is_same<T, int64_t>::value, bool>
  remove_pending_request(T request_id)
  {
    std::lock_guard guard(pending_requests_mutex_);
    return pending_requests_.erase(request_id) != 0u;
  }

  /// Cleanup a pending request.
  /**
   * Convenient overload, same as:
   *
   * `Client::remove_pending_request(this, future.request_id)`.
   */
  template<typename T>
  typename std::enable_if_t<std::is_same<T, ROSFutureAndRequestId>::value, bool>
  remove_pending_request(const T & future)
  {
    return this->remove_pending_request(future.request_id);
  }

  template<typename T>
  typename std::enable_if_t<std::is_same<T, CustomFutureAndRequestId>::value &&
    rclcpp::TypeAdapter<ServiceT>::is_specialized::value, bool>
  remove_pending_request(const T & future)
  {
    return this->remove_pending_request(future.request_id);
  }

  /// Cleanup a pending request.
  /**
   * Convenient overload, same as:
   *
   * `Client::remove_pending_request(this, future.request_id)`.
   */
  template<typename T>
  typename std::enable_if_t<std::is_same<T, ROSSharedFutureAndRequestId>::value, bool>
  remove_pending_request(const T & future)
  {
    return this->remove_pending_request(future.request_id);
  }

  template<typename T>
  typename std::enable_if_t<std::is_same<T, CustomSharedFutureAndRequestId>::value &&
    rclcpp::TypeAdapter<ServiceT>::is_specialized::value, bool>
  remove_pending_request(const T & future)
  {
    return this->remove_pending_request(future.request_id);
  }

  /// Cleanup a pending request.
  /**
   * Convenient overload, same as:
   *
   * `Client::remove_pending_request(this, future.request_id)`.
   */
  template<typename T>
  typename std::enable_if_t<std::is_same<T, ROSTotalSharedFutureWithRequestAndRequestId>::value,
    bool>
  remove_pending_request(const T & future)
  {
    return this->remove_pending_request(future.request_id);
  }

  template<typename T>
  typename std::enable_if_t<std::is_same<T,
    CustomTotalSharedFutureWithRequestAndRequestId>::value &&
    rclcpp::TypeAdapter<ServiceT>::is_specialized::value, bool>
  remove_pending_request(const T & future)
  {
    return this->remove_pending_request(future.request_id);
  }

  template<typename T>
  typename std::enable_if_t<std::is_same<T, ROSCustomSharedFutureWithRequestAndRequestId>::value &&
    rclcpp::TypeAdapter<ServiceT>::is_specialized::value, bool>
  remove_pending_request(const T & future)
  {
    return this->remove_pending_request(future.request_id);
  }

  template<typename T>
  typename std::enable_if_t<std::is_same<T, CustomROSSharedFutureWithRequestAndRequestId>::value &&
    rclcpp::TypeAdapter<ServiceT>::is_specialized::value, bool>
  remove_pending_request(const T & future)
  {
    return this->remove_pending_request(future.request_id);
  }

  /// Clean all pending requests.
  /**
   * \return number of pending requests that were removed.
   */
  size_t
  prune_pending_requests()
  {
    std::lock_guard guard(pending_requests_mutex_);
    auto ret = pending_requests_.size();
    pending_requests_.clear();
    return ret;
  }

  /// Clean all pending requests older than a time_point.
  /**
   * \param[in] time_point Requests that were sent before this point are going to be removed.
   * \param[inout] pruned_requests Removed requests id will be pushed to the vector
   *  if a pointer is provided.
   * \return number of pending requests that were removed.
   */
  template<typename AllocatorT = std::allocator<int64_t>>
  size_t
  prune_requests_older_than(
    std::chrono::time_point<std::chrono::system_clock> time_point,
    std::vector<int64_t, AllocatorT> * pruned_requests = nullptr)
  {
    std::lock_guard guard(pending_requests_mutex_);
    auto old_size = pending_requests_.size();
    for (auto it = pending_requests_.begin(), last = pending_requests_.end(); it != last; ) {
      if (it->second.first < time_point) {
        if (pruned_requests) {
          pruned_requests->push_back(it->first);
        }
        it = pending_requests_.erase(it);
      } else {
        ++it;
      }
    }
    return old_size - pending_requests_.size();
  }

  /// Configure client introspection.
  /**
   * \param[in] clock clock to use to generate introspection timestamps
   * \param[in] qos_service_event_pub QoS settings to use when creating the introspection publisher
   * \param[in] introspection_state the state to set introspection to
   */
  void
  configure_introspection(
    Clock::SharedPtr clock, const QoS & qos_service_event_pub,
    rcl_service_introspection_state_t introspection_state)
  {
    rcl_publisher_options_t pub_opts = rcl_publisher_get_default_options();
    pub_opts.qos = qos_service_event_pub.get_rmw_qos_profile();

    rcl_ret_t ret = rcl_client_configure_service_introspection(
      client_handle_.get(),
      node_handle_.get(),
      clock->get_clock_handle(),
      srv_type_support_handle_,
      pub_opts,
      introspection_state);

    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to configure client introspection");
    }
  }

protected:
  template<bool is_adapted_type = rclcpp::TypeAdapter<ServiceT>::is_specialized::value>
  struct CallbackWithRequestTypeValueVariantHelper {};

  template<>
  struct CallbackWithRequestTypeValueVariantHelper<false>
  {
    using CallbackWithRequestTypeValueVariant = std::variant<
      std::tuple<ROSTotalCallbackWithRequestType, ROSSharedRequest,
      ROSTotalSharedFutureWithRequest, ROSTotalPromiseWithRequest>>;
  };

  template<>
  struct CallbackWithRequestTypeValueVariantHelper<true>
  {
    using CallbackWithRequestTypeValueVariant = std::variant<
      std::tuple<ROSTotalCallbackWithRequestType, ROSSharedRequest,
      ROSTotalSharedFutureWithRequest, ROSTotalPromiseWithRequest>,
      std::tuple<CustomTotalCallbackWithRequestType, CustomSharedRequest,
      CustomTotalSharedFutureWithRequest, CustomTotalPromiseWithRequest>>;
  };

  using CallbackWithRequestTypeValueVariant =
    typename CallbackWithRequestTypeValueVariantHelper<>::CallbackWithRequestTypeValueVariant;

  template<bool is_adapted_type = rclcpp::TypeAdapter<ServiceT>::is_specialized::value>
  struct CallbackTypeValueVariantHelper {};

  template<>
  struct CallbackTypeValueVariantHelper<false>
  {
    using CallbackTypeValueVariant = std::variant<std::tuple<ROSCallbackType, ROSSharedFuture,
        ROSPromise>>;
  };

  template<>
  struct CallbackTypeValueVariantHelper<true>
  {
    using CallbackTypeValueVariant = std::variant<
      std::tuple<ROSCallbackType, ROSSharedFuture, ROSPromise>,
      std::tuple<CustomCallbackType, CustomSharedFuture, CustomPromise>>;
  };

  using CallbackTypeValueVariant = typename CallbackTypeValueVariantHelper<>::
    CallbackTypeValueVariant;

  template<bool is_adapted_type = rclcpp::TypeAdapter<ServiceT>::is_specialized::value>
  struct CallbackInfoVariantHelper {};

  template<>
  struct CallbackInfoVariantHelper<false>
  {
    using CallbackInfoVariant = std::variant<
      ROSPromise,
      CallbackTypeValueVariant,
      CallbackWithRequestTypeValueVariant>;
  };

  template<>
  struct CallbackInfoVariantHelper<true>
  {
    using CallbackInfoVariant = std::variant<
      ROSPromise,
      CustomPromise,
      CallbackTypeValueVariant,
      CallbackWithRequestTypeValueVariant>;
  };

  using CallbackInfoVariant = typename CallbackInfoVariantHelper<>::CallbackInfoVariant;

  template<typename T>
  typename std::enable_if_t<
    rosidl_generator_traits::is_message<T>::value &&
    std::is_same<T, ROSServiceRequestType>::value,
    int64_t
  >
  async_send_request_impl(const T & request, CallbackInfoVariant value)
  {
    int64_t sequence_number;
    std::lock_guard<std::mutex> lock(pending_requests_mutex_);
    rcl_ret_t ret = rcl_send_request(get_client_handle().get(), &request, &sequence_number);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send request");
    }
    pending_requests_.try_emplace(
      sequence_number,
      std::make_pair(std::chrono::system_clock::now(), std::move(value)));
    return sequence_number;
  }

  template<typename T>
  typename std::enable_if_t<
    rclcpp::TypeAdapter<ServiceT>::is_specialized::value &&
    std::is_same<T, ServiceRequestType>::value,
    int64_t
  >
  async_send_request_impl(const T & request, CallbackInfoVariant value)
  {
    int64_t sequence_number;
    std::lock_guard<std::mutex> lock(pending_requests_mutex_);

    ROSServiceRequestType ros_service_request;
    rclcpp::TypeAdapter<ServiceT>::convert_to_ros_service_request(
      request, ros_service_request);

    rcl_ret_t ret = rcl_send_request(
      get_client_handle().get(), &ros_service_request, &sequence_number);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send request");
    }
    pending_requests_.try_emplace(
      sequence_number,
      std::make_pair(std::chrono::system_clock::now(), std::move(value)));
    return sequence_number;
  }

  std::optional<CallbackInfoVariant>
  get_and_erase_pending_request(int64_t request_number)
  {
    std::unique_lock<std::mutex> lock(pending_requests_mutex_);
    auto it = this->pending_requests_.find(request_number);
    if (it == this->pending_requests_.end()) {
      RCUTILS_LOG_DEBUG_NAMED(
        "rclcpp",
        "Received invalid sequence number. Ignoring...");
      return std::nullopt;
    }
    auto value = std::move(it->second.second);
    this->pending_requests_.erase(request_number);
    return value;
  }

  RCLCPP_DISABLE_COPY(Client)

  std::unordered_map<
    int64_t,
    std::pair<
      std::chrono::time_point<std::chrono::system_clock>,
      CallbackInfoVariant>>
  pending_requests_;
  std::mutex pending_requests_mutex_;

private:
  const rosidl_service_type_support_t * srv_type_support_handle_;
};

}  // namespace rclcpp

#endif  // RCLCPP__CLIENT_HPP_
