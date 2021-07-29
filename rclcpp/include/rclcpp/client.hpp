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

#include <atomic>
#include <future>
#include <unordered_map>
#include <memory>
#include <optional>  // NOLINT, cpplint doesn't think this is a cpp std header
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <variant>  // NOLINT

#include "rcl/client.h"
#include "rcl/error_handling.h"
#include "rcl/wait.h"

#include "rclcpp/exceptions.hpp"
#include "rclcpp/function_traits.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rclcpp/visibility_control.hpp"

#include "rcutils/logging_macros.h"

#include "rmw/error_handling.h"
#include "rmw/rmw.h"

namespace rclcpp
{

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
  virtual ~ClientBase();

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

  rclcpp::node_interfaces::NodeGraphInterface::WeakPtr node_graph_;
  std::shared_ptr<rcl_node_t> node_handle_;
  std::shared_ptr<rclcpp::Context> context_;

  std::shared_ptr<rcl_client_t> client_handle_;

  std::atomic<bool> in_use_by_wait_set_{false};
};

template<typename ServiceT>
class Client : public ClientBase
{
public:
  using Request = typename ServiceT::Request;
  using Response = typename ServiceT::Response;

  using SharedRequest = typename ServiceT::Request::SharedPtr;
  using SharedResponse = typename ServiceT::Response::SharedPtr;

  using Promise = std::promise<SharedResponse>;
  using PromiseWithRequest = std::promise<std::pair<SharedRequest, SharedResponse>>;

  using SharedPromise = std::shared_ptr<Promise>;
  using SharedPromiseWithRequest = std::shared_ptr<PromiseWithRequest>;

  using Future = std::future<SharedResponse>;
  using SharedFuture = std::shared_future<SharedResponse>;
  using SharedFutureWithRequest = std::shared_future<std::pair<SharedRequest, SharedResponse>>;

  using CallbackType = std::function<void (SharedFuture)>;
  using CallbackWithRequestType = std::function<void (SharedFutureWithRequest)>;

  RCLCPP_SMART_PTR_DEFINITIONS(Client)

  /// A convenient Client::Future and request id pair.
  class FutureAndRequestId
  {
public:
    FutureAndRequestId(Future impl, int64_t req_id)
    : impl_(std::move(impl)), req_id_(req_id)
    {}

    /// Allow implicit conversions to `std::future` by reference.
    // TODO(ivanpauno): Maybe, deprecate this in favor of get_future() (?)
    operator Future &() {return impl_;}

    /// Deprecated, use take_future() instead.
    /**
     * Allow implicit conversions to `std::future` by value.
     * \deprecated
     */
    [[deprecated("FutureAndRequestId: use take_future() instead of an implicit conversion")]]
    operator Future() {return impl_;}

    /// Returns the internal `std::future`, moving it out.
    /**
     * After calling this method, the internal future gets invalidated.
     */
    Future
    take_future() {return impl_;}

    /// Getter for the internal future.
    Future &
    get_future() {return impl_;}

    /// Getter for the internal future.
    const Future &
    get_future() const {return impl_;}

    /// Returns the request id associated with this future.
    int64_t get_request_id() const {return req_id_;}

    // delegate future like methods in the std::future impl_

    /// See std::future::share().
    SharedFuture share() noexcept {return impl_.share();}
    /// See std::future::get().
    SharedResponse get() {return impl_.get();}
    /// See std::future::valid().
    bool valid() const noexcept {return impl_.valid();}
    /// See std::future::wait().
    void wait() const {return impl_.wait();}
    /// See std::future::wait_for().
    template<class Rep, class Period>
    std::future_status wait_for(
      const std::chrono::duration<Rep, Period> & timeout_duration) const
    {
      return impl_.wait_for(timeout_duration);
    }
    /// See std::future::wait_until().
    template<class Clock, class Duration>
    std::future_status wait_until(
      const std::chrono::time_point<Clock, Duration> & timeout_time) const
    {
      return impl_.wait_until(timeout_time);
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

private:
    Future impl_;
    int64_t req_id_;
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
  : ClientBase(node_base, node_graph)
  {
    using rosidl_typesupport_cpp::get_service_type_support_handle;
    auto service_type_support_handle =
      get_service_type_support_handle<ServiceT>();
    rcl_ret_t ret = rcl_client_init(
      this->get_client_handle().get(),
      this->get_rcl_node_handle(),
      service_type_support_handle,
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
   * \param[out] response_out The reference to a Service Response into
   *   which the middleware will copy the response being taken.
   * \param[out] request_header_out The request header to be filled by the
   *   middleware when taking, and which can be used to associte the response
   *   to a specific request.
   * \returns true if the response was taken, otherwise false.
   * \throws rclcpp::exceptions::RCLError based exceptions if the underlying
   *   rcl function fail.
   */
  bool
  take_response(typename ServiceT::Response & response_out, rmw_request_id_t & request_header_out)
  {
    return this->take_type_erased_response(&response_out, request_header_out);
  }

  /// Create a shared pointer with the response type
  /**
   * \return shared pointer with the response type
   */
  std::shared_ptr<void>
  create_response() override
  {
    return std::shared_ptr<void>(new typename ServiceT::Response());
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
    auto opt = this->get_and_erase_pending_request(request_header->sequence_number);
    if (!opt) {
      return;
    }
    auto & value = *opt;
    auto typed_response = std::static_pointer_cast<typename ServiceT::Response>(
      std::move(response));
    if (std::holds_alternative<Promise>(value)) {
      auto & promise = std::get<Promise>(value);
      promise.set_value(std::move(typed_response));
    } else if (std::holds_alternative<CallbackType>(value)) {
      Promise promise;
      promise.set_value(std::move(typed_response));
      const auto & callback = std::get<CallbackType>(value);
      callback(promise.get_future().share());
    } else if (std::holds_alternative<std::pair<CallbackWithRequestType, SharedRequest>>(value)) {
      PromiseWithRequest promise;
      const auto & pair = std::get<std::pair<CallbackWithRequestType, SharedRequest>>(value);
      promise.set_value(std::make_pair(std::move(pair.second), std::move(typed_response)));
      pair.first(promise.get_future().share());
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
  FutureAndRequestId
  async_send_request(SharedRequest request)
  {
    Promise promise;
    auto future = promise.get_future();
    auto req_id = async_send_request_impl(
      *request,
      std::move(promise));
    return FutureAndRequestId(std::move(future), req_id);
  }

  /// Send a request to the service server and schedule a callback in the executor.
  /**
   * Similar to the previous overload, but a callback will automatically be called when a response is received.
   *
   * If the callback is never called, because we never got a reply for the service server, remove_pending_request()
   * has to be called with the returned request id or prune_pending_requests().
   * Not doing so will make the `Client` instance use more memory each time a response is not
   * received from the service server.
   * In this case, it's convenient to setup a timer to cleanup the pending requests.
   *
   * \param[in] request request to be send.
   * \param[in] cb callback that will be called when we get a response for this request.
   * \return the request id representing the request just sent.
   */
  // TODO(ivanpauno): Link to example that shows how to cleanup requests.
  template<
    typename CallbackT,
    typename std::enable_if<
      rclcpp::function_traits::same_arguments<
        CallbackT,
        CallbackType
      >::value
    >::type * = nullptr
  >
  int64_t
  async_send_request(SharedRequest request, CallbackT && cb)
  {
    return async_send_request_impl(
      *request,
      CallbackType{std::forward<CallbackT>(cb)});
  }

  /// Send a request to the service server and schedule a callback in the executor.
  /**
   * Similar to the previous method, but you can get both the request and response in the callback.
   *
   * \param[in] request request to be send.
   * \param[in] cb callback that will be called when we get a response for this request.
   * \return the request id representing the request just sent.
   */
  // TODO(ivanpauno): Deprecate this.
  // If someone wants the request they can capture it in the lambda.
  template<
    typename CallbackT,
    typename std::enable_if<
      rclcpp::function_traits::same_arguments<
        CallbackT,
        CallbackWithRequestType
      >::value
    >::type * = nullptr
  >
  int64_t
  async_send_request(SharedRequest request, CallbackT && cb)
  {
    auto & req = *request;
    return async_send_request_impl(
      req,
      std::make_pair(CallbackWithRequestType{std::forward<CallbackT>(cb)}, std::move(request)));
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
  bool
  remove_pending_request(int64_t request_id)
  {
    std::lock_guard guard(pending_requests_mutex_);
    return pending_requests_.erase(request_id) != 0u;
  }

  /// Cleanup a pending request.
  /**
   * Convenient overload, same as:
   *
   * `Client::remove_pending_request(this, future.get_request_id())`.
   */
  bool
  remove_pending_request(const FutureAndRequestId & future)
  {
    return this->remove_pending_request(future.get_request_id());
  }

  /// Clean all pending requests.
  /**
   * \return number of pending requests that were removed.
   */
  size_t
  prune_requests()
  {
    std::lock_guard guard(pending_requests_mutex_);
    auto ret = pending_requests_.size();
    pending_requests_.clear();
    return ret;
  }

protected:
  using PendingRequestsMapValue = std::variant<
    std::promise<SharedResponse>,
    CallbackType,
    std::pair<CallbackWithRequestType, SharedRequest>>;

  RCLCPP_PUBLIC
  int64_t
  async_send_request_impl(const Request & request, PendingRequestsMapValue value)
  {
    int64_t sequence_number;
    rcl_ret_t ret = rcl_send_request(get_client_handle().get(), &request, &sequence_number);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send request");
    }
    {
      std::lock_guard<std::mutex> lock(pending_requests_mutex_);
      pending_requests_.try_emplace(sequence_number, std::move(value));
    }
    return sequence_number;
  }

  RCLCPP_PUBLIC
  std::optional<PendingRequestsMapValue>
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
    auto value = std::move(it->second);
    this->pending_requests_.erase(request_number);
    return value;
  }

  RCLCPP_DISABLE_COPY(Client)

  std::unordered_map<int64_t, PendingRequestsMapValue> pending_requests_;
  std::mutex pending_requests_mutex_;
};

}  // namespace rclcpp

#endif  // RCLCPP__CLIENT_HPP_
