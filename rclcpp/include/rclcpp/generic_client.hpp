// Copyright 2023 Sony Group Corporation.
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

#ifndef RCLCPP__GENERIC_CLIENT_HPP_
#define RCLCPP__GENERIC_CLIENT_HPP_

#include <map>
#include <memory>
#include <future>
#include <string>
#include <tuple>
#include <vector>
#include <utility>

#include "rcl/client.h"

#include "rclcpp/client.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rcpputils/shared_library.hpp"

#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

namespace rclcpp
{
class GenericClient : public ClientBase
{
public:
  using Request = void *;   // Deserialized data pointer of request message
  using Response = void *;  // Deserialized data pointer of response message

  using SharedResponse = std::shared_ptr<void>;

  using Promise = std::promise<SharedResponse>;
  using SharedPromise = std::shared_ptr<Promise>;

  using Future = std::future<SharedResponse>;
  using SharedFuture = std::shared_future<SharedResponse>;

  using CallbackType = std::function<void (SharedFuture)>;

  RCLCPP_SMART_PTR_DEFINITIONS(GenericClient)

  /// A convenient GenericClient::Future and request id pair.
  /**
   * Public members:
   * - future: a std::future<void *>.
   * - request_id: the request id associated with the future.
   *
   * All the other methods are equivalent to the ones std::future provides.
   */
  struct FutureAndRequestId
    : detail::FutureAndRequestId<Future>
  {
    using detail::FutureAndRequestId<Future>::FutureAndRequestId;

    /// See std::future::share().
    SharedFuture share() noexcept {return this->future.share();}

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

  /// A convenient GenericClient::SharedFuture and request id pair.
  /**
   * Public members:
   * - future: a std::shared_future<SharedResponse>.
   * - request_id: the request id associated with the future.
   *
   * All the other methods are equivalent to the ones std::shared_future provides.
   */
  struct SharedFutureAndRequestId
    : detail::FutureAndRequestId<std::shared_future<SharedResponse>>
  {
    using detail::FutureAndRequestId<std::shared_future<SharedResponse>>::FutureAndRequestId;
  };

  GenericClient(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
    const std::string & service_name,
    const std::string & service_type,
    rcl_client_options_t & client_options);

  RCLCPP_PUBLIC
  SharedResponse
  create_response() override;

  RCLCPP_PUBLIC
  std::shared_ptr<rmw_request_id_t>
  create_request_header() override;

  RCLCPP_PUBLIC
  void
  handle_response(
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<void> response) override;

  /// Send a request to the service server.
  /**
   * This method returns a `FutureAndRequestId` instance
   * that can be passed to Executor::spin_until_future_complete() to
   * wait until it has been completed.
   *
   * If the future never completes,
   * e.g. the call to Executor::spin_until_future_complete() times out,
   * GenericClient::remove_pending_request() must be called to clean the client internal state.
   * Not doing so will make the `GenericClient` instance to use more memory each time a response is
   * not received from the service server.
   *
   * ```cpp
   * auto future = generic_client->async_send_request(my_request);
   * if (
   *   rclcpp::FutureReturnCode::TIMEOUT ==
   *   executor->spin_until_future_complete(future, timeout))
   * {
   *   generic_client->remove_pending_request(future);
   *   // handle timeout
   * } else {
   *   handle_response(future.get());
   * }
   * ```
   *
   * \param[in] request request to be send.
   * \return a FutureAndRequestId instance.
   */
  RCLCPP_PUBLIC
  FutureAndRequestId
  async_send_request(const Request request);

  /// Send a request to the service server and schedule a callback in the executor.
  /**
   * Similar to the previous overload, but a callback will automatically be called when a response
   * is received.
   *
   * If the callback is never called, because we never got a reply for the service server,
   * remove_pending_request() has to be called with the returned request id or
   * prune_pending_requests().
   * Not doing so will make the `GenericClient` instance use more memory each time a response is not
   * received from the service server.
   * In this case, it's convenient to setup a timer to cleanup the pending requests.
   *
   * \param[in] request request to be send.
   * \param[in] cb callback that will be called when we get a response for this request.
   * \return the request id representing the request just sent.
   */
  template<
    typename CallbackT,
    typename std::enable_if<
      rclcpp::function_traits::same_arguments<
        CallbackT,
        CallbackType
      >::value
    >::type * = nullptr
  >
  SharedFutureAndRequestId
  async_send_request(const Request request, CallbackT && cb)
  {
    Promise promise;
    auto shared_future = promise.get_future().share();
    auto req_id = async_send_request_impl(
      request,
      std::make_tuple(
        CallbackType{std::forward<CallbackT>(cb)},
        shared_future,
        std::move(promise)));
    return SharedFutureAndRequestId{std::move(shared_future), req_id};
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
    return detail::prune_requests_older_than_impl(
      pending_requests_,
      pending_requests_mutex_,
      time_point,
      pruned_requests);
  }

  /// Clean all pending requests.
  /**
   * \return number of pending requests that were removed.
   */
  RCLCPP_PUBLIC
  size_t
  prune_pending_requests();

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
  RCLCPP_PUBLIC
  bool
  remove_pending_request(
    int64_t request_id);

  /// Cleanup a pending request.
  /**
   * Convenient overload, same as:
   *
   * `GenericClient::remove_pending_request(this, future.request_id)`.
   */
  RCLCPP_PUBLIC
  bool
  remove_pending_request(
    const FutureAndRequestId & future);

  /// Cleanup a pending request.
  /**
   * Convenient overload, same as:
   *
   * `GenericClient::remove_pending_request(this, future.request_id)`.
   */
  RCLCPP_PUBLIC
  bool
  remove_pending_request(
    const SharedFutureAndRequestId & future);

  /// Take the next response for this client.
  /**
   * \sa ClientBase::take_type_erased_response().
   *
   * \param[out] response_out The reference to a Service Response into
   *   which the middleware will copy the response being taken.
   * \param[out] request_header_out The request header to be filled by the
   *   middleware when taking, and which can be used to associate the response
   *   to a specific request.
   * \returns true if the response was taken, otherwise false.
   * \throws rclcpp::exceptions::RCLError based exceptions if the underlying
   *   rcl function fail.
   */
  RCLCPP_PUBLIC
  bool
  take_response(Response response_out, rmw_request_id_t & request_header_out)
  {
    return this->take_type_erased_response(response_out, request_header_out);
  }

protected:
  using CallbackTypeValueVariant = std::tuple<CallbackType, SharedFuture, Promise>;
  using CallbackInfoVariant = std::variant<
    std::promise<SharedResponse>,
    CallbackTypeValueVariant>;  // Use variant for extension

  RCLCPP_PUBLIC
  int64_t
  async_send_request_impl(
    const Request request,
    CallbackInfoVariant value);

  std::optional<CallbackInfoVariant>
  get_and_erase_pending_request(
    int64_t request_number);

  RCLCPP_DISABLE_COPY(GenericClient)

  std::map<int64_t, std::pair<
      std::chrono::time_point<std::chrono::system_clock>,
      CallbackInfoVariant>> pending_requests_;
  std::mutex pending_requests_mutex_;

private:
  std::shared_ptr<rcpputils::SharedLibrary> ts_lib_;
  const rosidl_typesupport_introspection_cpp::MessageMembers * response_members_;
};
}  // namespace rclcpp

#endif  // RCLCPP__GENERIC_CLIENT_HPP_
