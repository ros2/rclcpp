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
  using Request = void *;   // Serialized data pointer of request message
  using Response = void *;  // Serialized data pointer of response message

  using SharedResponse = std::shared_ptr<void>;

  using Promise = std::promise<SharedResponse>;
  using SharedPromise = std::shared_ptr<Promise>;

  using Future = std::future<SharedResponse>;
  using SharedFuture = std::shared_future<SharedResponse>;

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
  RCLCPP_PUBLIC
  FutureAndRequestId
  async_send_request(const Request request);

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

  RCLCPP_PUBLIC
  size_t
  prune_pending_requests();

  RCLCPP_PUBLIC
  bool
  remove_pending_request(
    int64_t request_id);

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
  using CallbackInfoVariant = std::variant<
    std::promise<SharedResponse>>;  // Use variant for extension

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
