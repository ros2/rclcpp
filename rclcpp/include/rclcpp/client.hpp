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

#include <future>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>

#include "rcl/client.h"
#include "rcl/error_handling.h"
#include "rcl/wait.h"

#include "rclcpp/function_traits.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/visibility_control.hpp"

#include "rmw/error_handling.h"
#include "rmw/rmw.h"

namespace rclcpp
{

namespace node
{
class Node;
}  // namespace node

namespace client
{

class ClientBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(ClientBase);

  RCLCPP_PUBLIC
  ClientBase(
    rclcpp::node::Node * parent_node,
    const std::string & service_name);

  RCLCPP_PUBLIC
  virtual ~ClientBase();

  RCLCPP_PUBLIC
  const std::string &
  get_service_name() const;

  RCLCPP_PUBLIC
  const rcl_client_t *
  get_client_handle() const;

  RCLCPP_PUBLIC
  bool
  service_is_ready() const;

  template<typename RatioT = std::milli>
  bool
  wait_for_service(
    std::chrono::duration<int64_t, RatioT> timeout = std::chrono::duration<int64_t, RatioT>(-1))
  {
    return wait_for_service_nanoseconds(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timeout)
    );
  }

  virtual std::shared_ptr<void> create_response() = 0;
  virtual std::shared_ptr<rmw_request_id_t> create_request_header() = 0;
  virtual void handle_response(
    std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> response) = 0;

protected:
  RCLCPP_DISABLE_COPY(ClientBase);

  bool
  wait_for_service_nanoseconds(std::chrono::nanoseconds timeout);

  rcl_node_t *
  get_rcl_node_handle() const;

  rclcpp::node::Node * node_;
  std::shared_ptr<rcl_node_t> node_handle_;

  rcl_client_t client_handle_ = rcl_get_zero_initialized_client();
  std::string service_name_;
};

template<typename ServiceT>
class Client : public ClientBase
{
public:
  using SharedRequest = typename ServiceT::Request::SharedPtr;
  using SharedResponse = typename ServiceT::Response::SharedPtr;

  using Promise = std::promise<SharedResponse>;
  using PromiseWithRequest = std::promise<std::pair<SharedRequest, SharedResponse>>;

  using SharedPromise = std::shared_ptr<Promise>;
  using SharedPromiseWithRequest = std::shared_ptr<PromiseWithRequest>;

  using SharedFuture = std::shared_future<SharedResponse>;
  using SharedFutureWithRequest = std::shared_future<std::pair<SharedRequest, SharedResponse>>;

  using CallbackType = std::function<void(SharedFuture)>;
  using CallbackWithRequestType = std::function<void(SharedFutureWithRequest)>;

  RCLCPP_SMART_PTR_DEFINITIONS(Client);

  Client(
    rclcpp::node::Node * parent_node,
    const std::string & service_name,
    rcl_client_options_t & client_options)
  : ClientBase(parent_node, service_name)
  {
    using rosidl_generator_cpp::get_service_type_support_handle;
    auto service_type_support_handle =
      get_service_type_support_handle<ServiceT>();
    if (rcl_client_init(&client_handle_, this->get_rcl_node_handle(),
      service_type_support_handle, service_name.c_str(), &client_options) != RCL_RET_OK)
    {
      // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
      throw std::runtime_error(
        std::string("could not create client: ") +
        rcl_get_error_string_safe());
      // *INDENT-ON*
    }
  }

  virtual ~Client()
  {
    if (rcl_client_fini(&client_handle_, this->get_rcl_node_handle()) != RCL_RET_OK) {
      fprintf(stderr,
        "Error in destruction of rmw client handle: %s\n", rmw_get_error_string_safe());
    }
  }

  std::shared_ptr<void>
  create_response()
  {
    return std::shared_ptr<void>(new typename ServiceT::Response());
  }

  std::shared_ptr<rmw_request_id_t>
  create_request_header()
  {
    // TODO(wjwwood): This should probably use rmw_request_id's allocator.
    //                (since it is a C type)
    return std::shared_ptr<rmw_request_id_t>(new rmw_request_id_t);
  }

  void
  handle_response(std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<void> response)
  {
    std::lock_guard<std::mutex> lock(pending_requests_mutex_);
    auto typed_response = std::static_pointer_cast<typename ServiceT::Response>(response);
    int64_t sequence_number = request_header->sequence_number;
    // TODO(esteve) this should throw instead since it is not expected to happen in the first place
    if (this->pending_requests_.count(sequence_number) == 0) {
      fprintf(stderr, "Received invalid sequence number. Ignoring...\n");
      return;
    }
    auto tuple = this->pending_requests_[sequence_number];
    auto call_promise = std::get<0>(tuple);
    auto callback = std::get<1>(tuple);
    auto future = std::get<2>(tuple);
    this->pending_requests_.erase(sequence_number);
    call_promise->set_value(typed_response);
    callback(future);
  }

  SharedFuture
  async_send_request(SharedRequest request)
  {
    return async_send_request(request, [](SharedFuture) {});
  }

  template<
    typename CallbackT,
    typename std::enable_if<
      rclcpp::function_traits::same_arguments<
        CallbackT,
        CallbackType
      >::value
    >::type * = nullptr
  >
  SharedFuture
  async_send_request(SharedRequest request, CallbackT && cb)
  {
    std::lock_guard<std::mutex> lock(pending_requests_mutex_);
    int64_t sequence_number;
    if (RCL_RET_OK != rcl_send_request(get_client_handle(), request.get(), &sequence_number)) {
      // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
      throw std::runtime_error(
        std::string("failed to send request: ") + rcl_get_error_string_safe());
      // *INDENT-ON*
    }

    SharedPromise call_promise = std::make_shared<Promise>();
    SharedFuture f(call_promise->get_future());
    pending_requests_[sequence_number] =
      std::make_tuple(call_promise, std::forward<CallbackType>(cb), f);
    return f;
  }

  template<
    typename CallbackT,
    typename std::enable_if<
      rclcpp::function_traits::same_arguments<
        CallbackT,
        CallbackWithRequestType
      >::value
    >::type * = nullptr
  >
  SharedFutureWithRequest
  async_send_request(SharedRequest request, CallbackT && cb)
  {
    SharedPromiseWithRequest promise = std::make_shared<PromiseWithRequest>();
    SharedFutureWithRequest future_with_request(promise->get_future());

    auto wrapping_cb = [future_with_request, promise, request, &cb](SharedFuture future) {
        auto response = future.get();
        promise->set_value(std::make_pair(request, response));
        cb(future_with_request);
      };

    async_send_request(request, wrapping_cb);

    return future_with_request;
  }

private:
  RCLCPP_DISABLE_COPY(Client);

  std::map<int64_t, std::tuple<SharedPromise, CallbackType, SharedFuture>> pending_requests_;
  std::mutex pending_requests_mutex_;
};

}  // namespace client
}  // namespace rclcpp

#endif  // RCLCPP__CLIENT_HPP_
