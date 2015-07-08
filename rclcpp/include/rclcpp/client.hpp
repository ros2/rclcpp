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

#ifndef RCLCPP_RCLCPP_CLIENT_HPP_
#define RCLCPP_RCLCPP_CLIENT_HPP_

#include <future>
#include <map>
#include <memory>
#include <utility>

#include <rmw/error_handling.h>
#include <rmw/rmw.h>

#include <rclcpp/macros.hpp>
#include <rclcpp/utilities.hpp>

namespace rclcpp
{

// Forward declaration for friend statement
namespace executor
{
class Executor;
} // namespace executor

namespace client
{

class ClientBase
{
  friend class rclcpp::executor::Executor;

public:
  RCLCPP_MAKE_SHARED_DEFINITIONS(ClientBase);

  ClientBase(rmw_client_t * client_handle, const std::string & service_name)
  : client_handle_(client_handle), service_name_(service_name)
  {}

  ~ClientBase()
  {
    if (client_handle_ != nullptr) {
      rmw_destroy_client(client_handle_);
      client_handle_ = nullptr;
    }
  }

  std::string get_service_name()
  {
    return this->service_name_;
  }

  const rmw_client_t * get_client_handle()
  {
    return this->client_handle_;
  }

  virtual std::shared_ptr<void> create_response() = 0;
  virtual std::shared_ptr<void> create_request_header() = 0;
  virtual void handle_response(
    std::shared_ptr<void> & request_header, std::shared_ptr<void> & response) = 0;

private:
  RCLCPP_DISABLE_COPY(ClientBase);

  rmw_client_t * client_handle_;
  std::string service_name_;

};

template<typename ServiceT>
class Client : public ClientBase
{
public:
  typedef std::promise<typename ServiceT::Response::SharedPtr> Promise;
  typedef std::shared_ptr<Promise> SharedPromise;
  typedef std::shared_future<typename ServiceT::Response::SharedPtr> SharedFuture;

  typedef std::function<void (SharedFuture)> CallbackType;

  RCLCPP_MAKE_SHARED_DEFINITIONS(Client);

  Client(rmw_client_t * client_handle, const std::string & service_name)
  : ClientBase(client_handle, service_name)
  {}

  std::shared_ptr<void> create_response()
  {
    return std::shared_ptr<void>(new typename ServiceT::Response());
  }

  std::shared_ptr<void> create_request_header()
  {
    // TODO(wjwwood): This should probably use rmw_request_id's allocator.
    //                (since it is a C type)
    return std::shared_ptr<void>(new rmw_request_id_t);
  }

  void handle_response(std::shared_ptr<void> & request_header, std::shared_ptr<void> & response)
  {
    auto typed_request_header = std::static_pointer_cast<rmw_request_id_t>(request_header);
    auto typed_response = std::static_pointer_cast<typename ServiceT::Response>(response);
    int64_t sequence_number = typed_request_header->sequence_number;
    // TODO this must check if the sequence_number is valid otherwise the call_promise will be null
    auto tuple = this->pending_requests_[sequence_number];
    auto call_promise = std::get<0>(tuple);
    auto callback = std::get<1>(tuple);
    auto future = std::get<2>(tuple);
    this->pending_requests_.erase(sequence_number);
    call_promise->set_value(typed_response);
    callback(future);
  }

  SharedFuture async_send_request(
    typename ServiceT::Request::SharedPtr & request)
  {
    return async_send_request(request, [](SharedFuture f) {});
  }

  SharedFuture async_send_request(
    typename ServiceT::Request::SharedPtr & request,
    CallbackType cb)
  {
    int64_t sequence_number;
    // TODO(wjwwood): Check the return code.
    rmw_ret_t status = rmw_send_request(get_client_handle(), request.get(), &sequence_number);
    if (status != RMW_RET_OK) {
      // *INDENT-OFF* (prevent uncrustify from making unecessary indents here)
      throw std::runtime_error(
        std::string("failed to send request: ") +
        (rmw_get_error_string() ? rmw_get_error_string() : ""));
      // *INDENT-ON*
    }

    SharedPromise call_promise = std::make_shared<Promise>();
    SharedFuture f(call_promise->get_future());
    pending_requests_[sequence_number] = std::make_tuple(call_promise, cb, f);
    return f;
  }

private:
  RCLCPP_DISABLE_COPY(Client);

  std::map<int64_t, std::tuple<SharedPromise, CallbackType, SharedFuture>> pending_requests_;
};

} /* namespace client */
} /* namespace rclcpp */

#endif /* RCLCPP_RCLCPP_CLIENT_HPP_ */
