/* Copyright 2014 Open Source Robotics Foundation, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef RCLCPP_RCLCPP_CLIENT_HPP_
#define RCLCPP_RCLCPP_CLIENT_HPP_

#include <memory>

#include <ros_middleware_interface/functions.h>
#include <ros_middleware_interface/handles.h>

#include <rclcpp/utilities.hpp>
#include <rclcpp/macros.hpp>
#include <future>
#include <map>
#include <utility>

namespace rclcpp
{

// Forward declaration for friend statement
namespace node {class Node;}

namespace client
{

class ClientBase
{
  friend class rclcpp::executor::Executor;
public:
  RCLCPP_MAKE_SHARED_DEFINITIONS(ClientBase);

  ClientBase(
    ros_middleware_interface::ClientHandle client_handle,
    std::string &service_name)
    : client_handle_(client_handle), service_name_(service_name)
  {}

  std::string get_service_name()
  {
    return this->service_name_;
  }

  ros_middleware_interface::ClientHandle get_client_handle()
  {
    return this->client_handle_;
  }

  virtual std::shared_ptr<void> create_response() = 0;
  virtual std::shared_ptr<void> create_request_header() = 0;
  virtual void handle_response(std::shared_ptr<void> &response, std::shared_ptr<void> &req_id) = 0;

private:
  RCLCPP_DISABLE_COPY(ClientBase);

  ros_middleware_interface::ClientHandle client_handle_;
  std::string service_name_;

};

template<typename ServiceT>
class Client : public ClientBase
{
public:
 typedef std::promise<typename ServiceT::Response::Ptr> Promise;
 typedef std::shared_ptr<Promise> SharedPromise;
 typedef std::shared_future<typename ServiceT::Response::Ptr> SharedFuture;

  RCLCPP_MAKE_SHARED_DEFINITIONS(Client);

  Client(ros_middleware_interface::ClientHandle client_handle,
         std::string& service_name)
    : ClientBase(client_handle, service_name)
  {}

  std::shared_ptr<void> get_response(int64_t sequence_number)
  {
    auto pair = this->pending_requests_[sequence_number];
    return pair.second;
  }

  std::shared_ptr<void> create_response()
  {
    return std::shared_ptr<void>(new typename ServiceT::Response());
  }

  std::shared_ptr<void> create_request_header()
  {
    return std::shared_ptr<void>(new ros_middleware_interface::RequestId());
  }

  void handle_response(std::shared_ptr<void> &response, std::shared_ptr<void> &req_id)
  {
    auto typed_req_id = std::static_pointer_cast<ros_middleware_interface::RequestId>(req_id);
    auto typed_response = std::static_pointer_cast<typename ServiceT::Response>(response);
    int64_t sequence_number = typed_req_id->sequence_number;
    auto pair = this->pending_requests_[sequence_number];
    auto call_promise = pair.first;
    this->pending_requests_.erase(sequence_number);
    call_promise->set_value(typed_response);
  }

  SharedFuture async_send_request(
    typename ServiceT::Request::Ptr &request,
    typename ServiceT::Response::Ptr &response)
  {
    int64_t sequence_number = ::ros_middleware_interface::send_request(get_client_handle(), request.get());

    SharedPromise call_promise = std::make_shared<Promise>();
    pending_requests_[sequence_number] = std::make_pair(call_promise, response);

    SharedFuture f(call_promise->get_future());
    return f;
  }

private:
  RCLCPP_DISABLE_COPY(Client);

  std::map<int64_t, std::pair<SharedPromise, typename ServiceT::Response::Ptr> > pending_requests_;
};

} /* namespace client */
} /* namespace rclcpp */

#endif /* RCLCPP_RCLCPP_CLIENT_HPP_ */
