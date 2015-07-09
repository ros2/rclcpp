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

#ifndef RCLCPP_RCLCPP_SERVICE_HPP_
#define RCLCPP_RCLCPP_SERVICE_HPP_

#include <functional>
#include <memory>
#include <string>

#include <rmw/error_handling.h>
#include <rmw/rmw.h>

#include <rclcpp/macros.hpp>


namespace rclcpp
{

// Forward declaration for friend statement
namespace executor
{
class Executor;
} // namespace executor

namespace service
{

class ServiceBase
{
  friend class rclcpp::executor::Executor;

public:
  RCLCPP_MAKE_SHARED_DEFINITIONS(ServiceBase);

  ServiceBase(
    rmw_service_t * service_handle,
    const std::string service_name)
  : service_handle_(service_handle), service_name_(service_name)
  {}

  ~ServiceBase()
  {
    if (service_handle_ != nullptr) {
      rmw_destroy_service(service_handle_);
      service_handle_ = nullptr;
    }
  }

  std::string get_service_name()
  {
    return this->service_name_;
  }

  const rmw_service_t * get_service_handle()
  {
    return this->service_handle_;
  }

  virtual std::shared_ptr<void> create_request() = 0;
  virtual std::shared_ptr<void> create_request_header() = 0;
  virtual void handle_request(
    std::shared_ptr<void> & request_header,
    std::shared_ptr<void> & request) = 0;

private:
  RCLCPP_DISABLE_COPY(ServiceBase);

  rmw_service_t * service_handle_;
  std::string service_name_;

};

template<typename ServiceT>
class Service : public ServiceBase
{
public:
  typedef std::function<
      void (
        const std::shared_ptr<typename ServiceT::Request> &,
        std::shared_ptr<typename ServiceT::Response> &)> CallbackType;

  typedef std::function<
      void (
        const std::shared_ptr<rmw_request_id_t> &,
        const std::shared_ptr<typename ServiceT::Request> &,
        std::shared_ptr<typename ServiceT::Response> &)> CallbackWithHeaderType;
  RCLCPP_MAKE_SHARED_DEFINITIONS(Service);

  Service(
    rmw_service_t * service_handle,
    const std::string & service_name,
    CallbackType callback)
  : ServiceBase(service_handle, service_name), callback_(callback), callback_with_header_(nullptr)
  {}

  Service(
    rmw_service_t * service_handle,
    const std::string & service_name,
    CallbackWithHeaderType callback_with_header)
  : ServiceBase(service_handle, service_name), callback_(nullptr),
    callback_with_header_(callback_with_header)
  {}

  std::shared_ptr<void> create_request()
  {
    return std::shared_ptr<void>(new typename ServiceT::Request());
  }

  std::shared_ptr<void> create_request_header()
  {
    // TODO(wjwwood): This should probably use rmw_request_id's allocator.
    //                (since it is a C type)
    return std::shared_ptr<void>(new rmw_request_id_t);
  }

  void handle_request(std::shared_ptr<void> & request_header, std::shared_ptr<void> & request)
  {
    auto typed_request = std::static_pointer_cast<typename ServiceT::Request>(request);
    auto typed_request_header = std::static_pointer_cast<rmw_request_id_t>(request_header);
    auto response = std::shared_ptr<typename ServiceT::Response>(new typename ServiceT::Response);
    if (callback_with_header_ != nullptr) {
      callback_with_header_(typed_request_header, typed_request, response);
    } else {
      callback_(typed_request, response);
    }
    send_response(typed_request_header, response);
  }

  void send_response(
    std::shared_ptr<rmw_request_id_t> & req_id,
    std::shared_ptr<typename ServiceT::Response> & response)
  {
    rmw_ret_t status = rmw_send_response(get_service_handle(), req_id.get(), response.get());
    if (status != RMW_RET_OK) {
      // *INDENT-OFF* (prevent uncrustify from making unecessary indents here)
      throw std::runtime_error(
        std::string("failed to send response: ") +
        (rmw_get_error_string() ? rmw_get_error_string() : ""));
      // *INDENT-ON*
    }
  }

private:
  RCLCPP_DISABLE_COPY(Service);

  CallbackType callback_;
  CallbackWithHeaderType callback_with_header_;
};

} /* namespace service */
} /* namespace rclcpp */

#endif /* RCLCPP_RCLCPP_SERVICE_HPP_ */
