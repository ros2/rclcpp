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

#ifndef RCLCPP__SERVICE_HPP_
#define RCLCPP__SERVICE_HPP_

#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/any_service_callback.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"

namespace rclcpp
{
namespace service
{

class ServiceBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(ServiceBase);

  RCLCPP_PUBLIC
  ServiceBase(
    std::shared_ptr<rmw_node_t> node_handle,
    rmw_service_t * service_handle,
    const std::string service_name);

  RCLCPP_PUBLIC
  virtual ~ServiceBase();

  RCLCPP_PUBLIC
  std::string
  get_service_name() const;

  RCLCPP_PUBLIC
  const rmw_service_t *
  get_service_handle() const;

  virtual std::shared_ptr<void> create_request() const = 0;
  virtual std::shared_ptr<void> create_request_header() const = 0;
  virtual void handle_request(
    std::shared_ptr<void> request_header,
    std::shared_ptr<void> request) const = 0;

private:
  RCLCPP_DISABLE_COPY(ServiceBase);

  std::shared_ptr<rmw_node_t> node_handle_;

  rmw_service_t * service_handle_;
  std::string service_name_;
};

using any_service_callback::AnyServiceCallback;

template<typename ServiceT>
class Service : public ServiceBase
{
public:
  using CallbackType = std::function<
      void(
        const std::shared_ptr<typename ServiceT::Request>,
        std::shared_ptr<typename ServiceT::Response>)>;

  using CallbackWithHeaderType = std::function<
      void(
        const std::shared_ptr<rmw_request_id_t>,
        const std::shared_ptr<typename ServiceT::Request>,
        std::shared_ptr<typename ServiceT::Response>)>;
  RCLCPP_SMART_PTR_DEFINITIONS(Service);

  Service(
    std::shared_ptr<rmw_node_t> node_handle,
    rmw_service_t * service_handle,
    const std::string & service_name,
    AnyServiceCallback<ServiceT> any_callback)
  : ServiceBase(node_handle, service_handle, service_name), any_callback_(any_callback)
  {}

  Service() = delete;

  std::shared_ptr<void> create_request() const
  {
    return std::shared_ptr<void>(new typename ServiceT::Request());
  }

  std::shared_ptr<void> create_request_header() const
  {
    // TODO(wjwwood): This should probably use rmw_request_id's allocator.
    //                (since it is a C type)
    return std::shared_ptr<void>(new rmw_request_id_t);
  }

  void handle_request(std::shared_ptr<void> request_header, std::shared_ptr<void> request) const
  {
    auto typed_request = std::static_pointer_cast<typename ServiceT::Request>(request);
    auto typed_request_header = std::static_pointer_cast<rmw_request_id_t>(request_header);
    auto response = std::shared_ptr<typename ServiceT::Response>(new typename ServiceT::Response);
    any_callback_.dispatch(typed_request_header, typed_request, response);
    send_response(typed_request_header, response);
  }

  void send_response(
    std::shared_ptr<rmw_request_id_t> req_id,
    std::shared_ptr<typename ServiceT::Response> response) const
  {
    rmw_ret_t status = rmw_send_response(get_service_handle(), req_id.get(), response.get());
    if (status != RMW_RET_OK) {
      // *INDENT-OFF* (prevent uncrustify from making unecessary indents here)
      throw std::runtime_error(
        std::string("failed to send response: ") + rmw_get_error_string_safe());
      // *INDENT-ON*
    }
  }

private:
  RCLCPP_DISABLE_COPY(Service);

  AnyServiceCallback<ServiceT> any_callback_;
};

}  // namespace service
}  // namespace rclcpp

#endif  // RCLCPP__SERVICE_HPP_
