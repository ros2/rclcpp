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

#include "rcl/error_handling.h"
#include "rcl/service.h"

#include "rclcpp/any_service_callback.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
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
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(ServiceBase)

  RCLCPP_PUBLIC
  ServiceBase(
    std::shared_ptr<rcl_node_t> node_handle,
    const std::string & service_name);

  RCLCPP_PUBLIC
  explicit ServiceBase(
    std::shared_ptr<rcl_node_t> node_handle);

  RCLCPP_PUBLIC
  virtual ~ServiceBase();

  RCLCPP_PUBLIC
  std::string
  get_service_name();

  RCLCPP_PUBLIC
  rcl_service_t *
  get_service_handle();

  RCLCPP_PUBLIC
  const rcl_service_t *
  get_service_handle() const;

  virtual std::shared_ptr<void> create_request() = 0;
  virtual std::shared_ptr<rmw_request_id_t> create_request_header() = 0;
  virtual void handle_request(
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<void> request) = 0;

protected:
  RCLCPP_DISABLE_COPY(ServiceBase)

  RCLCPP_PUBLIC
  rcl_node_t *
  get_rcl_node_handle();

  RCLCPP_PUBLIC
  const rcl_node_t *
  get_rcl_node_handle() const;

  std::shared_ptr<rcl_node_t> node_handle_;

  rcl_service_t * service_handle_ = nullptr;
  std::string service_name_;
  bool owns_rcl_handle_ = true;
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
  RCLCPP_SMART_PTR_DEFINITIONS(Service)

  Service(
    std::shared_ptr<rcl_node_t> node_handle,
    const std::string & service_name,
    AnyServiceCallback<ServiceT> any_callback,
    rcl_service_options_t & service_options)
  : ServiceBase(node_handle, service_name), any_callback_(any_callback)
  {
    using rosidl_typesupport_cpp::get_service_type_support_handle;
    auto service_type_support_handle = get_service_type_support_handle<ServiceT>();

    // rcl does the static memory allocation here
    service_handle_ = new rcl_service_t;
    *service_handle_ = rcl_get_zero_initialized_service();

    rcl_ret_t ret = rcl_service_init(
      service_handle_,
      node_handle.get(),
      service_type_support_handle,
      service_name.c_str(),
      &service_options);
    if (ret != RCL_RET_OK) {
      if (ret == RCL_RET_SERVICE_NAME_INVALID) {
        auto rcl_node_handle = get_rcl_node_handle();
        // this will throw on any validation problem
        rcl_reset_error();
        expand_topic_or_service_name(
          service_name,
          rcl_node_get_name(rcl_node_handle),
          rcl_node_get_namespace(rcl_node_handle),
          true);
      }

      rclcpp::exceptions::throw_from_rcl_error(ret, "could not create service");
    }
  }

  Service(
    std::shared_ptr<rcl_node_t> node_handle,
    rcl_service_t * service_handle,
    AnyServiceCallback<ServiceT> any_callback)
  : ServiceBase(node_handle),
    any_callback_(any_callback)
  {
    // check if service handle was initialized
    // TODO(karsten1987): Take this verification
    // directly in rcl_*_t
    // see: https://github.com/ros2/rcl/issues/81
    if (!service_handle->impl) {
      // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
      throw std::runtime_error(
        std::string("rcl_service_t in constructor argument must be initialized beforehand."));
      // *INDENT-ON*
    }
    service_handle_ = service_handle;
    service_name_ = std::string(rcl_service_get_service_name(service_handle));
    owns_rcl_handle_ = false;
  }

  Service() = delete;

  virtual ~Service()
  {
    // check if you have ownership of the handle
    if (owns_rcl_handle_) {
      if (rcl_service_fini(service_handle_, node_handle_.get()) != RCL_RET_OK) {
        std::stringstream ss;
        ss << "Error in destruction of rcl service_handle_ handle: " <<
          rcl_get_error_string_safe() << '\n';
        (std::cerr << ss.str()).flush();
        rcl_reset_error();
      }
      delete service_handle_;
    }
  }

  std::shared_ptr<void> create_request()
  {
    return std::shared_ptr<void>(new typename ServiceT::Request());
  }

  std::shared_ptr<rmw_request_id_t> create_request_header()
  {
    // TODO(wjwwood): This should probably use rmw_request_id's allocator.
    //                (since it is a C type)
    return std::shared_ptr<rmw_request_id_t>(new rmw_request_id_t);
  }

  void handle_request(std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<void> request)
  {
    auto typed_request = std::static_pointer_cast<typename ServiceT::Request>(request);
    auto response = std::shared_ptr<typename ServiceT::Response>(new typename ServiceT::Response);
    any_callback_.dispatch(request_header, typed_request, response);
    send_response(request_header, response);
  }

  void send_response(
    std::shared_ptr<rmw_request_id_t> req_id,
    std::shared_ptr<typename ServiceT::Response> response)
  {
    rcl_ret_t status = rcl_send_response(get_service_handle(), req_id.get(), response.get());

    if (status != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(status, "failed to send response");
    }
  }

private:
  RCLCPP_DISABLE_COPY(Service)

  AnyServiceCallback<ServiceT> any_callback_;
};

}  // namespace service
}  // namespace rclcpp

#endif  // RCLCPP__SERVICE_HPP_
