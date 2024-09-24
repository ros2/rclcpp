// Copyright 2024 Sony Group Corporation.
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

#include "rclcpp/generic_service.hpp"

namespace rclcpp
{
GenericService::GenericService(
  std::shared_ptr<rcl_node_t> node_handle,
  const std::string & service_name,
  const std::string & service_type,
  GenericServiceCallback any_callback,
  rcl_service_options_t & service_options)
: ServiceBase(node_handle),
  any_callback_(any_callback)
{
  const rosidl_service_type_support_t * service_ts;
  try {
    ts_lib_ = get_typesupport_library(
      service_type, "rosidl_typesupport_cpp");

    service_ts = get_service_typesupport_handle(
      service_type, "rosidl_typesupport_cpp", *ts_lib_);

    auto request_type_support_intro = get_message_typesupport_handle(
      service_ts->request_typesupport,
      rosidl_typesupport_introspection_cpp::typesupport_identifier);
    request_members_ = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
      request_type_support_intro->data);

    auto response_type_support_intro = get_message_typesupport_handle(
      service_ts->response_typesupport,
      rosidl_typesupport_introspection_cpp::typesupport_identifier);
    response_members_ = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
      response_type_support_intro->data);
  } catch (std::runtime_error & err) {
    RCLCPP_ERROR(
      rclcpp::get_node_logger(node_handle_.get()).get_child("rclcpp"),
      "Invalid service type: %s",
      err.what());
    throw rclcpp::exceptions::InvalidServiceTypeError(err.what());
  }

  // rcl does the static memory allocation here
  service_handle_ = std::shared_ptr<rcl_service_t>(
    new rcl_service_t, [handle = node_handle_, service_name](rcl_service_t * service)
    {
      if (rcl_service_fini(service, handle.get()) != RCL_RET_OK) {
        RCLCPP_ERROR(
          rclcpp::get_node_logger(handle.get()).get_child("rclcpp"),
          "Error in destruction of rcl service handle: %s",
          rcl_get_error_string().str);
        rcl_reset_error();
      }
      delete service;
    });
  *service_handle_.get() = rcl_get_zero_initialized_service();

  rcl_ret_t ret = rcl_service_init(
    service_handle_.get(),
    node_handle.get(),
    service_ts,
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
  TRACETOOLS_TRACEPOINT(
    rclcpp_service_callback_added,
    static_cast<const void *>(get_service_handle().get()),
    static_cast<const void *>(&any_callback_));
#ifndef TRACETOOLS_DISABLED
  any_callback_.register_callback_for_tracing();
#endif
}

bool
GenericService::take_request(
  SharedRequest request_out,
  rmw_request_id_t & request_id_out)
{
  request_out = create_request();
  return this->take_type_erased_request(request_out.get(), request_id_out);
}

std::shared_ptr<void>
GenericService::create_request()
{
  Request request = new uint8_t[request_members_->size_of_];
  request_members_->init_function(request, rosidl_runtime_cpp::MessageInitialization::ZERO);
  return std::shared_ptr<void>(
    request,
    [this](void * p)
    {
      request_members_->fini_function(p);
      delete[] reinterpret_cast<uint8_t *>(p);
    });
}

std::shared_ptr<void>
GenericService::create_response()
{
  Response response = new uint8_t[response_members_->size_of_];
  response_members_->init_function(response, rosidl_runtime_cpp::MessageInitialization::ZERO);
  return std::shared_ptr<void>(
    response,
    [this](void * p)
    {
      response_members_->fini_function(p);
      delete[] reinterpret_cast<uint8_t *>(p);
    });
}

std::shared_ptr<rmw_request_id_t>
GenericService::create_request_header()
{
  return std::make_shared<rmw_request_id_t>();
}

void
GenericService::handle_request(
  std::shared_ptr<rmw_request_id_t> request_header,
  std::shared_ptr<void> request)
{
  auto response = any_callback_.dispatch(
    this->shared_from_this(), request_header, request, create_response());
  if (response) {
    send_response(*request_header, response);
  }
}

void
GenericService::send_response(rmw_request_id_t & req_id, SharedResponse & response)
{
  rcl_ret_t ret = rcl_send_response(get_service_handle().get(), &req_id, response.get());

  if (ret == RCL_RET_TIMEOUT) {
    RCLCPP_WARN(
      node_logger_.get_child("rclcpp"),
      "failed to send response to %s (timeout): %s",
      this->get_service_name(), rcl_get_error_string().str);
    rcl_reset_error();
    return;
  }
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send response");
  }
}

}  //  namespace rclcpp
