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

#include <future>

#include "rclcpp/generic_client.hpp"
#include "rclcpp/typesupport_helpers.hpp"

#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"

namespace rclcpp
{
GenericClient::GenericClient(
  rclcpp::node_interfaces::NodeBaseInterface * node_base,
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
  const std::string & service_name,
  const std::string & service_type,
  rcl_client_options_t & client_options)
: ClientBase(node_base, node_graph)
{
  ts_lib_ = get_typesupport_library(
    service_type, "rosidl_typesupport_cpp");

  auto service_ts_ = get_service_typesupport_handle(
    service_type, "rosidl_typesupport_cpp", *ts_lib_);

  auto response_type_support_intro = get_message_typesupport_handle(
    service_ts_->response_typesupport,
    rosidl_typesupport_introspection_cpp::typesupport_identifier);
  response_members_ = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
    response_type_support_intro->data);

  rcl_ret_t ret = rcl_client_init(
    this->get_client_handle().get(),
    this->get_rcl_node_handle(),
    service_ts_,
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
    rclcpp::exceptions::throw_from_rcl_error(ret, "could not create generic client");
  }
}

std::shared_ptr<void>
GenericClient::create_response()
{
  void * response = new uint8_t[response_members_->size_of_];
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
GenericClient::create_request_header()
{
  // TODO(wjwwood): This should probably use rmw_request_id's allocator.
  //                (since it is a C type)
  return std::shared_ptr<rmw_request_id_t>(new rmw_request_id_t);
}

void
GenericClient::handle_response(
  std::shared_ptr<rmw_request_id_t> request_header,
  std::shared_ptr<void> response)
{
  auto optional_pending_request =
    this->get_and_erase_pending_request(request_header->sequence_number);
  if (!optional_pending_request) {
    return;
  }
  auto & value = *optional_pending_request;
  if (std::holds_alternative<Promise>(value)) {
    auto & promise = std::get<Promise>(value);
    promise.set_value(std::move(response));
  }
}

size_t
GenericClient::prune_pending_requests()
{
  std::lock_guard guard(pending_requests_mutex_);
  auto ret = pending_requests_.size();
  pending_requests_.clear();
  return ret;
}

bool
GenericClient::remove_pending_request(int64_t request_id)
{
  std::lock_guard guard(pending_requests_mutex_);
  return pending_requests_.erase(request_id) != 0u;
}

std::optional<GenericClient::CallbackInfoVariant>
GenericClient::get_and_erase_pending_request(int64_t request_number)
{
  std::unique_lock<std::mutex> lock(pending_requests_mutex_);
  auto it = pending_requests_.find(request_number);
  if (it == pending_requests_.end()) {
    RCUTILS_LOG_DEBUG_NAMED(
      "rclcpp",
      "Received invalid sequence number. Ignoring...");
    return std::nullopt;
  }
  auto value = std::move(it->second.second);
  pending_requests_.erase(request_number);
  return value;
}

GenericClient::FutureAndRequestId
GenericClient::async_send_request(const Request request)
{
  Promise promise;
  auto future = promise.get_future();
  auto req_id = async_send_request_impl(
    request,
    std::move(promise));
  return FutureAndRequestId(std::move(future), req_id);
}

int64_t
GenericClient::async_send_request_impl(const Request request, CallbackInfoVariant value)
{
  int64_t sequence_number;
  std::lock_guard<std::mutex> lock(pending_requests_mutex_);
  rcl_ret_t ret = rcl_send_request(get_client_handle().get(), request, &sequence_number);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send request");
  }
  pending_requests_.try_emplace(
    sequence_number,
    std::make_pair(std::chrono::system_clock::now(), std::move(value)));
  return sequence_number;
}

}  // namespace rclcpp
