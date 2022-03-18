// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/service.hpp"

#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/any_service_callback.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"

using rclcpp::ServiceBase;

ServiceBase::ServiceBase(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base)
: node_handle_(node_base->get_shared_rcl_node_handle()),
  context_(node_base->get_context()),
  node_logger_(rclcpp::get_node_logger(node_base->get_shared_rcl_node_handle().get()))
{}

ServiceBase::~ServiceBase()
{
  std::lock_guard<std::recursive_mutex> lock(ipc_mutex_);
  if (!use_intra_process_) {
    return;
  }
  auto ipm = weak_ipm_.lock();
  if (!ipm) {
    RCLCPP_WARN(
      rclcpp::get_logger("rclcpp"),
      "Intra process manager died before than a service.");
    return;
  }
  ipm->remove_service(intra_process_service_id_);
}

bool
ServiceBase::take_type_erased_request(void * request_out, rmw_request_id_t & request_id_out)
{
  rcl_ret_t ret = rcl_take_request(
    this->get_service_handle().get(),
    &request_id_out,
    request_out);
  if (RCL_RET_SERVICE_TAKE_FAILED == ret) {
    return false;
  } else if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
  return true;
}

const char *
ServiceBase::get_service_name()
{
  return rcl_service_get_service_name(this->get_service_handle().get());
}

std::shared_ptr<rcl_service_t>
ServiceBase::get_service_handle()
{
  return service_handle_;
}

std::shared_ptr<const rcl_service_t>
ServiceBase::get_service_handle() const
{
  return service_handle_;
}

rcl_node_t *
ServiceBase::get_rcl_node_handle()
{
  return node_handle_.get();
}

const rcl_node_t *
ServiceBase::get_rcl_node_handle() const
{
  return node_handle_.get();
}

bool
ServiceBase::exchange_in_use_by_wait_set_state(bool in_use_state)
{
  return in_use_by_wait_set_.exchange(in_use_state);
}

rclcpp::QoS
ServiceBase::get_response_publisher_actual_qos() const
{
  const rmw_qos_profile_t * qos =
    rcl_service_response_publisher_get_actual_qos(service_handle_.get());
  if (!qos) {
    auto msg =
      std::string("failed to get service's response publisher qos settings: ") +
      rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(msg);
  }

  rclcpp::QoS response_publisher_qos =
    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(*qos), *qos);

  return response_publisher_qos;
}

rclcpp::QoS
ServiceBase::get_request_subscription_actual_qos() const
{
  const rmw_qos_profile_t * qos =
    rcl_service_request_subscription_get_actual_qos(service_handle_.get());
  if (!qos) {
    auto msg =
      std::string("failed to get service's request subscription qos settings: ") +
      rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(msg);
  }

  rclcpp::QoS request_subscription_qos =
    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(*qos), *qos);

  return request_subscription_qos;
}

void
ServiceBase::setup_intra_process(
  uint64_t intra_process_service_id,
  IntraProcessManagerWeakPtr weak_ipm)
{
  std::lock_guard<std::recursive_mutex> lock(ipc_mutex_);
  intra_process_service_id_ = intra_process_service_id;
  weak_ipm_ = weak_ipm;
  use_intra_process_ = true;
}

rclcpp::Waitable::SharedPtr
ServiceBase::get_intra_process_waitable()
{
  std::lock_guard<std::recursive_mutex> lock(ipc_mutex_);
  // If not using intra process, shortcut to nullptr.
  if (!use_intra_process_) {
    return nullptr;
  }
  // Get the intra process manager.
  auto ipm = weak_ipm_.lock();
  if (!ipm) {
    throw std::runtime_error(
            "ServiceBase::get_intra_process_waitable() called "
            "after destruction of intra process manager");
  }

  // Use the id to retrieve the intra-process service from the intra-process manager.
  return ipm->get_service_intra_process(intra_process_service_id_);
}

void
ServiceBase::set_on_new_request_callback(rcl_event_callback_t callback, const void * user_data)
{
  rcl_ret_t ret = rcl_service_set_on_new_request_callback(
    service_handle_.get(),
    callback,
    user_data);

  if (RCL_RET_OK != ret) {
    using rclcpp::exceptions::throw_from_rcl_error;
    throw_from_rcl_error(ret, "failed to set the on new request callback for service");
  }
}
