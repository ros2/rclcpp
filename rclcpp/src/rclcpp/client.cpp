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

#include "rclcpp/client.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include "rcl/graph.h"
#include "rcl/node.h"
#include "rcl/wait.h"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/logging.hpp"

using rclcpp::ClientBase;
using rclcpp::exceptions::InvalidNodeError;
using rclcpp::exceptions::throw_from_rcl_error;

ClientBase::ClientBase(
  rclcpp::node_interfaces::NodeBaseInterface * node_base,
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph)
: node_graph_(node_graph),
  node_handle_(node_base->get_shared_rcl_node_handle()),
  context_(node_base->get_context())
{
  std::weak_ptr<rcl_node_t> weak_node_handle(node_handle_);
  rcl_client_t * new_rcl_client = new rcl_client_t;
  *new_rcl_client = rcl_get_zero_initialized_client();
  client_handle_.reset(
    new_rcl_client, [weak_node_handle](rcl_client_t * client)
    {
      auto handle = weak_node_handle.lock();
      if (handle) {
        if (rcl_client_fini(client, handle.get()) != RCL_RET_OK) {
          RCLCPP_ERROR(
            rclcpp::get_node_logger(handle.get()).get_child("rclcpp"),
            "Error in destruction of rcl client handle: %s", rcl_get_error_string().str);
          rcl_reset_error();
        }
      } else {
        RCLCPP_ERROR(
          rclcpp::get_logger("rclcpp"),
          "Error in destruction of rcl client handle: "
          "the Node Handle was destructed too early. You will leak memory");
      }
      delete client;
    });
}

ClientBase::~ClientBase()
{
  // Make sure the client handle is destructed as early as possible and before the node handle
  client_handle_.reset();
}

bool
ClientBase::take_type_erased_response(void * response_out, rmw_request_id_t & request_header_out)
{
  rcl_ret_t ret = rcl_take_response(
    this->get_client_handle().get(),
    &request_header_out,
    response_out);
  if (RCL_RET_CLIENT_TAKE_FAILED == ret) {
    return false;
  } else if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
  return true;
}

const char *
ClientBase::get_service_name() const
{
  return rcl_client_get_service_name(this->get_client_handle().get());
}

std::shared_ptr<rcl_client_t>
ClientBase::get_client_handle()
{
  return client_handle_;
}

std::shared_ptr<const rcl_client_t>
ClientBase::get_client_handle() const
{
  return client_handle_;
}

bool
ClientBase::service_is_ready() const
{
  bool is_ready;
  rcl_ret_t ret = rcl_service_server_is_available(
    this->get_rcl_node_handle(),
    this->get_client_handle().get(),
    &is_ready);
  if (RCL_RET_NODE_INVALID == ret) {
    const rcl_node_t * node_handle = this->get_rcl_node_handle();
    if (node_handle && !rcl_context_is_valid(node_handle->context)) {
      // context is shutdown, do a soft failure
      return false;
    }
  }
  if (ret != RCL_RET_OK) {
    throw_from_rcl_error(ret, "rcl_service_server_is_available failed");
  }
  return is_ready;
}

bool
ClientBase::wait_for_service_nanoseconds(std::chrono::nanoseconds timeout)
{
  auto start = std::chrono::steady_clock::now();
  // make an event to reuse, rather than create a new one each time
  auto node_ptr = node_graph_.lock();
  if (!node_ptr) {
    throw InvalidNodeError();
  }
  // check to see if the server is ready immediately
  if (this->service_is_ready()) {
    return true;
  }
  if (timeout == std::chrono::nanoseconds(0)) {
    // check was non-blocking, return immediately
    return false;
  }
  auto event = node_ptr->get_graph_event();
  // update the time even on the first loop to account for time spent in the first call
  // to this->server_is_ready()
  std::chrono::nanoseconds time_to_wait =
    timeout > std::chrono::nanoseconds(0) ?
    timeout - (std::chrono::steady_clock::now() - start) :
    std::chrono::nanoseconds::max();
  if (time_to_wait < std::chrono::nanoseconds(0)) {
    // Do not allow the time_to_wait to become negative when timeout was originally positive.
    // Setting time_to_wait to 0 will allow one non-blocking wait because of the do-while.
    time_to_wait = std::chrono::nanoseconds(0);
  }
  do {
    if (!rclcpp::ok(this->context_)) {
      return false;
    }
    // Limit each wait to 100ms to workaround an issue specific to the Connext RMW implementation.
    // A race condition means that graph changes for services becoming available may trigger the
    // wait set to wake up, but then not be reported as ready immediately after the wake up
    // (see https://github.com/ros2/rmw_connext/issues/201)
    // If no other graph events occur, the wait set will not be triggered again until the timeout
    // has been reached, despite the service being available, so we artificially limit the wait
    // time to limit the delay.
    node_ptr->wait_for_graph_change(
      event, std::min(time_to_wait, std::chrono::nanoseconds(RCL_MS_TO_NS(100))));
    // Because of the aforementioned race condition, we check if the service is ready even if the
    // graph event wasn't triggered.
    event->check_and_clear();
    if (this->service_is_ready()) {
      return true;
    }
    // server is not ready, loop if there is time left
    if (timeout > std::chrono::nanoseconds(0)) {
      time_to_wait = timeout - (std::chrono::steady_clock::now() - start);
    }
    // if timeout is negative, time_to_wait will never reach zero
  } while (time_to_wait > std::chrono::nanoseconds(0));
  return false;  // timeout exceeded while waiting for the server to be ready
}

rcl_node_t *
ClientBase::get_rcl_node_handle()
{
  return node_handle_.get();
}

const rcl_node_t *
ClientBase::get_rcl_node_handle() const
{
  return node_handle_.get();
}

bool
ClientBase::exchange_in_use_by_wait_set_state(bool in_use_state)
{
  return in_use_by_wait_set_.exchange(in_use_state);
}
