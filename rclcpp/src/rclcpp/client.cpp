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

#include <chrono>
#include <cstdio>
#include <string>

#include "rcl/graph.h"
#include "rcl/node.h"
#include "rcl/wait.h"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/node.hpp"

using rclcpp::client::ClientBase;
using rclcpp::exceptions::throw_from_rcl_error;

ClientBase::ClientBase(
  rclcpp::node::Node * parent_node,
  const std::string & service_name)
: node_(parent_node), node_handle_(parent_node->get_shared_node_handle()),
  service_name_(service_name)
{}

ClientBase::~ClientBase() {}

const std::string &
ClientBase::get_service_name() const
{
  return this->service_name_;
}

const rcl_client_t *
ClientBase::get_client_handle() const
{
  return &client_handle_;
}

bool
ClientBase::service_is_ready() const
{
  bool is_ready;
  rcl_ret_t ret =
    rcl_service_server_is_available(this->get_rcl_node_handle(), &client_handle_, &is_ready);
  if (ret != RCL_RET_OK) {
    throw_from_rcl_error(ret, "rcl_service_server_is_available failed");
  }
  return is_ready;
}

bool
ClientBase::wait_for_service_nanoseconds(std::chrono::nanoseconds timeout)
{
  auto start = std::chrono::steady_clock::now();
  // check to see if the server is ready immediately
  if (this->service_is_ready()) {
    return true;
  }
  if (timeout == std::chrono::nanoseconds(0)) {
    // check was non-blocking, return immediately
    return false;
  }
  // make an event to reuse, rather than create a new one each time
  auto event = node_->get_graph_event();
  // update the time even on the first loop to account for time in first server_is_read()
  std::chrono::nanoseconds time_to_wait = timeout - (std::chrono::steady_clock::now() - start);
  if (timeout > std::chrono::nanoseconds(0) && time_to_wait < std::chrono::nanoseconds(0)) {
    // Do not allow the time_to_wait to become negative when timeout was originally positive.
    // Setting time_to_wait to 0 will allow one non-blocking wait because of the do-while.
    time_to_wait = std::chrono::nanoseconds(0);
  }
  // continue forever if timeout is negative, otherwise continue until out of time_to_wait
  // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
  do {
    node_->wait_for_graph_change(event, time_to_wait);
    if (event->check_and_clear()) {
      if (this->service_is_ready()) {
        return true;
      }
    }
    // server is not ready, loop if there is time left
    time_to_wait = timeout - (std::chrono::steady_clock::now() - start);
  } while (timeout < std::chrono::nanoseconds(0) || time_to_wait > std::chrono::nanoseconds(0));
  // *INDENT-ON*
  return false;  // timeout exceeded while waiting for the server to be ready
}

rcl_node_t *
ClientBase::get_rcl_node_handle() const
{
  return node_handle_.get();
}
