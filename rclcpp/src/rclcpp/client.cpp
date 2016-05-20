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

using rclcpp::client::ClientBase;

ClientBase::ClientBase(
  std::shared_ptr<rcl_node_t> node_handle,
  const std::string & service_name)
: node_handle_(node_handle), service_name_(service_name),
  wait_set_(rcl_get_zero_initialized_wait_set())
{
  // TODO(wjwwood): use a memory strategy's allocator rather than the default.
  rcl_ret_t ret = rcl_wait_set_init(&wait_set_, 0, 1, 0, 0, 0, rcl_get_default_allocator());
  if (ret != RCL_RET_OK)
  {
    wait_set_ = rcl_get_zero_initialized_wait_set();
    throw std::runtime_error(
      std::string("failed to create wait set: ") + rcl_get_error_string_safe());
  }
  const rcl_guard_condition_t * graph_guard_condition =
    rcl_node_get_graph_guard_condition(node_handle_.get());
  if (!graph_guard_condition) {
    throw std::runtime_error(
      std::string("failed to get graph guard condition: ") + rcl_get_error_string_safe());
  }
  ret = rcl_wait_set_add_guard_condition(&wait_set_, graph_guard_condition);
  if (ret != RCL_RET_OK) {
    throw std::runtime_error(
      std::string("failed to add guard condition to wait set: ") + rcl_get_error_string_safe());
  }
}

ClientBase::~ClientBase()
{
  if (rcl_wait_set_fini(&wait_set_) != RCL_RET_OK) {
    throw std::runtime_error(
      std::string("failed to finalize rcl_wait_set_t: ") + rcl_get_error_string_safe());
  }
}

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
ClientBase::server_is_ready() const
{
  bool is_ready;
  rcl_ret_t ret = rcl_service_server_is_available(node_handle_.get(), &client_handle_, &is_ready);
  if (ret != RCL_RET_OK) {
    throw std::runtime_error(
      std::string("rcl_service_server_is_available failed: ") + rcl_get_error_string_safe());
  }
  return is_ready;
}

bool
ClientBase::wait_for_server_nanoseconds(std::chrono::nanoseconds timeout)
{
  // if timeout is > 0, end_time is in the future
  // if timeout is == 0, end_time will be in the past
  // if timeout is < 0, end_time is already in the past
  auto end_time = std::chrono::steady_clock::now() + timeout;
  std::chrono::nanoseconds time_to_wait = timeout;
  // the do-while ensures that rcl_wait is always called at least once.
  do {
    rcl_ret_t ret = rcl_wait(&wait_set_, time_to_wait.count());
    if (ret == RCL_RET_TIMEOUT) {
      // was not ready within the given timeout
      break;
    }
    if (ret != RCL_RET_OK) {
      throw std::runtime_error(
        std::string("failed to wait on wait set: ") + rcl_get_error_string_safe());
    }
    if (this->server_is_ready()) {
      return true;
    }
    std::chrono::nanoseconds time_left = end_time - std::chrono::steady_clock::now();
    if (time_left <= std::chrono::nanoseconds(0)) {
      // end_time is in the past, no time left to wait
      break;
    }
    // otherwise prepare to wait for the remaining time
    time_to_wait = time_left;
  } while (std::chrono::steady_clock::now() < end_time);
  // ran out of time, server was not ready
  return false;
}
